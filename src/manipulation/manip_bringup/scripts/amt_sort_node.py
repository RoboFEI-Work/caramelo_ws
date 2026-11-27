#!/usr/bin/env python3
"""amt_sort_node: servidor da action /amt1_sort (AMT refeita em 2026-11-16).

Ordena os blocos da mesa de precisao: os ids que vem do arquivo da competicao
(expected_tags) terminam em ordem crescente da esquerda para a direita, ou na
ordem explicita do arquivo (target_order). A conta esta' em amt_sort_core.py.

Como funciona (quem move o braco para olhar e' o executor: tag_esquerda,
tag_direita e pegar_obj, uma chamada com observe_only=true em cada olhada):
  1. liga a camera (/manip/observe_active, ouvido pelo perception_gate)
     e junta quadros do /detections: a ORDEM dos blocos na fila (so' ids). As
     olhadas se emendam pelos blocos em comum e ficam na memoria ate' a chamada
     que ordena (ou 2 min);
  2. so' com busca_passo_m > 0 (bancada; no robo as olhadas laterais ja' cobrem
     os 7 slots): faltou bloco, a base anda de lado, olha de novo e volta;
  3. planeja as trocas usando os potes de bordo como espera;
  4. executa como CLIENTE de /pick_tag e /place_tag (place com stack_on =
     tag do slot: e' o caminho novo da PP, com a camera medindo o furo). A base
     fica onde esta' e so' anda quando o pick/place pede (fora de alcance, ou
     slot fora do eixo no place) => /nudge_base e tenta de novo; por ultimo uma
     tentativa "final" (o pick/place faz a varredura completa). No fim da
     chamada a base volta ao centro, de onde as olhadas cobrem a fileira;
  5. se uma operacao falhar, para de ordenar e devolve a' mesa o que estiver a
     bordo (no lugar certo se estiver vazio, senao em qualquer slot vazio).

Este no' NAO move o braco: so' olha, anda a base de lado e manda pick/place.

Resultado (2026-11-17, "disse que ordenou sem ter ordenado"): success so' quando
a CAMERA ve a mesa em ordem e nao ha' nada a fazer. Depois de mexer em bloco a
chamada responde success=false / fail_reason=falta_conferir, e o executor roda
outra rodada (olhadas + esta chamada) que confere e arruma o que faltou.
"""

import math
import os
import threading
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from caramelo_msgs.action import AlignToDock, NudgeBase
from my_robot_msgs.action import Amt1Sort, PickTag, PlaceTag
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import yaml

import amt_sort_core as A


class AmtSort(Node):
    def __init__(self):
        super().__init__("amt1_sort_action_server")
        dp = self.declare_parameter
        self.slot_ids = [int(v) for v in dp("slot_tag_ids", [101, 102, 103, 104, 105, 106, 107]).value]
        self.observar_s = float(dp("observar_s", 2.5).value)
        # 2026-11-17: com o PC carregado o /detections caiu a < 1 Hz e uma olhada valeu com 1 quadro.
        self.min_quadros = int(dp("min_quadros", 4).value)
        self.camera_espera_s = float(dp("camera_espera_s", 25.0).value)
        # 2026-11-17: 0 no robo. A busca andava a base antes de qualquer pick e, com a volta para
        # "onde viu", dava o vai e volta; as olhadas laterais do braco ja' cobrem os 7 slots.
        self.busca_passo_m = float(dp("busca_passo_m", 0.0).value)       # 0 = sem busca lateral
        # curso lateral total da base. 0.45: em 17/11 17h17 o slot 101 estava a 0,385 m do eixo (o
        # docking parou 6 cm fora do centro) e com 0.35 o 2o passo do alinhamento foi recusado.
        self.desvio_max_m = float(dp("desvio_max_m", 0.45).value)
        # 2026-11-17 (pedido do operador): perdeu a Raspberry por um tempo, espera e retoma na MESMA
        # operacao enquanto a missao estiver rodando. 10 min; antes eram 45 s e ela desistia.
        self.braco_espera_s = float(dp("braco_espera_s", 600.0).value)
        self.passo_slot_m = float(dp("passo_slot_m", 0.107).value)       # distancia entre slots vizinhos
        self.max_nudges = int(dp("max_nudges", 2).value)                 # por operacao (alcance)
        self.op_prazo_s = float(dp("op_prazo_s", 420.0).value)
        # 2026-11-17 (travou no robo): o pedido seguinte chegava ao place 33 ms depois do
        # fim do anterior, com ele ainda desmontando a interface do MoveIt. Respiro entre ops.
        self.pausa_s = float(dp("pausa_entre_ops_s", 1.0).value)
        # 2026-11-26 (operador: "retire todas as falas do robo [...] deixar ele o mais rapido possivel"): sem voz por padrao.
        self.falar = bool(dp("speech_enabled", False).value)
        self.potes_yaml = os.path.expanduser(
            dp("container_state_file", "~/.config/caramelo/container_states.yaml").value)
        self._g = ReentrantCallbackGroup()
        self._obs = None
        self._fila, self._fila_t = None, 0.0     # olhadas acumuladas entre chamadas
        self._desvio = 0.0             # quanto a base andou de lado desde o docking (+ = esquerda)
        self._juntas_t = None          # chegada do ultimo /joint_states (None = nunca: bancada sem braco)
        self._ocupado = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._on_juntas, qos_profile_sensor_data,
                                 callback_group=self._g, raw=True)
        self.create_subscription(AprilTagDetectionArray, dp("detections_topic", "/detections").value,
                                 self._on_det, 20, callback_group=self._g)
        self._fala = self.create_publisher(String, "/manip/speech", 10)
        # 2026-11-27 (pedido do operador): "assim que ele identificar a ordem atual da
        # mesa, colocar na telinha ORDEM ATUAL: TAGS, e conforme ele der um place novo
        # atualizar essa ordem". Latched; vazio = some da tela. Quem mostra: task_viewer
        # (entre os pick/place; durante eles a tela mostra a tag da operacao).
        self._ordem_pub = self.create_publisher(String, "/manip/amt_ordem", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._ordem_ate = None     # hora (monotonic) de tirar a ordem da tela
        self.create_timer(1.0, self._ordem_timer, callback_group=self._g)
        self._olhando = self.create_publisher(Bool, "/manip/observe_active", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._pick = ActionClient(self, PickTag, "/pick_tag", callback_group=self._g)
        self._place = ActionClient(self, PlaceTag, "/place_tag", callback_group=self._g)
        self._nudge = ActionClient(self, NudgeBase, "/nudge_base", callback_group=self._g)
        self._atracar = ActionClient(self, AlignToDock, "/align_to_dock", callback_group=self._g)
        ActionServer(self, Amt1Sort, "/amt1_sort", execute_callback=self._executar,
                     goal_callback=self._aceitar, cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                     callback_group=self._g)
        self.get_logger().info("amt_sort pronto: /amt1_sort (olha, anda a base de lado e manda pick/place).")

    def _aceitar(self, _goal):
        return GoalResponse.REJECT if self._ocupado.locked() else GoalResponse.ACCEPT

    def _on_det(self, msg):
        obs = self._obs
        if obs is not None and msg.detections:
            tags = {d.id: [[c.x, c.y] for c in d.corners] for d in msg.detections}
            obs.juntar(tags)

    def _on_juntas(self, _cru):
        self._juntas_t = time.monotonic()

    def _braco_vivo(self):
        return self._juntas_t is None or time.monotonic() - self._juntas_t < 1.5

    def _esperar_braco(self, gh):
        """As juntas vem da Raspberry pelo cabo. Em 17/11 o adaptador USB-Ethernet do PC caiu 3
        vezes (16:11, 16:16, 16:21): /joint_states mudo por 5 a 20 s e, na ultima, ate' o fim; o
        pick seguinte falhou e a AMT seguiu como se nada. Espera voltar antes de mandar operacao."""
        if self._braco_vivo():
            return True
        t0, aviso = time.monotonic(), -1e9
        while rclpy.ok() and not gh.is_cancel_requested and time.monotonic() - t0 < self.braco_espera_s:
            gh.publish_feedback(Amt1Sort.Feedback(current_stage="esperando_braco", ops_done=0, ops_total=0))
            if self._braco_vivo():
                time.sleep(3.0)        # a descoberta do DDS e o MoveIt se reencontram
                if self._braco_vivo():
                    self._falar("!O braco voltou, continuando de onde parei")
                    return True
            if time.monotonic() - aviso > 30.0:
                aviso = time.monotonic()
                self._falar("!Perdi o contato com o braco, esperando voltar")
            time.sleep(0.5)
        return False

    def _nos_potes(self):
        """ids dos blocos que o yaml dos potes de bordo diz estarem a bordo."""
        try:
            potes = (yaml.safe_load(open(self.potes_yaml, encoding="utf-8")) or {}).get("containers") or {}
            return [int(str(p.get("tag_frame", "")).replace("tag_", "")) for p in potes.values()
                    if p.get("occupied") and str(p.get("tag_frame", "")).startswith("tag_")]
        except (OSError, ValueError, AttributeError, yaml.YAMLError) as ex:
            self.get_logger().warn(f"[AMT] nao li {self.potes_yaml}: {ex}")
            return []

    def _mostrar_ordem(self, estado):
        """'ORDEM ATUAL: 3 1 - 2 4 5 6': os slots da esquerda para a direita, '-' = vazio."""
        txt = " ".join("-" if estado.get(s) is None else str(estado[s]) for s in self.slot_ids)
        self._ordem_ate = None
        self._ordem_pub.publish(String(data=f"ORDEM ATUAL: {txt}"))

    def _limpar_ordem(self, em_s=0.0):
        """Tira a ordem da tela agora ou daqui a em_s (a rodada seguinte da AMT a repoe)."""
        if em_s <= 0.0:
            self._ordem_ate = None
            self._ordem_pub.publish(String(data=""))
        else:
            self._ordem_ate = time.monotonic() + em_s

    def _ordem_timer(self):
        if self._ordem_ate is not None and time.monotonic() >= self._ordem_ate:
            self._limpar_ordem()

    def _falar(self, texto):
        self.get_logger().info(f"[AMT] {texto}")
        if self.falar:
            self._fala.publish(String(data=texto))

    # ---- chamadas de action com batimento de feedback -------------------------
    def _chamar(self, cliente, goal, gh, etapa, feitos=0, total=0):
        """Manda o goal e espera o resultado. None = sem servidor / recusado / prazo / cancelado."""
        time.sleep(self.pausa_s)
        if not cliente.wait_for_server(timeout_sec=5.0):
            return None
        pronto = threading.Event()
        fut = cliente.send_goal_async(goal)
        fut.add_done_callback(lambda _f, p=pronto: p.set())
        t0 = time.monotonic()
        handle, res_fut = None, None
        while rclpy.ok():
            gh.publish_feedback(Amt1Sort.Feedback(current_stage=etapa, ops_done=feitos, ops_total=total))
            if pronto.wait(2.0):
                if handle is None:
                    handle = fut.result()
                    if not handle.accepted:
                        return None
                    pronto = threading.Event()
                    res_fut = handle.get_result_async()
                    res_fut.add_done_callback(lambda _f, p=pronto: p.set())
                    continue
                return res_fut.result().result
            # Prazo tambem para o ACEITE (30 s): em 17/11 o place travado nem respondia ao
            # pedido novo e este laco esperava para sempre, sem obedecer ao cancelamento.
            if gh.is_cancel_requested or time.monotonic() - t0 > (self.op_prazo_s if handle else 30.0):
                if handle is not None:
                    handle.cancel_goal_async()
                return None
        return None

    def _andar(self, gh, dy, res):
        """Um passo lateral da base (+ = esquerda), dentro do curso total. True se andou.
        2026-11-17 17h57: o passo de 0,25 foi recusado inteiro (obstaculo_lateral: perna da mesa
        perto) e o place tentou de longe, sem alcance. Barrado por obstaculo => tenta 0,15 e 0,10."""
        if abs(self._desvio + dy) > self.desvio_max_m + 1e-6:
            self.get_logger().warn(f"[AMT] passo lateral de {dy:+.2f} m passaria do curso ({self.desvio_max_m:.2f} m).")
            return False
        for passo in [dy] + [math.copysign(m, dy) for m in (0.15, 0.10) if m < abs(dy) - 0.01]:
            n = self._chamar(self._nudge, NudgeBase.Goal(dy=float(passo), timeout=0.0), gh, f"nudge_{passo:+.2f}")
            res.nudges += 1
            if n is not None:
                self._desvio += float(n.travelled)
                res.total_shift_m += float(n.travelled)
            if n is not None and "recuou" in str(n.reason):
                # 2026-11-17 18h30 (pedido do operador): o nudge deu re' e a base ficou longe da mesa
                # (0,53 m e 8 graus torta): refaz o docking (dock_id vazio = repetir o ultimo). A base
                # volta ao centro; True = a operacao e' pedida de novo e pede o passo que precisar.
                self._falar("!Fiquei longe da mesa, vou atracar de novo")
                a = self._chamar(self._atracar, AlignToDock.Goal(), gh, "refazendo_docking")
                self.get_logger().info(
                    f"[AMT] docking refeito: {'sem resposta' if a is None else a.message or a.success}")
                if a is None or not a.success:
                    return False          # nao atracou: a conta do desvio fica como esta'
                self._desvio = 0.0
                return True
            if n is not None and n.success:
                return True
            self.get_logger().warn(
                f"[AMT] a base nao andou {passo:+.2f} m ({'sem resposta' if n is None else n.reason}).")
            if n is None or "obstaculo" not in str(n.reason) or abs(float(n.travelled)) > 0.02:
                break                  # so' recusa por obstaculo, sem ter andado, merece passo menor
        return False

    def _ir_para(self, gh, desvio, res):
        """Leva a base ate' o desvio lateral pedido (passos de 0,10 a 0,25 m). Melhor esforco."""
        d = desvio - self._desvio
        if abs(d) < 0.08:
            return True
        return self._andar(gh, math.copysign(min(0.25, max(0.10, abs(d))), d), res)

    def _observar(self, gh, obs):
        """Junta quadros por observar_s (contados a partir do primeiro que chegar)."""
        n0, t0, primeiro = obs.quadros, time.monotonic(), None
        self._obs = obs
        try:
            while rclpy.ok() and not gh.is_cancel_requested:
                gh.publish_feedback(Amt1Sort.Feedback(current_stage="observando", ops_done=0, ops_total=0))
                if primeiro is None and obs.quadros > n0:
                    primeiro = time.monotonic()
                dt = None if primeiro is None else time.monotonic() - primeiro
                if dt is not None and dt >= self.observar_s and (
                        obs.quadros - n0 >= self.min_quadros or dt >= self.observar_s + 6.0):
                    break
                if primeiro is None and time.monotonic() - t0 > self.camera_espera_s:
                    break
                time.sleep(0.2)
        finally:
            self._obs = None
        return obs.estado()[0]

    def _buscar(self, gh, obs, esperados, res):
        """Faltou bloco do arquivo, ou tag de slot para ancorar a fila: olha tambem com a base
        deslocada para cada lado (so' com busca_passo_m > 0: bancada, que tem uma vista so')."""
        meio = (len(self.slot_ids) - 1) / 2.0
        lados = []
        for _ in range(2):
            estado, ancorado = obs.estado()
            if ancorado and not set(esperados) - {c for c in estado.values() if c is not None}:
                break
            desc = [i for i, s in enumerate(self.slot_ids) if s not in estado]
            # slots de indice baixo (101...) ficam a' esquerda do robo: base para a esquerda (+)
            lado = 1.0 if sum(i < meio for i in desc) >= sum(i > meio for i in desc) else -1.0
            if lado in lados:
                lado = -lado              # desse lado ja' olhou
            lados.append(lado)
            self._falar("Vou olhar mais para a " + ("esquerda" if lado > 0 else "direita"))
            if not self._ir_para(gh, lado * self.busca_passo_m, res):
                break
            time.sleep(0.5)
            self._observar(gh, obs)
        self._ir_para(gh, 0.0, res)
        return obs.estado()[0]

    def _operar(self, gh, op, cubo, slot, goal, feitos, total, res):
        """Um pick ou place. (ok, motivo)"""
        nudges, final, repetiu, reatracou = 0, False, False, False
        while True:
            if not self._esperar_braco(gh):
                return False, "cancelado" if gh.is_cancel_requested else "braco_sem_resposta"
            if op == "pick":
                g = PickTag.Goal(tag_frame=f"tag_{cubo}", table_pose=goal.table_pose, final_attempt=final)
                r = self._chamar(self._pick, g, gh, f"pick_tag_{cubo}_slot_{slot}", feitos, total)
            else:
                g = PlaceTag.Goal(tag_frame=f"tag_{cubo}", table_pose=goal.table_pose, ws=goal.ws,
                                  stack_on=f"tag_{slot}", final_attempt=final)
                r = self._chamar(self._place, g, gh, f"place_tag_{cubo}_slot_{slot}", feitos, total)
            if r is None:
                return False, "cancelado" if gh.is_cancel_requested else f"{op}_sem_resposta"
            # skipped = o pick/place PULOU (fora de alcance, tag nao vista) e responde
            # success=true: nao conta como feito (17/11: bloco 1 voltou ao pote e a AMT seguiu).
            if r.success and not r.skipped:
                return True, ""
            if not self._braco_vivo() and not repetiu:
                repetiu = True        # falhou porque o braco sumiu no meio: espera voltar e repete
                continue
            passo = float(r.suggested_base_shift_m)
            andou_de_lado = False
            if not final and r.unreachable and abs(passo) > 1e-3 and nudges < self.max_nudges:
                # sem fala aqui: o pick/place e o nudge ja' avisam (3 frases para 1 evento atrasavam a voz)
                self.get_logger().info(f"[AMT] {op} do bloco {cubo} pediu a base {passo:+.2f} m de lado.")
                nudges += 1
                if self._andar(gh, passo, res):
                    continue
            # 2026-11-26 (operador: "pega a tag e depois ve que nao consegue chegar no ponto e
            # fica preso nesse looping infinito; e' bom ele mexer pra frente, caso seja possivel,
            # ou se alinhar mais com a tag"): fora de alcance e o passo de lado nao resolveu
            # (sugestao 0 = de lado nao alcanca, ou a base nao andou). Antes caia direto na
            # tentativa final, que falhava igual, e a rodada seguinte repetia tudo.
            #   1) alinhar com o slot pela geometria da fileira (slot fora da frente do robo);
            #   2) ir PARA FRENTE: refazer o docking (dock_id vazio = o ultimo), que leva a
            #      base 'a distancia de trabalho da mesa sem raspar; uma vez por operacao.
            x = (self.slot_ids.index(slot) - (len(self.slot_ids) - 1) / 2.0) * self.passo_slot_m + self._desvio
            if not final and r.unreachable and abs(x) > 0.05 and nudges < self.max_nudges:
                self.get_logger().info(
                    f"[AMT] {op} do bloco {cubo} fora de alcance; alinho a base com o slot {slot} ({x:+.2f} m).")
                nudges += 1
                if self._andar(gh, -math.copysign(min(0.25, abs(x)), x), res):
                    andou_de_lado = True
                    continue
            if not final and r.unreachable and not reatracou and not andou_de_lado:
                reatracou = True
                self.get_logger().info(f"[AMT] {op} do bloco {cubo} fora de alcance; chego mais perto da mesa (re-docking).")
                self._falar("Nao alcanco, vou chegar mais perto da mesa")
                a = self._chamar(self._atracar, AlignToDock.Goal(), gh, "refazendo_docking_alcance")
                if a is not None and a.success:
                    self._desvio = 0.0
                    continue
                self.get_logger().warn(
                    f"[AMT] re-docking para alcance: {'sem resposta' if a is None else a.message or 'falhou'}.")
            # Nao viu o alvo e a base esta' longe dele (ficou alinhada com outro slot num place):
            # anda ate' o slot pela geometria da fileira (slot do meio em frente ao robo no docking).
            x = (self.slot_ids.index(slot) - (len(self.slot_ids) - 1) / 2.0) * self.passo_slot_m + self._desvio
            if not final and r.skipped and not r.unreachable and abs(x) > 0.10 and nudges < self.max_nudges:
                self.get_logger().info(f"[AMT] {op} do bloco {cubo}: slot {slot} a {x:+.2f} m de lado; ando ate' ele.")
                nudges += 1
                if self._andar(gh, -math.copysign(min(0.25, abs(x)), x), res):
                    continue
            if not final:
                final = True          # ultima tentativa: escada completa e varredura do pick/place
                continue
            return False, r.fail_reason or f"{op}_falhou"

    # ---- a rotina --------------------------------------------------------------
    def _executar(self, gh):
        with self._ocupado:
            return self._rotina(gh)

    def _rotina(self, gh):
        goal, res = gh.request, Amt1Sort.Result()
        esperados = [int(v) for v in goal.expected_tags]
        capacidade = goal.max_onboard if 0 < goal.max_onboard <= 3 else 3
        direcao = goal.direction or "left_to_right"
        if self._fila is None or time.monotonic() - self._fila_t > 120.0:
            self._fila, self._desvio = A.Fileira(self.slot_ids), 0.0
        obs = self._fila
        self._olhando.publish(Bool(data=True))      # perception_gate liga a camera
        try:
            estado = self._observar(gh, obs)
            self._fila_t = time.monotonic()
            if goal.observe_only:
                # Uma olhada (o executor esta' passando o braco pelas poses): so' acumula.
                res.observed_order = A.ordem(estado, self.slot_ids, direcao)
                res.partial = True
                res.message = f"olhada: {obs.quadros} quadro(s) ate' agora; fila {estado}"
                self.get_logger().info(f"[AMT] {res.message}")
                gh.succeed()
                return res
            if esperados and self.busca_passo_m > 0.0:
                estado = self._buscar(gh, obs, esperados, res)
            ancorado = obs.estado()[1]
            if not ancorado:
                # Sem tag de slot para dizer ONDE a fila comeca, o estado e' um chute (17/11: so' de
                # pegar_obj saia 1 slot deslocado). Nao mexe em bloco com chute: a rodada falha e olha de novo.
                self.get_logger().warn(f"[AMT] fila sem ancora (nenhuma tag de slot util): {estado} - descartada.")
                estado = {}
            vistos = sorted(c for c in estado.values() if c is not None)
            esperados = esperados or vistos
            res.missing_tags = sorted(set(esperados) - set(vistos))
            res.observed_order = A.ordem(estado, self.slot_ids, direcao)
            if not vistos:
                res.fail_reason, res.message = "observacao_falhou", (
                    f"nao vi a fila de blocos ({obs.quadros} quadros do /detections com tag)")
                self.get_logger().warn(f"[AMT] {res.message}")
                self._falar("Nao consegui ver os blocos na mesa de precisao")
                gh.succeed()
                return res
            # Blocos do arquivo que ja' estao nos potes de bordo (missao anterior interrompida).
            a_bordo = [c for c in self._nos_potes() if c in esperados and c not in vistos]
            res.missing_tags = sorted(set(res.missing_tags) - set(a_bordo))
            if a_bordo:
                self.get_logger().info(f"[AMT] ja' a bordo (container_states.yaml): {a_bordo}")
            estado = A.completar(estado, [c for c in esperados if c not in a_bordo], self.slot_ids)
            self._mostrar_ordem(estado)
            alvo = A.alvo_por_ordem(self.slot_ids, [int(v) for v in goal.target_order] or sorted(esperados),
                                    direcao)
            ops, sobra = A.planejar(estado, alvo, a_bordo=a_bordo, capacidade=capacidade)
            nada_a_fazer = not ops
            self.get_logger().info(
                f"[AMT] vistos {estado} ({obs.quadros} quadros) | "
                f"alvo {alvo} | {len(ops)} operacao(oes): {ops} | fora do plano: {sobra}"
                f" | nao vistos: {list(res.missing_tags)}")
            if ops:
                self._falar(f"Ordem atual: {', '.join(str(c) for c in res.observed_order)}. "
                            f"Vou fazer {len(ops)} movimentos")
            # 2026-11-17 18h (operador: "por que tentou colocar a tag 1 em varios slots?"): antes, a
            # 1a falha derrubava o plano inteiro e o que estava a bordo ia para QUALQUER slot vazio
            # (bloco 1 no 107). Agora: o bloco que falhou fica de fora desta rodada (preso), o resto e'
            # replanejado e segue; bloco a bordo so' volta para o SEU slot ou para o slot de onde saiu.
            destino = {c: s for s, c in alvo.items()}
            bordo, presos, origem, feitas = list(a_bordo), set(), {}, 0
            while True:
                ops, _sobra = A.planejar(estado, {s: c for s, c in alvo.items() if c not in presos},
                                         a_bordo=bordo, capacidade=capacidade)
                if not ops:
                    break
                op, cubo, slot = ops[0]
                ok, motivo = self._operar(gh, op, cubo, slot, goal, feitas, feitas + len(ops), res)
                if not ok:
                    self.get_logger().warn(f"[AMT] {op} do bloco {cubo} no slot {slot} falhou: {motivo}")
                    res.fail_reason = motivo if motivo in ("cancelado", "braco_sem_resposta") else f"{op}_falhou"
                    if motivo in ("cancelado", "braco_sem_resposta"):
                        break
                    presos.add(cubo)
                    continue
                feitas += 1
                if op == "pick":
                    estado[slot] = None
                    origem[cubo] = slot
                    bordo.append(cubo)
                    res.picks += 1
                else:
                    estado[slot] = cubo
                    self._mostrar_ordem(estado)
                    bordo.remove(cubo)
                    res.places += 1
            # O que sobrou a bordo: o slot dele (se nao foi o que falhou) ou o slot de onde saiu.
            # Nunca um slot qualquer; o que nao couber fica no pote e a proxima rodada tenta de novo.
            if res.fail_reason not in ("cancelado", "braco_sem_resposta"):
                for cubo in list(bordo):
                    for slot in dict.fromkeys([None if cubo in presos else destino.get(cubo), origem.get(cubo)]):
                        if slot is None or estado.get(slot, 0) is not None:
                            continue
                        ok, _m = self._operar(gh, "place", cubo, slot, goal, feitas, feitas, res)
                        if ok:
                            estado[slot] = cubo
                            self._mostrar_ordem(estado)
                            bordo.remove(cubo)
                            res.places += 1
                            break
            sobra = sorted(c for s, c in estado.items() if c is not None and c in destino and destino[c] != s)
            # A base volta ao centro UMA vez, no fim: as olhadas da rodada de conferencia cobrem os 7
            # slots a partir do centro (bancada 17/11: com a base 14 cm de lado a ponta nao aparecia).
            if res.fail_reason not in ("cancelado", "braco_sem_resposta"):
                self._ir_para(gh, 0.0, res)
            res.final_order = A.ordem(estado, self.slot_ids, direcao)
            feito = not res.fail_reason and not sobra and not res.missing_tags and not bordo
            # success so' com a mesa VISTA em ordem nesta chamada (nenhuma operacao): depois de
            # mexer em bloco quem confere e' a rodada seguinte do executor (olhadas + esta chamada).
            res.success = feito and nada_a_fazer
            res.partial = not res.success and not bordo
            if feito and not nada_a_fazer:
                res.fail_reason = "falta_conferir"
            res.message = (f"ordem final {list(res.final_order)}; {res.picks} pick(s), {res.places} place(s), "
                           f"{res.nudges} passo(s) de base"
                           + (f"; a bordo {bordo}" if bordo else "")
                           + (f"; fora do plano {sobra}" if sobra else "")
                           + (f"; nao vistos {list(res.missing_tags)}" if res.missing_tags else ""))
            self._falar("!Conferi, a mesa de precisao esta em ordem" if res.success else
                        "Movimentos feitos, vou conferir a mesa" if feito else
                        "!O braco nao respondeu, parei a ordenacao" if res.fail_reason == "braco_sem_resposta" else
                        "!Nao consegui ordenar tudo na mesa de precisao")
            self.get_logger().info(f"[AMT] {res.message}")
            # Mesa em ordem (ou cancelado): sai da tela; senao vem outra rodada (olhadas +
            # esta chamada), que repoe a ordem. Sem rodada seguinte em 2 min: sai.
            self._limpar_ordem(0.0 if res.success or gh.is_cancel_requested else 120.0)
            if gh.is_cancel_requested:
                res.fail_reason = "cancelado"
                gh.canceled()
            else:
                gh.succeed()
            return res
        finally:
            self._obs = None
            if not goal.observe_only:
                self._fila = None          # a proxima missao comeca do zero
                self._olhando.publish(Bool(data=False))


def main():
    rclpy.init()
    no = AmtSort()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(no)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
