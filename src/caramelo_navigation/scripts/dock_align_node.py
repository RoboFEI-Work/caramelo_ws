#!/usr/bin/env python3
"""Alinhamento fino HOLONOMICO a um dock (ADR-05, opcao c).

Complementa o opennav_docking: o controller do Docking Server e' unicycle
(so linear.x + angular.z, sem vy), entao ele nao usa as rodas mecanum para
corrigir lateralmente. Este no faz o trecho FINAL de alinhamento com holonomia
(vx, vy, wz), depois que o Docking Server ja levou o robo ate o staging/approach.

O que este no faz (v4, 2026-08-21 — "reto, depois ajeita, nunca encosta"):
  1. Le a pose alvo do dock em docking.yaml (mesma base do opennav_docking).
  2. (Opcional) refina a pose alvo pela FACE da mesa no /scan -> corrige a
     distancia (X) e o yaw, que sao observaveis numa face reta.
  3. MURO DE SEGURANCA por LiDAR a cada ciclo (20 Hz, idade pela hora de
     chegada do scan): hard_min ACIMA do bico do LiDAR (o que toca a mesa);
     abaixo dele o no RECUA ate o pre-docking e tenta de novo
     (approach_retries), so falhando depois de esgotar — e a camada de missao
     decide pular a mesa. Scan velho/ausente ou face fora do plano = NAO anda.
  4. Fases: DRIVE (reta continua, cortada antes do standoff para o coast do
     ESC fechar) -> STANDOFF (so aqui gira e anda de lado: esquadra o yaw pela
     face e centra no CENTRO DA FACE medido pelo LiDAR, em pulsos curtos com
     checagem lateral) -> APPROACH (pulsos no piso com portao de distancia
     por pior caso, auto-calibrado pelo proprio LiDAR) -> FINISH (sem giro;
     so re por pulso se passou). Sucesso = front na janela E yaw da face
     valido na tolerancia. Nada de lateral no staging.

LIMITES HONESTOS (ADR-05):
  - Y (lateral) so e' observavel quando as DUAS bordas da face aparecem no
    scan; sem elas o no NAO mexe no lateral (fica onde o Nav2 deixou). O
    LiDAR e' cego para o que esta ao lado do chassi (x < 0.32 m) e abaixo do
    seu plano (~25 cm) — por isso os pulsos laterais sao curtos e so no
    standoff.
  - PISO DO ESC (2026-08-03, firmware corrigido): 0.1215 m/s / 0.315 rad/s de
    corpo, EXECUTADOS EXATOS (ganhos do PI + mapa ancorado; ver
    caramelo_hardware_ws/docs). O piso e por RODA (2.43 rad/s): em comandos
    mistos vx/vy/wz uma roda individual pode ficar sub-piso e o driver a
    arredonda — a garantia "executa exato" vale p/ movimentos de eixo puro.

Nada aqui foi validado no robo — a percepcao e os ganhos precisam de dados reais.
"""
import math
import os
import sys

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import tf2_ros

# Helpers compartilhados de docking (carregam docking.yaml por pasta de mapa).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from caramelo_docking_common import load_yaml, resolve_map_folder, docking_file  # noqa: E402

from caramelo_msgs.action import AlignToDock  # noqa: E402


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quat(z, w, x=0.0, y=0.0):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _fit_line_tls(points):
    """Ajuste de reta por minimos quadrados totais (2x2) sem numpy.

    Retorna (cx, cy, nx, ny) = ponto medio + vetor NORMAL unitario da reta,
    ou None se os pontos forem poucos/degenerados. Usado para estimar a face
    plana da mesa (distancia e orientacao)."""
    n = len(points)
    if n < 8:
        return None
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    # Autovetor do menor autovalor da covariancia = direcao NORMAL da reta.
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    # theta e' a direcao principal (ao longo da reta); a normal e' perpendicular.
    nx = -math.sin(theta)
    ny = math.cos(theta)
    return cx, cy, nx, ny


class DockAlignNode(Node):
    def __init__(self):
        super().__init__("dock_align_node")

        # --- Parametros ---
        self.declare_parameter("reference_topic", "/mecanum_controller/reference")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("control_frequency", 20.0)
        # Limites coerentes com o Nav2 (controller_server.yaml).
        # 0.20 -> 0.13 (v4): a reta do staging e' curta (~0.15-0.25 m) e o
        # coast do ESC cresce com a velocidade.
        self.declare_parameter("max_vel_x", 0.13)
        # Piso fisico do ESC pos-fix do firmware (2026-08-03): 650rpm = 0.1215
        # m/s / 0.315 rad/s de corpo, executado EXATO (mapa ancorado 1570us).
        self.declare_parameter("min_body_vel", 0.1215)
        self.declare_parameter("min_body_omega", 0.315)
        self.declare_parameter("kp_linear", 1.2)
        self.declare_parameter("default_xy_tolerance", 0.03)
        # 0.05 -> 0.09 (teste 3 em 2026-08-18): a leitura de yaw da face
        # oscila +-4 graus entre fits — exigir 2.9 e' pedir menos que o ruido
        # e o robo fica em retoque infinito ate o timeout. 5 graus fecha, e o
        # pick compensa o residuo pela camera/tag.
        self.declare_parameter("default_yaw_tolerance", 0.09)
        # 30 -> 60 (2026-08-18): o align agora faz o approach INTEIRO
        # (staging->dock, ~45cm) e o orcamento cobre recuo+retries.
        # 60 -> 90 (v4): 2 tentativas de ate 40 s + recuo.
        self.declare_parameter("default_timeout", 90.0)
        self.declare_parameter("settle_cycles", 5)

        # ------- MURO DE SEGURANCA por LiDAR (2026-08-18, pedido do operador:
        # "o robo NUNCA pode bater na mesa"). Distancia frontal medida a CADA
        # ciclo por minimo robusto do corredor frontal — nao o fit TLS, cujos
        # guards descartam vista parcial justamente perto da mesa.
        self.declare_parameter("wall_enabled", True)
        # Distancia de parada frontal. -1 = usa desired_face_dist do dock
        # (docking.yaml, por mesa) e, na falta, refine_desired_face_dist
        # global; tudo <0 = muro so aplica o hard_min.
        self.declare_parameter("stop_face_dist", -1.0)
        # Banda de histerese do HOLD (ideia do operador: parar a X com folga
        # de +-0.5cm para nao ligar/desligar o freio a cada ciclo).
        # 0.005 -> 0.015 (v4): janela de sucesso [stop-0.01, stop+band+0.01]
        # = 3,5 cm, maior que o ruido do percentil (+-3 mm) e o passo do pulso.
        self.declare_parameter("wall_hysteresis_band", 0.015)
        # Abaixo disso: para tudo e RECUA para tentar de novo. O que toca a
        # mesa e' o BICO do LiDAR (apendice frontal, ponta ~0.38-0.40 m do
        # centro — costmap.yaml usa 0.395): hard_min tem de ficar ACIMA dele.
        # 0.32 (antigo) = bico 5,5 cm DENTRO da mesa. CAMPO 2026-08-21: o
        # robo ENCOSTOU com o muro lendo 0.398 => a ponta real esta em ~0.40
        # (valor do footprint 0.395). 0.42 = 2,5 cm de folga do contato real.
        self.declare_parameter("hard_min_face_dist", 0.42)
        # Idade medida pela HORA DE CHEGADA do scan (imune a relogio da Pi).
        # 0.5 -> 0.2 s: a 0.12 m/s cada 0.1 s e' 1,2 cm de atraso.
        self.declare_parameter("wall_max_scan_age", 0.2)
        self.declare_parameter("wall_min_points", 5)
        # Corredor frontal = meia-largura do chassi 0.195 + 1.5cm de margem.
        self.declare_parameter("wall_corridor_half_width", 0.21)
        self.declare_parameter("wall_max_range", 1.5)
        # k-esimo menor x (fracao dos pontos do corredor): rejeita speckle
        # isolado sem esconder obstaculo real.
        self.declare_parameter("wall_percentile", 0.10)
        # Teto de vx proporcional a folga ate o stop (alem do perfil P).
        self.declare_parameter("wall_kp", 1.5)
        # RECUO+RETRY (pedido do operador 2026-08-18): em hard-stop/timeout de
        # aproximacao, recua ate a distancia de pre-docking e tenta de novo
        # este numero de vezes antes de reportar falha (a missao pula a mesa).
        # 2 -> 0 (v5, pedido do operador: SEM vai-e-vem): uma tentativa so.
        self.declare_parameter("approach_retries", 0)

        # ------- Fluxo v4 (2026-08-21, pedido do operador): NADA de lateral no
        # staging (a rajada cega do v3 batia em caixas laterais invisiveis ao
        # LiDAR). staging -> RETA -> STANDOFF (esquadra o yaw e centra na face
        # pelo LiDAR, longe o bastante para o bico girar) -> APROXIMACAO FINAL
        # por pulsos com portao de distancia -> FINISH sem giro.
        # Distancia ACIMA do stop onde o robo para para esquadrar/centrar.
        self.declare_parameter("standoff_gap", 0.12)
        # O ESC mantem a ultima referencia ~0,5 s depois do neutro (TURNOFF):
        # qualquer movimento continuo e' CORTADO quando falta exatamente este
        # coast, e o ESC termina. MEDIDO 2026-08-21 (pulse_test.py cont, robo
        # real): x andou 15 cm p/ 12,2 comandados => coast ~3 cm (0.04 c/ margem);
        # giro 20-30 graus p/ 18 => ~6 graus (0.10 rad); lateral andou 10 cm
        # p/ 12,2 (escorrega, sem coast) => 0.03.
        self.declare_parameter("esc_coast_dist", 0.04)
        # Mesma ideia para o giro (rad) e para o lateral (m). Abaixo disso nao
        # ha como corrigir — o residuo e' aceito (a camera do pick compensa).
        self.declare_parameter("yaw_coast_est", 0.10)
        self.declare_parameter("center_coast_est", 0.03)
        # Raio do canto do bico ~0.38-0.40 m: girar so com front >= isto; pulso
        # lateral so com front >= lateral_min_front.
        self.declare_parameter("rotate_min_front", 0.46)
        # 0.48 -> 0.42 (campo 2026-08-21): lateral puro nao avanca o bico;
        # o operador confirmou que a 0,43 e' seguro andar de lado.
        self.declare_parameter("lateral_min_front", 0.41)
        # Esquadro no standoff: alvo 0.06 rad (3.4 graus), no maximo 4 s;
        # depois aceita ate a tolerancia de sucesso (yaw_tolerance).
        self.declare_parameter("standoff_yaw_tol", 0.06)
        self.declare_parameter("standoff_square_max_s", 4.0)
        # Assentamento apos cada comando: front estavel em N scans NOVOS
        # (eps) ou teto de tempo — so entao medir/pulsar de novo.
        self.declare_parameter("settle_stable_scans", 3)
        self.declare_parameter("settle_stable_eps", 0.003)
        self.declare_parameter("settle_max_s", 1.5)
        # Sem medicao do muro: scan velho (STALE) ou corredor vazio (BLIND).
        # Nos dois casos o robo NAO anda; passado o teto, a tentativa termina
        # (recuo+retry). Mesa fora do plano do LiDAR NAO e' dockavel por aqui.
        self.declare_parameter("stale_abort_s", 1.0)
        self.declare_parameter("blind_wait_s", 1.0)
        # Pulsos no piso: comprimento base (ciclos) por eixo e escalonamento
        # (+1 ciclo se "nao moveu", ate o max). O ESC precisa de >200 ms para
        # ARMAR a partir do repouso: 4 ciclos = 200 ms (campo 18/08 mediu
        # ~2,4 cm/3,6 graus); 6 ciclos = 300 ms. O comprimento que funcionou e'
        # MANTIDO (nao volta ao base) — so reescala se deixar de mover.
        self.declare_parameter("pulse_on_cycles", 4)
        self.declare_parameter("pulse_on_max_cycles", 6)
        # Portao de distancia da aproximacao final: so pulsa para frente se
        # front - safety * max(learned, pulse_travel_est) >= janela_inferior.
        # `learned` = deslocamento medido pelo proprio LiDAR nos pulsos
        # anteriores; pulse_travel_est = pior caso ate a medicao de campo.
        # 2026-08-21: pulso de 4-6 ciclos andou 10-14 cm (medido) — pulsos
        # nao servem de aproximacao fina; 0.12 fecha o portao no APPROACH e
        # deixa os pulsos so para a RE. approach_mode default = single_cut.
        self.declare_parameter("pulse_travel_est", 0.12)
        self.declare_parameter("pulse_travel_safety", 1.5)
        # Pulso que andou mais que (portao + esta margem) = anomalia (ESC/
        # atrito): encerra a tentativa. E' margem sobre o pior caso vigente,
        # nunca um teto absoluto abaixo da estimativa.
        self.declare_parameter("pulse_travel_margin", 0.03)
        # Passo minimo de giro por pulso (~0,315 rad/s x (0,2+0,5) s ~ 0,2 rad
        # se o ESC retiver a referencia): so pulsar giro se o erro for maior
        # que metade disto, senao o esquadro vira pingue-pongue.
        self.declare_parameter("pulse_yaw_est", 0.12)
        # Pior caso do pulso lateral (centragem) ate a medicao de campo.
        self.declare_parameter("center_pulse_travel_est", 0.12)
        # Se o portao proibir o pulso mas o front estiver ate esta folga
        # acima da janela, aceita parar ali (nunca forcar para encostar).
        self.declare_parameter("approach_accept_slack", 0.05)
        # "pulses" (default) ou "single_cut" (reta continua cortada em
        # stop + band/2 + esc_coast_dist, depois assenta) — escolher pela
        # medicao de campo dos pulsos (plano 2026-08-21, M1).
        self.declare_parameter("approach_mode", "single_cut")
        # Centragem na face (standoff): estimador endurecido + pulsos vy.
        self.declare_parameter("center_enabled", True)
        self.declare_parameter("center_half_width_y", 0.65)
        self.declare_parameter("center_edge_margin", 0.08)
        # 0.50 -> 0.48 (campo 22/08: a prateleira mede 0.49).
        self.declare_parameter("center_width_min", 0.48)
        self.declare_parameter("center_width_max", 0.95)
        self.declare_parameter("center_max_residual", 0.012)
        self.declare_parameter("center_samples", 5)
        self.declare_parameter("center_spread_max", 0.02)
        self.declare_parameter("center_deadband", 0.025)
        self.declare_parameter("center_max_travel", 0.15)
        self.declare_parameter("center_pulse_cycles", 4)
        self.declare_parameter("center_max_pulses", 4)
        # Faixa lateral (alem do corredor) vigiada antes de cada pulso vy.
        self.declare_parameter("side_clear_half_width", 0.50)
        # FINISH: correcoes (re/yaw) no maximo N antes de encerrar a tentativa.
        self.declare_parameter("finish_max_corrections", 6)
        # Recuo final (apos esgotar tentativas/timeout) com orcamento proprio.
        self.declare_parameter("final_retreat_grace_s", 15.0)
        # Sanidade do pre-docking (campo 22/08, WS5): o align so comeca se o
        # robo estiver a menos disto do staging gravado — pega AMCL perdido e
        # pose de dock desatualizada ANTES de andar rumo a face errada.
        self.declare_parameter("staging_sanity_max_m", 0.6)

        # ------- Refino de yaw sob suspeita (operador 2026-08-18: "as vezes
        # piora"): so aplica a correcao quando 2 fits confiantes consecutivos
        # concordam; e da para desligar de vez por parametro.
        self.declare_parameter("refine_correct_yaw", True)
        self.declare_parameter("refine_yaw_agree_tol", 0.035)

        # Janela frontal (m) onde procuramos a face da mesa para refino.
        self.declare_parameter("refine_front_min_x", 0.10)
        self.declare_parameter("refine_front_max_x", 1.20)
        self.declare_parameter("refine_half_width_y", 0.45)
        # Guardas anti-falso-positivo do refino (2026-07-23):
        # residuo RMS maximo do ajuste de reta (face plana de verdade) e largura
        # minima da face detectada (rejeita perna de mesa/objeto isolado).
        self.declare_parameter("refine_max_residual", 0.02)
        # 0.30 -> 0.50 (2026-08-03, gauntlet WS1 r3): um fit com largura 0.39m
        # (segmento parcial da face vista de perto) passou nos guards com
        # residuo 5mm e girou o alvo 16.7 graus. Fits saudaveis medem 0.61-0.65
        # (face real 0.80m). Abaixo de 0.50 = vista degradada, sem confianca.
        self.declare_parameter("refine_min_face_width", 0.50)
        # Distancia frontal DESEJADA robo->face na pose final (calibrar no robo:
        # posicione o robo na pose perfeita e leia o face_dist do log). Valor
        # negativo = ainda nao calibrado -> o refino corrige SO o yaw.
        self.declare_parameter("refine_desired_face_dist", -1.0)
        # (v4) Sem re-refino periodico: depois do refino inicial o controle e'
        # todo pela face no LiDAR (front/yaw/centro), nao pelo alvo do mapa.

        self._ref_topic = self.get_parameter("reference_topic").value
        self._global_frame = self.get_parameter("global_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._cb_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(TwistStamped, self._ref_topic, 10)
        self._last_scan = None
        # SensorDataQoS (2026-08-18): compativel com driver reliable OU
        # best-effort; a assinatura antiga (reliable/depth 10) quebraria em
        # silencio se o driver publicasse best-effort.
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value,
            self._scan_cb, qos_profile_sensor_data, callback_group=self._cb_group,
        )
        # Cache da TF base<-frame_do_scan (LiDAR parafusado = estatica).
        self._scan_tf = None
        self._scan_tf_frame = None
        # Ultimo yaw de face confiante (frame do MAPA) p/ o portao de
        # consistencia do refino de yaw.
        self._last_face_yaw_map = None
        # desired_face_dist por dock (docking.yaml), carregado por goal.
        self._dock_face_dist = None
        # Scan: hora de CHEGADA (idade imune ao relogio da Pi) e contador de
        # scans novos (uma medida por scan).
        self._last_scan_rx_ns = 0
        self._scan_seq = 0
        self._stamp_warned = False
        # Menor x fora do corredor, por lado (esq py>0, dir py<0): checagem
        # lateral antes dos pulsos vy do standoff.
        self._side_min = (None, None)
        self._wall_age = None
        self._center_reject = ""
        # Largura da face medida no refino do staging (coerencia do estimador
        # de centro perto da mesa).
        self._last_face_width = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._action = ActionServer(
            self, AlignToDock, "align_to_dock",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"dock_align_node pronto. Publica em '{self._ref_topic}', "
            f"frames {self._global_frame}->{self._base_frame}."
        )

    def _scan_cb(self, msg):
        self._last_scan = msg
        self._last_scan_rx_ns = self.get_clock().now().nanoseconds
        self._scan_seq += 1

    # ------------------------------------------------------------------ TF
    def _robot_pose(self):
        """Pose (x, y, yaw) do robo no global_frame via TF, ou None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return t.x, t.y, _yaw_from_quat(q.z, q.w, q.x, q.y)

    # --------------------------------------------------------------- Docks
    def _load_dock_pose(self, dock_id, map_name, map_dir):
        """Pose do dock + metadados: (x, y, yaw, desired_face_dist|None,
        staging_offset). desired_face_dist e' a chave OPCIONAL por mesa
        (2026-08-18); staging_offset vem do dock_plugins pelo type do dock e
        dimensiona o recuo do retry (fallback 0.45)."""
        map_folder = resolve_map_folder(map_name, map_dir or None)
        data = load_yaml(docking_file(map_folder))
        docks = data.get("docks", {})
        entry = docks.get(dock_id.strip().upper())
        if entry is None:
            raise KeyError(f"dock '{dock_id}' nao esta em {docking_file(map_folder)}")
        pose = entry.get("pose", [0.0, 0.0, 0.0])
        if entry.get("frame", "map") != self._global_frame:
            self.get_logger().warn(
                f"dock '{dock_id}' esta no frame '{entry.get('frame')}', "
                f"mas o no usa '{self._global_frame}'.")

        face_dist = entry.get("desired_face_dist")
        face_dist = float(face_dist) if face_dist is not None else None

        staging_offset = 0.45
        plugin = (data.get("dock_plugins", {}) or {}).get(entry.get("type", ""), {})
        if isinstance(plugin, dict) and "staging_x_offset" in plugin:
            staging_offset = abs(float(plugin["staging_x_offset"]))

        return (float(pose[0]), float(pose[1]), float(pose[2]),
                face_dist, staging_offset)

    # --------------------------------------------------------- Refino LiDAR
    def _scan_points_in_base(self, scan):
        """Pontos validos do scan TRANSFORMADOS para o base_frame (2D).

        2026-07-23: a versao antiga usava os angulos crus do scan assumindo
        0 grau = frente do robo — mas o LiDAR e' montado de cabeca para baixo
        (RPY 180/0/180) e o crop do scan_normalizer poe a frente fisica em
        ~180 graus, entao px saia sempre <= 0 e o refino NUNCA achava pontos.
        Agora usamos a TF real (base <- frame do scan), que absorve a inversao.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, scan.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(f"refino: sem TF {self._base_frame}<-{scan.header.frame_id}: {exc}")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # Linhas X e Y da matriz de rotacao (aplicadas a pontos no plano do laser).
        r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        r01 = 2.0 * (q.x * q.y - q.z * q.w)
        r10 = 2.0 * (q.x * q.y + q.z * q.w)
        r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)

        pts = []
        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment
            if not math.isfinite(r) or r <= scan.range_min or r >= scan.range_max:
                continue
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            pts.append((r00 * lx + r01 * ly + t.x, r10 * lx + r11 * ly + t.y))
        return pts

    # ------------------------------------------------- Muro de seguranca
    def _wall_tf(self, scan_frame):
        """TF base<-scan cacheada (LiDAR parafusado): (r00,r01,r10,r11,tx,ty)."""
        if self._scan_tf is not None and self._scan_tf_frame == scan_frame:
            return self._scan_tf
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, scan_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(
                f"muro: sem TF {self._base_frame}<-{scan_frame}: {exc}")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        self._scan_tf = (
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            2.0 * (q.x * q.y - q.z * q.w),
            2.0 * (q.x * q.y + q.z * q.w),
            1.0 - 2.0 * (q.x * q.x + q.z * q.z),
            t.x, t.y,
        )
        self._scan_tf_frame = scan_frame
        return self._scan_tf

    def _front_wall_distance(self):
        """Distancia frontal REAL + angulo da face no corredor do chassi.

        Distancia = minimo robusto (k-esimo menor x) — de proposito NAO e' o
        fit TLS completo: o muro precisa funcionar com vista parcial e com
        QUALQUER obstaculo (mao, caixa, quina), justamente perto da mesa.
        face_yaw = fit TLS LEVE nos mesmos pontos (guards relaxados — e'
        sinal de CONTROLE relativo, nao reposicionamento de alvo): o angulo
        da normal da face no frame do robo, que a reta final usa para
        "andar reto e so aparar o angulo" (pedido do operador 2026-08-18).

        Retorna (dist|None, n_pts, scan_age_s, face_yaw|None):
          dist None + n_pts == -1  -> sem medicao (scan ausente/velho/sem TF)
          dist None + n_pts >= 0   -> corredor vazio no plano do LiDAR (BLIND)
        """
        scan = self._last_scan
        self._side_min = (None, None)
        if scan is None:
            return None, -1, float("inf"), None
        # Idade pela hora de CHEGADA (v4): o carimbo vem do relogio da Pi e ja
        # esteve 24 dias atrasado (2026-08-21). Avisa uma vez se divergir.
        now_ns = self.get_clock().now().nanoseconds
        scan_age = (now_ns - self._last_scan_rx_ns) * 1e-9
        stamp_age = (now_ns - Time.from_msg(scan.header.stamp).nanoseconds) * 1e-9
        if abs(stamp_age - scan_age) > 0.3 and not self._stamp_warned:
            self._stamp_warned = True
            self.get_logger().warn(
                f"/scan: carimbo difere da chegada em {stamp_age - scan_age:+.2f} s "
                "(relogio Pi x PC?) — usando a hora de chegada.")
        if scan_age > self.get_parameter("wall_max_scan_age").value:
            return None, -1, scan_age, None
        tfm = self._wall_tf(scan.header.frame_id)
        if tfm is None:
            return None, -1, scan_age, None
        r00, r01, r10, r11, ox, oy = tfm

        max_range = self.get_parameter("wall_max_range").value
        half_w = self.get_parameter("wall_corridor_half_width").value
        side_w = self.get_parameter("side_clear_half_width").value
        # Pre-crop por range ANTES de transformar (custo Python): folga = o
        # deslocamento do LiDAR em relacao ao base_frame.
        range_cut = max_range + math.hypot(ox, oy) + 0.05

        pts = []
        left_min = right_min = None
        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment
            if (not math.isfinite(r) or r <= scan.range_min
                    or r >= scan.range_max or r > range_cut):
                continue
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            px = r00 * lx + r01 * ly + ox
            if not (0.05 <= px <= max_range):
                continue
            py = r10 * lx + r11 * ly + oy
            apy = abs(py)
            if apy <= half_w:
                pts.append((px, py))
            elif apy <= side_w:
                # Faixa lateral: o que esta ao lado do corredor, a frente do
                # bico (px >= 0.32 e' o que o LiDAR consegue ver).
                if py > 0.0:
                    left_min = px if left_min is None else min(left_min, px)
                else:
                    right_min = px if right_min is None else min(right_min, px)
        self._side_min = (left_min, right_min)

        n = len(pts)
        if n < int(self.get_parameter("wall_min_points").value):
            return None, n, scan_age, None
        xs = sorted(p[0] for p in pts)
        k = max(1, math.ceil(self.get_parameter("wall_percentile").value * n))
        front = xs[k - 1]

        # Fit leve para o angulo da face: so pontos perto do plano frontal
        # (rejeita fundo/objetos atras da mesa), guards de controle.
        face_yaw = None
        band_pts = [p for p in pts if p[0] <= front + 0.10]
        fit = _fit_line_tls(band_pts)
        if fit is not None and len(band_pts) >= 30:
            cx, cy, nx, ny = fit
            if nx < 0:
                nx, ny = -nx, -ny
            residual = math.sqrt(
                sum(((p[0] - cx) * nx + (p[1] - cy) * ny) ** 2
                    for p in band_pts) / len(band_pts))
            # Guard de extensao (missao 2026-08-18/WS2): fit estreito num
            # canto/objeto passava no residuo e cuspia yaw de lixo — a reta
            # parava a cada segundo para "aparar" ruido.
            along = [-(p[0] - cx) * ny + (p[1] - cy) * nx for p in band_pts]
            extent = max(along) - min(along)
            if (residual <= 0.025 and extent >= 0.25
                    and abs(math.atan2(ny, nx)) <= math.radians(35)):
                face_yaw = math.atan2(ny, nx)
        return front, n, scan_age, face_yaw

    def _side_clear(self, direction, front, margin=0.05):
        """True se nada na faixa lateral do lado do movimento esta mais perto
        que a face (caixa/objeto protruso junto ao bico). Cego para o que
        esta AO LADO do chassi (px < 0.32) e abaixo do plano do LiDAR — por
        isso os pulsos laterais sao curtos e so no standoff."""
        side = self._side_min[0] if direction > 0.0 else self._side_min[1]
        return side is None or front is None or side > front - margin

    def _face_center_from_scan(self, front):
        """(desvio lateral do centro da face em m, +y = esquerda; largura) ou None.

        Endurecido (campo 18/08: o centro pulava -7 -> +17 cm em 2 s porque
        max/min eram dominados por 1 ponto perdido):
          (1) banda em x ao redor do `front` do muro;
          (2) fit TLS + so INLIERS a <= 2,5 cm da reta;
          (3) segmentacao por GAP ao longo da reta: fica o segmento que
              contem o eixo do robo (py=0) — senao None (nunca "o maior":
              centraria na mesa vizinha);
          (4) residuo pequeno, largura plausivel E coerente com a largura
              medida no refino do staging, e as DUAS bordas longe do limite
              da janela lateral (borda cortada = centro enviesado).
        """
        scan = self._last_scan
        if scan is None or front is None:
            self._center_reject = "sem scan/front"
            return None
        hy = self.get_parameter("center_half_width_y").value
        edge_margin = self.get_parameter("center_edge_margin").value
        pts = self._scan_points_in_base(scan)
        if pts is None:
            self._center_reject = "sem TF"
            return None
        # Duas bandas (campo 22/08: WS2/WS6 tinham coisas 15-25 cm ATRAS da
        # face e o fit unico saia com residuo de 6-13 cm): o AJUSTE DA RETA
        # usa so a banda RASA (+10 cm — o suficiente para ate ~7 graus de
        # inclinacao); a banda funda entra depois, so como fonte de inliers
        # para largura/bordas.
        deep = [p for p in pts
                if front - 0.03 <= p[0] <= front + 0.25 and abs(p[1]) <= hy]
        band = [p for p in deep if p[0] <= front + 0.10]
        fit = _fit_line_tls(band)
        if fit is None:
            self._center_reject = "fit falhou (poucos pontos na banda)"
            return None
        cx, cy, nx, ny = fit
        if nx < 0:
            nx, ny = -nx, -ny
        inl = [p for p in deep if abs((p[0] - cx) * nx + (p[1] - cy) * ny) <= 0.025]
        if len(inl) < 30:
            self._center_reject = "menos de 30 inliers"
            return None
        rms = math.sqrt(
            sum(((p[0] - cx) * nx + (p[1] - cy) * ny) ** 2 for p in inl) / len(inl))
        if rms > self.get_parameter("center_max_residual").value:
            self._center_reject = f"residuo alto ({rms * 1000:.0f} mm, {len(inl)} pts)"
            return None
        tang = (-ny, nx)
        # Coordenada ao longo da reta e a lateral (y) de cada inlier, ordenadas.
        along = sorted(
            ((p[0] - cx) * tang[0] + (p[1] - cy) * tang[1], p[1]) for p in inl)
        # Segmento que contem o eixo do robo (py cruza zero).
        segs = []
        cur = [along[0]]
        for prev, item in zip(along, along[1:]):
            if item[0] - prev[0] > 0.05:
                segs.append(cur)
                cur = []
            cur.append(item)
        segs.append(cur)
        seg = None
        for candidate in segs:
            ys = [q[1] for q in candidate]
            if min(ys) <= 0.0 <= max(ys):
                seg = candidate
                break
        if seg is None:
            self._center_reject = "nenhum segmento contem o eixo do robo"
            return None
        width = seg[-1][0] - seg[0][0]
        wmin = self.get_parameter("center_width_min").value
        wmax = self.get_parameter("center_width_max").value
        if not (wmin <= width <= wmax):
            self._center_reject = f"largura {width:.2f} fora de [{wmin:.2f},{wmax:.2f}]"
            return None
        # (Sem comparar com a largura do refino do staging: aquela janela de
        # +-0.45 m CORTA uma mesa de 0.80 — e' por isso que os logs de campo
        # sempre mostravam 0.62-0.65 m. Estimadores diferentes nao se comparam.)
        y_lo = min(q[1] for q in seg)
        y_hi = max(q[1] for q in seg)
        if y_hi > hy - edge_margin or y_lo < -(hy - edge_margin):
            self._center_reject = f"borda na janela (y {y_lo:+.2f}..{y_hi:+.2f}, limite {hy - edge_margin:.2f})"
            return None  # borda encostada no limite da janela: cortada
        # Desvio do centro da face em relacao ao ponto onde o EIXO do robo
        # (direcao +x) cruza a face — e' isso que a reta final vai "acertar",
        # mesmo com yaw residual. tang ~ (0, +1) => +y = esquerda.
        mid_along = 0.5 * (seg[0][0] + seg[-1][0])
        if abs(nx) < 1e-6:
            self._center_reject = "face paralela ao eixo"
            return None
        t_hit = (cx * nx + cy * ny) / nx          # eixo do robo: (t, 0)
        along_hit = (t_hit - cx) * tang[0] + (0.0 - cy) * tang[1]
        return mid_along - along_hit, width

    def _resolve_stop_dist(self):
        """Distancia de parada frontal: param > por-dock > global > desligado."""
        stop = self.get_parameter("stop_face_dist").value
        if stop is None or stop < 0.0:
            stop = self._dock_face_dist if self._dock_face_dist is not None \
                else self.get_parameter("refine_desired_face_dist").value
        return stop if stop is not None and stop >= 0.0 else None

    def _refine_target_from_scan(self, target):
        """Refina a pose alvo pela FACE da mesa no /scan.

        Corrige YAW (esquadra com a normal da face) e, se
        refine_desired_face_dist estiver calibrado (>=0), tambem X (distancia
        perpendicular a face). Y (lateral) continua por conta da localizacao
        (problema da abertura, ADR-05) — no pick, a camera fecha a lateral.

        Retorna (target_refinado, confiante: bool). Sem confianca, o chamador
        usa a pose do banco (comportamento identico ao antigo).
        """
        scan = self._last_scan
        if scan is None:
            self.get_logger().warn("refino: nenhum /scan recebido ainda.")
            return target, False
        # Guard de idade (2026-08-03): com o re-refino periodico, um /scan
        # CONGELADO (LiDAR/USB) faria o alvo seguir o robo — runaway rumo a
        # mesa. Scan velho -> sem confianca, mantem o alvo vigente.
        # Idade pela hora de CHEGADA (v4), como no muro.
        scan_age = (self.get_clock().now().nanoseconds - self._last_scan_rx_ns) * 1e-9
        if scan_age > 0.5:
            self.get_logger().warn(
                f"refino: /scan com {scan_age:.1f}s de idade; descartado.")
            return target, False
        fx_min = self.get_parameter("refine_front_min_x").value
        fx_max = self.get_parameter("refine_front_max_x").value
        hy = self.get_parameter("refine_half_width_y").value
        max_residual = self.get_parameter("refine_max_residual").value
        min_width = self.get_parameter("refine_min_face_width").value
        # Distancia desejada: mesma resolucao do muro (por-dock > global).
        desired_dist = self._resolve_stop_dist()
        desired_dist = desired_dist if desired_dist is not None else -1.0

        all_pts = self._scan_points_in_base(scan)
        if all_pts is None:
            return target, False
        pts = [p for p in all_pts if fx_min <= p[0] <= fx_max and abs(p[1]) <= hy]

        fit = _fit_line_tls(pts)
        if fit is None:
            self.get_logger().warn(
                f"refino: pontos insuficientes na janela frontal ({len(pts)} < 8).")
            return target, False
        cx, cy, nx, ny = fit
        # Normal apontando para FRENTE do robo (da base em direcao a face).
        if nx < 0:
            nx, ny = -nx, -ny
        face_dist = cx * nx + cy * ny
        face_yaw = math.atan2(ny, nx)  # orientacao da normal no frame do robo

        # Guardas anti-falso-positivo: face plana (residuo), larga o bastante
        # (nao e' perna de mesa/objeto), pouco inclinada e a distancia sensata.
        residual_rms = math.sqrt(
            sum(((p[0] - cx) * nx + (p[1] - cy) * ny) ** 2 for p in pts) / len(pts))
        tang = (-ny, nx)
        along = [ (p[0] - cx) * tang[0] + (p[1] - cy) * tang[1] for p in pts ]
        face_width = max(along) - min(along)

        # Log SEMPRE (e' o instrumento de calibracao do refine_desired_face_dist).
        self.get_logger().info(
            f"refino face: dist={face_dist:.3f} m, yaw_rel={math.degrees(face_yaw):.1f} deg, "
            f"largura={face_width:.2f} m, residuo={residual_rms*1000:.0f} mm, pts={len(pts)}")

        if residual_rms > max_residual:
            self.get_logger().warn("refino: residuo alto (face nao-plana?); descartando.")
            return target, False
        if face_width < min_width:
            self.get_logger().warn("refino: face estreita demais (perna/objeto?); descartando.")
            return target, False
        if abs(_normalize_angle(face_yaw)) > math.radians(30) or not (0.05 < face_dist < fx_max):
            self.get_logger().warn("refino: face torta/distante demais; descartando.")
            return target, False

        # Largura da face vista daqui (staging): referencia de coerencia do
        # estimador de centro perto da mesa (v4).
        self._last_face_width = face_width

        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn("refino: sem TF do robo; descartando.")
            return target, False
        rx, ry, ryaw = pose

        tx, ty, tyaw = target
        # Yaw alvo: esquadrar com a normal da face (direcao dela no mapa) —
        # MAS so quando habilitado E dois fits confiantes consecutivos
        # concordarem (2026-08-18: operador viu o refino PIORAR o yaw as
        # vezes; um fit isolado nunca mais gira o alvo sozinho).
        face_yaw_map = _normalize_angle(ryaw + face_yaw)
        apply_yaw = bool(self.get_parameter("refine_correct_yaw").value)
        if apply_yaw:
            agree_tol = self.get_parameter("refine_yaw_agree_tol").value
            if (self._last_face_yaw_map is None or
                    abs(_normalize_angle(face_yaw_map - self._last_face_yaw_map))
                    > agree_tol):
                self.get_logger().info(
                    "refino yaw: aguardando 2a leitura concordante "
                    f"(esta: {math.degrees(face_yaw_map):.1f} deg no mapa).")
                apply_yaw = False
        self._last_face_yaw_map = face_yaw_map
        refined_yaw = face_yaw_map if apply_yaw else tyaw

        refined_tx, refined_ty = tx, ty
        if desired_dist >= 0.0:
            # Plano da face no mapa: p . n_map = c_face. Move o alvo do banco ao
            # longo da normal para que a distancia final robo->face = calibrada.
            n_map = (math.cos(ryaw + face_yaw), math.sin(ryaw + face_yaw))
            c_face = rx * n_map[0] + ry * n_map[1] + face_dist
            desired_proj = c_face - desired_dist
            shift = desired_proj - (tx * n_map[0] + ty * n_map[1])
            refined_tx = tx + shift * n_map[0]
            refined_ty = ty + shift * n_map[1]
            self.get_logger().info(
                f"refino aplicado: X ajustado {shift*100:+.1f} cm ao longo da normal "
                f"(alvo dist={desired_dist:.3f} m); yaw "
                f"{'esquadrado %+.1f deg' % math.degrees(face_yaw) if apply_yaw else 'MANTIDO (portao/desligado)'}. "
                "Y mantido do banco.")
        else:
            self.get_logger().info(
                "refino aplicado: distancia nao calibrada (X do banco); yaw "
                f"{'esquadrado %+.1f deg' % math.degrees(face_yaw) if apply_yaw else 'MANTIDO (portao/desligado)'}.")
        return (refined_tx, refined_ty, refined_yaw), True

    # ------------------------------------------------------------- Controle
    def _publish_cmd(self, vx, vy, wz):
        # Eixo PURO por construcao (v4): o piso do ESC e' por roda; comando
        # misto deixa rodas sub-piso e o driver arredonda — o movimento sai
        # torto. Qualquer mistura aqui e' bug de logica.
        assert sum(1 for v in (vx, vy, wz) if abs(v) > 1e-9) <= 1, (vx, vy, wz)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self._cmd_pub.publish(msg)

    # ---------------------------------------------- Primitivas do fluxo v4
    class _AttemptAbort(Exception):
        """Encerra a tentativa de dentro das primitivas (hard_stop, cancel,
        anomalia). `outcome` e' o mesmo vocabulario de _run_attempt."""

        def __init__(self, outcome, info=None):
            super().__init__(outcome)
            self.outcome = outcome
            self.info = info

    def _check_guards(self, goal_handle, deadline, attempt_end):
        """Cancel/timeouts/hard_min — chamado a cada ciclo de QUALQUER
        primitiva. Devolve (front, npts, face_yaw) do muro."""
        if goal_handle.is_cancel_requested:
            self._publish_cmd(0.0, 0.0, 0.0)
            raise self._AttemptAbort("canceled")
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s > deadline:
            self._publish_cmd(0.0, 0.0, 0.0)
            raise self._AttemptAbort("timeout", "timeout no alinhamento")
        if now_s > attempt_end:
            self._publish_cmd(0.0, 0.0, 0.0)
            raise self._AttemptAbort(
                "soft_timeout", "tentativa estourou o orcamento")
        front, npts, age, face_yaw = self._front_wall_distance()
        self._wall_age = age
        if front is not None:
            self._saw_face_this_goal = True
        hard_min = self.get_parameter("hard_min_face_dist").value
        if (bool(self.get_parameter("wall_enabled").value)
                and front is not None and front < hard_min):
            self._publish_cmd(0.0, 0.0, 0.0)
            raise self._AttemptAbort(
                "hard_stop", f"muro: face a {front:.3f}m < hard_min {hard_min:.3f}m")
        return front, npts, face_yaw

    def _settle(self, goal_handle, rate, deadline, attempt_end, phase_name):
        """Para (publicando zero) ate o front ficar ESTAVEL em N scans novos
        ou ate o teto de tempo. Devolve (front, npts, face_yaw_mediana).
        front None = sem medicao (STALE/BLIND) apos o teto."""
        n_stable = int(self.get_parameter("settle_stable_scans").value)
        eps = self.get_parameter("settle_stable_eps").value
        max_s = self.get_parameter("settle_max_s").value
        start_s = self.get_clock().now().nanoseconds * 1e-9
        last_seq = -1
        hist = []
        yaws = []
        front = npts = None
        while rclpy.ok():
            front, npts, face_yaw = self._check_guards(goal_handle, deadline, attempt_end)
            self._publish_cmd(0.0, 0.0, 0.0)
            self._feedback(goal_handle, phase_name, 0.0, 0.0, 0.0)
            if self._scan_seq != last_seq and front is not None:
                last_seq = self._scan_seq
                hist.append(front)
                if face_yaw is not None:
                    yaws.append(face_yaw)
                hist = hist[-n_stable:]
                yaws = yaws[-5:]
                if len(hist) == n_stable and max(hist) - min(hist) <= eps:
                    break
            if self.get_clock().now().nanoseconds * 1e-9 - start_s > max_s:
                break
            rate.sleep()
        face_yaw_f = sorted(yaws)[len(yaws) // 2] if yaws else None
        return front, npts, face_yaw_f

    def _pulse(self, goal_handle, rate, deadline, attempt_end, axis, sign,
               cycles, phase_name):
        """Pulso de `cycles` ciclos no PISO do eixo (x|y|z), com os guards a
        cada ciclo, seguido de assentamento. Devolve o resultado do _settle."""
        min_v = self.get_parameter("min_body_vel").value
        min_w = self.get_parameter("min_body_omega").value
        for _ in range(max(1, int(cycles))):
            front, _n, _fy = self._check_guards(goal_handle, deadline, attempt_end)
            if front is None:
                break  # STALE/BLIND no meio do pulso: nao anda as cegas
            if axis == "x":
                self._publish_cmd(math.copysign(min_v, sign), 0.0, 0.0)
            elif axis == "y":
                self._publish_cmd(0.0, math.copysign(min_v, sign), 0.0)
            else:
                self._publish_cmd(0.0, 0.0, math.copysign(min_w, sign))
            self._feedback(goal_handle, phase_name, 0.0, 0.0, 0.0)
            rate.sleep()
        self._publish_cmd(0.0, 0.0, 0.0)
        return self._settle(goal_handle, rate, deadline, attempt_end, phase_name)

    def _cut_drive(self, goal_handle, rate, deadline, attempt_end, axis, sign,
                   error_fn, coast, max_s, phase_name):
        """Movimento CONTINUO no piso do eixo, cortado quando o erro vivo
        (error_fn() -> valor com sinal, ou None) fica <= coast — o ESC retem a
        referencia ~0,5 s e termina o curso. Sem medicao por 3 scans novos ou
        inversao de sinal: corta. Devolve o resultado do _settle."""
        min_v = self.get_parameter("min_body_vel").value
        min_w = self.get_parameter("min_body_omega").value
        start_s = self.get_clock().now().nanoseconds * 1e-9
        miss = 0
        last_seq = -1
        while rclpy.ok():
            front, _n, _fy = self._check_guards(goal_handle, deadline, attempt_end)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - start_s > max_s:
                break
            if self._scan_seq != last_seq:
                last_seq = self._scan_seq
                err = error_fn(front)
                if err is None:
                    miss += 1
                    if miss >= 3:
                        break
                else:
                    miss = 0
                    if err * sign <= coast:
                        break  # falta so o coast: corta e deixa o ESC terminar
            if axis == "x":
                self._publish_cmd(math.copysign(min_v, sign), 0.0, 0.0)
            elif axis == "y":
                self._publish_cmd(0.0, math.copysign(min_v, sign), 0.0)
            else:
                self._publish_cmd(0.0, 0.0, math.copysign(min_w, sign))
            self._feedback(goal_handle, phase_name, 0.0, 0.0, 0.0)
            rate.sleep()
        self._publish_cmd(0.0, 0.0, 0.0)
        return self._settle(goal_handle, rate, deadline, attempt_end, phase_name)

    def _measure_center(self, goal_handle, rate, deadline, attempt_end):
        """Mediana de N medidas do centro da face em scans DISTINTOS; None se
        o estimador nao confiou ou as medidas divergiram."""
        n = int(self.get_parameter("center_samples").value)
        spread_max = self.get_parameter("center_spread_max").value
        vals = []
        widths = []
        last_seq = -1
        tries = 0
        start_s = self.get_clock().now().nanoseconds * 1e-9
        while len(vals) < n and tries < 4 * n:
            if self.get_clock().now().nanoseconds * 1e-9 - start_s > 3.0:
                break  # sem scans novos (STALE): nao esperar o orcamento inteiro
            front, _npts, _fy = self._check_guards(goal_handle, deadline, attempt_end)
            self._publish_cmd(0.0, 0.0, 0.0)
            if self._scan_seq != last_seq:
                last_seq = self._scan_seq
                tries += 1
                res = self._face_center_from_scan(front)
                if res is not None:
                    vals.append(res[0])
                    widths.append(res[1])
            rate.sleep()
        if len(vals) < max(3, n - 1):
            self.get_logger().info(
                f"centro: {len(vals)}/{n} medidas validas — ultima rejeicao: {self._center_reject}")
            return None
        vals.sort()
        widths.sort()
        if vals[-1] - vals[0] > spread_max or widths[-1] - widths[0] > 2.0 * spread_max:
            self.get_logger().info(
                f"centro: medidas dispersas (centro {vals[0]*100:+.1f}..{vals[-1]*100:+.1f} cm, "
                f"largura {widths[0]:.2f}..{widths[-1]:.2f})")
            return None
        return vals[len(vals) // 2]

    def _run_attempt(self, goal_handle, bank_target, g, xy_tol, yaw_tol,
                     deadline, attempt_budget, rate, attempt, attempts,
                     staging_offset):
        """v5 (pedido do operador 2026-08-21, 'o pre-docking esta otimo'):
        do staging, (A) RETO no piso guiado pelo muro ate a distancia de
        parada (corte descontando o coast medido) e (B) UM ajuste lateral
        opcional pelo centro da face. Sem esquadro ativo, sem re, sem retry
        de aproximacao — se nao der, para onde esta (o recuo so acontece na
        falha, em _execute, para nao deixar o robo colado na mesa).

        Retorna (outcome, info) como antes; "success" -> (ex, ey, eyaw).
        """
        max_vx = self.get_parameter("max_vel_x").value
        kp = self.get_parameter("kp_linear").value
        min_v = self.get_parameter("min_body_vel").value
        period = 1.0 / float(self.get_parameter("control_frequency").value)
        band = self.get_parameter("wall_hysteresis_band").value
        coast = self.get_parameter("esc_coast_dist").value
        center_coast = self.get_parameter("center_coast_est").value
        center_enabled = bool(self.get_parameter("center_enabled").value)
        center_deadband = self.get_parameter("center_deadband").value
        center_max_travel = self.get_parameter("center_max_travel").value
        lateral_min = self.get_parameter("lateral_min_front").value
        stale_abort_s = self.get_parameter("stale_abort_s").value
        blind_wait_s = self.get_parameter("blind_wait_s").value
        hard_min = self.get_parameter("hard_min_face_dist").value

        target = bank_target
        if g.use_lidar_refine:
            self._feedback(goal_handle, "refinando", 0.0, 0.0, 0.0)
            target, confident = self._refine_target_from_scan(target)
            for _ in range(max(1, int(0.3 / period))):
                rate.sleep()
            target, confident = self._refine_target_from_scan(target)
            if not confident:
                self.get_logger().warn(
                    "refino por LiDAR sem confianca; usando pose do banco.")
        tx, ty, tyaw = target

        stop = self._resolve_stop_dist()
        if stop is None:
            return "soft_timeout", "sem distancia de parada (stop_face_dist)"
        if stop < hard_min + 0.03 - 1e-6:
            self.get_logger().warn(
                f"stop {stop:.3f} a menos de 3 cm do hard_min {hard_min:.3f}: "
                f"parada ajustada para {hard_min + 0.03:.3f}")
            stop = hard_min + 0.03
        window_lo = stop - 0.02
        window_hi = stop + band + max(0.04, xy_tol)

        start_s = self.get_clock().now().nanoseconds * 1e-9
        attempt_end = start_s + attempt_budget
        tag = f"[{g.dock_id}]"

        def log(msg):
            self.get_logger().info(f"{tag} {msg}")

        def amcl_errors():
            pose = self._robot_pose()
            if pose is None:
                return 0.0, 0.0, 0.0
            rx, ry, ryaw = pose
            dx, dy = tx - rx, ty - ry
            return (math.cos(ryaw) * dx + math.sin(ryaw) * dy,
                    -math.sin(ryaw) * dx + math.cos(ryaw) * dy,
                    _normalize_angle(tyaw - ryaw))

        # Sanidade: estou mesmo no pre-docking gravado? (dock + staging_offset
        # ao longo do yaw do dock). Longe demais = mesa errada/AMCL perdido:
        # abortar PARADO e' mais seguro que dockar na face errada.
        sanity = self.get_parameter("staging_sanity_max_m").value
        pose_now = self._robot_pose()
        if pose_now is not None and sanity > 0.0:
            sx = tx - staging_offset * math.cos(tyaw)
            sy = ty - staging_offset * math.sin(tyaw)
            d_staging = math.hypot(pose_now[0] - sx, pose_now[1] - sy)
            if d_staging > sanity:
                return "soft_timeout", (
                    f"fora do pre-docking: estou a {d_staging:.2f} m do staging "
                    f"gravado (max {sanity:.2f}) — mesa errada ou AMCL perdido; "
                    "nao vou me mover")

        # Esquadro por AMCL: so quando o refino NAO confiou (mesa diagonal =
        # Nav2 chega torto e a vista de esguelha reprova o fit). Gira longe da
        # mesa (rotate_min_front) e refaz o refino ja de frente. A guarda de
        # sanidade acima garante que o AMCL merece confianca aqui.
        if g.use_lidar_refine and not confident:
            f0, _n0, _fy0 = self._check_guards(goal_handle, deadline, attempt_end)
            eyaw0 = amcl_errors()[2]
            rotate_min = self.get_parameter("rotate_min_front").value
            yaw_coast = self.get_parameter("yaw_coast_est").value
            if abs(eyaw0) > yaw_coast + 0.02 and f0 is not None and f0 >= rotate_min:
                log(f"esquadro por AMCL: {math.degrees(eyaw0):+.1f} graus (refino sem face)")
                self._cut_drive(goal_handle, rate, deadline, attempt_end,
                                "z", 1.0 if eyaw0 > 0 else -1.0,
                                lambda _fr: amcl_errors()[2],
                                yaw_coast, abs(eyaw0) / 0.315 + 2.0, "esquadrando")
                target, confident = self._refine_target_from_scan(target)
                if confident:
                    tx, ty, tyaw = target
                    log("refino apos esquadro: ok")
                else:
                    log("refino apos esquadro: ainda sem confianca — sigo pelo banco")

        def lateral_adjust(where):
            """UM ajuste lateral pelo centro da face, se confiavel e seguro."""
            front, npts, face_yaw = self._check_guards(
                goal_handle, deadline, attempt_end)
            if not center_enabled or front is None:
                return front, npts, face_yaw
            if front < lateral_min:
                log(f"lateral({where}): front={front:.3f} < {lateral_min:.2f} — pulando")
                return front, npts, face_yaw
            c = self._measure_center(goal_handle, rate, deadline, attempt_end)
            if c is None:
                log(f"lateral({where}): centro sem confianca ({self._center_reject}) — nao mexo")
            elif abs(c) <= max(center_deadband, center_coast):
                log(f"lateral({where}): centrado ({c*100:+.1f} cm)")
            elif abs(c) > center_max_travel + center_coast:
                log(f"lateral({where}): desvio {c*100:+.1f} cm maior que o curso — nao mexo")
            elif not self._side_clear(1.0 if c > 0 else -1.0, front):
                log(f"lateral({where}): obstaculo na faixa lateral — nao mexo")
            else:
                c_hist = []

                def center_err(front_now):
                    res = self._face_center_from_scan(front_now)
                    if res is None:
                        return None
                    c_hist.append(res[0])
                    del c_hist[:-3]
                    return sorted(c_hist)[len(c_hist) // 2]

                before_c = c
                front, npts, face_yaw = self._cut_drive(
                    goal_handle, rate, deadline, attempt_end,
                    "y", 1.0 if c > 0 else -1.0, center_err, center_coast,
                    (abs(c) + 0.05) / min_v + 1.0, "centrando")
                res = self._measure_center(goal_handle, rate, deadline, attempt_end)
                log(f"lateral({where}): {before_c*100:+.1f} -> "
                    f"{'?' if res is None else '%+.1f' % (res*100)} cm")
            return front, npts, face_yaw

        try:
            # ---------- (A) ajuste lateral no STAGING (longe = mais seguro) --
            lateral_adjust("staging")

            # ---------- (B) RETO ate a janela, guiado pelo muro — e FIM -----
            waited_since = None
            while rclpy.ok():
                front, npts, face_yaw = self._check_guards(
                    goal_handle, deadline, attempt_end)
                ex, ey, eyaw = amcl_errors()
                now_s = self.get_clock().now().nanoseconds * 1e-9
                if front is None:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    waited_since = waited_since or now_s
                    self._feedback(
                        goal_handle,
                        "sem_scan" if npts < 0 else "face_invisivel",
                        ex, ey, eyaw)
                    limit = stale_abort_s if npts < 0 else blind_wait_s
                    if now_s - waited_since > limit:
                        return "soft_timeout", (
                            "STALE: /scan velho — nao avanco as cegas"
                            if npts < 0 else
                            "BLIND: face fora do plano do LiDAR")
                    rate.sleep()
                    continue
                waited_since = None
                lag = min_v * ((self._wall_age or 0.0) + period)
                if front - lag <= stop + coast:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    log(f"reto: corte a front={front:.3f} (alvo {stop:.3f} + coast {coast:.2f})")
                    break
                vx = max(min_v, min(max_vx, kp * (front - stop)))
                self._feedback(goal_handle, "aproximando", ex, ey, eyaw)
                self._publish_cmd(vx, 0.0, 0.0)
                rate.sleep()

            front, npts, face_yaw = self._settle(
                goal_handle, rate, deadline, attempt_end, "assentando")
            log(f"reto: parou em front={front if front is None else round(front, 3)} "
                f"(janela [{window_lo:.3f}, {window_hi:.3f}])")

            # ---------- (C) retoque lateral NA MESA, se ainda estiver torto --
            # (lateral puro nao avanca o bico; confirmado em campo 21/08)
            front, npts, face_yaw = lateral_adjust("mesa")

            # ---------- resultado ----------
            if front is None:
                return "soft_timeout", "sem medicao do muro apos a chegada"
            ex, ey, eyaw = amcl_errors()
            yaw_txt = "?" if face_yaw is None else f"{math.degrees(face_yaw):+.1f}"
            if front > window_hi + 0.03:
                return "soft_timeout", (
                    f"parou longe demais (front {front:.3f} > {window_hi + 0.03:.3f})")
            log(f"pronto: front={front:.3f} yaw_face={yaw_txt} graus "
                f"(erro X {((front - stop) * 100):+.1f} cm)")
            return "success", (front - stop, ey,
                               face_yaw if face_yaw is not None else eyaw)
        except self._AttemptAbort as ab:
            self._publish_cmd(0.0, 0.0, 0.0)
            return ab.outcome, ab.info

        self._publish_cmd(0.0, 0.0, 0.0)
        return "shutdown", None

    def _execute(self, goal_handle):
        """Orquestrador: N tentativas de aproximacao; entre elas, RECUO ate a
        folga de pre-docking. v5: uma tentativa por padrao (approach_retries 0
        e' aceitavel); apos falhar TUDO, recua ate o staging antes de abortar
        para nao deixar o robo colado na mesa."""
        g = goal_handle.request
        result = AlignToDock.Result()

        xy_tol = g.xy_tolerance or self.get_parameter("default_xy_tolerance").value
        yaw_tol = g.yaw_tolerance or self.get_parameter("default_yaw_tolerance").value
        timeout = g.timeout or self.get_parameter("default_timeout").value
        period = 1.0 / float(self.get_parameter("control_frequency").value)
        rate = self.create_rate(1.0 / period)

        try:
            bx, by, byaw, face_dist, staging_offset = self._load_dock_pose(
                g.dock_id, g.map_name, g.map_dir)
        except Exception as exc:  # noqa: BLE001
            return self._abort(goal_handle, result, f"falha ao carregar dock: {exc}")
        bank_target = (bx, by, byaw)
        self._dock_face_dist = face_dist
        self._last_face_yaw_map = None  # portao de consistencia zera por goal
        self._last_face_width = None    # largura de referencia e' por mesa
        # So recuar se a face foi VISTA neste goal: re cega a partir de uma
        # pose desconhecida (campo 2026-08-21: 2x 0,35 m de re sem sensor
        # traseiro) e' pior que ficar parado.
        self._saw_face_this_goal = False

        attempts = max(1, int(self.get_parameter("approach_retries").value) + 1)
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        attempt_budget = max(40.0, timeout / attempts)
        grace = float(self.get_parameter("final_retreat_grace_s").value)

        last_reason = "sem tentativa"
        for attempt in range(1, attempts + 1):
            outcome, info = self._run_attempt(
                goal_handle, bank_target, g, xy_tol, yaw_tol,
                deadline, attempt_budget, rate, attempt, attempts,
                staging_offset)

            if outcome == "success":
                ex, ey, eyaw = info
                result.success = True
                result.message = f"alinhado (tentativa {attempt}/{attempts})"
                result.final_x_error = ex
                result.final_y_error = ey
                result.final_yaw_error = eyaw
                goal_handle.succeed()
                return result
            if outcome == "canceled":
                self._publish_cmd(0.0, 0.0, 0.0)
                goal_handle.canceled()
                result.success = False
                result.message = "cancelado"
                return result
            if outcome == "shutdown":
                self._publish_cmd(0.0, 0.0, 0.0)
                return self._abort(goal_handle, result, "no encerrado")

            last_reason = info
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if outcome in ("hard_stop", "soft_timeout") and attempt < attempts \
                    and now_s < deadline and self._saw_face_this_goal:
                self.get_logger().warn(
                    f"[{g.dock_id}] tentativa {attempt}/{attempts} falhou "
                    f"({info}); recuando para tentar de novo.")
                self._feedback(goal_handle, "recuando", 0.0, 0.0, 0.0)
                if self._retreat(goal_handle, staging_offset, rate,
                                 deadline, period) == "canceled":
                    goal_handle.canceled()
                    result.success = False
                    result.message = "cancelado durante o recuo"
                    return result
                continue
            break

        # Falhou tudo: RECUA ate o staging ANTES de abortar (nao deixar o robo
        # colado na mesa — o proximo Nav2 giraria no lugar contra ela). Mas SO
        # se a face chegou a ser vista: sem referencia, ficar parado.
        if not self._saw_face_this_goal:
            self._publish_cmd(0.0, 0.0, 0.0)
            return self._abort(
                goal_handle, result,
                f"alinhamento falhou sem nunca ver a face ({last_reason}) — "
                "sem recuo (o robo nao esta de frente para a mesa?)")
        # Pedido do operador (2026-08-21): SEM re, a menos que tenha parado
        # perigosamente perto (a proxima navegacao giraria contra a mesa).
        front_now, _n, _a, _f = self._front_wall_distance()
        if front_now is not None and front_now >= 0.44:
            self._publish_cmd(0.0, 0.0, 0.0)
            return self._abort(
                goal_handle, result,
                f"alinhamento falhou ({last_reason}) — parado a front="
                f"{front_now:.3f} (sem recuo)")
        self.get_logger().warn(
            f"[{g.dock_id}] alinhamento falhou ({last_reason}); recuando ate o "
            "staging antes de reportar.")
        self._feedback(goal_handle, "recuando", 0.0, 0.0, 0.0)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self._retreat(goal_handle, staging_offset, rate,
                         max(deadline, now_s) + grace, period) == "canceled":
            goal_handle.canceled()
            result.success = False
            result.message = "cancelado durante o recuo final"
            return result

        self._publish_cmd(0.0, 0.0, 0.0)
        return self._abort(
            goal_handle, result,
            f"alinhamento falhou apos {attempt} tentativa(s): {last_reason}")

    def _retreat(self, goal_handle, staging_offset, rate, deadline, period):
        """Re' pura ate a folga de pre-docking (vx<0 afasta da mesa — sempre
        seguro). Alvo: front_dist >= stop + staging_offset; fallback por
        integracao do comando quando a face nao e' visivel. Melhor esforco:
        timeout local nao aborta o goal (a proxima tentativa parte de onde
        estiver)."""
        stop_dist = self._resolve_stop_dist()
        base = stop_dist if stop_dist is not None else 0.35
        target_front = base + staging_offset
        speed = max(self.get_parameter("min_body_vel").value, 0.13)
        max_time = (staging_offset / speed) * 2.0 + 2.0
        start_s = self.get_clock().now().nanoseconds * 1e-9
        traveled = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_cmd(0.0, 0.0, 0.0)
                return "canceled"
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - start_s > max_time or now_s > deadline:
                break
            front, _npts, _age, _fyaw = self._front_wall_distance()
            if front is not None and front >= target_front:
                break
            if front is None and traveled >= staging_offset:
                break
            self._publish_cmd(-speed, 0.0, 0.0)
            traveled += speed * period
            rate.sleep()

        self._publish_cmd(0.0, 0.0, 0.0)
        self.get_logger().info(
            f"recuo concluido (~{traveled:.2f}m comandados).")
        return "ok"

    def _feedback(self, goal_handle, phase, ex, ey, eyaw):
        fb = AlignToDock.Feedback()
        fb.phase = phase
        fb.x_error = ex
        fb.y_error = ey
        fb.yaw_error = eyaw
        goal_handle.publish_feedback(fb)

    def _abort(self, goal_handle, result, message):
        self.get_logger().warn(message)
        result.success = False
        result.message = message
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DockAlignNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
