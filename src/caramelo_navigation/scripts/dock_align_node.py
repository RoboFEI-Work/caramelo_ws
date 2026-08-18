#!/usr/bin/env python3
"""Alinhamento fino HOLONOMICO a um dock (ADR-05, opcao c).

Complementa o opennav_docking: o controller do Docking Server e' unicycle
(so linear.x + angular.z, sem vy), entao ele nao usa as rodas mecanum para
corrigir lateralmente. Este no faz o trecho FINAL de alinhamento com holonomia
(vx, vy, wz), depois que o Docking Server ja levou o robo ate o staging/approach.

O que este no faz (reformulado 2026-08-18 — "docking que nunca bate"):
  1. Le a pose alvo do dock em docking.yaml (mesma base do opennav_docking).
  2. (Opcional) refina a pose alvo pela FACE da mesa no /scan -> corrige a
     distancia (X) e o yaw, que sao observaveis numa face reta.
  3. MURO DE SEGURANCA por LiDAR a cada ciclo (20 Hz): a distancia frontal
     REAL a face/obstaculo limita o avanco (APPROACH->HOLD com histerese);
     abaixo do hard_min o no RECUA ate o pre-docking e tenta de novo
     (approach_retries), so falhando depois de esgotar — e a camada de missao
     decide pular a mesa. Scan velho/ausente = nao avanca.
  4. Controle por EIXO sequencial (YAW -> Y -> X): o piso do ESC so executa
     exato em eixo puro; o X e dirigido pela distancia REAL do LiDAR quando
     disponivel (mata o vies do AMCL), com fallback no alvo do banco (BLIND).

LIMITES HONESTOS (ADR-05):
  - Y (lateral ao longo da face) NAO e' observavel numa face reta (problema da
    abertura). Este no deixa o Y por conta da localizacao (AMCL/EKF, ~2-5 cm).
    Corrigir Y de verdade exige detectar os CANTOS da mesa (casar retangulo
    ~80x50 cm) -> marcado como AJUSTAR_NO_ROBO em _refine_target_from_scan().
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
        self.declare_parameter("max_vel_x", 0.20)
        self.declare_parameter("max_vel_y", 0.16)
        # 0.9 -> 0.5 (2026-08-03): coerente com o max_vel_theta do DWB; giro
        # lento o bastante p/ o AMCL acompanhar e nao passar do alvo no fino.
        self.declare_parameter("max_vel_theta", 0.5)
        # Piso fisico do ESC pos-fix do firmware (2026-08-03): 650rpm = 0.1215
        # m/s / 0.315 rad/s de corpo, executado EXATO (mapa ancorado 1570us).
        self.declare_parameter("min_body_vel", 0.1215)
        self.declare_parameter("min_body_omega", 0.315)
        self.declare_parameter("kp_linear", 1.2)
        self.declare_parameter("kp_theta", 1.5)
        self.declare_parameter("default_xy_tolerance", 0.03)
        # 0.05 -> 0.09 (teste 3 em 2026-08-18): a leitura de yaw da face
        # oscila +-4 graus entre fits — exigir 2.9 e' pedir menos que o ruido
        # e o robo fica em retoque infinito ate o timeout. 5 graus fecha, e o
        # pick compensa o residuo pela camera/tag.
        self.declare_parameter("default_yaw_tolerance", 0.09)
        # 30 -> 60 (2026-08-18): o align agora faz o approach INTEIRO
        # (staging->dock, ~45cm) e o orcamento cobre recuo+retries.
        self.declare_parameter("default_timeout", 60.0)
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
        self.declare_parameter("wall_hysteresis_band", 0.005)
        # Abaixo disso: para tudo e RECUA para tentar de novo (chassi frontal
        # fica a 0.285m do centro; 0.32 = ~3.5cm de folga). Calibrar em campo.
        self.declare_parameter("hard_min_face_dist", 0.32)
        self.declare_parameter("wall_max_scan_age", 0.5)
        self.declare_parameter("wall_min_points", 5)
        # Corredor frontal = meia-largura do chassi 0.195 + 1.5cm de margem.
        self.declare_parameter("wall_corridor_half_width", 0.21)
        self.declare_parameter("wall_max_range", 1.5)
        # k-esimo menor x (fracao dos pontos do corredor): rejeita speckle
        # isolado sem esconder obstaculo real.
        self.declare_parameter("wall_percentile", 0.10)
        # Teto de vx proporcional a folga ate o stop (alem do perfil P).
        self.declare_parameter("wall_kp", 1.5)
        # Face nao vista (mesa baixa fora do plano do LiDAR): avanco permitido
        # devagar, X pelo alvo do banco. >= 1.08x piso 0.1215 senao o floor
        # infla de volta.
        self.declare_parameter("blind_max_vx", 0.13)
        # RECUO+RETRY (pedido do operador 2026-08-18): em hard-stop/timeout de
        # aproximacao, recua ate a distancia de pre-docking e tenta de novo
        # este numero de vezes antes de reportar falha (a missao pula a mesa).
        self.declare_parameter("approach_retries", 2)

        # ------- Fluxo v3 (missao 2026-08-18, pedido do operador): chegou no
        # pre-docking -> FRENTE direto ate a distancia segura -> so entao
        # apara o angulo se precisar. SEM fases de ajuste no staging (o
        # pingue-pongue lateral/yaw com o ruido do AMCL comia o orcamento e
        # o robo "nao ia pra frente"), e o AMCL FORA do criterio de sucesso.
        # Na RETA: so para p/ aparar o angulo se a face do LiDAR passar disso
        # (mediana de 5; nao interromper por menos que o ruido de +-4 graus).
        self.declare_parameter("drive_yaw_fix_tol", 0.09)
        # Correcao lateral UNICA (feed-forward, no maximo 2 rajadas) antes da
        # reta, so se o AMCL indicar desvio maior que isto. Depois disso o Y
        # nunca mais e' tocado — residuo lateral e' problema da camera no pick.
        self.declare_parameter("y_oneshot_tol", 0.05)
        # BLIND (mesa baixa, LiDAR nao ve a face): teto de avanco por
        # integracao do comando = staging_offset + esta margem — foi por aqui
        # que o robo BATEU na missao (dirigia pelo AMCL sem limite).
        self.declare_parameter("blind_travel_margin", 0.05)

        # ------- Refino de yaw sob suspeita (operador 2026-08-18: "as vezes
        # piora"): so aplica a correcao quando 2 fits confiantes consecutivos
        # concordam; e da para desligar de vez por parametro.
        self.declare_parameter("refine_correct_yaw", True)
        self.declare_parameter("refine_yaw_agree_tol", 0.035)

        # ------- PULSO-E-ESPERA no regime do piso (teste 1 em 2026-08-18: o
        # comando continuo no piso invertia de sinal a cada cruzamento e o
        # robo virava PENDULO de +-20 graus — momento + coast do ESC). Quando
        # o comando P fica ABAIXO do piso, manda o piso por pulse_on_cycles
        # ciclos e espera floor_coast_cycles parado antes de reavaliar.
        # 10 -> 6 (teste 3): com pulsos de 200ms o momento e' pequeno; 0.5s de
        # espera deixava a fase de giro com 13s. 0.3s assenta e anda 40% mais.
        self.declare_parameter("floor_coast_cycles", 6)
        # Teste 2 em 2026-08-18: pulso de 1 ciclo (50ms) no piso NAO vence a
        # inercia/partida do ESC — o robo "nem mexeu". Se o pulso nao mover o
        # erro, o proximo ESCALA +2 ciclos (ate o max); moveu, volta ao base.
        # 4 -> 2 (pedido do operador pos-missao: "girar mais devagar"): pulso
        # base de 100ms = ~1.8 graus; o escalonamento cobre o atrito sozinho.
        self.declare_parameter("pulse_on_cycles", 2)
        self.declare_parameter("pulse_on_max_cycles", 12)
        # Anti-zigue-zague na RETA (missao 2026-08-18, WS2 com cena suja): o
        # desvio de angulo so interrompe a reta depois de N ciclos seguidos
        # acima do limiar; cada sessao de aparo dura no maximo
        # drive_yaw_fix_max_s e depois ha um refratario sem novos aparos.
        self.declare_parameter("drive_yaw_fix_cycles", 6)
        self.declare_parameter("drive_yaw_fix_max_s", 2.5)
        self.declare_parameter("drive_yaw_fix_cooldown_s", 1.5)
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
        # Re-refino periodico (s) durante o loop de alinhamento (2026-08-03):
        # o AMCL agora corrige map->odom DURANTE o docking (update_min 0.05) e a
        # pose refinada de 1x so envelhece; re-ancorar na face real a cada ~2s.
        # Os guards (residual/largura) descartam fit ruim e mantem o alvo vigente.
        # <= 0 desliga (comportamento antigo: refina 1x antes do loop).
        self.declare_parameter("refine_period", 2.0)
        # Trava de salto dos RE-refinos periodicos (2026-08-03): a mesa nao se
        # move — correcao maior que isso vs alvo vigente = fit ruim, descarta.
        # O 1o refino (pre-loop) fica livre: e ele que come o offset do AMCL
        # (ja medimos 11.4 graus legitimos).
        self.declare_parameter("refine_max_step_xy", 0.04)
        self.declare_parameter("refine_max_step_yaw", 0.14)

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
        # Centro lateral da mesa medido pelo refino (bordas visiveis) — usado
        # pelo Y1; None = sem medicao (cai no AMCL).
        self._face_center_y = None

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
        if scan is None:
            return None, -1, float("inf"), None
        scan_age = (
            self.get_clock().now() - Time.from_msg(scan.header.stamp)
        ).nanoseconds * 1e-9
        if scan_age > self.get_parameter("wall_max_scan_age").value:
            return None, -1, scan_age, None
        tfm = self._wall_tf(scan.header.frame_id)
        if tfm is None:
            return None, -1, scan_age, None
        r00, r01, r10, r11, ox, oy = tfm

        max_range = self.get_parameter("wall_max_range").value
        half_w = self.get_parameter("wall_corridor_half_width").value
        # Pre-crop por range ANTES de transformar (custo Python): folga = o
        # deslocamento do LiDAR em relacao ao base_frame.
        range_cut = max_range + math.hypot(ox, oy) + 0.05

        pts = []
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
            if abs(py) <= half_w:
                pts.append((px, py))

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
        scan_age = (
            self.get_clock().now() - Time.from_msg(scan.header.stamp)
        ).nanoseconds * 1e-9
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

        # Centro LATERAL da face (2026-08-18, "nao para no centro da mesa"):
        # quando as DUAS bordas aparecem (largura plausivel de mesa 0.8m), o
        # ponto medio da extensao e' o centro da mesa no frame do robo —
        # centragem do Y1 por LiDAR, sem depender do AMCL.
        if 0.55 <= face_width <= 0.95:
            mid_along = 0.5 * (max(along) + min(along))
            self._face_center_y = cy + mid_along * tang[1]
            self.get_logger().info(
                f"refino: centro da mesa a {self._face_center_y*100:+.1f} cm "
                "lateral (bordas visiveis).")

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
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self._cmd_pub.publish(msg)

    def _apply_floor(self, vx, vy, wz, moving_xy, moving_yaw):
        """Respeita o piso do ESC: se a base precisa se mover mas o comando ficou
        abaixo do minimo executavel, sobe para o minimo (senao nao anda). Isso
        causa movimento em degrau perto do goal — limite fisico ate o firmware."""
        min_v = self.get_parameter("min_body_vel").value
        min_w = self.get_parameter("min_body_omega").value
        speed = math.hypot(vx, vy)
        if moving_xy and 0.0 < speed < min_v:
            scale = min_v / speed
            vx *= scale
            vy *= scale
        if moving_yaw and 0.0 < abs(wz) < min_w:
            wz = math.copysign(min_w, wz)
        return vx, vy, wz

    def _execute(self, goal_handle):
        """Orquestrador: N tentativas de aproximacao; entre elas, RECUO ate a
        folga de pre-docking (pedido do operador 2026-08-18 — nao perder a
        task: quem pula a mesa e a camada de missao, depois que TODAS as
        tentativas daqui falharem)."""
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

        attempts = max(1, int(self.get_parameter("approach_retries").value) + 1)
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        # Piso de 25s por tentativa (teste 3: pulsos sao lentos — 20s morria
        # no meio dos retoques finais); o deadline global segue mandando.
        attempt_budget = max(25.0, timeout / attempts)

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
                    and now_s < deadline:
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

        self._publish_cmd(0.0, 0.0, 0.0)
        return self._abort(
            goal_handle, result,
            f"alinhamento falhou apos {attempt} tentativa(s): {last_reason}")

    def _run_attempt(self, goal_handle, bank_target, g, xy_tol, yaw_tol,
                     deadline, attempt_budget, rate, attempt, attempts,
                     staging_offset):
        """Uma tentativa completa de aproximacao (fases YAW -> Y -> X + muro).

        Retorna (outcome, info): "success" (info=(ex,ey,eyaw)), "canceled",
        "shutdown", "hard_stop"/"soft_timeout"/"timeout" (info=motivo).
        """
        max_vx = self.get_parameter("max_vel_x").value
        max_vy = self.get_parameter("max_vel_y").value
        max_wz = self.get_parameter("max_vel_theta").value
        kp = self.get_parameter("kp_linear").value
        kp_t = self.get_parameter("kp_theta").value
        min_v = self.get_parameter("min_body_vel").value
        min_w = self.get_parameter("min_body_omega").value
        settle_target = int(self.get_parameter("settle_cycles").value)
        period = 1.0 / float(self.get_parameter("control_frequency").value)
        refine_period = float(self.get_parameter("refine_period").value)
        max_step_xy = float(self.get_parameter("refine_max_step_xy").value)
        max_step_yaw = float(self.get_parameter("refine_max_step_yaw").value)

        wall_on = bool(self.get_parameter("wall_enabled").value)
        band = self.get_parameter("wall_hysteresis_band").value
        hard_min = self.get_parameter("hard_min_face_dist").value
        wall_kp = self.get_parameter("wall_kp").value
        blind_max_vx = self.get_parameter("blind_max_vx").value

        drive_yaw_fix = self.get_parameter("drive_yaw_fix_tol").value
        y_oneshot_tol = self.get_parameter("y_oneshot_tol").value

        floor_coast = int(self.get_parameter("floor_coast_cycles").value)
        pulse_base = max(1, int(self.get_parameter("pulse_on_cycles").value))
        pulse_max = max(
            pulse_base, int(self.get_parameter("pulse_on_max_cycles").value))
        pulse_state = {"on": 0, "len": pulse_base, "err0": None}

        def pulse_axis(raw, floor, moving, err_now):
            """Comando do eixo ativo com pulso-e-espera no regime do piso.

            |raw| >= piso: drive continuo (erro grande). Abaixo do piso e
            ainda fora da tolerancia: pulso de pulse_len ciclos no piso +
            floor_coast ciclos parado — sem acumular momento (o pendulo do
            teste 1). Pulso que NAO moveu o erro (atrito/partida do ESC,
            teste 2: "nem mexeu") escala +2 ciclos ate pulse_max; moveu,
            volta ao base."""
            nonlocal coast_countdown
            if not moving:
                coast_countdown = 0
                pulse_state["on"] = 0
                pulse_state["err0"] = None
                return 0.0
            if abs(raw) >= floor:
                coast_countdown = 0
                pulse_state["on"] = 0
                pulse_state["err0"] = None
                return raw
            if pulse_state["on"] > 0:
                pulse_state["on"] -= 1
                return math.copysign(floor, raw)
            if coast_countdown > 0:
                coast_countdown -= 1
                return 0.0
            if pulse_state["err0"] is not None:
                if abs(err_now - pulse_state["err0"]) < 0.004:
                    pulse_state["len"] = min(pulse_state["len"] + 2, pulse_max)
                else:
                    pulse_state["len"] = pulse_base
            pulse_state["err0"] = err_now
            pulse_state["on"] = pulse_state["len"] - 1
            coast_countdown = floor_coast
            return math.copysign(floor, raw)

        target = bank_target
        if g.use_lidar_refine:
            self._feedback(goal_handle, "refinando", 0.0, 0.0, 0.0)
            target, confident = self._refine_target_from_scan(target)
            # 2a leitura logo em seguida destrava o portao de consistencia do
            # yaw (que exige dois fits concordantes) sem esperar o re-refino.
            for _ in range(max(1, int(0.3 / period))):
                rate.sleep()
            target, confident = self._refine_target_from_scan(target)
            if not confident:
                self.get_logger().warn(
                    "refino por LiDAR sem confianca; usando pose do banco.")

        tx, ty, tyaw = target
        start_s = self.get_clock().now().nanoseconds * 1e-9
        settle = 0
        phase = "Y1"
        prev_phase = phase
        coast_countdown = 0
        wall_hold = False
        last_refine = self.get_clock().now()
        stale_logged = False
        face_hist = []  # mediana de 5 do yaw da face (fit oscila +-4 graus)
        y_bursts = 0
        y_burst_cycles = 0
        y_burst_dir = 1.0
        blind_travel = 0.0
        blind_cap = staging_offset + float(
            self.get_parameter("blind_travel_margin").value)
        self._face_center_y = None  # re-mede por tentativa (refino inicial)
        yaw_over = 0
        yaw_fixing = False
        fix_bout_end_s = 0.0
        fix_cooldown_until_s = 0.0
        drive_fix_cycles = int(self.get_parameter("drive_yaw_fix_cycles").value)
        drive_fix_max_s = float(self.get_parameter("drive_yaw_fix_max_s").value)
        drive_fix_cooldown_s = float(
            self.get_parameter("drive_yaw_fix_cooldown_s").value)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_cmd(0.0, 0.0, 0.0)
                return "canceled", None

            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s > deadline:
                self._publish_cmd(0.0, 0.0, 0.0)
                return "timeout", "timeout no alinhamento"
            if now_s - start_s > attempt_budget:
                self._publish_cmd(0.0, 0.0, 0.0)
                return "soft_timeout", (
                    f"tentativa estourou {attempt_budget:.0f}s")

            # Re-refino periodico (re-ancora o alvo na face; guards mantem o
            # alvo vigente sem confianca).
            if (g.use_lidar_refine and refine_period > 0.0
                    and (self.get_clock().now() - last_refine).nanoseconds
                    * 1e-9 >= refine_period):
                last_refine = self.get_clock().now()
                new_target, confident = self._refine_target_from_scan(
                    (tx, ty, tyaw))
                if confident:
                    step_xy = math.hypot(new_target[0] - tx, new_target[1] - ty)
                    step_yaw = abs(_normalize_angle(new_target[2] - tyaw))
                    if step_xy <= max_step_xy and step_yaw <= max_step_yaw:
                        tx, ty, tyaw = new_target
                    else:
                        self.get_logger().warn(
                            "re-refino descartado: salto implausivel "
                            f"(dxy={step_xy * 100:.1f}cm, "
                            f"dyaw={math.degrees(step_yaw):.1f}deg)")

            pose = self._robot_pose()
            if pose is None:
                rate.sleep()
                continue
            rx, ry, ryaw = pose

            dx = tx - rx
            dy = ty - ry
            ex = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            ey = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
            eyaw = _normalize_angle(tyaw - ryaw)
            dist = math.hypot(ex, ey)

            # ------------- MURO + face: mede SEMPRE (hard_min vale em toda
            # fase). face_yaw = angulo da face no frame do ROBO (LiDAR) — o
            # sinal de yaw da reta final, imune ao ruido de yaw do AMCL.
            front = None
            npts = -1
            face_yaw = None
            if wall_on:
                front, npts, _age, face_yaw = self._front_wall_distance()
                if front is not None and front < hard_min:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    return "hard_stop", (
                        f"muro: face a {front:.3f}m < hard_min {hard_min:.3f}m")
            stop_dist = self._resolve_stop_dist()
            lidar_mode = front is not None and stop_dist is not None
            # eyaw de CONTROLE: MEDIANA de 5 do yaw da face (o fit oscila
            # +-4 graus ciclo a ciclo) quando visivel, senao AMCL.
            if face_yaw is not None:
                face_hist.append(face_yaw)
                if len(face_hist) > 5:
                    face_hist.pop(0)
            else:
                face_hist.clear()
            face_yaw_f = (
                sorted(face_hist)[len(face_hist) // 2] if face_hist else None)
            eyaw_ctrl = face_yaw_f if face_yaw_f is not None else eyaw

            # ------------- Sucesso (v3): quem decide e' o SENSOR — front na
            # janela de parada e face esquadrada (mediana). O AMCL SAIU do
            # criterio (vies >3cm travava o docking; lateral residual e'
            # problema da camera no pick). Sem face (BLIND), vale o AMCL.
            if lidar_mode:
                in_window = (
                    stop_dist - 0.03 <= front <= stop_dist + band + 0.01)
                yaw_ok = face_yaw_f is None or abs(face_yaw_f) <= yaw_tol
                if phase == "FINISH" and in_window and yaw_ok:
                    settle += 1
                    self._publish_cmd(0.0, 0.0, 0.0)
                    if settle >= settle_target:
                        return "success", (front - stop_dist, ey, eyaw_ctrl)
                    rate.sleep()
                    continue
            else:
                if dist <= xy_tol and abs(eyaw) <= yaw_tol:
                    settle += 1
                    self._publish_cmd(0.0, 0.0, 0.0)
                    if settle >= settle_target:
                        return "success", (ex, ey, eyaw)
                    rate.sleep()
                    continue
            settle = 0

            # ------------- Fases v3 (pedido do operador na missao de
            # 2026-08-18): Y1 (correcao lateral UNICA, opcional, LONGE da
            # mesa) -> DRIVE (reto ate a parada) -> FINISH (apara so o
            # angulo). Nada de lateral perto da mesa: o muro so cobre o
            # avanco frontal, e foi a dancinha lateral/giro colada na mesa
            # que encostou na missao.
            if phase == "Y1" and y_burst_cycles <= 0:
                # Fonte do desvio lateral: centro da mesa por LiDAR quando as
                # bordas aparecem (missao 2026-08-18: "nao para no centro"),
                # senao o AMCL.
                ey_src = (self._face_center_y
                          if self._face_center_y is not None else ey)
                if abs(ey_src) > y_oneshot_tol and y_bursts < 2:
                    y_burst_cycles = max(1, int(abs(ey_src) / min_v / period))
                    y_burst_dir = 1.0 if ey_src > 0 else -1.0
                    y_bursts += 1
                    self._face_center_y = None  # consome; re-refino re-mede
                else:
                    phase = "DRIVE"
            if phase == "DRIVE" and lidar_mode and front <= stop_dist + band:
                phase = "FINISH"
            elif phase == "FINISH" and lidar_mode and \
                    front > stop_dist + band + 0.03:
                phase = "DRIVE"  # afastou de verdade: re-aproxima

            if phase != prev_phase:
                coast_countdown = 0
                pulse_state["on"] = 0
                pulse_state["len"] = pulse_base
                pulse_state["err0"] = None
                prev_phase = phase

            vx = vy = wz = 0.0
            fb_phase = "alinhando"
            if phase == "Y1":
                # Rajada lateral feed-forward (dead-reckoning no piso, na
                # distancia do staging): anda |ey| de uma vez e pronto — sem
                # o pingue-pongue de perseguir o ruido do AMCL.
                fb_phase = "lateral"
                vy = y_burst_dir * min_v
                y_burst_cycles -= 1
            elif phase == "DRIVE":
                # Aparo de angulo com HISTERESE (missao/WS2: cena suja fazia
                # a reta parar a cada segundo p/ perseguir ruido): so entra
                # apos N ciclos seguidos acima do limiar, dura no maximo
                # drive_yaw_fix_max_s e respeita um refratario depois.
                if (face_yaw_f is not None
                        and abs(face_yaw_f) > drive_yaw_fix
                        and now_s >= fix_cooldown_until_s):
                    yaw_over += 1
                else:
                    yaw_over = 0
                if yaw_fixing:
                    if (face_yaw_f is None
                            or abs(face_yaw_f) <= 0.6 * drive_yaw_fix
                            or now_s >= fix_bout_end_s):
                        yaw_fixing = False
                        fix_cooldown_until_s = now_s + drive_fix_cooldown_s
                        yaw_over = 0
                elif yaw_over >= drive_fix_cycles:
                    yaw_fixing = True
                    fix_bout_end_s = now_s + drive_fix_max_s

                if yaw_fixing:
                    fb_phase = "girando"
                    wz = pulse_axis(
                        _clamp(kp_t * (face_yaw_f or 0.0), max_wz), min_w,
                        True, face_yaw_f or 0.0)
                elif lidar_mode:
                    # RETA CONTINUA (sem pulsos): piso ate entrar na banda —
                    # a 0.12 m/s o passo de deteccao e' ~6mm por ciclo.
                    fb_phase = "aproximando"
                    vx = max(min_v, min(max_vx, kp * (front - stop_dist)))
                else:
                    # BLIND (face fora do plano do LiDAR): avanco LENTO pelo
                    # AMCL com TETO por integracao do comando — nunca dirigir
                    # sem limite atras de um alvo de mapa.
                    fb_phase = "aproximando"
                    if blind_travel >= blind_cap:
                        self._publish_cmd(0.0, 0.0, 0.0)
                        return "hard_stop", (
                            f"BLIND: avancei {blind_travel:.2f}m sem ver a "
                            f"face (teto {blind_cap:.2f}m) e o AMCL nao "
                            "confirmou chegada")
                    if ex > xy_tol:
                        vx = min(blind_max_vx, max(min_v, kp * ex))
                        blind_travel += vx * period
            else:  # FINISH — na mesa: SO apara o angulo; re se entrou >3cm.
                fb_phase = "encostado"
                if face_yaw_f is not None and abs(face_yaw_f) > yaw_tol:
                    wz = pulse_axis(
                        _clamp(kp_t * face_yaw_f, max_wz), min_w, True,
                        face_yaw_f)
                elif lidar_mode and front < stop_dist - 0.03:
                    vx = pulse_axis(
                        _clamp(kp * (front - stop_dist), max_vx), min_v, True,
                        front - stop_dist)

            # ------------- Portao do muro POR ULTIMO (depois do floor, senao
            # o floor re-infla o vx que o muro cortou).
            if wall_on and vx > 0.0:
                if front is None:
                    if npts < 0:
                        vx = 0.0  # STALE/sem TF: nao avanca as cegas
                        if not stale_logged:
                            self.get_logger().warn(
                                "muro: sem medicao valida do scan — avanco "
                                "bloqueado (STALE).")
                            stale_logged = True
                    else:
                        vx = min(vx, blind_max_vx)  # BLIND: devagar, X do banco
                else:
                    stale_logged = False
                    if stop_dist is not None:
                        if wall_hold:
                            if front >= stop_dist + band:
                                wall_hold = False
                        elif front <= stop_dist:
                            wall_hold = True
                        if wall_hold:
                            vx = 0.0
                            fb_phase = "segurando"
                        else:
                            vx = min(vx, wall_kp * (front - stop_dist))
                # Muro deixou o vx abaixo do piso executavel? Zera neste ciclo
                # (vy/wz seguem) — melhor parado que inflado contra a mesa.
                if 0.0 < vx < min_v:
                    vx = 0.0

            self._feedback(goal_handle, fb_phase, ex, ey, eyaw)
            self._publish_cmd(vx, vy, wz)
            rate.sleep()

        self._publish_cmd(0.0, 0.0, 0.0)
        return "shutdown", None

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
