#!/usr/bin/env python3
"""Visualizacao de diagnostico da corrida (2026-11-13).

Pedido do operador: ver no RViz (e no video da corrida) o que o Nav2 nao
publica sozinho. So' publica o que nao existe em nenhum outro topico:
  /diag/trilha_real    nav_msgs/Path   trajetoria percorrida (TF map->base_footprint,
                                       o que o robo ACREDITAVA; saltos = AMCL corrigindo)
  /diag/plano_anterior nav_msgs/Path   plano global anterior, so' quando um replan
                                       muda a rota (desvio medio > desvio_replan_m)
  /diag/eventos        MarkerArray     esfera vermelha + texto na pose do robo a cada
                                       WARN/ERROR dos nos de navegacao e a cada acao do
                                       collision_monitor
O resto vem direto dos topicos do Nav2 (/plan, /local_plan, costmaps): sem
republicar. Nao assina /marker, /evaluation nem /trajectories (o DWB/MPPI so'
montam essas mensagens quando ha assinante, dentro do laco de 20 Hz).
Quem sobe: caramelo_run_common.iniciar_run() (um por corrida, morre no fim).
Nao comanda nada: so' le TF/rosout/plan e publica em /diag/*.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # pragma: no cover - nav2_msgs sempre existe no robo
    CollisionMonitorState = None

# Nos cujos WARN/ERROR viram evento vermelho (prefixo do nome do logger).
NOS_NAV = (
    "controller_server", "planner_server", "bt_navigator", "behavior_server",
    "smoother_server", "velocity_smoother", "collision_monitor", "amcl",
    "localization_watchdog", "dock_align", "docking_server", "local_costmap",
    "global_costmap", "bt_yaml_executor", "map_server",
)
ACOES_CM = {1: "STOP", 2: "SLOWDOWN", 3: "APPROACH", 4: "LIMIT"}


def amostrar(pontos, n=40):
    """No maximo n pontos igualmente espacados (por indice)."""
    if len(pontos) <= n:
        return list(pontos)
    passo = (len(pontos) - 1) / (n - 1)
    return [pontos[round(i * passo)] for i in range(n)]


def desvio_medio(a, b):
    """Distancia media dos pontos de a ate o ponto mais proximo de b (m)."""
    if not a or not b:
        return float("inf")
    aa, bb = amostrar(a), amostrar(b, 80)
    return sum(min(math.hypot(x - u, y - v) for u, v in bb) for x, y in aa) / len(aa)


def mesmo_destino(a, b, tol=0.30):
    return bool(a and b) and math.hypot(a[-1][0] - b[-1][0], a[-1][1] - b[-1][1]) < tol


class RunDiagViz(Node):
    def __init__(self):
        super().__init__("run_diag_viz")
        self.declare_parameter("frame_mapa", "map")
        self.declare_parameter("frame_robo", "base_footprint")
        self.declare_parameter("passo_trilha_m", 0.02)
        self.declare_parameter("desvio_replan_m", 0.25)
        self.declare_parameter("max_eventos", 150)
        self.declare_parameter("eventos_com_texto", 4)
        self.declare_parameter("repeticao_min_s", 3.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.fmap, self.frobo = str(g("frame_mapa")), str(g("frame_robo"))
        self.passo = float(g("passo_trilha_m"))
        self.desvio_replan = float(g("desvio_replan_m"))
        self.max_ev = int(g("max_eventos"))
        self.n_texto = int(g("eventos_com_texto"))
        self.rep_s = float(g("repeticao_min_s"))

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub_trilha = self.create_publisher(Path, "/diag/trilha_real", latched)
        self.pub_anterior = self.create_publisher(Path, "/diag/plano_anterior", latched)
        self.pub_ev = self.create_publisher(MarkerArray, "/diag/eventos", latched)

        self.trilha = Path()
        self.trilha.header.frame_id = self.fmap
        self.pose = None          # (x, y, yaw) mais recente no mapa
        self.plano = None         # pontos [(x, y)] do ultimo /plan
        self.plano_msg = None
        self.eventos = []         # [(x, y, texto)]
        self.ultimo_por_chave = {}
        self.ultima_acao_cm = 0
        self.mudou_trilha = False

        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(Log, "/rosout", self.on_rosout, 200)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                     self.on_cm, 10)
        self.create_timer(0.1, self.amostrar_pose)
        self.create_timer(0.5, self.publicar_trilha)
        self.get_logger().info("trilha/plano anterior/eventos em /diag/* (frame %s)" % self.fmap)

    # ------------------------------------------------------------ trilha
    def amostrar_pose(self):
        try:
            tr = self.tf.lookup_transform(self.fmap, self.frobo, rclpy.time.Time())
        except TransformException:
            return
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.pose and math.hypot(t.x - self.pose[0], t.y - self.pose[1]) < self.passo \
                and abs(math.remainder(yaw - self.pose[2], math.tau)) < 0.1:
            return
        self.pose = (t.x, t.y, yaw)
        ps = PoseStamped()
        ps.header = tr.header
        ps.header.frame_id = self.fmap
        ps.pose.position.x, ps.pose.position.y = t.x, t.y
        ps.pose.orientation = q
        self.trilha.poses.append(ps)
        self.mudou_trilha = True

    def publicar_trilha(self):
        if not self.mudou_trilha:
            return
        self.mudou_trilha = False
        self.trilha.header.stamp = self.get_clock().now().to_msg()
        self.pub_trilha.publish(self.trilha)

    # ------------------------------------------------------------ replan
    def on_plan(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            return
        if self.plano and mesmo_destino(self.plano, pts):
            if desvio_medio(pts, self.plano) > self.desvio_replan:
                self.pub_anterior.publish(self.plano_msg)  # rota mudou: guarda a velha
        elif self.plano:
            vazio = Path()
            vazio.header = msg.header
            self.pub_anterior.publish(vazio)  # goal novo: limpa a comparacao
        self.plano, self.plano_msg = pts, msg

    # ------------------------------------------------------------ eventos
    def on_rosout(self, m: Log):
        if m.level < Log.WARN or not m.name.split(".")[0].startswith(NOS_NAV):
            return
        if "missed its desired rate" in m.msg:  # repete a corrida toda; o relatorio ja' conta
            return
        self.evento(f"{m.name.split('.')[0]}: {m.msg}", chave=(m.name, m.msg[:40]))

    def on_cm(self, m):
        if m.action_type != self.ultima_acao_cm and m.action_type in ACOES_CM:
            self.evento(f"collision_monitor {ACOES_CM[m.action_type]} ({m.polygon_name})")
        self.ultima_acao_cm = m.action_type

    def evento(self, texto, chave=None):
        agora = time.monotonic()
        chave = chave or texto
        if agora - self.ultimo_por_chave.get(chave, -1e9) < self.rep_s:
            return
        self.ultimo_por_chave[chave] = agora
        if self.pose is None:
            return
        hora = time.strftime("%H:%M:%S")
        self.eventos.append((self.pose[0], self.pose[1], f"{hora} {texto[:70]}"))
        self.eventos = self.eventos[-self.max_ev:]
        self.publicar_eventos()

    def publicar_eventos(self):
        arr = MarkerArray()
        limpar = Marker()
        limpar.action = Marker.DELETEALL
        arr.markers.append(limpar)
        stamp = self.get_clock().now().to_msg()
        n = len(self.eventos)
        for i, (x, y, texto) in enumerate(self.eventos):
            esf = Marker()
            esf.header.frame_id, esf.header.stamp = self.fmap, stamp
            esf.ns, esf.id, esf.type = "eventos", i, Marker.SPHERE
            esf.pose.position.x, esf.pose.position.y, esf.pose.position.z = x, y, 0.05
            esf.pose.orientation.w = 1.0
            esf.scale.x = esf.scale.y = esf.scale.z = 0.18  # ~20 px no video
            esf.color.r, esf.color.a = 1.0, 0.85
            arr.markers.append(esf)
            if i >= n - self.n_texto:  # texto so' nos ultimos (o video fica legivel)
                txt = Marker()
                txt.header = esf.header
                txt.ns, txt.id, txt.type = "eventos_texto", i, Marker.TEXT_VIEW_FACING
                txt.pose.position.x = x
                txt.pose.position.y = y + 0.25 + 0.25 * (n - 1 - i)
                txt.pose.position.z = 0.3
                txt.pose.orientation.w = 1.0
                txt.scale.z = 0.20  # altura em m; ~20 px no video
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 0.35, 0.35, 1.0
                txt.text = texto.replace(" ", "_")  # rviz2 espaca demais as palavras no 3D
                arr.markers.append(txt)
        self.pub_ev.publish(arr)


def main():
    rclpy.init()
    node = RunDiagViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
