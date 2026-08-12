"""Nó ROS único do painel web.

Um nó só, girando numa thread própria, guardando o último estado de cada coisa
que o painel mostra. O servidor HTTP lê esse estado; ele nunca fala ROS direto.
É a mesma regra da GUI Qt (widget não fala ROS), pelo mesmo motivo: o transporte
muda, o contrato não.

Por que o estado fica guardado em vez de ser repassado a cada mensagem: o painel
é para uso EM REDE, e numa rede de competição a banda que ele gastar sai da
navegação. O servidor amostra o estado na taxa que a tela precisa (10 Hz para a
pose, 5 Hz para o LiDAR), independente da taxa em que o robô publica.
"""
import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

try:
    from caramelo_msgs.msg import MissionStatus
except ImportError:  # pragma: no cover - o painel funciona sem a missão
    MissionStatus = None

ESTADOS_MISSAO = {
    0: "parado", 1: "planejando", 2: "pre-flight",
    3: "executando", 4: "concluida", 5: "falhou", 6: "abortada",
}


class NoDoPainel(Node):
    def __init__(self):
        super().__init__("caramelo_web")

        self._lock = threading.Lock()
        self.pose = None            # {x, y, yaw, t}
        self.scan = None            # {angle_min, angle_increment, ranges, range_max, t}
        self.diagnostico = {}       # nome -> {level, message}
        self.missao = None          # último MissionStatus, já traduzido

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            LaserScan, "/scan", self._ao_scan, qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._ao_diagnostico, 10)

        if MissionStatus is not None:
            # MESMA QoS do bt_yaml_executor: transient_local com histórico fundo.
            # O publicador nasce junto com a missão, então o painel é sempre um
            # late joiner -- com volatile ou depth 1 ele perderia o começo de
            # toda missão e só veria o último passo.
            qos = QoSProfile(
                depth=64,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                MissionStatus, "/caramelo/mission/status", self._ao_status_missao, qos)

        # Só a action, nunca /goal_pose: o bt_navigator também assina esse
        # tópico, e publicar nele dispararia DOIS goals para um clique.
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(0.1, self._amostrar_pose)

    # ------------------------------------------------------------- callbacks
    def _ao_scan(self, msg: LaserScan):
        with self._lock:
            self.scan = {
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "range_max": float(msg.range_max),
                # Decimado por 2 já aqui: o navegador não distingue 800 de 400
                # pontos na tela, e são bytes que não sobem na rede.
                "ranges": [
                    (None if not math.isfinite(r) else round(float(r), 3))
                    for r in msg.ranges[::2]
                ],
                "t": time.time(),
            }

    def _ao_diagnostico(self, msg: DiagnosticArray):
        with self._lock:
            for st in msg.status:
                self.diagnostico[st.name] = {
                    "level": int.from_bytes(st.level, "big")
                    if isinstance(st.level, bytes) else int(st.level),
                    "message": st.message,
                }

    def _ao_status_missao(self, msg):
        with self._lock:
            self.missao = {
                "estado": ESTADOS_MISSAO.get(int(msg.state), "?"),
                "mission_id": msg.mission_id,
                "task_id": msg.task_id,
                "passo": int(msg.action_index),
                "total": int(msg.action_total),
                "tipo": msg.action_kind,
                "alvo": msg.action_target,
                "subestagio": msg.stage,
                "mensagem": msg.message,
                "decorrido": float(msg.elapsed),
            }

    def _amostrar_pose(self):
        """Lê a TF map->base_footprint. Sem localização, a pose fica None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            return
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.pose = {
                "x": tf.transform.translation.x,
                "y": tf.transform.translation.y,
                "yaw": yaw,
                "t": time.time(),
            }

    # ---------------------------------------------------------------- leitura
    def instantaneo(self, com_scan: bool = True) -> dict:
        agora = time.time()
        with self._lock:
            pose = dict(self.pose) if self.pose else None
            scan = dict(self.scan) if (com_scan and self.scan) else None
            diag = dict(self.diagnostico)
            missao = dict(self.missao) if self.missao else None
        # "Dado velho" é diferente de "sem dado": sem esta marca, uma tela
        # congelada pareceria um robô parado.
        if pose and agora - pose["t"] > 2.0:
            pose["velho"] = True
        if scan and agora - scan["t"] > 2.0:
            scan["velho"] = True
        return {"pose": pose, "scan": scan, "diagnostico": diag, "missao": missao}

    def nos_no_grafo(self) -> list:
        return sorted(n for n, _ns in self.get_node_names_and_namespaces())

    def navegacao_pronta(self) -> bool:
        return self._nav.server_is_ready()

    def enviar_meta(self, x: float, y: float, yaw: float) -> tuple:
        if not self._nav.server_is_ready():
            return False, "Navegacao indisponivel (o Nav2 esta no ar?)."
        meta = NavigateToPose.Goal()
        meta.pose.header.frame_id = "map"
        meta.pose.header.stamp = self.get_clock().now().to_msg()
        meta.pose.pose.position.x = float(x)
        meta.pose.pose.position.y = float(y)
        meta.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        meta.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self._nav.send_goal_async(meta)
        return True, "Meta enviada."


def subir_no_em_thread() -> tuple:
    """Inicia o rclpy e gira o nó numa thread. Devolve (no, parar)."""
    if not rclpy.ok():
        rclpy.init()
    no = NoDoPainel()
    executor = SingleThreadedExecutor()
    executor.add_node(no)

    parar_evento = threading.Event()

    def girar():
        while rclpy.ok() and not parar_evento.is_set():
            executor.spin_once(timeout_sec=0.1)

    thread = threading.Thread(target=girar, daemon=True)
    thread.start()

    def parar():
        parar_evento.set()
        thread.join(timeout=2.0)
        executor.remove_node(no)
        no.destroy_node()

    return no, parar
