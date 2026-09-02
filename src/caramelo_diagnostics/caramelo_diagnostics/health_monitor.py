#!/usr/bin/env python3
"""Agregador de saude do Caramelo.

Observa o grafo ROS 2 (do PC) e publica um DiagnosticArray em /diagnostics, para
que tanto a GUI quanto rqt_robot_monitor / CLI vejam o mesmo estado (R4 — o
diagnostico nao vive so' na tela). Mede taxa de topicos-chave vindos da
Raspberry, checa TF, controllers (na Pi via rede) e o lifecycle do Nav2.

Cada componente vira um DiagnosticStatus com nivel (OK/WARN/ERROR/STALE) e
valores uteis para o modo Engenharia: topico esperado, ultima mensagem,
frequencia, causa provavel e acao recomendada.
"""
import time

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import BatteryState, LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
import tf2_ros

from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Trigger

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE


class _RateMonitor:
    """Conta mensagens de um topico para estimar a frequencia."""

    def __init__(self, node, topic, msg_type):
        self.topic = topic
        self.count = 0
        self.last_stamp = None
        node.create_subscription(msg_type, topic, self._cb, 10)

    def _cb(self, _msg):
        self.count += 1
        self.last_stamp = time.monotonic()

    def take_hz(self, period):
        hz = self.count / period if period > 0 else 0.0
        self.count = 0
        return hz

    def age(self):
        if self.last_stamp is None:
            return None
        return time.monotonic() - self.last_stamp


class HealthMonitor(Node):
    def __init__(self):
        super().__init__("caramelo_health_monitor")

        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("battery_warn_percent", 25.0)
        self.declare_parameter("min_scan_hz", 5.0)
        self.declare_parameter("min_imu_hz", 10.0)
        self.declare_parameter("min_odom_hz", 20.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("nav_lifecycle_manager", "lifecycle_manager_navigation")

        self._period = float(self.get_parameter("publish_period").value)
        self._min_scan = float(self.get_parameter("min_scan_hz").value)
        self._min_imu = float(self.get_parameter("min_imu_hz").value)
        self._min_odom = float(self.get_parameter("min_odom_hz").value)
        self._global_frame = self.get_parameter("global_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self._scan = _RateMonitor(self, self.get_parameter("scan_topic").value, LaserScan)
        self._imu = _RateMonitor(self, self.get_parameter("imu_topic").value, Imu)
        self._odom = _RateMonitor(self, self.get_parameter("odom_topic").value, Odometry)
        self._joints = _RateMonitor(self, self.get_parameter("joint_states_topic").value, JointState)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Bateria (sensor_msgs/BatteryState) — guarda a ultima leitura.
        self._battery = None
        self._battery_stamp = None
        self.create_subscription(
            BatteryState, self.get_parameter("battery_topic").value,
            self._battery_cb, 10)

        # Clientes de servico (nao-bloqueantes; cache atualizado por callback).
        self._ctrl_cli = self.create_client(
            ListControllers, "/controller_manager/list_controllers")
        mgr = self.get_parameter("nav_lifecycle_manager").value
        self._nav_cli = self.create_client(Trigger, f"/{mgr}/is_active")
        self._ctrl_cache = None   # lista de (nome, estado)
        self._nav_cache = None    # bool ou None
        # AUDITORIA 31/08 (vazamento): servico "pronto" no grafo mas mudo
        # empilhava um call_async por tick para sempre. 1 pendente por vez.
        self._ctrl_fut = None
        self._nav_fut = None

        self.create_timer(self._period, self._tick)
        self.get_logger().info("caramelo_health_monitor publicando em /diagnostics")

    # ------------------------------------------------------------- servicos
    def _poll_services(self):
        if self._ctrl_fut is not None and not self._ctrl_fut.done():
            self._ctrl_cache = None  # resposta pendente: nao empilhar outra
        elif self._ctrl_cli.service_is_ready():
            self._ctrl_fut = self._ctrl_cli.call_async(ListControllers.Request())
            self._ctrl_fut.add_done_callback(self._on_controllers)
        else:
            self._ctrl_cache = None
        if self._nav_fut is not None and not self._nav_fut.done():
            self._nav_cache = None
        elif self._nav_cli.service_is_ready():
            self._nav_fut = self._nav_cli.call_async(Trigger.Request())
            self._nav_fut.add_done_callback(self._on_nav)
        else:
            self._nav_cache = None

    def _on_controllers(self, fut):
        try:
            resp = fut.result()
            self._ctrl_cache = [(c.name, c.state) for c in resp.controller]
        except Exception:
            self._ctrl_cache = None

    def _on_nav(self, fut):
        try:
            self._nav_cache = bool(fut.result().success)
        except Exception:
            self._nav_cache = None

    def _battery_cb(self, msg):
        self._battery = msg
        self._battery_stamp = time.monotonic()

    # ------------------------------------------------------------- helpers
    @staticmethod
    def _status(name, level, message, pairs):
        st = DiagnosticStatus()
        st.name = name
        st.level = level
        st.message = message
        st.hardware_id = "caramelo"
        st.values = [KeyValue(key=k, value=str(v)) for k, v in pairs]
        return st

    def _rate_status(self, name, mon, min_hz, hz, topico, causa, acao):
        if mon.age() is None:
            level, msg = STALE, "sem dados"
        elif hz >= min_hz:
            level, msg = OK, "ok"
        elif hz > 0.0:
            level, msg = WARN, f"frequencia baixa ({hz:.1f} < {min_hz:.0f} Hz)"
        else:
            level, msg = ERROR, "sem publicacao"
        return self._status(name, level, msg, [
            ("topico_esperado", topico),
            ("frequencia_hz", f"{hz:.1f}"),
            ("minimo_hz", f"{min_hz:.0f}"),
            ("ultima_msg_s", "n/a" if mon.age() is None else f"{mon.age():.1f}"),
            ("causa_provavel", causa),
            ("acao_recomendada", acao),
        ])

    # ------------------------------------------------------------- tick
    def _tick(self):
        self._poll_services()

        scan_hz = self._scan.take_hz(self._period)
        imu_hz = self._imu.take_hz(self._period)
        odom_hz = self._odom.take_hz(self._period)
        joints_hz = self._joints.take_hz(self._period)

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        # 1. Rede / Raspberry: inferida pela presenca de topicos vindos da Pi.
        pi_alive = (self._scan.age() is not None and self._scan.age() < 2.0) or \
                   (self._joints.age() is not None and self._joints.age() < 2.0)
        arr.status.append(self._status(
            "caramelo/rede_raspberry",
            OK if pi_alive else ERROR,
            "Raspberry publicando" if pi_alive else "Sem dados da Raspberry",
            [("topicos_pi", "/scan, /joint_states"),
             ("joint_states_hz", f"{joints_hz:.1f}"),
             ("causa_provavel", "" if pi_alive else "Ethernet/ROS_DOMAIN_ID/bringup da Pi"),
             ("acao_recomendada", "" if pi_alive else "Verificar rede e hardware_bringup na Pi")]))

        # 2. Controllers (ros2_control na Pi, via rede).
        if self._ctrl_cache is None:
            arr.status.append(self._status(
                "caramelo/controllers", STALE, "controller_manager inacessivel",
                [("servico", "/controller_manager/list_controllers"),
                 ("acao_recomendada", "Subir o hardware_bringup na Raspberry")]))
        else:
            ativos = [n for n, s in self._ctrl_cache if s == "active"]
            need = {"mecanum_controller", "joint_state_broadcaster"}
            faltando = sorted(need - set(ativos))
            arr.status.append(self._status(
                "caramelo/controllers",
                OK if not faltando else ERROR,
                "todos ativos" if not faltando else f"inativos: {', '.join(faltando)}",
                [("ativos", ", ".join(sorted(ativos)) or "nenhum"),
                 ("causa_provavel", "" if not faltando else "controller nao carregou/ativou"),
                 ("acao_recomendada", "" if not faltando else "ros2 control list_controllers na Pi")]))

        # 3-5. LiDAR / IMU / Odometria (taxa).
        arr.status.append(self._rate_status(
            "caramelo/lidar", self._scan, self._min_scan, scan_hz,
            self.get_parameter("scan_topic").value,
            "LiDAR desconectado ou driver parado", "Verificar RPLIDAR e /dev/lidar_usb na Pi"))
        arr.status.append(self._rate_status(
            "caramelo/imu", self._imu, self._min_imu, imu_hz,
            self.get_parameter("imu_topic").value,
            "IMU desconectada ou baud errado", "Verificar WIT IMU e /dev/imu_usb na Pi"))
        arr.status.append(self._rate_status(
            "caramelo/odometria", self._odom, self._min_odom, odom_hz,
            self.get_parameter("odom_topic").value,
            "EKF ou controller parado", "Verificar EKF e mecanum_controller na Pi"))

        # 6. TF map->base_footprint.
        try:
            self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, rclpy.time.Time())
            tf_level, tf_msg = OK, f"{self._global_frame}->{self._base_frame} ok"
        except Exception:
            tf_level, tf_msg = ERROR, f"sem TF {self._global_frame}->{self._base_frame}"
        arr.status.append(self._status("caramelo/tf", tf_level, tf_msg, [
            ("cadeia", f"{self._global_frame}->odom->{self._base_frame}"),
            ("causa_provavel", "" if tf_level == OK else "AMCL/SLAM ou EKF nao publicando"),
            ("acao_recomendada", "" if tf_level == OK else "Subir localizacao ou mapeamento")]))

        # 7. Bateria (percentual + tensao; STALE se ninguem publica).
        warn_pct = float(self.get_parameter("battery_warn_percent").value)
        if self._battery is None or (time.monotonic() - self._battery_stamp) > 10.0:
            arr.status.append(self._status(
                "caramelo/bateria", STALE, "sem dados de bateria",
                [("topico_esperado", self.get_parameter("battery_topic").value),
                 ("acao_recomendada", "publicar sensor_msgs/BatteryState no robo")]))
        else:
            pct = self._battery.percentage
            # BatteryState usa 0..1 ou 0..100 dependendo do driver; normaliza.
            pct = pct * 100.0 if pct <= 1.0 else pct
            nivel = OK if pct >= warn_pct else WARN
            arr.status.append(self._status(
                "caramelo/bateria", nivel,
                f"{pct:.0f}%" + ("" if nivel == OK else " — bateria baixa"),
                [("percentual", f"{pct:.0f}"),
                 ("tensao_v", f"{self._battery.voltage:.1f}"),
                 ("minimo_warn", f"{warn_pct:.0f}")]))

        # 8. Nav2 (lifecycle da navegacao).
        if self._nav_cache is None:
            arr.status.append(self._status(
                "caramelo/nav2", STALE, "Nav2 nao iniciado",
                [("gerenciador", self.get_parameter("nav_lifecycle_manager").value),
                 ("acao_recomendada", "Subir a navegacao (bringup.launch.py)")]))
        else:
            arr.status.append(self._status(
                "caramelo/nav2",
                OK if self._nav_cache else WARN,
                "ativo" if self._nav_cache else "inativo",
                [("gerenciador", self.get_parameter("nav_lifecycle_manager").value)]))

        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
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
