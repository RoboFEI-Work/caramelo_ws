#!/usr/bin/env python3
"""Vigia de localizacao (2026-11-11): a odometria manda em janelas curtas.

Por que existe: nos testes com a caixa (obstaculo nao mapeado) junto da WS5
o AMCL "derrapou" — bag das 22h03: o giroscopio e o EKF giraram -467/-465
graus em 150 s e o AMCL so' -378 (perdeu 89 graus), atravessou a parede
central do mapa e o robo passou a planejar do lado errado. A odometria
(rodas + giroscopio fundidos no EKF da Pi) estava certa o tempo todo.

O que faz: a cada /amcl_pose compara a pose do AMCL com (ultima pose aceita
⊕ deslocamento da odometria desde entao). Dentro da folga -> aceita e segue
a correcao lenta do AMCL. Fora da folga por `strikes` atualizacoes seguidas
-> publica /initialpose com a pose propagada pela odometria (covariancia
pequena): o AMCL reinicia ali e nao consegue mais "andar" pelo mapa. Logica
pura em loc_watchdog_core.py (testada offline nos bags: 0 disparos nas
corridas boas, disparo aos 91 s na corrida ruim).

Quando NAO ajuda: sequestro real (robo carregado no colo) ou erro grosseiro
de odometria (rodas patinando contra um obstaculo por muito tempo) — nesses
casos o operador manda a pose pelo RViz (/initialpose externo e' aceito como
nova ancora). Desligar: use_loc_watchdog:=false no launch de localizacao.
"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from loc_watchdog_core import Params, Vigia  # noqa: E402

TAG_COV = 0.00123  # valor "assinatura" em cov[1] para reconhecer os nossos /initialpose


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LocalizationWatchdog(Node):
    def __init__(self):
        super().__init__("localization_watchdog")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("folga_xy_m", 0.10)
        self.declare_parameter("folga_xy_por_metro", 0.08)
        self.declare_parameter("folga_yaw_graus", 8.0)
        self.declare_parameter("folga_yaw_por_rad", 0.08)
        self.declare_parameter("strikes_para_reset", 2)
        self.declare_parameter("min_intervalo_reset_s", 2.0)
        self.declare_parameter("max_intervalo_s", 30.0)
        self.declare_parameter("reset_sigma_xy_m", 0.05)
        self.declare_parameter("reset_sigma_yaw_graus", 3.0)
        self.declare_parameter("enabled", True)
        g = self.get_parameter
        p = Params(
            folga_xy=float(g("folga_xy_m").value),
            folga_xy_por_metro=float(g("folga_xy_por_metro").value),
            folga_yaw=math.radians(float(g("folga_yaw_graus").value)),
            folga_yaw_por_rad=float(g("folga_yaw_por_rad").value),
            strikes_para_reset=int(g("strikes_para_reset").value),
            min_intervalo_reset_s=float(g("min_intervalo_reset_s").value),
            max_intervalo_s=float(g("max_intervalo_s").value),
        )
        self.vigia = Vigia(p)
        self.enabled = bool(g("enabled").value)
        self.sxy = float(g("reset_sigma_xy_m").value)
        self.syaw = math.radians(float(g("reset_sigma_yaw_graus").value))
        self.odom_pose = None
        self.odom_stamp = None
        self.n_resets = 0
        self.pub_init = self.create_publisher(
            PoseWithCovarianceStamped, str(g("initialpose_topic").value), 10)
        self.pub_status = self.create_publisher(String, "~/status", 10)
        self.create_subscription(Odometry, str(g("odom_topic").value), self.on_odom, 50)
        self.create_subscription(
            PoseWithCovarianceStamped, str(g("amcl_pose_topic").value), self.on_amcl, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, str(g("initialpose_topic").value), self.on_initialpose, 10)
        self.get_logger().info(
            f"folga {p.folga_xy:.2f} m + {p.folga_xy_por_metro:.2f}/m, "
            f"{math.degrees(p.folga_yaw):.0f} graus + {p.folga_yaw_por_rad:.2f}/rad, "
            f"{p.strikes_para_reset} strikes; {'ativo' if self.enabled else 'SO OBSERVA'}")

    @staticmethod
    def tsec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_odom(self, m: Odometry):
        self.odom_pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        self.odom_stamp = self.tsec(m.header.stamp)
        self.vigia.odom(self.odom_stamp, self.odom_pose)

    def on_initialpose(self, m: PoseWithCovarianceStamped):
        if abs(m.pose.covariance[1] - TAG_COV) < 1e-9:
            return  # foi nosso
        if self.odom_pose is None:
            return
        pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        self.vigia.ancorar(self.tsec(m.header.stamp), pose, self.odom_pose)
        self.get_logger().warn(
            f"/initialpose externo ({pose[0]:.2f},{pose[1]:.2f},{math.degrees(pose[2]):.0f} graus): "
            "nova ancora.")

    def on_amcl(self, m: PoseWithCovarianceStamped):
        if self.odom_pose is None:
            return
        t = self.tsec(m.header.stamp)
        amcl = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        reset = self.vigia.amcl(t, amcl, self.odom_pose)
        dxy, dyaw, fxy, fyaw, _ = self.vigia.ultimo if hasattr(self.vigia, "ultimo") else (0, 0, 0, 0, None)
        st = String()
        st.data = (f"desvio {dxy:.2f} m / {math.degrees(dyaw):.1f} graus (folga {fxy:.2f} / "
                   f"{math.degrees(fyaw):.1f}); strikes {self.vigia.e.strikes}; resets {self.n_resets}")
        self.pub_status.publish(st)
        if reset is None:
            return
        self.n_resets += 1
        self.get_logger().warn(
            f"AMCL fugiu da odometria: desvio {dxy:.2f} m / {math.degrees(dyaw):.0f} graus "
            f"(folga {fxy:.2f} m / {math.degrees(fyaw):.0f}). AMCL dizia "
            f"({amcl[0]:.2f},{amcl[1]:.2f},{math.degrees(amcl[2]):.0f}); reinicio em "
            f"({reset[0]:.2f},{reset[1]:.2f},{math.degrees(reset[2]):.0f}) [reset #{self.n_resets}]")
        if not self.enabled:
            return
        out = PoseWithCovarianceStamped()
        out.header.stamp = m.header.stamp
        out.header.frame_id = m.header.frame_id or "map"
        out.pose.pose.position.x = reset[0]
        out.pose.pose.position.y = reset[1]
        out.pose.pose.orientation.z = math.sin(reset[2] / 2.0)
        out.pose.pose.orientation.w = math.cos(reset[2] / 2.0)
        cov = [0.0] * 36
        cov[0] = self.sxy ** 2
        cov[7] = self.sxy ** 2
        cov[35] = self.syaw ** 2
        cov[1] = TAG_COV
        out.pose.covariance = cov
        self.pub_init.publish(out)


def main():
    rclpy.init()
    node = LocalizationWatchdog()
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
