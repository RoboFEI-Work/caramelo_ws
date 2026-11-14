#!/usr/bin/env python3
"""Limite de velocidade por proximidade (2026-11-11).

Pedido do operador depois dos bags de 11/11: "perto de objeto e em curva
fechada o robo nao precisa andar na maxima". O Nav2 ja' tem o canal para isso:
o controller_server assina nav2_msgs/SpeedLimit em /speed_limit e repassa ao
plugin (DWB: setSpeedLimit escala max_vel_x/y/theta e max_speed_xy; os minimos
nao mudam). Este no' le o /scan (LiDAR frontal, 180 graus) e publica:
  - alcance minimo < perto_m  -> percentual limite_pct (default 70 %)
  - alcance minimo > longe_m  -> 0.0 (= sem limite; o DWB volta aos tetos)
  - entre os dois: mantem o ultimo estado (histerese).
Por que 70 % e nao menos: max_vel_theta 0,50 x 0,70 = 0,35 = min_speed_theta;
abaixo disso o DWB nao teria NENHUMA amostra de giro valida. O RotationShim
gira a 0,35 por conta propria e nao e' afetado. O piso do ESC (0,1215 m/s)
tambem nao muda: 70 % de 0,60 = 0,42 m/s, bem acima.
Roda no launch de navegacao so' com use_speed_limit:=true (default false desde
2026-11-13: na arena ficava ligado 99 % do tempo e travava o giro em 0,35). Desligar =
use_speed_limit:=false; o controller volta a 100 % sozinho (publicamos 0.0 ao
encerrar e ele tambem ignora limites antigos a cada nova mensagem).
"""
import math

import rclpy
from nav2_msgs.msg import SpeedLimit
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SpeedLimitByScan(Node):
    def __init__(self):
        super().__init__("speed_limit_by_scan")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("speed_limit_topic", "/speed_limit")
        self.declare_parameter("perto_m", 0.70)      # abaixo disto: limita
        self.declare_parameter("longe_m", 0.85)      # acima disto: libera
        self.declare_parameter("limite_pct", 70.0)   # percentual aplicado quando perto
        self.declare_parameter("pct_minimo", 70.0)   # nunca abaixo (min_speed_theta do DWB)
        self.declare_parameter("alcance_minimo_m", 0.08)  # ignora leituras coladas no sensor
        self.declare_parameter("min_feixes", 3)      # precisa de N feixes abaixo de perto_m
        self.declare_parameter("taxa_hz", 5.0)       # cadencia maxima de publicacao
        self.declare_parameter("heartbeat_s", 1.0)   # republica o estado atual a cada N s

        self.perto = float(self.get_parameter("perto_m").value)
        self.longe = float(self.get_parameter("longe_m").value)
        if self.longe < self.perto:
            self.longe = self.perto
        pct_min = float(self.get_parameter("pct_minimo").value)
        self.limite = max(float(self.get_parameter("limite_pct").value), pct_min)
        self.limite = min(self.limite, 100.0)
        self.alc_min = float(self.get_parameter("alcance_minimo_m").value)
        self.min_feixes = int(self.get_parameter("min_feixes").value)
        self.periodo = 1.0 / max(float(self.get_parameter("taxa_hz").value), 0.5)
        self.heartbeat = float(self.get_parameter("heartbeat_s").value)

        self.pub = self.create_publisher(
            SpeedLimit, str(self.get_parameter("speed_limit_topic").value), 10)
        self.sub = self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value),
            self.on_scan, qos_profile_sensor_data)
        self.limitado = False
        self.ultimo_pub = self.get_clock().now()
        self.ultimo_min = float("inf")
        self.timer = self.create_timer(self.heartbeat, self.on_heartbeat)
        self.get_logger().info(
            f"limite {self.limite:.0f}% abaixo de {self.perto:.2f} m, libera acima de "
            f"{self.longe:.2f} m (scan {self.sub.topic_name} -> {self.pub.topic_name})")

    def on_scan(self, msg: LaserScan):
        piso = max(self.alc_min, float(msg.range_min))
        perto = 0
        menor = float("inf")
        for r in msg.ranges:
            if not math.isfinite(r) or r <= piso:
                continue
            if r < menor:
                menor = r
            if r < self.perto:
                perto += 1
        self.ultimo_min = menor
        novo = self.limitado
        if perto >= self.min_feixes:
            novo = True
        elif menor > self.longe:
            novo = False
        agora = self.get_clock().now()
        if novo != self.limitado:
            self.limitado = novo
            self.publicar(agora)
            self.get_logger().info(
                f"{'LIMITE ' + format(self.limite, '.0f') + '%' if novo else 'liberado'} "
                f"(menor alcance {menor:.2f} m)")
        elif (agora - self.ultimo_pub).nanoseconds * 1e-9 >= self.periodo and self.limitado:
            self.publicar(agora)

    def publicar(self, agora):
        m = SpeedLimit()
        m.header.stamp = agora.to_msg()
        m.percentage = True
        m.speed_limit = float(self.limite) if self.limitado else 0.0
        self.pub.publish(m)
        self.ultimo_pub = agora

    def on_heartbeat(self):
        # Republica o estado (o controller_server pode ter subido depois de nos).
        self.publicar(self.get_clock().now())


def main():
    rclpy.init()
    node = SpeedLimitByScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.limitado = False
            node.publicar(node.get_clock().now())  # solta o limite ao sair
        except Exception:  # noqa: BLE001 - encerrando
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
