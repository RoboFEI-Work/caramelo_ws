#!/usr/bin/env python3
"""Mede o deslocamento real de um pulso no piso do ESC, do jeito que o
dock_align_node comanda: N ciclos a 20 Hz no piso, seguidos de ZEROS
publicados continuamente (o driver da Pi NAO entra no watchdog).

Uso: python3 scripts/pulse_test.py x 4      # eixo x|y|z, ciclos
Mede com fita/regua o quanto o robo andou (ou girou) e anote. Rode 4, 5, 6 e
tambem 'cont' (contínuo de 1 s) para medir o coast do corte:
      python3 scripts/pulse_test.py x cont
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

FLOOR_V = 0.1215
FLOOR_W = 0.315


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    axis, cycles = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = Node("pulse_test")
    pub = node.create_publisher(TwistStamped, "/mecanum_controller/reference", 10)
    time.sleep(1.0)  # descoberta

    def send(v):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        if axis == "x":
            msg.twist.linear.x = v
        elif axis == "y":
            msg.twist.linear.y = v
        else:
            msg.twist.angular.z = v
        pub.publish(msg)

    floor = FLOOR_W if axis == "z" else FLOOR_V
    n = 20 if cycles == "cont" else int(cycles)
    print(f"pulso: {n} ciclos ({n * 50} ms) no piso do eixo {axis}, depois 2 s de zeros")
    for _ in range(n):
        send(floor)
        time.sleep(0.05)
    for _ in range(40):
        send(0.0)
        time.sleep(0.05)
    print("fim — meca o deslocamento")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
