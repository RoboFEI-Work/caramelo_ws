#!/usr/bin/env python3
"""Missao SO-NAVEGACAO: visita a final_robot_pose de cada service area em
sequencia, parando em cada uma (NavigateToPose), SEM docking. Para a ronda
WS1->WS2->FINISH sem depender do docking_server nem do piso do ESC.

Uso:  python3 mission_navigate.py <map_name> <AREA1> <AREA2> ...
Ex.:  python3 mission_navigate.py arena2_520 WS1 WS2 FINISH
"""
import math
import os
import sys
import time
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
import tf2_ros


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def resolve_areas_file(map_name):
    # tenta o src do workspace (fonte da verdade das poses)
    home = os.path.expanduser("~")
    candidates = [
        f"{home}/caramelo_ws/src/caramelo_mapping/maps/{map_name}/service_areas.yaml",
        f"{home}/caramelo_ws/install/caramelo_mapping/share/caramelo_mapping/maps/{map_name}/service_areas.yaml",
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    raise FileNotFoundError(f"service_areas.yaml nao encontrado para o mapa '{map_name}'. Tentei:\n  " + "\n  ".join(candidates))


class MissionNav(Node):
    def __init__(self, areas):
        super().__init__("mission_navigate")
        self.areas = areas
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def go(self, station):
        area = self.areas.get(station)
        if not area:
            self.get_logger().error(f"[{station}] nao existe em service_areas.yaml")
            return False
        p = area["final_robot_pose"]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(p["x"])
        goal.pose.pose.position.y = float(p["y"])
        qx, qy, qz, qw = yaw_to_quat(float(p["yaw"]))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/navigate_to_pose indisponivel (a navegacao esta rodando?)")
            return False
        self.get_logger().info(f"==> indo para {station} ({p['x']:.2f}, {p['y']:.2f}, {math.degrees(p['yaw']):.0f} deg)")
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        gh = send.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f"[{station}] goal rejeitado")
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        ok = res.result().status == 4  # SUCCEEDED
        xy = self.robot_xy()
        err = f"{math.hypot(xy[0]-p['x'], xy[1]-p['y'])*100:.1f} cm" if xy else "?"
        self.get_logger().info(f"[{station}] {'OK' if ok else 'FALHOU'} (erro de parada: {err})")
        return ok


def main():
    if len(sys.argv) < 3:
        print("Uso: python3 mission_navigate.py <map_name> <AREA1> <AREA2> ...")
        print("Ex.: python3 mission_navigate.py arena2_520 WS1 WS2 FINISH")
        return
    map_name = sys.argv[1]
    stations = [s.upper() for s in sys.argv[2:]]
    with open(resolve_areas_file(map_name)) as fh:
        areas = (yaml.safe_load(fh) or {}).get("service_areas", {})

    rclpy.init()
    node = MissionNav(areas)
    time.sleep(1.0)
    resultados = []
    for st in stations:
        ok = node.go(st)
        resultados.append((st, ok))
        if not ok:
            node.get_logger().warn(f"[{st}] falhou -> parando a missao.")
            break
    node.get_logger().info("===== RESUMO DA MISSAO =====")
    for st, ok in resultados:
        node.get_logger().info(f"  {st}: {'OK' if ok else 'FALHOU'}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
