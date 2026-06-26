#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from nav2_msgs.action import UndockRobot


class UndockClient(Node):
    def __init__(self):
        super().__init__("undock")
        self.client = ActionClient(self, UndockRobot, "/undock_robot")

    def send(self, dock_type: str, max_undocking_time: float) -> bool:
        goal = UndockRobot.Goal()
        goal.dock_type = dock_type or ""
        goal.max_undocking_time = float(max_undocking_time)
        self.get_logger().info("Esperando action server /undock_robot...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server /undock_robot nao respondeu.")
            return False
        self.get_logger().info(f"Enviando undock dock_type='{goal.dock_type}' max_undocking_time={max_undocking_time:.1f}s")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal de undocking rejeitado.")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.success:
            self.get_logger().info("Undock concluido com sucesso.")
            return True
        self.get_logger().error(f"Undock falhou: error_code={result.error_code} msg='{result.error_msg}'")
        return False


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="Afasta o robo da dock atual usando /undock_robot.")
    parser.add_argument("--dock-type", default="", help="Opcional. Ex: caramelo_front_dock.")
    parser.add_argument("--max-undocking-time", type=float, default=30.0, help="Tempo maximo para sair da dock.")
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = _parse_args(argv)
    rclpy.init(args=argv)
    node = UndockClient()
    try:
        ok = node.send(args.dock_type, args.max_undocking_time)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
