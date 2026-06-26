#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from caramelo_docking_common import docking_file, load_yaml, normalize_dock_id, resolve_map_folder

try:
    from nav2_msgs.action import DockRobot
except ImportError:
    from opennav_docking_msgs.action import DockRobot  # type: ignore

FEEDBACK_STATES = {
    0: "NONE",
    1: "NAV_TO_STAGING_POSE",
    2: "INITIAL_PERCEPTION",
    3: "CONTROLLING",
    4: "WAIT_FOR_CHARGE",
    5: "RETRY",
}


class DockToClient(Node):
    def __init__(self):
        super().__init__("dock_to")
        self.client = ActionClient(self, DockRobot, "/dock_robot")

    def send(self, dock_id: str, navigate_to_staging: bool, max_staging_time: float) -> bool:
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        goal.navigate_to_staging_pose = navigate_to_staging
        goal.max_staging_time = float(max_staging_time)
        self.get_logger().info("Esperando action server /dock_robot...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server /dock_robot nao respondeu.")
            return False
        self.get_logger().info(
            f"Enviando dock_id={dock_id}, navigate_to_staging_pose={navigate_to_staging}, max_staging_time={max_staging_time:.1f}s"
        )
        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal de docking rejeitado.")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f"Dock concluido com sucesso. retries={result.num_retries}")
            return True
        self.get_logger().error(f"Dock falhou: error_code={result.error_code} retries={result.num_retries} msg='{result.error_msg}'")
        return False

    def _feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        state = FEEDBACK_STATES.get(feedback.state, str(feedback.state))
        self.get_logger().info(f"Feedback docking: {state}, retries={feedback.num_retries}")


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="Manda o robo dockar usando um dock_id.")
    parser.add_argument("--dock-id", required=True, help="START, FINISH, WS1, SH1, PP1, RT1...")
    parser.add_argument("--map-name", default=None, help="Opcional: valida se o dock existe no mapa.")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    parser.add_argument("--no-nav-to-staging", action="store_true", help="Nao navegar ate staging pose.")
    parser.add_argument("--max-staging-time", type=float, default=60.0, help="Tempo maximo para chegar na staging pose.")
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def _validate_dock_exists(map_name: str, map_dir: str, dock_id: str):
    map_folder = resolve_map_folder(map_name, map_dir)
    data = load_yaml(docking_file(map_folder))
    docks = data.get("docks", {})
    if dock_id not in docks:
        raise RuntimeError(f"Dock '{dock_id}' nao existe em {docking_file(map_folder)}.")


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = _parse_args(argv)
    try:
        dock_id = normalize_dock_id(args.dock_id)
        if args.map_name:
            _validate_dock_exists(args.map_name, args.map_dir, dock_id)
    except Exception as exc:
        print(f"Erro nos argumentos: {exc}", file=sys.stderr)
        return 1
    rclpy.init(args=argv)
    node = DockToClient()
    try:
        ok = node.send(dock_id, not args.no_nav_to_staging, args.max_staging_time)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
