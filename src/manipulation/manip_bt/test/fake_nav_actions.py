#!/usr/bin/env python3
"""Servidores de action FALSOS para testar o bt_yaml_executor sem Nav2/docking.

Sobe 4 action servers com as mesmas interfaces usadas pela missao real:
  /dock_robot        (nav2_msgs/DockRobot)
  /undock_robot      (nav2_msgs/UndockRobot)
  /navigate_to_pose  (nav2_msgs/NavigateToPose)
  align_to_dock      (caramelo_msgs/AlignToDock) — nome RELATIVO, como no robo

Parametros:
  latency_sec  (default 1.0): tempo simulado de cada action.
  fail_stage   (default ""): dock|undock|align|nav -> o server correspondente aborta.
  reject_stage (default ""): dock|undock|align|nav -> o server correspondente
               rejeita o goal.

Todos os servers aceitam cancel (checado a cada 0.1 s durante a espera).

Uso:
  python3 fake_nav_actions.py --ros-args -p latency_sec:=0.5 -p fail_stage:=align
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from caramelo_msgs.action import AlignToDock

try:
    from nav2_msgs.action import DockRobot, UndockRobot
except ImportError:  # pragma: no cover
    from opennav_docking_msgs.action import DockRobot, UndockRobot  # type: ignore

from nav2_msgs.action import NavigateToPose


class FakeNavActions(Node):
    """No com os 4 action servers falsos (latencia, falha e rejeicao simulaveis)."""

    def __init__(self):
        super().__init__("fake_nav_actions")
        self.declare_parameter("latency_sec", 1.0)
        self.declare_parameter("fail_stage", "")
        self.declare_parameter("reject_stage", "")

        group = ReentrantCallbackGroup()
        self._servers = [
            ActionServer(
                self, DockRobot, "/dock_robot",
                execute_callback=lambda gh: self._execute(gh, "dock", self._dock_result),
                goal_callback=lambda req: self._goal_response("dock"),
                cancel_callback=self._cancel_response,
                callback_group=group),
            ActionServer(
                self, UndockRobot, "/undock_robot",
                execute_callback=lambda gh: self._execute(gh, "undock", self._undock_result),
                goal_callback=lambda req: self._goal_response("undock"),
                cancel_callback=self._cancel_response,
                callback_group=group),
            ActionServer(
                self, NavigateToPose, "/navigate_to_pose",
                execute_callback=lambda gh: self._execute(gh, "nav", self._nav_result),
                goal_callback=lambda req: self._goal_response("nav"),
                cancel_callback=self._cancel_response,
                callback_group=group),
            ActionServer(
                self, AlignToDock, "align_to_dock",  # nome RELATIVO de proposito
                execute_callback=lambda gh: self._execute(gh, "align", self._align_result),
                goal_callback=lambda req: self._goal_response("align"),
                cancel_callback=self._cancel_response,
                callback_group=group),
        ]
        self.get_logger().info(
            "Servers falsos no ar: /dock_robot, /undock_robot, /navigate_to_pose, "
            "align_to_dock (relativo).")

    # -- helpers ---------------------------------------------------------------
    def _param(self, name):
        return self.get_parameter(name).value

    def _goal_response(self, stage):
        if (self._param("reject_stage") or "").strip() == stage:
            self.get_logger().warn(f"goal REJEITADO em '{stage}' (reject_stage).")
            return GoalResponse.REJECT
        self.get_logger().info(f"goal aceito em '{stage}'.")
        return GoalResponse.ACCEPT

    def _cancel_response(self, goal_handle):  # noqa: ARG002
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle, stage, make_result):
        """Espera latency_sec em passos de 0.1 s, honrando cancel e fail_stage."""
        latency = max(0.0, float(self._param("latency_sec")))
        deadline = time.monotonic() + latency
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f"cancel recebido em {stage}")
                return make_result(success=False)
            time.sleep(0.1)

        if (self._param("fail_stage") or "").strip() == stage:
            goal_handle.abort()
            self.get_logger().error(f"'{stage}' abortado (fail_stage).")
            return make_result(success=False)

        goal_handle.succeed()
        self.get_logger().info(f"'{stage}' concluido com sucesso (fake).")
        return make_result(success=True)

    # -- montagem dos results por tipo -----------------------------------------
    @staticmethod
    def _dock_result(success):
        result = DockRobot.Result()
        result.success = success
        return result

    @staticmethod
    def _undock_result(success):
        result = UndockRobot.Result()
        result.success = success
        return result

    @staticmethod
    def _nav_result(success):  # noqa: ARG004
        return NavigateToPose.Result()  # result vazio: sucesso vem do status da action

    @staticmethod
    def _align_result(success):
        result = AlignToDock.Result()
        result.success = success
        result.message = "fake"
        result.final_x_error = 0.01
        result.final_y_error = 0.01
        result.final_yaw_error = 0.01
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeNavActions()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando servers falsos (Ctrl+C).")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
