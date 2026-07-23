#!/usr/bin/env python3
"""Servidores de action FALSOS de manipulacao p/ testar a missao sem braco.

Sobe /pick_tag e /place_tag (my_robot_msgs) com a mesma interface dos nos MTC
reais. Par do fake_nav_actions.py — juntos permitem rodar a missao completa
do bt_yaml_executor sem Nav2 e sem MoveIt.

ATENCAO: nao rodar com os mtc_*_action_node reais no ar (mesmos nomes de action).

Parametros:
  latency_sec  (default 1.0): tempo simulado de cada pick/place.
  fail_stage   (default ""): pick|place -> o server correspondente aborta.
  reject_stage (default ""): pick|place -> rejeita o goal.

Uso:
  python3 fake_pick_place.py --ros-args -p latency_sec:=0.5 -p fail_stage:=pick
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from my_robot_msgs.action import PickTag, PlaceTag


class FakePickPlace(Node):
    """No com /pick_tag e /place_tag falsos (latencia, falha e rejeicao simulaveis)."""

    def __init__(self):
        super().__init__("fake_pick_place")
        self.declare_parameter("latency_sec", 1.0)
        self.declare_parameter("fail_stage", "")
        self.declare_parameter("reject_stage", "")

        group = ReentrantCallbackGroup()
        self._servers = [
            ActionServer(
                self, PickTag, "/pick_tag",
                execute_callback=lambda gh: self._execute(gh, "pick", PickTag.Result),
                goal_callback=lambda req: self._goal_response("pick"),
                cancel_callback=self._cancel_response,
                callback_group=group),
            ActionServer(
                self, PlaceTag, "/place_tag",
                execute_callback=lambda gh: self._execute(gh, "place", PlaceTag.Result),
                goal_callback=lambda req: self._goal_response("place"),
                cancel_callback=self._cancel_response,
                callback_group=group),
        ]
        self.get_logger().info("Servers falsos no ar: /pick_tag, /place_tag.")

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

    def _execute(self, goal_handle, stage, result_type):
        latency = max(0.0, float(self._param("latency_sec")))
        deadline = time.monotonic() + latency
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f"cancel recebido em {stage}")
                result = result_type()
                result.success = False
                result.message = "cancelado (fake)"
                return result
            time.sleep(0.1)

        result = result_type()
        if (self._param("fail_stage") or "").strip() == stage:
            goal_handle.abort()
            self.get_logger().error(f"'{stage}' abortado (fail_stage).")
            result.success = False
            result.message = "falha simulada"
            return result

        goal_handle.succeed()
        self.get_logger().info(f"'{stage}' concluido com sucesso (fake).")
        result.success = True
        result.message = "fake"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakePickPlace()
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
