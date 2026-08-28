#!/usr/bin/env python3
"""Servidores de action FALSOS para testar o bt_yaml_executor sem Nav2/docking.

Sobe 5 action servers com as mesmas interfaces usadas pela missao real:
  /dock_robot        (nav2_msgs/DockRobot)
  /undock_robot      (nav2_msgs/UndockRobot)
  /navigate_to_pose  (nav2_msgs/NavigateToPose)
  align_to_dock      (caramelo_msgs/AlignToDock) — nome RELATIVO, como no robo
  nudge_base         (caramelo_msgs/NudgeBase)   — nome RELATIVO (2026-08-28,
                     fila de alcance; so sobe se a mensagem existir no ws)

Parametros:
  latency_sec  (default 1.0): tempo simulado de cada action.
  fail_stage   (default ""): dock|undock|align|nav|nudge -> o server aborta.
  reject_stage (default ""): dock|undock|align|nav|nudge -> rejeita o goal.
  nudge_slip   (default 0.85): travelled = dy * nudge_slip (a base para um
               pouco antes, como o ESC real).
  nudge_min_travel / nudge_max_travel (0.08 / 0.25): |dy| fora da faixa ->
               abort com reason abaixo_do_passo_minimo / acima_do_curso_maximo.

Topicos (como o dock_align_node real):
  /manip/base_shift_total (std_msgs/Float64, transient_local, + = esquerda):
      acumulado dos nudges com sucesso; 0.0 no start e em cada align_to_dock.
  /fake_nav/station (std_msgs/String, transient_local): dock_id de cada
      align_to_dock com sucesso — o fake_pick_place zera o acumulado dele.

Todos os servers aceitam cancel (checado a cada 0.1 s durante a espera).

Uso:
  python3 fake_nav_actions.py --ros-args -p latency_sec:=0.5 -p fail_stage:=align
  python3 fake_nav_actions.py --ros-args -p fail_stage:=nudge -p nudge_slip:=0.85
"""
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String

from caramelo_msgs.action import AlignToDock

try:
    from caramelo_msgs.action import NudgeBase
except ImportError:  # ws sem a action nova: sobe sem o nudge_base
    NudgeBase = None

try:
    from nav2_msgs.action import DockRobot, UndockRobot
except ImportError:  # pragma: no cover
    from opennav_docking_msgs.action import DockRobot, UndockRobot  # type: ignore

from nav2_msgs.action import NavigateToPose


class FakeNavActions(Node):
    """No com os action servers falsos (latencia, falha, rejeicao e nudge simulaveis)."""

    def __init__(self):
        super().__init__("fake_nav_actions")
        self.declare_parameter("latency_sec", 1.0)
        self.declare_parameter("fail_stage", "")
        self.declare_parameter("reject_stage", "")
        self.declare_parameter("nudge_slip", 0.85)
        self.declare_parameter("nudge_min_travel", 0.08)
        self.declare_parameter("nudge_max_travel", 0.25)

        self._lock = threading.Lock()
        self._base_shift_total = 0.0
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._shift_pub = self.create_publisher(Float64, "/manip/base_shift_total", latched)
        self._station_pub = self.create_publisher(String, "/fake_nav/station", latched)
        self._publish_shift_total()

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
                execute_callback=self._execute_align,
                goal_callback=lambda req: self._goal_response("align"),
                cancel_callback=self._cancel_response,
                callback_group=group),
        ]
        names = "/dock_robot, /undock_robot, /navigate_to_pose, align_to_dock (relativo)"
        if NudgeBase is not None:
            self._servers.append(ActionServer(
                self, NudgeBase, "nudge_base",  # nome RELATIVO, como no dock_align_node
                execute_callback=self._execute_nudge,
                goal_callback=lambda req: self._goal_response("nudge"),
                cancel_callback=self._cancel_response,
                callback_group=group))
            names += ", nudge_base (relativo)"
        else:
            self.get_logger().warn(
                "caramelo_msgs/NudgeBase nao encontrada: subindo SEM o nudge_base.")
        self.get_logger().info(f"Servers falsos no ar: {names}.")

    # -- helpers ---------------------------------------------------------------
    def _param(self, name):
        return self.get_parameter(name).value

    def _publish_shift_total(self):
        msg = Float64()
        with self._lock:
            msg.data = float(self._base_shift_total)
        self._shift_pub.publish(msg)
        self.get_logger().info(f"/manip/base_shift_total = {msg.data:+.3f} m")

    def _goal_response(self, stage):
        if (self._param("reject_stage") or "").strip() == stage:
            self.get_logger().warn(f"goal REJEITADO em '{stage}' (reject_stage).")
            return GoalResponse.REJECT
        self.get_logger().info(f"goal aceito em '{stage}'.")
        return GoalResponse.ACCEPT

    def _cancel_response(self, goal_handle):  # noqa: ARG002
        return CancelResponse.ACCEPT

    def _wait_latency(self, goal_handle, stage):
        """Espera latency_sec em passos de 0.1 s; False se o goal foi cancelado."""
        latency = max(0.0, float(self._param("latency_sec")))
        deadline = time.monotonic() + latency
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f"cancel recebido em {stage}")
                return False
            time.sleep(0.1)
        return True

    def _execute(self, goal_handle, stage, make_result):
        """Espera latency_sec honrando cancel e fail_stage."""
        if not self._wait_latency(goal_handle, stage):
            return make_result(success=False)

        if (self._param("fail_stage") or "").strip() == stage:
            goal_handle.abort()
            self.get_logger().error(f"'{stage}' abortado (fail_stage).")
            return make_result(success=False)

        goal_handle.succeed()
        self.get_logger().info(f"'{stage}' concluido com sucesso (fake).")
        return make_result(success=True)

    def _execute_align(self, goal_handle):
        """align_to_dock: como o real, zera o acumulado lateral e anuncia a estacao."""
        result = self._execute(goal_handle, "align", self._align_result)
        if result.success:
            with self._lock:
                self._base_shift_total = 0.0
            self._publish_shift_total()
            station = String()
            station.data = str(getattr(goal_handle.request, "dock_id", ""))
            self._station_pub.publish(station)
            self.get_logger().info(f"/fake_nav/station = '{station.data}'")
        return result

    def _execute_nudge(self, goal_handle):
        """nudge_base: valida a faixa, espera, acumula dy*slip e publica o total."""
        result = NudgeBase.Result()
        dy = float(getattr(goal_handle.request, "dy", 0.0))
        min_travel = float(self._param("nudge_min_travel"))
        max_travel = float(self._param("nudge_max_travel"))
        reason = ""
        if abs(dy) < min_travel:
            reason = "abaixo_do_passo_minimo"
        elif abs(dy) > max_travel + 1e-6:
            reason = "acima_do_curso_maximo"
        if reason:
            goal_handle.abort()
            result.success = False
            result.travelled = 0.0
            result.reason = reason
            self.get_logger().error(f"'nudge' dy={dy:+.3f} m recusado: {reason}.")
            return result

        if not self._wait_latency(goal_handle, "nudge"):
            result.success = False
            result.travelled = 0.0
            result.reason = "cancelado"
            return result

        if (self._param("fail_stage") or "").strip() == "nudge":
            goal_handle.abort()
            result.success = False
            result.travelled = 0.0
            result.reason = "obstaculo_lateral"
            self.get_logger().error("'nudge' abortado (fail_stage).")
            return result

        travelled = dy * float(self._param("nudge_slip"))
        with self._lock:
            self._base_shift_total += travelled
        goal_handle.succeed()
        result.success = True
        result.travelled = float(travelled)
        result.reason = ""
        self.get_logger().info(
            f"'nudge' concluido (fake): pedido {dy:+.3f} m, andou {travelled:+.3f} m.")
        self._publish_shift_total()
        return result

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
