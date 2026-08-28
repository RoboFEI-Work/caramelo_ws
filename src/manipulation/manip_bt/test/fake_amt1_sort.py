#!/usr/bin/env python3
"""Servidor FALSO de /amt1_sort (my_robot_msgs/Amt1Sort) para testar SO o executor.

Bancada da AMT1 (2026-08-28): o bt_yaml_executor (folha Amt1SortBT) e o
run_mission conversam com este server em vez do amt1_sort_action_node real.
Nao le TF, nao chama pick/place: so simula a duracao, o feedback por estagio,
a rejeicao, o abort, o result com success=false e o server "mudo" (watchdog).

ATENCAO: nao rodar com o amt1_sort_action_node real no ar (mesmo nome).

Parametros:
  latency_sec (default 2.0): duracao total simulada do goal; os estagios sao
      distribuidos por igual nesse tempo (cancel checado a cada 0.1 s).
  stages (string array): estagios publicados no feedback, na ordem.
  fail_stage (default ""):
      ""       -> SUCCEEDED com success=true (mesa ordenada);
      "abort"  -> goal_handle.abort() no fim (fail_reason pick_falhou);
      "result" -> SUCCEEDED com success=false, partial=true
                  (fail_reason fora_de_alcance) — a missao segue ao FINISH.
  reject (default false): rejeita o goal (GoalResponse.REJECT).
  silent (default false): NAO publica feedback nenhum — o watchdog de
      feedback da folha (amt1_stage_timeout) precisa disparar e cancelar.
  first_feedback_delay_sec (default 0.5): espera antes do 1o feedback — o
      cliente rclcpp so entrega feedback de goal cujo handle ja processou;
      publicar logo depois do accept perdia os 1os estagios.

Uso:
  python3 fake_amt1_sort.py --ros-args -p latency_sec:=1.0
  python3 fake_amt1_sort.py --ros-args -p fail_stage:=abort
  python3 fake_amt1_sort.py --ros-args -p silent:=true -p latency_sec:=60.0
"""
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from my_robot_msgs.action import Amt1Sort

DEFAULT_STAGES = [
    "starting",
    "checking_capacity",
    "waiting_servers",
    "observing_static",
    "registering_slots",
    "planning",
    "op_1/6_pick_5",
    "op_2/6_pick_1",
    "op_3/6_place_1_slot_0",
    "op_4/6_pick_4",
    "op_5/6_place_4_slot_3",
    "op_6/6_place_5_slot_4",
    "done",
]

# Exemplo do operador: 5,2,3,1,4,6 -> 1..6 com 3 picks e 3 places.
OBSERVED_ORDER = [5, 2, 3, 1, 4, 6]
FINAL_ORDER = [1, 2, 3, 4, 5, 6]


class FakeAmt1Sort(Node):
    """No com o /amt1_sort falso (latencia, estagios, abort, result ruim, reject, mudo)."""

    def __init__(self):
        super().__init__("fake_amt1_sort")
        self.declare_parameter("latency_sec", 2.0)
        self.declare_parameter("stages", DEFAULT_STAGES)
        self.declare_parameter("fail_stage", "")
        self.declare_parameter("reject", False)
        self.declare_parameter("silent", False)
        self.declare_parameter("first_feedback_delay_sec", 0.5)

        self._lock = threading.Lock()
        self._active = False
        group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, Amt1Sort, "/amt1_sort",
            execute_callback=self._execute,
            goal_callback=self._goal_response,
            cancel_callback=self._cancel_response,
            callback_group=group)
        self.get_logger().info(
            "Server falso no ar: /amt1_sort "
            f"(latency {self._param('latency_sec'):.1f} s, fail_stage='{self._param('fail_stage')}', "
            f"reject={self._param('reject')}, silent={self._param('silent')}).")

    def _param(self, name):
        return self.get_parameter(name).value

    def _goal_response(self, request):
        if bool(self._param("reject")):
            self.get_logger().warn("goal REJEITADO (reject:=true).")
            return GoalResponse.REJECT
        with self._lock:
            if self._active:
                self.get_logger().warn("goal REJEITADO: ja ha uma ordenacao em curso.")
                return GoalResponse.REJECT
            self._active = True
        tags = list(request.expected_tags)
        self.get_logger().info(
            f"goal aceito: ws={request.ws} table_pose={request.table_pose} "
            f"expected={tags} direction='{request.direction}' "
            f"max_onboard={request.max_onboard} observe_only={request.observe_only}")
        return GoalResponse.ACCEPT

    def _cancel_response(self, goal_handle):  # noqa: ARG002
        return CancelResponse.ACCEPT

    def _result(self, success, partial, fail_reason, message, done_stages):
        result = Amt1Sort.Result()
        result.success = success
        result.partial = partial
        result.message = message
        result.fail_reason = fail_reason
        result.observed_order = list(OBSERVED_ORDER)
        result.final_order = list(FINAL_ORDER) if success else list(OBSERVED_ORDER)
        result.missing_tags = []
        result.picks = len([s for s in done_stages if s.startswith("op_") and "_pick_" in s])
        result.places = len([s for s in done_stages if s.startswith("op_") and "_place_" in s])
        result.nudges = 0
        result.total_shift_m = 0.0
        return result

    def _execute(self, goal_handle):
        try:
            return self._execute_inner(goal_handle)
        finally:
            with self._lock:
                self._active = False

    def _execute_inner(self, goal_handle):
        latency = max(0.0, float(self._param("latency_sec")))
        stages = [str(s) for s in (self._param("stages") or []) if str(s)]
        silent = bool(self._param("silent"))
        fail_stage = (self._param("fail_stage") or "").strip()
        if not stages:
            stages = ["starting", "done"]
        per_stage = latency / len(stages)
        ops_total = len([s for s in stages if s.startswith("op_")])
        ops_done = 0
        done_stages = []

        first_delay = max(0.0, float(self._param("first_feedback_delay_sec")))
        deadline = time.monotonic() + first_delay
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("cancel recebido antes do 1o estagio")
                return self._result(False, False, "cancelado", "cancelado (fake)", done_stages)
            time.sleep(0.05)

        for stage in stages:
            if not silent:
                fb = Amt1Sort.Feedback()
                fb.current_stage = stage
                fb.ops_done = ops_done
                fb.ops_total = ops_total
                goal_handle.publish_feedback(fb)
                self.get_logger().info(f"feedback: {stage} ({ops_done}/{ops_total})")
            else:
                self.get_logger().info(f"(silent) estagio {stage} sem feedback")
            deadline = time.monotonic() + per_stage
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().warn(f"cancel recebido em {stage}")
                    return self._result(
                        False, False, "cancelado", f"cancelado em {stage} (fake)", done_stages)
                time.sleep(0.1)
            done_stages.append(stage)
            if stage.startswith("op_"):
                ops_done += 1

        if fail_stage == "abort":
            goal_handle.abort()
            self.get_logger().error("goal ABORTADO (fail_stage:=abort).")
            return self._result(False, False, "pick_falhou", "pick abortou (fake)", done_stages)
        if fail_stage == "result":
            goal_handle.succeed()
            self.get_logger().warn("goal SUCCEEDED com success=false (fail_stage:=result).")
            return self._result(
                False, True, "fora_de_alcance",
                "op_2/6_pick_1: fora de alcance (fake)", done_stages)
        if fail_stage:
            self.get_logger().warn(
                f"fail_stage='{fail_stage}' desconhecido (esperado abort|result): sucesso.")
        goal_handle.succeed()
        self.get_logger().info("goal concluido com sucesso (fake): mesa ordenada.")
        return self._result(True, False, "", "mesa ordenada (fake)", done_stages)


def main(args=None):
    rclpy.init(args=args)
    node = FakeAmt1Sort()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando o /amt1_sort falso (Ctrl+C).")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
