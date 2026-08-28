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

Fila de alcance (2026-08-28):
  unreachable_picks / unreachable_places (lista de strings "tag_7:0.20"): a tag
      so e alcancavel quando o deslocamento lateral ACUMULADO da base
      (assinado em /manip/base_shift_total, Float64 transient_local, + =
      esquerda) fica a <= reach_tolerance_m do valor pedido. O acumulado
      zera quando chega /fake_nav/station (String) — o align_to_dock do
      fake_nav_actions publica os dois, como o dock_align_node real.
  unseen_tags (lista de strings): tag nunca vista -> skipped=true SEM
      unreachable (fail_reason tag_nao_vista) — comportamento de hoje.
  reach_tolerance_m (default 0.06).
  Avaliacao depois de latency_sec. Com final_attempt=false: success=true,
  skipped=true, unreachable=true, suggested_base_shift_m = quantize(restante)
  (sinal * clamp(round(|r|/0.05)*0.05, 0.10, 0.25); 0 se |r| < 0.05). Com
  final_attempt=true: pick -> skipped (fail_reason fora_de_alcance_final);
  place -> success=true (fallback na mesa, como o no real).
  Mensagens sem os campos novos sao toleradas (getattr/hasattr).

Uso:
  python3 fake_pick_place.py --ros-args -p latency_sec:=0.5 -p fail_stage:=pick
  python3 fake_pick_place.py --ros-args -p "unreachable_picks:=['tag_7:0.20']" \
      -p "unseen_tags:=['tag_9']"
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String

from my_robot_msgs.action import PickTag, PlaceTag


def quantize_shift(remaining):
    """Sugestao de deslocamento como o server real: +-{0.10,0.15,0.20,0.25} ou 0."""
    mag = abs(remaining)
    if mag < 0.05:
        return 0.0
    mag = round(mag / 0.05) * 0.05
    mag = min(0.25, max(0.10, mag))
    return math.copysign(mag, remaining)


def parse_reach_specs(items):
    """["tag_7:0.20", "tag_8:-0.15"] -> {"tag_7": 0.20, "tag_8": -0.15}."""
    specs = {}
    for item in items:
        text = str(item).strip()
        if not text:
            continue
        if ":" not in text:
            raise ValueError(f"esperado 'tag:deslocamento', recebi '{text}'")
        tag, value = text.rsplit(":", 1)
        specs[tag.strip()] = float(value)
    return specs


class FakePickPlace(Node):
    """No com /pick_tag e /place_tag falsos (latencia, falha, rejeicao e alcance simulaveis)."""

    def __init__(self):
        super().__init__("fake_pick_place")
        self.declare_parameter("latency_sec", 1.0)
        self.declare_parameter("fail_stage", "")
        self.declare_parameter("reject_stage", "")
        self.declare_parameter("unreachable_picks", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("unreachable_places", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("unseen_tags", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("reach_tolerance_m", 0.06)

        self._lock = threading.Lock()
        self._base_shift_total = 0.0
        self._reach = {
            "pick": parse_reach_specs(self._list_param("unreachable_picks")),
            "place": parse_reach_specs(self._list_param("unreachable_places")),
        }
        self._unseen = {str(t).strip() for t in self._list_param("unseen_tags") if str(t).strip()}

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._shift_sub = self.create_subscription(
            Float64, "/manip/base_shift_total", self._on_shift_total, latched)
        self._station_sub = self.create_subscription(
            String, "/fake_nav/station", self._on_station, latched)

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
        self.get_logger().info(
            "Servers falsos no ar: /pick_tag, /place_tag. "
            f"fora de alcance: picks={self._reach['pick']} places={self._reach['place']} "
            f"nunca vistas={sorted(self._unseen)}")

    # -- helpers ---------------------------------------------------------------
    def _param(self, name):
        return self.get_parameter(name).value

    def _list_param(self, name):
        try:
            value = self.get_parameter(name).value
        except Exception:  # ParameterUninitializedException (nao declarado na linha de comando)
            return []
        if value is None:
            return []
        if isinstance(value, str):
            return [v for v in value.split(",") if v.strip()]
        return list(value)

    def _on_shift_total(self, msg):
        with self._lock:
            self._base_shift_total = float(msg.data)
        self.get_logger().info(f"base_shift_total = {msg.data:+.3f} m")

    def _on_station(self, msg):
        with self._lock:
            self._base_shift_total = 0.0
        self.get_logger().info(f"estacao '{msg.data}': deslocamento acumulado zerado.")

    def _goal_response(self, stage):
        if (self._param("reject_stage") or "").strip() == stage:
            self.get_logger().warn(f"goal REJEITADO em '{stage}' (reject_stage).")
            return GoalResponse.REJECT
        self.get_logger().info(f"goal aceito em '{stage}'.")
        return GoalResponse.ACCEPT

    def _cancel_response(self, goal_handle):  # noqa: ARG002
        return CancelResponse.ACCEPT

    @staticmethod
    def _set_if_has(result, field, value):
        if hasattr(result, field):
            setattr(result, field, value)

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

        req = goal_handle.request
        tag = str(getattr(req, "tag_frame", ""))
        # Mensagem antiga (sem o campo) = comportamento de sempre = tentativa final.
        final_attempt = bool(getattr(req, "final_attempt", True))
        extra = ""
        if stage == "place":
            extra = (f" tag={tag} mesa={getattr(req, 'table_pose', '?')}"
                     f" ws={getattr(req, 'ws', '')} stack_on={getattr(req, 'stack_on', '')}"
                     f" container_color={getattr(req, 'container_color', '')}")

        # Tag nunca vista: pula sem unreachable (como hoje).
        if tag in self._unseen:
            goal_handle.succeed()
            result.success = True
            result.skipped = True
            result.fail_reason = "tag_nao_vista"
            result.message = f"fake: a camera nunca viu {tag}"
            self._set_if_has(result, "unreachable", False)
            self._set_if_has(result, "suggested_base_shift_m", 0.0)
            self.get_logger().warn(f"'{stage}' PULADO: {tag} nunca vista (fake).{extra}")
            return result

        required = self._reach[stage].get(tag)
        if required is not None:
            with self._lock:
                total = self._base_shift_total
            remaining = required - total
            tolerance = float(self._param("reach_tolerance_m"))
            if abs(remaining) > tolerance:
                suggestion = quantize_shift(remaining)
                goal_handle.succeed()
                result.success = True
                self._set_if_has(result, "suggested_base_shift_m", float(suggestion))
                if not final_attempt:
                    result.skipped = True
                    result.fail_reason = "alvo_fora_de_alcance"
                    result.message = (f"fake: {tag} fora de alcance (acumulado {total:+.3f} m, "
                                      f"falta {remaining:+.3f} m)")
                    self._set_if_has(result, "unreachable", True)
                    self.get_logger().warn(
                        f"'{stage}' FORA DE ALCANCE {tag}: acumulado {total:+.3f} m, "
                        f"falta {remaining:+.3f} m -> sugestao {suggestion:+.2f} m "
                        f"(final_attempt=false).{extra}")
                    return result
                if stage == "pick":
                    result.skipped = True
                    result.fail_reason = "fora_de_alcance_final"
                    result.message = (f"fake: {tag} continua fora de alcance na tentativa "
                                      f"final (falta {remaining:+.3f} m) — pulado")
                    self._set_if_has(result, "unreachable", True)
                    self.get_logger().warn(
                        f"'pick' PULADO na tentativa final: {tag} fora de alcance "
                        f"(falta {remaining:+.3f} m).")
                    return result
                # place na tentativa final: solta na mesa (fallback), como o no real.
                result.skipped = False
                result.message = (f"fake: {tag} fora de alcance na tentativa final -> "
                                  f"soltou na mesa (fallback)")
                self._set_if_has(result, "unreachable", False)
                self.get_logger().warn(
                    f"'place' na tentativa final: {tag} fora de alcance (falta "
                    f"{remaining:+.3f} m) -> fallback na mesa (fake).{extra}")
                return result
            self.get_logger().info(
                f"'{stage}' {tag} ALCANCAVEL: acumulado {total:+.3f} m (falta "
                f"{remaining:+.3f} m, tolerancia {tolerance:.2f}).")

        goal_handle.succeed()
        self.get_logger().info(f"'{stage}' concluido com sucesso (fake).{extra}")
        result.success = True
        result.message = "fake"
        self._set_if_has(result, "unreachable", False)
        self._set_if_has(result, "suggested_base_shift_m", 0.0)
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
