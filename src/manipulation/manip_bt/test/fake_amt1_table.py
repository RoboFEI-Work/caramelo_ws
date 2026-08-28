#!/usr/bin/env python3
"""Mesa de precisao FALSA da AMT1 (2026-08-28): TF das tags + /pick_tag + /place_tag.

Bancada do amt1_sort_action_node sem robo, sem camera e sem MoveIt, em
ROS_DOMAIN_ID=99. Par do fake_nav_actions.py (nudge_base e
/manip/base_shift_total). ATENCAO: nao rodar com os mtc_*_action_node reais
no ar (mesmos nomes de action) nem com o fake_pick_place.py.

Modelo: cada tag e um cubo parado na mesa e a mesa fica parada no mundo
(odom); e a BASE que anda de lado. O y de odom->base_footprint e o ultimo
/manip/base_shift_total (Float64 latched do fake_nav/dock_align_node,
+ = esquerda), logo o x de uma tag em manip_base_link vira x0 + shift
(manip +X = direita do robo, +Y = frente).

TF difundida a rate_hz (stamp = agora):
  odom -> base_footprint                (0, shift, 0)
  base_footprint -> base_link           (0, 0, 0.096)
  base_link -> manip_base_link          (0.217, 0, 0.083) yaw -1.57 (URDF real)
  manip_base_link -> tag_N              (x0 + shift, y_front, z) SO das tags
                                        que estao na mesa e nao sao "unseen"
  manip_base_link -> camera_color_optical_frame   estatico (olhando para baixo)

/pick_tag (PickTag):
  tag em unseen_tags ou fora da mesa -> success=true, skipped=true,
      fail_reason tag_nao_encontrada (a camera nao viu);
  |x_manip atual| > reach_x_max e final_attempt=false -> success=true,
      skipped=true, unreachable=true, suggested_base_shift_m quantizado
      (sinal que reduz |x|, mira |x| = reach_x_max - reach_margin, passos de
      0.05 m, clamp 0.10..0.25) — como o pick real com a fila de alcance;
      com final_attempt=true -> skipped, fail_reason fora_de_alcance_final;
  senao tira a tag da mesa e marca o 1o container vazio em
      container_state_file (yaml igual ao real: containers.containerN com
      occupied / tag_frame / status filled|empty).
/place_tag (PlaceTag):
  acha o container pela tag no yaml (nao achou -> skipped,
      fail_reason tag_nao_esta_em_container, como o place real);
  le a TF manip_base_link -> stack_on (o no AMT1 difunde os slots
      tag_amt1_slot_k filhos de odom); TF ausente ou velha (> max_tf_age_sec)
      -> skipped, unreachable=true, sugestao 0, fail_reason base_nao_vista
      (ramo PP do place real);
  |x_manip do slot| > reach_x_max -> skipped + unreachable + sugestao
      quantizada (o cubo volta ao container);
  senao poe a tag no x do slot e libera o container. stack_on vazio -> poe
      a tag em x=0 (mesa), com WARN.
  fail_stage:=pick|place faz o goal falhar (fail_times vezes; 0 = sempre) no
      modo fail_mode: "abort" (default) aborta o goal (result fail_reason
      falha_simulada); "unreachable" responde success=true, skipped=true,
      unreachable=true e sugestao 0 — pick com fail_reason alvo_fora_de_alcance,
      place com base_nao_vista (o ramo PP do place real: cubo devolvido ao
      container). O no AMT1 re-registra os slots e repete 1x, depois
      fora_de_alcance -> recoverOnboard (sem cubo na garra);
  reject_stage:=pick|place rejeita o goal. Feedback com current_stage.

Ao sair (SIGINT) imprime "ordem final: [..]" (tags da mesa da esquerda para a
direita) e o que ficou a bordo.

Uso:
  python3 fake_amt1_table.py --ros-args -p "layout:=['5:-0.25','2:-0.15','3:-0.05','1:0.05','4:0.15','6:0.25']" \\
      -p container_state_file:=/tmp/amt1_bench/container_states.yaml
  python3 fake_amt1_table.py --ros-args -p "unseen_tags:=['tag_3']" -p fail_stage:=place
"""
import math
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

from my_robot_msgs.action import PickTag, PlaceTag

DEFAULT_LAYOUT = ["5:-0.25", "2:-0.15", "3:-0.05", "1:0.05", "4:0.15", "6:0.25"]


def quantize_shift(x, reach_x_max, margin):
    """Sugestao de deslocamento da base (+ = esquerda) para trazer |x| ao alcance.

    Mira |x| = reach_x_max - margin, em passos de 0.05 m (para cima), clamp
    0.10..0.25 como o server real. 0 se ja esta ao alcance.
    """
    needed = abs(x) - reach_x_max + margin
    if needed <= 0.0:
        return 0.0
    mag = math.ceil(needed / 0.05 - 1e-9) * 0.05
    mag = min(0.25, max(0.10, mag))
    # x > 0 (direita) => a base precisa ir para a DIREITA (dy < 0).
    return -math.copysign(mag, x)


def parse_layout(items):
    """["5:-0.25", "2:-0.15"] -> {5: -0.25, 2: -0.15} (tag -> x0 em manip_base_link)."""
    layout = {}
    for item in items:
        text = str(item).strip()
        if not text:
            continue
        if ":" not in text:
            raise ValueError(f"layout: esperado 'tag:x', recebi '{text}'")
        tag, value = text.rsplit(":", 1)
        tag = tag.strip()
        if tag.startswith("tag_"):
            tag = tag[4:]
        tag_id = int(tag)
        if tag_id in layout:
            raise ValueError(f"layout: tag {tag_id} repetida")
        layout[tag_id] = float(value)
    return layout


def quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_tf(stamp, parent, child, xyz, quat):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(xyz[0])
    msg.transform.translation.y = float(xyz[1])
    msg.transform.translation.z = float(xyz[2])
    msg.transform.rotation.x = float(quat[0])
    msg.transform.rotation.y = float(quat[1])
    msg.transform.rotation.z = float(quat[2])
    msg.transform.rotation.w = float(quat[3])
    return msg


class FakeAmt1Table(Node):
    """Mesa falsa: estado dos cubos, TF, /pick_tag e /place_tag com regra de alcance."""

    def __init__(self):
        super().__init__("fake_amt1_table")
        self.declare_parameter("layout", DEFAULT_LAYOUT)
        self.declare_parameter("y_front", 0.35)
        self.declare_parameter("z", 0.142)
        self.declare_parameter("reach_x_max", 0.28)
        self.declare_parameter("reach_margin", 0.05)
        self.declare_parameter("latency_sec", 0.3)
        self.declare_parameter("fail_stage", "")
        self.declare_parameter("fail_times", 1)
        self.declare_parameter("fail_mode", "abort")
        self.declare_parameter("reject_stage", "")
        self.declare_parameter("unseen_tags", Parameter.Type.STRING_ARRAY)
        self.declare_parameter(
            "container_state_file",
            os.path.expanduser("~/.config/caramelo/container_states.yaml"))
        self.declare_parameter(
            "container_names", ["container1", "container2", "container3"])
        self.declare_parameter("reset_container_state", True)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("max_tf_age_sec", 1.0)

        self._lock = threading.Lock()
        # Leitura-modificacao-escrita do yaml dos containers e atomica entre
        # picks/places concorrentes (o server e reentrante).
        self._state_lock = threading.RLock()
        self._table = parse_layout(self._list_param("layout"))   # tag -> x0
        self._unseen = set()
        for t in self._list_param("unseen_tags"):
            text = str(t).strip()
            if text.startswith("tag_"):
                text = text[4:]
            if text:
                self._unseen.add(int(text))
        self._shift = 0.0
        self._fails_left = {"pick": 0, "place": 0}
        stage = (self._param("fail_stage") or "").strip()
        if stage in self._fails_left:
            times = int(self._param("fail_times"))
            self._fails_left[stage] = times if times > 0 else -1   # -1 = sempre
        self._fail_mode = (self._param("fail_mode") or "abort").strip()
        if self._fail_mode not in ("abort", "unreachable"):
            self.get_logger().warn(
                f"fail_mode '{self._fail_mode}' desconhecido (abort|unreachable): usando abort.")
            self._fail_mode = "abort"
        self._state_file = str(self._param("container_state_file"))
        self._container_names = [str(c) for c in self._list_param("container_names")]
        if bool(self._param("reset_container_state")):
            self._reset_containers()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._shift_sub = self.create_subscription(
            Float64, "/manip/base_shift_total", self._on_shift_total, latched)

        self._tf_buffer = Buffer()
        # spin_thread=False: o no ja esta num MultiThreadedExecutor.
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._tf_pub = TransformBroadcaster(self)
        self._static_pub = StaticTransformBroadcaster(self)
        # Camera acima da mesa olhando para baixo: eixo optico z = -z de manip
        # (rotacao de 180 graus em x).
        self._static_pub.sendTransform(make_tf(
            self.get_clock().now().to_msg(), "manip_base_link", "camera_color_optical_frame",
            (0.0, float(self._param("y_front")), 0.40), (1.0, 0.0, 0.0, 0.0)))
        rate = max(1.0, float(self._param("rate_hz")))
        self._timer = self.create_timer(1.0 / rate, self._broadcast)

        group = ReentrantCallbackGroup()
        self._servers = [
            ActionServer(
                self, PickTag, "/pick_tag",
                execute_callback=self._execute_pick,
                goal_callback=lambda req: self._goal_response("pick"),
                cancel_callback=self._cancel_response,
                callback_group=group),
            ActionServer(
                self, PlaceTag, "/place_tag",
                execute_callback=self._execute_place,
                goal_callback=lambda req: self._goal_response("place"),
                cancel_callback=self._cancel_response,
                callback_group=group),
        ]
        self.get_logger().info(
            "Mesa falsa no ar: /pick_tag, /place_tag, TF odom->...->manip_base_link->tag_N. "
            f"ordem inicial: {self._order_text()} | nunca vistas: {sorted(self._unseen)} | "
            f"reach_x_max {float(self._param('reach_x_max')):.2f} m | "
            f"fail_stage '{stage}' x{self._fails_left.get(stage, 0)} modo {self._fail_mode} | "
            f"containers em {self._state_file}")

    # -- parametros ------------------------------------------------------------
    def _param(self, name):
        return self.get_parameter(name).value

    def _list_param(self, name):
        try:
            value = self.get_parameter(name).value
        except Exception:  # ParameterUninitializedException
            return []
        if value is None:
            return []
        if isinstance(value, str):
            return [v for v in value.split(",") if v.strip()]
        return list(value)

    # -- estado da mesa ----------------------------------------------------------
    def _order_ids(self):
        """Tags da mesa da esquerda (x menor) para a direita."""
        with self._lock:
            return [tag for tag, _ in sorted(self._table.items(), key=lambda kv: kv[1])]

    def _order_text(self):
        with self._lock:
            items = sorted(self._table.items(), key=lambda kv: kv[1])
        ids = ", ".join(str(tag) for tag, _ in items)
        xs = " ".join(f"{tag}@{x:+.3f}" for tag, x in items)
        return f"[{ids}] ({xs})"

    def print_final_order(self):
        onboard = self._onboard_tags()
        with self._lock:
            items = sorted(self._table.items(), key=lambda kv: kv[1])
            shift = self._shift
        ids = ", ".join(str(tag) for tag, _ in items)
        xs = " ".join(f"{tag}@{x:+.3f}" for tag, x in items)
        self.get_logger().info(
            f"ordem final: [{ids}] (x0: {xs}) | a bordo: {onboard} | shift {shift:+.3f} m")

    def _on_shift_total(self, msg):
        with self._lock:
            self._shift = float(msg.data)
        self.get_logger().info(f"/manip/base_shift_total = {msg.data:+.3f} m (odom->base_footprint.y)")

    # -- yaml dos containers -----------------------------------------------------
    def _load_state(self):
        root = {}
        try:
            with open(self._state_file, "r", encoding="utf-8") as f:
                root = yaml.safe_load(f) or {}
        except FileNotFoundError:
            root = {}
        if not isinstance(root, dict):
            root = {}
        containers = root.get("containers")
        if not isinstance(containers, dict):
            containers = {}
            root["containers"] = containers
        for name in self._container_names:
            if not isinstance(containers.get(name), dict):
                containers[name] = {"occupied": False, "tag_frame": "", "status": "empty"}
        return root

    def _save_state(self, root):
        directory = os.path.dirname(self._state_file)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = self._state_file + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            yaml.safe_dump(root, f, default_flow_style=False, sort_keys=False)
        os.replace(tmp, self._state_file)

    def _reset_containers(self):
        with self._state_lock:
            root = self._load_state()
            for name in self._container_names:
                root["containers"][name] = {"occupied": False, "tag_frame": "", "status": "empty"}
            root.pop("table_slots", None)
            self._save_state(root)
        self.get_logger().info(f"containers zerados em {self._state_file}")

    def _onboard_tags(self):
        with self._state_lock:
            root = self._load_state()
        return [
            c.get("tag_frame", "") for c in root["containers"].values()
            if isinstance(c, dict) and c.get("occupied")]

    def _occupy_first_empty(self, tag_frame):
        with self._state_lock:
            root = self._load_state()
            for name in self._container_names:
                c = root["containers"][name]
                if not c.get("occupied"):
                    root["containers"][name] = {
                        "occupied": True, "tag_frame": tag_frame, "status": "filled"}
                    self._save_state(root)
                    return name
        return None

    def _find_container(self, tag_frame):
        with self._state_lock:
            root = self._load_state()
        for name, c in root["containers"].items():
            if isinstance(c, dict) and c.get("occupied") and c.get("tag_frame") == tag_frame:
                return name
        return None

    def _free_container(self, name):
        with self._state_lock:
            root = self._load_state()
            root["containers"][name] = {"occupied": False, "tag_frame": "", "status": "empty"}
            self._save_state(root)

    # -- TF ------------------------------------------------------------------
    def _broadcast(self):
        stamp = self.get_clock().now().to_msg()
        y_front = float(self._param("y_front"))
        z = float(self._param("z"))
        with self._lock:
            shift = self._shift
            table = dict(self._table)
        msgs = [
            make_tf(stamp, "odom", "base_footprint", (0.0, shift, 0.0), (0.0, 0.0, 0.0, 1.0)),
            make_tf(stamp, "base_footprint", "base_link", (0.0, 0.0, 0.096), (0.0, 0.0, 0.0, 1.0)),
            make_tf(stamp, "base_link", "manip_base_link", (0.217, 0.0, 0.083), quat_yaw(-1.57)),
        ]
        for tag, x0 in table.items():
            if tag in self._unseen:
                continue
            msgs.append(make_tf(
                stamp, "manip_base_link", f"tag_{tag}", (x0 + shift, y_front, z), (0.0, 0.0, 0.0, 1.0)))
        self._tf_pub.sendTransform(msgs)

    def _lookup_slot_x(self, frame):
        """x do frame em manip_base_link (e idade da TF). None se nao houver TF."""
        try:
            tf = self._tf_buffer.lookup_transform("manip_base_link", frame, rclpy.time.Time())
        except Exception as ex:  # LookupException, ConnectivityException, ExtrapolationException
            return None, None, str(ex).splitlines()[0]
        stamp = rclpy.time.Time.from_msg(tf.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return float(tf.transform.translation.x), age, ""

    # -- action servers --------------------------------------------------------
    def _goal_response(self, stage):
        if (self._param("reject_stage") or "").strip() == stage:
            self.get_logger().warn(f"goal REJEITADO em '{stage}' (reject_stage).")
            return GoalResponse.REJECT
        self.get_logger().info(f"goal aceito em '{stage}'.")
        return GoalResponse.ACCEPT

    def _cancel_response(self, goal_handle):  # noqa: ARG002
        return CancelResponse.ACCEPT

    @staticmethod
    def _tag_id(tag_frame):
        text = str(tag_frame).strip()
        if text.startswith("tag_"):
            text = text[4:]
        try:
            return int(text)
        except ValueError:
            return -1

    def _wait_with_feedback(self, goal_handle, stage, feedback_type, stages):
        """Espera latency_sec publicando feedback; False se cancelado."""
        latency = max(0.0, float(self._param("latency_sec")))
        per_stage = latency / max(1, len(stages))
        for name in stages:
            fb = feedback_type()
            fb.current_stage = name
            goal_handle.publish_feedback(fb)
            deadline = time.monotonic() + per_stage
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().warn(f"cancel recebido em {stage} ({name})")
                    return False
                time.sleep(0.05)
        return True

    def _should_fail(self, stage):
        left = self._fails_left.get(stage, 0)
        if left == 0:
            return False
        if left > 0:
            self._fails_left[stage] = left - 1
        return True

    @staticmethod
    def _fill(result, success, skipped, fail_reason, message, unreachable=False, suggestion=0.0,
              target_x=0.0, target_y=0.0):
        result.success = success
        result.skipped = skipped
        result.fail_reason = fail_reason
        result.message = message
        result.unreachable = unreachable
        result.suggested_base_shift_m = float(suggestion)
        result.target_x = float(target_x)
        result.target_y = float(target_y)
        return result

    def _execute_pick(self, goal_handle):
        req = goal_handle.request
        tag_frame = str(req.tag_frame)
        tag = self._tag_id(tag_frame)
        final_attempt = bool(getattr(req, "final_attempt", True))
        result = PickTag.Result()
        self.get_logger().info(
            f"pick {tag_frame} (table_pose={req.table_pose} final_attempt={final_attempt}) "
            f"mesa {self._order_text()} shift {self._shift:+.3f}")
        if not self._wait_with_feedback(goal_handle, "pick", PickTag.Feedback,
                                        ["approach", "grasp", "retreat"]):
            return self._fill(result, False, False, "cancelado", "cancelado (fake)")
        if self._should_fail("pick"):
            if self._fail_mode == "unreachable":
                # Alvo visto mas sem IK e sem sugestao: o no re-registra e
                # repete 1x, depois responde fora_de_alcance (garra vazia).
                goal_handle.succeed()
                self.get_logger().warn(
                    f"'pick' PULADO {tag_frame}: fora de alcance sem sugestao (fail_mode=unreachable).")
                return self._fill(
                    result, True, True, "alvo_fora_de_alcance",
                    f"fake: {tag_frame} fora de alcance sem sugestao (fail_mode=unreachable)",
                    unreachable=True, suggestion=0.0)
            goal_handle.abort()
            self.get_logger().error("'pick' abortado (fail_stage).")
            return self._fill(result, False, False, "falha_simulada", "falha simulada (fake)")

        with self._lock:
            on_table = tag in self._table
            x_now = (self._table.get(tag, 0.0) + self._shift) if on_table else 0.0
        y_front = float(self._param("y_front"))
        if tag in self._unseen or not on_table:
            goal_handle.succeed()
            self.get_logger().warn(f"'pick' PULADO: {tag_frame} nunca vista pela camera (fake).")
            return self._fill(
                result, True, True, "tag_nao_encontrada",
                f"fake: a camera nao viu {tag_frame}")

        reach = float(self._param("reach_x_max"))
        if abs(x_now) > reach:
            suggestion = quantize_shift(x_now, reach, float(self._param("reach_margin")))
            goal_handle.succeed()
            if not final_attempt:
                self.get_logger().warn(
                    f"'pick' FORA DE ALCANCE {tag_frame}: x_manip {x_now:+.3f} m (> {reach:.2f}) "
                    f"-> sugestao {suggestion:+.2f} m (final_attempt=false).")
                return self._fill(
                    result, True, True, "alvo_fora_de_alcance",
                    f"fake: {tag_frame} em x={x_now:+.3f} m fora do alcance",
                    unreachable=True, suggestion=suggestion, target_x=x_now, target_y=y_front)
            self.get_logger().warn(
                f"'pick' PULADO na tentativa final: {tag_frame} x_manip {x_now:+.3f} m.")
            return self._fill(
                result, True, True, "fora_de_alcance_final",
                f"fake: {tag_frame} continua fora de alcance na tentativa final",
                unreachable=True, suggestion=suggestion, target_x=x_now, target_y=y_front)

        container = self._occupy_first_empty(tag_frame)
        if container is None:
            goal_handle.abort()
            self.get_logger().error(f"'pick' {tag_frame}: nenhum container vazio (fake).")
            return self._fill(result, False, False, "sem_container_vazio", "fake: containers cheios")
        with self._lock:
            self._table.pop(tag, None)
        goal_handle.succeed()
        self.get_logger().info(
            f"'pick' {tag_frame} ok (x_manip {x_now:+.3f} m) -> {container}. mesa agora {self._order_text()}")
        return self._fill(
            result, True, False, "", f"fake: {tag_frame} no {container}",
            target_x=x_now, target_y=y_front)

    def _execute_place(self, goal_handle):
        req = goal_handle.request
        tag_frame = str(req.tag_frame)
        tag = self._tag_id(tag_frame)
        stack_on = str(getattr(req, "stack_on", ""))
        final_attempt = bool(getattr(req, "final_attempt", True))
        result = PlaceTag.Result()
        self.get_logger().info(
            f"place {tag_frame} (table_pose={req.table_pose} ws={req.ws} stack_on='{stack_on}' "
            f"final_attempt={final_attempt}) shift {self._shift:+.3f}")
        if not self._wait_with_feedback(goal_handle, "place", PlaceTag.Feedback,
                                        ["approach", "release", "retreat"]):
            return self._fill(result, False, False, "cancelado", "cancelado (fake)")
        if self._should_fail("place"):
            if self._fail_mode == "unreachable":
                # Ramo PP do place real (kBaseNotSeen/kNoIk): nao cai na mesa,
                # devolve o cubo ao container e responde skipped+unreachable
                # com sugestao 0 — o cubo continua no container (yaml intacto).
                goal_handle.succeed()
                self.get_logger().warn(
                    f"'place' PULADO {tag_frame}: base {stack_on or '<sem stack_on>'} nao vista "
                    "(fail_mode=unreachable); cubo fica no container.")
                return self._fill(
                    result, True, True, "base_nao_vista",
                    f"fake: base {stack_on} nao vista (fail_mode=unreachable)",
                    unreachable=True, suggestion=0.0)
            goal_handle.abort()
            self.get_logger().error("'place' abortado (fail_stage).")
            return self._fill(result, False, False, "falha_simulada", "falha simulada (fake)")

        container = self._find_container(tag_frame)
        if container is None:
            goal_handle.succeed()
            self.get_logger().warn(f"'place' PULADO: {tag_frame} nao esta em nenhum container (yaml).")
            return self._fill(
                result, True, True, "tag_nao_esta_em_container",
                f"fake: {tag_frame} nao esta em container")

        y_front = float(self._param("y_front"))
        with self._lock:
            shift = self._shift
        if stack_on:
            x_slot, age, err = self._lookup_slot_x(stack_on)
            max_age = float(self._param("max_tf_age_sec"))
            if x_slot is None or age > max_age:
                goal_handle.succeed()
                why = err if x_slot is None else f"TF com {age:.2f} s de idade (> {max_age:.1f})"
                self.get_logger().warn(f"'place' {tag_frame}: base {stack_on} nao vista ({why}).")
                return self._fill(
                    result, True, True, "base_nao_vista",
                    f"fake: sem TF fresca de {stack_on} ({why})", unreachable=True, suggestion=0.0)
        else:
            x_slot = 0.0
            self.get_logger().warn(f"'place' {tag_frame} sem stack_on: soltando em x=0 (mesa).")

        reach = float(self._param("reach_x_max"))
        if abs(x_slot) > reach:
            suggestion = quantize_shift(x_slot, reach, float(self._param("reach_margin")))
            goal_handle.succeed()
            self.get_logger().warn(
                f"'place' FORA DE ALCANCE {tag_frame} -> {stack_on}: x_manip {x_slot:+.3f} m "
                f"(> {reach:.2f}) -> sugestao {suggestion:+.2f} m; cubo volta ao {container}.")
            return self._fill(
                result, True, True, "alvo_fora_de_alcance" if not final_attempt else
                "chegada_fora_da_tolerancia",
                f"fake: {stack_on} em x={x_slot:+.3f} m fora do alcance",
                unreachable=True, suggestion=suggestion if not final_attempt else 0.0,
                target_x=x_slot, target_y=y_front)

        with self._lock:
            self._table[tag] = x_slot - shift
        self._free_container(container)
        goal_handle.succeed()
        self.get_logger().info(
            f"'place' {tag_frame} ok em {stack_on or 'mesa'} (x_manip {x_slot:+.3f} m) <- {container}. "
            f"mesa agora {self._order_text()}")
        return self._fill(
            result, True, False, "", f"fake: {tag_frame} solto em {stack_on or 'mesa'}",
            target_x=x_slot, target_y=y_front)


def main(args=None):
    rclpy.init(args=args)
    node = FakeAmt1Table()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando a mesa falsa (Ctrl+C).")
    finally:
        node.print_final_order()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
