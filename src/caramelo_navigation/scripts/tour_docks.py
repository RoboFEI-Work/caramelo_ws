#!/usr/bin/env python3
# Tour de conferencia dos docks: leva o robo por TODOS os docks do mapa
# (WS1..WSn, SH*, PP*, RT*, em ordem numerica) e termina no FINISH, com a
# MESMA sequencia do GoToWS da missao (manip_bt/go_to_ws_bt.cpp) em cada
# dock: undock -> Nav2 ate a staging -> align_to_dock (que faz o approach).
# Mede por dock o tempo de cada etapa, o resultado de cada action e o erro
# da pose final (TF map -> base_footprint) no frame do dock; fala em
# /manip/speech e grava tour_<data>.md na pasta do mapa. Nao grava bag.
# Quem usa: operador, antes da missao, para conferir se cada ponto de
# docking esta bem gravado e onde o robo tem dificuldade de chegar.
# Como roda (stack no ar: ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=<mapa>):
#   ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026
#   ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026 --only WS1,WS3 --skip SH1 --yes
#   ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026 --no-align   # /dock_robot faz o approach, sem align
#   ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026 --finish ""  # nao vai ao FINISH no fim
# Ctrl+C: cancela a action em voo (o robo para) e grava o relatorio parcial.
import argparse
import datetime
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from caramelo_msgs.action import AlignToDock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from caramelo_docking_common import (  # noqa: E402
    docking_file, load_yaml, resolve_map_folder, service_areas_file,
    wait_for_action_servers, yaw_from_quaternion,
)
from caramelo_tour_common import (  # noqa: E402
    FLOW_NAV_ONLY, FLOW_OPENNAV_ALIGN, FLOW_OPENNAV_ONLY, FLOW_STAGING_ALIGN,
    STATUS_CANCELED, STATUS_FAIL, STATUS_OK, STATUS_SKIPPED, DockEntry, DockReport,
    align_status, format_report_md, format_table, load_dock_entries,
    parse_dock_list, plan_tour, pose_error_in_dock_frame, report_filename,
    speech_failed, speech_ready, speech_summary, summarize,
)

# Estados do feedback do /dock_robot (opennav_docking), como no dock_to.
DOCK_FEEDBACK_STATES = {
    0: "NONE", 1: "NAV_TO_STAGING_POSE", 2: "INITIAL_PERCEPTION",
    3: "CONTROLLING", 4: "WAIT_FOR_CHARGE", 5: "RETRY",
}

# Status de uma action (ActionOutcome.status).
A_OK, A_FAIL, A_REJECTED, A_TIMEOUT, A_CANCELED, A_NO_SERVER = (
    "ok", "falhou", "rejeitado", "timeout", "cancelado", "sem_server")

# Texto curto por status, para a voz e a coluna Mensagem.
STATUS_TEXT = {
    A_FAIL: "falhou", A_REJECTED: "rejeitado", A_TIMEOUT: "timeout",
    A_CANCELED: "cancelado", A_NO_SERVER: "sem action server",
}


@dataclass
class ActionOutcome:
    status: str
    seconds: float = 0.0
    message: str = ""
    result: object = None


def _pose_goal(pose, stamp) -> PoseStamped:
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = stamp
    goal.pose.position.x = float(pose[0])
    goal.pose.position.y = float(pose[1])
    goal.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
    goal.pose.orientation.w = math.cos(float(pose[2]) / 2.0)
    return goal


class TourDocks(Node):
    """Cliente das 4 actions da missao + TF + voz. Uma visita por dock."""

    def __init__(self, args, map_folder: Path):
        super().__init__("tour_docks")
        self.args = args
        self.map_folder = map_folder
        self.undock_client = ActionClient(self, UndockRobot, "/undock_robot")
        self.dock_client = ActionClient(self, DockRobot, "/dock_robot")
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        # Nome RELATIVO de proposito: e assim que o dock_align_node anuncia.
        self.align_client = ActionClient(self, AlignToDock, "align_to_dock")
        self.speech_pub = self.create_publisher(String, "/manip/speech", 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_warned = False
        self.interrupted = False
        self._active_goal = None
        self._last_feedback = None

    # ------------------------------------------------------------ util
    def say(self, text: str):
        if self.args.no_speech:
            return
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)

    def log(self, dock_id: str, text: str):
        self.get_logger().info(f"[{dock_id}] {text}")

    def interrupt(self):
        """Ctrl+C: a action em voo e cancelada no proximo passo do spin."""
        if not self.interrupted:
            self.interrupted = True
            print("\nCtrl+C: cancelando a action em voo e encerrando o tour "
                  "(relatorio parcial sera gravado)...", flush=True)

    def _spin_until(self, future, timeout_s: float, honor_interrupt: bool = True) -> bool:
        """Spin em passos de 0,1 s ate o future completar; False em timeout
        ou (se honor_interrupt) ao Ctrl+C."""
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if honor_interrupt and self.interrupted:
                return False
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def spin_for(self, seconds: float):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def graph_actions(self) -> set:
        return {name for name, _types in get_action_names_and_types(self)}

    def _wait_server(self, client: ActionClient, timeout_s: float) -> bool:
        """Como ActionClient.wait_for_server, mas spinando (TF e feedbacks
        continuam chegando) e saindo no Ctrl+C."""
        deadline = time.monotonic() + timeout_s
        while not client.server_is_ready():
            if self.interrupted or time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    # ------------------------------------------------------------- TF
    def robot_pose(self, settle_s: float = 0.5, wait_s: float = 2.0):
        """(x, y, yaw) do robo no map via TF map -> base_footprint (AMCL),
        ou None. Spinna `settle_s` antes (TF fresca) e insiste ate `wait_s`."""
        self.spin_for(settle_s)
        deadline = time.monotonic() + wait_s
        while True:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.args.global_frame, self.args.base_frame, Time())
                t, q = tf.transform.translation, tf.transform.rotation
                return (t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                if time.monotonic() > deadline:
                    if not self._tf_warned:
                        self.get_logger().warn(
                            f"sem TF {self.args.global_frame} -> {self.args.base_frame}: "
                            "erro da pose nao sera medido (AMCL no ar?)")
                        self._tf_warned = True
                    return None
                rclpy.spin_once(self, timeout_sec=0.1)

    # --------------------------------------------------------- actions
    def run_action(self, client: ActionClient, goal, label: str, deadline_s: float,
                   feedback_cb=None) -> ActionOutcome:
        """Manda o goal e espera o resultado ate `deadline_s`; em timeout ou
        Ctrl+C cancela o goal e espera o server confirmar (robo parado)."""
        t0 = time.monotonic()
        name = getattr(client, "_action_name", label)
        if not self._wait_server(client, self.args.server_wait_timeout):
            elapsed = time.monotonic() - t0
            if self.interrupted:
                return ActionOutcome(A_CANCELED, elapsed, "Ctrl+C antes do goal")
            return ActionOutcome(A_NO_SERVER, elapsed,
                                 f"action server '{name}' nao respondeu em "
                                 f"{self.args.server_wait_timeout:.0f} s")
        self._last_feedback = None
        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        if not self._spin_until(send_future, self.args.server_wait_timeout):
            status = A_CANCELED if self.interrupted else A_TIMEOUT
            return ActionOutcome(status, time.monotonic() - t0,
                                 f"'{name}' nao respondeu ao goal")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return ActionOutcome(A_REJECTED, time.monotonic() - t0, f"goal de {label} rejeitado")
        self._active_goal = handle
        result_future = handle.get_result_async()
        if not self._spin_until(result_future, deadline_s):
            status = A_CANCELED if self.interrupted else A_TIMEOUT
            self._cancel(handle, result_future)
            self._active_goal = None
            why = "Ctrl+C" if status == A_CANCELED else f"timeout de {deadline_s:.0f} s"
            return ActionOutcome(status, time.monotonic() - t0, f"{why}, goal cancelado")
        self._active_goal = None
        wrapped = result_future.result()
        result = wrapped.result
        elapsed = time.monotonic() - t0
        message = (getattr(result, "message", "") or getattr(result, "error_msg", "") or "")
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return ActionOutcome(A_CANCELED, elapsed, message or f"{label} cancelado", result)
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and getattr(result, "success", True):
            return ActionOutcome(A_OK, elapsed, message, result)
        code = getattr(result, "error_code", None)
        detail = message or f"status {wrapped.status}"
        if code:
            detail = f"{detail} (error_code={code})"
        return ActionOutcome(A_FAIL, elapsed, detail, result)

    def _cancel(self, handle, result_future):
        """Cancela e ESPERA a confirmacao (ate 1 s) e o resultado (ate 3 s):
        um cancel so enfileirado se perderia com o processo encerrando."""
        try:
            cancel_future = handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001 - goal ja terminou no server
            self.get_logger().warn(f"cancel: {exc}")
            return
        if not self._spin_until(cancel_future, 1.0, honor_interrupt=False):
            self.get_logger().error(
                "cancel: server nao confirmou o cancelamento em 1 s - confira se o robo parou.")
        self._spin_until(result_future, 3.0, honor_interrupt=False)

    def cancel_active(self):
        """Usado no encerramento forcado: cancela o que estiver em voo."""
        if self._active_goal is not None:
            handle, self._active_goal = self._active_goal, None
            self._cancel(handle, handle.get_result_async())

    # -------------------------------------------------------- feedbacks
    def _align_feedback(self, dock_id):
        def cb(msg):
            phase = msg.feedback.phase
            if phase != self._last_feedback:
                self._last_feedback = phase
                self.log(dock_id, f"align: {phase}")
        return cb

    def _dock_feedback(self, dock_id):
        def cb(msg):
            state = DOCK_FEEDBACK_STATES.get(msg.feedback.state, str(msg.feedback.state))
            if state != self._last_feedback:
                self._last_feedback = state
                self.log(dock_id, f"dock_robot: {state} (retries={msg.feedback.num_retries})")
        return cb

    # ------------------------------------------------------------ visita
    def _fail(self, rep: DockReport, stage: str, out: ActionOutcome) -> DockReport:
        rep.failed_stage = stage
        rep.status = STATUS_CANCELED if out.status == A_CANCELED else STATUS_FAIL
        rep.message = f"{stage}: {STATUS_TEXT.get(out.status, out.status)}"
        if out.message:
            rep.message += f" - {out.message}"
        self.get_logger().error(f"[{rep.dock_id}] {rep.message}")
        if rep.status == STATUS_FAIL:
            # "!Dock WS3 falhou: alinhamento" / "...: alinhamento timeout".
            reason = stage if out.status == A_FAIL else f"{stage} {STATUS_TEXT.get(out.status, out.status)}"
            self.say(speech_failed(rep.dock_id, reason))
        return rep

    def _measure(self, rep: DockReport, dock: DockEntry):
        pose = self.robot_pose()
        if pose is None:
            return
        rep.robot_pose = pose
        rep.err_long, rep.err_lat, rep.err_yaw = pose_error_in_dock_frame(pose, dock.pose)
        self.log(dock.dock_id,
                 f"pose final: long {rep.err_long * 100:+.1f} cm, lat {rep.err_lat * 100:+.1f} cm, "
                 f"yaw {math.degrees(rep.err_yaw):+.1f} graus (robo em {pose[0]:.3f}, "
                 f"{pose[1]:.3f}, {math.degrees(pose[2]):+.1f} graus)")

    def visit(self, dock: DockEntry, undock_type: Optional[str], flow: str) -> DockReport:
        """Mesma sequencia do GoToWS: (1) undock se o robo esta encostado,
        (2) chegada conforme o fluxo, (3) align salvo no-align/nav pura,
        (4) medida da pose final e fala."""
        args = self.args
        rep = DockReport(dock.dock_id, flow=flow)
        self.get_logger().info(f"=== Dock {dock.dock_id} ({flow}) ===")
        self.say(f"~Indo para {dock.dock_id}")

        # (1) undock do dock atual (pulado no primeiro: o robo esta no START).
        if undock_type is not None:
            goal = UndockRobot.Goal()
            goal.dock_type = undock_type
            goal.max_undocking_time = float(args.max_undocking_time)
            self.log(dock.dock_id, f"undock (dock_type='{undock_type}')...")
            out = self.run_action(self.undock_client, goal, "undock", args.max_undocking_time + 15.0)
            rep.t_undock = out.seconds
            self.log(dock.dock_id, f"undock: {out.status} em {out.seconds:.1f} s {out.message}".rstrip())
            if out.status != A_OK:
                return self._fail(rep, "undock", out)

        # (2) chegada.
        if flow == FLOW_NAV_ONLY:
            # Navegacao PURA (use_docking=false: START/FINISH), sem align.
            goal = NavigateToPose.Goal()
            goal.pose = _pose_goal(dock.pose, self.get_clock().now().to_msg())
            self.log(dock.dock_id, f"navegacao pura ate ({dock.pose[0]:.2f}, {dock.pose[1]:.2f})...")
            out = self.run_action(self.nav_client, goal, "navegacao", args.nav_timeout)
            rep.t_nav = out.seconds
            self.log(dock.dock_id, f"navegacao: {out.status} em {out.seconds:.1f} s {out.message}".rstrip())
            if out.status != A_OK:
                self._measure(rep, dock)
                return self._fail(rep, "navegacao", out)
            rep.message = "navegacao pura (sem dock)"
        elif flow == FLOW_STAGING_ALIGN:
            # Fluxo padrao do GoToWS: Nav2 ate a STAGING; o align faz o approach.
            staging = dock.staging
            goal = NavigateToPose.Goal()
            goal.pose = _pose_goal(staging, self.get_clock().now().to_msg())
            self.log(dock.dock_id, f"Nav2 ate a staging ({staging[0]:.2f}, {staging[1]:.2f}, "
                                   f"offset {dock.staging_x_offset:+.2f} m)...")
            out = self.run_action(self.nav_client, goal, "navegacao ate a staging",
                                  args.max_staging_time + 30.0)
            rep.t_nav = out.seconds
            self.log(dock.dock_id, f"staging: {out.status} em {out.seconds:.1f} s {out.message}".rstrip())
            if out.status != A_OK:
                self._measure(rep, dock)
                return self._fail(rep, "navegacao ate a staging", out)
            pose = self.robot_pose()
            if pose is not None:
                rep.staging_err = pose_error_in_dock_frame(pose, staging)
                self.log(dock.dock_id,
                         f"na staging: long {rep.staging_err[0] * 100:+.1f} cm, "
                         f"lat {rep.staging_err[1] * 100:+.1f} cm, "
                         f"yaw {math.degrees(rep.staging_err[2]):+.1f} graus")
        else:
            # Rollback do GoToWS (use_opennav_approach=true): /dock_robot navega
            # ate a staging e faz o approach uniciclo.
            goal = DockRobot.Goal()
            goal.use_dock_id = True
            goal.dock_id = dock.dock_id
            goal.navigate_to_staging_pose = True
            goal.max_staging_time = float(args.max_staging_time)
            self.log(dock.dock_id, "dock_robot (navega ate a staging + approach)...")
            out = self.run_action(self.dock_client, goal, "docking", args.max_staging_time + 90.0,
                                  feedback_cb=self._dock_feedback(dock.dock_id))
            rep.t_nav = out.seconds
            retries = getattr(out.result, "num_retries", None)
            self.log(dock.dock_id, f"dock_robot: {out.status} em {out.seconds:.1f} s "
                                   f"(retries={retries}) {out.message}".rstrip())
            if out.status != A_OK:
                self._measure(rep, dock)
                return self._fail(rep, "docking", out)
            rep.message = "dock_robot (approach uniciclo, sem align)"

        # (3) align_to_dock.
        if flow in (FLOW_STAGING_ALIGN, FLOW_OPENNAV_ALIGN):
            goal = AlignToDock.Goal()
            goal.dock_id = dock.dock_id
            goal.map_name = args.map_name
            goal.map_dir = args.map_dir or ""
            goal.xy_tolerance = 0.0   # 0 => default do dock_align_node
            goal.yaw_tolerance = 0.0
            goal.use_lidar_refine = bool(args.use_lidar_refine)
            goal.timeout = float(args.align_timeout)
            self.log(dock.dock_id, f"align_to_dock (use_lidar_refine={goal.use_lidar_refine})...")
            out = self.run_action(self.align_client, goal, "alinhamento", args.align_timeout + 15.0,
                                  feedback_cb=self._align_feedback(dock.dock_id))
            rep.t_align = out.seconds
            rep.align_message = out.message
            if out.result is not None:
                rep.align_x = float(out.result.final_x_error)
                rep.align_y = float(out.result.final_y_error)
                rep.align_yaw = float(out.result.final_yaw_error)
            self.log(dock.dock_id, f"align: {out.status} em {out.seconds:.1f} s - {out.message}")
            if out.status != A_OK:
                self._measure(rep, dock)
                return self._fail(rep, "alinhamento", out)
            rep.message = out.message
            rep.status = align_status(out.message)
        else:
            rep.status = STATUS_OK

        # (4) medida final + voz.
        self._measure(rep, dock)
        if rep.status != STATUS_OK:
            self.get_logger().warn(f"[{dock.dock_id}] chegou com ressalva: {rep.message}")
        self.say(speech_ready(dock.dock_id, rep.err_long, rep.err_lat, rep.err_yaw))
        return rep


# ------------------------------------------------------------------- CLI
def _parse_args(argv):
    p = argparse.ArgumentParser(
        description="Leva o robo por todos os docks do mapa (ordem numerica) e termina "
                    "no FINISH, medindo tempo e erro de pose em cada um.")
    p.add_argument("--map-name", required=True, help="Pasta do mapa (contem docking.yaml).")
    p.add_argument("--map-dir", default=None, help="Raiz alternativa dos mapas (opcional).")
    p.add_argument("--only", default=None, help="So estes docks, ex.: WS1,WS3 (ordem numerica).")
    p.add_argument("--skip", default=None, help="Pula estes docks, ex.: SH1.")
    p.add_argument("--finish", default="FINISH",
                   help='Dock final (default: FINISH; "" nao vai a lugar nenhum no fim).')
    p.add_argument("--no-align", action="store_true",
                   help="Sem align_to_dock: o /dock_robot (opennav) navega ate a staging e faz "
                        "o approach uniciclo, e a pose e medida ali (= use_opennav_approach + "
                        "skip_align do executor).")
    p.add_argument("--opennav-approach", action="store_true",
                   help="Rollback do GoToWS (use_opennav_approach=true): /dock_robot faz "
                        "staging + approach e depois o align_to_dock. Default: Nav2 ate a "
                        "staging e o align faz o approach (fluxo padrao da missao).")
    p.add_argument("--use-lidar-refine", dest="use_lidar_refine", action="store_true", default=True,
                   help="Refino da pose alvo pela face da mesa no /scan (default: ligado, "
                        "como o bt_yaml_executor).")
    p.add_argument("--no-lidar-refine", dest="use_lidar_refine", action="store_false",
                   help="Desliga o refino por LiDAR (pose do banco apenas).")
    p.add_argument("--undock-first", default=None, metavar="DOCK_ID",
                   help="O robo esta encostado neste dock: faz o undock (com o type dele) "
                        "antes do primeiro destino. Default: nenhum (robo no START).")
    p.add_argument("--stop-on-fail", action="store_true",
                   help="Encerra o tour na primeira falha (default: segue para o proximo dock).")
    p.add_argument("--report", default=None,
                   help="Arquivo do relatorio markdown (default: <pasta do mapa>/tour_<data>.md).")
    p.add_argument("--yes", action="store_true", help="Nao pede confirmacao antes de andar.")
    p.add_argument("--no-speech", action="store_true", help="Nao publica em /manip/speech.")
    # Timeouts: mesmos defaults do bt_yaml_executor (manip_bt).
    p.add_argument("--max-staging-time", type=float, default=120.0,
                   help="Tempo maximo do Nav2 ate a staging (s); o /dock_robot recebe o mesmo.")
    p.add_argument("--align-timeout", type=float, default=90.0,
                   help="Timeout do goal do align_to_dock (s).")
    p.add_argument("--max-undocking-time", type=float, default=30.0,
                   help="Tempo maximo do /undock_robot (s).")
    p.add_argument("--nav-timeout", type=float, default=180.0,
                   help="Timeout da navegacao pura (START/FINISH) (s).")
    p.add_argument("--server-wait-timeout", type=float, default=10.0,
                   help="Espera por cada action server antes de cada goal (s).")
    p.add_argument("--preflight-timeout", type=float, default=120.0,
                   help="Segundos maximos aguardando os action servers no pre-flight.")
    p.add_argument("--global-frame", default="map")
    p.add_argument("--base-frame", default="base_footprint")
    return p.parse_args(remove_ros_args(args=argv)[1:])


def _flow_for(dock: DockEntry, args) -> str:
    if not dock.use_docking:
        return FLOW_NAV_ONLY
    if args.no_align:
        return FLOW_OPENNAV_ONLY
    if args.opennav_approach:
        return FLOW_OPENNAV_ALIGN
    return FLOW_STAGING_ALIGN


def _flow_text(args) -> str:
    if args.no_align:
        return "undock -> /dock_robot (staging + approach uniciclo), sem align"
    if args.opennav_approach:
        return (f"undock -> /dock_robot (staging + approach) -> align_to_dock "
                f"(use_lidar_refine={str(args.use_lidar_refine).lower()})")
    return (f"undock -> Nav2 ate a staging -> align_to_dock faz o approach "
            f"(use_lidar_refine={str(args.use_lidar_refine).lower()})")


def _required_actions(args, entries, order) -> list:
    required = ["/navigate_to_pose", "/undock_robot"]
    flows = {_flow_for(entries[d], args) for d in order}
    if flows & {FLOW_OPENNAV_ALIGN, FLOW_OPENNAV_ONLY}:
        required.append("/dock_robot")
    if flows & {FLOW_STAGING_ALIGN, FLOW_OPENNAV_ALIGN}:
        required.append("/align_to_dock")
    return required


def _print_plan(order, entries, args, report_path: Path):
    print(f"\n=== Tour dos docks: {len(order)} destino(s) ===")
    print(f"Fluxo: {_flow_text(args)}")
    for i, dock_id in enumerate(order, start=1):
        dock = entries[dock_id]
        flow = _flow_for(dock, args)
        x, y, yaw = dock.pose
        extra = ""
        if flow != FLOW_NAV_ONLY:
            extra = f"  staging {dock.staging_x_offset:+.2f} m"
            if not dock.staging_from_yaml:
                extra += " (fallback: dock_plugins sem staging_x_offset)"
        if dock.pose_is_placeholder:
            extra += "  POSE PLACEHOLDER [0,0,0] -> sera pulado"
        print(f"  {i:2d}. {dock_id:<7} {dock.dock_type:<30} pose=({x:7.3f}, {y:7.3f}, "
              f"{math.degrees(yaw):+6.1f} graus)  {flow:<13}{extra}")
    print(f"Relatorio: {report_path}\n")


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = _parse_args(argv)
    command = " ".join(["tour_docks"] + remove_ros_args(args=argv)[1:])
    try:
        sys.stdout.reconfigure(line_buffering=True)  # plano/tabela na ordem dos logs
    except (AttributeError, ValueError):
        pass

    try:
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
        docking_path = docking_file(map_folder)
        if not docking_path.exists():
            raise FileNotFoundError(f"{docking_path} nao existe (rode validate_service_areas --sync-docking).")
        entries = load_dock_entries(load_yaml(docking_path), load_yaml(service_areas_file(map_folder)))
        if not entries:
            raise ValueError(f"{docking_path} nao tem docks.")
        order = plan_tour(entries.keys(), parse_dock_list(args.only), parse_dock_list(args.skip),
                          args.finish)
        undock_first = None
        if args.undock_first:
            undock_first = args.undock_first.strip().upper()
            if undock_first not in entries:
                raise ValueError(f"--undock-first: dock '{undock_first}' nao esta no docking.yaml")
    except (FileNotFoundError, ValueError) as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 1
    if not order:
        print("Nada a visitar (confira --only/--skip/--finish).", file=sys.stderr)
        return 1

    started = datetime.datetime.now()
    report_path = Path(args.report).expanduser() if args.report else map_folder / report_filename(started)
    print(f"Mapa: {map_folder}")
    _print_plan(order, entries, args, report_path)

    if not args.yes:
        try:
            answer = input("Executar o tour? O ROBO VAI SE MOVER. [s/N] ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            answer = ""
            print()
        if answer not in ("s", "sim"):
            print("Tour cancelado pelo operador.")
            return 0

    # Ctrl+C e' nosso (nao do rclpy): cancela a action em voo antes de sair.
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.SIGTERM)
    node = TourDocks(args, map_folder)

    def _on_sigint(signum, frame):  # noqa: ARG001
        if node.interrupted:
            raise KeyboardInterrupt  # segundo Ctrl+C: sai de qualquer jeito
        node.interrupt()

    def _preflight_sleep(seconds):
        node.spin_for(seconds)
        if node.interrupted:
            raise KeyboardInterrupt  # Ctrl+C no pre-flight: nada em voo, so sai

    previous = signal.signal(signal.SIGINT, _on_sigint)
    reports = []
    exit_code = 0
    preflight_ok = False
    try:
        print("Verificando os action servers (pre-flight)...")
        node.spin_for(1.0)  # descoberta do grafo
        preflight_ok = wait_for_action_servers(
            _required_actions(args, entries, order), args.preflight_timeout,
            {"/align_to_dock": "align_to_dock (dock_align_node)"},
            hint="O stack esta no ar? -> ros2 launch caramelo_bringup "
                 "robot_manipulation.launch.py map_name:=...",
            list_actions=node.graph_actions,
            sleep=_preflight_sleep)
        if not preflight_ok:
            raise RuntimeError("pre-flight falhou")

        node.say(f"Tour dos docks: {len(order)} destinos")
        # Estado "docked" como no blackboard do executor: o undock so acontece
        # se o destino anterior encostou (dock+align ok); apos navegacao pura
        # ou falha de chegada nao ha undock; falha NO undock mantem o estado.
        docked_type = entries[undock_first].dock_type if undock_first else None
        for dock_id in order:
            if node.interrupted:
                break
            dock = entries[dock_id]
            if dock.pose_is_placeholder:
                rep = DockReport(dock_id, flow=_flow_for(dock, args), status=STATUS_SKIPPED,
                                 message="pose placeholder [0,0,0] - grave a pose real")
                node.get_logger().error(f"[{dock_id}] {rep.message}")
                reports.append(rep)
                continue
            if dock.frame != args.global_frame:
                node.get_logger().warn(
                    f"[{dock_id}] dock no frame '{dock.frame}', tour usa '{args.global_frame}'.")
            flow = _flow_for(dock, args)
            rep = node.visit(dock, docked_type, flow)
            reports.append(rep)
            if rep.failed_stage == "undock":
                pass  # continua encostado: tenta o undock de novo no proximo
            elif rep.reached and flow != FLOW_NAV_ONLY:
                docked_type = dock.dock_type
            else:
                docked_type = None
            if rep.status == STATUS_CANCELED:
                break
            if rep.status == STATUS_FAIL and args.stop_on_fail:
                node.get_logger().error("--stop-on-fail: encerrando o tour.")
                break
    except RuntimeError:
        exit_code = 1
    except KeyboardInterrupt:
        node.get_logger().warn("Tour interrompido (Ctrl+C).")
        node.cancel_active()
        node.interrupted = True
    except ExternalShutdownException:
        node.get_logger().warn("rclpy encerrado externamente.")
    finally:
        signal.signal(signal.SIGINT, previous)
        finished = datetime.datetime.now()
        if not preflight_ok:
            pass
        elif reports:
            print("\n" + format_table(reports) + "\n")
            summary = summarize(reports)
            node.say(speech_summary(summary["n_reached"], summary["n_total"]))
            try:
                report_path.parent.mkdir(parents=True, exist_ok=True)
                report_path.write_text(format_report_md(
                    reports, map_name=args.map_name, map_folder=str(map_folder),
                    started=started, finished=finished, command=command,
                    flow_text=_flow_text(args)), encoding="utf-8")
                print(f"Relatorio: {report_path}")
            except OSError as exc:
                print(f"Nao consegui gravar o relatorio em {report_path}: {exc}", file=sys.stderr)
            if summary["n_reached"] < summary["n_total"]:
                exit_code = 1
        else:
            print("Nenhum dock visitado.")
            exit_code = 1
        if node.interrupted:
            exit_code = 130
        node.spin_for(0.3)  # deixa a ultima fala sair
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
