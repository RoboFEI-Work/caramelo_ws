#!/usr/bin/env python3
# Bancada OFFLINE do controller_server do Nav2 (RotationShim sobre DWB): mede a
# taxa real do laco de controle, os avisos "missed its desired rate" e a CPU do
# processo para varias variantes de controller_server.yaml, com um robo FAKE
# parado (odom, TF e scan sinteticos), sem RViz e sem hardware.
# Quem usa: equipe de navegacao, para achar o que derruba os 20 Hz sem o robo.
# Como roda: export ROS_DOMAIN_ID=99; python3 bench_controller_offline.py [--variant A ...]
#   (sem build; nao altera YAML de src/; copias e resultados em /tmp/bench_controller/).
#
# O que sobe (tudo no namespace /teste e no dominio 99):
#   - no bench_fake_world: TF estatico map->odom e base_footprint->laser_frame,
#     /odom a 50 Hz (+ TF odom->base_footprint, mesmo stamp), /scan a 10 Hz
#     (180 graus frontais; campo a --range m, ou parede a --wall m).
#   - processo controller_server (binario do nav2_controller) com a copia do
#     yaml da variante, cmd_vel remapeado para /teste/cmd_vel_nav; configure e
#     activate via servico change_state (sem lifecycle_manager).
#   - no bench_cmd_listener: so assina /teste/cmd_vel_nav (executor proprio).
#   - no bench_control_<n>: criado por rodada (lifecycle, parametros, action).
# O robo fake nao se move, entao o controller computa controle ate o
# PoseProgressChecker abortar; na copia do yaml movement_time_allowance vira
# 60 s so para a janela de medicao caber.
# Metricas: "cmd_vel Hz" = mensagens recebidas / janela; "laco Hz (log)" =
# estimativa pelo proprio log do controller (cada aviso "missed" traz a duracao
# do ciclo; o tempo restante da janela sao ciclos bons de 50 ms); CPU do
# processo por /proc/<pid>/stat (100% = 1 nucleo), por thread e do sistema.
# Nota sobre o laser: o frame fica em x 0.319 m, z +0.10 m, sem rotacao. O URDF
# real tem z negativo e rpy (pi,0,pi); com z < 0 a ObstacleLayer descartaria
# os pontos (min_obstacle_height 0.0) e o costmap ficaria vazio: o custo do
# raytrace/marcacao e o que importa aqui, nao a geometria exata da montagem.
# Nota sobre a diagonal: 45 graus cai EXATAMENTE no angular_dist_threshold do
# shim (0.785 rad); o shim assumiria e giraria no lugar, e o DWB nao seria
# medido. O default e 40 graus (--diag-deg) para o DWB trabalhar.

import argparse
import datetime as dt
import gc
import hashlib
import json
import math
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("ROS_DOMAIN_ID", "99")
if os.environ.get("ROS_DOMAIN_ID") != "99":
    print(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}: a bancada exige 99 "
          "(nunca rodar no dominio do robo).", file=sys.stderr)
    sys.exit(2)

import rclpy  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_prefix  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist  # noqa: E402
from lifecycle_msgs.msg import State, Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState, GetState  # noqa: E402
from nav2_msgs.action import FollowPath  # noqa: E402
from nav_msgs.msg import Odometry, Path as PathMsg  # noqa: E402
from rcl_interfaces.srv import GetParameters  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import parameter_value_to_python  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402

NS = "/teste"
CTRL_NODE = f"{NS}/controller_server"
COSTMAP_NODE = f"{NS}/local_costmap/local_costmap"
CMD_VEL_TOPIC = f"{NS}/cmd_vel_nav"
ACTION_NAME = f"{NS}/follow_path"
CLK_TCK = os.sysconf("SC_CLK_TCK")
LOG_RE = re.compile(r"^\[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$")
MISSED_RE = re.compile(r"missed its desired rate of ([0-9.]+) Hz\. Current loop rate is ([0-9.]+) Hz")

# Nomes dos publish_* conferidos em libdwb_core.so (Jazzy, nav2 1.3.13) e na doc
# configuring-dwb-controller: publish_evaluation, publish_global_plan,
# publish_transformed_plan, publish_local_plan, publish_trajectories,
# publish_cost_grid_pc (+ marker_lifetime).
DWB_PUBLISH_OFF = [
    ("DWBHolonomic.publish_evaluation", False),
    ("DWBHolonomic.publish_global_plan", False),
    ("DWBHolonomic.publish_transformed_plan", False),
    ("DWBHolonomic.publish_local_plan", False),
    ("DWBHolonomic.publish_trajectories", False),
    ("DWBHolonomic.publish_cost_grid_pc", False),
]
CRITICS_BASE_OBSTACLE = ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign",
                         "PathAlign", "PathDist", "GoalDist", "Twirling"]

# Catalogo: nome -> (descricao, lista de (caminho, valor)); valor None apaga a
# chave. Caminhos com prefixo "local_costmap." vao para o no do costmap local.
VARIANTS = {
    "A": ("atual (producao)", []),
    "B": ("vx 8 / vy 8 / vtheta 20 (pre-07/11)", [
        ("DWBHolonomic.vx_samples", 8), ("DWBHolonomic.vy_samples", 8),
        ("DWBHolonomic.vtheta_samples", 20)]),
    "C": ("atual + publish_* do DWB = false", list(DWB_PUBLISH_OFF)),
    "D": ("atual + debug_trajectory_details false", [
        ("DWBHolonomic.debug_trajectory_details", False)]),
    "E": ("atual, so DWBHolonomic carregado (sem RPP/MPPI)", [
        ("controller_plugins", ["DWBHolonomic"]), ("FollowPath", None), ("MPPIHolonomic", None)]),
    "F": ("atual + local_costmap 3x3 m", [("local_costmap.width", 3), ("local_costmap.height", 3)]),
    "G": ("atual + sim_time 1.2", [("DWBHolonomic.sim_time", 1.2)]),
    "H": ("vx 6 / vy 6 / vtheta 15", [
        ("DWBHolonomic.vx_samples", 6), ("DWBHolonomic.vy_samples", 6),
        ("DWBHolonomic.vtheta_samples", 15)]),
    "I": ("BaseObstacle no lugar de ObstacleFootprint (so medida)", [
        ("DWBHolonomic.critics", CRITICS_BASE_OBSTACLE),
        ("DWBHolonomic.ObstacleFootprint.scale", None),
        ("DWBHolonomic.ObstacleFootprint.sum_scores", None),
        ("DWBHolonomic.BaseObstacle.scale", 1.0),
        ("DWBHolonomic.BaseObstacle.sum_scores", False)]),
    "CD": ("extra: C + D (sem publicacao e sem debug)", list(DWB_PUBLISH_OFF) + [
        ("DWBHolonomic.debug_trajectory_details", False)]),
    "CDE": ("extra: C + D + E", list(DWB_PUBLISH_OFF) + [
        ("DWBHolonomic.debug_trajectory_details", False),
        ("controller_plugins", ["DWBHolonomic"]), ("FollowPath", None), ("MPPIHolonomic", None)]),
    "LG": ("extra (fora do default): linear_granularity 0.10", [
        ("DWBHolonomic.linear_granularity", 0.10)]),
    "A2": ("repeticao de A no fim (variacao)", []),
}
DEFAULT_ORDER = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "CD", "CDE", "A2"]
SCENARIOS = {
    "reta": "caminho reto de 4 m em +x, heading 0",
    "diagonal": "caminho de 4 m a --diag-deg graus, heading 0 nos poses (lattice omni)",
}


# --------------------------------------------------------------------------
# Utilidades de yaml / parametros
# --------------------------------------------------------------------------
def set_path(d, path, value):
    # Caminha pelo dicionario aceitando chaves literais com ponto (ex.:
    # "ObstacleFootprint.scale" dentro de DWBHolonomic). value None apaga.
    parts = path.split(".")
    node = d
    while parts:
        hit = None
        for n in range(len(parts), 0, -1):
            key = ".".join(parts[:n])
            if isinstance(node, dict) and key in node:
                hit = (key, n)
                break
        if hit is None:
            if value is None:
                return False
            if len(parts) == 1:
                node[parts[0]] = value
                return True
            node[parts[0]] = {}
            node = node[parts[0]]
            parts = parts[1:]
            continue
        key, n = hit
        if n == len(parts):
            if value is None:
                del node[key]
            else:
                node[key] = value
            return True
        node = node[key]
        parts = parts[n:]
    return False


def build_variant_yaml(src_yaml, out_path, changes, movement_time_allowance):
    with open(src_yaml, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    ctrl = data["controller_server"]["ros__parameters"]
    costmap = data["local_costmap"]["local_costmap"]["ros__parameters"]
    set_path(ctrl, "progress_checker.movement_time_allowance", float(movement_time_allowance))
    applied = []
    for path, value in changes:
        if path.startswith("local_costmap."):
            ok = set_path(costmap, path[len("local_costmap."):], value)
        else:
            ok = set_path(ctrl, path, value)
        applied.append({"param": path, "valor": value, "aplicado_no_yaml": ok})
    # Mesma arvore, aninhada no namespace de teste: /teste/controller_server e
    # /teste/local_costmap/local_costmap.
    out = {NS.strip("/"): data}
    with open(out_path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(out, fh, default_flow_style=False, sort_keys=False)
    return applied


def parse_custom_variant(text):
    # Formato: nome:caminho=valor;caminho=valor  (valor em sintaxe YAML).
    name, _, body = text.partition(":")
    changes = []
    for item in filter(None, body.split(";")):
        key, _, val = item.partition("=")
        changes.append((key.strip(), yaml.safe_load(val) if val.strip() else None))
    return name.strip(), changes


# --------------------------------------------------------------------------
# Leitura de CPU via /proc
# --------------------------------------------------------------------------
def read_proc_ticks(pid):
    try:
        with open(f"/proc/{pid}/stat") as fh:
            rest = fh.read().rsplit(")", 1)[1].split()
        return int(rest[11]) + int(rest[12])
    except (OSError, IndexError, ValueError):
        return None


def read_thread_ticks(pid):
    out = {}
    try:
        for tid in os.listdir(f"/proc/{pid}/task"):
            try:
                with open(f"/proc/{pid}/task/{tid}/stat") as fh:
                    rest = fh.read().rsplit(")", 1)[1].split()
                with open(f"/proc/{pid}/task/{tid}/comm") as fh:
                    comm = fh.read().strip()
                out[tid] = (comm, int(rest[11]) + int(rest[12]))
            except (OSError, IndexError, ValueError):
                continue
    except OSError:
        pass
    return out


def read_sys_cpu():
    with open("/proc/stat") as fh:
        f = fh.readline().split()
    vals = [int(x) for x in f[1:]]
    idle = vals[3] + vals[4]
    return sum(vals), idle


def read_rss_mb(pid):
    try:
        with open(f"/proc/{pid}/status") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return None


def own_cpu_seconds():
    t = os.times()
    return t.user + t.system


def cpu_mhz():
    # Clock medio e maximo dos nucleos agora (powersave baixa o clock por nucleo
    # quando o resto da maquina esta ocupada: o ciclo do DWB estica sem a CPU%
    # do processo mudar).
    vals = []
    try:
        with open("/proc/cpuinfo") as fh:
            for line in fh:
                if line.startswith("cpu MHz"):
                    vals.append(float(line.split(":")[1]))
    except (OSError, ValueError):
        pass
    if not vals:
        return None
    return {"medio": round(sum(vals) / len(vals)), "max": round(max(vals))}


def percentile(values, pct):
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * pct / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return s[int(k)]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


# --------------------------------------------------------------------------
# Nos ROS da bancada
# --------------------------------------------------------------------------
class FakeWorld(Node):
    # Robo parado na origem: odom + TF a 50 Hz, scan a 10 Hz, TFs estaticos.
    def __init__(self, scan_beams, field_range, wall, scan_hz=10.0, odom_hz=50.0):
        super().__init__("bench_fake_world", namespace=NS)
        self._tf = TransformBroadcaster(self)
        self._tf_static = StaticTransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._scan_pub = self.create_publisher(
            LaserScan, "/scan", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        now = self.get_clock().now().to_msg()
        statics = []
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.rotation.w = 1.0
        statics.append(t)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = "base_footprint"
        t2.child_frame_id = "laser_frame"
        t2.transform.translation.x = 0.319
        t2.transform.translation.z = 0.10
        t2.transform.rotation.w = 1.0
        statics.append(t2)
        self._tf_static.sendTransform(statics)

        self._scan = LaserScan()
        self._scan.header.frame_id = "laser_frame"
        self._scan.angle_min = -math.pi / 2.0
        self._scan.angle_max = math.pi / 2.0
        self._scan.angle_increment = math.pi / float(scan_beams - 1)
        self._scan.time_increment = 0.0
        self._scan.scan_time = 1.0 / scan_hz
        self._scan.range_min = 0.05
        self._scan.range_max = 30.0
        ranges = []
        for i in range(scan_beams):
            ang = self._scan.angle_min + i * self._scan.angle_increment
            r = field_range
            if wall is not None and math.cos(ang) > 0.05:
                # Parede plana em x = wall (frame do robo); o laser esta em x 0.319.
                r = min(r, (wall - 0.319) / math.cos(ang))
            ranges.append(float(r))
        self._scan.ranges = ranges
        self.create_timer(1.0 / odom_hz, self._odom_cb)
        self.create_timer(1.0 / scan_hz, self._scan_cb)

    def _odom_cb(self):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.rotation.w = 1.0
        self._tf.sendTransform(t)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)

    def _scan_cb(self):
        self._scan.header.stamp = self.get_clock().now().to_msg()
        self._scan_pub.publish(self._scan)


class CmdListener(Node):
    # So assina cmd_vel do controller de teste; fila funda para nao perder
    # mensagem se o Python atrasar a entrega (a contagem N/T fica robusta).
    def __init__(self):
        super().__init__("bench_cmd_listener", namespace=NS)
        self._lock = threading.Lock()
        self.cmd_times = []
        self.create_subscription(
            Twist, CMD_VEL_TOPIC, self._cmd_cb,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))

    def _cmd_cb(self, _msg):
        with self._lock:
            self.cmd_times.append(time.time())

    def reset(self):
        with self._lock:
            self.cmd_times = []

    def snapshot(self):
        with self._lock:
            return list(self.cmd_times)


class RunControl(Node):
    # Criado NOVO a cada rodada: cliente reaproveitado entre reinicios do
    # controller perdia a resposta ("failed to send response ... client will
    # not receive").
    def __init__(self, idx):
        super().__init__(f"bench_control_{idx}", namespace=NS)
        self.change_state = self.create_client(ChangeState, f"{CTRL_NODE}/change_state")
        self.get_state = self.create_client(GetState, f"{CTRL_NODE}/get_state")
        self.get_params_ctrl = self.create_client(GetParameters, f"{CTRL_NODE}/get_parameters")
        self.get_params_costmap = self.create_client(GetParameters, f"{COSTMAP_NODE}/get_parameters")
        self.follow_path = ActionClient(self, FollowPath, ACTION_NAME)

    def close(self):
        # ActionClient e clientes precisam ser destruidos ANTES do no, senao o
        # no continua anunciado no grafo ate o garbage collector passar.
        self.follow_path.destroy()
        for cl in (self.change_state, self.get_state, self.get_params_ctrl, self.get_params_costmap):
            self.destroy_client(cl)


class SpinThread:
    # Executor single-thread proprio por no (o MultiThreadedExecutor do rclpy
    # atrasava a entrega em segundos e gastava CPU).
    def __init__(self, node):
        self.node = node
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=3.0)
        if hasattr(self.node, "close"):
            self.node.close()
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        gc.collect()


def wait_future(fut, timeout, alive=None):
    t_end = time.time() + timeout
    while time.time() < t_end:
        if fut.done():
            return fut.result()
        if alive is not None and not alive():
            raise RuntimeError("processo do controller_server morreu durante a espera")
        time.sleep(0.02)
    raise TimeoutError("timeout esperando resposta ROS")


def wait_service(client, timeout, alive):
    t_end = time.time() + timeout
    while time.time() < t_end:
        if client.wait_for_service(timeout_sec=0.5):
            return True
        if not alive():
            raise RuntimeError("processo do controller_server morreu antes de expor servicos")
    raise TimeoutError(f"servico {client.srv_name} nao apareceu em {timeout} s")


def current_state(ctl, alive, timeout=5.0):
    res = wait_future(ctl.get_state.call_async(GetState.Request()), timeout, alive)
    return res.current_state.id


def lifecycle_transition(ctl, transition_id, target_state, alive, tries=3, timeout=15.0):
    # Reenvia se a resposta se perder; aceita se o estado alvo ja foi atingido.
    last = None
    for _ in range(tries):
        req = ChangeState.Request()
        req.transition.id = transition_id
        try:
            res = wait_future(ctl.change_state.call_async(req), timeout, alive)
            if res.success:
                return
            last = f"transicao {transition_id} recusada"
        except TimeoutError as exc:
            last = str(exc)
        try:
            if current_state(ctl, alive) == target_state:
                return
        except TimeoutError:
            pass
        time.sleep(0.5)
    raise RuntimeError(f"lifecycle: {last}")


def get_params(client, names, alive, tries=3, timeout=8.0):
    req = GetParameters.Request()
    req.names = list(names)
    last = None
    for _ in range(tries):
        try:
            res = wait_future(client.call_async(req), timeout, alive)
            break
        except TimeoutError as exc:
            last = exc
    else:
        raise last
    out = {}
    for name, val in zip(names, res.values):
        out[name] = "NAO_DECLARADO" if val.type == 0 else parameter_value_to_python(val)
    return out


def make_path(node, scenario, diag_deg, length=4.0, step=0.05):
    path = PathMsg()
    path.header.frame_id = "map"
    path.header.stamp = node.get_clock().now().to_msg()
    ang = 0.0 if scenario == "reta" else math.radians(diag_deg)
    n = int(round(length / step))
    for i in range(n + 1):
        d = i * step
        p = PoseStamped()
        p.header = path.header
        p.pose.position.x = d * math.cos(ang)
        p.pose.position.y = d * math.sin(ang)
        p.pose.orientation.w = 1.0  # heading 0 em todos os poses
        path.poses.append(p)
    return path


# --------------------------------------------------------------------------
# Processo do controller_server
# --------------------------------------------------------------------------
CHILD_PROCS = []  # todos os processos filhos criados; conferidos/encerrados no fim


class ControllerProcess:
    def __init__(self, exe, yaml_path, log_path, extra_env):
        self.lines = []  # (t_recebido, texto)
        self._lock = threading.Lock()
        self.log_fh = open(log_path, "w", encoding="utf-8")
        env = dict(os.environ)
        env.update(extra_env)
        env["ROS_DOMAIN_ID"] = "99"
        env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"
        env["RCUTILS_COLORIZED_OUTPUT"] = "0"
        cmd = [exe, "--ros-args", "-r", f"__ns:={NS}", "--params-file", yaml_path,
               "-r", f"cmd_vel:={CMD_VEL_TOPIC}", "--disable-external-lib-logs"]
        self.proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True, bufsize=1)
        self.pid = self.proc.pid
        CHILD_PROCS.append(self)
        self._threads = [threading.Thread(target=self._reader, args=(self.proc.stderr,), daemon=True),
                         threading.Thread(target=self._reader, args=(self.proc.stdout,), daemon=True)]
        for th in self._threads:
            th.start()

    def _reader(self, stream):
        for line in stream:
            line = line.rstrip("\n")
            with self._lock:
                self.lines.append((time.time(), line))
                self.log_fh.write(line + "\n")

    def alive(self):
        return self.proc.poll() is None

    def snapshot(self):
        with self._lock:
            return list(self.lines)

    def stop(self):
        # Encerra SO este PID: SIGINT -> SIGTERM -> SIGKILL, nunca pkill.
        if self.proc.poll() is None:
            for sig, wait_s in ((signal.SIGINT, 6.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
                try:
                    self.proc.send_signal(sig)
                except ProcessLookupError:
                    break
                try:
                    self.proc.wait(timeout=wait_s)
                    break
                except subprocess.TimeoutExpired:
                    continue
        for th in self._threads:
            th.join(timeout=2.0)
        try:
            self.log_fh.flush()
            self.log_fh.close()
        except OSError:
            pass
        return self.proc.returncode


# --------------------------------------------------------------------------
# Uma rodada (variante x cenario)
# --------------------------------------------------------------------------
PARAMS_CHECK_CTRL = [
    "controller_plugins", "controller_frequency", "progress_checker.movement_time_allowance",
    "DWBHolonomic.primary_controller", "DWBHolonomic.vx_samples", "DWBHolonomic.vy_samples",
    "DWBHolonomic.vtheta_samples", "DWBHolonomic.sim_time", "DWBHolonomic.linear_granularity",
    "DWBHolonomic.critics", "DWBHolonomic.debug_trajectory_details",
    "DWBHolonomic.publish_evaluation", "DWBHolonomic.publish_global_plan",
    "DWBHolonomic.publish_transformed_plan", "DWBHolonomic.publish_local_plan",
    "DWBHolonomic.publish_trajectories", "DWBHolonomic.publish_cost_grid_pc",
    "DWBHolonomic.short_circuit_trajectory_evaluation",
]
PARAMS_CHECK_COSTMAP = ["width", "height", "resolution", "update_frequency", "publish_frequency",
                        "always_send_full_costmap", "footprint"]
RUN_COUNTER = [0]


def loop_stats_from_log(lines, t0, t1, period=0.05):
    # Estima o laco pelo log: cada aviso "missed" da a duracao daquele ciclo;
    # o resto da janela foram ciclos bons de `period`.
    rates = []
    outros = {}
    for t_rx, line in lines:
        m = LOG_RE.match(line)
        t_log = float(m.group(2)) if m else t_rx
        if not (t0 <= t_log <= t1):
            continue
        mm = MISSED_RE.search(line)
        if mm:
            rates.append(float(mm.group(2)))
        elif m and m.group(1) in ("WARN", "ERROR", "FATAL"):
            key = re.sub(r"[0-9.]+", "#", m.group(4))[:90]
            outros[key] = outros.get(key, 0) + 1
    janela = t1 - t0
    t_missed = sum(1.0 / r for r in rates if r > 0)
    n_ok = max(0.0, janela - t_missed) / period
    total = len(rates) + n_ok
    out = {
        "missed_avisos": len(rates),
        "missed_por_10s": round(len(rates) / janela * 10.0, 1) if janela > 0 else None,
        "loop_log_hz": round(total / janela, 2) if janela > 0 else None,
        "ciclo_medio_ms": round(janela / total * 1e3, 1) if total > 0 else None,
        "frac_ciclos_perdidos": round(len(rates) / total, 3) if total > 0 else None,
        "outros_avisos_na_janela": outros,
    }
    if rates:
        out["missed_loop_hz"] = {"min": round(min(rates), 2), "p50": round(statistics.median(rates), 2),
                                 "max": round(max(rates), 2)}
    return out


def run_once(args, listener, exe, variant, yaml_path, scenario, log_dir):
    tag = f"{variant}_{scenario}"
    log_path = log_dir / f"{tag}.log"
    result = {"variante": variant, "cenario": scenario, "log": str(log_path), "erro": None}
    print(f"\n=== {tag}: iniciando controller_server ({yaml_path.name}) ===", flush=True)
    RUN_COUNTER[0] += 1
    ctl = RunControl(RUN_COUNTER[0])
    ctl_spin = SpinThread(ctl)
    listener.reset()
    proc = ControllerProcess(exe, str(yaml_path), str(log_path), {})
    result["pid"] = proc.pid
    goal_handle = None
    result_fut = None
    try:
        wait_service(ctl.change_state, 30.0, proc.alive)
        wait_service(ctl.get_state, 10.0, proc.alive)
        time.sleep(0.5)  # casamento completo cliente/servidor (resposta nao se perde)
        t_cfg = time.time()
        lifecycle_transition(ctl, Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE, proc.alive)
        result["t_configure_s"] = round(time.time() - t_cfg, 2)
        lifecycle_transition(ctl, Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE, proc.alive)
        if current_state(ctl, proc.alive) != State.PRIMARY_STATE_ACTIVE:
            raise RuntimeError("controller nao ficou ativo")
        wait_service(ctl.get_params_costmap, 10.0, proc.alive)
        result["params_controller"] = get_params(ctl.get_params_ctrl, PARAMS_CHECK_CTRL, proc.alive)
        result["params_costmap"] = get_params(ctl.get_params_costmap, PARAMS_CHECK_COSTMAP, proc.alive)
        nao_decl = [k for k, v in result["params_controller"].items() if v == "NAO_DECLARADO"]
        if nao_decl:
            result["params_nao_declarados"] = nao_decl
        time.sleep(1.0)  # costmap recebe os primeiros scans

        if not ctl.follow_path.wait_for_server(timeout_sec=15.0):
            raise TimeoutError("action follow_path nao apareceu")
        goal = FollowPath.Goal()
        goal.path = make_path(ctl, scenario, args.diag_deg)
        goal.controller_id = "DWBHolonomic"
        goal.goal_checker_id = "general_goal_checker"
        listener.reset()
        for tentativa in range(3):
            try:
                goal_handle = wait_future(ctl.follow_path.send_goal_async(goal), 10.0, proc.alive)
                break
            except TimeoutError:
                if tentativa == 2:
                    raise
        if not goal_handle.accepted:
            raise RuntimeError("goal FollowPath rejeitado")
        result_fut = goal_handle.get_result_async()

        # Janela comeca no 1o cmd_vel publicado pelo controller.
        t_end = time.time() + 20.0
        while not listener.snapshot():
            if time.time() > t_end:
                raise TimeoutError("controller nao publicou cmd_vel em 20 s")
            if result_fut.done():
                r = result_fut.result()
                raise RuntimeError(f"goal terminou antes do 1o cmd_vel: status {r.status} "
                                   f"error_code {r.result.error_code} {r.result.error_msg}")
            if not proc.alive():
                raise RuntimeError("controller_server morreu antes do 1o cmd_vel")
            time.sleep(0.02)
        t0 = time.time()
        ticks0 = read_proc_ticks(proc.pid)
        thr0 = read_thread_ticks(proc.pid)
        sys0 = read_sys_cpu()
        own0 = own_cpu_seconds()
        mhz_samples = [cpu_mhz()]
        early = None
        next_mhz = t0 + 5.0
        while time.time() - t0 < args.duration:
            if time.time() >= next_mhz:
                mhz_samples.append(cpu_mhz())
                next_mhz += 5.0
            if result_fut.done():
                r = result_fut.result()
                early = {"status": int(r.status), "error_code": int(r.result.error_code),
                         "error_msg": str(r.result.error_msg)}
                break
            if not proc.alive():
                early = {"status": -1, "error_code": -1, "error_msg": "processo morreu"}
                break
            time.sleep(0.05)
        t1 = time.time()
        ticks1 = read_proc_ticks(proc.pid)
        thr1 = read_thread_ticks(proc.pid)
        sys1 = read_sys_cpu()
        own1 = own_cpu_seconds()
        janela = t1 - t0
        result["t0"] = t0
        result["t1"] = t1
        result["janela_s"] = round(janela, 2)
        result["goal_terminou_cedo"] = early
        result["rss_mb"] = read_rss_mb(proc.pid)

        # Taxa de cmd_vel na janela (N/T; intervalos so informativos: a entrega
        # no rclpy vem em rajadas).
        times = [t for t in listener.snapshot() if t0 <= t <= t1]
        intervals = [b - a for a, b in zip(times, times[1:])]
        result["cmd_vel_msgs"] = len(times)
        result["cmd_vel_hz"] = round(len(times) / janela, 2) if janela > 0 else None
        if intervals:
            result["intervalo_entrega_ms"] = {"p10": round(percentile(intervals, 10) * 1e3, 1),
                                              "p50": round(percentile(intervals, 50) * 1e3, 1),
                                              "p90": round(percentile(intervals, 90) * 1e3, 1),
                                              "max": round(max(intervals) * 1e3, 1)}
        result.update(loop_stats_from_log(proc.snapshot(), t0, t1))
        # CPU do processo, por thread, do sistema e da propria bancada.
        if ticks0 is not None and ticks1 is not None and janela > 0:
            result["cpu_proc_pct"] = round((ticks1 - ticks0) / CLK_TCK / janela * 100.0, 1)
        threads = []
        for tid, (comm, tk1) in thr1.items():
            tk0 = thr0.get(tid, (comm, tk1))[1]
            pct = (tk1 - tk0) / CLK_TCK / janela * 100.0 if janela > 0 else 0.0
            threads.append({"tid": tid, "comm": comm, "pct": round(pct, 1)})
        threads.sort(key=lambda x: -x["pct"])
        result["cpu_threads_top"] = threads[:6]
        result["cpu_threads_total"] = len(thr1)
        dtot = sys1[0] - sys0[0]
        didle = sys1[1] - sys0[1]
        result["cpu_sistema_pct"] = round((dtot - didle) / dtot * 100.0, 1) if dtot > 0 else None
        result["cpu_bancada_pct"] = round((own1 - own0) / janela * 100.0, 1) if janela > 0 else None
        mhz_samples = [m for m in mhz_samples if m]
        if mhz_samples:
            result["cpu_mhz_medio"] = round(sum(m["medio"] for m in mhz_samples) / len(mhz_samples))
            result["cpu_mhz_max"] = max(m["max"] for m in mhz_samples)
        print(f"    janela {janela:.1f} s: cmd_vel {result['cmd_vel_hz']} Hz, laco(log) "
              f"{result['loop_log_hz']} Hz, missed/10s {result['missed_por_10s']}, CPU proc "
              f"{result.get('cpu_proc_pct')}%, sistema {result['cpu_sistema_pct']}%, bancada "
              f"{result['cpu_bancada_pct']}%, clock medio {result.get('cpu_mhz_medio')} MHz", flush=True)
    except Exception as exc:  # noqa: BLE001 - registra e segue para a proxima variante
        result["erro"] = f"{type(exc).__name__}: {exc}"
        tail = [ln for _, ln in proc.snapshot()[-15:]]
        result["log_tail"] = tail
        print(f"    ERRO em {tag}: {result['erro']}", flush=True)
        for ln in tail[-5:]:
            print(f"      | {ln[:160]}", flush=True)
    finally:
        # Cancela o goal (se vivo), encerra o processo pelo PID e o no cliente.
        if goal_handle is not None and result_fut is not None and not result_fut.done():
            try:
                wait_future(goal_handle.cancel_goal_async(), 3.0)
                wait_future(result_fut, 3.0)
            except Exception:  # noqa: BLE001
                pass
        rc = proc.stop()
        result["returncode"] = rc
        ctl_spin.stop()
        time.sleep(1.0)  # descoberta DDS assenta antes do proximo processo
    return result


# --------------------------------------------------------------------------
# Relatorio
# --------------------------------------------------------------------------
def fmt(v, nd=1):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def build_table(results, descriptions):
    hdr = ("| Variante | Cenário | cmd_vel Hz (N/T) | laço Hz (log) | ciclo médio ms | "
           "avisos missed /10 s | loop Hz nos avisos (mín/p50) | CPU proc % | CPU sistema % | clock MHz | Obs. |")
    sep = "|---|---|---|---|---|---|---|---|---|---|---|"
    rows = [hdr, sep]
    for r in results:
        obs = []
        if r.get("erro"):
            obs.append("ERRO: " + r["erro"][:60])
        if r.get("goal_terminou_cedo"):
            e = r["goal_terminou_cedo"]
            obs.append(f"goal terminou em {fmt(r.get('janela_s'))} s (code {e['error_code']})")
        if r.get("params_nao_declarados"):
            obs.append("nao declarado: " + ",".join(r["params_nao_declarados"]))
        for k, n in (r.get("outros_avisos_na_janela") or {}).items():
            obs.append(f"{n}x {k[:40]}")
        ml = r.get("missed_loop_hz") or {}
        rows.append("| {v} ({d}) | {c} | {hz} | {ll} | {cm} | {m10} | {mmin}/{mp50} | {cpu} | {sys} | {mhz} | {obs} |".format(
            v=r["variante"], d=descriptions.get(r["variante"], ""), c=r["cenario"],
            hz=fmt(r.get("cmd_vel_hz"), 2), ll=fmt(r.get("loop_log_hz"), 1), cm=fmt(r.get("ciclo_medio_ms")),
            m10=fmt(r.get("missed_por_10s")), mmin=fmt(ml.get("min")), mp50=fmt(ml.get("p50")),
            cpu=fmt(r.get("cpu_proc_pct")), sys=fmt(r.get("cpu_sistema_pct")), mhz=fmt(r.get("cpu_mhz_medio")),
            obs="; ".join(obs) or "-"))
    return "\n".join(rows)


def nav2_version():
    try:
        pkg = Path(get_package_prefix("nav2_controller")) / "share" / "nav2_controller" / "package.xml"
        m = re.search(r"<version>([^<]+)</version>", pkg.read_text())
        return m.group(1) if m else None
    except Exception:  # noqa: BLE001
        return None


def ros2_node_list():
    ros2 = shutil.which("ros2")
    if not ros2:
        return None
    env = dict(os.environ, ROS_DOMAIN_ID="99")
    try:
        out = subprocess.run([ros2, "node", "list", "--no-daemon", "--spin-time", "2"],
                             env=env, capture_output=True, text=True, timeout=30)
        return [ln for ln in out.stdout.splitlines() if ln.strip()]
    except (subprocess.TimeoutExpired, OSError):
        return None


def _read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def main():
    here = Path(__file__).resolve().parent
    default_yaml = here.parent / "config" / "controller_server.yaml"
    if not default_yaml.exists():
        default_yaml = Path.home() / "caramelo_ws/src/caramelo_navigation/config/controller_server.yaml"
    ap = argparse.ArgumentParser(description="Bancada offline do controller_server (DWB) no dominio 99.")
    ap.add_argument("--variant", action="append", default=None,
                    help="Nome do catalogo (A..I, CD, CDE, LG, A2, all) ou custom 'nome:caminho=valor;...'. Repetivel.")
    ap.add_argument("--scenario", choices=["reta", "diagonal", "both"], default="both")
    ap.add_argument("--duration", type=float, default=20.0, help="Janela de medicao em s apos o 1o cmd_vel (>= 20 recomendado).")
    ap.add_argument("--yaml", default=str(default_yaml), help="controller_server.yaml de origem (so leitura).")
    ap.add_argument("--out-dir", default="/tmp/bench_controller")
    ap.add_argument("--range", type=float, default=2.0, help="Alcance do scan em todo o campo (m); inf = livre.")
    ap.add_argument("--wall", type=float, default=None, help="Parede plana a esta distancia a frente (m).")
    ap.add_argument("--scan-beams", type=int, default=1441,
                    help="Feixes nos 180 graus (RPLidar S2 DenseBoost + angle_compensate ~ 2880/volta).")
    ap.add_argument("--diag-deg", type=float, default=40.0, help="Angulo da diagonal (45 engaja o shim).")
    ap.add_argument("--movement-time-allowance", type=float, default=60.0)
    ap.add_argument("--list-variants", action="store_true")
    args = ap.parse_args()

    if args.list_variants:
        for k, (desc, changes) in VARIANTS.items():
            print(f"{k:4s} {desc}: {changes}")
        return 0

    catalog = dict(VARIANTS)
    order = []
    for item in (args.variant or ["all"]):
        if item.lower() == "all":
            order.extend(DEFAULT_ORDER)
        elif ":" in item:
            name, changes = parse_custom_variant(item)
            catalog[name] = (f"custom {changes}", changes)
            order.append(name)
        elif item.upper() in catalog:
            order.append(item.upper())
        else:
            print(f"variante desconhecida: {item}", file=sys.stderr)
            return 2
    scenarios = ["reta", "diagonal"] if args.scenario == "both" else [args.scenario]
    src_yaml = Path(args.yaml)
    if not src_yaml.exists():
        print(f"yaml de origem nao existe: {src_yaml}", file=sys.stderr)
        return 2

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir)
    var_dir = out_dir / "variants" / stamp
    log_dir = out_dir / "logs" / stamp
    var_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    exe = str(Path(get_package_prefix("nav2_controller")) / "lib" / "nav2_controller" / "controller_server")
    if not os.access(exe, os.X_OK):
        print(f"binario nao encontrado: {exe}", file=sys.stderr)
        return 2

    meta = {
        "timestamp": stamp,
        "yaml_origem": str(src_yaml),
        "yaml_sha256": hashlib.sha256(src_yaml.read_bytes()).hexdigest()[:16],
        "nav2_controller_version": nav2_version(),
        "rmw": os.environ.get("RMW_IMPLEMENTATION", "(default rmw_fastrtps_cpp)"),
        "nproc": os.cpu_count(),
        "cpu_modelo": next((ln.split(":", 1)[1].strip() for ln in (_read("/proc/cpuinfo") or "").splitlines()
                            if ln.startswith("model name")), None),
        "governor": _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
        "loadavg_inicio": _read("/proc/loadavg"),
        "args": vars(args),
        "cenarios": SCENARIOS,
        "nota_laser": "frame laser_frame em x 0.319 z +0.10 sem rotacao (ver cabecalho)",
        "nota_metricas": "laço Hz (log) = (ciclos perdidos + tempo restante/50 ms)/janela, pelo log do controller; "
                         "cmd_vel Hz = mensagens/janela; CPU proc 100% = 1 nucleo.",
    }
    print(json.dumps(meta, indent=1, ensure_ascii=False), flush=True)

    rclpy.init(args=None)
    world = FakeWorld(args.scan_beams, args.range, args.wall)
    listener = CmdListener()
    spins = [SpinThread(world), SpinThread(listener)]
    results = []
    descriptions = {k: v[0] for k, v in catalog.items()}
    json_path = out_dir / f"{stamp}.json"
    try:
        time.sleep(1.0)  # TFs estaticos e primeiros scans no ar
        for variant in order:
            desc, changes = catalog[variant]
            yaml_path = var_dir / f"{variant}.yaml"
            applied = build_variant_yaml(src_yaml, yaml_path, changes, args.movement_time_allowance)
            for scenario in scenarios:
                r = run_once(args, listener, exe, variant, yaml_path, scenario, log_dir)
                r["descricao"] = desc
                r["mudancas"] = applied
                r["yaml"] = str(yaml_path)
                results.append(r)
                with open(json_path, "w", encoding="utf-8") as fh:
                    json.dump({"meta": meta, "resultados": results}, fh, indent=1, ensure_ascii=False)
    except KeyboardInterrupt:
        print("\nInterrompido pelo operador; encerrando processos pelos PIDs.", flush=True)
    finally:
        # Garante que nenhum filho sobrou (so pelos PIDs registrados).
        for cp in CHILD_PROCS:
            if cp.alive():
                print(f"encerrando controller_server remanescente pid {cp.pid}", flush=True)
                cp.stop()
        for sp in spins:
            sp.stop()
        rclpy.shutdown()
        gc.collect()

    table = build_table(results, descriptions)
    md_path = out_dir / f"{stamp}.md"
    md_path.write_text(table + "\n", encoding="utf-8")
    print("\n" + table)
    vivos = []
    for cp in CHILD_PROCS:
        try:
            os.kill(cp.pid, 0)
            vivos.append(cp.pid)
        except ProcessLookupError:
            pass
        except PermissionError:
            vivos.append(cp.pid)
    # A descoberta DDS demora a esquecer os participantes: tenta por ate ~20 s.
    nodes = None
    for _ in range(4):
        time.sleep(2.0)
        nodes = ros2_node_list()
        if not nodes:
            break
    meta["ros2_node_list_final"] = nodes
    meta["pids_filhos"] = [cp.pid for cp in CHILD_PROCS]
    meta["pids_filhos_vivos_no_fim"] = vivos
    with open(json_path, "w", encoding="utf-8") as fh:
        json.dump({"meta": meta, "resultados": results}, fh, indent=1, ensure_ascii=False)
    print(f"\nJSON: {json_path}\nTabela: {md_path}\nLogs: {log_dir}\nYAMLs: {var_dir}")
    print(f"PIDs filhos criados: {len(CHILD_PROCS)}; vivos no fim: {vivos or 'nenhum'}")
    if nodes is None:
        print("ros2 node list (dominio 99): nao foi possivel executar")
    elif nodes:
        print(f"ATENCAO: ainda ha nos no dominio 99: {nodes}")
        return 1
    else:
        print("ros2 node list (dominio 99): vazio, tudo encerrado.")
    return 1 if vivos else 0


if __name__ == "__main__":
    sys.exit(main())
