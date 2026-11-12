#!/usr/bin/env python3
# Bancada OFFLINE do planner global (SmacLattice): sobe map_server + planner_server
# + TF estatico em ROS_DOMAIN_ID=99 / namespace /teste com um lattice e overrides
# de parametros (copia do planner_server.yaml em /tmp; nada em src/ e' tocado),
# pede ComputePathToPose entre pares de docks do mapa e classifica cada passo do
# plano (frente / diagonal frente / lateral / diagonal re / re / giro no lugar).
# Quem usa: operador/agente, para comparar conjuntos de primitivas e penalidades
# ANTES de testar no robo. Encerra pelos PIDs tudo que subiu (nunca pkill).
# Como roda (sem robo; sem o install do pacote basta python3):
#   python3 src/caramelo_navigation/scripts/bench_lattice_offline.py \
#       --map-name arena_extendida --lattice <json> [--param SmacLattice.cost_penalty=4.0]
#   python3 src/caramelo_navigation/scripts/bench_lattice_offline.py --compare /tmp/bench_lattice/*.json
from __future__ import annotations

import argparse
import copy
import csv
import datetime
import json
import math
import os
import random
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

DEFAULT_DOMAIN_ID = "99"
DEFAULT_NS = "teste"
DEFAULT_MAP_NAME = "arena_extendida"
DEFAULT_OUT_ROOT = Path("/tmp/bench_lattice")
DEFAULT_PLANNER_ID = "SmacLattice"
# Mesmo default do opennav_docking quando o plugin nao declara staging_x_offset.
DEFAULT_STAGING_X_OFFSET = -0.45
# Passo com deslocamento menor que isto e' giro no lugar (se o yaw mudou) ou lixo.
IN_PLACE_EPS_M = 1e-3
YAW_EPS_RAD = 1e-6
# Origem do passo pelo espacamento: as primitivas do gerador tem passo <= 0.02 m
# (grade); a expansao analitica do Smac interpola a sqrt(2) celulas = 0.0283 m.
# Acima deste limiar o passo e' contado como "analitico" (reta ate o goal com o
# yaw interpolado entre bins, que no frame do heading vira lateral/re).
ANALYTIC_STEP_MIN_M = 0.024

# Classes de direcao do deslocamento no frame do heading da pose (graus).
# frente: |ang| < 22.5; diag_frente: 22.5..67.5; lateral: 67.5..112.5;
# diag_re: 112.5..157.5; re: > 157.5.
CLASSES = ["frente", "diag_frente", "lateral", "diag_re", "re"]
CLASS_SHORT = {"frente": "fre", "diag_frente": "dfr", "lateral": "lat",
               "diag_re": "dre", "re": "re", "giro": "giro"}

ERROR_NAMES = {
    0: "NONE", 200: "UNKNOWN", 201: "INVALID_PLANNER", 202: "TF_ERROR",
    203: "START_OUTSIDE_MAP", 204: "GOAL_OUTSIDE_MAP", 205: "START_OCCUPIED",
    206: "GOAL_OCCUPIED", 207: "TIMEOUT", 208: "NO_VALID_PATH",
}


# ----------------------------------------------------------------------------
# Caminhos do workspace
# ----------------------------------------------------------------------------
def workspace_root() -> Path:
    # scripts/ -> caramelo_navigation/ -> src/ -> raiz do workspace
    return Path(__file__).resolve().parents[3]


def default_lattice_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "lattice" / "caramelo_omni_2cm_16bins.json"


def default_planner_yaml() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "planner_server.yaml"


def resolve_map_folder(map_name: str, map_dir: Optional[str]) -> Path:
    candidates: List[Path] = []
    if map_dir:
        candidates.append(Path(map_dir).expanduser() / map_name)
    candidates.append(workspace_root() / "src" / "caramelo_mapping" / "maps" / map_name)
    candidates.append(Path.cwd() / "src" / "caramelo_mapping" / "maps" / map_name)
    for folder in candidates:
        if (folder / "map.yaml").exists():
            return folder.resolve()
    tested = "\n  - ".join(str(c) for c in candidates)
    raise FileNotFoundError(f"Mapa '{map_name}' sem map.yaml. Testados:\n  - {tested}")


# ----------------------------------------------------------------------------
# Docks -> poses de start/goal
# ----------------------------------------------------------------------------
def load_dock_poses(docking_yaml: Path, pose_source: str) -> Dict[str, Tuple[float, float, float]]:
    """Pose (x, y, yaw) por dock. 'staging' recua staging_x_offset ao longo do yaw
    do dock (e' para onde o Nav2 leva o robo antes do docking); 'dock' usa a
    pose do dock (igual ao final_robot_pose de service_areas.yaml)."""
    with docking_yaml.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    docks = data.get("docks") or {}
    plugins = data.get("dock_plugins") or {}
    poses: Dict[str, Tuple[float, float, float]] = {}
    for dock_id, dock in docks.items():
        pose = dock.get("pose") if isinstance(dock, dict) else None
        if not isinstance(pose, list) or len(pose) < 3:
            continue
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9:
            continue  # placeholder: pose ainda nao gravada
        if pose_source == "staging":
            plugin = plugins.get(dock.get("type", ""), {}) if isinstance(plugins, dict) else {}
            offset = float(plugin.get("staging_x_offset", DEFAULT_STAGING_X_OFFSET))
            yaw_off = float(plugin.get("staging_yaw_offset", 0.0))
            x += offset * math.cos(yaw)
            y += offset * math.sin(yaw)
            yaw = normalize_angle(yaw + yaw_off)
        poses[str(dock_id)] = (x, y, yaw)
    return poses


def build_pairs(poses: Dict[str, Tuple[float, float, float]], only: Optional[List[str]],
                sample: Optional[int], seed: int) -> List[Tuple[str, str]]:
    ids = [d for d in poses if not only or d in only]
    pairs = [(a, b) for a in ids for b in ids if a != b]
    if sample is not None and sample < len(pairs):
        rng = random.Random(seed)
        pairs = rng.sample(pairs, sample)
    return pairs


# ----------------------------------------------------------------------------
# Copia do planner_server.yaml com lattice/overrides e raiz do namespace
# ----------------------------------------------------------------------------
def parse_override(text: str) -> Tuple[List[str], object]:
    if "=" not in text:
        raise argparse.ArgumentTypeError(f"--param espera CHAVE=VALOR, veio '{text}'")
    key, raw = text.split("=", 1)
    return key.strip().split("."), yaml.safe_load(raw.strip())


def apply_override(tree: dict, path: List[str], value: object) -> None:
    node = tree
    for part in path[:-1]:
        if part not in node or not isinstance(node[part], dict):
            node[part] = {}
        node = node[part]
    old = node.get(path[-1])
    # Params file e' tipado: double declarado nao aceita int; coagimos.
    if isinstance(old, float) and isinstance(value, int) and not isinstance(value, bool):
        value = float(value)
    node[path[-1]] = value


def build_params_copy(src_yaml: Path, out_path: Path, ns: str, lattice: Path,
                      overrides: List[Tuple[List[str], object]], keep_obstacle_layer: bool,
                      map_topic: str) -> dict:
    with src_yaml.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    planner = data["planner_server"]["ros__parameters"]
    costmap = data["global_costmap"]["global_costmap"]["ros__parameters"]
    planner["use_sim_time"] = False
    costmap["use_sim_time"] = False
    planner[DEFAULT_PLANNER_ID]["lattice_filepath"] = str(lattice)
    costmap.setdefault("static_layer", {})["map_topic"] = map_topic
    if not keep_obstacle_layer:
        # Sem /scan na bancada: fica static_layer + denoise + inflation.
        costmap["plugins"] = [p for p in costmap.get("plugins", []) if p != "obstacle_layer"]
        costmap.pop("obstacle_layer", None)
    for path, value in overrides:
        if path[0] == "global_costmap":
            apply_override(costmap, path[1:], value)
        else:
            apply_override(planner, path, value)
    # Raiz = namespace: a chave 'planner_server:' sem namespace NAO casa com
    # /teste/planner_server (testado em Jazzy; e' o que o RewrittenYaml do
    # nav2_bringup faz com root_key).
    wrapped = {ns: {"planner_server": data["planner_server"],
                    "global_costmap": data["global_costmap"]}}
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(wrapped, handle, sort_keys=False)
    return wrapped


# ----------------------------------------------------------------------------
# Geometria e classificacao dos passos do plano
# ----------------------------------------------------------------------------
def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def classify_direction(rel_angle: float) -> str:
    deg = abs(math.degrees(rel_angle))
    if deg < 22.5:
        return "frente"
    if deg < 67.5:
        return "diag_frente"
    if deg < 112.5:
        return "lateral"
    if deg < 157.5:
        return "diag_re"
    return "re"


def classify_path(points: List[Tuple[float, float, float]]) -> dict:
    """points: lista (x, y, yaw) do plano. Cada passo entre poses consecutivas e'
    classificado pela direcao do deslocamento no frame do heading da pose de
    partida. Giro no lugar: deslocamento < 1 mm e yaw mudou (passos consecutivos
    de giro contam UM giro)."""
    length_by = {c: 0.0 for c in CLASSES}
    length_prim_by = {c: 0.0 for c in CLASSES}
    steps_by = {c: 0 for c in CLASSES}
    total = 0.0
    analytic_len = 0.0
    analytic_steps = 0
    turn_events = 0
    turn_steps = 0
    turn_angle = 0.0
    class_changes = 0
    prev_class: Optional[str] = None
    prev_was_turn = False
    sequence: List[str] = []  # classes dos segmentos (runs), para depurar
    for (x0, y0, yaw0), (x1, y1, yaw1) in zip(points, points[1:]):
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        dyaw = normalize_angle(yaw1 - yaw0)
        if dist < IN_PLACE_EPS_M:
            if abs(dyaw) > YAW_EPS_RAD:
                turn_steps += 1
                turn_angle += abs(dyaw)
                if not prev_was_turn:
                    turn_events += 1
                    sequence.append("giro")
                prev_was_turn = True
            continue
        cls = classify_direction(normalize_angle(math.atan2(dy, dx) - yaw0))
        length_by[cls] += dist
        steps_by[cls] += 1
        total += dist
        if dist > ANALYTIC_STEP_MIN_M:
            analytic_len += dist
            analytic_steps += 1
        else:
            length_prim_by[cls] += dist
        if prev_class is not None and cls != prev_class:
            class_changes += 1
        if cls != prev_class or prev_was_turn:
            sequence.append(cls)
        prev_class = cls
        prev_was_turn = False
    pct = {c: (100.0 * length_by[c] / total if total > 0 else 0.0) for c in CLASSES}
    prim_total = total - analytic_len
    pct_prim = {c: (100.0 * length_prim_by[c] / prim_total if prim_total > 0 else 0.0) for c in CLASSES}
    straight = math.hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1]) if len(points) >= 2 else 0.0
    return {
        "comprimento_m": round(total, 4),
        "reta_m": round(straight, 4),
        "comprimento_por_classe_m": {c: round(v, 4) for c, v in length_by.items()},
        "pct_por_classe": {c: round(v, 2) for c, v in pct.items()},
        "comprimento_analitico_m": round(analytic_len, 4),
        "pct_analitico": round(100.0 * analytic_len / total, 2) if total > 0 else 0.0,
        "passos_analiticos": analytic_steps,
        "comprimento_por_classe_primitivas_m": {c: round(v, 4) for c, v in length_prim_by.items()},
        "pct_por_classe_primitivas": {c: round(v, 2) for c, v in pct_prim.items()},
        "passos_por_classe": steps_by,
        "giros": turn_events,
        "passos_giro": turn_steps,
        "angulo_girado_rad": round(turn_angle, 4),
        "mudancas_classe": class_changes,
        "n_poses": len(points),
        "sequencia": sequence,
    }


def lattice_summary(lattice: Path) -> dict:
    """Resumo do JSON de primitivas: translacoes locais distintas (no heading 0)
    e contagem de giros/translacoes, para constar no relatorio."""
    with lattice.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    meta = data.get("lattice_metadata", {})
    prims = data.get("primitives", [])
    headings = meta.get("heading_angles", [])
    rotations = 0
    translations = 0
    local_vectors = set()
    for prim in prims:
        if prim.get("start_angle_index") != prim.get("end_angle_index"):
            rotations += 1
            continue
        translations += 1
        if prim.get("start_angle_index") == 0 and prim.get("poses"):
            px, py, _ = prim["poses"][-1]
            h = headings[0] if headings else 0.0
            lx = px * math.cos(h) + py * math.sin(h)
            ly = -px * math.sin(h) + py * math.cos(h)
            local_vectors.add((round(lx, 3), round(ly, 3)))
    return {
        "arquivo": str(lattice),
        "motion_model": meta.get("motion_model"),
        "num_headings": meta.get("num_of_headings"),
        "num_primitivas": meta.get("number_of_trajectories", len(prims)),
        "giros_no_lugar": rotations,
        "translacoes": translations,
        "translacoes_locais_heading0": sorted(local_vectors),
    }


# ----------------------------------------------------------------------------
# Processos filhos (so' PIDs guardados; nada de pkill)
# ----------------------------------------------------------------------------
class ChildProcesses:
    def __init__(self, env: dict, log_dir: Path, log_prefix: str):
        self.env = env
        self.log_dir = log_dir
        self.log_prefix = log_prefix
        self.procs: List[Tuple[str, subprocess.Popen, object]] = []

    def spawn(self, name: str, cmd: List[str]) -> int:
        log_path = self.log_dir / f"{self.log_prefix}_{name}.log"
        log = log_path.open("w", encoding="utf-8")
        log.write("# " + " ".join(cmd) + "\n")
        log.flush()
        proc = subprocess.Popen(cmd, env=self.env, stdout=log, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, start_new_session=True)
        self.procs.append((name, proc, log))
        print(f"[bancada] subiu {name:<16} PID {proc.pid}  log {log_path}")
        return proc.pid

    def alive(self) -> List[str]:
        return [name for name, proc, _ in self.procs if proc.poll() is None]

    def stop_all(self) -> List[dict]:
        report = []
        for name, proc, log in reversed(self.procs):
            how = "ja_encerrado"
            if proc.poll() is None:
                how = "SIGINT"
                try:
                    proc.send_signal(signal.SIGINT)
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    how = "SIGTERM"
                    proc.terminate()
                    try:
                        proc.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        how = "SIGKILL"
                        proc.kill()
                        proc.wait(timeout=3.0)
            try:
                log.close()
            except Exception:
                pass
            report.append({"nome": name, "pid": proc.pid, "como": how, "rc": proc.returncode})
            print(f"[bancada] encerrou {name:<14} PID {proc.pid} via {how} rc={proc.returncode}")
        return report


def executable_path(package: str, name: str) -> str:
    from ament_index_python.packages import get_package_prefix
    exe = Path(get_package_prefix(package)) / "lib" / package / name
    if not exe.exists():
        raise FileNotFoundError(f"Executavel {exe} nao existe (pacote {package})")
    return str(exe)


# ----------------------------------------------------------------------------
# Execucao da bancada (rclpy)
# ----------------------------------------------------------------------------
def run_bench(args) -> dict:
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import ComputePathToPose
    from nav_msgs.msg import OccupancyGrid

    ns = args.ns.strip("/")
    map_folder = resolve_map_folder(args.map_name, args.map_dir)
    map_yaml = map_folder / "map.yaml"
    docking_yaml = map_folder / "docking.yaml"
    lattice = Path(args.lattice).expanduser().resolve()
    if not lattice.exists():
        raise FileNotFoundError(f"Lattice nao existe: {lattice}")
    overrides = [parse_override(p) for p in (args.param or [])]
    label = args.label or default_label(lattice, overrides)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_json = Path(args.out).expanduser() if args.out else DEFAULT_OUT_ROOT / f"{stamp}_{label}_{args.map_name}.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)
    run_dir = out_json.parent / f"{out_json.stem}_run"
    run_dir.mkdir(parents=True, exist_ok=True)

    poses = load_dock_poses(docking_yaml, args.pose_source)
    pairs = build_pairs(poses, args.docks, args.pairs, args.seed)
    if not pairs:
        raise RuntimeError(f"Nenhum par de docks em {docking_yaml}")

    map_topic = f"/{ns}/map"
    params_copy = run_dir / "planner_server_bench.yaml"
    build_params_copy(default_planner_yaml(), params_copy, ns, lattice, overrides,
                      args.keep_obstacle_layer, map_topic)
    print(f"[bancada] mapa {map_yaml}")
    print(f"[bancada] lattice {lattice}")
    print(f"[bancada] params copiados em {params_copy} (overrides: {args.param or []})")
    print(f"[bancada] {len(poses)} docks com pose ({args.pose_source}), {len(pairs)} pares")

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    children = ChildProcesses(env, run_dir, "proc")
    result: dict = {
        "label": label,
        "mapa": args.map_name,
        "map_yaml": str(map_yaml),
        "docking_yaml": str(docking_yaml),
        "pose_source": args.pose_source,
        "lattice": lattice_summary(lattice),
        "overrides": [".".join(p) + "=" + json.dumps(v) for p, v in overrides],
        "params_copy": str(params_copy),
        "obstacle_layer_removida": not args.keep_obstacle_layer,
        "planner_id": args.planner_id,
        "domain_id": str(args.domain_id),
        "namespace": "/" + ns,
        "inicio": datetime.datetime.now().isoformat(timespec="seconds"),
        "docks": {k: list(v) for k, v in poses.items()},
        "planos": [],
    }

    node = None
    executor = None
    interrupted = False
    try:
        children.spawn("map_server", [
            executable_path("nav2_map_server", "map_server"), "--ros-args",
            "-r", f"__ns:=/{ns}", "-p", f"yaml_filename:={map_yaml}",
            "-p", "use_sim_time:=false", "-p", f"topic_name:={map_topic}", "-p", "frame_id:=map"])
        children.spawn("planner_server", [
            executable_path("nav2_planner", "planner_server"), "--ros-args",
            "-r", f"__ns:=/{ns}", "--params-file", str(params_copy)])
        # TF estatico map->odom->base_footprint (o costmap precisa da pose do
        # robo para atualizar; o start do plano vem do goal com use_start).
        tf_exe = executable_path("tf2_ros", "static_transform_publisher")
        children.spawn("tf_map_odom", [
            tf_exe, "--frame-id", "map", "--child-frame-id", "odom", "--ros-args",
            "-r", f"__ns:=/{ns}", "-r", "__node:=teste_tf_map_odom"])
        children.spawn("tf_odom_base", [
            tf_exe, "--frame-id", "odom", "--child-frame-id", "base_footprint", "--ros-args",
            "-r", f"__ns:=/{ns}", "-r", "__node:=teste_tf_odom_base"])

        rclpy.init(args=None)
        node = Node("teste_bench_lattice", namespace="/" + ns)
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def change_state(node_name: str, transition_id: int, label_t: str) -> None:
            client = node.create_client(ChangeState, f"/{ns}/{node_name}/change_state")
            deadline = time.monotonic() + args.startup_timeout
            while not client.wait_for_service(timeout_sec=1.0):
                dead = [n for n in ("map_server", "planner_server") if n not in children.alive()]
                if dead:
                    raise RuntimeError(f"processo(s) {dead} morreram antes de {label_t}; veja os logs em {run_dir}")
                if time.monotonic() > deadline:
                    raise TimeoutError(f"servico {node_name}/change_state nao apareceu em {args.startup_timeout}s")
            req = ChangeState.Request()
            req.transition.id = transition_id
            fut = client.call_async(req)
            executor.spin_until_future_complete(fut, timeout_sec=args.startup_timeout)
            if fut.result() is None or not fut.result().success:
                raise RuntimeError(f"{node_name}: transicao {label_t} falhou (veja {run_dir})")
            node.destroy_client(client)
            print(f"[bancada] {node_name}: {label_t} OK")

        # Mesma ordem do lifecycle manager: configura todos, depois ativa todos.
        change_state("map_server", Transition.TRANSITION_CONFIGURE, "configure")
        change_state("planner_server", Transition.TRANSITION_CONFIGURE, "configure")
        change_state("map_server", Transition.TRANSITION_ACTIVATE, "activate")
        change_state("planner_server", Transition.TRANSITION_ACTIVATE, "activate")

        # Espera o costmap global assumir o tamanho do mapa (static_layer recebeu /map).
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        got = {"map": None, "costmap": None}
        sub_map = node.create_subscription(OccupancyGrid, map_topic, lambda m: got.__setitem__("map", m), latched)
        sub_cm = node.create_subscription(OccupancyGrid, f"/{ns}/global_costmap/costmap",
                                          lambda m: got.__setitem__("costmap", m), latched)
        deadline = time.monotonic() + args.startup_timeout
        costmap_ready = False
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
            m, c = got["map"], got["costmap"]
            if m is not None and c is not None and c.info.width == m.info.width and c.info.height == m.info.height:
                costmap_ready = True
                break
        node.destroy_subscription(sub_map)
        node.destroy_subscription(sub_cm)
        if got["map"] is not None:
            result["mapa_celulas"] = [got["map"].info.width, got["map"].info.height]
        if costmap_ready:
            print(f"[bancada] costmap global pronto: {got['costmap'].info.width}x{got['costmap'].info.height} celulas")
        else:
            print("[bancada] AVISO: costmap global nao confirmou o tamanho do mapa; sigo mesmo assim")
        result["costmap_confirmado"] = costmap_ready

        action_name = f"/{ns}/compute_path_to_pose"
        client = ActionClient(node, ComputePathToPose, action_name)
        if not client.wait_for_server(timeout_sec=args.startup_timeout):
            raise TimeoutError(f"action {action_name} nao apareceu")

        def make_pose(xyyaw) -> PoseStamped:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = node.get_clock().now().to_msg()
            ps.pose.position.x = float(xyyaw[0])
            ps.pose.position.y = float(xyyaw[1])
            ps.pose.orientation.z = math.sin(xyyaw[2] / 2.0)
            ps.pose.orientation.w = math.cos(xyyaw[2] / 2.0)
            return ps

        def plan(a: str, b: str) -> dict:
            goal = ComputePathToPose.Goal()
            goal.start = make_pose(poses[a])
            goal.goal = make_pose(poses[b])
            goal.planner_id = args.planner_id
            goal.use_start = True
            t0 = time.monotonic()
            send = client.send_goal_async(goal)
            executor.spin_until_future_complete(send, timeout_sec=args.plan_timeout)
            handle = send.result()
            row = {"de": a, "para": b, "ok": False, "erro": None, "erro_msg": "",
                   "planning_time_s": None, "wall_s": None}
            if handle is None or not handle.accepted:
                row["erro"] = "GOAL_REJEITADO"
                row["wall_s"] = round(time.monotonic() - t0, 3)
                return row
            res_fut = handle.get_result_async()
            executor.spin_until_future_complete(res_fut, timeout_sec=args.plan_timeout)
            row["wall_s"] = round(time.monotonic() - t0, 3)
            if res_fut.result() is None:
                handle.cancel_goal_async()
                row["erro"] = "TIMEOUT_CLIENTE"
                return row
            res = res_fut.result().result
            row["planning_time_s"] = round(res.planning_time.sec + res.planning_time.nanosec * 1e-9, 4)
            row["erro_msg"] = res.error_msg
            if res.error_code != 0 or len(res.path.poses) < 2:
                row["erro"] = ERROR_NAMES.get(res.error_code, str(res.error_code))
                return row
            pts = [(p.pose.position.x, p.pose.position.y, yaw_from_quat(p.pose.orientation)) for p in res.path.poses]
            row["ok"] = True
            row.update(classify_path(pts))
            if args.save_paths:
                row["poses"] = [[round(x, 4), round(y, 4), round(w, 4)] for x, y, w in pts]
            return row

        for i in range(args.warmup):
            a, b = pairs[0]
            w = plan(a, b)
            print(f"[bancada] aquecimento {i + 1}: {a}->{b} ok={w['ok']} erro={w['erro']} t={w['planning_time_s']}")
            result.setdefault("aquecimento", []).append(w)

        print(f"\n{'par':<16}{'L(m)':>7}{'fre%':>6}{'dfr%':>6}{'lat%':>6}{'dre%':>6}{'re%':>6}{'ana%':>6}"
              f"{'giros':>6}{'troca':>6}{'t(s)':>7}  status")
        for idx, (a, b) in enumerate(pairs, 1):
            row = plan(a, b)
            result["planos"].append(row)
            print(format_row(row))
            sys.stdout.flush()
    except KeyboardInterrupt:
        interrupted = True
        print("\n[bancada] interrompido (SIGINT); encerrando o que subiu")
    finally:
        result["interrompido"] = interrupted
        result["fim"] = datetime.datetime.now().isoformat(timespec="seconds")
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        result["processos"] = children.stop_all()
        result["pids"] = [p["pid"] for p in result["processos"]]
        result["agregado"] = aggregate(result["planos"])
        write_outputs(result, out_json)
        print_aggregate(result)
        result["nos_restantes_dominio"] = list_remaining_nodes(args.domain_id)
        print(f"[bancada] nos restantes no dominio {args.domain_id}: {result['nos_restantes_dominio'] or 'nenhum'}")
        write_outputs(result, out_json)
        print(f"[bancada] resultado: {out_json}  (CSV ao lado; logs em {run_dir})")
    return result


def list_remaining_nodes(domain_id) -> List[str]:
    """Abre um contexto rclpy novo e lista os nos visiveis no dominio (sem o
    proprio), para provar que a bancada nao deixou nada para tras."""
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        # Contexto e executor proprios: o executor global pertence ao contexto
        # default, que ja foi encerrado pela bancada.
        ctx = Context()
        rclpy.init(context=ctx)
        node = Node("teste_bench_lattice_check", context=ctx)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.2)
        names = [f"{ns.rstrip('/')}/{n}" for n, ns in node.get_node_names_and_namespaces()
                 if n != "teste_bench_lattice_check"]
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        return sorted(names)
    except Exception as exc:  # nao e' motivo para falhar a bancada
        return [f"(erro ao listar: {exc})"]


# ----------------------------------------------------------------------------
# Agregacao, tabela, saida
# ----------------------------------------------------------------------------
def default_label(lattice: Path, overrides) -> str:
    label = lattice.stem
    for path, value in overrides:
        label += "__" + path[-1] + "-" + str(value).replace("/", "_")
    return label


def aggregate(rows: List[dict]) -> dict:
    ok = [r for r in rows if r.get("ok")]
    total_len = sum(r["comprimento_m"] for r in ok)
    len_by = {c: sum(r["comprimento_por_classe_m"][c] for r in ok) for c in CLASSES}
    len_prim_by = {c: sum(r["comprimento_por_classe_primitivas_m"][c] for r in ok) for c in CLASSES}
    analytic_len = sum(r["comprimento_analitico_m"] for r in ok)
    prim_total = total_len - analytic_len
    times = [r["planning_time_s"] for r in rows if r.get("planning_time_s") is not None]
    failures: Dict[str, int] = {}
    for r in rows:
        if not r.get("ok"):
            failures[str(r.get("erro"))] = failures.get(str(r.get("erro")), 0) + 1
    maioria_nao_frente = sum(1 for r in ok if r["pct_por_classe"]["frente"] < 50.0)
    return {
        "planos": len(rows),
        "ok": len(ok),
        "falhas": len(rows) - len(ok),
        "falhas_por_erro": failures,
        "comprimento_total_m": round(total_len, 3),
        "pct_por_classe": {c: round(100.0 * len_by[c] / total_len, 2) if total_len > 0 else 0.0 for c in CLASSES},
        "pct_lado_mais_re": round(100.0 * (len_by["lateral"] + len_by["diag_re"] + len_by["re"]) / total_len, 2) if total_len > 0 else 0.0,
        "pct_analitico": round(100.0 * analytic_len / total_len, 2) if total_len > 0 else 0.0,
        "pct_por_classe_primitivas": {c: round(100.0 * len_prim_by[c] / prim_total, 2) if prim_total > 0 else 0.0 for c in CLASSES},
        "giros_total": sum(r["giros"] for r in ok),
        "giros_por_plano": round(sum(r["giros"] for r in ok) / len(ok), 2) if ok else 0.0,
        "mudancas_classe_por_plano": round(sum(r["mudancas_classe"] for r in ok) / len(ok), 2) if ok else 0.0,
        "planos_maioria_nao_frente": maioria_nao_frente,
        "razao_comprimento_reta_media": round(statistics.mean(r["comprimento_m"] / r["reta_m"] for r in ok if r["reta_m"] > 0.05), 3) if ok else None,
        "planning_time_s": {
            "media": round(statistics.mean(times), 4) if times else None,
            "mediana": round(statistics.median(times), 4) if times else None,
            "max": round(max(times), 4) if times else None,
        },
    }


def format_row(row: dict) -> str:
    par = f"{row['de']}->{row['para']}"
    if not row.get("ok"):
        t = row.get("planning_time_s")
        return f"{par:<16}{'-':>7}{'':>42}{(t if t is not None else '-'):>7}  FALHA {row.get('erro')} {row.get('erro_msg', '')[:60]}"
    p = row["pct_por_classe"]
    return (f"{par:<16}{row['comprimento_m']:>7.2f}{p['frente']:>6.0f}{p['diag_frente']:>6.0f}{p['lateral']:>6.0f}"
            f"{p['diag_re']:>6.0f}{p['re']:>6.0f}{row['pct_analitico']:>6.0f}{row['giros']:>6d}{row['mudancas_classe']:>6d}"
            f"{row['planning_time_s']:>7.3f}  OK")


def print_aggregate(result: dict) -> None:
    agg = result["agregado"]
    p = agg["pct_por_classe"]
    print("\n[bancada] AGREGADO", result["label"], "mapa", result["mapa"])
    print(f"  planos {agg['planos']}  ok {agg['ok']}  falhas {agg['falhas']} {agg['falhas_por_erro']}")
    print(f"  % comprimento: frente {p['frente']:.1f}  diag_frente {p['diag_frente']:.1f}  lateral {p['lateral']:.1f}"
          f"  diag_re {p['diag_re']:.1f}  re {p['re']:.1f}   (lado+re {agg['pct_lado_mais_re']:.1f})")
    pp = agg["pct_por_classe_primitivas"]
    print(f"  so' primitivas (sem expansao analitica, {100 - agg['pct_analitico']:.1f}% do comprimento): frente {pp['frente']:.1f}"
          f"  diag_frente {pp['diag_frente']:.1f}  lateral {pp['lateral']:.1f}  diag_re {pp['diag_re']:.1f}  re {pp['re']:.1f}"
          f"   | expansao analitica {agg['pct_analitico']:.1f}%")
    print(f"  giros/plano {agg['giros_por_plano']}  mudancas de classe/plano {agg['mudancas_classe_por_plano']}"
          f"  planos com frente<50% {agg['planos_maioria_nao_frente']}/{agg['ok']}")
    t = agg["planning_time_s"]
    print(f"  planning_time s: media {t['media']}  mediana {t['mediana']}  max {t['max']}"
          f"   comprimento total {agg['comprimento_total_m']} m")


def write_outputs(result: dict, out_json: Path) -> None:
    with out_json.open("w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, ensure_ascii=False)
    out_csv = out_json.with_suffix(".csv")
    cols = ["de", "para", "ok", "erro", "erro_msg", "planning_time_s", "wall_s", "comprimento_m", "reta_m",
            *[f"pct_{c}" for c in CLASSES], "pct_analitico", *[f"prim_pct_{c}" for c in CLASSES], "giros", "angulo_girado_rad", "mudancas_classe", "n_poses"]
    with out_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(cols)
        for r in result.get("planos", []):
            pct = r.get("pct_por_classe", {})
            writer.writerow([r.get("de"), r.get("para"), int(bool(r.get("ok"))), r.get("erro"), r.get("erro_msg"),
                             r.get("planning_time_s"), r.get("wall_s"), r.get("comprimento_m"), r.get("reta_m"),
                             *[pct.get(c) for c in CLASSES], r.get("pct_analitico"),
                             *[r.get("pct_por_classe_primitivas", {}).get(c) for c in CLASSES], r.get("giros"), r.get("angulo_girado_rad"),
                             r.get("mudancas_classe"), r.get("n_poses")])


def compare(files: List[str]) -> None:
    """Tabela comparativa a partir dos JSONs gravados (nao precisa de ROS)."""
    rows = []
    for f in files:
        with open(f, "r", encoding="utf-8") as handle:
            rows.append(json.load(handle))
    rows.sort(key=lambda r: (r.get("mapa", ""), r.get("label", "")))
    hdr = (f"{'label':<34}{'mapa':<17}{'ok/tot':>8}{'fre%':>7}{'dfr%':>7}{'lat%':>7}{'dre%':>7}{'re%':>6}{'ana%':>6}"
           f"{'fre_p%':>7}{'lat+re_p%':>10}{'giros/p':>8}{'troca/p':>8}{'t_med':>7}{'t_max':>7}{'L_tot':>7}  falhas")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        a = r.get("agregado", {})
        p = a.get("pct_por_classe", {})
        t = a.get("planning_time_s", {})
        pp = a.get("pct_por_classe_primitivas", {})
        lado_re_p = pp.get("lateral", 0) + pp.get("diag_re", 0) + pp.get("re", 0)
        print(f"{r.get('label', '')[:33]:<34}{r.get('mapa', '')[:16]:<17}{a.get('ok', 0):>4}/{a.get('planos', 0):<3}"
              f"{p.get('frente', 0):>7.1f}{p.get('diag_frente', 0):>7.1f}{p.get('lateral', 0):>7.1f}{p.get('diag_re', 0):>7.1f}"
              f"{p.get('re', 0):>6.1f}{a.get('pct_analitico', 0):>6.1f}{pp.get('frente', 0):>7.1f}{lado_re_p:>10.1f}"
              f"{a.get('giros_por_plano', 0):>8.2f}{a.get('mudancas_classe_por_plano', 0):>8.2f}"
              f"{(t.get('mediana') or 0):>7.3f}{(t.get('max') or 0):>7.3f}{a.get('comprimento_total_m', 0):>7.1f}  {a.get('falhas_por_erro', {})}")
    print("\nLegenda: % do comprimento do plano por classe (fre frente, dfr diagonal frente, lat lateral,"
          " dre diagonal re, re re); ana% = parte do plano vinda da expansao analitica (passo 0.028 m);"
          " fre_p% e lat+re_p% = frente e lateral+diag_re+re contando SO' as primitivas; giros/p = giros no lugar por plano; troca/p = mudancas de classe por plano;"
          " t = planning_time (s) mediana/max; L_tot = soma dos comprimentos (m) dos planos OK.")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Bancada offline do SmacLattice (ROS_DOMAIN_ID=99, namespace /teste).")
    parser.add_argument("--map-name", default=DEFAULT_MAP_NAME)
    parser.add_argument("--map-dir", default=None, help="Raiz alternativa das pastas de mapa.")
    parser.add_argument("--lattice", default=str(default_lattice_path()), help="JSON de primitivas.")
    parser.add_argument("--param", action="append", metavar="CHAVE=VALOR",
                        help="Override no planner_server (ex.: SmacLattice.cost_penalty=4.0); "
                             "prefixo global_costmap. vai para o costmap (ex.: global_costmap.inflation_layer.inflation_radius=0.55).")
    parser.add_argument("--label", default=None, help="Nome da variante no relatorio (default: nome do lattice + overrides).")
    parser.add_argument("--pose-source", choices=["staging", "dock"], default="staging")
    parser.add_argument("--docks", nargs="*", default=None, help="Restringe aos docks listados.")
    parser.add_argument("--pairs", type=int, default=None, help="Amostra N pares ordenados (default: todos).")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default=None, help="JSON de saida (CSV ao lado).")
    parser.add_argument("--keep-obstacle-layer", action="store_true", help="Mantem obstacle_layer (sem /scan na bancada).")
    parser.add_argument("--planner-id", default=DEFAULT_PLANNER_ID)
    parser.add_argument("--domain-id", default=DEFAULT_DOMAIN_ID)
    parser.add_argument("--ns", default=DEFAULT_NS)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--plan-timeout", type=float, default=30.0)
    parser.add_argument("--warmup", type=int, default=1, help="Planos de aquecimento descartados.")
    parser.add_argument("--save-paths", action="store_true", help="Grava as poses dos planos no JSON.")
    parser.add_argument("--dry-run", action="store_true", help="So' lista docks/pares e gera a copia dos params.")
    parser.add_argument("--compare", nargs="+", metavar="JSON", help="So' imprime a tabela comparativa dos JSONs.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.compare:
        compare(args.compare)
        return 0
    if args.dry_run:
        folder = resolve_map_folder(args.map_name, args.map_dir)
        poses = load_dock_poses(folder / "docking.yaml", args.pose_source)
        pairs = build_pairs(poses, args.docks, args.pairs, args.seed)
        for d, p in poses.items():
            print(f"  {d:<8} x={p[0]:.3f} y={p[1]:.3f} yaw={p[2]:.3f}")
        print(f"{len(poses)} docks, {len(pairs)} pares; lattice {lattice_summary(Path(args.lattice))}")
        return 0

    def on_signal(signum, _frame):
        raise KeyboardInterrupt(f"sinal {signum}")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    result = run_bench(args)
    return 0 if result["agregado"]["ok"] > 0 and not result.get("interrompido") else 1


if __name__ == "__main__":
    sys.exit(main())
