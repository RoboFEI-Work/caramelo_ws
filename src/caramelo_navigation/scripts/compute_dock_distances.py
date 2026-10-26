#!/usr/bin/env python3
# Calcula o comprimento de caminho entre todos os pares de docks de um mapa
# (START, FINISH, WS1..n, SH1, PP1, ST1...) e grava dock_distances.yaml na
# pasta do mapa. O task_planner (manip_bt) le essa matriz para escolher a
# ORDEM das estacoes que minimiza o tempo da missao (docs/planejador_por_tempo.md).
#
# Como calcula: Dijkstra 8-conexo (A* sem heuristica, pois cada fonte serve
# todos os destinos) sobre a grade de ocupacao do map.pgm, com os obstaculos
# INFLADOS pelo raio inscrito do robo + margem (--inflation, default 0,25 m:
# footprint 0,79 x 0,39 m -> inscrito 0,195 m + 0,055 m). Parte da pose de
# STAGING de cada dock (pose do dock recuada staging_x_offset do dock_plugins),
# que e' onde o Nav2 de fato leva o robo antes do docking; se a celula cair na
# zona inflada, usa a celula livre mais proxima. Sem map.pgm/map.yaml, ou par
# sem caminho na grade, cai na distancia euclidiana (anotada no arquivo).
#
# Como roda (da raiz do workspace, com o install sourceado):
#   ros2 run caramelo_navigation compute_dock_distances --map-name arenalegal_2026
#   python3 src/caramelo_navigation/scripts/compute_dock_distances.py --map-folder <pasta>
# Quem usa: operador, depois de gravar/mudar poses de dock no mapa.
from __future__ import annotations

import argparse
import datetime
import heapq
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

SCHEMA_VERSION = 1
DEFAULT_INFLATION_M = 0.25
DEFAULT_STAGING_X_OFFSET = -0.45
# Raio maximo (m) para procurar celula livre quando a pose cai em obstaculo.
SNAP_MAX_RADIUS_M = 0.60
OUTPUT_FILE = "dock_distances.yaml"


# ---- leitura do mapa ---------------------------------------------------------
def load_pgm(path: Path) -> np.ndarray:
    """Le um PGM (P5 binario ou P2 ascii) e devolve matriz (linhas, colunas) uint8."""
    data = path.read_bytes()
    tokens: List[bytes] = []
    pos = 0
    # Cabecalho: magic, largura, altura, maxval (comentarios '#' ate o fim da linha).
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if pos < len(data) and data[pos:pos + 1] == b"#":
            while pos < len(data) and data[pos:pos + 1] not in (b"\n", b"\r"):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic = tokens[0]
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b"P5":
        pos += 1  # um unico byte de espaco apos o maxval
        bytes_per_px = 2 if maxval > 255 else 1
        raw = np.frombuffer(data[pos:pos + width * height * bytes_per_px],
                            dtype=np.uint16 if bytes_per_px == 2 else np.uint8)
        if raw.size != width * height:
            raise ValueError(f"{path}: PGM truncado ({raw.size} de {width * height} pixels)")
        grid = raw.reshape((height, width)).astype(np.float64)
    elif magic == b"P2":
        values = np.array(data[pos:].split(), dtype=np.float64)
        if values.size != width * height:
            raise ValueError(f"{path}: PGM ascii truncado")
        grid = values.reshape((height, width))
    else:
        raise ValueError(f"{path}: formato {magic!r} nao suportado (esperado P5/P2)")
    if maxval != 255:
        grid = grid * (255.0 / maxval)
    return grid.astype(np.uint8)


def classify_cells(image: np.ndarray, map_meta: dict) -> Tuple[np.ndarray, np.ndarray]:
    """Mesma regra do map_server (modo trinary): devolve (ocupado, desconhecido).

    p = (255 - v) / 255 (negate 0); ocupado se p > occupied_thresh, livre se
    p < free_thresh, desconhecido no meio.
    """
    negate = int(map_meta.get("negate", 0))
    occupied_thresh = float(map_meta.get("occupied_thresh", 0.65))
    free_thresh = float(map_meta.get("free_thresh", 0.196))
    value = image.astype(np.float64)
    p = value / 255.0 if negate else (255.0 - value) / 255.0
    occupied = p > occupied_thresh
    unknown = (~occupied) & (p >= free_thresh)
    return occupied, unknown


def inflate(obstacles: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilata a mascara de obstaculos por um disco de `radius_cells` (OR de deslocamentos)."""
    if radius_cells <= 0:
        return obstacles.copy()
    out = obstacles.copy()
    rows, cols = obstacles.shape
    r2 = radius_cells * radius_cells
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc > r2 or (dr == 0 and dc == 0):
                continue
            src_r0, src_r1 = max(0, -dr), min(rows, rows - dr)
            src_c0, src_c1 = max(0, -dc), min(cols, cols - dc)
            if src_r0 >= src_r1 or src_c0 >= src_c1:
                continue
            out[src_r0 + dr:src_r1 + dr, src_c0 + dc:src_c1 + dc] |= \
                obstacles[src_r0:src_r1, src_c0:src_c1]
    return out


def world_to_cell(x: float, y: float, origin: List[float], resolution: float,
                  rows: int) -> Tuple[int, int]:
    """(x, y) no frame map -> (linha, coluna) da imagem (linha 0 = topo = y maximo).

    O epsilon evita que 0,7 / 0,05 = 13,999... caia na celula de baixo.
    """
    col = int(math.floor((x - float(origin[0])) / resolution + 1e-9))
    row = rows - 1 - int(math.floor((y - float(origin[1])) / resolution + 1e-9))
    return row, col


def nearest_free_cell(free: np.ndarray, cell: Tuple[int, int],
                      max_radius_cells: int) -> Optional[Tuple[int, int]]:
    """Celula livre mais proxima (anel por anel, Chebyshev); a propria se ja for livre."""
    rows, cols = free.shape
    r0, c0 = cell
    if 0 <= r0 < rows and 0 <= c0 < cols and free[r0, c0]:
        return cell
    for radius in range(1, max_radius_cells + 1):
        best = None
        best_d2 = None
        for dr in range(-radius, radius + 1):
            for dc in (-radius, radius) if abs(dr) != radius else range(-radius, radius + 1):
                r, c = r0 + dr, c0 + dc
                if 0 <= r < rows and 0 <= c < cols and free[r, c]:
                    d2 = dr * dr + dc * dc
                    if best_d2 is None or d2 < best_d2:
                        best, best_d2 = (r, c), d2
        if best is not None:
            return best
    return None


# ---- busca -------------------------------------------------------------------
_NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
              (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))]


def dijkstra_8(free: np.ndarray, start: Tuple[int, int], targets: List[Tuple[int, int]],
               resolution: float) -> Dict[Tuple[int, int], float]:
    """Menor caminho 8-conexo de `start` ate cada alvo (metros).

    Para quando todos os alvos foram fechados; alvo inalcancavel fica fora do
    dicionario. Nao corta quinas: o passo diagonal exige as duas celulas
    ortogonais livres.
    """
    rows, cols = free.shape
    dist = np.full((rows, cols), np.inf, dtype=np.float64)
    pending = set(targets)
    result: Dict[Tuple[int, int], float] = {}
    if not free[start[0], start[1]]:
        return result
    dist[start] = 0.0
    heap: List[Tuple[float, int, int]] = [(0.0, start[0], start[1])]
    while heap and pending:
        d, r, c = heapq.heappop(heap)
        if d > dist[r, c]:
            continue
        if (r, c) in pending:
            pending.discard((r, c))
            result[(r, c)] = d * resolution
        for dr, dc, w in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= rows or nc < 0 or nc >= cols or not free[nr, nc]:
                continue
            if dr != 0 and dc != 0 and not (free[r + dr, c] and free[r, c + dc]):
                continue
            nd = d + w
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                heapq.heappush(heap, (nd, nr, nc))
    return result


# ---- docks -------------------------------------------------------------------
def load_docks(docking_yaml: Path) -> Tuple[Dict[str, dict], Dict[str, dict]]:
    with docking_yaml.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    docks = data.get("docks") or {}
    plugins = data.get("dock_plugins") or {}
    if not isinstance(docks, dict) or not docks:
        raise ValueError(f"{docking_yaml}: sem a chave 'docks'")
    return docks, plugins


def dock_poses(docks: Dict[str, dict], plugins: Dict[str, dict],
               use_staging: bool) -> Dict[str, Tuple[float, float]]:
    """Pose (x, y) de partida de cada dock com pose REAL (placeholder [0,0,0] fica fora)."""
    poses: Dict[str, Tuple[float, float]] = {}
    for dock_id, dock in docks.items():
        pose = dock.get("pose") if isinstance(dock, dict) else None
        if not isinstance(pose, list) or len(pose) < 3:
            continue
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9:
            continue  # placeholder: pose ainda nao gravada
        if use_staging:
            plugin = plugins.get(dock.get("type", ""), {}) if isinstance(plugins, dict) else {}
            offset = float(plugin.get("staging_x_offset", DEFAULT_STAGING_X_OFFSET))
            x += offset * math.cos(yaw)
            y += offset * math.sin(yaw)
        poses[str(dock_id)] = (x, y)
    return poses


def euclidean_matrix(poses: Dict[str, Tuple[float, float]]) -> Dict[str, Dict[str, float]]:
    ids = list(poses)

    def dist(a, b):
        return 0.0 if a == b else math.hypot(poses[a][0] - poses[b][0], poses[a][1] - poses[b][1])

    return {a: {b: dist(a, b) for b in ids} for a in ids}


def compute_distances(map_folder: Path, inflation_m: float = DEFAULT_INFLATION_M,
                      use_staging: bool = True, allow_unknown: bool = True,
                      force_euclidean: bool = False) -> dict:
    """Matriz de distancias entre os docks do mapa.

    Devolve o dicionario que vai para o YAML (method, distances_m,
    euclidean_pairs, skipped_docks...).
    """
    docking_yaml = map_folder / "docking.yaml"
    if not docking_yaml.exists():
        raise FileNotFoundError(f"docking.yaml nao encontrado em {map_folder}")
    docks, plugins = load_docks(docking_yaml)
    poses = dock_poses(docks, plugins, use_staging)
    skipped = sorted(d for d in docks if d not in poses)
    ids = list(poses)

    result = {
        "schema": SCHEMA_VERSION,
        "map": map_folder.name,
        "generated_by": "compute_dock_distances",
        "generated_at": datetime.datetime.now().replace(microsecond=0).isoformat(),
        "pose": "staging" if use_staging else "dock",
        "inflation_m": float(inflation_m),
        "docks": ids,
        "skipped_docks": skipped,   # pose placeholder [0,0,0]: sem distancia
        "euclidean_pairs": [],      # pares sem caminho na grade (euclidiana)
        "notes": [],
    }

    map_yaml = map_folder / "map.yaml"
    meta = None
    if map_yaml.exists() and not force_euclidean:
        with map_yaml.open("r", encoding="utf-8") as handle:
            meta = yaml.safe_load(handle) or {}
    image_path = map_folder / str(meta.get("image", "map.pgm")) if meta else None
    if meta is None or not image_path.exists():
        result["method"] = "euclidean"
        result["notes"].append(
            "sem map.yaml/map.pgm na pasta: distancia euclidiana entre as poses")
        result["distances_m"] = euclidean_matrix(poses)
        return result

    resolution = float(meta["resolution"])
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    image = load_pgm(image_path)
    occupied, unknown = classify_cells(image, meta)
    blocked = occupied if allow_unknown else (occupied | unknown)
    radius_cells = int(math.ceil(inflation_m / resolution))
    free = ~inflate(blocked, radius_cells)
    rows = free.shape[0]

    cells: Dict[str, Tuple[int, int]] = {}
    snap_extra: Dict[str, float] = {}
    snap_max = int(math.ceil(SNAP_MAX_RADIUS_M / resolution))
    for dock_id, (x, y) in poses.items():
        raw = world_to_cell(x, y, origin, resolution, rows)
        cell = nearest_free_cell(free, raw, snap_max)
        if cell is None:
            result["notes"].append(
                f"{dock_id}: pose fora do espaco livre (a mais de {SNAP_MAX_RADIUS_M} m) - euclidiana")
            continue
        cells[dock_id] = cell
        extra = math.hypot(cell[0] - raw[0], cell[1] - raw[1]) * resolution
        if extra > 0.0:
            snap_extra[dock_id] = extra
            result["notes"].append(
                f"{dock_id}: pose na zona inflada, deslocada {extra:.2f} m para celula livre")

    euclid = euclidean_matrix(poses)
    distances: Dict[str, Dict[str, float]] = {a: {} for a in ids}
    euclidean_pairs = set()
    for a in ids:
        distances[a][a] = 0.0
    for i, a in enumerate(ids):
        targets = [b for b in ids[i + 1:] if b in cells]
        found: Dict[Tuple[int, int], float] = {}
        if a in cells and targets:
            found = dijkstra_8(free, cells[a], [cells[b] for b in targets], resolution)
        for b in ids[i + 1:]:
            d = found.get(cells[b]) if (a in cells and b in cells) else None
            if d is None:
                d = euclid[a][b]
                euclidean_pairs.add((a, b))
            else:
                # O trecho ate a celula livre mais proxima tambem conta.
                d += snap_extra.get(a, 0.0) + snap_extra.get(b, 0.0)
            distances[a][b] = round(float(d), 3)
            distances[b][a] = round(float(d), 3)
    result["method"] = "astar8"
    result["resolution_m"] = resolution
    result["euclidean_pairs"] = [list(p) for p in sorted(euclidean_pairs)]
    result["distances_m"] = distances
    return result


def write_dock_distances(map_folder: Path, result: dict, output: Optional[Path] = None) -> Path:
    path = output or (map_folder / OUTPUT_FILE)
    header = (
        "# Matriz de distancias entre docks (metros, comprimento de caminho).\n"
        "# GERADO por compute_dock_distances (caramelo_navigation) - nao editar a mao.\n"
        "# Regerar depois de mudar poses de dock ou o mapa:\n"
        f"#   ros2 run caramelo_navigation compute_dock_distances --map-name {map_folder.name}\n"
        "# Lido pelo task_planner (manip_bt) para ordenar as estacoes por tempo;\n"
        "# sem este arquivo o planejador segue guloso (docs/planejador_por_tempo.md).\n")
    with path.open("w", encoding="utf-8") as handle:
        handle.write(header)
        yaml.safe_dump(result, handle, sort_keys=False, allow_unicode=False,
                       default_flow_style=None, width=100)
    return path


def format_table(result: dict) -> str:
    ids = result["docks"]
    dist = result["distances_m"]
    width = max(6, max((len(i) for i in ids), default=6))
    lines = [" " * width + "".join(f"{b:>{width + 1}}" for b in ids)]
    for a in ids:
        lines.append(f"{a:<{width}}" + "".join(f"{dist[a][b]:>{width + 1}.2f}" for b in ids))
    return "\n".join(lines)


def _resolve_folder(args) -> Path:
    if args.map_folder:
        folder = Path(args.map_folder).expanduser().resolve()
        if not folder.is_dir():
            raise FileNotFoundError(f"pasta do mapa nao existe: {folder}")
        return folder
    if not args.map_name:
        raise ValueError("passe --map-name <mapa> ou --map-folder <pasta>")
    # Import tardio: puxa ament_index_python (so no ambiente ROS sourceado).
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from caramelo_docking_common import resolve_map_folder  # noqa: E402
    return resolve_map_folder(args.map_name, args.map_dir)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Gera dock_distances.yaml (comprimento de caminho entre docks) na pasta do mapa.")
    parser.add_argument("--map-name", default=None, help="Nome da pasta do mapa (caramelo_mapping/maps).")
    parser.add_argument("--map-dir", default=None, help="Raiz alternativa dos mapas.")
    parser.add_argument("--map-folder", default=None, help="Pasta do mapa (caminho direto).")
    parser.add_argument("--inflation", type=float, default=DEFAULT_INFLATION_M,
                        help=f"Raio de inflacao dos obstaculos em m (default {DEFAULT_INFLATION_M}).")
    parser.add_argument("--dock-pose", action="store_true",
                        help="Partir da pose do dock em vez da pose de staging.")
    parser.add_argument("--block-unknown", action="store_true",
                        help="Tratar celulas desconhecidas como obstaculo (default: livres, como o planner).")
    parser.add_argument("--euclidean", action="store_true",
                        help="Forcar distancia euclidiana (ignora o map.pgm).")
    parser.add_argument("--output", default=None,
                        help="Arquivo de saida (default: <mapa>/dock_distances.yaml).")
    parser.add_argument("--dry-run", action="store_true", help="So imprime a matriz, nao grava.")
    args = parser.parse_args(argv)
    try:
        folder = _resolve_folder(args)
        result = compute_distances(
            folder, inflation_m=args.inflation, use_staging=not args.dock_pose,
            allow_unknown=not args.block_unknown, force_euclidean=args.euclidean)
    except Exception as exc:  # noqa: BLE001 - erro do operador, mensagem curta
        print(f"Erro: {exc}", file=sys.stderr)
        return 1
    print(f"Mapa: {folder}  metodo: {result['method']}  inflacao: {result['inflation_m']} m"
          f"  pose: {result['pose']}")
    for note in result["notes"]:
        print(f"  aviso: {note}")
    if result["skipped_docks"]:
        print(f"  docks sem pose gravada (fora da matriz): {', '.join(result['skipped_docks'])}")
    if result["euclidean_pairs"]:
        pares = ", ".join(f"{a}-{b}" for a, b in result["euclidean_pairs"])
        print(f"  pares sem caminho na grade (euclidiana): {pares}")
    print(format_table(result))
    if args.dry_run:
        return 0
    out = write_dock_distances(folder, result, Path(args.output) if args.output else None)
    print(f"Gravado: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
