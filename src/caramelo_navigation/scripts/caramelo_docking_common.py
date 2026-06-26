#!/usr/bin/env python3
"""Helpers for Caramelo docking files stored inside each map folder."""

from __future__ import annotations

import math
import os
import re
import shutil
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

DOCK_ID_PATTERN = re.compile(r"^(START|FINISH|WS[0-9]+|SH[0-9]+|PP[0-9]+|RT[0-9]+)$")
DEFAULT_DOCK_IDS = ["START", "FINISH", "WS1", "WS2", "WS3", "WS4", "SH1", "PP1", "RT1"]

DEFAULT_DOCK_PLUGINS = {
    "caramelo_front_dock": {
        "plugin": "opennav_docking::SimpleNonChargingDock",
        "docking_threshold": 0.05,
        "staging_x_offset": -0.70,
        "staging_yaw_offset": 0.0,
        "dock_direction": "forward",
        "rotate_to_dock": False,
        "use_external_detection_pose": False,
        "use_battery_status": False,
        "use_stall_detection": False,
    },
    "caramelo_precision_front_dock": {
        "plugin": "opennav_docking::SimpleNonChargingDock",
        "docking_threshold": 0.03,
        "staging_x_offset": -0.80,
        "staging_yaw_offset": 0.0,
        "dock_direction": "forward",
        "rotate_to_dock": False,
        "use_external_detection_pose": False,
        "use_battery_status": False,
        "use_stall_detection": False,
    },
    "caramelo_shelf_front_dock": {
        "plugin": "opennav_docking::SimpleNonChargingDock",
        "docking_threshold": 0.06,
        "staging_x_offset": -0.90,
        "staging_yaw_offset": 0.0,
        "dock_direction": "forward",
        "rotate_to_dock": False,
        "use_external_detection_pose": False,
        "use_battery_status": False,
        "use_stall_detection": False,
    },
}


def normalize_dock_id(dock_id: str) -> str:
    dock_id = (dock_id or "").strip().upper()
    if not DOCK_ID_PATTERN.match(dock_id):
        raise ValueError(
            "Use a nomenclatura do rulebook: WS1, WS2, SH1, PP1, "
            "RT1, START ou FINISH."
        )
    return dock_id


def infer_kind(dock_id: str) -> str:
    dock_id = normalize_dock_id(dock_id)
    if dock_id in ("START", "FINISH"):
        return dock_id
    return re.match(r"^[A-Z]+", dock_id).group(0)


def default_dock_type(dock_id: str) -> str:
    kind = infer_kind(dock_id)
    if kind == "SH":
        return "caramelo_shelf_front_dock"
    if kind in ("PP", "RT"):
        return "caramelo_precision_front_dock"
    return "caramelo_front_dock"


def _workspace_source_maps_from_share(share_dir: Path) -> Optional[Path]:
    for parent in share_dir.parents:
        if parent.name == "install":
            return parent.parent / "src" / "caramelo_mapping" / "maps"
    return None


def _cwd_source_map_roots() -> Iterable[Path]:
    cwd = Path.cwd().resolve()
    for parent in (cwd, *cwd.parents):
        candidate = parent / "src" / "caramelo_mapping" / "maps"
        if candidate.exists():
            yield candidate


def map_root_candidates(map_dir: Optional[str] = None) -> List[Path]:
    candidates: List[Path] = []
    if map_dir:
        candidates.append(Path(map_dir).expanduser())
    env_value = os.environ.get("CARAMELO_MAPS_DIR")
    if env_value:
        candidates.append(Path(env_value).expanduser())
    candidates.extend(_cwd_source_map_roots())
    try:
        share_dir = Path(get_package_share_directory("caramelo_mapping"))
        source_maps = _workspace_source_maps_from_share(share_dir)
        if source_maps:
            candidates.append(source_maps)
        candidates.append(share_dir / "maps")
    except PackageNotFoundError:
        pass

    unique: List[Path] = []
    seen = set()
    for candidate in candidates:
        resolved = candidate.resolve() if candidate.exists() else candidate.absolute()
        if resolved not in seen:
            unique.append(candidate)
            seen.add(resolved)
    return unique


def resolve_map_folder(map_name: str, map_dir: Optional[str] = None) -> Path:
    if not map_name:
        raise ValueError("map_name e obrigatorio.")
    checked: List[Path] = []
    for root in map_root_candidates(map_dir):
        root = root.expanduser()
        candidates = []
        if root.name == map_name:
            candidates.append(root)
        candidates.append(root / map_name)
        for candidate in candidates:
            checked.append(candidate)
            if candidate.exists() and (candidate / "map.yaml").exists():
                return candidate.resolve()
    checked_text = "\n  - ".join(str(path) for path in checked)
    raise FileNotFoundError(
        f"Nao encontrei a pasta do mapa '{map_name}' com map.yaml. Caminhos testados:\n  - {checked_text}"
    )


def docking_file(map_folder: Path) -> Path:
    return map_folder / "docking.yaml"


def service_areas_file(map_folder: Path) -> Path:
    return map_folder / "service_areas.yaml"


def load_yaml(path: Path, default: Optional[dict] = None) -> dict:
    if not path.exists():
        return dict(default or {})
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} precisa conter um dicionario YAML no topo.")
    return data


def backup_file(path: Path) -> Optional[Path]:
    if not path.exists():
        return None
    backup = path.with_name(path.name + ".bak")
    shutil.copy2(path, backup)
    return backup


def write_yaml(path: Path, data: dict, make_backup: bool = False) -> Optional[Path]:
    backup = backup_file(path) if make_backup else None
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=False, default_flow_style=False)
    return backup


def default_docking_database(dock_ids: Optional[Iterable[str]] = None) -> dict:
    ids = [normalize_dock_id(dock_id) for dock_id in (dock_ids or DEFAULT_DOCK_IDS)]
    return {
        "dock_plugins": dict(DEFAULT_DOCK_PLUGINS),
        "docks": {
            dock_id: {
                "type": default_dock_type(dock_id),
                "frame": "map",
                "pose": [0.0, 0.0, 0.0],
                "id": dock_id,
            }
            for dock_id in ids
        },
    }


def default_service_area(dock_id: str) -> dict:
    dock_id = normalize_dock_id(dock_id)
    kind = infer_kind(dock_id)
    base = {"kind": kind, "dock_id": dock_id, "approach": "front"}
    if kind == "START":
        base["notes"] = "Area inicial da arena."
    elif kind == "FINISH":
        base["notes"] = "Area final da arena."
    elif kind == "WS":
        base.update({"full_name": "Workstation", "table_height_m": 0.10, "width_m": 0.80, "depth_m": 0.50, "precision": "normal", "notes": "Workstation normal conforme rulebook."})
    elif kind == "SH":
        base.update({"full_name": "Shelf", "table_height_m": 0.10, "width_m": 0.80, "depth_m": 0.50, "precision": "normal", "notes": "Shelf. Usar dock type mais afastado."})
    elif kind == "PP":
        base.update({"full_name": "Precise Placement", "table_height_m": 0.10, "width_m": 0.80, "depth_m": 0.50, "precision": "high", "notes": "Precise Placement. Usar dock type mais preciso."})
    elif kind == "RT":
        base.update({"full_name": "Rotating Table", "table_height_m": 0.10, "diameter_m": 0.60, "precision": "high", "notes": "Rotating Table. Usar dock type mais preciso."})
    return base


def default_service_areas(dock_ids: Optional[Iterable[str]] = None) -> dict:
    ids = [normalize_dock_id(dock_id) for dock_id in (dock_ids or DEFAULT_DOCK_IDS)]
    return {"service_areas": {dock_id: default_service_area(dock_id) for dock_id in ids}}


def ensure_docking_files(map_folder: Path) -> Tuple[Path, Path, bool, bool]:
    dock_path = docking_file(map_folder)
    service_path = service_areas_file(map_folder)
    created_docking = False
    created_services = False
    if not dock_path.exists():
        write_yaml(dock_path, default_docking_database())
        created_docking = True
    if not service_path.exists():
        write_yaml(service_path, default_service_areas())
        created_services = True
    return dock_path, service_path, created_docking, created_services


def update_dock_pose(map_folder: Path, dock_id: str, dock_type: Optional[str], frame: str, pose: List[float], update_service_area: bool = False, service_fields: Optional[Dict[str, object]] = None):
    dock_id = normalize_dock_id(dock_id)
    dock_path = docking_file(map_folder)
    service_path = service_areas_file(map_folder)
    if not dock_path.exists():
        write_yaml(dock_path, default_docking_database([dock_id]))
    data = load_yaml(dock_path)
    data.setdefault("dock_plugins", dict(DEFAULT_DOCK_PLUGINS))
    data.setdefault("docks", {})
    data["docks"][dock_id] = {
        "type": dock_type or default_dock_type(dock_id),
        "frame": frame,
        "pose": [round(float(pose[0]), 4), round(float(pose[1]), 4), round(float(pose[2]), 6)],
        "id": dock_id,
    }
    dock_backup = write_yaml(dock_path, data, make_backup=True)

    service_backup = None
    if update_service_area:
        service_data = load_yaml(service_path, default_service_areas([dock_id]))
        service_data.setdefault("service_areas", {})
        entry = service_data["service_areas"].get(dock_id, default_service_area(dock_id))
        for key, value in (service_fields or {}).items():
            if value is not None:
                entry[key] = value
        entry.setdefault("kind", infer_kind(dock_id))
        entry.setdefault("dock_id", dock_id)
        entry.setdefault("approach", "front")
        service_data["service_areas"][dock_id] = entry
        service_backup = write_yaml(service_path, service_data, make_backup=service_path.exists())
    return dock_path, dock_backup, service_path if update_service_area else None, service_backup


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def service_fields_from_args(args) -> Dict[str, object]:
    fields: Dict[str, object] = {}
    if getattr(args, "kind", None):
        fields["kind"] = args.kind.upper()
    if getattr(args, "height", None) is not None:
        fields["table_height_m"] = float(args.height)
    if getattr(args, "width", None) is not None:
        fields["width_m"] = float(args.width)
    if getattr(args, "depth", None) is not None:
        fields["depth_m"] = float(args.depth)
    if getattr(args, "diameter", None) is not None:
        fields["diameter_m"] = float(args.diameter)
    if getattr(args, "precision", None):
        fields["precision"] = args.precision
    if getattr(args, "notes", None):
        fields["notes"] = args.notes
    return fields
