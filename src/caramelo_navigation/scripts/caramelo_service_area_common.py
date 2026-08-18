#!/usr/bin/env python3
"""Funcoes comuns para Service Areas do Caramelo."""

from __future__ import annotations

import copy
import math
import re
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

try:
    from ruamel.yaml import YAML
except ImportError:  # pragma: no cover - mantem compatibilidade em PCs sem ruamel.
    YAML = None  # type: ignore
    import yaml as pyyaml

from caramelo_docking_common import (
    backup_file,
    normalize_dock_id,
    resolve_map_folder,
    service_areas_file,
)


POSE_KEYS = ("x", "y", "yaw")
DEFAULT_SERVICE_AREA_IDS = ["START", "FINISH", "WS1", "WS2", "WS3", "SH1", "PP1", "RT1"]
ALLOWED_TYPES = {
    "start",
    "finish",
    "workstation",
    "shelf",
    "rotating_table",
    "precision_placement",
}
TYPE_ALIASES = {
    "START": "start",
    "FINISH": "finish",
    "WS": "workstation",
    "WORKSTATION": "workstation",
    "SH": "shelf",
    "SHELF": "shelf",
    "PP": "precision_placement",
    "PRECISION_PLACEMENT": "precision_placement",
    "PRECISIONPLACEMENT": "precision_placement",
    "RT": "rotating_table",
    "ROTATING_TABLE": "rotating_table",
    "ROTATINGTABLE": "rotating_table",
}
TYPE_CODES = {
    "start": "START",
    "finish": "FINISH",
    "workstation": "WS",
    "shelf": "SH",
    "precision_placement": "PP",
    "rotating_table": "RT",
}
VALID_HEIGHTS = (0.00, 0.05, 0.10, 0.15)


def yaml_engine():
    if YAML is None:
        return None
    yaml = YAML()
    yaml.default_flow_style = False
    yaml.indent(mapping=2, sequence=4, offset=2)
    yaml.preserve_quotes = True
    return yaml


def load_yaml(path: Path, default: Optional[dict] = None) -> dict:
    if not path.exists():
        return copy.deepcopy(default or {})
    if YAML is None:
        with path.open("r", encoding="utf-8") as handle:
            data = pyyaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise ValueError(f"{path} precisa conter um dicionario YAML no topo.")
        return data

    with path.open("r", encoding="utf-8") as handle:
        data = yaml_engine().load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} precisa conter um dicionario YAML no topo.")
    return data


def write_yaml(path: Path, data: dict, make_backup: bool = False) -> Optional[Path]:
    backup = backup_file(path) if make_backup and path.exists() else None
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        if YAML is None:
            pyyaml.safe_dump(data, handle, sort_keys=False, allow_unicode=False, default_flow_style=False)
        else:
            yaml_engine().dump(data, handle)
    return backup


def service_area_path(map_folder: Path) -> Path:
    return service_areas_file(map_folder)


def docking_path(map_folder: Path) -> Path:
    return map_folder / "docking.yaml"


def ws_table_mapping_path(map_folder: Path) -> Path:
    return map_folder / "ws_table_mapping.yaml"


def pose_dict(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> Dict[str, float]:
    return {
        "x": round(float(x), 4),
        "y": round(float(y), 4),
        "yaw": round(normalized_yaw(float(yaw)), 6),
    }


def pose_from_value(value, field_name: str = "pose") -> Dict[str, float]:
    if isinstance(value, dict):
        try:
            return pose_dict(value["x"], value["y"], value["yaw"])
        except KeyError as exc:
            raise ValueError(f"{field_name} precisa conter x, y e yaw.") from exc
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return pose_dict(value[0], value[1], value[2])
    raise ValueError(f"{field_name} precisa ser dicionario x/y/yaw ou lista [x, y, yaw].")


def pose_to_list(value) -> List[float]:
    pose = pose_from_value(value)
    return [pose["x"], pose["y"], pose["yaw"]]


def normalized_yaw(yaw: float) -> float:
    return math.atan2(math.sin(float(yaw)), math.cos(float(yaw)))


def approach_pose_from_final(final_pose: dict, offset: float) -> Dict[str, float]:
    x, y, yaw = final_pose["x"], final_pose["y"], normalized_yaw(final_pose["yaw"])
    return pose_dict(
        x - float(offset) * math.cos(yaw),
        y - float(offset) * math.sin(yaw),
        yaw,
    )


def type_from_area_id(area_id: str) -> str:
    area_id = normalize_dock_id(area_id)
    if area_id == "START":
        return "start"
    if area_id == "FINISH":
        return "finish"
    if area_id.startswith("SH"):
        return "shelf"
    if area_id.startswith("PP"):
        return "precision_placement"
    if area_id.startswith("RT"):
        return "rotating_table"
    return "workstation"


def normalize_area_type(area_type: Optional[str], area_id: Optional[str] = None) -> str:
    if area_type:
        key = str(area_type).strip()
        key_upper = key.upper()
        key_lower = key.lower()
        normalized = TYPE_ALIASES.get(key_upper, key_lower)
        if normalized in ALLOWED_TYPES:
            return normalized
        raise ValueError(
            f"type invalido '{area_type}'. Use start, finish, workstation, "
            "shelf, rotating_table ou precision_placement."
        )
    if area_id:
        return type_from_area_id(area_id)
    return "workstation"


def type_code(area_type: str) -> str:
    return TYPE_CODES[normalize_area_type(area_type)]


def default_offset(area_type: str) -> float:
    area_type = normalize_area_type(area_type)
    if area_type in ("start", "finish"):
        return 0.0
    if area_type == "shelf":
        return 0.50
    return 0.45


def default_tolerances(area_type: str) -> Tuple[float, float]:
    area_type = normalize_area_type(area_type)
    if area_type == "precision_placement":
        return 0.025, 0.035
    if area_type in ("workstation", "shelf", "rotating_table"):
        return 0.03, 0.04
    return 0.05, 0.05


def default_table(area_type: str) -> Dict[str, float]:
    area_type = normalize_area_type(area_type)
    if area_type in ("start", "finish"):
        return {"width": 0.0, "depth": 0.0, "height": 0.0}
    return {"width": 0.80, "depth": 0.50, "height": 0.10}


def default_station(area_type: str) -> Dict[str, float]:
    return default_table(area_type)


def default_free_area(area_type: str) -> Dict[str, float]:
    if normalize_area_type(area_type) in ("start", "finish"):
        return {"width": 0.0, "depth": 0.0}
    return {"width": 0.85, "depth": 0.70}


def default_use_docking(area_type: str) -> bool:
    return normalize_area_type(area_type) not in ("start", "finish")


def default_manipulation(area_type: str) -> bool:
    return normalize_area_type(area_type) not in ("start", "finish")


def default_notes(area_id: str, area_type: str) -> str:
    area_type = normalize_area_type(area_type)
    if area_type == "start":
        return "START - pose inicial do robo."
    if area_type == "finish":
        return "FINISH - pose final do robo."
    return f"{area_id} - pose final do robo em frente a mesa"


def validate_height(height: float, area_type: str) -> float:
    height = round(float(height), 2)
    if normalize_area_type(area_type) in ("start", "finish"):
        return height
    if not any(abs(height - allowed) < 1.0e-9 for allowed in VALID_HEIGHTS):
        allowed = ", ".join(f"{value:.2f}" for value in VALID_HEIGHTS)
        raise ValueError(f"height {height:.2f} invalido. Use uma das alturas: {allowed}.")
    return height


def _table_from_entry(entry: dict, area_type: str) -> dict:
    table = entry.get("table")
    if not isinstance(table, dict):
        table = entry.get("station") if isinstance(entry.get("station"), dict) else {}
    defaults = default_table(area_type)
    return {
        "width": float(table.get("width", defaults["width"])),
        "depth": float(table.get("depth", defaults["depth"])),
        "height": validate_height(table.get("height", defaults["height"]), area_type),
    }


def _docking_from_entry(entry: dict, area_type: str) -> dict:
    docking = entry.get("docking") if isinstance(entry.get("docking"), dict) else {}
    old_approach = entry.get("approach") if isinstance(entry.get("approach"), dict) else {}
    linear_tolerance, angular_tolerance = default_tolerances(area_type)
    return {
        "approach_offset": float(docking.get("approach_offset", old_approach.get("offset", default_offset(area_type)))),
        "linear_tolerance": float(docking.get("linear_tolerance", old_approach.get("linear_tolerance", linear_tolerance))),
        "angular_tolerance": float(docking.get("angular_tolerance", old_approach.get("angular_tolerance", angular_tolerance))),
        "use_docking": bool(docking.get("use_docking", default_use_docking(area_type))),
    }


def dock_type_for_area(area_id: str) -> str:
    return f"caramelo_{normalize_dock_id(area_id).lower()}_dock"


def dock_plugin_for_area(area_id: str, area: dict) -> dict:
    docking = area["docking"]
    return {
        "plugin": "opennav_docking::SimpleNonChargingDock",
        "docking_threshold": float(docking["linear_tolerance"]),
        "staging_x_offset": -float(docking["approach_offset"]),
        "staging_yaw_offset": 0.0,
        "dock_direction": "forward",
        "rotate_to_dock": False,
        "use_external_detection_pose": False,
        "use_battery_status": False,
        "use_stall_detection": False,
    }


def default_service_area(area_id: str, area_type: Optional[str] = None, pose: Optional[dict] = None) -> dict:
    area_id = normalize_dock_id(area_id)
    area_type = normalize_area_type(area_type, area_id)
    final_pose = pose_from_value(pose or pose_dict(), f"{area_id}.final_robot_pose")
    table = default_table(area_type)
    docking = {
        "approach_offset": default_offset(area_type),
        "linear_tolerance": default_tolerances(area_type)[0],
        "angular_tolerance": default_tolerances(area_type)[1],
        "use_docking": default_use_docking(area_type),
    }
    entry = {
        "type": area_type,
        "frame_id": "map",
        "final_robot_pose": final_pose,
    }
    if table["width"] > 0.0 and table["depth"] > 0.0:
        entry["table"] = table
    entry["docking"] = docking
    entry["manipulation_enabled"] = default_manipulation(area_type)
    entry["notes"] = default_notes(area_id, area_type)
    return entry


def default_service_areas_doc(map_name: str, area_ids: Optional[Iterable[str]] = None) -> dict:
    ids = [normalize_dock_id(area_id) for area_id in (area_ids or DEFAULT_SERVICE_AREA_IDS)]
    return {
        "service_areas": {
            area_id: default_service_area(area_id)
            for area_id in ids
        },
    }


def load_service_areas(map_folder: Path) -> dict:
    path = service_area_path(map_folder)
    data = load_yaml(path)
    if "service_areas" not in data or not isinstance(data["service_areas"], dict):
        raise ValueError(f"{path} precisa conter a chave service_areas.")
    return data


def area_type_from_entry(area_id: str, entry: dict) -> str:
    return normalize_area_type(entry.get("type") or entry.get("kind"), area_id)


def normalized_service_area(area_id: str, entry: Optional[dict] = None, frame_id: str = "map") -> dict:
    area_id = normalize_dock_id(area_id)
    entry = copy.deepcopy(entry or {})
    area_type = area_type_from_entry(area_id, entry)
    final_pose = pose_from_value(
        entry.get("final_robot_pose", entry.get("final_pose", entry.get("pose", pose_dict()))),
        f"{area_id}.final_robot_pose",
    )
    docking = _docking_from_entry(entry, area_type)
    approach_pose = approach_pose_from_final(final_pose, docking["approach_offset"])
    table = _table_from_entry(entry, area_type)

    normalized = {
        "type": area_type,
        "type_code": type_code(area_type),
        "frame_id": str(entry.get("frame_id") or frame_id or "map"),
        "final_robot_pose": final_pose,
        "approach_pose": approach_pose,
        "table": table,
        "docking": docking,
        "manipulation_enabled": bool(entry.get("manipulation_enabled", default_manipulation(area_type))),
        "notes": str(entry.get("notes") or default_notes(area_id, area_type)),
    }

    # Campos de compatibilidade para os scripts ja existentes.
    normalized["pose"] = copy.deepcopy(final_pose)
    normalized["final_pose"] = copy.deepcopy(final_pose)
    normalized["station"] = copy.deepcopy(table)
    normalized["approach"] = {
        "offset": docking["approach_offset"],
        "docking_offset": 0.0,
        "linear_tolerance": docking["linear_tolerance"],
        "angular_tolerance": docking["angular_tolerance"],
    }
    normalized["free_area"] = copy.deepcopy(default_free_area(area_type))
    normalized["dock"] = {
        "enabled": docking["use_docking"],
        "id": area_id,
        "type": dock_type_for_area(area_id),
    }
    return normalized


def service_area_to_yaml_entry(area_id: str, area: dict) -> dict:
    area_type = normalize_area_type(area["type"], area_id)
    entry = {
        "type": area_type,
        "frame_id": area["frame_id"],
        "final_robot_pose": pose_from_value(area["final_robot_pose"]),
    }
    table = _table_from_entry(area, area_type)
    if table["width"] > 0.0 and table["depth"] > 0.0:
        entry["table"] = table
    entry["docking"] = {
        "approach_offset": float(area["docking"]["approach_offset"]),
        "linear_tolerance": float(area["docking"]["linear_tolerance"]),
        "angular_tolerance": float(area["docking"]["angular_tolerance"]),
        "use_docking": bool(area["docking"]["use_docking"]),
    }
    entry["manipulation_enabled"] = bool(area["manipulation_enabled"])
    if area.get("notes"):
        entry["notes"] = str(area["notes"])
    return entry


def is_zero_pose(value) -> bool:
    pose = pose_from_value(value)
    return abs(pose["x"]) < 1.0e-9 and abs(pose["y"]) < 1.0e-9 and abs(pose["yaw"]) < 1.0e-9


def validate_service_areas_data(data: dict) -> Tuple[List[str], List[str]]:
    errors: List[str] = []
    warnings: List[str] = []
    services = data.get("service_areas")
    if not isinstance(services, dict) or not services:
        return ["service_areas precisa existir e nao pode estar vazio."], warnings

    for area_id, entry in services.items():
        try:
            if not isinstance(entry, dict):
                raise ValueError("entrada precisa ser um dicionario.")
            normalized = normalized_service_area(area_id, entry, entry.get("frame_id", "map"))
            if normalize_dock_id(area_id) != area_id:
                warnings.append(f"{area_id}: nome sera normalizado para {normalize_dock_id(area_id)}.")
            if normalized["type"] not in ALLOWED_TYPES:
                raise ValueError(f"type invalido '{normalized['type']}'.")
            validate_height(normalized["table"]["height"], normalized["type"])
            if normalized["type"] not in ("start", "finish") and is_zero_pose(normalized["final_robot_pose"]):
                warnings.append(f"{area_id} ainda esta em 0,0,0. Salve no robo real com save_service_area_pose.")
        except Exception as exc:
            errors.append(f"{area_id}: {exc}")
    return errors, warnings


WS_TABLE_MAPPING_HEADER = """\
# Mapeamento centralizado da arena (1 arquivo POR MAPA):
#  - ws_to_table_pose: WS da tarefa -> group_state do braco (lido pelo task_planner)
#  - ws_to_dock_id:    WS da tarefa -> dock_id do docking.yaml (lido pelo executor de missao)
# GERADO a partir do service_areas.yaml (altura da mesa -> Mesa0/5/10/15;
# shelf -> MesaSh). Pode editar a vontade: o sync so ADICIONA chaves que
# faltam, nunca sobrescreve valores existentes.
"""


def task_name_for_area(area_id: str) -> Optional[str]:
    """WS1 -> WS_1, SH1 -> SH_1 (vocabulario dos YAMLs de tarefa). START/FINISH -> None."""
    match = re.match(r"^([A-Z]+)([0-9]+)$", normalize_dock_id(area_id))
    if not match:
        return None
    return f"{match.group(1)}_{match.group(2)}"


def table_pose_for_area(area_type: str, height: float) -> str:
    """group_state do braco para a area: shelf -> MesaSh; demais pela altura."""
    if normalize_area_type(area_type) == "shelf":
        return "MesaSh"
    return f"Mesa{int(round(float(height) * 100.0))}"


def sync_ws_table_mapping_from_service_areas(
    map_folder: Path, make_backup: bool = False
) -> Tuple[Path, Optional[Path], int]:
    """Cria/completa o ws_table_mapping.yaml do mapa (2026-08-17).

    O run_mission exige o arquivo e o task_planner aborta sem a entrada da WS
    ("Missing WS->Mesa mapping") — antes ele era feito A MAO e mapa novo
    nascia sem ele. Regra conservadora: entradas existentes NUNCA sao
    sobrescritas (mapas antigos tem ajustes manuais validados); so entram as
    chaves que faltam, derivadas do service_areas.yaml.
    """
    service_data = load_service_areas(map_folder)
    frame_id = service_data.get("frame_id", "map")
    path = ws_table_mapping_path(map_folder)
    creating = not path.exists()
    data = load_yaml(path)
    table_pose_map = data.setdefault("ws_to_table_pose", {})
    dock_id_map = data.setdefault("ws_to_dock_id", {})

    added = 0
    for area_id, entry in service_data["service_areas"].items():
        normalized = normalized_service_area(area_id, entry, frame_id)
        if normalized["type"] in ("start", "finish"):
            continue
        task_name = task_name_for_area(area_id)
        if task_name is None:
            continue
        if task_name not in table_pose_map:
            table_pose_map[task_name] = table_pose_for_area(
                normalized["type"], normalized["table"]["height"])
            added += 1
        if task_name not in dock_id_map:
            dock_id_map[task_name] = normalize_dock_id(area_id)
            added += 1

    if not creating and added == 0:
        return path, None, 0

    backup = write_yaml(path, data, make_backup=make_backup)
    if creating:
        path.write_text(WS_TABLE_MAPPING_HEADER + path.read_text(encoding="utf-8"),
                        encoding="utf-8")
    return path, backup, added


def sync_docking_from_service_areas(map_folder: Path, make_backup: bool = False) -> Tuple[Path, Optional[Path], int]:
    service_data = load_service_areas(map_folder)
    frame_id = service_data.get("frame_id", "map")
    docks = {}
    dock_plugins = {}
    for area_id, entry in service_data["service_areas"].items():
        normalized = normalized_service_area(area_id, entry, frame_id)
        # TODAS as areas entram no docking.yaml — inclusive START/FINISH
        # (use_docking: false). O GoToWS navega ate elas SEM dockar, mas le a
        # pose do docking.yaml ("pose nao encontrada no docking.yaml" era o
        # erro quando o sync as pulava, corrigido 2026-08-17). use_docking
        # decide o COMPORTAMENTO (dock vs navegacao pura) no service_areas
        # .yaml, nao a presenca da pose aqui.
        dock_type = dock_type_for_area(area_id)
        dock_plugins[dock_type] = dock_plugin_for_area(area_id, normalized)
        docks[area_id] = {
            "type": dock_type,
            "frame": normalized["frame_id"],
            "pose": pose_to_list(normalized["final_robot_pose"]),
            "id": area_id,
        }
    data = {
        "dock_plugins": dock_plugins,
        "docks": docks,
    }
    path = docking_path(map_folder)
    backup = write_yaml(path, data, make_backup=make_backup)
    # Carona proposital (2026-08-17): TODO caller que sincroniza o docking
    # (save CLI, GUI save_pose, init) tambem ganha o ws_table_mapping.yaml —
    # e o gancho unico que garante mapa novo completo para o run_mission.
    sync_ws_table_mapping_from_service_areas(map_folder, make_backup=make_backup)
    return path, backup, len(docks)


def update_service_area_pose(
    map_folder: Path,
    map_name: str,
    area_id: str,
    pose: dict,
    *,
    area_type: Optional[str] = None,
    frame_id: str = "map",
    approach_offset: Optional[float] = None,
    docking_offset: Optional[float] = None,
    linear_tolerance: Optional[float] = None,
    angular_tolerance: Optional[float] = None,
    station_width: Optional[float] = None,
    station_depth: Optional[float] = None,
    station_height: Optional[float] = None,
    free_width: Optional[float] = None,
    free_depth: Optional[float] = None,
    dock_type: Optional[str] = None,
    notes: Optional[str] = None,
    overwrite: bool = False,
) -> Tuple[Path, Optional[Path]]:
    del docking_offset, free_width, free_depth, dock_type
    area_id = normalize_dock_id(area_id)
    service_path = service_area_path(map_folder)
    if service_path.exists():
        data = load_yaml(service_path)
    else:
        data = default_service_areas_doc(map_name)
    data.setdefault("service_areas", {})
    services = data["service_areas"]
    if area_id in services and not overwrite:
        existing_pose = normalized_service_area(area_id, services[area_id], frame_id)["final_robot_pose"]
        if not is_zero_pose(existing_pose):
            raise RuntimeError(f"{area_id} ja existe em {service_path}. Use --overwrite para sobrescrever.")

    existing = services.get(area_id, {})
    inferred_type = normalize_area_type(area_type or existing.get("type"), area_id)
    final_pose = pose_from_value(pose, f"{area_id}.final_robot_pose")
    entry = normalized_service_area(area_id, existing or default_service_area(area_id, inferred_type), frame_id)
    entry["type"] = inferred_type
    entry["frame_id"] = frame_id
    entry["final_robot_pose"] = final_pose

    if approach_offset is not None:
        entry["docking"]["approach_offset"] = float(approach_offset)
    if linear_tolerance is not None:
        entry["docking"]["linear_tolerance"] = float(linear_tolerance)
    if angular_tolerance is not None:
        entry["docking"]["angular_tolerance"] = float(angular_tolerance)
    if inferred_type in ("start", "finish"):
        entry["docking"]["use_docking"] = False
        entry["table"] = default_table(inferred_type)

    if station_width is not None and inferred_type not in ("start", "finish"):
        entry["table"]["width"] = float(station_width)
    if station_depth is not None and inferred_type not in ("start", "finish"):
        entry["table"]["depth"] = float(station_depth)
    if station_height is not None and inferred_type not in ("start", "finish"):
        entry["table"]["height"] = validate_height(station_height, inferred_type)
    else:
        entry["table"]["height"] = validate_height(entry["table"]["height"], inferred_type)

    if notes is not None:
        entry["notes"] = notes

    services[area_id] = service_area_to_yaml_entry(area_id, entry)
    backup = write_yaml(service_path, data, make_backup=service_path.exists())
    return service_path, backup
