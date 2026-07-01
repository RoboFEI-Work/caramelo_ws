#!/usr/bin/env python3
"""Helpers for Caramelo service area files."""

from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

try:
    from ruamel.yaml import YAML
except ImportError:  # pragma: no cover - kept for developer machines without ruamel.
    YAML = None  # type: ignore
    import yaml as pyyaml

from caramelo_docking_common import (
    DEFAULT_DOCK_IDS,
    DEFAULT_DOCK_PLUGINS,
    backup_file,
    default_dock_type,
    infer_kind,
    normalize_dock_id,
    resolve_map_folder,
    service_areas_file,
)


POSE_KEYS = ("x", "y", "yaw")
VALID_TYPES = {"START", "FINISH", "WS", "SH", "PP", "RT"}


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


def pose_dict(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> Dict[str, float]:
    return {
        "x": round(float(x), 4),
        "y": round(float(y), 4),
        "yaw": round(float(yaw), 6),
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
    kind = infer_kind(area_id)
    return kind if kind in VALID_TYPES else "WS"


def default_offset(area_type: str) -> float:
    if area_type == "SH":
        return 0.90
    if area_type in ("PP", "RT"):
        return 0.80
    return 0.70


def default_tolerances(area_type: str) -> Tuple[float, float]:
    if area_type in ("PP", "RT"):
        return 0.03, 0.04
    if area_type == "SH":
        return 0.06, 0.06
    return 0.05, 0.05


def default_station(area_type: str) -> Dict[str, float]:
    if area_type in ("START", "FINISH"):
        return {"width": 0.0, "depth": 0.0, "height": 0.0}
    if area_type == "RT":
        return {"width": 0.60, "depth": 0.60, "height": 0.10}
    return {"width": 0.80, "depth": 0.50, "height": 0.10}


def default_free_area(area_type: str) -> Dict[str, float]:
    if area_type == "SH":
        return {"width": 0.90, "depth": 0.90}
    if area_type in ("PP", "RT"):
        return {"width": 0.85, "depth": 0.80}
    return {"width": 0.85, "depth": 0.70}


def default_notes(area_id: str, area_type: str) -> str:
    if area_type == "START":
        return "Inicio logico da arena. O robo deve iniciar o SLAM aqui."
    if area_type == "FINISH":
        return "Pose final da prova. Salvar no robo real antes da rodada."
    if area_type == "SH":
        return "Shelf. Distancia maior por seguranca. AJUSTAR_NO_ROBO."
    if area_type == "PP":
        return "Precise Placement. Validar com baixa velocidade."
    if area_type == "RT":
        return "Rotating Table. AJUSTAR_NO_ROBO."
    return f"{area_id} Workstation. AJUSTAR_NO_ROBO."


def default_service_area(area_id: str, area_type: Optional[str] = None, pose: Optional[dict] = None) -> dict:
    area_id = normalize_dock_id(area_id)
    area_type = (area_type or type_from_area_id(area_id)).upper()
    final_pose = pose_from_value(pose or pose_dict())
    offset = default_offset(area_type)
    linear_tolerance, angular_tolerance = default_tolerances(area_type)
    return {
        "type": area_type,
        "frame_id": "map",
        "pose": copy.deepcopy(final_pose),
        "final_pose": copy.deepcopy(final_pose),
        "approach_pose": approach_pose_from_final(final_pose, offset),
        "approach": {
            "offset": offset,
            "docking_offset": 0.0,
            "linear_tolerance": linear_tolerance,
            "angular_tolerance": angular_tolerance,
        },
        "station": default_station(area_type),
        "free_area": default_free_area(area_type),
        "dock": {
            "enabled": True,
            "id": area_id,
            "type": default_dock_type(area_id),
        },
        "notes": default_notes(area_id, area_type),
    }


def default_service_areas_doc(map_name: str, dock_ids: Optional[Iterable[str]] = None) -> dict:
    ids = [normalize_dock_id(area_id) for area_id in (dock_ids or DEFAULT_DOCK_IDS)]
    return {
        "map_name": map_name,
        "frame_id": "map",
        "version": 1,
        "defaults": {
            "robot_footprint": {"length": 0.71, "width": 0.47},
            "approach_offset": 0.70,
            "free_area": {"width": 0.85, "depth": 0.70},
        },
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
    raw_type = entry.get("type") or entry.get("kind") or type_from_area_id(area_id)
    return str(raw_type).upper()


def normalized_service_area(area_id: str, entry: Optional[dict] = None, frame_id: str = "map") -> dict:
    area_id = normalize_dock_id(area_id)
    entry = copy.deepcopy(entry or {})
    area_type = area_type_from_entry(area_id, entry)
    if area_type not in VALID_TYPES:
        raise ValueError(f"{area_id}: type invalido '{area_type}'.")

    final_pose = pose_from_value(entry.get("final_pose", entry.get("pose", pose_dict())), f"{area_id}.final_pose")
    pose = pose_from_value(entry.get("pose", final_pose), f"{area_id}.pose")
    approach = entry.get("approach") if isinstance(entry.get("approach"), dict) else {}
    offset = float(approach.get("offset", default_offset(area_type)))
    linear_tolerance, angular_tolerance = default_tolerances(area_type)
    approach_pose = pose_from_value(
        entry.get("approach_pose", approach_pose_from_final(final_pose, offset)),
        f"{area_id}.approach_pose",
    )
    station = entry.get("station") if isinstance(entry.get("station"), dict) else default_station(area_type)
    free_area = entry.get("free_area") if isinstance(entry.get("free_area"), dict) else default_free_area(area_type)
    dock = entry.get("dock") if isinstance(entry.get("dock"), dict) else {}

    entry["type"] = area_type
    entry["frame_id"] = str(entry.get("frame_id") or frame_id or "map")
    entry["pose"] = pose
    entry["final_pose"] = final_pose
    entry["approach_pose"] = approach_pose
    entry["approach"] = {
        "offset": offset,
        "docking_offset": float(approach.get("docking_offset", 0.0)),
        "linear_tolerance": float(approach.get("linear_tolerance", linear_tolerance)),
        "angular_tolerance": float(approach.get("angular_tolerance", angular_tolerance)),
    }
    entry["station"] = {
        "width": float(station.get("width", default_station(area_type)["width"])),
        "depth": float(station.get("depth", default_station(area_type)["depth"])),
        "height": float(station.get("height", default_station(area_type)["height"])),
    }
    entry["free_area"] = {
        "width": float(free_area.get("width", default_free_area(area_type)["width"])),
        "depth": float(free_area.get("depth", default_free_area(area_type)["depth"])),
    }
    entry["dock"] = {
        "enabled": bool(dock.get("enabled", True)),
        "id": str(dock.get("id") or area_id),
        "type": str(dock.get("type") or default_dock_type(area_id)),
    }
    entry.setdefault("notes", default_notes(area_id, area_type))
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
            area_id = normalize_dock_id(area_id)
            if not isinstance(entry, dict):
                raise ValueError("entrada precisa ser um dicionario.")
            normalized = normalized_service_area(area_id, entry, data.get("frame_id", "map"))
            if area_id == "START" and not is_zero_pose(normalized["final_pose"]):
                warnings.append("START nao esta em 0,0,0. Isso quebra a convencao atual da arena.")
            if area_id != "START" and is_zero_pose(normalized["final_pose"]):
                warnings.append(f"{area_id} ainda esta em 0,0,0. Salvar no robo real com save_service_area_pose.")
            dock_id = normalize_dock_id(normalized["dock"]["id"])
            if dock_id != area_id:
                warnings.append(f"{area_id}: dock.id aponta para {dock_id}.")
        except Exception as exc:
            errors.append(f"{area_id}: {exc}")
    return errors, warnings


def sync_docking_from_service_areas(map_folder: Path, make_backup: bool = False) -> Tuple[Path, Optional[Path], int]:
    service_data = load_service_areas(map_folder)
    frame_id = service_data.get("frame_id", "map")
    docks = {}
    for area_id, entry in service_data["service_areas"].items():
        normalized = normalized_service_area(area_id, entry, frame_id)
        if not normalized["dock"]["enabled"]:
            continue
        dock_id = normalize_dock_id(normalized["dock"]["id"])
        docks[dock_id] = {
            "type": normalized["dock"]["type"],
            "frame": normalized["frame_id"],
            "pose": pose_to_list(normalized["final_pose"]),
            "id": dock_id,
        }
    data = {
        "dock_plugins": copy.deepcopy(DEFAULT_DOCK_PLUGINS),
        "docks": docks,
    }
    path = docking_path(map_folder)
    backup = write_yaml(path, data, make_backup=make_backup)
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
    area_id = normalize_dock_id(area_id)
    service_path = service_area_path(map_folder)
    if service_path.exists():
        data = load_yaml(service_path)
    else:
        data = default_service_areas_doc(map_name)
    data.setdefault("map_name", map_name)
    data.setdefault("frame_id", frame_id)
    data.setdefault("version", 1)
    data.setdefault("service_areas", {})
    services = data["service_areas"]
    if area_id in services and not overwrite:
        raise RuntimeError(f"{area_id} ja existe em {service_path}. Use --yes para sobrescrever.")

    existing = services.get(area_id, {})
    inferred_type = (area_type or existing.get("type") or existing.get("kind") or type_from_area_id(area_id)).upper()
    final_pose = pose_from_value(pose, f"{area_id}.pose")
    entry = normalized_service_area(area_id, existing or default_service_area(area_id, inferred_type), data.get("frame_id", frame_id))
    entry["type"] = inferred_type
    entry["frame_id"] = frame_id
    entry["pose"] = copy.deepcopy(final_pose)
    entry["final_pose"] = copy.deepcopy(final_pose)

    approach = entry.get("approach", {})
    if approach_offset is not None:
        approach["offset"] = float(approach_offset)
    if docking_offset is not None:
        approach["docking_offset"] = float(docking_offset)
    if linear_tolerance is not None:
        approach["linear_tolerance"] = float(linear_tolerance)
    if angular_tolerance is not None:
        approach["angular_tolerance"] = float(angular_tolerance)
    entry["approach"] = approach
    entry["approach_pose"] = approach_pose_from_final(final_pose, float(approach["offset"]))

    station = entry.get("station", default_station(inferred_type))
    if station_width is not None:
        station["width"] = float(station_width)
    if station_depth is not None:
        station["depth"] = float(station_depth)
    if station_height is not None:
        station["height"] = float(station_height)
    entry["station"] = station

    free_area = entry.get("free_area", default_free_area(inferred_type))
    if free_width is not None:
        free_area["width"] = float(free_width)
    if free_depth is not None:
        free_area["depth"] = float(free_depth)
    entry["free_area"] = free_area

    dock = entry.get("dock", {})
    dock["enabled"] = bool(dock.get("enabled", True))
    dock["id"] = dock.get("id") or area_id
    dock["type"] = dock_type or dock.get("type") or default_dock_type(area_id)
    entry["dock"] = dock
    if notes is not None:
        entry["notes"] = notes

    services[area_id] = entry
    backup = write_yaml(service_path, data, make_backup=service_path.exists())
    return service_path, backup
