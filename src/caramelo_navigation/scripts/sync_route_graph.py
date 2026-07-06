#!/usr/bin/env python3
"""Generate Nav2 Route GeoJSON graphs from Caramelo service area YAML files."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

from caramelo_service_area_common import (
    is_zero_pose,
    load_service_areas,
    normalized_service_area,
)


DATE_GENERATED = "2026-07-06"


def pose_for_route(area: dict) -> dict:
    if area["docking"]["use_docking"]:
        return area["approach_pose"]
    return area["final_robot_pose"]


def service_area_nodes(map_folder: Path) -> list[dict]:
    data = load_service_areas(map_folder)
    nodes = []
    for area_id, entry in sorted(data["service_areas"].items()):
        area = normalized_service_area(area_id, entry, entry.get("frame_id", "map"))
        if is_zero_pose(area["final_robot_pose"]):
            continue
        route_pose = pose_for_route(area)
        nodes.append(
            {
                "area_id": area_id,
                "area_type": area["type"],
                "frame_id": area["frame_id"],
                "route_pose": route_pose,
                "final_robot_pose": area["final_robot_pose"],
                "approach_pose": area["approach_pose"],
                "use_docking": area["docking"]["use_docking"],
            }
        )
    return nodes


def node_feature(node_id: int, node: dict) -> dict:
    route_pose = node["route_pose"]
    return {
        "type": "Feature",
        "properties": {
            "id": node_id,
            "frame": node["frame_id"],
            "metadata": {
                "service_area": {
                    "id": node["area_id"],
                    "type": node["area_type"],
                    "use_docking": node["use_docking"],
                    "route_pose": node["route_pose"],
                    "final_robot_pose": node["final_robot_pose"],
                    "approach_pose": node["approach_pose"],
                }
            },
        },
        "geometry": {
            "type": "Point",
            "coordinates": [route_pose["x"], route_pose["y"]],
        },
    }


def distance(a: dict, b: dict) -> float:
    return math.hypot(
        a["route_pose"]["x"] - b["route_pose"]["x"],
        a["route_pose"]["y"] - b["route_pose"]["y"],
    )


def edge_pairs(nodes: list[dict], max_neighbors: int) -> list[tuple[int, int]]:
    if len(nodes) < 2:
        return []
    directed = set()
    for start_index, start_node in enumerate(nodes):
        candidates = sorted(
            (
                (distance(start_node, end_node), end_index)
                for end_index, end_node in enumerate(nodes)
                if end_index != start_index
            ),
            key=lambda item: item[0],
        )
        for _, end_index in candidates[:max_neighbors]:
            directed.add((start_index, end_index))
            directed.add((end_index, start_index))
    return sorted(directed)


def edge_feature(edge_id: int, start_id: int, end_id: int, start: dict, end: dict) -> dict:
    start_pose = start["route_pose"]
    end_pose = end["route_pose"]
    return {
        "type": "Feature",
        "properties": {
            "id": edge_id,
            "startid": start_id,
            "endid": end_id,
            "cost": round(distance(start, end), 4),
            "metadata": {
                "from": start["area_id"],
                "to": end["area_id"],
            },
        },
        "geometry": {
            "type": "MultiLineString",
            "coordinates": [
                [
                    [start_pose["x"], start_pose["y"]],
                    [end_pose["x"], end_pose["y"]],
                ]
            ],
        },
    }


def build_geojson(map_folder: Path, max_neighbors: int) -> dict:
    nodes = service_area_nodes(map_folder)
    features = []
    node_ids = {index: index + 1 for index in range(len(nodes))}

    for index, node in enumerate(nodes):
        features.append(node_feature(node_ids[index], node))

    edge_id = len(nodes) + 1
    for start_index, end_index in edge_pairs(nodes, max_neighbors):
        features.append(
            edge_feature(
                edge_id,
                node_ids[start_index],
                node_ids[end_index],
                nodes[start_index],
                nodes[end_index],
            )
        )
        edge_id += 1

    return {
        "type": "FeatureCollection",
        "name": f"{map_folder.name}_route_graph",
        "crs": {
            "type": "name",
            "properties": {"name": "urn:ogc:def:crs:EPSG::3857"},
        },
        "date_generated": DATE_GENERATED,
        "features": features,
    }


def default_maps_root() -> Path:
    return Path(__file__).resolve().parents[2] / "caramelo_mapping" / "maps"


def map_folders_from_args(args) -> Iterable[Path]:
    if args.map_folder:
        for folder in args.map_folder:
            yield folder.expanduser().resolve()
        return

    maps_root = args.maps_root.expanduser().resolve()
    for candidate in sorted(maps_root.iterdir()):
        if (candidate / "service_areas.yaml").exists():
            yield candidate


def write_route_graph(map_folder: Path, max_neighbors: int) -> Path:
    graph = build_geojson(map_folder, max_neighbors)
    output = map_folder / "route_graph.geojson"
    with output.open("w", encoding="utf-8") as handle:
        json.dump(graph, handle, indent=2)
        handle.write("\n")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--maps-root",
        type=Path,
        default=default_maps_root(),
        help="Directory containing map folders. Used when --map-folder is omitted.",
    )
    parser.add_argument(
        "--map-folder",
        action="append",
        type=Path,
        help="Specific map folder to update. Can be passed more than once.",
    )
    parser.add_argument(
        "--max-neighbors",
        type=int,
        default=3,
        help="Number of nearest service areas connected from each route node.",
    )
    args = parser.parse_args()

    for map_folder in map_folders_from_args(args):
        output = write_route_graph(map_folder, args.max_neighbors)
        print(f"Generated {output}")


if __name__ == "__main__":
    main()
