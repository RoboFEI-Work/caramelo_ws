#!/usr/bin/env python3
import argparse
import sys

from caramelo_docking_common import resolve_map_folder
from caramelo_service_area_common import load_service_areas, normalized_service_area


def _fmt_pose(pose: dict) -> str:
    return f"({pose['x']:.3f}, {pose['y']:.3f}, {pose['yaw']:.3f})"


def main() -> int:
    parser = argparse.ArgumentParser(description="Lista Service Areas salvas no mapa.")
    parser.add_argument("--map-name", required=True, help="Nome da pasta do mapa.")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    args = parser.parse_args()

    try:
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
        data = load_service_areas(map_folder)
    except Exception as exc:
        print(f"Erro ao listar service areas: {exc}", file=sys.stderr)
        return 1

    print(f"Mapa: {args.map_name}")
    print(f"Arquivo: {map_folder / 'service_areas.yaml'}")
    print("")
    print(f"{'AREA':<8} {'TIPO':<7} {'FINAL':<28} {'APROX':<28} {'DOCK':<32} NOTES")
    print("-" * 130)
    for area_id, entry in data["service_areas"].items():
        area = normalized_service_area(area_id, entry, data.get("frame_id", "map"))
        dock = area["dock"]
        print(
            f"{area_id:<8} {area['type']:<7} "
            f"{_fmt_pose(area['final_pose']):<28} "
            f"{_fmt_pose(area['approach_pose']):<28} "
            f"{dock['type']:<32} {area.get('notes', '')}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
