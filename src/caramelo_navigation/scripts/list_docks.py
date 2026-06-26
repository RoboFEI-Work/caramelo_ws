#!/usr/bin/env python3
import argparse
import sys

from caramelo_docking_common import docking_file, load_yaml, resolve_map_folder, service_areas_file


def _fmt_pose(pose):
    if not isinstance(pose, list):
        return "[]"
    return "[" + ", ".join(f"{float(value):.3f}" for value in pose) + "]"


def main() -> int:
    parser = argparse.ArgumentParser(description="Lista docks salvos no mapa.")
    parser.add_argument("--map-name", required=True, help="Nome da pasta do mapa.")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    args = parser.parse_args()
    try:
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
        dock_path = docking_file(map_folder)
        service_path = service_areas_file(map_folder)
        dock_data = load_yaml(dock_path)
        service_data = load_yaml(service_path) if service_path.exists() else {}
    except Exception as exc:
        print(f"Erro ao listar docks: {exc}", file=sys.stderr)
        return 1
    services = service_data.get("service_areas", {}) if isinstance(service_data, dict) else {}
    docks = dock_data.get("docks", {}) if isinstance(dock_data, dict) else {}
    print(f"Mapa: {args.map_name}")
    print(f"Arquivo: {dock_path}")
    if service_path.exists():
        print(f"Service areas: {service_path}")
    print("")
    if not docks:
        print("Nenhum dock encontrado. Rode init_map_docking ou save_dock_pose primeiro.")
        return 0
    for dock_id, dock in docks.items():
        service = services.get(dock_id, {}) if isinstance(services, dict) else {}
        extras = []
        for key in ("kind", "table_height_m", "width_m", "depth_m", "diameter_m", "precision"):
            if key in service:
                extras.append(f"{key}={service[key]}")
        extras_text = "  " + " ".join(extras) if extras else ""
        print(f"{dock_id:<7} type={dock.get('type', ''):<32} pose={_fmt_pose(dock.get('pose', []))}{extras_text}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
