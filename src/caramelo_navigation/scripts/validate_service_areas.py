#!/usr/bin/env python3
import argparse
import sys

from caramelo_docking_common import resolve_map_folder
from caramelo_service_area_common import load_service_areas, sync_docking_from_service_areas, validate_service_areas_data


def main() -> int:
    parser = argparse.ArgumentParser(description="Valida service_areas.yaml e opcionalmente sincroniza docking.yaml.")
    parser.add_argument("--map-name", required=True, help="Nome da pasta do mapa.")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    parser.add_argument("--sync-docking", action="store_true", help="Atualiza docking.yaml a partir de service_areas.yaml.")
    args = parser.parse_args()

    try:
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
        data = load_service_areas(map_folder)
        errors, warnings = validate_service_areas_data(data)
        docking_result = None
        if not errors and args.sync_docking:
            docking_result = sync_docking_from_service_areas(map_folder, make_backup=True)
    except Exception as exc:
        print(f"Erro ao validar service areas: {exc}", file=sys.stderr)
        return 1

    print(f"Mapa: {args.map_name}")
    print(f"Arquivo: {map_folder / 'service_areas.yaml'}")
    if warnings:
        print("\nAvisos:")
        for warning in warnings:
            print(f"  - {warning}")
    if errors:
        print("\nErros:")
        for error in errors:
            print(f"  - {error}")
        return 1
    if docking_result:
        docking_path, backup, count = docking_result
        print(f"\nDocking sincronizado: {docking_path} ({count} docks)")
        if backup:
            print(f"Backup: {backup}")
    print("\nService Areas validas.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
