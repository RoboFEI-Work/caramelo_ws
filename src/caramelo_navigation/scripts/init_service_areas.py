#!/usr/bin/env python3
import argparse
import sys

from caramelo_service_area_common import (
    default_service_areas_doc,
    service_area_path,
    sync_docking_from_service_areas,
    write_yaml,
)
from caramelo_docking_common import DEFAULT_DOCK_IDS, normalize_dock_id, resolve_map_folder


def main() -> int:
    parser = argparse.ArgumentParser(description="Inicializa service_areas.yaml de um mapa.")
    parser.add_argument("--map-name", required=True, help="Nome da pasta do mapa.")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    parser.add_argument("--areas", nargs="*", default=DEFAULT_DOCK_IDS, help="Lista de areas. Ex: START WS1 WS2 SH1 PP1 RT1.")
    parser.add_argument("--force-template", action="store_true", help="Recria service_areas.yaml mesmo se ja existir.")
    parser.add_argument("--sync-docking", action="store_true", help="Sincroniza docking.yaml apos criar/validar.")
    args = parser.parse_args()

    try:
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
        path = service_area_path(map_folder)
        created = False
        if not path.exists() or args.force_template:
            area_ids = [normalize_dock_id(area_id) for area_id in args.areas]
            write_yaml(path, default_service_areas_doc(args.map_name, area_ids), make_backup=path.exists())
            created = True
        docking_result = None
        if args.sync_docking:
            docking_result = sync_docking_from_service_areas(map_folder, make_backup=True)
    except Exception as exc:
        print(f"Erro ao inicializar service areas: {exc}", file=sys.stderr)
        return 1

    print("Service Areas prontas:")
    print(f"  map_name: {args.map_name}")
    print(f"  map_folder: {map_folder}")
    print(f"  service_areas: {path} ({'criado' if created else 'ja existia'})")
    if docking_result:
        docking_path, backup, count = docking_result
        print(f"  docking: {docking_path} ({count} docks sincronizados)")
        if backup:
            print(f"  docking_backup: {backup}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
