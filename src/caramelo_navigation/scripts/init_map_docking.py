#!/usr/bin/env python3
import argparse
import sys

from caramelo_docking_common import ensure_docking_files, resolve_map_folder


def main() -> int:
    parser = argparse.ArgumentParser(description="Inicializa arquivos de docking em uma pasta de mapa.")
    parser.add_argument("--map-name", required=True, help="Nome da pasta do mapa.")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    args = parser.parse_args()
    try:
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
        dock_path, service_path, created_docking, created_services = ensure_docking_files(map_folder)
    except Exception as exc:
        print(f"Erro ao inicializar docking: {exc}", file=sys.stderr)
        return 1
    print("Arquivos de docking prontos:")
    print(f"  map_name: {args.map_name}")
    print(f"  map_folder: {map_folder}")
    print(f"  docking: {dock_path} ({'criado' if created_docking else 'ja existia'})")
    print(f"  service_areas: {service_path} ({'criado' if created_services else 'ja existia'})")
    print("Voce pode editar/remover/adicionar START, FINISH, WS, SH, PP e RT conforme a arena real.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
