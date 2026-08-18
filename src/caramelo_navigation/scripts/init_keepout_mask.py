#!/usr/bin/env python3
"""Cria a mascara de Keepout Filter (parede virtual, RK-04) para um mapa.

Gera um canvas TODO-LIVRE (branco) com a mesma geometria do map.pgm/map.yaml
do mapa, mais o keepout_mask.yaml correspondente. A partir daí, basta abrir o
keepout_mask.pgm num editor de imagem e pintar de PRETO as regioes que o robô
NAO pode pisar (ex.: fita zebrada de piso, que o LiDAR nao enxerga).

Nao depende de numpy/PIL — le e escreve PGM (P5) em Python puro.
"""
import argparse
import os
import sys


def _resolvedor_compartilhado():
    """caramelo_docking_common.resolve_map_folder, ou None se nao der para importar.

    O modulo mora ao lado deste arquivo (install(FILES) no mesmo lib/), mas com
    colcon --symlink-install o script executado e um LINK: dependendo de como
    foi invocado, o diretorio que entra no sys.path pode ser o do link ou o do
    alvo. Por isso os dois entram.
    """
    for pasta in (
        os.path.dirname(os.path.realpath(__file__)),
        os.path.dirname(os.path.abspath(__file__)),
    ):
        if pasta not in sys.path:
            sys.path.insert(0, pasta)
    try:
        from caramelo_docking_common import resolve_map_folder
    except Exception:  # noqa: BLE001 - rodando solto, sem o pacote ao lado
        return None
    return resolve_map_folder


def _resolve_map_folder(map_name, map_dir):
    """Retorna a pasta do mapa contendo map.yaml.

    Usa a MESMA regra do resto do sistema (caramelo_docking_common):
    --map-dir, depois $CARAMELO_MAPS_DIR, depois a arvore FONTE deduzida do
    workspace, e so' entao o share instalado. A interface grafica chama este
    script de QUALQUER diretorio, entao a busca nao pode depender do cwd.
    Antes daqui a lista era caseira e terminava num '~/caramelo_ws' cravado:
    em qualquer usuario/workspace com outro nome o script falhava com
    "nao encontrei a pasta do mapa" mesmo com o mapa ali, e $CARAMELO_MAPS_DIR
    era ignorado.

    O bloco local continua como ultimo recurso, para o caso do arquivo rodando
    solto (copiado para fora do pacote), sem caramelo_docking_common ao lado.
    """
    resolve_map_folder = _resolvedor_compartilhado()
    if resolve_map_folder is not None:
        try:
            return str(resolve_map_folder(map_name, map_dir))
        except FileNotFoundError:
            pass  # cai no fallback, que ainda testa caminhos relativos ao arquivo

    candidates = []
    if map_dir:
        map_dir = os.path.expanduser(map_dir)
        candidates.append(os.path.join(map_dir, map_name))
        candidates.append(map_dir)  # o proprio map_dir ja pode ser a pasta do mapa
    env_dir = os.environ.get("CARAMELO_MAPS_DIR")
    if env_dir:
        candidates.append(os.path.join(os.path.expanduser(env_dir), map_name))
    # realpath: com --symlink-install o arquivo em install/ aponta para o src,
    # e so' o realpath leva de volta a arvore fonte do workspace.
    for here in (os.path.dirname(os.path.realpath(__file__)),
                 os.path.dirname(os.path.abspath(__file__))):
        for root in (
            os.path.join(here, "..", "..", "caramelo_mapping", "maps"),
            os.path.join(here, "..", "..", "..", "src", "caramelo_mapping", "maps"),
        ):
            candidates.append(os.path.join(root, map_name))
    for candidate in candidates:
        if os.path.isfile(os.path.join(candidate, "map.yaml")):
            return os.path.abspath(candidate)
    raise FileNotFoundError(
        f"Nao encontrei a pasta do mapa '{map_name}' com map.yaml. "
        f"Passe --map-dir apontando para caramelo_mapping/maps."
    )


def _read_pgm_header(pgm_path):
    """Le largura e altura de um PGM binario (P5), pulando comentarios."""
    with open(pgm_path, "rb") as pgm_file:
        magic = pgm_file.read(2)
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"{pgm_path} nao e' PGM P5/P2 (magic={magic!r}).")
        tokens = []
        while len(tokens) < 3:
            token = b""
            while True:
                char = pgm_file.read(1)
                if not char:
                    raise ValueError(f"Cabecalho PGM incompleto em {pgm_path}.")
                if char in b"#":
                    pgm_file.readline()  # descarta o resto da linha de comentario
                    break
                if char.isspace():
                    if token:
                        tokens.append(token)
                    break
                token += char
        width, height, _maxval = (int(tokens[0]), int(tokens[1]), int(tokens[2]))
    return width, height


def _read_map_yaml(map_yaml_path):
    """Le resolution/origin do map.yaml sem depender de PyYAML (parse simples)."""
    resolution = None
    origin = None
    with open(map_yaml_path, "r", encoding="utf-8") as yaml_file:
        for line in yaml_file:
            stripped = line.strip()
            if stripped.startswith("resolution:"):
                resolution = float(stripped.split(":", 1)[1].strip())
            elif stripped.startswith("origin:"):
                raw = stripped.split(":", 1)[1].strip().strip("[]")
                origin = [float(part) for part in raw.split(",")]
    if resolution is None or origin is None:
        raise ValueError(f"resolution/origin ausentes em {map_yaml_path}.")
    return resolution, origin


def _write_free_pgm(pgm_path, width, height):
    """Escreve um PGM P5 todo branco (255 = livre)."""
    with open(pgm_path, "wb") as pgm_file:
        pgm_file.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        pgm_file.write(bytes([255]) * (width * height))


def _write_mask_yaml(yaml_path, resolution, origin):
    with open(yaml_path, "w", encoding="utf-8") as yaml_file:
        yaml_file.write("image: keepout_mask.pgm\n")
        # 'scale': pixel preto (0) -> celula letal (keepout); branco (255) -> livre.
        yaml_file.write("mode: scale\n")
        yaml_file.write(f"resolution: {resolution}\n")
        yaml_file.write(f"origin: [{origin[0]}, {origin[1]}, {origin[2] if len(origin) > 2 else 0.0}]\n")
        yaml_file.write("negate: 0\n")
        yaml_file.write("occupied_thresh: 0.65\n")
        yaml_file.write("free_thresh: 0.196\n")


def main():
    parser = argparse.ArgumentParser(
        description="Cria keepout_mask.{pgm,yaml} (canvas todo-livre) para um mapa."
    )
    parser.add_argument("--map-name", required=True, help="Nome da pasta do mapa.")
    parser.add_argument("--map-dir", default=None,
                        help="Raiz de caramelo_mapping/maps ou a propria pasta do mapa.")
    parser.add_argument("--force", action="store_true",
                        help="Sobrescreve keepout_mask.{pgm,yaml} se ja existirem.")
    args = parser.parse_args()

    try:
        map_folder = _resolve_map_folder(args.map_name, args.map_dir)
        map_yaml = os.path.join(map_folder, "map.yaml")
        map_pgm = os.path.join(map_folder, "map.pgm")
        if not os.path.isfile(map_pgm):
            raise FileNotFoundError(f"map.pgm nao encontrado em {map_folder}.")

        mask_pgm = os.path.join(map_folder, "keepout_mask.pgm")
        mask_yaml = os.path.join(map_folder, "keepout_mask.yaml")
        if (os.path.exists(mask_pgm) or os.path.exists(mask_yaml)) and not args.force:
            print(f"keepout_mask ja existe em {map_folder} (use --force para sobrescrever). "
                  "Nada alterado.")
            return 0

        width, height = _read_pgm_header(map_pgm)
        resolution, origin = _read_map_yaml(map_yaml)
        _write_free_pgm(mask_pgm, width, height)
        _write_mask_yaml(mask_yaml, resolution, origin)
    except Exception as exc:  # noqa: BLE001
        print(f"Erro ao criar keepout_mask: {exc}", file=sys.stderr)
        return 1

    print("Mascara de keepout criada (canvas todo-livre, sem nenhum bloqueio):")
    print(f"  map_folder: {map_folder}")
    print(f"  keepout_mask.pgm: {width}x{height} px, resolution {resolution} m/cel")
    print(f"  keepout_mask.yaml: origin {origin}")
    print("Pinte de PRETO (0) as paredes virtuais no keepout_mask.pgm e suba a")
    print("navegacao com use_keepout:=true.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
