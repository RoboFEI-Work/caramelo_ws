"""Leitura das arenas salvas em disco, para o painel web.

Mesma fonte de verdade da GUI Qt: a árvore FONTE de caramelo_mapping/maps. O
painel lê os arquivos direto, sem passar por ROS -- dá para inspecionar uma
arena com o robô desligado, e o mapa não precisa trafegar como OccupancyGrid.

O mapa NUNCA é streamado: vai por REST como PNG, com ETag, e o navegador só
baixa de novo quando o arquivo muda. Um OccupancyGrid de 697x1039 repetido no
Wi-Fi de competição tiraria banda da navegação.
"""
import io
import os
from pathlib import Path

import yaml

_PLACEHOLDER = 1e-6


def pasta_dos_mapas() -> Path:
    """A mesma resolução que a GUI usa: fonte do workspace, não o share."""
    env = os.environ.get("CARAMELO_MAPS_DIR")
    if env and Path(env).is_dir():
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("caramelo_mapping")
        if "/install/" in share:
            fonte = Path(share.split("/install/")[0]) / "src/caramelo_mapping/maps"
            if fonte.is_dir():
                return fonte
        if (Path(share) / "maps").is_dir():
            return Path(share) / "maps"
    except Exception:
        pass
    return Path.home() / "caramelo_ws/src/caramelo_mapping/maps"


def listar() -> list:
    raiz = pasta_dos_mapas()
    if not raiz.is_dir():
        return []
    return sorted(
        d.name for d in raiz.iterdir()
        if d.is_dir() and not d.name.startswith(".") and (d / "map.yaml").is_file())


def _placeholder(x, y, yaw) -> bool:
    return abs(x) < _PLACEHOLDER and abs(y) < _PLACEHOLDER and abs(yaw) < _PLACEHOLDER


def _ler_yaml(caminho: Path) -> dict:
    try:
        with caminho.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def metadados(nome: str) -> dict:
    """Tudo que a arena guarda, no formato que o canvas do painel consome."""
    pasta = pasta_dos_mapas() / nome
    mapa = _ler_yaml(pasta / "map.yaml")
    if not mapa:
        return {"erro": f"Arena '{nome}' nao tem map.yaml."}

    origem = mapa.get("origin", [0.0, 0.0, 0.0])
    dados = {
        "nome": nome,
        "resolucao": float(mapa.get("resolution", 0.05)),
        "origem": [float(origem[0]), float(origem[1])],
        "docks": [],
        "areas": [],
        "waypoints": [],
        "tem_keepout": (pasta / "keepout_mask.pgm").is_file(),
        "avisos": [],
    }

    largura = altura = 0
    try:
        from PIL import Image
        with Image.open(pasta / mapa.get("image", "map.pgm")) as img:
            largura, altura = img.size
    except Exception as exc:
        dados["avisos"].append(f"Nao consegui abrir a imagem do mapa ({exc}).")
    dados["largura_px"] = largura
    dados["altura_px"] = altura

    sem_pose = 0
    for ident, info in (_ler_yaml(pasta / "docking.yaml").get("docks") or {}).items():
        pose = info.get("pose") or [0.0, 0.0, 0.0]
        x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
        ph = _placeholder(x, y, yaw)
        sem_pose += ph
        dados["docks"].append(
            {"id": ident, "x": x, "y": y, "yaw": yaw, "sem_pose": ph})

    for ident, info in (_ler_yaml(pasta / "service_areas.yaml").get("service_areas") or {}).items():
        p = info.get("final_robot_pose") or {}
        x, y, yaw = (float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("yaw", 0.0)))
        dados["areas"].append({
            "id": ident, "x": x, "y": y, "yaw": yaw,
            "tipo": info.get("type", ""), "sem_pose": _placeholder(x, y, yaw)})

    for ident, info in (_ler_yaml(pasta / "waypoints.yaml").get("waypoints") or {}).items():
        dados["waypoints"].append({
            "id": ident,
            "x": float(info.get("x", 0.0)),
            "y": float(info.get("y", 0.0)),
            "yaw": float(info.get("yaw", 0.0))})

    # Avisos em linguagem de operador, iguais aos da GUI: a mesma arena tem que
    # contar a mesma história nas duas interfaces.
    if sem_pose:
        dados["avisos"].append(
            f"{sem_pose} dock(s) sem posicao gravada — a missao aborta ao chegar neles.")
    if not dados["tem_keepout"]:
        dados["avisos"].append("Sem paredes virtuais (fita de piso e' invisivel ao LiDAR).")
    if not dados["waypoints"]:
        dados["avisos"].append("Nenhum waypoint salvo.")
    return dados


def mapa_png(nome: str) -> tuple:
    """Devolve (bytes_png, etag). O ETag vem do mtime+tamanho do .pgm."""
    pasta = pasta_dos_mapas() / nome
    mapa = _ler_yaml(pasta / "map.yaml")
    imagem = pasta / mapa.get("image", "map.pgm")
    if not imagem.is_file():
        return None, None

    st = imagem.stat()
    etag = f'"{int(st.st_mtime)}-{st.st_size}"'

    from PIL import Image
    with Image.open(imagem) as img:
        buf = io.BytesIO()
        img.convert("L").save(buf, format="PNG", optimize=True)
    return buf.getvalue(), etag
