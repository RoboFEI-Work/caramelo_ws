"""Servidor do painel web: FastAPI + uvicorn, com um nó ROS ao lado.

Rotas:
  GET  /                      pagina do painel
  GET  /api/saude             ROS no ar, nos vistos, arena ativa, navegacao
  GET  /api/arenas            arenas salvas no robo
  GET  /api/arenas/{n}/meta   resolucao, origem, docks, areas, waypoints, avisos
  GET  /api/arenas/{n}/mapa.png   imagem do mapa (com ETag; nunca streamada)
  POST /api/meta              manda o robo navegar ate (x, y, yaw) em metros
  WS   /ws/estado             pose 10 Hz, LiDAR 5 Hz, diagnostico, missao

O painel e' de ENGENHARIA: serve para acompanhar e depurar de outro computador
da rede. Nao substitui a GUI Qt embarcada -- as duas falam o mesmo contrato ROS.
"""
import argparse
import asyncio
import json
import os
import sys
from pathlib import Path

from fastapi import FastAPI, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from caramelo_web import arenas
from caramelo_web.ros_node import subir_no_em_thread

_no = None
_parar_ros = None


def pasta_do_frontend() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("caramelo_web")) / "www"
        if share.is_dir():
            return share
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "www"


def arena_ativa() -> str:
    """Mesma fonte que a GUI e o launch: ~/.config/caramelo/robot_state.yaml."""
    caminho = Path.home() / ".config/caramelo/robot_state.yaml"
    try:
        import yaml
        with caminho.open("r", encoding="utf-8") as f:
            return str((yaml.safe_load(f) or {}).get("map_name", "") or "")
    except Exception:
        return ""


def criar_app() -> FastAPI:
    app = FastAPI(title="Caramelo — painel de engenharia", docs_url=None, redoc_url=None)
    www = pasta_do_frontend()

    @app.get("/api/saude")
    def saude():
        nos = _no.nos_no_grafo() if _no else []
        return {
            "ros": _no is not None,
            "nos": nos,
            "quantidade_nos": len(nos),
            "arena": arena_ativa(),
            "navegacao_pronta": bool(_no and _no.navegacao_pronta()),
        }

    @app.get("/api/arenas")
    def listar_arenas():
        return {"arenas": arenas.listar(), "ativa": arena_ativa()}

    @app.get("/api/arenas/{nome}/meta")
    def meta_da_arena(nome: str):
        dados = arenas.metadados(nome)
        if "erro" in dados:
            return JSONResponse(dados, status_code=404)
        return dados

    @app.get("/api/arenas/{nome}/mapa.png")
    def mapa_da_arena(nome: str, request_etag: str = None):
        png, etag = arenas.mapa_png(nome)
        if png is None:
            return JSONResponse({"erro": "arena sem imagem"}, status_code=404)
        # Cache agressivo de proposito: o mapa so' muda quando alguem edita a
        # arena, e rebaixa-lo a cada carregamento seria desperdicio de rede.
        return Response(
            content=png, media_type="image/png",
            headers={"ETag": etag, "Cache-Control": "no-cache"})

    @app.post("/api/meta")
    async def enviar_meta(payload: dict):
        if _no is None:
            return JSONResponse({"ok": False, "mensagem": "ROS fora do ar"}, status_code=503)
        try:
            x = float(payload["x"])
            y = float(payload["y"])
            yaw = float(payload.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError):
            return JSONResponse(
                {"ok": False, "mensagem": "informe x, y (metros) e yaw (rad)"},
                status_code=400)
        ok, mensagem = _no.enviar_meta(x, y, yaw)
        return {"ok": ok, "mensagem": mensagem}

    @app.websocket("/ws/estado")
    async def ws_estado(ws: WebSocket):
        await ws.accept()
        # Taxas fixas, independentes do que o robo publica: a pose a 10 Hz e o
        # LiDAR a 5 Hz sao o suficiente para a tela, e o resto da banda fica com
        # a navegacao.
        ciclo = 0
        try:
            while True:
                com_scan = (ciclo % 2 == 0)
                await ws.send_text(json.dumps(_no.instantaneo(com_scan=com_scan)))
                ciclo += 1
                await asyncio.sleep(0.1)
        except (WebSocketDisconnect, RuntimeError):
            return

    if www.is_dir():
        @app.get("/")
        def raiz():
            return FileResponse(www / "index.html")

        app.mount("/", StaticFiles(directory=str(www)), name="www")

    return app


def main(argv=None):
    global _no, _parar_ros

    parser = argparse.ArgumentParser(description="Painel web do Caramelo.")
    parser.add_argument("--host", default="0.0.0.0",
                        help="0.0.0.0 para acessar de outro computador da rede")
    parser.add_argument("--port", type=int, default=8080)
    # Argumentos do ROS (--ros-args ...) nao sao nossos; ignora sem reclamar.
    args, _resto = parser.parse_known_args(
        argv if argv is not None else sys.argv[1:])

    _no, _parar_ros = subir_no_em_thread()

    import uvicorn
    try:
        uvicorn.run(criar_app(), host=args.host, port=args.port, log_level="warning")
    finally:
        if _parar_ros:
            _parar_ros()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
