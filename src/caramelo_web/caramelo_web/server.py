"""Servidor do painel web: FastAPI + uvicorn, com um nó ROS ao lado.

LEITURA (livre, sem chave)
  GET  /                          página do painel
  GET  /api/acesso                a chave apresentada vale?
  GET  /api/saude                 ROS no ar, nós vistos, arena ativa, capacidades
  GET  /api/arenas                arenas salvas no robô
  GET  /api/arenas/{n}/meta       resolução, origem, docks, áreas, waypoints, avisos
  GET  /api/arenas/{n}/mapa.png   imagem do mapa (com ETag; nunca streamada)
  GET  /api/arenas/{n}/keepout.png paredes virtuais, ja tingidas
  GET  /api/arenas/{n}/nomes      nomes de ponto validos (o fim do texto livre)
  GET  /api/sistema               uso de processador, memoria e tempo ligado
  GET  /api/missoes               tasks de competição que dá para disparar
  GET  /api/missoes/plano         último plano montado (dry-run)
  GET  /api/tela/estado           dá para espelhar a tela deste computador?
  WS   /ws/estado                 pose 10 Hz, LiDAR 5 Hz, diagnóstico, missão

COM CHAVE (?t=... ou cabeçalho X-Caramelo-Chave)
  POST /api/meta                  navegar até (x, y, yaw) em metros
  POST /api/parar                 o botão vermelho: freia tudo (não é E-stop)
  POST /api/base                  levar o robô para o dock START
  POST /api/navegacao/cancelar    parar o que está indo
  POST /api/navegacao/costmaps    limpar a memória de obstáculos
  POST /api/pose_inicial          "o robô está aqui" (clique no mapa)
  POST /api/dock                  encostar num dock
  POST /api/undock                sair do dock
  POST /api/missao/plano          montar o plano sem mover o robô
  POST /api/missao/rodar          executar
  POST /api/missao/abortar        parar a missão
  POST /api/teleop                dirigir na mão (com watchdog no nó ROS)
  POST /api/arena_ativa           trocar a arena em que o robô acha que está
  POST /api/arenas/{n}/ponto      gravar uma estação, START ou FINISH
  DELETE /api/arenas/{n}/ponto/{id}  apagar um ponto da arena
  GET  /api/mapeamento/estado     o SLAM está ligado? dá para salvar?
  POST /api/mapeamento/iniciar    ligar o SLAM
  POST /api/mapeamento/cancelar   desligar o SLAM
  POST /api/mapeamento/salvar     gravar o mapa numa arena nova
  POST /api/mapeamento/ruido      limpar o bitmap do mapa
  POST /api/mapeamento/paredes    pintar as paredes virtuais
  POST /api/camera                ligar/desligar a câmera do robô
  GET  /api/camera.mjpeg          a imagem da câmera, enquanto alguém olha
  GET  /api/tela.mjpeg            espelho da tela do computador do robô
  WS   /ws/terminal               um bash de verdade dentro do navegador

Por que a divisão é essa: o painel escuta em 0.0.0.0 para ser aberto de outro
computador, e desde que ganhou terminal e teleop isso significaria dar shell e
volante a qualquer um na mesma rede. Ver caramelo_web/seguranca.py.
"""
import argparse
import asyncio
import json
import os
import sys
from pathlib import Path

from fastapi import FastAPI, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from starlette.background import BackgroundTask

from caramelo_web import arenas, mapeamento, missoes, seguranca, tela, terminal
from caramelo_web.ros_node import subir_no_em_thread

_no = None
_parar_ros = None
_chave = None

# Rotas de LEITURA que mesmo assim pedem a chave. O espelho de tela é a única:
# ele mostra o desktop inteiro do robô (e-mail aberto, senha na tela do
# terminal), então não é "informação do robô", é a máquina de alguém.
LEITURA_PROTEGIDA = {"/api/tela.mjpeg"}


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


def _sem_ros():
    return JSONResponse(
        {"ok": False, "mensagem": "O robô não está respondendo (o ROS caiu?)."},
        status_code=503)


def _resposta(par: tuple):
    """(ok, mensagem) do nó ROS vira JSON. 409 quando o robô recusou.

    409 e não 500: nada quebrou. O robô simplesmente não estava em condição de
    obedecer, e o navegador precisa distinguir isso de "o painel caiu".
    """
    ok, mensagem = par
    return JSONResponse({"ok": bool(ok), "mensagem": mensagem},
                        status_code=200 if ok else 409)


def _resposta_de_dicionario(dados: dict):
    """Resposta pronta dos módulos que já devolvem {ok, mensagem, http}.

    mapeamento.py resolve tudo sozinho e diz em que código a recusa cabe (400
    para pedido malformado, 409 para "o robô não estava em condição"). A rota
    só entrega -- reescrever a decisão aqui daria duas respostas diferentes
    para a mesma recusa.
    """
    dados = dict(dados or {})
    padrao = 200 if dados.get("ok") else 409
    http = int(dados.pop("http", padrao) or padrao)
    if dados.get("ok"):
        http = 200
    return JSONResponse(dados, status_code=http)


def _pose_do_robo():
    """(x, y, yaw) atual, ou None quando o robô não sabe onde está.

    Pose velha é pior que pose nenhuma aqui: gravar um ponto com a posição de
    dez segundos atrás cria uma estação alguns metros fora do lugar, e o erro
    só aparece na prova, quando o robô para no vazio.
    """
    if _no is None:
        return None
    foto = _no.instantaneo(com_scan=False, com_diagnostico=False)
    pose = foto.get("pose")
    if not pose or pose.get("velho"):
        return None
    return (float(pose["x"]), float(pose["y"]), float(pose["yaw"]))


def _uso_do_computador() -> dict:
    """Processador, memória e tempo ligado, lidos de /proc.

    Sem psutil de propósito: uma dependência a mais para instalar no robô, e
    tudo o que esta tela mostra está em três arquivos de texto que o Linux já
    mantém. Se algum não existir (outro sistema), a tela diz que este robô não
    informa, em vez de quebrar.
    """
    saida = {}
    try:
        with open("/proc/uptime", "r", encoding="ascii") as f:
            saida["tempo_ligado"] = float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        pass
    try:
        with open("/proc/loadavg", "r", encoding="ascii") as f:
            carga = float(f.read().split()[0])
        nucleos = os.cpu_count() or 1
        # Carga média e não porcentagem instantânea: a instantânea exigiria
        # duas leituras separadas por um intervalo, e a rota travaria por esse
        # intervalo a cada consulta.
        saida["cpu"] = min(100.0, 100.0 * carga / nucleos)
    except (OSError, ValueError, IndexError):
        pass
    try:
        valores = {}
        with open("/proc/meminfo", "r", encoding="ascii") as f:
            for linha in f:
                partes = linha.split()
                if len(partes) >= 2:
                    valores[partes[0].rstrip(":")] = float(partes[1])
        total = valores.get("MemTotal", 0.0)
        livre = valores.get("MemAvailable", valores.get("MemFree", 0.0))
        if total > 0:
            saida["memoria_total_mb"] = total / 1024.0
            saida["memoria_usada_mb"] = (total - livre) / 1024.0
            saida["memoria"] = 100.0 * (total - livre) / total
    except (OSError, ValueError):
        pass
    return saida


def criar_app(chave) -> FastAPI:
    app = FastAPI(title="Caramelo — painel", docs_url=None, redoc_url=None)
    www = pasta_do_frontend()

    @app.middleware("http")
    async def porteiro(pedido: Request, adiante):
        """Uma tranca só, na porta, em vez de uma conferência por rota.

        Espalhar a checagem pelas rotas é como se esquece de trancar uma: a
        rota nova nasce aberta e ninguém percebe. Aqui o padrão é o contrário —
        toda rota que MUDA algo nasce trancada.
        """
        muda_algo = pedido.method not in ("GET", "HEAD", "OPTIONS")
        if (muda_algo or pedido.url.path in LEITURA_PROTEGIDA) and \
                not chave.autorizado(pedido):
            return JSONResponse(seguranca.recado_sem_chave(), status_code=403)
        return await adiante(pedido)

    # ------------------------------------------------------------- leitura
    @app.get("/api/acesso")
    def acesso(pedido: Request):
        return {"autorizado": chave.autorizado(pedido)}

    @app.get("/api/saude")
    def saude():
        nos = _no.nos_no_grafo() if _no else []
        return {
            "ros": _no is not None,
            "nos": nos,
            "quantidade_nos": len(nos),
            "arena": arena_ativa(),
            "navegacao_pronta": bool(_no and _no.navegacao_pronta()),
            "capacidades": _no.capacidades() if _no else {},
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
    def mapa_da_arena(nome: str):
        png, etag = arenas.mapa_png(nome)
        if png is None:
            return JSONResponse({"erro": "arena sem imagem"}, status_code=404)
        # Cache agressivo de propósito: o mapa só muda quando alguém edita a
        # arena, e rebaixá-lo a cada carregamento seria desperdício de rede.
        return Response(
            content=png, media_type="image/png",
            headers={"ETag": etag, "Cache-Control": "no-cache"})

    @app.get("/api/arenas/{nome}/keepout.png")
    def paredes_da_arena(nome: str):
        """A máscara de paredes virtuais, já tingida pelo robô.

        Tingir aqui e não no navegador: a máscara crua obrigaria o JavaScript a
        varrer 700 mil pixels a cada troca de arena, no mesmo tablet que
        precisa responder ao dedo do operador.
        """
        png, etag = arenas.keepout_png(nome)
        if png is None:
            return JSONResponse({"erro": "esta arena não tem paredes virtuais"},
                                status_code=404)
        return Response(
            content=png, media_type="image/png",
            headers={"ETag": etag, "Cache-Control": "no-cache"})

    @app.get("/api/arenas/{nome}/nomes")
    def nomes_de_ponto(nome: str):
        """Os nomes de ponto que valem nesta arena.

        É o que substitui o campo de texto livre na hora de criar um ponto: o
        que a arena já tem (para corrigir) e o próximo número livre de cada
        sigla do regulamento (para criar).
        """
        return {"nomes": arenas.nomes_de_ponto_sugeridos(nome),
                "tipos": arenas.tipos_de_area()}

    @app.get("/api/sistema")
    def sistema():
        dados = _uso_do_computador()
        if not dados:
            return JSONResponse(
                {"ok": False,
                 "mensagem": "Este robô não informa uso de processador e memória."},
                status_code=404)
        return dados

    @app.get("/api/mapeamento/estado")
    def estado_do_mapeamento():
        # A lista de nós entra para o módulo poder distinguir "desligado" de
        # "ligado por fora deste painel" -- que é um estado real e confuso: o
        # botão daqui não consegue desligar aquilo.
        return mapeamento.estado(_no.nos_no_grafo() if _no else None)

    @app.get("/api/missoes")
    def listar_missoes():
        return {"missoes": missoes.listar()}

    @app.get("/api/missoes/plano")
    def plano_da_missao():
        return _no.plano_atual() if _no else {}

    @app.get("/api/tela/estado")
    def estado_da_tela():
        return tela.diagnostico()

    # -------------------------------------------------------------- escrita
    @app.post("/api/meta")
    async def enviar_meta(payload: dict):
        if _no is None:
            return _sem_ros()
        try:
            x, y = float(payload["x"]), float(payload["y"])
            yaw = float(payload.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError):
            return JSONResponse(
                {"ok": False, "mensagem": "Faltou dizer para onde ir."},
                status_code=400)
        return _resposta(_no.enviar_meta(x, y, yaw))

    @app.post("/api/parar")
    async def parar_robo():
        """O botão vermelho do menu principal.

        Rota própria, e não "cancelar + abortar" disparados pela tela: o nó
        zera o TELEOP antes de tudo, e parar de publicar não é parar -- o ESC
        desta base lê ausência de PWM como ré em velocidade máxima. Deixar a
        sequência com o navegador significaria que uma aba que perdeu a rede no
        meio dos dois pedidos pararia metade do robô.

        Continua NÃO sendo parada de emergência: este robô não tem botão de
        emergência de hardware, e o rótulo não promete um.
        """
        return _sem_ros() if _no is None else _resposta(_no.parar_robo())

    @app.post("/api/base")
    async def levar_para_base():
        """Manda o robô encostar no dock START da arena ativa.

        O nó confere de novo se dá -- e devolve o MESMO motivo que a barra
        inferior mostra. Não é desconfiança da tela: esta rota é HTTP, e um
        pedido que chega sem passar pela tela não tem botão cinza nenhum para
        respeitar.
        """
        return _sem_ros() if _no is None else _resposta(_no.levar_para_base())

    @app.post("/api/navegacao/cancelar")
    async def cancelar():
        return _sem_ros() if _no is None else _resposta(_no.cancelar_navegacao())

    @app.post("/api/navegacao/costmaps")
    async def costmaps():
        return _sem_ros() if _no is None else _resposta(_no.limpar_costmaps())

    @app.post("/api/pose_inicial")
    async def pose_inicial(payload: dict):
        if _no is None:
            return _sem_ros()
        try:
            x, y = float(payload["x"]), float(payload["y"])
            yaw = float(payload.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError):
            return JSONResponse(
                {"ok": False, "mensagem": "Faltou dizer onde o robô está."},
                status_code=400)
        return _resposta(_no.definir_pose_inicial(x, y, yaw))

    @app.post("/api/dock")
    async def dockar(payload: dict):
        if _no is None:
            return _sem_ros()
        return _resposta(_no.dockar(str(payload.get("dock_id", ""))))

    @app.post("/api/undock")
    async def desdockar(payload: dict):
        if _no is None:
            return _sem_ros()
        return _resposta(_no.desdockar(str(payload.get("tipo", ""))))

    @app.post("/api/missao/plano")
    async def montar_plano(payload: dict):
        if _no is None:
            return _sem_ros()
        # O caminho vem do navegador, ou seja, de fora. Só vale se for um dos
        # que o próprio painel listou -- senão o executor abriria qualquer
        # arquivo do disco que alguém pedisse.
        if not missoes.valida(str(payload.get("task_yaml", ""))):
            return JSONResponse(
                {"ok": False, "mensagem": "Escolha uma das tarefas da lista."},
                status_code=400)
        return _resposta(_no.rodar_missao(payload, dry_run=True))

    @app.post("/api/missao/rodar")
    async def rodar_missao(payload: dict):
        if _no is None:
            return _sem_ros()
        if not missoes.valida(str(payload.get("task_yaml", ""))):
            return JSONResponse(
                {"ok": False, "mensagem": "Escolha uma das tarefas da lista."},
                status_code=400)
        return _resposta(_no.rodar_missao(payload, dry_run=False))

    @app.post("/api/missao/abortar")
    async def abortar_missao():
        return _sem_ros() if _no is None else _resposta(_no.abortar_missao())

    @app.post("/api/teleop")
    async def dirigir(payload: dict):
        if _no is None:
            return _sem_ros()
        if payload.get("parar"):
            return _resposta(_no.teleop_parar())
        return _resposta(_no.teleop(
            payload.get("vx", 0.0), payload.get("vy", 0.0), payload.get("wz", 0.0)))

    # `def` e nao `async def` de proposito: tela.diagnostico() TIRA uma foto da
    # tela para conferir se dá, e isso bloqueia por dezenas de milissegundos.
    # Numa rota async isso travaria o event loop -- ou seja, o mapa, a pose e o
    # terminal de todo mundo -- a cada vez que alguém abrisse o espelho. Sendo
    # `def`, o FastAPI executa em thread separada; o gerador do stream continua
    # assíncrono e não é afetado.
    @app.get("/api/tela.mjpeg")
    def espelho_da_tela(largura: int = tela.LARGURA_PADRAO,
                        qualidade: int = tela.QUALIDADE_PADRAO,
                        fps: float = tela.FPS_PADRAO):
        # Conferido ANTES de abrir o stream: sem isso, "não há tela" chegaria ao
        # navegador como uma imagem que nunca carrega, sem uma linha de
        # explicação para o operador.
        situacao = tela.diagnostico()
        if not situacao["disponivel"]:
            return JSONResponse(
                {"ok": False, "mensagem": situacao["mensagem"]}, status_code=503)
        if tela.lotado():
            return JSONResponse(
                {"ok": False, "mensagem": "Já há espelhos de tela demais abertos. "
                                          "Feche uma das outras janelas."},
                status_code=503)
        transmissao = tela.Transmissao(largura, qualidade, fps)
        return StreamingResponse(
            transmissao.quadros(),
            media_type=f"multipart/x-mixed-replace; boundary={tela.LIMITE.decode()}",
            headers={"Cache-Control": "no-store", "Pragma": "no-cache"},
            # A vaga do espelho volta AQUI, e nao no fim do gerador. O servidor
            # executa a tarefa de fundo quando a resposta acaba -- inclusive
            # quando acaba porque o navegador sumiu -- enquanto o gerador
            # abandonado fica suspenso no yield ate o coletor de lixo passar.
            background=BackgroundTask(transmissao.encerrar))

    # --------------------------------------------------------------- arena
    @app.post("/api/arena_ativa")
    async def trocar_arena(payload: dict):
        """Troca a arena em que o robô acha que está.

        Mesmo arquivo que a GUI e o launch leem
        (~/.config/caramelo/robot_state.yaml), e por isso a troca feita aqui
        vale para o robô inteiro -- não só para esta aba.
        """
        # A tela manda "arena" e "nome" porque foi escrita em paralelo com esta
        # rota; qualquer um dos dois serve.
        nome = str(payload.get("arena") or payload.get("nome") or "")
        return _resposta(arenas.definir_arena_ativa(nome))

    @app.post("/api/arenas/{nome}/ponto")
    async def gravar_ponto(nome: str, payload: dict):
        return _resposta_de_dicionario(
            mapeamento.gravar_ponto(nome, payload or {}, _pose_do_robo()))

    @app.delete("/api/arenas/{nome}/ponto/{ident}")
    async def apagar_ponto(nome: str, ident: str):
        return _resposta_de_dicionario(mapeamento.apagar_ponto(nome, ident))

    # ---------------------------------------------------------- mapeamento
    @app.post("/api/mapeamento/iniciar")
    async def iniciar_mapeamento():
        return _resposta_de_dicionario(
            mapeamento.iniciar(_no.nos_no_grafo() if _no else None))

    @app.post("/api/mapeamento/cancelar")
    async def cancelar_mapeamento():
        return _resposta_de_dicionario(mapeamento.cancelar())

    @app.post("/api/mapeamento/salvar")
    async def salvar_mapa(payload: dict):
        return _resposta_de_dicionario(
            mapeamento.salvar_mapa(str(payload.get("nome") or "")))

    # `def` e não `async def`: as duas rotas abaixo abrem, editam e regravam um
    # PGM de centenas de milhares de pixels. Numa rota async isso travaria o
    # event loop -- ou seja, a pose, o mapa e o terminal de todo mundo --
    # enquanto o pincel é aplicado. Sendo `def`, o FastAPI as executa em thread
    # separada.
    @app.post("/api/mapeamento/ruido")
    def limpar_ruido(payload: dict):
        return _resposta_de_dicionario(
            mapeamento.editar_ruido(str(payload.get("arena") or ""), payload or {}))

    @app.post("/api/mapeamento/paredes")
    def pintar_paredes(payload: dict):
        return _resposta_de_dicionario(
            mapeamento.editar_paredes(str(payload.get("arena") or ""), payload or {}))

    # -------------------------------------------------------------- câmera
    @app.post("/api/camera")
    async def camera(payload: dict):
        if _no is None:
            return _sem_ros()
        if payload.get("desligar"):
            return _resposta(_no.desligar_camera())
        return _resposta(_no.assinar_camera(str(payload.get("topico") or "")))

    @app.get("/api/camera.mjpeg")
    async def camera_mjpeg(fps: float = 5.0):
        """A imagem da câmera enquanto alguém estiver olhando.

        MJPEG e não WebSocket pelo mesmo motivo do espelho de tela: o <img> do
        navegador desenha isto sozinho, sem uma linha de JavaScript por quadro.

        Pedir quadro é também o que diz ao nó "tem gente olhando" -- é isso que
        segura a assinatura viva. Fechada a aba, o gerador morre, ninguém mais
        pede, e o nó larga a câmera sozinho em alguns segundos.
        """
        if _no is None:
            return _sem_ros()

        intervalo = 1.0 / max(1.0, min(15.0, float(fps)))

        async def quadros():
            vazios = 0
            while True:
                jpeg, _info = _no.quadro_camera()
                if jpeg:
                    vazios = 0
                    yield (b"--" + tela.LIMITE + b"\r\n"
                           b"Content-Type: image/jpeg\r\n"
                           b"Content-Length: " + str(len(jpeg)).encode("ascii") +
                           b"\r\n\r\n" + bytes(jpeg) + b"\r\n")
                else:
                    vazios += 1
                    # Trinta ciclos sem imagem = a câmera não está publicando.
                    # Encerrar o stream faz o <img> disparar o erro, e a tela
                    # explica; ficar aberto para sempre deixaria uma imagem
                    # travada na tela parecendo uma câmera funcionando.
                    if vazios > 30:
                        return
                await asyncio.sleep(intervalo)

        return StreamingResponse(
            quadros(),
            media_type=f"multipart/x-mixed-replace; boundary={tela.LIMITE.decode()}",
            headers={"Cache-Control": "no-store", "Pragma": "no-cache"})

    # ---------------------------------------------------------- websockets
    @app.websocket("/ws/estado")
    async def ws_estado(ws: WebSocket):
        await ws.accept()
        # Taxas fixas, independentes do que o robô publica: a pose a 10 Hz e o
        # LiDAR a 5 Hz são o suficiente para a tela, e o resto da banda fica com
        # a navegação.
        ciclo = 0
        visto = 0
        try:
            while True:
                if _no is None:
                    # Acontece de verdade com --sem-ros (conferir rotas sem
                    # robô). Sem esta saída, o painel recebia um erro de
                    # servidor em vez de simplesmente não ter estado.
                    await ws.close(code=1011)
                    return
                # quadro() e não instantaneo(): é ele que decide a banda de cada
                # bloco E manda o menu principal junto do primeiro quadro de
                # cada conexão. MEDIDO: chamando instantaneo() direto, o menu
                # nunca chegava e o painel abria com os cinco cartões escritos
                # na cópia local do navegador -- ou seja, com títulos que
                # podiam já ter mudado na GUI sem ninguém perceber.
                quadro = _no.quadro(ciclo=ciclo, desde_evento=visto)
                visto = quadro["serie_evento"]
                await ws.send_text(json.dumps(quadro))
                ciclo += 1
                await asyncio.sleep(0.1)
        except (WebSocketDisconnect, RuntimeError):
            return

    @app.websocket("/ws/terminal")
    async def ws_terminal(ws: WebSocket, colunas: int = 100, linhas: int = 30):
        # Aceita antes de conferir a chave só para poder EXPLICAR a recusa: um
        # 403 seco apareceria na tela como um terminal preto sem motivo.
        await ws.accept()
        if not chave.autorizado(ws):
            await ws.send_text(json.dumps(
                {"t": "fim", "texto": seguranca.recado_sem_chave()["mensagem"]}))
            await ws.close(code=1008)
            return
        try:
            await terminal.atender(ws, colunas, linhas)
        except WebSocketDisconnect:
            pass
        finally:
            try:
                await ws.close()
            except RuntimeError:
                pass

    if www.is_dir():
        @app.get("/")
        def raiz():
            return FileResponse(www / "index.html")

        # follow_symlink=True NAO e detalhe: o projeto compila com
        # `colcon build --symlink-install`, entao TODO arquivo do www instalado
        # e um link para a arvore fonte. O StaticFiles, por padrao, resolve o
        # caminho real e recusa o que cai fora da pasta servida -- ou seja,
        # recusa tudo. MEDIDO (2026-08-18): sem esta flag, /vendor/xterm.js
        # responde 404 e a aba Terminal abre PRETA, sem erro nenhum na tela,
        # enquanto a pagina inicial funciona normalmente (ela vai por
        # FileResponse, que nao passa por essa checagem).
        app.mount("/", StaticFiles(directory=str(www), follow_symlink=True),
                  name="www")

    return app


def _anunciar(chave, host: str, porta: int) -> None:
    """Imprime o link com a chave. É a única vez que ela aparece.

    Vai por print e não por logger de propósito: precisa aparecer no terminal
    de quem ligou o robô mesmo com o log do ROS em WARN.
    """
    links = chave.links(host, porta)
    largura = max(60, max(len(u) for u in links) + 6)
    print("=" * largura)
    print(" PAINEL DO CARAMELO no ar.")
    print(" Abra um destes endereços no navegador (o link já leva a chave):")
    print("")
    for link in links:
        print("   " + link)
    print("")
    print(" Sem a chave o painel abre assim mesmo, mas só para OLHAR:")
    print(" não dá para comandar o robô, abrir terminal nem ver a tela.")
    if chave.fixa:
        print(" (chave fixada por --token)")
    print("=" * largura, flush=True)


def main(argv=None):
    global _no, _parar_ros, _chave

    parser = argparse.ArgumentParser(description="Painel web do Caramelo.")
    parser.add_argument("--host", default="0.0.0.0",
                        help="0.0.0.0 para acessar de outro computador da rede")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--token", default="",
        help="Fixa a chave de acesso em vez de sortear uma. Serve para o launch "
             "poder reimprimir o mesmo link, e para colar num favorito.")
    parser.add_argument(
        "--sem-ros", action="store_true",
        help="Sobe só a parte web. Serve para conferir as rotas sem robô.")
    # Argumentos do ROS (--ros-args ...) não são nossos; ignora sem reclamar.
    args, _resto = parser.parse_known_args(
        argv if argv is not None else sys.argv[1:])

    _chave = seguranca.ChaveDeAcesso(args.token)
    if not args.sem_ros:
        _no, _parar_ros = subir_no_em_thread()
    _anunciar(_chave, args.host, args.port)

    import uvicorn
    try:
        uvicorn.run(criar_app(_chave), host=args.host, port=args.port,
                    log_level="warning")
    finally:
        # O SLAM que este painel ligou morre com ele. Sem isto, fechar o painel
        # deixaria um slam_toolbox rodando que ninguém consegue mais desligar
        # pela tela -- e o próximo painel a subir só saberia dizer "está ligado,
        # mas não fui eu que liguei".
        mapeamento.encerrar_tudo()
        if _parar_ros:
            _parar_ros()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
