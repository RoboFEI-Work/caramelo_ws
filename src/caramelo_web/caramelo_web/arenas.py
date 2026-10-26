"""Tudo que o painel web precisa saber sobre uma arena, lido do disco.

Mesma fonte de verdade da GUI Qt: a arvore FONTE de caramelo_mapping/maps. O
painel le os arquivos direto, sem passar por ROS -- da para inspecionar uma
arena com o robo desligado, e o mapa nao precisa trafegar como OccupancyGrid.

O mapa NUNCA e streamado: vai por REST como PNG, com ETag, e o navegador so
baixa de novo quando o arquivo muda. Um OccupancyGrid de 697x1039 repetido no
Wi-Fi de competicao tiraria banda da navegacao.

ESTE ARQUIVO E O GEMEO DE map_preview.cpp (caramelo_gui/src/widgets). O painel
tem que DESENHAR A MESMA ARENA que a GUI desenha: mesmas camadas, mesmas cores,
mesmos avisos. Por isso saem daqui, alem das poses:

  - ESTILO_DO_MAPA  as cores e medidas que o canvas usa (copiadas de
                    map_preview.cpp). Um segundo lugar guardando "#eb5757" e
                    exatamente como as duas telas comecam a divergir: a GUI
                    pinta um dock zerado de vermelho e a web pinta de laranja,
                    e ninguem percebe ate a missao abortar na arena.
  - TIPOS_DE_AREA / tipos_de_dock()  as listas fechadas dos seletores. Regra do
                    painel: onde existe opcao valida nao existe campo de texto.
  - os AVISOS, ja em portugues de operador.

Uma diferenca proposital em relacao a GUI: nenhum aviso daqui cita nome de
arquivo. A GUI diz "Sem ws_table_mapping.yaml"; quem le a tela do painel nao
sabe ROS e nao vai abrir o disco do robo. O QUE falta e o mesmo, o COMO dizer
muda.
"""
import io
import os
import re
import threading
from pathlib import Path

import yaml

_PLACEHOLDER = 1e-6

# ---------------------------------------------------------------- catalogos
#
# Listas fechadas. O painel nunca oferece campo de texto onde existe opcao
# valida: nome digitado a mao ("ws 1", "Ws1", "mesa") grava do mesmo jeito e so
# aparece como erro na prova, com o cronometro correndo.

# Copiado de caramelo_gui/src/widgets/seletor_tipo_area.cpp -- os seis tipos do
# regulamento. `valor` e o que vai para o YAML; `rotulo` e o que o operador le.
TIPOS_DE_AREA = [
    {"valor": "start", "rotulo": "Inicio da prova",
     "ajuda": "Onde o robo comeca a prova. Toda missao parte daqui."},
    {"valor": "finish", "rotulo": "Fim da prova",
     "ajuda": "Onde o robo tem que terminar a prova."},
    {"valor": "workstation", "rotulo": "Bancada de trabalho",
     "ajuda": "Mesa comum de onde o robo pega e onde deixa objetos."},
    {"valor": "shelf", "rotulo": "Prateleira",
     "ajuda": "Estacao alta: o robo para mais longe e o braco trabalha em outra altura."},
    {"valor": "precision_placement", "rotulo": "Encaixe de precisao",
     "ajuda": "Mesa com cavidades: o objeto tem que entrar no furo do formato certo."},
    {"valor": "rotating_table", "rotulo": "Mesa giratoria",
     "ajuda": "Mesa que gira sem parar; o robo tem que acertar o tempo da pega."},
]

# Os plugins que o robo sempre tem. Uma arena recem-criada pode ainda nao ter
# docking.yaml, e deixar a tela sem opcao nenhuma seria pior que oferecer os
# tres padroes (mesma decisao de lerTiposDeDockDaArena).
TIPOS_DE_DOCK_PADRAO = [
    "caramelo_front_dock", "caramelo_shelf_front_dock", "caramelo_precision_front_dock"]

# Rotulos de seletor_tipo_dock.cpp. Tipo fora desta lista continua aparecendo
# pelo nome tecnico: esconder uma opcao que existe no arquivo seria pior que
# mostrar um nome feio.
_ROTULOS_DE_DOCK = {
    "caramelo_front_dock": (
        "Aproximacao normal (bancada)",
        "Encosta de frente na estacao, parando a 25 cm. E o padrao das bancadas."),
    "caramelo_shelf_front_dock": (
        "Aproximacao de prateleira",
        "Para mais longe (90 cm) porque a prateleira e alta e o braco precisa de espaco."),
    "caramelo_precision_front_dock": (
        "Aproximacao de precisao",
        "Chega com tolerancia menor, para as mesas de encaixe."),
}

# Regra de nome de ponto do regulamento, igual a validador_id_ponto.hpp.
# Ancorada nas duas pontas: "WS1 " ou "xWS1" nao passam -- o executor da missao
# compara o nome inteiro.
REGEX_ID_PONTO = re.compile(r"^(START|FINISH|WS[0-9]+|SH[0-9]+|PP[0-9]+|RT[0-9]+)$")

# Prefixos numerados, para o painel montar a lista de nomes possiveis em vez de
# pedir que o operador digite.
PREFIXOS_DE_PONTO = [
    {"prefixo": "WS", "rotulo": "Bancada", "tipo": "workstation"},
    {"prefixo": "SH", "rotulo": "Prateleira", "tipo": "shelf"},
    {"prefixo": "PP", "rotulo": "Encaixe de precisao", "tipo": "precision_placement"},
    {"prefixo": "RT", "rotulo": "Mesa giratoria", "tipo": "rotating_table"},
]

# Cores e medidas do desenho, copiadas de map_preview.cpp. Vao junto com os
# metadados para o canvas do painel nao precisar redigitar nenhuma delas.
ESTILO_DO_MAPA = {
    "fundo": "#0b1626",
    "grade": "rgba(255,255,255,0.15)",
    "grade_passo_m": 1.0,
    "grade_passo_minimo_px": 12.0,     # abaixo disso a grade vira ruido
    "keepout": "rgba(235,87,87,0.43)",
    "sem_pose": "#eb5757",             # pose [0,0,0]: nunca foi gravada
    "dock": "#f2994a",
    "dock_inicio": "#27ae60",          # START
    "dock_fim": "#9b51e0",             # FINISH
    "area": "#56ccf2",
    "area_preenchimento": "rgba(86,204,242,0.18)",
    "area_lado_px": 22.0,
    "waypoint": "#35c3f0",
    "edicao": "#f2c94c",
    "robo": "#ff9a2e",
    "raio_marcador_px": 7.0,
    "comprimento_seta_px": 22.0,
    "tolerancia_toque_px": 12.0,       # dedo em tela de robo nao acerta 7 px
    "rotulo_sem_pose": " (sem pose)",
}

# Camadas na MESMA ordem em que map_preview.cpp pinta: quem esta por cima no
# desenho e quem ganha o toque quando dois pontos se sobrepoem (acontece o
# tempo todo com varios pontos ainda em [0,0,0], empilhados na origem).
CAMADAS_DO_MAPA = [
    {"id": "keepout", "rotulo": "Paredes virtuais"},
    {"id": "grade", "rotulo": "Grade (1 m)"},
    {"id": "areas", "rotulo": "Service areas"},
    {"id": "docks", "rotulo": "Docks"},
    {"id": "waypoints", "rotulo": "Waypoints"},
    {"id": "rotulos", "rotulo": "Nomes"},
]

# Cache de metadados e de imagens ja codificadas. metadados() e chamado pelo
# HTTP e pelo no ROS (1 Hz, para os cartoes do menu); sem cache, cada chamada
# reabre um PGM de ~700 KB so para descobrir a largura da imagem.
_cache = {}
_cache_imagens = {}
_trava = threading.Lock()


# ------------------------------------------------------------------ caminhos
def pasta_dos_mapas() -> Path:
    """A mesma resolucao que a GUI usa: fonte do workspace, nao o share."""
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


def pasta_da_arena(nome: str):
    """Pasta de UMA arena, ou None se o nome nao serve.

    O nome chega pela URL, ou seja, de fora. Sem esta conferencia um pedido
    como /api/arenas/..%2f..%2f.ssh/meta faria o painel ler pasta de fora da
    arvore de mapas -- e a rota de leitura nem pede chave de acesso.
    """
    nome = (nome or "").strip()
    if not nome or nome.startswith(".") or "/" in nome or "\\" in nome or ".." in nome:
        return None
    pasta = pasta_dos_mapas() / nome
    return pasta if (pasta / "map.yaml").is_file() else None


def caminho_do_estado_do_robo() -> Path:
    """~/.config/caramelo/robot_state.yaml -- o mesmo arquivo da GUI e do launch.

    Ele diz em que arena o robo esta. Quem le e o caramelo_bringup na hora de
    subir; qualquer mudanca de formato tem que acontecer nos dois lados.
    """
    return Path.home() / ".config/caramelo/robot_state.yaml"


# -------------------------------------------------------------------- listas
def listar() -> list:
    raiz = pasta_dos_mapas()
    if not raiz.is_dir():
        return []
    return sorted(
        d.name for d in raiz.iterdir()
        if d.is_dir() and not d.name.startswith(".") and (d / "map.yaml").is_file())


def arena_ativa() -> str:
    """Em que arena o robo acha que esta. Vazio = nunca foi escolhida.

    Vazio nao vira um padrao silencioso: a tela tem que PEDIR a arena, porque
    subir localizado no mapa errado poe o robo em um lugar que nao existe.
    """
    return str(_ler_yaml(caminho_do_estado_do_robo()).get("map_name", "") or "")


def estado_da_arena_ativa() -> dict:
    """A arena gravada no robo ainda existe?

    Achado em campo: robot_state.yaml aponta para "warehouse", que nao esta mais
    em maps/. A GUI mostra o nome do mesmo jeito e o operador so descobre o
    problema quando a navegacao sobe sem mapa. Aqui a pergunta e respondida
    antes, e o menu principal pode dizer o que fazer.
    """
    nome = arena_ativa()
    if not nome:
        return {"nome": "", "existe": False, "motivo": "Nenhuma arena escolhida."}
    if pasta_da_arena(nome) is None:
        return {"nome": nome, "existe": False,
                "motivo": (f"A arena '{nome}' nao esta mais salva neste robo. "
                           "Escolha outra em Mapas.")}
    return {"nome": nome, "existe": True, "motivo": ""}


def pose_inicial_salva() -> dict:
    """Ultima posicao confirmada pelo operador, se for DESTA arena.

    A pose so vale para o mapa em que foi gravada: restaurar a pose de uma
    arena dentro de outra poria o robo em um lugar que nao existe (mesma regra
    de robot_state.cpp).
    """
    raiz = _ler_yaml(caminho_do_estado_do_robo())
    pose = raiz.get("initial_pose") or {}
    if not isinstance(pose, dict):
        return {}
    if str(pose.get("map_name", "")) != str(raiz.get("map_name", "")):
        return {}
    try:
        return {"x": float(pose.get("x", 0.0)), "y": float(pose.get("y", 0.0)),
                "yaw": float(pose.get("yaw", 0.0))}
    except (TypeError, ValueError):
        return {}


def definir_arena_ativa(nome: str) -> tuple:
    """Grava a arena escolhida, preservando o resto do arquivo.

    Preservar as outras chaves nao e detalhe: a pose inicial mora no mesmo
    arquivo, e reescreve-lo inteiro apagaria a localizacao que o operador
    acabou de confirmar.
    """
    if pasta_da_arena(nome) is None:
        return False, "Escolha uma das arenas da lista."
    caminho = caminho_do_estado_do_robo()
    raiz = _ler_yaml(caminho)
    if not isinstance(raiz, dict):
        raiz = {}
    if raiz.get("map_name") == nome:
        return True, f"O robo ja esta na arena {nome}."
    raiz["map_name"] = nome
    # A pose gravada era de OUTRA arena: mante-la aqui faria o robo subir
    # achando que ja sabe onde esta, no mapa errado.
    pose = raiz.get("initial_pose")
    if isinstance(pose, dict) and str(pose.get("map_name", "")) != nome:
        raiz.pop("initial_pose", None)
    try:
        caminho.parent.mkdir(parents=True, exist_ok=True)
        with caminho.open("w", encoding="utf-8") as f:
            yaml.safe_dump(raiz, f, allow_unicode=True, sort_keys=False)
    except OSError:
        return False, ("Nao consegui gravar a escolha da arena no robo. "
                       "O robo voltaria para a arena antiga ao religar.")
    return True, (f"Arena {nome} escolhida. Marque no mapa onde o robo esta "
                  "antes de mandar ele andar.")


def rotulo_de_tipo_de_dock(tipo: str) -> str:
    """Nome humano do tipo de aproximacao.

    Tipo que nao esta na tabela vira "Aproximacao WS1" em vez do nome do plugin:
    esconder uma opcao que existe no arquivo seria pior (a GUI ja decidiu isso),
    mas mostrar "caramelo_ws1_dock" para quem nao sabe ROS tambem nao ajuda.
    """
    conhecido = _ROTULOS_DE_DOCK.get(tipo)
    if conhecido:
        return conhecido[0]
    miolo = re.sub(r"^caramelo_|_dock$", "", str(tipo or "")).replace("_", " ").strip()
    return f"Aproximacao {miolo.upper()}" if miolo else str(tipo or "")


def tipos_de_area() -> list:
    return [dict(t) for t in TIPOS_DE_AREA]


def tipos_de_dock(nome: str) -> list:
    """Tipos de aproximacao que ESTA arena conhece, com rotulo humano.

    O "sair do dock" pede TIPO e nao id, e o painel so pode oferecer lista: um
    tipo digitado errado faz o undock recuar sem plugin, colado na mesa.
    """
    pasta = pasta_da_arena(nome)
    tipos = []
    if pasta is not None:
        tipos = [str(t) for t in (_ler_yaml(pasta / "docking.yaml").get("dock_plugins") or {})]
    if not tipos:
        tipos = list(TIPOS_DE_DOCK_PADRAO)
    saida = []
    for t in tipos:
        ajuda = _ROTULOS_DE_DOCK.get(t, ("", ""))[1]
        saida.append({"valor": t, "rotulo": rotulo_de_tipo_de_dock(t), "ajuda": ajuda})
    return saida


# ------------------------------------------------------------ nome de ponto
def id_de_ponto_normalizado(ident: str) -> str:
    """O operador digita "ws1" o tempo todo; recusar isso seria implicancia."""
    return (ident or "").strip().upper()


def id_de_ponto_aceitavel(ident: str) -> bool:
    return bool(REGEX_ID_PONTO.match(id_de_ponto_normalizado(ident)))


def erro_de_id_de_ponto(ident: str) -> str:
    """Frase pronta para a tela; vazia quando o nome serve.

    Diz o formato e da exemplos, porque "invalido" sozinho nao ensina ninguem
    a acertar.
    """
    limpo = (ident or "").strip()
    if not limpo:
        return ("Escolha o nome do ponto. Vale START, FINISH, ou a sigla da estacao "
                "com o numero: WS1 (bancada), SH1 (prateleira), PP1 (encaixe de "
                "precisao), RT1 (mesa giratoria).")
    if REGEX_ID_PONTO.match(limpo):
        return ""
    if REGEX_ID_PONTO.match(id_de_ponto_normalizado(limpo)):
        return (f'Escreva "{limpo}" em letras maiusculas e sem espacos: '
                f"{id_de_ponto_normalizado(limpo)}.")
    return (f'"{limpo}" nao serve como nome de ponto. Use START, FINISH, ou a sigla '
            "da estacao seguida do numero, sem espacos: WS1 (bancada), SH1 "
            "(prateleira), PP1 (encaixe de precisao), RT1 (mesa giratoria).")


def nomes_de_ponto_sugeridos(nome: str) -> list:
    """A lista que substitui o campo de texto na hora de criar um ponto.

    Devolve o que a arena JA tem (para o operador corrigir um ponto existente) e
    o proximo numero livre de cada sigla (para criar). Assim o nome nasce dentro
    do regulamento sem ninguem digitar nada.
    """
    dados = metadados(nome)
    existentes = set()
    for grupo in ("docks", "areas", "waypoints"):
        for item in dados.get(grupo, []):
            existentes.add(str(item.get("id", "")).upper())

    saida = [{"id": i, "existe": True} for i in sorted(existentes)]
    for fixo in ("START", "FINISH"):
        if fixo not in existentes:
            saida.append({"id": fixo, "existe": False, "rotulo": fixo})
    for p in PREFIXOS_DE_PONTO:
        usados = [
            int(i[len(p["prefixo"]):]) for i in existentes
            if i.startswith(p["prefixo"]) and i[len(p["prefixo"]):].isdigit()]
        proximo = (max(usados) + 1) if usados else 1
        saida.append({
            "id": f"{p['prefixo']}{proximo}", "existe": False,
            "rotulo": f"{p['rotulo']} {proximo}", "tipo_sugerido": p["tipo"]})
    return saida


# ------------------------------------------------------------------ leitura
def _placeholder(x, y, yaw) -> bool:
    """Pose que ninguem gravou ainda.

    docking.yaml e service_areas.yaml nascem com [0,0,0] e o executor da missao
    recusa esses valores em modo real -- por isso vale destacar antes, em vez de
    deixar passar ate o robo estar na arena.
    """
    return abs(x) < _PLACEHOLDER and abs(y) < _PLACEHOLDER and abs(yaw) < _PLACEHOLDER


def _ler_yaml(caminho: Path) -> dict:
    try:
        with caminho.open("r", encoding="utf-8") as f:
            dados = yaml.safe_load(f)
        return dados if isinstance(dados, dict) else {}
    except Exception:
        return {}


def _assinatura(pasta: Path) -> tuple:
    """Impressao digital da arena: se ela nao mudou, o cache continua valendo."""
    marcas = []
    for arquivo in ("map.yaml", "map.pgm", "docking.yaml", "service_areas.yaml",
                    "waypoints.yaml", "keepout_mask.pgm", "keepout_mask.yaml",
                    "ws_table_mapping.yaml"):
        try:
            st = (pasta / arquivo).stat()
            marcas.append((arquivo, int(st.st_mtime_ns), st.st_size))
        except OSError:
            marcas.append((arquivo, 0, 0))
    return tuple(marcas)


def _papel_do_dock(ident: str) -> str:
    """START e FINISH sao o comeco e o fim da prova: cor propria no desenho."""
    alto = (ident or "").upper()
    if alto == "START":
        return "inicio"
    if alto == "FINISH":
        return "fim"
    return "estacao"


def metadados(nome: str) -> dict:
    """Tudo que a arena guarda, no formato que o canvas do painel consome.

    E o gemeo de MapPreview::carregar: mesmas poses, mesmas camadas, mesmos
    avisos. O canvas nao le nenhum arquivo -- ele desenha o que vem daqui.
    """
    pasta = pasta_da_arena(nome)
    if pasta is None:
        return {"erro": f"Nao existe arena chamada '{nome}' neste robo."}

    assinatura = _assinatura(pasta)
    with _trava:
        guardado = _cache.get(nome)
    if guardado and guardado[0] == assinatura:
        return guardado[1]

    dados = _montar_metadados(nome, pasta)
    with _trava:
        _cache[nome] = (assinatura, dados)
    return dados


def _montar_metadados(nome: str, pasta: Path) -> dict:
    mapa = _ler_yaml(pasta / "map.yaml")
    if not mapa:
        return {"erro": f"A arena '{nome}' esta sem o arquivo do mapa."}

    origem = mapa.get("origin", [0.0, 0.0, 0.0])
    dados = {
        "nome": nome,
        "resolucao": float(mapa.get("resolution", 0.05)),
        # origin e a pose do pixel INFERIOR esquerdo da imagem, e a imagem cresce
        # para BAIXO. Errar este sinal espelha todos os marcadores do mapa.
        "origem": [float(origem[0]), float(origem[1])],
        "origem_yaw": float(origem[2]) if len(origem) > 2 else 0.0,
        "docks": [],
        "areas": [],
        "waypoints": [],
        "tem_keepout": (pasta / "keepout_mask.pgm").is_file(),
        "avisos": [],
        "camadas": [dict(c) for c in CAMADAS_DO_MAPA],
        "estilo": dict(ESTILO_DO_MAPA),
        "tipos_area": tipos_de_area(),
    }

    largura = altura = 0
    try:
        from PIL import Image
        with Image.open(pasta / mapa.get("image", "map.pgm")) as img:
            largura, altura = img.size
    except Exception:
        dados["avisos"].append(
            "Nao consegui abrir a imagem desta arena: o desenho do mapa nao vai aparecer.")
    dados["largura_px"] = largura
    dados["altura_px"] = altura

    # --- docks ---
    docking = _ler_yaml(pasta / "docking.yaml")
    if not (pasta / "docking.yaml").is_file():
        dados["avisos"].append(
            "Esta arena nao tem nenhum dock gravado: nao da para encostar em "
            "estacao nem rodar missao aqui.")
    elif not docking:
        dados["avisos"].append(
            "A lista de docks desta arena esta ilegivel e nao pode ser usada.")

    sem_pose_dock = 0
    for ident, info in (docking.get("docks") or {}).items():
        info = info if isinstance(info, dict) else {}
        pose = info.get("pose") or [0.0, 0.0, 0.0]
        try:
            x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        except (TypeError, ValueError, IndexError):
            x = y = yaw = 0.0
        ph = _placeholder(x, y, yaw)
        sem_pose_dock += int(ph)
        tipo = str(info.get("type") or "")
        dados["docks"].append({
            "id": str(ident), "x": x, "y": y, "yaw": yaw, "sem_pose": ph,
            "tipo": tipo,
            "tipo_rotulo": rotulo_de_tipo_de_dock(tipo),
            "papel": _papel_do_dock(ident)})
    dados["docks"].sort(key=lambda d: d["id"])
    dados["tipos_dock"] = tipos_de_dock(nome)

    # --- service areas ---
    areas = _ler_yaml(pasta / "service_areas.yaml")
    if (pasta / "service_areas.yaml").is_file() and not areas:
        dados["avisos"].append(
            "A lista de estacoes desta arena esta ilegivel e nao pode ser usada.")
    sem_pose_area = 0
    validos = {t["valor"] for t in TIPOS_DE_AREA}
    for ident, info in (areas.get("service_areas") or {}).items():
        info = info if isinstance(info, dict) else {}
        p = info.get("final_robot_pose") or {}
        try:
            x, y, yaw = (float(p.get("x", 0.0)), float(p.get("y", 0.0)),
                         float(p.get("yaw", 0.0)))
        except (TypeError, ValueError):
            x = y = yaw = 0.0
        ph = _placeholder(x, y, yaw)
        sem_pose_area += int(ph)
        tipo = str(info.get("type") or "")
        dados["areas"].append({
            "id": str(ident), "x": x, "y": y, "yaw": yaw, "sem_pose": ph,
            "tipo": tipo,
            "tipo_rotulo": rotulo_de_tipo_de_area(tipo),
            "tipo_valido": tipo in validos,
            "manipulacao": bool(info.get("manipulation_enabled", True))})
    dados["areas"].sort(key=lambda a: a["id"])

    # --- waypoints ---
    waypoints = _ler_yaml(pasta / "waypoints.yaml")
    if (pasta / "waypoints.yaml").is_file() and not waypoints:
        dados["avisos"].append(
            "A lista de pontos de passagem desta arena esta ilegivel.")
    for ident, info in (waypoints.get("waypoints") or {}).items():
        info = info if isinstance(info, dict) else {}
        try:
            x, y, yaw = (float(info.get("x", 0.0)), float(info.get("y", 0.0)),
                         float(info.get("yaw", 0.0)))
        except (TypeError, ValueError):
            x = y = yaw = 0.0
        dados["waypoints"].append({
            "id": str(ident), "x": x, "y": y, "yaw": yaw,
            "sem_pose": _placeholder(x, y, yaw)})
    dados["waypoints"].sort(key=lambda w: w["id"])

    # --- keepout ---
    # A mascara tem yaml proprio; na pratica ela nasce com a mesma resolucao e
    # origem do mapa, mas ler os dois evita um desenho deslocado quando alguem
    # reeditar so um deles.
    if dados["tem_keepout"]:
        km = _ler_yaml(pasta / "keepout_mask.yaml")
        ko = km.get("origin", origem)
        dados["keepout"] = {
            "resolucao": float(km.get("resolution", dados["resolucao"])),
            "origem": [float(ko[0]), float(ko[1])],
        }

    # --- avisos em linguagem de operador ---
    # Dizem o que FAZER, nao so o que ha de errado. Mesma lista da GUI, sem os
    # nomes de arquivo (ver o cabecalho deste modulo).
    if sem_pose_dock:
        dados["avisos"].append(
            f"{sem_pose_dock} dock(s) ainda sem posicao gravada (marcados em vermelho). "
            "Leve o robo ate cada um e grave a pose em Mapeamento.")
    if sem_pose_area:
        dados["avisos"].append(f"{sem_pose_area} service area(s) sem posicao gravada.")
    if not dados["tem_keepout"]:
        dados["avisos"].append(
            "Sem paredes virtuais nesta arena (fita de piso e invisivel ao LiDAR).")
    if not dados["waypoints"]:
        dados["avisos"].append("Nenhum waypoint salvo.")
    if not (pasta / "ws_table_mapping.yaml").is_file():
        dados["avisos"].append(
            "Esta arena nao diz em que altura o braco trabalha em cada bancada: "
            "missoes com manipulacao nao rodam aqui.")

    dados.update(_resumo_da_arena(dados))
    return dados


def rotulo_de_tipo_de_area(tipo: str) -> str:
    for t in TIPOS_DE_AREA:
        if t["valor"] == tipo:
            return t["rotulo"]
    return tipo or ""


def _resumo_da_arena(dados: dict) -> dict:
    """As perguntas que o MENU PRINCIPAL faz sobre a arena.

    Ficam calculadas aqui e nao na tela porque as duas interfaces tem que dar a
    MESMA resposta: "da para mandar o robo para a base?" nao pode depender de
    qual delas o operador abriu.
    """
    base = next((d for d in dados["docks"] if d["papel"] == "inicio"), None)
    pronta = True
    motivo = ""
    if not dados["docks"]:
        pronta, motivo = False, "Esta arena ainda nao tem docks gravados."
    elif any(d["sem_pose"] for d in dados["docks"]):
        pronta, motivo = False, ("Ha dock sem posicao gravada: a missao aborta ao "
                                 "chegar nele.")
    elif base is None:
        pronta, motivo = False, "Esta arena nao tem o ponto de inicio (START)."
    return {
        "base": {
            "existe": base is not None,
            "id": base["id"] if base else "",
            "sem_pose": bool(base["sem_pose"]) if base else False,
        },
        "pronta_para_missao": pronta,
        "motivo_nao_pronta": motivo,
    }


def resumo(nome: str) -> dict:
    """So o que o menu principal precisa saber da arena (cartoes e barra)."""
    dados = metadados(nome)
    if "erro" in dados:
        return {"erro": dados["erro"], "base": {"existe": False}, "pronta_para_missao": False,
                "motivo_nao_pronta": dados["erro"]}
    return {
        "nome": dados["nome"],
        "base": dados["base"],
        "pronta_para_missao": dados["pronta_para_missao"],
        "motivo_nao_pronta": dados["motivo_nao_pronta"],
        "docks": len(dados["docks"]),
        "areas": len(dados["areas"]),
        "waypoints": len(dados["waypoints"]),
        "tem_keepout": dados["tem_keepout"],
        "avisos": list(dados["avisos"]),
    }


# ------------------------------------------------------------------ imagens
def _imagem_em_cache(chave: str, etag: str):
    with _trava:
        guardado = _cache_imagens.get(chave)
    return guardado[1] if guardado and guardado[0] == etag else None


def _guardar_imagem(chave: str, etag: str, dados: bytes):
    with _trava:
        _cache_imagens[chave] = (etag, dados)


def _etag(caminho: Path) -> str:
    st = caminho.stat()
    return f'"{int(st.st_mtime)}-{st.st_size}"'


def mapa_png(nome: str) -> tuple:
    """Devolve (bytes_png, etag). O ETag vem do mtime+tamanho do .pgm.

    O PNG fica guardado junto com o ETag: o navegador revalida a cada
    carregamento de tela, e recodificar um PGM de 700x1000 a cada pedido
    gastaria CPU do mesmo computador que roda a navegacao.
    """
    pasta = pasta_da_arena(nome)
    if pasta is None:
        return None, None
    mapa = _ler_yaml(pasta / "map.yaml")
    imagem = pasta / mapa.get("image", "map.pgm")
    if not imagem.is_file():
        return None, None

    etag = _etag(imagem)
    guardado = _imagem_em_cache(f"mapa:{nome}", etag)
    if guardado is not None:
        return guardado, etag

    from PIL import Image
    with Image.open(imagem) as img:
        buf = io.BytesIO()
        img.convert("L").save(buf, format="PNG", optimize=True)
    png = buf.getvalue()
    _guardar_imagem(f"mapa:{nome}", etag, png)
    return png, etag


def keepout_png(nome: str) -> tuple:
    """A mascara de paredes virtuais, ja tingida, para o canvas so sobrepor.

    A GUI converte a mascara a cada repintura e pinta de vermelho translucido
    onde o operador marcou (a mascara nasce toda branca). Aqui a conversao
    acontece UMA vez, no robo: mandar a mascara crua obrigaria o navegador a
    varrer 700 mil pixels em JavaScript a cada troca de arena.
    """
    pasta = pasta_da_arena(nome)
    if pasta is None:
        return None, None
    mascara = pasta / "keepout_mask.pgm"
    if not mascara.is_file():
        return None, None

    etag = _etag(mascara)
    guardado = _imagem_em_cache(f"keepout:{nome}", etag)
    if guardado is not None:
        return guardado, etag

    from PIL import Image
    with Image.open(mascara) as img:
        cinza = img.convert("L")
        # 110 de 255 e a mesma opacidade que map_preview.cpp usa (qRgba(...,110)):
        # da para ver a parede virtual E o mapa por baixo dela.
        alfa = cinza.point(lambda v: 110 if v < 128 else 0)
        tingida = Image.new("RGBA", cinza.size, (235, 87, 87, 0))
        tingida.putalpha(alfa)
        buf = io.BytesIO()
        tingida.save(buf, format="PNG", optimize=True)
    png = buf.getvalue()
    _guardar_imagem(f"keepout:{nome}", etag, png)
    return png, etag
