"""Mapeamento pelo painel web: criar a arena e tudo o que vive dentro dela.

A GUI Qt ja faz isso (caramelo_gui/src/modules/mapeamento/ferramenta_mapeamento.cpp)
e ela e a fonte da verdade: mesma sequencia, mesmos programas, MESMO formato de
arquivo. Um mapa criado aqui tem que abrir la, e vice-versa -- se as duas
interfaces gravassem diferente, a arena feita numa quebraria na outra e o
operador so descobriria na prova.

A SEQUENCIA e ordenada de proposito, e a ordem e do operador:

  1. mapear            o SLAM roda enquanto alguem dirige o robo pelo ambiente
  2. limpar o ruido    a mao, no bitmap: a pessoa que passou vira chao de novo
  3. marcar os pontos  as estacoes e tambem o START -- que NEM SEMPRE fica onde
                       o mapeamento comecou, entao nada aqui assume isso
  4. paredes virtuais  o que o LiDAR nao enxerga (fita de piso, degrau, vidro)

Onde cada coisa fica gravada, tudo dentro da pasta da arena:
  map.pgm / map.yaml          o mapa; quem escreve e o map_saver do nav2
  service_areas.yaml          os pontos, no formato de caramelo_service_area_common
  docking.yaml                derivado das areas, POR MERGE (ver adiante)
  keepout_mask.pgm / .yaml    as paredes virtuais

POR QUE ESTE MODULO NAO FALA ROS DIRETO. O painel ja tem um no (ros_node.py) e o
rclpy ja esta inicializado neste processo; os programas de gravacao chamam
rclpy.init() por conta propria, entao rodar o main() deles aqui dentro seria
inicializar duas vezes. Dai as duas metades deste arquivo:

  - para GRAVAR ARQUIVO importamos os MODULOS compartilhados
    (caramelo_service_area_common / caramelo_docking_common). Sao codigo puro de
    arquivo, nao tocam em ROS, e sao exatamente o que save_service_area_pose.py
    executa depois de descobrir a pose. Importar o modulo -- em vez de reescrever
    o formato aqui -- e o que garante que a web grava igual a GUI.

  - para MEXER NO GRAFO (subir o SLAM, salvar o mapa) rodamos PROCESSO SEPARADO,
    que e o que a GUI tambem faz (core/launch_runner.cpp).

As listas fechadas (tipos de estacao, nome de ponto, arena ativa) NAO moram
aqui: sao as de arenas.py, que ja e o gemeo do map_preview.cpp da GUI. Duas
copias de uma lista sao duas chances de divergir.
"""
import os
import re
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from caramelo_web import arenas

# Nome de arena e nome de PASTA e vira argumento de launch. Sem esta regra um
# "../../.ssh" vindo do navegador viraria caminho de escrita; e um espaco no
# nome ja basta para o launch da navegacao nao achar o mapa depois.
REGRA_DE_NOME_DE_MAPA = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$")

# Alturas de mesa do regulamento (caramelo_service_area_common.VALID_HEIGHTS).
ALTURAS_DE_MESA = [0.00, 0.05, 0.10, 0.15]

# Pincel do bitmap do mapa. Os tres valores sao os da GUI
# (modules/editor_mapa/editor_mapa_module.cpp) e os do nav2 em modo trinary.
VALORES_DO_MAPA = {"livre": 254, "parede": 0, "desconhecido": 205}

# Pincel da mascara de keepout. ATENCAO: o livre da mascara e 255 e o do mapa e
# 254. Nao e descuido -- e o que init_keepout_mask.py grava, e a mascara e lida
# em modo 'scale', onde o valor do pixel vira custo direto.
VALORES_DA_MASCARA = {"parede": 0, "livre": 255}

# Quanto uma mancha solta pode ter, em metros quadrados, para a limpeza
# automatica considera-la ruido. Fica em AREA e nao em pixel porque um mapa a
# 0,02 m/celula e outro a 0,05 tem contagens de pixel completamente diferentes
# para a mesma sujeira no chao.
NIVEIS_DE_RUIDO = {"leve": 0.004, "medio": 0.012, "forte": 0.030}

PACOTE_DO_SLAM = "caramelo_mapping"
LAUNCH_DO_SLAM = "slam.launch.py"

PASTA_DE_LOG = Path.home() / ".cache/caramelo_web"
LOG_DO_MAPEAMENTO = PASTA_DE_LOG / "mapeamento.log"


# --------------------------------------------------------------- utilidades
def _anotar(texto: str) -> None:
    """Escreve no log do mapeamento. E onde a saida tecnica vai morar.

    Nenhuma saida de programa chega ao operador: ele nao le ROS. O que ele ve e
    uma frase em portugues; o motivo cru fica aqui, para quem for depurar pelo
    terminal do painel.
    """
    try:
        PASTA_DE_LOG.mkdir(parents=True, exist_ok=True)
        with LOG_DO_MAPEAMENTO.open("a", encoding="utf-8") as f:
            f.write(f"[{datetime.now().isoformat(timespec='seconds')}] {texto}\n")
    except OSError:
        pass


def _rodar(comando: list, tempo_limite: float = 60.0) -> tuple:
    """Roda um programa e devolve (codigo, saida). Nunca levanta por tempo.

    Codigo -1 significa "estourou o tempo"; -2, "nem chegou a executar".
    """
    try:
        fim = subprocess.run(
            comando, capture_output=True, text=True, timeout=tempo_limite)
    except subprocess.TimeoutExpired:
        _anotar(f"TEMPO ESGOTADO: {' '.join(comando)}")
        return -1, ""
    except (OSError, ValueError) as erro:
        _anotar(f"NAO EXECUTOU: {' '.join(comando)} -> {erro}")
        return -2, str(erro)
    saida = (fim.stdout or "") + (fim.stderr or "")
    if fim.returncode != 0:
        _anotar(f"CODIGO {fim.returncode}: {' '.join(comando)}\n{saida.strip()}")
    return fim.returncode, saida


def _falha(mensagem: str, http: int = 409, **extra) -> dict:
    """Resposta de recusa. 409 por padrao, pelo mesmo motivo do resto da API:
    nada quebrou, o robo e que nao estava em condicao de obedecer."""
    resposta = {"ok": False, "mensagem": mensagem, "http": http}
    resposta.update(extra)
    return resposta


def _feito(mensagem: str, **extra) -> dict:
    resposta = {"ok": True, "mensagem": mensagem}
    resposta.update(extra)
    return resposta


# ------------------------------------------------------- codigo compartilhado
_modulos = {}


def _modulo(nome: str):
    """Importa um modulo do caramelo_navigation pelo nome. Cacheado.

    Eles moram em lib/caramelo_navigation, ao lado dos programas que os usam --
    nao sao um pacote Python instalado, entao a pasta precisa entrar no sys.path
    na mao. A arvore FONTE entra junto porque, com
    `colcon build --symlink-install`, o que esta em install/ e link para src/, e
    um workspace ainda nao construido so tem a fonte.
    """
    if nome in _modulos:
        return _modulos[nome]

    if not _modulos:
        candidatos = []
        try:
            from ament_index_python.packages import get_package_prefix
            candidatos.append(
                Path(get_package_prefix("caramelo_navigation")) /
                "lib/caramelo_navigation")
        except Exception:
            pass
        candidatos.append(Path.home() / "caramelo_ws/src/caramelo_navigation/scripts")
        for pasta in candidatos:
            if pasta.is_dir() and str(pasta) not in sys.path:
                sys.path.insert(0, str(pasta))

    import importlib
    _modulos[nome] = importlib.import_module(nome)
    return _modulos[nome]


def _comuns():
    """O modulo que grava service_areas.yaml e docking.yaml."""
    return _modulo("caramelo_service_area_common")


# --------------------------------------------------------------- validacoes
def erro_de_nome_de_mapa(nome: str) -> str:
    """Frase pronta para a tela; vazia quando o nome serve.

    Nome de mapa e o UNICO texto livre desta parte do painel: um mapa novo, por
    definicao, ainda nao existe para estar numa lista.
    """
    nome = (nome or "").strip()
    if not nome:
        return "De um nome ao mapa antes de salvar."
    if not REGRA_DE_NOME_DE_MAPA.match(nome):
        return (f'"{nome}" nao serve como nome de mapa. Use letras, numeros, '
                "hifen e sublinhado, sem espacos e sem acentos -- por exemplo "
                "arena_regional.")
    return ""


def _pasta_da_arena(nome: str):
    """(pasta, "") ou (None, motivo). O nome vem da URL, ou seja, de fora."""
    pasta = arenas.pasta_da_arena(nome)
    if pasta is None:
        return None, (f"Nao encontrei a arena '{(nome or '').strip()}' neste robo. "
                      "Escolha uma das que aparecem na lista.")
    return pasta, ""


# ------------------------------------------------------------------- o SLAM
class _Mapeador:
    """O SLAM como processo filho do painel. Espelha o LaunchRunner da GUI."""

    def __init__(self):
        self._trava = threading.Lock()
        self._processo = None
        self._inicio = 0.0
        self._ultimo_salvo = None
        self._ultimo_fim = None

    def _rodando(self) -> bool:
        return self._processo is not None and self._processo.poll() is None

    def _colher(self) -> None:
        """Percebe que o filho morreu sozinho (LiDAR mudo, conflito, falta de RAM)."""
        if self._processo is not None and self._processo.poll() is not None:
            self._ultimo_fim = {"codigo": int(self._processo.returncode),
                                "quando": time.time()}
            self._processo = None

    def ligado(self) -> bool:
        with self._trava:
            self._colher()
            return self._rodando()

    def estado(self, nos=None) -> dict:
        with self._trava:
            self._colher()
            ligado = self._rodando()
            segundos = int(time.monotonic() - self._inicio) if ligado else 0
            ultimo_salvo = dict(self._ultimo_salvo) if self._ultimo_salvo else None
            ultimo_fim = dict(self._ultimo_fim) if self._ultimo_fim else None

        # Mapeamento ligado por FORA do painel (pela GUI, ou por um painel
        # anterior que foi morto a forca) e um estado real e confuso: o botao
        # daqui nao consegue desligar aquilo. Melhor dizer do que fingir.
        por_fora = bool(not ligado and nos and
                        any("slam_toolbox" in str(n) for n in nos))

        if ligado:
            mensagem = "Mapeando. Dirija o robo devagar, passando por todos os cantos."
        elif por_fora:
            mensagem = ("O mapeamento esta ligado, mas nao foi este painel que "
                        "ligou. Desligue por onde ligou.")
        elif ultimo_fim and ultimo_fim["codigo"] not in (0, -2):
            mensagem = ("O mapeamento parou sozinho. Confira se o LiDAR esta "
                        "respondendo e tente de novo.")
        else:
            mensagem = "Mapeamento desligado."

        return {
            "ok": True,
            "ligado": ligado,
            "ligado_por_fora": por_fora,
            "segundos": segundos,
            "mensagem": mensagem,
            "pode_salvar": ligado,
            "ultimo_mapa_salvo": ultimo_salvo,
            "ultimo_encerramento": ultimo_fim,
            "arena_ativa": arenas.arena_ativa(),
            # As listas que a tela precisa para NAO ter campo de texto livre.
            # Vem daqui junto com o estado porque a tela de mapeamento abre com
            # uma chamada so.
            "opcoes": {
                "tipos_de_area": arenas.tipos_de_area(),
                "alturas_de_mesa": ALTURAS_DE_MESA,
                "pincel_do_mapa": sorted(VALORES_DO_MAPA),
                "pincel_das_paredes": sorted(VALORES_DA_MASCARA),
                "niveis_de_ruido": ["leve", "medio", "forte"],
                "origens_de_ponto": ["no_mapa", "robo_aqui"],
            },
        }

    def iniciar(self, nos=None) -> dict:
        with self._trava:
            self._colher()
            if self._rodando():
                return _falha("O mapeamento ja esta ligado.")

            try:
                PASTA_DE_LOG.mkdir(parents=True, exist_ok=True)
                log = LOG_DO_MAPEAMENTO.open("a", encoding="utf-8")
            except OSError:
                log = None

            comando = [
                "ros2", "launch", PACOTE_DO_SLAM, LAUNCH_DO_SLAM,
                "use_rviz:=false",
                # Os marcadores leem as areas de um mapa JA salvo, e aqui o mapa
                # esta justamente sendo criado -- o valor padrao do launch e o
                # nome de uma arena que pode nem existir neste robo. Ficam de
                # fora: o painel desenha as areas por conta propria.
                "use_service_area_markers:=false",
            ]
            try:
                processo = subprocess.Popen(
                    comando,
                    stdout=(log or subprocess.DEVNULL), stderr=subprocess.STDOUT,
                    cwd=str(Path.home()),
                    # Sessao propria, e isto NAO e detalhe. Sem ela o filho fica
                    # no mesmo grupo de processos do painel, e o SIGINT que
                    # 'cancelar' manda no grupo do filho cairia no painel
                    # tambem: o servidor se mataria ao desligar o mapeamento. O
                    # preco e que o filho nao morre junto com um Ctrl+C no
                    # painel -- por isso o desligamento chama encerrar_tudo().
                    start_new_session=True)
            except OSError as erro:
                _anotar(f"nao consegui subir o mapeamento: {erro}")
                return _falha("Nao consegui ligar o mapeamento neste robo.")
            finally:
                if log is not None:
                    log.close()

            self._processo = processo
            self._inicio = time.monotonic()
            self._ultimo_fim = None

        aviso = ""
        # A trava mapear-x-navegar (RK-02) vive no proprio launch, e la ela
        # apenas AVISA: um `ros2 node list` pode mostrar no zumbi de um daemon
        # velho, e abortar por causa disso derrubava o launch inteiro antes de
        # qualquer no subir. Aqui e igual -- avisa, nao impede.
        if nos and any(str(n).endswith("amcl") for n in nos):
            aviso = (" Atencao: a navegacao parece estar ligada. O robo nao "
                     "consegue se localizar e mapear ao mesmo tempo; desligue a "
                     "navegacao.")
        return _feito(
            "Mapeamento ligado. Dirija o robo devagar, passando por todos os "
            "cantos." + aviso)

    def cancelar(self) -> dict:
        with self._trava:
            self._colher()
            if not self._rodando():
                return _falha("O mapeamento nao esta ligado.")
            processo = self._processo

        # Mesma escada da GUI (core/launch_runner.cpp): SIGINT no GRUPO, para os
        # nos filhos cairem juntos e limpos, como um Ctrl+C; so depois a forca.
        # Matar de cara deixaria o mapa pela metade e nos orfaos segurando o
        # LiDAR, e o proximo mapeamento subiria em cima deles.
        matou = False
        for sinal, espera in ((signal.SIGINT, 8.0), (signal.SIGTERM, 4.0)):
            try:
                os.killpg(processo.pid, sinal)
            except (ProcessLookupError, PermissionError, OSError):
                matou = True
                break
            try:
                processo.wait(timeout=espera)
                matou = True
                break
            except subprocess.TimeoutExpired:
                continue
        if not matou:
            processo.kill()
            try:
                processo.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass

        with self._trava:
            self._colher()
            self._processo = None
        return _feito("Mapeamento desligado.")

    def encerrar_tudo(self) -> None:
        """Chamado quando o painel esta descendo: nao deixa SLAM orfao."""
        if self.ligado():
            self.cancelar()

    def registrar_salvamento(self, arena: str) -> None:
        with self._trava:
            self._ultimo_salvo = {"arena": arena, "quando": time.time()}


_mapeador = _Mapeador()


def estado(nos=None) -> dict:
    return _mapeador.estado(nos)


def iniciar(nos=None) -> dict:
    return _mapeador.iniciar(nos)


def cancelar() -> dict:
    return _mapeador.cancelar()


def encerrar_tudo() -> None:
    _mapeador.encerrar_tudo()


def mapeando() -> bool:
    return _mapeador.ligado()


# -------------------------------------------------------------- salvar mapa
def salvar_mapa(nome: str) -> dict:
    """Grava o mapa que o SLAM tem agora numa pasta de arena com este nome.

    Usa o map_saver do nav2 com os MESMOS parametros da GUI (pgm, trinary,
    0.196 / 0.65) -- e por isso o arquivo sai identico ao que a GUI produz. E o
    programa de linha de comando, e nao o servico do no ativo: os dois chamam o
    mesmo nav2_map_server e escrevem o mesmo arquivo, mas o programa devolve
    codigo de saida, enquanto o servico devolveria um texto para o painel
    adivinhar se deu certo.
    """
    erro = erro_de_nome_de_mapa(nome)
    if erro:
        return _falha(erro, http=400)
    nome = nome.strip()

    pasta = arenas.pasta_dos_mapas() / nome
    ja_existia = (pasta / "map.yaml").is_file()
    pontos_antigos = 0
    tinha_paredes = False
    if ja_existia:
        antigo = arenas.metadados(nome)
        pontos_antigos = len(antigo.get("areas", [])) + len(antigo.get("docks", []))
        tinha_paredes = bool(antigo.get("tem_keepout"))

    try:
        pasta.mkdir(parents=True, exist_ok=True)
    except OSError as erro_io:
        _anotar(f"mkdir {pasta}: {erro_io}")
        return _falha("Nao consegui criar a pasta desta arena no robo.")

    codigo, _saida = _rodar([
        "ros2", "run", "nav2_map_server", "map_saver_cli",
        "-t", "map",
        "-f", str(pasta / "map"),
        "--occ", "0.65", "--free", "0.196",
        "--fmt", "pgm", "--mode", "trinary",
        # O tempo de espera padrao e curto; o mapa chega latched, mas num robo
        # ocupado a primeira mensagem pode demorar. Vai no fim de proposito: o
        # map_saver_cli exige --ros-args como ultimo bloco.
        "--ros-args", "-p", "save_map_timeout:=15.0",
    ], tempo_limite=60.0)

    if codigo != 0 or not (pasta / "map.yaml").is_file():
        # Pasta vazia deixada por uma tentativa que falhou so atrapalha: ela
        # apareceria na lista de arenas sem mapa nenhum dentro.
        if not ja_existia:
            try:
                pasta.rmdir()
            except OSError:
                pass
        if not _mapeador.ligado():
            return _falha("Nao ha mapa para salvar: o mapeamento nao esta ligado.")
        return _falha("Nao consegui salvar o mapa. Tente de novo em alguns segundos.")

    _mapeador.registrar_salvamento(nome)
    avisos = []
    if pontos_antigos:
        # O mapa novo pode ter outra origem que o antigo, e os pontos gravados
        # sao coordenadas em metros: eles NAO acompanham essa mudanca.
        avisos.append(
            f"Esta arena ja tinha {pontos_antigos} ponto(s) marcado(s). Confira "
            "no mapa se eles continuam no lugar certo.")
    if tinha_paredes:
        avisos.append(
            "As paredes virtuais desta arena foram desenhadas sobre o mapa "
            "anterior. Confira se ainda cobrem o lugar certo.")
    return _feito(f'Mapa salvo como "{nome}".', arena=nome, avisos=avisos)


# ------------------------------------------------------------------- pontos
def gravar_ponto(arena: str, pedido: dict, pose_do_robo=None) -> dict:
    """Grava um ponto da arena (estacao, START ou FINISH).

    pedido:
      id       obrigatorio, do regulamento (START, FINISH, WS1, SH1, PP1, RT1)
      tipo     um dos seis de arenas.TIPOS_DE_AREA; sem ele, deduzido do id
      origem   "no_mapa"   -> x, y, yaw vieram de um clique/arrasto no mapa
               "robo_aqui" -> usa a pose atual do robo (pose_do_robo)
      altura   opcional, uma das do regulamento

    As duas origens existem porque as duas acontecem: marcar clicando no mapa
    nao pode depender de levar o robo ate la, e gravar "onde o robo esta" e o
    unico jeito de acertar a pose de uma mesa de precisao.

    Escreve service_areas.yaml e, POR MERGE, docking.yaml. O merge e o ponto
    delicado: START e FINISH tem use_docking false, mas continuam sendo POSES
    ALVO da navegacao. Ja houve um bug que os apagava do docking.yaml a cada
    gravacao, e o sintoma era o fim da missao falhar dizendo que FINISH nao
    existe. Quem cuida disso e o sync_docking_from_service_areas -- por isso ele
    e CHAMADO, e nao reescrito aqui.
    """
    pasta, erro = _pasta_da_arena(arena)
    if pasta is None:
        return _falha(
            erro + " Se acabou de mapear, salve o mapa primeiro: os pontos "
                   "ficam guardados dentro da arena.", http=404)

    ident = str(pedido.get("id") or "")
    erro = arenas.erro_de_id_de_ponto(ident)
    if erro:
        return _falha(erro, http=400)
    ident = arenas.id_de_ponto_normalizado(ident)

    tipos = arenas.tipos_de_area()
    tipo = str(pedido.get("tipo") or "").strip()
    if tipo and tipo not in [t["valor"] for t in tipos]:
        return _falha(
            "Escolha o tipo da estacao na lista: "
            + ", ".join(t["rotulo"] for t in tipos) + ".", http=400)

    origem = str(pedido.get("origem") or "").strip()
    if origem == "robo_aqui":
        if not pose_do_robo:
            return _falha(
                "O robo nao sabe onde esta agora, entao nao da para gravar a "
                "pose dele. Ligue o mapeamento ou a localizacao e tente de novo.")
        x, y, yaw = pose_do_robo
    elif origem == "no_mapa":
        try:
            x = float(pedido["x"])
            y = float(pedido["y"])
            yaw = float(pedido.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError):
            return _falha("Faltou dizer em que lugar do mapa o ponto fica.",
                          http=400)
    else:
        return _falha(
            "Diga de onde vem a posicao: marcada no mapa ou a pose atual do robo.",
            http=400)

    altura = pedido.get("altura")
    if altura is not None:
        try:
            altura = round(float(altura), 2)
        except (TypeError, ValueError):
            return _falha("Nao entendi a altura da mesa.", http=400)
        if not any(abs(altura - a) < 1.0e-9 for a in ALTURAS_DE_MESA):
            return _falha(
                "A altura da mesa tem que ser uma das do regulamento: "
                + ", ".join(f"{a:.2f} m" for a in ALTURAS_DE_MESA) + ".",
                http=400)

    try:
        area = _comuns()
    except Exception as erro_import:
        _anotar(f"import do modulo de gravacao: {erro_import}")
        return _falha("Este robo nao tem instalada a parte que grava pontos.")

    avisos = []
    esperado = area.type_from_area_id(ident)
    if tipo and tipo != esperado:
        # Nao impede: o operador pode ter um motivo. Mas o tipo de aproximacao
        # do dock e deduzido do NOME e nao do tipo, entao a divergencia tem
        # consequencia na hora de encostar.
        rotulo = next((t["rotulo"] for t in tipos if t["valor"] == esperado),
                      esperado)
        avisos.append(f'O nome {ident} costuma ser "{rotulo}"; '
                      "voce escolheu outro tipo.")

    caminho = area.service_area_path(pasta)
    if not caminho.exists():
        # Nasce VAZIO de proposito. O caminho padrao do modulo criaria oito
        # areas de exemplo em 0,0,0; elas apareceriam na tela como estacoes
        # existentes e, pior, uma missao mandada para uma delas levaria o robo
        # ate a origem do mapa. Melhor a arena so conter o que foi marcado.
        try:
            area.write_yaml(caminho, {"service_areas": {}})
        except OSError as erro_io:
            _anotar(f"criando {caminho}: {erro_io}")
            return _falha("Nao consegui gravar os pontos desta arena.")

    parametros = {}
    if altura is not None:
        parametros["station_height"] = altura
    try:
        area.update_service_area_pose(
            map_folder=pasta,
            map_name=arena,
            area_id=ident,
            pose={"x": x, "y": y, "yaw": yaw},
            area_type=tipo or None,
            frame_id="map",
            # A tela ja e a confirmacao, como na GUI (goal.yes = true). Sem
            # isso, regravar um ponto que ficou torto falharia dizendo que ele
            # ja existe.
            overwrite=True,
            **parametros)
        area.sync_docking_from_service_areas(pasta, make_backup=True)
    except Exception as erro_grav:
        _anotar(f"gravando {ident} em {pasta}: {erro_grav}")
        return _falha(f"Nao consegui gravar o ponto {ident}: {erro_grav}")

    return _feito(
        f"Ponto {ident} gravado em x={x:.2f} m, y={y:.2f} m.",
        ponto={"id": ident, "tipo": tipo or esperado,
               "x": x, "y": y, "yaw": yaw},
        avisos=avisos)


def apagar_ponto(arena: str, ident: str) -> dict:
    """Tira o ponto da arena: das estacoes e tambem do docking.

    Apagar so das estacoes nao bastaria. O docking.yaml e escrito por MERGE (e
    tem que ser, senao START e FINISH somem dele), e merge nunca remove nada: o
    ponto apagado continuaria la, e o robo ainda aceitaria ser mandado para ele.
    """
    pasta, erro = _pasta_da_arena(arena)
    if pasta is None:
        return _falha(erro, http=404)

    ident = arenas.id_de_ponto_normalizado(ident)
    if not arenas.id_de_ponto_aceitavel(ident):
        return _falha(arenas.erro_de_id_de_ponto(ident), http=400)

    try:
        area = _comuns()
    except Exception as erro_import:
        _anotar(f"import do modulo de gravacao: {erro_import}")
        return _falha("Este robo nao tem instalada a parte que grava pontos.")

    achou = False
    try:
        for caminho, chave in ((area.service_area_path(pasta), "service_areas"),
                               (area.docking_path(pasta), "docks")):
            if not caminho.exists():
                continue
            dados = area.load_yaml(caminho)
            grupo = dados.get(chave)
            if isinstance(grupo, dict) and ident in grupo:
                del grupo[ident]
                area.write_yaml(caminho, dados, make_backup=True)
                achou = True
    except Exception as erro_grav:
        _anotar(f"apagando {ident} de {pasta}: {erro_grav}")
        return _falha(f"Nao consegui apagar o ponto {ident}.")

    if not achou:
        return _falha(f"O ponto {ident} nao existe nesta arena.", http=404)
    return _feito(f"Ponto {ident} apagado da arena.")


# ------------------------------------------------------- desenho no bitmap
def _geometria(pasta: Path) -> tuple:
    """(resolucao, origem_x, origem_y) do map.yaml da arena."""
    dados = arenas.metadados(pasta.name)
    return (float(dados.get("resolucao", 0.05)),
            float(dados.get("origem", [0.0, 0.0])[0]),
            float(dados.get("origem", [0.0, 0.0])[1]))


def _para_pixel(x: float, y: float, origem: tuple, resolucao: float,
                altura: int) -> tuple:
    """Metros no mapa -> pixel da imagem.

    Mesma conta do MapPreview da GUI: no map.yaml a origem e a pose do pixel
    INFERIOR esquerdo, e a imagem cresce para BAIXO. Inverter o y aqui e o que
    impede a parede virtual de sair espelhada em relacao a GUI -- e parede
    virtual espelhada e o robo entrando exatamente onde nao pode.
    """
    return ((x - origem[0]) / resolucao,
            altura - (y - origem[1]) / resolucao)


def _preparar_pinceladas(brutas, valores: dict) -> tuple:
    """Confere a lista de pinceladas vinda do navegador. Devolve (lista, erro).

    Cada pincelada e um ponto (x, y) ou um traco (x0, y0 -> x1, y1), sempre em
    METROS, com raio em metros. Em metros e nao em pixel porque a tela nao sabe
    (nem deve saber) a resolucao do mapa: a mesma pincelada tem que valer igual
    num mapa de 2 cm e num de 5 cm por celula.
    """
    if not isinstance(brutas, list):
        return None, "Nao entendi o desenho que veio da tela."
    if len(brutas) > 5000:
        return None, "Desenho grande demais de uma vez so. Grave por partes."

    lista = []
    for bruta in brutas:
        if not isinstance(bruta, dict):
            return None, "Nao entendi o desenho que veio da tela."
        valor = str(bruta.get("valor") or "")
        if valor not in valores:
            return None, "Escolha o que pintar: " + ", ".join(sorted(valores)) + "."
        try:
            raio = float(bruta.get("raio_m", 0.05))
        except (TypeError, ValueError):
            return None, "Nao entendi o tamanho do pincel."
        raio = max(0.01, min(2.0, raio))
        try:
            if "x1" in bruta:
                pontos = (float(bruta["x0"]), float(bruta["y0"]),
                          float(bruta["x1"]), float(bruta["y1"]))
            else:
                pontos = (float(bruta["x"]), float(bruta["y"]),
                          float(bruta["x"]), float(bruta["y"]))
        except (KeyError, TypeError, ValueError):
            return None, "Nao entendi onde o desenho fica no mapa."
        lista.append((valores[valor], raio, pontos))
    return lista, ""


def _pintar(imagem, pinceladas, origem, resolucao, altura) -> None:
    from PIL import ImageDraw
    desenho = ImageDraw.Draw(imagem)
    for valor, raio_m, (x0, y0, x1, y1) in pinceladas:
        raio = max(0.5, raio_m / resolucao)
        px0, py0 = _para_pixel(x0, y0, origem, resolucao, altura)
        px1, py1 = _para_pixel(x1, y1, origem, resolucao, altura)
        if (px0, py0) != (px1, py1):
            desenho.line([(px0, py0), (px1, py1)], fill=valor,
                         width=max(1, int(round(raio * 2))))
        # As pontas redondas vao a parte: a linha do PIL tem ponta reta, e um
        # traco desenhado como varios segmentos ficaria com falha nos cantos.
        for px, py in ((px0, py0), (px1, py1)):
            desenho.ellipse([px - raio, py - raio, px + raio, py + raio],
                            fill=valor)


def _quantos_mudaram(antes, depois) -> int:
    from PIL import ImageChops
    return sum(ImageChops.difference(antes, depois).histogram()[1:])


def _salvar_pgm(imagem, caminho: Path) -> None:
    """Grava P5 de 8 bits, que e o que o nav2 le.

    O format="PPM" do PIL, com a imagem em tons de cinza, escreve exatamente o
    cabecalho P5 com largura, altura e 255, seguido dos bytes crus -- o mesmo
    que o map_saver gera e que o init_keepout_mask.py escreve na mao.
    """
    imagem.convert("L").save(str(caminho), format="PPM")


# -------------------------------------------------------- limpeza de ruido
def _manchas_soltas(pixels: bytearray, largura: int, altura: int,
                    limite: int) -> list:
    """Manchas de pixel ocupado com ate `limite` pixels, 8-vizinhos.

    Uma mancha encostada numa parede de verdade nao aparece aqui: ela faz parte
    do mesmo grupo da parede, que e grande. Ou seja, so sai do mapa o que esta
    SOLTO no meio do nada -- a pessoa que passou na frente do LiDAR durante o
    mapeamento, o reflexo do vidro, a perna de cadeira que ja foi embora.
    """
    visitado = bytearray(largura * altura)
    achadas = []
    for inicio in range(largura * altura):
        if visitado[inicio] or pixels[inicio] > 100:
            continue
        visitado[inicio] = 1
        fila = [inicio]
        grupo = []
        grande = False
        while fila:
            atual = fila.pop()
            if not grande:
                grupo.append(atual)
                if len(grupo) > limite:
                    # Passou do tamanho: e obstaculo de verdade. A busca
                    # CONTINUA mesmo assim, so parando de guardar os pixels.
                    #
                    # Parar aqui seria o erro perigoso: o resto da parede
                    # ficaria sem visitar, a varredura de fora recomecaria
                    # dentro dela mais adiante, e o ultimo pedaco -- pequeno,
                    # porque o comeco ja foi marcado -- seria apagado como se
                    # fosse sujeira. Ou seja, um buraco no meio de uma parede
                    # de verdade, e o robo entrando por ele.
                    grande = True
                    grupo = []
            cx = atual % largura
            cy = atual // largura
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    vx, vy = cx + dx, cy + dy
                    if vx < 0 or vy < 0 or vx >= largura or vy >= altura:
                        continue
                    vizinho = vy * largura + vx
                    if visitado[vizinho] or pixels[vizinho] > 100:
                        continue
                    visitado[vizinho] = 1
                    fila.append(vizinho)
        if not grande:
            achadas.append(grupo)
    return achadas


def _valor_ao_redor(pixels: bytearray, largura: int, altura: int,
                    grupo: list) -> int:
    """O que havia em volta da mancha: chao livre ou area desconhecida.

    Apagar sempre para "livre" mentiria numa mancha que esta no meio do que o
    LiDAR nunca viu: o robo passaria a achar que ali da para andar.
    """
    dentro = set(grupo)
    livres = desconhecidos = 0
    for indice in grupo:
        cx = indice % largura
        cy = indice // largura
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                vx, vy = cx + dx, cy + dy
                if vx < 0 or vy < 0 or vx >= largura or vy >= altura:
                    continue
                vizinho = vy * largura + vx
                if vizinho in dentro:
                    continue
                if pixels[vizinho] >= 250:
                    livres += 1
                else:
                    desconhecidos += 1
    return (VALORES_DO_MAPA["livre"] if livres >= desconhecidos
            else VALORES_DO_MAPA["desconhecido"])


def editar_ruido(arena: str, pedido: dict) -> dict:
    """Passo 2: limpar o mapa, a mao (pinceladas) e/ou automatico (manchas).

    Escreve no proprio map.pgm, com backup datado ao lado -- exatamente o que o
    editor da GUI faz (modules/editor_mapa/editor_mapa_module.cpp).
    """
    pasta, erro = _pasta_da_arena(arena)
    if pasta is None:
        return _falha(erro, http=404)
    if _mapeador.ligado():
        # Com o SLAM ligado, o proximo "salvar mapa" sobrescreve o arquivo: a
        # limpeza iria para o lixo sem ninguem perceber.
        return _falha(
            "O mapeamento ainda esta ligado. Desligue e salve o mapa antes de "
            "limpar -- senao a limpeza se perde no proximo salvamento.")

    pinceladas, erro = _preparar_pinceladas(
        pedido.get("pinceladas") or [], VALORES_DO_MAPA)
    if erro:
        return _falha(erro, http=400)

    nivel = str(pedido.get("nivel") or "").strip()
    if nivel and nivel not in NIVEIS_DE_RUIDO:
        return _falha("Escolha a forca da limpeza: "
                      + ", ".join(sorted(NIVEIS_DE_RUIDO)) + ".", http=400)
    if not pinceladas and not nivel:
        return _falha("Nao veio nada para limpar.", http=400)

    from PIL import Image
    caminho = pasta / "map.pgm"
    if not caminho.is_file():
        return _falha("Esta arena nao tem imagem de mapa para limpar.", http=404)

    try:
        resolucao, origem_x, origem_y = _geometria(pasta)
        with Image.open(caminho) as bruta:
            imagem = bruta.convert("L")
    except Exception as erro_io:
        _anotar(f"abrindo {caminho}: {erro_io}")
        return _falha("Nao consegui abrir a imagem do mapa desta arena.")

    antes = imagem.copy()
    largura, altura = imagem.size

    if pinceladas:
        _pintar(imagem, pinceladas, (origem_x, origem_y), resolucao, altura)

    manchas = 0
    if nivel:
        limite = max(1, int(round(NIVEIS_DE_RUIDO[nivel] / (resolucao ** 2))))
        pixels = bytearray(imagem.tobytes())
        for grupo in _manchas_soltas(pixels, largura, altura, limite):
            novo = _valor_ao_redor(pixels, largura, altura, grupo)
            for indice in grupo:
                pixels[indice] = novo
            manchas += 1
        if manchas:
            imagem = Image.frombytes("L", (largura, altura), bytes(pixels))

    mudados = _quantos_mudaram(antes, imagem)
    if not mudados:
        return _feito("Nada mudou no mapa.", manchas=0, area_m2=0.0)

    try:
        backup = caminho.with_name(
            "map.pgm.bak_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
        backup.write_bytes(caminho.read_bytes())
        _salvar_pgm(imagem, caminho)
    except OSError as erro_io:
        _anotar(f"gravando {caminho}: {erro_io}")
        return _falha("Nao consegui gravar a imagem do mapa.")

    partes = []
    if manchas:
        partes.append(f"tirei {manchas} mancha(s) solta(s)")
    if pinceladas:
        partes.append("apliquei o que voce pintou")
    return _feito(
        "Mapa limpo: " + " e ".join(partes) + ". A versao anterior ficou guardada.",
        manchas=manchas,
        area_m2=round(mudados * resolucao * resolucao, 3))


# ---------------------------------------------------------- paredes virtuais
def _garantir_mascara(pasta: Path) -> str:
    """Cria keepout_mask.{pgm,yaml} em branco se ainda nao existirem.

    Quem sabe o FORMATO da mascara (canvas todo-livre, modo scale, limiares,
    geometria copiada do mapa) e o init_keepout_mask.py do caramelo_navigation,
    e e ele que escreve aqui -- reescrever o formato neste arquivo seria uma
    segunda fonte da mesma verdade, e as duas divergem no dia em que uma for
    corrigida.

    Sao as funcoes internas dele, e nao o main(): o main le sys.argv, e trocar o
    sys.argv do servidor por baixo de uma requisicao alcancaria as outras que
    estiverem em andamento na mesma hora.
    """
    if (pasta / "keepout_mask.pgm").is_file() and \
            (pasta / "keepout_mask.yaml").is_file():
        return ""
    try:
        km = _modulo("init_keepout_mask")
        largura, altura = km._read_pgm_header(str(pasta / "map.pgm"))
        resolucao, origem = km._read_map_yaml(str(pasta / "map.yaml"))
        km._write_free_pgm(str(pasta / "keepout_mask.pgm"), largura, altura)
        km._write_mask_yaml(str(pasta / "keepout_mask.yaml"), resolucao, origem)
    except Exception as erro:
        _anotar(f"criando keepout em {pasta}: {erro}")
        return "Nao consegui criar as paredes virtuais desta arena."
    return ""


def editar_paredes(arena: str, pedido: dict) -> dict:
    """Passo 4: parede virtual -- o que o LiDAR nao enxerga.

    A mascara e ACUMULATIVA: cada envio pinta por cima do que ja estava.
    "limpar": true zera tudo antes (o "apagar todas as paredes"). Pintar de
    "livre" e a borracha, para desfazer um trecho sem apagar o resto.
    """
    pasta, erro = _pasta_da_arena(arena)
    if pasta is None:
        return _falha(erro, http=404)
    if _mapeador.ligado():
        # A mascara tem que ter a geometria EXATA do map.pgm. Se o mapa for
        # salvo de novo depois, ela deixa de casar -- desenhar agora e desenhar
        # em cima de algo que ainda vai mudar.
        return _falha(
            "O mapeamento ainda esta ligado. Termine e salve o mapa antes de "
            "desenhar as paredes virtuais.")

    pinceladas, erro = _preparar_pinceladas(
        pedido.get("pinceladas") or [], VALORES_DA_MASCARA)
    if erro:
        return _falha(erro, http=400)
    limpar = bool(pedido.get("limpar"))
    if not pinceladas and not limpar:
        return _falha("Nao veio nenhuma parede para gravar.", http=400)

    erro = _garantir_mascara(pasta)
    if erro:
        return _falha(erro)

    from PIL import Image
    caminho = pasta / "keepout_mask.pgm"
    try:
        resolucao, origem_x, origem_y = _geometria(pasta)
        with Image.open(caminho) as bruta:
            imagem = bruta.convert("L")
        with Image.open(pasta / "map.pgm") as mapa:
            tamanho_do_mapa = mapa.size
    except Exception as erro_io:
        _anotar(f"abrindo {caminho}: {erro_io}")
        return _falha("Nao consegui abrir as paredes virtuais desta arena.")

    if imagem.size != tamanho_do_mapa:
        # Acontece quando o mapa foi salvo de novo depois da mascara. As duas
        # imagens precisam casar pixel a pixel, senao a parede virtual sai
        # deslocada no chao de verdade -- e o robo entra onde nao pode.
        return _falha(
            "As paredes virtuais desta arena nao batem com o tamanho do mapa. "
            "Apague as paredes e desenhe de novo.")

    # A copia do ANTES vem antes do limpar de proposito: senao o "apagar tudo"
    # apareceria na tela como "nada mudou".
    antes = imagem.copy()
    if limpar:
        imagem = Image.new("L", imagem.size, VALORES_DA_MASCARA["livre"])
    largura, altura = imagem.size
    if pinceladas:
        _pintar(imagem, pinceladas, (origem_x, origem_y), resolucao, altura)

    mudados = _quantos_mudaram(antes, imagem)
    if not mudados:
        # Acontece quando o traco caiu inteiro fora do mapa. Dizer "gravado"
        # aqui faria o operador achar que a parede existe.
        return _feito("Nada mudou nas paredes virtuais.", area_m2=0.0,
                      area_bloqueada_m2=round(
                          sum(quantos for valor, quantos
                              in enumerate(imagem.histogram()) if valor < 128)
                          * resolucao * resolucao, 3))
    try:
        _salvar_pgm(imagem, caminho)
    except OSError as erro_io:
        _anotar(f"gravando {caminho}: {erro_io}")
        return _falha("Nao consegui gravar as paredes virtuais.")

    bloqueado = sum(quantos for valor, quantos in enumerate(imagem.histogram())
                    if valor < 128)
    return _feito(
        "Paredes virtuais gravadas. O robo passa a desviar delas na proxima vez "
        "que a navegacao for ligada.",
        area_m2=round(mudados * resolucao * resolucao, 3),
        area_bloqueada_m2=round(bloqueado * resolucao * resolucao, 3))
