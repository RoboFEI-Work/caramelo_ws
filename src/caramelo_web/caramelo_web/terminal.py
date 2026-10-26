"""Terminal do robô dentro do navegador: um bash de verdade por conexão.

Por que pty e não um `subprocess` comum: sem terminal de verdade o bash não
entra em modo interativo, o `ros2` não completa com Tab, nada que use curses
desenha, e — o que mais importa numa emergência — o Ctrl+C não vira SIGINT.
Sinal de teclado só existe onde existe terminal de controle, porque é ele que
define qual grupo de processos está em primeiro plano. Um pipe não tem isso.

Cada conexão WebSocket abre UM pty e UM bash, e os dois morrem quando ela cai.
Não há sessão persistente de propósito: um bash órfão segurando `ros2 launch`
depois que o operador fechou a aba é exatamente o tipo de processo fantasma que
depois aparece como "o robô não anda" numa bancada de competição.

Protocolo do WebSocket (assimétrico, e é intencional):
  navegador -> robô : SEMPRE texto JSON. {"t":"in","d":"..."} para o que foi
                      digitado, {"t":"tam","colunas":N,"linhas":M} ao redimensionar.
  robô -> navegador : quadro BINÁRIO é saída crua do terminal; quadro de TEXTO é
                      recado do painel em JSON ({"t":"aviso"} / {"t":"fim"}).

A saída vai em binário porque UTF-8 não pode ser cortado no meio: se o servidor
decodificasse cada pedaço de `os.read`, um acento que caísse na fronteira de
dois blocos viraria lixo permanente na tela. Em binário, quem remonta é o
próprio xterm.js, que sabe esperar o resto do caractere.
"""
import asyncio
import fcntl
import json
import os
import pty
import signal
import struct
import subprocess
import termios
from pathlib import Path

LEITURA = 65536

# Teto de terminais simultâneos. Não é sobre segurança (quem tem a chave já tem
# o robô); é sobre memória: a máquina do robô é apertada e cada bash com um
# `ros2 launch` dentro pesa. Quatro cobre "eu, o mesmo eu num tablet, e sobra".
TETO_DE_SESSOES = 4

_abertas = 0


def _shell() -> str:
    return os.environ.get("SHELL") or "/bin/bash"


def _arquivo_de_rc() -> Path:
    """Escreve (e devolve) o rc que dá ambiente ROS ao terminal do painel.

    Por que um rc próprio em vez de confiar no `bash -l`: shell de login lê
    ~/.profile, e ~/.profile só chama o ~/.bashrc em ALGUNS layouts de Ubuntu.
    Quando não chama, o terminal do painel abre sem ROS_DOMAIN_ID=67 e o
    sintoma é cruel — `ros2 topic list` responde vazio, com o robô inteiro no
    ar, e a conclusão natural do operador é que o robô caiu.

    Sourcear o ~/.caramelo_env.sh de novo (o ~/.bashrc já o carrega na primeira
    linha) é inofensivo: o arquivo só exporta variáveis e define funções.
    """
    destino = Path.home() / ".cache/caramelo_web/terminal.bashrc"
    destino.parent.mkdir(parents=True, exist_ok=True)
    destino.write_text(
        "# Gerado pelo painel web do Caramelo. Nao edite: e' reescrito a cada\n"
        "# terminal aberto.\n"
        "[ -f /etc/bash.bashrc ] && . /etc/bash.bashrc\n"
        '[ -f "$HOME/.bashrc" ] && . "$HOME/.bashrc"\n'
        '[ -f "$HOME/.caramelo_env.sh" ] && . "$HOME/.caramelo_env.sh"\n',
        encoding="utf-8")
    return destino


def _ambiente() -> dict:
    """O ambiente do servidor, mais o que um terminal precisa para desenhar.

    Herdar o ambiente do processo é o ponto: o painel sobe pelo mesmo launch do
    robô, então o terminal nasce com o MESMO overlay sourceado que o resto do
    stack. Um terminal com outro AMENT_PREFIX_PATH mentiria sobre qual versão
    do código está rodando.
    """
    env = dict(os.environ)
    env["TERM"] = "xterm-256color"
    env["COLORTERM"] = "truecolor"
    # Sem locale UTF-8 o bash desenha caixinhas no lugar dos acentos e o
    # readline erra a contagem de colunas ao apagar caractere acentuado.
    if "UTF-8" not in (env.get("LANG") or "").upper():
        env["LANG"] = "C.UTF-8"
    return env


def _ajustar_janela(fd: int, colunas: int, linhas: int) -> None:
    # struct winsize é (linhas, colunas, x_pixels, y_pixels) NESTA ordem. Trocar
    # os dois primeiros é o erro clássico: a tela parece certa até a primeira
    # linha longa, que quebra na coluna errada.
    colunas = max(2, min(500, int(colunas)))
    linhas = max(1, min(300, int(linhas)))
    fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", linhas, colunas, 0, 0))


def _processos_da_sessao(sid: int) -> list:
    """PIDs que pertencem a sessao `sid`, do mais novo para o mais antigo.

    Le /proc em vez de chamar `ps`: o painel roda no robo durante a prova e nao
    e hora de depender de um processo externo. O campo 6 de /proc/<pid>/stat e
    o id da sessao; os anteriores sao pid, comm, estado e ppid. O comm vem entre
    parenteses e PODE conter espaco (e ate parentese), por isso o corte e feito
    no ultimo ')' da linha, nunca por split simples.
    """
    achados = []
    for entrada in os.listdir("/proc"):
        if not entrada.isdigit():
            continue
        try:
            with open("/proc/" + entrada + "/stat", "rb") as f:
                dados = f.read()
        except OSError:
            continue
        fim = dados.rfind(b")")
        if fim < 0:
            continue
        campos = dados[fim + 2:].split()
        if len(campos) < 4:
            continue
        try:
            if int(campos[3]) == sid:
                achados.append(int(entrada))
        except ValueError:
            continue
    # Do maior pid para o menor: os netos morrem antes do bash, para nao ficarem
    # orfaos nem por um instante.
    return sorted(achados, reverse=True)


def _sinalizar(pids: list, sinal) -> None:
    for pid in pids:
        try:
            os.kill(pid, sinal)
        except (ProcessLookupError, PermissionError, OSError):
            pass


class Sessao:
    """Um pty com um bash dentro."""

    def __init__(self, colunas: int = 100, linhas: int = 30):
        self.mestre, escravo = pty.openpty()
        try:
            _ajustar_janela(self.mestre, colunas, linhas)
            self.processo = subprocess.Popen(
                [_shell(), "--rcfile", str(_arquivo_de_rc()), "-i"],
                stdin=escravo, stdout=escravo, stderr=escravo,
                # login_tty(0) e não login_tty(escravo): quando o preexec_fn
                # roda, o Popen já duplicou o escravo em 0/1/2 e já fechou o
                # descritor original (close_fds). Usar o número antigo aqui
                # falha com EBADF e o terminal nunca abre.
                # O login_tty faz o essencial: setsid + TIOCSCTTY. Sem ele o
                # bash não tem terminal de controle e o Ctrl+C não mata nada.
                preexec_fn=lambda: os.login_tty(0),
                cwd=str(Path.home()), env=_ambiente(), close_fds=True)
        except Exception:
            os.close(self.mestre)
            os.close(escravo)
            raise
        os.close(escravo)
        # Não bloqueante porque quem lê é o event loop, não uma thread: um
        # os.read bloqueante aqui congelaria o painel inteiro (mapa, saúde,
        # missão) enquanto o terminal estivesse quieto.
        os.set_blocking(self.mestre, False)
        self._fechada = False

    def escrever(self, dados: bytes) -> None:
        if self._fechada:
            return
        try:
            os.write(self.mestre, dados)
        except OSError:
            # Escrever num pty cujo shell ja morreu falha, e tudo bem: quem
            # encerra a sessao e o lado da LEITURA, ao ver o fim do arquivo.
            # Marcar _fechada aqui seria pior do que o erro -- _fechada e a
            # marca de "ja limpei", entao um write perdido faria o encerrar()
            # sair na primeira linha e deixar bash, pty e netos para tras.
            pass

    def redimensionar(self, colunas: int, linhas: int) -> None:
        if self._fechada:
            return
        try:
            # O kernel manda SIGWINCH sozinho ao grupo de primeiro plano; é
            # assim que o vim e o htop reagem sem ninguém avisá-los.
            _ajustar_janela(self.mestre, colunas, linhas)
        except OSError:
            pass

    async def encerrar(self) -> None:
        if self._fechada:
            return
        self._fechada = True

        # A SESSAO inteira, nao o grupo do bash. MEDIDO (2026-08-18): fechar a
        # aba com um `sleep 300 &` e um `sleep 301` em primeiro plano deixava os
        # dois VIVOS, reparentados ao init. O motivo e que o bash interativo poe
        # CADA job no seu proprio grupo de processos -- entao matar o grupo do
        # bash mata so o bash -- e o SIGHUP que o kernel manda ao fechar o pty
        # vai apenas ao grupo de PRIMEIRO PLANO. O que junta todo mundo e a
        # sessao: o login_tty fez setsid, logo bash e todos os seus jobs
        # compartilham sid == pid do bash. Sem isto, um `ros2 launch` disparado
        # pelo painel seguiria segurando topicos do robo depois que o operador
        # fechou a aba, e o sintoma final seria "o robo nao anda".
        #
        # A lista sai ANTES de fechar o pty, enquanto todos ainda estao vivos.
        familia = _processos_da_sessao(self.processo.pid)
        _sinalizar(familia, signal.SIGHUP)

        try:
            os.close(self.mestre)
        except OSError:
            pass

        for _ in range(20):
            if self.processo.poll() is not None:
                break
            await asyncio.sleep(0.05)

        # Quem ignorou o SIGHUP vai no KILL. A sessao e reconsultada porque no
        # meio da espera podem ter nascido netos novos.
        teimosos = _processos_da_sessao(self.processo.pid)
        if teimosos:
            _sinalizar(teimosos, signal.SIGKILL)
        try:
            self.processo.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass


async def _avisar(ws, tipo: str, texto: str) -> None:
    try:
        await ws.send_text(json.dumps({"t": tipo, "texto": texto}))
    except Exception:
        pass


async def atender(ws, colunas: int = 100, linhas: int = 30) -> None:
    """Serve UM terminal nesta conexão, do accept ao último processo morto."""
    global _abertas

    if _abertas >= TETO_DE_SESSOES:
        await _avisar(
            ws, "fim",
            "Já há terminais demais abertos neste robô. Feche uma das outras "
            "abas de terminal e tente de novo.")
        return

    try:
        sessao = Sessao(colunas, linhas)
    except Exception as erro:
        await _avisar(
            ws, "fim",
            f"Não consegui abrir um terminal neste robô ({erro}). "
            "Isso costuma ser falta de memória: veja se sobrou algum programa "
            "pesado aberto.")
        return

    _abertas += 1
    laco = asyncio.get_running_loop()
    # Fila limitada: um cliente lento (aba em segundo plano, Wi-Fi ruim) não
    # pode fazer a memória do robô crescer sem teto por causa de um `cat` de
    # arquivo grande.
    fila = asyncio.Queue(maxsize=512)
    lendo = True

    def ao_haver_saida():
        nonlocal lendo
        try:
            dados = os.read(sessao.mestre, LEITURA)
        except BlockingIOError:
            return
        except InterruptedError:
            return
        except OSError:
            # EIO no mestre é como o Linux avisa "o outro lado fechou": é o fim
            # normal do shell, não uma falha. Qualquer outro OSError aqui também
            # só pode terminar em fechar a sessão, então o tratamento é um só.
            dados = b""
        if not dados:
            lendo = False
            try:
                laco.remove_reader(sessao.mestre)
            except (OSError, ValueError):
                pass
        if fila.full():
            # Descarta o pedaço MAIS ANTIGO. Numa enxurrada de saída, perder o
            # começo e manter o fim é o que o operador espera de um terminal.
            try:
                fila.get_nowait()
            except asyncio.QueueEmpty:
                pass
        fila.put_nowait(dados)

    laco.add_reader(sessao.mestre, ao_haver_saida)

    async def escoar():
        while True:
            dados = await fila.get()
            if not dados:
                await _avisar(ws, "fim", "O terminal foi encerrado.")
                return
            await ws.send_bytes(dados)

    async def receber():
        while True:
            mensagem = await ws.receive()
            if mensagem.get("type") == "websocket.disconnect":
                return
            texto = mensagem.get("text")
            if texto is None:
                continue
            try:
                pacote = json.loads(texto)
            except (ValueError, TypeError):
                continue
            tipo = pacote.get("t")
            if tipo == "in":
                sessao.escrever(str(pacote.get("d", "")).encode("utf-8"))
            elif tipo == "tam":
                sessao.redimensionar(
                    int(pacote.get("colunas", 100)), int(pacote.get("linhas", 30)))

    tarefas = [asyncio.create_task(escoar()), asyncio.create_task(receber())]
    try:
        await asyncio.wait(tarefas, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for tarefa in tarefas:
            tarefa.cancel()
        # remove_reader ANTES de fechar o descritor: um watcher apontando para
        # fd fechado faz o event loop girar em falso a 100% de CPU, e o painel
        # inteiro (que roda no mesmo loop) engasga junto.
        if lendo:
            try:
                laco.remove_reader(sessao.mestre)
            except (OSError, ValueError):
                pass
        await sessao.encerrar()
        _abertas -= 1
