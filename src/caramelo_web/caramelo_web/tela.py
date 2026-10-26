"""Espelho da tela do computador do robô, em MJPEG.

SÓ VISUALIZAÇÃO. O painel mostra a tela; não move o mouse nem digita nela.
Quem precisa agir tem a aba Terminal, que já passa pela mesma chave de acesso.
Controle remoto de verdade (VNC, RDP) pediria instalar servidor e mexer em
permissão de sistema — foi decidido não fazer isso.

Por que MJPEG e não vídeo de verdade: MJPEG é uma sequência de JPEGs num
`multipart/x-mixed-replace`, que QUALQUER navegador desenha dentro de um `<img>`
sem plugin, sem codec e sem WebRTC. O objetivo aqui é ACOMPANHAR ("o Gazebo
travou?", "aquele terminal está reclamando de quê?"), não gravar vídeo — por
isso ~10 fps, JPEG de qualidade média e largura reduzida. Numa rede de
competição a banda que este espelho gastar sai da navegação.

Captura via `mss` (X11/XWayland). Neste robô o ambiente é WSLg: existe
WAYLAND_DISPLAY, mas também existe DISPLAY=:0 com um XWayland servindo o mesmo
desktop, e é por ele que o mss enxerga a tela.
"""
import asyncio
import io
import os
from concurrent.futures import ThreadPoolExecutor

LIMITE = b"quadrocaramelo"

# Tetos de segurança da rede, não preferências. Passar disso não melhora nada
# que o operador consiga ver e come banda que a navegação precisa.
LARGURA_PADRAO = 1280
LARGURA_MAXIMA = 1920
QUALIDADE_PADRAO = 55
FPS_PADRAO = 10.0
FPS_MAXIMO = 15.0

# Cada espelho aberto segura uma thread e uma conexão com o servidor gráfico.
# Dois já é "eu e mais alguém olhando"; além disso é desperdício.
TETO_DE_ESPELHOS = 2

_ligados = set()


def _ha_ambiente_grafico() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def diagnostico() -> dict:
    """Dá para espelhar a tela agora? Resposta em linguagem de operador.

    Roda ANTES de começar o stream justamente para que a falha vire uma frase
    na tela em vez de uma conexão que abre e morre sem explicação nenhuma.
    """
    if not _ha_ambiente_grafico():
        return {
            "disponivel": False,
            "mensagem": (
                "Este computador está sem tela ligada (o robô subiu sem "
                "ambiente gráfico), então não há tela para espelhar."),
        }
    try:
        import mss  # noqa: F401
    except ImportError:
        return {
            "disponivel": False,
            "mensagem": (
                "O espelho de tela não está instalado neste robô. Quem cuida "
                "do robô precisa instalá-lo uma vez (veja o LEIAME do painel)."),
        }
    try:
        espelho = Espelho()
        espelho.quadro()
        espelho.fechar()
    except Exception as erro:
        return {
            "disponivel": False,
            "mensagem": f"Não consegui fotografar a tela do robô: {erro}",
        }
    return {"disponivel": True, "mensagem": "Tela disponível."}


class Espelho:
    """Uma conexão com o servidor gráfico. NÃO atravessa threads.

    O mss guarda a conexão com o X dentro de um objeto que só vale na thread
    que o criou; usar o mesmo objeto de duas threads dá erro de protocolo
    intermitente (a imagem sai rasgada ou a captura falha do nada). Por isso
    cada transmissão cria o seu e o usa numa thread só.
    """

    def __init__(self, largura_max: int = LARGURA_PADRAO,
                 qualidade: int = QUALIDADE_PADRAO):
        self.largura_max = max(320, min(LARGURA_MAXIMA, int(largura_max)))
        self.qualidade = max(20, min(90, int(qualidade)))
        self._sct = None

    def _conectar(self):
        if self._sct is None:
            import mss
            # mss.MSS e nao mss.mss(): a fabrica em minusculas foi marcada como
            # obsoleta no mss 10 e cospe DeprecationWarning no log do robo a
            # cada espelho aberto.
            self._sct = mss.MSS()
        return self._sct

    def quadro(self) -> bytes:
        from PIL import Image
        sct = self._conectar()
        # monitors[0] é a área que junta TODOS os monitores. É o que o operador
        # espera de "a tela do robô" quando existe mais de um.
        bruto = sct.grab(sct.monitors[0])
        imagem = Image.frombytes("RGB", bruto.size, bruto.bgra, "raw", "BGRX")
        if imagem.width > self.largura_max:
            altura = max(1, round(imagem.height * self.largura_max / imagem.width))
            # BILINEAR e não LANCZOS: a diferença some num JPEG de qualidade 55
            # e o LANCZOS custa CPU que o robô precisa para navegar.
            imagem = imagem.resize((self.largura_max, altura), Image.BILINEAR)
        buffer = io.BytesIO()
        imagem.save(buffer, format="JPEG", quality=self.qualidade, optimize=False)
        return buffer.getvalue()

    def fechar(self) -> None:
        if self._sct is not None:
            try:
                self._sct.close()
            except Exception:
                pass
            self._sct = None


class Transmissao:
    """Um espelho ligado: a vaga, a thread de captura e a conexão com o X.

    Existe como OBJETO, e não como um gerador solto, por causa de como o
    servidor encerra um stream. MEDIDO (2026-08-18): quando o navegador fecha a
    janela do espelho, o gerador do corpo fica suspenso no `yield` e só é
    finalizado quando o coletor de lixo passa por ele -- o `finally` pode
    demorar muito. Com a contabilidade morando lá, fechar e reabrir a janela
    respondia "já há espelhos de tela demais" e travava o recurso.

    Aqui quem devolve a vaga é `encerrar()`, chamado pelo servidor como tarefa
    de fim de resposta (BackgroundTask), que roda de forma determinística tanto
    no fim normal quanto na desconexão. O `finally` do gerador chama o mesmo
    método -- ele é idempotente de propósito.
    """

    def __init__(self, largura_max: int, qualidade: int, fps: float):
        self.fps = max(1.0, min(FPS_MAXIMO, float(fps)))
        self.espelho = Espelho(largura_max, qualidade)
        # Uma thread SÓ, e sempre a mesma: é o que mantém o objeto do mss preso
        # à thread que o criou.
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="tela")
        self._encerrada = False
        _ligados.add(self)

    async def quadros(self):
        """Gerador do corpo multipart."""
        laco = asyncio.get_running_loop()
        intervalo = 1.0 / self.fps
        try:
            while not self._encerrada:
                inicio = laco.time()
                try:
                    jpeg = await laco.run_in_executor(self.executor, self.espelho.quadro)
                except Exception:
                    # A sessão gráfica caiu no meio (usuário deslogou, Gazebo
                    # levou o X junto). Encerrar o stream limpo é melhor do que
                    # despejar exceção: o <img> só para de atualizar.
                    return
                yield (b"--" + LIMITE + b"\r\n"
                       b"Content-Type: image/jpeg\r\n"
                       b"Content-Length: " + str(len(jpeg)).encode("ascii") +
                       b"\r\n\r\n" + jpeg + b"\r\n")
                resto = intervalo - (laco.time() - inicio)
                if resto > 0:
                    await asyncio.sleep(resto)
        finally:
            await self.encerrar()

    async def encerrar(self) -> None:
        if self._encerrada:
            return
        self._encerrada = True
        _ligados.discard(self)
        # A limpeza vai para a MESMA thread da captura; fechar a conexão com o X
        # de fora dela é o mesmo erro de thread que se evitou lá em cima.
        try:
            laco = asyncio.get_running_loop()
            await laco.run_in_executor(self.executor, self.espelho.fechar)
        except Exception:
            pass
        self.executor.shutdown(wait=False)


def lotado() -> bool:
    return len(_ligados) >= TETO_DE_ESPELHOS
