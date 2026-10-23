#!/usr/bin/env python3
"""speech_node: a voz do robo (eventos de manipulacao, docking e missao).

O que faz: assina /manip/speech (std_msgs/String), sintetiza e toca cada
frase sem bloquear quem publica. A fala NUNCA trava uma acao (os nos so'
publicam) e a frase em reproducao NUNCA e' cortada. O que pode acontecer e'
uma frase ser DESCARTADA antes de ser dita, quando ja envelheceu ou foi
superada por outra mais nova — e isso e' logado e publicado, para o
operador saber o que o robo deixou de falar.

Prioridade pelo PREFIXO do texto (removido antes de falar):
  "!texto"  alta   — nunca descartada (resultado: entregou, falhou, pegou).
  "~texto"  baixa  — descartada assim que chegar qualquer frase mais nova.
  "texto"   normal — descartada se esperar mais que max_age_s na fila ou se
                     houver mais que max_pending_normal normais na frente.
Texto repetido dentro de dedupe_s nao e' dito de novo (salvo alta).

Pipeline: fila com politica (SpeechQueue, pura e testavel) -> sintese
(backend) com cache em disco por texto/voz (frases fixas viram WAV uma vez)
e pre-sintese da proxima frase enquanto a atual toca -> player (ffplay/
aplay/pw-play). Publica /manip/speech_status (String, latched: "falando: ...",
"ocioso", "descartada (...): ...") e /manip/speech_busy (Bool, latched).

Backends (parametro `backend`):
  * kokoro: Kokoro-82M em processo (torch CPU), vozes pt-BR pf_dora /
    pm_alex / pm_santa. Pacotes num venv (kokoro_site_packages, default
    ~/caramelo_tts_venv/lib/python3.12/site-packages). Sem o venv ou sem o
    modelo (primeira carga baixa do Hugging Face), cai para piper.
  * piper: binario (piper_executable, auto = ~/piper/piper, ...) +
    piper_model (.onnx). Voz recomendada: pt_BR-faber-medium.
  * espeak: fallback final (espeak-ng -w).
Os motores formam uma cadeia (pedido -> piper -> espeak): se o pedido nao
sobe, ou se uma frase falha nele, a proxima da cadeia assume aquela frase.
Com o modelo do Kokoro ja em cache (~/.cache/huggingface) o no' roda OFFLINE
(arena sem internet) sem esperar timeout do Hugging Face.
Efeito de voz (voice_effect: none | dog | dog_deep | cartoon_dog) continua
disponivel via filtro do ffplay; com Kokoro o default e' none.

Quem lanca: manip_bringup/launch/manip_stack.launch.py (use_audio). Testes
puros: test/test_speech_node.py. Como rodar na mao:
  ros2 run manip_audio speech_node --ros-args -p backend:=kokoro
  ros2 topic pub -1 /manip/speech std_msgs/String "{data: '!Teste de voz'}"
"""

import hashlib
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import wave
from dataclasses import dataclass, field
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

PRIORITY_LOW = 0
PRIORITY_NORMAL = 1
PRIORITY_HIGH = 2
PRIORITY_NAMES = {PRIORITY_LOW: 'baixa', PRIORITY_NORMAL: 'normal', PRIORITY_HIGH: 'alta'}


# ---------------------------------------------------------------------------
# Nucleo puro (sem ROS): texto, prioridade e politica da fila
# ---------------------------------------------------------------------------
def parse_speech_text(raw: str) -> Tuple[str, int]:
    """Separa o prefixo de prioridade ('!' alta, '~' baixa) e normaliza o
    texto (underscores viram espaco; espacos repetidos somem)."""
    text = (raw or '').strip()
    priority = PRIORITY_NORMAL
    if text.startswith('!'):
        priority = PRIORITY_HIGH
        text = text[1:]
    elif text.startswith('~'):
        priority = PRIORITY_LOW
        text = text[1:]
    return normalize_text(text), priority


def normalize_text(text: str) -> str:
    """Remove artefatos que viram pausas estranhas na sintese."""
    text = (text or '').replace('_', ' ')
    return re.sub(r'\s+', ' ', text).strip()


# O codigo dos nos escreve as frases em ASCII (convencao do time). O sintetizador
# le "nao" como "náo", entao devolvemos os acentos das palavras usuais antes de
# falar. So' palavras inteiras, sem diferenciar maiusculas (a inicial e' mantida).
PRONUNCIA = {
    'nao': 'não', 'braco': 'braço', 'posicao': 'posição', 'posicoes': 'posições',
    'camera': 'câmera', 'aproximacao': 'aproximação', 'forca': 'força',
    'solucao': 'solução', 'ordenacao': 'ordenação', 'estacao': 'estação',
    'atencao': 'atenção', 'espaco': 'espaço', 'esforco': 'esforço',
    'alcanca': 'alcança', 'alcancar': 'alcançar', 'comecando': 'começando',
    'ja': 'já', 'so': 'só', 'ate': 'até', 'esta': 'está', 'sai': 'saí',
    'proxima': 'próxima', 'proximo': 'próximo', 'numero': 'número',
    'memoria': 'memória', 'tolerancias': 'tolerâncias', 'intermediaria': 'intermediária',
    'concluida': 'concluída', 'ultima': 'última', 'tambem': 'também', 'entao': 'então',
    'voce': 'você', 'ola': 'olá', 'tres': 'três', 'rapido': 'rápido', 'vazio': 'vazio',
    'direcao': 'direção', 'informacao': 'informação', 'conexao': 'conexão',
    'navegacao': 'navegação', 'manipulacao': 'manipulação', 'calibracao': 'calibração',
    'automatico': 'automático', 'inicio': 'início', 'distancia': 'distância',
    'obstaculo': 'obstáculo', 'obstaculos': 'obstáculos', 'dificil': 'difícil',
}


def pronounce(text: str) -> str:
    """Devolve os acentos das palavras conhecidas (ver PRONUNCIA)."""
    def fix(m):
        w = m.group(0)
        acc = PRONUNCIA.get(w.lower())
        if acc is None:
            return w
        return acc.capitalize() if w[0].isupper() else acc
    return re.sub(r'[A-Za-z]+', fix, text)


@dataclass
class Utterance:
    text: str
    priority: int = PRIORITY_NORMAL
    t_recv: float = 0.0
    seq: int = 0


@dataclass
class Dropped:
    utterance: Utterance
    reason: str


@dataclass
class SpeechQueue:
    """Fila de frases com politica de descarte. Pura: recebe 'now' de fora.

    - alta: nunca descartada, sempre antes das normais/baixas;
    - normal: descartada se esperar mais que max_age_s, ou se houver mais que
      max_pending_normal normais pendentes (ficam as mais novas);
    - baixa: descartada assim que chegar qualquer frase mais nova;
    - texto igual ao ultimo dito/pendente dentro de dedupe_s: descartado
      (salvo alta).
    """
    max_age_s: float = 4.0
    max_pending_normal: int = 2
    dedupe_s: float = 8.0
    _items: List[Utterance] = field(default_factory=list)
    _recent: Dict[str, float] = field(default_factory=dict)
    _seq: int = 0

    def push(self, text: str, priority: int, now: float) -> Tuple[Optional[Utterance], List[Dropped]]:
        """Enfileira. Devolve (frase enfileirada ou None, descartadas)."""
        dropped: List[Dropped] = []
        text = normalize_text(text)
        if not text:
            return None, dropped
        last = self._recent.get(text)
        if priority != PRIORITY_HIGH and last is not None and now - last < self.dedupe_s:
            utt = Utterance(text, priority, now, self._seq)
            return None, [Dropped(utt, 'repetida')]
        self._seq += 1
        utt = Utterance(text, priority, now, self._seq)
        # Chegou frase nova: as baixas pendentes perdem a vez.
        keep: List[Utterance] = []
        for it in self._items:
            if it.priority == PRIORITY_LOW:
                dropped.append(Dropped(it, 'superada'))
            else:
                keep.append(it)
        self._items = keep
        self._items.append(utt)
        self._recent[text] = now
        # Excesso de normais: ficam as mais novas.
        normals = [it for it in self._items if it.priority == PRIORITY_NORMAL]
        excess = len(normals) - max(0, int(self.max_pending_normal))
        if excess > 0:
            for it in normals[:excess]:
                self._items.remove(it)
                dropped.append(Dropped(it, 'fila cheia'))
        return utt, dropped

    def pop(self, now: float) -> Tuple[Optional[Utterance], List[Dropped]]:
        """Proxima frase a falar (alta primeiro, depois ordem de chegada),
        descartando as normais que envelheceram. None se nao ha nada."""
        dropped: List[Dropped] = []
        while self._items:
            highs = [it for it in self._items if it.priority == PRIORITY_HIGH]
            utt = highs[0] if highs else self._items[0]
            self._items.remove(utt)
            age = now - utt.t_recv
            if utt.priority == PRIORITY_NORMAL and age > self.max_age_s:
                dropped.append(Dropped(utt, f'atrasada {age:.1f} s'))
                continue
            return utt, dropped
        return None, dropped

    def peek(self) -> Optional[Utterance]:
        """Proxima frase sem retirar (para pre-sintese)."""
        if not self._items:
            return None
        highs = [it for it in self._items if it.priority == PRIORITY_HIGH]
        return highs[0] if highs else self._items[0]

    def pending(self) -> int:
        return len(self._items)

    def clear(self) -> None:
        self._items.clear()


def cache_key(backend: str, voice: str, text: str) -> str:
    """Nome do WAV em cache: hash do backend + voz + texto normalizado."""
    h = hashlib.sha1(f'{backend}|{voice}|{normalize_text(text)}'.encode('utf-8')).hexdigest()
    return f'{h[:20]}.wav'


def write_wav_int16(path: Path, samples, sample_rate: int) -> None:
    """Grava float32 [-1, 1] (ou int16) como WAV mono 16 bits."""
    import numpy as np  # local: a lib do no' nao precisa de numpy para os testes puros

    arr = np.asarray(samples)
    if arr.dtype != np.int16:
        arr = np.clip(arr, -1.0, 1.0)
        arr = (arr * 32767.0).astype(np.int16)
    with wave.open(str(path), 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(sample_rate))
        w.writeframes(arr.tobytes())


# ---------------------------------------------------------------------------
# Backends de sintese: text -> WAV
# ---------------------------------------------------------------------------
class TtsBackend:
    name = 'base'
    voice = ''

    def synth(self, text: str, out_path: Path) -> bool:  # pragma: no cover - interface
        raise NotImplementedError


class EspeakBackend(TtsBackend):
    name = 'espeak'

    def __init__(self, executable='espeak-ng', voice='pt-br', rate=165, pitch=45, word_gap=0, volume=100):
        self.executable, self.voice = executable, voice
        self.rate, self.pitch, self.word_gap, self.volume = rate, pitch, word_gap, volume

    def command(self, text: str, out_path: Optional[Path] = None) -> List[str]:
        cmd = [self.executable, '-v', self.voice, '-s', str(self.rate), '-p', str(self.pitch),
               '-g', str(self.word_gap), '-a', str(self.volume), '-z']
        if out_path is not None:
            cmd += ['-w', str(out_path)]
        cmd.append(text)
        return cmd

    def synth(self, text: str, out_path: Path) -> bool:
        res = subprocess.run(self.command(text, out_path), check=False, capture_output=True, timeout=30)
        return res.returncode == 0 and out_path.exists() and out_path.stat().st_size > 44


class PiperBackend(TtsBackend):
    name = 'piper'

    def __init__(self, executable: str, model: str):
        self.executable, self.model = executable, model
        self.voice = Path(model).stem

    def command(self, out_path: Path) -> List[str]:
        return [self.executable, '--model', self.model, '--output_file', str(out_path)]

    def synth(self, text: str, out_path: Path) -> bool:
        res = subprocess.run(self.command(out_path), input=text, text=True, check=False,
                             capture_output=True, timeout=30)
        return res.returncode == 0 and out_path.exists() and out_path.stat().st_size > 44


class KokoroBackend(TtsBackend):
    """Kokoro-82M em processo. Carrega o modelo uma vez (primeira sintese
    ~segundos; a primeira carga baixa pesos do Hugging Face). Limita as
    threads do torch para nao disputar CPU com a navegacao."""
    name = 'kokoro'
    sample_rate = 24000

    def __init__(self, voice='pm_alex', speed=1.0, site_packages='', threads=4, lang_code='p'):
        self.voice, self.speed, self.threads, self.lang_code = voice, float(speed), int(threads), lang_code
        self.site_packages = site_packages
        self._pipe = None
        self._lock = threading.Lock()
        self.error = ''

    def load(self) -> bool:
        with self._lock:
            if self._pipe is not None:
                return True
            try:
                if self.site_packages and self.site_packages not in sys.path and Path(self.site_packages).is_dir():
                    sys.path.insert(0, self.site_packages)
                if (Path.home() / '.cache' / 'huggingface' / 'hub' / 'models--hexgrad--Kokoro-82M').is_dir():
                    # Modelo ja baixado: nao consultar o Hub (sem internet na arena
                    # cada arquivo esperaria o timeout de 10 s).
                    os.environ.setdefault('HF_HUB_OFFLINE', '1')
                import torch
                torch.set_num_threads(max(1, self.threads))
                from kokoro import KPipeline
                self._pipe = KPipeline(lang_code=self.lang_code, repo_id='hexgrad/Kokoro-82M')
                return True
            except Exception as exc:  # noqa: BLE001 - qualquer falha = backend indisponivel
                self.error = f'{exc.__class__.__name__}: {exc}'
                self._pipe = None
                return False

    def synth(self, text: str, out_path: Path) -> bool:
        if not self.load():
            return False
        import numpy as np
        chunks = []
        with self._lock:
            for _graphemes, _phonemes, audio in self._pipe(text, voice=self.voice, speed=self.speed):
                chunks.append(np.asarray(audio, dtype=np.float32))
        if not chunks:
            return False
        write_wav_int16(out_path, np.concatenate(chunks), self.sample_rate)
        return True


# ---------------------------------------------------------------------------
# No' ROS
# ---------------------------------------------------------------------------
class SpeechNode(Node):

    def __init__(self):
        super().__init__('speech_node')

        self.declare_parameter('backend', 'kokoro')
        # espeak
        self.declare_parameter('executable', 'espeak-ng')
        self.declare_parameter('voice', 'pt-br')
        self.declare_parameter('rate', 165)
        self.declare_parameter('pitch', 45)
        self.declare_parameter('word_gap', 0)
        self.declare_parameter('volume', 100)
        # piper
        self.declare_parameter('piper_executable', 'auto')
        self.declare_parameter('piper_model', '')
        # kokoro
        self.declare_parameter('kokoro_voice', 'pm_alex')
        self.declare_parameter('kokoro_speed', 1.0)
        self.declare_parameter('kokoro_site_packages',
                               str(Path.home() / 'caramelo_tts_venv' / 'lib' / 'python3.12' / 'site-packages'))
        self.declare_parameter('kokoro_threads', 4)
        # player / efeito
        self.declare_parameter('player_executable', 'auto')
        self.declare_parameter('voice_effect', 'none')
        self.declare_parameter('effect_sample_rate', 22050)
        # politica da fila
        self.declare_parameter('max_age_s', 4.0)
        self.declare_parameter('max_pending_normal', 2)
        self.declare_parameter('dedupe_s', 8.0)
        self.declare_parameter('cache_dir', str(Path.home() / '.cache' / 'caramelo_tts'))
        self.declare_parameter('warmup_text', 'Voz pronta')
        self.declare_parameter('queue_size', 10)   # compatibilidade (nao usado pela politica nova)

        self._backend = str(self.get_parameter('backend').value)
        self._executable = str(self.get_parameter('executable').value)
        self._voice = str(self.get_parameter('voice').value)
        self._rate = int(self.get_parameter('rate').value)
        self._pitch = int(self.get_parameter('pitch').value)
        self._word_gap = int(self.get_parameter('word_gap').value)
        self._volume = int(self.get_parameter('volume').value)
        self._piper_executable = self._resolve_piper_executable(str(self.get_parameter('piper_executable').value))
        self._piper_model = str(self.get_parameter('piper_model').value) or self._default_piper_model()
        self._player_executable = self._resolve_player_executable(str(self.get_parameter('player_executable').value))
        self._voice_effect = str(self.get_parameter('voice_effect').value)
        self._effect_sample_rate = int(self.get_parameter('effect_sample_rate').value)

        self._queue = SpeechQueue(
            max_age_s=float(self.get_parameter('max_age_s').value),
            max_pending_normal=int(self.get_parameter('max_pending_normal').value),
            dedupe_s=float(self.get_parameter('dedupe_s').value))
        self._queue_lock = threading.Lock()
        self._wakeup = threading.Event()
        self._prefetch_lock = threading.Lock()
        self._cache_dir = Path(str(self.get_parameter('cache_dir').value))
        self._cache_dir.mkdir(parents=True, exist_ok=True)
        self._running = True
        self._current_text = ''

        # Cadeia de motores: o pedido primeiro, depois as reservas.
        self._backends: List[TtsBackend] = []
        self._speech_available = self._setup_backends()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._status_pub = self.create_publisher(String, '/manip/speech_status', latched)
        self._busy_pub = self.create_publisher(Bool, '/manip/speech_busy', latched)
        self._publish_state(False, 'ocioso')

        self.create_subscription(String, '/manip/speech', self._on_speech, 10)
        self._worker = threading.Thread(target=self._speech_worker, daemon=True)
        self._worker.start()

    # ------------------------------------------------------------ backends
    @property
    def _tts(self) -> Optional[TtsBackend]:
        """Motor principal (primeiro da cadeia), ou None sem voz."""
        return self._backends[0] if self._backends else None

    def _setup_backends(self) -> bool:
        """Monta a cadeia [pedido, reservas...] e avisa o que subiu."""
        if self._backend == 'kokoro':
            k = KokoroBackend(
                voice=str(self.get_parameter('kokoro_voice').value),
                speed=float(self.get_parameter('kokoro_speed').value),
                site_packages=str(self.get_parameter('kokoro_site_packages').value),
                threads=int(self.get_parameter('kokoro_threads').value))
            if k.load():
                self._backends.append(k)
                self.get_logger().info(
                    f'Voz Kokoro pronta em /manip/speech (voz={k.voice}, velocidade={k.speed:.2f}, '
                    f'threads={k.threads}, cache={self._cache_dir})')
            else:
                self.get_logger().warning(f'Kokoro indisponivel ({k.error}) — tentando o Piper.')
        if self._backend in ('kokoro', 'piper'):
            reason = self._piper_unavailable_reason()
            if reason is None:
                self._backends.append(PiperBackend(self._piper_executable, self._piper_model))
                if len(self._backends) == 1:
                    self.get_logger().info(
                        f'Voz Piper pronta em /manip/speech (modelo={Path(self._piper_model).name}, '
                        f'player={self._player_executable}, efeito={self._voice_effect})')
            elif len(self._backends) == 0:
                self.get_logger().warning(f'{reason} — usando o espeak.')
        if shutil.which(self._executable) is not None:
            self._backends.append(EspeakBackend(self._executable, self._voice, self._rate, self._pitch,
                                                self._word_gap, self._volume))
            if len(self._backends) == 1:
                self.get_logger().info(f'Voz espeak pronta em /manip/speech (voz={self._voice})')
        if not self._backends:
            self.get_logger().error(
                f"Sem voz: nem Kokoro, nem Piper, nem '{self._executable}' (sudo apt install espeak-ng).")
            return False
        if not self._player_executable:
            self.get_logger().error('Sem voz: nenhum player de audio (ffplay/aplay/pw-play).')
            return False
        self.get_logger().info('Cadeia de voz: ' + ' -> '.join(b.name for b in self._backends))
        self._warmup()
        return True

    @staticmethod
    def _default_piper_model() -> str:
        """pt_BR-faber instalado no share do pacote ('' se nao instalado)."""
        try:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('manip_audio')) / 'models' / 'pt_BR-faber' / 'pt_BR-faber-medium.onnx'
        except Exception:  # noqa: BLE001 - sem ament_index = sem modelo default
            return ''
        return str(path) if path.exists() else ''

    def _piper_unavailable_reason(self) -> Optional[str]:
        if not self._piper_executable or shutil.which(self._piper_executable) is None:
            return 'Executavel do Piper nao encontrado'
        if not self._piper_supports_model_option():
            return f"'{self._piper_executable}' nao e' o Piper TTS (sem opcao --model)"
        if not self._piper_model:
            return "Parametro 'piper_model' vazio"
        if not Path(self._piper_model).exists():
            return f"Modelo Piper nao encontrado: '{self._piper_model}'"
        if not self._player_executable:
            return 'Nenhum player de audio (ffplay/aplay/pw-play)'
        return None

    def _warmup(self) -> None:
        """Primeira sintese fora do caminho critico (carrega modelo/voz). Entra
        na fila como baixa: e' dita se nada chegar antes — confirma ao operador
        que o audio funciona."""
        text = pronounce(normalize_text(str(self.get_parameter('warmup_text').value)))
        if text:
            self._enqueue(text, PRIORITY_LOW)

    # ------------------------------------------------------------ entrada
    def _on_speech(self, message: String) -> None:
        text, priority = parse_speech_text(message.data)
        if text:
            self._enqueue(pronounce(text), priority)

    def _enqueue(self, text: str, priority: int) -> None:
        now = time.monotonic()
        with self._queue_lock:
            utt, dropped = self._queue.push(text, priority, now)
        for d in dropped:
            self._log_dropped(d)
        if utt is not None:
            self._wakeup.set()
            # Pre-sintese imediata: quando chegar a vez, o WAV ja existe.
            threading.Thread(target=self._synth_cached, args=(utt.text,), daemon=True).start()

    def _log_dropped(self, d: Dropped) -> None:
        self.get_logger().info(f'[VOZ] descartada ({d.reason}, {PRIORITY_NAMES[d.utterance.priority]}): {d.utterance.text}')
        self._status_pub.publish(String(data=f'descartada ({d.reason}): {d.utterance.text}'))

    # ------------------------------------------------------------ sintese
    def _synth_cached(self, text: str) -> Optional[Path]:
        """WAV do texto (cache ou sintese) pelo primeiro motor da cadeia que
        conseguir. None se todos falharem."""
        for backend in self._backends:
            path = self._cache_dir / cache_key(backend.name, backend.voice, text)
            if path.exists() and path.stat().st_size > 44:
                return path
            with self._prefetch_lock:
                if path.exists() and path.stat().st_size > 44:
                    return path
                with NamedTemporaryFile(suffix='.wav', dir=str(self._cache_dir), delete=False) as tmp:
                    tmp_path = Path(tmp.name)
                t0 = time.monotonic()
                try:
                    ok = backend.synth(text, tmp_path)
                except (subprocess.TimeoutExpired, OSError, RuntimeError, ValueError, KeyError) as exc:
                    ok = False
                    self.get_logger().error(f'[VOZ] {backend.name} falhou ({exc.__class__.__name__}: {exc}): {text}')
                if ok:
                    os.replace(tmp_path, path)
                    self.get_logger().debug(
                        f'[VOZ] sintetizada ({backend.name}) em {(time.monotonic() - t0) * 1000:.0f} ms: {text}')
                    return path
                tmp_path.unlink(missing_ok=True)
                if backend is self._backends[-1]:
                    self.get_logger().error(f'[VOZ] sintese falhou em todos os motores: {text}')
                else:
                    self.get_logger().warning(f'[VOZ] {backend.name} nao sintetizou, tentando a reserva: {text}')
        return None

    # ------------------------------------------------------------ player
    def _play(self, audio_file: Path) -> None:
        if not self._player_executable:
            return
        subprocess.run(self._play_audio_command(audio_file), check=False, timeout=60)

    def _publish_state(self, busy: bool, status: str) -> None:
        self._busy_pub.publish(Bool(data=bool(busy)))
        self._status_pub.publish(String(data=status))

    def _speech_worker(self) -> None:
        while self._running:
            with self._queue_lock:
                utt, dropped = self._queue.pop(time.monotonic())
            for d in dropped:
                self._log_dropped(d)
            if utt is None:
                self._wakeup.wait(timeout=0.2)
                self._wakeup.clear()
                continue
            wav = self._synth_cached(utt.text)
            if wav is None:
                continue
            # Pre-sintese da proxima enquanto esta toca (nao bloqueia).
            with self._queue_lock:
                nxt = self._queue.peek()
            if nxt is not None:
                threading.Thread(target=self._synth_cached, args=(nxt.text,), daemon=True).start()
            self._current_text = utt.text
            self.get_logger().info(f'[VOZ] falando ({PRIORITY_NAMES[utt.priority]}, esperou '
                                   f'{time.monotonic() - utt.t_recv:.1f} s): {utt.text}')
            self._publish_state(True, f'falando: {utt.text}')
            try:
                self._play(wav)
            except (FileNotFoundError, PermissionError, subprocess.TimeoutExpired) as exc:
                self.get_logger().warning(f'[VOZ] player falhou ({exc.__class__.__name__}).')
            finally:
                self._current_text = ''
                self._publish_state(False, 'ocioso')

    # ------------------------------------------------------------ utilidades (compat com testes)
    @staticmethod
    def _normalize_text(text):
        return normalize_text(text)

    def _speech_command(self, text):
        return EspeakBackend(self._executable, self._voice, self._rate, self._pitch,
                             self._word_gap, self._volume).command(text)

    def _piper_command(self, output_file):
        return PiperBackend(self._piper_executable, self._piper_model).command(Path(str(output_file)))

    def _piper_supports_model_option(self):
        if not self._piper_executable:
            return False
        try:
            result = subprocess.run([self._piper_executable, '--help'], capture_output=True, text=True,
                                    check=False, timeout=5)
        except (FileNotFoundError, PermissionError, subprocess.TimeoutExpired):
            return False
        return '--model' in f'{result.stdout}\n{result.stderr}'

    def _voice_effect_filter(self):
        if Path(self._player_executable).name != 'ffplay':
            return ''
        sample_rate = max(8000, self._effect_sample_rate)
        if self._voice_effect == 'dog':
            return (f'asetrate={sample_rate}*0.94,aresample={sample_rate},atempo=1.04,'
                    'vibrato=f=4:d=0.04,volume=1.05')
        if self._voice_effect == 'dog_deep':
            return (f'asetrate={sample_rate}*0.82,aresample={sample_rate},atempo=1.10,'
                    'vibrato=f=5:d=0.08,volume=1.10')
        if self._voice_effect == 'cartoon_dog':
            return (f'asetrate={sample_rate}*0.82,aresample={sample_rate},atempo=1.12,'
                    'tremolo=f=7:d=0.18,vibrato=f=5:d=0.15')
        return ''

    def _play_audio_command(self, audio_file):
        player_name = Path(self._player_executable).name
        if player_name in ('aplay', 'pw-play', 'paplay'):
            return [self._player_executable, str(audio_file)]
        command = [self._player_executable, '-nodisp', '-autoexit', '-loglevel', 'error']
        audio_filter = self._voice_effect_filter()
        if audio_filter:
            command.extend(['-af', audio_filter])
        command.append(str(audio_file))
        return command

    @staticmethod
    def _resolve_player_executable(player_executable):
        if player_executable and player_executable != 'auto':
            return player_executable
        for candidate in ('ffplay', 'pw-play', 'paplay', 'aplay'):
            resolved = shutil.which(candidate)
            if resolved:
                return resolved
        return ''

    def _resolve_piper_executable(self, piper_executable):
        if piper_executable and piper_executable != 'auto':
            return piper_executable
        for candidate in (str(Path.home() / 'piper' / 'piper'), str(Path.home() / '.local' / 'bin' / 'piper'),
                          'piper-tts', 'piper'):
            resolved = shutil.which(candidate)
            if not resolved:
                continue
            self._piper_executable = resolved
            if self._piper_supports_model_option():
                return resolved
        return ''

    def _speak(self, text):
        """Fala sincrona (so' para testes/uso manual): sintetiza e toca."""
        if not self._speech_available:
            return
        wav = self._synth_cached(pronounce(normalize_text(text)))
        if wav is not None:
            self._play(wav)

    def destroy_node(self):
        self._running = False
        self._wakeup.set()
        self._worker.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
