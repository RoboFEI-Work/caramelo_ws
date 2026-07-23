#!/usr/bin/env python3

import queue
import re
import shutil
import subprocess
import threading
from pathlib import Path
from tempfile import NamedTemporaryFile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeechNode(Node):

    def __init__(self):
        super().__init__('manip_speech')

        self.declare_parameter('backend', 'espeak')
        self.declare_parameter('executable', 'espeak-ng')
        self.declare_parameter('voice', 'pt-br')
        self.declare_parameter('rate', 165)
        self.declare_parameter('pitch', 45)
        self.declare_parameter('word_gap', 0)
        self.declare_parameter('volume', 100)
        self.declare_parameter('piper_executable', 'piper-tts')
        self.declare_parameter('piper_model', '')
        self.declare_parameter('player_executable', 'auto')
        self.declare_parameter('voice_effect', 'none')
        self.declare_parameter('effect_sample_rate', 22050)
        self.declare_parameter('queue_size', 10)

        self._backend = str(self.get_parameter('backend').value)
        self._executable = str(self.get_parameter('executable').value)
        self._voice = str(self.get_parameter('voice').value)
        self._rate = int(self.get_parameter('rate').value)
        self._pitch = int(self.get_parameter('pitch').value)
        self._word_gap = int(self.get_parameter('word_gap').value)
        self._volume = int(self.get_parameter('volume').value)
        self._piper_executable = str(
            self.get_parameter('piper_executable').value
        )
        self._piper_executable = self._resolve_piper_executable(
            self._piper_executable
        )
        self._piper_model = str(self.get_parameter('piper_model').value)
        self._player_executable = str(
            self.get_parameter('player_executable').value
        )
        self._player_executable = self._resolve_player_executable(
            self._player_executable
        )
        self._voice_effect = str(self.get_parameter('voice_effect').value)
        self._effect_sample_rate = int(
            self.get_parameter('effect_sample_rate').value
        )
        configured_queue_size = (
            self.get_parameter('queue_size').value
        )
        queue_size = max(1, int(configured_queue_size))

        self._speech_available = self._log_backend_status()
        self._messages = queue.Queue(maxsize=queue_size)
        self._running = True
        self._worker = threading.Thread(
            target=self._speech_worker,
            daemon=True,
        )
        self._worker.start()

        self.create_subscription(String, '/manip/speech', self._on_speech, 10)

    def _log_backend_status(self):
        if self._backend == 'piper':
            if shutil.which(self._piper_executable) is None:
                return self._fallback_to_espeak(
                    "Executável Piper TTS não encontrado. "
                    'Instale o Piper para usar voz local por IA.'
                )
            if not self._piper_supports_model_option():
                return self._fallback_to_espeak(
                    f"Executável '{self._piper_executable}' não parece ser "
                    "o Piper TTS: ele não aceita a opção '--model'."
                )
            if not self._piper_model:
                return self._fallback_to_espeak(
                    "Parâmetro 'piper_model' vazio. Informe o caminho do "
                    'modelo .onnx para usar o backend piper.'
                )
            if not Path(self._piper_model).exists():
                return self._fallback_to_espeak(
                    f"Modelo Piper não encontrado: '{self._piper_model}'"
                )
            if not self._player_executable:
                return self._fallback_to_espeak(
                    "Nenhum player de áudio encontrado. Instale ffmpeg "
                    "para usar ffplay, ou alsa-utils para usar aplay."
                )
            if (
                self._voice_effect != 'none' and
                Path(self._player_executable).name != 'ffplay'
            ):
                self.get_logger().warning(
                    f"Efeito de voz '{self._voice_effect}' requer ffplay. "
                    f"Usando {self._player_executable} sem efeito."
                )
            self.get_logger().info(
                'Síntese local Piper pronta em /manip/speech, '
                f'modelo={self._piper_model}, player={self._player_executable}, '
                f'efeito={self._voice_effect}'
            )
            return True

        return self._log_espeak_status()

    def _log_espeak_status(self):
        if shutil.which(self._executable) is None:
            self.get_logger().error(
                f"Executável '{self._executable}' não encontrado. "
                'Instale com: sudo apt install espeak-ng'
            )
            return False
        else:
            self.get_logger().info(
                f'Síntese de voz pronta em /manip/speech, voz={self._voice}'
            )
            return True

    def _fallback_to_espeak(self, reason):
        if shutil.which(self._executable) is None:
            self.get_logger().error(reason)
            self.get_logger().error(
                f"Fallback espeak indisponível: executável "
                f"'{self._executable}' não encontrado."
            )
            return False

        self.get_logger().warning(
            f'{reason} Usando fallback espeak em /manip/speech.'
        )
        self._backend = 'espeak'
        return self._log_espeak_status()

    def _on_speech(self, message):
        text = self._normalize_text(message.data)
        if not text:
            return

        try:
            self._messages.put_nowait(text)
        except queue.Full:
            self.get_logger().warning(
                'Fila de voz cheia; descartando a mensagem mais antiga.'
            )
            try:
                self._messages.get_nowait()
                self._messages.task_done()
            except queue.Empty:
                pass
            self._messages.put_nowait(text)

    @staticmethod
    def _normalize_text(text):
        """Remove formatting artifacts that produce unnatural pauses."""
        text = text.replace('_', ' ')
        return re.sub(r'\s+', ' ', text).strip()

    def _speech_command(self, text):
        return [
            self._executable,
            '-v', self._voice,
            '-s', str(self._rate),
            '-p', str(self._pitch),
            '-g', str(self._word_gap),
            '-a', str(self._volume),
            '-z',
            text,
        ]

    def _piper_command(self, output_file):
        return [
            self._piper_executable,
            '--model', self._piper_model,
            '--output_file', str(output_file),
        ]

    def _piper_supports_model_option(self):
        if not self._piper_executable:
            return False

        try:
            result = subprocess.run(
                [self._piper_executable, '--help'],
                capture_output=True,
                text=True,
                check=False,
                timeout=5,
            )
        except (FileNotFoundError, PermissionError, subprocess.TimeoutExpired):
            return False

        help_text = f'{result.stdout}\n{result.stderr}'
        return '--model' in help_text

    def _voice_effect_filter(self):
        if Path(self._player_executable).name != 'ffplay':
            return ''

        sample_rate = max(8000, self._effect_sample_rate)
        if self._voice_effect == 'dog':
            return (
                f'asetrate={sample_rate}*0.94,aresample={sample_rate},'
                'atempo=1.04,'
                'vibrato=f=4:d=0.04,'
                'volume=1.05'
            )
        if self._voice_effect == 'dog_deep':
            return (
                f'asetrate={sample_rate}*0.82,aresample={sample_rate},'
                'atempo=1.10,'
                'vibrato=f=5:d=0.08,'
                'volume=1.10'
            )
        if self._voice_effect == 'cartoon_dog':
            return (
                f'asetrate={sample_rate}*0.82,aresample={sample_rate},'
                'atempo=1.12,'
                'tremolo=f=7:d=0.18,vibrato=f=5:d=0.15'
            )
        return ''

    def _play_audio_command(self, audio_file):
        player_name = Path(self._player_executable).name
        if player_name in ('aplay', 'pw-play', 'paplay'):
            return [self._player_executable, str(audio_file)]

        command = [
            self._player_executable,
            '-nodisp',
            '-autoexit',
            '-loglevel',
            'error',
        ]
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

        candidates = [
            str(Path.home() / 'piper' / 'piper'),
            str(Path.home() / '.local' / 'bin' / 'piper'),
            'piper-tts',
            'piper',
        ]
        for candidate in candidates:
            resolved = shutil.which(candidate)
            if not resolved:
                continue
            self._piper_executable = resolved
            if self._piper_supports_model_option():
                return resolved
        return ''

    def _speak_with_piper(self, text):
        if not self._speech_available:
            return

        with NamedTemporaryFile(suffix='.wav', delete=False) as audio_file:
            audio_path = Path(audio_file.name)

        try:
            result = subprocess.run(
                self._piper_command(audio_path),
                input=text,
                text=True,
                check=False,
                capture_output=True,
                timeout=30,
            )
            if result.returncode != 0 or audio_path.stat().st_size == 0:
                self.get_logger().error(
                    'Piper TTS falhou ao gerar áudio.'
                )
                return
            subprocess.run(
                self._play_audio_command(audio_path),
                check=False,
                timeout=30,
            )
        finally:
            audio_path.unlink(missing_ok=True)

    def _speak(self, text):
        if not self._speech_available:
            return

        if self._backend == 'piper':
            self._speak_with_piper(text)
            return

        subprocess.run(
            self._speech_command(text),
            check=False,
            timeout=30,
        )

    def _speech_worker(self):
        while self._running:
            try:
                text = self._messages.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                self._speak(text)
            except (FileNotFoundError, PermissionError):
                pass
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    'A síntese de voz excedeu 30 segundos.'
                )
            finally:
                self._messages.task_done()

    def destroy_node(self):
        self._running = False
        self._worker.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
