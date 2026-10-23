"""Testes puros do speech_node (sem ROS rodando): politica da fila,
prefixos de prioridade, cache e comandos dos backends/player."""

from pathlib import Path

from manip_audio import speech_node
from manip_audio.speech_node import (
    PRIORITY_HIGH,
    PRIORITY_LOW,
    PRIORITY_NORMAL,
    SpeechQueue,
    cache_key,
    parse_speech_text,
    pronounce,
)


class FakeCompletedProcess:
    def __init__(self, returncode=0, stdout='', stderr=''):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def test_speech_node_entry_point_is_available():
    assert callable(speech_node.main)


# --------------------------------------------------------------- texto
def test_normalize_text_removes_pause_artifacts():
    assert speech_node.SpeechNode._normalize_text('  tag_1\n encontrada  ') == 'tag 1 encontrada'


def test_parse_speech_text_reads_priority_prefix():
    assert parse_speech_text('!Entrega concluida') == ('Entrega concluida', PRIORITY_HIGH)
    assert parse_speech_text('~Procurando a tag 7') == ('Procurando a tag 7', PRIORITY_LOW)
    assert parse_speech_text('Mesa livre') == ('Mesa livre', PRIORITY_NORMAL)
    assert parse_speech_text('  ! com_espacos  ') == ('com espacos', PRIORITY_HIGH)
    assert parse_speech_text('') == ('', PRIORITY_NORMAL)


def test_pronounce_restores_accents_of_known_words_only():
    assert pronounce('Nao consegui pegar o bloco') == 'Não consegui pegar o bloco'
    assert pronounce('o braco nao alcanca a tag 7') == 'o braço não alcança a tag 7'
    assert pronounce('A mesa ja esta ordenada') == 'A mesa já está ordenada'
    assert pronounce('Procurando a tag WS01') == 'Procurando a tag WS01'
    assert pronounce('Camera ja alinhada') == 'Câmera já alinhada'


# --------------------------------------------------------------- fila
def test_queue_high_priority_goes_first_and_is_never_dropped():
    q = SpeechQueue(max_age_s=4.0, max_pending_normal=2)
    q.push('normal 1', PRIORITY_NORMAL, now=0.0)
    q.push('alta', PRIORITY_HIGH, now=0.1)
    # Envelheceu tudo: a normal e' descartada, a alta nao.
    utt, dropped = q.pop(now=10.0)
    assert utt.text == 'alta'
    assert [d.utterance.text for d in dropped] == []
    utt, dropped = q.pop(now=10.0)
    assert utt is None
    assert [(d.utterance.text, d.reason.split()[0]) for d in dropped] == [('normal 1', 'atrasada')]


def test_queue_drops_stale_normal_phrases_but_keeps_fresh_ones():
    q = SpeechQueue(max_age_s=4.0, max_pending_normal=5)
    q.push('velha', PRIORITY_NORMAL, now=0.0)
    q.push('nova', PRIORITY_NORMAL, now=3.0)
    utt, dropped = q.pop(now=5.0)
    assert utt.text == 'nova'
    assert [d.utterance.text for d in dropped] == ['velha']


def test_queue_keeps_only_newest_normal_phrases_when_full():
    q = SpeechQueue(max_age_s=100.0, max_pending_normal=2)
    for i in range(4):
        q.push(f'frase {i}', PRIORITY_NORMAL, now=float(i))
    assert q.pending() == 2
    assert q.pop(now=4.0)[0].text == 'frase 2'
    assert q.pop(now=4.0)[0].text == 'frase 3'


def test_queue_low_priority_is_superseded_by_any_newer_phrase():
    q = SpeechQueue()
    q.push('procurando a tag', PRIORITY_LOW, now=0.0)
    _, dropped = q.push('tag encontrada', PRIORITY_NORMAL, now=0.5)
    assert [(d.utterance.text, d.reason) for d in dropped] == [('procurando a tag', 'superada')]
    assert q.pop(now=0.6)[0].text == 'tag encontrada'


def test_queue_low_priority_is_spoken_when_nothing_newer_arrives():
    q = SpeechQueue()
    q.push('procurando a tag', PRIORITY_LOW, now=0.0)
    assert q.pop(now=30.0)[0].text == 'procurando a tag'


def test_queue_dedupes_repeated_text_except_high_priority():
    q = SpeechQueue(dedupe_s=8.0)
    assert q.push('mesma frase', PRIORITY_NORMAL, now=0.0)[0] is not None
    utt, dropped = q.push('mesma  frase', PRIORITY_NORMAL, now=3.0)
    assert utt is None and dropped[0].reason == 'repetida'
    assert q.push('mesma frase', PRIORITY_NORMAL, now=9.0)[0] is not None
    assert q.push('mesma frase', PRIORITY_HIGH, now=9.5)[0] is not None


def test_queue_peek_returns_next_without_removing():
    q = SpeechQueue()
    q.push('a', PRIORITY_NORMAL, now=0.0)
    q.push('b', PRIORITY_HIGH, now=0.1)
    assert q.peek().text == 'b'
    assert q.pending() == 2


def test_cache_key_depends_on_backend_voice_and_normalized_text():
    assert cache_key('piper', 'faber', 'Ola  mundo') == cache_key('piper', 'faber', 'Ola mundo')
    assert cache_key('piper', 'faber', 'Ola') != cache_key('kokoro', 'faber', 'Ola')
    assert cache_key('piper', 'faber', 'Ola') != cache_key('piper', 'jeff', 'Ola')
    assert cache_key('piper', 'faber', 'Ola').endswith('.wav')


# --------------------------------------------------------------- backends
def test_speech_command_uses_smoother_espeak_settings():
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._executable = 'espeak-ng'
    node._voice = 'pt-br'
    node._rate = 165
    node._pitch = 45
    node._word_gap = 0
    node._volume = 100

    command = node._speech_command('Teste de voz')

    assert command == [
        'espeak-ng', '-v', 'pt-br', '-s', '165', '-p', '45', '-g', '0', '-a', '100', '-z',
        'Teste de voz',
    ]


def test_espeak_backend_writes_wav_when_output_given():
    backend = speech_node.EspeakBackend('espeak-ng', 'pt-br', 165, 45, 0, 100)
    command = backend.command('Oi', Path('/tmp/voz.wav'))
    assert command[-3:] == ['-w', '/tmp/voz.wav', 'Oi']


def test_piper_command_uses_configured_model():
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._piper_executable = 'piper'
    node._piper_model = '/tmp/pt_BR-model.onnx'

    command = node._piper_command('/tmp/voice.wav')

    assert command == ['piper', '--model', '/tmp/pt_BR-model.onnx', '--output_file', '/tmp/voice.wav']


def test_piper_supports_model_option_detects_tts_binary(monkeypatch):
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._piper_executable = 'piper-tts'
    monkeypatch.setattr(speech_node.subprocess, 'run',
                        lambda *args, **kwargs: FakeCompletedProcess(stdout='--model MODEL'))
    assert node._piper_supports_model_option()


def test_piper_supports_model_option_rejects_other_piper(monkeypatch):
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._piper_executable = 'piper'
    monkeypatch.setattr(speech_node.subprocess, 'run',
                        lambda *args, **kwargs: FakeCompletedProcess(stdout='GTK options'))
    assert not node._piper_supports_model_option()


def test_piper_supports_model_option_rejects_empty_executable():
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._piper_executable = ''
    assert not node._piper_supports_model_option()


def test_speak_returns_when_backend_is_unavailable(monkeypatch):
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._speech_available = False

    def fail_run(*args, **kwargs):
        raise AssertionError('subprocess should not be called')

    monkeypatch.setattr(speech_node.subprocess, 'run', fail_run)
    node._speak('Teste')


# --------------------------------------------------------------- player
def _player_node(player, effect):
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._player_executable = player
    node._voice_effect = effect
    node._effect_sample_rate = 22050
    return node


def test_no_effect_plays_with_ffplay_without_filter():
    command = _player_node('ffplay', 'none')._play_audio_command('/tmp/voice.wav')
    assert command == ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', '/tmp/voice.wav']


def test_cartoon_dog_effect_adds_ffplay_audio_filter():
    command = _player_node('ffplay', 'cartoon_dog')._play_audio_command('/tmp/voice.wav')
    assert command == [
        'ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', '-af',
        'asetrate=22050*0.82,aresample=22050,atempo=1.12,tremolo=f=7:d=0.18,vibrato=f=5:d=0.15',
        '/tmp/voice.wav',
    ]


def test_dog_effect_adds_subtle_ffplay_audio_filter():
    command = _player_node('ffplay', 'dog')._play_audio_command('/tmp/voice.wav')
    assert command == [
        'ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', '-af',
        'asetrate=22050*0.94,aresample=22050,atempo=1.04,vibrato=f=4:d=0.04,volume=1.05',
        '/tmp/voice.wav',
    ]


def test_dog_deep_effect_adds_deeper_ffplay_audio_filter():
    command = _player_node('ffplay', 'dog_deep')._play_audio_command('/tmp/voice.wav')
    assert command == [
        'ffplay', '-nodisp', '-autoexit', '-loglevel', 'error', '-af',
        'asetrate=22050*0.82,aresample=22050,atempo=1.10,vibrato=f=5:d=0.08,volume=1.10',
        '/tmp/voice.wav',
    ]


def test_plain_player_command_uses_available_player_without_ffplay_flags():
    command = _player_node('aplay', 'dog')._play_audio_command('/tmp/voice.wav')
    assert command == ['aplay', '/tmp/voice.wav']
