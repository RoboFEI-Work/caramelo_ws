from manip_audio import speech_node


class FakeCompletedProcess:
    def __init__(self, returncode=0, stdout='', stderr=''):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def test_speech_node_entry_point_is_available():
    assert callable(speech_node.main)


def test_normalize_text_removes_pause_artifacts():
    assert (
        speech_node.SpeechNode._normalize_text(
            '  tag_1\n encontrada  '
        ) == 'tag 1 encontrada'
    )


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
        'espeak-ng',
        '-v', 'pt-br',
        '-s', '165',
        '-p', '45',
        '-g', '0',
        '-a', '100',
        '-z',
        'Teste de voz',
    ]


def test_piper_command_uses_configured_model():
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._piper_executable = 'piper'
    node._piper_model = '/tmp/pt_BR-model.onnx'

    command = node._piper_command('/tmp/voice.wav')

    assert command == [
        'piper',
        '--model', '/tmp/pt_BR-model.onnx',
        '--output_file', '/tmp/voice.wav',
    ]


def test_piper_supports_model_option_detects_tts_binary(monkeypatch):
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._piper_executable = 'piper-tts'

    monkeypatch.setattr(
        speech_node.subprocess,
        'run',
        lambda *args, **kwargs: FakeCompletedProcess(stdout='--model MODEL'),
    )

    assert node._piper_supports_model_option()


def test_piper_supports_model_option_rejects_other_piper(monkeypatch):
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._piper_executable = 'piper'

    monkeypatch.setattr(
        speech_node.subprocess,
        'run',
        lambda *args, **kwargs: FakeCompletedProcess(stdout='GTK options'),
    )

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


def test_cartoon_dog_effect_adds_ffplay_audio_filter():
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._player_executable = 'ffplay'
    node._voice_effect = 'cartoon_dog'
    node._effect_sample_rate = 22050

    command = node._play_audio_command('/tmp/voice.wav')

    assert command == [
        'ffplay',
        '-nodisp',
        '-autoexit',
        '-loglevel',
        'error',
        '-af',
        (
            'asetrate=22050*0.82,aresample=22050,atempo=1.12,'
            'tremolo=f=7:d=0.18,vibrato=f=5:d=0.15'
        ),
        '/tmp/voice.wav',
    ]


def test_dog_effect_adds_subtle_ffplay_audio_filter():
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._player_executable = 'ffplay'
    node._voice_effect = 'dog'
    node._effect_sample_rate = 22050

    command = node._play_audio_command('/tmp/voice.wav')

    assert command == [
        'ffplay',
        '-nodisp',
        '-autoexit',
        '-loglevel',
        'error',
        '-af',
        (
            'asetrate=22050*0.94,aresample=22050,atempo=1.04,'
            'vibrato=f=4:d=0.04,'
            'volume=1.05'
        ),
        '/tmp/voice.wav',
    ]


def test_dog_deep_effect_adds_deeper_ffplay_audio_filter():
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._player_executable = 'ffplay'
    node._voice_effect = 'dog_deep'
    node._effect_sample_rate = 22050

    command = node._play_audio_command('/tmp/voice.wav')

    assert command == [
        'ffplay',
        '-nodisp',
        '-autoexit',
        '-loglevel',
        'error',
        '-af',
        (
            'asetrate=22050*0.82,aresample=22050,atempo=1.10,'
            'vibrato=f=5:d=0.08,'
            'volume=1.10'
        ),
        '/tmp/voice.wav',
    ]


def test_plain_player_command_uses_available_player_without_ffplay_flags():
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._player_executable = 'aplay'
    node._voice_effect = 'dog'
    node._effect_sample_rate = 22050

    command = node._play_audio_command('/tmp/voice.wav')

    assert command == ['aplay', '/tmp/voice.wav']
