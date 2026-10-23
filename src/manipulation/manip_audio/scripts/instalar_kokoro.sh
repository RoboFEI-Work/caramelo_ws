#!/usr/bin/env bash
# Instala o motor de voz Kokoro-82M (pt-BR) num venv proprio, fora do Python
# do ROS: ~/caramelo_tts_venv. O speech_node (backend kokoro) importa os
# pacotes direto desse venv (parametro kokoro_site_packages). Rodar uma vez
# por PC, com internet (torch CPU ~200 MB + modelo ~330 MB do Hugging Face).
#
#   bash src/manipulation/manip_audio/scripts/instalar_kokoro.sh
#
# Sem este venv o speech_node cai sozinho para o Piper (modelo pt_BR-faber
# versionado em manip_audio/models), e sem Piper para o espeak-ng.
set -euo pipefail

VENV="${CARAMELO_TTS_VENV:-$HOME/caramelo_tts_venv}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

if [ ! -x "$VENV/bin/python" ]; then
    echo "==> criando venv em $VENV"
    # Ubuntu 24.04 do robo nao tem ensurepip: cria sem pip e instala pelo get-pip.
    python3 -m venv --without-pip "$VENV"
fi
if [ ! -x "$VENV/bin/pip" ]; then
    echo "==> instalando pip no venv"
    curl -sSL -m 120 -o "$TMP/get-pip.py" https://bootstrap.pypa.io/get-pip.py
    "$VENV/bin/python" -I "$TMP/get-pip.py" -q
fi

echo "==> torch (CPU) + kokoro"
"$VENV/bin/pip" install -q torch --index-url https://download.pytorch.org/whl/cpu
"$VENV/bin/pip" install -q kokoro soundfile

echo "==> baixando o modelo e sintetizando uma frase de teste"
"$VENV/bin/python" - <<'PY'
import time
import numpy as np
import soundfile as sf
from kokoro import KPipeline
t0 = time.time()
pipe = KPipeline(lang_code='p', repo_id='hexgrad/Kokoro-82M')
print(f"modelo carregado em {time.time() - t0:.1f} s")
# As tres vozes pt-BR ficam em cache (a arena nao tem internet).
for voice in ('pm_alex', 'pm_santa', 'pf_dora'):
    t0 = time.time()
    audio = np.concatenate([np.asarray(a) for _, _, a in pipe('Voz pronta.', voice=voice)])
    print(f"{voice}: sintese de teste em {time.time() - t0:.2f} s ({len(audio) / 24000:.2f} s de audio)")
    sf.write(f'/tmp/kokoro_teste_{voice}.wav', audio, 24000)
print("ok: /tmp/kokoro_teste_<voz>.wav (toque com: aplay /tmp/kokoro_teste_pm_alex.wav)")
PY
