# Voz do robô (`manip_audio/speech_node`)

A voz existe para o operador **ouvir o que o robô está fazendo** sem olhar o log. Dois
compromissos que o nó garante sempre:

- **a fala nunca trava uma ação** — os nós só publicam em `/manip/speech` e seguem;
- **uma frase nunca é cortada no meio** — o que toca, toca até o fim.

O que o nó pode fazer é **descartar** uma frase antes de dizê-la, quando ela já ficou velha
ou foi superada por outra mais nova. Cada descarte vai para o log (`[VOZ] descartada ...`)
e para `/manip/speech_status`.

## 1. Como os nós falam

Publicar `std_msgs/String` em `/manip/speech`. O **primeiro caractere** diz a prioridade:

| Prefixo | Prioridade | Política | Uso |
|---|---|---|---|
| `!` | alta | nunca descartada; fala antes das outras | resultado: `!Peguei o bloco`, `!Entreguei o bloco`, `!Falha: ...` |
| (nenhum) | normal | descartada se esperar mais que `max_age_s` (4 s) ou se houver mais que `max_pending_normal` (2) normais na frente — ficam as mais novas | decisões: `Mesa livre, entregando no centro` |
| `~` | baixa | descartada assim que chegar **qualquer** frase mais nova | progresso: `~Procurando a tag 7`, `~Indo para a pose de entrega` |

Texto repetido em menos de `dedupe_s` (8 s) não é dito de novo (salvo alta). O código dos
nós escreve as frases em ASCII (convenção do time); o nó devolve os acentos das palavras
usuais antes de sintetizar (`PRONUNCIA` em `speech_node.py`: `nao → não`, `braco → braço`...).

Regra para escrever uma frase nova: curta (uma linha), diz **o que o robô está fazendo ou
decidiu**, sem "atenção", sem "com sucesso", sem número de tentativa a menos que importe.
Resultado leva `!`; "estou indo/procurando/olhando" leva `~`.

## 2. Pipeline

```
/manip/speech ──> SpeechQueue (política acima) ──> síntese (cache em disco) ──> player
                                                   └─ pré-sintetiza a PRÓXIMA frase
                                                      enquanto a atual toca
```

- Cache: `~/.cache/caramelo_tts/<sha1(backend|voz|texto)>.wav`. Frase fixa é sintetizada
  **uma vez por PC**; da segunda vez sai na hora. Trocar de voz muda a chave.
- Ao chegar uma frase, a síntese já começa em paralelo; quando chega a vez dela o WAV
  costuma estar pronto. A latência que sobra é o player (ffplay/aplay).
- Status: `/manip/speech_status` (String, latched) = `falando: <texto>` | `ocioso` |
  `descartada (<motivo>): <texto>`; `/manip/speech_busy` (Bool, latched). O
  `vision_debug` mostra o `falando:` na barra da câmera.

## 3. Motores (`backend`)

| backend | o que é | qualidade | custo |
|---|---|---|---|
| `kokoro` (default) | Kokoro-82M, pt-BR, em processo (torch CPU) | natural | ~0,3–0,6 s por frase curta na CPU do PC; modelo carregado uma vez no boot |
| `piper` | binário `~/piper/piper` + `models/pt_BR-faber-medium.onnx` (o antigo `pt_BR-jeff` continua versionado) | robótica, mas limpa | ~0,3 s por frase |
| `espeak` | `espeak-ng -w` | reserva | instantâneo |

Se o backend pedido não estiver disponível o nó **cai para o próximo** sozinho e avisa no
log (`Kokoro indisponivel (...) — tentando o Piper.`).

Vozes do Kokoro em pt-BR: `pm_alex` (default), `pm_santa` (masculinas) e `pf_dora`
(feminina). Amostras para escolher de ouvido: `~/amostras_voz/` (`aplay <arquivo>`).

### Instalar o Kokoro num PC novo

```bash
bash src/manipulation/manip_audio/scripts/instalar_kokoro.sh     # com internet, uma vez
```

Cria `~/caramelo_tts_venv` (fora do Python do ROS), instala torch CPU + kokoro e baixa o
modelo do Hugging Face (~330 MB). O `speech_node` importa os pacotes desse venv pelo
parâmetro `kokoro_site_packages`. Sem o venv, o robô fala com o Piper.

## 4. Parâmetros e launch

`robot_manipulation.launch.py` / `manip_stack.launch.py`:

| arg | default | nota |
|---|---|---|
| `use_audio` | `true` | sobe o `speech_node` |
| `speech_backend` | `kokoro` | `kokoro` / `piper` / `espeak` |
| `kokoro_voice` | `pm_alex` | `pm_alex` / `pm_santa` / `pf_dora` |
| `voice_effect` | `none` | `dog`, `dog_deep`, `cartoon_dog` (filtro do ffplay) |
| `piper_model` | `models/pt_BR-faber/pt_BR-faber-medium.onnx` | só com `speech_backend:=piper` |

Parâmetros só do nó (ros-args): `max_age_s`, `max_pending_normal`, `dedupe_s`,
`cache_dir`, `kokoro_speed` (1.0; 1.1 fala um pouco mais rápido), `kokoro_threads` (4).

## 5. Testar sem o robô

```bash
ros2 run manip_audio speech_node --ros-args -p backend:=kokoro -p kokoro_voice:=pm_alex
ros2 topic echo /manip/speech_status &
ros2 topic pub -1 /manip/speech std_msgs/String "{data: '~Procurando a tag 7'}"
ros2 topic pub -1 /manip/speech std_msgs/String "{data: '!Peguei o bloco'}"
```

Esperado: a baixa é dita se nada chegar antes; publicando as duas em sequência a baixa é
descartada (`descartada (superada)`) e só a alta toca. Testes puros (sem ROS):
`python3 -m pytest src/manipulation/manip_audio/test -q`.

## 6. Problemas comuns

- **Robô mudo e log diz `Sem voz: executavel 'espeak-ng' nao encontrado`** — nem Kokoro
  nem Piper disponíveis; `sudo apt install espeak-ng` para a reserva, ou instalar o
  Kokoro (seção 3).
- **Primeira frase demora no boot** — o Kokoro carrega o modelo (alguns segundos) e faz um
  aquecimento (`warmup_text`). Depois disso o cache resolve.
- **Fala "náo"** — palavra ASCII fora da tabela `PRONUNCIA`; acrescentar lá.
- **Muitas frases `descartada (atrasada)`** — o robô está publicando mais do que dá para
  falar; rebaixar as de progresso para `~` ou encurtar. Não aumentar `max_age_s`: o
  objetivo é a voz acompanhar o robô, não narrar o passado.
- **NUNCA `apt install piper`** — é um utilitário de mouse gamer, não o TTS.
