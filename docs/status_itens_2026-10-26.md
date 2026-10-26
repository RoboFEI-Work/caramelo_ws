# Quadro de status dos itens (2026-10-26)

Atualizado pelo Claude a cada item que termina. Legenda: ✅ feito · 🔄 em andamento · ⏳ aguardando (robô/dados) · ⬜ não iniciado.

| # | Item | Status | Quem | Onde ver |
|---|---|---|---|---|
| 3 | Manipulador lento e com jitter (Dynamixel) | ✅ Pi commitada (453fec1: latency 1 ms, return delay 0, perfil por tempo, 100 Hz); MoveIt 3,0 rad/s e 8,0 rad/s² (home↔pegar_obj 2,15 s → 0,80 s). Pendente: você julgar de olho; achado: joint4 para em 1,915 rad (alvo 2,199 do `pegar_obj`) com corrente baixa → limite de posição do próprio servo, ver abaixo | Claude (main) | `caramelo_hardware_ws` na Pi, `manip_moveit_config/config/joint_limits.yaml` |
| 1 | Tour por todos os docks do mapa + FINISH | ✅ script pronto e mesclado (b86cad1): `ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026`; 179 testes passam. ⏳ Falta rodar no robô (precisa do stack completo no ar e robô localizado no START) | Claude (main) | `caramelo_navigation/scripts/tour_docks.py`, guia_missoes §2.1 |
| 7 | Investigação do incidente na arena | ✅ mesclado (378d8cd + 87f140f): relatório com reconstrução das runs de 22 e 23/10 pelos logs, causa comprovada (GoToWS liberava a manipulação só com `success`, ignorando erro de pose e `yaw_nao_medido`), gate de pose com re-dock automático, checagem de base parada por odometria, 20 gtests + cenário de bancada 8/8. Sem vídeo/transcrição (pendência registrada). ⏳ validar no robô com `pose_gate_strict=false` (default) lendo `pose validada / FORA DOS LIMITES` e `base parada` no log; depois ligar strict | Claude (main) | `docs/investigacao_incidente_2026-10.md`, guia §5.1 |
| 4 | Planejador por tempo (grafo de distâncias, busca) | ✅ mesclado (2330a54): matriz `dock_distances.yaml` por mapa + branch-and-bound no `task_planner` (sem matriz = guloso de sempre). Ganho estimado: ATT1 −7 %, ATT2 −4 a −14 %, FINAL −9 a −18 %; BMT/BTT iguais. 13 gtests + 36 pytests passam. ⏳ validar numa missão real; gravar a pose da ST1 no mapa (hoje fica fora da matriz) | Claude (main) | `docs/planejador_por_tempo.md`, `manip_bt/src/task_plan_search.cpp` |
| 5 | Merge da branch robot_ui + atualizar interface | ✅ mesclado (6e5f8fe): GUI e painel web adaptados (tasks 2026, voz, câmera anotada, `mission_server` com action `/caramelo/run_mission`), 131 testes ok, GUI sobe. ⏳ missão ponta a ponta pela GUI no robô; painel web precisa de `python3 -m pip install --user --break-system-packages fastapi uvicorn` (não instalado neste PC) | Claude (main) | `docs/interface_grafica.md` |
| 2 | Docks colados na parede (costmap/staging) | ⏳ depende do tour (item 1) no robô | Claude (main) | `caramelo_navigation/config/*.yaml`, `maps/arenalegal_2026/docking.yaml` |
| 2.5 | Borda da mesa por profundidade (D455) para docking | ⬜ começa quando um agente liberar; precisa de gravação da câmera na arena | — | — |

## Medições já feitas hoje (item 3)

| Condição | SyncRead 7 servos | Ping |
|---|---|---|
| U2D2 com `latency_timer=16` (padrão FTDI) | 16,0 ms | 16 ms |
| `latency_timer=1` (regra udev) | 9,0 ms | 2 ms |
| + `return_delay=0` nos servos | 3,0 ms | 0,8 ms |

Braço agora a 100 Hz (`rw_rate=100`, controllers 100 Hz), perfil por tempo 20 ms nos servos, sem overrun no `ros2_control_node`.

## Velocidade do braço (item 3) — medido com o MoveIt real (`/move_action`, scaling 1,0)

| Limites (`joint_limits.yaml`) | home ↔ pegar_obj | pico de velocidade | erro final após 0,5 s |
|---|---|---|---|
| 1,2 rad/s / 1,0 rad/s² (antes) | 2,15 s | 1,06 rad/s | j2 −7, j3 −22, **j4 +280** mrad |
| 2,5 / 4,0 | 1,10 s | 2,11 rad/s | idem |
| **3,0 / 8,0 (atual)** | **0,80 s** | 2,95 rad/s | j1 +1, j2 −5, j3 −23, **j4 +275** mrad |

Sem avisos do move_group nem do JTC em nenhum caso. Teto físico do XM540-W270: ~3,1 rad/s sem carga.

**Achado no joint 4 (confirmado lendo a EEPROM dos servos)**: `Max Position Limit` do servo 4 = 3295 ticks = **1,913 rad**; o `pegar_obj` pede 2,199 rad (3482 ticks), então o servo para em 1,915 com corrente baixa. O JTC aceita porque a tolerância de goal é 0,30 rad e o erro é 0,28. Isso já acontecia antes de hoje. Outras poses do SRDF também passam do limite: `tag_direita`/`tag_esquerda`/`MesaSh` (1,94 rad, cortadas em 25 mrad). Limites lidos nos servos (ticks → rad):

| servo | min | max | poses do SRDF perto do limite |
|---|---|---|---|
| 2 | 1032 (−1,56) | 3146 (+1,68) | `home` j2 = −1,54 (folga 18 mrad) |
| 3 | 547 (−2,30) | 3455 (+2,16) | `pegar_obj_sh` j3 = 2,16 (no limite) |
| 4 | 859 (−1,82) | **3295 (+1,91)** | `pegar_obj` j4 = 2,20 (**corta 0,29 rad**), `tag_*` e `MesaSh` 1,94 |

Decisão do time: (a) subir o limite do servo 4 no Dynamixel Wizard se o cabo da câmera e a mecânica permitirem, ou (b) corrigir URDF (joint4 upper 2,2 → 1,91) e regravar as poses do SRDF para o que o braço realmente faz hoje. Em (b) nada muda fisicamente, mas o planejador deixa de confiar na tolerância de 0,30 rad do JTC e passa a recusar IK além de 1,91.
