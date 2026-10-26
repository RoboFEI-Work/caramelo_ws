# Quadro de status dos itens (2026-10-26)

Atualizado pelo Claude a cada item que termina. Legenda: ✅ feito · 🔄 em andamento · ⏳ aguardando (robô/dados) · ⬜ não iniciado.

| # | Item | Status | Quem | Onde ver |
|---|---|---|---|---|
| 3 | Manipulador lento e com jitter (Dynamixel) | ✅ Pi commitada (453fec1: latency 1 ms, return delay 0, perfil por tempo, 100 Hz); MoveIt 3,0 rad/s e 8,0 rad/s² (home↔pegar_obj 2,15 s → 0,80 s). Pendente: você julgar de olho; achado: joint4 para em 1,915 rad (alvo 2,199 do `pegar_obj`) com corrente baixa → limite de posição do próprio servo, ver abaixo | Claude (main) | `caramelo_hardware_ws` na Pi, `manip_moveit_config/config/joint_limits.yaml` |
| 1 | Tour por todos os docks do mapa + FINISH | 🔄 agente escrevendo o script; depois rodo no robô | agente item-1 | `caramelo_navigation/scripts/tour_docks.py` |
| 7 | Investigação do incidente na arena | 🔄 agente investigando com logs e código; **vídeo com áudio não está neste PC** (pendência: caminho do vídeo) | agente item-7 | `docs/investigacao_incidente_2026-10.md` |
| 4 | Planejador por tempo (grafo de distâncias, busca) | 🔄 agente | agente item-4 | `manip_bt/src/task_planner.cpp`, `docs/planejador_por_tempo.md` |
| 5 | Merge da branch robot_ui + atualizar interface | 🔄 agente | agente item-5 | `caramelo_gui`, `caramelo_web`, `docs/interface_grafica.md` |
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

**Achado no joint 4**: em `pegar_obj` (alvo 2,199 rad) o servo para em 1,915 rad com corrente de 0,07 A (não está forçando contra batente). Tudo indica `Max Position Limit` gravado na EEPROM do servo 4 (~3296 ticks). O JTC aceita porque a tolerância de goal é 0,30 rad e o erro é 0,28. Isso já acontecia antes de hoje (erro máximo 279 mrad na linha de base). Decisão do time: subir o limite no servo (se o cabo da câmera permitir) ou corrigir URDF/SRDF para 1,9 rad e regravar `pegar_obj`; até lá a câmera olha 16° diferente do planejado.
