# Quadro de status dos itens (2026-10-26)

Atualizado pelo Claude a cada item que termina. Legenda: ✅ feito · 🔄 em andamento · ⏳ aguardando (robô/dados) · ⬜ não iniciado.

| # | Item | Status | Quem | Onde ver |
|---|---|---|---|---|
| 3 | Manipulador lento e com jitter (Dynamixel) | ✅ Pi commitada (453fec1: latency 1 ms, return delay 0, perfil por tempo, 100 Hz); MoveIt 3,0 rad/s e 8,0 rad/s² (home↔pegar_obj 2,15 s → 0,80 s). Pendente: você julgar de olho; achado: joint4 para em 1,915 rad (alvo 2,199 do `pegar_obj`) com corrente baixa → limite de posição do próprio servo, ver abaixo | Claude (main) | `caramelo_hardware_ws` na Pi, `manip_moveit_config/config/joint_limits.yaml` |
| 1 | Tour por todos os docks do mapa + FINISH | ✅ rodado 2× pelo operador (19:45 e 19:52): 9/9 docks chegaram, mas 5–6 com aviso; WS1 yaw −31°/−22°, WS3 lateral +16/+17 cm, WS6 yaw −9°/−11°, SH1 lateral −20 cm, repetível entre os dois tours. Relatórios em `maps/arenalegal_2026/tour_2026-10-26_19*.md` | operador | `maps/arenalegal_2026/tour_*.md` |
| 7 | Investigação do incidente na arena | ✅ mesclado (378d8cd + 87f140f): relatório com reconstrução das runs de 22 e 23/10 pelos logs, causa comprovada (GoToWS liberava a manipulação só com `success`, ignorando erro de pose e `yaw_nao_medido`), gate de pose com re-dock automático, checagem de base parada por odometria, 20 gtests + cenário de bancada 8/8. Sem vídeo/transcrição (pendência registrada). ⏳ validar no robô com `pose_gate_strict=false` (default) lendo `pose validada / FORA DOS LIMITES` e `base parada` no log; depois ligar strict | Claude (main) | `docs/investigacao_incidente_2026-10.md`, guia §5.1 |
| 4 | Planejador por tempo (grafo de distâncias, busca) | ✅ mesclado (2330a54): matriz `dock_distances.yaml` por mapa + branch-and-bound no `task_planner` (sem matriz = guloso de sempre). Ganho estimado: ATT1 −7 %, ATT2 −4 a −14 %, FINAL −9 a −18 %; BMT/BTT iguais. 13 gtests + 36 pytests passam. ⏳ validar numa missão real; gravar a pose da ST1 no mapa (hoje fica fora da matriz) | Claude (main) | `docs/planejador_por_tempo.md`, `manip_bt/src/task_plan_search.cpp` |
| 5 | Merge da branch robot_ui + atualizar interface | ✅ mesclado (6e5f8fe): GUI e painel web adaptados (tasks 2026, voz, câmera anotada, `mission_server` com action `/caramelo/run_mission`), 131 testes ok, GUI sobe. ⏳ missão ponta a ponta pela GUI no robô; painel web precisa de `python3 -m pip install --user --break-system-packages fastapi uvicorn` (não instalado neste PC) | Claude (main) | `docs/interface_grafica.md` |
| 2 | Docking torto/descentrado (WS1, WS2, WS3, WS6) e navegação perto das paredes | ✅ diagnóstico (33 agentes) + correções imediatas implementadas, revisadas 2× e commitadas: esquadro por AMCL desligado (`amcl_square_enabled=false`), `confident` preservado, espera de base parada antes do align (tour, GoToWS e dock_align), esquadro pelo LiDAR ANTES do lateral, corte lateral com previsão de lag, janela do refino 0,35 só na WS6, `corrected_once` só se girou, `PoseProgressChecker` no Nav2, tour reprovando pelos números (exit≠0), instrumentação `[inst]`, script `gravar_bag_tour`. 253 pytests + gtests + cenário de bancada 8/8. ⏳ validar no robô: relançar o stack, tour com bag, dockings 1/2/3/6 isoladas + WS4 como controle, trena nos cantos (roteiro abaixo) | Claude + workflow | `docs/diagnostico_docking_tour_2026-10-26.md`, `docs/docking_alinhamento_holonomico.md` (bloco 2026-10-26), guia §2.2 |
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

## Docking (item 2) — o que o diagnóstico encontrou (resumo; detalhes em `docs/diagnostico_docking_tour_2026-10-26.md`)

| Dock | Onde o erro nasce | Causa (confiança) | Correção |
|---|---|---|---|
| WS6 (−9°/−11° nos 2 tours) | esquadro por AMCL no staging: refino rejeitado por "resíduo alto" (bloco a 0,42 m à esquerda dentro da janela 0,45) → gira −9,7° pelo AMCL lido com a base ainda girando, contra o LiDAR que pedia +5° CCW; depois nenhum fit perto da mesa passa (face 0,44–0,47 m < 0,50) | C1 + C4 (alta) | I1 `amcl_square_enabled=false`; R3 `refine_half_width_y: 0.35` na WS6; I3 esperar parada |
| WS1 (−31°/−22°) | T2: 2ª leitura do refino sobrescreve `confident` e dispara esquadro por AMCL (sobre-giro ~20°); T1: 0/5 fits no fim (fit leve não mede > 23,6°) → `yaw_nao_medido` = sucesso sem girar | C1 + C4 + C7 (alta) | I2 `confident = c1 or c2`; I1; I7 tour reprova pelos números |
| WS3 (+16/+17,5 cm) | `lateral(staging)` roda ANTES do esquadro e centra o ponto onde o eixo torto fura a face: demanda +14 cm = `0,65·tan(yaw)` + sobrecurso 7–8 cm (ESC retém referência ~0,5 s) | C3 (alta, 3/3) | I4 esquadro antes do lateral + centro pelo pé da perpendicular; R6a corte com previsão de lag |
| WS2 (−14 cm / −7,6°) | nenhum esquadro: fits rejeitados por resíduo 21–43 mm (estrutura 15–25 cm atrás da face na banda +0,30); lateral é discordância AMCL × LiDAR | C4 (alta) | R2 (resíduo por inliers) e R1 depois dos testes T2–T4 |
| Navegação | `SimpleProgressChecker` (0,10 m / 6 s, só translação) aborta o `FollowPath` no giro inicial do RotationShim em 12–15 pernas → 22 clears do costmap local → BackUp cego de 0,10 m (staging WS5-T2 −10 cm). Um quase-contato quantificado (19:47:33, WS4→WS5). Contato não pode ser afirmado nem negado | C8 (alta no mecanismo, média no perigo) | I5 `PoseProgressChecker`; R7 (BT sem clear na 1ª falha) depois do T8 |

Refutadas como causa do torto: mapa deslocado (F19: yaw gravado ≤ 5° da normal), erro de transformação de frames (F7: contas batem em ≤ 1,7 cm), base girando (C2: contribui ≤ 2°, mas contamina as medidas → I3).

Limites de precisão derivados do código da manipulação (propostos, a confirmar no teste T7): yaw ≤ 5° (0,09 rad), lateral ≤ 10 cm, front 0,36–0,42 m. O gate atual (0,20 rad / 0,20 m) é ~2× mais frouxo e não estrito.

Dados que faltam para fechar o resto: rosbag do tour (`/scan`, `/tf`, `/odom`, `/amcl_pose`, costmaps, cmd_vel), trena nos cantos do robô em WS1/2/3/6 e numa boa, o que há a 4,5 cm do chão nas mesas (o LiDAR vê a estrutura, não o tampo).

### Item 2 — o que foi aplicado e como validar (2026-10-26, noite)

Aplicado (todas as mudanças têm teste unitário; nada foi rodado no robô ainda):

| # | Mudança | Arquivo | Efeito esperado no próximo tour |
|---|---|---|---|
| I1 | esquadro por AMCL desligado por default (`amcl_square_enabled: false` no launch); ligado, só sem face nenhuma, base parada, mediana de 5 amostras, veto por sinal do LiDAR, limite 0,20 rad | `dock_align_node.py`, `docking_server.launch.py` | WS6 e WS1 deixam de girar −9…−12° contra o LiDAR; log mostra "esquadro por AMCL desligado ... o AMCL pede X" |
| I2 | 2ª leitura do refino não sobrescreve `confident` (alvo = última confiante) | `dock_align_node.py` | WS1-T2 mantém o alvo refinado |
| I3 | espera de base parada (0,02 m/s / 0,05 rad/s por 0,5 s, teto 3 s) antes de medir e antes do align: tour, GoToWS (estado `parada_staging`) e início do `_run_attempt` | `tour_docks.py`, `go_to_ws_bt.cpp`, `dock_align_node.py` | linha "na staging" sem a deriva de +7°; "Reached" → 1º refino ≥ 1,2 s |
| I4 | `refit_square("staging")` antes do `lateral_adjust("staging")` quando o refino confiou; centro continua pelo eixo (`c_axis`), pé da perpendicular só no log | `dock_align_node.py` | WS3 deixa de carregar `d·tan(yaw)`: lateral(staging) pede ≤ 3 cm, não +14 |
| R6a | corte do lateral pelo ponto previsto (coast + v·(idade+período)) | `dock_align_node.py` | sobrecurso lateral de 7–8 cm cai para ~2 |
| I8 | `corrected_once` só se o refit girou; 1 ciclo de correção | `dock_align_node.py` | log "correcao sem esquadro" em vez de "já corrigido" |
| R3 | `refine_half_width_y: 0.35` na WS6 (service_areas.yaml → sync → docking.yaml) | yamls, `caramelo_service_area_common.py` | refino do staging da WS6 confiante (resíduo < 20 mm; hoje 46–53) |
| I5 | `PoseProgressChecker` (0,10 m / 0,5 rad / 6 s) | `controller_server.yaml` | zero "Failed to make progress" no giro inicial; sem clear/BackUp em cadeia; ~90 s a menos por tour |
| I7 | tour classifica pelos números: yaw > 5,2° ou lateral > 10 cm (qualquer fonte) = reprovada; exit ≠ 0 | `caramelo_tour_common.py`, `tour_docks.py` | relatório "N ok / M aviso / K reprovadas"; os 16 dockings de hoje dariam 14 reprovadas |
| I6 | instrumentação `[inst]` (motivo de rejeição dos fits, corte, odometria, AMCL × LiDAR em cada transição) + legenda de sinais no relatório | `dock_align_node.py`, tour | cada docking explicável só pelo log |
| I9 | WS5 do `service_areas.yaml` igual ao `docking.yaml` | yaml | `--sync-docking` não reverte a WS5 |
| 8 | `ros2 run caramelo_navigation gravar_bag_tour` (rosbag com `/scan`, `/tf`, `/odom`, `/amcl_pose`, costmaps, cmd_vel, feedback das actions) | script novo, guia §2.2 | próximo tour com bag |

Não aplicado de propósito (dependem dos testes T1–T8 do diagnóstico): R1/R2 (aceitar face estreita consistente, resíduo por inliers — WS2 continua sem esquadro), R4 (giro em malha fechada, faixa morta 5,2–6,9°), R5 (lateral como critério de falha do align), R7 (BT sem clear na 1ª falha, `footprint_padding`), M1 (gate estrito da missão), M3 (regravar poses), I1(e) (desfazer giro do AMCL pela odometria).

Roteiro de validação (operador, robô ligado, stack relançado para carregar o `dock_align_node` e o `controller_server.yaml` novos):
1. `ros2 run caramelo_navigation gravar_bag_tour` num terminal (deixa gravando).
2. `ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026 --report` com o robô no START. Esperado: nenhuma linha "esquadro por AMCL:" girando; "lateral(staging)" com demanda ≤ 3 cm na WS3; WS6 com "refino face ... janela ±0.35" confiante; zero "Failed to make progress" no log do controller; relatório com reprovadas listadas.
3. `--only WS1,WS2,WS3,WS6,WS4` 3× (WS4 = controle). Em cada parada: trena das duas quinas dianteiras até a face (diferença ≤ 3,5 cm = < 5°) e do centro do robô ao centro do tampo.
4. Comparar trena × `yaw_face`/`Y(AMCL)` do log × Lat/Yaw do relatório. Se o LiDAR e a trena concordarem e o AMCL não, a fonte do Y do gate é o próximo passo (R5).

