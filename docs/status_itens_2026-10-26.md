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


## 2026-10-27 — regressão do item 3: pick aborta com "object_still_grasped" (bloco preso na garra)

**Sintoma (tour das 14:05, WS5/tag_13)**: a pega deu certo (`Bloco na garra`, esforço 1,1 N·m), mas o transporte falhou 2× e o pick abortou com `object_still_grasped: transporte falhou 2x com o bloco na garra`; a estação foi abortada. O place das 14:12 falhou do mesmo jeito (`surface above target [j2/j3 por ultimo]`). Em todos os casos o braço **não se mexeu**: o `execute()` voltou em 5 ms.

**Causa (log do move_group)**: `Invalid Trajectory: start point deviates from current robot state more than 0.01 at joint 'manip_joint2'` (e j1, j5). Cadeia:
1. Com 3,0 rad/s / 8,0 rad/s² (047d2dc) o plano acaba antes do servo: o JTC da Pi declara SUCCEEDED no fim do tempo da trajetória porque a tolerância de goal é 0,30 rad, com as juntas ainda convergindo.
2. O move_group "espera o braço parar" olhando só 3 amostras seguidas de `joint_states` com variação < 0,01 rad por amostra. A 100 Hz isso são 30 ms: uma junta a 0,5 rad/s passa por parada (a 25 Hz, antes de 26/10, a mesma checagem cobria 120 ms e o plano era 2,5× mais lento).
3. O nó planeja a próxima trajetória a partir desse estado; ~80 ms depois a validação do move_group vê a junta > 0,01 rad fora do ponto inicial e recusa. O nó lê "execução falhou" e, com o bloco na garra, aborta (correto: nunca pular com o bloco preso).

Evidência de que é regressão: zero recusas em todos os logs até 23/10 (um deles com 451 execuções); 1, 2 e 3 (+1 no place) nos três stacks desde a noite de 26/10. O joint 4 no limite do servo (1,915 rad) **não** entra nessa conta: o plano parte do estado atual e o servo está parado lá.

**Correção aplicada (sem commit ainda)**:
- `manip_task_execution/include/manip_task_execution/arm_settle.hpp` (novo): após cada `execute()` bem-sucedido, pick, place e AMT1 esperam todas as juntas do grupo ficarem dentro de 4 mrad por 120 ms seguidos (até 1,5 s; depois avisa e segue). Não exige chegar no alvo, só parar — joint 4 no limite e garra no bloco passam. Parâmetros nos três nós: `post_move_settle_window_s` (0,12; 0 desliga), `post_move_settle_tol_rad` (0,004), `post_move_settle_timeout_s` (1,5). Custo: ~120–300 ms por movimento (log `braco assentou em N ms` quando passou de 170 ms).
- `manip_moveit_config/config/moveit_controllers.yaml`: `allowed_start_tolerance: 0.03` (era o default 0,01) como rede para a deriva residual e para quem não passa pela espera (`go_to_named_pose_bt`, commander).
- Compilado (`colcon build --packages-select manip_task_execution`, sem avisos). O yaml é symlink: vale ao relançar o stack de manipulação.

**Validar no robô**: relançar o stack, repetir a WS5 (pick + place). Esperado: nenhuma linha `Invalid Trajectory` no log do move_group; linhas `braco assentou em ... ms` nos nós dizendo quanto o braço ainda andava depois do SUCCEEDED. Se aparecer `braco AINDA se mexia apos 1.50 s`, o servo não está convergindo (ver corrente/limite) — aí a conversa é no JTC/servo, não aqui. Alternativa se a espera incomodar: voltar `joint_limits.yaml` para 2,5/4,0 ou 1,2/1,0.

## 2026-10-27 — docking pela câmera + prateleira mais lenta

### Docking: a câmera do punho manda, o LiDAR apoia

**Pedido**: o docking "às vezes muito bom, às vezes muito ruim"; usar a câmera, que já vê a mesa no pré-docking com o braço na pose de navegação.

**Medido no robô (braço em `home`)**: D455 a 0,48 m do chão, x = 0,26 m, olhando 23° para baixo (imagem de cabeça para baixo). Vê o chão a partir de x ≈ 0,60 m e um tampo de 0,10 m a partir de x ≈ 0,53 m: do staging (face a ~0,63 m) o tampo 0,80 × 0,50 m aparece inteiro; na parada final (face a ~0,40 m) a borda da frente some, mas as laterais continuam visíveis. **Achado à parte**: o TF da câmera nessa pose está **5,45° errado em pitch** (o chão projetado sobe 5 cm a cada 0,5 m; roll 0,4°). O detector se corrige sozinho (ver abaixo), mas o mesmo erro pode estar na percepção do pick/place — vale conferir a calibração das juntas 2–4 / montagem da câmera.

**Como mede** (`caramelo_navigation/scripts/caramelo_dock_camera.py`, numpy puro): depth → nuvem no `base_footprint` → planos horizontais por RANSAC → chão e tampo (0,03–0,30 m acima do chão, altura medida **sob a câmera**, invariante ao erro de inclinação) → corrige a inclinação girando a nuvem em torno da câmera → componente conexa do tampo que contém o eixo do robô → borda da frente (menor x por faixa de y) e laterais (extremos por faixa ao longo da normal), descartando pontos colados na borda da imagem (lá é campo de visão, não mesa). Sai yaw da face, `c_axis`/`c_foot` e largura, nas mesmas convenções do LiDAR. Testes sintéticos (`test/test_dock_camera.py`, renderização por raio com a pose real e o erro de 5,45°): yaw ±0,3°, centro ±5 mm do staging; ±0,6° / ±8 mm só pelas laterais na parada final; cubos no tampo e mesa vizinha não atrapalham.

**Sem dimensões fixas (decisão do operador, 27/10)**: as mesas mudam de largura, profundidade e altura (5–15 cm) de arena para arena; nada disso é configurado. A mesa é reconhecida pelo **formato**: plano horizontal 3–20 cm acima do chão, cuja área (fechamento + abertura morfológica: filetes finos como a fatia de uma parede na altura do tampo somem) forma um retângulo — borda da frente reta (a **mais próxima** com comprimento, não a com mais pontos), laterais paralelas à normal, tampo preenchendo ≥ 75 % do retângulo observado, ≤ 8 % fora dele e fundo reto (tampo em L/peça grudada atrás = "fundo em degrau", sem centro). A largura medida com as duas laterais vistas fica guardada **só naquele docking** para centrar quando uma lateral sair da imagem. O `table.width/height` do `service_areas.yaml` não é usado (estava errado: 0,80 × 0,10 contra 0,61 × 0,15 medidos).

**Paredes (operador, 27/10: muitas mesas têm parede atrás e/ou numa lateral, sempre branca como o tampo)**: cada ponto do depth ganha a normal local (diferença para o vizinho contínuo, ~2 cm); o que não é horizontal (normal > 50° da vertical) sai antes de qualquer conta — parede atrás, à esquerda, à direita ou no canto não entra no tampo nem gruda na lateral. A cor não importa (o depth é infravermelho). Testes: 15 cenários de parede (atrás/esq/dir/2 cantos × 3 poses), centro ±1,5 cm, largura ±3 cm. No robô: ~7 mil pontos de parede descartados por frame, medida igual à de antes.

**Primeira medida no robô (27/10, pose aproximada de pré-docking, mesa com parede atrás)**: 15 frames, yaw −2,8° (dispersão 0,6°), `c_foot` +2,1 cm (dispersão 0,9), largura 0,61 m, frente 0,731 m, mesa 0,157 m, inclinação corrigida 5,8°. Antes da abertura morfológica, metade dos frames tomava a fatia da parede (1,17 m) como frente — caso coberto agora por `test_parede_logo_atras_da_mesa`. ~30 ms por frame.

**Fluxo novo no `align_to_dock`** (`camera_*`, default ligado):
1. **Staging**: mede a mesa pela câmera (mediana de 5 frames, base parada) fundida com o LiDAR (peso 0,8 para a câmera quando concordam; discordando, só a câmera). Gira e depois anda de lado **por odometria**, remedindo após cada movimento (até 3 movimentos). Alinhado pela câmera ⇒ o esquadro e o lateral do LiDAR no staging são pulados.
2. **Pausa do reto** (~0,55 m): mesma coisa pela câmera; sem medida dela, o refit do LiDAR de sempre.
3. **Lateral na mesa** e **yaw final**: fusão câmera+LiDAR. Na correção final de yaw, recua até 0,60 m (a borda volta ao campo de visão) e esquadra pela câmera.
4. Sem medida da câmera (braço fora de `home`, mesa não vista, `camera_enabled:=false`) ⇒ exatamente o fluxo LiDAR anterior. Prateleira (`camera_skip_types: [shelf]`) fica só no LiDAR.

**Limite que continua**: o giro mínimo da base é o coast do ESC (~6°, `yaw_coast_est` 0,10 + 0,02). Yaw abaixo de ~7° a câmera mede mas não consegue corrigir girando — isso é atuador, não medida.

**Validar no robô** (stack relançado — o `dock_align_node` é cópia instalada):
1. Robô parado de frente para uma mesa, braço em `home`: `ros2 run caramelo_navigation dock_camera_probe --frames 20`. Esperado: yaw e `c_foot` estáveis (dispersão < 1° / < 1 cm), largura = a da mesa real, `incl` ≈ 5,5–6°.
2. **Calibração**: robô docado perfeito pela trena → `camera_yaw_offset = −(yaw em rad)`, `camera_y_offset = −(c_foot em m)` no `docking_server.launch.py`.
3. Tour com `gravar_bag_tour`: procurar no log `[cam] staging/0:` e `camera(staging): alinhado`, comparar com a trena nas quinas. Se piorar: `camera_enabled:=false` volta ao LiDAR.

### Place na prateleira mais lento

O braço passa todo esticado (`pirocao`, todas as juntas em 0) e tremia a 3,0 rad/s. No `mtc_place_action_node`, os três movimentos da prateleira (pre_container → `pirocao`, `pirocao` → `MesaSh`, `MesaSh` → `pirocao` após soltar) rodam com `shelf_velocity_scaling` 0,35 / `shelf_acceleration_scaling` 0,25 (lidos a cada movimento: `ros2 param set /place_action_server shelf_velocity_scaling 0.25` para testar). O resto do ciclo continua a 1,0. Log: `[SHELF] ... com velocidade 0.35 / aceleracao 0.25`. Fora do place: o pick da prateleira e o primeiro movimento da task seguinte (que parte do `pirocao`) continuam em velocidade normal.

### Tour com câmera (27/10, ~18h) — piorou: diagnóstico e correções

**Resultado**: 2 ok / 7 reprovadas (WS5 perfeita; WS1/2/3/6/SH1/PP1 com 8–25° no fim). **A medida da câmera estava boa** (concordava com o LiDAR: −6,6×−6,3, −7,6×−7,9, −11,3×−9,2 graus); **o giro é que falhou**: os giros pela câmera eram fechados pela ODOMETRIA, que subestimou o giro real em 30–70% (pedido −7,7 → odometria −11,4 → câmera/LiDAR viram ~19° de giro real). Cada passo passava do ponto ~2× e o robô entrou em vaivém de ±11° (WS2: −7,6 → +11,8 → −11,3 → +11,6…; WS6: −8,8 → +11,3 → −9,6 → +10,9) até acabar os passos. O esquadro antigo do LiDAR, que corta pelo yaw da face medido durante o giro, parava a ±1,5° (WS1: −10,2 → +1,2).

**Correções**:
- Giro da câmera cortado pela **variação do yaw da face medida pelo LiDAR durante o giro** (a câmera dá o alvo, o LiDAR acompanha); odometria só sem face no LiDAR.
- **Trava contra vaivém**: se um giro passa para o outro lado, não gira de novo (o giro mínimo da base é maior que o erro).
- Frente × laterais discordando (WS1/WS3/WS6: frente batia com o LiDAR, laterais davam o dobro): vale a frente.
- "Fundo em degrau" vira só informação (objetos no tampo escondiam o fundo e reprovavam mesas boas).
- Corte pelo limite da região de busca (1,6 m) e pela **faixa sem depth da D455** (~50 px num lado) não vira mais "lateral" (o probe mostrou largura 0,40 inventada assim).

**Imagem de debug** (pedido do operador): `ros2 run rqt_image_view rqt_image_view /dock_align/camera_debug`. Publicada pelo `dock_align_node` em cada frame medido durante o docking e pelo `dock_camera_probe --frames 0` (contínuo, robô parado). Esquerda: depth girado 180° com o tampo em verde e o descartado como parede/vertical em vermelho. Direita: vista de cima (robô embaixo, grade 10 cm) com o que a câmera vê (cinza), tampo (verde), borda da frente (amarelo), laterais (ciano), eixo do robô (branco), centro e normal (magenta) e o texto da decisão (verde = mediu centro, amarelo = só yaw, vermelho = sem mesa + motivo). `camera_debug_image:=false` desliga; só desenha se alguém assina.

### Giro em pulsos e viewer único (27/10, após o tour)

**Giro** (operador: "ele gira sempre a mesma quantidade, passa, gira de volta e nunca se ajusta, só entorta"): o ESC (FOC, piso 650 rpm, rampas no firmware) faz todo giro contínuo sair um degrau de ~12–20°, por mais cedo que o corte seja. Agora (`rotate_mode: pulsos`, default; `continuo` = antigo) todo giro do docking — pela câmera e o esquadro pelo LiDAR — é feito em **pulsos medidos**: pulso de T s → para → mede pelo LiDAR (yaw da face, parado) quanto girou → aprende `giro(T) = c + k·T` (pulso mínimo ajusta `c` = menor giro possível; pulso maior ajusta `k`) → próximo pulso pelo que falta, até 2° (`rot_pulse_tol`). Não dá pulso que passaria do ponto (menor giro > 1,5 × erro). O modelo é mantido entre dockings. Log: `pulso N: T s -> girou X graus (lidar); falta Y; modelo c=.. k=..`. Simulação (base com menor giro 8–13°, ruído ±15%): 120 giros, 79 pulsos, 5 trocas de sentido. Limite físico: erro menor que ~2/3 do menor giro da base não se corrige girando. Se o log mostrar `c` alto, testar `rot_pulse_t_min` menor (0,10).

**Viewer único**: o `rqt_image_view` que abre com o robô (`/manip/vision_debug/image`) passa a mostrar a imagem do docking enquanto o `dock_align_node` mede pela câmera (o `vision_debug` repassa `/dock_align/camera_debug` no mesmo tópico) e volta sozinho para a visão do pick/place 2 s depois (`dock_hold_s`).

### Braço em `home` antes de todo docking (27/10)

A câmera do docking só vê a mesa com o braço na pose de navegação (`home`).
- **Missões**: o `task_plan_search` já emite `home` antes de cada `goto` (e o executor tem o `home` do prólogo). Novo no `bt_yaml_executor`: se a ação anterior a um `goto` com docking pode ter deixado o braço fora de `home` (pick/place/amt1), ele insere `<ForceSuccess><GoToNamedPose home/></ForceSuccess>` (falhar não derruba a missão; o docking cai no LiDAR). Sem duplicar quando já há `home`. Parâmetro `home_before_dock` (default true). Conferido por `--dry-run` num YAML escrito à mão.
- **`tour_docks`**: antes de cada dock (depois do undock, longe da mesa) manda o braço para `home` pelo MoveIt (`/move_action`, juntas lidas do `caramelo.srdf`), pulando se já está lá (0,05 rad). `--arm-pose ''` desliga.
- `--dry-run` do executor agora imprime plano e XML **antes** de montar a árvore (antes travava esperando o `/move_group` com o stack fora do ar).

### Tour com câmera + pulsos (27/10, ~20h): 8/9 ok — agora o TEMPO

Resultado: 8 ok, PP1 "reprovada" por 7,6° de yaw no AMCL (o operador viu a mesa boa; tolerância do relatório 5,2°). Align 12–68 s por dock (mediana ~36 s). Para onde ia o tempo (log do `dock_align_node`):
1. **Cada medida da câmera ~3,5–4 s**: a assinatura do depth era best-effort e a imagem (600 KB) perdia fragmentos — chegavam ~0,3 frame/s e toda medida esperava o teto de 3 s. Reliable: ~10 frames/s.
2. **Movimentos que não andam, repetidos**: laterais de 3–4 cm (abaixo do passo mínimo da base) andaram 0,0 cm três vezes seguidas (WS3 12 s, WS5).
3. **Medida extra** depois de "nenhum pulso cabe".
4. **Câmera perto da mesa**: na parada final só vê as laterais e deu −20° com o LiDAR em −3° → recuo + correção à toa (WS1, WS3: +15–20 s cada).
5. Perseguir 2°/2 cm quando o manipulador tolera mais.

**Correções**: depth/camera_info em RELIABLE (nó e probe); 3 frames por medida, teto 1,5 s; tolerâncias da câmera 4° / 4 cm (`camera_yaw_tol` 0,07, `camera_center_tol` 0,04 — o pulso ainda mira 2° quando gira); sem nova medida quando nenhum pulso cabe (decide o centro com a mesma); lateral que não andou (< 1 cm) não se repete e conta como "melhor possível"; câmera desligada abaixo de `camera_min_front` 0,50 m (lateral na mesa e yaw final só LiDAR); sem a pausa no meio da reta quando o staging fechou pela câmera; detector 190 → 100 ms/frame (RANSAC e morfologia vetorizados com scipy) e imagem de debug 60–80 → 34 ms. Esperado: 1–3 medidas de ~1 s por dock em vez de 6–10 de ~4 s.
