# Guia do operador — Missões do Caramelo (nav + manipulação)

Validado no robô real em 2026-07-22 (missão completa autônoma WS1→WS2→FINISH
com pick e place). Detalhes técnicos: `docs/relatorio_integracao_manip_2026-07-22.md`.

---

## 1. Subir o sistema

```bash
# 1. Na Raspberry (como sempre):
ros2 launch raspberry_bringup hardware_bringup.launch.py

# 2. No PC (navegação + manipulação juntas):
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=arena2_520

# 3. Localizar o robô: GUI SO COMO MONITOR (fantasma -> pose inicial).
#    NAO clicar em "iniciar navegacao" na GUI — a navegacao JA esta no ar
#    pelo launch acima (clicar duplicaria o Nav2).
```

Args úteis do launch: `use_nav:=false` (só manipulação), `use_camera:=false`,
`use_apriltag:=false`, `use_audio:=false`, `use_rviz:=true` (RViz da nav),
`use_rviz_manip:=true` (RViz do MoveIt), `use_mock_hardware:=true` (tudo
simulado no PC, sem robô).

## 1a. RUNBOOK 100% TERMINAL (sem GUI) — atualizado 2026-07-23

```bash
# ── 1. SUBIR (terminal 1; Pi ja com hardware_bringup rodando) ──────────────
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=arena2_520 use_rviz:=true
# use_rviz e' opcional (so para VER; o 2D Pose Estimate dele e' util).
# Aguarde ~40 s. NAO precisa cronometrar: o run_mission (passo 4) espera
# sozinho com progresso ("aguardando move_group... ok (14s)").

# ── 2. LOCALIZAR (terminal 2) ──────────────────────────────────────────────
# Opcao A — RViz: botao "2D Pose Estimate" -> clique onde o robo esta e
#           arraste na direcao que ele aponta (scan deve "vestir" as paredes).
#   REGRA COMPROVADA (23/07): de ZOOM antes de clicar (semente precisa) e
#   depois faca VAI-E-VEM de ~0,5 m ate a nuvem verde de particulas FECHAR —
#   o AMCL so corrige quando o robo anda (medido: 10cm sem ritual -> 2cm com).
# Opcao B — robo na zona START, sem RViz:
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0979, y: 0.0384},
   orientation: {z: -0.1095, w: 0.9940}}, covariance: [0.25,0,0,0,0,0,
   0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}'
# Conferir:
ros2 run tf2_ros tf2_echo map base_footprint     # pose deve bater com a real

# ── 3. SALVAR DOCK (robo parado na pose ideal em frente a mesa) ────────────
ros2 topic pub --once /save_dock_pose std_msgs/msg/String "{data: 'WS1'}"
ros2 topic pub --once /save_dock_pose std_msgs/msg/String "{data: 'WS2'}"
# Conferir:
grep -A6 "WS1:" ~/caramelo_ws/src/caramelo_mapping/maps/arena2_520/docking.yaml
# So salve com a localizacao BOA (scan batendo com o mapa). Perto da mesa o
# AMCL degrada (mesas remontadas != mapa) — com o refine calibrado (passo 5),
# o erro X/yaw da gravacao deixa de ser critico.

# ── 4. MISSAO (terminal 2) ─────────────────────────────────────────────────
cd ~/caramelo_ws
ros2 run caramelo_navigation run_mission --map-name arena2_520 --task missions/arena_test.yaml
#   --dry-run            so mostra o plano (nao espera nada, nao move nada)
#   --use-lidar-refine   docking ancorado na face FISICA da mesa (apos passo 5)
#   --preflight-timeout 180   se o notebook estiver lento na subida
# Ctrl-C = cancela tudo limpo (nav + braco).

# ── 5. CALIBRAR O REFINO LiDAR (uma vez; robo posicionado NA MAO na pose
#       perfeita em frente a mesa) ─────────────────────────────────────────
ros2 action send_goal /align_to_dock caramelo_msgs/action/AlignToDock \
  "{dock_id: WS1, map_name: arena2_520, use_lidar_refine: true, timeout: 1.0}"
# No log do launch (terminal 1) aparece: "refino face: dist=0.XXX m, ...".
# Grave esse valor AO VIVO:
ros2 param set /dock_align_node refine_desired_face_dist 0.XXX
# E deixe permanente editando o mesmo valor em:
#   src/caramelo_navigation/launch/docking_server.launch.py (refine_desired_face_dist)
# A partir dai, missoes com --use-lidar-refine param X (distancia a face real)
# e yaw esquadrado com a mesa, mesmo com AMCL/mapa deslocados.
```

## 1b. Passo a passo com visualização (nav + MoveIt + câmera)

```bash
# T1 (PC): sistema completo
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=arena2_520

# T2: RViz da NAVEGACAO (mapa, robo, scan, costmaps)
rviz2 -d ~/caramelo_ws/src/caramelo_localization/rviz/global_localization.rviz
#   -> ajustar pose: botao "2D Pose Estimate", clique onde o robo esta e
#      ARRASTE na direcao que ele aponta. Certo = scan "vestindo" as paredes.
#      (ou GUI com o fantasma — sem clicar "iniciar navegacao"!)

# T3: RViz da MANIPULACAO (braco em tempo real + PAINEL DA CAMERA embutido)
rviz2 -d ~/caramelo_ws/src/manipulation/manip_bringup/config/manip_moveit_rasp.rviz
#   (so a camera: ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw)

# T4: enviar a task (mostra o plano e pergunta s/N; Ctrl-C cancela limpo)
cd ~/caramelo_ws && ros2 run caramelo_navigation run_mission \
  --map-name arena2_520 --task missions/arena_test.yaml
```

CPU do notebook: se a nav engasgar com 2 RViz, feche o da navegacao durante a
missao (a pose ja foi ajustada; o do MoveIt e mais leve).

## 2. Rodar uma missão

```bash
ros2 run caramelo_navigation run_mission --map-name arena2_520 --task missions/minha_missao.yaml
```

- Ele traduz a tarefa, MOSTRA o plano de ações e pede confirmação (s/N).
- `--dry-run`  → só mostra o plano e o XML, nada se move.
- `--simulate-nav` → navegação simulada (testar manipulação sem Nav2).
- `--yes` → sem confirmação. `--finish ""` → não vai ao FINISH no fim.
- Cada missão gera um YAML de ações auditável em `~/.ros/caramelo_missions/`.
- Ctrl-C no meio: cancela limpo os goals em voo (nav e braço).

## 3. Onde e como definir a missão

Crie/edite arquivos em **`~/caramelo_ws/missions/`** (nome livre, formato =
YAML da competição, igual `missions/arena_test.yaml`):

```yaml
task_id: MINHA_MISSAO
name: Descricao livre
duration_min: 10
active_service_areas: [WS_1, WS_2]   # estacoes usadas na missao

objects:                             # id = numero da AprilTag do objeto
  - {id: 01, type: ATTC, color: RED}     # tag 1 -> frame tag_1
  - {id: 03, type: ATTC, color: BLUE}    # tag 3 -> frame tag_3

containers:                          # potes NA MESA (opcional): ids 10..16 -> ct_10..ct_16
  - {id: 10, color: BLUE, at: WS_2}      # use [] se nao houver

start_state:                         # onde cada coisa COMECA
  WS_1: {obj_ids: [01, 03], cont_ids: []}
  WS_2: {obj_ids: [], cont_ids: [10]}

finish_state:                        # onde cada coisa deve TERMINAR
  WS_1: {obj_ids: [], cont_ids: []}
  WS_2: {obj_ids: [01, 03], cont_ids: [10]}
```

**Regras práticas:**
- Só use estações com pose REAL gravada no `docking.yaml` do mapa (hoje:
  **WS_1 e WS_2**). As demais (WS_3, WS_4, SH_1, PP_1, RT_1) têm pose
  placeholder [0,0,0] e o executor **aborta de propósito antes de mover**
  (fail-fast) até você gravar as poses.
- Máx. **3 objetos por viagem** (potes de bordo) — o planejador divide em
  várias idas sozinho.
- A ida ao **FINISH no fim é automática** (não declarar na tarefa).
- ids de tag disponíveis (cfg da competição): 1..7 → `tag_1..tag_7`,
  10..16 → `ct_10..ct_16` (potes), 20..28 → `tag_20..tag_28`, 42 → `tag_42`.

### 3b. YAML novo do @Work Commander (25/08)

O planner aceita os dois formatos. O novo:

```yaml
schema_version: 1
task_id: BTT2
version: equipe            # "equipe" = sem cores; a versão do robô traz a cor no type
active_service_areas: [WS_1, WS_2, WS_3, WS_6]
objects:
- {id: 4, type: ATTC}      # cor desconhecida -> place na mesa
- {id: 8, type: ATTC_RED}  # cor no type (aceita RED/BLUE, vermelho/azul, ATTC-blue, ...)
start_state:
  WS_2: {obj_ids: [13, 4]}
finish_state:
  WS_6: {obj_ids: [30, 13, 4]}
```

- Não há mais `containers:`/`cont_ids` nem tag nos containers: **sem bloco `containers:` no
  YAML, se o objeto tem cor (RED/BLUE) e vai para uma mesa comum, o place tenta o container
  dessa cor visto pela câmera** (`container_color` na ação); sem container visível (ou sem o
  detector no ar), solta na mesa. Formato antigo **com** `containers:` + "inside": só os objetos
  das constraints vão para container (pela COR do container, que precisa ser RED/BLUE); os
  outros vão para a mesa como antes.
- Ids 1..30 têm frame `tag_<id>` em `tags_36h11.yaml`; objeto sem frame agora é **erro** do
  planner (antes sumia do plano em silêncio). Objeto só no `start_state` vira aviso (não é
  pego); só no `finish_state` é erro. `type` com "DECOY" é ignorado. `stacks:` continua (§3a).

### 3a. Empilhar no YAML da competição (24/08)

No `finish_state` da estação de destino, declare as pilhas de BAIXO para CIMA:

```yaml
finish_state:
  WS_6:
    obj_ids: [04, 07]
    cont_ids: []
    stacks: [[04, 07]]          # 07 em cima do 04 (aceita [4, 7, 9] para 3 níveis)
    # ou: constraints: ["O7 must be on top of O4"]
```

O `task_planner` solta a base antes do topo (na mesma visita) e emite
`stack_on: tag_4` no place do topo — ver §6a para o que o place faz com isso.
Exemplo pronto: `missions/empilhar_ws4_ws6.yaml`. Casos inválidos (ciclo,
objeto que não termina nessa estação, topo com duas bases) abortam o planner
com mensagem clara.

## 4. ws_table_mapping.yaml — qual dos dois vale?

Existem duas cópias, mas **só uma manda**:

| Arquivo | Papel |
|---|---|
| `src/caramelo_mapping/maps/<mapa>/ws_table_mapping.yaml` | **CANÔNICO** — é o que o run_mission usa. Edite SEMPRE este. |
| `src/manipulation/manip_bt/behavior_tree_manip/ws_table_mapping.yaml` | Fallback antigo (só se rodar o task_planner na mão sem o 4º argumento). Ignorar. |

O arquivo do mapa tem DUAS chaves, lidas por partes diferentes do sistema:

```yaml
ws_to_table_pose:   # ALTURA DO BRACO em cada mesa (lido pelo task_planner)
  WS_1: Mesa0       # Mesa0/Mesa5/Mesa10/Mesa15 = poses do braco p/ 0/5/10/15 cm
  WS_2: Mesa5
ws_to_dock_id:      # qual DOCK do docking.yaml corresponde a estacao (lido pelo executor)
  WS_1: WS1
  WS_2: WS2
```

Trocou a altura física de uma mesa? Ajuste o `ws_to_table_pose` dela aqui.

## 5. Demais arquivos por mapa (a "carteira de identidade" da arena)

Tudo em `src/caramelo_mapping/maps/<mapa>/`:
- `docking.yaml` — pose e tipo de cada dock (**regravar pela GUI**: posicione o
  robô na distância ideal da mesa — pick validado com objeto a ~40 cm — e use
  "Salvar pose como dock" escolhendo a estação; vale na próxima missão).
- `service_areas.yaml` — pose final, use_docking e manipulation_enabled por área.
- `ws_table_mapping.yaml` — seção 4.

Config das tags (ids→frames→tamanhos): `src/manipulation/manip_bringup/config/tags_36h11.yaml`
(vendorado — NÃO depende mais do pacote apriltag_ros de fora; o binário vem do ~/ros2_ws).

## 6. Nível baixo (controle total da ordem, sem planejador)

Escreva você mesmo o YAML de AÇÕES (exemplos gerados em
`~/.ros/caramelo_missions/`):

```yaml
actions:
  - {kind: goto, ws: WS_1, mesa: Mesa0}
  - {kind: pick, tag_frame: tag_1}
  - {kind: home, pose_name: home}
  - {kind: goto, ws: WS_2, mesa: Mesa5}
  - {kind: place, tag_frame: tag_1, table_pose: Mesa5}
```

```bash
ros2 run manip_bt bt_yaml_executor minhas_acoes.yaml --ros-args \
  -p map_folder:=/home/linux24-04/caramelo_ws/src/caramelo_mapping/maps/arena2_520
# uteis: --dry-run | -p simulate_navigation:=true | -p finish_dock_id:="''"
```

O undock antes de sair de uma estação é automático (embutido no goto seguinte);
`home` antes de cada goto é responsabilidade de quem escreve as ações.

Desde 28/08 os picks/places de uma mesa comum (`WS_*`, `PP_*`) entram
automaticamente numa **fila de alcance** (`<ReachQueue>`, ver §6d): o que voltar
"alvo visto mas fora de alcance" é adiado, a base se ajusta de lado e a fila
repete. Um `home` no meio da estação fecha a fila (o próximo pick/place da
mesma estação abre outra). `SH_*` e START/FINISH ficam fora. Para o XML
antigo, byte a byte: `-p reach_queue_enabled:=false`.

## 6a. Empilhar objetos (`stack_on`, 24/08)

Só no nível baixo (`actions:`) — o planejador da competição não gera isso.
No `place` do objeto de CIMA, acrescente `stack_on: tag_<id da base>`; a base
tem que ser solta ANTES, na mesma estação. `table_pose` continua obrigatório
(altura da mesa e fallback: se a tag base não for vista pela câmera, o objeto
é solto na mesa normalmente). Não vale para a prateleira.

```yaml
actions:
  - {kind: goto, ws: WS_4, mesa: Mesa15}
  - {kind: pick, tag_frame: tag_4, table_pose: Mesa15}
  - {kind: pick, tag_frame: tag_7, table_pose: Mesa15}
  - {kind: home}
  - {kind: goto, ws: WS_6, mesa: Mesa15}
  - {kind: place, tag_frame: tag_4, table_pose: Mesa15, ws: WS_6}
  - {kind: place, tag_frame: tag_7, table_pose: Mesa15, ws: WS_6, stack_on: tag_4}
  - {kind: home}
```

Exemplo pronto: `src/caramelo_bringup/missions/empilhar_ws4_ws6_actions.yaml`.
O que o place faz com `stack_on`: vê a tag base pela câmera (a partir de
`pegar_obj`, com o bloco na garra), vai para (x, y) dela com o TCP em
`z_base + stack_place_z_offset` (parâmetro do `place_action_server`, 0,05 m =
cubo de 42 mm + folga — **calibrar no robô**), passando por uma pré-pose
`stack_pre_lift_m` (0,05) acima e descendo na vertical; depois de soltar sobe
de volta. Parâmetros: `stack_place_z_offset`, `stack_pre_lift_m`,
`stack_arrival_tolerance_m` (0,03), `stack_fallback_to_table` (true),
`stack_tilt_ladder_deg` ([15, 30]).

## 6a-bis. Container por cor (`container_color`, 25/08)

> Manutenção completa (trocar container, cor, garra; que arquivo/linha/parâmetro mexer):
> [containers_por_cor.md](containers_por_cor.md).

No nível baixo: `- {kind: place, tag_frame: tag_8, table_pose: Mesa15, ws: WS_6, container_color: RED}`
(RED|BLUE; nunca junto com `stack_on`). O place: publica a cota da borda em
`/manip/container_plane_z`, espera o TF `ct_vermelho`/`ct_azul` do `container_detector`
(até `container_detect_timeout_sec` 3 s), alinha lateral se preciso, vai para
(x, y) do container com o fundo do cubo `container_drop_clearance_m` (1 cm) acima da
borda (`container_rim_height_m` 0,07), abre. A descida é em 3 movimentos (pedido de
25/08): base e punho (j1, j4, j5) com o braço na altura atual → ombro+cotovelo (j2, j3)
até a pose de elevação (`stack_pre_lift_m` 5 cm acima) → descida vertical curta.
Saída (25/08): fecha a garra (`container_close_gripper_on_exit` true,
`container_exit_gripper_position` 0,170 = fechada; use p.ex. −0,05 para fechar só parcialmente),
mede o esforço — se pegou o bloco de volta, reabre e sai com a garra aberta — sobe na
vertical até a pose de elevação e volta a `pegar_obj` com j2/j3 primeiro (os dedos abertos
batiam na parede e empurravam o container). Não viu / sem IK → solta na mesa.
Params do `place_action_server`: `container_frame_red/blue`, `container_rim_height_m`,
`container_drop_clearance_m`, `cube_height_m`, `container_detect_timeout_sec`,
`container_arrival_tolerance_m` (erro em XY, 3 cm), `container_arrival_z_tolerance_m` (erro em Z,
6 cm — o braço estendido afunda ~2,5 cm sistematicamente, medido 25/08), `container_tilt_ladder_deg`,
`container_fallback_to_table`, `container_close_gripper_on_exit`, `container_exit_gripper_position`,
`container_slot_offsets_long_m` / `container_slot_offsets_short_m` (vários blocos no MESMO
container, 25/08: candidatos de deslocamento do ponto de soltura no eixo longo/curto do
container, default [+3, −3, 0] cm no longo; cada bloco vai para o candidato mais longe dos
que a memória de slots diz que já foram soltos lá — 1º a +3 cm, 2º a −3 cm, 3º no centro).

**Calibrar as cores do container** (`manip_bringup/config/container_detector.yaml`,
todos reajustáveis com `ros2 param set /container_detector ...`):
```bash
# sem robô, com uma foto:
ros2 run manip_bringup container_detector_node.py --image foto.jpg --out overlay.png
ros2 run manip_bringup container_detector_node.py --image foto.jpg --probe 800,600   # HSV do pixel
# no robô:
ros2 launch ... use_container_detector:=true container_detector_always_on:=true
ros2 topic echo /manip/container_detector/status
ros2 run rqt_image_view rqt_image_view /manip/container_detector/debug_image
ros2 run tf2_ros tf2_echo manip_base_link ct_vermelho
```
**A garra é VERMELHA e aparece na imagem** (câmera e garra são solidárias → sempre nos
mesmos pixels). O detector apaga essas regiões da máscara antes de segmentar:
`exclusion_polys_norm` (dedos ABERTOS, cantos superiores — valem sempre) e
`exclusion_polys_holding_norm` + `exclusion_rects_norm` (dedos FECHADOS + cubo carregado —
só com a garra fechada, lida de `/joint_states`: `gripper_joint` `manip_joint6` >
`gripper_holding_above` −0,12). Para recalibrar na arena (câmera trocada/remontada):
```bash
# foto com a garra ABERTA sobre a mesa branca, SEM container vermelho na imagem:
ros2 run manip_bringup container_detector_node.py --image garra_aberta.png --fit-gripper
#   -> cole as linhas "topo" em exclusion_polys_norm
# foto com a garra FECHADA num cubo AZUL (mesma condição):
ros2 run manip_bringup container_detector_node.py --image cubo_na_garra.png --fit-gripper                 # dedos fechados
ros2 run manip_bringup container_detector_node.py --image cubo_na_garra.png --fit-gripper --fit-color azul --margin-px 20   # cubo
#   -> cole em exclusion_polys_holding_norm (junte o topo do cubo até y=0)
# conferir: --out overlay.png (polígonos em amarelo); --gripper-open simula garra vazia
```
Blob cortado pela borda ou por uma região apagada = "parcial": usa o centroide da parte
visível (cai dentro da abertura), mas precisa de ≥ `partial_min_area_frac` (25 %) da área
esperada. Blob inteiro: solidez ≥ 0,75, proporção 1,1–2,6, retangularidade ≥ 0,70 e lados
dentro de ±`dims_tolerance_frac` de 170 × 105 mm. Forma 2D NÃO separa dedo de container
(dedo cortado pela borda vira quadrilátero) — por isso as máscaras fixas.
`yaw_axis_offset_deg` = 90 se os dedos tiverem que cruzar o lado curto.

## 6b. Parâmetros novos do executor (23/07)

- `startup_home` (true): braço vai a `home` ANTES da primeira ação da missão.
- `startup_pose_name` ("home"): pose do prólogo.
- `max_tag_age_sec` (1.0, nos nós de pick/place): a tag precisa ter sido VISTA
  há menos de 1 s — protege contra alvo fantasma (TF velha com robô movido).
  Se o pick reclamar "a camera NAO esta vendo a tag agora", a tag está fora do
  campo de visão (não é erro de IK).
- Fila de alcance (28/08, ver §6d): `reach_queue_enabled` (true),
  `reach_queue_ws_prefixes` (`["WS_","PP_"]`), `reach_queue_max_nudges` (2),
  `reach_queue_max_shift_m` (0.25), `reach_queue_min_shift_m` (0.10),
  `nudge_timeout` (20 s, goal do `nudge_base`).

- Ordem das juntas na pega (28/08, pedido do operador): na aproximação por cima o pick
  gira **j1 primeiro** (log `mesa custom down [so j1]`) e só depois desce o resto
  (`[resto]`); na volta a `pegar_obj` com o bloco, recolhe j2..j5 (`[j1 parado]`) e gira
  **j1 por último** (`[j1 por ultimo]`) — o braço estendido nunca varre a mesa girando.

## 6c. Raspberry NOVA (ou reinstalada): restaurar o SSH sem senha

Troca de Pi = host key nova + authorized_keys vazio. No PC:

```bash
# 1. Limpar a chave da Pi ANTIGA do known_hosts (senão o ssh recusa conectar):
ssh-keygen -R raspberrypi.local

# 2. Copiar a chave publica do PC para a Pi nova (senha padrao: raspberrypi):
sshpass -p raspberrypi ssh-copy-id -o StrictHostKeyChecking=accept-new raspberrypi@raspberrypi.local

# 3. Testar (nao pode pedir senha):
ssh raspberrypi@raspberrypi.local 'echo ok'
```

Se o PC ainda não tiver chave (`ls ~/.ssh/id_*` vazio): `ssh-keygen -t ed25519`
(enter em tudo) antes do passo 2. Regras que continuam valendo: 1 conexão SSH
por vez; NTP da Pi (sem RTC, deriva rápido — conferir `timedatectl` sempre).

## 6d. Fila de alcance por estação (`ReachQueue` + `nudge_base`, 28/08)

**Problema**: o robô estaciona na mesa numa pose que nem sempre deixa o braço
alcançar todos os objetos/containers (28/08: container azul a +20 cm → alvo
3 cm fora do alcance → cubo caiu na mesa). **Solução**: por estação de mesa
comum, o executor embrulha os picks/places numa `<ReachQueue>`:

1. Passada 1 na ordem do planner (places, depois picks) com `final_attempt=false`.
   Alvo VISTO sem IK → o servidor sai cedo com `success=true, skipped=true,
   unreachable=true, suggested_base_shift_m=±0.10..0.25` (o place antes
   devolve o cubo ao container de bordo) → a fila loga `ADIADO pick_N` e adia.
   Tag nunca vista → pulada como sempre (`tag_nao_vista`), NÃO é adiada.
2. Fila vazia com adiados → `AJUSTE LATERAL k/2: ±0.xx m` = action `nudge_base`
   (dock_align_node, malha fechada por odometria, `+` = esquerda), com a
   sugestão do 1º adiado limitada a `[min_shift_m, max_shift_m]`; sucesso →
   refaz os adiados. No máximo `reach_queue_max_nudges` (2) por estação.
3. Ajuste falhou / orçamento esgotado / sugestão 0 → `PASSAGEM FINAL: N
   acao(oes) pendente(s)` com `final_attempt=true` = comportamento antigo
   (escada completa + pular / fallback na mesa). Fecha com
   `concluida (ajustes=k, total=±0.xx m)`.
4. Pick/place com FAILURE de verdade (abort, servidor sumiu) → a fila falha
   como a Sequence de antes (missão aborta / estação pulada pela Fallback).

**Uma fila por estação**: um `home` no meio da estação fecha a fila (o
`NudgeBase` entra antes do `home`) e os picks/places seguintes da **mesma**
estação ficam fora dela (XML antigo, `final_attempt=true`) — o orçamento de
`reach_queue_max_nudges` (2) vale por estação, não por fila.

O acumulado dos ajustes vai em `/manip/base_shift_total` (Float64,
transient_local; só o dock_align_node publica; zera a cada `align_to_dock`) e o
nó de place usa esse offset na memória de slots.

**XML gerado** (`--dry-run`): `<ReachQueue name="rq_<g>" ws=… max_nudges="2"
max_shift_m="0.25" min_shift_m="0.10" final_attempt="{rq_<g>_final}"
shift_m="{rq_<g>_shift_m}">` + folhas com `name="pick_<i>"`,
`final_attempt="{rq_<g>_final}"`, `unreachable=`, `suggested_base_shift_m=`,
`skipped=` + `<NudgeBase dy="{rq_<g>_shift_m}" timeout="{nudge_timeout}"
ws=…/>` como último filho (`<g>` = índice do goto da estação). Fora da fila as
folhas ficam idênticas ao XML antigo (porta `final_attempt` ausente = true).
Com `-p simulate_navigation:=true` o `NudgeBase` é simulado (loga
`[SIMULADO] ajuste lateral …; base NAO se move`).

**Bancada sem robô** (`ROS_DOMAIN_ID=99`, sem MoveIt — sem `home` no YAML):

```bash
# gtest do nó de controle (folhas roteirizadas)
colcon test --packages-select manip_bt --ctest-args -R test_reach_queue && colcon test-result --verbose
# cenário completo com os fakes (nav + nudge_base + pick/place "fora de alcance")
src/manipulation/manip_bt/test/run_reach_queue_scenario.sh        # all | main | fail_pick | unseen | fail_nudge
```

O cenário (`test/reach_queue_actions.yaml`) confere por grep: `ADIADO pick_2 …
+0.20` → `AJUSTE LATERAL 1/2: +0.20` → pick ok → `AJUSTE LATERAL 2/2: +0.25` →
`PASSAGEM FINAL` → `SUCCESS`; e as regressões `fail_stage:=pick` (FAILURE, exit
1), só tags nunca vistas (sem ADIADO) e `fail_stage:=nudge` (PASSAGEM FINAL sem
mover). Fakes: `fake_pick_place.py -p unreachable_picks:=['tag_7:0.20']`
(alcançável só quando o acumulado de `/manip/base_shift_total` chega a ≤ 6 cm
do valor), `-p unseen_tags:=['tag_9']`; `fake_nav_actions.py -p
nudge_slip:=0.85 -p fail_stage:=nudge`.

## 7. Avisos rápidos (aprendidos na validação)

- **Cabo de rede caiu durante trajetória do braço** = o braço termina o
  movimento mas o PC dá timeout/abort ("Execute request aborted"). Fixar bem os
  cabos; repetir a ação resolve.
- **Câmera "Device or resource busy"** ao relançar = sobrou processo do launch
  anterior segurando a USB. Matar os órfãos (move_group, mtc_*, realsense,
  apriltag) antes de subir de novo.
- **Robô "teleportando" na GUI/RViz** = DOIS AMCLs (stack de nav duplicado —
  launch antigo que não morreu) brigando pelo map→odom. Conferir com
  `ps -eo pid,lstart,comm | grep amcl` e matar o launch antigo INTEIRO. O
  mesmo vale p/ nós rodados à mão dias antes (ex.: dock_align_node duplicado
  rouba goals da action com params errados — e, desde 28/08, também o
  `nudge_base` da fila de alcance: um dock_align_node velho responde ao ajuste
  lateral com params/guardas antigos ou `ocupado`, e a fila cai direto na
  PASSAGEM FINAL sem mover a base). Higiene: antes de relançar,
  `ps -eo pid,lstart,comm | grep -E "amcl|nav|dock_align"` e limpar o velho.
- **Câmera muda com nó no ar** = provavelmente porta USB 2 ("Device USB type:
  2.1" + "Reduced performance" no log). O D455 exige USB 3 (porta azul) — sem
  isso o nó sobe mas NÃO publica imagens.
- **Voz**: motor Piper em `~/piper/piper` + `espeak-ng`/`ffmpeg` (ffplay aplica
  o efeito dog). NUNCA `apt install piper` (é um app de mouse gamer). Tirar o
  efeito: `voice_effect` no speech_node do manip_stack.launch.py.
- **Script Python novo em install(PROGRAMS) precisa de `chmod +x` no src** —
  com `--symlink-install`, sem o bit de execução o launch falha com
  "executable not found on the libexec directory".
- Robô **sem voz** neste PC: falta o executável piper/espeak-ng
  (`sudo apt install espeak-ng` habilita o fallback).
- Warnings "Parameter 'X' is not supported" no launch = a RealSense reclamando
  de args do launch pai. Benigno, pode ignorar.
- **Robô ruim de localização que PIORA com o tempo/movimento** = relógio da Pi
  derivou (o gremlin clássico). Medir: stamp do /scan vs relógio do PC (>150 ms
  = doente). Curar: `sudo systemctl restart systemd-timesyncd` NA PI. Solução
  definitiva pendente: chrony servidor no notebook (timesyncd quer internet).
- **"Dock requested has no valid plugin"** = docking.yaml com esquema de
  dock_plugins errado (o `init_service_areas --sync-docking` gera por-dock;
  o launch validado espera os 3 tipos caramelo_front/precision/shelf_dock).
  Copiar o bloco `dock_plugins` de um mapa validado e normalizar os `type`.
- **Calibração do refine**: o goal de "espiar" (`timeout: 1.0`) só é read-only
  ANTES de setar `refine_desired_face_dist`; depois de calibrado o refine AGE
  (move o robô para a distância certa) — o que é o comportamento de missão.
- Mapa atual da missão: **arena3_520** (default do robot_manipulation.launch).
  O arena2_520 fica como fallback histórico.
- **"No such file: .../caramelo_hardware_ws/.../global_localization.launch.py"**
  (27/07) = ordem dos workspaces invertida. Causa: `install/setup.bash` GRAVA a
  cadeia de underlays da hora do build — rebuildar um ws num terminal com outro
  ws sourceado envenena a ordem para sempre (o pacote duplicado
  caramelo_localization passa a resolver para o hardware_ws). Correção aplicada
  no ~/.bashrc: sourcear `local_setup.bash` (ignora cadeias gravadas; a ordem
  do bashrc vira a única autoridade). REGRA DE OURO para builds: rode `colcon
  build` sempre de um terminal NOVO (bashrc padrão), nunca de um com sources
  manuais extras. E após mexer no bashrc: FECHAR o terminal e abrir outro
  (`exec bash` não limpa — o path antigo fica herdado nas variáveis).
