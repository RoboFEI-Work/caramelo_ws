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

## 3c. AMT1 — Advanced Manipulation Task 1 (ordenar a mesa de precisão, 28/08)

**O que é**: na mesa de precision placement (`PP_1` → pose `Mesa10`, dock `PP1`) há 6 cubos com
tag em ordem aleatória; o robô deve deixá-los em **ordem crescente de id, da esquerda para a
direita** (vista do robô docado). As 6 posições são onde as tags estão no início. O robô usa
os containers de bordo como buffer (precisa de **2 livres** no mínimo: pega A, pega B, solta B
onde estava A…).

**Como o planner reconhece**: `name` contendo "advanced manipulation task 1" (sem distinguir
maiúsculas) OU `task_id` começando com `AMT1`. Aí ele ignora o planejamento de transferências e
emite só `goto PP_1` + `amt1_sort` (a rotina inteira roda no nó `/amt1_sort`). Exemplo
completo em [missions/amt1_pp1.yaml](../missions/amt1_pp1.yaml): `active_service_areas: [PP_1]`
(obrigatório — estação fora da lista é descartada), 6 objetos em `start_state.PP_1`,
`finish_state` igual ao `start_state`.

**O que o robô faz** (nó `amt1_sort_action_node`, `manip_task_execution`): prólogo `home` →
dock na PP1 → observa a mesa de `pegar_obj`, `tag_esquerda` e `tag_direita` (TF das tags) →
**faltou tag esperada → busca lateral obrigatória** (28/08, ver abaixo) → ordena as posições por x
→ fala "Ordem atual: 5, 2, 3, 1, 4, 6. Vou ordenar de 1 a 6" → publica os frames `tag_amt1_slot_k`
(no `odom`, sob cada cubo) → executa a ordenação por ciclos com os picks/places normais
(`/pick_tag`, `/place_tag` com `stack_on: tag_amt1_slot_k`) → se um alvo ficar fora do alcance,
desloca a base (`nudge_base`), re-registra os slots e repete → volta a `pegar_obj` → `home` → FINISH.

**Pré-requisito no robô — calibrar `nudge_odom_lateral_gain` com trena ANTES de confiar na
busca**: a régua de tudo o que segue (posições da busca, volta ao centro, "indo para onde vi a
tag") é o `/manip/base_shift_total` do `dock_align_node`, que soma o lateral medido pelo `/odom`
do EKF — e o EKF só integra as rodas, **não vê o escorregamento lateral** (~18 % no campo: pedir
0,20 m anda ~0,165 m real). Com o ganho em 1,0 (default) o erro se acumula a cada passo e o robô
"volta ao centro" num lugar que não é o centro. Procedimento (docado na PP1, braço em
`pegar_obj`): marcar a posição da base com fita, `ros2 action send_goal /nudge_base
caramelo_msgs/action/NudgeBase "{dy: 0.20, timeout: 20.0}"`, medir com trena o deslocamento real,
ganho = real / odom bruto (o `dock_align_node` loga `fim (...): lateral ... (odom bruto ... x
ganho ...)`), esperado ~0,82; gravar ao vivo com `ros2 param set /dock_align_node
nudge_odom_lateral_gain 0.82` e deixar permanente em
`src/caramelo_navigation/launch/docking_server.launch.py` (lista `parameters` do `dock_align_node`,
como o `refine_desired_face_dist` do §1a); repetir com `dy: -0.20` e conferir que o
`/manip/base_shift_total` volta a ~0 depois de ida e volta. Só então a busca lateral vale alguma
coisa (a bancada não cobre isto: a mesa falsa não escorrega).

**Observação de `tag_direita` → `tag_esquerda`** (31/08, pedido do operador, validado em teste):
`observe_poses`/`reobserve_poses` = `[tag_direita, tag_esquerda]`, com **j2 = 0** nessas duas poses
no SRDF — assim as duas vistas cobrem todas as 6 tags. O que ainda faltar é procurado girando
**só a j1** (abaixo). No pick, se a tag não está na imagem, o nó de
pick usa o fluxo **validado ao vivo em 29/08**: varredura j1 (±0,3/±0,6 rad) quando a tag não está
na imagem e pré-alinhamento pelas poses `tag_esquerda/direita/cima` (31/08: após os picks falharem
no robô, `pick_point_j1_at_stale_tag` e `pick_align_j1_only` voltaram a **false** por default —
religáveis por parâmetro; o buffer TF segue com `tf_cache_sec` 120 s). A sequência normal do pick
continua j1 primeiro → re-detecção → IK recalculada → `gripper_open` → pega → subida vertical.
- **Parada em `tcp2` antes da pega** (31/08): após o alinhamento, o braço vai à pose nomeada
  `pick_pre_ik_pose` (default `tcp2`; vazio desliga; estágio `going_tcp2`), re-detecta a tag dali e a
  escada de IK usa a TF fresca. Pose ausente no SRDF → WARN e segue como antes. Só mesa comum —
  **a pose `tcp2` precisa existir no SRDF** (criar no MoveIt Setup Assistant / à mão).

**Precisão do place** (29/08, pedido do operador "maior precisão possível"), três camadas:
1. **Mediana das leituras**: em cada parada de observação todas as leituras frescas de cada tag
   são acumuladas e a melhor (eixo óptico) vira a mediana, eixo a eixo, das leituras a ≤
   `sample_merge_radius_m` (2 cm) dela, com ≥ `sample_merge_min` (3) leituras — log
   `tag_N em <pose>: mediana de K leitura(s)`.
2. **Re-registro local antes de cada place** (`place_local_reregister`, estágio
   `place_local_reregister_slot_k`): aponta só a j1 para o slot alvo, lê `place_local_dwell_s` (1 s),
   e corrige **só o slot alvo** pelo delta médio das âncoras originais intactas a ≤
   `place_local_anchor_radius_m` (0,30) — 1 âncora já vale, espalhamento ≤ `reregister_max_spread_m`,
   |delta| ≤ `place_local_max_shift_m` (3 cm). A correção fica num campo separado do slot
   (`local_dx/dy`), **sobrescrita** a cada medida (nunca acumula), entra só no frame difundido e é
   zerada no re-registro global. Corrige deriva do odom e o viés câmera→braço perto do alvo.
3. **Verificação/ajuste** (`verify_after_sort`): terminado o plano, observa de novo (pegar_obj +
   varredura silenciosa só pelas tags que deveriam estar na mesa), mede o erro 2D de cada tag
   contra o slot base **relativo às âncoras intactas vistas na mesma observação** (o delta médio
   delas é subtraído — deriva do odom/viés do ponto de vista não viram "erro"; log
   `verify: referência = K âncora(s), delta (...)` e `verify: tag_N no slot j: erro X mm`); só cubos
   que o **robô pousou**
   com erro > `verify_tol_m` (15 mm) são pegos e recolocados (até `refine_max_ops` = 3, ops
   `ajuste_i/N_...`, com o re-registro local); tags intactas fora da tolerância são só informativas.
   Falha no ajuste **não rebaixa** o sort (vira nota "ajuste falhou (...)") **só** se nenhum cubo ficou a
   bordo, o filho não abortou/estourou prazo e o braço está em estado conhecido; verificação
   incompleta (tag pousada não vista) não fala "tudo no lugar" — WARN e nota. A correção local é
   re-medida quando um place é repetido após re-registro global, e zerada quando a medida é rejeitada. Depois, relatório final (`verify_final: ... pior erro X mm`). Fala "Conferi a mesa: tudo
   no lugar" ou "Vou ajustar N tags". Custo: ~10 s de medida + ~60 s por tag ajustada.

**Varredura só com a j1** (29/08, pedido do operador — modo padrão `search_mode: j1`): se depois
das poses de observação faltar alguma tag de `expected_tags`, o robô fala "Vi as tags 1, 2, 3.
Faltou a 4. Vou girar a câmera para procurar" e, da `transport_pose` (`pegar_obj`), gira **apenas a
junta 1** pelas paradas de `search_j1_offsets_deg` (default `[-30, 30, -60, 60]`, + = esquerda,
|offset| ≤ `search_j1_max_abs_deg` 75), lendo as tags `search_j1_dwell_s` (1 s) em cada parada e
**acumulando**; primeiro a menor amplitude e, em cada par, o lado onde viu **menos** tags; para ao
ver todas; volta a j1 de transporte. A base **não anda** (T0 e slots não mudam). Se o movimento
de uma parada falha, a varredura é encerrada (WARN) e o braço volta à j1 de transporte; só essa
volta falhar derruba a observação (`observacao_falhou`). Sem estado atual do braço a varredura é
pulada (nota no resultado). Amostra fora do nível da mesa não conta como "vista".
Se ainda faltar tag, segue para a ordenação parcial (`allow_partial`) — sem falar "não consigo me
mover". Modos: `j1` (só varredura), `base` (só a busca lateral abaixo, comportamento de 28/08),
`j1_then_base` (varredura e, se ainda faltar, a base). Estágios `search_j1_±NN`, `search_j1_return`.
A bancada (`run_amt1_scenario.sh`) roda com `search_mode:=base` (sem braço não há varredura).

**Busca lateral com a base** (28/08; só nos modos `base`/`j1_then_base`): se depois
das poses de observação faltar alguma tag de `expected_tags` (e pelo menos `min_seen_tags` tags
esperadas foram vistas — senão a base nem anda), o robô fala "Vi as tags … Faltou a … Vou me mover
para procurar" e leva a base (sequência de `nudge_base`) até cada posição de `search_offsets_m`
(default `[0.20, -0.20]` m, + = esquerda, **relativos ao `/manip/base_shift_total` do início do
goal**), re-observando das `search_observe_poses` e **acumulando** as tags (o que já foi visto
fica; a amostra vista do centro ganha da vista com a base deslocada; TF de antes do fim do
último nudge é descartada). Para assim que vê todas ("Encontrei todas as tags") e, com
`return_to_center_after_search`, volta ao shift inicial e **re-registra os slots** pelas tags
intactas visíveis de lá. As posições das tags são guardadas num frame de referência fixo T0 (pose
de `manip_base_link` em `odom` no início do goal), então tags vistas com a base deslocada ficam
comparáveis com as vistas do centro (sem T0 o nó re-captura; se ainda não houver, não busca).
**Orçamentos**: a busca tem contador **próprio**, `search_max_nudges` (8: ida, volta ao centro e
os "indo para onde vi a tag"), separado do `max_nudges` (6) das operações; `max_total_shift_m`
(0,35) é o curso físico e vale para os dois; `result.nudges` soma tudo e a `message` termina com
`nudges N (busca a, ops b)`. Orçamento esgotado = fica onde deu (a busca para, a volta ao centro
pode não acontecer e as ops rodam de onde a base ficou; o nó avisa). Se ainda faltar tag:
`allow_partial` **false (default)** → `fail_reason: tag_nao_encontrada`, `missing_tags`
preenchido, `partial: true`, **nenhum cubo movido**; `allow_partial: true` → ordena só o que viu
(`partial: true`, `fail_reason: ''`). `observe_only: true` **não move a base** (28/08): a busca
só roda com `search_in_observe_only: true` (o nó avisa "a base VAI ANDAR").

**"Indo para onde vi a tag"** (28/08): cada tag guarda o shift da base em que a melhor amostra
foi vista (atualizado a cada re-registro). Antes de cada pick, se a base está a mais de
`search_arrive_tol_m + 0,04` m (0,12 m) desse shift, o nó leva a base até lá (estágio
`pick_goto_seen_<tag>`, log `indo para onde vi a tag_N: base em ... m, tag vista em ... m`,
conta no orçamento da busca) e re-registra os slots. Se mesmo assim o pick responder
`tag_nao_encontrada`, o nó vai para onde a viu (1x por op) e repete; já estando lá, re-observa só
com o braço (`retry_not_seen`) e depois desiste (`tag_nao_encontrada`, cubos a bordo devolvidos).
Um pick "fora de alcance" (viu a tag, não alcançou) atualiza esse shift para a posição atual, e o
goto só acontece na 1ª tentativa da op (depois do nudge de alcance o pick é a autoridade — sem
pingue-pongue). Sem orçamento da busca o nó tenta o pick de onde está ("tento daqui") e o pick
fora de alcance ainda tem o `max_nudges` das ops. Uma tag vista **só** na posição deslocada
continua dependendo desse goto (ou da varredura do pick real) para ser pega — por isso o
orçamento da busca é maior que o das ops.

**Passo parcial** (28/08): `nudge_base` que aborta com `curso_incompleto` (ou falha não física
que andou > 2 cm — o `dock_align_node` sempre devolve `travelled`) conta como nudge **parcial**:
o nó soma o `travelled` da odometria (slots e `total_shift_m`), continua a sequência (busca) ou
re-registra e repete a op; só as falhas físicas (`muro`, `obstaculo_lateral`, `timeout`,
`sem_odometria`, `sem_scan`, `muito_perto_da_mesa`, `cancelado`...) viram `nudge_falhou`.

**Rodar**: `ros2 run caramelo_navigation run_mission --map-name arenalegal_2026 --task
/home/linux24-04/caramelo_ws/missions/amt1_pp1.yaml [--dry-run]` (o preflight passa a exigir
`/amt1_sort`). Só observar e falar o plano sem mover cubo:
`ros2 action send_goal /amt1_sort my_robot_msgs/action/Amt1Sort "{ws: PP_1, table_pose: Mesa10,
expected_tags: [1,2,3,4,5,6], max_onboard: 3, observe_only: true}" --feedback`.

**Parâmetros** (nó): `observe_poses`, `observe_dwell_s` 1.5, `min_seen_tags` 2 (parâmetro do nó;
`max_onboard` vem do YAML da ação: 3; 0 = todos os containers vazios; abaixo disto a busca nem
roda), `allow_partial` **false** (28/08; true = ordena o que viu quando falta tag), busca lateral:
`observe_poses` **[pegar_obj]** (29/08), `reobserve_poses` [pegar_obj], precisão: `sample_merge_min` 3,
`sample_merge_radius_m` 0.02, `place_local_reregister` true, `place_local_dwell_s` 1.0,
`place_local_anchor_radius_m` 0.30, `place_local_max_shift_m` 0.03, `verify_after_sort` true,
`verify_tol_m` 0.015, `refine_max_ops` 3, `verify_final_report` true;
`search_mode` **j1** (29/08; `base` | `j1_then_base`), `search_j1_offsets_deg` [-30, 30, -60, 60],
`search_j1_max_abs_deg` 75, `search_j1_dwell_s` 1.0 (varredura só com a j1);
`search_enabled` true, `search_offsets_m` [0.20, -0.20], `search_observe_poses` (= `observe_poses`),
`search_arrive_tol_m` 0.08 (|alvo − posição| abaixo disto = chegou; o nudge real recusa passos <
0.08; o nó nunca deixa ficar abaixo de `min_shift_m`/2), `return_to_center_after_search` true,
`search_max_nudges` 8 (orçamento **próprio** da busca + "indo para onde vi a tag"),
`search_in_observe_only` false (true = `observe_only` também anda de lado); `nudge_enabled` true,
`max_nudges` 6 (só as ops), `min_shift_m` 0.10, `max_shift_m` 0.25, `max_total_shift_m` 0.35 (curso
físico, compartilhado com a busca), `reregister_after_nudge` true (vale também para a volta da
busca e para o goto antes do pick), `retry_not_seen` 1, `op_timeout_s` 300 (prazo de cada
pick/place filho), `cube_height_m` 0.042,
`observe_move_enabled` (false = bancada sem MoveIt). Executor: `amt1_timeout` 1800 s (prazo total da rotina; estourar só
gera **um WARN** e a rotina segue) e `amt1_stage_timeout` 300 s (watchdog por feedback — o **único**
que cancela o goal; 6 picks + 6 places levam 12–18 min).

**Resultado** (`/amt1_sort`): `success` = tudo na ordem; `partial` = mesa consistente (nada a bordo)
mas não totalmente ordenada; `fail_reason` ∈ buffer_insuficiente | poucas_tags_vistas |
tag_nao_encontrada | fora_de_alcance | pick_falhou | place_falhou | nudge_falhou |
container_inconsistente | observacao_falhou | cancelado. `tag_nao_encontrada` antes de qualquer
pick = faltou tag mesmo após a busca lateral com `allow_partial=false` (`missing_tags` diz quais,
`picks: 0`, mesa intacta, `nudges` conta os da busca). Falha "limpa" (alvo fora de alcance, tag
não vista no pick — garra vazia): o nó devolve os cubos a bordo aos slots livres (`recover_onboard`)
e responde `partial=true`. Abort ou prazo estourado de um pick/place (`pick_falhou`/`place_falhou`): o
cubo pode estar na garra, então o nó **não solta nada** — os cubos ficam nos containers (yaml
preservado), `partial=false`, só o braço volta a `pegar_obj`; o operador resolve à mão antes de
outro goal.

**Bancada sem robô** (`ROS_DOMAIN_ID=99`): `src/manipulation/manip_bt/test/run_amt1_scenario.sh all`
(planner + variantes AMT-1/"Test 1"/PP inativa, regressão AMT2/AMT10, dry-run + validação de
`max_onboard`/`expected_tags`, main 5,2,3,1,4,6, dois ciclos, nudge, buffer 1, **busca lateral**
(`search`: mesa falsa com campo de visão `view_x_max` 0.25 e pontas em ±0.35 → vê 4, anda até +0.20 e
−0.20, acha as duas, volta ao centro, re-registra e, antes do pick_5 e do pick_4, vai para onde viu
cada tag → ordena tudo com 6 nudges: busca 6, ops 0), **orçamentos separados** (`search_budget`:
`search_max_nudges` 4 esgotado na busca, "tento daqui" e as ops ainda usam 2 dos 6 nudges de alcance
delas → success), **pick que respeita o campo de visão** (`search_pick_view`: mesa falsa com
`pick_respects_view` → o pick da tag_5 só passa porque o nó foi antes para onde a viu; variante
`_nobudget` → sem orçamento da busca o nó repete 1x com o braço e falha limpo, 0 picks), **passo
parcial** (`short_course`: `fake_nav_actions.py -p nudge_short_course:=1` → 1º nudge aborta com
`curso_incompleto` e `travelled` = 80 % → o nó conta o passo, re-registra e repete; variante
`_search` com o passo parcial na busca), tag nunca vista (`unseen`: busca sem achar →
`tag_nao_encontrada`, 0 picks; `unseen_partial` com `allow_partial:=true` → ordena o que viu), place
fora de alcance com recuperação, place abortado sem recuperação, pick que estoura o prazo, cancel,
missão completa, executor com fake: ok/abort/result/reject/watchdog/prazo total). A mesa falsa
(`fake_amt1_table.py`) aceita `view_x_max` (tag só entra no TF enquanto |x0 + shift| ≤
`view_x_max`; shift + = esquerda faz o x das tags crescer) e `pick_respects_view` (o `/pick_tag`
também responde `tag_nao_encontrada` fora do campo de visão); o `fake_nav_actions.py` aceita
`nudge_short_course` (N primeiros nudges abortam com `curso_incompleto`, `travelled` = dy×0,8 e o
total publicado com o parcial, como o `dock_align_node`). gtest da biblioteca:
`colcon test --packages-select manip_task_execution --ctest-args -R test_amt1` (inclui as 720
permutações de 6).

**Sem `nudge_base` no ar** (29/08, visto em campo): a busca lateral precisa do `dock_align_node`
(sobe com a navegação). Com `--simulate-nav`, ou com o `docking_server` desligado, o nó loga
`nudge_base indisponivel` e, em vez de desistir, **ordena as tags que viu** (fala "Não consigo
me mover para procurar. Vou ordenar as tags que vi"). Para testar a busca de verdade a
navegação precisa estar no ar (o `nudge_base` aparece em `ros2 action list`).

**Perto demais da mesa** (29/08, visto em campo): o docking para com o bico a ~0,35–0,39 m e o
`nudge_base` recusava (`muito_perto_da_mesa`). Agora o `nudge_base` **recua em linha reta**
(`nudge_backoff_*`: até 12 cm, fala "Vou recuar um pouco antes de me mover de lado") e só então
anda de lado; se nem recuando houver folga, recusa. Na AMT1 uma recusa de segurança do nudge
durante a busca vira "busca impossível" → ordena o que viu; `nudge_falhou` só para muro/timeout.

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
- AMT1 (28/08, ver §3c): kind `amt1_sort` (`ws`, `table_pose`, `expected_tags: [ids]`,
  `direction`, `max_onboard`) — exige um `goto` anterior, fecha a fila de alcance e roda a rotina
  inteira no nó `/amt1_sort`; params do executor `amt1_timeout` (1800 s, prazo total — estourar só
  avisa) e `amt1_stage_timeout` (300 s, watchdog por feedback — o único que cancela o goal).
- Fila de alcance (28/08, ver §6d): `reach_queue_enabled` (true),
  `reach_queue_ws_prefixes` (`["WS_","PP_"]`), `reach_queue_max_nudges` (2),
  `reach_queue_max_shift_m` (0.25), `reach_queue_min_shift_m` (0.10),
  `nudge_timeout` (20 s, goal do `nudge_base`).

- Ordem das juntas na pega (28/08, pedido do operador): na aproximação por cima o pick
  gira **j1 primeiro** (log `mesa custom down [so j1]`) e só depois desce o resto
  (`[resto]`); na volta a `pegar_obj` com o bloco, recolhe j2..j5 (`[j1 parado]`) e gira
  **j1 por último** (`[j1 por ultimo]`) — o braço estendido nunca varre a mesa girando.
- **Garra larga ao chegar em `pegar_obj`** (29/08): em mesa comum, assim que o braço chega
  em `pegar_obj` a garra vai para `gripper_full_open` (estágio `gripper_full_open_at_pegar_obj`)
  antes do ciclo de pega; na prateleira não (dedos esbarrariam nas tábuas). Parâmetro
  `pick_gripper_pose_at_pegar_obj` (default `gripper_full_open`; vazio desliga). Se a pose não
  existir no SRDF vira WARN e o pick segue com a abertura normal. A pose larga serve **só para
  enxergar as tags** (dedos fora da imagem na detecção e na re-detecção pós-j1): antes da
  descida (`[resto]`) a garra volta a `gripper_open` (estágio `gripper_open_before_descent`);
  se essa volta falhar o ciclo não desce de garra larga (falha da tentativa, entra no retry).
- **Subida vertical pós-pega** (29/08): em mesa comum, com o bloco preso e verificado, o braço sobe
  `pick_lift_after_grasp_m` (default 0,05; 0 desliga) na vertical — mesmo x,y, mesma inclinação e
  punho da pega (IK custom; estágio `lifting_after_grasp`) — e só então recolhe para `pegar_obj`
  com j1 por último. Sem IK na altura pedida tenta a metade; sem nada, recolhe como antes (WARN).
- Re-detecção após a j1 (29/08): a câmera erra o ponto da tag quando ela está longe; com a
  j1 já apontada o pick espera `pick_recompute_ik_settle_ms` (300), lê uma TF **fresca** da tag
  e recalcula a IK **uma vez** (estágio `re_detecting_after_j1`, log `IK recalculada apos a j1:
  alvo moveu dx dy dz`). Sem TF fresca mantém a primeira IK; se de perto não houver IK, vira
  "fora de alcance" (fila de alcance). Desligar: `pick_recompute_ik_after_j1:=false`.

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
`reach_queue_max_nudges` (2) vale por estação, não por fila. O kind `amt1_sort` (§3c) também
fecha a fila de alcance da estação (o `NudgeBase` entra antes dele) e roda fora dela — o nó
`/amt1_sort` faz os próprios ajustes de base.

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
