# Mesa de Precisão (PP) — arquitetura atual, auditoria e recomendação

**Data:** 2026-11-14 · **Fase:** AUDITORIA (somente leitura) · **Branch:** `detectando_container_e_usando_yaml_novo` (HEAD `fbb8eab`)

**Revisão de 14/11:** aplicadas as 17 correções do verificador independente e acrescentado o
teste de visão feito no robô em 14/11 (seção 18). O teste muda a recomendação: o alinhamento
deve medir a aresta do cubo contra a borda do furo-alvo na mesma imagem, com as tags vizinhas
como apoio, em vez de depender só da tag do slot.

Nenhum código, config, URDF ou `.jarvis` foi alterado para produzir este documento. O único uso
do robô foi o teste de 14/11, pelo fluxo normal de missão. Ele é a
síntese de 5 frentes de auditoria (fluxo de execução; AprilTag + câmera; detector de furos;
TF/calibração/cinemática; cubo na garra + histórico) e 2 de pesquisa (percepção/geometria;
calibração/controle).

## Como ler

- **FATO** = conferido em código, config, log ou bag, com evidência `arquivo:linha`.
- **HIPÓTESE** = explicação que os dados sustentam, mas que não foi provada.
- **ESTIMATIVA** = número derivado por geometria ou análise offline, não medido no encaixe.
- "(conferido)" = o sintetizador releu o trecho no código/log antes de escrever. O resto vem
  das frentes, com a evidência que elas entregaram.

Abreviações de caminho (relativos a `/home/robofei-work/caramelo_ws`, salvo indicação):

| Sigla | Arquivo |
|---|---|
| PLACE | `src/manipulation/manip_task_execution/src/mtc_place_action_node.cpp` |
| PICK | `src/manipulation/manip_task_execution/src/mtc_pick_action_node.cpp` |
| PREC | `src/manipulation/manip_task_execution/src/place_precision.cpp` (+ `include/.../place_precision.hpp`) |
| IK | `src/manipulation/manip_task_execution/include/manip_task_execution/custom_ik.hpp` (+ `src/custom_ik.cpp`) |
| SETTLE | `src/manipulation/manip_task_execution/include/manip_task_execution/arm_settle.hpp` |
| HOLE | `src/manipulation/manip_bringup/scripts/pp_hole_detector_node.py` |
| HOLEY | `src/manipulation/manip_bringup/config/pp_hole_detector.yaml` |
| TAGS | `src/manipulation/manip_bringup/config/tags_36h11.yaml` |
| LAUNCH | `src/manipulation/manip_bringup/launch/manip_stack.launch.py` |
| BT | `src/manipulation/manip_bt/src/bt_yaml_executor.cpp` |
| HW | `/home/robofei-work/caramelo_hardware_ws/src` |
| ARMX | `HW/caramelo_description/urdf/mech/manipulator_arm.xacro` |
| CTRL | `HW/raspberry_bringup/config/caramelo_controllers.yaml` |
| DRV | `HW/caramelo_hardware/include/caramelo_hardware/xm540_driver.hpp` |
| RUN | `runs/run_20261113_210343` (única corrida em bag com PP usada na auditoria) |
| LOG | `RUN/logs/rosout.log` |
| RUN14 | `runs/run_20261114_112951` (teste de visão de 14/11, seção 18) |
| LOG14 | `RUN14/logs/rosout.log` |
| SCR | `/tmp/claude-1000/-home-robofei-work-caramelo-ws/99a9f780-9c3a-4228-b6e8-536d77cf4a47/scratchpad` (scripts e fontes das frentes; **temporário**). **Cópia persistente desde 14/11:** `runs/pp_auditoria_20261114/`, com as mesmas subpastas; as citações `SCR/...` valem trocando o prefixo. |

### O que o sintetizador conferiu pessoalmente

1. Alvo do slot = tag + (0; −0,052 m) somado nos eixos de `manip_base_link`, lógica de pilha
   ligada por default: PLACE:186-203, 3940-3962 (conferido).
2. Tamanho das tags 0.041 para todos os ids listados e o comentário sobre `fx`: TAGS:5, 32-42,
   60-66 (conferido). Calibração de fábrica da cor em 640x480: `fx = 386,39`, modelo Inverse
   Brown Conrady com coeficientes não nulos; profundidade `fx = 390,67`:
   `SCR/pp_apriltag_camera/rs_enumerate_devices_c.txt:175-181, 252-261` (conferido).
3. O apriltag recebe `image_raw` remapeada como `image_rect`; perfil default 640x480x5:
   LAUNCH:169-176, 571-589 (conferido).
4. Chegada e verificação só informativas; controlador aceita 0,30 rad por junta; a espera de
   assentamento só exige parar: PLACE:134-135, 329-330, 4276-4291, 5884-5891; CTRL:86-89,
   107-111; SETTLE:19-22 (conferido).
5. Câmera presa ao `link5` com extrínseca nominal e `tcp` a 0,17 m: ARMX:254-264, 332-336,
   350-354; driver sem endereços de ganho e com a constante 651,0886: DRV:7-30, 179 (conferido).
6. Log da corrida de 13/11: LOG:15146, 15193-15196, 15199, 15608, 15611, 15746, 15886, 15975,
   16177, 16611, 17059, 17485, 17728 (conferido).
7. Centro do furo = centro do `minAreaRect`; plano vem de `/manip/pp_plane_z`: HOLE:177-178,
   198, 234-252, 636-642; HOLEY:22, 24, 33, 40, 48, 59, 67, 75, 101 (conferido).
8. Cálculo de "fora de alcance" do slot usa a escada da pilha; re-tentativa do fim só roda com
   árvore em SUCESSO: PLACE:4127-4131, 6509-6513; BT:1565-1566 (conferido).
9. Regra da cavidade e da tag nos regulamentos baixados pela frente de pesquisa:
   `SCR/pp_pesquisa_calib/txt/rulebook2023.txt:1013-1021, 1936-1946` e
   `SCR/pp_pesquisa_calib/cbr_rulebook_2025.txt:414-415, 496-509` (conferido).

### Avisos sobre os dados

- **A frente "pesquisa calibração/controle" chegou truncada.** Só o método 1 (fechar a malha no
  TCP medido) chegou, e incompleto. As fontes que ela baixou estão em `SCR/pp_pesquisa_calib/`;
  o sintetizador leu apenas trechos delas (identificados na seção 12). A análise dos demais
  métodos dessa frente **não está neste documento**.
- **A auditoria tinha uma só corrida em bag com PP** (RUN: 2 slots, 1 soltura). O histórico vem
  de 99 tentativas em 16 logs antigos do place (`~/.ros/log/mtc_place_action_node_*.log`), que
  só trazem o que o nó escreveu. O teste de 14/11 (RUN14) acrescenta uma segunda corrida, com 1
  soltura e imagens.
- **Quase não há verdade de campo.** Até 13/11 nenhum log, bag ou foto dizia se um cubo assentou
  no furo; o único número de resultado era a leitura de verificação feita pela mesma câmera, que
  tem viés próprio. O primeiro resultado físico registrado é o de 14/11 (seção 18): 1 soltura, o
  cubo não entrou.
- **O código de manipulação da corrida de 13/11 é o mesmo do HEAD.** RUN foi gravada em
  `git=b5a44fe` (`RUN/metadata.json:72`) com a árvore suja só fora de `src/manipulation`
  (`RUN/config/git_diff.patch`: 15 arquivos, nenhum em `src/manipulation`), e
  `git diff b5a44fe HEAD -- src/manipulation` é vazio (conferido). É isso que autoriza cruzar
  número de linha do código com o LOG. RUN14 foi gravada no HEAD `fbb8eab` com `git_diff.patch`
  vazio (conferido).
- `run_20261114_110352` estava sendo gravada durante esta auditoria (tarefa ATT1) e não foi lida.
- **Evidência em diretórios temporários.** Os scripts das frentes, o `rs_enumerate`, os
  regulamentos e os arquivos do teste foram copiados em 14/11 para
  `/home/robofei-work/caramelo_ws/runs/pp_auditoria_20261114/` (101 MB). `/tmp/pp_hole_debug`
  (643 MB, 1715 imagens de 12-13/11, base das medidas de P14, P15 e P16) **não foi copiado**, por
  decisão do operador, e some no reboot: esses números deixam de ser reproduzíveis depois disso.

---

## 1. Resumo executivo

O encaixe não tem uma causa única. A tabela lista oito fontes de erro; as quatro primeiras
passam, cada uma, da folga da cavidade. Elas não são independentes (a escala da tag contamina o
offset, o plano do detector e a altura; seção 9), e nada no fluxo corrige nenhuma delas depois
que o cubo é pego. Em ordem de impacto provável:

| # | O que impede o encaixe | Tamanho | Natureza |
|---|---|---|---|
| 1 | **A posição lida da tag do slot depende da pose do braço.** A mesma tag parada sai em pontos diferentes de `manip_base_link` conforme de onde é vista. O alvo usa uma só dessas leituras. | Longe × perto: mediana 43 mm em xy e 43 mm em z (n=97). Na mesma pose repete em menos de 1 mm. | FATO (medido). Causa **em disputa** (10.1): `size` 0.041 para um quadrado preto de ~32-33 mm (escala ~1,25) explica a maior parte da discrepância entre vistas, mas com a escala corrigida o tampo sai ~8 cm acima da altura física, e isso não tem explicação. Sobra ~18 mm rms que a escala não explica (erro angular de ~6-8° na cadeia; a leitura de longe é feita com a ferramenta a ~20° da vertical, P05) — HIPÓTESE. |
| 2 | **O braço não chega onde é mandado e solta assim mesmo.** Erro de regime dos servos sob gravidade (j2, j3). | Na soltura gravada: TCP 6,6-7,0 mm mais perto do robô, 15 mm mais baixo, ferramenta a 2,7°. Nas 92 solturas de PP com log: mediana 1,8 cm (norma 3D). | FATO (medido). Linear com o esforço (R² 0,99), logo compensável — a causa "malha só proporcional" é HIPÓTESE. |
| 3 | **Do agarre à soltura é malha aberta e não há critério de sucesso.** O alvo é lido uma vez, 15-20 s antes, de garra vazia; o cubo é buscado às cegas; a verificação pós-soltura só registra. | Todo o erro de 1, 2, 5 e 6 chega inteiro ao furo. Em 13/11 o nó leu o cubo a 16,8 mm do alvo e respondeu "Entreguei o bloco". Em 14/11 leu 19,1 mm, decidiu "repetir", respondeu "Entreguei o bloco" e o cubo estava fora do furo. | FATO. |
| 4 | **A única correção que enxerga o furo não corrige.** O detector de buracos não atuou em 3 de 3 observações na corrida gravada; em 12/11, com fusão em todos os slots, o resíduo lido foi mediana 14,4 mm; em 14/11 atuou (fusão com peso 0,7) e o cubo ficou fora do furo. | Resíduo pós-soltura lido pelo robô: mediana 14-21 mm em todas as sessões; só 9 de 84 leituras com até 8 mm. | FATO. A conversão métrica do furo herda a escala e o plano da tag (10-30 %) — FATO medido offline em 5 quadros. |
| 5 | **Offset tag→furo fixo (0; −52 mm) nos eixos do robô**, um valor para os 7 slots, medido pela própria câmera. | 0,9 mm por grau de desalinhamento do docking; distância real estimada em 40-46 mm (não 52). | FATO (código) + HIPÓTESE (valor inflado pela escala). |
| 6 | **A pose do cubo dentro da garra é desconhecida.** Re-pega cega no container de bordo, com a ferramenta a 6-11° da vertical. | 2-3 mm observados só na direção de fechamento; o resto não foi medido. Em 14/11 a tag do cubo não foi detectada em nenhum quadro com ele na garra (fica cortada pela borda de cima da imagem); a aresta frontal e os cantos inferiores do cubo ficam visíveis. | FATO (não há medida) + HIPÓTESE (inclinação). |
| 7 | **Altura de soltura indeterminada.** O frame `tcp` (0,17 m) fica 10-13 mm além da ponta do dedo no CAD; o z "fecha" por cancelamento de dois erros. | Entre ~0 e ~4 cm acima do tampo, conforme a hipótese. | FATO (CAD) + HIPÓTESE. |
| 8 | **Confiabilidade:** a PP só desce a 0°, o cálculo de alcance usa a escada da pilha, e um abort derruba a árvore inteira. | Em 13/11: 2 de 3 tentativas "fora de alcance"; missão terminou em FALHA com ações pendentes. | FATO. |

**Teste no robô em 14/11 (seção 18), primeira verdade de campo.** Missão normal, 1 soltura no
slot 104, com o detector de furos atuando. O cubo **não entrou**: ficou pousado sobre a tag do
slot, ao lado do furo, com erro de centímetros. A verificação do próprio nó leu 19,1 mm,
decidiu "repetir" e, mesmo assim, o nó respondeu "Entreguei o bloco" e a árvore terminou em
SUCESSO. Com o cubo na garra, a tag do slot-alvo some atrás do cubo na descida final e a tag
do cubo nunca é detectada; o que aparece na mesma imagem é a aresta frontal do cubo, a borda do
furo-alvo logo adiante dela e o furo e a tag do slot vizinho.

Dois pontos que mudam a conta e precisam de régua antes de qualquer decisão:

- **A folga real pode ser menor do que se supõe.** O regulamento diz "dimensão do objeto mais
  2 mm de offset em cada dimensão" (cubo de 42 mm → cavidade de 44 mm, ou seja, ~1 mm por lado
  se os 2 mm forem o total). A medida por homografia de uma frente deu abertura de
  44,2-45,4 mm. Os comentários do código falam em 5 cm e folga de 3,5 mm por lado. HIPÓTESE:
  a folga é de 1 a 1,7 mm por lado.
- **O tamanho da tag está em disputa** (seção 10.1). Há indícios para 0,032-0,033 e para 0,041,
  e um fator de escala uniforme não satisfaz todos ao mesmo tempo. A régua (distância entre os
  centros de duas tags vizinhas da PP: ~110 mm ou ~140 mm) e o teste T1 (profundidade alinhada
  contra a distância da tag) têm de vir antes de qualquer conclusão sobre a causa.

**Recomendação em uma frase:** parar de tentar acertar o furo por posição absoluta e passar a
medir, na mesma imagem e na pose de soltura, a aresta visível do cubo contra a borda do
furo-alvo, com as tags e os furos dos slots vizinhos como apoio, corrigindo com pequenos
movimentos até convergir (olhar-mover-olhar). A malha não pode depender só da tag do slot, que
o cubo tapa (seção 18). Antes de codar, um teste-portão barato (seção 15); a calibração completa
da cadeia fica para depois da competição (seção 14). O primeiro passo, de custo quase nulo, é o
nó parar de declarar sucesso quando a própria verificação decide "repetir".

---

## 2. Arquitetura atual

### 2.1 Nós e pacotes

| Nó | Pacote | Papel na PP | Evidência |
|---|---|---|---|
| `run_mission.py` | `caramelo_navigation` | Dispara a missão | frente fluxo |
| `task_planner` | `manip_bt` | Lê o YAML 2026 (`precision_tables.PP.slot_tags`, `finish_state.PP.slots`) e gera `place` com `at_tag: tag_10x`, `ws: PP` | `manip_bt/src/task_planner.cpp:1100-1116`; `task_plan_search.cpp:1045-1046` |
| `bt_yaml_executor` | `manip_bt` | Executa a árvore; copia `at_tag` para `stack_on` do goal; fila de alcance (`ReachQueue`, prefixos WS/PP/SH, até 2 nudges laterais) | BT:677-698, 1451-1458; `reach_queue_bt.cpp:233-251` |
| `mtc_place_action_node` (`place_action_server`) | `manip_task_execution` | Servidor de `/place_tag`: observa o slot, pega o cubo do container de bordo, solta | PLACE |
| `mtc_pick_action_node` (`pick_action_server`) | `manip_task_execution` | Servidor de `/pick_tag`: pega o cubo na mesa e guarda no container de bordo | PICK:1840-1855, 2915-2924 |
| `amt1_sort_action_node` | `manip_task_execution` | Servidor de `/amt1_sort` (AMT1); cliente de `/pick_tag` e `/place_tag` com frames `tag_amt1_slot_k`. **Não auditado por inteiro.** | LAUNCH:493-524 |
| `move_group` | MoveIt 2 | Planejamento (OMPL RRTConnect + parametrização por tempo), `pick_ik` | `manip_moveit_config/config/kinematics.yaml:1-10` |
| `realsense2_camera` | driver | D455, cor RGB8 e profundidade Z16 em 640x480 a 5 fps, `align_depth` ligado, `publish_tf=false` | LAUNCH:169-176, 543-556 (conferido) |
| `apriltag_node` | `apriltag_ros` | `/detections` (cantos em pixel) + TF `camera_color_optical_frame → tag_N`; `nice +15`, respawn | LAUNCH:571-589 (conferido) |
| `pp_hole_detector_node.py` | `manip_bringup` | Centro das cavidades por cor; `PoseArray` em `manip_base_link` | HOLE; LAUNCH:616-628 |
| `container_detector_node.py` | `manip_bringup` | Containers vermelho/azul da mesa (`ct_vermelho`/`ct_azul`). Não localiza cubos. | LAUNCH:595-598 |
| `workstation_surface_node.py` | `manip_bringup` | Superfície da mesa; único consumidor da profundidade alinhada | `config/workstation_surface.yaml:96-112` |
| `robot_state_publisher`, `arm_controller` (JTC), interface de hardware | na Raspberry (`caramelo_description`, `raspberry_bringup`, `caramelo_hardware`) | TF do braço a partir dos encoders; execução das trajetórias | ARMX; CTRL; DRV |

Launch: `caramelo_bringup/launch/robot_manipulation.launch.py:278-295` inclui LAUNCH.

### 2.2 Actions e tópicos

| Interface | Tipo | Quem usa |
|---|---|---|
| `/place_tag` | `my_robot_msgs/action/PlaceTag` (campos `tag_frame`, `table_pose`, `ws`, `stack_on`, `container_color`, `final_attempt`, `place_joints`, `wall_sides`). **Não tem campo de PP**: o nó classifica o alvo pelo id (101..107 = slot; 200 = pilha). | BT → PLACE (PLACE:169-172, 6467-6472) |
| `/pick_tag` | `my_robot_msgs/action/PickTag` | BT → PICK |
| `/amt1_sort` | action própria | BT → AMT1 |
| `/detections` | `apriltag_msgs/AprilTagDetectionArray` (cantos, homografia, **sem pose**) | apriltag → bag; só o `vision_debug_node.py` assina (`manip_bringup/scripts/vision_debug_node.py:22, 707`), para visualização; nenhum nó de controle consome os cantos |
| `/tf`, `/tf_static` | TF | todos |
| `/joint_states`, `/arm_controller/controller_state` | ~100 Hz | MoveIt, SETTLE |
| `/manip/pp_plane_z` | `Float64` latched | PLACE → HOLE (PLACE:4647-4666) |
| `/manip/place_active` | habilita o detector | PLACE → HOLE (HOLE:613-626) |
| `/manip/pp_hole_detector/holes`, `/status`, `/debug_image` | `PoseArray` em `manip_base_link`, texto, imagem | HOLE → PLACE |

**Parâmetros:** o launch não passa nenhum parâmetro da PP (`slot_place_*`,
`slot_tilt_ladder_deg`, `slot_hole_*`, `slot_use_stack_logic`) nem `stack_*` ao nó de place.
Passa só 5 `slot_*` de seleção de vaga na mesa comum (`slot_selection_enabled`,
`slot_min_clearance_m`, `slot_observe_empty_gripper_first`, `slot_fallback_free_xy_enabled`,
`slot_obstacle_tag_frames`; LAUNCH:468-476, conferido). Para a PP valem os defaults do
`declare_parameter` no código. Quem quiser ajustar a PP hoje precisa recompilar ou usar
`ros2 param set`.

---

## 3. Fluxo de execução da PP

1. **Missão.** O planejador emite `place` com `at_tag: tag_10x`, `ws: PP`, `table_pose: Mesa10`
   (`RUN/config/20261113_210337_actions.yaml:134-141`). O executor repassa `at_tag` como
   `stack_on`. O place fica dentro de uma `ReachQueue`.
2. **Classificação.** O nó reconhece o slot pelo id 101..107 e, com
   `slot_use_stack_logic=true` (PLACE:201, conferido), usa a **lógica do empilhar**
   (`preObserveStackBase` + `placeStackedOnFrame`). O encaixe antigo com verificação de assento,
   re-pega e espiral (`placeIntoPrecisionSlot`, PLACE:5352) existe e está **desligado**.
3. **Observação de garra vazia** (PLACE:7460-7481, 4329-4592):
   `opening_gripper` (full_open) → `stack_pre_observe` (pose `pegar_obj`, lê a tag de longe) →
   `stack_pre_observe_lateral` (se |x| > 0,10 m) → `stack_pre_observe_tcp2` (garra vazia ~13 cm
   acima do ponto tag+offset; lê a tag **de perto**; publica `/manip/pp_plane_z`; espera até
   2,5 s pelo detector de buracos).
4. **Busca do cubo às cegas** (PLACE:7534-7619): volta a `pegar_obj` → `gripper_open` →
   `pre_container` → `containerN` (pose nomeada do SRDF) → `closing_gripper` → `pre_container`
   → `pegar_obj` → `going_table`.
5. **Soltura sem reler** (PLACE:3835-3845, 4198-4201): `stack_using_preseen_base` →
   `stack_j1_aim` → `stack_pre_pose` (2 cm acima) → `stack_final_approach` →
   `opening_gripper_final` → `stack_lift_after_release`.
6. **Verificação só informativa:** `stack_verifying` (lê a tag do cubo) →
   `returning_pegar_obj_final` → `done`. O goal responde sucesso depois de abrir a garra.

**Cálculo do alvo** (PLACE:3940-3962, 4609-4622, 4762-4821):

- xy = tag (leitura de perto, garra vazia) + (0; −0,052 m) **nos eixos de `manip_base_link`**;
  se o detector viu um buraco a até 5 cm na leitura de perto, média ponderada com peso 0,7 no
  buraco; sem buraco pré-visto, tenta o buraco "ao vivo" (`nearestLiveHole`: última mensagem do
  detector com até 2 s, posterior à última mudança de plano; PLACE:4667-4680, 4776-4781,
  conferido); senão o offset tag→buraco aprendido (2 amostras ou mais, só em RAM); senão o
  nominal.
- z = z da tag + 0,042 (cubo) + 0,005 (folga).
- yaw = yaw da tag **módulo 90°**: o punho vai ao quarto de volta de menor giro a partir do yaw
  atual dos dedos (`nearestQuarterTurn` = `remainder(yaw_tag − yaw_atual, π/2)`; PREC:50-55,
  PLACE:4848-4857, conferido). O yaw lido não é arredondado: um erro de 3° na leitura passa
  inteiro para o cubo.
- Descida só vertical (`slot_tilt_ladder_deg=[0]`), como **alvo de juntas** enviado ao MoveIt
  (não há trecho cartesiano), com fator de velocidade e aceleração 1.0 (PLACE:1573-1591,
  4241-4252, 7032-7035).

**Falha** (PLACE:6382-6396, 6506-6557): tag não vista, sem IK ou fora de alcance → o cubo volta
ao container de bordo e o goal sai `unreachable` (sugestão de deslocamento 0 na PP). Na
tentativa final o goal aborta, a folha vira FAILURE e a árvore inteira termina em FALHA; a
re-tentativa do fim da missão só roda com árvore em SUCESSO (BT:1565-1566, conferido).

**A corrida de 13/11 21:19, linha a linha** (LOG, conferido):

| Hora | Evento | Linha |
|---|---|---|
| 21:19:19.7 | `tag_104` de longe, mediana de 7: `[-0.0088 0.4670 -0.0834]` | 15146 |
| 21:19:22.2 | De perto (TCP2): só 3 leituras em 700 ms → leitura única `[-0.0163 0.4090 -0.0589]`; "base moveu dx=−7,5 dy=−58,0 dz=+24,5 mm" | 15193-15196 |
| 21:19:24.7 | Detector: "não segmentou nada" em 2,5 s → alvo pela tag | 15199 |
| 21:19:36.9 | Alvo final xy `[-0.0163 0.3570]` = tag + (0; −52 mm), "+0,0/+0,0 mm do nominal" | 15608 |
| 21:19:37.0 | Alvo do TCP `[-0.016 0.357 -0.012]`; juntas finais `[1.616 0.723 0.849 1.570 -1.544]` | 15611 |
| 21:19:39.2 | "erro de FK na chegada: 1.7 cm (informativo)"; juntas reais j2 = 0,7603 (+37 mrad) | 15746 + `/joint_states` do bag |
| 21:19:43.5 | Verificação: dxy = 16,8 mm, dz = −2,4 mm, "mas não mexo" | 15886 |
| 21:19:45.5 | "Entreguei o bloco" | 15975 |
| 21:20:10.4 e 21:20:44.8 | Slot 105: IK sem solução a 0° para `[0.098 0.381 -0.007]`; cubo pego e devolvido duas vezes | 16611, 17485 |
| 21:20:54.0 | "Árvore terminou com FALHA" | 17728 |

A base não se moveu entre a observação e a soltura (odometria: 0,0 mm e 0,000° entre 21:19:16 e
21:19:45, frente fluxo).

---

## 4. Fluxo de percepção

### 4.1 AprilTag

- **Entrada:** imagem **crua** da cor (`image_rect` remapeado para
  `/camera/camera/color/image_raw`) e `camera_info`, QoS best-effort (LAUNCH:582-586, conferido).
- **Detector:** família 36h11, `max_hamming 0`, `decimate 2.0`, `refine true`,
  `sharpening 0.25`, 4 threads (TAGS:3-20, conferido).
- **Pose:** PnP nos 4 cantos, só RGB, intrínsecos de fábrica, **sem correção de distorção** e sem
  profundidade. Uma frente refez o PnP com `cv2.solvePnP` a partir dos cantos de `/detections` e
  reproduziu o TF publicado com erro mediano de 0,30 mm (n=888). A pose sai só como TF.
- **Tamanho:** `size: 0.032` no topo vale só para ids fora da lista; a lista `sizes` põe 0.041
  em todos os ids usados (0.04 no id 5) (TAGS:5, 60-66, conferido).
- **Taxa:** 5 fps nominais, 4,0 Hz efetivos, 17 % dos intervalos com quadro perdido, buracos de
  até 2,2 s. A perda é medida **em `/detections`** (o bag exclui `/camera/*`); não se sabe se
  quem perde é a câmera ou o `apriltag_node`, que roda com `nice +15` e QoS best-effort
  (LAUNCH:574, 587, conferido).
- **Uso no place:** `lookupTransform(manip_base_link ← tag_N)` com idade máxima de 1 s; mediana
  de 5 leituras distintas em 700 ms depois de o braço parar (PLACE:142-145, 1763-1907). A
  mediana funciona na maior parte das vezes: nos 16 logs antigos a leitura de perto saiu por
  mediana (6 a 13 leituras) em 70 de 71 casos, e em 13/11 a leitura de longe saiu por mediana
  de 7 (LOG:15146). Em 13/11 a leitura de perto caiu para leitura única 3 de 3 vezes; na
  `tag_104`, porque faltaram 2 quadros dentro da janela. "Leitura distinta" é um **stamp de TF
  novo**, não um quadro de câmera (o dedupe é pelo stamp do `lookupTransform(TimePointZero)`;
  PLACE:1678-1687, 1797-1806, conferido): houve 7 "leituras" numa janela com no máximo 5
  quadros, então a mediana tem menos amostras independentes do que o log diz. HIPÓTESE do
  mecanismo: o stamp comum alterna entre a TF das juntas (~18 Hz no bag) e a da tag.
- **Ruído com câmera parada:** desvio-padrão mediano 0,15 / 0,12 / 0,22 mm em x/y/z e 0,04° de
  yaw (101 segmentos, 724 leituras). **O ruído não é o problema; o erro é sistemático.**

### 4.2 Detector de furos (`pp_hole_detector_node.py`)

1. Reduz a imagem a 320x240, converte para HSV, máscara marrom (H 5-25, S 50-255, V 40-230),
   zera a região da garra, morfologia (abertura 3 px, fechamento 5 px) (HOLE:134-231).
2. Contornos externos filtrados por área, solidez, proporção, retangularidade e "anel branco"
   em volta (`white_v_min 140`).
3. **Centro = centro do `cv2.minAreaRect` em pixel**; o ângulo do retângulo é descartado e a
   pose sai sem orientação (HOLE:177-178, 198, conferido).
4. Pixel → métrico por **ray-cast de um pixel** (com undistort) pela TF da câmera até o plano
   horizontal `z = plane_z`, onde `plane_z` é o z da tag do slot lida de perto
   (`/manip/pp_plane_z`; sem tópico, −0,029). Não usa profundidade nem homografia
   (`container_detector_node.py:366-400`; HOLE:234-264, 636-642).
5. Rastreio por proximidade (3 cm), até 5 amostras em 2,5 s, a 3 Hz, só com a câmera parada;
   publica com 2 amostras ou mais. A "mediana" é o elemento de índice `len//2` (HOLE:267-318).
6. Só roda durante o place (`/manip/place_active`).

### 4.3 Fusão no place

- Dois caminhos: o buraco visto na leitura **de perto** de garra vazia (PLACE:4470-4487,
  conferido) e, sem ele, o buraco "ao vivo" (`nearestLiveHole`: última mensagem do detector com
  até 2 s, posterior à última mudança de plano, a até 5 cm; PLACE:258-259, 4667-4680, 4776-4781,
  conferido). Visto de longe, onde o detector funciona melhor (4 furos estáveis em 13/11), não é
  usado.
- Alvo xy = 0,3 × (tag + nominal) + 0,7 × buraco (`slot_hole_weight 0.7`, PLACE:276-279).
- Sem buraco: mediana do vetor (buraco − tag) aprendido (`TagHoleOffsetLearner`, só em RAM,
  nunca zerado enquanto o nó vive; PREC:351-408). Sem isso: tag + nominal.
- As justificativas escritas no código ("o erro de escala entra igual nos dois e se cancela";
  "peso 0,7 porque o detector também erra") não vêm com medição (`place_precision.hpp:292-300`).

---

## 5. Transformações existentes

| Transformação | Valor | Origem | Calibrada? |
|---|---|---|---|
| `base_footprint → base_link → manip_base_link` | xyz 0,217 0 0,083; rpy 0 0 −1,57 | `HW/caramelo_description/urdf/robots/robot.urdf.xacro:10-11` (args) e `:29-34` (instância); o default da macro em `mech/caramelo_manipulator.xacro:12` é `0 0 0` e a marca `AJUSTAR_NO_ROBO` fica em `:16`; `base_footprint → base_link` em `mech/mobile_base.xacro:113-117` (conferido) | Não. −1,57 em vez de −π/2 (0,8 mrad). |
| `manip_base_link → link1..link5` | j1 em z; j2, j3, j4 em y (z = 0,09775 / 0,185 / 0,205); j5 em z (xyz −0,03 0 0,07815) | ARMX:268-354 + encoders, publicado pelo `robot_state_publisher` da Raspberry | Comprimentos nominais. Zero = tick 2048 em todas as juntas, sem offset por junta no software (DRV). |
| `link5 → tcp` / `tcp2` | z = 0,17 / 0,30 | ARMX:332-336, 350-354 (conferido) | Não. Pelo STL do dedo a ponta fica a 0,157-0,160 m: o `tcp` está 10-13 mm além dela. |
| `link5 → camera_bottom_screw_frame → … → camera_color_optical_frame` | Montagem xyz 0 −0,055 0,0125, rpy 0 −π/2 −π/2; composto (−0,0115; −0,0405; +0,0237) m com rotação Rz(180°) exata | ARMX:254-264 (conferido); `_d455.urdf.xacro` com `use_nominal_extrinsics=true`; driver com `publish_tf=false` | Não. Eixo óptico modelado como exatamente paralelo ao eixo da ferramenta. |
| `camera_color_optical_frame → tag_N` | PnP por quadro | `apriltag_ros` | Intrínsecos de fábrica; `size` em disputa. |
| `manip_base_link → pp_buraco_i` | ray-cast no plano da tag | HOLE | Herda toda a cadeia acima. |
| Modelo da IK própria (`ArmModel`) | Cópia manual dos comprimentos da URDF | IK:42-69 | Duplicado: calibrar a URDF sem atualizar este arquivo faria leitura e movimento usarem geometrias diferentes. |

**Estado da calibração:**

- **Mão-olho:** existem `manip_bringup/scripts/calib_camera_record.py` e `calib_camera_solve.py`
  (6 graus de liberdade de montagem rígida + 1 ponto 3D por tag). Em 09/11, com 129 amostras, o
  ajuste rígido não fechou (rms 35 → 32 mm); com um fator de escala caiu para 13,7-17 mm.
  **Nada foi aplicado à URDF** e os dados (`/tmp/calib.jsonl`) não existem mais. O gravador não
  salva `/joint_states` e o modelo não estima escala nem zeros de junta.
- **Erro angular já medido e nunca corrigido:** com o braço em home, o chão projetado pela TF da
  câmera sobe 5,45° em pitch (`caramelo_navigation/scripts/caramelo_dock_camera.py:23-27`). O
  docking contorna ajustando o plano na própria profundidade; a cadeia usada por pick/place
  continua igual.
- **Intrínsecos:** os de fábrica. Não há calibração própria.
- **Zeros de junta, ganhos dos servos, `Homing Offset`:** não escritos pelo driver; valores
  reais na EEPROM/RAM não foram lidos.

**Tolerâncias do lado MoveIt/IK** (valem para o plano, não para a chegada real):

- o place não define tolerância de goal no `MoveGroupInterface` (nenhum `setGoal*Tolerance` em
  PLACE, conferido; vale o default; o pick define, PICK:2773-2774);
- a IK própria aceita solução com até 0,5 mm de erro de posição (`accept_position_error`,
  IK:69, conferido);
- `pick_ik`: `position_threshold` 1 mm e `orientation_threshold` 0,08 rad (4,6°)
  (`manip_moveit_config/config/kinematics.yaml`, conferido).

O plano fecha em 0,5-1 mm; a execução é aceita com até 0,30 rad por junta (P01).

Convenção: em `manip_base_link`, +x é a direita e +y a frente do robô (PLACE:121-125).

---

## 6. Como o cubo é localizado depois de agarrado

**Não é.** Não existe estimativa da transformação garra→cubo em nenhum ponto do código.

- **Antes da pega na mesa:** o cubo é localizado só pela AprilTag dele; o TCP vai ao centro da
  tag com offset z = 0 e o j5 alinha a linha dos dedos ao yaw da tag módulo 90°
  (PICK:1840-1855, 2396-2404).
- **Depois do pick:** o cubo é solto num container de bordo em pose nomeada do SRDF
  (`container1..3`; PICK:2915-2924).
- **No place:** é re-pego indo à mesma pose nomeada e fechando a garra, **sem visão**; a
  conferência de esforço nessa pega está desligada (`place_verify_container_grasp=false`;
  PLACE:427-428, 7542-7566).
- **Suposição fixa do place:** TCP no plano da face de cima do cubo, centro do cubo no eixo da
  ferramenta, faces paralelas aos dedos, cubo de 42 mm (PLACE:4607-4623, 4843-4856).

O que o bag mostra (frente cubo/histórico):

- A tag do cubo carregado **está no campo da câmera** (em ~(274, 96) px, 137 px de lado), mas
  **não foi detectada em 8 de 10 ciclos**. A causa (oclusão pelos dedos, desfoque a 9-12 cm,
  `decimate 2.0`) não foi determinada. Em 14/11 foram 0 detecções em 82 quadros com o cubo na
  garra (2 tentativas); no quadro da descida final a tag aparece cortada pela borda de cima da
  imagem (seção 18). HIPÓTESE: é essa a causa nas poses de soltura; outras poses não testadas.
- A tag do **slot** pode ser detectada com o cubo na garra, mas não sempre. No place de 13/11
  (bag RUN) o id 104 aparece em 10 mensagens de `/detections` de +952,0 a +954,4 s e em 5
  quadros seguidos com o braço parado na pose final (+954,6 a +955,4 s). Isso é n = 1, slot
  central (x = −0,016); o slot 105 nunca chegou à pré-pose (LOG:16611, 17485). Em 14/11 a tag
  104 foi vista em 5 de 7 quadros da pré-pose e em **nenhum** da descida final: o cubo a tapou
  (seção 18). **A recomendação da seção 14 não se apoia mais só nessa tag.**
- Ao fechar no container a garra desloca o cubo 9-14 px (2-3 mm) e o alinha a 1-2° do eixo. Os
  dedos centram só na direção de fechamento; a direção perpendicular e a altura ficam livres.
- As poses de pega nos containers não são verticais: ferramenta a 6°, 11° e 9° da vertical em
  `container1/2/3` (`manip_moveit_config/config/caramelo.srdf:127-147`). HIPÓTESE: o cubo sai
  inclinado; a 10° um cubo de 42 mm ocupa ~48,7 mm na horizontal. Depende do fundo do container.

---

## 7. Dependências e versões confirmadas

| Item | Versão / valor | Evidência |
|---|---|---|
| ROS 2 | Jazzy | ambiente |
| `ros-jazzy-apriltag` | 3.4.5 | `dpkg -l` (conferido) |
| `ros-jazzy-apriltag-ros` | 3.4.0 (binário apt; sem overlay) | `dpkg -l` (conferido) |
| `ros-jazzy-apriltag-msgs` | 2.0.2 | `dpkg -l` (conferido) |
| `ros-jazzy-realsense2-camera` / `librealsense2` | 4.58.4 / 2.58.4 | `dpkg -l` (conferido) |
| `ros-jazzy-image-proc` | 5.0.13 (instalado, não usado) | `dpkg -l` (conferido) |
| `ros-jazzy-pick-ik` | 1.1.2 | `dpkg -l` (conferido) |
| MoveIt 2 | 2.12.4 | frente TF |
| OpenCV | 4.6.0 | `dpkg -l` (conferido) |
| Câmera | Intel RealSense D455 (`8086:0b5c`), FW 5.17.0.9, em USB 3.2 | `lsusb`; log do driver da corrida |
| Perfil da câmera na corrida | cor RGB8 e profundidade Z16, 640x480 a 5 fps (default pensado para USB 2) | LAUNCH:169-176; log do driver |
| Modos disponíveis em USB 3.2 | cor 640x480 a 60/30/15/5 Hz; 1280x720 a 30/15/10/5 Hz | `rs-enumerate-devices -c` |
| Intrínsecos de fábrica da cor (640x480) | fx 386,39, fy 385,79, ppx 324,35, ppy 239,72; Inverse Brown Conrady, k1 = −0,056 | `SCR/pp_apriltag_camera/rs_enumerate_devices_c.txt:252-261` (conferido) |
| Servos | ids 1-5 XM540-W270; 6-7 XM430-W350; 4096 pulsos/volta; folga de catálogo 0,25° | memória do projeto; e-manual Robotis |
| Regulamento | cavidade = objeto + 2 mm por dimensão; cubo 42 mm; tag de 40 mm "incluindo 1 bit de borda preta e 1 bit de borda branca"; tile de PP com tag a offset constante do furo | `SCR/pp_pesquisa_calib/cbr_rulebook_2025.txt:414-415, 496-509` (conferido) |

---

## 8. Problemas identificados

Impacto em mm no plano xy, salvo indicação. "n.m." = não medido.

### A. Execução e controle do braço

| ID | Problema | Evidência | Impacto |
|---|---|---|---|
| P01 | **Erro de regime dos servos sob gravidade, sem compensação.** Na soltura da PP as juntas param fora do comando (j2 37 mrad, j3 9,6 mrad) e ficam lá, igual antes e depois de abrir a garra (não é contato). A espera só exige parar; o JTC aceita 0,30 rad; o "erro de FK na chegada" é informativo. | LOG:15611, 15746 + `/joint_states` do bag (3 frentes, FK própria); PLACE:134-135, 4276-4291; CTRL:86-89, 107-111; SETTLE:19-22 (conferido) | FATO: 6,6-7,0 mm em y, 15 mm em z, ferramenta a 2,7° (1 soltura). Faixa de 3,4 a 9,1 mm radial em 4 casos. Em 92 solturas de PP: mediana 1,8 cm (norma 3D). Em 76 trechos parados da corrida: xy mediana 4,2 mm, p90 6,6 mm. |
| P02 | Descida final em espaço de juntas, velocidade e aceleração 1.0 (limites 3,0 rad/s e 8,0 rad/s²), sem trecho cartesiano lento. | PLACE:4241-4252, 7032-7035 | n.m. Agrava P01; pode empurrar o cubo na entrada. |
| P03 | Piso do atuador em malha aberta seria da ordem da folga. É soma de folga de catálogo, não medida; sob gravidade a folga de j2-j4 tende a ficar encostada de um lado (repetível). | DRV; catálogo XM540 | ESTIMATIVA no alcance da PP (0,36 m): 1 tick = 0,4-0,6 mm por junta (RSS 1,0 mm); folga de catálogo = 1,1-1,6 mm por junta (RSS 2,9 mm). |
| P04 | Driver converte com 651,0886 em vez de 4096/2π = 651,8986 (0,124 %) e trunca o comando em vez de arredondar. | DRV:30, 179 (conferido) | ESTIMATIVA: ~1,0-1,3 mm no TCP; até 1 tick de viés por junta. |
| P05 | Limites reais dos servos menores que os do modelo: j4 para em 1,913 rad (URDF, SRDF e IK usam 2,2); j3 em ~2,17 (URDF 2,6). | ARMX:290-307; IK:57-58; `.jarvis/estado.md` T9 | 4,5 mm quando corta 18 mrad (visto em picks). Não ocorreu na soltura de PP de 13/11. **Ocorre em toda leitura de longe:** a pose `pegar_obj` pede j4 = 2,19911 (`manip_moveit_config/config/caramelo.srdf:67-73`, conferido) e o servo para em 1,9168 (bag RUN, `/joint_states` em +934,3 a +935,0 s): 0,282 rad (16,2°) a menos. A ferramenta fica a ~20° da vertical em vez dos ~6° planejados. A TF usa o encoder, então a cadeia continua coerente, mas a leitura de longe é sempre oblíqua, o que pesa em P06 e P07. |

### B. Percepção por AprilTag

| ID | Problema | Evidência | Impacto |
|---|---|---|---|
| P06 | **A leitura da tag depende da pose do braço.** | LOG:15146, 15195, 15196, 16177, 17059 (conferido); 97 tentativas nos logs antigos | FATO: mediana 43 mm em xy e 43 mm em z entre longe e perto. Slots vizinhos 104 e 105 saíram com alvos 24 mm desalinhados em y. A leitura de longe é feita com a ferramenta a ~20° da vertical (corte do j4, P05), o que pesa nessa discrepância. |
| P06a | Causa candidata de P06, **em disputa** (10.1): `size` 0.041 é o lado do adesivo; o apriltag mede pelo quadrado preto (~32-33 mm). | TAGS:32-42, 60-66 (conferido); 112 pares de vistas paradas: discrepância 50,3 mm rms cai para 18,5 mm com fator 0,803; triangulação independente do `size` dá lado de 31,2 / 31,8 / 33,0 mm; solver de 09/11 achou 0,806; regulamento: tag de 40 mm com bordas → 32 mm | Em disputa (10.1). A escala 0,80 reduz a discrepância entre vistas, mas põe o tampo em ~0,00, cerca de 8 cm acima da altura física (−0,079), e este documento não explica esse erro. Indício só de imagem a favor de 32 mm: a abertura de 44,2-45,4 mm por homografia só bate com a regra (42 + 2 mm) se o quadrado preto tiver 32 mm; com 41 mm daria ~57 mm. Um fator de escala uniforme não satisfaz as duas coisas; T1 vem antes de qualquer conclusão. ESTIMATIVA, se a hipótese de escala valer: erro lateral = ~20-28 % do afastamento da tag ao eixo óptico: ~7 mm com a tag a 47 px do centro, 22-33 mm a 150-200 px. Previsto no slot em 13/11: ~8 mm (104) e ~21 mm (105). |
| P07 | Resíduo dependente da pose mesmo com a escala corrigida, e todas as tags horizontais aparecem inclinadas ~8° para −y em `manip_base_link`. Um erro rígido câmera↔link5 não explica (ajuste dá menos de 0,4°). | `SCR/pp_apriltag_camera/extr.py`, `tilt.py` (98 segmentos); `caramelo_dock_camera.py:23-27` (5,45° em home) | FATO: 18,5 mm rms entre vistas; ~11 mm na sequência longe → perto. Causa (zero ou folga de junta de pitch, base do braço desnivelada, flexão) é HIPÓTESE. |
| P08 | Diagnóstico errado gravado no YAML: diz que `fx = 386` seria a focal da profundidade e que a cor teria ~470-495. | TAGS:37-42 × `rs_enumerate_devices_c.txt:252-261` (conferido: `fx` da cor = 386,39; profundidade = 390,67) | Indireto: quem seguir o comentário procura no lugar errado. Não há `camera_info` a corrigir. |
| P09 | Câmera a 5 fps, embora esteja em USB 3.2, e 17 % dos intervalos de `/detections` com quadro perdido. A origem da perda (câmera, ou `apriltag_node` com `nice +15` e QoS best-effort) não foi determinada. Quando faltam quadros na janela de 700 ms, a mediana cai para leitura única. | LAUNCH:169-176, 574, 587; PLACE:142-145, 1678-1687, 1797-1806; LOG:15146, 15193-15195 (conferido); 16 logs antigos; LOG14:570, 608, 1304, 1867 (conferido) | A mediana funcionou em 70 de 71 leituras de perto nos logs antigos; caiu para leitura única nas 3 leituras de perto de 13/11 e em 4 de 6 leituras em 14/11. A leitura única não rejeita outlier: prejuízo típico pequeno (repete em menos de 1 mm), mas herda inteiro qualquer flip ou vibração. As "leituras distintas" são stamps de TF, não quadros (4.1), então mesmo a mediana tem menos amostras independentes do que o log diz. Subir o fps não resolve por si; falta explicar a perda de quadros. |
| P10 | Flips de ambiguidade do PnP em tags pequenas e fora do centro; o nó usa `solvePnP` default sem conferir a segunda solução. | 63 de 985 leituras de tags fixas (6,4 %); `tag_102` em 39 % (yaw salta de +1,3° para ~+42°) | Raro, mas grosseiro: se a leitura única cair num flip, o yaw do slot sai dezenas de graus errado. |
| P11 | Yaw de uma tag fixa muda até 3,6° entre vistas paradas (ruído dentro da vista: 0,04°). | `SCR/pp_apriltag_camera/extr.py` | ESTIMATIVA: ~1,8 mm no canto de um cubo de ~50 mm a 3°. |
| P12 | Imagem crua, distorção não corrigida no PnP (o detector de furos corrige; o apriltag não). | LAUNCH:582-583 (conferido) | ESTIMATIVA: lateral 0,01-0,16 mm na mediana (p90 ~1,2-2,4 mm); profundidade até +6,5-7 mm na periferia. Secundário. |
| P13 | Leitura com a câmera ainda oscilando: 5 tags se movem juntas ~4 mm pico a pico com os encoders indicando braço parado. | `SCR/pp_apriltag_camera/analyze.py` (t = 926,9 s) | Até ~2 mm numa leitura única. A espera por encoder não enxerga flexão. |

### C. Detector de furos e fusão

| ID | Problema | Evidência | Impacto |
|---|---|---|---|
| P14 | **O detector não atua no caso real.** Em 13/11: `tag_104` "não segmentou nada"; `tag_105` (2 vezes) só viu o furo do vizinho a 140 mm. Causas reproduzidas offline: limiar de branco marginal (tampo sob a sombra do braço com V = 119-138 contra `white_v_min 140`); furo cortado pela máscara dos dedos porque a pré-pose de perto é mirada pela leitura de longe; cavidade com conteúdo vermelho/branco fora da faixa marrom. | LOG:15199 (conferido), 16183; `/tmp/pp_hole_debug/20261113_2119*`, `20261113_2120*` | Cai em tag + offset fixo, sem aviso além de um WARN. Histórico: visto em 35/39 (09/11), 19/19 (12/11), 0/3 (13/11), 2/2 (14/11). Em 14/11 foi visto e fundido, e o cubo ficou fora do furo assim mesmo (seção 18). |
| P15 | A conversão métrica do furo herda a escala e o plano da tag. O nó mede tag→buraco = 61,0 mm (mediana, n=58) e passo entre furos de 127-129 mm; pela homografia no plano da tag (lado 32 mm) são 46,4 mm e 106-110 mm. O passo muda de 110 para 128 mm só trocando o plano (−0,029 → −0,084). | `~/.ros/log/mtc_place_action_node_*.log`; `/tmp/pp_hole_debug/log.txt:193, 212-213, 338`; `SCR/pp_detector_furos/homog.py` | FATO (inconsistência de 10-30 %). ESTIMATIVA: 1,3 mm de xy por 5 mm de erro de plano a 100 px do centro; 6,5 mm com 25 mm de erro. |
| P16 | O centro detectado não é o centro da abertura: a máscara pega só o fundo visível; a parede/sombra fica fora. Segmentação a 320x240. | 5 quadros de 12/11 medidos por homografia: mancha de 40,0-43,6 mm contra abertura de 44,2-45,4 mm | ESTIMATIVA: 1,3-2,1 mm em direção ao centro da imagem para furos a 150-190 px; ~0 no centro. Quantização ~0,6-0,75 mm. |
| P17 | Fusão por peso fixo 0,7 de duas medidas da mesma pose de câmera, com viés comum. | PLACE:276-279, 4782-4797 | FATO: em 12/11, com fusão em todos os slots, resíduo lido mediana 14,4 mm (3 de 11 dentro de 12 mm). O ganho sobre "só tag" não está medido. Em 14/11, com fusão (alvo a −1,9/−5,8 mm do nominal), resíduo lido de 19,1 mm e cubo fisicamente fora do furo. |
| P18 | Yaw do furo não é estimado (o ângulo do `minAreaRect` é descartado). | HOLE:178, 747-749 | ESTIMATIVA: cubo de ~41,5 mm em abertura de ~45 mm tolera ~4,8° com centro perfeito; ~2° com 1 mm de erro de centro. |

### D. Geometria e offsets

| ID | Problema | Evidência | Impacto |
|---|---|---|---|
| P19 | Offset tag→furo fixo (0; −52 mm) somado nos eixos da base, sem girar pelo yaw da tag ou da mesa; um valor para os 7 slots; os −3 mm em x medidos em 09/11 não foram aplicados; o offset aprendido mistura docks e slots e vive só em RAM. | PLACE:186-203, 3940-3943 (conferido), 4806-4809 | 0,9 mm por grau de desalinhamento (2,7 mm a 3°). HIPÓTESE (depende da disputa do `size`, 10.1): o valor está na métrica inflada; a distância real seria 40-46 mm, o que dá 6-12 mm de erro em y se o vetor for executado em metros verdadeiros. Em 14/11 o detector mediu tag→buraco = −60,3 mm em y (LOG14:1309). |
| P20 | O frame `tcp` não é um ponto físico conferido e a cadeia em z fecha por ajuste: z do tampo sai −0,083 (longe), −0,059 (perto), ~0,00 (triangulado/escala corrigida); a memória espera −0,079. | ARMX:332-336; STL do dedo; PLACE:4609-4622 | Altura real de soltura indeterminada (entre ~0 e ~4 cm). Em 13/11 o erro de z da leitura e a queda do braço se cancelaram por acaso. |
| P21 | Modelo cinemático duplicado (URDF de `caramelo_description`, `manip_description` e `ArmModel` da IK). | IK:42-69 | 0 mm hoje; vira erro direto na primeira calibração aplicada só de um lado. |

### E. Cubo na garra

| ID | Problema | Evidência | Impacto |
|---|---|---|---|
| P22 | Nenhuma estimativa da pose do cubo na garra; re-pega cega; ferramenta a 6-11° nos containers; tag do cubo carregado quase nunca detectada (0 de 82 quadros em 14/11). | PLACE:7542-7566, 4607-4623; `caramelo.srdf:127-147`; bag; RUN14 | n.m. (2-3 mm observados na direção de fechamento). Em 14/11 a aresta frontal e os cantos inferiores do cubo ficam visíveis com ele na garra: dá para medir o cubo pelas arestas, sem a tag (seção 18). |

### F. Fluxo e confiabilidade

| ID | Problema | Evidência | Impacto |
|---|---|---|---|
| P23 | Malha aberta do agarre à soltura: ~17 s e uma ida ao container entre a leitura e a soltura, sem reobservar, embora haja o que observar com o cubo na garra (tag do slot na pré-pose; borda do furo-alvo e slots vizinhos na descida; seção 18). | PLACE:3835-3845, 4198-4201; LOG:15195 → 15746 | Todo erro passa inteiro. |
| P24 | Sem verificação de assento nem nova tentativa; sucesso declarado ao abrir a garra. O modo com checagem, re-pega e espiral está desligado. | PLACE:201, 329-330, 5884-5891 (conferido); LOG:15886, 15975 | Cubo que não entrou fica como está e a missão conta como entregue. Aconteceu em 14/11: `[REF] decisao da verificacao: repetir`, depois "Entreguei o bloco" e árvore em SUCESSO, com o cubo fora do furo (LOG14:1870-1871, 1945, 2017, conferido). |
| P25 | Alcance: a PP só desce a 0°, mas o cálculo de "fora de alcance" usa a escada da pilha (0/15/30°) e responde "resolve sem mover a base"; a IK do alvo não é testada de garra vazia, antes de buscar o cubo; o abort derruba a árvore. | PLACE:4127-4131 × 6509-6513 (conferido); BT:1565-1566; LOG:16611, 17485, 17728 | Falhas por alcance: 1 de 39 (09/11), 4 de 19 (12/11), 2 de 3 (13/11). Em 13/11 ficaram ações de outras estações por fazer. |

### G. Observabilidade e documentação

| ID | Problema | Evidência | Impacto |
|---|---|---|---|
| P26 | O bag de corrida exclui todo `/camera/*`, inclusive `camera_info`; o log do driver RealSense não é copiado para o run; não existe gabarito externo do encaixe. | `RUN/metadata.json` (exclude-regex); `caramelo_navigation/scripts/caramelo_bag_common.py` | Não dá para auditar pelos runs os intrínsecos e o perfil de cada corrida, nem medir taxa de sucesso. |
| P27 | Premissas contraditórias: tag "no fundo da cavidade" × "no tampo"; folga "~2 mm" × "~3,5 mm por lado"; `slot_cavity_depth_m` 0.005 no place × 0.0005 no AMT1; `hole_side_m 0.05` × abertura medida de 44-45 mm; `docs/place_precisao_e_pilha.md` descreve o modo legado e diz "não validado no robô". | PLACE:160-163, 176-184, 199; `amt1_sort_action_node.cpp:197`; HOLEY:75 | Risco de ajustar parâmetro pela premissa errada. |
| P28 | `save_debug_dir` ligado grava ~2 PNG/s em `/tmp` durante cada place. | HOLEY:33 (conferido); T2 do `.jarvis` | Operacional (disco/CPU). |

---

## 9. Orçamento de erro

**Tudo aqui é estimativa, exceto onde está escrito "medido".** As linhas não são independentes
(a escala da tag contamina o offset, o plano do detector e a altura) e algumas se cancelam em
geometrias específicas, então **não some nem tire a raiz da soma dos quadrados**. A tabela serve
para decidir o que medir primeiro.

| Fonte de erro | Estimativa no plano xy | Base | Como medir |
|---|---|---|---|
| Escala da pose da tag (`size`) | 20-28 % do vetor câmera→tag: ~7 mm com a tag a 47 px do centro; 22-33 mm a 150-200 px | Em disputa, 10.1 (112 pares de vistas; triangulação) | Régua entre os centros de duas tags vizinhas (~110 mm ou ~140 mm); profundidade alinhada no pixel da tag a 0,35 m ou mais; paquímetro no quadrado preto |
| Erro angular e residual da cadeia (pitch ~6-8°, mão-olho, zeros de junta) | ~18 mm rms entre vistas depois de corrigir a escala; ~11 mm entre longe e perto | Medido (resíduo); causa é HIPÓTESE | Inclinômetro no link5 na pose de leitura; calibração que grave `/joint_states` e estime escala + zeros + mão-olho juntos |
| Queda do braço em regime | 6,6-7,0 mm (faixa 3,4-9,1); z −15 mm; inclinação 2,7° | Medido (1 soltura com juntas; 76 trechos parados; norma em 92 solturas) | `/joint_states` × comando em N repetições no mesmo alvo, com e sem cubo, em 3 slots |
| Resolução e folga dos servos | RSS 1,0 mm (1 tick por junta); RSS 2,9 mm (folga de catálogo) | ESTIMATIVA derivada de catálogo e jacobiano, não medida; sob gravidade a folga de j2-j4 tende a ficar encostada de um lado (repetível), então o efeito real pode ser bem menor | Relógio comparador; aproximar o mesmo ponto pelos dois sentidos |
| Offset tag→furo: direção (eixos da base) | 0,9 mm por grau de yaw do docking (3,6 mm a 4°) | Derivado | Yaw do dock em relação à mesa (régua ou yaw das tags) |
| Offset tag→furo: valor (−52 mm contra 40-46 mm reais) | 6-12 mm em y | HIPÓTESE | Régua do centro da tag ao centro da cavidade, por slot |
| Offset em x não aplicado | 3 mm | Medido em 09/11 (27 leituras), na métrica da câmera | Idem |
| Colagem de cada adesivo (posição e giro) | desconhecido (ordem de mm) | n.m. | Homografia tag→furo por slot, ou régua |
| Centro do furo pelo detector | viés 1,3-2,1 mm; quantização 0,6-0,75 mm | Medido offline em 5 quadros | Reprocessar as imagens de `/tmp/pp_hole_debug` com borda em resolução cheia |
| Plano do ray-cast do detector | 1,3 mm por 5 mm de erro de z a 100 px (6,5 mm com 25 mm) | Derivado | Altura real do tampo em `manip_base_link` |
| Leitura única, vibração, flip | ruído 0,15-0,25 mm; vibração até ~2 mm; flip em 6,4 % das leituras de tags fixas | Medido | Achar a origem da perda de quadros em `/detections`; teste de normal fora da vertical |
| Yaw da tag entre vistas | até 3,6° → ~1,8 mm no canto do cubo | Medido (2 tags) | Yaw por várias tags (reta pelos centros) |
| Distorção ignorada no PnP | lateral 0,01-0,16 mm (p90 ~1,2-2,4); profundidade até 6,5-7 mm na periferia | Estimado offline | `camera_info` real; PnP com cantos desdistorcidos |
| Cubo na garra (posição) | 2-3 mm na direção de fechamento; perpendicular n.m. | Observado (3 amostras) | Posição em pixel da tag do cubo após 10 re-pegas por container |
| Cubo na garra (inclinação) | até +6,7 mm de ocupação horizontal a 10° | HIPÓTESE | Foto lateral do cubo na garra; geometria do container |
| Altura de soltura | 0 a ~4 cm acima do tampo | HIPÓTESE (CAD × leituras) | Folga entre o fundo do cubo e o tampo com o braço parado na pose de soltura |
| Constante e truncamento do driver | ~1,0-1,3 mm; até 1 tick por junta | Derivado | Não precisa medir; corrigir |
| Corte do j4 no limite do servo | 4,5 mm quando ocorre na pega ou na soltura. Na leitura de longe ocorre sempre (16,2°): sem erro direto de TCP, mas com a câmera oblíqua (P05) | Medido em picks e no bag RUN | Conferir j4 comandado × limite nas poses da PP |
| Inclinação do chassi com o braço estendido | não avaliado | — | Inclinômetro no chassi |

**Fechamento:** o resíduo ponta a ponta lido pelo próprio robô é mediana 14-21 mm (leitura com
viés), e em 14/11 o cubo ficou fisicamente fora do furo (seção 18). A folga é de 1 a 2 mm por
lado. ESTIMATIVA: mesmo com tudo calibrado, o piso do atuador em malha aberta seria ~3 mm RSS,
acima da folga. Esse número é soma de folga de catálogo, não medida, e sob gravidade a folga de
j2-j4 tende a ser repetível. **O que está medido é que hoje o posicionamento absoluto em malha
aberta não fecha a conta. Se ele fecharia com tudo calibrado, só o teste T2 (aproximação pelos
dois sentidos) diz.**

---

## 10. Limitações atuais

### 10.1 Pontos em que as frentes divergem ou os dados não decidem

1. **Tamanho da tag.** A favor de ~0,032-0,033: régua do operador (quadrado preto de 32 mm);
   definição oficial do `size` (lado entre as bordas preto/branco); regulamento (tag de 40 mm
   incluindo bordas → 32 mm); ajuste de escala em 112 pares de vistas (0,803); triangulação
   independente do `size` (31-33 mm); solver de 09/11 (0,806). A favor de 0,041: a decisão
   vigente de 09/11 (0.032 quebrou todos os picks, alvo 4,5 cm alto); a leitura de longe dá
   z = −0,083, igual ao esperado para mesa de 10 cm (−0,079); e — ponto levantado só nesta
   síntese — com 0.041 as tags vizinhas ficam a 139,5-146,6 mm, igual ao tile oficial de
   140 x 140 mm do regulamento de 2023, enquanto com 0.032 ficariam a 109-114 mm. Todos os
   indícios numéricos a favor de 0,032 passam pela cinemática do braço; os físicos (régua,
   regulamento) não, com uma exceção que é só de imagem: a abertura da cavidade medida por
   homografia (44,2-45,4 mm) só bate com a regra (42 + 2 mm) se o quadrado preto tiver 32 mm;
   com 41 mm daria ~57 mm. Contra 0,032 pesa um erro que este documento não explica: com a
   escala corrigida o tampo sai em ~0,00, cerca de 8 cm acima da altura física (−0,079: mesa de
   10 cm medida pelo operador, memória `projeto-caramelo-tag-size-causa-raiz.md:16-17`). Um
   fator de escala uniforme não satisfaz as duas coisas. **A régua entre duas tags vizinhas e o
   teste T1 (profundidade alinhada contra a distância da tag) vêm antes de qualquer conclusão
   sobre a causa.**
2. **Distância real tag→furo:** 52 mm (código), 61,0 mm (detector, n=58), 46,4 mm (homografia
   com tag de 32 mm), 40,6 mm (52/1,281), "uns 4 cm" (operador, 04/11). Não há medida de régua.
3. **Folga da cavidade:** 2 mm por lado (doc e decisão de 21/10), 3,5 mm por lado (comentário do
   código), "objeto + 2 mm por dimensão" (regulamento), abertura de 44,2-45,4 mm (homografia).
4. **Altura do tampo em `manip_base_link`:** −0,079 (memória), −0,083 (longe), −0,059 (perto),
   ~0,00 a +0,005 (triangulado / escala corrigida).
5. **Padrão "dy +10 mm à esquerda, −15 mm à direita"** (base da hipótese de giro de ~3° do T1):
   uma frente mostrou que em 12/11, só a 0°, a direita deu dy +10,4 mm (n=9). O sinal acompanha
   a inclinação de 15/30° usada nas laterais em 09/11, não o lado. Aprender resíduo por lado
   ajustaria um artefato.
6. **Resíduo depois de corrigir a escala:** 18,5 mm rms (112 pares) numa frente; 8 mm para a
   `tag_104` e 30 mm no pior dos 4 conjuntos em outra. Amostras diferentes; mesma ordem.
7. **Distância mínima da profundidade em 640x480:** 0,29 m (cálculo de uma frente), 0,32 m
   (datasheet citado pela pesquisa), 0,45-0,5 m (config e `.jarvis`). Não medido.
8. **Erro de j2:** as frentes usam convenções de sinal opostas (real − comando = +37 mrad) e dão
   dy de −6,6 a −7,0 mm. É o mesmo fato.
9. **Critério de sucesso não definido.** O regulamento de 2023 pontua "Cavity Placement" com o
   objeto sobre o tile certo e, a mais, "Precision Placement" quando ele atravessa a cavidade
   (`SCR/pp_pesquisa_calib/txt/rulebook2023.txt:1936-1940`, conferido). A mesa do laboratório
   tem cavidade rasa, com fundo. Sem saber qual critério vale em 18/11 não dá para dizer quanto
   erro é aceitável (pergunta 5 da seção 17).

### 10.2 Limitações do sistema

- Não existe campo nem modo próprio de PP na action; o slot é tratado como uma pilha.
- Não existe realimentação depois da pega, nem critério de sucesso, nem nova tentativa.
- Não existe medida do cubo na garra.
- A cadeia geométrica é 100 % nominal e depende de constantes empíricas que se compensam (tag
  0.041, −52 mm, +47 mm, `tcp` 0,17).
- HIPÓTESE não medida: a profundidade da D455 não serviria na distância da leitura de perto
  (item 7 de 10.1; no código só há o comentário "não serve a < 0,4 m", PLACE:4602).
- Braço de 5 graus de liberdade com servos em malha de posição: sem controle de força nem
  complacência; movimentos ponto a ponto pelo MoveIt, sem interface de velocidade.
- Os parâmetros da PP só existem como defaults no código.

### 10.3 Limitações desta auditoria

- Uma corrida em bag na auditoria, mais a de 14/11 (seção 18); estatística fraca para tudo que
  depende de `/joint_states` e `/tf`.
- Nenhuma medida de régua foi feita. O único uso do robô foi o teste de 14/11: 1 soltura, sem
  foto de cima com escala.
- Quase toda a evidência quantitativa estava em diretórios temporários. Em 14/11 ela foi
  copiada para `runs/pp_auditoria_20261114/`; `/tmp/pp_hole_debug` (643 MB) não foi copiado, por
  decisão do operador, e some no próximo reboot.
- EEPROM/RAM dos servos, arquivos que rodam de fato na Raspberry, `camera_info` publicado e
  iluminação/exposição não foram vistos.
- `amt1_sort_action_node.cpp` (4483 linhas) e `placeIntoPrecisionSlot` não foram lidos por
  inteiro. No AMT1 o mesmo offset aparece multiplicado por uma rotação
  (`amt1_sort_action_node.cpp:1545`); não auditado.
- pytest e gtest não foram rodados.
- Uma das duas frentes de pesquisa chegou truncada.

---

## 11. Respostas diretas às 10 perguntas do operador

**1. Como funciona a PP hoje?**
O slot é tratado como uma pilha. O robô abre a garra, olha a tag do slot de longe e de perto
(garra vazia), guarda essa leitura, vai buscar o cubo no container de bordo por pose nomeada,
volta e solta em tag + (0; −52 mm), 47 mm acima do z da tag, descendo na vertical em espaço de
juntas. Do agarre à soltura não há nenhuma medição; a verificação depois só registra; o sucesso
é declarado ao abrir a garra.

**2. Quais nós?**
`task_planner` e `bt_yaml_executor` (`manip_bt`); `mtc_place_action_node` (`/place_tag`),
`mtc_pick_action_node` (`/pick_tag`) e `amt1_sort_action_node` (`/amt1_sort`) em
`manip_task_execution`; `move_group`; `realsense2_camera`; `apriltag_node`;
`pp_hole_detector_node.py`; na Raspberry, `robot_state_publisher` e o `arm_controller`.
Detalhe na seção 2.

**3. Como as AprilTags são usadas?**
Tags 36h11. A do cubo dá a posição de pega na mesa. As 101-107 (uma por slot) dão a posição, o
z e o yaw do alvo; a 200 é a referência da pilha. A pose vem de PnP nos 4 cantos da imagem RGB
crua, com `size` 0.041 e intrínsecos de fábrica, publicada só como TF. O place lê a TF em
`manip_base_link` e usa a mediana de 5 ou mais leituras em 700 ms; quando faltam quadros na
janela, cai para uma leitura só (1 de 71 leituras de perto nos logs antigos; 3 de 3 em 13/11;
4 de 6 leituras em 14/11).

**4. Como os furos são detectados?**
Por cor: máscara HSV marrom em 320x240, contornos, filtros de forma e "anel branco". O centro é
o do `minAreaRect` em pixel, projetado por ray-cast no plano horizontal cuja altura é o z da tag
lida de perto. Não usa profundidade, bordas nem homografia, e não estima o yaw. Na corrida
gravada em 13/11 não detectou o furo alvo em nenhuma das 3 tentativas; em 14/11 detectou nas 2
e o cubo ficou fora do furo assim mesmo.

**5. Como a pose de destino é calculada?**
xy = tag (leitura de perto) + (0; −0,052 m) nos eixos do robô, fundido com o buraco (peso 0,7)
quando visto de garra vazia ou, sem isso, "ao vivo" (mensagem do detector com até 2 s), ou com
o offset aprendido; z = z da tag + 0,042 + 0,005; yaw = yaw da tag módulo 90° (o quarto de
volta de menor giro do punho; o yaw lido entra inteiro, sem arredondar). A IK própria converte
em juntas (q5 fixo pelo yaw) e o MoveIt executa.

**6. Quais TFs?**
`base_footprint → base_link → manip_base_link → link1..5 → tcp / tcp2` e
`link5 → camera_* → camera_color_optical_frame → tag_N`. Tudo nominal: comprimentos digitados
no xacro, extrínseca da câmera de CAD, sem mão-olho, sem zeros de junta. Detalhe na seção 5.

**7. Como o cubo é localizado depois da pega?**
Não é. O place assume que o centro da face de cima do cubo está no `tcp` e que as faces estão
paralelas aos dedos. A tag do cubo fica no campo da câmera quando ele está na garra, mas quase
nunca é detectada: em 14/11, nenhuma vez em 82 quadros, porque aparece cortada pela borda de
cima da imagem. O que a câmera vê do cubo na garra é a aresta frontal e os cantos inferiores da
face dele; na descida final essa aresta aparece na mesma imagem que a borda do furo-alvo
(seção 18). Ou seja, dá para medir o cubo em relação ao furo pelas arestas, sem a tag, mas hoje
nada faz isso.

**8. Principais fontes prováveis de erro?**
(1) leitura da tag dependente da pose, por escala (em disputa) e erro angular da cadeia
(centímetros);
(2) braço que para ~7 mm fora em xy e 15 mm em z; (3) malha aberta, sem verificação nem nova
tentativa; (4) detector de furos que não corrige; (5) offset fixo nos eixos do robô; (6) cubo
na garra não medido; (7) altura de soltura indeterminada. Seções 8 e 9.

**9. Que informações faltam?**
Medidas de régua (quadrado preto da tag, passo entre tags, tag→furo, lado e profundidade da
cavidade, altura do tampo, flange→ponta do dedo); verdade de campo do encaixe (foto de cima por
tentativa); ganhos e limites reais dos servos; `camera_info` publicado; geometria do container
de bordo; o critério de sucesso que vale em 18/11 (Cavity ou Precision Placement); a origem da
perda de quadros em `/detections`; o passo mínimo efetivo do braço sob carga; a visibilidade do
furo e do cubo nos slots laterais; repetibilidade do erro do braço.
Seção 17.

**10. Métodos mais promissores?**
Para este robô: (a) alinhar por imagem com o cubo na garra, em passos olhar-mover-olhar,
medindo na mesma imagem a aresta visível do cubo contra a borda do furo-alvo, com as tags e os
furos dos slots vizinhos como apoio (a tag do slot-alvo é tapada pelo cubo na descida;
seção 18); (b) verificar de forma relativa e tentar de novo, a começar por não declarar sucesso
quando a verificação que já existe decide "repetir"; (c) corrigir a chegada do braço pelas
juntas medidas, com a ressalva de que isso exige reafinar os offsets de z (14.2); (d) medir o
vetor tag→furo no plano da própria tag (homografia), por slot. Calibração completa (tamanho da
tag, mão-olho, zeros) é o caminho de médio prazo. Seções 12 a 14 e 18.

---

## 12. Revisão técnica dos métodos

Só entram referências que as frentes de pesquisa abriram ou baixaram. "PUBLICADO" é o que a
fonte diz; o resto é derivação ou hipótese de engenharia. Nenhum número de impacto abaixo é
medida de encaixe.

| Método | O que resolve | Base publicada | Compatibilidade com o Caramelo | Impacto esperado (estimativa) | Limite |
|---|---|---|---|---|---|
| M1. Escala métrica: `size` = lado do quadrado preto | Erro de ~28 % no vetor câmera→tag, se a hipótese de escala valer (em disputa, 10.1) | [R1] sec. III-C; [R3] (definição do tamanho) | Parâmetro já existe; conflita com a decisão de 09/11 | ~11 mm por 4 cm de afastamento lateral | Trocar sem recalibrar quebra o pick (já aconteceu) |
| M2. Imagem retificada ou cantos desdistorcidos antes do PnP | Viés dependente da posição na imagem | [R2] Fig. 8; [R1] sec. VI-B | `image_proc` instalado; ou PnP próprio sobre `/detections` | Até ~7 mm de profundidade na periferia; ~1 mm perto do centro | Sentido do modelo de distorção do driver não verificado |
| M3. IPPE com as duas soluções + teste de razão + prior do plano da mesa | Flips de orientação | [R4]; OpenCV 4.6 (`SOLVEPNP_IPPE_SQUARE`, `solvePnPGeneric`) | Alta; cabe como guarda na mediana existente | Remove os flips observados (`tag_102`) | Não corrige viés sistemático |
| M4. Referencial da mesa com várias tags | Yaw e plano da mesa | [R5]; [R4]; Umeyama (Eigen) | Média; `pnp_bundle` só existe no `apriltag_ros` master | Yaw de 1-2° para ~0,2° (0,9-1,8 mm → ~0,2 mm no braço de 52 mm) | Exige o layout; não corrige câmera↔braço |
| M5. Vetor tag→furo no referencial da tag, tabela por slot | Dependência do yaw do docking e da colagem | [R1] sec. III-C; [R3] | Alta | 0,9 mm por grau + colagem por slot | Exige sessão de calibração na arena |
| M6. Medir tag→furo na mesma imagem por homografia do tampo | Tira URDF, FK, `fx` e altura do plano da medida do furo | [R6]; [R1] sec. III-C | Média-alta; `/detections` já traz cantos e homografia | Remove ~4 mm no exemplo de 5 mm de erro de plano a 40° | Extrapolação amplifica ruído dos cantos (~3x); só vale no plano do tampo |
| M7. Quadrilátero do furo com bordas em subpixel na resolução cheia | Erro e viés do centro do furo | [R1] sec. III-B; [R2] sec. III-F; OpenCV `cornerSubPix` | Alta; nó Python existente | De ~1,3-2,1 mm para sub-mm, se o contraste for bom | Sombra da garra desloca a borda |
| M8. Profundidade da D455 só como verificação de escala e de plano | Fecha a dúvida 0,032 × 0,041 | [R7] Tabelas 4-11 e 4-14; [R8] | Alta para diagnóstico | Nenhum ganho direto no encaixe | Distância mínima de ~0,32 m em 640x480 pelo datasheet (não medida; item 7 de 10.1); ±2 % |
| M9. Geometria de observação padronizada (tag centrada, mesma distância) | Transforma erro variável por slot em viés comum | [R1] sec. VI-B; [R2]; [R9] Tabelas 3 e 5 | Alta; só poses | Com a hipótese de escala: de ~11 mm para menos de 3 mm, comum a todos os slots | Não elimina o erro; exige offset calibrado na mesma geometria |
| M10. Fechar a malha no TCP medido antes de soltar | Erro de regime dos servos | [R10] (passo 7: pré-calibrar o robô); [R11] (ganhos default P 800 / I 0) | Alta; não depende de visão | Remove ~7 mm em xy e ~15 mm em z | Não corrige zero de junta, flexão nem folga. O z de hoje fecha por cancelamento (P20): tirar só a queda sobe a soltura ~15 mm e exige reafinar os offsets de z (14.2) |
| M11. Realimentação visual (servo visual / olhar-mover-olhar) | Erros de calibração e de execução entram dentro da malha | [R12]; [R13]; [R14]; [R15] | Ver seção 13 | Ver seção 13 | Visibilidade do alvo com o cubo na garra: a tag do slot-alvo é tapada na descida; ficam a borda do furo, a aresta do cubo e os slots vizinhos (seção 18) |

**Sobre M11:** a análise da frente de pesquisa não chegou. O que o sintetizador leu nas fontes
baixadas: [R13] afirma que em inserções de tolerância muito pequena "boa calibração muitas
vezes não basta", porque a incerteza do sistema depende do efeito combinado de muitos fatores
difíceis de modelar; [R12] formula o servo visual como a regulação do erro entre características
medidas na imagem e seus valores desejados; [R14] (robOTTO 2019, como trabalho futuro) descreve
exatamente o problema desta mesa: para lidar com os erros de movimento do braço, o controlador
move o objeto um pouco sobre a abertura enquanto a visão compara objeto e abertura, e a visão
confere o alinhamento durante a descida; [R15] (b-it-bots 2013) usa câmera na garra com servo
visual como recurso quando o planejamento falha. O regulamento de 2023 [R16] permite
"estratégias de encaixe por forma, como balançar o objeto depois de colocá-lo".

### Referências

- [R1] Olson, "AprilTag: A robust and flexible visual fiducial system", ICRA 2011 — https://april.eecs.umich.edu/media/pdfs/olson2011tags.pdf
- [R2] Wang e Olson, "AprilTag 2", IROS 2016 — https://april.eecs.umich.edu/media/pdfs/wang2016iros.pdf
- [R3] AprilTag User Guide — https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide
- [R4] Collins e Bartoli, IPPE (IJCV 2014), README do repositório — https://github.com/tobycollins/IPPE
- [R5] Tutorial ArUco Board, opencv_contrib 4.6.0; `pnp_bundle` do `apriltag_ros` master — https://raw.githubusercontent.com/christianrauch/apriltag_ros/master/src/pose_estimation.cpp
- [R6] Tutorial de homografia do OpenCV — https://raw.githubusercontent.com/opencv/opencv/4.x/doc/tutorials/features2d/homography/homography.markdown
- [R7] RealSense D400 Series Datasheet, Rev 021 — https://dev.realsenseai.com/download/42003/
- [R8] Guia de tuning das câmeras de profundidade RealSense — https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance
- [R9] Abbas et al., Sensors 19(24):5480, 2019 — https://pmc.ncbi.nlm.nih.gov/articles/PMC6960891/
- [R10] Tsai e Lenz, IEEE T-RA 5(3):345-358, 1989 — https://kmlee.gatech.edu/me6406/handeye.pdf
- [R11] e-manual Robotis XM540-W270 — https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
- [R12] Chaumette e Hutchinson, "Visual Servo Control, Part I: Basic Approaches", tutorial de 2006 — cópia em `SCR/pp_pesquisa_calib/txt/ChaHut06.txt` (URL não chegou ao sintetizador)
- [R13] Haugaard, Buch e Iversen, "Self-supervised deep visual servoing for high precision peg-in-hole insertion", arXiv:2206.08800, 2022
- [R14] robOTTO, Team Description Paper RoboCup@Work 2019 — cópia em `SCR/pp_pesquisa_calib/txt/robotto2019.txt:306-321`
- [R15] b-it-bots, Team Description Paper RoboCup@Work 2013 — cópia em `SCR/pp_pesquisa_calib/txt/bitbots2013.txt:185-195`
- [R16] RoboCup@Work Rulebook 2023 — cópia em `SCR/pp_pesquisa_calib/txt/rulebook2023.txt:1013-1021, 1936-1946`; regulamento CBR 2025 em `SCR/pp_pesquisa_calib/cbr_rulebook_2025.txt`

Baixadas pela frente truncada e **não lidas** pelo sintetizador (análise não disponível):
Gans, Hutchinson e Corke, IJRR 22:955, 2003; Hutchinson, Hager e Corke, tutorial de servo visual
de 1996 (extração de texto falhou); Lian et al., arXiv:2103.05140; Xu et al., arXiv:1904.05240;
TDPs de AutonOhm (2017, 2023), b-it-bots (2019: reconhecimento de cavidade por contornos,
homografia e casamento de modelos), LUHbots (2016) e SWOT (2025, 2026).

Fontes que as frentes não conseguiram abrir: página HTML do `solvePnP` em docs.opencv.org (403;
usaram o markdown-fonte e o header instalado); artigo de suporte RealSense 360059129453 (403);
wiki de calibração de bundle do `apriltag_ros`; artigo de Kallwies et al. 2020 (só a página do
projeto).

---

## 13. Três abordagens comparadas

- **Abordagem 1 — melhorar a arquitetura atual.** Mantém leitura única de garra vazia e
  soltura em malha aberta. Conserta o que está barato: perda de quadros (fps a testar), offset
  no frame da tag e
  por slot, compensação da queda do braço, limiares do detector, cálculo de alcance,
  verificação com nova tentativa.
- **Abordagem 2 — sistema geométrico integrado com referencial da mesa.** Corrige a cadeia na
  raiz (tamanho da tag, distorção, mão-olho, zeros de junta, `tcp`, gravidade), constrói um
  referencial da mesa com várias tags e o layout do tile, e posiciona em malha aberta nesse
  referencial.
- **Abordagem 3 — realimentação visual (olhar-mover-olhar).** Com o cubo na garra e o braço
  parado perto da pose de soltura, mede na mesma imagem o erro relativo entre o furo-alvo e o
  cubo (bordas do furo contra as arestas visíveis do cubo, com as tags e os furos dos slots
  vizinhos como apoio; seção 18), corrige com um movimento pequeno, mede de novo, e só desce
  quando o erro estiver dentro do limite; depois verifica.

| Critério | 1. Melhorar a atual | 2. Geométrico integrado | 3. Olhar-mover-olhar |
|---|---|---|---|
| Precisão potencial (ESTIMATIVA) | 5-10 mm. Os erros dominantes dependem da pose e não são absorvidos por offset fixo; só chega ao furo com busca/espiral. | 3-5 mm. Limitada pela exatidão absoluta do braço (piso ~3 mm RSS de folga de catálogo, não medido; T2) e pelo resíduo de calibração. | 1-2 mm é **suposição**. Limitada pelo pixel (~0,65 mm a 0,25 m), pelo passo mínimo efetivo do braço sob carga (**não medido**: com o j2 parado a ~24 ticks do alvo, correções de 2-4 ticks podem não mover nada) e pelo que se sabe do cubo na garra. A validar no teste-portão (seção 15). |
| Complexidade | Baixa | Alta | Média |
| Custo | Dias | Semanas, com sessões de calibração | ~1 semana com testes |
| Integração | Trivial (parâmetros e trechos do place) | Toca URDF, IK duplicada e todos os offsets do pick; risco de quebrar o pick, como em 09/11 | Encaixa nas primitivas que já existem (mediana de tag, alvo de juntas, espera de assentamento, `/detections`) |
| Robustez | Média-baixa: depende de constantes que se compensam numa geometria | Boa enquanto calibrado; frágil a pancada na câmera ou troca de servo | Alta a erros de calibração, escala e queda do braço; sensível à visibilidade das feições (o cubo tapa a tag do slot-alvo; visibilidade provada só no slot central), à sombra da garra sobre a borda do furo e à perda de quadros |
| Dependência de calibração | Alta e implícita | Muito alta e explícita | Baixa: uma referência ensinada |
| Hardware | Nenhum | Nenhum obrigatório (ajuda: padrão de calibração, inclinômetro) | Nenhum (15 fps em USB 3 a testar) |
| Risco antes de 18/11 | Baixo | Alto | Alto: custo de ~1 semana e faltam 4 dias. Só depois do teste-portão (seção 15) |
| Histórico | É o caminho seguido de 04/11 a 13/11; o resíduo lido não saiu de 14-21 mm, e em 14/11 o cubo ficou fora do furo | Tentativa parcial em 09/11 (mão-olho rígido) não fechou | Não tentado |

---

## 14. Arquitetura recomendada e por quê

**Recomendação: a Abordagem 3 como núcleo, apoiada nos itens baratos da Abordagem 1 e na
representação "furo no referencial da tag" da Abordagem 2. A calibração completa da Abordagem 2
fica para depois da competição.**

**Ajuste depois do teste de 14/11 (seção 18):** a malha visual da Abordagem 3 deve medir, na
mesma imagem, a **aresta visível do cubo contra a borda do furo-alvo**, com as tags e os furos
dos slots vizinhos como apoio (escala local, yaw e identidade do slot). A tag do slot-alvo serve
na pré-pose, mas o cubo a tapa na descida final, então ela não pode ser a única feição. E, pela
regra do projeto "confiabilidade acima de melhoria" (`.jarvis/estado.md:13-14`), C3 só entra
antes de 18/11 se passar num teste-portão barato (seção 15).

### 14.1 Por quê

1. **Hoje a conta não fecha em malha aberta.** A folga é de 1-2 mm por lado; a queda sob
   gravidade é de ~7 mm; a cadeia câmera-braço erra centímetros de forma dependente da pose. O
   piso do atuador (~3 mm RSS) é ESTIMATIVA de catálogo, não medida: T2 decide se a malha aberta
   calibrada fecharia. Os ajustes de offsets feitos desde 21/10 não moveram o resíduo lido
   (medido de 04/11 a 13/11), e em 14/11 o cubo ficou fora do furo por centímetros com a fusão
   ligada.
2. **Com câmera na garra, o erro que importa é relativo.** Se o furo e o cubo são medidos na
   mesma imagem, na pose de soltura, a escala da tag, a extrínseca, os zeros de junta e a queda
   do braço viram erro comum. Hoje eles não se cancelam porque a pose de leitura é diferente da
   de soltura e há 17 s e uma ida ao container entre as duas.
3. **Um movimento pequeno erra pouco.** Se a medida tem erro proporcional de 25-30 % (escala
   mais rotação), corrigir 15 mm deixa ~4 mm; a segunda iteração deixa ~1 mm; a terceira,
   abaixo do passo do servo. A iteração converge enquanto o erro proporcional for menor que
   100 % (ESTIMATIVA; o limite prático é a folga e a resolução dos servos). **Não medido:** o
   passo mínimo efetivo do braço sob carga. Em 13/11 o j2 parou a ~24 ticks do alvo; correções
   de 2-4 ticks podem não mover nada.
4. **O sensor já está lá e vê o que precisa (seção 18).** Repetibilidade dos cantos da tag de
   0,15 mm com a câmera parada; `/detections` já publica os cantos em pixel. Na descida final de
   14/11, com o cubo na garra, aparecem na mesma imagem a aresta frontal do cubo, a borda de
   baixo do furo-alvo com os dois cantos inferiores, e o furo e a tag do slot vizinho. A tag do
   slot-alvo **não** é feição garantida: foi vista até o fim em 13/11 (1 slot central, 1
   corrida) e tapada pelo cubo em 14/11. Visibilidade nos slots laterais: não testada.
5. **Não depende de resolver a disputa do tamanho da tag.** A malha trabalha em pixels, com a
   escala local dada por medidas físicas vistas na própria imagem (lado do furo, lado do cubo,
   passo entre slots), então respeita a decisão vigente de não mexer em `tags_36h11.yaml` antes de
   18/11.
6. **O tile ajuda.** O regulamento diz que a tag fica a offset constante do furo; o vetor
   tag→furo pode ser uma constante por slot, expressa no frame da tag.

### 14.2 Arquitetura em camadas

| Camada | O que faz | Depende de | Elimina |
|---|---|---|---|
| C0. Medir | Régua e foto de cima por tentativa (seções 15 e 17) | Operador | Dúvidas de 10.1; cria a verdade de campo |
| C1. Chegar onde foi mandado | Depois do assentamento, comparar juntas medidas com o alvo e recomandar a diferença (1-2 iterações), ou pré-compensar com o ganho medido (~0,014 rad/Nm). Sem visão. **Ressalva (P20):** em 13/11 o TCP ficou 15 mm abaixo do comandado e o z fechou por cancelamento. Tirar só a queda sobe a soltura ~15 mm (o fundo do cubo passaria de ~1 cm para ~2,5 cm acima do tampo físico a −0,079) e, aplicada ao pick, repete o mecanismo que quebrou os picks em 09/11. Exige reafinar os offsets de z junto. | `/joint_states`, SETTLE; offsets de z do place (e do pick, se aplicada lá) | P01 (~7 mm xy, ~15 mm z, 2,7°) |
| C2. Furo no referencial da tag | Vetor tag→furo por slot, no frame da tag, girado pelo yaw da tag ou da mesa; medido por régua ou por homografia na mesma imagem | Cantos de `/detections` | P19; parte de P15 |
| C3. Olhar-mover-olhar com o cubo na garra | Dois estágios. (1) Na pré-pose, enquanto a tag do slot-alvo ainda é vista: alinhamento grosso pela tag e pelas vizinhas. (2) Na descida, com a tag-alvo já tapada: medir na mesma imagem a borda do furo-alvo (borda de baixo e cantos inferiores) contra a aresta frontal e os cantos inferiores do cubo, com as tags e os furos dos slots vizinhos dando escala, yaw e a identidade do slot; calcular o erro em relação à referência ensinada, mover a diferença, assentar, reler; repetir até o erro ficar abaixo do limite ou estourar o número de iterações; então descer devagar, reconferindo, e abrir | Borda do furo-alvo e aresta do cubo visíveis e segmentáveis (sombra da garra); ao menos um slot vizinho visível; quadros sem perda; passo mínimo do braço (teste-portão, seção 15) | P06, P07, P13, P20 (parte), P22 (parte), P23 |
| C4. Verificar e tentar de novo | Passo zero: não responder sucesso quando a verificação que já existe decide "repetir" (em 14/11 ela decidiu e o nó disse "Entreguei"). Depois: medir a tag do cubo em relação à tag do slot **na mesma imagem** (vetor esperado = o de C2) e a altura. Tag do slot-alvo sumida depois da soltura, com as vizinhas visíveis, é sinal de cubo em cima dela (14/11). Se não assentou, balançar ou re-pegar e repetir C3; relatar falha de verdade | C2; decidir com o operador o que a missão faz com a falha (hoje um abort derruba a árvore, P25) | P24 |
| C5. Raiz (depois da competição) | Tamanho da tag + mão-olho + zeros de junta estimados juntos; cantos desdistorcidos; `tcp` medido; ganho integral nos servos; limites de j3/j4; detector por borda | Sessões de calibração | P06a, P07, P12, P04, P05, P16 |

**A referência ensinada de C3** é uma só operação: com o cubo encaixado à mão num slot e a
garra fechada nele (depois erguida à altura de medida), gravar onde as bordas do furo aparecem
em relação às arestas do cubo, e onde aparecem as tags e os furos vizinhos. Como a referência é
furo contra cubo, ela vale para qualquer slot sem depender do vetor tag→furo; C2 passa a servir
ao estágio grosso e à verificação C4.

**O cubo dentro da garra (P22):** medindo a aresta do próprio cubo, a variação da pega entra na
malha em vez de ficar de fora, ao menos no que a aresta frontal mostra (posição lateral e giro
em torno do eixo da ferramenta). Continuam sem medida a inclinação do cubo e a aresta de trás,
que a câmera não vê. A tag do cubo não serve para isso nas poses de soltura: fica cortada pela
borda de cima da imagem. O teste T4 da seção 15 mede a dispersão; se a inclinação passar do que
a folga tolera, a saída é centrar o cubo mecanicamente na re-pega.

### 14.3 Itens baratos da Abordagem 1 que entram junto

- Não declarar sucesso quando a verificação que já existe decide "repetir" (P24). O que a
  missão faz com a falha é decisão do operador, porque hoje um abort derruba a árvore (P25).
- **A testar, não é item barato:** `camera_profile` 640x480x15. A câmera está em USB 3.2 e o
  apriltag usa 6,9 % de um núcleo, mas o próprio launch registra que a 15 fps o driver
  reiniciava em loop (em USB 2; LAUNCH:171-173, conferido), hoje já há perda de quadros a 5 fps
  e não há medida de estabilidade a 15 fps em USB 3.2. Subir o fps também não "destrava a
  mediana" (4.1): antes é preciso achar a origem da perda de quadros em `/detections`.
- Cálculo de "fora de alcance" do slot com a escada do slot, e teste da IK do alvo de garra
  vazia, antes de buscar o cubo (P25).
- Gravar `camera_info` no bag e copiar o log do driver para o run (P26).
- `save_debug_dir: ""` (P28) — depois de copiar as imagens que servem de regressão.
- Corrigir o comentário errado do YAML sobre `fx` (P08). Só o comentário, não o valor.
- Descida final mais lenta (P02).

### 14.4 O que não fazer agora

- **Não trocar o tamanho da tag isoladamente.** Já quebrou o pick em 09/11; os offsets de z do
  pick e do place foram afinados sobre o valor atual.
- **Não aprender resíduo por lado** (item 5 de 10.1).
- **Não aumentar o peso do detector de furos** enquanto a conversão métrica dele herdar a escala
  e o plano da tag.

**O que entra antes de 18/11 é decisão do operador.** O conjunto de menor risco é C0 + 14.3.
C1 não depende de visão, mas não é pequena nem isolada: mexe na altura de soltura e exige
reafinar os offsets de z. C3 é a que muda o resultado e é a de maior risco de prazo (custo de
~1 semana, faltam 4 dias): só vale começar a codar se o teste-portão da seção 15 passar.

---

## 15. Plano de testes progressivo

Regra geral: uma mudança por vez; cada tentativa gera uma linha de planilha e uma foto de cima
com escala (régua na cena). Sem a foto não há verdade de campo.

| Etapa | O que fazer | O que medir | Critério para avançar |
|---|---|---|---|
| T0. Medidas físicas (sem robô) | Paquímetro e régua | Lado do quadrado preto e do adesivo (slots e cubos); distância centro a centro de duas tags vizinhas; centro da tag → centro da cavidade, por slot; lado e profundidade da cavidade; lado do cubo; altura do tampo e do eixo do j2 ao chão; flange → ponta do dedo e → face de cima do cubo na garra | Tabela preenchida; decide 10.1 itens 1 a 4 |
| T1. Calibração e escala (robô parado) | Câmera a 0,35 m ou mais de uma tag; duas tags na mesma imagem; `ros2 topic echo --once` do `camera_info`; inclinômetro no link5 na pose de leitura de perto | z da tag × profundidade alinhada no mesmo pixel; distância entre tags medida × régua; K, D e modelo publicados; inclinação real × inclinação pela TF | Escala conhecida com erro menor que 3 %; erro angular quantificado |
| T2. Braço | 10 repetições na pose de soltura de 3 slots (esquerda, centro, direita), com e sem cubo; depois com a correção C1; aproximar pelo mesmo ponto vindo de cima e de baixo | Juntas medidas × comandadas; erro de TCP por FK; dispersão; diferença entre sentidos (folga) | Erro de TCP por FK abaixo de 1,5 mm e repetível |
| T3. Mesa | Ler cada um dos 7 slots de uma pose padronizada (tag centrada, mesma altura); repetir após re-docking | Posição em pixel e lado da tag; yaw por tag × yaw pela reta dos centros; vetor tag→furo por homografia × régua de T0 | Vetor tag→furo por slot com erro de até 1 mm contra a régua |
| T4. Cubo | 10 re-pegas em cada container de bordo; garra parada diante da câmera | Posição em pixel e ângulo da aresta frontal e dos cantos inferiores do cubo após fechar (a tag fica cortada pela borda da imagem, seção 18); taxa de detecção da tag do cubo na garra; foto lateral (inclinação) | Dispersão conhecida; se passar de ~1 mm, tratar P22 antes de T6 |
| TP. Teste-portão de C3 (antes de codar; sem código novo) | (a) Com o cubo na garra, parar na pré-pose e na altura de soltura do slot central e dos laterais (101/102 e 106/107), salvando os quadros como em 14/11. (b) Com o cubo na garra na pose de soltura, comandar passos de 1, 2 e 3 mm em x e em y, indo e voltando, 5 vezes cada | (a) Por slot: a borda do furo-alvo e a aresta do cubo aparecem? Quais tags vizinhas são detectadas? (b) Deslocamento real por passo (juntas medidas e pixel de uma tag vizinha) contra o comandado; menor passo que move de fato | Borda do furo e aresta do cubo visíveis nos slots que contam, e passo de 1-2 mm reproduzido com erro menor que 0,5 mm. Se falhar, C3 não entra antes de 18/11 |
| T5. Alinhamento (sem soltar) | Com o cubo na garra, rodar C3 e parar antes de descer; 10 tentativas por slot em 3 slots | Iterações até convergir; erro final na imagem; erro real cubo × furo pela foto de cima; tempo | Erro real de até 1,5 mm em 9 de 10 |
| T6. Inserção | Descer e soltar; variar a altura de soltura; testar o balanço | Assentou ou não (operador + foto); dz lido; verificação relativa C4 × verdade de campo | C4 concorda com o operador em todas; assentamento em 8 de 10 ou mais |
| T7. Repetibilidade | **30 tentativas:** 5 dockings diferentes x 6 slots, missão real | Por tentativa: slot, yaw do dock, pixel da tag, iterações, erro final na imagem, erro de FK, assentou?, tentativas, tempo. Taxa de sucesso com intervalo de confiança | Meta a definir com o operador. Referência: 27 em 30 = 90 %, com intervalo de 95 % de ~74 % a ~97 % |

Linha de base: antes de mudar qualquer coisa, rodar T7 reduzido (10 tentativas) com o código
atual e a foto de cima. Hoje não se sabe a taxa de sucesso de partida; o único dado com verdade
de campo é o de 14/11: 0 de 1.

---

## 16. Arquivos que precisarão ser modificados

Previsão; nada foi alterado nesta fase.

| Arquivo | Mudança prevista | Camada |
|---|---|---|
| PLACE | Correção de chegada depois do assentamento, onde hoje só loga o erro de FK (~4255-4296); laço olhar-mover-olhar entre a pré-pose e a descida final (~4198-4252); offset no frame da tag (3940-3953, 4806-4809); cálculo de alcance do slot (6509-6513); IK do alvo em `preObserveStackBase` (4329-4592); verificação relativa e nova tentativa (5882-5891); novos parâmetros | C1, C2, C3, C4, 14.3 |
| PREC + `test/test_place_precision.cpp` | Funções puras novas (erro na imagem → correção; offset no frame da tag; tabela por slot) com gtest | C2, C3 |
| LAUNCH e `caramelo_bringup/launch/robot_manipulation.launch.py:77-84` | `camera_profile`; passar os parâmetros `slot_*` por YAML em vez de defaults no código | 14.3 |
| TAGS | Agora: só corrigir o comentário das linhas 35-42. Depois: `sizes` | 14.3; C5 |
| HOLEY e HOLE | `save_debug_dir`; para C3: medir, com o cubo na garra, a borda do furo-alvo contra as arestas do cubo em resolução cheia (hoje a região da garra é zerada na máscara); depois: homografia, limiares; regressão com os quadros reais (os 415 de RUN14 servem) | 14.3; C3; C5 |
| `manip_bringup/test/test_pp_hole_detector.py` | Regressão com imagens reais | C5 |
| `caramelo_navigation/scripts/caramelo_bag_common.py` | Incluir `camera_info` e `/detections` completos no bag; copiar o log do driver | 14.3 |
| SETTLE | Se a correção de chegada for implementada como espera "no alvo" | C1 |
| ARMX, `manip_description/urdf/arm.xacro`, IK | Extrínseca da câmera, `tcp`, limites de j3/j4 — **os três juntos** (P21) | C5 |
| `manip_bringup/scripts/calib_camera_record.py` e `calib_camera_solve.py` | Gravar `/joint_states`; estimar escala e zeros de junta | C5 |
| CTRL, DRV, `HW/caramelo_hardware/src/arm_hw_interface.cpp` | Ganho integral, constante 651,8986, arredondamento (workspace da Raspberry) | C5 |
| `amt1_sort_action_node.cpp` | Manter coerente com o place (mesmos offsets; `slot_cavity_depth_m`) | C2 |
| `docs/place_precisao_e_pilha.md`, `.jarvis/estado.md`, `.jarvis/decisoes.md` | Atualizar (o doc descreve o modo legado; T1, T4 e T5 mudam de premissa) | — |

---

## 17. Perguntas objetivas ao operador

Só o que não dá para descobrir pelo código.

1. **O cubo 4 assentou no furo 104 em 13/11 às 21:19?** E em geral: de cada 10 tentativas,
   quantas entram, e como falha quando falha (fica na borda do lado do robô, em cima da tag,
   torto)? Em 14/11 ficou em cima da tag (seção 18): é o modo de falha habitual?
2. **Distância centro a centro entre duas tags vizinhas da PP** (régua): perto de 110 mm ou de
   140 mm?
3. **Lado do quadrado preto** das tags dos slots e dos cubos (paquímetro), e lado do adesivo.
4. **Distância do centro da tag ao centro da cavidade**, lado e profundidade da cavidade, lado
   do cubo. As tags dos slots foram coladas à mão ou o tile as posiciona?
5. **Qual critério de sucesso vale em 18/11?** "Cavity Placement" (objeto sobre o tile certo) ou
   "Precision Placement" (o objeto atravessa a cavidade)? Disso depende quanto erro é aceitável
   (item 9 de 10.1). **A mesa do laboratório é igual à da competição** (tile oficial, cor do
   fundo, cavidade rasa ou vazada)? A mesa de 13/11 era a mesma de 12/11? O que eram os
   retalhos vermelho e branco dentro das cavidades?
6. **Altura do tampo da PP ao chão** e altura do eixo do j2 ao chão (a URDF prevê 0,277 m).
7. **Container de bordo:** o fundo é plano? Quanta folga lateral o cubo tem lá dentro?
8. **Distância do flange (link5) à ponta do dedo** e à face de cima do cubo quando ele está na
   garra.
9. **Os arquivos que rodam na Raspberry** (`caramelo_controllers.yaml`, URDF) são os mesmos da
   cópia deste PC? É possível ler ganhos P/I/D, `Homing Offset` e limites dos servos?
10. **A câmera fica sempre em porta USB 3?** Pode-se assumir 640x480 a 15 ou 30 fps? (A
    estabilidade a 15 fps em USB 3.2 ainda não foi medida.)
11. **Na competição há tempo de setup na mesa de precisão** para ensinar uma referência e medir
    os slots? Quantos cubos vão para a PP nas provas que contam?
12. **Antes de 18/11:** só medidas e correções de baixo risco (C0 + 14.3), ou também C1 e C3? A
    decisão de não mexer no tamanho da tag antes de 18/11 continua valendo?
13. **Quando a verificação decide "repetir"**, o place deve responder falha (hoje isso derruba a
    árvore), tentar de novo, ou seguir como está?

---

## 18. Teste de visão de 14/11 (cubo na garra sobre o furo)

Único teste desta auditoria feito no robô. Responde à pergunta que decidia a viabilidade da
Abordagem 3: o que a câmera vê com o cubo na garra em cima do furo. Os fatos foram medidos pelo
coordenador; "(conferido)" marca o que foi relido no log, no CSV ou nos quadros nesta revisão.

**Dados:** RUN14 (`git=fbb8eab`, `git_diff.patch` vazio); `RUN14/pp_visao/quadros/` (415 JPEG
640x480, ~5 por segundo); `RUN14/pp_visao/quadros.csv` (t, arquivo, etapa do place, ids em
`/detections`); `RUN14/pp_visao/montagem.jpg`; LOG14. Não há foto de cima com régua.

### 18.1 O que foi feito

Missão pelo `bt_yaml_executor` normal: ir à PP (Mesa10) e fazer o `place` da `tag_1` (cubo posto
à mão no container 1 de bordo) em `at_tag: tag_104`
(`RUN14/config/20261114_112951_pp_teste_visao_actions.yaml:6-9`, conferido).

- **1ª tentativa:** tag lida de perto em x = 0,247; a IK própria não achou solução para
  `[0.240 0.329 -0.038]`; `unreachable_check`; cubo devolvido ao container1, "pronto para o
  ajuste da base" (LOG14:645, 981-987, 1180, conferido).
- **2ª tentativa**, com a tag de perto em x = 0,065: completou. O nó disse "Entreguei o bloco",
  a árvore terminou em SUCESSO e a missão saiu com código 0 (LOG14:1945, 2017;
  `runs/pp_auditoria_20261114/pp_teste/rodar.log:304`, conferido).

### 18.2 O que o nó registrou (2ª tentativa; LOG14, conferido)

| Evento | Valor | Linha |
|---|---|---|
| `tag_104` de longe | mediana de 5: `[0.0544 0.4644 -0.1196]` | 1266 |
| `tag_104` de perto (TCP2) | leitura única: `[0.0647 0.4245 -0.0838]`; tampo z = −0,084 | 1304-1307 |
| Buraco visto de garra vazia | dx = −2,8 e dy = −8,3 mm do alvo nominal | 1308 |
| Offset tag→buraco medido | dx = −2,8 e dy = −60,3 mm (2 amostras) | 1309 |
| Fusão | tag + nominal `[0.0647 0.3725]`, buraco `[0.0619 0.3641]`, peso 0,70 → alvo `[0.0627 0.3666]` (−1,9/−5,8 mm do nominal) | 1635-1636 |
| Alvo do TCP | `[0.063 0.367 -0.037]`, ferramenta a 0° | 1639 |
| Chegada | "erro de FK na chegada: 1.5 cm (informativo)" | 1751 |
| Verificação | `tag_1` por leitura única em `[0.0575 0.3850 -0.0387]`: dxy = 19,1 mm (18,4 mm além do alvo em y, 5,2 mm em x), dz = +3,0 mm; tolerância de 12 mm | 1867-1869 |
| Decisão | "[REF] decisao da verificacao: repetir"; "fora da tolerancia [...], mas nao mexo na pilha (stack_verify_correct=false)" | 1870-1871 |
| Resposta | "Entreguei o bloco"; "Arvore terminou com SUCESSO" | 1945, 2017 |

Longe contra perto nesta corrida: dx = +10,3, dy = −39,9 e dz = +35,8 mm, o mesmo padrão de P06.
Quatro das seis leituras de tag caíram para leitura única (LOG14:570, 608, 1304, 1867), como
em P09.

### 18.3 Resultado físico

- **FATO** (quadros 00388 e 00410; 00388 conferido): o cubo **não entrou** na cavidade. Ficou
  pousado em cima da AprilTag do slot 104, ao lado do furo. No quadro 00388 sobra visível só
  uma faixa da tag, e o cubo encobre a borda do furo do lado da tag.
- **FATO** (CSV, conferido): depois da soltura a tag 104 não é mais detectada (0 de 31 quadros
  em `stack_verifying`, `returning_pegar_obj_final` e `done`), enquanto a 105 aparece em 24 e a
  103 em 14. A `tag_1` do cubo aparece em 29 de 31 (9 de 10 na verificação).
- **ESTIMATIVA do tamanho do erro:** 3-5 cm pela imagem (coordenador); 19,1 mm pela leitura do
  próprio nó. Nenhuma das duas é medida de régua. O quadro 00388 é oblíquo e o topo do cubo
  fica 42 mm acima do tampo, então a paralaxe aumenta a cobertura aparente da tag; a leitura do
  nó tem o viés de P06. Nas duas contas o erro é de **centímetros**, 10 a 30 vezes a folga.
- **Direção:** para o lado da tag (+y, longe do robô), tanto na imagem quanto na leitura do nó.
  HIPÓTESE: como a queda do braço puxa o TCP para −y (P01), o erro dominante neste teste veio
  do alvo (percepção e offset), não da execução.

### 18.4 O que a câmera vê, etapa por etapa (2ª tentativa)

Número de quadros de cada etapa em que cada tag aparece em `/detections`
(`RUN14/pp_visao/quadros.csv`, quadros 00287 a 00414; tabela gerada do CSV).

| Etapa | Garra | Quadros | 104 (alvo) | 103 | 105 | Outras de slot | `tag_1` (cubo) |
|---|---|---|---|---|---|---|---|
| `opening_gripper` | vazia | 3 | 3 | 3 | 3 | 101:1 102:3 106:3 | 0 |
| `stack_pre_observe` | vazia | 9 | 9 | 9 | 9 | 101:9 102:9 106:9 | 0 |
| `stack_pre_observe_tcp2` | vazia | 11 | 11 | 11 | 11 | 101:2 102:2 106:2 | 0 |
| `stack_pre_observe_return` | vazia | 11 | 9 | 9 | 11 | 101:1 102:6 106:7 107:3 | 0 |
| `gripper_open_before_pre_container` | vazia | 3 | 3 | 3 | 3 | 101:3 102:3 106:3 | 0 |
| `going_pre_container` | vazia | 9 | 3 | 3 | 3 | 101:2 102:3 106:2 | 2 |
| `going_container` | vazia | 7 | 0 | 0 | 0 | — | 3 |
| `closing_gripper` | com cubo | 2 | 0 | 0 | 0 | — | 0 |
| `returning_pre_container` | com cubo | 7 | 0 | 0 | 0 | — | 0 |
| `going_pegar_obj` | com cubo | 16 | 1 | 1 | 1 | 102:1 106:1 | 0 |
| `stack_using_preseen_base` | com cubo | 1 | 1 | 1 | 1 | 101:1 102:1 106:1 | 0 |
| **`stack_j1_aim`** | com cubo | 4 | **4** | 4 | 4 | 101:3 102:4 106:4 | 0 |
| **`stack_pre_pose`** | com cubo | 7 | **5** | 4 | 7 | 102:2 106:2 | 0 |
| **`stack_final_approach`** | com cubo | 3 | **0** | 0 | 2 | — | 0 |
| **`opening_gripper_final`** | com cubo | 2 | **0** | 0 | 0 | — | 0 |
| `stack_lift_after_release` | cubo solto | 2 | 0 | 0 | 0 | — | 1 |
| `stack_verifying` | cubo solto | 10 | 0 | 0 | 8 | — | 9 |
| `returning_pegar_obj_final` | cubo solto | 6 | 0 | 1 | 3 | — | 6 |
| `done` | cubo solto | 15 | 0 | 13 | 13 | 101:9 102:13 106:3 | 14 |

O que a tabela e os quadros mostram:

- **Tag do slot-alvo (104):** vista em 4 de 4 quadros em `stack_j1_aim` e nos 5 primeiros dos 7
  de `stack_pre_pose`; perdida nos 2 últimos da pré-pose e em toda a descida final, tapada pelo
  cubo. A última detecção foi 1,4 s antes de a garra abrir.
- **Tags vizinhas:** a 105 aparece em 7 de 7 na pré-pose e em 2 de 3 na descida final; a 103 em
  4 de 7 e depois em nenhum. Na abertura da garra nenhuma tag é detectada.
- **Tag do cubo (`tag_1`):** 0 de 42 quadros com o cubo na garra (0 de 82 somando a 1ª
  tentativa). Ela aparece quase inteira, mas cortada pela borda de cima da imagem.
- **Quadro 00378, descida final** (conferido, medido por máscara de cor): aparecem na mesma
  imagem a aresta frontal inferior do cubo (194 px de largura, x de 175 a 369), a borda de baixo
  e os dois cantos inferiores do furo-alvo logo adiante dela, e o furo inteiro e a tag do slot
  vizinho (105, detectada). **O furo-alvo não aparece inteiro:** a faixa visível mede 114 x 42 px
  contra 121 x 125 px do furo vizinho, ou seja, cerca de um terço da altura; o resto fica atrás
  do cubo. No quadro 00381, com a garra abrindo, a faixa tem 123 x 66 px. ESTIMATIVA: ~0,37 mm
  por pixel no tampo nessa pose (furo de ~45 mm em 121 px).

### 18.5 Implicação para a arquitetura

1. **A tag do slot-alvo não pode ser a única feição da malha visual.** O cubo a tapa justamente
   quando está perto ou em cima dela. As feições que sobram na descida são a borda do
   furo-alvo contra as arestas visíveis do cubo, na mesma imagem, e as tags e os furos dos
   slots vizinhos. A tag-alvo continua útil no estágio grosso, na pré-pose.
2. **A pose do cubo na garra pode ser medida pelas arestas visíveis**, mesmo sem detectar a tag
   dele. Isso tira P22 do escuro no que a aresta frontal mostra; a inclinação continua sem
   medida.
3. **A verificação já sabia do erro e o nó declarou sucesso.** O primeiro ganho de
   confiabilidade é de fluxo, não de visão (P24; item novo em 14.3).
4. **Ver o furo de garra vazia não bastou.** O detector atuou, a fusão moveu o alvo 6 mm, e o
   cubo ficou fora por centímetros: a observação foi feita de outra pose, ~16 s antes da
   soltura, na mesma métrica da tag.
5. **Limites deste teste:** uma soltura, um slot (104, central), sem foto de cima com régua. Não
   foram testadas a visibilidade nos slots laterais nem a segmentação da borda do furo sob a
   sombra da garra. Que essas feições deem erro de milímetro é HIPÓTESE; o teste-portão da
   seção 15 é o que decide.

### 18.6 Onde está a evidência

As evidências da auditoria foram copiadas para
`/home/robofei-work/caramelo_ws/runs/pp_auditoria_20261114/` (101 MB): `pp_apriltag_camera`,
`pp_cubo_garra`, `pp_cubo_historico`, `pp_detector_furos`, `pp_fluxo`, `pp_pesquisa_calib`,
`pp_pesquisa_percepcao`, `pp_tf`, `pp_verificador` e `pp_teste` (YAML e scripts do teste de
14/11). Isso fecha a lacuna dos diretórios temporários apontada pelo verificador.
`/tmp/pp_hole_debug` (643 MB) **não** foi copiado, por decisão do operador.
