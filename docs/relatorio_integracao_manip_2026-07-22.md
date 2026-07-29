# Relatório — Integração manip_ws → caramelo_ws (2026-07-22)

Missão unificada nav + manipulação. Plano aprovado pelo operador; validação em
mock 100% concluída neste PC; validação no robô real pendente (Pi desconectada
no momento — checklist pronto na seção 5).

---

## 1. Arquitetura encontrada

**caramelo_ws (PC, branch robot_manipulation)** — navegação validada no robô:
Nav2 (DWB+RotationShim), AMCL, docking server (opennav), `dock_align_node`
(action RELATIVA `align_to_dock`, <3cm/~2°), registro de estações POR MAPA
(`docking.yaml` + `service_areas.yaml` em `caramelo_mapping/maps/<mapa>/`),
orquestrador `mission_runner.py` (script de autonomia superior — DockRobot não
roda em BT do Nav2) com placeholder de manipulação (sleep 5 s). GUI sem
orquestração de missão. Sem BT.CPP standalone, sem Groot.

**manip_ws (branch manip_dev)** — manipulação validada em OUTRO computador:
MoveIt2 + MoveIt Task Constructor + pick_ik; action servers `/pick_tag` e
`/place_tag` (MTC, trava flock, container_states, verificação de esforço de
preensão, telemetria); BT.CPP **v3** com `bt_yaml_executor` que monta a árvore
em runtime do YAML de ações (blackboard `action_<i>_<campo>`); `task_planner` /
`competition_yaml_translator` (CLI offline: YAML da competição → YAML de
ações); placeholder de navegação `SimulatedGoto` (sleep 5 s); áudio Piper;
caminhos hardcoded `$HOME/manip_ws/...`; `src/apriltag_ros/` = submódulo NÃO
inicializado (vazio neste clone).

**caramelo_hardware_ws (Pi, branch dev)** — descoberta central da auditoria: o
braço **já estava nativamente integrado**: joints `manip_joint1..7`,
controladores `arm_controller`/`gripper_controller` (JTC,
`follow_joint_trajectory`), URDF combinada (base+braço+RealSense, braço
enraizado em `manip_base_link`) publicada pela Pi, `/joint_states` cobrindo o
braço. Os nós MTC já usavam os frames do robô completo (`base_footprint`,
`manip_base_link`) — foram escritos para o robô integrado.

**apriltag_ros**: o código/binário vem do `~/ros2_ws` (clone do
christianrauch, buildado, sobrepõe o /opt/ros). O cfg da competição existia só
como modificação local NÃO commitada no ros2_ws → vendorado (ver seção 3).

## 2. Estratégia implementada

- **Executor único de missão = `bt_yaml_executor`** (manip_bt). Novo nó C++
  `GoToWS` (StatefulActionNode, padrão do PickTagBT) substitui o
  `SimulatedGoto` e reproduz via action clients a sequência validada do
  mission_runner: `[se dockado] undock → [use_docking] DockRobot
  (staging+approach) → AlignToDock fino → SUCCESS` / `navegação pura p/
  START/FINISH`. Estado `docked`/`current_dock_id`/`current_dock_type` flui
  pelo blackboard entre instâncias. SUCCESS somente alinhado/chegado; falha /
  rejeição / timeout cancela o goal em voo e propaga FAILURE à árvore.
- **Undock embutido no INÍCIO do próximo GoToWS** → ordem física
  `pick/place → home (planner já emite) → undock → navegar`, sem tocar no
  task_planner.
- **Epílogo automático**: após a última ação, o gerador anexa `home` + goto
  FINISH (navegação pura) — param `finish_dock_id` (default FINISH, "" desliga).
- **Fail-fast**: na GERAÇÃO da árvore, estação inexistente no docking.yaml ou
  com pose placeholder [0,0,0] aborta ANTES de mover o robô; pick/place em
  estação com `manipulation_enabled: false` gera warning.
- **Ctrl-C limpo**: handler próprio de SIGINT/SIGTERM → `haltTree()` → cancels
  chegam aos servers (consertou também o bug latente de goals órfãos de
  pick/place). Exit 130.
- **Mapeamento centralizado por mapa**: `ws_table_mapping.yaml` na pasta do
  mapa ganhou a chave irmã `ws_to_dock_id` (WS_1→WS1...). O task_planner lê só
  `ws_to_table_pose` (INTOCADO). Cadeia: WS_1 (tarefa) → Mesa0 (braço) + WS1
  (dock) → pose/type (docking.yaml) + use_docking/manipulation_enabled
  (service_areas.yaml).
- **MoveIt no PC com a URDF combinada da Pi**: `move_group_caramelo.launch.py`
  faz xacro do MESMO `caramelo_description/robots/robot.urdf.xacro` (git
  compartilhado PC↔Pi) + nova SRDF `caramelo.srdf` (virtual_joint →
  base_footprint; ACM com chassi/rodas; grupos e group_states intactos). SEM
  RSP e SEM ros2_control no PC em modo real (TF única = Pi). Nós MTC obtêm
  URDF/SRDF por TÓPICO (fallback do RDFLoader — verificado no código).
  RealSense com `publish_tf:=false` (frames vêm da URDF da Pi).
- **Mock fiel**: `use_mock_hardware:=true` sobe RSP + ros2_control
  mock_components com a MESMA URDF combinada e os MESMOS nomes de controllers
  — testa a SRDF e a cadeia MoveIt reais (os dois blocos ros2_control do xacro
  respeitam `use_mock_components`; verificado).
- **UX**: `ros2 launch caramelo_bringup robot_manipulation.launch.py` (nav +
  pilha manip, args p/ mock/câmera/apriltag/áudio/RViz) + `ros2 run
  caramelo_navigation run_mission --map-name <mapa> --task <tarefa.yaml>`
  (planeja → mostra o plano → confirma → executa; `--dry-run`,
  `--simulate-nav`, `--yes`; actions auditável em `~/.ros/caramelo_missions/`).
- **Decisões do operador**: BT.CPP v3 mantida (apt; Groot2 = melhoria futura
  via port v4); `mission_runner.py` intacto como utilitário de teste de
  navegação (só docstring). GUI fora do escopo desta entrega (backend
  primeiro); `use_gui` declarado sem efeito.
- **Qualidade**: revisão adversarial (workflow 3 eixos) achou e corrigiu 1 bug
  de runtime ANTES do primeiro build (`UnknownGoalHandleError` na corrida
  resultado×timeout no cancel) + 4 itens menores (formato XML v3, include
  <vector>, erros de YAML com nome do arquivo, nota do halt no GoToNamedPose).

## 3. Arquivos

### Adicionados (untracked — para sua revisão antes do commit)
- `src/manipulation/` — os 9 pacotes migrados do manip_ws (my_robot_msgs,
  manip_description, manip_hardware, manip_moveit_config, manip_bt,
  manip_task_execution, manip_commander, manip_audio, manip_bringup), SEM
  build/install/log e SEM o apriltag_ros vazio.
  - NOVOS dentro de manip_bt: `include/manip_bt/{go_to_ws_bt,mission_map_config}.hpp`,
    `src/{go_to_ws_bt,mission_map_config}.cpp`,
    `test/{fake_nav_actions,fake_pick_place}.py`.
  - NOVOS dentro de manip_moveit_config: `config/caramelo.srdf`,
    `launch/move_group_caramelo.launch.py`.
  - NOVOS dentro de manip_bringup: `launch/manip_stack.launch.py`,
    `config/caramelo_mock_controllers.yaml`, `config/tags_36h11.yaml` (cfg da
    competição — o SEU, com a vírgula do id 16 corrigida),
    `docs_falas_robo.txt` (em manip_audio).
- `src/caramelo_bringup/` — NOVO pacote launch-only:
  `launch/robot_manipulation.launch.py` (comando único).
- `src/caramelo_navigation/scripts/run_mission.py` — wrapper da missão.
- `src/caramelo_mapping/maps/arena2_520/ws_table_mapping.yaml` — mapeamento
  centralizado (ws_to_table_pose + ws_to_dock_id).

### Alterados (tracked, 11 linhas no total)
- `src/caramelo_navigation/CMakeLists.txt` — instala `run_mission` (6 linhas,
  mesmo padrão do mission_runner).
- `src/caramelo_navigation/scripts/mission_runner.py` — 5 linhas de docstring
  (missão completa agora é run_mission; este vira utilitário/referência A/B).

### Alterados DENTRO da cópia migrada (manipulation/, untracked)
- `manip_bt`: `bt_yaml_executor.cpp` (GoToWS no lugar do SimulatedGoto,
  epílogo, fail-fast, SIGINT, --dry-run, remoção do hardcode $HOME/manip_ws),
  CMakeLists/package.xml (+nav2_msgs, +caramelo_msgs, novas fontes).
- `manip_task_execution`: defaults `container_state_file` →
  `~/.config/caramelo/container_states.yaml` e lock →
  `/tmp/caramelo_manip_action.lock` (pick e place; params já existiam — o
  launch também passa explícito).
- `manip_moveit_config/package.xml`: warehouse_ros_mongo→sqlite,
  +caramelo_description, +pick_ik.
- `manip_bringup`: `manip_pc.launch.xml` (só a linha do apriltag_params_file →
  cfg vendorado), package.xml (+caramelo_description).
- `manip_audio/package.xml`: removida dep inválida `ament_python` (rosdep).

### Removidos / Movidos
- Nada removido dos repositórios. O `~/manip_ws` original está INTOCADO
  (fonte histórica). Dados movidos p/ fora do git:
  `~/.config/caramelo/container_states.yaml` (estado runtime).

## 4. Alterações no hardware workspace

**Nenhuma alteração foi necessária no caramelo_hardware_ws.** (git status
vazio, branch dev.) A Pi já publica a URDF combinada e os controladores que o
MoveIt espera. Única pendência CONDICIONAL: gate do TCP na validação real
(item 12 da seção 5) — a URDF da Pi tem tcp em z=0.20 e a standalone do
manip_ws tinha 0.17; se a medição física der 0.17, a correção é 1 linha em
`caramelo_description/urdf/mech/manipulator_arm.xacro:327` (sincronizar PC+Pi
via git e rebuildar os dois).

## 5. Validações executadas

**Pré-build**: xacro da URDF combinada renderiza; TODOS os links da
caramelo.srdf existem nela (comm vazio); raiz = base_footprint; py_compile de
todos os launches/scripts novos; revisão adversarial C++ (6 veredictos de
fatos + 3 eixos de código).

**Build**: `rm -rf build install log` + `colcon build --symlink-install` →
**19/19 pacotes, 2min17s, zero erros** (deps novas: ros-jazzy-
behaviortree-cpp-v3 3.8.8, pick-ik, moveit-task-constructor-{core,msgs,
capabilities}, warehouse-ros-sqlite — instaladas pelo operador).

**Mock (sem Pi) — 9/9 itens do plano + extras:**
1. TF única: `base_footprint→…→tcp` (z 0.945) e `→camera_color_optical_frame`
   resolvem; 1 publisher de /robot_description. ✓
2. `/joint_states` com 11 juntas (4 rodas + manip_joint1..7). ✓
3. jsb/arm_controller/gripper_controller **active**. ✓
4. move_group sem NENHUM erro de SRDF/link. ✓
5. MoveIt ponta-a-ponta: `home`, `pegar_obj`, `pre_container` planejados e
   EXECUTADOS no braço mock via arm_controller. ✓
6. `/pick_tag` e `/place_tag` no ar (nós MTC reais). ✓
7. `run_mission --dry-run`: plano correto; **fail-fast** comprovado (BMT pede
   WS_8 → "estacao nao existe no docking.yaml", aborta antes de mover). ✓
8. **Missão completa** (BMT adaptado WS_1↔WS_2) com GoToWS REAL contra fakes:
   dock→align→picks→home→undock→dock→align→places→home→undock→navegação pura
   até o FINISH real (0.80,-6.82) → **SUCCESS**. Falha injetada no dock →
   FAILURE no ponto certo. Ctrl-C no meio do dock → exit 130 + "cancel
   recebido em dock" no server. ✓
9. `~/.config/caramelo/container_states.yaml` resetado pelo pick real no
   startup (3× occupied:false, mtime novo). ✓

Observação de log: `unknown goal response, ignoring` do rclcpp_action é ruído
conhecido com 2+ clients da mesma action no mesmo processo (os vários GoToWS
da árvore); os resultados chegaram corretos em todos os casos.

**Robô real (Etapa 4) — itens 1-11 VALIDADOS em 2026-07-22 (Pi + câmera no PC):**
1. ✓ git HEAD idêntico PC↔Pi (f795d10), NTP da Pi sincronizado.
2. ✓ /robot_description: 1 publisher (RSP da Pi).
3. ✓ Sem 2º RSP; RealSense com publish_tf desligado (TF da câmera só da URDF).
4. ✓ Cadeias TF resolvem (base_footprint→tcp, →camera_color_optical_frame).
5. ✓ /joint_states ~100 Hz com o braço + /dynamic_joint_states (efforts).
6. ✓ 4 controllers da Pi active (mecanum/arm/gripper/jsb) via DOMAIN 67.
7. ✓ move_group carrega URDF combinada + caramelo.srdf sem erros.
8. ✓ **Braço FÍSICO moveu para `home`** via executor→move_group(PC)→
   arm_controller(Pi) — Execute success ~4,2 s; TF confirmou.
9. ✓ Garra abriu (-0.2265) e fechou (0.170) via gripper_controller
   (2× SUCCEEDED, error_code 0).
10. ✓ RealSense ~15 Hz, frame_id = camera_color_optical_frame.
11. ✓ AprilTag: tag id 1 detectada → frame `tag_1`; `tf2_echo base_footprint
    tag_1` = [0.67, -0.03, -0.03] (cadeia geométrica coerente).
Notas: warnings "Parameter X is not supported" no log = rs_launch.py reclamando
de launch args do contexto pai (benigno; silenciar com GroupAction scoped no
futuro); áudio degradado neste PC (sem executável piper/espeak-ng —
`sudo apt install espeak-ng` habilita o fallback).

12. ✓ **gate TCP fechado**: operador decidiu manter 0.200 m (valor da URDF da
    Pi, confirmado via TF) — NENHUMA mudança na Pi.
13. ✓ **PICK REAL SUCCEEDED**: pipeline MTC completo no robô físico —
    pre_approach → camera_xy_alignment → final_approach → closing_gripper →
    **verifying_grasp (esforço ok)** → deposito no container1 → done.
    container_states persistiu `container1: occupied, tag_frame: tag_1`.
14. ✓ **PLACE REAL SUCCEEDED**: garra pegou o objeto DENTRO do pote → levou à
    mesa (Mesa0) → depositou → container1 esvaziado no estado. (1ª tentativa
    abortou em `going_container` porque o cabo ethernet caiu DURANTE a
    execução — o braço chegou, o resultado nunca voltou ao PC e o
    MoveGroupInterface deu timeout; retry limpo passou. Lição operacional:
    quedas de rede viram abort de execução — fixar bem o cabo/considerar
    tolerâncias de rede em competição.)

15. ✓✓✓ **MISSÃO INTEGRADA COMPLETA — SUCCESS NO ROBÔ REAL** (mesmo dia):
    - Gate de 1 estação: GoToWS navegou→dockou→alinhou na WS1 com erro
      x=1,7cm y=1,1cm yaw≈2° (padrão do mission_runner validado). Undock ok.
    - Missão `missions/arena_test.yaml` via run_mission (3min45s, zero
      intervenções): WS1 dock+align (0,5/1,3cm/0,5°) → **PICK real** (câmera+
      esforço, 21s) → home → WS2 undock→dock+align (2,4/1,4cm/2°) →
      **PLACE real** (Mesa5) → home → undock → navegação pura até FINISH
      (0,80,-6,82) → **BT SUCCESS**.
    - Observação do operador: o ponto de dock da mesa (gravado 21/07) ficou
      um pouco longe da mesa re-montada — regravar pela GUI ("Salvar pose
      como dock") com o robô na distância ideal do braço (~40cm do objeto).
    - Incidentes resolvidos na sessão: (a) launch antigo deixou órfãos
      segurando a câmera ("Device or resource busy") — matar por comm antes
      de relançar; (b) `twist_relay.py` e mais 4 scripts SEM +x no src →
      com --symlink-install o launch falha ("not found on libexec") — bits
      corrigidos (aparecem no git como mode change).

## 6. Comandos finais

```bash
# (uma vez, PC) deps:
sudo apt install ros-jazzy-behaviortree-cpp-v3 ros-jazzy-pick-ik \
  ros-jazzy-moveit-task-constructor-core ros-jazzy-moveit-task-constructor-msgs \
  ros-jazzy-moveit-task-constructor-capabilities ros-jazzy-warehouse-ros-sqlite

# build (PC):
cd ~/caramelo_ws && rosdep install --from-paths src --ignore-src -r -y
rm -rf build install log && colcon build --symlink-install

# Pi (operador, como sempre):
ros2 launch raspberry_bringup hardware_bringup.launch.py

# PC — bringup completo (nav + manip):
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=arena2_520

# Missão (planeja → mostra → confirma → executa):
ros2 run caramelo_navigation run_mission --map-name arena2_520 --task BMT.yaml
#   --dry-run   so mostra o plano e o XML
#   --simulate-nav  navegacao simulada (sem Nav2)
#   --yes       sem confirmacao

# Mock completo no PC (sem robo):
ros2 launch caramelo_bringup robot_manipulation.launch.py \
  use_mock_hardware:=true use_nav:=false use_camera:=false \
  use_apriltag:=false use_audio:=false

# Conferencias:
ros2 action list | grep -E "pick_tag|place_tag|dock|align"
ros2 control list_controllers
ros2 run tf2_ros tf2_echo base_footprint tcp
```

## 7. Estado do Git (nenhum commit executado)

```
caramelo_ws (robot_manipulation):
 M src/caramelo_navigation/CMakeLists.txt          | +6
 M src/caramelo_navigation/scripts/mission_runner.py | +5
 ?? src/caramelo_bringup/
 ?? src/caramelo_mapping/maps/arena2_520/ws_table_mapping.yaml
 ?? src/caramelo_navigation/scripts/run_mission.py
 ?? src/manipulation/

manip_ws (manip_dev): limpo — INTOCADO
caramelo_hardware_ws (dev): limpo — INTOCADO
```

## Adendo 2026-07-23 — Diagnóstico e correção da manipulação em campo

Sintomas do teste de campo do operador: pick falhava 3× (garra fechando no ar),
braço não ia a home no início, voz muda, flood de logs no launch.

**Causa raiz do pick (hipótese do OPERADOR, confirmada no código):**
`getTagTransform` usava `tf2::TimePointZero` sem checagem de idade — tag vista
uma vez ficava ~10 s no buffer TF; com o robô/braço movido depois (câmera fica
no link5!), o alvo do grasp virava fantasma → garra fechava no ar → verificação
de esforço reprovava → 3 tentativas trocando solver → "Pick skipped" mascarado
como success. Corrigido: param `max_tag_age_sec` (1.0 s) no pick E no place —
tag precisa ter sido vista AGORA.

Demais correções: prólogo `home` no executor (params `startup_home`=true /
`startup_pose_name`); WARN gritante no PickTagBT quando o pick é pulado;
include da RealSense escopado (GroupAction scoped, zera 58 warnings de
startup); `decimate` 1.0→2.0 no cfg das tags (decisão do operador; notebook
saturava). Voz: binário Piper em `~/piper/` (release oficial rhasspy — CUIDADO:
`apt install piper` é um app de mouse) + `espeak-ng`/`ffmpeg` via apt (ffplay
aplica o efeito dog); modelo já estava no share.

**Validação 23/07 (robô real, mesa de 15 cm):** braço→pegar_obj, tag_1 fresca a
[0.55, -0.03, 0.19] (~34 cm do ombro), pick na PRIMEIRA tentativa com esforço
M6=1,00/M7=0,96 N·m (limiar 0,15) → container → `Pick completed`; place de
volta à mesa `Place completed`; prólogo home visível no dry-run; flood de
startup zerado; voz Piper+dog falando.

**VEREDICTO FINAL DO TCP: 0.200 CORRETO** (pick perfeito com tag fresca) — as
falhas de campo eram 100% a tag velha. **caramelo_hardware_ws permanece
intocado em definitivo.**

Nota operacional: a câmera D455 EXIGE porta USB 3 (numa 2.1 o nó sobe com
"Reduced performance" e NÃO publica imagens — foi a causa da câmera muda após
retrocar o cabo).

## Adendo 2026-07-23 (parte 2) — Mapa novo, AMCL, relógio e MISSÃO NO ARENA3_520

Problema de campo: perto das mesas o robô "entrava na mesa" no RViz (AMCL
arrastado ~30-40 cm; paredes deslocavam junto). Diagnóstico em 3 camadas +
correções, todas VALIDADAS com missão completa ao final:

1. **Mapa arena3_520 (novo, via slam_toolbox + teleop, smear 0.05→0.03)**:
   mesas nítidas (protocolo: aproximar devagar, parar 3 s, 2-3 ângulos por
   mesa). Aceitação por régua quantitativa (scan×mapa): baseline 2,0 cm; na
   mesa 4,5 cm (antes: 30-40 cm). Região da WS2 é localmente mais fraca
   (~10 cm) — coberto pelo refine.
2. **AMCL endurecido (A/B medido)**: `laser_likelihood_max_dist` 2.0→0.5
   (obstáculo deslocado deixa de "atrair" a pose) e `recovery_alpha_slow`
   0.1→0.001 (alphas iguais = recuperação NUNCA disparava — bug teórico).
   Validado: após o conserto do relógio o AMCL se RE-ANCOROU SOZINHO (2,0 cm
   sem 2D Pose). Regra operacional comprovada com dados: 2D Pose com ZOOM +
   vai-e-vem de 0,5 m até a nuvem fechar (10 cm→6 cm→2 cm nos três níveis de
   capricho).
3. **Relógio da Pi derivou DE NOVO (832 ms→envenenava o AMCL em movimento
   ~16 cm a 0,2 m/s)**: `systemctl restart systemd-timesyncd` na Pi resolveu
   (121 ms depois, ~100 ms é a própria varredura do LiDAR). PENDÊNCIA de
   infra: chrony servidor NESTE notebook (timesyncd depende de internet).
4. **Refine LiDAR calibrado e OPERANTE**: `refine_desired_face_dist=0.403 m`
   (WS1, resíduo 3 mm, face 0,60-0,73 m detectada nas duas mesas). Em teste
   isolado corrigiu +3,7 cm e convergiu a x=2,4 cm; NA MISSÃO: erro X final
   **2 mm (WS1) e 0,0 mm (WS2)** — docking ancorado na mesa FÍSICA.
5. **Colisão primitive: 1 ajuste de SRDF** — em home, as caixas envolventes de
   link2/link3 interpenetravam o chassi (~1-2 cm; falso-positivo de bounding;
   "Planning failed" instantâneo). Pares base_link↔link2/link3 desabilitados
   na caramelo.srdf; regressão de 13 poses (todas as group_states) passou no
   braço físico em 41 s.
6. **BUG do gerador**: `init_service_areas --sync-docking` escreve
   `dock_plugins` num esquema POR-DOCK (caramelo_ws1_dock...) incompatível com
   o launch validado (3 tipos genéricos) → "Dock requested has no valid
   plugin". Workaround aplicado: bloco dock_plugins copiado do mapa validado +
   tipos re-normalizados. CORRIGIR o gerador em sessão futura.
7. Observado 1×: "Failed to make progress" do Nav2 na navegação ao staging
   (retry interno resolveu; na missão final não ocorreu). Monitorar; se
   recorrer, é item próprio (progress checker × piso do ESC × inflação).

**MISSÃO FINAL NO ARENA3_520 (3min48s, SUCCESS)**: prólogo home → WS1
dock+align (x=-0,002 y=0,006) → PICK → home → WS2 undock+dock+align
(x=-0,000 y=-0,024) → PLACE → home → FINISH. Poses gravadas: START
[0.239,-0.002], WS1 [0.884,-2.964], WS2 [1.593,-0.987], FINISH [1.881,-6.450].
Default da missão agora é arena3_520 (robot_manipulation.launch.py).

### Pendência nova (23/07, pedido do operador): garra pega o bloco "na borda"
Descer o ponto de pega ~1 cm. Correção: **encurtar o tcp** em
`caramelo_hardware_ws/src/caramelo_description/urdf/mech/manipulator_arm.xacro`
(tcp_fixed, z **0.20 → 0.19**) — tcp mais curto = link5/dedos 1 cm mais
próximos da tag = pega mais profunda. É a ÚNICA mudança no hardware_ws:
sincronizar via git e REBUILDAR NOS DOIS LADOS (PC e Pi). Validar depois:
(a) pick nas mesas usadas; (b) folga dos dedos com o TAMPO na mesa mais baixa
(pega 1 cm mais funda = dedo 1 cm mais perto da mesa); (c) NÃO mexer no
refine_desired_face_dist (é distância da BASE à mesa, independente).

## Pendências
- Etapa 4 (validação real) quando a Pi for conectada — checklist na seção 5.
- Gate TCP 0.17×0.20 antes do primeiro pick real.
- WS3/WS4/SH1/PP1/RT1 com pose placeholder no arena2_520 (fail-fast protege;
  gravar poses reais pela GUI).
- Groot2: futuro opcional (port manip_bt p/ BT.CPP v4 + Groot2Publisher).
- GUI: integração com a missão fica para etapa própria (decisão do operador).
