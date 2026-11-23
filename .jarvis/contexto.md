# Contexto duradouro — caramelo_ws

Última revisão: 2026-11-10 (noite, após leitura aprofundada). Fontes: `README.md` da raiz, `git log`, docs/, transcrições das sessões de 19/10 a 10/11 e três relatórios de agentes de leitura (não salvos; resumo aqui).

## Objetivo e escopo

Workspace ROS 2 Jazzy do **PC de operação** do robô Caramelo (equipe RoboFEI @Work,
RoboCup@Work): navegação (Nav2), docking, manipulação (MoveIt + IK própria), missões,
GUI Qt e painel web. Drivers físicos ficam no outro repositório, `~/caramelo_hardware_ws`
(Raspberry Pi no robô e cópia no PC, de onde vem o URDF). Detalhes e ordem de `source`:
`README.md` (topo e §5).

**Competição: 2026-11-18.** Prioridade até lá é confiabilidade; "falhar em melhorar nunca
pode piorar" (operador, 2026-11-07).

## Organização do workspace

- `src/caramelo_*` — navegação, localização, mapeamento, GUI, web, msgs, utils, diagnósticos,
  `caramelo_behaviors` (plugin de escape, novo em 2026-11-07).
- `src/manipulation/` — `manip_task_execution` (pick, place, amt1: nós grandes, ~7k linhas o
  place), `manip_bringup` (launch do stack, detectores de visão, configs das tags),
  `manip_bt` (BT da missão, task planner), `manip_audio` (voz), moveit config, hardware, description.
- `missions/` — fonte única dos YAMLs de tarefa; `missions/tasks_geradas*/` são geradas.
- `docs/` — runbooks e relatórios (índice no `README.md` §7). `docs/historico/` é antigo e
  não reflete o estado atual.
- `scripts/limpar_stack.sh` — rodar após todo Ctrl-C antes de relançar.
- `.claude/worktrees/` — worktrees temporários de agentes (ignorados pelo git desde c1c7873).

Mapa de pacotes completo: `README.md` §2. Convenções (nomes, comentários, caminhos,
calibração só no YAML/launch com data e motivo): `README.md` §8.

## Restrições técnicas e preferências do operador (confirmadas)

- **Este PC é o do robô.** O operador lança navegação/manipulação em terminais próprios a
  qualquer momento. Nunca `pkill -f` por nome de nó; nós de teste só com `-r __ns:=/teste`
  ou nome diferente, matar pelo PID. Origem: incidente de 2026-11-07 (ver `decisoes.md`).
- **Nó de teste com `__ns:=/teste` NÃO isola tópico absoluto** (ex.: `topic_name: /joy_vel` no joy_teleop): remapear o tópico explicitamente (`-r /x:=/teste/x`) e conferir `ros2 topic info` antes de publicar. O shell do agente enxerga a rede ROS do operador (incidente 2026-11-23, sem movimento porque as rodas estavam fora do ar).
- **Nunca mover o robô** por conta própria; ferramentas marcadas "MOVE O ROBÔ" no README só com o operador.
- **Commits só quando pedidos explicitamente.** Push é manual, pelo operador. Git user `RoboFEIatWork`.
- Uma mudança de parâmetro por vez; validar fisicamente antes de alterar calibração global
  (lição do tamanho da tag, 2026-11-09).
- Respostas, docs e commits em pt-BR. Comentários de código em pt-BR **ASCII** (sem
  acentos), objetivos, datados, com o porquê. Markdown pode ter acentos.
- Dois computadores: PC de desenvolvimento (commita e envia ao GitHub) e este PC do robô.
- Build: `colcon build --symlink-install` em terminal novo. Após atualização de biblioteca
  do apt que quebre o link (ex.: fastcdr), apagar `build/<pkg>` dos pacotes afetados e rebuildar.
- D455: profundidade inválida abaixo de ~0,5 m; a visão de perto usa só cor + plano conhecido.
- graphify (instalado em 10/11): nunca gerar `graphify-out/` dentro do repo; usar `export GRAPHIFY_OUT="$HOME/.cache/graphify-out/caramelo_ws"` e citar o sha usado. Nunca rodar `graphify install`, `graphify claude install` nem `uv tool upgrade graphifyy`.

## Arquitetura e fluxo de uma missão (confirmado no código em 10/11)

- **Launch único**: `src/caramelo_bringup/launch/robot_manipulation.launch.py` (mapa padrão `arena3_520`, `nav_profile` `dwb|mppi`) → `caramelo_navigation/bringup.launch.py` (AMCL + map_server, depois de 5 s os servers do Nav2, docking, service areas, `nav2_twist_relay`) + `manip_bringup/launch/manip_stack.launch.py` (move_group, actions `/pick_tag`, `/place_tag`, `/amt1_sort`, RealSense, apriltag, detectores de container/superfície/buraco, `vision_debug`, voz).
- **Cadeia de velocidade**: `/cmd_vel_nav` → `velocity_smoother` → `collision_monitor` → `/cmd_vel` → `nav2_twist_relay` → `/mecanum_controller/reference` (na Raspberry).
- **BTs** em `src/caramelo_navigation/behavior_tree/caramelo_lattice_{dwb,mppi}.xml` e `_through_*`: SmacLattice → (SavitzkyGolay só na to_pose) → FollowPath com o controlador do perfil; recovery RoundRobin: `escape_free_space` (plugin `caramelo_behaviors`) → limpar costmap global + Wait → BackUp 0,10 m. Não há Spin.
- **Missão**: `ros2 run caramelo_navigation run_mission <task.yaml>` (ou action `/caramelo/run_mission` do `mission_server`, que sobe com a GUI ou o painel web, não com o bringup) → `compute_dock_distances` → `manip_bt task_planner` (sem ROS; gera `~/.ros/caramelo_missions/<stamp>_actions.yaml`) → pre-flight das actions → `manip_bt bt_yaml_executor` (BT.CPP v3; folhas GoToWS, PickTag, PlaceTag, ReachQueue, NudgeBase, Amt1Sort, GoToNamedPose). A missão roda em até 3 árvores: principal, re-tentativa do que faltou, epílogo.
- **GoToWS** (`src/manipulation/manip_bt/src/go_to_ws_bt.cpp`): `/undock_robot` → NavigateToPose até a staging (pose do dock + `staging_x_offset` de `docking.yaml`) → espera base parada por `/odom` → `align_to_dock` (`dock_align_node`: lateral, reta pelo LiDAR, refit, câmera D455 com peso 0,8, yaw em pulsos) → gate de pose (`pose_gate_strict` default false). `DockRobot` do opennav só como rollback. `tour_docks` repete essa sequência e grava `tour_<data>.md` na pasta do mapa.
- **Manipulação**: `mtc_pick_action_node` (escada de IK própria 0/15/30°, pick_ik como 3ª tentativa, verificação de pega por esforço, cubo vai para container de bordo com estado em `~/.config/caramelo/container_states.yaml`); `mtc_place_action_node` (ramos: pilha/slot por `stack_on`, container por cor `ct_*`, superfície livre pelo `surface_place_planner`, poses fixas como fallback; publica `/manip/pp_plane_z` e assina `/manip/pp_hole_detector/holes`); `amt1_sort_action_node` (observa de `tag_direita`/`tag_esquerda`, cria `tag_amt1_slot_k` em `odom`, casa com tags 101–107, chama pick/place/`nudge_base`). O prefixo "mtc" é histórico: o MoveIt Task Constructor não é usado. Os três nós compartilham ~13 helpers copiados (README §9.14).
- **Interfaces próprias**: `caramelo_msgs` (actions `AlignToDock`, `NudgeBase`, `RunMission`, `SaveServiceAreaPose`; msg `MissionStatus`; srvs de service areas) e `my_robot_msgs` (actions `PickTag`, `PlaceTag`, `Amt1Sort`).
- **GUI Qt** (`caramelo_gui`, ~9,4k linhas, RViz embutido, 0 testes) e **painel web** (`caramelo_web`, FastAPI na 8080; `fastapi` NÃO está instalado neste PC). Ambos só listam tarefas de `missions/` raiz e `missions/tasks geradas/` (não `tasks_geradas2/`).
- **Mapas**: `src/caramelo_mapping/maps/<mapa>/` (`map.yaml`, `service_areas.yaml`, `docking.yaml`, `dock_distances.yaml`, `tour_*.md`). Em uso em 2026-10/11: `arenalegal_2026` e `arena_extendida`.
- **Testes existentes (contagem por grep, sem rodar)**: manipulação 149 `TEST` C++ + 122 `def test_`; navegação 6 arquivos pytest (2 não registrados no CMake); web 4; GUI, msgs, localização, mapeamento, diagnósticos: nenhum.

## Cadeia de navegação e movimento (lida em 2026-11-10; configuração declarada, NÃO confirmada em execução)

Fonte: YAMLs/launches deste repo + `~/caramelo_hardware_ws` (cópia no PC, branch `rasp_5`, 2 commits à frente de `origin/rasp_5`; **o que roda de fato na Pi não foi verificado**). Valores numéricos: ver os arquivos, aqui só o essencial.

**Fluxo de comando (PC → Pi)**
`bt_navigator` (BT por `nav_profile`) → `planner_server` (SmacLattice, replan 1 Hz) → `smoother_server` (Savitzky-Golay, só NavigateToPose) → `controller_server` 20 Hz (`DWBHolonomic` = RotationShim + DWB; ou `MPPIHolonomic` Omni) publica Twist em `/cmd_vel_nav` (behavior_server também) → `velocity_smoother` 20 Hz OPEN_LOOP → `/cmd_vel_smoothed` → `collision_monitor` (só `FootprintApproach` 1,2 s ligado; `PolygonStop` desligado) → `/cmd_vel` → `nav2_twist_relay` (`caramelo_utils/twist_relay.py`: carimba com o relógio do **PC**, frame `base_footprint`, sem watchdog) → `/mecanum_controller/reference` (TwistStamped) → **Pi**: `mecanum_drive_controller` (100 Hz, `reference_timeout` 0,15 s, sem limites próprios) → interfaces `velocity` das 4 rodas → `MobileBaseHWInterface` → `MaxonMotorsNode` (laço próprio 100 Hz, pulsos servo 50 Hz por lgpio) → ESC B-G431B-ESC1 (PI de velocidade no **firmware**, fora do repo).
- **Fora dessa cadeia, direto na referência das rodas**: `dock_align_node` (docking, nudge_base) — pula smoother e collision monitor. Teleop: `caramelo_utils/teleop.launch.py` no PC é usado **só pelo teclado** (o joy_node/joy_teleop dele existem no launch mas o controle não é usado ali; correção do operador 2026-11-22); o **controle** é o `joystick.launch.py` da Pi (lançado à parte, não está no `hardware_bringup`), direto em `/mecanum_controller/reference`, sem twist_mux (angular 8,0 no eixo 3, linear 1,0 no eixo 1, sem y, deadman botão 5, deadzone 0,5). O painel web publica em `/cmd_vel_key`, que só chega às rodas se o teleop do PC estiver no ar.
- **Piso do ESC**: só no driver da Pi (`min_wheel_rad_per_sec` 2,43 = 650 rpm, histerese de desligamento 0,4×; bandas 1460/1540 µs em torno de 1500). No PC, por eixo: `velocity_smoother` deadband pequeno de propósito (decisão 2026-08-03); DWB `min_speed_xy` 0,131 / `min_speed_theta` 0,35; `dock_align` `min_body_vel` 0,1215 / `min_body_omega` 0,315 (= 2,43×0,05 e 0,1215/0,3855).
- **Limites de velocidade** (iguais em DWB, MPPI e smoother): vx 0,60, vy ±0,40, wz 0,50; acel 0,7 / −0,9 (x,y), 2,0 (θ). Ré: DWB não (min_vel_x 0); MPPI e smoother até −0,25; BackUp/escape usam 0,25.

**Planejamento e controle**
- SmacLattice com `config/lattice/caramelo_omni_2cm_16bins.json`: modelo **omni**, 16 direções, raio de giro 0 (gira no lugar), 368 primitivas, inclui translação lateral e para trás; `allow_reverse_expansion` false; `cost_penalty` 3,0; tolerância 0,10. ThetaStar declarado, não usado pelas BTs.
- DWB: RotationShim gira no lugar se o caminho desvia > 0,785 rad (a 0,35 rad/s) e no fim (`rotate_to_goal_heading`); críticos PathDist 24, GoalDist 18, PathAlign 14, GoalAlign 8, RotateToGoal 32, ObstacleFootprint 1,0, Oscillation, Twirling; sim_time 1,7 s; amostras 10×8×21.
- Goal checker usado pelas BTs: `general_goal_checker` (xy 0,10, yaw 0,20, stateful). Progress checker: `PoseProgressChecker` (0,10 m ou 0,5 rad em 6 s).
- Recovery (BTs DWB/MPPI, 2026-11-07): planner → limpa global só se `WouldAPlannerRecoveryHelp`; controller → limpa local (exceto 1 m) só se `WouldAControllerRecoveryHelp`; depois RoundRobin: `escape_free_space` 0,20 m a 0,25 (plugin `caramelo_behaviors`, permite ré) → limpa global + Wait 1 s → BackUp 0,10 m. 3 tentativas. Não há Spin.

**Costmaps**
- Footprint igual nos dois: retângulo x −0,285…+0,395 / y ±0,195 com ressalto frontal de ±0,055 entre 0,285 e 0,395 (frente do LiDAR). Sem padding.
- Local: `odom`, rolling 5×5 m, 0,02, update 10 / publish 5 Hz; obstacle (só `/scan`, `inf_is_valid` true, marca até 2,5 m, limpa até 3 m) + denoise + inflação 0,75 / escala 3,0. Global: `map`, static + obstacle + denoise + inflação 0,75 / 3,0, 1 Hz, `allow_unknown`. Keepout só com `use_keepout:=true` (default false).
- `/scan` = LiDAR RPLIDAR S2 (`sllidar_ros2`, DenseBoost) → `scan_normalizer.py` corta para **só os 180° da frente** (`fill_with_inf`). Traseira e parte das laterais são cegas.

**Estado e realimentação (Pi)**
- Encoders: quadratura x4 amostrada no RP1, 114 688 contagens/volta da roda; medidas à mão 2–3% abaixo do esperado (não corrigido). Odometria integrada pelo `mecanum_drive_controller` → `/odom/wheel` (covariância fixa 0,1), `enable_odom_tf` false.
- EKF `robot_localization` (Pi, 40 Hz, 2D): funde `/odom/wheel` (vx, vy, vyaw) + `/imu/data_raw` (só vyaw) → `/odom` + TF `odom→base_footprint` (único publicador). IMU `wit_ros2_imu` ~10 Hz, covariância zero segundo o próprio repo; `imu_covariance.py` existe e não é lançado.
- AMCL (PC): `OmniMotionModel`, alphas 0,2, likelihood field, 600–1000 partículas, update 0,05 m / 0,05 rad, publica `map→odom`, `transform_tolerance` 1,0. `transform_tolerance` 0,6 nos costmaps/controller porque "o EKF da Pi tem picos de atraso de TF ~0,5 s" (comentário do operador, 07/11).
- TF: `map→odom` AMCL (PC); `odom→base_footprint` EKF (Pi); `base_footprint→base_link` e sensores: `robot_state_publisher` da Pi (URDF `caramelo_description`); câmera D455 presa no `link5` do braço.
- Cinemática: mecanum 4 rodas, raio 0,05; URDF x ±0,236 / y ±0,145 (soma 0,381) × controller `sum_of_robot_center_projection` 0,3855. Rodas esquerdas com sinal −1 no driver.

**Arquivos principais**

| Componente | Arquivo |
|---|---|
| Launch nav (nós, remaps, perfis, respawn) | `src/caramelo_navigation/launch/navigation.launch.py` |
| BTs | `src/caramelo_navigation/behavior_tree/caramelo_lattice_{dwb,mppi}.xml`, `_through_*` |
| Controller + costmap local | `src/caramelo_navigation/config/controller_server.yaml` |
| Planner + costmap global | `src/caramelo_navigation/config/planner_server.yaml`, `config/lattice/` |
| Smoothers, behaviors, collision monitor | `config/{smoother_server,velocity_smoother,behavior_server,collision_monitor,bt_navigator}.yaml` |
| Escape | `src/caramelo_behaviors/` |
| Relay e teleop | `src/caramelo_utils/caramelo_utils/twist_relay.py`, `launch/teleop.launch.py`, `config/twist_mux_topics.yaml` |
| Docking (publica direto nas rodas) | `src/caramelo_navigation/scripts/dock_align_node.py`, `launch/docking_server.launch.py` |
| AMCL | `src/caramelo_localization/config/amcl.yaml` |
| Bringup da Pi | `~/caramelo_hardware_ws/src/raspberry_bringup/launch/hardware_bringup.launch.py` |
| ros2_control | `~/caramelo_hardware_ws/src/raspberry_bringup/config/caramelo_controllers.yaml`, `src/caramelo_description/urdf/control/mobile_base.ros2_control.xacro` |
| Driver das rodas | `~/caramelo_hardware_ws/src/caramelo_hardware/src/{mobile_base_hw_interface,maxon_motors_node}.cpp` |
| EKF | `~/caramelo_hardware_ws/src/caramelo_localization/config/ekf.yaml` |
| Geometria / sensores | `~/caramelo_hardware_ws/src/caramelo_description/urdf/mech/mobile_base.xacro` |
| LiDAR | `~/caramelo_hardware_ws/src/raspberry_bringup/{config/rplidar_s2.yaml,scripts/scan_normalizer.py}` |
| Docs do ESC, odometria, relógio | `~/caramelo_hardware_ws/docs/{esc_stm32_comportamento_e_riscos,calibracao_odometria,relogio_chrony}.md` |

## Comandos documentados × executados

- **Executados com sucesso nesta máquina (relato das sessões)**: `colcon build --symlink-install` completo (09/11); `colcon test --packages-select manip_task_execution --ctest-args -R 'test_'` (25/25, 09/11); `python3 -m pytest src/manipulation/manip_bringup/test/test_pp_hole_detector.py` (7/7, 09/11); gtest do `caramelo_behaviors` (07/11, 7/7 na versão daquele dia); pytest de `caramelo_navigation` (293 ids em cache de 27/10, sem falhas).
- **Só documentados (README §5/§6), sem registro de execução recente**: `colcon test` de `manip_bt`, `caramelo_gui`, `caramelo_web`; `custom_ik_check`; cenários `run_*_scenario.sh`; `tour_docks` (rodado pelo operador em 26 e 31/10, não por agente).

## Comandos verificados

```bash
./scripts/limpar_stack.sh                                   # antes de qualquer launch
colcon build --symlink-install                              # build completo
colcon test --packages-select manip_task_execution --ctest-args -R 'test_'   # 25 gtests (2026-11-09)
python3 -m pytest src/manipulation/manip_bringup/test -q    # inclui test_pp_hole_detector (7 testes)
```

Outros testes e ferramentas: `README.md` §5 e §6. Logs dos nós: `~/.ros/log/`.

## Onde a memória e os históricos ficam (verificado em 2026-11-10)

| O quê | Onde | Automático? | Compartilhado? |
|---|---|---|---|
| Memória compartilhada | `.jarvis/` (este diretório, dentro do repo, ainda não commitado) | **Não**: só se o agente seguir o protocolo do `CLAUDE.md`/README | Via git, quando commitado e pushado |
| Instruções carregadas no início | `CLAUDE.md` na raiz (Claude Code carrega sozinho) | Carregar sim; isso **não** grava nada | Via git |
| Memória pessoal do Claude Code | `~/.claude/projects/-home-robofei-work-caramelo-ws/memory/` (12 arquivos + `MEMORY.md`, 19/10 → 10/11) | O Claude escreve quando julga relevante; não é garantido | **Não** (por usuário e por máquina) |
| Transcrições das conversas | `~/.claude/projects/-home-robofei-work-caramelo-ws/*.jsonl` (3 sessões, 19/10 → 10/11, 57 MB) | **Sim**, o Claude Code grava cada turno (evidência: as 3 existem e cobrem o período) | **Não**; pode ser compactada/apagada pelo próprio Claude Code |
| Logs dos nós ROS | `~/.ros/log/` (2491 pastas) | Sim (ROS) | Não |
| Hooks/automação para atualizar `.jarvis/` | nenhum (`~/.claude/settings.json` só tem `modelSettings`; sem `.claude/settings*.json` no projeto) | — | — |
| Docs do projeto | `docs/`, `README.md` | Não | Via git |

O que **não** está sendo preservado: respostas e decisões tomadas em conversas que não viraram doc, commit ou `.jarvis/` (ex.: motivo do AMCL 600/1000); resultados de testes no robô (só existem como fala do operador nas transcrições); o trabalho do PC de desenvolvimento entre commits. Sugestões que dependem de autorização futura: commitar `.jarvis/`; um hook de fim de sessão que lembre de atualizar `estado.md`; gravar os relatórios de tour/teste em `docs/` ou no `.jarvis/historico.md` no mesmo dia.

## Referências principais

| Para | Onde |
|---|---|
| Runbook de competição, formato do YAML de missão | `docs/guia_missoes.md` |
| Precision placement (PP) e pilha, fusão tag + buraco | `docs/place_precisao_e_pilha.md` (§3.1) |
| Tuning do DWB e decisões de navegação de 07/11 | `docs/tuning_dwb_2026-11-07.md` |
| Docking: diagnóstico e correções | `docs/diagnostico_docking_tour_2026-10-26.md`, `docs/docking_alinhamento_holonomico.md` |
| Quadro de status dos itens de outubro | `docs/status_itens_2026-10-26.md` |
| Voz do robô | `docs/voz_do_robo.md` |
| Decisões pendentes do time (licença, dois escritores de YAML, etc.) | `README.md` §9 |
| Parâmetros do detector de buracos da PP | `src/manipulation/manip_bringup/config/pp_hole_detector.yaml` |
| Tamanhos das tags | `src/manipulation/manip_bringup/config/tags_36h11.yaml` |
| Docking: parâmetros vigentes (câmera, pulsos, piso do ESC) | `src/caramelo_navigation/launch/docking_server.launch.py` |
| Formato das tarefas e onde o planner lê | `missions/README.md`, `src/caramelo_navigation/caramelo_navigation/run_mission.py` |
| Interface gráfica e painel web (pendências de validação na §7) | `docs/interface_grafica.md` |
