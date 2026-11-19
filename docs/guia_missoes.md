# Guia do operador — Missões do Caramelo (nav + manipulação)

Runbook de competição: subir o sistema, rodar uma missão, escrever o YAML da tarefa,
preparar os arquivos do mapa e entender o executor. Validado no robô real em
2026-07-22 (missão completa autônoma WS1→WS2→FINISH com pick e place); o relatório
daquela integração está em `docs/historico/relatorio_integracao_manip_2026-07-22.md`
(histórico). Datas das mudanças: §7.

Sumário: 1 Subir · 2 Rodar · 3 Formato do YAML (competição, novo, empilhar, container,
superfície, AMT, nível baixo) · 4 Arquivos por mapa · 5 Executor (parâmetros, fila de
alcance, memória de slots) · 6 Problemas comuns · 7 Histórico.

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
`use_rviz_manip:=true` (RViz do MoveIt), `use_commander:=false` (não sobe o
`commander`, ferramenta de bancada do braço), `use_mock_hardware:=true` (tudo
simulado no PC, sem robô).

### 1.1 Runbook 100 % terminal (sem GUI)

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
#   Pose do START do arenalegal_2026 (docking.yaml docks.START.pose:
#   x -0.807, y 0.0419, yaw -0.00846 rad; quaternion z = sin(yaw/2),
#   w = cos(yaw/2)). Para outro mapa, leia a pose no docking.yaml dele —
#   o exemplo antigo (0.0979, 0.0384, yaw -0.22) era de outro mapa.
#   O amcl.yaml tem set_initial_pose: true com (0, 0, 0): se o robo NAO estiver
#   fisicamente no START, ou o START do mapa nao for a origem, este /initialpose
#   (ou o 2D Pose Estimate) e' obrigatorio antes de qualquer missao/tour.
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -0.807, y: 0.0419},
   orientation: {z: -0.00423, w: 0.99999}}, covariance: [0.25,0,0,0,0,0,
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

### 1.2 Passo a passo com visualização (nav + MoveIt + câmera)

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
#   (camera + o que o robo ENTENDEU — tags, containers, mapa da mesa — em OUTRO
#    terminal, ou use_vision_viewer:=true no launch; ver §6.1:
#    ros2 run rqt_image_view rqt_image_view /manip/vision_debug/image)

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
- A **ordem das estações** sai de uma busca que minimiza o tempo estimado quando o mapa
  tem `dock_distances.yaml` (linha `Tempo estimado guloso: … ; otimizado: …` no log do
  planner); sem o arquivo, o planejador guloso de sempre. Ver `docs/planejador_por_tempo.md`.
- Ctrl-C no meio: cancela limpo os goals em voo (nav e braço).
- Cada corrida grava **vídeo do RViz + rosbag + logs + config + relatório** em
  `runs/run_<stamp>/` (§2.3): `--no-bag` desliga; `--bag-extra` e `--bag-camera` gravam mais.

### 2.1 Conferir os docks antes da missão (`tour_docks`)

Antes de rodar a missão numa arena nova (ou depois de mexer no mapa/poses), leve o robô por
**todos** os docks e veja onde ele chega bem e onde apanha. **Move o robô**; stack no ar (§1) e
robô localizado no START.

```bash
ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026
ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026 --only WS1,WS3 --skip SH1 --yes
ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026 --no-align       # só /dock_robot (opennav), sem align
ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026 --undock-first WS3   # robô ainda encostado na WS3
ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026 --finish ""      # não vai ao FINISH no fim
```

- Ordem: WS1..WSn, SH*, PP*, RT* (numérica dentro de cada tipo: WS10 depois de WS9) e
  `--finish` por último (default FINISH). START nunca é visitado.
- Em cada dock, a **mesma sequência do GoToWS** da missão: `/undock_robot` (se o anterior encostou)
  → Nav2 até a staging (pose do dock + `staging_x_offset` do `docking.yaml`) → `align_to_dock`
  (faz o approach; refino LiDAR ligado como no executor, `--no-lidar-refine` desliga).
  `--opennav-approach` usa o `/dock_robot` como no rollback `use_opennav_approach`; `--no-align`
  = `/dock_robot` faz o approach e nada de align. START/FINISH (`use_docking: false` no
  `service_areas.yaml`) são navegação pura. Timeouts = defaults do `bt_yaml_executor`
  (`--max-staging-time 120`, `--align-timeout 90`, `--max-undocking-time 30`).
- Mostra o plano e pede confirmação (`--yes` pula). Ctrl-C cancela a action em voo (o robô para)
  e grava o relatório parcial. Falha num dock não interrompe o tour (`--stop-on-fail` interrompe).
- Voz (`/manip/speech`): `!Dock WS3 pronto, erro 3 centimetros e 2 graus` /
  `!Dock WS3 falhou: alinhamento`. Não grava bag.

**O que olhar no relatório** (`maps/<mapa>/tour_<data>.md`, ou `--report arquivo.md`; a mesma
tabela sai no terminal):

| Coluna | Significado |
|---|---|
| `Status` | `ok`; `ok_aviso` = alinhou, mas o align avisou `yaw_fora_da_tolerancia`/`yaw_nao_medido`; `falhou` (a Mensagem diz a etapa: undock, navegacao ate a staging, docking, alinhamento); `pulado` = pose placeholder `[0,0,0]`. |
| `Undock/Nav/Align (s)` | Tempo de cada etapa. Nav alto = Nav2 sofrendo para chegar na staging (caminho, keepout, AMCL); Align alto = muitas correções (face mal vista, yaw torto, retry). |
| `Long/Lat (cm)`, `Yaw (graus)` | Onde o robô parou **em relação ao ponto do dock, no frame do dock** (TF `map → base_footprint`, AMCL): Long + = passou do ponto (mais perto da mesa), Lat + = à esquerda, Yaw + = girado à esquerda. |
| Detalhes → Staging | O mesmo erro na pré-docking, logo após o Nav2 — separa "o Nav2 não chega" de "o align desloca". |
| Detalhes → Align reportou | O que o `dock_align_node` mediu: `final_x_error` = front − stop pelo LiDAR (+ = longe da mesa), `final_y_error` (lateral, AMCL), `final_yaw_error` = yaw da face. |

Regra prática: erro **sistemático** (mesmo sinal toda vez que visita aquele dock) = pose gravada
deslocada → regrave (`save_service_area_pose` + `validate_service_areas --sync-docking`); erro que
**varia** entre visitas ou tempo alto = ponto difícil (AMCL degrada ali, staging perto de parede ou
vizinho) → ajustar `approach_offset`, keepout ou a pose. Com `--no-align` o erro mostra o que o
opennav sozinho entrega (sem o align corrigindo por cima).

### 2.2 Próximo tour: o que gravar

Os dois tours de 2026-10-26 não tinham bag e os logs do Nav2 não trazem pose
(`docs/diagnostico_docking_tour_2026-10-26.md`, seção 8). Sem `/scan`, `/tf`, `/odom` e
`/amcl_pose` não se decide "base ainda girando" × "AMCL atualizando tarde", nem o que o
LiDAR vê na WS6/WS2, nem a folga real nas passagens. Antes do próximo tour (ou de repetir
WS1/WS2/WS3/WS6 individualmente):

```bash
# 1. Subir o stack com a saída de TODOS os processos também em arquivo (além da tela):
#    fica em ~/.ros/log/<data>-<host>-<pid>/launch.log + um <nome>-<n>-stdout.log por nó,
#    com a hora do launch — serve para correlacionar dock_align_node (python) com
#    controller_server/amcl (C++), que já deixam log por PID em ~/.ros/log de qualquer jeito.
OVERRIDE_LAUNCH_PROCESS_OUTPUT=both ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=arenalegal_2026

# 2. Terminal 2: gravar o bag (mcap + zstd, sai em ~/bags/<stamp>_tour_<mapa>; mesma lista e
#    compressão do rosbag por corrida do §2.3). --extra acrescenta scan_raw, particle_cloud e o
#    costmap global (o que a versão de 10/2026 gravava sempre, sem compressão). O comando é
#    impresso no início; Ctrl-C fecha o bag e imprime `ros2 bag info`.
ros2 run caramelo_navigation gravar_bag_tour --mapa arenalegal_2026 --extra   # ou: gravar_bag_tour ws6_3x --mapa ...
#    (BAG_DIR=/outro/lugar muda a pasta; fonte: src/caramelo_navigation/scripts/gravar_bag_tour.sh,
#     que chama o caramelo_bag_common.py)

# 3. Terminal 3: o tour (ou --only WS6 repetido 3x) como no §2.1.
ros2 run caramelo_navigation tour_docks --map-name arenalegal_2026 --yes

# 4. No fim, junto com o bag e o tour_<data>.md: trena nos cantos dianteiros → face
#    (yaw físico) e centro do robô → centro do tampo (Y físico) nas WS1/2/3/6 e numa boa
#    (WS4/WS5). É a única forma de arbitrar AMCL × LiDAR, que discordam 14–25 cm / ~20°.
```

Nível de log: o `ros2 launch` não tem flag global de `--log-level`, e os nós do Nav2 não
sobem com o serviço `set_logger_levels`. Para ver DWB/RotationShim (só falam em DEBUG) ou o
AMCL passo a passo, adicione `arguments=["--ros-args", "--log-level", "<nó>:=debug"]` no
`Node(...)` correspondente (`dock_align_node` em `docking_server.launch.py`; `amcl` em
`caramelo_localization/launch/global_localization.launch.py`; `controller_server` em
`caramelo_navigation/launch/navigation.launch.py`) **só para a sessão de teste** — em DEBUG o `controller_server` a 20 Hz
enche o log em minutos. Em INFO já aparecem "Failed to make progress", "Reached the goal",
"clear entirely the local_costmap" e as linhas do `dock_align_node`.

O que conferir depois (T8 do diagnóstico, com o `PoseProgressChecker` de 2026-10-26):

```bash
grep -c "Failed to make progress" ~/.ros/log/controller_server_*.log      # esperado 0 nas pernas
grep -c "clear entirely the local_costmap" ~/.ros/log/*.log               # esperado 0 durante as pernas
grep -c "Running backup" ~/.ros/log/behavior_server_*.log                 # esperado 0 durante as pernas
ros2 bag info ~/bags/<stamp>_tour_<mapa>                                  # todos os tópicos com mensagens
ros2 run caramelo_navigation bag_resumo ~/bags/<stamp>_tour_<mapa>        # taxa do laço, relógio, caranguejo (§2.3)
```

### 2.3 Rosbag por corrida

Desde 2026-11-11 toda missão — `run_mission` no terminal ou GUI/painel web via `mission_server`
— grava um rosbag sozinha, do fim do pre-flight ao fim do executor (também com Ctrl-C, cancel
ou falha). A lógica mora em `src/caramelo_navigation/scripts/caramelo_bag_common.py`
(`iniciar_bag`/`parar_bag`); o desenho veio da pesquisa de 10/11 (rosbag2 0.26.11, mcap, zstd).

**Desde 2026-11-13 a corrida é gravada inteira** (`caramelo_run_common.py`,
`iniciar_run`/`parar_run`), numa pasta `runs/run_YYYYMMDD_HHMMSS/` na raiz do workspace (fora do
git; `CARAMELO_RUNS_DIR` muda; limpeza > 40 GB/40 corridas, `.keep` preserva):

| Item | O que é |
|---|---|
| `rviz_video.mp4` | A janela do RViz que o `robot_manipulation.launch.py` abre com `use_rviz:=true` (título com `global_localization.rviz`; `CARAMELO_VIDEO_JANELA=<trecho>` escolhe outra), como o operador a vê. ffmpeg x11grab 10 fps, VAAPI da Intel (~10–15 % de 1 núcleo). Hora local `HH:MM:SS.mmm` carimbada em cada quadro. A janela pode ficar atrás de outras; **não minimizar nem redimensionar** durante a corrida. Sem esse RViz aberto, a corrida grava o resto e o `metadata.json` diz por quê. |
| `rosbag/` | O mesmo bag de sempre (lista abaixo), mais `/diag/*`. |
| `logs/` | Trecho de `~/.ros/log` escrito durante a corrida, `rosout.log` (com hora local e t_bag), `eventos.csv`, saídas do ffmpeg/nó de diagnóstico/análise. |
| `config/` | YAMLs, BTs e launch da navegação, `amcl.yaml`, YAMLs do mapa, task e actions, `git_status.txt` + `git_diff.patch`, `params/` (`ros2 param dump` dos nós do Nav2 no início). |
| `metadata.json` | Horário, task, mapa, perfil, git (rev + sujo), resultado do executor, epoch do 1º quadro do vídeo. |
| `analysis.md` | Gerado 1–3 min depois do fim (`analisar_run`, nice 19): linha do tempo, pernas (giro %, lateral %, replans, desvio do plano, laço Hz), anomalias com hora local / t_bag / t_video, logs WARN+ agrupados, relógio, **hipóteses não confirmadas** e como confirmar. |

Cores no RViz do robô: **azul** `/plan`, **verde** `/local_plan`, **amarelo** trajetória
percorrida (`/diag/trilha_real`), **azul claro** plano anterior quando um replan muda a rota,
**laranja** `/expansions` (só com `debug_visualizations` do SmacLattice, hoje desligado),
**vermelho** eventos (`/diag/eventos`: WARN/ERROR dos nós de navegação e ações do collision
monitor, esfera na pose do robô). `/diag/*` vem do `run_diag_viz`, que sobe e morre com a corrida.

Corrida sem missão (goals pelo RViz, teste de docking): `ros2 run caramelo_navigation gravar_run
--task manual --mapa arena_extendida` até o Ctrl-C (`gravar_bag_tour` faz o mesmo). Reanalisar:
`ros2 run caramelo_navigation analisar_run runs/run_<stamp>` (também aceita bag antigo de `~/bags`).

- **Onde fica** (até 12/11; hoje é `runs/run_<stamp>/rosbag/`): `~/bags/<stamp>_<task>_<mapa>/`, com o mesmo `stamp` do
  `~/.ros/caramelo_missions/<stamp>_actions.yaml`. Dentro: `metadata.yaml`, `<pasta>_N.mcap`
  (um arquivo novo a cada 500 MB), `corrida.yaml` (task, mapa, actions, git, `nav_profile`,
  hora, hostname, lista de tópicos) e `record.log` (saída do `ros2 bag record`, mostra quais
  tópicos ele de fato assinou). mcap com `zstd_fast` por chunk e cache de 16 MB (numa queda
  perde no máximo ~10 s). Lista mínima: cadeia `/cmd_vel_nav → /cmd_vel_smoothed → /cmd_vel →
  /mecanum_controller/reference`, `/maxon/wheel_velocity`, `/odom`, `/odom/wheel`, IMU,
  `/joint_states`, `/amcl_pose`, `/scan`, `/tf`, `/tf_static`, `/plan`, `/local_plan`, costmap
  local, footprint, `/collision_monitor_state`, `/rosout`, `/caramelo/mission/status` e o
  feedback/status de `navigate_to_pose`, `follow_path`, `dock_robot` e `align_to_dock`
  (`TOPICOS_MIN`). Estimativa: ~60 MB/min brutos, bem menos comprimido. **Pendente**: medir
  na primeira corrida real se o laço do controller piora (`grep -c "missed its desired rate"`
  no log do `controller_server` com e sem bag) antes de confiar nele em competição.
- **Como desligar**: `--no-bag` no `run_mission`; `bag_enabled:=false` no
  `mission_server.launch.py`; ou `CARAMELO_BAG=0` no ambiente (vale para os dois caminhos).
  Com menos de 10 GB livres a missão roda sem bag (aviso `[bag] ... GB livres`) — nunca
  aborta por causa do bag. `CARAMELO_BAG_DIR=/outro/lugar` muda a pasta raiz.
- **Mais dados, sob demanda**: `--bag-extra` (`bag_extra:=true`) acrescenta
  `/global_costmap/costmap`, `/particle_cloud`, `/transformed_global_plan`,
  `/received_global_plan`, `/scan_raw`, `/behavior_tree_log`, `/diagnostics`, `/speed_limit`
  (dobra o tamanho). `--bag-camera` (`bag_camera:=true`) sobe um `image_transport republish`
  raw → JPEG e grava `/manip/vision_debug/image/compressed` — assinar a imagem acorda o
  processamento do `vision_debug_node`, por isso é opt-in.
- **Limpeza automática**: ao iniciar cada corrida, apaga as pastas mais antigas de `~/bags`
  até ficar ≤ 40 GB e ≤ 40 corridas. `touch ~/bags/<corrida>/.keep` protege uma corrida
  importante.
- **Como ler**: `ros2 bag info ~/bags/<corrida>` e
  `ros2 run caramelo_navigation bag_resumo ~/bags/<corrida>` (duração, tamanho, mensagens e
  bytes por tópico; taxa real de `/cmd_vel_nav` = laço do controller, com intervalos > 100 ms;
  offset `header.stamp` × recepção em `/odom` e `/scan` = relógio Pi × PC, esperado |mediana|
  < 5 ms com chrony; e, por goal do `/navigate_to_pose`, a fração do tempo em movimento com
  |vy| > |vx| — o "caranguejo"; `--json` para scripts). Replay só com RViz, **nunca com o
  Nav2 no ar**: `ros2 bag play ~/bags/<corrida> --clock 100 --topics /tf /tf_static /scan
  /local_costmap/costmap /plan /cmd_vel_nav` + `rviz2 --ros-args -p use_sim_time:=true`.
  Análise própria em Python: `rosbag2_py` (modelo em `bag_resumo.py`; o campo `t` do
  `read_next()` é o instante de recepção no PC).
- **O que NUNCA gravar e por quê**: `/marker`, `/evaluation`, `/cost_cloud` (DWB) e
  `/trajectories`, `/optimal_trajectory` (MPPI) — o Nav2 só **constrói** essas mensagens quando
  há assinante, e o gravador viraria esse assinante dentro do laço de 20 Hz que já perde taxa
  (o DWB montaria 1680 trajetórias por ciclo só para o marker); `*/costmap_raw` e
  `*/costmap_updates` (vazios com `always_send_full_costmap: true`); imagem crua da câmera
  (~276 MB/min e acorda o `vision_debug_node`). A lista é `TOPICOS_NUNCA` no
  `caramelo_bag_common.py` e `montar_topicos` recusa qualquer um deles. Também não se usa
  `-a` nem `--qos-profile-overrides-path` (o record adota o QoS do publisher).
- **Parada**: SIGINT no gravador, espera até 30 s (ele fecha o `metadata.yaml`), SIGTERM de
  reserva, **nunca SIGKILL** (perderia cache e metadata). Pasta sem `metadata.yaml` (queda de
  energia): `ros2 bag reindex ~/bags/<corrida> mcap`. Testes: `test_bag_common.py` (sem
  ROS) e `CARAMELO_BAG_SMOKE=1 ... test_bag_common_ros.py` (fumaça em `ROS_DOMAIN_ID=99`).

## 3. Formato do YAML da missão

Crie/edite arquivos em `~/caramelo_ws/missions/` (fonte única; `missions/README.md`
lista os prontos). Subseções: 3.1 competição (antigo) · 3.2 **YAML 2026 do @Work Commander
(formato oficial, o principal)** · 3.3 empilhar · 3.4 container por cor · 3.5 superfície
livre · 3.6 AMT · 3.7 nível baixo. O `task_planner` detecta o formato sozinho; os três
convivem.

### 3.1 YAML da competição (formato antigo)

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

### 3.2 YAML 2026 do @Work Commander (formato oficial)

É o arquivo que a ferramenta oficial **@Work Commander (edição 2026)** entrega às equipes —
exemplos reais de todas as provas em `missions/tasks geradas/` (BMT, BTT1, BTT2, ATT1, ATT2,
AMT, FINAL; os mesmos arquivos são fixtures do teste automatizado, ver abaixo). O cabeçalho
de cada arquivo descreve o contrato; resumo real (ATT1, cortado):

```yaml
task_id: ATT1
name: Advanced Transportation Test I
active_service_areas: [ST, WS_1, WS_3, WS_5, WS_6, SH_1]   # PP e ST SEM sufixo
objects:                      # só o id (a COR dos ATTCs NÃO é informada)
- {id: 3}
- {id: 14}
- {id: 30}
containers:                   # sem id: cor + estação
- {color: RED, at: WS_3}
stacking_areas:
  ST: {reference_tag: 200}    # tag de referência da pilha (tags_36h11: tag_200)
precision_tables:             # só nas provas com PP
  PP: {slot_tags: [101, 102, 103, 104, 105, 106, 107]}
start_state:
  WS_1: {obj_ids: [3, 14], table: [14, 3]}
  WS_3: {obj_ids: [], table: [], containers: {RED: []}}
  ST:   {obj_ids: [], stack: []}
  SH_1: {obj_ids: [], levels: {BOTTOM: [], TOP: []}}
finish_state:
  WS_3: {obj_ids: [3, 7], table: [3], containers: {RED: [7]}}
  ST:   {obj_ids: [14, 30], stack: [14, 30]}            # base .. topo
  SH_1: {obj_ids: [27], levels: {BOTTOM: [], TOP: [27]}}
```

Cada área tem `obj_ids` (todos os objetos, sem significado de posição) **e** um sub-bloco
que diz ONDE eles estão; a união dos sub-blocos é igual a `obj_ids` (o planner usa a união e
avisa se diferir). O que o robô faz com cada sub-bloco:

| Sub-bloco | Significado | O que o planner gera |
|---|---|---|
| `table: [ids]` | soltos sobre a mesa, esquerda → direita | `pick`/`place` normais (a ordem esquerda→direita **não** gera ação) |
| `containers: {RED: [ids]}` | dentro do container daquela cor | `place` com `container_color: RED\|BLUE` na estação (§3.4); objeto que **começa** dentro de container vira pick normal pela tag (aviso) |
| `levels: {BOTTOM: [], TOP: []}` | prateleira `SH_*` | `pick`/`place` normais com `table_pose: MesaSh` + campo informativo `shelf_level: BOTTOM\|TOP` |
| `slots: {101: id, 102: null}` | mesa de precisão `PP` | PP como **destino** (vindo de outra estação): `place` com `at_tag: tag_<slot>` — chegada em paradas com a câmera medindo o furo ([pp_furo.md](pp_furo.md), refeita em 2026-11-16); PP como **origem**: pick normal pela tag do objeto. Trocar de slot dentro da própria PP fica fora do plano (aviso) |
| `stack: [base, …, topo]` | pilha sobre a tag de `stacking_areas.<ST>.reference_tag` | base: `place` com `stack_on: tag_200`; cada seguinte: `stack_on: tag_<anterior>` (§3.3), na mesma visita. Pilha que **já existe** no `start_state` é desfeita antes: quem está em cima sai primeiro e o `place` na tag/base espera o ocupante ser pego (o planner reserva lugar a bordo para isso) |

Mais regras do formato:
- **Detecção**: presença de `precision_tables`/`stacking_areas`, container sem `id` ou qualquer
  sub-bloco acima. O log mostra `formato 2026 do @Work Commander detectado`.
- **Estações sem sufixo** (`PP`, `ST`): o planner procura no `ws_table_mapping.yaml` a chave
  exata e, se não houver, o **único** candidato `PP_<n>`/`ST_<n>` (`PP` → `PP_1: Mesa10`,
  `ST` → `ST_1: Mesa15`); o executor faz o mesmo para o dock (`ws_to_dock_id`). Zero ou dois
  candidatos = erro dizendo o que acrescentar. A fila de alcance (§5.2) usa os prefixos
  `WS`/`PP` (sem `_`), então `PP` entra nela.
- **Containers**: só `finish_state.<WS>.containers.<COR>` manda para container (sem inferência
  por cor do objeto — o objeto não tem cor). Cor precisa ser RED/BLUE (detector).
- **Tags**: ids 1..30 (`tag_<id>`), 101..107 (`tag_101..tag_107`, slots) e 200 (`tag_200`,
  referência da pilha) estão em `tags_36h11.yaml`; id sem frame é **erro** do planner.
- **AMT**: `task_id: AMT` / `name: Advanced Manipulation Test` → `goto PP` + `amt1_sort` com os ids do
  arquivo (`expected_tags`) e a ordem pedida (`target_order`, de `finish_state.PP.slots`; nos
  arquivos da competição é a ordem crescente). Ver §3.6.
- Mesmo objeto em outro nível da prateleira (mesma estação) também é pick + place. Objeto só no `start_state` vira aviso (não é pego); só no `finish_state`
  é erro.

**Limitações (ainda sem validação no robô)**: `shelf_level` é só informativo (o pick/place da prateleira ainda
não escolhe o nível — aviso `nivel da prateleira ainda nao e' usado` no executor); o
**tamanho das tags 101..107/200** em `tags_36h11.yaml` (0,041) é chute: conferir com o
rulebook/mesa real; a pose do dock **ST1** ainda não existe no `docking.yaml`/`service_areas.yaml`
do `arenalegal_2026` (o executor aborta de forma clara); uma pilha que já existe no
`start_state.<ST>.stack` é desfeita na ordem certa (ocupante da tag_200/base é pego antes do
`place`, com lugar a bordo reservado para ele) e, se o ocupante não tiver destino, o planner
falha com `ocupa esse lugar e nao sera pego` em vez de gerar um plano impossível — os
arquivos oficiais sempre começam com `stack: []`, então isso só foi testado no planner
(`test_task_planner_formato2026.py`, casos `START_STACK_CASES`), não no robô.

**Teste automatizado** (sem ROS no ar, roda o binário sobre os 7 arquivos + fixtures antigos):
```bash
colcon test --packages-select manip_bt --ctest-args -R formato2026   # após o build
# ou, com o install sourceado (acha o binário por `ros2 pkg prefix manip_bt` ou TASK_PLANNER_BIN):
python3 -m pytest src/manipulation/manip_bt/test/test_task_planner_formato2026.py -q
```

#### Formato "novo 25/08" (anterior, ainda aceito)

```yaml
schema_version: 1
task_id: BTT2
version: equipe            # "equipe" = sem cores; a versão do robô traz a cor no type
active_service_areas: [WS_1, WS_2, WS_3, WS_6]
objects:
- {id: 4, type: ATTC}      # place na mesa
- {id: 8, type: ATTC_RED}  # cor no type = so' a cor da peca; place na mesa tambem
start_state:
  WS_2: {obj_ids: [13, 4]}
finish_state:
  WS_6: {obj_ids: [30, 13, 4]}
```

- Não há `containers:`/`cont_ids` nem tag nos containers. **A cor ao lado do tipo
  (`ATTC_RED`, `ATTC-blue`...) é só a cor da peça: não manda para o container daquela cor**
  (regra antiga, removida em 2026-11-19). Formato antigo **com** `containers:` + "inside": só os
  objetos das constraints vão para container (pela COR do container, que precisa ser RED/BLUE);
  os outros vão para a mesa.
- Objeto sem frame em `tags_36h11.yaml` é **erro** do planner (antes sumia do plano em
  silêncio). Objeto só no `start_state` vira aviso (não é pego); só no `finish_state` é erro.
  `type` com "DECOY" é ignorado. `stacks:` continua (§3.3).

### 3.3 Empilhar

#### No YAML da competição (`stacks:`)

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
`stack_on: tag_4` no place do topo — ver abaixo o que o place faz com isso.
Exemplo pronto (já traduzido para o nível baixo, formato de ações):
`missions/exemplos/empilhar_ws4_ws6_actions.yaml`. Casos inválidos (ciclo,
objeto que não termina nessa estação, topo com duas bases) abortam o planner
com mensagem clara.

#### No nível baixo (`stack_on`)

No nível baixo (`actions:`) — é o que o `task_planner` emite a partir de `stacks:`
(acima). No `place` do objeto de CIMA, acrescente `stack_on: tag_<id da base>`; a base
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

Exemplo pronto: `missions/exemplos/empilhar_ws4_ws6_actions.yaml`.
O que o place faz com `stack_on`: vê a tag base pela câmera (a partir de
`pegar_obj`, com o bloco na garra), vai para (x, y) dela com o TCP em
`z_base + stack_place_z_offset` (parâmetro do `place_action_server`, 0,05 m =
cubo de 42 mm + folga — **calibrar no robô**), passando por uma pré-pose
`stack_pre_lift_m` (0,05) acima e descendo na vertical; depois de soltar sobe
de volta. Parâmetros: `stack_place_z_offset`, `stack_pre_lift_m`,
`stack_arrival_tolerance_m` (0,03), `stack_fallback_to_table` (true),
`stack_tilt_ladder_deg` ([15, 30]).

#### Formato 2026: pilha sobre a tag 200

No YAML 2026 a pilha começa sobre o adesivo `tag_200` da área de empilhamento
(`stack_on: tag_200` no primeiro cubo, depois `stack_on: tag_<anterior>`): pilha
de até 4 com verificação após cada cubo. A mesa de precisão (`at_tag`) foi
refeita em 2026-11-16: ver [pp_furo.md](pp_furo.md). Parâmetros e limitações da pilha em
[place_precisao_e_pilha.md](place_precisao_e_pilha.md) — **ainda sem validação
no robô**.

### 3.4 Container por cor (`container_color`)

> Manutenção completa (trocar container, cor, garra; que arquivo/parâmetro mexer):
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

### 3.5 Place em superfície livre

Em mesa comum (`WS_*`) o place agora **olha o tampo** com a RealSense antes de soltar e escolhe um
ponto que cumpre o rulebook 4.4 (2 cm de bordas e objetos, 10 cm de parede, até 20 cm da frente),
em vez das 3 poses fixas `MesaX/.1/.2` — que continuam como fallback. Paredes por estação no
`ws_table_mapping.yaml` do mapa (`ws_walls: {WS_3: [left]}`) ou por ação (`wall_sides: [right]`).
Kill-switch: `place_surface_planner:=false`. Detalhes, parâmetros e roteiro de validação:
[docs/place_superficie_livre.md](place_superficie_livre.md). **Ainda sem teste no robô.**

### 3.6 AMT — ordenar os blocos da mesa de precisão (refeita em 2026-11-16, rodadas em 11-17)

Os blocos começam nos slots da PP em ordem desconhecida (`start_state.PP.slots: null`) e têm de
terminar com os ids do arquivo em **ordem crescente da esquerda para a direita** (slots 101, 102,
…). O `task_planner` reconhece a task pelo `task_id`/`name` e emite `goto PP` + uma ação
`amt1_sort`. O executor roda **rodadas** (parâmetro `amt1_rounds`, 4): em cada uma o braço passa
por `tag_esquerda`, `tag_direita` e `pegar_obj` (`amt1_look_poses`), chamando o servidor
`/amt1_sort` em cada pose — nas duas primeiras só para olhar (`observe_only`), na última para
ordenar. A rodada só termina bem quando o servidor **vê a mesa em ordem sem ter nada a fazer**;
depois de mexer em bloco ele responde `falta_conferir` e a rodada seguinte confere e arruma o que
faltou. Esgotadas as rodadas a missão segue para o FINISH. O servidor é o `amt_sort_node.py`
(`manip_bringup`, conta em `amt_sort_core.py`):

1. descobre **quem está em qual slot** só por ids de tag. Nas poses laterais (mais altas) a tag
   do slot ocupado aparece **colada no bloco**: o bloco está nesse slot. De `pegar_obj` o bloco
   tapa a tag do slot e vale a **ordem dos blocos na fila** (vizinhos a ~3,45 lados de tag),
   emendada com as outras olhadas pelos blocos em comum. Tag de slot sozinha, longe da borda da
   imagem = slot vazio. Sem nenhuma tag de slot para dizer onde a fila começa, ou com leitura
   contraditória, **não mexe em bloco**: a rodada falha e olha de novo;
2. planeja as trocas usando os potes de bordo como espera (só mexe em quem está fora do lugar;
   bloco que já estava num pote, pelo `container_states.yaml`, entra no plano);
3. executa com `/pick_tag` e `/place_tag` (`stack_on` = tag do slot: o place da PP, com a câmera
   medindo o furo — [pp_furo.md](pp_furo.md)). A base fica onde está e só anda de lado
   (`/nudge_base`) quando a operação pede: bloco fora de alcance no pick, **slot fora do eixo do
   robô no place** (`pp_alinhar_base_m`, 8 cm), ou alvo não visto com a base longe dele. No fim
   de cada rodada a base volta ao centro, de onde as olhadas cobrem a fileira;
4. se uma operação falhar, devolve à mesa o que estiver a bordo;
5. antes de cada operação confere se o `/joint_states` está chegando; se o braço sumiu (rede da
   Raspberry), espera até `braco_espera_s` (10 min), avisando a cada 30 s, e retoma na mesma
   operação quando ele voltar; se não voltar, para e avisa.

O nó não move o braço. Parâmetros: `observar_s` 2.5, `min_quadros` 4, `max_nudges` 2,
`desvio_max_m` 0.45, `busca_passo_m` 0 (a busca andando a base só é usada na bancada, que tem uma
vista só). Resultado no log: `[AMT] ordem final [...]`; fala "Conferi, a mesa de precisão está em
ordem" só com a mesa vista em ordem.
Bancada sem robô: `src/manipulation/manip_bt/test/run_amt_scenario.sh` (arquivo da competição →
planner → executor → nó de verdade → mesa falsa). Só observar, sem mover bloco:
`ros2 action send_goal /amt1_sort my_robot_msgs/action/Amt1Sort "{ws: PP, table_pose: Mesa10, expected_tags: [1,2,3,4,5,6], observe_only: true}"`.

### 3.7 Nível baixo (YAML de ações, sem planejador)

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
  -p map_folder:=$HOME/caramelo_ws/src/caramelo_mapping/maps/arena2_520
# uteis: --dry-run | -p simulate_navigation:=true | -p finish_dock_id:="''"
```

O undock antes de sair de uma estação é automático (embutido no goto seguinte);
`home` antes de cada goto é responsabilidade de quem escreve as ações.

Desde 28/08 os picks/places de uma mesa comum (`WS_*`, `PP_*`) entram
automaticamente numa **fila de alcance** (`<ReachQueue>`, ver §5.2): o que voltar
"alvo visto mas fora de alcance" é adiado, a base se ajusta de lado e a fila
repete. Um `home` no meio da estação fecha a fila (o próximo pick/place da
mesma estação abre outra). `SH_*` e START/FINISH ficam fora. Para o XML
antigo, byte a byte: `-p reach_queue_enabled:=false`.

## 4. Arquivos por mapa

### 4.1 ws_table_mapping.yaml

Arquivo **canônico**: `src/caramelo_mapping/maps/<mapa>/ws_table_mapping.yaml` — é o que
o `run_mission` passa ao `task_planner` (4º argumento) e ao executor. **Edite sempre este.**
Não existe mais cópia de fallback no share do `manip_bt`: a antiga
`behavior_tree_manip/ws_table_mapping.yaml` virou fixture de teste
(`src/manipulation/manip_bt/test/fixtures/`, usada só pela bancada) e o `run_mission`
não procura `--task` nem o mapping fora da pasta do mapa — sem o arquivo no mapa ele
aborta antes de mover.

O arquivo tem DUAS chaves, lidas por partes diferentes do sistema:

```yaml
ws_to_table_pose:   # ALTURA DO BRACO em cada mesa (lido pelo task_planner)
  WS_1: Mesa0       # Mesa0/Mesa5/Mesa10/Mesa15 = poses do braco p/ 0/5/10/15 cm
  WS_2: Mesa5
ws_to_dock_id:      # qual DOCK do docking.yaml corresponde a estacao (lido pelo executor)
  WS_1: WS1
  WS_2: WS2
```

Trocou a altura física de uma mesa? Ajuste o `ws_to_table_pose` dela aqui. Opcional:
`ws_walls: {WS_3: [left]}` declara paredes por estação para o place em superfície
livre (§3.5).

### 4.2 Demais arquivos por mapa (a "carteira de identidade" da arena)

Tudo em `src/caramelo_mapping/maps/<mapa>/`:
- `docking.yaml` — pose e tipo de cada dock (**regravar pela GUI**: posicione o
  robô na distância ideal da mesa — pick validado com objeto a ~40 cm — e use
  "Salvar pose como dock" escolhendo a estação; vale na próxima missão).
- `service_areas.yaml` — pose final, use_docking e manipulation_enabled por área.
- `ws_table_mapping.yaml` — §4.1.
- `dock_distances.yaml` — comprimento de caminho entre os docks (gerado:
  `ros2 run caramelo_navigation compute_dock_distances --map-name <mapa>`); o `task_planner`
  usa para ordenar as estações por tempo. **Regerar** depois de regravar poses de dock ou
  trocar o mapa (`docs/planejador_por_tempo.md`).

Config das tags (ids→frames→tamanhos): `src/manipulation/manip_bringup/config/tags_36h11.yaml`
(vendorado — NÃO depende mais do pacote apriltag_ros de fora; o binário vem do ~/ros2_ws).

## 5. Executor: parâmetros, fila de alcance e memória de slots

### 5.1 Parâmetros do executor (`bt_yaml_executor`)

- `startup_home` (true): braço vai a `home` ANTES da primeira ação da missão.
- `startup_pose_name` ("home"): pose do prólogo.
- `max_tag_age_sec` (1.0, nos nós de pick/place): a tag precisa ter sido VISTA
  há menos de 1 s — protege contra alvo fantasma (TF velha com robô movido).
  Se o pick reclamar "a camera NAO esta vendo a tag agora", a tag está fora do
  campo de visão (não é erro de IK).
- Fila de alcance (28/08, ver §5.2): `reach_queue_enabled` (true),
  `reach_queue_ws_prefixes` (`["WS","PP"]`, sem `_` desde o formato 2026: cobre `WS_1` e `PP`), `reach_queue_max_nudges` (2),
  `reach_queue_max_shift_m` (0.25), `reach_queue_min_shift_m` (0.10),
  `nudge_timeout` (20 s, goal do `nudge_base`).
- **Gate de pose do `GoToWS`** (2026-10-26, runs de 22–23/10 — `docs/investigacao_incidente_2026-10.md`):
  depois do `align_to_dock` o executor confere o result (`final_x/y/yaw_error` e o prefixo da
  `message`: `yaw_fora_da_tolerancia` = yaw da face pelo LiDAR, `yaw_nao_medido` = yaw pelo AMCL)
  contra limites **duros**: `pose_gate_max_x` (0.10 m), `pose_gate_max_y` (0.20 m — meia-largura do
  chassi 0,195 + 0,20 ≤ meia-largura da mesa 0,40), `pose_gate_max_yaw` (0.20 rad = 11,5°, a
  tolerância de yaw do Nav2; o place por superfície erra ≤ 3 cm na zona com esse yaw). Fora dos
  limites: **re-docka** (`pose_gate_retries`, 1: undock → staging → align) e, ainda fora,
  `pose_gate_strict` (true: a estação falha e a missão pula para a próxima com a voz "Estação X
  torta"; false, **default até validar no robô**: segue com `POSE FORA DOS LIMITES` no log e a voz
  "Estação X torta, vou tentar assim mesmo"). `pose_gate_enabled:=false`
  desliga tudo. Linhas de log: `[WSn] pose validada` / `pose ACEITA COM RESSALVA` (dentro dos
  limites duros, acima da tolerância do docking) / `POSE FORA DOS LIMITES (...)`.
- **Base parada antes de manipular** (2026-10-26): com `stop_check_enabled` (true) o `GoToWS` só
  devolve SUCCESS quando `/odom` (`odom_topic`) fica abaixo de `stop_check_max_lin` (0.02 m/s) e
  `stop_check_max_ang` (0.05 rad/s) por `stop_check_window_s` (0.5 s) contínuos, com amostra
  mais nova que `stop_check_odom_max_age_s` (0.5 s). Não é atraso fixo: sem odometria fresca
  até `stop_check_timeout_s` (3 s) a estação **falha** (`stop_check_require_odom`, true; false =
  aviso e segue); em movimento após o timeout, falha com o diagnóstico "quem mais publica em
  `/mecanum_controller/reference`?". Log: `[WSn] base parada: 0.6 s abaixo de ...`. Limites bem
  abaixo do piso do ESC (0,1215 m/s / 0,315 rad/s); a janela cobre a retenção da referência
  (~0,5 s). Resumo por estação no fim: `[WSn] resumo da estacao (WS_n): nav_staging 22.9 s |
  align 11.5 s | parada 0.6 s | total ...; dockings 1; pose x=.. y=.. yaw=.. (lidar|amcl)`.
  Bancada: `src/manipulation/manip_bt/test/run_pose_gate_scenario.sh all` (fakes em
  `ROS_DOMAIN_ID=99`, 8 casos).
- **Base parada antes do align** (2026-10-26, tours de 26/10 — `docs/diagnostico_docking_tour_2026-10-26.md`,
  F14/C2): o `general_goal_checker` do Nav2 aceita a staging sem olhar velocidade e a base ainda
  gira 0,2–0,5 rad/s por ~1 s depois do "Reached the goal"; o align partia nesse transitório e o
  esquadro por AMCL lia o erro mudando ~15°/s. Agora o `GoToWS` tem a etapa `parada_staging`
  entre o Nav2 (ou o `/dock_robot` do rollback `use_opennav_approach`) e o `align_to_dock`, com o
  mesmo detector e os mesmos `stop_check_*` da parada pós-align. Diferença de política: no
  timeout (`stop_check_timeout_s`) só **avisa e segue** para o align (o align comanda a base de
  qualquer forma; o gate duro continua sendo a parada pós-align). `stop_check_enabled:=false`
  desliga as duas. Log: `[WSn] base parada em 1.1 s apos a staging (yaw mudou +6.3 graus; ...)`
  ou `base AINDA EM MOVIMENTO 3.0 s apos a staging ... sigo para o align`; a parada pós-align
  também passou a informar `yaw mudou ... desde o align`. No resumo da estação aparece
  `parada_staging 1.1 s`.

- Ordem das juntas na pega (28/08, pedido do operador): na aproximação por cima o pick
  gira **j1 primeiro** (log `mesa custom down [so j1]`) e só depois desce o resto
  (`[resto]`); na volta a `pegar_obj` com o bloco, recolhe j2..j5 (`[j1 parado]`) e gira
  **j1 por último** (`[j1 por ultimo]`) — o braço estendido nunca varre a mesa girando.
- **Onde a garra fica larga** (16/11, pedido do operador): `gripper_full_open` só no `home`, ao
  empilhar e ao olhar o container da mesa (`ct_*`) de garra vazia. No resto é
  `gripper_open`: pick, soltura em mesa comum, prateleira e container da mesa, e devolução ao
  pote de bordo. No place a escolha sai do goal: `stack_on` (pilha) ou `container_color`.
- **Garra larga ao chegar em `pegar_obj`** (29/08; **desligada por padrão desde 16/11**): com o
  parâmetro preenchido, em mesa comum, assim que o braço chega
  em `pegar_obj` a garra vai para `gripper_full_open` (estágio `gripper_full_open_at_pegar_obj`)
  antes do ciclo de pega; na prateleira não (dedos esbarrariam nas tábuas). Parâmetro
  `pick_gripper_pose_at_pegar_obj` (default vazio = desligado; `gripper_full_open` religa). Se a pose não
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

### 5.2 Fila de alcance por estação (`ReachQueue` + `nudge_base`)

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

### 5.3 Memória de slots

A cada place bem-sucedido o nó grava em `~/.config/caramelo/container_states.yaml` (chave
`table_slots`, por WS) onde a ponta soltou o bloco: slot (`Mesa15.1`, `superficie`, `stack:tag_4`,
`ct_vermelho`), tag, x/y/z em `manip_base_link` (no frame da pose de docking) e o instante. Na
próxima passada esses pontos entram como obstáculos virtuais (`memoria:<tag>@<slot>`, +2 cm de
folga) para o caso de a câmera não ver mais a tag. **Sintoma antigo**: a mesma tag entrava duas
vezes (TF + memória, em posições um pouco diferentes pelo erro de re-docking) e o robô "via mais
tags do que existem"; o paliativo era `slot_memory_ttl_sec: 9`. Agora a câmera vence (log
`memoria: tag_N@slot substituida pela deteccao da camera`) e o registro de uma tag é **apagado**
quando: o pick confirma a pega (`memoria: tag_N removida (bloco pego)`), o place a solta em outro
lugar (antes de gravar o novo), o launch reseta o yaml no início da rodada, ou vence o TTL.
Parâmetros do `place_action_server`: `slot_memory_enabled` (true) e `slot_memory_ttl_sec`
(600 s — só protege contra registro órfão). Para zerar à mão, apague a chave `table_slots`.

## 6. Problemas comuns (aprendidos na validação)

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
- **Voz**: Kokoro (venv `~/caramelo_tts_venv`, `manip_audio/scripts/instalar_kokoro.sh`)
  → Piper (`~/piper/piper`) → `espeak-ng`, nessa ordem de fallback; `ffplay` toca.
  NUNCA `apt install piper` (é um app de mouse gamer). Trocar voz/motor/efeito:
  `kokoro_voice`, `speech_backend`, `voice_effect` no launch. Frases descartadas
  aparecem no log como `[VOZ] descartada`. [voz_do_robo.md](voz_do_robo.md).
- **Script Python novo em install(PROGRAMS) precisa de `chmod +x` no src** —
  com `--symlink-install`, sem o bit de execução o launch falha com
  "executable not found on the libexec directory".
- Robô **sem voz** neste PC: nem Kokoro, nem Piper, nem espeak-ng
  (`sudo apt install espeak-ng` habilita a reserva; o Kokoro é o script acima).
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

### 6.1 Ver o que a câmera enxerga

A imagem crua (`/camera/camera/color/image_raw`) não mostra nada do que a percepção
entendeu. O nó `vision_debug` (sobe por padrão no `manip_stack`, `use_vision_debug`)
publica a mesma imagem **anotada** em `/manip/vision_debug/image` — e não gasta nada
enquanto ninguém olha (só processa com um espectador, até 10 Hz).

```bash
# num terminal, com o stack no ar:
ros2 run rqt_image_view rqt_image_view /manip/vision_debug/image
# ou já aberto pelo launch:
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=<mapa> use_vision_viewer:=true
```

O que aparece por cima da imagem:

| Desenho | Significado |
|---|---|
| Contorno **verde** com ponto no centro, `tag_<id>` e `m=<margem>` | AprilTag vista agora (`/detections`); a margem de decisão maior = leitura mais firme. |
| Contorno **amarelo** | Tag lida com bit corrigido (`hamming > 0`) — menos confiável. |
| Caixa **vermelha** / **azul** com a distância | Container por cor (`ct_vermelho` / `ct_azul`): borda nominal 170 × 105 mm (traço grosso) e fundo 70 mm abaixo (traço fino). |
| Tinta **verde** / **vermelha** / **cinza** no tampo | Mapa da mesa (`/manip/workstation_surface/grid`) projetado de volta: tampo livre / objeto / fora da mesa. Sem tinta = desconhecido. |
| Barra no topo | `PICK` / `PLACE` / `observando mesa`, ids das tags vistas, containers e distância, idade do mapa (e quantas células livre/objeto/fora), última frase falada, fps real e tempo de processamento. `tf=latest` = TF da câmera sem o stamp da imagem (braço se movendo). |

Limitações (por desenho, não por defeito): tags só aparecem **enquanto vistas**
(0,5 s de idade); containers e mapa só existem **durante o place** (o
`container_detector` publica TF só com `/manip/place_active` e só enquanto vê o pote,
1 s de idade; o mapa é latched e fica desenhado até 10 s após a última fusão). Fora
do place a imagem mostra só a câmera, as tags e a barra. Ajustes ao vivo:
`ros2 param set /vision_debug grid_alpha 0.5` (ou `rate_hz`, `grid_max_age_s`...).

## 7. Histórico de mudanças

Datas que estavam nos títulos das seções (a cronologia completa fica no git):

- **2026-07-22** — primeira missão completa nav + manipulação no robô (§1).
- **2026-07-23** — runbook 100 % terminal (§1.1); parâmetros `startup_home`,
  `startup_pose_name`, `max_tag_age_sec` (§5.1).
- **2026-08-24** — empilhar: `stacks:` no YAML da competição e `stack_on` no nível baixo (§3.3).
- **2026-08-25** — YAML novo do @Work Commander (§3.2); container por cor sem tag (§3.4).
- **2026-08-28** — AMT1 (§3.6); fila de alcance `ReachQueue` + `nudge_base` (§5.2);
  busca lateral com a base; ordem das juntas na pega (§5.1).
- **2026-11-11** — rosbag automático por corrida (`caramelo_bag_common`, `bag_resumo`,
  `gravar_bag_tour` reescrito sobre o módulo) (§2.3).
- **2026-08-29** — varredura só com a j1, precisão do place (mediana, re-registro local,
  verificação), garra larga em `pegar_obj`, subida vertical pós-pega, re-detecção após a j1,
  recuo antes do lateral (§3.6, §5.1).
- **2026-08-31** — observação `tag_direita → tag_esquerda`, place por juntas gravadas, yaw
  de cubo normalizado, pré-ponto TCP2 no pick e no empilhamento (§3.6).
- **2026-10-19** — place em superfície livre (§3.5); memória de slots com TTL 600 s e
  dedup câmera × memória (§5.3); `nudge_base` não se movia (piso do ESC) — correção no
  `dock_align_node` (`docs/docking_alinhamento_holonomico.md`).
- **2026-10-21** — visão de depuração `/manip/vision_debug/image` (tags, containers, mapa
  da mesa e status sobre a imagem da câmera; §6.1); YAML 2026 do @Work Commander como
  formato principal (sub-blocos table/containers/levels/slots/stack, `PP`/`ST` sem sufixo,
  `at_tag`, `shelf_level`, `target_order` da AMT; §3.2, §3.6) com teste automatizado sobre
  os arquivos oficiais; pilha já existente no `start_state` desfeita antes do `place` na
  tag/base (revisão do mesmo dia).
- **2026-10-26** — `tour_docks`: tour de conferência de todos os docks com a sequência do
  GoToWS, voz e relatório `tour_<data>.md` por mapa (§2.1); pré-flight do `run_mission`
  compartilhado (`wait_for_action_servers` em `caramelo_docking_common`).
- **2026-10-26** — ordem das estações por tempo: `dock_distances.yaml` por mapa
  (`compute_dock_distances`, §4.2) e branch-and-bound no `task_planner` com o guloso como
  limite (`docs/planejador_por_tempo.md`); sem a matriz, guloso de sempre.
- **2026-10-26** — investigação das runs de 22–23/10 (`docs/investigacao_incidente_2026-10.md`):
  gate de pose e checagem de parada no `GoToWS` (`pose_gate_*`, `stop_check_*`; §5.1), re-dock
  automático uma vez, resumo de tempos por estação; cenário de bancada
  `run_pose_gate_scenario.sh`.
