# caramelo_gui

Interface gráfica de operação do Caramelo no PC: aplicação Qt5/C++ com RViz
embutido. Abre num **menu inicial** (Operação, Competição, Mapas, Mapeamento,
Modo Avançado); o mapa aparece nos contextos que precisam dele. A GUI é só um
**cliente** do grafo ROS 2 — tudo que ela faz também pode ser feito por
terminal. Guia completo (telas, interfaces, o que falta validar):
[docs/interface_grafica.md](../../docs/interface_grafica.md).

## Como rodar

```bash
source ~/caramelo_ws/install/setup.bash
ros2 launch caramelo_gui caramelo_gui.launch.py
```

| Arg | Default | Efeito |
|---|---|---|
| `use_diagnostics` | `true` | Sobe o `caramelo_diagnostics/diagnostics.launch.py` (`health_monitor` → `/diagnostics`, cartões de saúde). `false` se já estiver no ar por outro launch. |
| `use_mission_server` | `true` | Sobe o `caramelo_navigation/mission_server.launch.py` (action `/caramelo/run_mission`, tela Competição). Um por robô: o nó sai sozinho se já houver outro no grafo. |
| `use_sim_time` | `false` | `true` em simulação — a GUI precisa do mesmo relógio do resto (senão o RViz descarta TF em rajada e satura a CPU). |

O executável é `caramelo_gui` (sem `name:=` no launch: o processo contém mais de
um nó interno e o remap de nome único os colidia).

## AVISO: GUI com `robot_manipulation.launch.py` no ar

O stack de competição sobe por

```bash
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=<mapa>
```

e o Nav2 **já está rodando**. A seção Operação → Navegar detecta o `bt_navigator`
e desliga o botão "Ligar navegação"; ainda assim, use a GUI como monitor,
para localizar (fantasma → `/initialpose`), para rodar a missão (Competição) e
para comissionar arenas. "Iniciar SLAM" com a localização ativa só **avisa**
(RK-02) e sobe mesmo assim — derrube o Nav2 antes de mapear.

## Telas

| Contexto | Seções | Resumo |
|---|---|---|
| **Menu inicial** | — | cartões com estado/motivo, arena ativa, *Parar o robô* (cancela navegação + aborta missão; não é E-stop), *Levar para a base*, *Diagnóstico* |
| **Operação** (com mapa) | Mapa, Navegar, Waypoints, Docking, Localização, Teleop | meta pelo mapa, Nav2 (se ainda não estiver no ar), waypoints, `/dock_robot` / `align_to_dock` / `/undock_robot` / `/save_dock_pose`, fantasma → `/initialpose`, `teleop.launch.py` |
| **Competição** | Missão | task (`missions/` e `missions/tasks geradas/`, formato 2026) + mapa → *Ver plano* → *Verificar servidores* → *EXECUTAR* / *ABORTAR*; estado, última linha do executor e voz do robô (`/manip/speech_status`) |
| **Mapas** | Arenas | escolher a arena ativa, preview com docks/áreas/paredes/waypoints e avisos, criar/arrastar/remover pontos (scripts com `--pose`), limpar ruído, paredes virtuais, renomear/duplicar/excluir |
| **Mapeamento** (com mapa) | Mapeamento | 1 criar o mapa (SLAM + teleop + salvar) → 2 marcar pontos (pose do robô ou clique) → 3 paredes virtuais → 4 conferir |
| **Modo Avançado** (com mapa) | Estado do robô, Sensores, Service areas | cartões do `/diagnostics`; LiDAR/IMU/câmera (prefere `/manip/vision_debug/image`, a câmera do punho anotada); salvar/listar/validar service areas |

Faixa superior: `< Menu`, contexto, **Arena**, **estado** e o botão **Logs** (`/rosout`).

## Launches e scripts que a GUI roda como processos filhos

`ros2 launch`/`ros2 run` dentro de `QProcess` (`src/core/launch_runner.cpp`,
`src/modules/mapeamento/scripts_do_mapa.cpp`), derrubados com SIGINT.

| Onde | Comando | Args |
|---|---|---|
| Mapeamento passo 1 | `caramelo_mapping/slam.launch.py` | `use_rviz:=false` (+ `use_service_area_markers:=false` em mapa novo) |
| Mapeamento passo 1 | `caramelo_utils/teleop.launch.py` | — |
| Operação → Navegar | `caramelo_navigation/bringup.launch.py` | `map_name:=<arena> use_rviz:=false use_docking:=<bool> use_sim_time:=<bool>` |
| Mapas / Mapeamento | `init_service_areas --map <arena> --map-dir <maps> --areas START FINISH --sync-docking` | arena recém-salva |
| Mapas / Mapeamento | `init_keepout_mask --map-name <arena> --map-dir <maps>` | máscara em branco |
| Mapas / Mapeamento | `save_service_area_pose --map <arena> --name WS1 --type workstation --height h --overwrite [--pose x y yaw]` | sem `--pose` usa a pose atual do robô (TF) |
| Mapas / Mapeamento | `save_dock_pose --map-name <arena> --dock-id START [--dock-type t] [--pose x y yaw]` | idem |

## Interfaces ROS usadas

Nós do processo: `caramelo_gui_bridge` (rclcpp; `src/bridge/`) e `caramelo_gui_rviz`
(RViz embutido, `src/core/rviz_frame.cpp`). Os módulos em `src/modules/` não criam
publishers/subscribers próprios; usam o `RosBridge` e suas fachadas
(`MissionBridge`, `SensorBridge`, `ManualLocalization`, `WaypointManager`, `RobotState`).

| Tópico / serviço / action | Tipo | Uso |
|---|---|---|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | cartões (nomes `caramelo/*` do `health_monitor`) |
| `/rosout` | `rcl_interfaces/Log` | painel Logs |
| `/scan`, `/imu/data_raw` (ou `/imu`), `sensor_msgs/Image` | sensores | Sensores (só enquanto a tela está aberta); preview do scan na localização |
| `/caramelo_gui/goal_pose` | `geometry_msgs/PoseStamped` | ferramenta "Definir destino" (privado: `bt_navigator` também assina `/goal_pose`) |
| `/initialpose` | `PoseWithCovarianceStamped` | pose inicial para o AMCL |
| `/save_dock_pose` | `std_msgs/String` | regravar a pose de um dock (`dock_pose_recorder_node`) |
| `/caramelo/mission/status` | `caramelo_msgs/MissionStatus` (transient_local, 64) | progresso da missão |
| `/manip/speech_status` | `std_msgs/String` (latched) | voz do robô na tela Competição |
| `/caramelo/run_mission` | action `caramelo_msgs/RunMission` | missão (`mission_server`) |
| `navigate_to_pose`, `follow_waypoints`, `/dock_robot`, `/undock_robot` | nav2 actions | navegação, waypoints, docking |
| `align_to_dock` | action `caramelo_msgs/AlignToDock` | alinhamento fino (`dock_align_node`) |
| `/caramelo/service_areas/save_pose` | action `caramelo_msgs/SaveServiceAreaPose` | salvar pose atual como área |
| `/caramelo/service_areas/list`, `/validate` | srv `caramelo_msgs/{List,Validate}ServiceAreas` | listar / validar (+sync docking) |
| `/map_server/load_map`, `/map_saver_server/save_map` | nav2 srv | usar arena / salvar mapa do SLAM |
| `/global_costmap/clear_entirely_global_costmap`, `/local_costmap/...` | nav2 srv | limpar costmaps |

Interactive markers: `localizacao_manual` (robô-fantasma) e `waypoints`. TF: `map -> base`.

## Onde ficam os mapas e as tasks

`MapasModule::mapsDir()` resolve, nesta ordem: `CARAMELO_MAPS_DIR`; a árvore **fonte** do
workspace (`<ws>/src/caramelo_mapping/maps`, deduzida do share instalado);
`share/caramelo_mapping/maps`; `~/caramelo_ws/src/caramelo_mapping/maps`. Tudo que a GUI
salva vai para a pasta fonte (depois, `colcon build --packages-select caramelo_mapping`
para o `map_name:=` do launch enxergar a arena nova).

Tasks: `<ws>/missions` e `<ws>/missions/tasks geradas` (YAML com `task_id`), `<ws>`
deduzido da pasta dos mapas. Arena ativa e pose inicial: `~/.config/caramelo/robot_state.yaml`
(memória da interface; o stack do robô não lê).

## Estrutura do código

```text
src/main.cpp                 entrada (QApplication + rclcpp)
src/bridge/                  RosBridge, MissionBridge, SensorBridge, ManualLocalization,
                             WaypointManager, RobotState, tipos Qt (health/mission)
src/core/                    MainWindow (faixa + contextos), HomeScreen, ContextScreen,
                             RVizFrame, LaunchRunner, StateMachine, icones
src/widgets/                 MapPreview, seletores (arena, dock, área, tipos)
src/modules/<contexto>/      um par .hpp/.cpp por seção
resources/rviz/gui_main.rviz config do RViz embutido
resources/tema.qss           tema Qt
launch/caramelo_gui.launch.py
```

## Convenções

- C++: classes `CamelCase`, métodos `camelCase`, membros com `_` no fim,
  constantes `kCamelCase`; variáveis novas em `snake_case`.
- Comentários de código em português ASCII (sem acentos), objetivos.
- Cada módulo é um `QWidget` que recebe o `RosBridge*`; ROS só pelo bridge.
- Nenhum texto de tela cita tópico/nó quando o leitor é o operador; o detalhe
  técnico fica no Modo Avançado e nos tooltips.
