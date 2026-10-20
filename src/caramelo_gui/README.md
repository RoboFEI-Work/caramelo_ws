# caramelo_gui

Interface gráfica de operação do Caramelo no PC: aplicação Qt5/C++ com RViz
embutido. O mapa fica sempre visível no centro, com o robô em cima, e uma barra
lateral dá acesso aos módulos. A GUI é só um **cliente** do grafo ROS 2 — tudo
que ela faz também pode ser feito por terminal (ver `docs/`).

## Como rodar

```bash
source ~/caramelo_ws/install/setup.bash
ros2 launch caramelo_gui caramelo_gui.launch.py
```

Argumento do launch:

| Arg | Default | Efeito |
|---|---|---|
| `use_diagnostics` | `true` | Sobe junto o `caramelo_diagnostics/diagnostics.launch.py` (agregador de saúde que publica `/diagnostics`, consumido pelos cartões do módulo Início). Use `false` se o agregador já estiver no ar por outro launch. |

O executável é `caramelo_gui` (sem `name:=` no launch: o processo contém mais de
um nó interno e o remap de nome único os colidia).

## AVISO: GUI com `robot_manipulation.launch.py` no ar

Quando o sistema de competição já foi subido por

```bash
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=<mapa>
```

o Nav2 **já está rodando**. Nesse cenário use a GUI **só como monitor e para
localizar** o robô (fantasma arrastável -> `/initialpose`) e **NÃO clique em
"Ligar navegação (Nav2)"** no módulo Navegação: isso sobe um segundo
`bringup.launch.py` e duplica o Nav2 (dois AMCL, dois bt_navigator, status
mentindo). O mesmo vale para **"Iniciar SLAM" com a localização ativa**: o
`slam.launch.py` apenas **AVISA** (RK-02, não aborta) e sobe mesmo assim — SLAM
e AMCL passam a publicar `map->odom` ao mesmo tempo e o TF fica piscando.
Derrube o Nav2 antes de mapear.

Fluxo completo de missão: `docs/guia_missoes.md`.

## Módulos (barra lateral)

| # | Módulo | O que faz |
|---|---|---|
| 0 | **Início** | "Bringup visual": um cartão por subsistema (rede/Raspberry, controllers, LiDAR, IMU, odometria, TF, Nav2) com cor por estado, alimentado por `/diagnostics`. Modo Operador mostra só a mensagem; modo Engenharia expande causa/ação/frequência. |
| 1 | **Mapa & Camadas** | Painel do RViz embutido: liga/desliga camadas (Map, ParticleCloud, Service Areas, ...), ferramentas Interagir / Mover câmera / Definir Goal e a localização rápida (Posicionar robô -> arrastar -> Confirmar pose). |
| 2 | **Navegação** | Sobe/derruba o stack Nav2 + AMCL (`caramelo_navigation/bringup.launch.py`) com mapa, docking server e relógio de simulação; envia meta pela ferramenta "Definir Goal", cancela e limpa os costmaps; mostra o status da action. |
| 3 | **Docking** | Dock / undock (`/dock_robot`, `/undock_robot`), alinhamento fino holonômico (`align_to_dock`) e "salvar pose atual como dock" (`/save_dock_pose`). |
| 4 | **Localização** | Relocalização manual assistida: arrasta o robô-fantasma no mapa enquanto o scan reprojetado acompanha; ao confirmar, publica em `/initialpose`. |
| 5 | **Mapas** | Lista os mapas de `caramelo_mapping/maps` com miniatura, carrega um mapa em runtime (`/map_server/load_map`), renomeia, duplica, exclui (vai para a lixeira) e salva o mapa atual do SLAM. |
| 6 | **Mapeamento** | Inicia/para o SLAM (`caramelo_mapping/slam.launch.py`, sem RViz próprio — o mapa cresce no RViz embutido). Se `maps/<nome>/map.yaml` já existe passa `map_name:=<nome>` (markers das service areas desse mapa); se é mapa novo passa `use_service_area_markers:=false`, porque o `service_area_markers_node` morre na partida quando a pasta não existe. "Salvar mapa" grava em `maps/<nome>/`. |
| 7 | **Teleop** | Liga/desliga o `caramelo_utils/teleop.launch.py` (joystick + teclado + twist_mux) como processo gerido pela GUI. |
| 8 | **Service Areas** | Fluxo guiado: posicionar o robô, "Salvar pose atual", escolher nome/tipo; lista e valida as service areas pelo `service_area_manager_node`. |
| 9 | **Waypoints** | Opcional (o padrão do robô é o planejamento livre do Nav2): criar na pose do robô, mover/girar por Interactive Marker, salvar por mapa (`maps/<mapa>/waypoints.yaml`), navegar até um e "seguir todos" (`follow_waypoints`). |
| 10 | **Editor de Mapa** | Limpeza raster do `map.pgm` (pincel Parede/Livre/Desconhecido, zoom, desfazer), salvar com backup automático, ajuste da origem no `map.yaml` e recarregar no `map_server`. Edita só o mapa base; keepout continua em máscara separada. |

Além dos módulos há o botão **Logs** (painel oculto com o `/rosout` dos outros
nós, WARN+ e INFO) e a barra de estado da máquina de estados da GUI.

## Launches que a GUI sobe como processos filhos

Cada um roda como `ros2 launch <pacote> <arquivo> <args>` dentro de um
`QProcess` (`src/core/launch_runner.cpp`) e é derrubado com SIGINT, como um
Ctrl+C.

| Módulo | Launch | Args passados |
|---|---|---|
| Mapeamento | `caramelo_mapping/launch/slam.launch.py` | `map_name:=<nome>` (só se `maps/<nome>/map.yaml` existe; senão `use_service_area_markers:=false`) `use_rviz:=false` `use_sim_time:=<bool>` |
| Navegação | `caramelo_navigation/launch/bringup.launch.py` | `map_name:=<mapa>` `use_rviz:=false` `use_docking:=<bool>` `use_sim_time:=<bool>` |
| Teleop | `caramelo_utils/launch/teleop.launch.py` | — |

## Interfaces ROS usadas

Nós do processo: `caramelo_gui_bridge` (rclcpp, toda a comunicação ROS dos
módulos passa por ele — `src/bridge/`) e `caramelo_gui_rviz` (RViz embutido,
`src/core/rviz_frame.cpp`). Os módulos em `src/modules/` não criam
publishers/subscribers próprios; usam o `RosBridge`.

Tópicos:

| Tópico | Tipo | Direção | Uso |
|---|---|---|---|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | assina | cartões do Início |
| `/rosout` | `rcl_interfaces/Log` | assina | painel Logs |
| `/scan` | `sensor_msgs/LaserScan` | assina | preview do scan na localização manual |
| `/caramelo_gui/goal_pose` | `geometry_msgs/PoseStamped` | assina | ferramenta "Definir Goal" do RViz embutido (tópico privado: o `bt_navigator` também assina `/goal_pose`, e o compartilhado disparava dois `NavigateToPose`) |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | publica | pose inicial para o AMCL |
| `/save_dock_pose` | `std_msgs/String` | publica | salvar pose atual como dock |
| `/localizacao_manual/scan_preview` | `visualization_msgs/Marker` | publica | scan reprojetado do fantasma |

Interactive markers (servers): `localizacao_manual` (robô-fantasma) e
`waypoints`. TF: listener `map -> base` para a pose atual do robô.

Actions (clientes):

| Action | Tipo |
|---|---|
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` |
| `follow_waypoints` | `nav2_msgs/FollowWaypoints` |
| `/dock_robot` | `nav2_msgs/DockRobot` |
| `/undock_robot` | `nav2_msgs/UndockRobot` |
| `align_to_dock` | `caramelo_msgs/AlignToDock` |
| `/caramelo/service_areas/save_pose` | `caramelo_msgs/SaveServiceAreaPose` |

Serviços (clientes):

| Serviço | Tipo |
|---|---|
| `/map_server/load_map` | `nav2_msgs/LoadMap` |
| `/map_saver_server/save_map` | `nav2_msgs/SaveMap` (o nó do `slam.launch.py` chama-se `map_saver_server`) |
| `/caramelo/service_areas/list` | `caramelo_msgs/ListServiceAreas` |
| `/caramelo/service_areas/validate` | `caramelo_msgs/ValidateServiceAreas` |
| `/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/ClearEntireCostmap` |
| `/local_costmap/clear_entirely_local_costmap` | `nav2_msgs/ClearEntireCostmap` |

## Onde ficam os mapas

`MapasModule::mapsDir()` resolve, nesta ordem: variável de ambiente
`CARAMELO_MAPS_DIR`; a árvore **fonte** do workspace
(`<ws>/src/caramelo_mapping/maps`, deduzida a partir do share instalado);
`share/caramelo_mapping/maps`; e por fim `~/caramelo_ws/src/caramelo_mapping/maps`.
Mapas salvos, renomeados ou editados pela GUI vão para a pasta fonte, não para o
`install/`.

## Estrutura do código

```text
src/main.cpp                 entrada (QApplication + rclcpp)
src/bridge/                  RosBridge, ManualLocalization, WaypointManager, HealthTypes
src/core/                    MainWindow, RVizFrame, LaunchRunner, StateMachine
src/modules/<modulo>/        um par .hpp/.cpp por módulo da barra lateral
resources/rviz/gui_main.rviz config do RViz embutido
resources/tema.qss           tema Qt
launch/caramelo_gui.launch.py
```

## Convenções

- C++: classes `CamelCase`, métodos `camelCase`, membros com `_` no fim,
  constantes `kCamelCase`.
- Variáveis locais e parâmetros **novos** em `snake_case`. Há locais antigas em
  `camelCase` estilo Qt (`sideLayout`, `logsBtn`, `panelIndex`...); não renomear
  em massa — ajustar quando o arquivo for tocado.
- Comentários de código em português ASCII (sem acentos), objetivos.
- Cada módulo é um `QWidget` que recebe o `RosBridge*`; ROS só pelo bridge.
