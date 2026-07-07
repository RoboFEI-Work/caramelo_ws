# Caramelo WS

Workspace ROS 2 Jazzy do robo Caramelo para mapeamento, localizacao,
navegacao, service areas, docking, rotas e ferramentas de operacao no PC.

Este workspace e o lado do computador de operacao. Ele nao deve iniciar os
drivers fisicos da Raspberry. O hardware embarcado fica no
`caramelo_hardware_ws` e publica os topicos que este workspace consome.

## Visao Geral

Fluxo principal:

1. Raspberry sobe LiDAR, odometria, TF, `robot_description` e controladores.
2. PC roda `caramelo_ws`.
3. PC mapeia com SLAM Toolbox ou Cartographer.
4. Cada arena ganha uma pasta propria em `src/caramelo_mapping/maps/<map_name>`.
5. As Service Areas sao salvas no mapa final.
6. `docking.yaml` e `route_graph.geojson` sao gerados a partir das Service Areas.
7. Nav2 usa `SmacLattice + DWB` como perfil principal.
8. O comando final passa por smoother, collision monitor e relay para a Raspberry.

Topicos esperados da Raspberry:

- `/scan`: LaserScan do LiDAR.
- `/odom`: odometria usada pelo Nav2, AMCL e SLAM.
- `/tf` e `/tf_static`: arvore TF do robo.
- `/robot_description`: URDF completo para RViz.
- `/joint_states`: estados das juntas.
- `/mecanum_controller/reference`: comando `TwistStamped` do controlador mecanum.

Diagnostico rapido:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

Se `/scan`, `/odom` ou `odom -> base_footprint` nao existem, corrija primeiro
o bringup da Raspberry.

## Preparar O Workspace

Em todo terminal novo:

```bash
cd ~/caramelo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Build completo:

```bash
cd ~/caramelo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build dos pacotes mais usados:

```bash
colcon build --symlink-install --packages-select \
  caramelo_msgs caramelo_mapping caramelo_localization \
  caramelo_navigation caramelo_utils
source install/setup.bash
```

Quando criar ou alterar mapas, Service Areas, docking ou route graph, faca pelo
menos:

```bash
colcon build --symlink-install --packages-select caramelo_mapping caramelo_navigation
source install/setup.bash
```

## Pacotes Principais

- `caramelo_mapping`: SLAM Toolbox, Cartographer, mapas e RViz de mapeamento.
- `caramelo_localization`: AMCL, map server e RViz de localizacao.
- `caramelo_navigation`: Nav2, BTs, planners, controllers, docking, rotas e service areas.
- `caramelo_utils`: teleop e relay `Twist -> TwistStamped`.
- `caramelo_msgs`: actions e services proprios do projeto.
- `caramelo_planning`: utilitarios de planejamento de tarefas.

## Pasta De Cada Mapa

Cada arena ou sala deve ter uma pasta propria:

```text
src/caramelo_mapping/maps/<map_name>/
  map.yaml
  map.pgm
  service_areas.yaml
  docking.yaml
  route_graph.geojson
```

Arquivos:

- `map.yaml`: metadata do mapa de ocupacao.
- `map.pgm`: imagem do mapa.
- `service_areas.yaml`: arquivo humano principal das areas da arena.
- `docking.yaml`: arquivo tecnico derivado para o Nav2 Docking Server.
- `route_graph.geojson`: grafo do Nav2 Route Server.

Mapas presentes agora:

- `sala_520`
- `arena_teste`

## Criar Um Mapa Novo

Exemplo com nome `arena_2026`.

### 1. Subir SLAM Toolbox

```bash
ros2 launch caramelo_mapping slam.launch.py map_name:=arena_2026
```

Parametros uteis:

- `map_name`: nome da pasta do mapa.
- `use_sim_time`: default `false`.
- `use_rviz`: default `true`.
- `map_resolution`: default `0.02`.
- `slam_scan_topic`: default `/scan`.
- `scan_range_max`: default `30.0`.
- `use_service_area_markers`: default `true`.
- `service_area_marker_topic`: default `/caramelo/service_areas/markers`.

### 2. Mover O Robo

Use teleop em outro terminal:

```bash
ros2 launch caramelo_utils teleop.launch.py
```

Dicas:

- comece no `START`;
- ande devagar;
- evite giros muito rapidos;
- passe de novo por regioes ja mapeadas para ajudar loop closure;
- evite pessoas e objetos mexendo perto do LiDAR.

### 3. Salvar O Mapa

```bash
mkdir -p src/caramelo_mapping/maps/arena_2026
ros2 run nav2_map_server map_saver_cli -f src/caramelo_mapping/maps/arena_2026/map
```

Depois:

```bash
colcon build --symlink-install --packages-select caramelo_mapping
source install/setup.bash
```

## Service Areas

Service Area e uma pose oficial da arena: `START`, `FINISH`, `WS1`, `SH1`,
`PP1`, `RT1`, etc.

O arquivo principal e:

```text
src/caramelo_mapping/maps/<map_name>/service_areas.yaml
```

Conceitos importantes:

- `final_robot_pose`: pose final do robo parado em frente a mesa.
- `final_robot_pose` nao e a pose da mesa.
- `approach_pose`: calculada automaticamente a partir da `final_robot_pose`.
- `approach_offset`: distancia atras da pose final usada para a aproximacao.
- poses `0,0,0` sao placeholders e sao ignoradas no route graph.

Tipos aceitos:

- `start`
- `finish`
- `workstation`
- `shelf`
- `precision_placement`
- `rotating_table`

Alturas aceitas para mesas:

```text
0.00, 0.05, 0.10, 0.15
```

### Inicializar Service Areas De Um Mapa

```bash
ros2 run caramelo_navigation init_service_areas --map arena_2026 --sync-docking
```

Com lista custom:

```bash
ros2 run caramelo_navigation init_service_areas \
  --map arena_2026 \
  --areas START FINISH WS1 WS2 WS3 SH1 PP1 RT1 \
  --sync-docking
```

### Salvar Uma Pose

Leve o robo ate a pose final real e execute:

```bash
ros2 run caramelo_navigation save_service_area_pose \
  --map arena_2026 \
  --name WS1 \
  --type workstation \
  --height 0.10
```

Para sobrescrever uma pose existente:

```bash
ros2 run caramelo_navigation save_service_area_pose \
  --map arena_2026 \
  --name WS1 \
  --type workstation \
  --height 0.10 \
  --overwrite
```

START e FINISH:

```bash
ros2 run caramelo_navigation save_service_area_pose --map arena_2026 --name START --type start --height 0.00 --overwrite
ros2 run caramelo_navigation save_service_area_pose --map arena_2026 --name FINISH --type finish --height 0.00 --overwrite
```

Ajustes importantes:

- `--approach-offset`: distancia da approach pose ate a pose final.
- `--linear-tolerance`: tolerancia linear para docking.
- `--angular-tolerance`: tolerancia angular para docking.
- `--width` e `--depth`: tamanho aproximado da mesa no marker.
- `--no-sync-docking`: salva sem atualizar `docking.yaml`.

### Validar Service Areas

```bash
ros2 run caramelo_navigation list_service_areas --map arena_2026
ros2 run caramelo_navigation validate_service_areas --map arena_2026 --sync-docking
```

Use `--sync-docking` sempre depois de editar `service_areas.yaml` manualmente.

## Docking

O docking usa dois estagios:

1. Nav2 navega ate a `approach_pose` ou staging pose.
2. Nav2 Docking Server faz a aproximacao final curta e lenta ate a dock pose.

Arquivo tecnico:

```text
src/caramelo_mapping/maps/<map_name>/docking.yaml
```

Regra de manutencao:

- edite `service_areas.yaml`;
- rode `validate_service_areas --sync-docking`;
- evite editar `docking.yaml` manualmente, porque ele e derivado.

Subir navegacao com docking:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026 use_docking:=true
```

Conferir actions:

```bash
ros2 action list | grep dock
```

Dockar por ID:

```bash
ros2 run caramelo_navigation dock_to --map-name arena_2026 --dock-id WS1
```

Para testar apenas o Docking Server sem a etapa de navegar ate staging:

```bash
ros2 run caramelo_navigation dock_to --map-name arena_2026 --dock-id WS1 --no-nav-to-staging
```

Parametros base do docking ficam em:

```text
src/caramelo_navigation/launch/docking_server.launch.py
```

Valores atuais foram deixados conservadores para aproximacao final:

- `v_linear_min: 0.015`
- `v_linear_max: 0.06`
- `v_angular_max: 0.25`
- `slowdown_radius: 0.45`
- `dock_prestaging_tolerance: 0.25`
- `use_collision_detection: true`

## Route Graph

O Nav2 Route Server usa:

```text
src/caramelo_mapping/maps/<map_name>/route_graph.geojson
```

Esse arquivo e gerado a partir de `service_areas.yaml`. Cada Service Area valida
vira um no. Poses placeholder `0,0,0` sao ignoradas.

Gerar para todos os mapas:

```bash
python3 src/caramelo_navigation/scripts/sync_route_graph.py
```

Gerar para um mapa especifico:

```bash
python3 src/caramelo_navigation/scripts/sync_route_graph.py \
  --map-folder src/caramelo_mapping/maps/arena_2026
```

Controlar quantos vizinhos cada no conecta:

```bash
python3 src/caramelo_navigation/scripts/sync_route_graph.py \
  --map-folder src/caramelo_mapping/maps/arena_2026 \
  --max-neighbors 3
```

Como o launch usa o grafo:

- `use_route_server:=true` e o default.
- Se o `route_graph.geojson` tiver pelo menos 1 no e 1 aresta, o Route Server sobe.
- Se o grafo estiver vazio, o launch pula o Route Server para nao quebrar o Nav2.

Edicao manual:

- pode ajustar arestas no GeoJSON se a arena tiver corredores obrigatorios;
- preserve `Point` para nos;
- preserve `MultiLineString` para arestas;
- preserve `startid` e `endid` nas arestas;
- depois de editar manualmente, nao rode `sync_route_graph` sem querer, porque ele regenera o arquivo.

## Nav2 Do Caramelo

Perfil principal:

- Planner: `SmacLattice`.
- Controller: `DWBHolonomic`.
- Smoother: `savitzky_golay_smoother`.
- Goal checker: `general_goal_checker`.
- Footprint: poligono SE2 nao circular com ressalto frontal em `+X`.

Perfil experimental:

- Planner: `SmacLattice`.
- Controller: `MPPIHolonomic`.
- Smoother: `savitzky_golay_smoother`.

Arquivos principais:

```text
src/caramelo_navigation/config/planner_server.yaml
src/caramelo_navigation/config/controller_server.yaml
src/caramelo_navigation/config/smoother_server.yaml
src/caramelo_navigation/config/velocity_smoother.yaml
src/caramelo_navigation/config/collision_monitor.yaml
src/caramelo_navigation/config/route_server.yaml
src/caramelo_navigation/config/waypoint_follower.yaml
src/caramelo_navigation/config/bt_navigator.yaml
src/caramelo_navigation/behavior_tree/caramelo_lattice_dwb.xml
src/caramelo_navigation/behavior_tree/caramelo_lattice_mppi.xml
```

Primitives custom do Smac:

```text
src/caramelo_navigation/config/lattice/caramelo_omni_2cm_16bins.json
```

Elas usam:

- `motion_model: omni`
- `grid_resolution: 0.02`
- `num_of_headings: 16`
- `turning_radius: 0.0`

Para regenerar:

```bash
python3 src/caramelo_navigation/scripts/generate_caramelo_omni_lattice.py
```

Depois recompile:

```bash
colcon build --symlink-install --packages-select caramelo_navigation
source install/setup.bash
```

## Bringup De Navegacao

Com RViz:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026
```

Com docking:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026 use_docking:=true
```

Sem RViz:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026 use_rviz:=false
```

Perfil MPPI experimental:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026 nav_profile:=mppi
```

Sem Route Server:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026 use_route_server:=false
```

Sem Waypoint Follower:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026 use_waypoint_follower:=false
```

Cadeia de velocidade:

```text
controller_server -> /cmd_vel_nav
velocity_smoother -> /cmd_vel_smoothed
collision_monitor -> /cmd_vel
twist_relay.py -> /mecanum_controller/reference
```

## Parametros Dos Launches Principais

### `caramelo_navigation bringup.launch.py`

Este e o launch principal para operacao no PC. Ele inclui localizacao, Nav2 e
opcionalmente RViz.

- `map_name`: mapa em `src/caramelo_mapping/maps/<map_name>`. Default `sala_520`.
- `use_sim_time`: usar clock de simulacao. Default `false`.
- `use_rviz`: abre RViz. Default `true`.
- `rviz_config`: caminho do arquivo RViz.
- `use_docking`: sobe Nav2 Docking Server. Default `false`.
- `navigation_start_delay`: espera antes de iniciar Nav2 depois da localizacao. Default `5.0`.
- `costmap_resolution`: default `0.02`; use `map` para ler de `map.yaml`.
- `nav_profile`: `dwb` ou `mppi`. Default `dwb`.
- `use_route_server`: tenta subir Route Server se o grafo do mapa for valido. Default `true`.
- `use_waypoint_follower`: sobe Waypoint Follower. Default `true`.
- `use_cmd_vel_relay`: converte `/cmd_vel` para `TwistStamped`. Default `true`.
- `cmd_vel_topic`: topico final depois do collision monitor. Default `/cmd_vel`.
- `mecanum_reference_topic`: topico do controlador mecanum. Default `/mecanum_controller/reference`.
- `use_service_area_manager`: sobe API de service areas. Default `true`.
- `use_service_area_markers`: publica markers das areas. Default `true`.
- `service_area_marker_topic`: default `/caramelo/service_areas/markers`.

### `caramelo_navigation navigation.launch.py`

Sobe localizacao e stack Nav2 sem abrir RViz do bringup. Use quando ja existe um
RViz aberto ou em testes na Raspberry/SSH.

Tem quase os mesmos parametros do `bringup.launch.py`, exceto `use_rviz` e
`rviz_config`.

### `caramelo_mapping slam.launch.py`

Sobe SLAM Toolbox e ferramentas de mapa.

- `map_name`: nome do mapa em criacao ou edicao.
- `use_sim_time`: default `false`.
- `use_rviz`: default `true`.
- `map_resolution`: default `0.02`.
- `slam_scan_topic`: default `/scan`.
- `scan_range_max`: default `30.0`.
- `use_service_area_markers`: default `true`.
- `service_area_marker_topic`: default `/caramelo/service_areas/markers`.

### `caramelo_localization global_localization.launch.py`

Sobe AMCL e map server para um mapa salvo.

```bash
ros2 launch caramelo_localization global_localization.launch.py map_name:=arena_2026
```

## Launches Avulsos

SLAM Toolbox:

```bash
ros2 launch caramelo_mapping slam.launch.py map_name:=arena_2026
```

Cartographer, para comparar com SLAM Toolbox:

```bash
ros2 launch caramelo_cartographer cartographer.launch.py
```

Somente localizacao:

```bash
ros2 launch caramelo_localization global_localization.launch.py map_name:=arena_2026
```

Nav2 sem RViz do bringup:

```bash
ros2 launch caramelo_navigation navigation.launch.py map_name:=arena_2026
```

Teleop:

```bash
ros2 launch caramelo_utils teleop.launch.py
```

Somente Docking Server, quando a stack de navegacao ja esta rodando:

```bash
ros2 launch caramelo_navigation docking_server.launch.py map_name:=arena_2026
```

## Testar Navegacao

Depois do bringup:

```bash
ros2 topic echo /global_costmap/published_footprint --once
ros2 topic echo /local_costmap/published_footprint --once
ros2 action list | grep navigate
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
ros2 topic echo /mecanum_controller/reference
```

No RViz:

- confirme pose inicial do AMCL;
- confirme footprint poligonal com ressalto frontal;
- confirme inflacao larga ao redor dos obstaculos;
- envie goal curto;
- envie goal lateral curto;
- envie goal com yaw final diferente para testar giro no proprio eixo;
- confira se `/cmd_vel_nav` mostra `angular.z` dominante quando ele alinha yaw.

## Fluxo Recomendado Para Uma Arena Nova

1. Criar mapa com SLAM:

```bash
ros2 launch caramelo_mapping slam.launch.py map_name:=arena_2026
```

2. Salvar mapa:

```bash
mkdir -p src/caramelo_mapping/maps/arena_2026
ros2 run nav2_map_server map_saver_cli -f src/caramelo_mapping/maps/arena_2026/map
```

3. Inicializar Service Areas:

```bash
ros2 run caramelo_navigation init_service_areas --map arena_2026 --sync-docking
```

4. Reabrir com Nav2:

```bash
colcon build --symlink-install --packages-select caramelo_mapping caramelo_navigation
source install/setup.bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026 use_docking:=true
```

5. Salvar poses reais:

```bash
ros2 run caramelo_navigation save_service_area_pose --map arena_2026 --name START --type start --height 0.00 --overwrite
ros2 run caramelo_navigation save_service_area_pose --map arena_2026 --name WS1 --type workstation --height 0.10 --overwrite
ros2 run caramelo_navigation save_service_area_pose --map arena_2026 --name FINISH --type finish --height 0.00 --overwrite
```

6. Validar docking:

```bash
ros2 run caramelo_navigation validate_service_areas --map arena_2026 --sync-docking
```

7. Gerar route graph:

```bash
python3 src/caramelo_navigation/scripts/sync_route_graph.py --map-folder src/caramelo_mapping/maps/arena_2026
```

8. Build final:

```bash
colcon build --symlink-install --packages-select caramelo_mapping caramelo_navigation
source install/setup.bash
```

9. Testar:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026 use_docking:=true
ros2 run caramelo_navigation dock_to --map-name arena_2026 --dock-id WS1
```

## Problemas Comuns

### O mapa nao aparece no RViz

```bash
ros2 topic echo /map --once
```

Confira se o SLAM, Cartographer ou `map_server` esta ativo.

### O SLAM Toolbox nao recebe laser

```bash
ros2 topic echo /scan --once
```

Se o LiDAR estiver em outro topico, informe no launch:

```bash
ros2 launch caramelo_mapping slam.launch.py slam_scan_topic:=/nome_do_scan
```

### O robo planeja mas nao anda

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /mecanum_controller/reference
ros2 topic echo /odom --once
```

Se `/cmd_vel` existe mas `/mecanum_controller/reference` nao muda, confira:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_2026 use_cmd_vel_relay:=true
```

### O robo acha que chegou sem se mover

Normalmente e problema de tempo, AMCL ou TF.

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /odom --once
```

### O Route Server nao sobe

Veja no log:

```text
Route server enabled: False
```

Isso acontece quando o mapa nao tem `route_graph.geojson` valido ou quando o
grafo esta vazio. Salve poses reais e rode:

```bash
python3 src/caramelo_navigation/scripts/sync_route_graph.py --map-folder src/caramelo_mapping/maps/<map_name>
```

### Docking fica perdido no fim

Confira:

- pose final salva no `service_areas.yaml`;
- `approach_offset`;
- `linear_tolerance`;
- `angular_tolerance`;
- se o robo chegou bem na `approach_pose` antes do docking final;
- se `use_docking:=true` foi usado no bringup.

Depois de editar:

```bash
ros2 run caramelo_navigation validate_service_areas --map <map_name> --sync-docking
python3 src/caramelo_navigation/scripts/sync_route_graph.py --map-folder src/caramelo_mapping/maps/<map_name>
colcon build --symlink-install --packages-select caramelo_mapping caramelo_navigation
source install/setup.bash
```

### O mapa salvo nao carrega

Confira:

- existe `src/caramelo_mapping/maps/<map_name>/map.yaml`;
- existe `src/caramelo_mapping/maps/<map_name>/map.pgm`;
- o pacote `caramelo_mapping` foi rebuildado;
- o terminal recebeu `source install/setup.bash`;
- o launch foi chamado com `map_name:=<map_name>`.

## Referencias Internas

Tutorial detalhado de Service Areas e Docking:

```text
docs/service_areas_docking.md
```

Roteiro completo para criar mapa novo e validar Service Areas, docking e rotas:

```text
docs/passo_a_passo_mapa_service_areas_docking_route.md
```

O plano de rede PC/Raspberry deve ser mantido junto da documentacao da equipe
ou copiado para `docs/` quando for versionado neste workspace.
