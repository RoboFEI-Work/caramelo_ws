# Passo A Passo Para Criar Mapa, Service Areas, Docking E Route Graph

Este roteiro e para a equipe seguir durante teste do robo. Ele cobre o fluxo
desde abrir um mapa novo ate validar navegacao, Service Areas, docking e rotas.

Use este guia quando for criar uma arena nova, atualizar uma arena existente ou
preparar o robo para uma rodada de testes.

## Convencoes

Neste documento, substitua `arena_2026` pelo nome real do mapa:

```bash
export MAP_NAME=arena_2026
```

Sempre rode os comandos a partir do workspace do PC:

```bash
cd ~/caramelo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Pasta final do mapa:

```text
src/caramelo_mapping/maps/<MAP_NAME>/
  map.yaml
  map.pgm
  service_areas.yaml
  docking.yaml
  route_graph.geojson
```

Arquivos importantes:

- `map.yaml` e `map.pgm`: mapa de ocupacao usado pelo AMCL/Nav2.
- `service_areas.yaml`: arquivo humano principal, editavel.
- `docking.yaml`: gerado a partir do `service_areas.yaml`.
- `route_graph.geojson`: gerado a partir do `service_areas.yaml`.

Regra de ouro:

- edite `service_areas.yaml`;
- gere de novo `docking.yaml`;
- gere de novo `route_graph.geojson`;
- rode build;
- reinicie o bringup.

## 0. Checklist Antes De Comecar

No PC:

```bash
cd ~/caramelo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  caramelo_msgs caramelo_mapping caramelo_localization \
  caramelo_navigation caramelo_utils
source install/setup.bash
```

Confirme que a Raspberry esta no mesmo `ROS_DOMAIN_ID` do PC.

Confira os topicos vindos do robo:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /robot_description --once
ros2 topic echo /joint_states --once
```

Confira TF:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_frame
```

Resultado esperado:

- `/scan` chega continuamente.
- `/odom` chega continuamente.
- existe TF `odom -> base_footprint`.
- existe TF do robo ate o frame do LiDAR.
- RViz no PC consegue ver o `RobotModel`.

Se isso falhar, pare aqui e corrija o `caramelo_hardware_ws` na Raspberry.

## 1. Criar A Pasta Do Mapa

Escolha um nome curto e sem espacos:

```bash
export MAP_NAME=arena_2026
mkdir -p src/caramelo_mapping/maps/${MAP_NAME}
```

Inicialize os arquivos de Service Areas desde o comeco:

```bash
ros2 run caramelo_navigation init_service_areas \
  --map ${MAP_NAME} \
  --areas START FINISH WS1 WS2 WS3 SH1 PP1 RT1 \
  --sync-docking
```

Isso cria ou valida:

```text
src/caramelo_mapping/maps/<MAP_NAME>/service_areas.yaml
src/caramelo_mapping/maps/<MAP_NAME>/docking.yaml
```

As poses com `0,0,0` sao placeholders. Elas ainda nao servem para teste real.

## 2. Fazer O Mapa Com SLAM Toolbox

Terminal 1:

```bash
ros2 launch caramelo_mapping slam.launch.py map_name:=${MAP_NAME}
```

O launch sobe:

- `slam_toolbox`;
- `map_saver_server`;
- RViz de SLAM;
- markers das Service Areas em `/caramelo/service_areas/markers`.

Parametros uteis:

- `map_name`: nome do mapa.
- `map_resolution`: default `0.02`.
- `slam_scan_topic`: default `/scan`.
- `scan_range_max`: default `30.0`.
- `use_rviz`: default `true`.
- `use_service_area_markers`: default `true`.

Se quiser abrir sem RViz:

```bash
ros2 launch caramelo_mapping slam.launch.py map_name:=${MAP_NAME} use_rviz:=false
```

Terminal 2, teleop:

```bash
ros2 launch caramelo_utils teleop.launch.py
```

Durante o mapeamento:

- ande devagar;
- evite giros rapidos;
- passe pelas paredes e corredores;
- volte por regioes ja mapeadas para ajudar loop closure;
- evite pessoas passando na frente do LiDAR;
- nao salve mapa enquanto o SLAM ainda esta ajustando muito o frame `map`.

## 3. Salvar O Mapa

Quando o mapa estiver bom:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f src/caramelo_mapping/maps/${MAP_NAME}/map
```

Confira se foram criados:

```bash
ls src/caramelo_mapping/maps/${MAP_NAME}/map.yaml
ls src/caramelo_mapping/maps/${MAP_NAME}/map.pgm
```

Build para instalar o mapa:

```bash
colcon build --symlink-install --packages-select caramelo_mapping
source install/setup.bash
```

## 4. Reabrir O Mapa Com Nav2

Agora feche o SLAM e abra o mapa salvo com localizacao e Nav2:

```bash
ros2 launch caramelo_navigation bringup.launch.py \
  map_name:=${MAP_NAME} \
  use_docking:=true
```

O bringup sobe:

- AMCL;
- `map_server`;
- `controller_server`;
- `planner_server`;
- `smoother_server`;
- `bt_navigator`;
- `behavior_server`;
- `velocity_smoother`;
- `collision_monitor`;
- Service Area markers;
- Docking Server, se `use_docking:=true`;
- Route Server, se o grafo tiver nos e arestas;
- Waypoint Follower, se habilitado;
- RViz, se `use_rviz:=true`.

Parametros importantes:

- `map_name`: mapa a abrir.
- `use_docking`: sobe docking. Use `true` para teste de WS/SH/PP/RT.
- `nav_profile`: `dwb` ou `mppi`. Default `dwb`.
- `costmap_resolution`: default `0.02`; use `map` para ler do `map.yaml`.
- `use_route_server`: default `true`; so sobe se `route_graph.geojson` for valido.
- `use_waypoint_follower`: default `true`.
- `use_cmd_vel_relay`: default `true`.
- `use_rviz`: default `true`.

Se o AMCL nao estiver alinhado:

1. No RViz, clique em `2D Pose Estimate`.
2. Clique na posicao real do robo.
3. Arraste na orientacao real.
4. Espere as particulas estabilizarem.

## 5. Salvar START E FINISH Reais

Use teleop para posicionar o robo no START real.

```bash
ros2 run caramelo_navigation save_service_area_pose \
  --map ${MAP_NAME} \
  --name START \
  --type start \
  --height 0.00 \
  --overwrite
```

Use teleop para posicionar o robo no FINISH real.

```bash
ros2 run caramelo_navigation save_service_area_pose \
  --map ${MAP_NAME} \
  --name FINISH \
  --type finish \
  --height 0.00 \
  --overwrite
```

START e FINISH normalmente nao usam docking:

```yaml
docking:
  use_docking: false
```

## 6. Salvar Workstations E Outras Areas

Para cada area, leve o robo manualmente ate a pose final desejada. A pose salva
e a pose final do robo parado em frente a mesa, nao a pose da mesa.

### WS

```bash
ros2 run caramelo_navigation save_service_area_pose \
  --map ${MAP_NAME} \
  --name WS1 \
  --type workstation \
  --height 0.10 \
  --approach-offset 0.45 \
  --overwrite
```

Repita para `WS2`, `WS3`, etc.

### Shelf

```bash
ros2 run caramelo_navigation save_service_area_pose \
  --map ${MAP_NAME} \
  --name SH1 \
  --type shelf \
  --height 0.10 \
  --approach-offset 0.50 \
  --overwrite
```

### Precision Placement

```bash
ros2 run caramelo_navigation save_service_area_pose \
  --map ${MAP_NAME} \
  --name PP1 \
  --type precision_placement \
  --height 0.10 \
  --linear-tolerance 0.025 \
  --angular-tolerance 0.035 \
  --overwrite
```

### Rotating Table

```bash
ros2 run caramelo_navigation save_service_area_pose \
  --map ${MAP_NAME} \
  --name RT1 \
  --type rotating_table \
  --height 0.10 \
  --overwrite
```

Alturas aceitas:

```text
0.00, 0.05, 0.10, 0.15
```

Campos que voce pode ajustar no comando:

- `--approach-offset`: distancia da approach pose ate a pose final.
- `--linear-tolerance`: tolerancia linear para docking.
- `--angular-tolerance`: tolerancia angular para docking.
- `--width`: largura da mesa para marker.
- `--depth`: profundidade da mesa para marker.
- `--notes`: comentario humano.

## 7. Conferir E Editar service_areas.yaml

Abra:

```text
src/caramelo_mapping/maps/<MAP_NAME>/service_areas.yaml
```

Cada area deve ter formato parecido com:

```yaml
WS1:
  type: workstation
  frame_id: map
  final_robot_pose:
    x: 1.7744
    y: -0.8495
    yaw: 3.071961
  table:
    width: 0.8
    depth: 0.5
    height: 0.1
  docking:
    approach_offset: 0.45
    linear_tolerance: 0.03
    angular_tolerance: 0.04
    use_docking: true
  manipulation_enabled: true
  notes: WS1 - pose final do robo em frente a mesa
```

Pode editar manualmente:

- `notes`;
- `table.width`;
- `table.depth`;
- `table.height`;
- `docking.approach_offset`;
- `docking.linear_tolerance`;
- `docking.angular_tolerance`;
- `docking.use_docking`;
- `manipulation_enabled`.

Evite editar manualmente `x`, `y`, `yaw` sem medir no robo, porque isso muda a
pose final real.

## 8. Validar Service Areas E Sincronizar Docking

Liste as areas:

```bash
ros2 run caramelo_navigation list_service_areas --map ${MAP_NAME}
```

Valide e gere `docking.yaml`:

```bash
ros2 run caramelo_navigation validate_service_areas \
  --map ${MAP_NAME} \
  --sync-docking
```

Resultado esperado:

- areas reais nao aparecem como placeholder `0,0,0`;
- `docking.yaml` e atualizado;
- areas com docking ficam com dock plugin proprio.

Arquivo gerado:

```text
src/caramelo_mapping/maps/<MAP_NAME>/docking.yaml
```

Nao edite `docking.yaml` como arquivo principal. Se precisar mudar tolerancia,
offset ou pose, mude `service_areas.yaml` e rode `validate_service_areas` de novo.

## 9. Gerar Route Graph

O route graph e usado pelo Nav2 Route Server. Ele e derivado das Service Areas.

Gere o grafo:

```bash
python3 src/caramelo_navigation/scripts/sync_route_graph.py \
  --map-folder src/caramelo_mapping/maps/${MAP_NAME}
```

O arquivo gerado e:

```text
src/caramelo_mapping/maps/<MAP_NAME>/route_graph.geojson
```

Conferir quantidade de nos e arestas:

```bash
python3 -m json.tool src/caramelo_mapping/maps/${MAP_NAME}/route_graph.geojson | head
```

Regras do gerador:

- cada Service Area valida vira um no;
- poses `0,0,0` sao ignoradas;
- areas com `use_docking: true` usam a `approach_pose` como no de rota;
- START e FINISH usam a `final_robot_pose`;
- arestas sao criadas entre vizinhos proximos;
- `--max-neighbors` controla quantos vizinhos cada no conecta.

Exemplo conectando mais vizinhos:

```bash
python3 src/caramelo_navigation/scripts/sync_route_graph.py \
  --map-folder src/caramelo_mapping/maps/${MAP_NAME} \
  --max-neighbors 4
```

Importante:

- o Route Server so sobe se o grafo tiver pelo menos 1 no e 1 aresta;
- se o mapa ainda so tem placeholders, o launch pula o Route Server;
- se voce editar o GeoJSON manualmente, nao rode o gerador sem querer depois.

## 10. Build Final Depois Dos Arquivos Do Mapa

Sempre depois de criar ou editar mapa, Service Areas, docking ou route graph:

```bash
colcon build --symlink-install --packages-select caramelo_mapping caramelo_navigation
source install/setup.bash
```

## 11. Testar Navegacao Basica

Suba Nav2:

```bash
ros2 launch caramelo_navigation bringup.launch.py \
  map_name:=${MAP_NAME} \
  use_docking:=true \
  nav_profile:=dwb
```

Confira:

```bash
ros2 action list | grep navigate
ros2 topic echo /global_costmap/published_footprint --once
ros2 topic echo /local_costmap/published_footprint --once
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
ros2 topic echo /mecanum_controller/reference
```

No RViz:

1. ajuste pose inicial se necessario;
2. envie um goal curto para frente;
3. envie um goal lateral curto;
4. envie um goal com yaw final diferente;
5. confirme que o robo consegue girar no proprio eixo perto do goal;
6. confira se o footprint poligonal nao entra em obstaculos;
7. confira se a inflacao esta larga e suave.

Se quiser comparar MPPI:

```bash
ros2 launch caramelo_navigation bringup.launch.py \
  map_name:=${MAP_NAME} \
  use_docking:=true \
  nav_profile:=mppi
```

Use `mppi` apenas como perfil experimental. O perfil principal do robo real e
`dwb`.

## 12. Testar Markers De Service Areas

Com o bringup aberto, confira:

```bash
ros2 topic echo /caramelo/service_areas/markers --once
```

No RViz deve aparecer:

- nome da area;
- seta da pose final;
- ponto da approach pose;
- retangulo aproximado da mesa.

Se os markers nao aparecem:

```bash
ros2 launch caramelo_navigation bringup.launch.py \
  map_name:=${MAP_NAME} \
  use_service_area_markers:=true
```

## 13. Testar Docking

Suba o bringup com docking:

```bash
ros2 launch caramelo_navigation bringup.launch.py \
  map_name:=${MAP_NAME} \
  use_docking:=true \
  nav_profile:=dwb
```

Confira actions:

```bash
ros2 action list | grep dock
```

Teste docking em uma workstation:

```bash
ros2 run caramelo_navigation dock_to \
  --map-name ${MAP_NAME} \
  --dock-id WS1
```

Comportamento esperado:

1. Nav2 navega ate a staging/approach pose.
2. Docking Server faz aproximacao final curta e lenta.
3. Robo para em frente a mesa.

Para testar somente a aproximacao final, posicione o robo perto da area e rode:

```bash
ros2 run caramelo_navigation dock_to \
  --map-name ${MAP_NAME} \
  --dock-id WS1 \
  --no-nav-to-staging
```

Se o docking ficar perdido no final, ajuste nesta ordem:

1. confirme `final_robot_pose` da area;
2. confira yaw final do robo;
3. reduza ou aumente `approach_offset`;
4. aumente temporariamente `linear_tolerance`;
5. aumente temporariamente `angular_tolerance`;
6. rode `validate_service_areas --sync-docking`;
7. teste de novo.

## 14. Testar Route Server

Com o bringup aberto:

```bash
ros2 action list | grep -i route
ros2 topic list | grep route
```

No log do launch deve aparecer algo parecido com:

```text
Using route graph: .../route_graph.geojson (3 nodes, 6 edges)
Route server enabled: True
```

Se aparecer:

```text
Route server enabled: False
```

entao o grafo esta vazio ou invalido. Confira:

```bash
python3 src/caramelo_navigation/scripts/sync_route_graph.py \
  --map-folder src/caramelo_mapping/maps/${MAP_NAME}
```

Depois:

```bash
colcon build --symlink-install --packages-select caramelo_mapping caramelo_navigation
source install/setup.bash
```

## 15. Testar Waypoint Follower

O Waypoint Follower sobe por default com:

```bash
use_waypoint_follower:=true
```

Confira action:

```bash
ros2 action list | grep waypoint
```

O uso pratico para tarefas completas ainda depende do executor de tarefas da
equipe, mas o server ja deve estar disponivel para receber listas de poses.

## 16. Checklist Final Da Arena

Antes de chamar o mapa de pronto, confirme:

- `map.yaml` existe.
- `map.pgm` existe.
- `service_areas.yaml` existe.
- START tem pose real.
- FINISH tem pose real.
- WS/SH/PP/RT usados no teste tem pose real.
- Poses nao usadas podem continuar `0,0,0`.
- `validate_service_areas --sync-docking` passou.
- `docking.yaml` foi atualizado.
- `route_graph.geojson` foi gerado.
- `colcon build` foi rodado depois das alteracoes.
- Nav2 abre com `map_name:=<MAP_NAME>`.
- AMCL localiza o robo corretamente.
- Footprint aparece correto no RViz.
- Goal curto funciona.
- Goal lateral funciona.
- Giro no proprio eixo funciona.
- Docking em WS1 funciona.
- `/cmd_vel_nav`, `/cmd_vel_smoothed`, `/cmd_vel` e `/mecanum_controller/reference` aparecem.

## 17. Comandos Rapidos

Build:

```bash
colcon build --symlink-install --packages-select caramelo_mapping caramelo_navigation
source install/setup.bash
```

SLAM:

```bash
ros2 launch caramelo_mapping slam.launch.py map_name:=${MAP_NAME}
```

Salvar mapa:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f src/caramelo_mapping/maps/${MAP_NAME}/map
```

Nav2:

```bash
ros2 launch caramelo_navigation bringup.launch.py \
  map_name:=${MAP_NAME} \
  use_docking:=true
```

Salvar WS1:

```bash
ros2 run caramelo_navigation save_service_area_pose \
  --map ${MAP_NAME} \
  --name WS1 \
  --type workstation \
  --height 0.10 \
  --overwrite
```

Validar docking:

```bash
ros2 run caramelo_navigation validate_service_areas \
  --map ${MAP_NAME} \
  --sync-docking
```

Gerar route graph:

```bash
python3 src/caramelo_navigation/scripts/sync_route_graph.py \
  --map-folder src/caramelo_mapping/maps/${MAP_NAME}
```

Dockar:

```bash
ros2 run caramelo_navigation dock_to \
  --map-name ${MAP_NAME} \
  --dock-id WS1
```

## 18. Problemas Comuns

### Nao consigo salvar Service Area

Confira TF:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Se nao existe TF `map -> base_footprint`, o script nao consegue salvar a pose.

### Marker aparece no lugar errado

Provaveis causas:

- pose inicial do AMCL errada;
- pose salva durante SLAM antes de loop closure;
- `final_robot_pose` editada manualmente;
- `map_name` errado no launch.

Corrija reabrindo o mapa com Nav2 e salvando de novo com `--overwrite`.

### Docking aproxima torto

Confira:

- yaw salvo em `final_robot_pose`;
- `approach_offset`;
- tolerancias;
- se o robo esta chegando bem ate a approach pose;
- se a area correta foi usada no `dock_to`.

### Route Server nao sobe

O route graph precisa ter nos e arestas. Se todas as poses forem `0,0,0`, o
grafo fica vazio e o launch pula o Route Server.

### Robo planeja mas nao anda

Confira cadeia de comando:

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
ros2 topic echo /mecanum_controller/reference
```

Se `/cmd_vel` existe mas `/mecanum_controller/reference` nao muda, suba com:

```bash
use_cmd_vel_relay:=true
```

### Robo gira demais perto do goal

Teste:

- yaw final da seta no RViz;
- tolerancia do `general_goal_checker`;
- se o goal esta muito perto de obstaculo;
- se o path do Smac esta fazendo curva desnecessaria.

O perfil atual usa `SmacLattice + DWB` e primitives omni custom com
`turning_radius: 0.0`.

