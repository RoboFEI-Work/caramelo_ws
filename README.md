# Caramelo WS

Workspace ROS 2 Jazzy do robo Caramelo para mapeamento, localizacao e navegacao.

Este workspace roda no computador de operacao. Ele nao inicia o driver do LiDAR, IMU ou controladores da Raspberry; ele consome os topicos que ja chegam do workspace embarcado.

## Topicos Esperados

Antes de mapear ou navegar, confira se a Raspberry esta publicando pelo menos:

- `/scan`: LaserScan do LiDAR.
- `/odom`: odometria usada pelo Nav2/SLAM.
- `/tf` e `/tf_static`: arvore TF do robo.
- `/robot_description`: modelo do robo para o RViz.
- `/joint_states`: estados das juntas.
- `/mecanum_controller/reference`: comando `TwistStamped` usado pelo controlador mecanum.

Comandos rapidos de diagnostico:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

Se `/scan`, `/odom` ou o TF `odom -> base_footprint` nao existirem, resolva isso no workspace da Raspberry antes de iniciar SLAM/Nav2.

## Preparar o Workspace

Em todo terminal novo:

```bash
cd ~/caramelo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Quando alterar codigo, launch, config ou adicionar mapas:

```bash
cd ~/caramelo_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Para buildar somente os pacotes mais usados:

```bash
colcon build --packages-select caramelo_mapping caramelo_localization caramelo_navigation caramelo_utils
source install/setup.bash
```

## Mapas Disponiveis

Os mapas ficam em:

```text
src/caramelo_mapping/maps/<nome_do_mapa>/map.yaml
```

Mapas presentes no workspace:

- `arena_fei`
- `cbr_vitoria`
- `robocup_salvador`
- `sala_520`
- `small_house`

## Fluxo 1: Mapear com SLAM Toolbox

Use este fluxo como padrao do projeto. Ele abre o `slam_toolbox`, o `map_saver_server`, o RViz e um adaptador `/scan -> /scan_fixed`.

O adaptador existe porque alguns drivers de LiDAR publicam `/scan` com quantidade variavel de leituras. O `slam_toolbox` espera uma geometria fixa, entao o workspace republica uma grade angular fixa em `/scan_fixed` sem mexer no pacote da Raspberry.

### 1. Iniciar SLAM Toolbox

Terminal 1:

```bash
ros2 launch caramelo_mapping slam.launch.py
```

Parametros principais:

- `use_sim_time`: default `false`.
- `map_resolution`: default `0.02`, ou seja, 2 cm por celula.
- `use_rviz`: default `true`.
- `use_docking`: default `false`, sobe o Nav2 Docking Server usando o `docking.yaml` do mapa.
- `scan_topic`: default `/scan`, topico bruto vindo da Raspberry.
- `slam_scan_topic`: default `/scan_fixed`, topico usado pelo SLAM Toolbox.
- `use_scan_normalizer`: default `true`.
- `fixed_scan_count`: default `0`, aprende a quantidade de raios pelo primeiro scan.
- `scan_range_max`: default `30.0`, alcance maximo do LiDAR em metros.

Exemplo fixando manualmente a quantidade de raios, caso o primeiro scan venha ruim:

```bash
ros2 launch caramelo_mapping slam.launch.py fixed_scan_count:=3217
```

Exemplo sem RViz:

```bash
ros2 launch caramelo_mapping slam.launch.py use_rviz:=false
```

### 2. Conferir no RViz

No RViz do SLAM, confira:

- `Fixed Frame`: `map`.
- Mapa em `/map`.
- Laser em `/scan_fixed`.
- Grafo do SLAM em `/slam_toolbox/graph_visualization`.
- TF sem erro entre `map`, `odom`, `base_footprint` e frame do laser.

Se o mapa so cria o primeiro frame e para, veja se o `/scan_fixed` existe:

```bash
ros2 topic echo /scan_fixed --once
```

### 3. Movimentar o Robo

Terminal 2:

```bash
ros2 launch caramelo_utils teleop.launch.py
```

Dicas para mapa melhor:

- ande devagar;
- evite girar muito rapido;
- passe pelas paredes e corredores;
- volte por regioes ja mapeadas para fechar loop;
- evite pessoas passando perto do LiDAR durante o mapeamento;
- nao empurre o robo manualmente.

### 4. Salvar o Mapa

Escolha o nome do mapa. Exemplo: `minha_sala`.

Terminal 3:

```bash
mkdir -p src/caramelo_mapping/maps/minha_sala
ros2 run nav2_map_server map_saver_cli -f src/caramelo_mapping/maps/minha_sala/map
```

Isso cria:

```text
src/caramelo_mapping/maps/minha_sala/map.yaml
src/caramelo_mapping/maps/minha_sala/map.pgm
```

Depois instale o mapa:

```bash
colcon build --packages-select caramelo_mapping
source install/setup.bash
```

## Fluxo 2: Mapear com Cartographer

Use este fluxo quando quiser comparar com o SLAM Toolbox ou quando o ambiente tiver mais objetos dinamicos. O Cartographer usa `/scan` e `/odom` diretamente e publica o mapa via `cartographer_occupancy_grid_node`.

### 1. Iniciar Cartographer

Terminal 1:

```bash
ros2 launch caramelo_cartographer cartographer.launch.py
```

Parametros principais:

- `use_sim_time`: default `false`.
- `use_rviz`: default `true`.
- `scan_topic`: default `/scan`.
- `odom_topic`: default `/odom`.
- `resolution`: default `0.05`.
- `publish_period_sec`: default `1.0`.
- `configuration_basename`: default `turtlebot3_lds_2d.lua`.

Exemplo com mapa publicado a 2 cm por celula:

```bash
ros2 launch caramelo_cartographer cartographer.launch.py resolution:=0.02
```

Exemplo usando outro topico de scan:

```bash
ros2 launch caramelo_cartographer cartographer.launch.py scan_topic:=/scan odom_topic:=/odom
```

### 2. Movimentar e Salvar

Use o mesmo teleop:

```bash
ros2 launch caramelo_utils teleop.launch.py
```

Para salvar o mapa, use o mesmo comando do Nav2:

```bash
mkdir -p src/caramelo_mapping/maps/minha_sala_cartographer
ros2 run nav2_map_server map_saver_cli -f src/caramelo_mapping/maps/minha_sala_cartographer/map
colcon build --packages-select caramelo_mapping
source install/setup.bash
```

## Qual Mapeamento Usar?

Use SLAM Toolbox quando:

- voce quer o fluxo padrao do workspace;
- quer ver o grafo do SLAM no RViz;
- quer usar resolucao de 2 cm por padrao;
- o `/scan_fixed` esta funcionando bem.

Use Cartographer quando:

- voce quer comparar qualidade de mapa;
- o ambiente tem pessoas ou objetos dinamicos;
- voce quer testar uma alternativa que atualiza submaps de forma diferente;
- o SLAM Toolbox estiver sensivel demais ao LiDAR.

Depois de salvar o mapa, a navegacao com Nav2 e a mesma para mapas criados por qualquer uma das duas ferramentas.

## Fluxo 3: Navegar com Nav2

Use este fluxo quando ja existe um mapa salvo em `src/caramelo_mapping/maps/<map_name>/map.yaml`.

### 1. Iniciar Bringup de Navegacao

Terminal 1:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520
```

Esse launch sobe:

- AMCL;
- `map_server`;
- Nav2: `controller_server`, `planner_server`, `smoother_server`, `bt_navigator`, `behavior_server`;
- relay de `/cmd_vel` para `/mecanum_controller/reference`;
- RViz.

Parametros principais:

- `map_name`: default `small_house`.
- `use_sim_time`: default `false`.
- `use_rviz`: default `true`.
- `navigation_start_delay`: default `5.0`.
- `costmap_resolution`: default `0.02`, usa 2 cm por celula. Passe `map` para usar a resolucao do `map.yaml`.
- `use_cmd_vel_relay`: default `true`.
- `cmd_vel_topic`: default `/cmd_vel`.
- `mecanum_reference_topic`: default `/mecanum_controller/reference`.

Exemplos:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=small_house
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_fei
```

Sem RViz:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520 use_rviz:=false
```

Usando a resolucao armazenada no mapa:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520 costmap_resolution:=map
```

### Nota Tecnica: Inflacao Curta

O Caramelo usa propositalmente uma inflacao curta para manter a zona critica
visual grudada nos obstaculos:

```yaml
inflation_radius: 0.245
cost_scaling_factor: 30.0
```

Com o footprint atual, o raio circunscrito e aproximadamente `0.426 m`.
Por isso, Smac e MPPI exibem um warning informando que a inflacao e menor que
o recomendado para otimizar a verificacao de colisao. O warning e esperado
nesta configuracao: a verificacao continua usando o footprint real, mas pode
consumir mais CPU durante planejamento e controle.

Esta escolha foi feita para evitar uma faixa azul ampla no RViz e permitir
passagens mais justas. Considere reverter se aparecerem timeouts de planejamento,
quedas frequentes na taxa do controller, alto uso de CPU ou navegacao lenta em
corredores estreitos.

Para voltar ao perfil mais conservador, ajuste `controller_server.yaml`,
`planner_server.yaml` e `costmap.yaml` para:

```yaml
inflation_radius: 0.44
cost_scaling_factor: 12.0
```

No bloco `FollowPath` de `controller_server.yaml`, mantenha os parametros
coerentes com a inflacao revertida:

```yaml
cost_scaling_dist: 0.44
inflation_cost_scaling_factor: 12.0
```

Depois de qualquer mudanca, recompile e reinicie o bringup:

```bash
colcon build --packages-select caramelo_navigation
source install/setup.bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520
```

### 2. Definir Pose Inicial

No RViz:

1. Clique em `2D Pose Estimate`.
2. Clique na posicao real do robo no mapa.
3. Arraste para apontar a orientacao correta.
4. Espere as particulas do AMCL ficarem perto do robo.

Se a pose inicial estiver errada, o Nav2 pode planejar corretamente no mapa, mas o robo vai tentar navegar a partir do lugar errado.

### 3. Enviar Objetivo

No RViz voce pode usar:

- `Nav2 Goal`, recomendado para acionar o `NavigateToPose` do Nav2.
- `2D Goal Pose`, tambem funciona quando o plugin esta conectado ao Nav2.

Depois de enviar o objetivo, confira:

```bash
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel
ros2 topic echo /mecanum_controller/reference
```

## Fluxo 4: Docking nas Workstations da Competicao

Use este fluxo para configurar pontos de parada de frente nas Service Areas da RoboCup@Work e depois mandar o robo dockar usando apenas o ID oficial da area.

O banco de docking fica dentro da pasta de cada mapa:

```text
src/caramelo_mapping/maps/<map_name>/
  map.yaml
  map.pgm
  docking.yaml
  service_areas.yaml
```

IDs aceitos pelo projeto:

- `START` e `FINISH`;
- `WS1`, `WS2`, `WS3`... para Workstations;
- `SH1`, `SH2`... para Shelves;
- `PP1`, `PP2`... para Precise Placement;
- `RT1`, `RT2`... para Rotating Tables.

Nao use `workstation_01` ou nomes parecidos como ID principal. Use a nomenclatura do rulebook.

### 1. Inicializar o mapa para docking

Rode uma vez por mapa:

```bash
ros2 run caramelo_navigation init_map_docking --map-name sala_520
```

Isso cria, se ainda nao existirem:

- `src/caramelo_mapping/maps/sala_520/docking.yaml`;
- `src/caramelo_mapping/maps/sala_520/service_areas.yaml`.

Tambem existe inicializacao automatica: se voce subir o Docking Server com um mapa que ja tem `map.yaml`, mas ainda nao tem `docking.yaml`, o launch cria os dois arquivos iniciais automaticamente:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=meu_mapa use_docking:=true
```

Mesmo assim, para conferir e ajustar os nomes antes da competicao, eu recomendo rodar:

```bash
ros2 run caramelo_navigation list_docks --map-name meu_mapa
```

### 2. Subir Nav2 com Docking Server

```bash
ros2 launch caramelo_navigation bringup.launch.py \
  map_name:=sala_520 \
  use_docking:=true
```

Se quiser subir somente o Docking Server, sem abrir outro Nav2:

```bash
ros2 launch caramelo_navigation docking_server.launch.py map_name:=sala_520
```

### 3. Salvar a pose final de uma Workstation

Coloque o robo parado de frente para a workstation, exatamente na pose final desejada. Depois rode:

```bash
ros2 run caramelo_navigation save_dock_pose \
  --map-name sala_520 \
  --dock-id WS1 \
  --dock-type caramelo_front_dock \
  --update-service-area \
  --kind WS
```

Exemplos para outros tipos:

```bash
ros2 run caramelo_navigation save_dock_pose --map-name sala_520 --dock-id SH1 --dock-type caramelo_shelf_front_dock --update-service-area --kind SH
ros2 run caramelo_navigation save_dock_pose --map-name sala_520 --dock-id PP1 --dock-type caramelo_precision_front_dock --update-service-area --kind PP --precision high
ros2 run caramelo_navigation save_dock_pose --map-name sala_520 --dock-id RT1 --dock-type caramelo_precision_front_dock --update-service-area --kind RT --diameter 0.60 --precision high
```

O script busca a TF `map -> base_link`. Se `base_link` falhar, tenta `base_footprint`. Antes de atualizar `docking.yaml`, ele cria `docking.yaml.bak`.

### 4. Salvar pose por topico

Terminal 1:

```bash
ros2 run caramelo_navigation dock_pose_recorder_node --ros-args -p map_name:=sala_520
```

Terminal 2:

```bash
ros2 topic pub --once /save_dock_pose std_msgs/msg/String "{data: 'WS1'}"
```

Tambem pode informar o tipo no topico:

```bash
ros2 topic pub --once /save_dock_pose std_msgs/msg/String "{data: 'SH1:caramelo_shelf_front_dock'}"
```

### 5. Conferir docks salvos

```bash
ros2 run caramelo_navigation list_docks --map-name sala_520
```

### 6. Mandar dockar por ID

Com o Nav2 e o Docking Server ativos:

```bash
ros2 run caramelo_navigation dock_to --dock-id WS1
```

Com validacao no banco do mapa:

```bash
ros2 run caramelo_navigation dock_to --map-name sala_520 --dock-id WS1
```

Se o robo ja estiver perto da staging pose:

```bash
ros2 run caramelo_navigation dock_to --dock-id WS1 --no-nav-to-staging
```

O comportamento esperado e: o Nav2 navega ate a staging pose e o Docking Server faz a aproximacao final de frente, devagar.

### 7. Navegar entre docks salvos

Depois que voce salvou `WS1`, `WS2`, `SH1`, `PP1` etc., navegar entre elas vira uma sequencia de comandos por ID.

Se o robo esta livre no meio da arena, basta mandar o proximo dock:

```bash
ros2 run caramelo_navigation dock_to --map-name sala_520 --dock-id WS1
ros2 run caramelo_navigation dock_to --map-name sala_520 --dock-id WS2
ros2 run caramelo_navigation dock_to --map-name sala_520 --dock-id SH1
```

Se o robo esta encostado/dockado em uma workstation, primeiro afaste ele da dock atual e depois mande a proxima:

```bash
ros2 run caramelo_navigation undock --dock-type caramelo_front_dock
ros2 run caramelo_navigation dock_to --map-name sala_520 --dock-id WS2
```

Para shelf ou precise placement, use o tipo correspondente no `undock` quando precisar:

```bash
ros2 run caramelo_navigation undock --dock-type caramelo_shelf_front_dock
ros2 run caramelo_navigation undock --dock-type caramelo_precision_front_dock
```

Dica pratica para competicao: salve todas as poses finais com `save_dock_pose`, confira com `list_docks`, suba o bringup com `use_docking:=true`, e durante a prova chame apenas `dock_to --dock-id <AREA>`.


### Mapa novo: fluxo mais automatico

Depois de salvar um mapa novo em `src/caramelo_mapping/maps/meu_mapa/map.yaml`, faca:

```bash
colcon build --packages-select caramelo_mapping caramelo_navigation
source install/setup.bash
ros2 run caramelo_navigation init_map_docking --map-name meu_mapa
ros2 run caramelo_navigation list_docks --map-name meu_mapa
```

Ou deixe o launch criar os arquivos iniciais automaticamente:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=meu_mapa use_docking:=true
```

Depois, posicione o robo na frente de cada Service Area e salve as poses reais:

```bash
ros2 run caramelo_navigation save_dock_pose --map-name meu_mapa --dock-id WS1 --dock-type caramelo_front_dock --update-service-area --kind WS
ros2 run caramelo_navigation save_dock_pose --map-name meu_mapa --dock-id WS2 --dock-type caramelo_front_dock --update-service-area --kind WS
ros2 run caramelo_navigation save_dock_pose --map-name meu_mapa --dock-id FINISH --dock-type caramelo_front_dock --update-service-area --kind FINISH
```

Os arquivos ficam junto do mapa, entao cada arena tem seu proprio banco de docks.

Parametros de seguranca usados no Docking Server:

- `v_linear_min: 0.03`;
- `v_linear_max: 0.10`;
- `v_angular_max: 0.30`;
- `slowdown_radius: 0.35`;
- `use_collision_detection: true`.

## Launches Separados

Somente SLAM Toolbox:

```bash
ros2 launch caramelo_mapping slam.launch.py
```

Somente Cartographer:

```bash
ros2 launch caramelo_cartographer cartographer.launch.py
```

Somente localizacao com AMCL:

```bash
ros2 launch caramelo_localization global_localization.launch.py map_name:=sala_520
```

Somente stack Nav2 e localizacao, sem RViz do bringup:

```bash
ros2 launch caramelo_navigation navigation.launch.py map_name:=sala_520
```

Teleop:

```bash
ros2 launch caramelo_utils teleop.launch.py
```

## Problemas Comuns

### O mapa nao aparece no RViz

```bash
ros2 topic echo /map --once
```

Se nao chegar nada, veja se o SLAM, Cartographer ou `map_server` esta ativo.

### O SLAM Toolbox mapeia so o primeiro frame

Confira se o normalizador esta publicando:

```bash
ros2 topic echo /scan_fixed --once
```

Se `/scan_fixed` nao existir, rode o SLAM com:

```bash
ros2 launch caramelo_mapping slam.launch.py use_scan_normalizer:=true
```

Se ainda falhar, fixe a quantidade de raios observada no log:

```bash
ros2 launch caramelo_mapping slam.launch.py fixed_scan_count:=3217
```

### O robo planeja mas nao anda

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /mecanum_controller/reference
ros2 topic echo /odom --once
```

Se `/cmd_vel` existe mas `/mecanum_controller/reference` nao muda, confira `use_cmd_vel_relay:=true` no bringup.

### O robo acha que chegou sem se mover

Normalmente e problema de tempo ou TF. Confira:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /odom --once
```

### O mapa salvo nao carrega

Confira:

- existe `src/caramelo_mapping/maps/<map_name>/map.yaml`;
- o pacote foi rebuildado depois de criar o mapa;
- voce rodou `source install/setup.bash`;
- o launch foi chamado com `map_name:=<map_name>`.
