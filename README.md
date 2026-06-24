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
