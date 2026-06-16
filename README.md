# Caramelo WS

Workspace ROS 2 do robo Caramelo para mapeamento, localizacao e navegacao.

Este README foca no uso pratico do workspace:

- como mapear um ambiente novo;
- como navegar com o robo usando um mapa salvo.

Os comandos abaixo assumem ROS 2 Jazzy e que o robo ja esta publicando os topicos principais:

- `/scan` para o LiDAR;
- `/odom` para odometria;
- TF entre `map`, `odom`, `base_footprint`/`base_link` e o frame do laser;
- `/cmd_vel` ou o pipeline de controle configurado no robo.

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

## Fluxo 1: Mapear um Ambiente

Use este fluxo quando voce quer criar um mapa novo de uma sala, corredor ou laboratorio.

### 1. Ligue o robo e confira os topicos

Com o robo ligado, confira se chegam dados basicos:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

Se `/scan` ou `/odom` nao aparecerem, resolva isso antes de abrir o SLAM.

### 2. Abra o SLAM

Terminal 1:

```bash
ros2 launch caramelo_mapping slam.launch.py
```

Parametros uteis:

- `use_sim_time` default: `false`
- `slam_config` default: `caramelo_mapping/config/slam_toolbox.yaml`

Exemplo usando tempo de simulacao:

```bash
ros2 launch caramelo_mapping slam.launch.py use_sim_time:=true
```

### 3. Abra o RViz do SLAM

Se o RViz nao abrir automaticamente no seu fluxo, rode:

```bash
rviz2 -d $(ros2 pkg prefix caramelo_mapping)/share/caramelo_mapping/rviz/slam.rviz
```

No RViz, confira:

- mapa aparecendo em `/map`;
- laser em `/scan`;
- TF sem erro;
- o mapa crescendo enquanto o robo se move.

### 4. Movimente o robo devagar

Terminal 2:

```bash
ros2 launch caramelo_utils teleop.launch.py
```

Use movimentos lentos e suaves. Para mapas melhores:

- passe pelas bordas do ambiente;
- evite girar rapido;
- volte por regioes ja mapeadas para fechar loop;
- nao empurre o robo manualmente enquanto ele mapeia.

### 5. Salve o mapa

Escolha um nome para o mapa. Exemplo: `sala_520`.

Terminal 3:

```bash
mkdir -p src/caramelo_mapping/maps/sala_520
ros2 run nav2_map_server map_saver_cli -f src/caramelo_mapping/maps/sala_520/map
```

Isso deve criar:

- `src/caramelo_mapping/maps/sala_520/map.yaml`
- `src/caramelo_mapping/maps/sala_520/map.pgm`

Depois instale o mapa no workspace:

```bash
colcon build --packages-select caramelo_mapping
source install/setup.bash
```

## Fluxo 2: Navegar com o Robo

Use este fluxo quando voce ja tem um mapa salvo em `src/caramelo_mapping/maps/<nome_do_mapa>/map.yaml`.

Mapas existentes neste workspace:

- `small_house`
- `sala_520`

### 1. Inicie a navegacao completa

Terminal 1:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520
```

Esse launch abre:

- localizacao global com AMCL;
- `map_server`;
- Nav2 (`controller_server`, `planner_server`, `smoother_server`, `bt_navigator`, `behavior_server`);
- RViz.

Parametros uteis:

- `map_name` default: `small_house`
- `use_sim_time` default: `false`
- `use_rviz` default: `true`
- `rviz_config` default: `caramelo_localization/rviz/global_localization.rviz`

Exemplo sem RViz:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520 use_rviz:=false
```

### 2. Defina a pose inicial no RViz

No RViz:

1. Clique em `2D Pose Estimate`.
2. Clique no mapa onde o robo esta fisicamente.
3. Arraste para apontar a orientacao correta do robo.
4. Confira se a nuvem de particulas fica perto da posicao real.

Se a pose inicial estiver errada, a navegacao vai planejar errado mesmo com o mapa certo.

### 3. Envie um objetivo de navegacao

No RViz:

1. Clique em `2D Goal Pose`.
2. Clique no destino desejado no mapa.
3. Arraste para definir a orientacao final.

O Nav2 deve publicar o plano e comandar o robo ate o destino.

### 4. Acompanhe os topicos principais

Em outro terminal, se quiser diagnosticar:

```bash
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel
ros2 lifecycle nodes
```

Topicos importantes:

- `/map`: mapa carregado;
- `/particle_cloud`: particulas do AMCL;
- `/plan`: caminho planejado;
- `/cmd_vel`: comando de velocidade;
- `/scan`: leitura do LiDAR.

## Launches Separados

Somente localizacao com AMCL:

```bash
ros2 launch caramelo_localization global_localization.launch.py map_name:=sala_520
```

Somente stack Nav2 sem RViz:

```bash
ros2 launch caramelo_navigation navigation.launch.py map_name:=sala_520
```

Somente SLAM Toolbox:

```bash
ros2 launch caramelo_mapping slam.launch.py
```

Cartographer 2D:

```bash
ros2 launch caramelo_cartographer cartographer.launch.py
```

Teleop:

```bash
ros2 launch caramelo_utils teleop.launch.py
```

## Dicas de Problemas Comuns

Se o mapa nao aparece:

```bash
ros2 topic echo /map --once
```

Se o robo nao localiza:

```bash
ros2 topic echo /particle_cloud --once
ros2 run tf2_tools view_frames
```

Se o robo planeja mas nao anda:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom --once
```

Se o mapa salvo nao carrega:

- confira se existe `src/caramelo_mapping/maps/<map_name>/map.yaml`;
- rode `colcon build --packages-select caramelo_mapping`;
- rode `source install/setup.bash`;
- chame o launch com `map_name:=<map_name>`.

## Exemplo Completo

Mapear uma sala chamada `lab_novo`:

```bash
ros2 launch caramelo_mapping slam.launch.py
ros2 launch caramelo_utils teleop.launch.py
mkdir -p src/caramelo_mapping/maps/lab_novo
ros2 run nav2_map_server map_saver_cli -f src/caramelo_mapping/maps/lab_novo/map
colcon build --packages-select caramelo_mapping
source install/setup.bash
```

Navegar usando esse mapa:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=lab_novo
```
