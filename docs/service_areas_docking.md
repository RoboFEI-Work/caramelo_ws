# Service Areas E Docking Do Caramelo

Service Area e o ponto da arena onde o robo deve parar para manipular uma mesa
ou finalizar uma etapa. No Caramelo, a pose salva e sempre a pose final do robo
parado em frente a area.

Importante: `final_robot_pose` nao e a pose da mesa. A mesa fica a frente do
robo, na distancia escolhida pela equipe para manipulacao.

## Arquivo Por Mapa

Cada mapa guarda suas areas junto do mapa:

```text
src/caramelo_mapping/maps/<mapa>/map.yaml
src/caramelo_mapping/maps/<mapa>/map.pgm
src/caramelo_mapping/maps/<mapa>/service_areas.yaml
```

O arquivo humano principal e `service_areas.yaml`. O `docking.yaml` e derivado
dele e existe apenas para o Nav2 Docking Server.

## Mapa Primeiro Ou Simultaneo?

O fluxo recomendado e hibrido:

1. Durante o mapeamento, voce pode salvar poses provisorias para START, FINISH,
   WS, SH, PP e RT.
2. Depois que o mapa estiver salvo e estavel, voce deve reabrir esse mapa com
   Nav2/AMCL e sobrescrever as poses finais.

Isso acontece porque o SLAM ainda pode ajustar o frame `map` enquanto fecha loop
e melhora o mapa. Se voce salvar uma Service Area muito cedo, a pose pode ficar
um pouco deslocada depois que o mapa for otimizado.

Regra pratica:

- para rascunho e organizacao: salve durante o SLAM;
- para competicao e docking preciso: salve ou sobrescreva depois do mapa final.

## Fluxo Recomendado Para Um Mapa Novo

### 1. Subir SLAM Com Markers

```bash
ros2 launch caramelo_mapping slam.launch.py map_name:=arena_teste
```

Esse launch abre o SLAM, RViz e os markers de Service Areas.

### 2. Salvar Poses Provisorias Durante O SLAM

Pare o robo em frente a area, alinhado com a mesa, e salve:

```bash
ros2 run caramelo_navigation save_service_area_pose --map arena_teste --name START --type start --height 0.00
ros2 run caramelo_navigation save_service_area_pose --map arena_teste --name WS1 --type workstation --height 0.10
ros2 run caramelo_navigation save_service_area_pose --map arena_teste --name FINISH --type finish --height 0.00
```

Se ja existir uma pose real e voce quiser atualizar:

```bash
ros2 run caramelo_navigation save_service_area_pose --map arena_teste --name WS1 --type workstation --height 0.10 --overwrite
```

### 3. Salvar O Mapa

```bash
mkdir -p src/caramelo_mapping/maps/arena_teste
ros2 run nav2_map_server map_saver_cli -f src/caramelo_mapping/maps/arena_teste/map
colcon build --packages-select caramelo_mapping caramelo_navigation
source install/setup.bash
```

### 4. Reabrir O Mapa Com Nav2

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena_teste use_docking:=true
```

Use o teleop para levar o robo novamente ate a posicao final real de cada area e
sobrescreva as poses:

```bash
ros2 run caramelo_navigation save_service_area_pose --map arena_teste --name WS1 --type workstation --height 0.10 --overwrite
ros2 run caramelo_navigation save_service_area_pose --map arena_teste --name SH1 --type shelf --height 0.10 --overwrite
ros2 run caramelo_navigation save_service_area_pose --map arena_teste --name PP1 --type precision_placement --height 0.10 --overwrite
ros2 run caramelo_navigation save_service_area_pose --map arena_teste --name RT1 --type rotating_table --height 0.10 --overwrite
```

### 5. Validar O Banco De Areas

```bash
ros2 run caramelo_navigation list_service_areas --map arena_teste
ros2 run caramelo_navigation validate_service_areas --map arena_teste --sync-docking
```

## Salvar Uma Pose

Leve o robo manualmente ate a posicao ideal em frente a mesa, alinhe o robo e
salve a pose atual:

```bash
ros2 run caramelo_navigation save_service_area_pose --map sala_520 --name WS1 --type workstation --height 0.10
```

Para atualizar uma pose ja salva:

```bash
ros2 run caramelo_navigation save_service_area_pose --map sala_520 --name WS1 --type workstation --height 0.10 --overwrite
```

START e FINISH usam o mesmo comando, mudando o tipo:

```bash
ros2 run caramelo_navigation save_service_area_pose --map sala_520 --name START --type start --height 0.00
ros2 run caramelo_navigation save_service_area_pose --map sala_520 --name FINISH --type finish --height 0.00
```

Alturas aceitas para areas com mesa:

```text
0.00, 0.05, 0.10, 0.15
```

## Approach Pose

A `approach_pose` e calculada automaticamente. Ela fica alguns centimetros antes
da `final_robot_pose`, voltando no eixo oposto ao yaw final do robo.

Exemplo: se `approach_offset` for `0.45`, o robo navega primeiro ate uma pose
0.45 m atras da pose final. Depois o Docking Server faz a aproximacao precisa.

## Visualizar No RViz

Durante SLAM:

```bash
ros2 launch caramelo_mapping slam.launch.py map_name:=sala_520
```

Durante navegacao:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520 use_docking:=true
```

O RViz mostra o topico:

```text
/caramelo/service_areas/markers
```

Markers principais:

- texto com o nome da area;
- ponto da `final_robot_pose`;
- seta do yaw final;
- ponto da `approach_pose`;
- retangulo aproximado da mesa.

## Validar E Sincronizar Docking

Depois de editar `service_areas.yaml` manualmente:

```bash
ros2 run caramelo_navigation validate_service_areas --map sala_520 --sync-docking
```

Isso recria o `docking.yaml` tecnico usado pelo Docking Server.

### Cuidados Aprendidos Em 2026-10-26

Detalhes em `docs/diagnostico_docking_tour_2026-10-26.md` (F18, F19) e na secao
"Tour de 2026-10-26" de `docking_alinhamento_holonomico.md`.

- **Os dois arquivos divergem em silencio.** A GUI (`/save_dock_pose`) e o
  `a1d8e07` gravaram a WS5 so no `docking.yaml`; o `service_areas.yaml` ficou
  com a pose antiga, 8,8 cm ao lado, por dois meses. Um `--sync-docking` feito
  por qualquer outro motivo teria revertido a WS5. Antes de sincronizar, confira
  pose a pose (`final_robot_pose` x `docks.<id>.pose`); em 2026-10-26 a WS5 foi
  igualada ao `docking.yaml` e hoje os dez docks sao identicos.
- **Yaw gravado nao e a normal da mesa.** Pelo `map.pgm`, o yaw gravado fica
  ate 5 graus fora da normal da face desenhada (WS1 ~5, WS6 3-4, PP1 3-4,
  FINISH 6) e a distancia gravada ate a face vai de 0,29 m (WS6) a 0,43 m (WS4).
  O align esquadra pela face e para a 0,38 m pelo LiDAR, entao essas folgas
  aparecem como `Yaw`/`Long` no relatorio do `tour_docks` sem que o docking
  esteja errado. Para regravar (M3 do diagnostico): robo esquadrado pelo LiDAR
  (`dock_pose_recorder_node`), depois `service_areas.yaml` + sync, nunca so um.
- **Sinais no relatorio x no align.** `Long/Lat/Yaw` do tour = robo - dock no
  frame do dock (Lat + = robo a esquerda); `final_x/y/yaw_error` do align =
  alvo - robo no frame do robo. Mesma medida, sinal trocado (F7).

## Testar Docking

Suba a navegacao com docking:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520 use_docking:=true
```

Confira se a action existe:

```bash
ros2 action list | grep dock
```

Mande o robo usar uma area:

```bash
ros2 run caramelo_navigation dock_to --map-name sala_520 --dock-id WS1
```

Parametros marcados para ajustar no robo:

- `approach_offset`;
- `linear_tolerance`;
- `angular_tolerance`;
- pose real de cada `final_robot_pose`.

## Onde Ficam Os Parametros

- Docking Server e alinhamento fino (`dock_align_node`): lista `parameters` em
  `src/caramelo_navigation/launch/docking_server.launch.py` (velocidades,
  tolerancias, `refine_desired_face_dist`, `nudge_*`). Nao copie valores para
  docs: o launch e a fonte. Detalhes do alinhamento: `docking_alinhamento_holonomico.md`.
- Por area: `approach_offset`, `linear_tolerance`, `angular_tolerance` e
  `final_robot_pose` ficam em `service_areas.yaml`; depois rode
  `validate_service_areas --map <mapa> --sync-docking`.
- Dois jeitos de gravar a pose de um dock: pela GUI ("Salvar pose como dock",
  topico `/save_dock_pose`, grava SO `docking.yaml`) ou por `service_areas.yaml`
  + sync (`save_service_area_pose`, `validate_service_areas --sync-docking`).
  Um sync a partir de `service_areas.yaml` regrava `docking.yaml` do zero e
  descarta o que foi salvo so pela GUI — decisao pendente no README da raiz.

## Route Graph (Opcional)

O Nav2 Route Server usa `src/caramelo_mapping/maps/<mapa>/route_graph.geojson`,
derivado de `service_areas.yaml` (poses `0,0,0` sao ignoradas):

```bash
ros2 run caramelo_navigation sync_route_graph --map-folder src/caramelo_mapping/maps/<mapa>
```

O launch so sobe o Route Server (`use_route_server`, default true) se o grafo
tiver pelo menos 1 no e 1 aresta; senao pula sem quebrar o Nav2. Nada no fluxo
de missao (`run_mission`) depende dele. Passo a passo e edicao manual do GeoJSON:
`passo_a_passo_mapa_service_areas_docking_route.md` secoes 9 e 14.
