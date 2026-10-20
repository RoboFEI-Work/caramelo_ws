# Place em superfície livre — a câmera escolhe onde soltar (2026-10-19)

**Estado: implementado e testado em bancada (gtest + pytest + fotos reais da RealSense +
missão em MOCK). NÃO testado no robô.** Roteiro de validação no robô na §7.

## 0. O que muda (30 segundos)

Antes, o place em mesa comum (`WS_*`) escolhia entre **3 poses fixas** do SRDF
(`MesaX`, `MesaX.1`, `MesaX.2`) pela distância às AprilTags vistas. Não sabia onde a mesa
acaba, não via objeto sem tag nem parede, e só soltava em 3 pontos.

Agora o robô **olha o tampo** com a RealSense do punho, monta um **mapa top-down da mesa**
(1 cm por célula: tampo livre / objeto / fora da mesa / não visto) e escolhe um ponto (x, y)
que cumpre o rulebook (RoboCup@Work 4.4, *Manipulation Zone*):

| Regra | Como entra |
|---|---|
| Zona de manipulação: até **20 cm** da frente da service area | profundidade do centro do cubo em [2 cm + ½ cubo, 20 cm − ½ cubo] a partir da reta da frente |
| **2 cm** da borda da service area até a zona | cubo (círculo circunscrito, r = 29 mm) a ≥ 2 cm de qualquer célula "fora da mesa" |
| **2 cm** entre objetos (e entre objeto e container) | ≥ 2 cm de qualquer célula "objeto" (câmera), tag vista por TF, container (`ct_*`) e slot da memória |
| **10 cm** da parede até o começo da zona | lados declarados em `ws_walls` (mapa) ou detectados pelo LiDAR → x do cubo a ≥ 10 cm da borda/parede desse lado |
| Objeto nunca parcialmente fora do visto | o cubo inteiro precisa estar em células **vistas** (o que está fora da imagem não conta) |

Depois o braço vai à pose de elevação (5 cm acima, j1/j4/j5 primeiro, j2/j3 por último),
desce na vertical, confere a chegada por FK, solta, sobe na vertical e recolhe — igual ao
place em container. Sem o nó de superfície no ar, mesa não vista ou sem espaço: cai nas
**3 poses fixas** de sempre (byte a byte o fluxo de 21/08).

Vale só para **mesa comum** (`WS_*`). Precision placement (`PP_*`, posições pré-definidas),
prateleira (`SH_*`), container por cor (`container_color`) e empilhar (`stack_on`) seguem como
antes — o rulebook isenta esses casos da regra dos 2 cm.

## 1. Arquivos

| Papel | Arquivo |
|---|---|
| Mapa da mesa (nó Python, OpenCV) | `src/manipulation/manip_bringup/scripts/workstation_surface_node.py` |
| Parâmetros do mapa | `src/manipulation/manip_bringup/config/workstation_surface.yaml` |
| Regras do rulebook + candidatos (C++ puro, gtest) | `src/manipulation/manip_task_execution/{include/manip_task_execution,src}/surface_place_planner.{hpp,cpp}` |
| Integração no place (observação, IK, descida) | `src/manipulation/manip_task_execution/src/mtc_place_action_node.cpp` (`placeWithSurfacePlanner`) |
| `wall_sides` no goal / porta do BT / executor | `my_robot_msgs/action/PlaceTag.action`, `manip_bt/src/place_tag_bt.cpp`, `manip_bt/src/bt_yaml_executor.cpp`, `manip_bt/src/mission_map_config.cpp` |
| Paredes por estação (por mapa) | `src/caramelo_mapping/maps/<mapa>/ws_table_mapping.yaml` (bloco `ws_walls`) |
| Launch (nó + args) | `src/manipulation/manip_bringup/launch/manip_stack.launch.py` |
| Testes | `manip_task_execution/test/test_surface_place_planner.cpp`, `manip_bringup/test/test_workstation_surface.py` |

## 2. Como funciona

```
place (braço em pegar_obj com o cubo)
  1. publica a cota do TAMPO em /manip/table_plane_z  (FK da pose MesaX − altura do cubo)
  2. frente de referência: LiDAR (face da mesa no /scan, mediana do y) → senão docking (0,38 m
     do bico ⇒ y ≈ 0,163 em manip_base_link)
  3. /manip/surface_observe = true  → o nó de superfície ZERA a grade e começa a fundir vistas
  4. para cada pose de observação [pegar_obj, tag_direita, tag_esquerda]:
        para, espera 0,6 s, lê TF das tags/containers (1 s) + memória de slots,
        espera ≥ 2 grades novas, roda o planejador; "lazy": para na 1ª vista com plano completo
  5. /manip/surface_observe = false
  6. IK própria nos melhores candidatos (0°, 15°, 30°; alvo E elevação) → sem IK = fora de alcance
     (fila de alcance: devolve o cubo ao pote de bordo e sugere deslocamento da base)
  7. volta a pegar_obj se saiu de lá → elevação → descida vertical → chegada por FK (xy ≤ 2 cm,
     z ≤ 6 cm) → solta (run_place_cycle) → sobe → recolhe; grava o ponto na memória de slots
```

**Nó de superfície** (`workstation_surface_node.py`), por quadro, só com `surface_observe`:

1. HSV → **branco** (S ≤ 70, V ≥ 100) = tampo; o resto = **não branco**. Dedos vermelhos e cubo
   carregado apagados pelos polígonos do `container_detector.yaml` (fonte única) → não visto.
2. Manchas brancas pequenas (< 2 % da imagem: etiqueta, reflexo) e buracos brancos fechados por
   não branco (tag no cubo) viram não branco.
3. Cada célula de 1 cm do plano do tampo é projetada na imagem (K + distorção da `camera_info`,
   TF `manip_base_link ← camera_color_optical_frame`) e recebe o rótulo do pixel. Fora da imagem,
   raio rasante (< 18°) ou longe (> 1,2 m) = não visto. Quadros com a câmera em movimento são
   descartados.
4. Fusão por **votos** (branco × não branco) — empate = não branco (conservador).
5. **Bordas** = retas longas (RANSAC, ≥ 20 cm, cobertura ≥ 50 %) na fronteira branco/não branco,
   com não branco dominando o lado de fora **e sem tampo branco além delas** (até 25 cm). Só o
   que fica além de uma borda vira **fora da mesa**; todo o resto não branco é **objeto**. Por
   quê: por cor, um container no canto da imagem e o chão são iguais; dois containers com as
   faces alinhadas fizeram uma reta de 39 cm nas fotos de 25/08 — mas havia tampo além dela, e
   a regra do item 5 a rejeita. Cubo (4 cm) e container (17 cm) nunca viram borda.
6. Publica `nav_msgs/OccupancyGrid` em `/manip/workstation_surface/grid` (frame
   `manip_base_link`; −1 não visto, 0 livre, 70 fora, 100 objeto), status em
   `/manip/workstation_surface/status`, overlay em `.../debug_image` e a grade desenhada em
   `.../grid_image` (robô embaixo, bordas em amarelo).

**Planejador** (`surface_place_planner`): pinta tags/containers/memória como círculos de
obstáculo; transformada de distância exata (fora / objeto / não visto); ajusta a reta da frente
(rejeita se longe do prior do LiDAR/docking em mais de 6 cm → usa o prior); bordas laterais pela
mediana; filtra as células livres pelas regras da §0; pontua (folga a objetos ×1,0, à borda ×0,5,
teto 6 cm; menos distância ao ponto preferido = FK da `MesaX` ×0,6); devolve candidatos
espaçados de 2 cm. Sem a borda de um lado com parede declarada, a referência é a **extensão do
tampo livre visto** daquele lado (conservador).

Projeção do topo de um cubo (41 mm) no plano do tampo: a mancha sai um pouco **maior e mais
longe da câmera** que o pé do objeto — erra para o lado seguro da folga de 2 cm.

## 3. Paredes (10 cm)

Declare por estação no `ws_table_mapping.yaml` do mapa (lado visto pelo **robô docado de frente
para a mesa**):

```yaml
ws_walls:
  WS_3: [left]
  WS_6: [left, right]
```

O executor manda isso em `PlaceTag.goal.wall_sides` (também aceito por ação no YAML de baixo
nível: `wall_sides: [right]`). Opcional: `place_surface_wall_lidar:=true` detecta uma parede
pelo `/scan` (linha de pontos ao lado da mesa, até 30 cm além da borda) e usa o **x da parede**
em vez da borda — só **acrescenta** paredes, nunca remove uma declarada. Desligado por default
até ser validado na arena (uma mesa vizinha pode parecer parede: erra para o lado conservador).

## 4. Parâmetros que importam

Nó de place (`ros2 param set /place_action_server <nome> <valor>`; relidos a cada goal):

| Parâmetro | Default | Quando mexer |
|---|---|---|
| `surface_place_enabled` | true | kill-switch (launch: `place_surface_planner:=false`) |
| `surface_observe_poses` | `[pegar_obj, tag_direita, tag_esquerda]` | poses de observação; a 1ª `pegar_obj` não move |
| `surface_observe_lazy` | true | false = sempre olha de todas as poses (mais lento, mapa mais completo) |
| `surface_edge_clearance_m` / `surface_object_clearance_m` / `surface_wall_clearance_m` / `surface_zone_depth_m` | 0.02 / 0.02 / 0.10 / 0.20 | rulebook; `surface_safety_margin_m` (0.005) soma a todas |
| `surface_footprint_radius_m` | 0.029 | círculo circunscrito do cubo de 41 mm |
| `surface_front_fallback_y_m` | 0.163 | frente quando nem LiDAR nem câmera a veem (= dist. docking 0,38 − 0,217 do braço) |
| `surface_front_lidar_enabled` | true | frente pela face da mesa no `/scan` (half_width 0.25, janela ±0.12 do fallback) |
| `surface_pre_lift_m` | 0.05 | elevação antes da descida vertical |
| `surface_arrival_xy_tolerance_m` / `_z_` | 0.02 / 0.06 | chegada por FK (o braço estendido afunda ~2,5 cm em z) |
| `surface_tilt_ladder_deg` | `[15, 30]` | inclinação da ferramenta se 0° não tiver IK |
| `surface_max_ik_candidates` | 12 | candidatos testados na IK |
| `surface_fallback_legacy_slots` | true | false = falha o place em vez de cair nas poses fixas |

Nó de superfície (`ros2 param set /workstation_surface ...`, `config/workstation_surface.yaml`):
`white_s_max` / `white_v_min` (cor do tampo), `edge_min_length_m` (0.20), `edge_outside_max_white_frac`
(0.10), `min_ray_elevation_deg` (18), `resolution` (0.01), grade `grid_x/y_min/max`.

## 5. Calibrar / depurar sem robô

```bash
cd ~/caramelo_ws && source install/setup.bash
# Overlay (verde = tampo, vermelho = não branco, cinza = garra/não visto) + HSV do branco:
ros2 run manip_bringup workstation_surface_node.py --image foto.png \
    --config src/manipulation/manip_bringup/config/workstation_surface.yaml \
    --gripper-yaml src/manipulation/manip_bringup/config/container_detector.yaml --out overlay.png
# + grade com uma câmera sintética (x,y,z,pitch em manip_base_link; pegar_obj ≈ -0.01,0.16,0.31,84):
ros2 run manip_bringup workstation_surface_node.py --image foto.png ... --camera=-0.011,0.164,0.311,84.4 \
    --plane-z 0.083 --out overlay.png      # gera overlay_grid.png com as bordas em amarelo
# HSV de um pixel: --probe u,v
```

Ao vivo: `ros2 param set /workstation_surface always_on true`, `ros2 topic echo
/manip/workstation_surface/status`, `rqt_image_view /manip/workstation_surface/grid_image`; no RViz,
display *Map* em `/manip/workstation_surface/grid` (fixed frame `manip_base_link`).

Testes: `colcon test --packages-select manip_task_execution manip_bringup --ctest-args -R
"surface|workstation"` (15 gtests de regras/retas/paredes, 12 pytests de segmentação/projeção/fusão).

## 6. Sintomas → causa → parâmetro

| Sintoma (log `[SUPERFICIE]`) | Causa provável | Mexer em |
|---|---|---|
| `workstation_surface_node nao esta no ar` → poses fixas | nó não subiu / `use_surface_detector:=false` | launch; `ros2 node list` |
| `mesa nao vista pela camera` | `white_v_min` alto demais (tampo na sombra) ou baixo (chão/parede cinza entram) | `--image` offline, `white_v_min` 90–120 |
| `nenhum ponto livre que respeite as regras` em mesa com espaço | frente de referência errada (docking torto, LiDAR não vê a face) ou vista só de `pegar_obj` com o cubo tapando | ver `frente fallback`/`LiDAR: frente` no log; `surface_observe_lazy:=false` |
| `frente ... REJEITADA (longe do prior)` | faces de objetos alinhadas fingindo frente (esperado) ou prior errado | conferir `frente fallback y`; `surface_front_prior_tolerance_m` |
| Candidatos colados num container sem tag | container não ficou "objeto": cor clara demais | `white_s_max` ↓; conferir overlay |
| `nenhum com IK — fora de alcance` | ponto livre longe de lado (mesa de 80 cm) | esperado: fila de alcance desloca a base (`ReachQueue`) |
| `chegada fora da tolerancia` | braço afundando / IK no limite | `surface_arrival_z_tolerance_m` ↑; ver `tilt` usado |
| Parede declarada mas candidato perto dela | borda daquele lado não vista e extensão do tampo visto curta | esperado conservador; `place_surface_wall_lidar:=true` |

## 7. Validação no robô (pendente)

1. **Bancada mock** já feita (19/10, 2 rodadas: sem parede e com `wall_sides: right`):
   `manip_stack.launch.py use_mock_hardware:=true use_camera:=false use_apriltag:=false
   use_audio:=false use_container_detector:=false reset_container_states:=false` + a foto
   `docs/imagens/containers/realsense_pegar_obj.png` publicada como stream (640x480, fx=385) +
   `PlaceTag{tag_frame: tag_5, table_pose: Mesa15, ws: WS_1}`. Resultado: containers viraram
   obstáculo, 5 candidatos, alvo a 4,2 cm do objeto mais próximo e 17 cm da frente, IK a 0°,
   descida/subida vertical, memória gravada (`Place completed [slot=superficie ...]`). Com a
   parede declarada o robô olhou das 3 poses (borda direita nunca vista na foto fixa) e soltou
   respeitando `x <= borda_vista − 10 cm − ½ cubo`.
2. Robô docado numa mesa **vazia**: `ros2 param set /workstation_surface always_on true` e conferir
   no `grid_image` que a frente aparece em y ≈ 0,16 e o tampo é livre; `always_on false` depois.
3. Missão de 1 place (mesa vazia): o cubo deve cair ~4–5 cm atrás da frente, no centro.
4. Mesa com 2 cubos sem tag + 1 container: conferir no log `obstaculo extra`, `candidato ...
   folga obj ≥ 2,0 cm` e medir com régua a distância real.
5. Mesa com parede declarada (`ws_walls`): medir os 10 cm.
6. Se der problema, `place_surface_planner:=false` devolve o comportamento de 21/08.
