# Containers por cor — o que ajustar quando o container muda

Guia de manutenção do place em container **sem tag** (vermelho/azul, por câmera),
validado no robô em 25/08/2026 (8 rodadas na Mesa15). Diz **qual arquivo, qual linha e
qual parâmetro** mexer para cada mudança física. Linhas conferidas em 25/08 na branch
`detectando_container_e_usando_yaml_novo` — se o arquivo mudar, procure pelo nome do
parâmetro (`grep -n nome arquivo`).

## 0. Como funciona (30 segundos)

```
task YAML ──task_planner──► place{container_color: RED|BLUE}
   ──executor/PlaceTagBT──► PlaceTag.goal.container_color
place node (mtc_place_action_node.cpp)
   1. publica a cota da BORDA do container (/manip/container_plane_z)
      = FK(MesaX).z − cubo 0,042 + container_rim_height_m
   2. container_detector_node.py (só durante o place): imagem → HSV → mancha da cor
      → centro → ray-cast no plano da borda → TF manip_base_link→ct_vermelho|ct_azul
   3. place: espera o TF (≤3 s, busca lateral), pré-alinha, escolhe o ponto de
      soltura (deslocado no eixo longo se já houver bloco lá), desce em 3 movimentos,
      solta, fecha a garra, checa esforço, sobe vertical, recolhe
```

Arquivos:

| Papel | Arquivo |
|---|---|
| Detector (código) | `src/manipulation/manip_bringup/scripts/container_detector_node.py` |
| Detector (parâmetros) | `src/manipulation/manip_bringup/config/container_detector.yaml` |
| Place (código + defaults dos parâmetros) | `src/manipulation/manip_task_execution/src/mtc_place_action_node.cpp` |
| Launch (nós, args, obstáculos) | `src/manipulation/manip_bringup/launch/manip_stack.launch.py` |
| Planner (cor → container) | `src/manipulation/manip_bt/src/task_planner.cpp` |
| Executor (valida `container_color`) | `src/manipulation/manip_bt/src/bt_yaml_executor.cpp` |
| Action | `src/manipulation/my_robot_msgs/action/PlaceTag.action` (linha 17) |
| Fotos e overlays de calibração | `~/caramelo_ws/missions/fotos_containers/` |

Dois jeitos de mudar um parâmetro:

- **Ao vivo (teste)**: `ros2 param set /container_detector <nome> <valor>` ou
  `ros2 param set /place_action_server <nome> <valor>` — vale até o nó reiniciar.
- **Permanente**: detector → editar `container_detector.yaml`; place → editar o default
  no `.cpp` (linhas abaixo) e recompilar (`colcon build --packages-select
  manip_task_execution`), **ou** acrescentar a chave no dicionário `parameters` do nó
  `place_action` em `manip_stack.launch.py` (perto da linha 349,
  `"container_state_file": container_state_file,`) — sem recompilar.
- Depois de mudar: reiniciar o nó (o launch tem `respawn`: `kill <PID>` do nó basta).

## 1. Container com outras DIMENSÕES (comprimento × largura × altura)

Medidas atuais: 170 × 105 × 70 mm (medidas em 25/08). Cubo 42 mm.

| O quê | Onde | Valor atual | Como calcular |
|---|---|---|---|
| Lado longo | `container_detector.yaml` → `container_long_m` | 0.170 | medido |
| Lado curto | `container_detector.yaml` → `container_short_m` | 0.105 | medido |
| Altura (parede) no detector | `container_detector.yaml` → `container_height_m` | 0.070 | medido (só corrige o viés da parede) |
| Tolerância dos lados (mínimo) | `container_detector.yaml` → `dims_tolerance_frac` | 0.6 | blob inteiro precisa medir ≥ 40 % do nominal |
| Tolerância dos lados (máximo) | `container_detector.yaml` → `dims_tolerance_upper_frac` | 1.0 | blob inteiro pode medir até 2× — de perto as paredes inflam a mancha (27/08: 0,20 × 0,176 m era rejeitado). Blob cortado: só a área conta |
| **Altura da borda no place** | `mtc_place_action_node.cpp` linha 117 `container_rim_height_m` | 0.07 | = altura do container (borda acima do tampo) |
| Folga de soltura | linha 119 `container_drop_clearance_m` | 0.01 | fundo do cubo acima da borda; o braço afunda ~2,5 cm estendido, então 0,01 = cubo ~1,5 cm dentro |
| Deslocamento para vários blocos | linha 123 `container_slot_offsets_long_m` | [0.03, −0.03, 0.0] | ver §1.1 |
| Deslocamento no lado curto | linha 125 `container_slot_offsets_short_m` | [0, 0, 0] | só se o lado curto couber 2 cubos com folga |
| Folga do container como obstáculo | `clearanceForFrame` (prefixo `ct_`) | 0.13 | ≈ metade da diagonal + margem |
| Yaw confiável (mancha alongada) | `container_detector.yaml` → `yaw_min_aspect_full` / `yaw_min_aspect_partial` | 1.3 / 1.5 | razão lado longo/curto medida no plano; abaixo disso usa o último yaw confiável da cor (`yaw_memory_s` 60 s). Para um container quase quadrado, baixar |
| Centro estendido (mancha cortada) | `container_detector.yaml` → `partial_extend_enable` / `_max_shift_m` / `_min_visible_frac` | true / 0.05 / 0.35 | mancha cortada de UM lado (borda/garra) no eixo conhecido → centro empurrado para o lado cortado por (nominal − visível)/2 |

### 1.1 Como escolher os deslocamentos (`container_slot_offsets_long_m`)

Regra: `d = L/2 − cubo/2 − 0,02` no máximo, onde `L` é o lado longo interno e 0,02 é a
margem para os dedos abertos não pegarem a parede. Para L = 0,170: d ≤ 0,085 − 0,021 −
0,02 = 0,044 → usamos **0,03** (dois cubos a 6 cm de centro a centro = 1,8 cm entre eles).
Sequência `[+d, −d, 0]`: 1º bloco num lado, 2º no outro, 3º no meio (cai em cima/entre).
Para um container maior (ex. 220 mm) dá para usar `[0.06, -0.06, 0.0]` ou 4 posições
`[0.06, -0.06, 0.02, -0.02]`. O place escolhe sempre o candidato **mais longe dos blocos
já soltos** (memória de slots), então a ordem da lista só importa para o 1º bloco.

### 1.2 Mesa de outra altura

Nada a fazer: a cota da borda vem da FK da pose nomeada da mesa (`tabletopZForPose`) +
`container_rim_height_m`. Só confira no log: `[CONTAINER] plano da borda em
manip_base_link: z=… (tampo … + borda 0.070)`.

## 2. Container de outra COR (ou vermelho/azul de outro tom)

### 2.1 Mesmo par vermelho/azul, tom diferente (caso comum na arena)

Tudo em `container_detector.yaml`, bloco `vermelho:` / `azul:` (linhas ~95–110):

```yaml
    vermelho:
      h_ranges: [0, 10, 170, 179]   # pares [lo, hi, lo, hi] de matiz (OpenCV H 0–179)
      s_min: 90                     # saturação mínima (0–255)
      v_min: 50                     # brilho mínimo (0–255)
    azul:
      h_ranges: [85, 125]
      s_min: 80
      v_min: 30                     # interior sombreado do bin chega a V≈50
```

Como medir o tom (sem robô, com uma foto da RealSense, ou com o celular):

```bash
cd ~/caramelo_ws && source install/setup.bash
# HSV de um pixel do container (u,v em pixels):
ros2 run manip_bringup container_detector_node.py --image foto.png --probe 500,300
# Overlay: o que foi aceito/rejeitado e os percentis H/S/V dentro da mancha:
ros2 run manip_bringup container_detector_node.py --image foto.png \
    --config src/manipulation/manip_bringup/config/container_detector.yaml --out overlay.png
```

Regras práticas: H da RealSense difere do celular (azul do bin: RealSense 92–95, celular
99–105) — **calibre com a câmera do robô**. Deixe a faixa de H ~10 de folga para cada
lado; baixe `v_min` se o interior do bin (sombra) ficar fora da mancha; suba `s_min` se
a mesa/branco entrar. Salvar um frame da RealSense:

```bash
ros2 run image_view image_saver --ros-args -r image:=/camera/camera/color/image_raw \
    -p filename_format:=frame.png -p save_all_image:=false   # ou use rqt_image_view
```

Ao vivo: `ros2 param set /container_detector azul.h_ranges "[95, 130]"` e olhe
`ros2 topic echo /manip/container_detector/status` (mostra cor, x/y/yaw, área vs.
esperada, lados, motivos de rejeição e o estado da garra).

### 2.2 Uma cor NOVA (ex. verde) ou trocar os nomes

Precisa tocar em 5 lugares (todos simples):

1. **Detector** — `container_detector.yaml`: acrescente `verde` em `colors: [vermelho,
   azul, verde]` e um bloco `verde: {enabled: true, frame: ct_verde, h_ranges: [40, 85],
   s_min: 80, s_max: 255, v_min: 40, v_max: 255}`. (O default interno está em
   `container_detector_node.py` linha 54 `DEFAULT_COLORS`, mas o yaml manda.)
2. **Place: cor → frame** — `mtc_place_action_node.cpp`:
   - linhas 112–115: declare `container_frame_green` ("ct_verde") como os de red/blue;
   - linha 2232 `containerFrameForColor`: acrescente `if (color == "GREEN") return
     container_frame_green_;`
   - logo abaixo, `containerColorPt`: `GREEN → "verde"` (nome falado);
   - linha 2802 (`handle_goal`): a validação `!= "RED" && != "BLUE"` ganha `&& != "GREEN"`.
3. **Executor** — `bt_yaml_executor.cpp` linha 449: mesma validação (`RED|BLUE|GREEN`).
4. **Planner** — `task_planner.cpp` linha 99–100 `canonicalColor` (aliases `GREEN`,
   `VERDE`) e linha 148 `colorHasContainer` (`|| color == "GREEN"`).
5. **Launch** — `manip_stack.launch.py` linha 78: `slot_obstacle_tag_frames += ["ct_vermelho",
   "ct_azul", "ct_verde"]` (o container passa a repelir os slots dos places normais).

Recompilar: `colcon build --packages-select manip_task_execution manip_bt` (o detector é
Python, não precisa). Docs: `docs/guia_missoes.md` §6a-bis.

## 3. Garra ou câmera trocada / remontada (dedos vermelhos na imagem)

A garra é vermelha e aparece na imagem; o detector **apaga** regiões fixas da máscara
(câmera e garra são solidárias). Se a câmera for reposicionada ou a garra trocada,
refaça os polígonos — 5 minutos:

```bash
# 1) garra ABERTA sobre a mesa branca, SEM container vermelho na imagem → salve foto A
# 2) garra FECHADA num cubo AZUL, mesma condição → salve foto B
ros2 run manip_bringup container_detector_node.py --image A.png --fit-gripper
#    → cole as linhas "topo" em exclusion_polys_norm (container_detector.yaml ~linha 58)
ros2 run manip_bringup container_detector_node.py --image B.png --fit-gripper
#    → dedos fechados: cole em exclusion_polys_holding_norm (~linha 68)
ros2 run manip_bringup container_detector_node.py --image B.png --fit-gripper --fit-color azul --margin-px 20
#    → cubo: pegue o polígono do centro-alto, estenda o topo até y=0 e cole também em
#      exclusion_polys_holding_norm
# 3) confira: --out overlay.png (polígonos em amarelo; --gripper-open simula garra vazia)
```

Os polígonos "holding" só valem com a garra fechada (lê `/joint_states`,
`gripper_joint: manip_joint6`, `gripper_holding_above: -0.12`; aberta = −0,2265, fechada
= 0,170 no SRDF). Se trocar o modelo da garra, ajuste esses dois valores
(`container_detector.yaml` ~linha 73). Detalhe de implementação: os polígonos são
preenchidos **um por vez** (`fillPoly` com vários polígonos sobrepostos abre buraco).

## 4. Aproximação, descida e saída (place node)

| Parâmetro (linha em `mtc_place_action_node.cpp`) | Atual | Quando mexer |
|---|---|---|
| `container_tilt_ladder_deg` (136) | [15, 30] | container mais fundo/alto → ferramenta mais inclinada; sempre tenta 0° antes |
| `stack_pre_lift_m` (97) | 0.05 | altura da pose de elevação (descida vertical parte daqui); ↑ se a garra roçar a borda ao entrar |
| `container_arrival_tolerance_m` (132) | 0.03 | erro XY aceito na chegada (decide se cai dentro) |
| `container_arrival_z_tolerance_m` (134) | 0.06 | erro Z aceito — o braço estendido afunda 1,4–3,2 cm (medido); **não baixe abaixo de 0,04** |
| `container_close_gripper_on_exit` (121) | true | false = sair só com a subida vertical, garra aberta |
| `container_exit_gripper_position` (127) | 0.170 (fechada) | 0.0 = fecha parcialmente (não re-pega cubo; mais estreita que aberta) |
| `container_detect_timeout_sec` (130) | 3.0 | espera pela primeira detecção |
| `container_search_lateral` / `_timeout_sec` (145/147) | true / 1.5 | não viu de `pegar_obj` → olha de `tag_direita`/`tag_esquerda` |
| `container_fallback_to_table` (138) | true | false = falha o place se não viu o container. **28/08 (fila de alcance)**: com `goal.final_attempt=false` (dentro de um `ReachQueue`), "sem IK em nenhum candidato" NÃO cai mais na mesa: o cubo volta ao container de bordo e o place devolve `unreachable=true` + `suggested_base_shift_m` para o executor deslocar a base e repetir; com `final_attempt=true` (passada final ou XML antigo) vale o comportamento desta linha |
| `unreachable_bailout_enabled` / `unreachable_shift_candidates_m` / `unreachable_shift_margin_m` | true / [0.10,0.15,0.20,0.25] / 0.02 | busca de deslocamento da base verificada pela IK (`reach_shift`); ver `docs/guia_missoes.md` §6d |

Sequência de estágios (para ler o log): `container_detecting` → `container_pre_approach`
→ `container_approach` (3 movimentos: j1/j4/j5 → j2+j3 até a elevação → descida
vertical) → `opening_gripper_final` → `container_closing_gripper` →
`container_verifying_empty` (re-pegou? reabre e fala "Peguei o bloco de volta") →
`container_lift_after_release` → `returning_pegar_obj_final`. Fora de alcance (28/08):
`unreachable_check` → `returning_cube_pegar_obj` → `returning_cube_pre_container` →
`returning_cube_container` → `returning_cube_opening_gripper` → `returning_cube_pre_container_out`
→ `returning_cube_pegar_obj_final` → `skipped_unreachable`.

## 5. Teste depois de qualquer mudança (roteiro de 10 min)

1. Detector offline nas fotos: `--out overlay.png` → um blob por cor, `ACEITO`, sem
   dedos marcados.
2. Detector ao vivo com a garra parada olhando os containers:
   `ros2 param set /container_detector always_on true` e
   `ros2 topic echo /manip/container_detector/status` → `x/y` plausíveis (y ≈ 0,3–0,45 m à
   frente), `lados` perto de 0,17/0,105. Depois `always_on false`.
3. Missão de manipulação sem navegação (robô parado na mesa), duas tags na mesa:

```yaml
actions:
  - {kind: pick,  tag_frame: tag_5, table_pose: Mesa15, ws: WS_1}
  - {kind: pick,  tag_frame: tag_6, table_pose: Mesa15, ws: WS_1}
  - {kind: place, tag_frame: tag_5, table_pose: Mesa15, ws: WS_1, container_color: RED}
  - {kind: place, tag_frame: tag_6, table_pose: Mesa15, ws: WS_1, container_color: BLUE}
```

```bash
ros2 run manip_bt bt_yaml_executor missao.yaml --ros-args \
  -p map_folder:=$HOME/caramelo_ws/src/caramelo_mapping/maps/arenalegal_2026 \
  -p simulate_navigation:=true -p "finish_dock_id:=''"
```

   No log do place procure: `[CONTAINER] manip_base_link <- ct_azul: x=… y=…`,
   `alvo [...]`, `em posicao … (erro FK … dx dy dz)` — xy ≤ 0,5 cm é o normal;
   `[CONTAINER-EXIT] place grasp effort … avg=0.01` = não re-pegou.
4. Para testar 2 blocos no mesmo container, use a mesma cor nos dois places e confira no
   log `N bloco(s) ja soltos … candidato k/3 … D cm do bloco mais proximo` (D ≈ 6 cm).
   A memória de slots fica em `~/.config/caramelo/container_states.yaml` (`table_slots`),
   é zerada no launch e tem TTL de 900 s; para zerar à mão apague a chave `table_slots`.

**Nunca cancele (Ctrl-C) o executor com um pick em andamento** para trocar parâmetro: o
cubo fica na garra e a próxima missão abre a garra em `home`. Espere terminar.

## 6. Sintomas → causa → parâmetro

| Sintoma | Causa provável | Mexer em |
|---|---|---|
| "Nao vi o container" e solta na mesa | cor fora da faixa / container fora da vista / detector desligado | §2.1; `use_container_detector:=true`; status topic |
| Marca a garra como container | polígonos da garra desatualizados | §3 |
| Centro puxado para um lado | cubo da mesma cor encostado no container gruda na mancha | afaste o cubo; (pendente: abertura morfológica maior) |
| Cai perto da parede | `wall_bias_max_m`/`dims_tolerance_frac`; ou blob parcial (container cortado na imagem) | reposicione o robô mais perto; `container_search_lateral` |
| Viu de `pegar_obj`, pré-alinhou e "Nao vi" (log.txt: `lado_curto(0.17x m)`/`lado_longo`) | de perto a mancha projetada fica maior que o pote | `dims_tolerance_upper_frac` (1.0); conferir os `_overlay.png` de `fotos_containers/debug/` |
| Solta encostado na parede depois de pré-alinhar (log.txt: `lados=0.18/0.18`, `yaw` muito diferente do visto de longe, `yaw=…(?)`) | mancha cortada/quase quadrada: yaw ambíguo → deslocamento de 3 cm no eixo errado; centroide puxado para o lado visível | `yaw_min_aspect_*` + memória de yaw (`(mem)` no status) e `partial_extend_*` (`desloc=… cm` no status) |
| Quero ver o que o detector viu num place | gravação ligada? | `save_debug_dir` (yaml) → `<ts>_raw.png`, `<ts>_overlay.png`, `log.txt`; reprocessar: `--image <ts>_raw.png --config ... --out x.png` |
| Container longe para o lado: log `candidato k/3 … sem IK` em todos e depois `unreachable_check` / "está fora do meu alcance, vou precisar me mover" | braço não alcança da pose de estacionamento | esperado: o executor (`ReachQueue`) desloca a base (`nudge_base`, ≤ 25 cm, até 2×) e repete; se ainda assim não alcançar, passada final solta na mesa. Ver `docs/guia_missoes.md` §6d |
| Bate na parede ao descer | pose de elevação baixa / ferramenta pouco inclinada | `stack_pre_lift_m` ↑, `container_tilt_ladder_deg` |
| "chegada … cm em z" e fallback | afundamento > tolerância | `container_arrival_z_tolerance_m` ↑ (0,06 hoje) |
| 2º bloco cai em cima do 1º e a garra re-pega | deslocamentos desligados/iguais | `container_slot_offsets_long_m` (§1.1) |
| Re-pega mesmo com deslocamento | bloco alto | `container_exit_gripper_position: 0.0` ou `container_close_gripper_on_exit: false` |
| Detector some após `ros2 param set` | valor inválido rejeitado (lista de tamanhos diferentes, polígono malformado) | ver `[container_detector] parametro rejeitado:` no log |
