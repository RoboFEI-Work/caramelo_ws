# Relatório dos dois testes ATT1 de 2026-11-11 (bags 18h58 e 19h16)

Escrito por Claude na noite de 11/11 a partir dos dois rosbags gravados pelo `run_mission`
(ver `docs/guia_missoes.md` §2.3). Nada aqui foi observado ao vivo: tudo vem dos tópicos
gravados (`/cmd_vel_nav`, `/mecanum_controller/reference`, `/maxon/wheel_velocity`,
`/joint_states`, `/odom`, `/amcl_pose`, `/plan`, `/scan`, `/local_costmap/costmap`,
`/rosout`, status das actions). Scripts usados estão em
`src/caramelo_navigation/scripts/bag_forense/` (uso ad hoc, não instalados).

| Bag | Pasta em `~/bags/` | Duração | Resultado da missão |
|---|---|---|---|
| 1 | `20261111_185756_ATT1_seed797916_equipe_arena_extendida` | 669 s | Terminou; sem obstáculo novo |
| 2 | `20261111_191614_ATT1_seed797916_equipe_arena_extendida` | 984 s | 2 goals abortados (WS5→WS8 e WS5→WS7), robô empurrado à mão; obstáculo novo no meio do mapa |

Os dois rodaram com HEAD 833c13d, perfil `dwb`, lattice `omni` (o `corrida.yaml` diz
`nav_profile: desconhecido` porque o launch do `mission_server` ainda não repassa o perfil
ao gravador; cosmético, anotado em pendências).

## 0. Resumo em cinco linhas

1. **O robô passou mais tempo girando parado do que andando**: 55 % do tempo de navegação
   no bag 1 e 74 % no bag 2 (98 s e 349 s). Quem gira é o RotationShim a 0,35 rad/s; em
   pernas curtas (< 0,5 m, como SH1→WS2) ele entra num vaivém de ±170° (76 s para andar
   0,22 m). É o "fica girando para a esquerda e para a direita" que o operador viu — não é
   o AMCL se relocalizando.
2. **Os dois goals abortados foram o mesmo beco**: o obstáculo novo fechou a rota direta de
   WS5 para o lado oeste; o planner mandou contornar a parede central pelo leste (5,8 m em
   vez de 3,4 m), o que exige meia-volta no corredor entre a mesa WS5 e a parede. Ali o
   costmap local marcava custo 253 nas 8 direções em volta do robô; o DWB só achava giro
   legal, ficou oscilando ±30° por 60–80 s, o `PoseProgressChecker` abortou 14 vezes, e a
   cada limpeza do costmap global o plano voltava para a rota bloqueada e o shim girava
   180° de novo. Quando finalmente saiu, cortou a quina leste da parede (4,8, 0,75) — a
   colisão relatada.
3. **Há "trancos" físicos, confirmados por encoder**: 7 eventos no bag 1 e 17 no bag 2 em
   que as rodas giraram a 8–16 rad/s (0,4–0,8 m/s de roda) com a referência pedindo no
   máximo 2–5 rad/s. O pior: 0,75 s a 16 rad/s no par esquerdo logo após o undock da WS5
   (bag 1, 168 s), com a referência em zero — o robô andou sozinho ~0,3 m para trás e caiu
   em "Start occupied". O tranco da WS1 relatado é o de 281 s (giro a 2,1 rad/s medido com
   comando de 0,35). Isso está abaixo de `/mecanum_controller/reference` (Pi/Maxon/ESC):
   fora do que podemos mexer antes de 18/11, mas a equipe de hardware precisa saber.
4. **O laço do DWB rodou a 7 Hz** (mediana; mínimo 3,6 Hz; 500 e 1728 avisos "missed its
   desired rate") em vez de 20 Hz — igual ao que o plano de 11/11 já media.
5. **Correção aplicada para o próximo teste (rodada 1)**: `rotate_to_heading_once: true`
   no shim. **Preparada e desligada (rodada 2)**: `bt_replan:=invalid`. Detalhes em §3.

## 1. Tempo de navegação por perna

`girando` = comandos puros de rotação (vx = vy = 0, |wz| ≥ 0,3) em `/cmd_vel_nav`;
`andando` = comandos com translação. `v útil` = reta início→fim dividida pelo tempo andando.

**Bag 1** (nav = 178 s de 669 s; girando 98 s = 55 %)

| Perna (s) | Reta | Dur. | Girando | Andando | v útil | Obs. |
|---|---|---|---|---|---|---|
| 19–28 START→WS1 | 1,04 m | 8,6 s | 4,8 s (55 %) | 3,2 s | 0,33 | |
| 77–97 WS1→WS5 | 4,34 m | 20,5 s | 10,0 s (49 %) | 9,4 s | 0,46 | |
| 169–185 WS5→WS8 | 2,84 m | 16,0 s | 5,4 s (34 %) | 8,6 s | 0,33 | "Start occupied" + escape 0,20 m no início (tranco de 168 s) |
| 256–282 WS8→WS1 | 6,67 m | 25,8 s | 10,5 s (41 %) | 13,9 s | 0,48 | |
| 324–361 WS1→WS2 | 1,60 m | 37,6 s | 27,3 s (73 %) | 8,9 s | 0,18 | "Start occupied" na staging; volta completa de 350° |
| 397–407 SH1→WS2 | 0,14 m | 9,9 s | 8,0 s (81 %) | 1,2 s | 0,12 | |
| 478–502 WS2→WS5 | 4,07 m | 24,2 s | 11,4 s (47 %) | 10,8 s | 0,38 | |
| 544–559 WS5→WS7 | 1,58 m | 14,9 s | 9,4 s (63 %) | 4,8 s | 0,33 | |
| 634–655 WS7→FINISH | 2,35 m | 21,0 s | 11,1 s (53 %) | 8,0 s | 0,29 | "Start occupied" perto do FINISH |

**Bag 2** (nav = 470 s de 984 s; girando 349 s = 74 %)

| Perna (s) | Reta | Dur. | Girando | Andando | v útil | Obs. |
|---|---|---|---|---|---|---|
| 16–26 | 1,27 m | 9,0 s | 5,0 s | 3,8 s | 0,34 | |
| 62–93 WS1→WS5 | 4,26 m | 31,2 s | 18,1 s (58 %) | 12,5 s | 0,34 | |
| **161–286 WS5→WS8** | 2,98 m | **124,7 s** | 101,1 s (81 %) | 17,7 s | 0,17 | **abortado** (beco, colisão na quina) |
| 287–306 →WS1 | 3,10 m | 18,8 s | 10,0 s | 8,0 s | 0,39 | |
| 379–397 WS1→WS2 | 1,58 m | 18,2 s | 13,2 s (72 %) | 4,9 s | 0,32 | |
| **435–512 SH1→WS2** | 0,22 m | **76,5 s** | 63,6 s (83 %) | 4,4 s | 0,05 | vaivém de ±170° |
| 584–612 WS2→WS5 | 4,06 m | 28,4 s | 15,1 s | 12,3 s | 0,33 | |
| **650–750 WS5→WS7** | 1,66 m | **99,8 s** | 84,0 s (84 %) | 11,3 s | 0,15 | **abortado** (mesmo beco) |
| 767–786 →WS8 | 3,17 m | 18,9 s | 9,7 s | 8,9 s | 0,36 | re-tentativa, deu certo |
| 851–873 WS8→WS7 | 1,61 m | 21,9 s | 16,1 s (74 %) | 5,2 s | 0,31 | |
| 958–981 WS7→FINISH | 2,34 m | 22,8 s | 12,7 s | 8,3 s | 0,28 | "Start occupied" perto do FINISH |

Mesmo nas pernas boas, metade do tempo é giro parado. A velocidade útil fica em
0,3–0,5 m/s porque a translação em si anda a 0,5–0,6 m/s.

## 2. Achados, com a evidência

### A1. Giro parado: quem gira é o shim, e as pernas curtas viram vaivém

- Nas pernas normais, os comandos de giro vêm a 20 Hz (intervalo < 80 ms) — assinatura do
  RotationShim, que não chama o DWB enquanto gira. O DWB, quando roda, publica a ~7 Hz.
- Cada perna paga o giro inicial para o rumo do path (até 180° a 0,35 rad/s = 9 s) e o giro
  final para o yaw do goal (`rotate_to_goal_heading`), e **paga de novo a cada replan**
  (1 Hz) em que o rumo medido no ponto a 0,5 m do path difere mais de 45° do yaw atual —
  no bag 2 isso aconteceu em 17 de 112 planos da perna 161–286 e em 42 de 73 na perna
  435–512.
- **Perna SH1→WS2 (bag 2, 435–512 s, 0,22 m)**: o path tem 0,03–0,29 m. O shim mira o
  último ponto do path (fica atrás do robô, rumo −126° a −178°) e gira até lá; quando o
  AMCL (ruído ~16 cm) diz que o robô entrou na janela de 0,10 m, o ramo `rotate_to_goal_heading`
  gira para o yaw do goal (0°); o AMCL sai da janela, o ramo do rumo volta a mandar olhar
  para trás. Resultado: 61,6 s de giro a 20 Hz, 10 trocas de sentido, 170° cada. O mesmo
  par SH1→WS2 no bag 1 (397–407) levou 10 s por sorte.
- **Staging da WS2 (bag 1, 340–361 s)**: depois do escape, o path encurtou para 0,04–0,10 m;
  o rumo ao último ponto ficou a ±180° e o robô deu **uma volta completa de 350°** (yaw
  176 → −160 → −91 → −1 → 54 → 100 → 142 → 163) até o checker fechar.
- Figura: `docs/img/testes_2026-11-11/b1_324_361.png` (setas em todas as direções no ponto
  (7,4; 2,2)).

### A2. Os dois goals abortados: beco no corredor da WS5 com a parede central

Cronologia da perna 161–286 (a de 650–750 é idêntica, só muda o goal):

1. 161 s: undock da WS5 (robô a (3,7; 1,55), yaw 86°). Plano de 5,8 m: **leste**, contorna
   a quina leste da parede central (~(4,9; 0,9)) e volta por baixo até WS8. No bag 1 a mesma
   perna tinha 3,4 m indo direto para oeste. O obstáculo novo está, portanto, na rota oeste
   (figura `b2_161_286.png`: planos claros = oeste, planos escuros = leste).
2. 161–165: shim gira −93° (86° → −7°) sem problema.
3. 165–189: o path segue estável para leste (rumo 0–15°), mas o robô só oscila (yaw −7, 23,
   1, 27, 19, 16, 29, −5, 49, 21, 34 …) e anda 7 cm. Os comandos são **do DWB** (intervalo
   ~140 ms): 471× (0, 0, −0,35), 435× (0, 0, +0,35), 60 trocas de sinal nos 125 s. Na
   avaliação do DWB só a rotação mínima legal estava sobrando: em 189,7 s o
   `escape_free_space` mediu "custo máx 253 nas 8 direções, letal a 10 cm à direita, 8 cm
   diag-ré-direita, 18 cm na ré" — isto é, toda translação levava o centro do robô para
   célula ≥ 253 (os críticos `PathDist`/`GoalDist` rejeitam, `stop_on_failure`), só girar
   era legal, e o `Oscillation` solta o sentido a cada 0,2 rad → vaivém. O `PoseProgressChecker`
   (0,10 m ou 0,5 rad em 6 s) não vê progresso líquido e aborta: **14 "Failed to make
   progress"** no bag 2.
4. 189,7 e 670,4 s: recovery `ClearGlobalAndWait` apaga o costmap global → o obstáculo novo
   some → **o plano volta para a rota oeste (3,0–3,2 m)** → o shim gira 180° → o LiDAR
   enxerga o obstáculo de novo → plano volta para leste (5,6 m) → o shim gira 180° de volta
   (plan_rumo: 189,9–202,7 s e 671,6–684,9 s). Cada ciclo custa ~15 s e duas meias-voltas
   num corredor em que o nariz (0,395 m) varre um círculo de 0,88 m de diâmetro.
5. 264–285 s: o robô finalmente engrena para leste a 0,5 m/s e **corta a quina**: para em
   (4,81; 0,75) com yaw −44°, em cima do fim da parede central do mapa. O planner responde
   "Start occupied" (costmap global = mapa + inflação) e o `bt_navigator` aborta o goal.
   O DWB não sabia da parede: o costmap local não tem `static_layer` e o LiDAR, frontal, só
   tinha marcado a face norte da parede (figura `b2_cm_189.png`, direita: a quina a leste
   do robô aparece como mancha de custo médio, não letal).
6. Em seguida o `bt_yaml_executor` pulou a estação e fez a re-tentativa, que deu certo
   (767–786 s: outra rota, o obstáculo já estava mapeado no costmap global).

Dois pontos para a geometria: a meia-volta parada precisa de ≥ 0,88 m livres (raio
circunscrito 0,44 m com o nariz de 0,395 m); e o costmap local, pelo relato do escape,
não tinha esse espaço em (3,5–3,9; 1,3). Nenhum parâmetro faz o robô sair desse beco de
ré ou de lado "olhando para trás": `min_vel_x` é 0 (sem ré, decisão do time) e o shim exige
o nariz no rumo.

### A3. Trancos: rodas a 8–16 rad/s sem comando

Critério: alguma roda com |ω| > 8 rad/s e > 2,5× o teto implicado pela referência
`(|vx| + |vy| + 0,3·|wz|)/0,05`. `/joint_states` (encoder) mostra o mesmo que
`/maxon/wheel_velocity`, e `/odom` confirma o movimento do corpo.

| Bag | t (s) | Dur. | Roda máx | Referência no instante | Corpo (odom) | Contexto |
|---|---|---|---|---|---|---|
| 1 | 110,3 | 0,05 s | 8,6 | (0; 0,13; 0) → (0; 0; 0) | wz 0,41 | align WS5 |
| 1 | **168,2** | **0,75 s** | **16,3** | (−0,13; 0; −0,11) → **(0; 0; 0)**, gap de 305 ms na referência | **v 0,47 m/s, wz 1,0** | fim do undock WS5; par esquerdo a −16; robô recua ~0,3 m sozinho; 1,5 s depois "No valid trajectories" + "Start occupied" |
| 1 | 173,5 | 0,14 s | 16,1 | (0,09; 0,09; 0,35) | v 0,50 | DWB, saída da WS5 |
| 1 | **280,9** | **0,80 s** | **13,4** | **(0; 0; −0,35) constante** | **wz 2,08** | chegada à staging da WS1 (o tranco relatado): rodas pulam de ±3 para ±11,5 com o comando parado |
| 1 | 498,4 | — | 8,9 | (0; 0; −0,35) | wz 0,52 | |
| 1 | 645,5 | 0,15 s | 15,9 | (0,05; −0,09; −0,35) | v 0,46 | |
| 1 | 649,0 | 1,55 s | 12,5 | (0,20; 0; 0,03) | v 0,42, wz 0,67 | antes do "Start occupied" do FINISH |
| 2 | 17 eventos | até 0,9 s | 8–16 | idem: (0;0;±0,35) ou vx/vy pequenos + wz | v até 0,51 | 190,6 s com referência **zero** (gap 222 ms) e o robô se mexendo a 0,33 m/s durante o "não vou me mover" do escape |

Padrões: (i) ocorre com referência **zero ou constante**, então não é o Nav2 nem o
smoother; (ii) os dois eventos com a referência em zero coincidem com um buraco > 150 ms
entre mensagens de referência (`reference_timeout` do controlador da Pi é 0,15 s); (iii)
os demais acontecem em comandos mistos pequenos (vx ou vy ≈ 0,05–0,15 com wz 0,35), em que
rodas individuais ficam perto do piso de 2,43 rad/s. Hipóteses para a equipe de hardware:
integrador do PI do ESC acumulando enquanto a roda está abaixo do piso e descarregando;
ou tratamento de "comando zero"/timeout no `MaxonMotorsNode`. Nada disso pode ser alterado
antes da competição (decisão do time), mas explica colisões e "Start occupied" que não
têm causa no software do PC. Um recuo de 0,3 m sem comando numa staging a 0,2 m da mesa é
risco real.

### A4. Laço do controller a 7 Hz

`Control loop missed its desired rate of 20 Hz`: bag 1 n = 500, mediana 6,9 Hz, p10 5,4,
mín. 4,2; bag 2 n = 1728, mediana 7,0, p10 5,5, mín. 3,6. `/cmd_vel_nav` com intervalo
mediano de 50 ms (giro do shim) e p95 de 170 ms (ciclos do DWB). Conforme o plano de 11/11
(Fase 0/1): governador de energia e amostras do DWB. A 0,5 m/s, 7 Hz são 7 cm por ciclo —
relevante para a quina da parede.

### A5. Avisos secundários

- `dock_align_node`: "refino: resíduo alto (face não-plana?)" na WS5 (2×) e "face larga
  demais (vizinho?)" na WS1 e WS2 → "refino por LiDAR sem confiança; usando pose do banco".
  Sem efeito visível nos bags; registrar.
- `/scan` chega ao PC com `header.stamp` 108 ms atrás do relógio local (o driver carimba o
  início da varredura de 100 ms). Dentro das tolerâncias (0,6 s). `/odom`: 10 ms.
- `collision_monitor`: 6 e 10 ativações curtas (0,1–0,8 s) de `approach`; nenhuma parada.
- `corrida.yaml` com `nav_profile: desconhecido`: o `mission_server.launch.py` não repassa
  `bag_nav_profile`. Cosmético.

## 3. O que foi mudado e o que está preparado

Regra do operador: uma mudança por rodada; "falhar em melhorar nunca pode piorar". Nada foi
commitado.

### Rodada 1 — APLICADA: `rotate_to_heading_once: true` (shim do DWBHolonomic)

Arquivo: `src/caramelo_navigation/config/controller_server.yaml` (bloco `DWBHolonomic`,
comentário datado). Efeito esperado: o shim gira **uma vez** no início da perna e no yaw
final; não gira mais a cada replan nem atrás do último ponto de um path de 15 cm. Ataca
A1 inteiro (vaivém da SH1→WS2, volta de 350°, re-engajamentos) e metade de A2 (as meias-voltas
a cada limpeza do costmap global). O que observar: pernas curtas (SH1→WS2, stagings após
escape) devem fechar em < 15 s; nas curvas fechadas do meio do path o robô vai arquear /
andar de lado em vez de parar e girar — se isso virar caranguejo feio, é o preço, e a rodada
seguinte (lattice `nariz`) é a resposta. **Reverter**: `rotate_to_heading_once: false` e
relançar. Precisa de `source install/setup.bash` (feito o build; só YAML).

### Rodada 2 — PREPARADA, desligada: `bt_replan:=invalid`

Novo `behavior_tree/caramelo_lattice_dwb_replan_if_invalid.xml` + argumento `bt_replan`
(`rate` = default = BT atual; `invalid` = replaneja só se o goal mudou, o path suavizado
cruzou célula letal — serviço `is_path_valid` do planner, modo footprint — ou passou 15 s),
repassado por `bringup.launch.py` e `robot_manipulation.launch.py`. Padrão
`nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml` do Nav2 Jazzy.
Ataca o flip-flop de rota depois de `ClearEntireCostmap` e os "Start occupied" a cada
segundo com o robô parado na staging (7 nos dois bags). Ligar: acrescentar
`bt_replan:=invalid` ao launch. Build feito; `--show-args` OK nos três launches; a seleção
do XML foi testada em Python (dwb+invalid → arquivo novo; mppi+invalid → erro claro).

### Rodadas seguintes (em ordem sugerida, uma por teste)

3. **Laço do DWB** (plano §3 Fase 0/1): `powerprofilesctl set performance` antes do teste
   (não é mudança de código) e `vx_samples 10→8`, `vy_samples 8→6`, `vtheta_samples 21→15`.
4. **Caranguejo/corredores**: `lattice_set:=nariz` (já pronto) **ou** voltar
   `ObstacleFootprint.scale` 1,0 → 0,02 (valor até 07/11; a 1,0 o crítico vale até 5 pt,
   da ordem do ganho de avançar 0,5 m, e numa base omni isso empurra de lado). Escolher um.
5. **Pernas curtas pelo mission_server**: quando a staging destino está a < 0,3 m da pose
   atual (SH1→WS2 = 0,245 m), pular o `NavigateToPose` e ir direto ao `align_to_dock`
   (código em `go_to_ws_bt.cpp`/`mission_server.py`; proposta, não feita). Alternativa de
   YAML: `general_goal_checker.xy_goal_tolerance` 0,10 → 0,20 só se o docking tolerar.
6. **Beco** (A2 item 3): sem solução por parâmetro com `min_vel_x` 0 e shim. Operacional:
   se o árbitro puser obstáculo no corredor da WS5, o planejador de tarefas deveria ordenar
   as estações para não sair da WS5 pelo leste; ou aceitar o custo e confiar na
   re-tentativa (que funcionou). Experimento só depois da competição: `min_vel_x −0,25`
   (ré às cegas) com `rotate_to_heading_once`.
7. **Hardware (A3)**: ticket para a equipe da Pi/ESC com a tabela acima; os bags têm
   `/joint_states` a 100 Hz e `/mecanum_controller/reference` para reproduzir.

## 4. Próximo teste: comandos

Igual ao de hoje (o bag já liga sozinho); só o perfil de energia antes:

```bash
powerprofilesctl set performance
ros2 launch caramelo_bringup robot_manipulation.launch.py \
    map_name:=arena_extendida use_rviz:=true camera_profile:=640x480x15 \
    verify_grasp_effort:=true max_pose_error_m:=0.0 use_vision_viewer:=true \
    nav_profile:=dwb lattice_set:=omni bt_replan:=rate
```

(`bt_replan:=rate` é o default; está aí só para lembrar que a rodada 2 é `invalid`.)
Depois do teste: `ros2 run caramelo_navigation bag_resumo ~/bags/<pasta>` e me mandar a
pasta — a comparação a fazer é a coluna "girando" da §1 por perna.

## 5. Figuras

- `docs/img/testes_2026-11-11/b2_161_286.png` — bag 2, goal WS5→WS8: mapa, trilha do AMCL
  (cor = tempo), planos (claro = cedo, escuro = tarde), scan (vermelho), footprint final
  (magenta) sobre a quina da parede; à direita o costmap local no instante do "Start occupied".
- `docs/img/testes_2026-11-11/b2_650_751.png` — bag 2, goal WS5→WS7: o mesmo beco.
- `docs/img/testes_2026-11-11/b2_cm_189.png` — costmap local no momento em que o escape
  mediu 253 em volta (robô em (3,50; 1,28), yaw 30°; verde = scan).
- `docs/img/testes_2026-11-11/b1_324_361.png` — bag 1, staging da WS2: a volta de 350°.

---

# Rodada 2 — bag 3 (20h31, `rotate_to_heading_once` ligado) e o que mudou

## 6. O que o bag 3 mostrou (`~/bags/20261111_203107_*`, 842 s, missão completa com 1 re-tentativa)

Comparado com os bags 1 e 2: navegação caiu de 178/470 s para **278 s em 11 pernas**
(incluindo as duas que ontem abortaram), giro parado de 55–74 % para **44 %**, laço do DWB
de 7 Hz para **16 Hz** (p10 12 Hz; `powerprofilesctl performance` fez efeito), trancos de
7/17 para 4. Nas pernas longas o giro ficou em 28–40 % (giro inicial + final, legítimos a
0,35 rad/s). Nenhum goal abortado. Ainda assim três pernas gastaram 130 s dos 278:

| Perna | Dur. | O que aconteceu | Causa (medida) |
|---|---|---|---|
| 142–198 WS5→WS8 (2,85 m) | 56 s | O obstáculo continuava na rota oeste: plano de 6 m pelo leste. Shim girou 93° certo (uma vez). Depois o **DWB** ficou 28 s alternando (0,0,±0,35) e passos laterais de 6 cm no corredor WS5/parede (53 % dos comandos abaixo do piso), 5 "Failed to make progress", escape diag-ré 0,21 m. Aos 170 s o costmap liberou a rota oeste e o robô foi (2,6 m). | Corredor: perímetro do footprint em custo 220–253 em qualquer pose (`escape` mediu frente 220, lados 245–253); `ObstacleFootprint.scale 1.0` = até 5 pt, da ordem do ganho de avançar; DWB prefere girar/dar passinhos. Figura `b3_142_198.png`. |
| 261–304 WS8→WS1 (6,6 m) | 42 s | Caranguejo: 106× (0,−0,29,+0,5) e 84× (0,−0,17,+0,35) = andar de lado para a direita girando para a esquerda; encostou na WS6 (scan 0,06 m), "No valid trajectories" ×11, "Start occupied" ×2 em (2,0; −0,2), escape e backup falharam ("Collision Ahead"). Depois andou 5 m limpo a 0,6 m/s. | Mesmo crítico: andar de lado para o custo menor sai mais barato que seguir o path. O shim não participou (6 s de giro na perna toda). |
| 425–457 SH1→WS2 (0,07 m) | 32 s | Robô já estava a 7 cm da staging; precisava só girar 105°. Shim engajou em 22 dos 27 planos (`isGoalChanged` devolve true enquanto `in_rotation_`), perseguindo o último ponto de um path de 3–29 cm cujo rumo muda com o ruído do AMCL; 640° de giro acumulado. | `rotate_to_heading_once` não cobre este caso: enquanto gira, cada path novo é "novo". O ramo do yaw do goal só assume dentro de 0,10 m e o AMCL oscila ±16 cm. |

Trancos (4): 170,2 s e 176,8 s (saída da WS5), 266,3 s (WS6), 550,6 s (referência zero,
robô a 0,28 m/s por 0,2 s). Mesma assinatura de ontem.

## 7. Rodada 2 — aplicada (um pacote; cada item reversível sozinho)

| # | Mudança | Arquivo | Ataca | Reverter |
|---|---|---|---|---|
| 2a | `ObstacleFootprint.scale` **1.0 → 0.02** (valor até 07/11) | `config/controller_server.yaml` | corredor (P1) e caranguejo/WS6 (P2) | `1.0` |
| 2b | **Perna curta**: se a staging destino está a < 0,30 m da pose do AMCL, o `GoToWS` manda a BT `caramelo_lattice_dwb_curta.xml` (goal checker `staging_goal_checker`, xy 0,30 m) no campo `behavior_tree` do `NavigateToPose`: o robô já conta como na posição e o shim só gira para o yaw do goal | `manip_bt/src/go_to_ws_bt.cpp` (+hpp), `bt_yaml_executor.cpp` (params `staging_short_dist` 0.30, `staging_short_bt_xml`), `controller_server.yaml` (checker novo), BT nova | SH1→WS2 e stagings pós-escape (P3) | `staging_short_dist:=0` no executor, ou apagar o checker |
| 2c | **Velocidade por proximidade** (pedido do operador): nó `speed_limit_by_scan` publica `/speed_limit` 70 % quando o LiDAR vê algo a < 0,70 m e libera acima de 0,85 m; o `controller_server` aplica no DWB (`setSpeedLimit`: tetos ×0,7 → 0,42 / 0,28 / 0,35; mínimos não mudam). Não vai abaixo de 70 % porque 0,50 × 0,7 = 0,35 = `min_speed_theta`. O shim não é afetado | `scripts/speed_limit_by_scan.py`, CMake, `navigation.launch.py` (`use_speed_limit`, default true) + passthrough nos 2 launches | colisões em curva perto de mesa/parede (quina da parede, WS6) | `use_speed_limit:=false` |
| 2d | **Bag com tudo**: `--all-topics` + regex de exclusão (imagens cruas, profundidade, nuvens, `costmap_raw/updates`, `/marker`, `/evaluation`, `/cost_cloud`, `/trajectories`, theora). Entram braço, garra, MoveIt, actions de pick/place/dock/align, tags, detecções, fala, missão, diagnósticos. Câmera anotada comprimida continua opt-in (`--bag-camera`) | `scripts/caramelo_bag_common.py` (+2 testes) | histórico completo do robô (pedido do operador) | `CARAMELO_BAG_TUDO=0` |

Continua da rodada 1: `rotate_to_heading_once: true`. Continua preparado e desligado:
`bt_replan:=invalid`.

Verificado sem robô: build de `caramelo_navigation`, `caramelo_bringup`, `manip_bt`;
`--show-args` dos 3 launches; 18 testes do bag; gtests do `manip_bt`; nó de velocidade
testado no domínio 99 com scan falso (2 m → sem limite; 0,5 m → 70 %; 0,78 m → mantém;
2 m → libera). As falhas de `colcon test` do `manip_bt` são lint pré-existente (flake8 em
`test/compare_plan_times.py`, uncrustify no `bt_yaml_executor.cpp` já divergia antes).

## 8. Por que não "tirar o shim" nesta rodada

O bag 3 separa os papéis: nas pernas longas o shim gastou 6–8 s por perna (giro inicial +
final, inevitáveis a 0,35 rad/s) e **não** esteve nas três pernas ruins — ali quem mandava
era o DWB (2a) ou a perna curta (2b). Tirar o shim agora transferiria o giro inicial para o
DWB, que gira pior (amostra mínima, vaivém de ±11°) e andaria de lado para o rumo. Se
depois de 2a o caranguejo persistir, a próxima é o lattice `nariz` (planner gera path de
frente), e só então faz sentido medir o shim fora.

## 9. Próximo teste

```bash
powerprofilesctl set performance
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=arena_extendida use_rviz:=true camera_profile:=640x480x15 verify_grasp_effort:=true max_pose_error_m:=0.0 use_vision_viewer:=true
```

Conferir depois que subir: `ros2 param get /controller_server DWBHolonomic.ObstacleFootprint.scale`
→ 0.02; `ros2 topic echo /speed_limit --once` → mensagem; no log do executor da missão,
na saída da SH1: "staging a 0,xx m da pose atual (< 0,30): BT curta". O bag agora grava
tudo: esperar 150–250 MB por missão de 15 min.

O que eu vou olhar no bag: giro por perna (meta < 35 % nas longas, < 15 s na SH1→WS2),
lateral por perna (meta < 5 %), scan mínimo nas curvas (meta > 0,15 m), tempo total de
navegação (meta < 220 s para esta task).

---

# Rodada 3 — testes das 21h08 e 21h15 (caixa "obstáculo dinâmico" junto da WS5)

Bags: `~/bags/20261111_210855_*` (962 MB; o gravador não foi encerrado e seguiu até 21h46,
então ele contém as DUAS corridas: A = 0–190 s, B = a partir de 382 s; `metadata.yaml` foi
refeito com `ros2 bag reindex`) e `~/bags/20261111_211530_*` (corrida B isolada, 535 s).
Primeiros bags com todos os tópicos (198): braço, garra, MoveIt, actions, detecções, fala.

## 10. O que aconteceu (as duas corridas, igual)

1. START→WS1, dock, pick: normais (align por câmera esquadrou a 1–4°).
2. WS1→WS5: navegou bem (25–33 s). Ao chegar na staging da WS5 (3,68; 1,39) virado para
   oeste (178°) e começar o giro para o yaw do goal (90°, de frente para a mesa), com a
   caixa à vista, **o AMCL derrapou**: em 12 s a correção `map→odom` foi de
   (9,81; 0,57; 170°) para (8,93; −1,72; 139°) — 2,4 m e 31° — com o robô girando parado.
   A pose estimada saiu de (3,68; 1,37) para (4,12; 2,15): **dentro da mesa WS5 do mapa**,
   com σ = 0,13 m (o filtro estava convicto). Corrida B: a mesma derrapagem às 87 s e de
   novo às 201 s (+0,08 m por atualização, três atualizações seguidas, sempre para nordeste).
3. Com a pose dentro da mesa: planner "Start occupied" em TODO pedido (76 no bag B),
   `escape_free_space` "NENHUMA direção segura (99 células ruins sob o robô)" porque o
   costmap GLOBAL (mapa estático) diz letal, `BackUp` "Collision Ahead", e o
   `bt_yaml_executor` pulou **8 estações em 90 s** (cada `goto` falha em 3 s sem o robô se
   mexer: WS5, WS8, WS1, WS2, SH1, WS5, WS7… tudo da mesma pose). A missão acabou ali.
4. O "virou para o lado oposto na hora do docking": é o mesmo fenômeno. O shim gira para o
   yaw do goal **no frame do mapa**; com o yaw do AMCL 31° errado (e crescendo), o robô
   físico vira para outro lado, e o `dock_align` com câmera parte de uma pose que não é a
   da mesa.
5. Fora isso: o nó de velocidade funcionou (limitou/liberou 2280 vezes no bag B), o DWB
   não entrou em vaivém (giro 21–32 % do tempo de nav, lateral 15 %), a BT curta não foi
   acionada nestas corridas (SH1→WS2 ficou a 0,24–0,29 m, limiar 0,30; levou 2–12 s), e
   as chegadas nas stagings fecharam com erro de yaw ≤ 10° e posição ≤ 0,25 m.

## 11. Por que o AMCL derrapa com a caixa

- A pose derrapada **casa melhor com o mapa** do que a pose certa quando a caixa está na
  frente do LiDAR: medi, com o scan de 102–104 s, 43–76 % dos feixes a < 10 cm de parede
  do mapa na pose do AMCL contra 0–10 % na pose propagada pela odometria. As paredes da
  caixa, deslocadas 0,4–0,8 m, encaixam nas bordas da WS5 e da parede central.
- Com `z_hit 0,8 / z_rand 0,1` (mapa "afiado", defaults do Nav2 são 0,5/0,5) cada feixe
  que encaixa pesa muito e os que não encaixam quase nada; o filtro colapsa em cima do
  encaixe falso em poucas atualizações (o giro na staging gera uma a cada 3°).
- `do_beamskip` estava `false` e, mesmo que estivesse `true`, só funciona no modelo
  `likelihood_field_prob` (os parâmetros `beam_skip_*` já configurados eram inertes).
- `recovery_alpha 0,001/0,1` injeta partículas aleatórias quando a verossimilhança cai —
  exatamente o que a caixa provoca (viu-se n=1000 e σ 0,4 m nos dois reinícios).

## 12. Correções desta rodada (aplicadas, aguardam teste)

| # | Mudança | Arquivo | Reverter |
|---|---|---|---|
| 3a | AMCL: `laser_model_type likelihood_field_prob`, `do_beamskip true` (beam_skip_distance 0,5 / threshold 0,3 / error 0,9 já estavam), `z_hit 0,8→0,5`, `z_rand 0,1→0,5`, `recovery_alpha_slow/fast → 0,0` | `caramelo_localization/config/amcl.yaml` | valores antigos no comentário |
| 3b | `escape_free_space`: se o costmap global diz ≥ 20 células letais sob o robô e o local (LiDAR) diz 0, ignora o global neste escape e sai pelo LiDAR ("mapa e LiDAR discordam = AMCL deslocado"); é o único jeito de o filtro receber movimento | `caramelo_behaviors/src/escape_free_space.cpp` (+hpp), `behavior_server.yaml` (`ignore_map_if_contradiction`, `map_contradiction_min_cells`) | `ignore_map_if_contradiction: false` |
| 3c | `GoToWS`: a navegação até a staging que falha é re-tentada (`staging_nav_retries` 2) depois de `staging_nav_retry_wait_s` 4 s com 3 pedidos de `/request_nomotion_update` ao AMCL (refaz a atualização do filtro parado). Antes: 1 falha = estação pulada em 3 s | `manip_bt/src/go_to_ws_bt.cpp` (+hpp, novo estágio `kStagingNavRetry`), `bt_yaml_executor.cpp` (params), `package.xml`/CMake (`std_srvs`) | `staging_nav_retries:=0` |
| 3d | Bag: regex de exclusão ganhou `/bond` (117 k msgs), `/controller_manager/*` (5 × 53 k), camadas `*_raw`/`obstacle_layer`/`static_layer`, `dynamic_joint_states`, `monitored_planning_scene`, `display_planned_path`, marcadores | `caramelo_bag_common.py` (+teste) | — |

Verificado sem robô: build de `caramelo_localization`, `caramelo_behaviors`, `manip_bt`,
`caramelo_navigation`; AMCL subido no domínio 99 com o YAML novo e parâmetros lidos de
volta (`likelihood_field_prob`, `do_beamskip True`, 0,5/0,5, 0/0); gtests (escape 9/9,
manip_bt 55/55); 18 pytests do bag; estilo dos arquivos editados igual ao anterior.

**Atenção para o gravador**: o bag das 21h08 ficou aberto até 21h46 (duas corridas no mesmo
arquivo, 962 MB). Isso acontece quando o `run_mission` é morto sem passar pelo `finally`
(kill/fechar terminal). Encerrar a missão com Ctrl-C uma vez e esperar o "[bag] parado".

## 13. O que olhar no próximo teste (com a caixa no mesmo lugar)

- Durante o giro na staging da WS5: `ros2 topic echo /amcl_pose` deve ficar em (3,6–3,8;
  1,3–1,5) com σ < 0,2; no bag, `map→odom` não pode mudar mais que ~0,2 m / 5° com o robô
  parado (script `amcl_e_chegadas.py 78:115`).
- Se mesmo assim derrapar: o log do `behavior_server` deve mostrar "mapa global diz N
  células LETAIS… ignora o costmap global" e o robô sair 0,20 m; e o `GoToWS` "tentativa
  1/2… /request_nomotion_update" em vez de pular a estação.
- Se a derrapagem sumir mas o robô ainda falhar em sair do cercado: aí é geometria (saída
  da caixa × footprint), não localização.

---

# Rodada 4 — teste das 22h03 (rodada 3 ligada): o AMCL atravessou a parede; a odometria estava certa

Bag `~/bags/20261111_220304_*` (240 s). Confirmado pelos logs que a rodada 3 estava ativa
(escape "mapa global diz N células LETAIS… ignora o costmap global" ×4; `likelihood_field_prob`).

## 14. O que os dados mostram

| Janela | Giro pelo **giroscópio** (IMU, independente das rodas) | Giro pela **odometria** (EKF) | Giro pelo **AMCL** |
|---|---|---|---|
| 74–92 s | −115° | −115° | −135° |
| 92–110 s | −74° | −72° | −42° |
| 110–130 s | −68° | −69° | −42° |
| 130–150 s | −153° | −155° | −152° |
| **150–170 s** | **−80°** | **−88°** | **−8°** |
| 74–230 s (total) | **−467°** | **−465°** | **−378°** |

Giroscópio e odometria concordam a 2° em 150 s; o AMCL perdeu 89° de yaw. Em 93–99 s a
pose do AMCL cruzou a parede central (de (4,06; 1,21) para (3,94; 0,55) sem passagem no
mapa), com saltos de até 0,23 m por atualização e nuvem de partículas apertada (σ 0,04 m,
3× mais apertada que antes). Depois disso o robô, fisicamente no corredor norte (WS5),
planejava como se estivesse no corredor sul: "Start occupied" ×30, "no valid path", e o
escape novo chegou a disparar 4× ("mapa diz 20–45 letais, LiDAR diz 0") — correto, mas
tarde demais. **O operador está certo: a odometria + IMU era a fonte confiável e o AMCL a
destruiu.** O `likelihood_field_prob` com beamskip piorou (nuvem mais apertada → salto
mais rápido); as duas mudanças de 3a que ajudam (z 0,5/0,5, recovery 0) ficam.

Nota de leitura: `map→odom` "variou 7 m" neste bag, mas isso é o braço de alavanca de um
erro de yaw de 80° com a origem do odom a 10 m; a medida certa é a pose do AMCL.

## 15. Correção desta rodada: vigia de localização (odometria manda em janelas curtas)

Novo nó `localization_watchdog` (pacote `caramelo_localization`, ligado por default no
`global_localization.launch.py`, `use_loc_watchdog:=false` desliga):

- A cada `/amcl_pose`, compara a pose do AMCL com `(última pose aceita ⊕ deslocamento da
  odometria desde então)`.
- Folga: 0,10 m + 8 % da distância odométrica; 8° + 8 % do giro odométrico (desde a
  última aceitação). Dentro da folga → aceita e passa a seguir a correção lenta do AMCL.
- Fora da folga em 2 atualizações seguidas → publica `/initialpose` com a pose propagada
  pela odometria (σ 5 cm / 3°): o AMCL reinicia ali e não consegue mais "andar" pelo mapa.
- `/initialpose` vindo do RViz (operador) é aceito como nova âncora.
- Lógica pura em `loc_watchdog_core.py`; `loc_watchdog_offline <bag>` reproduz sobre um bag.

Replay offline sobre os bags de hoje (`ros2 run caramelo_localization loc_watchdog_offline`):

| Bag | Atualizações do AMCL | Fora da folga | Disparos | Observação |
|---|---|---|---|---|
| 18h58 (bom) | 1012 | 0 | 0 | |
| 20h31 (bom) | 1463 | 0 | 0 | |
| 21h15 (caixa, derrapou 0,5 m) | 1039 | 0 | 0 | a derrapagem foi "escondida" pelo giro grande na staging; folga de 8 % do giro cobriu — ver §16 |
| **22h03 (atravessou a parede)** | 519 | 373 | 1º aos **91,2 s** (0,08 m / 11°), 2º aos 95,8 s (0,20 m) | teria segurado o AMCL em (3,86; 1,21), do lado certo |

Offline o vigia não consegue reiniciar o AMCL de verdade (por isso "56 resets" na
listagem completa: ele repete a cada 2 s); ao vivo, o 1º reset já re-ancora o filtro.

Também: `amcl.yaml` volta a `likelihood_field` / `do_beamskip false` (comentário datado);
`z_hit/z_rand` 0,5/0,5 e `recovery_alpha` 0/0 mantidos. Verificado: build, `--show-args`,
replay offline acima, nó subido no domínio 99 com odometria e AMCL falsos (não dispara com
drift de 2 %; dispara quando o AMCL translada 0,15 m/atualização com o robô só girando).

## 16. Limites e próximos passos

- O vigia não cobre sequestro real nem patinagem longa das rodas contra obstáculo (aí a
  odometria mente). Se acontecer, RViz "2D Pose Estimate" re-ancora os dois.
- A derrapagem de 0,5 m das 21h15 passou pela folga porque o robô girava ~90° (8 % de
  90° = 7°, e o deslocamento de 0,5 m veio em passos de 0,08 m com aceitação entre eles —
  o vigia segue correções lentas). Se no próximo bag a staging da WS5 derrapar de novo,
  apertar `folga_xy_por_metro` para 0,05 e exigir `strikes_para_reset` 1.
- Próximo teste, com a caixa: olhar `ros2 topic echo /localization_watchdog/status` e o
  log "AMCL fugiu da odometria"; no bag, `loc_watchdog_offline` e a tabela de giros.
