# Navegação do Caramelo: diagnóstico e plano (2026-11-11)

Escrito para a equipe RoboFEI @Work a 7 dias da competição (18–20/11). Reúne o que foi
levantado em 10–11/11 a partir de: documentação do Nav2 Jazzy (1.3.13 instalado), fonte do
Nav2 no branch `jazzy`, configs deste repositório, 47 sessões de log em `~/.ros/log`
(05 a 10/11) e uma bancada offline do planner. **Nada aqui foi validado no robô ainda.**
Regra do time continua: uma mudança por rodada, com medição, e "falhar em melhorar nunca
pode piorar".

Relato do operador (10/11): (1) "caranguejo" em trajetórias longas: anda na diagonal ou de
lado e gira no próprio eixo até conseguir andar na diagonal de novo; (2) colide com o que o
LiDAR vê mal, manobras arriscadas, parece ocupar mais espaço do que precisa; (3) MPPI não
deu certo nos testes; DWBHolonomic e SmacLattice foram os melhores até agora; (4) sem rosbag.
Restrição: ESC, driver das rodas e motores não serão mexidos antes da competição.

---

## 1. O que os dados mostram (fatos)

| # | Fato | Evidência |
|---|---|---|
| F1 | O laço do `controller_server` **nunca** fechou 20 Hz em nenhuma das 47 sessões. Antes de 07/11: mediana 12,7 Hz. Depois (DWB com 1588 trajetórias por ciclo): mediana 7,0 Hz, p10 5,4, mínimo 2,1. | `controller_server_*.log`, aviso "Control loop missed its desired rate"; grade 10×9×22 = 1980 twists, 1588 válidos (bate com "No valid trajectories out of 1588") |
| F2 | O que mudou em 07/11 (304fb09) e explica a queda: `vx_samples` 8→10, `vtheta_samples` 20→21, `max_vel_x`/`max_speed_xy` 0,40→0,60 (mais poses por trajetória). Carga por ciclo dobrou: ~20 mil → ~36 mil poses, ~3,9 milhões de células rasterizadas pelo `ObstacleFootprint`. | `git diff 26d665c 304fb09 -- config/controller_server.yaml`; contas sobre o fonte do `dwb_plugins` |
| F3 | O DWB ainda copia as 1588 trajetórias por ciclo para a mensagem de avaliação mesmo sem ninguém assinar: `publish_evaluation` e `publish_trajectories` estão no default `true`. | `publisher.hpp` jazzy (`shouldRecordEvaluation`) |
| F4 | O defeito mais frequente nos logs não é do controlador, é do planner: **702 "Start occupied"** em 5 dias, 65% deles com o robô a mais de 1,5 m do destino (em movimento, sempre nos mesmos trechos: corredor x 4,4–5,2 / y 0,6–1,4 e área x 3,2–3,6). 82 dos 84 goals falhos têm "Start occupied" na janela; 53 são cascata (nova falha < 15 s após a anterior, mesmo ponto). | `planner_server_*.log`, `bt_navigator_*.log` |
| F5 | "Start occupied" do Smac = célula do **centro** do robô em custo 253/254 na primeira iteração. A faixa 253 é fixada pelo raio inscrito do footprint (0,195 m), não pela inflação. Não é limpo por `ClearEntireCostmap` quando vem da static layer. | `collision_checker.cpp`, `a_star.cpp` jazzy |
| F6 | "No valid trajectories: ObstacleFootprint/Trajectory Hits Obstacle" = perímetro do footprint em célula **letal** (254); custo 253 não rejeita. Com `failure_tolerance` 0,3 s e laço a 7 Hz, 2 ciclos ruins já abortam o FollowPath. 794 eventos em rajadas. | `obstacle_footprint.cpp`, `controller_server.cpp`; logs |
| F7 | O lattice próprio (`caramelo_omni_2cm_16bins.json`, 07/2026) tem 240 giros no lugar (para qualquer heading, custo fixo `rotation_penalty` 1,0 = 1/8 de um passo reto) e 128 translações de heading fixo: só 16 para a frente (0,16 m), 32 laterais (0,10), 32 diagonais frente (0,141), 48 de ré. **No custo do Smac, frente, lado, diagonal e ré custam o mesmo por metro** (`arc_length` 0 em todas; `reverse_penalty` só vale com `allow_reverse_expansion`, que o planner desliga para omni). | `node_lattice.cpp` (getTraversalCost), `smac_planner_lattice.cpp` |
| F8 | Bancada offline (156 pares staging→staging em `arena_extendida`): com o lattice atual, só **7–8% do comprimento das primitivas é para a frente**; 80–86% é lateral, diagonal de ré ou ré. Isso bate com os 77% medidos em 05/11. `cost_penalty` 4,0 não muda nada. | `bench_lattice_offline.py`, bateria de 10/11 (tabela na §4) |
| F9 | A **expansão analítica** do Smac (`analytic_expansion_max_length` 3,0 m) responde por 41–50% do comprimento dos planos e, para omni, é uma reta até o goal com o yaw interpolado linearmente: caranguejo e giro ao mesmo tempo nos últimos metros, **mesmo com um lattice só de frente**. | `analytic_expansion.cpp` jazzy; bancada (`so_frente`: 100% frente nas primitivas, 25% lado+ré no plano final) |
| F10 | O Savitzky-Golay da BT **reescreve o yaw de todas as poses** com a tangente do caminho. O heading do lattice nem chega ao controlador; o planner checou o footprint numa orientação e o robô executa noutra. | `smoother_utils.hpp` jazzy; BT `caramelo_lattice_dwb.xml` |
| F11 | O RotationShim mede o rumo pela posição do ponto a 0,5 m do início do plano e gira só se o erro passa de `angular_dist_threshold` 0,785 rad. As diagonais do lattice são exatamente 0,7854 rad. A BT replaneja a 1 Hz e o shim reavalia a cada plano novo. Resultado compatível com "anda na diagonal, para, gira, anda". | `nav2_rotation_shim_controller.cpp`; `controller_server.yaml` |
| F12 | Nenhum crítico do DWB puxa o nariz para a direção do movimento: PathAlign/GoalAlign/PathDist/GoalDist olham só posições. Andar de lado custa < 1 ponto; girar andando perde ~3,5 pontos de PathDist; girar parado perde ~9 de GoalDist. `PreferForwardCritic` só pune `vx < 0` e penalizaria girar 10× mais que andar de lado: não serve. | `dwb_critics` jazzy |
| F13 | O LiDAR publicado cobre só os 180° da frente (`scan_normalizer`). Nada atrás nem nas laterais traseiras é marcado nem limpo; o collision monitor só protege a frente. Ré existe no escape, no BackUp e no MPPI (`vx_min` −0,25). | `~/caramelo_hardware_ws/.../scan_normalizer.yaml`; configs |
| F14 | Risco de "colidir com o que o LiDAR vê mal": `inf_is_valid: true` (ligado em 07/11) **apaga ativamente** até 3 m o que devolve `inf` (preto fosco, vidro, bordas finas); `denoise_layer.minimal_group_size 2` a 2 cm apaga pernas finas de 1 célula; `footprint_padding 0,0` deixa o DWB aceitar perímetro a 2 cm do letal (default da doc 0,01). | `obstacle_layer.cpp`; doc do denoise e do costmap |
| F15 | MPPI (07 e 09/11): 0 de 15 goals. "Failed to make progress" exatamente aos 6 s (robô não saiu do lugar) e "Optimizer fail to compute path" em rajadas de 0,4 s. `VelocityDeadbandCritic [0.25,0,0]` com peso 35 é por eixo e penaliza `v = 0`: ficar parado ou girar no lugar custa ~22, dar ré a 0,25 custa ~3, diagonal custa 0. Nenhum crítico do 1.3.13 penaliza `vy`. | `velocity_deadband_critic.cpp`, `prefer_forward_critic.cpp`; logs |
| F16 | Quatro parâmetros do YAML do SmacLattice não existem no Jazzy e são ignorados: `goal_heading_mode`, `coarse_search_resolution`, `use_quadratic_cost_penalty`, `downsample_obstacle_heuristic`. | `smac_planner_lattice.cpp` (configure) |
| F17 | Dois travamentos de dados da Pi (07/11 13:08, 7 s; 09/11 13:46, 17 s): scan e TF param juntos; o collision monitor parou o robô, corretamente. | `collision_monitor_*.log` |
| F18 | 09/11 foi o melhor dia (62/66 goals, 0,22 m/s efetivo); 10/11 regrediu (37/55, 0,14 m/s) **com a mesma configuração**: a diferença está no ambiente/posição dos obstáculos e nas cascatas de "Start occupied", não num parâmetro. | `bt_navigator_*.log` |

O que os logs **não** têm: nenhuma velocidade comandada ou medida, pose, costmap ou plano.
O caranguejo não pode ser medido com a base atual. Daí o rosbag ser o item 0.

## 2. Cadeia de causas (hipóteses, com o que as sustenta)

**Caranguejo (H1, forte).** Três camadas que se somam. O planner não tem razão nenhuma para
preferir o nariz à frente (F7) e fecha o fim do caminho com uma reta de heading interpolado
(F9); o suavizador apaga o heading do plano (F10); o controlador nunca vira o nariz sozinho
(F12), só o shim gira, e justamente no limiar das diagonais a cada replan (F11). O
experimento "lattice frente" de 05/11 atacava a camada 1, mas foi testado junto com outras
3 mudanças e piorou; o sintoma não foi registrado. A piora percebida em 09/11 é compatível
com a velocidade 0,40→0,60 e a inflação 0,55→0,75 tornarem o ciclo mais visível (nada no
shim, BT ou lattice mudou em 07/11).

**Laço lento (H2, forte).** Carga do DWB dobrou em 07/11 (F2) sobre um laço que já não
fechava 20 Hz (F1), mais a cópia inútil da avaliação (F3). Fatores secundários não medidos:
perfil de energia do PC, espera de TF da Pi (até 0,6 s), RViz no mesmo PC.

**"Start occupied" e cascatas (H3, aberta).** Centro do robô em custo 253 nos mesmos
trechos, todos os dias (F4, F5). Candidatos: borda de mesa do mapa estático + erro de AMCL;
algo do próprio robô visto pelo LiDAR; mesa baixa vista só pela borda; staging colado na
mesa. A camada de missão reenvia o goal na hora e repete a falha. Sem bag não se decide.

**Colisões e "ocupa espaço" (H4, média).** `inf_is_valid` e denoise apagando obstáculos
finos/escuros (F14); perímetro a 2 cm do letal (padding 0); lateral/diagonal a 0,40 m/s
com as laterais cegas (F13); laço a 5–7 Hz = até 8,6 cm por ciclo sem comando novo.

**MPPI (H5, forte no mecanismo).** O deadband crítico atual torna parar e girar os
movimentos mais caros e deixa ré e diagonal grátis (F15); com `az_max` 2,0 a rampa de giro
fica abaixo do piso do ESC nos primeiros ciclos e o progress checker aborta aos 6 s.

## 3. Plano, por fases (uma mudança por rodada)

Legenda: **ao vivo** = `ros2 param set`, reverte na hora; **relançar** = YAML + relançar o
stack. "Medir" sempre inclui a mediana de "Current loop rate" no log do controller e, a
partir da fase 0, o bag.

### Fase 0: instrumentar e tirar peso morto (sem mudar comportamento)

| # | Mudança | Onde | Como medir / reverter |
|---|---|---|---|
| 0.1 | **Rosbag por corrida** — **implementado em 11/11** (mcap + zstd_fast, lista mínima de 28 tópicos, split 500 MB, cache 16 MB, parada por SIGINT, limpeza automática, `corrida.yaml` com git rev), ligado por padrão no `run_mission` e no `mission_server`; opt-out `--no-bag` / `CARAMELO_BAG=0`; `--bag-extra` (costmap global, partículas), `--bag-camera` (JPEG). Leitura: `ros2 run caramelo_navigation bag_resumo ~/bags/<corrida>` (Hz real do laço, offset de relógio Pi×PC, fração de caranguejo por goal). Doc: `docs/guia_missoes.md` §2.3. Testes: 23 unitários + fumaça real em domínio 99. **Pendente**: medir numa corrida se o laço piora com o bag ligado. | `scripts/caramelo_bag_common.py`, `bag_resumo.py`, `run_mission.py`, `mission_server.py`, `gravar_bag_tour.sh` | `bag_resumo`; comparar mediana do laço com e sem bag |
| 0.2 | **Perfil de energia do PC em desempenho antes da corrida** (`powerprofilesctl set performance`); conferir `energy_performance_preference`. A bancada mediu clock médio de 1,1–1,6 GHz em `powersave` com o laço a 100% de um núcleo: quando a máquina enche (GUI, câmera, manipulação), o núcleo do controller desacelera e o ciclo estica 1,5–2×, o que reproduz os 5–8 Hz dos logs. | sistema | `ros2 topic hz /cmd_vel_nav` antes/depois; `grep MHz /proc/cpuinfo` durante a corrida |
| 0.3 | `DWBHolonomic.publish_evaluation: false` e `publish_trajectories: false` (manter `publish_local_plan`). **Rebaixado**: a bancada do controlador não mediu ganho sem RViz assinando; fica como higiene, sem expectativa de Hz. | `controller_server.yaml` (relançar) | — |
| 0.4 | `failure_tolerance` 0,3 → 1,0 s: rajadas curtas de "Hits Obstacle" deixam de abortar o FollowPath (progress checker de 6 s continua como rede). | `controller_server.yaml` (ao vivo) | nº de "patience exceeded" e de recoveries por goal |
| 0.5 | Limpar do YAML os 4 parâmetros inexistentes (F16) e os comentários errados sobre PathDist/GoalDist rejeitarem 253 e sobre "1890 trajetórias". | `planner_server.yaml`, `controller_server.yaml` | nenhum efeito |
| 0.5b | `planner_server.costmap_update_timeout: 2.0` (ou costmap global a 2 Hz): o planner espera o costmap global, que atualiza a 1 Hz, com timeout default de 1 s → "Costmap timed out waiting for update" (13× nos logs) e replan perdido. | `planner_server.yaml` | contar a mensagem antes/depois |
| 0.5c | `smoother_server.yaml`: `window_size`, `poly_order` e `enforce_path_inversion` não existem no SavitzkyGolay jazzy (janela fixa de 7); remover e anotar. | `smoother_server.yaml` | nenhum efeito |
| 0.6 | Ligar log DEBUG só do shim na próxima sessão de teste: `--ros-args --log-level RotationShimController:=debug`. | `navigation.launch.py` (arg de teste) | ver quando/onde o shim engaja |

### Fase 1: devolver o laço do controlador (meta ≥ 17 Hz, ideal 20)

O que a bancada offline mediu (PC quase ocioso, sem RViz, 4 rodadas por variante; tabela
completa na §4.2): a configuração atual fica em 15–17 Hz com **100% de um núcleo na thread
do DWB**; o custo dominante é o `ObstacleFootprint` (perímetro de 8 pontos rasterizado em
cada pose de cada trajetória) × 1890 trajetórias × poses; publicar markers, `debug_trajectory_details`
e ter RPP/MPPI carregados não mudam nada mensurável.

| # | Mudança | Onde | Medir / reverter |
|---|---|---|---|
| 1.1 | **Amostras do DWB**: `vx_samples` 10 → 8, `vy_samples` 8 → 6, `vtheta_samples` 21 → 15 (bancada "H": 6/6/15 deu 17–20 Hz e tirou o processo da saturação; 8/6/15 é o meio-termo que preserva a grade de vx do estudo de 07/11). Conferir que a grade ainda contém 0,35 rad/s (vtheta ímpar e simétrico) e 0,133 m/s lateral. | `controller_server.yaml` (relançar) | mediana do laço no log; `bag_resumo` (Hz real de `/cmd_vel_nav`) |
| 1.2 | Se ainda < 17 Hz: `sim_time` 1,7 → 1,2 (bancada "G": −35% de avisos) **ou** costmap local 5×5 → 4×4 m (bancada "F": +1,5 Hz), um de cada vez. Não baixar `controller_frequency` só para calar o aviso. | idem | idem |
| 1.3 | Fechar aplicativos de desktop no PC do robô durante a corrida (Firefox, VS Code): na bancada, com eles ativos o laço caiu de 16 para 8 Hz com o mesmo YAML. | operação | CPU do sistema no `bag_resumo`/`top` |

### Fase 2: nariz para a frente nas pernas longas (planner), holonômico onde encurta

A bateria 2 da bancada (§4) mudou a ordem: a alavanca é o **lattice + analítica juntos**.
Subir `rotation_penalty` sozinho com o lattice atual **piora** (87% lateral) e limitar só a
analítica também não resolve (78%).

| # | Mudança | Onde | Medir / reverter |
|---|---|---|---|
| 2.1 | **Lattice "nariz"** (relançar com `lattice_set:=nariz`, default continua `omni`): frente 0,16; diagonais frente com comprimento declarado ×1,5; laterais ×2,0; sem as 48 primitivas de ré; giros como hoje. O launch impõe junto `analytic_expansion_max_length` 3,0 → **0,8** e `rotation_penalty` 1,0 → **3,0**. Na bancada: 90% do plano para a frente (99% nas primitivas), 4,0 giros/plano, comprimento +0,6%, 0 falhas em 156 pares, tempo de plano máx 0,5 s. O lateral continua existindo onde encurta pela metade (ajuste junto à bancada). | `scripts/generate_caramelo_omni_lattice.py`, `config/lattice/caramelo_nariz_2cm_16bins.json`, `navigation.launch.py` | 3 pernas longas no robô com bag: fração do tempo com |vy|>|vx| em `/cmd_vel_nav`; nº de giros no lugar; tempo de plano; reverter = `lattice_set:=omni` |
| 2.2 | Se ainda houver parada-giro nas diagonais que sobrarem: `angular_dist_threshold` 0,785 → 0,95 rad (ao vivo) ou `rotate_to_heading_once: true` (ao vivo) para o shim não reavaliar a cada replan. | `controller_server.yaml` | paradas/giros por perna (bag + DEBUG do shim) |
| 2.3 | **Não** ligar `use_path_orientations` enquanto a BT passar pelo Savitzky-Golay (F10): só mudaria trechos de ré e viraria o robô de costas. **Não** subir `rotation_penalty` com o lattice `omni`. | — | — |

### Fase 3: colisões, espaço e chegada

| # | Mudança | Onde | Medir / reverter |
|---|---|---|---|
| 3.1 | `footprint_padding` 0,0 → 0,03 **só no costmap local** (não no global: lá aumentaria a faixa 253 e os "Start occupied"). | `controller_server.yaml` (relançar) | passagem mais apertada da arena ainda passa |
| 3.2 | Testar ao vivo, parado em frente a uma perna de mesa e a um objeto escuro: `obstacle_layer.scan.inf_is_valid false` e `denoise_layer.minimal_group_size 1` no costmap local; decidir pelo que aparece/some no RViz. | `controller_server.yaml` (ao vivo) | bag desse teste |
| 3.3 | "Start occupied": gravar bag na saída de estação e nos trechos x 4,4–5,2 / 3,2–3,6; conferir no mapa se a borda da mesa fica a < 0,195 m do centro na pose de docking; afastar staging de WS4/WS5/WS6/WS8 (as que falham); na missão, não reenviar o goal na hora após "Goal failed" no mesmo ponto. | `maps/<arena>/service_areas.yaml`, camada de missão (`manip_bt`) | nº de "Start occupied" por corrida |
| 3.4 | Chegada antes do docking: `StoppedGoalChecker` (xy 0,10, yaw 0,20, parado) só para o staging, para o Nav2 só dar sucesso com o robô parado (ESC faz coast ~0,5 s e o `dock_align` publica direto nas rodas). | `controller_server.yaml` + `goal_checker_id` na perna de staging | depende de `/odom` ficar < 0,05 m/s parado |
| 3.5 | Se quiser menos risco lateral: `max_vel_y` 0,40 → 0,30 no DWB (não via collision monitor, que não tem dados atrás). | `controller_server.yaml` | ajuste fino do staging |

### Fase 4: MPPI (experimental; só se sobrar pista, ou depois da competição)

Um grupo por vez, sempre com bag: (1) `VelocityDeadbandCritic` [0.25,0,0]/35 → [0.13,0,0]/10 e
`vx_min` −0,25 → 0,0 (ré só no escape/BackUp); (2) `vy_max` 0,20, `vy_std` 0,08,
`PathAngleCritic` modo 0 / 0,5 rad / peso 3; (3) se o laço continuar < 20 Hz: `time_steps` 40,
`batch_size` 1500, `trajectory_point_step` 3; (4) ideia a testar: segunda instância
`MPPIDiff` (DiffDrive) para pernas longas e `MPPIHolonomic` só no staging. Não atualizar o
MPPI para 1.3.14 por fonte agora.

### O que não fazer agora

- Mexer em `transform_tolerance` para "resolver" o laço (só muda quando desiste).
- `use_realtime_priority` sem configurar `limits.conf` (o configure falha).
- Voltar `cost_scaling_factor` 10 / inflação 0,55: 0,75/3,0 é a única combinação com
  gradiente no centro dos corredores de 1,0–1,2 m.
- Adicionar `PreferForwardCritic` ao DWB (F12).
- Trocar a resolução do costmap global ou o mapa: o lattice está amarrado a 2 cm. Fica como
  opção pós-competição (5 cm, tempo de plano hoje 1,7–3,3 s).
- Smac Hybrid-A* (Dubin) como atalho "zero caranguejo": caminho tipo carro, sem giro no
  lugar; só como experimento controlado, nunca direto em missão.

## 4. Bancada offline do planner (`bench_lattice_offline.py`)

Sobe `map_server` + `planner_server` em `ROS_DOMAIN_ID=99`, namespace `/teste_lattice`, com
uma cópia do `planner_server.yaml` e o lattice sob teste; pede `ComputePathToPose` para todos
os pares de staging do `docking.yaml` e classifica cada passo do plano pela direção do
deslocamento no frame do heading da pose. Só static layer + inflação (sem `/scan`). Como rodar:

```bash
source /opt/ros/jazzy/setup.bash && source ~/caramelo_hardware_ws/install/local_setup.bash && source ~/caramelo_ws/install/local_setup.bash
python3 src/caramelo_navigation/scripts/bench_lattice_offline.py --map-name arena_extendida \
    --lattice <json> [--param SmacLattice.rotation_penalty=3.0] [--out x.json]
python3 src/caramelo_navigation/scripts/bench_lattice_offline.py --compare /tmp/bench_lattice/<bateria>/*.json
```

Bateria 1 (10/11, `arena_extendida`, 156 pares, 0 falhas em todas). `fre_p%` = frente só nas
primitivas; `lat+re_p%` = lateral + diagonal de ré + ré nas primitivas; `ana%` = parte do
plano vinda da expansão analítica; t = tempo de plano mediana/máximo (s).

| variante | fre% | diag fr% | lat% | diag ré% | ré% | ana% | fre_p% | lat+re_p% | giros/plano | t_med | t_max |
|---|---|---|---|---|---|---|---|---|---|---|---|
| atual (omni, 8 translações) | 9,2 | 18,0 | 36,3 | 27,4 | 9,0 | 41,1 | 6,9 | 79,6 | 2,4 | 0,009 | 0,18 |
| atual + cost_penalty 4,0 | 9,2 | 15,1 | 32,3 | 32,7 | 10,7 | 41,6 | 7,6 | 81,9 | 2,8 | 0,011 | 0,24 |
| "frente" do stash (frente + 2 diagonais) | 32,6 | 45,8 | 11,5 | 7,2 | 2,9 | 42,2 | 41,8 | 0,0 | 3,9 | 0,008 | 0,13 |
| sem ré (frente + lat + diag fr) | 16,5 | 37,3 | 34,2 | 8,2 | 3,8 | 42,7 | 14,0 | 39,6 | 3,4 | 0,009 | 0,11 |
| só frente + giro | 65,1 | 10,2 | 9,9 | 9,8 | 5,0 | 42,5 | 100,0 | 0,0 | 4,2 | 0,007 | 0,07 |
| atual, analítica ~desligada | 8,2 | 17,7 | 38,2 | 27,9 | 7,9 | 0,1 | 8,2 | 74,0 | 5,3 | 0,031 | 0,28 |
| só frente, analítica ~desligada | 100,0 | 0 | 0 | 0 | 0 | 0,0 | 100,0 | 0,0 | 6,9 | 0,028 | 0,34 |

Leitura: o lado/ré do plano atual vem metade das primitivas e metade da reta analítica;
limitar a analítica e encarecer lateral/ré são as duas alavancas. Nota: a bancada mede o
**plano**, não o que o DWB executa, e os tempos de plano aqui (ms) são bem menores que os
1,7–3,3 s vistos nos logs reais (costmap com obstáculos e CPU disputada).

Bateria 2 (11/11, `arena_extendida`, 156 pares, 0 falhas em todas). an0,8 = `analytic_expansion_max_length` 0,8; rot = `rotation_penalty`.

| variante | fre% | diag fr% | lat% | diag ré% | ré% | ana% | fre_p% | lat+re_p% | giros/plano | trocas/plano | t_max | L_tot (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| atual + an0,8 | 7,5 | 16,1 | 39,9 | 28,5 | 7,9 | 14,3 | 7,2 | 77,8 | 3,9 | 9,6 | 0,42 | 666,6 |
| atual + rot 5 | 6,0 | 16,6 | 47,0 | 24,2 | 6,2 | 39,7 | 2,3 | **87,4** | 0,6 | 8,0 | 0,26 | 657,9 |
| "frente" do stash + an0,8 + rot 3 | 35,0 | 60,5 | 3,4 | 0,8 | 0,2 | 12,9 | 37,1 | 0,0 | 2,8 | 5,8 | 0,37 | 666,4 |
| **nariz** (frente + diag ×1,5 + lat ×2, sem ré) + an0,8 + rot 3 | **90,0** | 5,8 | 3,0 | 0,9 | 0,3 | 12,0 | 99,0 | 0,2 | 4,0 | 2,5 | 0,51 | 660,7 |
| nariz + rot 3, analítica 3,0 (default) | 65,4 | 11,9 | 10,1 | 8,1 | 4,5 | 43,2 | 98,9 | 0,2 | 2,9 | 2,7 | 0,32 | 649,3 |
| frente + lat ×2 (sem diagonal) + an0,8 + rot 5 | 91,0 | 5,2 | 2,9 | 0,8 | 0,2 | 12,2 | 99,8 | 0,2 | 3,7 | 2,3 | 0,59 | 660,7 |

Leitura: (a) com o lattice atual nenhuma penalidade resolve; `rotation_penalty` alto só tira
os giros e deixa o robô deslizar mais de lado; (b) o conjunto "nariz" com a analítica
limitada a 0,8 m entrega 90% do plano de frente sem alongar o caminho; (c) a diagonal ×1,5
quase não é usada (5,8%), ou seja, sobra para desviar sem parar quando compensa; (d) os
giros por plano sobem de 2,4 para ~4: cada quina vira giro no lugar a 0,35 rad/s (90° ≈ 4,5 s),
que é o custo a medir no robô.

### 4.1 Como testar o lattice `nariz` (`lattice_set:=nariz`)

Implementado em 11/11 (worktree `agent-a4c58d1b95e4b6190`, não commitado): o gerador
`generate_caramelo_omni_lattice.py` tem conjuntos nomeados `omni` e `nariz`
(`config/lattice/caramelo_nariz_2cm_16bins.json` versionado) e o `navigation.launch.py` ganhou
o argumento `lattice_set` (default `omni` = comportamento atual, nada muda sem pedir).
Com `nariz` o launch troca o `lattice_filepath` **e** impõe na cópia do YAML
`SmacLattice.analytic_expansion_max_length 0.8` e `SmacLattice.rotation_penalty 3.0` (os
valores medidos junto na bancada; o `planner_server.yaml` não muda). O argumento passa pelo
`bringup.launch.py` e pelo `robot_manipulation.launch.py` como o `nav_profile`.

```bash
# exige rebuild do caramelo_navigation (JSON novo em config/lattice/ e launch)
ros2 launch caramelo_navigation navigation.launch.py map_name:=<mapa> lattice_set:=nariz
# ou, pelo bringup unificado:
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=<mapa> lattice_set:=nariz
# voltar (default):
ros2 launch caramelo_navigation navigation.launch.py map_name:=<mapa> lattice_set:=omni
```

O que observar (uma perna longa de cada vez, com bag):
- **Plano no RViz** (`/plan`): as setas das poses devem apontar na direção do deslocamento
  (nariz à frente); diagonais só em desvio curto; lateral quase inexistente.
- **Quinas**: giro no lugar antes de seguir (esperado ~4 giros por plano contra ~2 hoje; a
  0,35 rad/s um giro de 90° leva ~4,5 s). Se o robô ficar girando e re-girando a cada
  replanejamento de 1 Hz, é o RotationShim/BT, não o lattice: anotar e voltar ao `omni`.
- **Tempo de plano** no log do `planner_server`: na bancada o máximo subiu de ~0,1 s para
  ~0,6 s; no robô (costmap com obstáculos, CPU disputada) deve ficar abaixo de
  `max_planning_time` 5,0 s. Se aparecer "Planner timed out" ou "no valid path", voltar.
- **Execução**: fração do tempo com |vy| > |vx| em `/cmd_vel_nav` (deve cair), raspões na
  lateral (devem sumir), e se o DWB segue o plano sem "caranguejar" nas retas.
- Voltar = relançar com `lattice_set:=omni` (default); nenhum YAML foi alterado.

Validação offline da implementação (bateria 3, 11/11, `ROS_DOMAIN_ID=99`, namespace
`/teste_lattice3`, staging→staging, 0 falhas em todas; o `omni` regenerado é byte-idêntico
ao versionado, `cmp` rc=0):

| arena (pares) | conjunto | fre% | diag fr% | lat% | diag ré% | ré% | ana% | fre_p% | lat+re_p% | giros/plano | t_med | t_max | L_tot (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| arena_extendida (156) | omni (default) | 9,2 | 18,0 | 36,3 | 27,4 | 9,0 | 41,1 | 6,9 | 79,6 | 2,4 | 0,009 | 0,105 | 654,0 |
| arena_extendida (156) | **nariz** + an0,8 + rot 3 | **89,9** | 5,9 | 3,1 | 0,8 | 0,3 | 12,5 | 98,9 | 0,2 | 4,0 | 0,039 | 0,569 | 661,0 (+1,1 %) |
| arenalegal_2026 (90) | omni (default) | 9,8 | 15,3 | 36,5 | 27,1 | 11,4 | 50,6 | 7,0 | 86,1 | 1,7 | 0,006 | 0,081 | 296,7 |
| arenalegal_2026 (90) | **nariz** + an0,8 + rot 3 | **87,5** | 8,0 | 3,1 | 1,2 | 0,3 | 16,8 | 98,6 | 0,2 | 3,3 | 0,015 | 0,345 | 299,3 (+0,9 %) |

Saída completa: `/tmp/bench_lattice/bateria3_lattice_set_*/` (JSON + CSV + logs por rodada).

### 4.2 Bancada offline do controlador (`bench_controller_offline.py`)

Sobe `controller_server` + costmap local em `ROS_DOMAIN_ID=99`, namespace `/teste`, com
odometria e `/scan` falsos (robô parado, campo a 2 m, 1441 feixes), manda um `FollowPath`
reto de 4 m e mede a taxa de `/teste/cmd_vel_nav`, os avisos de "missed rate" e a CPU do
processo por 20 s, por variante de YAML. Recusa rodar fora do domínio 99; mata só os PIDs
que criou. Como rodar:

```bash
export ROS_DOMAIN_ID=99
python3 src/caramelo_navigation/scripts/bench_controller_offline.py            # 12 variantes x 2 cenarios (~13 min)
python3 src/caramelo_navigation/scripts/bench_controller_offline.py --variant A --variant H --scenario reta
```

Resultado de 11/11 (média de 4 rodadas; PC em `powersave`, sem RViz, desktop com Firefox/VS Code abertos):

| Variante | laço Hz | ciclo ms | avisos/10 s | CPU do processo (% de 1 núcleo) |
|---|---|---|---|---|
| A atual (produção) | 15,6 | 64 | 156 | 103 |
| A2 repetição de A no fim | 16,5 | 61 | 165 | 103 |
| B vx 8 / vy 8 / vtheta 20 (pré-07/11) | 16,9 | 59 | 145 | 103 |
| C atual + `publish_*` do DWB false | 10,9 (ruído do desktop) | 97 | 109 | 103 |
| D atual + `debug_trajectory_details` false | 13,3 | 75 | 133 | 104 |
| E só DWBHolonomic carregado | 14,6 | 69 | 146 | 104 |
| F costmap local 3×3 m | 17,2 | 58 | 147 | 103 |
| G `sim_time` 1,2 | 17,0 | 59 | 102 | 102 |
| **H vx 6 / vy 6 / vtheta 15** | **18,9** | 53 | **45** | **87** |
| I `BaseObstacle` no lugar de `ObstacleFootprint` (só medida, não é proposta) | 19,7 | 51 | 9 | 62 |

Leitura: o laço é CPU-bound numa thread (97% no DWB, 1–3% no costmap, <1% DDS); só reduzir
trajetórias (H) ou o custo por pose (I) devolve os 20 Hz; o resto é efeito de segunda ordem
ou ruído. O ruído entre rodadas (A vs A2, C) vem do resto do PC: com `powersave` o clock fica
em 1,1–1,6 GHz e cai mais quando a máquina enche.

## 5. Fontes principais (Nav2 Jazzy)

- Smac Lattice: `configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/`; fonte `nav2_smac_planner/src/{node_lattice,smac_planner_lattice,analytic_expansion,collision_checker,a_star}.cpp`; gerador `nav2_smac_planner/lattice_primitives/README.md`.
- DWB: `controller_plugins/dwb_controller/` (iterator, standard_traj_generator, visualization, críticos); fonte `nav2_dwb_controller/{dwb_core,dwb_critics,dwb_plugins}`.
- RotationShim: `controller_plugins/configuring_rotation_shim_controller/`; fonte `nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp`.
- MPPI: `controller_plugins/mppi_controller/configuring_mppic/`; fonte `nav2_mppi_controller/src/critics/*`.
- Costmaps, collision monitor, velocity smoother, controller server, behavior server e guia de tuning: `docs.nav2.org/jazzy/tuning/` e `configuration_and_development/configuration_guide/core_servers/*`.
- rosbag2 0.26.11 (mcap default, `zstd_fast`): `ros2 bag record --help` local e README do `rosbag2_storage_mcap`.

Observação: a doc mudou de estrutura; URLs antigas `/jazzy/configuration/packages/...` dão
404; as páginas vivem em `/jazzy/configuration_and_development/...`.

## 6. Estado em 11/11 02h e pendências

**Pronto no PC (compilado, testado, não commitado, parâmetros de movimento inalterados):**
rosbag por corrida ligado por padrão; `lattice_set:=nariz` disponível (default `omni`);
duas bancadas offline instaladas; `planner_server.yaml` sem os 4 parâmetros inertes.

**Verificação adversarial**: 11 de 27 vereditos concluídos, nenhum refutou o núcleo
(laço CPU-bound do DWB, custo direcional nulo do lattice, reta analítica, shim no limiar,
deadband do MPPI); correções foram de precisão e já estão refletidas acima. Os 16 restantes
caíram no limite de API (religar depois das 5h). Auditoria parâmetro a parâmetro concluída
(itens 0.5, 0.5b, 0.5c vêm dela).

- Tudo o que depende do robô: cada fase acima, com bag. Primeira corrida = **baseline com o
  bag ligado, sem mexer em nada**, para ter a medida de referência do caranguejo e do laço.
- Tudo o que depende do robô: cada fase acima, com bag.
- O que está rodando na Raspberry (branch, firmware do ESC, IMU ativa) não foi conferido.

## 7. Primeira rodada de feedback com bags (11/11 à noite)

Os dois primeiros bags das tasks ATT1 estão analisados em `docs/relatorio_testes_2026-11-11.md`.
Em resumo: 55–74 % do tempo de navegação é giro parado do RotationShim (reengaja a cada
replan; em paths < 0,5 m vira vaivém), os dois goals abortados foram um beco no corredor
WS5/parede central, e há trancos físicos abaixo da referência. A ordem das fases deste plano
foi revista lá (§3): rodada 1 = `rotate_to_heading_once: true` (aplicada); rodada 2 =
`bt_replan:=invalid` (pronta); depois a Fase 1 (amostras) e a Fase 2 (`nariz`) daqui.
