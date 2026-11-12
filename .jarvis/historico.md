# Histórico (resumos datados)

Fontes: transcrições em `~/.claude/projects/-home-robofei-work-caramelo-ws/` (sessões `c4e8f5da` 19–27/10, `668df389` 27/10, `99a9f780` 27/10–10/11; horários em UTC, local = UTC−3), `git log`, docs. Antes de 19/10 só há os commits (jun–set/2026 vieram do PC de desenvolvimento; branches `robot_manipulation*`, `robot_ui`, `dev`).

## 2026-10-19 a 23 — Place pela visão, tags duplicadas, voz (sessão c4e8f5da)
- **Pedido (19/10)**: integrar o place via visão do outro PC (commit "vision_place") com as correções locais de tags duplicadas; critério: colocar cubos seguindo o rulebook sem ver tag "fantasma".
- **21/10**: teste de missão BMT no robô; operador informa as regras físicas da PP/pilha (ver `decisoes.md`); análise da run em `docs/analise_run_2026-10-21.md`; pediu ver a câmera anotada (nó `vision_debug`).
- **23/10**: contexto perdido por fim de tokens; operador pede retomar pelos prompts anteriores; escolhe −0,25 (parâmetro não identificado na transcrição); pede a fila de voz com prioridade (robô "age mais rápido que a voz") → `docs/voz_do_robo.md`.

## 2026-10-26 e 27 — Lista de 7 itens e docking (sessões c4e8f5da, 668df389, 99a9f780)
- **26/10**: lista de itens do operador (tour de docks, docking perto de paredes, borda por profundidade, Dynamixel/U2D2, planejador por tempo, merge da GUI, incidente da arena). Vários agentes em worktrees (`.claude/worktrees/agent-*`, branches `worktree-agent-*`); tudo mesclado em 26/10 (commits b86cad1…1ec95ca). Quadro: `docs/status_itens_2026-10-26.md`.
- **27/10**: erro de compilação do pick corrigido (sessão curta); pedido de câmera no docking + shelf lenta; AMCL 350/600 pedido; tours de 26/10 analisados.

## 2026-10-31 — Arena refeita
- Remapeamento (`arena_extendida`), service areas, docking, tour de 31/10, missões geradas; commits 0e76eee, 1b7b64c, 7023a02, e5fa745 (AMCL 600/1000), 76041b2, 76ca80a. Operador pediu os commits ("claude dá o commit para mim").


## 2026-10-26 a 31 — Itens 1–7 do operador
- **Pedido**: tour de docks, docking torto, braço lento, planejador por tempo, merge da GUI, investigação do incidente da arena.
- **Ação/resultado**: todos mesclados (ver quadro em `docs/status_itens_2026-10-26.md` e commits 2330a54…76ca80a). 31/10: docking por câmera D455, giro em pulsos, mapa `arena_extendida`.
- **Pendências**: validações no robô (docking, planejador, gate de pose) sem registro posterior.

## 2026-11-03 a 05 — Pilha, slots da PP e task planner (sessão 99a9f780)
- **03/11**: "verificar a tag que vai empilhar antes de pegar o bloco" → observação prévia de garra vazia; "verifica, mexe e derruba tudo" → verificação só informativa; "escadinha em x" → offset da pilha; "braço não alcança na referência".
- **04/11**: "o centro da tag fica uns 4 cm acima do espaço" → slots pela lógica de pilha com −4 cm; AMT1 casando slot↔tag 101–107; "cada hora numa pose diferente" → ainda sem solução definitiva (origem da T1); operador pergunta se dá para calibrar a câmera durante a task.
- **05/11**: task planner (menos passagens, re-tentativa no fim), blocos mais afastados; tarde: "espaços apertados, curva fechada de lado, só LiDAR na frente" → experimento de lattice "frente".
- **Commit**: 26d665c (05/11) reúne tudo isso.

## 2026-11-06 — Nav lattice "frente" revertido; início do detector de buraco
- **Pedido**: "volta a navegação como estava antes, essa última modificação piorou o robô" → stash. À noite: "está chegando muito perto dos obstáculos e às vezes colide" → "faça o passo 1". Depois: "PP é branca e o buraco é marrom, consegue identificar o centro do buraco para usar como referência?" → primeira versão do `pp_hole_detector` (ficou sem commit e foi sobrescrita em 07/11; recuperada em 09/11).

## 2026-11-07 — Rodada de navegação + incidente do pkill
- **Pedido**: alternar DWB/MPPI, tuning, escape de obstáculo, recoveries.
- **Ação**: pacote `caramelo_behaviors` (EscapeFreeSpace), BTs gated, DWB ObstacleFootprint 1.0, costmaps, respawn. Voz: silenciados avisos do torch.
- **Resultado**: gtest 7/7; escape testado em `/teste`. Incidente: `pkill -f` matou o controller real (ver `decisoes.md`). Nada validado no robô ainda.

## 2026-11-09 — PP: recuperação, fusão tag + buraco, tamanho da tag, commits
- **Data confirmada** pela transcrição: 14h às 21h20 locais de 09/11 (17:02Z a 00:19Z).
- **Pedidos**: (1) "voltamos commits, perdeu o empilhamento?" → não, estava em 26d665c; o perdido era o detector de 06/11, recuperado do transcript e de blobs soltos. (2) Build quebrado em `libfastcdr.so.2.2.7` → caches de `build/` apagados, 20 pacotes OK. (3) "seria bom uma fusão dos dois" → fusão implementada (workflow com revisores, 11 problemas corrigidos). (4) Calibração câmera–braço rodada → inconclusiva. (5) Tag 0.032 testada por pedido ("0.32 de ponta a ponta") → quebrou os picks → voltou a 0.041. (6) "faça todos os ajustes para ele colocar no buraco certo" → offset −0,052, escadinha fora do slot, debug de imagens ligado. (7) "parece que não usa a visão" → logs provam uso em 5/6 slots; resíduo 8–17 mm com sinal dependente do lado. (8) "explique e commite tudo" → 4 commits c1c7873..833c13d.
- **Pendências**: foto do resultado (T1), T2–T7 em `estado.md`.

## 2026-11-10 — Memória compartilhada `.jarvis/` (dois pedidos)
- **Pedido 1 (17h50 local)**: sistema simples de memória persistente entre agentes (README, contexto, estado, decisões, histórico, skills) + integração mínima; sem commit. **Ação**: criada `.jarvis/` e `CLAUDE.md` mínimo na raiz.
- **Pedido 2 (noite)**: leitura aprofundada do workspace, recuperar históricos, cruzar fontes e consolidar `.jarvis/`; só leitura, sem corrigir nada, sem commit. **Ação**: 3 agentes de leitura (nav, manipulação, GUI/docs) + git + 3 transcrições + memória pessoal; `estado.md` ganhou T9–T13, 20 divergências e seção de cobertura; `decisoes.md` ganhou 6 decisões recuperadas; `contexto.md` ganhou arquitetura e persistência. Nada fora de `.jarvis/` foi alterado; nada commitado.
- **Correção registrada**: o relato do pedido 1 dizia que o `CLAUDE.md` aponta para "§5 e §8 do README do Jarvis"; na verdade aponta para §5 e §8 do `README.md` do projeto (build/testes e convenções).

## 2026-11-10 (noite) — Complementos do Claude Code na máquina (fora do repo)
- **Pedido**: instalar ponytail, document-skills e graphify em todas as instâncias do Claude Code desta máquina; auditar se a versão diferir da auditada em 08/10; sem commit.
- **Resultado**: uma instância (Linux, binário da extensão do VS Code 2.1.322; `claude` não está no PATH). document-skills instalado (commit 2c5958e do `anthropics/skills`, auditado: sem hooks, sem rede). graphify 0.9.80 (CLI via uv 0.13.7 + skill em `~/.claude/skills/graphify`), teste OK. **ponytail NÃO instalado**: versão 5.3.3 difere da auditada (5.0.0), com módulo novo que grava CSV por turno e tem webhook opcional, e falta Node (apt tem 18.19.1, exige sudo). Depois o operador instalou o Node 18.19.1 (`sudo apt install nodejs`) e o ponytail 5.3.3, aceitando a versão; hook de início testado à mão em 10/11, OK. `userConfig` não configurado (ledger CSV ligado por padrão, webhook vazio). Atalho `~/.local/bin/claude` criado (chama o binário mais novo da extensão).
- **Achado à parte (não corrigido)**: desde 04/11 o apt termina com erro nos pacotes `linux-modules-nvidia-580/550` porque faltam os headers do kernel 7.0.0-42 (`/usr/src/linux-headers-7.0.0-42-generic` não existe). O módulo NVIDIA não carrega; a GPU MX570 não é usada pelo robô. Corrigir só depois de 18/11.
- **Mudanças fora do repo**: `~/.claude/settings.json` (marketplaces, `enabledPlugins`, `env.GRAPHIFY_NO_AUTO_REFRESH`; backup `settings.json.bak-plugins`); `~/.bashrc` e `~/.profile` ganharam `. "$HOME/.local/bin/env"` (backups `*.bak-uv`).

## 2026-11-10 (noite) — Contexto da cadeia de navegação e movimento (T14)
- **Pedido**: recuperar o contexto técnico de Nav2, controle, ros2_control, baixo nível, odometria e localização; só ler e atualizar `.jarvis/`; não mexer em código/parâmetros nem mover o robô; esperar o relato dos defeitos.
- **Ação**: leitura direcionada das configs do Nav2 deste repo e um agente de leitura no `~/caramelo_hardware_ws`; conferidos à mão EKF, controller da Pi e geometria do URDF. Recuperados das transcrições os relatos de 05–07/11.
- **Resultado**: mapa em `contexto.md`; fatos F1–F9, hipóteses H1–H4 e lacunas em `estado.md` (T14). Nada alterado fora de `.jarvis/`.

## 2026-11-10 (noite) — Bancada OFFLINE do SmacLattice (agente em worktree, ROS_DOMAIN_ID=99)
- **Pedido**: bancada offline reprodutível para comparar conjuntos de primitivas e penalidades do planner global antes de testar no robô; sem commit; sem tocar YAML de produção; nunca `pkill`.
- **Ação**: script novo `src/caramelo_navigation/scripts/bench_lattice_offline.py` (+ `install(PROGRAMS ... RENAME bench_lattice_offline)` no CMakeLists) no worktree `.claude/worktrees/agent-ab0581f4212b2a731` (branch `worktree-agent-ab0581f4212b2a731`, apontada para 833c13d; **não commitado, não mesclado**). Sobe `map_server` + `planner_server` (cópia do `planner_server.yaml` em /tmp com raiz = namespace, `lattice_filepath` absoluto, `obstacle_layer` removida por não haver `/scan`) + 2 TF estáticos em `/teste_lattice`, faz as transições de lifecycle por serviço, espera o costmap assumir o tamanho do mapa, chama `ComputePathToPose` (`SmacLattice`, `use_start`) entre todos os pares ordenados de staging de docks do `docking.yaml`, classifica cada passo no frame do heading (frente ±22,5°, diag frente, lateral, diag ré, ré, giro no lugar) e separa passos de primitiva (≤ 0,02 m) dos da expansão analítica (0,028 m). Encerra pelos PIDs (try/finally + SIGINT). Lattices de teste gerados em `/tmp/bench_lattice/lattices/` (gerador do stash@{0} e cópia parametrizada no scratchpad da sessão; o `omni` regenerado é byte-idêntico ao versionado e o `frente` ao do stash).
- **Rodadas**: 9 variantes × 2 mapas (`arena_extendida` 13 docks/156 pares; `arenalegal_2026` 10 docks/90 pares), 0 falhas (nenhum "Start occupied" com `tolerance` 0,10 e staging). Saída: `/tmp/bench_lattice/bateria_20261110_210558/` (JSON+CSV+logs por variante). Tabela: `python3 src/caramelo_navigation/scripts/bench_lattice_offline.py --compare /tmp/bench_lattice/bateria_20261110_210558/*.json`.
- **Números (arena_extendida; % do comprimento do plano; arenalegal_2026 entre parênteses quando difere muito)**:
  - atual (omni 8 translações): frente 9 / diag-frente 18 / lateral 36 / diag-ré 27 / ré 9; contando só primitivas: frente 7 %, lado+ré **80 %** (86 % na arenalegal); 2,4 giros/plano; 7,8 trocas de classe/plano; t mediana 9 ms, máx 0,18 s.
  - atual + `cost_penalty` 4,0: praticamente igual (frente 9, lado+ré primitivas 82 %); t máx 0,24 s.
  - "frente" do stash (frente + 2 diagonais frontais): frente 33 / diag-frente 46 / lateral 12 / diag-ré 7 / ré 3; só primitivas: 0 % lado+ré; 3,9 giros/plano; 5,1 trocas.
  - sem ré (frente + laterais + diag frontais): frente 17, lateral 34 (45 na arenalegal); só primitivas lado+ré 40 % (62 %).
  - frente + ré 0,08 + diag frontais: ré 32 % (26 %) do plano.
  - só frente + giro: frente 65 % (58 %); só primitivas 100 % frente; 4,2 giros/plano; 2,6 trocas.
  - **Expansão analítica = 41–43 % (arena_extendida) e 50 % (arenalegal) do comprimento em TODAS as variantes** (`analytic_expansion_max_length` 3,0 m): é uma reta em xy até o goal com o yaw interpolado entre bins, que no frame do heading vira lateral/ré mesmo quando o lattice só tem frente. Com `analytic_expansion_max_length` 0,02 (praticamente desligada): atual → frente 8 / lado+ré 74 %, 5,3 giros/plano; "frente" do stash → frente 40 / diag-frente 60 / 0 lateral-ré, 7 giros/plano; só frente → 100 % frente, 0 trocas, 6,9 giros/plano; planning_time mediana sobe de ~9 ms para ~30 ms (máx 0,34 s), comprimento +2 %.
- **Limites da medida**: offline, só static_layer + inflação (sem `/scan`), staging→staging; mede o plano, não o que o DWB executa; a classe "lateral" inclui a interpolação de yaw da expansão analítica. Não houve interpretação além dos números.
- **Pendências**: operador decidir conjunto/parâmetros a testar no robô; se quiser, mesclar o script do worktree; a bancada do controller é de outro agente.

## 2026-11-10 (noite) → 11/11 (madrugada) — Pesquisa e plano de navegação (T14)
- **Relato do operador**: caranguejo em pernas longas, colisões com o que o LiDAR vê mal, MPPI não deu certo, DWB+SmacLattice melhores, sem rosbag; ESC/driver intocáveis até a competição; "dividir em vários agentes paralelos".
- **Ação**: workflow com 6 pesquisas na doc/fonte Nav2 jazzy + 3 análises locais (forense de 47 sessões de log, matemática do caranguejo, auditoria) + verificação adversarial (parte caiu no limite de API e foi religada); bancadas offline do planner (2 baterias, 24 rodadas) e do controller; agentes de implementação (rosbag por corrida; `lattice_set:=nariz`).
- **Resultado**: `docs/plano_navegacao_2026-11-11.md`. Nada alterado nos parâmetros de produção; nada no robô; nada commitado.

## 2026-11-11 (madrugada) — `lattice_set:=nariz` implementado como opção desligada (agente em worktree)
- **Pedido**: conjuntos nomeados no gerador do lattice, argumento `lattice_set` (default `omni`) no launch com os dois overrides do SmacLattice, limpeza dos 4 parâmetros inexistentes no Jazzy, validação offline em duas arenas, doc. Sem commit; default de produção inalterado.
- **Ação** (worktree `.claude/worktrees/agent-a4c58d1b95e4b6190`, branch `worktree-agent-a4c58d1b95e4b6190` sobre 833c13d; **não commitado, não mesclado**): `scripts/generate_caramelo_omni_lattice.py` (TRANSLATION_SETS `omni`/`nariz`, fator de custo só no `trajectory_length`; sem argumento gera os dois JSONs), novo `config/lattice/caramelo_nariz_2cm_16bins.json` (320 primitivas), `navigation.launch.py` (LATTICE_SETS + arg `lattice_set`; `nariz` injeta `analytic_expansion_max_length 0.8` e `rotation_penalty 3.0` na cópia do YAML), repasse em `caramelo_navigation/launch/bringup.launch.py` e `caramelo_bringup/launch/robot_manipulation.launch.py`, `planner_server.yaml` só comentários + remoção de `goal_heading_mode`, `coarse_search_resolution`, `use_quadratic_cost_penalty`, `downsample_obstacle_heuristic` (0 ocorrências em `libnav2_smac_planner_lattice.so`; os dois últimos só existem no Hybrid). Doc: §4.1 para `docs/plano_navegacao_2026-11-11.md` (texto pronto no scratchpad da sessão; o worktree não pode editar o checkout principal).
- **Verificado**: `omni` regenerado byte-idêntico ao versionado (`cmp` rc=0); `--show-args` dos 3 launches lista `lattice_set` (rc=0); função de override testada direto (omni só caminho; nariz caminho + 0.8 + 3.0). Bateria 3 (`/tmp/bench_lattice/bateria3_lattice_set_*`, domínio 99, ns `/teste_lattice3`, 16 PIDs encerrados por SIGINT): frente omni→nariz 9,2 %→89,9 % (arena_extendida, 156 pares) e 9,8 %→87,5 % (arenalegal_2026, 90 pares); 0 falhas; giros/plano 2,4→4,0 e 1,7→3,3; t_max 0,10→0,57 s e 0,08→0,35 s; comprimento +1,1 % / +0,9 %.
- **Pendências**: operador decidir se mescla o worktree e faz rebuild do `caramelo_navigation`; teste no robô com bag (ver §4.1 do plano). Nós `/teste/*` e `/caramelo_bag_*` vistos no domínio 99 são de outros agentes (bancada do controller e rosbag), não desta rodada.
- **11/11 01h**: rosbag por corrida mesclado e testado na árvore principal (patch do worktree agent-ae3973a0652de779b; CMake mesclado à mão com o bloco da bancada). Build caramelo_navigation OK; 23 pytests + fumaça real (domínio 99) OK.

## 2026-11-11 (madrugada) — Bancada offline do controller_server (agente em worktree, domínio 99)
- **Pedido**: medir offline o que derruba os 20 Hz do `controller_server` (DWB + RotationShim) por variante de `controller_server.yaml`, sem robô e sem RViz.
- **Ação**: `src/caramelo_navigation/scripts/bench_controller_offline.py` (novo; roda com `python3`, sem build; odom/TF/scan falsos, controller real em `/teste` no domínio 99, FollowPath reto e diagonal 40°, 2 séries × 12 variantes × 2 cenários, janelas de 20 s). Resultados em `/tmp/bench_controller/` (`20261111_000851.json`, `20261111_001941.json`, `consolidado_2026-11-11.md`; voláteis).
- **Resultado**: config atual = laço ~15–17 Hz com a thread de controle a ~97% de UM núcleo (CPU-bound, single-thread); costmap update = 1–3%. Só duas variantes devolvem 20 Hz: `I` (BaseObstacle no lugar de ObstacleFootprint, CPU 52–76%, só medida) e `H` (6/6/15 amostras, CPU 73–101%). `F` (costmap 3×3), `G` (sim_time 1,2) e `B` (8/8/20) dão ~17 Hz; `C` (publish_* off), `D` (debug off) e `E` (só DWB carregado) não mudam nada mensurável. O ciclo estica de ~60 para 80–125 ms quando o resto do PC está carregado (governor `powersave`, clock médio 1,1–1,6 GHz): explica os 5–8 Hz dos logs reais (robô roda AMCL, GUI, câmeras junto). Nenhum nome de parâmetro foi recusado pelo Nav2.
- **Pendências**: nada alterado em produção (só medidas); decisão de tuning fica com o time; a bancada mede com robô parado (custo por ciclo é independente do movimento).

## 2026-11-11 (noite) — Primeiros bags das tasks ATT1 analisados; rodada 1 aplicada
- **Pedido do operador**: dois testes ATT1 (18h58 sem obstáculo, com ressalvas: giro "para relocalizar", problemas de dock, tranco na WS1; 19h16 com obstáculo novo no meio do mapa: colisão na parede central, robô "girando perdido", empurrado à mão). Analisar os bags, relatar, já corrigir; ciclo de feedback a repetir.
- **Ação**: `bag_resumo` + 7 scripts ad hoc (agora em `src/caramelo_navigation/scripts/bag_forense/`): rosout, CSV de velocidades/pose/status, janelas, rumo por plano, figuras mapa+trilha+planos+costmap, detecção de trancos com `/joint_states`. Fontes Nav2 Jazzy conferidas (shim: `rotate_to_heading_once`, `isCollisionFree` só letal; planner `is_path_valid` footprint só letal; DWB `ObstacleFootprint.getScale = resolução × scale`).
- **Resultado**: `docs/relatorio_testes_2026-11-11.md` (A1 giro parado 55/74 %, A2 beco WS5 e quina, A3 trancos 7+17, A4 laço 7 Hz, A5 avisos). Aplicado: `rotate_to_heading_once: true`. Preparado: `bt_replan:=invalid` (BT nova + arg em 3 launches). Build OK, `--show-args` OK, seleção do XML testada em Python. Nada commitado; robô não foi tocado.
- **Pendências**: teste da rodada 1 com bag; depois rodada 2; amostras do DWB; `lattice_set:=nariz` ou `ObstacleFootprint.scale` 0,02; pernas curtas pelo mission_server; ticket de hardware dos trancos; `bag_nav_profile` no launch do mission_server.

## 2026-11-11 (22h–23h) — Bag 3 (rodada 1 testada) e rodada 2 aplicada
- **Pedido**: "ainda está dando o mesmo problema"; analisar tudo, não só a repetição; ir mais devagar perto de objetos/curvas; "o DWB entra em conflito com o rotation shim"; bag com TUDO (manipulação, visão, controle, missão).
- **Análise**: `bag_forense` sobre `20261111_203107_*`: nav 278 s, giro 44 %, laço 16 Hz; três pernas ruins dissecadas (corredor WS5 = DWB, WS6 = caranguejo do DWB, SH1→WS2 = shim em path curto). Mediu-se também que o giro real do shim = 0,91–0,94 do comandado (não há overshoot pelo piso de 0,34) e que o vaivém vinha de troca de alvo, não de rampa.
- **Mudanças**: `controller_server.yaml` (`ObstacleFootprint.scale` 0.02, `staging_goal_checker`), `behavior_tree/caramelo_lattice_dwb_curta.xml`, `manip_bt` (`go_to_ws_bt.cpp/.hpp`: assinatura `/amcl_pose`, BT curta por distância; `bt_yaml_executor.cpp`: params `staging_short_dist`/`staging_short_bt_xml`), `scripts/speed_limit_by_scan.py` + CMake + `use_speed_limit` em 3 launches, `caramelo_bag_common.py` modo TUDO (+2 testes, 18 passando). Build dos 3 pacotes, `--show-args`, gtests manip_bt e fumaça do nó (domínio 99) OK. Relatório §6–§9. Nada commitado.

## 2026-11-12 (00h–01h) — Bags 4/5 (caixa dinâmica): AMCL derrapa; rodada 3 aplicada
- **Pedido**: "ele identificou o obstáculo e não reconheceu onde estava… se perdeu… na hora do docking virou para o lado oposto… analisa de forma completa".
- **Análise**: reindex do bag 4 (gravador ficou aberto, 2 arquivos); `amcl_e_chegadas.py` (saltos, σ, `map→odom`, nuvem de partículas, chegadas), overlay scan×mapa, teste de encaixe do scan nas duas hipóteses de pose. Conclusão: derrapagem do AMCL na staging da WS5 em ambas as corridas, cascata de falhas por "Start occupied".
- **Mudanças**: `amcl.yaml` (modelo prob + beamskip + 0,5/0,5 + recovery 0; validado no domínio 99), `escape_free_space` (contradição mapa×LiDAR), `go_to_ws_bt` (+`kStagingNavRetry`, cliente `/request_nomotion_update`; params no executor; `std_srvs`), regex do bag. Build de 4 pacotes, gtests e pytests OK, estilo inalterado. Relatório §10–§13. Nada commitado.

## 2026-11-11 (22h10–22h40) — Bag 22h03: AMCL cruzou a parede; vigia de localização
- **Pedido**: "deu o mesmo problema… o robô estava do lado errado do mapa… a confiança da odometria/IMU tem que ser maior".
- **Análise**: giro integrado por janela: giroscópio × EKF × AMCL (−467 / −465 / −378° em 150 s) → AMCL errado, odometria certa; pose do AMCL cruzou a parede central aos 93–99 s. EKF da Pi já funde vyaw do IMU (`imu0_config`).
- **Mudanças**: `caramelo_localization/scripts/{loc_watchdog_core,localization_watchdog,loc_watchdog_offline}.py`, CMake/package.xml, `global_localization.launch.py` (`use_loc_watchdog`), `amcl.yaml` (modelo de volta a `likelihood_field`). Replay offline em 4 bags; fumaça no domínio 99. Relatório §14–§16. Nada commitado (o operador disse "dei commit", mas `git log` continua em 833c13d).
