# Decisões

Formato: data · decisão · motivo · consequências · origem · status.

## 2026-11-09 — Tamanho das tags AprilTag fica 0.041 m
- **Motivo**: 0.032 (medida do quadrado preto) deixou todos os picks 4,5 cm altos; 0.041 dá alturas fisicamente consistentes (mesa de 10 cm).
- **Consequências**: não mexer em `tags_36h11.yaml` antes de 18/11; calibração câmera–braço (T4) fica aberta.
- **Origem**: operador ("acredito que o tamanho de antes estava certo mesmo"); nota datada no YAML.
- **Status**: vigente.

## 2026-11-09 — PP usa fusão tag + buraco (peso 0,7 no buraco), offset aprendido, escadinha fora do slot
- **Motivo**: tag sozinha dava 3–5 cm de erro; o detector vê a cavidade real. Peso 0,7 porque tag e buraco vêm da mesma câmera e compartilham viés.
- **Consequências**: `slot_place_y_offset_m` −0,04 → −0,052 (mediana de 27 leituras AMT1) em place e amt1; garra abre antes de observar; plano da mesa publicado só após leitura de perto.
- **Origem**: operador ("seria bom se fosse uma fusão dos dois"; "faça todos os ajustes…"). Commit d1cc749; `docs/place_precisao_e_pilha.md` §3.1.
- **Status**: vigente. Substitui "slot só pela tag + escadinha".

## 2026-11-07 — DWB continua titular; MPPI disponível por `nav_profile:=mppi`
- **Motivo**: 11 dias para a competição; MPPI refeito mas sem validação no robô.
- **Consequências**: DWB com `ObstacleFootprint.scale` 1.0 (antes 0.02, repulsão nula). MPPI sem RotationShim por escolha do operador ("deixa ele trabalhar por completo").
- **Origem**: operador; `docs/tuning_dwb_2026-11-07.md`; commit 304fb09.
- **Status**: vigente.

## 2026-11-07 — Nunca `pkill -f` por nome de nó no PC do robô
- **Motivo**: um `pkill -f controller_server` para derrubar um nó de teste matou o controller real; lifecycle manager desativou todo o Nav2.
- **Consequências**: nós de teste em `-r __ns:=/teste` ou com outro nome; matar pelo PID guardado; preferir validar YAML por parse + log do launch real.
- **Origem**: incidente na sessão de 2026-11-07. Recuperação: subir o nó à mão com os params do launch → `ros2 lifecycle set /<nó> cleanup` → `ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"`.
- **Status**: vigente.

## 2026-11-06 — Experimento "lattice frente" revertido
- **Motivo**: piorou no robô real.
- **Consequências**: fica em `git stash` (stash@{0}) para consulta; não aplicar.
- **Origem**: operador. Status: vigente.

## 2026-11-03 a 05 — Pilha e slots da PP pela lógica de pilha (commit 26d665c, 2026-11-05)
- **Decisão**: `stack_verify_correct=false` (verificação pós-soltura só informativa); escadinha −12 mm na pilha; slots 101–107 tratados como "pilha de 1" com observação prévia de garra vazia.
- **Motivo**: operador em 03/11: "ele está empilhando mas depois verifica, mexe e derruba tudo" e "está fazendo uma escadinha em x"; em 04/11: "logo depois de finalizar ele tenta re-ajeitar e ferra tudo". Origem: sessão 99a9f780 (03–04/11); doc `docs/place_precisao_e_pilha.md`.
- **Status**: vigente; a escadinha no slot foi removida em 2026-11-09 (ver acima).

## 2026-10-26 — Docking: esquadro por AMCL desligado, esquadro pelo LiDAR antes do lateral
- **Resumo**: `amcl_square_enabled=false`, espera de base parada, `PoseProgressChecker` no Nav2, tour reprova pelos números.
- **Origem**: diagnóstico em `docs/diagnostico_docking_tour_2026-10-26.md`; commit 1ec95ca. Validação no robô: desconhecida (T6).
- **Status**: vigente.

## 2026-11-05 — Task planner: menos passagens por estação e re-tentativa no fim
- **Decisão**: o planejador agrupa o que fazer em cada WS para passar uma vez (operador: "na final de ontem passou na SH 3 vezes"), e o que não foi concluído é re-tentado numa passada antes do FINISH (`retry_unfinished_at_end`, default true, em `bt_yaml_executor.cpp`). Também pedido: blocos mais afastados entre si na mesa (folga em `mtc_place_action_node.cpp`, 2026-11-05).
- **Origem**: pedido do operador em 2026-11-05 (sessão 99a9f780); commit 26d665c ("arrumando task planner", que também carrega a pilha de 03–04/11).
- **Status**: vigente. Validação em missão real: não registrada.

## 2026-11-07 — Limites de velocidade ditados pelo operador
- **Decisão**: mínimo linear operacional 0,25 m/s, mínimo angular 0,35 rad/s, máximo linear 0,60 m/s; ré "não recomendada" (sem sensor atrás), mas permitida no MPPI (`vx_min −0,25`) e proibida no DWB. Recovery de ré cego substituído pelo escape inteligente (`caramelo_behaviors`).
- **Motivo**: piso físico do ESC 0,1215 m/s / 0,315 rad/s por roda; o robô "trava ou dá ré" nas zonas infladas e bate no que não vê.
- **Origem**: operador, 2026-11-07 (sessão 99a9f780); `controller_server.yaml`, `velocity_smoother.yaml` (comentários datados). Nota: DWB `min_speed_xy` ficou 0,131 (não 0,25) porque 0,25 impedia fechar o goal.
- **Status**: vigente, sem validação no robô.

## 2026-10-31 — AMCL com 600–1000 partículas (substitui 350–600 de 27/10)
- **Decisão anterior (27/10)**: operador pediu "min 350, max 600" após discutir que havia poucas partículas.
- **Decisão atual**: `min_particles 600 / max_particles 1000` (`src/caramelo_localization/config/amcl.yaml`, commit e5fa745 de 31/10, dia do remapeamento da arena).
- **Motivo da mudança**: não registrado nas transcrições (o turno de 31/10 trata do remapeamento). Hipótese: arena maior.
- **Status**: vigente; a de 27/10 está substituída.

## 2026-10-27 — Câmera D455 entra no docking com peso maior que o LiDAR; braço em `home` antes de dockar; shelf de cima mais lenta
- **Pedido do operador** (27/10): "quando chega no pré-docking ativa a câmera para ver a mesa e alinhar no centro sem bater, a câmera tem peso maior por ser mais precisa"; "as mesas nunca terão a mesma largura, a visão tem que reconhecer padrões, não calibrar por distância fixa"; "paredes sempre brancas"; diminuir a velocidade só nas poses do topo da shelf (braço esticado treme).
- **Implementação**: commit 0e76eee (31/10): `camera_enabled: True`, `camera_weight 0.8`, giro em pulsos medidos pelo LiDAR, braço no `home` antes do dock; commit 1b7b64c (31/10): espera de assentamento (`arm_settle.hpp`), shelf de cima mais lenta, `allowed_start_tolerance 0.03`.
- **Status**: vigente; resultado no robô não registrado (T6).

## 2026-10-21 — Regras físicas da PP e da pilha informadas pelo operador
- **Fatos** (operador, sessão c4e8f5da, 21/10): ids 101–107 e 200 são adesivos colados na mesa, não cubos; 101–107 ficam nos espaços da precision table, 200 marca a pilha; cavidade da PP tem 3–6 mm de profundidade (nunca confirmada) e ~2 mm de folga por lado; o cubo não precisa ficar com a tag para cima; tags dos adesivos têm o mesmo tamanho das dos cubos; pilha deve chegar a 4 cubos. Em 04/11: "o centro da tag fica uns 4 cm acima do centro do espaço".
- **Consequência**: todo o place em slot/pilha usa essas tags como referência; `slot_cavity_depth_m 0.005`; `slot_place_y_offset_m` começou em −0,04 e virou −0,052 em 09/11.
- **Status**: vigente.

## 2026-10-19 — Place em superfície livre pela câmera do punho (vindo do PC de desenvolvimento, commit "vision_place")
- **Pedido**: juntar o trabalho do outro PC (place pela visão, rulebook) com as correções locais de tags duplicadas por memória antiga.
- **Status**: implementado (`surface_place_planner`, `workstation_surface_node.py`), doc `docs/place_superficie_livre.md`. Um primeiro teste em 21/10 (run BMT) é citado na própria doc, mas o cabeçalho diz "NÃO testado" (D3/D4 em `estado.md`). Validação formal: desconhecida.

## 2026-08-03 — Deadband do `velocity_smoother` pequeno; piso do ESC só no driver, por roda
- **Motivo**: subir o deadband para o piso (0,1215/0,315) quebrava o BackUp (0,13), travava a RotationShim (rampa em passos de 0,10 nunca cruzava 0,315) e zerava diagonais válidas. O piso físico é por roda e é o driver que o aplica.
- **Origem**: comentário datado em `src/caramelo_navigation/config/velocity_smoother.yaml`. **Status**: vigente.

## 2026-09-14 e 2026-11-07 — Tetos de velocidade: x 0,40 → 0,60, y fica 0,40, wz fica 0,50
- **Motivo**: pedido do operador; y fica em 0,40 porque a lateral é a direção que o LiDAR frontal menos vê e a que satura o ESC na diagonal. Smoother é o último clamp e tem que acompanhar o controller.
- **Origem**: comentários em `velocity_smoother.yaml` e `controller_server.yaml`. **Status**: vigente; sem validação no robô depois de 07/11.

## 2026-11-06/07 — `cost_penalty` 4,0 testado ao vivo, volta a 3,0
- **Contexto**: "passo 1" proposto em 06/11 para afastar o caminho das mesas (`ros2 param set`), após o revert do experimento lattice frente.
- **Resultado**: em 07/11 o operador informou ter voltado `cost_penalty` 3,0 e costmap local 5×5 m. Efeito do 4,0 não registrado.
- **Status**: 3,0 vigente.

## 2026-10-27 — Nunca fechar o giro da base pela odometria
- **Motivo**: no docking, o giro fechado pela odometria subestimou o giro real em 30–70% (mecanum), gerando vaivém de ±11°. Giro passou a ser cortado pelo LiDAR e em pulsos medidos.
- **Origem**: memória pessoal (docking 27/10), commit 0e76eee. **Status**: vigente no `dock_align_node`; o Nav2 continua usando `/odom` (EKF) normalmente.

## 2026-11-11 — Caranguejo: atacar no planner (lattice "nariz" + analítica 0,8 + rotation 3), não no DWB
- **Decisão técnica proposta (aguarda teste no robô)**: novo conjunto de primitivas sem ré, com lateral ×2 e diagonal ×1,5 no `trajectory_length` declarado, `analytic_expansion_max_length` 0,8 e `rotation_penalty` 3,0, como opção `lattice_set:=nariz` (default segue `omni`).
- **Motivo**: no Smac jazzy frente/lado/ré custam igual por metro e a reta analítica interpola o yaw; o DWB não tem crítico que vire o nariz; o shim só gira acima de 45° (= diagonal do lattice). Bancada: 7% → 90% do plano para a frente, +0,6% de comprimento, 0 falhas.
- **Não fazer**: `rotation_penalty` alto com o lattice atual (87% lateral na bancada); `PreferForwardCritic` no DWB; `use_path_orientations` com Savitzky-Golay na BT.
- **Origem**: `docs/plano_navegacao_2026-11-11.md` §3–4; bateria 2 em `/tmp/bench_lattice/bateria2_*`. **Status**: proposta.

## 2026-11-11 — Laço do controller: cortar carga do DWB antes de qualquer outro tuning
- **Proposta**: `publish_evaluation`/`publish_trajectories` false; `vy_samples` 8 → 4; perfil de energia em desempenho; `failure_tolerance` 0,3 → 1,0 s. Meta ≥ 13 Hz (nível de antes de 07/11).
- **Motivo**: mediana 12,7 Hz → 7,0 Hz quando 304fb09 dobrou as poses por ciclo; o DWB copia 1588 trajetórias por ciclo sem assinante. **Status**: proposta.

## 2026-11-11 (noite) — Rodada 1 pós-bags: `rotate_to_heading_once: true` no RotationShim
- **Decisão**: o shim gira só no início da perna e no yaw final; no meio do path o rumo é do DWB. Aplicado em `controller_server.yaml` (DWBHolonomic), não commitado, aguarda teste.
- **Motivo**: nos bags de 11/11, 55–74 % do tempo de navegação foi giro parado a 0,35 rad/s, reengajado a cada replan (17/112 planos na perna abortada) e, em paths < 0,5 m, alternando ±170° entre o último ponto do path e o yaw do goal (SH1→WS2: 76 s para 0,22 m; volta de 350° na staging da WS2). Risco aceito: curvas fechadas no meio do path viram arco/caranguejo.
- **Origem**: `docs/relatorio_testes_2026-11-11.md` §A1/§3. **Status**: aplicada, aguarda bag.

## 2026-11-11 (noite) — Rodada 2 preparada: replanejar só se precisar (`bt_replan:=invalid`)
- **Decisão**: BT alternativa `caramelo_lattice_dwb_replan_if_invalid.xml` (PathExpiringTimer 15 s + GlobalUpdatedGoal + IsPathValid, padrão Jazzy), selecionável por `bt_replan` nos 3 launches; default continua `rate` (1 Hz).
- **Motivo**: cada `ClearEntireCostmap` global devolvia a rota bloqueada e o shim girava 180° ida e volta; "Start occupied" a cada segundo com o robô parado na staging. `is_path_valid` em modo footprint só invalida com célula LETAL.
- **Não fazer junto com a rodada 1** (uma mudança por teste). **Status**: pronta, desligada.

## 2026-11-11 (noite) — Trancos do ESC: registrar, não mexer
- **Fato**: rodas a 8–16 rad/s com referência zero/constante (encoder confirma), 24 eventos em 2 bags; um recuo de 0,3 m sem comando após o undock da WS5. Abaixo de `/mecanum_controller/reference`.
- **Decisão**: continua a regra "ESC/driver/motores intocáveis até 18/11"; o relatório vira ticket para a equipe de hardware (T17). No software do PC, só evitar comandos mistos pequenos onde possível (observação, não ação).

## 2026-11-11 (23h) — Rodada 2 pós-bag 3: quatro mudanças reversíveis, uma só rodada
- **Decisão**: aplicar juntas (operador pediu "faz funcionar" e o bag completo): (a) `ObstacleFootprint.scale` 1.0 → 0.02 (valor até 07/11); (b) perna curta: `GoToWS` manda BT `caramelo_lattice_dwb_curta.xml` (checker xy 0,30) quando a staging está a < 0,30 m; (c) nó `speed_limit_by_scan` → `/speed_limit` 70 % abaixo de 0,70 m (nunca < 70 %: 0,50×0,7 = `min_speed_theta`); (d) bag `--all-topics` + regex de exclusão.
- **Motivo**: bag 3 mostrou que o shim não estava nas pernas ruins; quem mandava era o DWB (corredor e caranguejo até encostar na WS6) e a perna curta. O pedido de "mais devagar perto de objeto" tem canal nativo no Nav2 (`speed_limit_topic`).
- **Não feito**: tirar o shim (o giro inicial/final iria para o DWB, que gira pior); `bt_replan:=invalid` segue desligado; nada no ESC.
- **Origem**: `docs/relatorio_testes_2026-11-11.md` §6–§9. **Status**: aplicada, aguarda bag 4.

## 2026-11-12 (01h) — Rodada 3: AMCL robusto a obstáculo não mapeado + não pular estação na 1ª falha
- **Decisão**: (a) `amcl.yaml`: `likelihood_field_prob` + `do_beamskip true`, `z_hit/z_rand` 0,5/0,5, `recovery_alpha` 0/0 (todos defaults do Nav2 exceto o beamskip); (b) `escape_free_space` ignora o costmap global quando ele diz ≥ 20 letais sob o robô e o local diz 0; (c) `GoToWS` re-tenta a staging 2× com 4 s e `/request_nomotion_update` antes de pular a estação.
- **Motivo**: bags 21h08/21h15 — a caixa dinâmica fez o AMCL colapsar numa pose 0,5–0,8 m deslocada durante o giro na staging da WS5; com a pose dentro da mesa nada mais funciona e a missão pula tudo em 90 s. O modelo afiado (0,8/0,1) e a injeção de partículas são o que mais favorece o encaixe falso; o beamskip é o mecanismo do AMCL para feixes que o mapa não explica.
- **Não feito**: mexer em `alpha1..5`, `update_min_a`, `laser_likelihood_max_dist`; relocalização global automática (arriscado em arena simétrica).
- **Origem**: `docs/relatorio_testes_2026-11-11.md` §10–§13. **Status**: aplicada, aguarda teste com a caixa.

## 2026-11-11 (22h40) — Rodada 4: a odometria manda em janelas curtas (vigia de localização)
- **Decisão**: novo nó `localization_watchdog` (ligado por default): se a pose do AMCL se afastar de (última aceita ⊕ odometria) mais que 0,10 m + 8 %/m ou 8° + 8 %/rad em 2 atualizações seguidas, reinicia o AMCL por `/initialpose` na pose da odometria. `/initialpose` do RViz é aceito como nova âncora. AMCL volta a `likelihood_field` (sem beamskip); z 0,5/0,5 e recovery 0 ficam.
- **Motivo**: bag 22h03: giroscópio e EKF concordam a 2° em 150 s; AMCL perdeu 89° e atravessou a parede central com a caixa à vista. Ajustar o modelo de sensor não bastou (prob+beamskip piorou). O operador pediu explicitamente mais confiança na odometria/IMU.
- **Limite**: não cobre sequestro nem patinagem longa; aí é "2D Pose Estimate" no RViz.
- **Origem**: `docs/relatorio_testes_2026-11-11.md` §14–§16; `loc_watchdog_offline` sobre os bags. **Status**: aplicada, aguarda teste.

## 2026-11-13 — Corrida gravada inteira: vídeo da janela do RViz do robô + bag + logs + relatório
- **Decisão**: `run_mission`, `mission_server` e o CLI `gravar_run` gravam `runs/run_<stamp>/` (`caramelo_run_common.py`). Vídeo = a janela do RViz que o `robot_manipulation.launch.py` abre (`use_rviz:=true`), achada pelo título (`global_localization.rviz`) e gravada por id de janela com ffmpeg x11grab + VAAPI; o gravador nunca abre nem fecha RViz. Sincronização por relógio de parede: hora local carimbada no quadro + epoch do 1º quadro no `metadata.json`; bag e rosout no relógio do PC.
- **Motivo**: o operador corrigiu no meio do trabalho ("não o rviz da bag, e sim a tela do rviz que inicia com o robot_manipulation"); a primeira versão abria um RViz próprio de diagnóstico, descartada. Gravar a janela do operador mostra o que ele viu e não duplica a carga de render.
- **Também**: cores fixas no `global_localization.rviz` (azul `/plan`, verde `/local_plan`, amarelo trilha, azul claro plano anterior, laranja `/expansions`, vermelho eventos; linhas sem setas); nó `run_diag_viz` só publica o que o Nav2 não publica (`/diag/*`). Nada de `/marker`, `/evaluation` ou `/trajectories`, que pesam no laço de 20 Hz. Nenhum parâmetro de planner/controller mudou.
- **Origem**: pedido do operador de 13/11 ("diagnóstico visual da navegação"); testes no domínio 99. **Status**: vigente, aguarda 1ª corrida real.

## Regras de trabalho (vigentes, do operador)
- Commits só quando pedidos; push manual.
- Uma mudança por vez; "falhar em melhorar nunca pode piorar".
- Não rodar o robô por conta própria.
- Comentários de código em pt-BR ASCII, datados, com o porquê.
- Valores de calibração só no YAML/launch que os declara, com data e motivo (README §8).
- Não rodar o robô por conta própria; o operador avisa quando o robô está ligado ("vc tem acesso ao robo completo", 26/10) e isso vale só para aquele momento.
