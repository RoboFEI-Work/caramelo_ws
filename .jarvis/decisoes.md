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
- **Origem**: pedido do operador de 13/11 ("diagnóstico visual da navegação"); testes no domínio 99. **Status**: substituída em 13/11 16h50 (abaixo): o vídeo deixou de ser a janela do operador.

## 2026-11-13 (16h50) — Tela do operador limpa; vídeos de RViz escondidos por corrida
- **Decisão**: o RViz do `robot_manipulation` (`global_localization.rviz`) mostra só mapa, robô, LiDAR, plano global e footprint (o resto fica no arquivo, desligado). A gravação sobe um X virtual (Xvfb) com cinco RViz escondidos (`caramelo_navigation/rviz/diag_{geral,colisao,controlador,global,custos}.rviz`: colisão e controlador seguem o robô; os outros enquadram o mapa) e grava um vídeo por vista em `runs/<run>/videos/`. `ros2 param dump` passou para depois da corrida.
- **Motivo**: operador: "a tela tem que ter poucas informações, senão não vê nada"; quer várias análises separadas, escondidas, gravadas na pasta da corrida. O gerenciador de janelas (mutter) não deixa janela fora da tela, então só um X virtual esconde. O dump no início coincidiu com o laço do controller a 6–8 Hz na perna 1 das duas corridas de 13/11 (12/11: 17 Hz).
- **Custo/limite**: Xvfb renderiza por software (llvmpipe, 1 thread, nice 15, 10 fps); `metadata.json` grava `video.cpu_nucleos` para decidir quantas vistas cabem (`CARAMELO_VIDEO_VISTAS`). Requer `sudo apt install xvfb`; sem ele a corrida grava o resto e diz por quê.
- **Status**: implementada e testada sem Xvfb (vistas renderizadas na tela real no domínio 99); falta a 1ª corrida com Xvfb instalado.

## Regras de trabalho (vigentes, do operador)
- Commits só quando pedidos; push manual.
- Uma mudança por vez; "falhar em melhorar nunca pode piorar".
- Não rodar o robô por conta própria.
- Comentários de código em pt-BR ASCII, datados, com o porquê.
- Valores de calibração só no YAML/launch que os declara, com data e motivo (README §8).
- Não rodar o robô por conta própria; o operador avisa quando o robô está ligado ("vc tem acesso ao robo completo", 26/10) e isso vale só para aquele momento.

## 2026-11-13 — Plano do lattice vai cru ao controlador; shim usa o yaw do plano
- Decisao: BTs do DWB sem SmoothPath + RotationShim use_path_orientations true, com o shim corrigido no workspace (backport de frame; upstream 1.3.14/jazzy HEAD de 03/11 ainda nao corrige).
- Porque: operador pediu "do jeito certo", nao remendo; smoothers do 1.3.13 destroem o yaw planejado; shim do apt compara frames diferentes.
- Memoria de obstaculo: falha do planner limpa so' +-0,5 m em volta do robo no global; total so' como ultimo recurso.
- Guarda no launch impede o true com o shim do apt.

## 2026-11-13 (noite) — DWB: corte do plano 1,0 m; sem re' no DWB; yaw do plano so' com lattice nariz
- forward_prune_distance = prune_distance = 1.0 (DWB). Porque: DWB 1.3.13 corta por distancia em linha reta; em rota em U o goal local caia na perna de volta (atras do robo). Medido no bag 18h57 e reproduzido em replay e simulacao. Regra: o corte tem que ser menor que a distancia entre as pernas do U.
- Re' do DWB desligada (min_vel_x 0.0): o diagnostico "beco sem re'" estava errado; sem LiDAR atras, re' longa e' cega. Recoveries (escape/BackUp) continuam.
- use_path_orientations true so' com shim corrigido E lattice nariz (trava no launch).

## 2026-11-13 (noite) — AMCL confia na odometria (alphas baixos)
- alpha1/2 0.005 (rotacao: giroscopio), alpha3/5 0.03 (translacao: mecanum erra ate' ~13%), alpha4 0.005 (translacao por giro).
- Porque: com 0.2 a nuvem tinha liberdade para girar/andar atras de encaixe falso em obstaculo nao mapeado; operador: "a odometria tem boa confianca, o robo nunca e' teleportado". Validado por replay do bag ruim e do bom.
- Rejeitado: mais particulas (piora), mapa de voxels 3D (nao ataca a localizacao 2D), likelihood_field_prob+beamskip (ja' piorou em 11/11).

## 2026-11-14 — Precision Table: arquitetura aprovada pelo operador
- Aprovado: olhar-mover-olhar com o cubo na garra, medindo aresta do cubo x borda do furo na mesma imagem (tags vizinhas de apoio), referencia ensinada, passos pequenos, correcao tambem na descida, verificacao que manda repetir.
- Criterio de sucesso (operador): cubo PERFEITAMENTE encaixado no slot, sem sobrepor nada e sem jogo. Em cima do tile/da tag nao vale.
- Protocolo de movimento: para mexer o robo (navegar, localizar, pegar cubo, mover o braco) perguntar pela caixa de resposta (AskUserQuestion) e so' seguir com a aprovacao; enquanto nao aprovar, continuar com programacao e analise offline.
- Pendente do operador: distancia centro a centro entre tags vizinhas da PP (110 ou 140 mm).

## 2026-11-16 — Garra larga (gripper_full_open) so' em quatro casos
- Decisao do operador: gripper_open em tudo; gripper_full_open so' no HOME, ao empilhar, ao por na PP e ao olhar o container da mesa (ct) de garra vazia.
- No place a escolha sai do goal: stack_on (pilha e PP) ou container_color. A soltura dentro do ct e a devolucao ao pote de bordo sao em gripper_open.
- No pick a garra larga em pegar_obj (pedido de 29/08) fica desligada por padrao; religa com pick_gripper_pose_at_pegar_obj:=gripper_full_open.

## 2026-11-16 — A missao nao cai por falha de pick/place: replaneja e tenta depois
- Decisao do operador: "ele nao pode falhar a missao"; em falha, ajustar o plano e tentar o place mais tarde.
- Regra implementada no bt_yaml_executor: objeto que falhou fica a bordo e fora da rodada seguinte; o resto e' replanejado contando o pote ocupado; ele volta na rodada posterior; 2 quedas no mesmo objeto = desiste so' dele. FINISH sempre (menos Ctrl-C).
- Desligar tudo (arvore unica como antes de 05/11): `retry_unfinished_at_end:=false`. Teto de rodadas: `replan_max_rounds` (6).

## 2026-11-16 — PP: alinhamento por visao direta, com referencia ensinada (pp_servo_node)
- Operador pediu a PP "de modo interativo" no estilo do codigo de referencia galera33/manip. Implementado como a arquitetura olhar-mover-olhar ja' aprovada (14/11): erro em PIXELS contra a referencia ensinada com o cubo no furo; ganho medido por balanco; garra sempre na vertical.
- Entra no place por parametro (`pp_align_mode`, off por padrao) e so' no slot da PP. O no' Python mexe o braco direto pelo arm_controller, em passos de milimetros.
- Nada de trocar tag_size, TF da camera ou offsets antes da competicao: a referencia ensinada dispensa.

## 2026-11-16 (18h55) — PP: malha com ganho por MODELO e movimento unico (substitui balanco + passinhos)
- Motivo (medido no robo, places das 18h26 e 18h30): balanco de +-3 mm some na folga da j1 (~1 grau) e na flecha de j2/j3 -> ganho lixo -> a malha piorou os dois places. Cada trajetoria curta da' um tranco no braco (arm_controller recomeca do medido).
- Decisao: ganho por modelo (tag como regua + angulo do punho), constantes ajustadas a movimentos reais gravados; erro inteiro num movimento lento, depois retoques (max 5); ganho <= 1 para chegar por um lado so' (nao inverter a j1); altura fechada pelo lado da tag; se a vista piorar, volta ao inicio e o place segue como antes.
- Alvo continua sendo a vista da tag na pre-pose de um place bom (por slot). Pedido do operador de alinhar "com o buraco" e de empurrar o cubo se cair fora: ainda nao atendidos (so' depois de ver a malha nova no robo).
- Nao mexer antes de 20/11: parametros do arm_controller (interpolate_from_desired_state) e ganhos dos Dynamixel, embora sejam a causa de fundo do tranco e da flecha.

## 2026-11-16 (19h10) — PP: malha de visao (pp_servo_node) descartada
- Decisao do operador, depois de um dia de testes sem nenhum encaixe atribuivel a' malha. As duas decisoes de 16/11 acima sobre a malha (referencia ensinada; ganho por modelo) ficam SEM EFEITO: o codigo foi removido.
- Nao reintroduzir malha/alinhamento por visao na PP sem pedido explicito do operador. Ultima ideia dele, em aberto: empurrar o cubo com a garra fechada quando cair fora do furo.

## 2026-11-16 (19h20) — branch `mudanca_pp`: mesa de precisao removida do codigo
- Decisao do operador: remover TUDO da PP nessa branch. Escolhas dele: missao com entrega na PP pula o objeto (planner, com aviso) e a missao segue; a AMT1 sai junto.
- Consequencia: nessa branch o robo nao pontua em precision placement nem na AMT; em troca nao gasta tempo nem derruba missao por causa da PP.
- Ponto de entrada para refazer a PP do zero, se o operador pedir: o planner (isPpStation em buildTransfers) e o `stack_on` do place; o executor recusa `at_tag`.

## 2026-11-16 (21h30) — PP nova: medir na imagem pela tag do slot, mover so' pelo place
- Decisoes do operador: verificar o furo e o centro; aproximar aos poucos recalculando; cubo em cima do furo durante a descida; punho com a camera vendo o furo; furo sumiu -> desce reto.
- Decisoes minhas (dizer se mudarem): (1) regua = tag do slot (homografia), sem TF/hand-eye; (2) ponto de controle = FUNDO DO CUBO no frame da camera, calibrado por places bons; (3) quando o cubo tapa o furo, usa tag + vetor tag->furo medido antes, em vez de descer cego de 5 cm; (4) o no' de visao nunca move o braco; (5) alvo novo = TCP comandado + erro medido; (6) 3 paradas de centimetros, no maximo 2 correcoes finas.

## 2026-11-16 (noite) — AMT por ids + faixa lateral que acompanha o curso
- AMT: o servidor so' guarda IDS de tag (nunca coordenada), entao a base pode andar de lado no meio da rotina; quem move o braco para olhar e' o executor (GoToNamedPose), nao o servidor. Alvo = target_order do arquivo, senao ordem crescente.
- Guarda lateral do dock_align_node: obstaculo so' conta se estiver dentro do que o robo vai alcancar NESTE curso (corredor + falta + 0,11 m). Motivo e numeros no historico. Volta ao antigo com `side_clear_reach_extra:=0`.

## 2026-11-17 (17h) — AMT so' diz que terminou com a mesa VISTA em ordem; base alinhada no place
- Pedidos do operador: conferencia no fim (olhar a mesa, arrumar o que faltar); no place a base anda de lado para ficar alinhada com o slot ("isso precisa funcionar"); sem vai e volta no pick.
- Decisoes minhas (dizer se mudarem): (1) `success` do /amt1_sort = mesa vista em ordem SEM nenhuma operacao na chamada; quem repete e' o executor (`amt1_rounds` 4), e esgotadas as rodadas a missao segue para o FINISH; (2) sem tag de slot para ancorar a fila o no' nao mexe em bloco (antes chutava "comeca no 101"); (3) alinhamento do place pelo protocolo de alcance que ja' existia (unreachable + passo), limite 8 cm, de garra vazia; (4) a base volta ao centro uma vez no fim de cada rodada, porque as olhadas so' cobrem a fileira a partir do centro; (5) `nudge_coast_est` 0,06 e nao 0,085 (o medido), por falta de medida com corte antes dos 6 cm.
- Nao mexi: pose `pegar_obj` (j4 2,199 contra o limite de 1,913; T9) e a causa fisica da queda do adaptador de rede (T35).

## 2026-11-19 — Cor no `type` do objeto nao manda mais para o container
- Pedido do operador: "a cor do lado [da tag] e' apenas a cor da tag", nao e' para colocar no container daquela cor.
- Removida do task_planner.cpp (buildTransfers) a regra do formato "novo 25/08" sem `containers:`: objeto `ATTC_RED`/`ATTC-blue` indo para WS ganhava `container_color`. Agora so' vai para container o que o yaml manda explicitamente (`finish_state.<WS>.containers.<COR>` no 2026; constraint "inside" no antigo). Status: vigente.

## 2026-11-19 — Shelf com bloco: pegar_obj_sh -> pre_container num movimento so'
- Pedido do operador: "quando ele pega o objeto na shelf, ele ta indo pra uma pose muito parecida de pegar_obj; tem que sair direto de pegar_obj_sh e ir pra pre_container".
- Causa (run_20261118_115655): depois de pegar_obj_sh o transporte fazia `pre_container [j1 parado]` (pre_container com a j1 ainda na estante: 1,57 / 0 / 1,08 / 1,82, parecida com pegar_obj) e so' depois `[j1 por ultimo]`.
- Decisao: na shelf o recuo volta SEMPRE a pegar_obj_sh e dali vai ao pre_container num movimento unico (`shelf_pre_container_direto` true; false = volta as duas etapas, por `ros2 param set`, sem relancar). Mesa inalterada.
- **Substitui** a "SAIDA DIRETA" de 2026-11-18 (parar na fase 1 e pular pegar_obj_sh): interpretacao errada do pedido; o parametro `shelf_direto_ao_pre_container` foi removido. Status: vigente.

## 2026-11-19 — Container: alvo mais perto do centro (offsets 0,03 -> 0,022)
- Pedido do operador: "as vezes ele acha o ponto nas laterais do container e acaba derrubando/mexendo ele; tem como pegar uma area um pouco menor?".
- Fato (run_20261118_115655): alvo = centro detectado +-0,030 no lado longo; garra a 15-30 graus. Container 0,170 x 0,105, cubo 0,042 -> ~3,4 cm do cubo a' parede. tasks_geradas4: no maximo 2 blocos por container.
- Decisao: `container_slot_offsets_long_m` [0.022, -0.022, 0.0] (cubos quase encostados, +0,8 cm de folga por lado) e leitura do parametro a cada place (ajuste sem relancar). Nao mexi no detector (centro / wall_bias / partial_extend): sem bag desses casos nao sei se o erro e' do centro. Status: vigente.

## 2026-11-19 — Pilha: conferir a base com olhadas de perto repetidas
- Pedido do operador: "para empilhar em mesa de 5 ele coloca muito errado, faca com que ele verifique melhor o ponto antes de empilhar".
- Fato: nao ha log de pilha na Mesa5 neste PC (runs/ e ~/.ros/log so' tem pilha na Mesa15, WS_6, 18/11). Nessas, longe x perto diferiu 2-5 cm (dz +31 a +46 mm) e havia UMA leitura de perto, de um pre-ponto mirado pela leitura de longe.
- Decisao: ate' 3 olhadas de perto, re-mirando pela leitura nova, ate' duas seguidas concordarem (xy 8 mm, z 12 mm); senao usa a ultima com aviso. Vale para toda pilha (nao so' Mesa5); slot da PP fica com 1 olhada. Params lidos a cada place. Status: vigente.

## 2026-11-19 — Container: 1o bloco no centro, 2o em cima do 1o
- Pedido do operador: "para o ct, faca ele colocar o segundo bloco aonde colocou o primeiro, mas um pouco mais alto; e para o primeiro bloco sempre de prioridade para o ponto mais central possivel".
- Decisao: `container_empilha_no_primeiro` true — sem bloco solto, ordem por |deslocamento| (centro 1o, laterais so' sem IK); com bloco solto, candidato mais perto dele (na deteccao atual) e alvo +`container_empilha_extra_z_m` 0,03 por bloco solto. "Um pouco mais alto" = 3 cm (escolha minha: cubo 4,2 em container de 7 -> topo do 1o 2,8 cm abaixo da borda). Substitui na pratica a regra de 25/08 (o mais longe dos soltos); os offsets 0,022 de T41 viram so' fallback de IK. Status: vigente.

## 2026-11-19 — Pilha: sempre no mesmo ponto, cada vez mais alto
- Pedido do operador: "faca o empilhar colocar sempre no mesmo ponto, porem cada vez mais alto". (O operador vinha ajustando a escadinha `stack_place_y_offset_m` a mao: -0,02 -> 0,01 -> 0,0, commits empilhar/empilha/empilhaa.)
- Decisao: `stack_mesmo_ponto` true — cubo sobre cubo usa xy do 1o cubo da cadeia (findStackRoot na memoria 'stack:') e z = topo dele + cube_height x (niveis-1) + stack_place_z_offset; a leitura da tag so' confere (> `stack_mesmo_ponto_max_desvio_m` 0,05 -> vale a leitura). 1o cubo (sobre tag_200) nao muda. Status: vigente.

## 2026-11-19 — Planner: na mesma mesa, pick antes do place
- Pedido do operador: "caso ele chegue em uma mesa onde ele tenha que pegar um bloco e por um bloco, ele priorize pegar o bloco primeiro".
- Decisao: o plano (quais picks/places em cada visita) nao muda; so' a ordem de emissao (`orderVisitSteps` em task_plan_search.cpp): pega tudo o que cabe e e' permitido, solta UM quando lotado/travado e volta aos picks. Usado no emitActions (task_planner) e na re-tentativa do bt_yaml_executor. Limite fisico: com 3 a bordo o 1o passo e' place. `picks_first=false` volta a ordem antiga (so' no codigo; nao ha flag de linha de comando). Status: vigente.
- Complemento (mesmo dia): "para a shelf nao precisa ter essa modificacao" -> visitas a ws "SH*" ficam na ordem antiga (places, depois picks). Teste VisitOrder.ShelfKeepsPlacesFirst.
