# Docking — Alinhamento Fino Holonômico (ADR-05, opção c)

## Por que existe

O `opennav_docking` (Jazzy) traz o framework de docking (database de docks, staging,
retries, action, undock), mas o **controlador dele é unicycle** (`controller.hpp`:
`k_phi, k_delta, v_linear, v_angular` — só `linear.x` + `angular.z`, **sem `vy`**, e
não é plugin). Ou seja: com o Docking Server puro, o Caramelo dockaria manobrando
como carrinho, **sem usar as rodas mecanum**.

Decisão do dono (ADR-05 opção c): **manter o `opennav_docking` pelo framework** e
adicionar um **nó de alinhamento fino holonômico** para o trecho final. Isso NÃO
duplica o Nav2 (não existe controlador holonômico de docking lá).

## Arquitetura em dois estágios

```
1) opennav_docking  -> leva o robô até o STAGING/APPROACH do dock (unicycle, ok)
2) dock_align_node  -> alinhamento fino HOLONÔMICO (vx, vy, wz) até a tolerância
```

O `dock_align_node` (`caramelo_navigation`, sobe junto do `docking_server.launch.py`)
expõe a action **`caramelo_msgs/action/AlignToDock`** (`align_to_dock`; clientes: a
folha `GoToWS` do `manip_bt` e o módulo Docking da GUI):
- Lê a pose alvo do dock em `maps/<mapa>/docking.yaml` (mesma base do Docking Server).
- (Opcional) refina a pose pela **face** da mesa no `/scan`.
- Servo-controle holonômico P (vx, vy, wz) → publica `TwistStamped` em
  `/mecanum_controller/reference` até `xy_tolerance`/`yaw_tolerance` ou timeout.

> **Orquestração (Fase 6):** por causa da armadilha do Nav2 (`DockRobot` não pode ser
> chamado de dentro de uma BT do Nav2), o encadeamento `NavigateToPose → DockRobot →
> AlignToDock` deve ficar numa árvore de autonomia SUPERIOR (ou script), não na BT de
> navegação.

## Limites honestos (precisam de robô/firmware)

1. **Y (lateral) não é observável na face reta** (problema da abertura, ADR-05). O nó
   corrige X e yaw pela face; **Y fica por conta da localização** (AMCL/EKF, ~2–5 cm).
   Fechar Y de verdade exige **detectar os cantos da mesa** (casar retângulo ~80×50 cm)
   — está marcado `AJUSTAR_NO_ROBO` em `_refine_target_from_scan()` e precisa de dados
   reais do LiDAR para implementar/tunar.
2. **Piso do ESC:** a base não anda abaixo de ~0,19 m/s nem gira abaixo de ~0,49 rad/s
   (firmware — ver `caramelo_hardware_ws/docs/esc_stm32_comportamento_e_riscos.md`).
   O nó respeita esse piso (`min_body_vel`/`min_body_omega`), mas isso **limita a
   precisão fina** e causa movimento em degrau perto do goal. A precisão milimétrica só
   vem depois de baixar `speed_min` no firmware (ADR-06).
3. **Mesas de 5 cm** ficam abaixo do plano do LiDAR → invisíveis; nelas o docking é só
   por localização (campo `visivel_lidar` no service_areas).
4. **Colisão vs. docking:** o nó publica direto em `/mecanum_controller/reference`
   (fora do collision_monitor) porque durante o docking o robô se aproxima da mesa de
   propósito. Cuidado ao testar — comece com rodas suspensas.

## Tuning quando o robô estiver disponível

- Ganhos `kp_linear`, `kp_theta`; limites `max_vel_*`; `min_body_vel/omega` (medir o
  piso real do ESC); `xy_tolerance`/`yaw_tolerance`.
- Implementar/validar a detecção de cantos para corrigir Y em `_refine_target_from_scan`.
- Gravar as poses reais dos docks (hoje são placeholders `[0,0,0]`) com as ferramentas
  existentes (`dock_pose_recorder_node`, `save_dock_pose`).

Texto original de 2026-07: o alinhamento foi validado no robô em 2026-07-21 (ver
`docs/historico/relatorio_sessoes_2026-07-14_a_21.md`); o refino pela face é
calibrado por `refine_desired_face_dist` (`docs/guia_missoes.md` §1.1).

## Ajuste lateral `nudge_base` (2026-08-28, fila de alcance)

O mesmo `dock_align_node` expõe uma segunda action, **`caramelo_msgs/action/NudgeBase`**
(nome relativo `nudge_base`, como `align_to_dock`): "anda `dy` metros de lado", pedida
pela árvore (`ReachQueue`/`NudgeBase` do `manip_bt`, via `bt_yaml_executor`) quando o
braço não alcança um alvo na mesa, e pelo `amt1_sort_action_node` (busca lateral e
ajustes da AMT1). A Raspberry não muda: usa o mesmo `/mecanum_controller/reference` e o `/odom`
do EKF.

### Contrato

| Campo | Sentido |
|---|---|
| goal `dy` (m) | + = **esquerda** (eixo +y de `base_footprint`); `|dy|` em `[nudge_min_travel, nudge_max_travel]` |
| goal `timeout` (s) | 0 ⇒ `nudge_default_timeout` |
| result `success` | `true` só se andou `≥ |dy| − nudge_accept_slack` no sentido pedido |
| result `travelled` (m) | curso REAL com sinal, medido por odometria (preenchido também nas falhas) |
| result `reason` | `""` no sucesso; senão uma das razões abaixo |
| feedback `travelled` | curso parcial (mesma medida) |

Razões (`reason`): `pedido_invalido`, `abaixo_do_passo_minimo`, `acima_do_curso_maximo`,
`sem_odometria`, `sem_scan`, `muito_perto_da_mesa`, `obstaculo_lateral`, `muro`, `timeout`,
`cancelado`, `ocupado`, `curso_incompleto`, `sem_progresso` (todas listadas no comentário
de `caramelo_msgs/action/NudgeBase.action`).

Frames: `manip_base_link` +X = direita do robô, +Y = frente; `base_footprint` +y =
esquerda. A base andar `dy` para a esquerda ⇒ o `x_manip` de um alvo fixo na mesa vira
`x + dy`.

### Guardas (na ordem; nenhuma delas move o robô)

0. `dy` ou `timeout` NaN/inf → `pedido_invalido` (revisão 28/08, achado 4: NaN passaria
   nas comparações de faixa — não é menor nem maior que nada — e o curso só acabaria por
   timeout).
1. `|dy| < nudge_min_travel` → `abaixo_do_passo_minimo`; `|dy| > nudge_max_travel` →
   `acima_do_curso_maximo` (a árvore quantiza em ±{0,10; 0,15; 0,20; 0,25}).
2. Odometria recebida há menos de `nudge_odom_max_age` (0,2 s) → senão `sem_odometria`.
3. Muro (`_front_wall_distance`, o mesmo do docking): sem medição (scan ausente/velho/sem
   TF, ou face fora do plano do LiDAR) → `sem_scan` quando `nudge_require_scan=true`
   (default). `front < hard_min_face_dist` → `muro`. **Face inclinada** (achado 1): num
   lateral ao longo de uma face não perpendicular o bico avança `|dy|·sin(face_yaw)`
   (0,25 m a 5° = 2,2 cm — e o pouso fica a 0,365-0,39 m com `hard_min` 0,35). A guarda
   usa o `face_yaw` do fit TLS leve de `_front_wall_distance` e exige
   `front − |dy|·sin(|face_yaw|) ≥ lateral_min_front` (0,36) → senão `muito_perto_da_mesa`
   (conservador: usa `|face_yaw|`, independe do sentido). Face **sem ângulo confiável**
   (fit reprovado) com `nudge_require_scan=true` → `sem_scan` (não dá para prever o avanço);
   com `false` assume face perpendicular e avisa.
4. Faixa lateral (`_side_clear`, `side_clear_half_width`) do lado do movimento livre até a
   face → senão `obstaculo_lateral`.
5. Fala **"Vou me ajustar para a esquerda/direita"** em `/manip/speech` e espera
   `nudge_speech_lead_s` (1 s) ANTES do primeiro comando (cancel respeitado na espera).

### Malha por odometria

- Curso contínuo no **piso do ESC** (`min_body_vel`, eixo y puro — o `assert` de eixo puro
  de `_publish_cmd` continua valendo) cortado quando o que falta já cabe no
  `center_coast_est` (o lateral não tem coast medido; 0,03 m) **mais o atraso da medida**
  (achado 5, mesma conta da reta do align): corta quando
  `|dy| − travelled·sinal ≤ coast + min_body_vel·(idade_odom + período)`; por isso
  `nudge_odom_max_age` caiu para 0,2 s (mais velha que isso a conta não vale — corta).
  Teto de tempo `(|dy| + 0,05)/v_partida + 1 s`, com `v_partida` = velocidade real de
  partida (`_start_speeds`, piso × (1 + `floor_start_margin`)), não o piso nu.
- Medida: `_lateral_displacement(x0, y0, yaw0, x, y) = −sin(yaw0)·(x−x0) + cos(yaw0)·(y−y0)`
  — projeção do delta de `/odom` no eixo +y INICIAL do robô (função pura, testada em
  `test/test_nudge_math.py` com yaw 0/±90/180 e movimento puro em x = 0), **× 
  `nudge_odom_lateral_gain`** (achado 2): o `/odom` do EKF integra só a velocidade das
  rodas (`odom0_config` funde vx, vy, vyaw) e **não vê o escorregamento lateral** (~18% no
  campo) — o bruto superestima o curso. O ganho (real/odom) vale para o corte, o aceite, o
  feedback e o total publicado; o log de fim mostra o bruto e o corrigido (`lateral
  +16,4 cm (odom bruto +20,0 cm x ganho 0.82; ...)`), além do longitudinal e do Δyaw para
  detectar deriva.
- A cada ciclo (não só a cada scan novo, diferente do `_cut_drive`): guardas de
  cancel/timeout/`hard_min` (`_check_guards`), muro sem medição ⇒ corta, odometria velha
  ⇒ corta, faixa lateral ocupada ⇒ `obstaculo_lateral`, e **front caindo mais que
  `nudge_front_drift_max`** (0,02 m) em relação ao início do curso ⇒ `muro` (achado 1b: a
  face inclinada empurra o bico para a mesa antes de o `hard_min` pegar).
- **Assentamento por odometria** (achado 3): num lateral puro o front não muda, então o
  `_settle` do docking (front estável em 3 scans) devolveria enquanto o ESC ainda retém a
  referência (~0,5 s) e o `travelled` sairia com a base em movimento. O nudge publica zero
  até o lateral ficar dentro de `nudge_settle_eps` (3 mm) **da amostra que abriu a janela**
  por `nudge_settle_min_s` (0,8 s ≥ retenção do ESC), teto `nudge_settle_max_s` (3 s);
  só então mede e decide. Comparar com a âncora da janela (e não só com a amostra anterior)
  evita que um rastejo de 2,4 mm/amostra a 50 Hz passe por parado. Depois de um abort
  (`muro`, `obstaculo_lateral`, cancel, timeout) espera parar do mesmo jeito, sem guardas
  (já dispararam; a referência é zero), para o total publicado ser a posição real.
- Resultado: `travelled·sinal ≥ |dy| − nudge_accept_slack` ⇒ sucesso; senão
  `curso_incompleto` (abort, `travelled` preenchido). Medidas de campo (21/08): o lateral
  escorrega ~18% (12,2 cm pedidos → 10 cm) e o passo mínimo é ~10 cm.

### Parâmetros `nudge_*`

| Parâmetro | Default | Uso |
|---|---|---|
| `nudge_max_travel` | 0,25 m | curso máximo por chamada |
| `nudge_min_travel` | 0,08 m | passo mínimo executável (a árvore pede ≥ 0,10) |
| `nudge_default_timeout` | 15 s | quando o goal traz `timeout: 0` |
| `nudge_accept_slack` | 0,03 m | folga do aceite |
| `nudge_odom_max_age` | 0,2 s | odometria mais velha = cega (era 0,5; a idade entra no corte) |
| `nudge_require_scan` | true | `false` SÓ para mesas invisíveis ao LiDAR (sem muro!); também dispensa o `face_yaw` |
| `nudge_front_drift_max` | 0,02 m | front caiu mais que isto durante o curso ⇒ `muro` (ruído do front ≈ `settle_stable_eps`; subir se disparar à toa) |
| `nudge_odom_lateral_gain` | 1,0 | ganho real/odom do lateral (**calibrar com trena**, esperado ~0,82 — ver abaixo) |
| `nudge_settle_eps` | 0,003 m | lateral "parado": dentro disto da âncora da janela |
| `nudge_settle_min_s` | 0,8 s | tempo parado exigido (≥ retenção do ESC ~0,5 s) |
| `nudge_settle_max_s` | 3,0 s | teto do assentamento (mede assim mesmo, com aviso) |
| `odom_topic` | `/odom` | odometria do EKF |
| `speech_enabled` | true | fala antes de mover |
| `nudge_speech_lead_s` | 1,0 s | espera após a fala, antes do 1º comando |

**Calibrar `nudge_odom_lateral_gain` com trena** (uma vez por piso/pneu): com o ganho em
1,0, pedir `dy: 0.20` (e −0,20) com o braço em `home`, medir o deslocamento real da base
e ler no log de fim o `odom bruto`; `ganho = real / odom bruto` (média dos dois sentidos;
esperado ~0,82 pelo escorregamento de ~18% medido em 21/08). Sem calibrar (1,0) o aceite
conta só com `nudge_accept_slack` e o `/manip/base_shift_total` fica ~18% acima do real —
o place desloca a memória de slots demais.

### `/manip/base_shift_total`

`std_msgs/Float64`, QoS **transient_local, depth 1, reliable** (latched), + = esquerda.
Total lateral acumulado desde o último `align_to_dock`; publicado **somente** pelo
`dock_align_node`: `0.0` na subida do nó e no início de cada `align_to_dock`; somado com o
`travelled` de cada nudge — **inclusive nos parciais** (`curso_incompleto`, `muro`,
`timeout`, `cancelado`), porque a base se moveu de fato. O place node aplica esse valor
como offset em x na memória de slots (leitura `x + offset`, gravação `x − offset`).

### Concorrência com `align_to_dock`

Um único lock de movimento: o `goal_callback` das duas actions rejeita quando outro goal
está em curso, e o `execute` ainda tenta `acquire(blocking=False)` — se perder, aborta com
`ocupado`. Nunca os dois publicam na mesma referência ao mesmo tempo. Um segundo
`dock_align_node` no ar "rouba" `nudge_base` do mesmo jeito que rouba `align_to_dock`.

### Segurança

- Publica **direto** em `/mecanum_controller/reference`, **fora do collision_monitor**
  (como o docking). O LiDAR é cego para o que está ao lado do chassi (x < 0,32 m) e abaixo
  do seu plano: a faixa lateral só cobre o que está à frente do bico. Por isso o robô
  **fala antes de mover** e o operador afasta objetos.
- Sem scan (ou face invisível) o robô **não anda** por padrão; `nudge_require_scan=false`
  remove o muro por completo — usar só com a mesa livre e alguém ao lado.
- Bancada (sem robô): rode o nó em `ROS_DOMAIN_ID=99` com `reference_topic` remapeado
  para um tópico dummy e sem `/scan`/`/odom`: `dy: 0.10` devolve `sem_odometria`; com um
  `/odom` falso, `sem_scan`; `dy: 0.05` → `abaixo_do_passo_minimo`; `dy: .nan` ou
  `timeout: .inf` → `pedido_invalido`. Nada se move.
- Protocolo no robô (plano §8.1): braço em `home`, `ros2 action send_goal /nudge_base
  caramelo_msgs/action/NudgeBase "{dy: 0.10}"`, medir com trena 0,10/0,15/0,20/0,25/−0,10,
  calibrar `nudge_odom_lateral_gain` (acima) e ajustar `nudge_accept_slack`/
  `center_coast_est`. Conferir no log `assentou=sim` (senão subir `nudge_settle_max_s`) e
  que `muro` por `nudge_front_drift_max` não dispara com a mesa perpendicular.

### Recuo antes do lateral (29/08)
Com o bico a menos de `lateral_min_front` (0,36 m; o docking para em 0,35–0,39), o `nudge_base`
não recusa mais de cara: recua em x puro (`_nudge_backoff`, mesmo movimento do undock, corte
descontando `nudge_backoff_coast` 0,06) até `lateral_min_front + nudge_backoff_margin` (0,03),
limitado a `nudge_backoff_max` (0,12), re-mede a face e segue com o lateral. Desligar:
`nudge_backoff_enabled:=false`. O recuo NÃO entra em `/manip/base_shift_total` (só o lateral).

### `nudge_base` não se movia (2026-10-19)

**Sintoma.** O robô fala "Vou me ajustar para a esquerda", o executor loga
`[NudgeBase WS_1] ajuste lateral +0.10 m (esquerda), timeout 20 s...` e, 5 s depois,
`ajuste de +0.10 m FALHOU: curso_incompleto (andou +0.000 m)`. Nada se move.

**Causa (confirmada numericamente).** O nudge partia do **repouso** comandando exatamente
`min_body_vel = 0.1215 m/s`. Na Raspberry o `mecanum_drive_controller` (`wheels_radius`
0,05; `sum_of_robot_center_projection_on_X_Y_axis` 0,3855) converte isso em roda =
`0.1215/0.05 = 2.4299999999999997 rad/s` — **um ULP abaixo** do piso do ESC
(`min_wheel_rad_per_sec = 2.43`). O driver (`maxon_motors_node.cpp`,
`velocity_to_pulse_width_us`) tem histerese: **ligar** exige `|cmd| ≥ piso`; só **desliga**
abaixo de `0.4·piso`. Logo as rodas nunca partiam, a odometria lia 0,000 m e o resultado era
`curso_incompleto`. O docking funcionava porque parte com `vx = kp·erro > piso` e, já em
movimento, a histerese mantém as rodas quando o comando cai para o piso. O giro tem o mesmo
problema (`0.315·0.3855/0.05 = 2.42865 < 2.43`); o `_retreat` já usava 0,13 m/s por isso.

**Correção (`dock_align_node.py`).**
- Parâmetro `floor_start_margin` (fração, default **0,07**) e helper puro
  `_floor_start_speed(piso, margem) = piso·(1+margem)`: **toda primitiva que parte do
  repouso** (`_nudge_drive`, `_cut_drive` nos 3 eixos — inclusive `min_body_omega` —,
  `_nudge_backoff` e `_retreat`, que mantém o mínimo de 0,13) comanda
  `0.1215·1.07 = 0.130 m/s` (2,60 rad/s na roda) / `0.315·1.07 = 0.337 rad/s`. Margem
  negativa, NaN ou inf vira 0 (WARN uma vez, em `_start_speeds`). O corte do nudge
  (`_nudge_cut_reached`, argumento `v_cmd`) desconta o lag com a mesma velocidade
  comandada, e o teto de tempo do curso usa essa velocidade de partida.
- Depois do `_nudge_backoff`, se a face não mudou mais de 1 cm, o nó loga WARN
  `recuo sem efeito (base nao respondeu?)` com o mesmo diagnóstico do watchdog abaixo.
- **Watchdog de progresso** em `_nudge_drive`: `nudge_no_progress_s` (1,0 s) e
  `nudge_no_progress_min_m` (0,01 m) — comandando há ≥ 1 s com `|travelled| < 1 cm`
  (`_nudge_no_progress`, pura) ⇒ zera a referência, loga o diagnóstico em WARN e aborta
  com a razão **nova `sem_progresso`** (as demais razões não mudaram de texto; o
  `amt1_sort_action_node` compara strings exatas e trata `sem_progresso` com
  `|travelled| ≤ 2 cm` como falha, não como recusa nem parcial).
- Os três parâmetros estão declarados em `docking_server.launch.py`.

**Como diagnosticar (linhas de log do `dock_align_node`).**
- No início de cada nudge: `[nudge +0.10] comando: vy=+0.1300 m/s (piso 0.1215 x 1.07) em
  /mecanum_controller/reference: publicadores=1 (este no' incluso), assinantes=1` —
  `publicadores > 1` = alguém mais (Nav2, teleop) está escrevendo na referência e pode
  zerar o comando; `assinantes=0` = o `mecanum_controller` não está ouvindo.
- Se a base não responder: `nudge_base: base nao respondeu ao comando (vy=+0.1300 m/s por
  1.0 s, odom bruto +0.0 cm): conferir publicadores em /mecanum_controller/reference
  (count_publishers=N), relogio PC x Pi (reference_timeout 0.15 s no mecanum_controller) e
  o piso do ESC (roda >= 2.43 rad/s)` seguido de `fim (no_progress: sem progresso em 1.0 s;
  saida do curso: sem_progresso)`.
- A linha de fim sempre traz o motivo de saída do laço do curso: `fim (ok; saida do curso:
  corte): ...` — valores: `corte` (normal), `teto_de_tempo`, `sem_scan`, `odometria_velha`,
  `sem_progresso`, ou o outcome do abort (`hard_stop`, `side_blocked`, `canceled`, `timeout`).
- Testes puros: `python3 -m pytest src/caramelo_navigation/test/test_nudge_math.py -q`
  (inclui o caso que falhava, `0.1215/0.05 < 2.43`, as bordas do watchdog e as margens
  inválidas de `floor_start_margin`).

## Run de 2026-10-21: docking torto na WS1

**Sintoma.** `[WS1] pronto: front=0.386 yaw_face=-19.7 graus`; o `GoToWS` logou
`alinhado (... yaw=-0.345)` e seguiu para a manipulação com a mesa 20° torta (análise
completa em `docs/analise_run_2026-10-21.md`, §3).

**Causa (5 linhas).**
1. O staging das WS fica a 0,35 m atrás da pose do dock → robô a ~0,70 m da face; daí o
   fit do refino (janela até 1,20 m) pegou **mesa + vizinho**: largura 0,98 m, resíduo
   61 mm → descartado nas duas leituras.
2. Sem fit, o nó fez o **esquadro por AMCL: +12,3°** — o mapa/AMCL não batia com a mesa
   física (o LiDAR, logo depois, mediu a face a **−15,1°**, resíduo 13 mm, largura 0,64).
3. Esse fit confiável ficou preso no portão `refine_yaw_agree_tol` ("aguardando 2ª leitura
   concordante"): só havia **uma** leitura por goal depois do esquadro → `yaw MANTIDO`.
4. O reto e o lateral não corrigem yaw; o sucesso só validava a **janela frontal**
   (`default_yaw_tolerance` 0,09 rad existia mas não era checada no fim).
5. O `GoToWS` só olha `success`.

**O que mudou (`dock_align_node.py`).**
- **2ª leitura imediata no refino**: quando o 1º fit é confiável e o portão pede
  concordância, o nó espera um `/scan` **novo** (até `refine_second_read_max_s`, 0,6 s),
  refaz o fit com as mesmas guardas e, se concordar (`refine_yaw_agree_tol`), aplica o yaw
  **no mesmo goal**. Continua exigindo dois fits confiáveis (resíduo e largura).
- **Largura máxima do fit** (`refine_max_face_width`, 0,90 m; a mesa tem 0,80): fit mais
  largo é `face larga demais (vizinho?)` e, nesse caso, **não há esquadro por AMCL** no
  staging — o esquadro fica para o refit perto da mesa. O esquadro por AMCL continua só
  quando não há fit nenhum (outros motivos) **e** o AMCL passou na sanidade do staging.
  Ordem das guardas: largura máxima → resíduo → largura mínima
  (`face_fit_reject_reason`).
- **Refit perto da mesa** (`refine_refit_front_m`, 0,55 m): durante o reto, **uma vez**, o
  nó pausa (zero + assentamento) quando vai parar a ~0,55 m — o gatilho desconta o coast
  (0,10) e o lag, como o corte final, senão pararia em 0,45 < `rotate_min_front` 0,46 —,
  refaz o fit numa **banda rasa** ao redor do muro (`[front−0,03, front+0,30]`, face
  isolada dos vizinhos), em 2 leituras concordantes, e se `|yaw_rel| > yaw_tol`, acima do
  coast de giro e `front ≥ rotate_min_front`, **esquadra pela face do LiDAR**
  (`_cut_drive` em z com erro vivo = `face_yaw` do muro, não o AMCL). Com
  `front < rotate_min_front` não gira e registra.
- **Validação final de yaw**: depois do `lateral(mesa)`, mede o yaw da face (mediana do
  fit leve do muro em até 5 scans, dispersão ≤ `final_yaw_spread_max` 0,07 rad). Decisão
  (`final_yaw_decision`): dentro de `yaw_tolerance` (goal, ou `default_yaw_tolerance`
  0,09) → `pronto`; fora e ainda sem correção neste goal (`final_yaw_correction` true,
  sobrando ≥ 20 s do `align_timeout`) → **recua até `rotate_min_front`+0,05, esquadra pela
  face, refaz reto e lateral, mede de novo** (custo registrado no log). Ainda fora →
  `result.final_yaw_error` = yaw da face e `message` começa com
  **`yaw_fora_da_tolerancia (x graus)`**: com `final_yaw_strict` false (default,
  competição) o goal **sucede com WARN grande**; com true, **falha** (abort, com o recuo
  de segurança de sempre). Sem fit confiável no fim → `yaw_nao_medido` (sucesso; o yaw
  do AMCL vai só para o log e para `final_yaw_error` — nada gira pelo AMCL).
  Entre a tolerância (5,2°) e o coast do giro (`yaw_coast_est`+0,02 = 6,9°) o giro não
  corrige: aceita com aviso sem recuar.
- Todas as primitivas novas passam por `_check_guards` (hard_min, cancel, timeout do goal
  e da tentativa); o `_retreat` ganhou `target_front` opcional.

**Parâmetros novos** (declarados em `docking_server.launch.py`):

| Parâmetro | Default | Uso |
|---|---|---|
| `refine_second_read_max_s` | 0,6 s | espera por um `/scan` novo para a 2ª leitura do refino no mesmo goal |
| `refine_refit_front_m` | 0,55 m | pausa do reto para refit/esquadro pelo LiDAR (≤ 0 desliga) |
| `refine_max_face_width` | 0,90 m | fit mais largo = mesa + vizinho (sem esquadro por AMCL) |
| `final_yaw_correction` | true | recua + esquadra + refaz reto/lateral uma vez se o yaw final estiver fora |
| `final_yaw_strict` | false | true: ainda fora ⇒ goal falha; false: sucesso com WARN |
| `final_yaw_spread_max` | 0,07 rad | dispersão máxima das leituras de yaw na medida final |

**Como ler o log** (`dock_align_node`, prefixo `[WSn]`):
- `refino face: ... largura=0.98 m` + `refino: face larga demais (vizinho?); descartando.`
  + `esquadro por AMCL pulado: face larga demais no staging` — caso da WS1.
- `refino yaw: 2a leitura concorda (a vs b deg no mapa) — aplico no mesmo goal.` /
  `2a leitura DISCORDA` / `sem 2a leitura confiavel em 0.6 s (...)`.
- `reto: pausa para refit a front=0.655 (alvo 0.55 + coast 0.10)` → `refit(reto) face: ...`
  (duas linhas) → `refit(reto): yaw_rel -2.8 graus dentro da tolerancia` **ou**
  `esquadro por LiDAR (reto): yaw_rel -15.1 graus a front=0.552` →
  `esquadro por LiDAR (reto): yaw_face apos o giro -1.9 graus, front=0.548 (2.1 s)` **ou**
  `refit(reto): ... front=0.44 < rotate_min_front 0.46 — NAO giro`.
- Fim: `pronto: front=0.386 yaw_face=-1.9 graus (erro X +0.6 cm)`; ou
  `yaw_face -19.7 graus > tolerancia 5.2: recuo ate front 0.51, esquadro pela face ...` →
  `recuo concluido (~0.18m comandados, alvo front 0.51)` → `refit(correcao) ...` →
  `correcao final de yaw custou 11.3 s (restam 70 s do align_timeout)` → `pronto: ...`.
  Se ainda fora: `========== YAW_FORA_DA_TOLERANCIA (-8.0 GRAUS) ==========` e
  `pronto: ... — yaw_fora_da_tolerancia (-8.0 graus)` (goal sucede; com strict, `abortando`).
  Sem fit: `yaw_nao_medido: 2/5 fits confiaveis da face; AMCL diz +3.1 graus (so' para o log)`.
- Testes puros: `python3 -m pytest src/caramelo_navigation/test -q` (`yaw_needs_correction`,
  `refit_trigger`, `face_fit_acceptable`/`face_fit_reject_reason`, `final_yaw_decision`).

**Mapa (aplicado em 2026-10-23, decisão do operador):** nas WS do `arenalegal_2026`,
`approach_offset` **0,35 → 0,25** em `service_areas.yaml` (é daí que o sync gera o
`docking.yaml`; `staging_x_offset` −0,25 nos `caramelo_ws*_dock`), pré-docking a ~0,6 m da
face em vez de ~0,7. SH1 (0,4) e PP1 (0,35) ficaram como estavam. Motivo: a 0,7 m o fit do refino (janela de ±0,45 m e até
1,20 m) pega os vizinhos da mesa (0,87–0,98 m de "face" nas duas estações da run) e é
descartado; a ~0,6 m a face aparece isolada (fits de 0,61–0,65 m, resíduo ~13 mm) e o
refino do staging volta a valer — o refit a 0,55 m vira redundância, não a única chance.
A reta fica ~10 cm mais curta (o coast de 0,10 m continua coberto: corte em 0,48). Decidir
por estação e regravar o mapa; `staging_sanity_max_m` (0,6) continua válido.

## Runs de 2026-10-22/23: o cliente passa a validar a pose (gate no `GoToWS`)

Análise completa em `docs/investigacao_incidente_2026-10.md`. Resumo do que o
`dock_align_node` devolveu e do que o executor fez com isso:

| Estação / hora | `dock_align` | result (`x`, `y`, `yaw`) | GoToWS (antes) |
|---|---|---|---|
| PP1 22/10 15:42 | refino descartado (resíduo 54 mm, largura 0,82), esquadro por AMCL +7,7°, 2 giros pelo LiDAR (−12,8°, −15,3°), centro largura 0,29, **`yaw_nao_medido`** (AMCL +15,5°) | −0,011 / +0,084 / **+0,271** (AMCL) | pronto → AMT1 perdeu as 2 tags da direita |
| WS3 23/10 18:51 | refino OK (−7,3°), lateral(staging) −7,8 cm, esquadro pelo LiDAR −8,5°, centro "nenhum segmento contém o eixo do robô", **`yaw_nao_medido`** (AMCL +18,3°) | −0,006 / **+0,304** / **+0,319** (AMCL) | pronto → place/pick com a tag a −0,454 m |
| WS6 23/10 18:49 / 19:01 / 19:10 | refino descartado (resíduo 55 mm; depois "face larga" 1,1–1,2 m: a PP1 está a 0,58 m), esquadro por AMCL −11…−15°, refit "face estreita" 0,46–0,50, correção final **piorou** (+7,6 → −12,7) | yaw **+0,152 / −0,222 / +0,127** (LiDAR), `yaw_fora_da_tolerancia` | pronto (strict desligado) |

O nó continua com `final_yaw_strict=false` (decisão de competição): o result carrega a
ressalva na `message` e o erro em `final_yaw_error` (LiDAR, ou AMCL quando
`yaw_nao_medido`). **Quem decide agora é o `GoToWS`** (`manip_bt`): limites duros
|y| ≤ 0,20 m, |yaw| ≤ 0,20 rad, re-dock uma vez e `pose_gate_strict` — ver
`docs/guia_missoes.md` §5.1. Mudança aqui: as linhas `pronto:` e `yaw_nao_medido`
passam a mostrar o **Y pelo AMCL** (`erro X +0.5 cm, Y(AMCL) +30.4 cm`), que o log
não trazia e o result já levava.

O que ainda depende de dados do robô (`/scan` gravado no staging e na mesa):
o Y lateral continua "do banco" (ADR-05) e o centro da face falhou em metade das
medidas de 23/10; na WS6 o fit mistura a face com a PP1 vizinha e o esquadro por
AMCL gira para o lado errado; na PP1 às 15:42 havia algo não-plano a 0,75 m
(nas outras 6 docagens do dia o fit foi limpo).

## Ferramentas de calibração

- **`scripts/pulse_test.py` — MOVE O ROBÔ.** Publica direto em
  `/mecanum_controller/reference` (fora do collision monitor): rodas suspensas ou área
  livre, alguém no botão de emergência. `python3 scripts/pulse_test.py x 4` (eixo `x|y|z`,
  N ciclos a 20 Hz no piso do ESC) e `python3 scripts/pulse_test.py x cont` (1 s contínuo)
  medem com fita/régua o deslocamento real de um pulso e o coast do corte; os valores
  alimentam `esc_coast_dist`/`yaw_coast_est`/`center_coast_est` na lista `parameters`
  do `dock_align_node` em `docking_server.launch.py` (a data da última medição está no
  comentário do launch).
- **`nudge_odom_lateral_gain` com trena** — procedimento na seção "Parâmetros
  `nudge_*`" acima (`ros2 action send_goal /nudge_base ...`, ganho = real / odom bruto).
- **`refine_desired_face_dist`** — robô posicionado à mão na pose ideal, goal de
  `align_to_dock` com `use_lidar_refine: true, timeout: 1.0` e leitura do log
  (`docs/guia_missoes.md` §1.1).
- Testes puros do nó: `python3 -m pytest src/caramelo_navigation/test/test_nudge_math.py -q`.
