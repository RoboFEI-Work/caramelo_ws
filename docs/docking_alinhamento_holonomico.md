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

O `dock_align_node` (`caramelo_navigation`) expõe a action
**`caramelo_msgs/action/AlignToDock`** (`align_to_dock`):
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

**Nada disto foi validado em hardware** — o controle é testável em simulação; a
percepção precisa de scan real.

## Ajuste lateral `nudge_base` (2026-08-28, fila de alcance)

O mesmo `dock_align_node` expõe uma segunda action, **`caramelo_msgs/action/NudgeBase`**
(nome relativo `nudge_base`, como `align_to_dock`): "anda `dy` metros de lado", pedida
pela árvore (`ReachQueue`/`NudgeBase` do `manip_bt`) quando o braço não alcança um alvo
na mesa. A Raspberry não muda: usa o mesmo `/mecanum_controller/reference` e o `/odom`
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
`cancelado`, `ocupado`, `curso_incompleto`. (`pedido_invalido` — revisão 28/08 — ainda não
consta no comentário de `NudgeBase.action`; o `.action` não foi alterado.)

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
  Teto de tempo `(|dy| + 0,05)/min_body_vel + 1 s`.
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
