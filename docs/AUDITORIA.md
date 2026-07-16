# AUDITORIA — Estado Real do Sistema Caramelo (Fase 0)

**Data:** 2026-07-15
**Escopo:** os dois workspaces do robô real — `caramelo_ws` (PC principal) e `caramelo_hardware_ws` (Raspberry Pi). Documento descreve o que **existe hoje**, não o que deveria existir (regra da Fase 0 do plano mestre).

**Legenda de status:**
- ✅ **VERIFICADO** — confirmado diretamente no código/arquivos desta máquina.
- 🔴 **VERIFICAR NA PI / ROBÔ LIGADO** — só pode ser confirmado rodando na Raspberry ou com o robô energizado.
- ❓ **PRECISA DA SUA RESPOSTA** — decisão ou fato físico que só você sabe.

> **Nota importante sobre estado do código vs. robô rodando.** Este documento reflete o **código no repositório** (branch `robot_docking`/`develop`), que já inclui as correções feitas nesta sessão. O robô físico só passa a rodar essas correções depois de `git pull` + `colcon build` na Pi. As mudanças de mapa do ESC, watchdog e geometria (último commit) **ainda não foram validadas no hardware** — ver §16.

> **Atualização 2026-07-15 — respostas do dono do projeto:**
> 1. **Recorte 180° do LiDAR (§9): É PROPOSITAL.** Resolvido — deixa de ser pendência.
> 2. **Contator/E-stop (§12): NÃO EXISTE ainda.** Confirmado ausente → RK-13/16 seguem BLOQUEANTES; é hardware, prioridade nº 1 do plano.
> 3. **Sincronia de relógio (§4): setup de NTP preparado** em `docs/sincronizacao_relogio_ntp.md` (PC como servidor da LAN + Pi cliente). Falta aplicar nas máquinas reais (requer sudo).

---

## 0. Sumário executivo

**O que está sólido:**
- Distro Jazzy confirmada nas duas pontas; nenhum pacote viola a regra R1 (tudo `ament_cmake`).
- Cadeia de comando e de odometria mapeadas ponta a ponta, com frames/tópicos consistentes entre PC e Pi.
- Um único dono para cada TF no caminho de bringup correto (EKF publica `odom→base_footprint`; controlador com `enable_odom_tf: false`).

**O que é BLOQUEANTE (Fase 0.5 / Fase 1 do plano):**
1. 🔴❓ **Segurança elétrica (RK-13/16):** rodas giram sozinhas no boot/sem bringup, e o firmware do ESC entra em **ré total ao perder o sinal PWM**. Não é corrigível por software na Pi. Precisa de contator de potência amarrado ao E-stop. **Status do contator: desconhecido — precisa da sua resposta.**
2. ✅ **Teto de perda de contagem do encoder (RK-15):** a odometria é confiável nas velocidades atuais do Nav2 (≤ 0,20 m/s), mas perde contagem em alta velocidade. O que nos protege hoje é o próprio teto físico do ESC (~10,3 rad/s de roda). **Não subir os limites de velocidade do Nav2** sem antes resolver a leitura do encoder (ADR-06).

**O que precisa de você / do robô:**
- Confirmar existência de contator/E-stop (❓).
- Rodar comandos de verificação na Pi (versões, tópicos ativos, offset de relógio, tópico real da IMU) — lista pronta na §15.
- Confirmar se o recorte de 180° traseiros do LiDAR é intencional (❓).

---

## 1. Distro, versões e workspaces

| Item | Estado |
|---|---|
| Distro ROS 2 | ✅ **Jazzy** (PC: `ROS_DISTRO=jazzy`, `/opt/ros/` só tem jazzy). 🔴 Confirmar a mesma na Pi. |
| Workspaces do robô real | ✅ `caramelo_ws` (PC, 7 pacotes) e `caramelo_hardware_ws` (Pi, 4 pacotes). |
| Regra R1 (tudo `ament_cmake`) | ✅ **Nenhuma violação** — não há `ament_python` em nenhum pacote. |

**Versões-chave instaladas no PC** (✅; 🔴 as da Pi podem diferir — confirmar):

| Pacote | Versão (PC) |
|---|---|
| nav2 (stack completo) | 1.3.11 (apt tem 1.3.12 disponível) |
| ros2_control / controller_manager | 4.44.0 |
| ros2_controllers | 4.39.0 |
| mecanum_drive_controller | 4.39.0 |
| slam_toolbox | 2.8.4 |
| robot_localization | 3.8.3 |
| opennav_docking | 1.3.11 |

> Divergência interna: `ros2_control` (4.44) está mais novo que `ros2_controllers`/`mecanum_drive_controller` (4.39). Funciona, mas convém alinhar via apt e **garantir que a Pi está nas mesmas versões** — mismatch de versão de controller entre as máquinas não afeta (o controller roda só na Pi), mas o `caramelo_description` e as mensagens precisam ser compatíveis.

---

## 2. Arquitetura de pacotes

**`caramelo_ws/src` (PC) — 7 pacotes:**

| Pacote | Tipo | Papel | Observação |
|---|---|---|---|
| `caramelo_navigation` | ament_cmake (+scripts py) | **Orquestrador Nav2**: todos os params, BTs, launches, ~18 scripts de docking/service-area | pacote central |
| `caramelo_localization` | ament_cmake+py | Launch/config de map_server+AMCL | módulo Python e `install(PROGRAMS)` **vazios** — sem nós reais |
| `caramelo_mapping` | ament_cmake+py | slam_toolbox, salvamento de mapa | nó `mapping_with_known_poses.py` |
| `caramelo_cartographer` | ament_cmake | Mapeamento alternativo (Cartographer) | sem nós próprios |
| `caramelo_msgs` | ament_cmake+rosidl | 1 action + 3 srv (service areas) | interfaces próprias |
| `caramelo_utils` | ament_cmake+py | `twist_relay.py`, teleop, twist_mux | ponte cmd_vel→reference |
| `caramelo_planning` | ament_cmake | **Stub vazio** (src/ e include/ vazios) | placeholder |

**`caramelo_hardware_ws/src` (Pi) — 4 pacotes:**

| Pacote | Tipo | Papel |
|---|---|---|
| `raspberry_bringup` | ament_cmake (+scripts) | **Bringup do robô**: ros2_control, spawners, lidar, IMU, EKF, mesh_server |
| `caramelo_hardware` | ament_cmake (C++) | **Plugin ros2_control**: `mobile_base_hw_interface`, `arm_hw_interface`, `maxon_motors_node`. Backend pigpio opcional |
| `caramelo_description` | ament_cmake | **URDF/xacro/meshes** — fonte única da geometria |
| `caramelo_localization` | ament_cmake | Config/launch do EKF (robot_localization) |

**⚠️ Dependências externas do `raspberry_bringup`** (não estão nestes workspaces — vêm de `ros2_ws` ou outra fonte na Pi): `sllidar_ros2`, `wit_ros2_imu`, `apriltag_ros`, `manip_moveit_config`, `mtc_tutorial`, `my_robot_msgs`, `manip_audio`, `manip_commander`. 🔴 Confirmar na Pi que estão instaladas.

---

## 3. Rede e descoberta DDS

| Item | Estado (PC) |
|---|---|
| `RMW_IMPLEMENTATION` | ✅ vazio → default **Fast DDS** (`rmw_fastrtps_cpp`) |
| `ROS_DOMAIN_ID` | ✅ **67** (em `~/.bashrc`) |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | ✅ **SUBNET** |
| `ROS_LOCALHOST_ONLY` | ✅ não setado (correto p/ multi-máquina) |
| Perfil DDS (cyclonedds.xml/fastdds) | ✅ **não existe** em nenhum workspace — depende do default |

- Único tweak em código: `hardware_bringup.launch.py` remove `ROS_LOCALHOST_ONLY` do ambiente (defensivo).
- Dependência extra de rede: o `robot_description` aponta meshes para `http://raspberrypi.local:8000` (servidas pela Pi). RViz do PC baixa via HTTP+mDNS → precisa de `raspberrypi.local` resolvível e porta 8000 aberta.

🔴 **Verificar na Pi:** `ROS_DOMAIN_ID=67`, RMW igual (vazio→FastDDS), discovery SUBNET, localhost não-setado. **Qualquer divergência de RMW ou domínio quebra a descoberta PC↔Pi silenciosamente.**

---

## 4. Sincronização de tempo ⚠️

| Item | Estado (PC) |
|---|---|
| chrony / ntpd / ptp | ✅ **não instalados** |
| Serviço ativo | ✅ `systemd-timesyncd` (NTP da internet), relógio sincronizado |
| Mecanismo comum PC↔Pi | ✅ **NÃO EXISTE** nenhum |

**RISCO (RK-06):** todos os nós usam `use_sim_time: false` (relógio de parede). Sem uma fonte de tempo comum, se o relógio da Pi divergir do PC, os timestamps de TF/scan/odom não batem e o Nav2 solta *"message filter dropping / extrapolation into the future"*. O `timesyncd` do PC aponta para a internet — **em arena sem internet, não converge**.

🔴 **Verificar na Pi + ❓ decidir:** medir o offset de relógio Pi↔PC (`chronyc`/`timedatectl` nas duas). Recomendação: NTP local na LAN (o PC como servidor, ou um servidor comum), sobretudo em competição sem internet. O plano pede offset < 5 ms.

---

## 5. Launch files e bringups (14 launches)

**Bringups principais:**
- **Pi:** ✅ `raspberry_bringup/launch/hardware_bringup.launch.py` — RSP + ros2_control + spawners (mecanum/arm/gripper/JSB) + lidar + scan_normalizer + IMU + **EKF** + mesh_server. Cadeia de odom limpa: controller→`/odom/wheel`, EKF→`/odom` + TF.
- **PC (navegação):** ✅ `caramelo_navigation/launch/bringup.launch.py` → inclui `navigation.launch.py` → inclui `global_localization.launch.py` (AMCL) e, opcional, `docking_server.launch.py`.
- **PC (mapeamento):** ✅ `caramelo_mapping/launch/slam.launch.py` (SLAM) ou `caramelo_cartographer/launch/cartographer.launch.py`.

**⚠️ Foot-guns confirmados (não são bugs ativos, mas riscos se rodados por engano):**

1. **RK-01 — `robot_state_publisher` em 3 launches** (todos na Pi): `hardware_bringup` (✅ correto), `caramelo_hardware/hardware_interface.launch.py` (⚠️ legado), `caramelo_description/display.launch.py` (⚠️ bancada). RSP duplicado só ocorre se mais de um rodar junto. **Regra: usar exclusivamente `hardware_bringup.launch.py` na Pi.**

2. **`hardware_interface.launch.py` — conflito de odometria/TF:** este launch legado remapeia o controlador direto para `/tf` e `/odom` (`tf_odometry→/tf`, `odometry→/odom`), **sem EKF e sem IMU**, e gera o xacro sem os args de manipulador/realsense/meshes-HTTP. Se rodado junto do bringup real → dois publishers de `/odom` e TF `odom→base_footprint` duplicada. **Recomendação: aposentar ou marcar como não-usar.**

3. **RK-02 — SLAM + AMCL:** `slam_toolbox` só aparece em `slam.launch.py`; `amcl` só em `global_localization.launch.py`. **Nenhum launch acopla os dois.** O risco só se materializa se o operador subir mapeamento e navegação manualmente ao mesmo tempo — não há trava de launch impedindo, mas nada os junta.

---

## 6. Cadeia de comando (velocidade) ✅

```
Nav2 controller_server ──/cmd_vel_nav (Twist)
  → velocity_smoother  ──/cmd_vel_smoothed
  → collision_monitor  ──/cmd_vel
  → twist_relay.py (adiciona header, frame base_footprint) ──/mecanum_controller/reference (TwistStamped)
        ⇒ [Ethernet/DDS] ⇒ Pi
  → mecanum_drive_controller → MobileBaseHWInterface::write() → MaxonMotorsNode → PWM → ESCs → motores
```

- **Tópico final que sai do PC:** ✅ `/mecanum_controller/reference`, tipo `geometry_msgs/msg/TwistStamped`, frame `base_footprint`. Consistente com o contrato do Jazzy (TwistStamped, sem `use_stamped_vel`).
- **twist_mux NÃO está no caminho autônomo** — só no teleop manual (`caramelo_utils/teleop.launch.py`). Na Pi, o joystick publica direto em `/mecanum_controller/reference`. Há, portanto, **dois produtores** desse tópico (Nav2 e joystick da Pi) sem árbitro em runtime — aceitável hoje (usa-se um de cada vez), registrado.
- `mecanum_controller.reference_timeout: 0.5 s` — se a referência parar, o controller zera as rodas.

---

## 7. Cadeia de feedback / odometria ✅ (com ressalva RK-15)

```
encoders (quadratura, GPIO pigpio) → MaxonMotorsNode::update_cycle() ~100 Hz (posição/velocidade atômicas)
  → MobileBaseHWInterface::read() @100 Hz (posição REAL do encoder, SEM deadband)   ← corrigido nesta sessão
  → mecanum_drive_controller → /odom/wheel (nav_msgs/Odometry)
  → EKF (robot_localization, 100 Hz) funde /odom/wheel (vx,vy,vyaw) + /imu/data_raw (vyaw)
  → /odom + TF odom→base_footprint
```

- ✅ O `read()` usa `get_feedback()` (posição acumulada do encoder), sem o antigo deadband de 0,03 rad/s.
- ✅ `mecanum_drive_controller` (Jazzy 4.39) monta a odometria a partir das interfaces de **velocidade** das rodas — por isso o deadband corrompia `/odom/wheel` diretamente.
- ⚠️ **Ressalva RK-15 (ver §11):** confiável só até ~0,3 m/s de translação.

---

## 8. TF — propriedade e duplicatas

| Transform / tópico | Publisher | Máquina | Status |
|---|---|---|---|
| `odom → base_footprint` + `/odom` | EKF `ekf_filter_node` (`publish_tf: true`) | Pi | ✅ único no bringup correto |
| `map → odom` | AMCL (navegação) / slam_toolbox (mapeamento) | PC | ✅ mutuamente exclusivos |
| `base_footprint → base_link → sensores` + `/robot_description` | `robot_state_publisher` | Pi | ✅ só na Pi |
| `/odom/wheel` | `mecanum_controller` (`enable_odom_tf: false`) | Pi | ✅ não publica TF |

- ✅ **AMCL com TF:** confirmar que `amcl.yaml` tem `tf_broadcast`. Pelo plano, no modo "AMCL+EKF global" o AMCL deveria ter `tf_broadcast: false` e um EKF global no PC publicar `map→odom`. **Hoje NÃO há EKF global no PC** — o AMCL publica `map→odom` diretamente (arquitetura mais simples, válida). O plano prevê a evolução para dois-EKFs; hoje está no modo AMCL-direto. Registrado como decisão em aberto (ADR do plano), não como erro.
- ⚠️ Duplicata latente só via `hardware_interface.launch.py` (§5).

---

## 9. Sensores

**LiDAR:** ✅ RPLIDAR S2 (`sllidar_ros2`), `/dev/lidar_usb`, baud 1e6, frame `laser_frame`, DenseBoost ~10 Hz. Publica `scan` remapeado→`/scan_raw` → `scan_normalizer.py` → `/scan`.
- ⚠️❓ **`scan_normalizer` recorta o FOV para a metade TRASEIRA** (angle_min 1,571 → angle_max 4,712 rad = 90°–270°). AMCL/costmap/SLAM enxergam só 180° atrás — a frente fica cega. O comentário diz ser intencional, mas **impacta navegação e detecção de obstáculo à frente**. Confirmar se é desejado.

**IMU:** ✅ `wit_ros2_imu`, `/dev/imu_usb`, baud 9600, frame `imu_link`.
- ✅ **Tópico:** a análise do código-fonte do driver (em `~/ros2_ws`) mostra publicação **hardcoded em `/imu/data_raw`**, que **casa** com o `imu0` do EKF. 🔴 Confirmar na Pi com `ros2 topic list` (drivers WIT às vezes publicam `/imu/data`; se publicar nome diferente, o EKF fica sem correção de yaw silenciosamente).
- ⚠️ Timestamp na hora de chegada no nó (não na captura), 9600 baud é baixo. EKF funde só `vyaw` da IMU → impacto limitado. Melhoria = Fase futura (ADR-06/P4).

---

## 10. Camada motor / ESC — achado crítico ✅

**Hardware:** motores Maxon brushless + **4× ESC B-G431B-ESC1** (STM32G431), firmware ST Motor Control SDK **6.4.1**, **modificado pela equipe** para girar nos dois sentidos (modificação dentro de `esc.c` da árvore do SDK).

**Como o ESC realmente interpreta o comando** (verificado no código-fonte):
- ✅ Roda **controle de velocidade em malha fechada** (FOC + sensores Hall, PI a 1 kHz).
- ✅ Mapa pulso→velocidade é **AFIM com piso**, não proporcional:
  - `1520 µs` = 1000 rpm de motor = **3,74 rad/s de roda** (piso — não gira mais devagar)
  - `2000 µs` = 5364 rpm = **20,06 rad/s de roda** (escala cheia)
- ✅ O mapa PROPORCIONAL antigo do driver (0..21,3 rad/s) executava **~1,6× o comandado** (medido: cmd 4 → 6,3 rad/s de roda). **Corrigido nesta sessão** para o inverso afim.
- ✅ Teto físico medido ~10,3 rad/s de roda = saturação de corrente (clamp 4 A) / tensão do barramento (24 V), **não** limite de firmware.

**Consequências que persistem (não são consertáveis no driver PWM):**
1. ✅ **Piso de 3,74 rad/s** ≈ 0,19 m/s de translação / 0,49 rad/s de rotação. **Manobras mais lentas são fisicamente impossíveis** com este firmware — é a causa do "chicote" nas curvas lentas. Conserto real = baixar `speed_min_valueRPM` no firmware (ADR-06).
2. ✅ **Neutro não é freio:** segura a última velocidade ~0,5 s antes de parar (coast).
3. 🔴 **BUG CRÍTICO:** perda do sinal PWM → **ré total** (o firmware força `Ton=0`, que a modificação de ré interpreta como velocidade máxima reversa). Ver §12.

Documentação completa: `caramelo_hardware_ws/docs/esc_stm32_comportamento_e_riscos.md`.

---

## 11. Encoder e teto de perda de contagem (RK-15) ✅

- ✅ **Onde está ligado:** encoder de quadratura lido na **Pi via GPIO (notificação pigpio)**, decodificação por software, sem contador de hardware. Amostragem pigpio de 5 µs (200 kHz).
- ✅ **Montagem/redução:** encoder no **eixo do motor** (1024 CPT), antes da redução **1:28**. Contagens por volta de roda = `1024 × 28 × 4 = 114688` (validado pelo teste de girar a roda na mão: ~6,2 rad/volta nas 4 rodas → **a constante está certa**).
- ✅ **Diâmetro da roda:** Ø100 mm (desenho oficial) → `wheels_radius: 0.05` correto.

**A conta de perda de contagem (RK-15):**

| Velocidade de roda | Bordas/s por canal | Intervalo entre bordas | vs. amostragem 5 µs | Veredito |
|---|---|---|---|---|
| 4 rad/s (Nav2 hoje, ≤0,20 m/s) | ~36 k | ~27 µs | 5+ amostras/borda | ✅ **confiável** |
| 10,3 rad/s (teto físico do ESC) | ~94 k | ~10,6 µs | ~2 amostras/borda | ⚠️ na beira |
| 30 rad/s (1,5 m/s do rulebook) | ~273 k | ~3,7 µs | < 1 amostra/borda | 🔴 **perda garantida** |

**Conclusão:** a odometria é confiável nas velocidades que o Nav2 comanda hoje; o que impede o desastre é o **próprio teto do ESC (~10,3 rad/s)**. **NÃO subir os limites de velocidade do Nav2** rumo aos 1,5 m/s do rulebook sem antes mudar a leitura do encoder (ADR-06 / plano B dos Halls: 48 transições/volta de motor, já lidos pelo ESC). O teste de girar a roda na mão validou a *constante*, não a integridade em alta velocidade.

---

## 12. Segurança elétrica (Fase 0.5 / RK-13 / RK-16) 🔴❓

**Sintomas (relatados/confirmados no firmware):**
- Rodas giram com a bateria ligada **sem o bringup** e **durante o boot** da Pi.
- Perda do sinal PWM (cabo solto, pigpiod/Pi travada, Pi desligada) → **ré total** no ESC.

**Por que o software da Pi NÃO resolve:** com a Pi desligada ou bootando, o nosso código não existe. O watchdog de comando que adicionamos no driver só age **enquanto Pi + pigpiod + cabo estão vivos** — não cobre justamente os momentos do bug.

**O que fizemos nesta sessão (paliativo parcial):** watchdog no `MaxonMotorsNode` → PWM neutro se o `write()` parar por 0,5 s. Ajuda em "Nav2/write morreu com o robô andando", **não** cobre boot/Pi-off/cabo solto.

**O que o plano manda (e concordo):** contator/relé no barramento de potência dos motores, fechado pela Pi só após o bringup completo, e de preferência amarrado ao E-stop (o rulebook já exige E-stop). Estado seguro = default sem energia lógica.

❓ **PRECISA DA SUA RESPOSTA:** já existe algum corte de potência independente de software (contator, relé, E-stop cortando os motores)? Se não, esta é a prioridade nº 1 do plano e não é código — é hardware.

---

## 13. Parâmetros e divergências ✅

**Consistência de frames/tópicos PC↔Pi:** ✅ `base_footprint`, `odom`, `map`, `/scan`, `/odom` **idênticos** em AMCL, costmaps, SLAM, EKF e controllers. Sem divergência.

**Taxas:** ✅ Pi controller_manager 100 Hz, EKF 100 Hz, braço 25 Hz (`rw_rate`+async); PC controller 20 Hz, planner/costmap global 1 Hz. Coerente.

**Geometria (corrigida nesta sessão):** ✅ `wheels_radius: 0.05`; `sum_of_robot_center_projection_on_X_Y_axis: 0.3855` (lx 0,2355 + ly 0,150, do desenho oficial).

**Pontos de atenção já cobertos acima:** os dois launches concorrentes de odometria (§5), o recorte 180° do LiDAR (§9), o tópico da IMU (§9).

---

## 14. Riscos consolidados (mapeados ao plano)

| ID | Risco | Status atual |
|---|---|---|
| RK-01 | RSP duplicado | ✅ Mitigado se usar só `hardware_bringup`. Foot-gun documentado |
| RK-02 | SLAM+AMCL juntos | ✅ Não acoplados; sem trava de launch (baixo risco operacional) |
| RK-06 | Sincronia de relógio ausente | 🔴 **Aberto** — sem chrony/NTP comum PC↔Pi |
| RK-13 | Rodas giram sem software | 🔴❓ **Aberto/BLOQUEANTE** — precisa de contator |
| RK-15 | Perda de contagem em alta velocidade | ✅ Sob controle nas velocidades atuais; **não subir velocidade** |
| RK-16 | Perda de sinal PWM = movimento | 🔴 **Aberto** — bug de firmware do ESC (ré total) |
| — | `hardware_interface.launch.py` remapeia TF/odom | ⚠️ Legado — recomendar aposentar |
| — | LiDAR recortado 180° traseiro | ⚠️❓ Confirmar intenção |
| — | Tópico da IMU (`/imu/data_raw`) | 🔴 Confirmar na Pi (código sugere OK) |

---

## 15. Pendências — comandos para rodar na Pi (checklist Fase 0)

Rode na Raspberry (com o bringup ativo onde indicado) e cole a saída:

```bash
# Versões na Pi (comparar com o PC, §1)
ros2 pkg xml -t version ros2_control controller_manager mecanum_drive_controller robot_localization

# Ambiente de rede (comparar com PC: DOMAIN=67, RMW vazio→FastDDS, SUBNET)
echo "RMW=$RMW_IMPLEMENTATION DOMAIN=$ROS_DOMAIN_ID RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE LOCALHOST=$ROS_LOCALHOST_ONLY"

# Com o hardware_bringup rodando:
ros2 control list_controllers          # todos active?
ros2 topic list                        # a IMU publica /imu/data_raw ?
ros2 topic hz /joint_states /odom/wheel /odom /imu/data_raw /scan
ros2 run tf2_tools view_frames         # árvore única, sem duplicata de odom→base_footprint

# Sincronia de relógio (rodar nas DUAS máquinas e comparar)
timedatectl
```

**Verificações físicas / suas respostas (❓):**
- Existe contator/relé/E-stop cortando a potência dos motores? (RK-13/16)
- O recorte de 180° traseiros do LiDAR é intencional? (§9)
- De qual workspace a Pi builda o `caramelo_description`? (há cópias em vários diretórios — deve ser `caramelo_hardware_ws`)

---

## 16. O que mudamos nesta sessão (para o repositório refletir a realidade)

Todas em `caramelo_hardware_ws` (branch `develop`), **algumas ainda não validadas no hardware**:

| Mudança | Arquivo | Validado no robô? |
|---|---|---|
| `read()` com encoder real, sem deadband | `mobile_base_hw_interface.cpp/.hpp` | ✅ Sim (teste da mão + taxas) |
| controller_manager 100 Hz | `caramelo_controllers.yaml` | ✅ Sim (~99,9 Hz) |
| Braço `rw_rate=25` + `is_async=true` | `manipulator.ros2_control.xacro` | ✅ Sim (sem overrun) |
| EKF 100 Hz | `ekf.yaml` | ✅ Sim |
| Geometria `sum=0.3855` | `caramelo_controllers.yaml` | ⚠️ Não isoladamente |
| **Mapa afim do ESC + watchdog** | `maxon_motors_node.cpp/.hpp`, `mobile_base.ros2_control.xacro` | 🔴 **Não — validar (§15)** |
| Shutdown limpo do encoder | `maxon_motors_node.cpp` | ⚠️ Parcial |
| Docs (calibração, RT, ESC) | `caramelo_hardware_ws/docs/` | — |

---

*Fim da auditoria. Nenhuma linha de código de aplicação foi escrita nesta fase — apenas este documento e as correções de baixo nível já discutidas e aprovadas por você nas etapas anteriores.*
