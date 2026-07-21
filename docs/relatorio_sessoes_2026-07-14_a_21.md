# Relatório — Restauração completa do Caramelo (14 a 21 de julho de 2026)

> Contexto para retomada de sessão. Robô: Caramelo (RoboCup@Work, mecanum, ROS 2 Jazzy).
> Arquitetura: PC/notebook roda Nav2 (`caramelo_ws`); Raspberry Pi roda hardware (`caramelo_hardware_ws`).
> SSH na Pi: `raspberrypi@raspberrypi.local` (senha `raspberrypi`, chave instalada, ControlMaster ativo — **1 conexão por vez**, nunca em rajada). ROS_DOMAIN_ID=67 fixo nas duas máquinas.

## 1. Missão original e resultado

**Pedido inicial:** "a navegação está inativa; prioridade é o robô voltar a navegar; GUI por último".
**Resultado:** navegação restaurada e MUITO além — o robô executa a **ronda completa de competição 100% autônoma** (4+ execuções): navega → docka na bancada → alinha holonomicamente (<3 cm / ~2°) → undocka → próxima estação → FINISH. GUI com backend funcional (teste final pendente).

## 2. O que foi consertado, por área

### 2.1 Firmware dos ESCs (B-G431B-ESC1, projeto `~/Downloads/Projeto_com_hall_final_22_06_2026`)
Gravado nos **4 ESCs** pelo operador. Mudanças (autorizadas pelo operador; antes o firmware era intocável):
- `speed_min_valueRPM` 1000→**300** (`mc_parameters.c:195`) → piso de roda 3,74→**1,12 rad/s** = **0,056 m/s** de corpo (gearbox 28:1, roda Ø100 mm).
- **Anti-disparada** no `esc.c` (middleware MCSDK): pulso só é comando se **válido** (≥~900 µs) e **fresco** (captura ≤35 ms); watchdog de sinal agora **PARA o motor** (antes só zerava o pulso e, em ré, zero = ré total); arming de ré simétrico ao forward (era **instantâneo** por bug `counter<ARMING_TIME`).
- **Bandas alargadas** (`parameters_conversion_g4xx.h`): partida 1540/1460 µs, arming 1530/1470 µs, neutro 1500 µs (banda morta ±30 µs > tolerância ±1,5 % do oscilador das placas — antes ±10 µs, menor que o erro de medição).
- Resultado validado: bateria sem Pi = parado (só beep); bringup carregando = parado; perda de PWM = para em ~0,5 s.

### 2.2 Driver da Pi (`caramelo_hardware_ws`, dev @ f795d10 — PC e Pi IDÊNTICOS, push pendente)
- **Bug histórico morto**: "rodas giram sem parar no bringup" era a Pi mandando **1000 µs = ré máxima por ~8 s** — comandos NaN do ros2_control até o controlador ativar → `lround(NaN)` → clamp em 1000. Fix: guarda `isfinite`→neutro no `maxon_motors_node` + `on_activate` zera comandos (flagrado com monitor pigpio; script reutilizável em `~/pwm_monitor.py` na Pi).
- Mapa afim casado com firmware novo: `kPulseUs` 1540/1460, margem partida 30 µs, URDF `min_wheel_rad_per_sec` 1,12.
- EKF: **40 Hz** (Pi não sustenta 50 nos picos), `print_diagnostics: true` (alimenta card da GUI); **mesh server com `nice +15`** (servir STL ao RViz roubava ciclo do EKF).
- Regras operacionais: **`sudo poweroff` antes de cortar energia da Pi** (2 corrupções de SD por corte a frio); desligar bateria dos motores ANTES da Pi (bug PWM-loss).

### 2.3 Relógio (resolvido de vez)
- Pi sem RTC + deriva rápida quebrava tudo (collision_monitor descartava scan "1,4 s velho" → robô planejava e não andava).
- **Chrony servidor no PC** (10.42.0.1, `allow 10.42.0.0/24`, `local stratum 10`) + **timesyncd cliente na Pi** → offset <3 ms, autocorrige após cada boot. Doc: `docs/sincronizacao_relogio_ntp.md`.

### 2.4 Navegação (`caramelo_ws`, dev @ HEAD atual — ~16 commits, push pendente)
- Mapa novo **`arena2_520`** (sala_520 estava desatualizado; AMCL não convergia — sempre salvar mapa com `map_saver_cli` ANTES do Ctrl+C no SLAM; mapa precisa chegar ao install: `colcon build caramelo_mapping`).
- **DWB é o controlador titular** (BT fixo, sem ControllerSelector — o painel do RViz publicava MPPI por engano). MPPI: anda mas rasteja; tuning fica para sessão dedicada.
- **RotationShimController** envolve o DWB: erro de rumo >45° → gira NO LUGAR pelo lado curto (0,6 rad/s); giro final do goal idem. NÃO baixar o limiar para 30° (testado: shim engaja na zona inflada perto de bancada, colisão-check bloqueia, e a recovery dá ré cega rumo à mesa).
- **Recovery de escape**: limpa costmaps + **ré 0,20 m** (20 s de allowance — o collision_monitor freia a ré e 10 s não bastavam) + wait; 6 retries. Spin continua desligado. Ré é CEGA (lidar frontal).
- **Tolerâncias ≥ ruído do AMCL** (lição central): xy 0,10 m / yaw 0,20 rad. Janela menor que o ruído (~16 cm/~10°) = robô caça o goal para sempre.
- DWB: 8×8×20=1280 trajetórias (CPU), pisos casados ao ESC (`min_speed_xy` 0,10 / `min_speed_theta` 0,25), sem ré (`min_vel_x` 0), crítico `ObstacleFootprint` (não BaseObstacle), inflação 0,55 m, Twirling 1.0, progress checker 6 s/0,10 m. AMCL 500 partículas (CPU).
- **CPU do notebook é o gargalo** (some no PC definitivo): fechar RViz nas rondas; nav enxuta = `use_service_area_manager:=false use_service_area_markers:=false` (o manager come 22 %!); loop do controlador cai de 20 Hz para 8–13 Hz quando satura → AMCL pula → sucessos falsos.

### 2.5 Docking (validado ponta a ponta)
- Cadeia: `dock_robot` (staging→approach) + **`dock_align_node`** (alinhamento fino holonômico; erros ~1 cm/1–2°; pisos novos `min_body_vel 0.06 / min_body_omega 0.16`) + `undock_robot`. Align e `dock_pose_recorder_node` agora sobem junto do `docking_server.launch.py`.
- Params persistidos: v_linear 0,08–0,12, v_angular 0,4, `use_collision_detection false` (dock encostado na bancada sempre dispara o checker), `docking_threshold` 0,10, **staging 0,25 m** (0,45/0,7 caíam em parede; undock recua até a staging — precisa de espaço físico atrás!), undock tolerâncias 0,15 m / **3,14 rad** (ângulo não importa no undock).
- **Truque de lifecycle**: params do controlador do docking só valem no `configure`. Para aplicar sem derrubar tudo: `ros2 param set` → `RESET`+`STARTUP` via `/lifecycle_manager_docking/manage_nodes` (usar `ros2 lifecycle set` direto não funciona — o bond re-ativa por baixo). SEMPRE verificar com `param get` depois.
- `service_areas.yaml` × `docking.yaml`: `save_dock_pose` só atualiza docking.yaml — sincronizar as áreas (feito para arena2_520). Docks derivam das áreas: `final_robot_pose` == pose do dock.

### 2.6 Missão (`mission_runner`)
- `ros2 run caramelo_navigation mission_runner --map-name arena2_520 --stations WS1 WS2 --finish FINISH --max-staging-time 120`
- Estações = dock+align+undock; **START/FINISH = navegação pura** (não são bancadas; o dock() ali ficava ajustando milímetros e estourava tempo). Pausa de manipulação **5 s** por estação (`--manip-delay`, placeholder da Fase 8 braço).
- `mission_navigate.py` = ronda só-navegação (não usar para bancada: pose na zona inflada dá "Start occupied"/sucesso falso).

### 2.7 GUI (`caramelo_gui`, Qt/C++ com RViz embutido — backend corrigido, visual intocado)
Corrigido: mapa default arena2_520 (5 módulos); serviço `/map_saver_server/save_map`; checkbox docking nasce marcado; **goal em tópico privado** `/caramelo_gui/goal_pose` (bt_navigator também ouve `/goal_pose` → goal duplicado); undock com tipo explícito; diagnósticos **consolidados por nome** (barra caía p/ OFFLINE com snapshots parciais); labels curtos nos fantasmas de área; modo fantasma **esconde robô/scan reais** e **ativa ferramenta Interact** (sem ela o clique só movia a câmera); preview do scan do fantasma com **rotação 3D completa** — descoberta: **o LiDAR monta DE CABEÇA PARA BAIXO (TF base→laser RPY [180,0,180])**; extrair só yaw espelhava o scan.
**Pendente da GUI:** teste final do fluxo completo (fantasma→goal→docking→áreas) após os últimos fixes; conferir se os footprints das áreas ficam certos com a localização correta (suspeita: o "180°" era sintoma de localização espelhada pelo preview antigo).

## 3. Fatos técnicos permanentes (não esquecer)
- Piso do ESC: roda ≥1,12 rad/s (0,056 m/s corpo); zona morta <0,028 m/s; comandos entre 0,028–0,056 sobem para 0,056.
- **LiDAR de cabeça para baixo** (RPY 180/0/180) e **FOV útil = 180° frontais** (scan_normalizer corta a metade traseira) → ré e laterais são cegas.
- IMU `/imu/data_raw` a 10 Hz (fundida no EKF). `/odom` 40 Hz (EKF), `/joint_states` 100 Hz, `/scan` 10 Hz.
- Footprint 0,79×0,39 m (frente 0,395 / trás 0,285). controller_manager 100 Hz FIFO na Pi.
- pkill/pgrep: usar truque do colchete (`padrao[x]`) e NUNCA colocar o comando de restart no mesmo compound do pkill.
- `ros2 lifecycle` RESET+STARTUP relê os params **do processo**, não os arquivos — mudou arquivo, ou set ao vivo antes do ciclo, ou restart do processo.

## 4. Estado dos repositórios (push pendente pelo operador!)
- `caramelo_ws` dev: ~16 commits locais (de 6037e21 até o HEAD atual — nav, mapa, docking, missão, GUI).
- `caramelo_hardware_ws` dev @ f795d10: PC e Pi **idênticos e limpos** (sincronizados via bundle). Pós-push, `git pull` na Pi será fast-forward.
- Firmware ESC: alterações no projeto STM32 (fora do git? conferir com a equipe) — gravadas nos 4 ESCs.
- Comando: `cd ~/caramelo_ws && git push origin dev` e `cd ~/caramelo_hardware_ws && git push origin dev`.

## 5. Próximos passos sugeridos
1. **GUI**: re-teste final do fluxo (fantasma agora arrastável e com scan certo) + verificar footprints das áreas com localização boa.
2. **Push** dos dois repositórios.
3. Calibrar o **refino LiDAR do `dock_align_node`** (hoje "sem confiança", usa pose do banco).
4. **MPPI**: sessão dedicada de tuning (anda mas rasteja; ambiente já saneado).
5. **Fase 8**: manipulação (braço) no gancho do mission_runner.
6. PC definitivo: replicar setup do notebook (checklist na memória: sshpass, chave SSH+ControlMaster, chrony, rede compartilhada 10.42.0.1, bashrc) — lá a CPU deixa de ser gargalo.

## 6. Comandos essenciais
```bash
# Nav enxuta (terminal):
ros2 launch caramelo_navigation bringup.launch.py map_name:=arena2_520 use_rviz:=true \
  use_service_area_manager:=false use_service_area_markers:=false
# Ronda completa:
ros2 run caramelo_navigation mission_runner --map-name arena2_520 \
  --stations WS1 WS2 --finish FINISH --max-staging-time 120
# GUI (sobe tudo por dentro):
ros2 launch caramelo_gui caramelo_gui.launch.py
# Docking avulso / undock / align server:
ros2 run caramelo_navigation dock_to --dock-id WS1 --map-name arena2_520 --max-staging-time 120
ros2 run caramelo_navigation undock
# (align sobe junto do docking_server; avulso: ros2 run caramelo_navigation dock_align_node \
#   --ros-args -p min_body_vel:=0.06 -p min_body_omega:=0.16)
# Gravar poses (robô parado no lugar):
ros2 run caramelo_navigation save_service_area_pose --map arena2_520 --name WS1 --type workstation \
  --linear-tolerance 0.10 --angular-tolerance 0.10 --overwrite --no-sync-docking
ros2 run caramelo_navigation save_dock_pose --map-name arena2_520 --dock-id WS1
ros2 service call /docking_server/reload_database nav2_msgs/srv/ReloadDockDatabase \
  "{filepath: '/home/linux24-04/caramelo_ws/src/caramelo_mapping/maps/arena2_520/docking.yaml'}"
```
