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
