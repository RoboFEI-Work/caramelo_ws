# PLANO DE CORRECAO PRIORIZADO — Manipulacao Caramelo (2026-08-07)

Regras do operador respeitadas: planning time mantido em 15 s; todos os deadlines abaixo sao **maiores que o pior caso saudavel** (so eliminam espera infinita, nunca cancelam acao viva); yaw da tag continua fora da pose de grasp.

---

## FASE 1 — ANTI-TRAVAMENTO (minimo para voltar a testar com seguranca)

**1.1 — Deadline client-side em TODO plan()/execute() de MoveGroupInterface** `[S1/S2/S9 — dono dos 2 travamentos observados]`
- Arquivos: `mtc_pick_action_node.cpp` L1079/L1092 (planAndExecute), L1239/L1264/L1281 (moveToTarget); `mtc_place_action_node.cpp` L407-408/L419 (planAndExecute), L567-568/L578-579/L591 (moveToTarget).
- Mudanca: helper unico `bool callWithDeadline(fn, timeout)` usando `std::async` + `future.wait_for()`. No timeout: logar CRITICO, chamar `iface->stop()`, retornar falha (thread orfã fica em busy-poll de 1 ms — barata; documentar que N timeouts consecutivos ⇒ reiniciar o no). Deadlines: **plan = 15 s (planning time) + 10 s de margem = 25 s**; **execute = duracao da trajetoria planejada × 3.0 + 3 s + 10 s** (espelha o monitor server-side, nunca mata execucao lenta saudavel do Dynamixel).
- Justificativa tecnica: MoveIt 2.12.4 faz `while(!done) sleep(1ms)` sem timeout e sem checar `rclcpp::ok()`; goal-response perdida no DDS = `done` nunca vira true. Nao ha fix possivel via cancel/parametro — tem que ser wrapper no cliente.
- Risco: **baixo** (timeout so dispara em cenario ja-morto). Atencao: no place, timeout deve seguir o caminho de `finish_failure` para liberar o **flock** (hoje thread pendurada segura `/tmp/caramelo_manip_action.lock` e mata pick E place ate matar o processo).

**1.2 — Timeout + try/catch nos construtores de MoveGroupInterface** `[risco S2 novo; deadlock de flock no place]`
- Arquivos: `mtc_pick_action_node.cpp` L717, L725, L1683; `mtc_place_action_node.cpp` L791-792.
- Mudanca: passar `wait_for_servers = Duration::from_seconds(15)` (3o arg) e envolver em try/catch de `std::runtime_error` (waitForAction lanca no timeout) → abortar o goal com mensagem clara em vez de bloquear para sempre. **No place, mover a construcao dos MGIs para DEPOIS do `findContainerByTag` (L797)** — o caminho de skip deixa de pagar 1-2 s e de segurar o flock a toa.
- Risco: **baixo**; move_group demorando >15 s no boot vira falha de goal explicita (aceitavel — o BT reporta).

**1.3 — Remover o bloco approach_task MTC** `[S4 — elimina o sintoma inteiro + 2 esperas infinitas]`
- Arquivo: `mtc_pick_action_node.cpp` L1724-1742 (e funcoes `createApproachTask` L1297-1330 / `executeTask` com `task.plan` L1355 e `task.execute` L1368).
- Mudanca: deletar o bloco. Verificacao adversarial confirmou: e CurrentState→MoveTo("pegar_obj") **redundante** (L1698-1722 ja levou o braco la com recovery), a falha e sempre engolida, e `task.execute()` do MTC 0.1.5 espera result **sem timeout** (resposta perdida = server "busy" para sempre — bate com os commits "server ocupado"). Bonus provavel: reduz o residuo de scene-diffs do S7.
- Risco: **quase nulo** — o fallback ja e o caminho que executa 100% dos picks hoje.

**1.4 — Guarda getCurrentState(2.0) no fallback de IK aproximada do PLACE** `[S1 nao mitigado no place]`
- Arquivo: `mtc_place_action_node.cpp` antes da L576 (`setApproximateJointValueTarget` L577).
- Mudanca: espelhar exatamente a guarda do pick (`mtc_pick_action_node.cpp` L1251): `if (!arm->getCurrentState(2.0)) return false;`.
- Risco: **nulo** (copia de mitigacao ja validada em campo).

**1.5 — Watchdog de progresso nos clients BT (rede de seguranca)** `[qualquer hang residual vira falha visivel]`
- Arquivos: `pick_tag_bt.cpp`, `place_tag_bt.cpp` — usar o molde pronto de `go_to_ws_bt.cpp` L58-67/289-298.
- Mudanca: deadline **rearmado a cada feedback de stage** (ex.: 120 s sem mudanca de stage ⇒ cancel + FAILURE). Acao saudavel publica stage continuamente, logo nunca e cancelada — respeita a regra do operador. Cancel nao solta thread travada no server (item 1.1 resolve isso), mas converte "6 min mudo" em falha imediata e logada.
- Risco: **baixo**; escolher janela com folga (retry x3 completo do pick fica dentro de ~120 s/stage).

---

## FASE 2 — CORRECAO FUNCIONAL (S3–S6, S8)

**2.1 — `reset_container_states` default → false** `[S6 parte 3 — rota mais provavel do sintoma de campo]`
- Arquivo: `manip_stack.launch.py` L133.
- Mudanca: `default_value="false"`; reset explicito so por argumento (`reset_container_states:=true`) no inicio de rodada/competicao. Documentar no README do bringup.
- Risco: **baixo** — yaml podre entre rodadas passa a exigir reset manual; e o trade correto (relaunch pos-crash preserva o estado real dos containers).

**2.2 — Distinguir falha PRE-grasp de POS-grasp no pick** `[S3/S6/S10 — skip nao pode engolir falha com bloco na garra]`
- Arquivo: `mtc_pick_action_node.cpp` — causa em L1635-1640, decisao em L1859-1865.
- Mudanca: `transferGraspedObjectToContainer` retorna enum (`OK` / `FAIL_HOLDING` L1400-1425 / `FAIL_AFTER_RELEASE` L1434). (a) `FAIL_AFTER_RELEASE`: bloco JA esta no container ⇒ chamar `setOccupied` e tratar como **sucesso** (corrige o bug do container reutilizado com 2 blocos). (b) `FAIL_HOLDING`: **nunca** virar `success=true` skip — retentar o transfer 1x; persistindo, abortar com message `object_still_grasped` (o gripper_open automatico do proximo goal L1689-1696 derrubaria o bloco no chassi).
- Risco: **medio** (mexe na semantica do skip de competicao) — mas o skip continua valendo para todas as falhas pre-grasp, que era a intencao original.

**2.3 — Gate de retry: toda falha pre-grasp e retentavel** `[S3 — "tentou so 1 vez"]`
- Arquivo: `mtc_pick_action_node.cpp` L1785-1787 (gate), L1548/1558/1572/1589/1603/1612 (returns).
- Mudanca: trocar `failed_grasp_verification` por `retryable_failure` setado em TODOS os returns pre-grasp (tag nao vista, align, re-detect, moveToTarget/IK, telemetria de esforco indisponivel L1598-1604, fechar garra) — o loop existente de 3 tentativas passa a cobrir exatamente o caso mais comum dos logs (falha de IK/plan), que hoje quebra na 1a. Break imediato so em cancelamento e no `FAIL_HOLDING` do 2.2.
- Risco: **baixo** — custo maximo 3x o tempo por tag, ja aceito pelo design atual.

**2.4 — PLACE: verificacao de objeto preso por esforco** `[S6 partes 1-2]`
- Arquivo: `mtc_place_action_node.cpp` L906-912 (+ subscription nova).
- Mudanca: portar do pick a infra de `/dynamic_joint_states` + `verifyGraspByEffort` (pick L824-889) para depois do `gripper_close` no container. Garra fechou no vazio ⇒ reabrir, **nao** chamar `setEmpty` (L838-840 hoje corrompe o yaml no sucesso fantasma), retornar falha/skip distinta. E no `finish_skip` (L771-780): parar de retornar `success=true` mudo — ver 2.5.
- Risco: **medio** — limiares de esforco precisam de 1 sessao de calibracao no robo real; comecar com os do pick.

**2.5 — Contrato de skip explicito na action** `[S3/S6 — BT cego para skips]`
- Arquivos: `.action` de Pick/Place (+ `pick_tag_bt.cpp` L107-126/L118, `place_tag_bt.cpp` L113, `task_planner.cpp`).
- Mudanca: adicionar `bool skipped` (e idealmente `string fail_reason`) ao result; BT loga WARN distinto e o task_planner conta skips na telemetria da missao. Elimina o matching por substring `"skipped"` (pick_tag_bt L118).
- Risco: **baixo** (rebuild das interfaces; mudanca mecanica).

**2.6 — CurrentStateMonitor: acabar com o cold-start por goal** `[S8 → alimenta S1/S5/S9]`
- Arquivo: `mtc_pick_action_node.cpp` L907-913 (`releaseExecutionResources`) e L1682-1683; place equivalente.
- Mudanca: (a) manter um MGI membro persistente (ou simplesmente NAO zerar `active_arm_/active_gripper_` no release — o weak_ptr do monitor compartilhado fica vivo e a subscription de joint_states nunca esfria); (b) warmup unico no inicio de `execute()`: `getCurrentState(5.0)` antes do loop de tentativas. (c) **Reutilizar o MGI entre attempts** (L1760): a recriacao nao troca solver de IK (ver 2.7) — so os params dinamicos importam.
- Risco: **baixo**; memoria de 2 MGIs residentes e desprezivel.

**2.7 — Desmontar o placebo da escada de IK** `[S3/S5 — attempt_2 e re-roll, attempt_3 anuncia troca que nao ocorre]`
- Arquivo: `mtc_pick_action_node.cpp` L623-685 (applyIkProfile), L722-728, L1760, L1791 (fala).
- Mudanca: remover a troca de solver (nunca chega ao move_group; localmente o plugin e cacheado) e o `mode="Speed"` invalido do attempt_3 (rejeitado pelo validador do pick_ik e ignorado em silencio). Manter por tentativa **so o que age de verdade**: goal tolerances no MotionPlanRequest (L1231-1233) + params numericos dinamicos do pick_ik (rotation_scale, mdw — valem no fallback local L1261) + `enforce_pitch_pi`. Corrigir a fala ("vou relaxar as tolerancias", nao "trocar o modelo de IK"). Checar retorno de `set_parameter`.
- Risco: **nulo** — remove comportamento inexistente, nao muda o que funciona.

**2.8 — Yaw radial: corrigir origem e cobrir a tentativa 3** `[S5]`
- Arquivo: `mtc_pick_action_node.cpp` L1207-1208, L1221, L1774, L1210.
- Mudanca: (a) `yaw_radial = atan2(y, x - 0.217)` (eixo da junta 1 esta em x=+0.217 m do base_footprint — `robot.urdf.xacro:10`; erro atual de 15-25° e absorvido pela junta 5 mas tira o punho do zero natural); melhor ainda: 1 lookupTransform estatico de `manip_base_link`. (b) **L1221 (attempt 3)**: substituir o `yaw` cru da tag pelo mesmo yaw radial — regra do operador exige tag fora da pose de grasp em TODAS as tentativas; manter a estrategia pitch=0 se desejada, so trocar o yaw. (c) Guarda de estado no TOPO de `moveToTarget` (cobre `getCurrentPose` L1210, que hoje devolve identidade em silencio).
- Risco: **baixo**; convencao RPY(0,π,ψ) ja validada como alcancavel para este 5-DOF com qualquer ψ.

**2.9 — Sucesso fisico ≠ sucesso de escrita do yaml** `[coerencia S6; evita missao abortada com objeto entregue]`
- Arquivos: `mtc_pick_action_node.cpp` L1844-1852; `mtc_place_action_node.cpp` L836-870.
- Mudanca: falha de I/O do yaml apos sucesso fisico ⇒ `success=true` + WARN + flag no message (nao abortar; nao reexecutar place de garra vazia).
- Risco: **baixo**.

---

## FASE 3 — HIGIENE E ROBUSTEZ

**3.1** `go_to_named_pose_bt.cpp` L189/L223-227: **home com 2-3 tentativas** (guarda `getCurrentState(2.0)` entre plans) `[S10]`; L208-221: fallback-para-home deve retornar **FAILURE** (ou porta de saida), nunca SUCCESS fingindo estar na pose pedida; `execute()` L208/230 entra no wrapper de deadline do 1.1. No task_planner: home falho no epilogo nao deve impedir o goto FINISH. Risco baixo.

**3.2** `container_state_store.cpp` L29-53: `fsync` antes do rename; eliminar a janela remove+rename (usar rename atomico com overwrite); proteger read-modify-write entre pick/place com o flock existente. Risco baixo. `[fecha rota residual do S6]`

**3.3** QoS de `/dynamic_joint_states` → `SensorDataQoS` (pick L442-449 e no port do 2.4); avaliar reduzir `joint_state_broadcaster` da Pi para 50 Hz (**usuario aplica na Pi por conta propria** — corta ~metade do trafego do enlace critico). Risco baixo. `[S9]`

**3.4** Dead code: remover `runCameraAlignmentRetry` (L1494-1521), `use_camera_alignment_retry`, `fallback_ik_profile_`, `createArmInterface(true)` — parametros que prometem retry inexistente. Risco nulo.

**3.5** `pick_tag_bt.cpp:49` / `place_tag_bt.cpp:57`: trocar `wait_for_action_server(10s)` hardcoded pelo `server_wait_timeout` do blackboard (padrao ja usado em go_to_ws_bt L174). Risco nulo.

**3.6** Threads destacadas (place L740-747, pick L990-995): trocar `.detach()` por `std::jthread` membro com join no destrutor (evita use-after-free no Ctrl-C). Risco baixo.

**3.7** `manip_stack.launch.py` L393-405: renice one-shot de 12 s → `respawn=True` nos nos de percepcao + loop de renice (ou systemd-run com CPUWeight). Sustenta as condicoes que amplificam S2/S4/S8/S9. Risco baixo.

**3.8** S7 (documentar, sem fix no ws): warning vem do `PlanningSceneMonitor::getUpdatedFrameTransforms` do move_group varrendo TODOS os frames TF — frames `tag_N` stale (apriltag so publica com tag visivel) disparam 1 warn por tag por request de plan. Benigno; util como termometro de dropout da camera. Opcional: baixar log level do planning_scene_monitor.

**3.9** Menores: `bt_yaml_executor.cpp:369-376` — epilogo deve consultar `useDocking(finish_id)` em vez de `false` hardcoded; nome de no com endereco de memoria em go_to_named_pose_bt L124 impede override de params; SRDF — medir e desabilitar apenas pares fantasma fisicamente impossiveis (camera_link×link2, link4/5×rodas) ANTES do proximo falso-positivo tipo S10.

---

## Mapa sintoma → item
| Sintoma | Itens |
|---|---|
| S1 | 1.1, 1.4, 2.8c |
| S2 | 1.1, 1.2, 1.3, 1.5 |
| S3 | 2.2, 2.3, 2.5, 2.7 |
| S4 | 1.3 |
| S5 | 2.7, 2.8 |
| S6 | 2.1, 2.2, 2.4, 2.5, 2.9, 3.2 |
| S7 | 1.3 (parcial), 3.8 |
| S8 | 2.6 |
| S9 | 1.1, 3.3 |
| S10 | 2.2, 3.1, 3.9 |

**Ordem de execucao sugerida:** Fase 1 completa (1.1-1.4 sao ~1 dia de trabalho, 1.5 meio dia) → teste em mock → 2.1/2.2/2.3 (semantica de skip) → 2.6/2.7/2.8 (IK/estado) → 2.4/2.5/2.9 (place + contrato) → teste no robo real → Fase 3 conforme folga. Validacao da Fase 1: derrubar o move_group no meio de um pick em mock e confirmar que o goal falha em <30 s com o flock liberado.

---

## ADENDO (pedido do operador, 2026-08-07): gate da percepcao
Camera D455 + AprilTag devem rodar SO durante pick/place (robo parado na mesa):
- ~350 MB de RAM + o grosso do ruido de CPU/USB somem durante a navegacao;
- resolve na raiz o conflito USB2 (banda so e disputada quando importa);
- desenho: realsense+apriltag em launch separado controlado por servico
  (start no inicio do pick/place, stop no fim), ou gate de subscription
  (lazy publishing do image_transport) com no ponte. Detalhar na Fase 3.
