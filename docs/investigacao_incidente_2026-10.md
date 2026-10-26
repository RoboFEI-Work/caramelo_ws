# Investigação técnica — runs de 2026-10-22 e 2026-10-23 na arena (`arenalegal_2026`)

Item 7 da lista do operador. Base: logs de `~/.ros/log` (launches de 22/10 14:00 e
15:38 e de 23/10 18:24), os `actions.yaml` gerados em `~/.ros/caramelo_missions/`,
o código da branch `detectando_container_e_usando_yaml_novo` (até `cf401be`) e a
análise anterior `docs/analise_run_2026-10-21.md`. **Não há rosbag** de nenhuma
dessas runs e **o vídeo com a narração do operador não está neste PC** (procurado
em `~` e `/media`; nenhum `.mp4/.mov/.mkv` dos últimos 30 dias) — ver §G.

Legenda usada em todo o documento:

- **FATO** — está no log (com hora e arquivo).
- **RELATO** — vem do operador. *Nenhum relato foi recuperado* (sem vídeo/transcrição);
  o que o texto do item 7 descreve (fitas no piso, objetos azuis, elementos circulares)
  é a vistoria posterior, não a execução.
- **HIPÓTESE** — explicação técnica compatível com os fatos, ainda sem prova.
- **COMPROVADO** — causa demonstrada no código e reproduzida em teste.

Horários em hora local (-03:00). Os logs usam epoch; a referência é
`1792688457 = 22/10 14:00:57` e `1792790661 = 23/10 18:24:21`.

---

## A. Reconstrução do incidente

### A.1 O que rodou (FATOS)

| Data/hora | Launch / executor | Tarefa (`actions.yaml`) | Estações | Resultado |
|---|---|---|---|---|
| 22/10 14:04–14:15 | `bt_yaml_executor_278915` | `20261022_140335`: `goto PP_1` + `amt1_sort` (tags 1..6, `left_to_right`, 3 a bordo) = `missions/task_AMT1_pp1.yaml` | PP1 → FINISH | AMT1 **incompleta** (`nudge_falhou`, 1 cubo a bordo); navegação para FINISH **falhou**; árvore **FALHA** |
| 22/10 14:18–14:23 | 5 execuções (`289791`…`291040`) | mesma AMT1 | PP1 → FINISH | docking OK; AMT1 abortada em 0,6 s por `container_inconsistente` (o yaml de containers dizia que a tag_1 estava no container1); 1 chegou ao FINISH, 4 interrompidas por Ctrl-C |
| 22/10 15:41–15:47 | `bt_yaml_executor_302018` (launch 15:38, **já com o código do docking de `8214d30`**, commitado só em 23/10) | mesma AMT1 | PP1 → FINISH | docking com `yaw_nao_medido` (AMCL +15,5°); AMT1 viu só 4 de 6 tags (faltaram 4 e 6), 3 picks/3 places; árvore **SUCESSO** |
| 23/10 18:26–18:31 | `400827` (launch 18:24, código = `cf401be`, mapa com staging −0,25) | `20261023_182404` = BMT: WS5 → WS1 (tags 13, 17, 26) | WS5, WS1, FINISH | 3 picks + 3 places OK; **SUCESSO** |
| 23/10 18:34–18:40 | `404899` | idem BMT | WS5, WS1, FINISH | OK; **SUCESSO** |
| 23/10 18:47–18:54 | `409565` | `20261023_184620`: WS4 (pick 9) → WS6 (place 9, pick 15) → WS3 (place 15, pick 6) → WS4 (place 6) | WS4, WS6, WS3, WS4, FINISH | todas as ações "concluídas com sucesso"; docking torto na WS6 (+8,7°) e **WS3 com y +0,304 m / yaw +0,319 rad pelo AMCL**; **SUCESSO** |
| 23/10 19:00 | `414307` | `20261023_190022` | — | Ctrl-C antes de começar |
| 23/10 19:01–19:11 | `414382` | `20261023_190056`: WS6 (pick 3, 27) → WS4 (place 3, 27; pick 5) → WS5 (place 5; pick 24) → WS1 (place 24; pick 18) → WS6 (place 18) | WS6, WS4, WS5, WS1, WS6, FINISH | todas as ações OK; WS6 com **−12,7°** (19:01) e **+7,3°** (19:10) após a correção; **SUCESSO** |

Entre as runs de 23/10 o operador mandou o robô de volta ao START por goal do RViz
(18:32, 18:41, 18:55, 19:12): as quatro voltas precisaram de recovery (`backup`) e
uma delas de cancelamento + replanejamento (planner a 0,5 Hz) — **FATO**, fora da
missão, registrado por completude.

### A.2 Os três dockings que liberaram a manipulação com a pose fora do esperado (FATOS)

**(1) PP1, 22/10 15:42:17–15:42:29** (`python3_301343`, `bt_yaml_executor_302018`)

```
15:42:17 refino face: dist=0.753 yaw_rel=9.5  largura=0.82 residuo=54 mm -> descartado (face nao-plana)
         refino face: dist=0.746 yaw_rel=5.2  largura=0.80 residuo=36 mm -> descartado
         [PP1] esquadro por AMCL: +7.7 graus (refino sem face)
15:42:19 refino face: dist=0.738 yaw_rel=-10.9/-12.8 (2 leituras concordam) -> yaw esquadrado -12.8
         [PP1] lateral(staging): centro sem confianca — nao mexo
15:42:21 refit(reto) a 0.579: yaw_rel=-15.3 (2 leituras) -> esquadro por LiDAR -> apos o giro -1.6
15:42:25 reto: parou em front=0.369
15:42:27 centro: 0/5 medidas validas — largura 0.29 fora de [0.48,0.95]
15:42:29 [PP1] yaw_nao_medido: 1/5 fits confiaveis da face; AMCL diz +15.5 graus
         [PP1] pronto: front=0.369 yaw_face=? — yaw_nao_medido
15:42:29 go_to_ws_bt: [PP1] alinhado (erro x=-0.011 y=0.084 yaw=0.271)  ->  pronto para manipulacao.
15:42:55 amt1: varredura j1: faltam [4, 6] — 4 vistas a esquerda, 0 a direita; paradas -30/+30/-60/+60 nao acharam
15:46:38 amt1: INCOMPLETA: ordenadas as tags vistas; faltaram [4, 6]; verify tag_3 erro 45.7 mm, tag_5 26.4 mm
```

Na run das 14:05 do mesmo dia, na mesma mesa, as tags 4 e 6 estavam nos slots mais
à direita (x = +0,076 e +0,215 m no frame do braço). O robô girou −12,8° −13,7°
(dois giros pelo LiDAR) sobre um esquadro de +7,7° pelo AMCL; o LiDAR, a 0,37 m,
viu uma "face" de 0,29 m (não é a mesa de 0,80). Quem estava certo (LiDAR ou AMCL)
**não dá para saber pelos logs** — ver hipóteses em §B.

**(2) WS3, 23/10 18:51:11–18:51:22** (`python3_400321`, `bt_yaml_executor_409565`)

```
18:50:48 bt_navigator: (-0.09,-5.52) -> staging (-0.03,-1.97): Goal succeeded (22 s)
18:51:11 refino face: dist=0.643 yaw_rel=-7.1/-7.3 largura=0.61 residuo=4 mm (2 leituras: 136.2 vs 135.9 no mapa; banco 137.2)
         refino aplicado: X +2.4 cm; yaw esquadrado -7.3; Y mantido do banco
18:51:14 [WS3] lateral(staging): -7.8 -> ? cm          (andou ~8 cm para a DIREITA; remedida dispersa +2.7..+6.8)
18:51:15 refit(reto) a 0.608: yaw_rel=-8.5 (2 leituras) -> esquadro por LiDAR -> apos o giro -1.7, front=0.616
18:51:18 reto: parou em front=0.374
18:51:20 centro: 0/5 medidas validas — nenhum segmento contem o eixo do robo
18:51:22 [WS3] yaw_nao_medido: 0/5 fits confiaveis da face; AMCL diz +18.3 graus
         [WS3] pronto: front=0.374 yaw_face=? — yaw_nao_medido
18:51:22 go_to_ws_bt: [WS3] alinhado (erro x=-0.006 y=0.304 yaw=0.319)  ->  pronto para manipulacao.
18:51:44 place tag_15: LiDAR frente da mesa em y=0.173 (502 pts); camera: frente NAO vista, bordas NAO vistas; 36 candidatos
18:51:47 "Achei lugar livre, entregando a esquerda" [-0.155 0.325]; solto com erro FK 0.7 cm  -> OK
18:52:05 pick tag_6: tag vista ha 50 s; varredura 4/4 -> achada em j1 +0.60 rad (34 graus); IK em [-0.454 0.212] so a 30 graus
18:52:50 pick OK (grasp 0.97 Nm)
```

`final_y_error` = +0,304 m significa, pelo AMCL, **alvo 30 cm à esquerda do robô**
(robô 30 cm à direita da pose gravada). Três indícios independentes apontam o mesmo
lado: (a) o LiDAR a 0,37 m não achou segmento de face que contivesse o eixo do robô;
(b) a câmera não viu a borda direita da mesa; (c) a tag_6 só apareceu com a j1 a +34°
e foi pega a x = −0,454 m (limite do alcance, à esquerda). Com o robô centrado, x = −0,45
seria a 5 cm da borda esquerda da mesa; com o robô 30 cm à direita, seria o meio da mesa.

**(3) WS6, 23/10 18:49 (+8,7°), 19:01 (−12,7°) e 19:10 (+7,3°)** (`python3_400321`)

Nos três dockings da WS6 o padrão é o mesmo:
`refino: residuo alto (face nao-plana?)` (54–56 mm, largura 0,74–0,76) →
`esquadro por AMCL` (−10,8 / −15,3 / −14,5°) → novo fit `face larga demais`
(1,11–1,18 m, yaw_rel 25–31°) → `lateral(staging): obstaculo na faixa lateral` ou
`largura 0,46–0,48 fora de [0,48, 0,95]` → `refit(reto): face estreita demais`
(0,46–0,48) → reto até 0,37 → `yaw_face` 7,6–8,9° > 5,2° → recuo + refit(correção)
(às 19:01 o fit de 0,50 m deu +7,4° e o giro levou a face a −0,8°; **depois do novo
reto a medida final foi −12,7°**; às 19:10 o refit da correção foi descartado por
resíduo 28 mm) → `YAW_FORA_DA_TOLERANCIA` → goal **sucede** (`final_yaw_strict=false`)
→ `go_to_ws_bt: alinhado ... yaw=0.152 / -0.222 / 0.127` → **pronto para manipulação**.
As manipulações na WS6 terminaram todas com "concluído com sucesso" (place 9, pick 15,
picks 3 e 27, place 18).

A WS6 (dock `(-0.16, -5.71)`, yaw −92,7°) tem a PP1 (`(-0.40, -5.18)`, yaw −177°) a
0,58 m de distância: do staging da WS6 o LiDAR enxerga a face da WS6 **e** o lado da
PP1 no mesmo plano — é o que os fits "largos" (1,1 m, 25–31°) e "não-planos" mostram.

### A.3 Os outros eventos de 22/10 (FATOS)

- **14:12:10 — verificação da AMT1**: tag_1 no slot 0 com erro 26,8 mm (> 15 mm) →
  `ajuste`: re-pick OK (14:12:57), re-place por `stack_pre_approach` →
  `unreachable_check` → cubo devolvido ao container (14:13:43) → `nudge +0.10` →
  recuo de 7 cm (front 0,359 < 0,36) → lateral abortado por `side_blocked` após
  +10,8 cm (14:13:49) → segundo `nudge +0.10` abortado por `side_blocked` após
  +13,9 cm (14:14:36) → `INCOMPLETA (nudge_falhou)`, tag_1 a bordo, base deslocada
  +0,25 m para a esquerda.
- **14:14:40–14:14:57 — FINISH falhou**: undock (recuo 0,35 m a partir da pose
  deslocada: robô em `(-0.15, -5.49)`), `NavigateToPose (0.84, -6.94)`:
  `compute_path_to_pose Aborting` 8 vezes em 7 s, `backup` ×2 + `wait`,
  `collision_monitor: approach` → `Goal failed` → `[FINISH] navegacao pura falhou` →
  **árvore FALHA**. Ctrl-C do launch às 14:24.
- Nos 5 dockings seguintes na PP1 (14:19–14:23) o fit do LiDAR foi bom
  (resíduo 3–7 mm, largura 0,63–0,64) e o yaw final ficou entre −7,2° e +4,8°.

### A.4 Tabela de reconstrução

| Etapa | Esperado | Observado (FATO) | Evidência | Subsistema | Possível causa | Confiança | Teste para confirmar |
|---|---|---|---|---|---|---|---|
| Nav2 até a staging (todas as estações) | chegar a ≤ 0,10 m / 0,20 rad do staging | `Goal succeeded` em todos os 20 gotos de 22–23/10; 3 vezes com um `compute_path_to_pose Aborting` + limpeza de costmap no meio (19:08 WS1, 19:11 FINISH) | `bt_navigator_*`, `planner_server_*` | Nav2 | — | alta | — |
| Nav2 FINISH após nudges na PP1 (22/10 14:14) | chegar ao FINISH | planner abortou 8×, 2 backups, `Goal failed` | `bt_navigator_278234`, `planner_server_278231`, `behavior_server_278233` | Nav2/costmap | HIPÓTESE: footprint dentro da zona inflada (0,55 m) entre PP1 (0,6 m à frente) e WS6 depois de 0,25 m de deslocamento lateral; "start occupied" | média | rosbag de `/global_costmap/costmap`, `/scan`, `/tf` repetindo a sequência (nudge +0,25 na PP1 → FINISH) |
| Refino LiDAR no staging | 2 fits confiáveis da face (0,6–0,8 m, resíduo ≤ 20 mm) | OK em WS1/WS3/WS4/WS5 (resíduo 3–7 mm); **descartado em todos os dockings da WS6 e na PP1 15:42** (resíduo 36–56 mm; largura 0,74–0,82; depois 1,1–1,2 m) | `python3_301343`, `python3_400321` | `dock_align_node._refine_target_from_scan`/`_fit_face` | HIPÓTESE: vizinho (PP1 ao lado da WS6) e/ou objeto a 5–10 cm do chão na frente da mesa | alta (WS6) / média (PP1) | gravar `/scan` parado no staging da WS6 e da PP1 e plotar os pontos no frame da base |
| Esquadro por AMCL (sem fit) | girar só se o AMCL merecer confiança | WS6: −10,8 / −15,3 / −14,5°; PP1 15:42: +7,7° — em todos os casos o LiDAR mediu depois o oposto ou um valor muito diferente | idem | `_run_attempt` (bloco "esquadro por AMCL") | HIPÓTESE: AMCL com erro de yaw de 10–15° perto dessas mesas (mapa × mesa física), como já visto na WS1 em 21/10 | média | comparar `/amcl_pose` com o yaw da face pelo LiDAR numa docagem gravada |
| Lateral (Y) no staging e na mesa | centrar na face | 23/10: em 13 dockings, 6 "centro sem confiança/disperso" na mesa; "Y mantido do banco" sempre | idem | `_face_center_from_scan`, `lateral_adjust` | FATO de projeto: Y não é observável na face reta (ADR-05); o Y final herda a tolerância do Nav2 (0,10 m) + AMCL | alta | medir com trena o desvio lateral real em 5 dockings e comparar com `final_y_error` |
| WS3 18:51 — pose final | robô centrado e perpendicular | AMCL: y +0,304 m, yaw +18,3°; LiDAR: sem face no eixo do robô; braço achou a tag a −0,454 m | §A.2(2) | docking + localização | HIPÓTESE principal: robô **fisicamente** ~0,3 m à direita (staging com erro + lateral(staging) −8 cm com sobrecurso + rumo do reto após esquadro contra face errada). Alternativa: salto do AMCL ao chegar na mesa (3 indícios contra) | média | rosbag (`/scan`, `/amcl_pose`, `/odom`, `/tf`) + trena na pose final na WS3 |
| Validação final de yaw (dock_align) | medir yaw da face; fora → corrigir uma vez | WS6 19:01: +7,6° → correção → **−12,7°** (piorou); 18:49: +8,9 → +8,7; 19:10: +8,4 → +7,3 | `python3_400321` | `refit_square("correcao")`, `_measure_face_yaw` | HIPÓTESE: o fit "estreito" (0,46–0,50 m) e o fit leve do muro medem superfícies diferentes (PP1 ao lado) | média | mesmo rosbag da WS6 |
| Liberação da manipulação (GoToWS) | só com pose validada | `pronto para manipulacao` com y +0,304 m / yaw 0,319 rad (WS3), yaw 0,271 rad (PP1), −0,222 rad (WS6) | `bt_yaml_executor_*` | `manip_bt/go_to_ws_bt.cpp` | **COMPROVADO**: o nó só testava `result->success`; `final_*_error` e a `message` (`yaw_nao_medido`, `yaw_fora_da_tolerancia`) eram apenas logados | comprovada | `test_pose_gate` (gtest) + `run_pose_gate_scenario.sh retry/strict` |
| Base parada antes do pick/place | velocidade zero por uma janela | nenhuma checagem no executor (o `dock_align` assenta pelo scan; o ESC retém a referência ~0,5 s) | código | `go_to_ws_bt.cpp` | FATO (lacuna); sem evidência de que tenha causado falha nestas runs | — | `run_pose_gate_scenario.sh moving/no_odom` |
| Manipulação nas poses tortas | — | place/pick "concluídos" na WS3 (y 0,30?) e WS6 (−12,7°); na PP1 15:42 faltaram as 2 tags da direita | logs de `mtc_*`, `amt1_sort` | braço/visão | o pick é guiado pela tag (varredura ±60° da j1) e o place lê a frente da mesa pelo LiDAR: toleram vários graus; a PP1 perdeu o lado direito | alta (fato) / média (causa) | — |
| Fim da missão | FINISH | 22/10 14:14 FALHA; demais SUCESSO | `bt_yaml_executor_*` | — | — | alta | — |
| Intervenção humana | — | Ctrl-C em 4 runs de 22/10 (14:20–14:23) e em 23/10 19:00; voltas ao START por RViz entre as runs | `bt_yaml_executor_*`, `bt_navigator_400312` | — | FATO | alta | — |

**O primeiro momento em que o comportamento esperado deixou de acontecer**, em cada
caso: (1) PP1 15:42:17 — refino descartado por face não-plana; (2) WS3 18:51:14 —
lateral(staging) com remedida dispersa após andar 8 cm, seguido de esquadro contra
uma face que às 18:51:20 já não aparecia no eixo do robô; (3) WS6 — refino
descartado em todos os dockings (vizinho). Em todos: 18:51:22 / 15:42:29 / 19:01:57 —
o GoToWS liberou a manipulação sem olhar os erros.

**Informação que falta para fechar a ordem exata**: o `/scan` de cada docking (o que
o LiDAR via a 5–10 cm do chão), a pose real do robô (trena ou vídeo da execução) e a
narração do operador (qual estação ele considera "o incidente").

---

## B. Diagnóstico técnico (por probabilidade e criticidade)

1. **COMPROVADO — orquestração: a manipulação começava com qualquer `success` do
   docking.** `GoToWSBT::onRunning` (caso `kAligning`) ignorava `final_x/y/yaw_error` e
   a `message` do `AlignToDock`. O próprio `dock_align_node` já avisava
   (`yaw_fora_da_tolerancia`, `yaw_nao_medido`) e, por decisão de 8214d30
   (`final_yaw_strict=false`), sucedia. Resultado: 4 manipulações em poses
   fora de qualquer tolerância em 2 dias (WS3 +30 cm/+18°; PP1 +15,5°; WS6 −12,7° e
   +8,7°). Criticidade alta: na WS3 o braço trabalhou a 45 cm de lateral; na PP1 a
   AMT1 perdeu 2 de 6 cubos. **Corrigido** (§D/§E).
2. **FATO/COMPROVADO — não havia condição de parada antes da percepção/manipulação.**
   Sem evidência de dano nestas runs, mas é a lacuna que o item 7 pede fechada.
   **Corrigido** com janela de odometria, sem atraso fixo.
3. **PROVÁVEL — Y (lateral) do docking é fracamente controlado.** O refino corrige X e
   yaw; Y "mantido do banco" (ADR-05). O centro da face só corrige quando o fit é
   confiável e, em 23/10, metade das medidas na mesa foi rejeitada. O Y final é a
   soma: tolerância da staging (0,10 m) + erro do AMCL + sobrecurso do lateral
   (até +40 % no ESC, visto nos nudges de 22/10: pedido 10 cm, andou 13,9) + desvio
   do rumo no reto quando o esquadro usa uma face errada. Explica a WS3 sem precisar
   de salto do AMCL. Não corrigido no `dock_align` (precisa de `/scan` gravado para
   tunar); o gate novo segura a manipulação acima de 0,20 m.
4. **PROVÁVEL — geometria WS6/PP1 (vizinhas a 0,58 m) derruba o fit do LiDAR.** Todos
   os dockings da WS6 tiveram refino descartado, esquadro pelo AMCL de 11–15° e
   yaw final de 7–13°. O `staging_x_offset −0,25` (c402ebf) não resolveu a WS6. Causa
   provável do "docking torto" se o relato do operador for sobre a WS6. Precisa de
   dados (§G) antes de mexer em `refine_*`/`center_*` ou na pose do dock.
5. **PROVÁVEL — na PP1 às 15:42 havia algo não-plano na frente da mesa** (resíduo 54 mm
   a 0,75 m; "face" de 0,29 m a 0,37 m). Nas 6 outras docagens da PP1 do mesmo dia o fit
   foi limpo. Objeto/pé/caixa a 5–10 cm do chão é a hipótese; sem `/scan` não dá para
   afirmar. Consequência: yaw incerto (LiDAR −1,6° × AMCL +15,5°) e 2 tags fora do
   campo da câmera mesmo com varredura de ±60°.
6. **PROVÁVEL — FINISH falhou (22/10 14:14) por planejamento a partir de uma pose
   dentro da zona inflada** após os nudges (+0,25 m) na PP1. O executor seguiu
   corretamente para a FALHA; o que falta é uma recuperação (voltar ao staging
   gravado antes de navegar). Não corrigido nesta entrega.
7. **FATO — AMT1 22/10 14:12: verificação achou tag_1 26,8 mm fora; o re-place foi
   "inalcançável" e os dois nudges para a esquerda bateram em `side_blocked`.** A faixa
   lateral de 0,50 m, com o robô já deslocado para a borda esquerda da mesa de 0,80 m,
   enxerga o que está além da mesa. Comportamento de proteção correto; a estratégia
   de ajuste (deslocar a base para re-colocar 1 cubo 2,7 cm fora) é discutível, mas
   fora do escopo deste incidente.

**Causa principal com evidência suficiente**: a nº 1 (liberação sem validação).
As causas nº 3–5 explicam *por que* a pose saiu errada em cada mesa, mas cada uma
precisa de um `/scan` gravado para virar "comprovada".

---

## C. Auditoria do código

| Arquivo | Classe/função | Parâmetro | Comportamento (antes) | Por que produz o erro | Correção |
|---|---|---|---|---|---|
| `src/manipulation/manip_bt/src/go_to_ws_bt.cpp` | `GoToWSBT::onRunning` (`kAligning`) → `finishSuccess` | — | `if (!success) fail(); log(erros); finishSuccess(true)` | manipulação começa com y 0,30 m / yaw 18° | gate de pose (`onAlignResult`), re-dock, `failAtDock`, checagem de parada (`onStopCheckTick`), resumo por etapa |
| `src/manipulation/manip_bt/src/go_to_ws_bt.cpp` | `Session<ActionT>` / `startStagingNav`, `startAlign`, `startUndock` | — | a sessão não era limpa antes de um novo goal | com re-dock na mesma execução o `poll` devolvia o `result_future` antigo (visto no cenário: 2º align "durou" 0,1 s e repetiu a pose ruim) | `session.reset()` em cada `start*` |
| `src/manipulation/manip_bt/src/bt_yaml_executor.cpp` | `main` (blackboard) | `align_timeout`, `skip_unreachable_station`… | sem limites de pose | — | novos `pose_gate_*`, `stop_check_*`, `odom_topic` |
| `src/caramelo_navigation/scripts/dock_align_node.py` | `_run_attempt` bloco (D) `final_yaw_decision` | `final_yaw_strict=false`, `default_yaw_tolerance=0.09` | fora da tolerância após 1 correção → `success=true` + `message` com prefixo | o cliente precisa ler a `message`/erros — o GoToWS não lia | (mantido; é decisão de competição) log do Y pelo AMCL nas linhas `pronto:` e `yaw_nao_medido` |
| `dock_align_node.py` | `_measure_face_yaw`, `_face_center_from_scan` | `center_samples=5`, `final_yaw_spread_max=0.07`, `center_width_min/max=0.48/0.95` | sem fit confiável → `yaw_nao_medido` (AMCL só informativo), centro "sem confiança" → Y não corrigido | na WS3 0/5 fits; na PP1 largura 0,29; na WS6 0,46–0,48 | sem mudança (precisa de scan gravado) |
| `dock_align_node.py` | `_refine_target_from_scan` | `refine_max_face_width=0.90`, `refine_max_residual=0.02`, `refine_half_width_y=0.45` | "Y mantido do banco" sempre | Y herda Nav2+AMCL | sem mudança (ADR-05); documentado |
| `dock_align_node.py` | `_run_attempt` "esquadro por AMCL" | `rotate_min_front=0.46`, `yaw_coast_est=0.10` | gira pelo AMCL quando não há fit (exceto "face larga") | na WS6/PP1 girou 8–15° contra o que o LiDAR mediu em seguida | sem mudança; candidato a desligar por mesa depois de medir (§G) |
| `src/caramelo_navigation/launch/docking_server.launch.py` | `dock_align` params | `final_yaw_strict`, `refine_refit_front_m=0.55`, `hard_min_face_dist=0.35` | — | — | sem mudança |
| `src/caramelo_navigation/config/controller_server.yaml` | `general_goal_checker` | `xy_goal_tolerance=0.10`, `yaw_goal_tolerance=0.20` | staging aceita a 10 cm / 11,5° | define o piso do erro de Y e a origem do limite duro de yaw (0,20 rad) do gate | sem mudança (justificado nos comentários do yaml) |
| `controller_server.yaml`, `planner_server.yaml` | costmaps | footprint `0.395/-0.285 × ±0.195`, `inflation_radius=0.55` | — | com o robô a 0,6 m da PP1 após os nudges, o planner abortou (14:14) | sem mudança (não reduzir margens) |
| `src/caramelo_mapping/maps/arenalegal_2026/docking.yaml` | docks WS6/PP1 | `staging_x_offset −0,25 / −0,35` | WS6 e PP1 a 0,58 m uma da outra | fit do LiDAR mistura as duas | medir antes de regravar (§G) |
| `src/caramelo_msgs/action/AlignToDock.action` | result | `message`, `final_yaw_error` | contrato: prefixos `yaw_fora_da_tolerancia`/`yaw_nao_medido`; `final_yaw_error` = LiDAR ou AMCL conforme o prefixo | — | sem mudança de interface; o GoToWS passa a interpretar (`classifyAlignMessage`) |
| `src/manipulation/manip_bt/test/fake_nav_actions.py` | `_execute_align`, `_publish_odom` | `align_bad_count`, `align_bad_*`, `odom_publish`, `odom_moving_s` | fake sem `/odom` e sempre com pose boa | — | novos parâmetros para os cenários do gate |

Verificado também (sem defeito encontrado nestas runs): tratamento de ±π no
`dock_align` (`_normalize_angle` em todas as diferenças de yaw; a WS4 a 179° e a PP1 a
−177° docaram corretamente em 7 ocasiões), cálculo da staging (`stagingPose` =
dock − offset ao longo do yaw; conferido com os logs `navegando ate a staging`),
cancelamento de goals no Ctrl-C (`GoToWS interrompido — goals cancelados` em todas as
interrupções), timeouts das actions (nenhum estourou).

---

## D. Plano de correção

**Imediatas (implementadas nesta entrega)**

1. Gate de pose no `GoToWS` com limites duros justificados pela geometria
   (`include/manip_bt/pose_gate.hpp`): |y| ≤ 0,20 m, |yaw| ≤ 0,20 rad, |x| ≤ 0,10 m.
   Fora: **re-dock** (undock → staging → align) 1 vez; ainda fora: `pose_gate_strict`
   (default **true**: falha a estação, que a missão pula via `skip_unreachable_station`,
   com voz "Estação X torta"; `false` reproduz o comportamento antigo com aviso).
2. Checagem de parada por odometria antes de liberar a manipulação (`StopDetector`):
   |v| ≤ 0,02 m/s e |w| ≤ 0,05 rad/s por 0,5 s contínuos, odometria com ≤ 0,5 s de
   idade, timeout 3 s; sem odometria → falha (`stop_check_require_odom=true`).
3. Logs de diagnóstico: fonte do yaw (LiDAR/AMCL), veredito do gate com os valores e os
   limites, tempo por etapa e número de dockings no "resumo da estação"; Y pelo AMCL
   nas linhas finais do `dock_align`.
4. Testes: 20 gtests do gate/parada, 2 pytests novos das decisões de 23/10, cenário de
   bancada `run_pose_gate_scenario.sh` (8 casos) com fakes em `ROS_DOMAIN_ID=99`.

**Recomendadas (próximas, com o robô)**

5. Validar os limiares no robô: rodar uma missão BMT com `pose_gate_strict:=false` e
   conferir, no log, `pose validada` / `ACEITA COM RESSALVA` / `FORA DOS LIMITES` e o
   tempo de `base parada` (esperado 0,5–0,8 s); só então deixar `strict=true`.
6. Validar o re-dock (undock a partir de uma pose deslocada 0,2 m) com alguém no botão.
7. WS6/PP1: gravar `/scan` no staging e na mesa; decidir entre regravar o dock da WS6,
   estreitar `refine_half_width_y`/`center_half_width_y` por mesa, ou desligar o esquadro
   por AMCL nessas duas.
8. Recuperação após falha de navegação com o robô colado/deslocado numa mesa: antes de
   `NavigateToPose`, voltar ao **staging gravado** (não ao "0,35 m atrás de onde estou").
9. Atualizar as 2 expectativas defasadas de `run_amt1_scenario.sh mission` (textos
   `alvo crescente`/`target=[crescente]` do formato 2026).

**Dependem de medições**

10. `nudge_odom_lateral_gain` e sobrecurso do lateral do `dock_align` (trena).
11. Ruído real de `/odom` parado (para confirmar 0,02 m/s / 0,05 rad/s) e do AMCL em
    yaw perto das mesas (para decidir se 0,20 rad é o limite certo com `yaw_nao_medido`).
12. Altura do plano do LiDAR versus mesas de 10 cm e objetos no chão (explica "face
    não-plana" e "face estreita").

---

## E. Implementação (diffs resumidos)

Commit na branch desta worktree (ver hash no relatório). Arquivos:

- `src/manipulation/manip_bt/include/manip_bt/pose_gate.hpp` (**novo**, header-only,
  sem ROS): `classifyAlignMessage`, `evaluatePoseGate` (`PoseGateLimits`,
  `PoseGateVerdict`), `StopDetector` (janela de velocidade por odometria, estados
  `sem_odometria`/`em_movimento`/`parado`). Limites e justificativa no cabeçalho.
- `src/manipulation/manip_bt/include/manip_bt/go_to_ws_bt.hpp`: estágio `kStopCheck`;
  `onAlignResult`, `startStopCheck`, `onStopCheckTick`, `failAtDock`, `speak`,
  `markStage`/`logStageSummary`; assinatura de `/odom` (`nav_msgs`); membros do gate.
- `src/manipulation/manip_bt/src/go_to_ws_bt.cpp`:
  - `onStart`: lê `pose_gate_*`, `stop_check_*`, `odom_topic` do blackboard e loga os
    limites por estação;
  - `kAligning` → `onAlignResult`: gate → (ok | ressalva) → `startStopCheck`;
    fora → re-dock (`startUndock` com `docked=true`) → `failAtDock` (strict) ou aviso;
  - `kStopCheck` tratado antes do deadline genérico; timeout → falha com diagnóstico
    (quem mais publica em `/mecanum_controller/reference`?) ou aviso
    (`stop_check_require_odom=false`);
  - `failAtDock`: marca `docked=true`/`current_dock_*` para o próximo GoToWS fazer
    undock (evita o Nav2 partir colado na mesa);
  - `session.reset()` antes de cada goal (bug do re-dock);
  - resumo por etapa (`undock | nav_staging | align | parada | total; dockings N`).
- `src/manipulation/manip_bt/src/bt_yaml_executor.cpp`: parâmetros `pose_gate_enabled`
  (true), `pose_gate_max_x` (0,10), `pose_gate_max_y` (0,20), `pose_gate_max_yaw` (0,20),
  `pose_gate_retries` (1), `pose_gate_strict` (false até validar no robô; depois true), `stop_check_enabled` (true),
  `stop_check_max_lin` (0,02), `stop_check_max_ang` (0,05), `stop_check_window_s` (0,5),
  `stop_check_timeout_s` (3,0), `stop_check_odom_max_age_s` (0,5),
  `stop_check_require_odom` (true), `odom_topic` (`/odom`).
- `src/manipulation/manip_bt/CMakeLists.txt`, `package.xml`: dependência `nav_msgs`;
  gtest `test_pose_gate`.
- `src/manipulation/manip_bt/test/test_pose_gate.cpp` (**novo**): 20 casos com os
  valores reais das runs.
- `src/manipulation/manip_bt/test/fake_nav_actions.py`: `align_bad_count`,
  `align_bad_x/y/yaw/message`, `/odom` falso (`odom_publish`, `odom_rate_hz`,
  `odom_moving_s`); `message` default igual à do nó real.
- `src/manipulation/manip_bt/test/pose_gate_actions.yaml`,
  `run_pose_gate_scenario.sh` (**novos**): cenário de bancada em `ROS_DOMAIN_ID=99`.
- `src/caramelo_navigation/scripts/dock_align_node.py`: Y pelo AMCL nas linhas
  `pronto:` e `yaw_nao_medido` (só log).
- `src/caramelo_navigation/test/test_nudge_math.py`: 2 testes das decisões finais de
  23/10 (WS6 e `yaw_nao_medido`).
- Docs: este arquivo; `docs/guia_missoes.md` §5.1/§7; `docs/docking_alinhamento_holonomico.md`;
  `README.md` (índice).

### E.1 Estatística do diff (`git diff --cached --stat`, antes da inserção desta tabela)

```
 README.md                                          |   2 +
 docs/docking_alinhamento_holonomico.md             |  25 ++
 docs/guia_missoes.md                               |  27 ++
 docs/investigacao_incidente_2026-10.md             | 384 +++++++++++++++++++++
 src/caramelo_navigation/scripts/dock_align_node.py |   8 +-
 src/caramelo_navigation/test/test_nudge_math.py    |  29 ++
 src/manipulation/manip_bt/CMakeLists.txt           |  10 +
 .../manip_bt/include/manip_bt/go_to_ws_bt.hpp      |  47 ++-
 .../manip_bt/include/manip_bt/pose_gate.hpp        | 243 +++++++++++++
 src/manipulation/manip_bt/package.xml              |   1 +
 src/manipulation/manip_bt/src/bt_yaml_executor.cpp |  43 +++
 src/manipulation/manip_bt/src/go_to_ws_bt.cpp      | 306 +++++++++++++++-
 src/manipulation/manip_bt/test/fake_nav_actions.py |  85 ++++-
 .../manip_bt/test/pose_gate_actions.yaml           |  15 +
 .../manip_bt/test/run_pose_gate_scenario.sh        | 219 ++++++++++++
 src/manipulation/manip_bt/test/test_pose_gate.cpp  | 275 +++++++++++++++
 16 files changed, 1700 insertions(+), 19 deletions(-)
```

Nenhuma interface de action/topic mudou; os parâmetros novos têm default que
preserva a missão (só o `pose_gate_strict=true` e o `stop_check_require_odom=true`
mudam o desfecho de uma estação, e apenas fora dos limites duros / sem `/odom`).

---

## F. Validação (comandos executados e resultados)

Tudo abaixo foi executado nesta worktree, em `ROS_DOMAIN_ID=99`, **sem tocar no robô**
(domínio 67). Scripts auxiliares usados para sourcear o ambiente ficaram fora do repo.

| Teste | Comando | Resultado |
|---|---|---|
| Build | `colcon build --symlink-install --packages-up-to manip_bt caramelo_navigation` | OK (7 pacotes, 1 min 13 s) |
| gtest gate/parada | `colcon test --packages-select manip_bt` → `test_pose_gate` | **20/20 passaram** |
| gtest fila de alcance (regressão) | idem → `test_reach_queue` | **17/17 passaram** |
| pytest task_planner formato 2026 (regressão) | idem → `test_task_planner_formato2026` | **18/18 passaram** |
| pytest dock_align (funções puras) | `python3 -m pytest src/caramelo_navigation/test/test_nudge_math.py -q` | **120 passaram** (118 anteriores + 2 novos) |
| Cenário do gate (fakes) | `src/manipulation/manip_bt/test/run_pose_gate_scenario.sh all` | **8/8 casos OK**: `good`, `retry` (pose da WS3 no 1º align → re-dock → 2º bom → place), `strict` (sempre ruim → re-dock → estação pulada, missão segue), `lenient`, `moving` (odom em movimento 5 s → "AINDA EM MOVIMENTO", estação pulada), `no_odom` (falha) e `no_odom_lenient` (aviso), `disabled` |
| Cenário da fila de alcance (regressão) | `run_reach_queue_scenario.sh all` | **4/4 casos OK** (`main`, `fail_pick`, `unseen`, `fail_nudge`) |
| Cenário AMT1 `mission` (regressão) | `run_amt1_scenario.sh mission` | gate + parada OK na PP1 (`pose validada`, `base parada: 0.5 s`, `pronto`), AMT1 rodou até o fim; **2 verificações falharam por texto defasado do próprio script** (`left_to_right, max 3 a bordo` × log atual `left_to_right, alvo crescente, max 3 a bordo`; `expected=[...] direction=` × `expected=[...] target=[crescente] direction=`) — defasagem pré-existente desde o formato 2026, não relacionada |
| Linters do `colcon test` | `flake8`, `pep257`, `uncrustify` | falham **antes e depois** da mudança em arquivos não alterados (`task_planner.cpp`, `mission_map_config.cpp`, aspas duplas em todo o `fake_nav_actions.py`, etc.); nos arquivos novos/alterados o `uncrustify` passa (`pose_gate.hpp`, `go_to_ws_bt.cpp`, `test_pose_gate.cpp`) e o `bt_yaml_executor.cpp` só mantém 3 hunks pré-existentes |

**Não executado** (impossível aqui): qualquer teste no robô real; reprodução com
rosbag (não existe); transcrição do vídeo (não está no PC).

Reproduzir: `source install/setup.bash` da worktree/branch, depois os comandos da tabela.

---

## G. Pendências

1. **Vídeo com a narração do operador** — não está neste PC. Necessário para saber
   qual estação/execução o operador chama de "o incidente" e o que ele viu (robô
   torto? deslocado? bateu? ficou parado?). Pedir o arquivo (ou a transcrição) e a
   data/hora aproximada; cruzar com a tabela §A.1.
2. **Rosbag de uma docagem na WS3, na WS6 e na PP1** com `/scan`, `/tf`, `/tf_static`,
   `/odom`, `/amcl_pose`, `/mecanum_controller/reference`, `/cmd_vel`,
   `/manip/base_shift_total` e `/rosout` (câmera não é necessária; ~20 MB/min).
   Sem isso as causas nº 3–5 de §B continuam hipóteses.
3. **Medida com trena** da pose final do robô em relação ao centro da mesa em 5
   dockings (y real × `final_y_error`) — valida o limite de 0,20 m e o sobrecurso do
   lateral.
4. **Validação no robô das mudanças** (§D.5–6): primeiro com `pose_gate_strict:=false`
   e `stop_check_require_odom:=true`, lendo os logs `[WSn] pose ...` e
   `[WSn] base parada: ...`; depois `strict=true`. Conferir que o undock do re-dock a
   partir de uma pose deslocada é seguro.
5. **Ruído real de `/odom` parado e do AMCL em yaw** perto das mesas (§D.11).
6. **Altura do plano do LiDAR × mesas de 10 cm** e o que havia na frente da PP1 às 15:42
   de 22/10 (§B.5).
7. **FINISH após nudges** (22/10 14:14): recuperação para o staging gravado antes de
   navegar (§D.8) — não implementada.
8. **`run_amt1_scenario.sh mission`**: atualizar os 2 textos esperados (§F).
9. Memória do projeto: as runs de 23/10 (pós-8214d30) ainda não estavam registradas;
   este documento passa a ser a referência.
