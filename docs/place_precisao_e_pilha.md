# Pilha sobre a tag 200 (2026-10-21)

> **2026-11-16 (branch `mudanca_pp`, decisão do operador):** o encaixe na mesa de precisão
> (slots 101–107, cavidade, detector de furo, AMT1 antiga) foi **removido do código** e a mesa de
> precisão e a AMT foram refeitas: ver [pp_furo.md](pp_furo.md). Este documento vale só para a
> pilha; qualquer menção restante a slot/PP abaixo é histórica.

**Estado: implementado, compilado em build isolado e coberto por gtest (helpers puros).
NÃO validado no robô.** Roteiro de calibração na §6 e limitações na §8.

## 0. O que muda (30 segundos)

No formato 2026 da competição, as tags **101 a 107** e **200** não são cubos: são adesivos de
41 mm (mesmo tamanho das tags dos cubos) colados na mesa.

| Tag | Onde está | O que o robô faz |
|---|---|---|
| 101–107 | No **fundo** de cada cavidade da mesa de *precision placement* (cavidade com a forma do cubo, ~2 mm de folga e só **3–6 mm de profundidade**, como brinquedo de encaixe) | **Encaixa** o cubo: desce na vertical, solta, olha a tag do próprio cubo. Assentado em z = sucesso; apoiado na **borda** = re-pega (TCP nunca abaixo da borda) e tenta de novo com busca em espiral + correção pelo erro medido; meio dentro / não visto = não desce, deixa o cubo, avisa e recolhe o braço |
| 200 | No **tampo** da zona de empilhamento | **Primeiro cubo da pilha**: solta sobre a marca com folga (compensando a inclinação da ferramenta), confere pela tag do cubo e corrige uma vez se saiu do lugar. Marca não vista / fora de alcance: o cubo volta ao container de bordo — **nunca** cai na mesa comum |
| tag de um cubo (ex. `tag_4`) | Topo de um cubo já na mesa | **Pilha** como antes, mais: alinhamento de 90°, base pela memória quando a câmera não a vê, nível da pilha, verificação após soltar com uma correção |

O executor continua mandando só `goal.stack_on` (`tag_101`, `tag_200`, `tag_4`…); o nó de place
reconhece o tipo **pelo id** (parâmetros `slot_tag_ids` / `stack_reference_tag_ids`). Não há
campo novo na action `PlaceTag`.

A RealSense **não** dá profundidade útil a menos de 0,4 m, então tudo aqui é **AprilTag
(mediana de várias leituras) + FK do braço**. A verificação é sempre pela **tag do cubo que
acabou de ser solto**, lida de cima.

## 1. Arquivos

| Papel | Arquivo |
|---|---|
| Helpers puros (classificação, giro de 90°, espiral, assentamento, nível da pilha, decisão) | `src/manipulation/manip_task_execution/{include/manip_task_execution,src}/place_precision.{hpp,cpp}` |
| gtest dos helpers | `src/manipulation/manip_task_execution/test/test_place_precision.cpp` |
| Integração no nó de place | `src/manipulation/manip_task_execution/src/mtc_place_action_node.cpp` (`placeIntoPrecisionSlot`, `placeStackedOnFrame`, `verifyAndCorrectStackedCube`, `dropCubeAt` + `solveDropLadder`, `regraspCube`, `observePlacedCube`, `retractAfterLeavingCube`) |
| Conferência de alcance offline | `custom_ik_check` (lib/manip_task_execution do install) |

## 2. Como o nó decide o modo (`classifyStackTarget`)

```
stack_on vazio            -> place normal (slot, superfície, container por cor)
stack_on = tag_<n>, n em slot_tag_ids (101..107)      -> kSlot      (encaixe)
stack_on = tag_<n>, n em stack_reference_tag_ids (200) -> kReference (1º cubo da pilha)
qualquer outro (tag_4, tag_amt1_slot_2, ct_*)           -> kCube      (pilha de sempre)
```

Um frame que não casa `tag_<n>` é sempre `kCube` — o comportamento atual (AMT1 com
`tag_amt1_slot_k` e `place_joints` continua byte a byte).

## 4. Modo REFERÊNCIA (tag 200, primeiro cubo da pilha)

Mesmo fluxo de `placeStackedOnFrame` com `z_tcp = z_tag + ref_drop_clearance_m + cube_height_m`
(fundo do cubo 5 mm acima da marca) **mais** a compensação da inclinação da ferramenta
(`tiltDropZCompensation`, 4,0 mm a 15° e 4,9 mm a 30°: sem ela, a 30° a quina do cubo chegava
a 0,1 mm do adesivo antes de abrir), yaw alinhado à tag, varredura lateral se não vista de
`pegar_obj`. Depois de soltar e subir (`runPlaceCycle`), **`stack_verifying`**: lê a tag do cubo;
aceita se `|xy − xy_tag| <= stack_verify_xy_tolerance_m` e `|z − (z_tag + cube_height_m)| <=
stack_verify_z_tolerance_m`; fora → **`stack_recorrect`** (regras da §5); depois aceita com
aviso. Memória: `stack:tag_200` com o z do **topo** do cubo (visto, ou esperado se a tag não foi
vista).

Tag 200 **não vista** (nem de lado) ou **sem alcance**: a tag é o marco da pilha, então **não há
fallback na mesa comum** (a pilha inteira nasceria fora do marco e a cadeia `stack:tag_200` da
memória não existiria): o cubo volta ao container de bordo e o goal sai como `unreachable`
(bailout de alcance com sugestão de deslocamento quando a tag foi vista; sugestão 0 quando não).
Mesmo caminho que a precision placement (`PP_*`) já usava; `stack_fallback_to_table` continua
valendo só para `stack_on` de cubo em mesa comum.

## 5. Pilha de até 4 (stack_on = tag de um cubo)

Sobre o fluxo existente (`placeStackedOnFrame`, TCP = `z_base + stack_place_z_offset`):

- **Yaw**: com `stack_align_yaw` (default true) os dedos vão ao múltiplo de 90° do yaw da base
  mais próximo do punho atual (menor giro do cubo na garra). `false` = `computeWristForCubeYaw`
  de sempre (menor |q5|).
- **Base não vista** (ocluída pelo cubo na garra / fora do quadro): `stack_memory_fallback`
  usa o último registro `stack:` da memória da WS cujo `tag_frame` é a base
  (x + deslocamento acumulado da base, y, z = topo do cubo). Fala "vou empilhar pela memoria";
  as re-leituras seguintes esperam só `precision_verify_timeout_ms` e, se a tag aparecer de
  perto, ela vence. Sem yaw na memória: mantém a linha dos dedos.
- **Nível**: `stackLevelFromMemory` segue a cadeia `stack:tag_200 → tag_A`, `stack:tag_A → tag_B`…
  e loga "nivel do cubo novo: N"; acima de `stack_max_level` só avisa. O alcance real é a IK:
  alvo **e** pré-pose (`stack_pre_lift_m` acima) na escada `stack_tilt_ladder_deg`; nenhum
  resolve → `unreachable=true` + `suggested_base_shift_m` (`findBaseShiftForReach`, como a fila
  de alcance espera).
- **Verificação após soltar** (`stack_verify_after_release`, default true): xy e z do cubo solto
  contra base + cubo (`stack_verify_*_tolerance_m`); fora → **`stack_recorrect`**, uma vez, com
  regras pela **altura vista** (`pileFixFor`):
  - cubo **sobre a base** (`|dz| <= stack_verify_z_tolerance_m`), deslocado → re-pega e solta
    com a correção xy (erro medido, limitado a `slot_correction_max_m`);
  - cubo **tombado / caiu um nível** (dz fora da tolerância, até um cubo abaixo) → re-pega e
    solta de novo **no alvo original, sem correção** (o xy de um cubo caído não é erro
    sistemático, é onde ele parou);
  - cubo **abaixo do piso de re-pega** (`topo esperado − cube_height_m − tolerância`) → não
    desce até lá; fica onde está e a memória grava `stack_caiu:<base>` (obstáculo, fora da
    cadeia da pilha).
  O TCP da re-pega nunca vai abaixo desse piso. A memória `stack:<base>` é regravada com o topo
  visto — é o que o próximo nível usa quando a base ficar ocluída.

Quando a tag base **é** vista, o único efeito novo antes de soltar é o alinhamento de yaw
(desligável); a verificação acontece depois da entrega e nunca a reprova, só corrige.

## 6. Parâmetros (todos em `place_action_server`)

| Parâmetro | Default | Calibrar no robô? | Para que serve |
|---|---|---|---|
| `stack_reference_tag_ids` | [200] | — | ids que viram modo REFERÊNCIA |
| `ref_drop_clearance_m` | 0.005 | sim | folga sobre a tag 200 |
| `stack_verify_after_release` | true | — | liga a verificação da pilha |
| `stack_verify_xy_tolerance_m` / `stack_verify_z_tolerance_m` | 0.012 / 0.012 | sim | tolerâncias da pilha |
| `stack_align_yaw` | true | — | alinhamento de 90° pelo punho atual |
| `stack_memory_fallback` | true | — | base pela memória |
| `stack_max_level` | 4 | — | só aviso |
| `precision_verify_timeout_ms` | 4000 | sim | espera pela tag nas leituras de verificação/memória (mais amostras na mediana) |
| `stack_pre_observe_max_olhadas` | 3 | sim | 2026-11-19: olhadas de perto (garra vazia) na base antes de pegar o bloco; cada uma re-mira o pré-ponto TCP2 pela leitura anterior. 1 = uma olhada (antes). Lido a cada place |
| `stack_pre_observe_converge_xy_m` / `_z_m` | 0.008 / 0.012 | sim | duas leituras seguidas dentro disso = ponto CONFERIDO; esgotou as olhadas = usa a última, com aviso |

Parâmetros existentes reusados sem mudança: `stack_place_z_offset` (0.05), `stack_pre_lift_m`,
`stack_tilt_ladder_deg`, `stack_tag_samples/dwell/max_spread`, `stack_settle_ms`,
`place_pre_ik_tcp2_z` (pose de verificação), `cube_height_m` (0.042), `verify_grasp_effort`,
`container_search_lateral`, `slot_memory_*`.

## 7. O que ver no `vision_debug` e nos logs

- Estágios do feedback: `slot_detecting`, `slot_search`, `slot_pre_approach`, `slot_aligning`,
  `slot_inserting`, `slot_verifying`, `slot_reobserving`, `slot_regrasp`, `slot_retry_N`,
  `slot_retract`; pilha: `stack_verifying`, `stack_recorrect`, `stack_recorrect_drop`,
  `stack_verifying_2`.
- `[SLOT] slot tag_10x em [...] yaw=...; TCP de soltura ... acima (cubo + cavidade + folga)` —
  confira o z; `[slot_inserting] soltar em [...] (compensacao de inclinacao +x mm) ... final a
  0 graus, pre-pose a 15 graus`.
- `[SLOT] cubo tag_N visto em [...] (leitura k/2): dxy=.. mm, dz=.. mm do assentado, .. mm da
  borda -> ENCAIXADO | APOIADO NA BORDA | MEIO DENTRO / TOMBADO | leitura inconsistente` e
  `[SLOT] decisao apos a soltura k/N (leitura j): encaixado | re-pegar_na_borda |
  olhar_de_novo | deixar_onde_esta`.
- `[SLOT] re-pega na borda: TCP z=... (visto ..., borda ...; piso = borda)` — o z comandado
  nunca fica abaixo da borda.
- `[STACK] cubo visto z=... (topo esperado ..., piso de re-pega ...): corrigir_xy |
  soltar_no_alvo_original | nao_re-pegar`.
- `[SLOT] correcao medida: cubo caiu (+x +y) mm do mandado ... -> correcao acumulada` — esse
  número é o erro sistemático do braço naquela região; se for estável entre tentativas a
  correção converge em 1–2 solturas.
- `[STACK] nivel do cubo novo: N (M cubo(s) abaixo; tag base z=...; topo esperado z=...)`,
  `[STACK] base tag_X pela MEMORIA (...)`, `[STACK] cubo tag_N visto ... dxy dz`,
  `[STACK] decisao da verificacao: ...`.
- No `vision_debug` a tag do slot só aparece com o slot **vazio**; depois de soltar, só a tag do
  cubo. A pose de verificação deixa a câmera ~13 cm acima do topo esperado — se a tag não
  aparecer ali, o problema é oclusão pelos dedos abertos ou foco: tente `place_pre_ik_tcp2_z`
  maior (0.33–0.35).

## 8. Alcance da pilha e do slot (custom_ik_check, offline)

Com o modelo da IK e a convenção do nó (tampo = FK da pose − `cube_height_m`), a `Mesa15` fica
em z = −0,013 m e y = 0,362 m (`manip_base_link`). Pré-pose = alvo + `stack_pre_lift_m`, cujo
default **real é 0,02** (o 0,05 só aparece em docs antigas; nenhum yaml/launch o sobrescreve).
O n-ésimo cubo da pilha tem a tag da base em tampo + (n−1)·0,041; o TCP de soltura é
base + `stack_place_z_offset` (0,05) para n ≥ 2 e tag 200 + 0,047 para n = 1. Menor inclinação
da escada em que **alvo e pré-pose** resolvem (`custom_ik_check 0 <y> <z> tilt<t>`):

| Cubo novo | z alvo TCP | y = 0,36 (pose Mesa15) | y = 0,30 | y = 0,25 |
|---|---|---|---|---|
| 1 (sobre a tag 200) | 0,034 | 0° | 0° | 0° |
| 2 | 0,078 | 15° | 0° | 0° |
| 3 | 0,119 | 15° | 15° | 0° |
| 4 | 0,160 | **30°** | 15° | 15° |
| 5 | 0,201 | 30° | 30° | 30° |

`custom_ik_check shift 0 0.3616 0.1596 0.02` responde `alcancavel_agora=sim, tilt 30`.

Conclusões (corrigem a versão anterior desta seção, que usava lift 0,05 e contava cada nível com
um cubo a mais): **a pilha de 4 cubos é alcançável na distância de docking atual**, o 4º cubo
a 30°; docar mais perto (y ≤ 0,30) só baixa a inclinação (15° no 4º). 0° resolve o 1º cubo em
qualquer y e, a y ≤ 0,30, também o 2º. Não há o que decidir sobre docking para a pilha; o que
vale testar no robô é a precisão da soltura a 30° no 4º cubo.

Soltar inclinado desce a quina do cubo `(w/2)·sin t − h·(1−cos t)` a mais que na vertical
(4,0 mm a 15°, 4,9 mm a 30° para 42 mm). Na tag 200 e no slot o nó soma isso ao z do TCP
(`tiltDropZCompensation`) para manter a folga nominal (5 e 4 mm); na pilha de cubos (offset
50 mm = 42 + 8) a folga de 8 mm já cobre 30° (sobram ~3 mm) e nada mudou.

Slot (TCP = tag + 0,005 + 0,003 + 0,042, ≈ tampo + 0,045): a y = 0,36 o alvo resolve a 0°, mas a
pré-pose a 0° só com lift ≤ 0,02 (0,03 e 0,05 não resolvem a 0°; a y ≤ 0,34 resolvem). Por isso
`slot_pre_lift_m` = 0,02 e a pré-pose pode ficar em 15° com a descida final a 0°
(`solveDropLadder`). Com a compensação de 15° (+4 mm) o alvo continua resolvendo a 0°.

## 9. Limitações (sem validação no robô)

- Não testado com a mesa de precisão real: `slot_cavity_depth_m` = 5 mm é o meio da faixa que
  o operador informou (3–6 mm); as tolerâncias são chute informado (§6). Com cavidade tão rasa
  a decisão encaixado/apoiado pela altura da tag fica no limite do ruído da câmera: a borda só
  dispara re-pega com duas leituras concordantes.
- A tag do cubo precisa estar na face de **cima** (a verificação lê o topo). Cubo que cai de
  lado ou tombado dentro da cavidade dá "meio dentro / tombado" ou "leitura inconsistente" →
  o nó **não** re-pega (não desce abaixo da borda); deixa o cubo e falha com
  `slot_nao_encaixou`.
- A pose de verificação pode não enxergar a tag se os dedos abertos ocluírem; o nó tenta uma
  segunda leitura de mais alto e, se continuar, no slot falha com `slot_nao_verificado` (o cubo
  pode estar encaixado, mas não há como saber sem ver) e na pilha aceita "não verificado". O
  erro FK da soltura é só informativo: mede o rastreamento do braço, não onde o cubo está.
- Sem a mesa na planning scene: a proteção contra empurrar a mesa de precisão é só o piso de
  re-pega (borda da cavidade no slot; topo esperado − um cubo na pilha).
- A re-pega confia no esforço da garra (`verify_grasp_effort`); sem telemetria assume que pegou.
- `stack_arrival_tolerance_m` (conferência de chegada por FK) continua desligada; a verificação
  nova é independente dela.
- Pilha pela memória não tem yaw (a memória guarda só x, y, z).
- A estação ST (zona de empilhamento) precisa existir nos mapas/`service_areas.yaml`; o nó só
  trata o place.
