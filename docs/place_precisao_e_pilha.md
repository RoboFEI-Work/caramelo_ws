# Place de precisão e pilha — encaixe no slot (tags 101–107) e pilha sobre a tag 200 (2026-10-21)

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

## 3. Modo SLOT (encaixe de precisão)

Geometria (tudo em `manip_base_link`): a tag do slot está no **fundo** da cavidade, em `z_tag`;
o tampo fica `slot_cavity_depth_m` acima. O TCP do braço fica no plano da face de **cima** do cubo
carregado (convenção do pick), então o TCP de soltura é

```
z_tcp = z_tag + slot_cavity_depth_m + slot_drop_clearance_m + cube_height_m
```

(fundo do cubo `slot_drop_clearance_m` = 3 mm acima do tampo). Se (x, y) e yaw estiverem
alinhados, o cubo cai dentro ao abrir a garra.

A cavidade é **rasa** (operador, 2026-10-21: entre 3 e 6 mm, nunca confirmada; default 5 mm).
Logo um cubo encaixado e um apoiado na borda diferem em z só pela cavidade. A tolerância de
assentamento é cortada para **metade da cavidade** (`effectiveSeatTolerance`, ≈ 2,5 mm), para as
faixas "encaixado" e "na borda" não se sobreporem, e a borda só é decidida com **duas leituras**
(uma leitura ruidosa não dispara re-pega). Isso está no limite do que a tag resolve: ver a
calibração em §6.

Fluxo (braço em `pegar_obj` com o cubo):

1. **`slot_detecting`** — mediana da tag do slot (`stack_tag_samples` leituras). Não vista de
   `pegar_obj`: olha de `tag_direita` / `tag_esquerda` (como o container por cor). Não vista:
   o cubo volta ao container de bordo (`base_nao_vista`, `unreachable` com sugestão 0 — nunca cai
   na mesa).
2. Pré-alinhamento lateral se |x| > 10 cm (igual à pilha).
3. **`slot_aligning`** — pré-ponto TCP2 (garra ~13 cm acima do slot), re-leitura de perto, yaw
   dos dedos = yaw da tag alinhado ao múltiplo de 90° mais próximo do punho atual
   (`nearestQuarterTurn`).
4. **`slot_inserting`** — pré-pose `slot_pre_lift_m` (0,02) acima, descida **vertical** (escada
   `slot_tilt_ladder_deg` = [0, 15]). A pré-pose pode ficar em 15° com a descida final a 0°
   (`solveDropLadder`): o que importa é o cubo vertical ao abrir. Se a descida final só resolve
   a 15°, o nó avisa que o encaixe é de **baixa confiança**. O z do TCP recebe a compensação da
   inclinação (`tiltDropZCompensation`: a quina baixa do cubo desce `(w/2)·sin t − h·(1−cos t)`,
   4,0 mm a 15°), para a folga de 4 mm não virar zero. Abre, sobe.
5. **`slot_verifying`** — vai à pose de verificação (TCP2 sobre o topo esperado) e lê a tag do
   **próprio cubo** por mediana; se não decide, uma segunda leitura de mais alto
   (**`slot_reobserving`**, `slot_reobserve_extra_z_m`). Decisão (`decideSlotAction`, pura e
   testada), só pela **altura** da tag:
   - **encaixado**: `z_cubo` entre `z_tag + cube_height_m − slot_cavity_depth_m` e
     `z_tag + cube_height_m + slot_seat_tolerance_m` → sucesso, fala "Encaixei o cubo". O xy
     **não** decide: a tag do slot é lida do pré-ponto TCP2 e a do cubo da pose de verificação
     (configurações diferentes do braço, 1–2 cm de FK entre elas); um cubo assentado em z não
     pode estar fora da cavidade. Se `|xy| > slot_xy_tolerance_m`, só um aviso de diagnóstico;
   - **apoiado na borda**: `z_cubo >= z_tag + slot_cavity_depth_m + cube_height_m −
     slot_seat_tolerance_m` → **`slot_retry_N`**: re-pega o cubo onde a câmera o viu
     (`regraspCube`: TCP no plano da tag, mas **nunca abaixo da borda** — os dedos chegam ao
     tampo, não entram nele; dedos nas faces, fecha, confere esforço da garra), sobe, e solta
     de novo no próximo ponto da espiral (`spiralOffsets(slot_search_step_m,
     slot_search_radius_m)`: (0,0), (+s,0), (0,+s), (−s,0), (0,−s), (+s,+s)…) **mais** a
     correção pelo erro medido (onde o cubo caiu − onde o TCP foi mandado, limitada a
     `slot_correction_max_m`; yaw idem, ±10°). Até `slot_max_attempts` solturas; um ponto da
     espiral sem IK cai no ponto da primeira soltura;
   - **meio dentro / tombado** (z entre os dois) ou **abaixo do fundo** (leitura ruim):
     **nunca re-pega** (o TCP iria abaixo da borda e os dedos para dentro do tampo); segunda
     leitura e, se continuar, deixa o cubo onde está;
   - **não visto** (tag ocluída): segunda leitura de mais alto; se continuar, **não declara
     encaixe** nem re-pega às cegas: `fail_reason = slot_nao_verificado` (o cubo pode estar
     encaixado, mas o nó não viu).
6. Falha final: o cubo fica onde está, o braço **recolhe a `pegar_obj`** (`slot_retract`, j2/j3
   primeiro), fala "Nao consegui encaixar o cubo no slot" (ou "Nao vi o cubo depois de
   soltar"), `success=false`, `fail_reason = slot_nao_encaixou` | `slot_nao_verificado`. O
   container de bordo é marcado vazio (o cubo está na mesa) e a posição vai para a memória de
   slots (`slot_falhou:tag_10x` / `slot_nao_verificado:tag_10x`) para virar obstáculo.

Memória em sucesso: `slot:tag_10x` com a posição **vista** do cubo.

Guarda física: o nó **não** tem a mesa na planning scene, então a única proteção contra empurrar
a mesa de precisão é nunca comandar um z de re-pega abaixo da borda da cavidade — por isso só o
caso "apoiado na borda" re-pega.

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
| `slot_tag_ids` | [101..107] | — | ids que viram modo SLOT |
| `stack_reference_tag_ids` | [200] | — | ids que viram modo REFERÊNCIA |
| `slot_cavity_depth_m` | 0.005 | **sim** | fundo da cavidade (tag) → tampo (3–6 mm na mesa real) |
| `slot_drop_clearance_m` | 0.003 | sim | fundo do cubo acima do tampo ao soltar |
| `slot_pre_lift_m` | 0.02 | — | pré-pose acima da soltura (a 0,36 m, 0° só resolve a pré-pose com lift ≤ 0,02) |
| `slot_tilt_ladder_deg` | [0, 15] | — | 0 preferido na descida final; a pré-pose pode usar 15 |
| `slot_seat_tolerance_m` | 0.0025 | sim | tolerância em z do "encaixado" e do "na borda"; nunca passa de cavidade/2 |
| `slot_xy_tolerance_m` | 0.008 | sim | só diagnóstico: aviso de discrepância de FK entre as duas leituras |
| `slot_reobserve_extra_z_m` | 0.04 | sim | segunda leitura da tag do cubo de mais alto |
| `slot_search_step_m` / `slot_search_radius_m` | 0.003 / 0.009 | sim | espiral |
| `slot_max_attempts` | 6 | sim | solturas no máximo |
| `slot_use_measured_correction` / `slot_correction_max_m` | true / 0.02 | sim | correção pelo erro visto |
| `ref_drop_clearance_m` | 0.005 | sim | folga sobre a tag 200 |
| `stack_verify_after_release` | true | — | liga a verificação da pilha |
| `stack_verify_xy_tolerance_m` / `stack_verify_z_tolerance_m` | 0.012 / 0.012 | sim | tolerâncias da pilha |
| `stack_align_yaw` | true | — | alinhamento de 90° pelo punho atual |
| `stack_memory_fallback` | true | — | base pela memória |
| `stack_max_level` | 4 | — | só aviso |
| `precision_verify_timeout_ms` | 4000 | sim | espera pela tag nas leituras de verificação/memória (mais amostras na mediana) |

Parâmetros existentes reusados sem mudança: `stack_place_z_offset` (0.05), `stack_pre_lift_m`,
`stack_tilt_ladder_deg`, `stack_tag_samples/dwell/max_spread`, `stack_settle_ms`,
`place_pre_ik_tcp2_z` (pose de verificação), `cube_height_m` (0.042), `verify_grasp_effort`,
`container_search_lateral`, `slot_memory_*`.

### Como calibrar `slot_cavity_depth_m` e as tolerâncias

1. Meça a cavidade com paquímetro (fundo → tampo). Se a tag tiver espessura, o plano da tag é o
   fundo "visto": a profundidade efetiva é tampo − topo do adesivo.
2. Com o robô docado na mesa de precisão e um cubo **encaixado à mão** num slot, rode um
   `ros2 topic echo /tf` (ou o `vision_debug`) e anote `z` de `tag_10x` (slot vizinho vazio) e da
   tag do cubo: a diferença deve ser `cube_height_m` (≈ 42 mm). Repita a leitura umas 10
   vezes: se a dispersão passar de ± 2 mm, a leitura em z não separa encaixado de apoiado
   (a diferença entre os dois é a cavidade, ~5 mm). Nesse caso suba `stack_tag_samples`
   (mais detecções na mediana) ou baixe a pose de verificação (`place_pre_ik_tcp2_z`).
3. Repita com o cubo **apoiado por cima** (de propósito, desalinhado): a diferença deve subir
   para ≈ 42 + cavidade (≈ 47 mm). O valor medido vai para `slot_cavity_depth_m`;
   `slot_seat_tolerance_m` fica em metade disso ou menos.
4. `slot_xy_tolerance_m` não decide nada: é o limiar do aviso "encaixado em z mas dxy=... entre
   as duas leituras". Anote o dxy típico de cubos encaixados: ele é o erro de FK entre o
   pré-ponto TCP2 e a pose de verificação, útil para calibrar a FK, não o encaixe.
5. `slot_drop_clearance_m`: comece em 3 mm; se o cubo bater no tampo antes de abrir (erro FK na
   soltura subindo, log "FK na soltura"), suba para 6–8 mm; se cair torto por altura, desça. A
   compensação da inclinação já entra sozinha (log "compensacao de inclinacao +x mm").
6. Primeiro teste com `slot_max_attempts: 2` para ver o ciclo re-pega/solta acontecer uma vez e
   conferir o log "correcao medida: cubo caiu (dx dy) mm do mandado".

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
