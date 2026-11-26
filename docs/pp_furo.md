# Mesa de precisão (PP) — refeita do zero em 2026-11-16

Branch `mudanca_pp`. A PP antiga (encaixe pela tag + offset fixo, detector de furo por cor,
AMT1) foi removida; esta é a nova. Pedido do operador: verificar o furo e o centro dele, chegar
**aos poucos** recalculando o ponto, pensar a descida como **o cubo em cima do furo**, e descer
reto em z quando a câmera não ajudar mais.

## Como funciona

1. **Planejador/executor**: objeto que termina num slot da PP vindo de outra estação vira
   `place` com `at_tag: tag_<slot>` (o executor repassa em `stack_on`). Reordenar blocos dentro
   da própria PP numa task que não é a AMT segue como pick + place com `at_tag`; a task AMT vira
   `goto PP` + `amt1_sort` (seção AMT abaixo).
2. **Garra vazia, antes de pegar o bloco** (`stack_pre_observe*`): o braço olha a tag do slot de
   longe e de perto. De perto o furo aparece inteiro e o `pp_furo_node` mede o **centro do furo
   em relação à tag** (em mm, na própria imagem). Fica guardado.
3. **Com o bloco** (`placeOnPpSlot` no `mtc_place_action_node`), punho com a câmera virada para
   a frente:
   | Etapa (feedback) | Onde o cubo para | O que a câmera mede |
   |---|---|---|
   | `pp_aproximando_1` | 6 cm antes do furo, 6 cm acima | furo inteiro à vista |
   | `pp_aproximando_2` | 2,5 cm antes, 4 cm acima | furo meio tapado: vale tag + vetor guardado |
   | `pp_sobre_o_furo` | em cima, 2 cm acima | furo tapado: tag + vetor guardado |
   | `pp_corrigindo` (até 2×) | em cima | só se faltar mais de 1 mm ou 1° |
   | `pp_descendo` | desce reto em z | — |
   Em cada parada o alvo é recalculado: TCP **comandado** + quanto a câmera diz que o cubo tem
   de andar. A descida final usa a altura **vista** do fundo do cubo (o braço cede 7–13 mm com
   o cubo) e termina 5 mm abaixo do tampo. Sem medida nenhuma, desce reto pelo palpite da tag.
4. **Falhas**: slot não visto ou sem IK → o bloco volta ao pote de bordo e o executor tenta
   depois (replanejamento). Sem IK a 0° em cima do furo → fila de alcance (desloca a base).

## AMT (ordenar os blocos da PP)

Refeita no mesmo dia sobre este place: `amt_sort_node.py` + `amt_sort_core.py` (`manip_bringup`),
servidor `/amt1_sort`. Olha a fileira (três poses do braço), descobre quem está em qual slot pela
ordem dos blocos na fila e pela tag de slot colada no bloco (visível das poses laterais), planeja
as trocas e manda `/pick_tag` e `/place_tag` (`stack_on` = tag do slot); só diz que terminou
quando vê a mesa em ordem (rodadas de conferência). Detalhes e
parâmetros em [guia_missoes.md](guia_missoes.md) §3.6; bancada `manip_bt/test/run_amt_scenario.sh`.

**Base alinhada com o slot (2026-11-17)**: se a tag do slot estiver a mais de `pp_alinhar_base_m`
(8 cm) do eixo do robô, o place sai de garra vazia pela fila de alcance (`unreachable` + passo
sugerido = −x da tag, entre 10 e 25 cm) **antes de pegar o bloco do pote**; quem chamou (a AMT ou
a fila de alcance do executor) anda a base de lado e pede o place de novo, já alinhado. Na
tentativa final (`final_attempt`) o place segue de onde estiver. `0` desliga.

**Slot ocupado**: na olhada de garra vazia, se houver tag de bloco em cima do slot, o
`pp_furo_node` responde `ocupado=<id>` e o place não solta ali (`fail_reason: slot_ocupado`; o
bloco volta ao pote).

## A medida (`manip_bringup/scripts/pp_furo.py`, sem ROS)

- **Furo**: a homografia dos 4 cantos da tag leva a imagem a uma vista de cima em mm; o furo é
  o quadrado marrom/escuro colado na tag. Slot 104 em 16/11: centro (+0,3; −45,8) mm da tag,
  ±0,5 mm entre quadros e corridas; furo 45,0 × 44,2 mm (cubo 42 → ~1,3 mm de folga por lado).
- **Cubo**: PnP da tag dá o tampo no frame da câmera; o **fundo do cubo** é um ponto fixo no
  frame da câmera (`fundo_cubo_cam_mm`), calibrado pelos três places de 16/11 em que o cubo
  entrou (repetiu em 0,5 mm). O pé desse ponto no tampo menos o centro do furo é o erro, já em
  "lado" (j1) e "frente" (raio) pelo ângulo do punho.
- Não usa TF nem a calibração câmera→braço. Relógio de câmera adiantado não estraga a medida
  (imagem e tags são do mesmo quadro).

Serviço: `/manip/pp_furo/medir` (`std_srvs/Trigger`), slot em `/manip/pp_furo/slot`. Resposta:
`furo=visto|pela_tag; lat_mm=..; rad_mm=..; giro_graus=..; alt_mm=..; n=..; vetor_mm=..`.
Cada medida grava o quadro e os números em `~/.ros/pp_furo/<data>/`.

## Parâmetros (`place_action_server`)

`pp_slot_tag_ids` [101..107] · `pp_recuo_m` [0.06, 0.025, 0.0] · `pp_altura_m` [0.06, 0.04, 0.02]
· `pp_tolerancia_m` 0.001 · `pp_correcoes_max` 2 · `pp_entra_no_furo_m` 0.005 ·
`pp_tag_ate_o_furo_m` 0.046 (só aponta a 1ª parada; o valor real é medido) ·
`pp_alinhar_base_m` 0.08 (0 = não pede a base alinhada).
Nó `pp_furo`: `fundo_cubo_cam_mm`, `lado_tag_mm` 31.8, `angulo_ref_graus` 180, `amostras` 3.

## O que conferir no log

```
[PP] furo de tag_104 visto de garra vazia: INTEIRO, centro medido (...vetor_mm=+0.3,-45.8)
[PP] parada 1/3 (60 mm antes, 60 mm acima): furo=visto; lat_mm=...; rad_mm=+6x; ...
[PP] parada 3/3 (0 mm antes, 20 mm acima): furo=pela_tag; lat_mm=+0.x; rad_mm=+0.x; ...
[PP] descendo reto NN mm (fundo do cubo visto a NN mm do tampo) e soltando.
```
Na parada 1 o `rad_mm` deve dar perto de +60 (o cubo ainda está 6 cm antes); na 3, perto de 0.

## Verificado e não verificado (16/11)

- Verificado: medida contra 7 quadros reais (`test/test_pp_furo.py`, 8 testes); nó de ponta a
  ponta com quadros reais em domínio de teste; planejador (34 testes); compilação do place.
- **Não verificado**: o caminho novo do place nunca rodou no robô. Outros slots além do 104.
  A pega do cubo torta na garra não é medida (o ponto do fundo do cubo é fixo).
- Recalibrar `fundo_cubo_cam_mm` se a câmera ou a garra forem trocadas: com um place que
  entrou, é o centro do furo (frame da câmera) + 15 mm na normal do tampo, na parada final.

## Furo pela borda (2026-11-26, PP da arena)

A PP da arena tem o **fundo do furo claro** (não marrom), furo de ~46 mm e uma faixa de tampo
de ~10 mm entre a tag e o furo: centro do furo a ~55 mm do centro da tag (no laboratório,
45,8 mm), medido numa foto de celular. A máscara de cor dava `sem_furo` e escolhia a
orientação ao acaso. `FuroCfg.metodo` (parâmetro `metodo` do `pp_furo_node`):

- `cor`: só o método original (fundo marrom/escuro, furo colado na tag).
- `borda`: quadrado de `borda_lado_mm` (42–52) alinhado à tag com o maior contraste nas
  quatro bordas, miolo liso (`borda_interior_max`, para não achar o cubo, que tem tag em cima)
  e borda do lado da tag além do quadrado preto + 4 mm. Inteiro só se a pior borda passar de
  `borda_contraste_min` (8× o gradiente mediano; furos reais dão ≥ 20, falsos ≤ 4,4).
- `auto` (padrão): `cor`; se ela não achar o furo inteiro, `borda`.

Testes: `test/test_pp_furo.py` (foto da arena + não-regressão dos quadros do laboratório).
**Falta validar com a D455 na arena.** Se o furo continuar sem medida, o palpite da 1ª parada
é `pp_tag_ate_o_furo_m` do place (0,046; na arena ~0,055).

## Ajuste do cubo depois de soltar (2026-11-26)

Pedido do operador: depois de soltar, ver onde o cubo ficou e, se ficou fora do furo, empurrar com
os dedos (um fecha mais que o outro e a garra reabre para não pegar o cubo de volta).
Código: `ppAjustarCubo` em `mtc_place_action_node.cpp`, chamado depois da subida que segue a soltura.

1. Volta à 1ª parada do place (de lá a câmera vê o furo inteiro) e lê a tag do cubo e a do slot.
2. Furo = tag do slot + `pp_ajuste_tag_ate_o_furo_m` na direção do robô. Erro = cubo − furo nessa
   direção (+ = mais perto da tag do slot). O erro de lado (ao longo da fileira) também é medido.
3. Erro ≤ `pp_ajuste_tolerancia_m`: nada a fazer.
4. Fora da tolerância: gira o punho 90° em relação ao place (dedos ao longo de tag↔furo), abre a
   garra toda, desce com o TCP sobre o furo na altura da soltura.
5. Fecha primeiro o dedo do lado para onde o cubo escapou (`pp_ajuste_dedo_empurra`) e depois o
   outro, que fecha menos (`pp_ajuste_dedo_apoio`). Em seguida abre em `gripper_open` e sobe.
   O lado de cada dedo vem do TF de `finger_left`/`finger_right` depois do giro do punho.
6. Olha de novo para registrar o resultado (até `pp_ajuste_max` empurrões).

Cada volta empurra no eixo de maior erro. No eixo **de lado** (ao longo da fileira) o punho fica
como no place e a garra abre só até `pp_ajuste_dedo_aberto_lado` (= `gripper_open`): toda aberta
ela bate nos blocos vizinhos. Com abertura pequena, se o cubo escapou mais que a folga dos dedos,
o dedo pode descer em cima dele.

Parâmetros (todos valem ao vivo com `ros2 param set /place_action_server ...`):

| Parâmetro | Padrão | Para quê |
|---|---|---|
| `pp_ajuste_habilitado` | true | false desliga o ajuste |
| `pp_ajuste_tag_ate_o_furo_m` | 0.055 | centro da tag → centro do furo (arena; no laboratório ~0.046) |
| `pp_ajuste_tolerancia_m` | 0.006 | erro aceito sem empurrar |
| `pp_ajuste_max` | 2 | empurrões por place (um por eixo) |
| `pp_ajuste_dedo_aberto` | -0.994838 | garra toda aberta para descer em volta do cubo (eixo tag↔furo) |
| `pp_ajuste_dedo_aberto_lado` | -0.2265 | abertura no eixo de lado (não pode abrir toda: vizinhos) |
| `pp_ajuste_dedo_empurra` | 0.170 | até onde fecha o dedo que empurra (= `gripper_close`) |
| `pp_ajuste_dedo_apoio` | 0.0 | até onde fecha o outro (menos, para não prender o cubo) |
| `pp_ajuste_z_extra_m` | 0.0 | soma à altura da soltura na descida |
| `pp_ajuste_pre_lift_m` | 0.04 | pré-pose acima do furo |

No log: `[PP-AJUSTE] ... erro +12.3 mm na direcao da tag` e `fecho primeiro o dedo ESQUERDO (j6)`.
NÃO testado no robô: os valores dos dedos e a altura da descida precisam ser conferidos devagar.
