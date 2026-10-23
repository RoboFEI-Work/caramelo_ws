# Análise da run de 2026-10-21 — espaço livre na 2ª mesa e docking torto

Baseada nos logs dos nós em `~/.ros/log` (stack subiu às 19:36; missão BMT: WS5 → WS1,
3 cubos). Os números abaixo são dos logs, não do vídeo.

## 1. O que aconteceu, em ordem

| Hora (s) | Evento | Evidência |
|---|---|---|
| 2008–2015 | Docking na WS5: refino LiDAR descartado (resíduo 43 mm, face "0,87 m"), lateral no staging, reto, **pronto: front 0,367 m, yaw_face +2,6°** | `dock_align` |
| 2015–2127 | 3 picks na WS5 sem ajuste lateral | executor |
| 2155 | Docking na WS1: refino descartado (resíduo 61 mm, face **0,98 m** a 0,68 m) → **esquadro por AMCL +12,3°** | `dock_align` |
| 2156 | Novo fit LiDAR confiante (resíduo 13 mm, largura 0,64): **yaw_rel −15,1°**, mas "refino yaw: aguardando 2ª leitura concordante" → yaw **mantido** | `dock_align` |
| 2160–2162 | Reto até front 0,386; centro "largura 0,44 fora de [0,48, 0,95]" → não mexe; **pronto: yaw_face −19,7°**; executor: "alinhado (yaw −0,345)" e segue | `dock_align`, executor |
| 2179–2193 | Place 1 (tag_13) pela superfície: 3 quadros, 368 células "além", 1 reta; frente não vista → prior y=0,163; 139 válidas → 28 candidatos; soltou em [0,005, 0,335]; **OK** | place, surface |
| 2212–2231 | Place 2 (tag_17): vista 1 do `pegar_obj`: **tampo livre 200, fora 2480**; vista 2: frente vista a y=0,28 (12 cm além do prior) mas 1905 obstáculos; vista 3: 1611 fora, 2132 obstáculos, **0 candidatos** → fallback poses fixas → Mesa15.1 (**OK**) | place |
| 2250–2269 | Place 3 (tag_26): idem, 0 candidatos → Mesa15.2 (**OK**) | place |
| 2299 | Árvore terminou com SUCESSO | executor |

Os três cubos foram entregues (um pela superfície, dois pelas poses fixas). O que falhou
foi o planejador de superfície nas entregas 2 e 3, e o docking na WS1.

## 2. Problema 1 — por que "não há espaço livre" com a mesa meio vazia

### 2.1 Causa principal: profundidade abaixo do alcance mínimo da D455 (Hipótese A, confirmada)

Na observação 1 só 3 quadros entraram e 368 células foram marcadas "além do plano"; nas
observações 2 e 3, com 10–12 quadros, foram **1.781 e 1.733 células "além"** (= fora da mesa)
e **2.132 e 2.085 "objeto"** (contra 283 na observação 1), com **zero retas** de borda.
Nada mudou na mesa entre as observações além de um cubo a mais.

A câmera, em `pegar_obj`, está a ~0,31 m do tampo. A D455 não mede de forma confiável abaixo
de ~0,4–0,5 m: devolve valores maiores que o real (viram "além" → fora da mesa) e menores
(viram "acima" → objeto). O nó usa `depth_min_valid_m = 0,25` sobre a **medida**, que não
protege contra lixo de 0,45 m medido a 0,31 m reais. O quadro 1 escapou porque a profundidade
ainda não tinha chegado (graça de 1 s) e a classificação foi só por cor.

Resposta direta à pergunta do documento: **sim, a introdução da profundidade piorou** este
caso, porque foi aplicada numa distância em que o sensor não funciona. Ela continua útil para
diferenciar mesa de chão, mas só onde o alcance esperado ao plano for maior que o mínimo do
sensor.

### 2.2 Causa secundária: ilhas de "fora da mesa" dentro do tampo confundem as bordas (Hipótese B, parcial)

O planejador acha a borda lateral como "célula de tampo cujo vizinho em +x é fora". Com ilhas
de "fora" no meio do tampo (as da profundidade), ele achou **borda direita em x = −0,29 e
borda esquerda em x = −0,22** — as duas à esquerda do robô, a "direita" mais à esquerda que a
"esquerda". As paredes não estavam declaradas, então isso não eliminou candidatos por si só;
o que eliminou foi a distância mínima de 5,4 cm a qualquer célula "fora" (`d_off`) somada aos
2.000 falsos obstáculos.

Os filtros de contorno **não** são a causa: os filtros são por célula (distância a fora,
a objeto, a desconhecido, profundidade na zona), não por área mínima de contorno. O que
faltava era o planejador dizer **quantas células cada regra rejeitou** — hoje só "válidas 0".

### 2.3 Margens (Hipótese C): não são a causa

Folgas aplicadas: 2 cm da borda, 2 cm entre objetos, 10 cm de parede, zona de 2–20 cm,
mais 2,9 cm de raio do cubo e 0,5 cm de segurança. Aplicadas **uma vez**, no centro do cubo.
Na entrega 1, com classificação correta, 139 células válidas e 28 candidatos sobraram.

### 2.4 Transformações (Hipótese D): corretas no robô, mas o prior da frente estava errado

As tags viraram obstáculos em posições consistentes entre vistas (tag_13 em
[0,02, 0,37] nas três observações; os cubos soltos ficaram a ≤ 1,1 cm do alvo pela FK).
O problema geométrico foi outro: com a frente da mesa não vista, o planejador usa o prior
`y = 0,163` (robô a 0,38 m da face). Na WS1 o robô estava **girado 19,7°** e a câmera viu a
frente real em **y = 0,28–0,31**, 12–15 cm além do prior. Como `surface_front_prior_tolerance_m`
é 0,06, a frente vista foi **rejeitada** em favor do prior errado, e a zona de 2–20 cm foi
imposta no lugar errado da mesa.

## 3. Problema 2 — docking torto na WS1 e pré-docking longe

Cadeia observada:

1. **Staging a 0,35 m atrás da pose do dock** (`docking.yaml`): o robô chega a ~0,70 m da face.
   A essa distância o LiDAR enxerga "faces" de 0,87–0,98 m (mesa + vizinhos) com resíduo
   alto → o refino é **descartado** nas duas estações.
2. Sem refino, o nó faz o **esquadro por AMCL** (+12,3°). O mapa não bate com a mesa física
   (a leitura LiDAR seguinte, confiável, disse −15°).
3. O refino de yaw pelo LiDAR exige **duas leituras concordantes em fits confiáveis**
   (`refine_yaw_agree_tol`); o primeiro fit confiável veio só depois do esquadro e o nó não
   tirou uma segunda leitura: "yaw MANTIDO (portão)".
4. O "reto" e o retoque lateral não corrigem yaw. O nó declarou **pronto** com
   `yaw_face = −19,7°` porque o sucesso só valida a janela de distância frontal;
   `default_yaw_tolerance` (0,09 rad) existe mas **não é verificada** no fim.
5. O `GoToWS` só olha `success` e seguiu para a manipulação.

Relação com o Problema 1: **existe**. O docking torto deslocou a frente da mesa em 12–15 cm
no referencial do braço e inclinou a perspectiva (a frente vista tinha inclinação de 11,6°),
o que por si só já derrubaria a zona de manipulação. A profundidade ruim foi a causa
dominante nesta run, mas um docking de 20° derruba o planejador mesmo com a classificação
perfeita.

## 4. Legenda da imagem de depuração (`/manip/vision_debug/image`)

| Desenho | Cor | Significado |
|---|---|---|
| Tinta sobre o tampo | verde | célula livre |
| Tinta | vermelho (rosado sobre branco) | célula com objeto (cor não-branca ou profundidade "acima") |
| Tinta | cinza | fora da mesa (além de uma reta de borda ou profundidade "além") |
| Sem tinta | — | desconhecido (não visto, raio rasante, garra) |
| Contorno branco | — | polígono do tampo visto |
| Reta | amarela | frente da mesa (lado de fora = robô) |
| Reta | magenta (roxa) | fundo da mesa |
| Reta | ciano | laterais |
| Tracejado | laranja | zona de manipulação (2–20 cm da frente, 2 cm das laterais) |
| Quadrado | verde / amarelo | AprilTag vista (amarelo = leitura com correção de bits) |
| Caixa | vermelha / azul | container por cor (só durante o place) |
| Barra no topo | — | PICK/PLACE, ids vistos, idade do mapa e contagem livre/obj/fora, `bordas: ...` ou `sem frente`, última fala, fps |

"Bordas: nenhuma / sem frente" na barra significa que o nó de superfície não achou reta de
borda — exatamente o caso das observações 2 e 3 (`lines: 0`).

## 5. O que muda (implementado nesta revisão)

Visão e planejador:
- A profundidade só classifica um pixel se o **alcance esperado ao plano** for maior que
  `depth_min_reliable_m` (0,45 m, D455) — a medida sozinha não basta. Quadro cuja profundidade
  contradiz a cor em mais de uma fração do tampo é descartado com aviso.
- Ilhas de "fora da mesa" cercadas por tampo são removidas antes das bordas e das distâncias;
  borda direita só vale à direita do centro do tampo (e vice-versa).
- Frente vista pela câmera com suporte em duas vistas concordantes **vence o prior**, mesmo
  longe dele; o prior só entra quando a câmera não a vê.
- O planejador conta **quantas células cada regra rejeitou** (borda, objeto, desconhecido,
  zona, parede, IK) e o place loga isso; a barra do viewer mostra.
- Se as três vistas derem zero candidatos e a profundidade tiver sido usada, o nó replaneja
  **só com a cor** (os votos são guardados separados) antes de cair nas poses fixas.

Docking:
- O refino tira uma **segunda leitura imediatamente** quando a primeira é confiável, em vez
  de esperar o próximo goal; e refaz o fit mais perto da mesa (≈ 0,55 m), onde a face aparece
  isolada.
- **Validação final de yaw**: ao fim do reto, se |yaw_face| > tolerância, o robô recua até a
  distância de giro, esquadra pela face do LiDAR e aproxima de novo (uma vez). Se ainda assim
  ficar fora, o resultado diz explicitamente `yaw_fora_da_tolerancia` (com `strict` desligado
  segue com aviso; ligado, falha).
- Mapa (decisão do operador, aplicada em 2026-10-23): `approach_offset` 0,35 → 0,25 nas WS do
  `arenalegal_2026/service_areas.yaml` (fonte do sync) e `staging_x_offset` −0,25 no
  `docking.yaml` derivado; pré-docking a ~0,6 m da face. SH1 e PP1 continuam como estavam.

## 6. Arquitetura recomendada: híbrida, com a geometria no comando

A tarefa tem cubos de 41 mm com tag, mesa branca de geometria conhecida e câmera a 0,3 m. A
representação principal deve vir de **cor (tampo branco) + tags (obstáculos com raio
conhecido) + geometria (frente pelo LiDAR/docking, zona do rulebook)**. A profundidade entra
como **validação** só onde é confiável (chão e o que está além da mesa, a > 0,45 m), nunca
como fonte de "objeto" sobre o tampo a 0,3 m. Objetos sem tag continuam cobertos pela cor
(não-branco sobre o tampo = objeto).

Três estados, sempre distintos: **livre** (visto e branco), **ocupado** (objeto, tag, container,
memória), **desconhecido** (não visto) — e o desconhecido nunca vira livre.

## 7. O que ainda depende do robô

- Medir no D455 real a partir de que distância a profundidade bate com o plano
  (`depth_min_reliable_m`); o valor 0,45 é o da ficha técnica.
- Confirmar o yaw final em cada estação depois da correção (linha `pronto: ... yaw_face`).
- Confirmar em campo o pré-docking a 0,25 m (WS): o fit do refino deve sair com largura 0,6–0,8 m.
