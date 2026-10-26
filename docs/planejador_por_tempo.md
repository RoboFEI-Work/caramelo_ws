# Planejador de missão por tempo — ordem das estações

Desde 2026-10-26 o `task_planner` (manip_bt) escolhe a **ordem das visitas às estações**
minimizando o tempo total estimado da missão, usando o mapa conhecido (docks de START,
FINISH, WS, SH, PP, ST) como grafo de distâncias. Antes a ordem era gulosa: a próxima
estação era a que tinha mais places/picks possíveis, sem olhar distância, e empates eram
sorteados.

Resumo do que muda para o operador:

- Se a pasta do mapa tiver `dock_distances.yaml`, o planner roda a busca e imprime
  `Tempo estimado guloso: … ; otimizado: …`. O `run_mission` mostra isso antes de pedir
  confirmação.
- Se **não** tiver, o comportamento é exatamente o de antes (guloso), com a dica
  `sem dock_distances.yaml em <pasta>: ordem gulosa (gere com: …)`.
- As regras de pick/place não mudaram: capacidade 3 a bordo, só entrega o que já pegou,
  pilha inicial desfeita antes do place, topo só com a base disponível, container por cor,
  slot da PP, nível da SH. AMT continua `goto PP + amt1_sort` (não há transporte a ordenar).
- O YAML de ações ganha um bloco informativo `plan_estimate` (modo, tempo, visitas, trocas,
  distância). O executor lê só `actions`, como sempre.

## 1. Modelo de custo

Para um plano = sequência de visitas (estação, places, picks):

```
tempo = Σ_trocas ( dist(anterior, estação) / v_nav + docking )
      + n_picks · pick + n_places · place
      + dist(última, FINISH) / v_nav
```

- `dist` vem da matriz `dock_distances.yaml` (comprimento de caminho entre as poses de
  staging dos docks, §2). A primeira troca parte do START.
- Visitas consecutivas à **mesma** estação (o planner volta a ela sem mover, quando um pick
  liberou um place bloqueado pela pilha inicial) não pagam navegação nem docking — a emissão
  também não repete o `goto`.
- O FINISH não tem docking (`use_docking: false`).

Parâmetros (defaults no código, `task_plan::CostModel`; sobrescritos por variável de
ambiente, §4):

| Parâmetro | Default | Variável | De onde veio |
|---|---|---|---|
| `v_nav_mps` | 0,25 m/s | `CARAMELO_PLANNER_V_NAV_MPS` | Run de 21/10 (`docs/analise_run_2026-10-21.md`): WS5→WS1 (~4,2 m de caminho) em ~28 s incluindo home+undock; WS1→FINISH (~6,2 m) em ~30 s. O operador sugeriu 0,3; os logs ficam entre 0,2 e 0,3 — 0,25 é o meio. |
| `docking_s` | 25 s | `CARAMELO_PLANNER_DOCKING_S` | Custo fixo por **troca de estação**: braço em `home` (~5 s) + undock + aproximação final + `align_to_dock` (7 s na WS5, mais com refino/validação de yaw). Igual ao sugerido pelo operador. |
| `pick_s` | 35 s | `CARAMELO_PLANNER_PICK_S` | 3 picks na WS5 em 112 s (2015→2127 s) ≈ 37 s cada. O operador sugeriu 20 s; os logs dizem mais. |
| `place_s` | 35 s | `CARAMELO_PLANNER_PLACE_S` | 3 places na WS1 em 107 s (2162→2269 s) ≈ 36 s cada (observação da mesa + place + volta). Sugerido 15 s; logs dizem mais. |

Observação importante: **todo plano válido tem o mesmo número de picks e de places** (cada
objeto é pego e solto uma vez). Logo `pick_s` e `place_s` só mudam o tempo absoluto
estimado — a ordem ótima é decidida por **navegação + número de trocas de estação**. Com
docking = 25 s e v = 0,25 m/s, uma troca a mais equivale a ~6 m de caminho: o otimizador
primeiro reduz trocas (viagens cheias, aproveitar a estação em que está) e depois a distância.

## 2. Matriz de distâncias (`dock_distances.yaml`)

Gerada por `compute_dock_distances` (caramelo_navigation) e gravada na pasta do mapa, ao lado
de `docking.yaml` / `ws_table_mapping.yaml` — é por isso que o `task_planner` a acha sem
mudar de interface (ele já recebe o `ws_table_mapping.yaml` do mapa como 4º argumento).

```bash
# da raiz do workspace, com o install sourceado
ros2 run caramelo_navigation compute_dock_distances --map-name arenalegal_2026
# ou, sem ROS: python3 src/caramelo_navigation/scripts/compute_dock_distances.py --map-folder <pasta>
# opções: --inflation 0.25  --dock-pose  --block-unknown  --euclidean  --dry-run  --output <arquivo>
```

Como calcula:

1. Lê `docking.yaml` (poses dos docks com pose real; placeholder `[0,0,0]` fica fora, listado
   em `skipped_docks`) e o `dock_plugins` para recuar cada pose pelo `staging_x_offset` do
   tipo — a **pose de staging** é onde o Nav2 leva o robô antes do docking, então é a
   distância que a navegação de fato percorre (`pose: staging` no arquivo; `--dock-pose`
   usa a pose do dock).
2. Lê `map.yaml` + `map.pgm` com a regra do `map_server` (`negate`, `occupied_thresh`,
   `free_thresh`); desconhecido é tratado como livre (como `allow_unknown: true` do
   `planner_server.yaml`; `--block-unknown` inverte).
3. Infla os obstáculos por `--inflation` (default **0,25 m**: raio inscrito do footprint
   0,79 × 0,39 m = 0,195 m + 0,055 m de margem; o `inflation_radius: 0.55` do costmap é o campo
   potencial, não o raio letal). Pose que cair na zona inflada é deslocada para a célula
   livre mais próxima (até 0,6 m; o deslocamento soma na distância e vira aviso em `notes`).
4. Dijkstra 8-conexo (custo 1 / √2 por célula, sem cortar quinas) de cada dock até todos os
   outros. Par sem caminho na grade cai na euclidiana e é listado em `euclidean_pairs`.
5. Sem `map.yaml`/`map.pgm` (ou `--euclidean`): toda a matriz é euclidiana (`method:
   euclidean`).

No `arenalegal_2026` (197 × 433 células a 2 cm) roda em ~0,5 s. Matriz atual:

```
        START FINISH    WS1    WS2    WS3    WS4    WS5    WS6    SH1    PP1
START    0.00   7.85   1.99   3.13   3.06   4.07   5.92   6.59   6.79   6.32
FINISH   7.85   0.00   6.16   4.72   4.87   3.92   1.99   1.80   1.06   1.82
WS1      1.99   6.16   0.00   1.43   1.75   2.75   4.22   5.28   5.09   5.01
WS2      3.13   4.72   1.43   0.00   0.69   1.39   2.79   3.92   3.66   3.59
WS3      3.06   4.87   1.75   0.69   0.00   1.04   2.93   3.53   3.81   3.25
WS4      4.07   3.92   2.75   1.39   1.04   0.00   1.99   2.53   2.86   2.25
WS5      5.92   1.99   4.22   2.79   2.93   1.99   0.00   1.40   1.01   1.18
WS6      6.59   1.80   5.28   3.92   3.53   2.53   1.40   0.00   0.87   0.34
SH1      6.79   1.06   5.09   3.66   3.81   2.86   1.01   0.87   0.00   0.89
PP1      6.32   1.82   5.01   3.59   3.25   2.25   1.18   0.34   0.89   0.00
```

**Quando regerar**: depois de regravar qualquer pose de dock (GUI / `save_dock_pose`),
trocar `staging_x_offset`, ou trocar o mapa. O arquivo guarda `generated_at`, `pose`,
`inflation_m` e `method` para auditoria; é versionado junto com o mapa.

Estação da task sem dock gravado (hoje: `ST` → `ST1` não existe no `docking.yaml` do
`arenalegal_2026`): o planner avisa `par(es) sem distancia — dock sem pose gravada: ST (ST1)
— usando a media das conhecidas` e segue otimizando o resto. Dock com pose gravada **depois**
da matriz (não está no arquivo) usa a reta entre as poses. Grave a pose e regere.

## 3. Busca

Implementação em `src/manipulation/manip_bt/src/task_plan_search.cpp`
(`task_plan::planOptimized`), separada do `task_planner.cpp` para o gtest.

- **Estado**: estação atual, conjunto de objetos pegos, conjunto de objetos soltos (o
  inventário é pegos − soltos). Chave de memória: índice da estação + duas máscaras de bits
  (até 64 objetos).
- **Ação (filho)**: ir a uma estação X, soltar **tudo** o que pode ser solto ali
  (`placesAtWs`: as mesmas regras do guloso — bases antes dos topos, place bloqueado por
  ocupante da pilha inicial fica a bordo) e pegar **um subconjunto** dos picks permitidos
  (`picksAtWs` com a lista `only`; cada subconjunto é simulado com `pickAllowedNow`, que
  aplica capacidade, `pick_after`, base disponível e a reserva de lugar a bordo para
  ocupantes). Soltar nunca é adiado (não custa navegação e libera lugar); pegar menos do que
  cabe é uma opção da busca (às vezes vale deixar um cubo para a próxima passada). Ficar na
  mesma estação sem mover só é filho quando um pick liberou um place — só picks seria
  redundante com um subconjunto maior na visita anterior.
- **Custo** do filho: navegação + docking (se trocou de estação) + places·place + picks·pick.
  Terminal (nada pendente): + navegação até o FINISH.
- **Branch-and-bound em profundidade**, filhos em ordem de `g + h`. Limite inferior `h`
  (admissível): picks/places pendentes × custo; um docking por estação com trabalho pendente
  (a atual também, se nada pode ser feito nela agora); e a **árvore geradora mínima** (Prim)
  das distâncias entre {atual} ∪ {estações pendentes} ∪ {FINISH} dividida por `v_nav` —
  qualquer rota que ainda passe por todas e termine no FINISH é um passeio que conecta esses
  nós, logo mede pelo menos a MST.
- **Limite superior inicial**: o plano guloso (sempre calculado). A busca prune `f > guloso`
  até achar o primeiro plano (assim encontra um plano ≤ guloso) e depois `f ≥ melhor`.
- **Tabela de transposição**: estado já alcançado mais barato é descartado; subárvore
  fechada guarda `limite_na_saída − g` como cota do que falta (válida porque o limite só
  cai) e poda revisitas.
- **Prazo** (`CARAMELO_PLANNER_TIME_LIMIT_S`, default 2 s): estourou → devolve o melhor
  plano fechado até ali, ou o guloso se nenhum fechou, e o log diz `prazo estourado, melhor
  encontrado` em vez de `busca completa`. `optimal: false` no `plan_estimate`.
- **Determinismo**: estações em ordem alfabética, subconjuntos em ordem crescente, desempate
  estável — a mesma task com a mesma matriz dá sempre o mesmo plano (o sorteio do guloso só
  afeta o limite inicial).
- Sem matriz (ou pasta do mapa inconsistente, ou nenhuma distância conhecida) → `planGreedy`,
  que é o código antigo movido sem alteração (mesmas mensagens de erro, mesma ordem de
  emissão); `CARAMELO_PLANNER_SEED` fixa o sorteio para reproduzir um plano guloso.

O que a busca responde às perguntas do operador:

- **Pick × place**: não há prioridade fixa; o custo decide. Chegando numa estação, solta tudo
  o que pode (grátis) e pega o que compensa; ir soltar antes de pegar mais só acontece
  quando reduz trocas ou distância.
- **3 cubos a bordo**: nenhum pick cabe; os únicos filhos são estações com place possível —
  a busca escolhe a que leva ao menor total (não a mais próxima necessariamente, porque
  olha o resto da rota).

## 4. Variáveis de ambiente

Lidas pelo `task_planner` (o `run_mission` repassa o ambiente):

| Variável | Default | Efeito |
|---|---|---|
| `CARAMELO_PLANNER_MODE` | `auto` | `greedy` força o guloso mesmo com matriz (ainda imprime o tempo estimado guloso). |
| `CARAMELO_PLANNER_V_NAV_MPS` | 0.25 | velocidade média de navegação (m/s) |
| `CARAMELO_PLANNER_DOCKING_S` | 25 | custo fixo por troca de estação (s) |
| `CARAMELO_PLANNER_PICK_S` | 35 | custo por pick (s) |
| `CARAMELO_PLANNER_PLACE_S` | 35 | custo por place (s) |
| `CARAMELO_PLANNER_TIME_LIMIT_S` | 2.0 | prazo da busca (s) |
| `CARAMELO_PLANNER_SEED` | aleatório | semente do sorteio de empates do guloso |

Exemplo: `CARAMELO_PLANNER_MODE=greedy ros2 run caramelo_navigation run_mission … --dry-run`
mostra o plano antigo para comparar.

## 5. Resultado nas 7 tasks oficiais (`missions/tasks geradas/`)

Mapa `arenalegal_2026`, defaults acima, `compare_plan_times.py --seeds 20` (o guloso sorteia
empates: mostra o intervalo em 20 sementes). Gerado com
`TASK_PLANNER_BIN=build/manip_bt/task_planner python3 src/manipulation/manip_bt/test/compare_plan_times.py`.

| Task | Guloso (s) min–max | Guloso: trocas / m (melhor) | Otimizado (s) | Otimizado: trocas / m | Ganho vs pior / melhor guloso | Busca; rota otimizada |
|---|---|---|---|---|---|---|
| BMT | 325 | 2 / 16,3 | **325** | 2 / 16,3 | 0 % / 0 % | 9 nós, 0 ms; WS_5 > WS_1 |
| BTT1 | 370–372 | 4 / 15,1 | **370** | 4 / 15,0 | 1 % / 0 % | 14 nós, 0 ms; WS_3 > WS_4 > WS_6 > WS_3 |
| BTT2 | 565 | 5 / 22,4 | **563** | 5 / 21,9 | 0 % / 0 % | 84 nós, 4 ms; WS_5 > WS_1 > WS_6 > WS_4 > WS_5 |
| ATT1 | 724–732 | 8 / 26,1 | **671** | 7 / 19,0 | 8 % / 7 % | 185 nós, 9 ms; WS_1 > WS_3 > WS_5 > WS_3 > ST > WS_6 > SH_1 (ST sem dock: média) |
| ATT2 | 854–949 | 9 / 17,3 | **816** | 7 / 20,3 | 14 % / 4 % | 710 nós, 37 ms; WS_4 > WS_5 > WS_6 > SH_1 > WS_1 > WS_2 > PP |
| FINAL | 1163–1286 | 13 / 34,6 | **1058** | 10 / 27,0 | 18 % / 9 % | 12 216 nós, 0,64 s; WS_2 > SH_1 > WS_5 > WS_4 > WS_6 > ST > WS_1 > WS_3 > SH_1 > PP (ST sem dock: média) |
| AMT | — | — | — | — | — | sem transporte: `goto PP + amt1_sort`, como antes |

Leitura: nas provas básicas (BMT/BTT) o guloso já era ótimo ou quase (poucas estações, sem
escolha real); nas avançadas a busca economiza de 1 a 4 **trocas de estação** (cada uma
~25 s + caminho) — ATT2 de 9–10 para 7 visitas, FINAL de 13–15 para 10. A ordem ótima do
FINAL começa pela WS_2 (perto do START), varre descendo e termina na PP, ao lado do FINISH.
Todas as 7 fecharam a busca completa bem dentro do prazo de 2 s neste notebook; a FINAL (10
objetos, 9 estações) é a mais pesada (0,6–1,1 s dependendo da carga), ver limitações.

## 6. Testes

```bash
# gtest do núcleo (modelo de custo, guloso, busca: capacidade cheia, pick depois do place
# liberar espaço, empate, 3 a bordo, pilha inicial, prazo, distância faltando)
colcon test --packages-select manip_bt --ctest-args -R test_task_plan_search
# pytest sobre o binário: 7 tasks oficiais com e sem matriz, determinismo, variáveis, pilha inicial
colcon test --packages-select manip_bt --ctest-args -R formato2026
# pytest do compute_dock_distances num mapa sintético (PGM, inflação, Dijkstra, fallback, YAML)
colcon test --packages-select caramelo_navigation --ctest-args -R test_compute_dock_distances
# tabela guloso x otimizado
TASK_PLANNER_BIN=build/manip_bt/task_planner python3 src/manipulation/manip_bt/test/compare_plan_times.py
```

`run_mission --dry-run` também exercita o planner (imprime o plano e o tempo estimado), mas
o executor em `--dry-run` instancia os nós da BT e espera o `/move_group` — precisa do stack
de manipulação no ar. Sem robô, o caminho é o pytest/binário acima.

## 7. Limitações e próximos passos

- **ST1 sem pose** no `arenalegal_2026`: ATT1 e FINAL usam a média das distâncias conhecidas
  para a ST. Gravar a pose do dock ST1 e regerar a matriz.
- Os custos são **constantes**: não modelam fila de alcance/nudges, retentativas de pick,
  containers, prateleira (SH mais lenta) nem o tempo de docking variável por estação. Quem
  tiver logs novos pode recalibrar pelas variáveis de ambiente sem rebuild.
- A matriz usa a pose de staging e um raio de inflação fixo; o caminho real do Smac Lattice
  pode diferir (curvas, custo do campo potencial). Erro de ±10 % na distância não muda a
  ordem na prática (a decisão é dominada pelo número de trocas).
- Prazo de 2 s medido neste notebook; num computador mais lento a FINAL pode estourar e
  devolver o melhor plano fechado (ainda ≤ guloso). Aumentar `CARAMELO_PLANNER_TIME_LIMIT_S`
  se o log disser `prazo estourado`.
- Limite de 64 objetos por task (máscaras de 64 bits) — as tasks oficiais têm até 10.
- A busca enumera subconjuntos de picks por estação (até 2^k, k = picks pendentes ali); com
  mais de ~12 objetos na mesma estação o custo por nó cresce — fora do alcance das tasks.
- Não há replanejamento em execução: se uma estação é pulada (goto falhou), o executor segue
  a ordem planejada.
