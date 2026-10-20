# missions/ — tarefas do Caramelo (fonte única)

Cada `*.yaml` desta pasta é uma **tarefa** no formato do YAML da competição
(RoboCup@Work / @Work Commander, formatos antigo e novo), consumida por:

```bash
cd ~/caramelo_ws
ros2 run caramelo_navigation run_mission --map-name <mapa> --task missions/<arquivo>
```

`--task` é um caminho relativo ao diretório atual (ou absoluto); não há fallback para
share de pacote — esta pasta é a única fonte de tarefas. O `task_planner` (`manip_bt`)
traduz a tarefa numa lista de ações gravada em `~/.ros/caramelo_missions/` e o
`bt_yaml_executor` a executa. Formato, regras e semântica (cores, pilhas, AMT1):
`docs/guia_missoes.md` §3.

Convenção de cabeçalho de cada arquivo: nome em 1 linha, montagem física, comando de
uso. Os comentários por objeto descrevem o trajeto real (`origem -> destino`).

| Arquivo | O que testa |
|---|---|
| `arena_test.yaml` | 5 cubos entre shelf e 4 mesas (pick e place na prateleira, transferências). |
| `task_BTT_I.yaml` | Basic Transportation Task, 3 cubos / 3 mesas. |
| `task_BTT_II.yaml` | Basic Transportation Task, 5 cubos / 4 mesas. |
| `task_BMT.yaml` | Basic Manipulation Task, 3 cubos WS_1 → WS_4 + 1 decoy. |
| `task_ATT_1.yaml` | Advanced Transportation: shelf, pilha e containers por cor. |
| `task_AMT1_pp1.yaml` | Advanced Manipulation Task 1: ordenar os 6 cubos da PP_1 (`amt1_sort`). |
| `tarefa_completa_tags0a7.yaml` | Tudo em uma: shelf, mesas, pilha e container por cor (8 cubos). |
| `task_portas_abertas.yaml` | Demonstração Portas Abertas (mapa `arena_portas_abertas`). |

`exemplos/`: YAML de **ações** (nível baixo, lido direto pelo `bt_yaml_executor`,
`docs/guia_missoes.md` §3.7), não de tarefa — o cabeçalho de cada arquivo traz o comando.

Fotos de calibração dos containers (antes aqui em `fotos_containers/`) estão em
`docs/imagens/containers/`.
