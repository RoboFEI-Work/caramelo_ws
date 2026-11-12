# bag_forense — scripts ad hoc da analise dos bags de 2026-11-11

Nao sao instalados pelo CMake; rodam com `python3 <script>` depois de `source install/setup.bash`
(precisam de `rosbag2_py`, `numpy`, `matplotlib`). Resultado documentado em
`docs/relatorio_testes_2026-11-11.md`. Para a visao geral de um bag use primeiro
`ros2 run caramelo_navigation bag_resumo <pasta>`.

| Script | O que faz | Uso |
|---|---|---|
| `dump_rosout.py` | `/rosout` -> `<stamp>_rosout.txt` (t relativo, nivel, no, msg) | `dump_rosout.py <bag> [<bag>...]` |
| `extract_csv.py` | velocidades (nav/cmd/ref/odom/wheel), amcl, status das actions, plan, scan por setor -> `<stamp>_data.csv` | `extract_csv.py <bag> [...]` |
| `analise_janelas.py` | por janela de tempo: comandos abaixo do piso, giro puro, odom real, trilha AMCL, scan minimo | `analise_janelas.py <csv> 161:286 650:751` |
| `analise_pernas.py` | saltos em reference/rodas, evolucao do `/plan` por janela | `analise_pernas.py <csv> [a:b ...]` |
| `plan_rumo.py` | por plano: rumo do ponto a 0,5 m (o que o shim ve), erro vs yaw, yaw do goal | `plan_rumo.py <bag> a:b [...]` |
| `plot_janela.py` | PNG: mapa + trilha AMCL + planos + scan + footprint; costmap local num instante | `plot_janela.py <bag> a b saida.png [t_costmap]` |
| `trancos.py` | eventos de roda > 8 rad/s e > 2,5x a referencia; confere com `/joint_states` | `trancos.py <csv> <bag> [lx+ly]` |

Os CSV/TXT sao escritos no diretorio atual. O mapa usado pelo `plot_janela.py` esta fixo em
`arena_extendida` (editar `MAPD`).
