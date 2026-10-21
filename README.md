# Caramelo WS — workspace do PC de operação (ROS 2 Jazzy)

Workspace do **PC de operação** do robô Caramelo (RoboFEI @Work, RoboCup@Work):
navegação (Nav2), docking, manipulação (MoveIt + IK própria), missões de competição,
interface gráfica e ferramentas. O robô funciona com **dois computadores** na mesma rede
(`ROS_DOMAIN_ID=67`):

| Computador | O que faz | Workspace |
|---|---|---|
| **Raspberry Pi** (no robô) | Motores, LiDAR, IMU, braço (ros2_control), odometria (EKF), `robot_description` | `caramelo_hardware_ws` (repositório separado) |
| **PC de operação** (este) | Mapa, localização, Nav2, docking, manipulação, missões, GUI | `caramelo_ws` |

Este workspace **não** inicia drivers físicos. Ele **depende** do `~/caramelo_hardware_ws`
também no PC, porque o pacote `caramelo_description` (URDF do robô) vem de lá — sem ele o
`move_group` não sobe. Ordem de `source` em todo terminal (detalhes e `.bashrc` em
[docs/setup_notebook_novo.md](docs/setup_notebook_novo.md), Passo 8):

```bash
source /opt/ros/jazzy/setup.bash
source ~/caramelo_hardware_ws/install/local_setup.bash
source ~/caramelo_ws/install/local_setup.bash      # por último: vence
```

> ⚠️ **Segurança** — leia os avisos do `caramelo_hardware_ws/README.md` antes de energizar
> os motores: há um comportamento perigoso conhecido do driver de motor (ESC) quando o
> sinal de controle cai — o robô pode **disparar de ré sozinho**. Ferramentas marcadas
> com **MOVE O ROBÔ** publicam direto na referência das rodas, fora do collision monitor.

---

## 1. Rodar uma missão de competição

```bash
# 0. No PC, antes de qualquer launch (mata processos ROS órfãos do launch anterior):
./scripts/limpar_stack.sh

# 1. Na Raspberry (sempre primeiro):
ros2 launch raspberry_bringup hardware_bringup.launch.py

# 2. No PC — navegação + manipulação juntas (default do mapa: arena3_520):
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=<mapa>

# 3. Localizar o robô: RViz "2D Pose Estimate" (use_rviz:=true) ou a GUI
#    (fantasma -> Confirmar pose). Fazer vai-e-vem de ~0,5 m até a nuvem do AMCL fechar.

# 4. A missão (da raiz do workspace; a tarefa vem de missions/):
cd ~/caramelo_ws
ros2 run caramelo_navigation run_mission --map-name <mapa> --task missions/task_X.yaml
```

Opções do `run_mission`: `--dry-run` (só mostra o plano, nada se move),
`--use-lidar-refine` (docking ancorado na face física da mesa), `--simulate-nav` (testa a
manipulação parado, sem Nav2), `--yes`, `--finish ""` (não vai ao FINISH),
`--preflight-timeout`. O planner mostra o plano e pede confirmação; Ctrl-C cancela limpo.
Tudo em MOCK no PC, sem robô: `robot_manipulation.launch.py use_mock_hardware:=true` +
`run_mission ... --simulate-nav`.

> **Não ligue o Nav2 pela GUI** ("Ligar navegação") com o `robot_manipulation.launch.py`
> no ar: duplica AMCL/Nav2 e o robô "teleporta". A GUI é monitor e ferramenta de
> comissionamento (mapa novo, service areas, docking avulso).

Runbook completo, formato do YAML da tarefa e calibrações: [docs/guia_missoes.md](docs/guia_missoes.md).

## 2. Mapa de pacotes

**Navegação e base** (`src/`)

| Pacote | O que é |
|---|---|
| `caramelo_navigation` | Nav2 do Caramelo: `bringup.launch.py` (= `navigation.launch.py` + RViz; é o que a GUI chama), `navigation.launch.py` (AMCL via `global_localization`, planner SmacLattice, controller DWB holonômico, smoother, collision monitor, relay `twist_relay`, service areas, inclui `docking_server.launch.py`), `docking_server.launch.py` (opennav_docking + `dock_align_node` com as actions `align_to_dock` e `nudge_base` + `dock_pose_recorder_node`). CLIs (`ros2 run caramelo_navigation <nome>`): `run_mission`, `init_service_areas`, `save_service_area_pose`, `list_service_areas`, `validate_service_areas`, `init_keepout_mask`, `sync_route_graph`, `generate_caramelo_omni_lattice`, `dock_to`, `undock`, `list_docks`, `save_dock_pose`, `init_map_docking` (legado). Teste: `test/test_nudge_math.py`. |
| `caramelo_localization` | AMCL + map_server (`global_localization.launch.py map_name:=<mapa>`); com `use_keepout:=true` sobe os servidores do Keepout Filter. |
| `caramelo_mapping` | slam_toolbox (`slam.launch.py map_name:=<mapa>`) + map saver + markers das service areas. Guarda os **mapas** em `maps/<mapa>/` (§3). |
| `caramelo_cartographer` | Alternativa de mapeamento com cartographer_ros, herdada do turtlebot3; não usada pela GUI nem pelo fluxo de missão. |
| `caramelo_utils` | `twist_relay` (Twist → TwistStamped em `/mecanum_controller/reference`) e `teleop.launch.py` (joystick, teclado em xterm, twist_mux). |
| `caramelo_bringup` | `robot_manipulation.launch.py map_name:=<mapa>` — launch unificado de competição (`caramelo_navigation/bringup.launch.py` sem RViz + `manip_bringup/manip_stack.launch.py`); args `use_nav`, `use_camera`, `use_apriltag`, `use_audio`, `use_container_detector`, `use_rviz`, `use_rviz_manip`, `use_mock_hardware`… |

**Manipulação** (`src/manipulation/`, mapa detalhado em [src/manipulation/README.md](src/manipulation/README.md))

| Pacote | O que é |
|---|---|
| `manip_bt` | Cérebro da missão (BehaviorTree.CPP): `task_planner` (YAML da competição → lista de ações) e `bt_yaml_executor` (folhas `GoToWS`, `PickTag`, `PlaceTag`, `ReachQueue` + `NudgeBase`, `Amt1Sort`, `GoToNamedPose`). Bancada sem robô em `test/` (fakes). |
| `manip_task_execution` | Nós que movem o braço: `mtc_pick_action_node` → nó `pick_action_server` → action `/pick_tag`; `mtc_place_action_node` → `place_action_server` → `/place_tag`; `amt1_sort_action_node` → `amt1_sort_action_server` → `/amt1_sort`. Libs puras (gtest): `custom_ik`, `reach_shift`, `amt1_sort_planner`, `surface_place_planner`, `container_state_store`. Ferramenta `custom_ik_check`. |
| `manip_bringup` | `manip_stack.launch.py`: move_group, commander, os 3 action nodes, RealSense, AprilTag (`config/tags_36h11.yaml`), `container_detector_node.py`, `workstation_surface_node.py`, `perception_gate_node.py`, speech; braço mock no PC. |
| `manip_moveit_config` | MoveIt do braço: `move_group_caramelo.launch.py` + `config/caramelo.srdf` (poses nomeadas). `move_group.launch.py` é a bancada standalone. |
| `manip_audio` | `speech_node` (nó `speech_node`): fala o que chega em `/manip/speech` (Piper ou espeak-ng). |
| `manip_commander` | Executável `commander`: tópicos `open_gripper`, `go_to_*_target` → MoveIt (teleop manual do braço). Ferramenta de bancada; `use_commander:=false` no `robot_manipulation.launch.py` deixa fora da competição. |
| `manip_description`, `manip_hardware` | Bancada standalone do braço sozinho (URDF e plugin ros2_control copiados da Pi); o robô real usa `caramelo_description`/`caramelo_hardware` do `caramelo_hardware_ws`. |

**Interfaces ROS**

| Pacote | O que é |
|---|---|
| `caramelo_msgs` | Actions `AlignToDock`, `NudgeBase` (servidas pelo `dock_align_node`; clientes `GoToWS`, `amt1_sort`, GUI) e `SaveServiceAreaPose`; services `InitServiceAreas`, `ListServiceAreas`, `ValidateServiceAreas`. Compilar antes dos demais. |
| `my_robot_msgs` | **Interfaces de manipulação**: actions `PickTag`, `PlaceTag`, `Amt1Sort` (nome genérico herdado do template; também usado pela Pi — não renomear sem coordenar). |

**Operação e ferramentas**

| Pacote | O que é |
|---|---|
| `caramelo_gui` | App Qt5/C++ com RViz embutido (`ros2 launch caramelo_gui caramelo_gui.launch.py`): módulos Início (saúde via `/diagnostics`), Mapa & Camadas, Navegação, Docking, Localização (fantasma → `/initialpose`), Mapas, Mapeamento, Teleop, Service Areas, Waypoints, Editor de Mapa. Sobe como filhos `slam.launch.py`, `bringup.launch.py` e `teleop.launch.py`. Ver [src/caramelo_gui/README.md](src/caramelo_gui/README.md). |
| `caramelo_diagnostics` | `health_monitor` → `DiagnosticArray` em `/diagnostics` (rede com a Pi, controllers, lidar, imu, odometria, TF, bateria, Nav2); `diagnostics.launch.py`, incluído pela GUI. |
| `scripts/` (raiz) | `limpar_stack.sh` e `pulse_test.py` (§6). |
| `missions/` (raiz) | Tarefas da competição — **fonte única** ([missions/README.md](missions/README.md)). |

## 3. Pasta de cada mapa

`src/caramelo_mapping/maps/<mapa>/` — veja os existentes com `ls src/caramelo_mapping/maps`
(pastas `*_bkp_*` são backups manuais; `*.yaml.bak` são backups automáticos ignorados pelo git).

| Arquivo | Papel |
|---|---|
| `map.yaml`, `map.pgm` | Mapa de ocupação (AMCL/Nav2). |
| `service_areas.yaml` | **Fonte humana**: pose final do robô em cada área (START, FINISH, WS*, SH*, PP*…), `use_docking`, tolerâncias. |
| `docking.yaml` | **Derivado** de `service_areas.yaml` (`validate_service_areas --sync-docking`) para o Docking Server e o `dock_align_node`; não edite à mão. |
| `ws_table_mapping.yaml` | **Obrigatório para `run_mission`**: estação → pose do braço (`Mesa0/5/10/15`), estação → dock, paredes opcionais (`ws_walls`). |
| `keepout_mask.{pgm,yaml}` | Opcional: paredes virtuais (fita no chão). |
| `route_graph.geojson` | Opcional: grafo do Nav2 Route Server (não usado pela missão). |
| `waypoints.yaml` | Gravado pelo módulo Waypoints da GUI. |

## 4. Fluxos de operação

- **Mapa novo** — `slam.launch.py map_name:=<mapa>` + teleop, salvar com `map_saver_cli`,
  rebuild de `caramelo_mapping`. Por terminal: [docs/passo_a_passo_mapa_service_areas_docking_route.md](docs/passo_a_passo_mapa_service_areas_docking_route.md); pela GUI: [docs/tutorial_gui_simulacao.md](docs/tutorial_gui_simulacao.md).
- **Service areas e docking** — parar o robô na pose final de cada área, `save_service_area_pose` (ou GUI → Service Areas), `validate_service_areas --sync-docking`, testar com `dock_to`; o alinhamento fino holonômico é o `dock_align_node`. [docs/service_areas_docking.md](docs/service_areas_docking.md), [docs/docking_alinhamento_holonomico.md](docs/docking_alinhamento_holonomico.md).
- **Keepout (paredes virtuais)** — `init_keepout_mask --map-name <mapa>`, pintar de preto no GIMP, subir com `use_keepout:=true`. [docs/keepout_paredes_virtuais.md](docs/keepout_paredes_virtuais.md).
- **Route graph (opcional)** — `sync_route_graph --map-folder src/caramelo_mapping/maps/<mapa>`; o launch só sobe o Route Server se o grafo for válido. [docs/service_areas_docking.md](docs/service_areas_docking.md).
- **GUI** — `ros2 launch caramelo_gui caramelo_gui.launch.py`: monitor de saúde, localizar pelo fantasma, mapear, editar mapa, cadastrar áreas, goal e waypoints; com o launch de missão no ar, use só como monitor. [src/caramelo_gui/README.md](src/caramelo_gui/README.md).
- **Relógio PC ↔ Pi** — obrigatório em arena sem internet (chrony no PC). [docs/sincronizacao_relogio_ntp.md](docs/sincronizacao_relogio_ntp.md).

## 5. Build e testes

```bash
cd ~/caramelo_ws            # terminal NOVO (bashrc padrão), nunca com sources manuais extras
colcon build --symlink-install
source install/local_setup.bash
```

Depois de mapas/service areas/docking: `colcon build --symlink-install --packages-select
caramelo_mapping caramelo_navigation`. Scripts Python novos em `install(PROGRAMS)` precisam
de `chmod +x` no `src/` (senão "executable not found on the libexec directory").

Depois de pacotes/executáveis **removidos ou renomeados** (2026-10: `caramelo_planning`,
`mission_runner`, `init_keepout_mask.py`, `twist_relay.py`, `health_aggregator`, os pacotes
Python vazios de `caramelo_mapping`/`caramelo_localization`), o colcon **não apaga** os
artefatos antigos: `rm -rf build/caramelo_planning install/caramelo_planning` e o `build/` +
`install/` dos pacotes afetados antes do `colcon build`.

Testes:

```bash
colcon test --packages-select manip_task_execution --ctest-args -R 'test_'   # 4 gtests; sem -R o lint uncrustify falha
colcon test --packages-select manip_bt --ctest-args -R test_reach_queue
python3 -m pytest src/caramelo_navigation/test/test_nudge_math.py -q
python3 -m pytest src/manipulation/manip_bringup/test src/manipulation/manip_audio/test -q
```

Antes de subir qualquer stack: `./scripts/limpar_stack.sh` (e `ros2 node list | sort |
uniq -c | awk '$1>1'` tem que sair vazio).

## 6. Ferramentas

| Ferramenta | Uso |
|---|---|
| `scripts/limpar_stack.sh` | Mata processos ROS órfãos no PC (nav2, move_group, manip_*, realsense, apriltag, rviz) e reinicia o daemon. Rodar após todo Ctrl-C. |
| `scripts/pulse_test.py` — **MOVE O ROBÔ** | Calibração do coast/piso do ESC usada pelo `dock_align_node`: `python3 scripts/pulse_test.py x 4` (rodas suspensas ou área livre). Valores vão para `docking_server.launch.py`. [docs/docking_alinhamento_holonomico.md](docs/docking_alinhamento_holonomico.md). |
| `ros2 run manip_task_execution custom_ik_check [x y z [modo] [free] \| shift x y z [lift]]` | Confere a IK própria e a tabela da fila de alcance sem robô. |
| `src/manipulation/manip_bt/test/run_reach_queue_scenario.sh`, `run_amt1_scenario.sh` | Bancada sem robô (`ROS_DOMAIN_ID=99`) com os fakes `fake_nav_actions.py`, `fake_pick_place.py`, `fake_amt1_table.py`, `fake_amt1_sort.py`. |
| `ros2 run manip_bringup container_detector_node.py --image foto.png --out overlay.png` | Detector de containers offline (idem `workstation_surface_node.py --image`). Fotos em `docs/imagens/containers/`. |
| `ros2 run rqt_image_view rqt_image_view /manip/vision_debug/image` | Câmera do punho **anotada** ao vivo (tags, containers por cor, mapa da mesa, estado pick/place, fala) — nó `vision_debug` do `manip_stack`, custo zero sem espectador; `use_vision_viewer:=true` no launch abre direto. [docs/guia_missoes.md §6.1](docs/guia_missoes.md). |
| `ros2 run caramelo_navigation generate_caramelo_omni_lattice` | Regenera as primitivas do SmacLattice (`config/lattice/`). |

## 7. Documentação

| Documento | O que ensina |
|---|---|
| [docs/guia_missoes.md](docs/guia_missoes.md) | **Runbook de competição**: subir, rodar, formato do YAML (empilhar, container por cor, superfície livre, AMT1, nível baixo), arquivos por mapa, executor, problemas comuns. |
| [docs/containers_por_cor.md](docs/containers_por_cor.md) | Manutenção do place em container sem tag: o que ajustar quando o pote, a cor, a garra ou a câmera mudam. |
| [docs/place_superficie_livre.md](docs/place_superficie_livre.md) | Place em superfície livre: a câmera do punho escolhe onde soltar (rulebook 4.4); ainda sem teste no robô. |
| [docs/docking_alinhamento_holonomico.md](docs/docking_alinhamento_holonomico.md) | `dock_align_node`: alinhamento fino com rodas mecanum, `nudge_base`, guardas, calibração. |
| [docs/service_areas_docking.md](docs/service_areas_docking.md) | Conceitos de service areas e docking, onde ficam os parâmetros, route graph. |
| [docs/passo_a_passo_mapa_service_areas_docking_route.md](docs/passo_a_passo_mapa_service_areas_docking_route.md) | Arena nova por terminal: mapa → áreas → docking → rotas → testes. |
| [docs/keepout_paredes_virtuais.md](docs/keepout_paredes_virtuais.md) | Paredes virtuais (fita no chão que o laser não vê). |
| [docs/sincronizacao_relogio_ntp.md](docs/sincronizacao_relogio_ntp.md) | Relógio PC ↔ Raspberry com chrony. |
| [docs/setup_notebook_novo.md](docs/setup_notebook_novo.md) | Preparar outro PC: pacotes, clones, ordem de build/source, rede, chrony, SSH da Pi, armadilhas. |
| [docs/tutorial_gui_simulacao.md](docs/tutorial_gui_simulacao.md) | GUI passo a passo (exige o `caramelo_simulation_ws`, repositório separado ausente neste PC). |
| [docs/historico/](docs/historico/README.md) | Auditorias e relatórios de 2026-07/08 — **não refletem o estado atual**. |
| `docs/imagens/containers/` | Fotos da RealSense, overlays e montagem usadas na calibração dos containers. |
| READMEs de pacote | [src/manipulation/README.md](src/manipulation/README.md), [src/caramelo_gui/README.md](src/caramelo_gui/README.md), [src/caramelo_msgs/README.md](src/caramelo_msgs/README.md), [src/caramelo_mapping/README.md](src/caramelo_mapping/README.md), [src/caramelo_localization/README.md](src/caramelo_localization/README.md), [src/manipulation/manip_description/README.md](src/manipulation/manip_description/README.md), [src/manipulation/manip_hardware/README.md](src/manipulation/manip_hardware/README.md), [missions/README.md](missions/README.md). |
| `~/caramelo_hardware_ws/docs/` | Calibração de odometria, tempo real na Pi, comportamento e riscos do ESC, chrony. |

## 8. Convenções do repositório

- **Nomenclatura ROS 2**: Python PEP8 (`snake_case`, classes `CamelCase`); C++ tipos `CamelCase`,
  métodos `camelCase`, variáveis `snake_case`, membros com `_` no fim, constantes `kCamelCase`;
  executáveis, nós, tópicos, parâmetros e launches em `snake_case` **sem** sufixo `.py`
  (`install(PROGRAMS ... RENAME nome)`).
- **Comentários de código** em português **ASCII** (sem acentos), objetivos, com cabeçalho
  de 3–6 linhas (o que faz / quem usa / como roda). Markdown pode ter acentos.
- **Docs sem números de linha**: cite o nome do parâmetro, função ou arquivo (`grep -n`).
- **Caminhos** relativos à raiz do workspace (`~/caramelo_ws`); nunca `/home/<usuário>/...`.
- **`missions/` é a fonte única de tarefas**; YAMLs de amostra não vivem em share de pacote.
- **Valores de calibração** (velocidades, distâncias, ganhos) só no launch/YAML que os
  declara, com a data da última medição e o motivo em 1 linha; docs e README apontam o
  arquivo, não copiam o valor.
- `package.xml`: descrição real em 1 frase e maintainer `Equipe RoboFEI @Work`.

## 9. Decisões pendentes para o time

1. **Licença**: não há licença comum (Apache-2.0 em `caramelo_utils`/`cartographer`/
   `diagnostics`/`gui`, BSD-3-Clause em `manip_moveit_config`, `TODO` nos demais). Escolher
   uma e aplicar em todos os `package.xml`.
2. **Dois escritores de `service_areas.yaml` / `docking.yaml`** (bug de perda de pose): o
   botão "Salvar pose como dock" da GUI (`/save_dock_pose` → `dock_pose_recorder_node` →
   `update_dock_pose`) grava a pose **só** em `docking.yaml` e deixa em `service_areas.yaml`
   uma entrada no formato antigo sem `final_robot_pose`; qualquer sync a partir de
   `service_areas.yaml` (GUI Service Areas, `save_service_area_pose`,
   `validate_service_areas --sync-docking`, `init_service_areas --sync-docking`) reconstrói
   `docking.yaml` do zero e devolve essa pose a `(0,0,0)`. Recomendação: fonte única =
   `service_areas.yaml` (recorder grava via `update_service_area_pose` + sync; `docking.yaml`
   sempre derivado) e depois aposentar `init_map_docking`, `list_docks` e os args
   `--kind/--height` de `save_dock_pose`. Exige rebuild de `caramelo_navigation` e teste com a GUI.
3. **Dois relays Twist→TwistStamped** quando `teleop.launch.py` e `navigation.launch.py` estão
   no ar (`nav2_twist_relay` e o `twist_mux` do teleop, ambos em `/mecanum_controller/reference`):
   decidir um árbitro único (`twist_mux_topics.yaml` não foi alterado) e validar com rodas suspensas.
4. **`caramelo_cartographer`**: mantido; `config/turtlebot3_lds_2d.lua` e `rviz/tb3_cartographer.rviz`
   ainda com nome tb3 — renomear se o pacote ficar, ou apagar se ninguém usa.
5. **Service `/caramelo/service_areas/init`** (`InitServiceAreas.srv`) não tem cliente: dar o
   botão "Inicializar áreas" à GUI ou remover service + `.srv` (rebuild de msgs, navigation e gui).
6. **Route Server / `sync_route_graph` / `route_server.yaml`**: `route_graph.geojson` só existe em
   3 mapas e nada chama `compute_route`; documentado aqui como opcional — ou remover de vez.
7. **`dock_align_node`, fase reta e `lateral_adjust`**: seguem com o piso exato do ESC como
   limite inferior (preexistente); se um goal partir do repouso com front-stop curto pode ficar
   parado até o `soft_timeout` — garantir `staging_offset` suficiente ou usar a velocidade de
   partida nos primeiros ciclos.
8. **Nó de diagnóstico** renomeado de `caramelo_health_monitor` para `health_monitor` (executável,
   launch e YAML); a GUI só depende dos nomes de status. Ajustar scripts externos (`ros2 param set`, rqt) se houver.
9. **`package.xml` de `caramelo_mapping` e `caramelo_localization`**: dependências enxugadas /
   acrescentadas conforme o que os launches usam — conferir no build completo.
10. **`maps/sala_520_bkp_premap`** (backup manual rastreado) e a lista de mapas oficiais: decidir
    o que sai do `src/`.
11. **`~/manip_ws`**: cópia antiga e divergente dos pacotes de manipulação fora do repo (com
    alterações locais não commitadas); `.vscode/c_cpp_properties.json` ainda aponta para ela.
    Verificar commits não enviados e apagar.
12. **`.vscode/browse.vc.db*`** (>1,3 GB de cache do IntelliSense dentro do workspace): apagar com
    o VS Code fechado e apontar o `includePath` para `${workspaceFolder}/install`.
13. **uncrustify em `manip_task_execution`**: 9 arquivos fora do padrão (os 3 nós em indent 4);
    `colcon test` sem `-R 'test_'` fica vermelho. Reformatar num commit só de formatação.
14. **Duplicação nos 3 nós de manipulação** (~30 helpers copiados byte a byte entre pick/place/amt1,
    `mtc_pick_action_node` inclui MTC sem usar): extrair para uma lib comum quando houver janela de teste no robô.

## 10. Problemas comuns

| Sintoma | Causa / onde olhar |
|---|---|
| Robô "teleportando" na GUI/RViz; `/map` nunca publica; actions com "unknown result" | Dois AMCL / dois move_group (launch antigo que não morreu). `./scripts/limpar_stack.sh` e `ros2 node list \| sort \| uniq -c \| awk '$1>1'`. [docs/guia_missoes.md](docs/guia_missoes.md) §6. |
| Localização piora com o tempo; "message filter dropping" | Relógio da Pi derivou (sem RTC). [docs/sincronizacao_relogio_ntp.md](docs/sincronizacao_relogio_ntp.md). |
| Planeja mas não anda; `/cmd_vel` existe e `/mecanum_controller/reference` não muda | Relay/twist_mux ou relógio. [docs/passo_a_passo_mapa_service_areas_docking_route.md](docs/passo_a_passo_mapa_service_areas_docking_route.md) §18. |
| `No such file ... caramelo_hardware_ws/.../global_localization.launch.py` | Ordem dos workspaces invertida (`setup.bash` em vez de `local_setup.bash`). [docs/setup_notebook_novo.md](docs/setup_notebook_novo.md) Passo 8. |
| Fala "vou me ajustar" e a base não se move (`curso_incompleto` / `sem_progresso`) | Piso do ESC, publicadores concorrentes na referência ou um `dock_align_node` velho. [docs/docking_alinhamento_holonomico.md](docs/docking_alinhamento_holonomico.md). |
| Câmera "Device or resource busy" / nó sobe sem imagens | Processo órfão segurando a USB, ou porta USB 2 (o D455 exige USB 3). [docs/guia_missoes.md](docs/guia_missoes.md) §6. |
| "Dock requested has no valid plugin" | `docking.yaml` com esquema de `dock_plugins` errado. [docs/guia_missoes.md](docs/guia_missoes.md) §6. |
| `Route server enabled: False` | Grafo vazio ou ausente; é opcional. [docs/service_areas_docking.md](docs/service_areas_docking.md). |
| Mapa salvo não carrega | `map.yaml`/`map.pgm` ausentes, `caramelo_mapping` sem rebuild ou terminal sem source. [docs/passo_a_passo_mapa_service_areas_docking_route.md](docs/passo_a_passo_mapa_service_areas_docking_route.md) §18. |
