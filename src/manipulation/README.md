# src/manipulation — pacotes de manipulação do Caramelo (PC)

Nove pacotes. Os **vivos** no fluxo de competição estão em negrito; os demais são a
bancada standalone de 2026-07 (braço sozinho), mantidos só como referência.

| Pacote | Papel |
|---|---|
| **`manip_bt`** | Cérebro da missão (BehaviorTree.CPP v3). `task_planner` traduz o YAML da competição (formato oficial 2026 do @Work Commander — `missions/tasks geradas/`, `docs/guia_missoes.md` §3.2 — e os dois anteriores) em ações; `bt_yaml_executor` monta e tica a árvore (folhas `GoToWS`, `PickTag`, `PlaceTag`, `ReachQueue` + `NudgeBase`, `Amt1Sort`, `GoToNamedPose`). `MissionMapConfig` lê `docking.yaml`, `service_areas.yaml` e `ws_table_mapping.yaml` da pasta do mapa. Teste do planner sobre os arquivos oficiais: `colcon test --packages-select manip_bt --ctest-args -R formato2026`. |
| **`manip_task_execution`** | Nós C++ que movem o braço (MoveIt `MoveGroupInterface` + IK própria): `mtc_pick_action_node` → nó `pick_action_server` → action `/pick_tag`; `mtc_place_action_node` → `place_action_server` → `/place_tag` (slot nomeado, pilha `stack_on`, container por cor, superfície livre, memória de slots); `amt1_sort_action_node` → `amt1_sort_action_server` → `/amt1_sort`. Apesar do prefixo `mtc`, nenhum usa MoveIt Task Constructor. Libs puras com gtest: `custom_ik`, `reach_shift`, `amt1_sort_planner`, `surface_place_planner`, `container_state_store`. Ferramenta: `ros2 run manip_task_execution custom_ik_check`. |
| **`my_robot_msgs`** | Interfaces de manipulação (nome genérico herdado do template; não renomear sem coordenar com o `caramelo_hardware_ws`, que também as usa): actions `PickTag`, `PlaceTag`, `Amt1Sort`. |
| **`manip_bringup`** | `launch/manip_stack.launch.py` (incluído por `caramelo_bringup/robot_manipulation.launch.py`): `move_group`, `commander`, reset dos containers de bordo, os 3 action nodes, RealSense, `apriltag_node` (`config/tags_36h11.yaml`), `container_detector_node.py` (containers por cor → TF `ct_vermelho`/`ct_azul`), `workstation_surface_node.py` (mapa top-down da mesa), `vision_debug_node.py` (câmera anotada com tags/containers/mapa em `/manip/vision_debug/image`, só processa com espectador), `perception_gate_node.py` (opt-in), `speech_node`; braço mock no PC com `use_mock_hardware:=true`. Configs em `config/`; pytest sem ROS em `test/`. |
| **`manip_moveit_config`** | Configuração MoveIt do braço. Vivo: `launch/move_group_caramelo.launch.py` + `config/caramelo.srdf` (poses nomeadas `home`, `pegar_obj`, `Mesa*`, `tag_*`, `container*`), `kinematics.yaml` (pick_ik). `move_group.launch.py` e `config/manip.*` são a bancada standalone do Setup Assistant. |
| **`manip_audio`** | `speech_node`: assina `/manip/speech`, fila com prioridade pelo prefixo da frase (`!` alta, `~` baixa), cache de WAV e motores Kokoro (venv, `scripts/instalar_kokoro.sh`) → Piper (`models/pt_BR-faber`, `pt_BR-jeff`) → espeak-ng; publica `/manip/speech_status`. [docs/voz_do_robo.md](../../docs/voz_do_robo.md). |
| **`manip_commander`** | Executável `commander`: nó que assina `open_gripper`, `go_to_joint_target`, `go_to_pose_target`, `go_to_named_target` e executa via MoveIt (API de teleop manual; sobe no `manip_stack`). |
| `manip_description` | URDF/meshes do braço **sozinho** para a bancada standalone; o robô real usa `caramelo_description` (`caramelo_hardware_ws`). Ver `manip_description/README.md`. |
| `manip_hardware` | Cópia do plugin ros2_control do braço da Pi; no PC nunca roda (braço mock). Ver `manip_hardware/README.md`. |

## Fluxo de uma missão

```
missions/task_X.yaml
  └─ ros2 run caramelo_navigation run_mission --map-name <mapa> --task missions/task_X.yaml
       ├─ task_planner  <competicao.yaml> <actions.yaml> <tags_36h11.yaml> <ws_table_mapping.yaml>
       │     → ~/.ros/caramelo_missions/<data>_actions.yaml  (goto / pick / place / amt1_sort / home)
       └─ bt_yaml_executor <actions.yaml> --ros-args -p map_folder:=...   [--dry-run]
             ├─ GoToWS      → undock → NavigateToPose → DockRobot → align_to_dock (dock_align_node)
             ├─ PickTag     → /pick_tag   (pick_action_server)
             ├─ PlaceTag    → /place_tag  (place_action_server)
             ├─ ReachQueue  → adia o que está fora de alcance; NudgeBase → nudge_base (dock_align_node)
             ├─ Amt1Sort    → /amt1_sort  (amt1_sort_action_server; cliente de pick/place/nudge)
             └─ GoToNamedPose → MoveIt direto (home)
```

Um só nó move o braço por vez: `flock` em `/tmp/caramelo_manip_action.lock`
(`manipulator_execution_lock.hpp`). Estado dos containers de bordo e memória de slots por
WS: `~/.config/caramelo/container_states.yaml` (zerado a cada launch).

Como roda na competição: `ros2 launch caramelo_bringup robot_manipulation.launch.py
map_name:=<mapa>` e depois o `run_mission` acima. Guia do operador: `docs/guia_missoes.md`.

## Testes de bancada (sem robô)

- gtest das libs: `colcon test --packages-select manip_task_execution --ctest-args -R 'test_'`
  (sem o filtro, o lint uncrustify falha); `colcon test --packages-select manip_bt
  --ctest-args -R test_reach_queue`.
- pytest sem ROS: `python3 -m pytest src/manipulation/manip_bringup/test
  src/manipulation/manip_audio/test -q`.
- Fakes em `manip_bt/test/` (`ROS_DOMAIN_ID=99`, nunca encostam no robô):
  `run_reach_queue_scenario.sh` (fila de alcance com `fake_nav_actions.py` +
  `fake_pick_place.py`) e `run_amt1_scenario.sh all` (AMT1 com `fake_amt1_table.py` /
  `fake_amt1_sort.py`); fixtures de YAML em `manip_bt/test/fixtures/`.
- Detectores offline numa foto: `ros2 run manip_bringup container_detector_node.py --image
  foto.png --out overlay.png` e `ros2 run manip_bringup workstation_surface_node.py --image
  foto.png ...` (`docs/containers_por_cor.md`, `docs/place_superficie_livre.md`).
- Ver o que a câmera enxerga no robô: `ros2 run rqt_image_view rqt_image_view
  /manip/vision_debug/image` (`docs/guia_missoes.md` §6.1).
- Missão em mock no PC: `robot_manipulation.launch.py use_mock_hardware:=true` +
  `run_mission ... --simulate-nav`.
