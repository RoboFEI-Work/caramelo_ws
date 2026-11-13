cmake_minimum_required(VERSION 3.8)
project(caramelo_navigation)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_tree config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  scripts/caramelo_docking_common.py
  scripts/caramelo_bag_common.py
  scripts/caramelo_run_common.py
  scripts/caramelo_dock_camera.py
  scripts/caramelo_dock_camera_debug.py
  scripts/run_mission.py
  scripts/caramelo_service_area_common.py
  scripts/caramelo_tour_common.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mission_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_server
)

install(PROGRAMS
  scripts/init_keepout_mask.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME init_keepout_mask
)

install(PROGRAMS
  scripts/init_map_docking.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME init_map_docking
)

install(PROGRAMS
  scripts/save_dock_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME save_dock_pose
)

install(PROGRAMS
  scripts/dock_pose_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_pose_recorder_node
)

install(PROGRAMS
  scripts/dock_align_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_align_node
)

install(PROGRAMS
  scripts/dock_camera_probe.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_camera_probe
)

install(PROGRAMS
  scripts/run_mission.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_mission
)

install(PROGRAMS
  scripts/dock_to.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_to
)

install(PROGRAMS
  scripts/list_docks.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME list_docks
)

install(PROGRAMS
  scripts/undock.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME undock
)

install(PROGRAMS
  scripts/tour_docks.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tour_docks
)

install(PROGRAMS
  scripts/init_service_areas.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME init_service_areas
)

install(PROGRAMS
  scripts/save_service_area_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME save_service_area_pose
)

install(PROGRAMS
  scripts/list_service_areas.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME list_service_areas
)

install(PROGRAMS
  scripts/validate_service_areas.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_service_areas
)

install(PROGRAMS
  scripts/service_area_markers_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME service_area_markers_node
)

install(PROGRAMS
  scripts/service_area_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME service_area_manager_node
)

install(PROGRAMS
  scripts/generate_caramelo_omni_lattice.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_caramelo_omni_lattice
)

install(PROGRAMS
  scripts/sync_route_graph.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sync_route_graph
)

# Matriz de distancias entre docks (dock_distances.yaml do mapa), lida pelo
# task_planner do manip_bt para ordenar as estacoes por tempo (2026-10-26).
install(PROGRAMS
  scripts/compute_dock_distances.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME compute_dock_distances
)

# Rosbag do tour / dos dockings (2026-10-26, diagnostico secao 8.1): os tours
# nao tinham bag e os logs do Nav2 nao trazem pose. Nao e lancado pelo launch;
# o operador roda antes do tour_docks. Desde 2026-11-11 e' um atalho para o
# modo CLI do caramelo_bag_common.py (mesma lista/compressao das corridas).
# RENAME => copia (nao symlink) mesmo com --symlink-install: editar o .sh
# exige rebuild do pacote.
install(PROGRAMS
  scripts/gravar_bag_tour.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME gravar_bag_tour
)

# Resumo offline de um bag de corrida (rosbag2_py): duracao, tamanho, msgs
# por topico, taxa real de /cmd_vel_nav, offset de relogio Pi x PC e fracao
# de "caranguejo" por goal (2026-11-11).
install(PROGRAMS
  scripts/bag_resumo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_resumo
)

# Limite de velocidade por proximidade (2026-11-11): publica nav2_msgs/SpeedLimit
# em /speed_limit a partir do menor alcance do /scan; o controller_server
# repassa ao DWB (setSpeedLimit). Ligado por use_speed_limit no navigation.launch.py.
install(PROGRAMS
  scripts/speed_limit_by_scan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME speed_limit_by_scan
)

# Gravacao completa por corrida (2026-11-13): runs/run_<stamp>/ com video da
# janela do RViz do robo (global_localization.rviz), bag, logs, config,
# metadata.json e analysis.md. gravar_run = corrida manual ate o Ctrl-C; run_diag_viz = trilha
# real / plano anterior / eventos em /diag/*; analisar_run = relatorio do bag.
install(PROGRAMS
  scripts/caramelo_run_common.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gravar_run
)
install(PROGRAMS
  scripts/run_diag_viz.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_diag_viz
)
install(PROGRAMS
  scripts/analisar_run.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME analisar_run
)

# Bancada OFFLINE do controller_server (2026-11-11): controller + costmap local
# com odom/scan falsos em ROS_DOMAIN_ID=99 e mede Hz do laco do DWB por variante.
install(PROGRAMS
  scripts/bench_controller_offline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bench_controller_offline
)

# Bancada OFFLINE do SmacLattice (2026-11-10): map_server + planner_server em
# ROS_DOMAIN_ID=99 e mede % frente/lateral/re/giros por conjunto de primitivas.
install(PROGRAMS
  scripts/bench_lattice_offline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bench_lattice_offline
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Matematica pura do nudge_base (projecao da odometria, regra de aceite),
  # sem ROS ativo (2026-08-28).
  ament_add_pytest_test(test_nudge_math test/test_nudge_math.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # Logica pura do tour_docks (ordem dos docks, erro no frame do dock,
  # relatorio) e a espera de action servers do pre-flight, sem ROS ativo.
  ament_add_pytest_test(test_tour_docks test/test_tour_docks.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # compute_dock_distances num mapa sintetico (PGM, inflacao, Dijkstra,
  # fallback euclidiano, YAML), sem ROS ativo (2026-10-26).
  ament_add_pytest_test(test_compute_dock_distances test/test_compute_dock_distances.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # Helpers puros do mission_server (action /caramelo/run_mission usada pela
  # GUI e pelo painel web), sem grafo ROS.
  ament_add_pytest_test(test_mission_server_helpers test/test_mission_server_helpers.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # Rosbag por corrida: montagem do comando, politica de limpeza (pastas
  # falsas em tmp), opt-out e parada sem SIGKILL, sem ROS ativo (2026-11-11).
  ament_add_pytest_test(test_bag_common test/test_bag_common.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # Fumaca real (publica um topico falso, grava 5 s, le com bag_resumo).
  # Pula sozinho a menos que CARAMELO_BAG_SMOKE=1; forca ROS_DOMAIN_ID=99.
  ament_add_pytest_test(test_bag_common_ros test/test_bag_common_ros.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # Corrida completa (2026-11-13): vista do RViz, comando do ffmpeg, recorte dos
  # logs, pasta/metadata com tudo desligado, e as regras do analisar_run.
  ament_add_pytest_test(test_run_common test/test_run_common.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

ament_package()
