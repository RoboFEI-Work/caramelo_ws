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
  scripts/caramelo_service_area_common.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/init_keepout_mask.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
