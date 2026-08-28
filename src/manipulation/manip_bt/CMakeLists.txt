cmake_minimum_required(VERSION 3.8)
project(manip_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(caramelo_msgs REQUIRED)

add_executable(manip_bt_executor
  src/bt_executor.cpp
  src/go_to_named_pose_bt.cpp
  src/pick_tag_bt.cpp
  src/place_tag_bt.cpp
)

add_executable(bt_yaml_executor
  src/bt_yaml_executor.cpp
  src/go_to_named_pose_bt.cpp
  src/go_to_ws_bt.cpp
  src/mission_map_config.cpp
  src/nudge_base_bt.cpp
  src/pick_tag_bt.cpp
  src/place_tag_bt.cpp
  src/reach_queue_bt.cpp
)

add_executable(competition_yaml_translator
  src/competition_yaml_translator.cpp
)

add_executable(task_planner
  src/task_planner.cpp
)

target_include_directories(manip_bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(manip_bt_executor PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  manip_bt_executor
  ament_index_cpp
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  moveit_ros_planning_interface
  my_robot_msgs
)

target_include_directories(bt_yaml_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(bt_yaml_executor PUBLIC c_std_99 cxx_std_17)

target_link_libraries(bt_yaml_executor yaml-cpp)
ament_target_dependencies(
  bt_yaml_executor
  ament_index_cpp
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  moveit_ros_planning_interface
  my_robot_msgs
  nav2_msgs
  caramelo_msgs
)

install(TARGETS manip_bt_executor bt_yaml_executor
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(competition_yaml_translator yaml-cpp)
ament_target_dependencies(
  competition_yaml_translator
  ament_index_cpp
)

install(TARGETS competition_yaml_translator
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(task_planner yaml-cpp)
ament_target_dependencies(
  task_planner
  ament_index_cpp
)

install(TARGETS task_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY behavior_tree_manip/
  DESTINATION share/${PROJECT_NAME}/behavior_tree_manip
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Fila de alcance (2026-08-28): o ReachQueue e' logica pura de BT — testa
  # com folhas roteirizadas, sem ROS no ar (so' o logger do rclcpp).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_reach_queue
    test/test_reach_queue.cpp
    src/reach_queue_bt.cpp
  )
  target_include_directories(test_reach_queue PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_compile_features(test_reach_queue PUBLIC cxx_std_17)
  ament_target_dependencies(test_reach_queue rclcpp behaviortree_cpp_v3)
endif()

ament_package()
