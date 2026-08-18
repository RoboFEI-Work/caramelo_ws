cmake_minimum_required(VERSION 3.8)
project(manip_task_execution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(my_robot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(container_state_store src/container_state_store.cpp)
target_include_directories(container_state_store PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(container_state_store PUBLIC cxx_std_17)
target_link_libraries(container_state_store yaml-cpp)

# IK propria do braco (5 GDL): prateleira, mesa comum (tentativas 1-2 do
# ladder do pick) e entrega em alvo TF do place (2026-08-17).
# Ver o cabecalho de include/manip_task_execution/custom_ik.hpp para o porque.
add_library(custom_ik src/custom_ik.cpp)
target_include_directories(custom_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(custom_ik PUBLIC cxx_std_17)

# Executavel de conferencia: compara o solver C++ com o script Python do
# operador numa grade de alvos (pega erro de transcricao).
add_executable(custom_ik_check src/custom_ik_check.cpp)
target_compile_features(custom_ik_check PUBLIC cxx_std_17)
target_link_libraries(custom_ik_check custom_ik)

add_executable(mtc_pick_action_node src/mtc_pick_action_node.cpp)
target_include_directories(mtc_pick_action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(mtc_pick_action_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  mtc_pick_action_node
  "moveit_task_constructor_core"
  "moveit_ros_planning_interface"
  "my_robot_msgs"
  "control_msgs"
  "rclcpp"
  "rclcpp_action"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)
target_link_libraries(mtc_pick_action_node container_state_store custom_ik)

add_executable(mtc_place_action_node src/mtc_place_action_node.cpp)
target_include_directories(mtc_place_action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(mtc_place_action_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  mtc_place_action_node
  "control_msgs"
  "moveit_ros_planning_interface"
  "my_robot_msgs"
  "rclcpp"
  "rclcpp_action"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)
target_link_libraries(mtc_place_action_node container_state_store custom_ik)

install(
  TARGETS
    container_state_store
    custom_ik
    custom_ik_check
    mtc_pick_action_node
    mtc_place_action_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
endif()

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
endif()

ament_package()
