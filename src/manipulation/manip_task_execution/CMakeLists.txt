cmake_minimum_required(VERSION 3.8)
project(manip_task_execution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2026-08-24: a IK custom (custom_ik.cpp, Newton amortecido com Eigen, 343
# sementes x 250 iteracoes) leva ~23 s por solve em -O0 e ~0,1 s em -O2. Um
# `colcon build` sem --cmake-args compila SEM otimizacao, e o pick "custom"
# ficava 200x mais lento em outro PC. Sem build type explicito, usa
# RelWithDebInfo (-O2 -g). Quem passar -DCMAKE_BUILD_TYPE mantem o seu.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Build type (default RelWithDebInfo: a IK custom precisa de -O2)" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(caramelo_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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

# Fila de alcance (2026-08-28): "quantos cm de lado a base precisa andar"
# verificado pela mesma IK/escada que o pick/place usam.
add_library(reach_shift src/reach_shift.cpp)
target_include_directories(reach_shift PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(reach_shift PUBLIC cxx_std_17)
target_link_libraries(reach_shift custom_ik)

# AMT1 (2026-08-28): planejador PURO da ordenacao por ciclos (slots por x,
# plano pick/place com buffer 2, oraculo simulatePlan, re-registro dos
# slots). Sem ROS: testado por gtest com as 720 permutacoes.
add_library(amt1_sort_planner src/amt1_sort_planner.cpp)
target_include_directories(amt1_sort_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(amt1_sort_planner PUBLIC cxx_std_17)

# Executavel de conferencia: compara o solver C++ com o script Python do
# operador numa grade de alvos (pega erro de transcricao).
add_executable(custom_ik_check src/custom_ik_check.cpp)
target_compile_features(custom_ik_check PUBLIC cxx_std_17)
target_link_libraries(custom_ik_check custom_ik reach_shift)

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
target_link_libraries(mtc_pick_action_node container_state_store custom_ik reach_shift)

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
target_link_libraries(mtc_place_action_node container_state_store custom_ik reach_shift)

# AMT1 (2026-08-28): servidor /amt1_sort — observa a mesa de precision
# placement, planeja (amt1_sort_planner) e executa picks/places como CLIENTE
# de /pick_tag e /place_tag, com nudge_base (caramelo_msgs) para alcance.
add_executable(amt1_sort_action_node src/amt1_sort_action_node.cpp)
target_include_directories(amt1_sort_action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(amt1_sort_action_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  amt1_sort_action_node
  "caramelo_msgs"
  "geometry_msgs"
  "moveit_ros_planning_interface"
  "my_robot_msgs"
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)
target_link_libraries(amt1_sort_action_node
  amt1_sort_planner container_state_store custom_ik yaml-cpp)

install(
  TARGETS
    container_state_store
    custom_ik
    reach_shift
    amt1_sort_planner
    custom_ik_check
    mtc_pick_action_node
    mtc_place_action_node
    amt1_sort_action_node
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

  # Fila de alcance (2026-08-28): a busca de deslocamento lateral e' testada
  # contra a IK REAL (alvos calibrados). Precisa de -O2 (ja forcado acima);
  # TIMEOUT folgado porque a IK roda ~10 buscas e a maquina pode estar
  # compilando outros pacotes em paralelo.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_reach_shift test/test_reach_shift.cpp TIMEOUT 300)
  target_link_libraries(test_reach_shift reach_shift)

  # AMT1 (2026-08-28): planejador puro, 720 permutacoes x 2 buffers contra o
  # oraculo simulatePlan (< 1 s; TIMEOUT folgado para maquina ocupada).
  ament_add_gtest(test_amt1_sort_planner test/test_amt1_sort_planner.cpp TIMEOUT 120)
  target_link_libraries(test_amt1_sort_planner amt1_sort_planner)
endif()

ament_package()
