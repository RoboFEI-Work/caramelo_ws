cmake_minimum_required(VERSION 3.8)
project(caramelo_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(caramelo_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

# Qt: gerar MOC automaticamente para classes com Q_OBJECT nos headers incluidos.
set(CMAKE_AUTOMOC ON)

set(SOURCES
  src/main.cpp
  src/bridge/ros_bridge.cpp
  src/bridge/manual_localization.cpp
  src/core/rviz_frame.cpp
  src/core/state_machine.cpp
  src/core/launch_runner.cpp
  src/core/main_window.cpp
  src/modules/inicio/inicio_module.cpp
  src/modules/navegacao/navegacao_module.cpp
  src/modules/docking/docking_module.cpp
  src/modules/localizacao/localizacao_module.cpp
  src/modules/mapas/mapas_module.cpp
  src/modules/mapeamento/mapeamento_module.cpp
  src/modules/teleop/teleop_module.cpp
  src/modules/service_areas/service_areas_module.cpp
)

add_executable(caramelo_gui ${SOURCES})
target_include_directories(caramelo_gui PRIVATE src)

ament_target_dependencies(caramelo_gui
  ament_index_cpp
  rclcpp
  rclcpp_action
  diagnostic_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  interactive_markers
  tf2_ros
  nav2_msgs
  caramelo_msgs
  rviz_common
  rviz_rendering
  rviz_default_plugins
)

target_link_libraries(caramelo_gui
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)

install(TARGETS caramelo_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resources launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
