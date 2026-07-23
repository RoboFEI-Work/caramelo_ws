cmake_minimum_required(VERSION 3.8)
project(manip_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(test_moveit src/tests_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)

add_executable(commmander src/commander_template.cpp)
ament_target_dependencies(commmander rclcpp moveit_ros_planning_interface example_interfaces)

install(TARGETS
  test_moveit
  commmander
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
