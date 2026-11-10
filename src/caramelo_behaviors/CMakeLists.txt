cmake_minimum_required(VERSION 3.8)
project(caramelo_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp rclcpp_lifecycle rclcpp_action nav2_core nav2_behaviors nav2_costmap_2d
  nav2_util nav2_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs pluginlib)

# Plugin do behavior_server (pluginlib, base nav2_core::Behavior).
add_library(caramelo_escape_free_space_behavior SHARED src/escape_free_space.cpp)
target_include_directories(caramelo_escape_free_space_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(caramelo_escape_free_space_behavior PUBLIC cxx_std_17)
ament_target_dependencies(caramelo_escape_free_space_behavior ${dependencies})
# geometry_msgs::msg::Pose2D e' deprecated no Jazzy mas e' a API do checker do nav2.
target_compile_options(caramelo_escape_free_space_behavior PRIVATE -Wno-deprecated-declarations)
pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS caramelo_escape_free_space_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES behavior_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_escape_scoring test/test_escape_scoring.cpp)
  target_include_directories(test_escape_scoring PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_escape_scoring PRIVATE cxx_std_17)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(caramelo_escape_free_space_behavior)
ament_export_dependencies(${dependencies})
ament_package()
