cmake_minimum_required(VERSION 3.8)
project(manip_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/perception_gate_node.py
    scripts/container_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Nucleo do detector de containers (segmentacao + projecao), sem ROS.
  ament_add_pytest_test(test_container_detector test/test_container_detector.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

ament_package()
