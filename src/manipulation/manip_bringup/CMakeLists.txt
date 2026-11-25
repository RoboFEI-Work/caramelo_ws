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
    scripts/pp_furo_node.py
    scripts/pp_furo.py
    scripts/amt_sort_node.py
    scripts/amt_sort_core.py
    scripts/workstation_surface_node.py
    scripts/vision_debug_node.py
    scripts/task_viewer_node.py
    scripts/calib_camera_record.py
    scripts/calib_camera_solve.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Nucleo do detector de containers (segmentacao + projecao), sem ROS.
  ament_add_pytest_test(test_container_detector test/test_container_detector.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # Mapa da mesa: segmentacao + projecao + fusao, sem ROS.
  ament_add_pytest_test(test_workstation_surface test/test_workstation_surface.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # Visao de depuracao: projecao, pintura da grade, tags, barra, gate de idade, sem ROS.
  ament_add_pytest_test(test_vision_debug test/test_vision_debug.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # Mesa de precisao (2026-11-16): furo e cubo medidos em quadros reais da D455.
  ament_add_pytest_test(test_pp_furo test/test_pp_furo.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # AMT refeita (2026-11-16): quem esta' em qual slot e o plano de trocas.
  ament_add_pytest_test(test_amt_sort_core test/test_amt_sort_core.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

ament_package()
