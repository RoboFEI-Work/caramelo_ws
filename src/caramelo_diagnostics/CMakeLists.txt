cmake_minimum_required(VERSION 3.8)
project(caramelo_diagnostics)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Modulo Python importavel: caramelo_diagnostics/
ament_python_install_package(${PROJECT_NAME})

# Executavel chamavel por ros2 run
install(PROGRAMS
  scripts/health_aggregator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME health_aggregator
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
