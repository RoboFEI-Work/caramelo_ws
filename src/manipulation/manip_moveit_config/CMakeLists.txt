cmake_minimum_required(VERSION 3.22)
project(manip_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

# launch/: move_group_caramelo.launch.py (robo real) e move_group.launch.py
# (bancada standalone).
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
