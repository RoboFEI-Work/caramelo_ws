"""Sobe o servidor da action /caramelo/run_mission.

Fica separado do bringup de navegacao de proposito: o servidor precisa do stack
de manipulacao (move_group, pick/place) tanto quanto do Nav2, entao quem o inclui
e' o caramelo_bringup, que sobe os dois. Assim ele tambem pode ser subido
sozinho, para testar a missao com os fakes de manip_bt/test/.

bag_enabled / bag_extra / bag_camera: rosbag por corrida (caramelo_bag_common),
mesmos efeitos de --gravar / --bag-extra / --bag-camera do run_mission.
bag_enabled e' false por padrao desde 2026-11-25 (a gravacao pesava no PC).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Usa o clock simulado (true em Gazebo)",
        ),
        DeclareLaunchArgument(
            "bag_enabled", default_value="false",  # 2026-11-25: desligado por padrao
            description="Grava o rosbag da corrida em ~/bags (CARAMELO_BAG=0 tambem desliga)",
        ),
        DeclareLaunchArgument(
            "bag_extra", default_value="false",
            description="Bag com os topicos EXTRA (costmap global, particle_cloud, scan_raw...)",
        ),
        DeclareLaunchArgument(
            "bag_camera", default_value="false",
            description="Bag com a camera do punho anotada (JPEG via republish)",
        ),
        Node(
            package="caramelo_navigation",
            executable="mission_server",
            name="caramelo_mission_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "bag_enabled": LaunchConfiguration("bag_enabled"),
                "bag_extra": LaunchConfiguration("bag_extra"),
                "bag_camera": LaunchConfiguration("bag_camera"),
            }],
        ),
    ])
