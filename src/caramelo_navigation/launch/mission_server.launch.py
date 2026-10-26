"""Sobe o servidor da action /caramelo/run_mission.

Fica separado do bringup de navegacao de proposito: o servidor precisa do stack
de manipulacao (move_group, pick/place) tanto quanto do Nav2, entao quem o inclui
e' o caramelo_bringup, que sobe os dois. Assim ele tambem pode ser subido
sozinho, para testar a missao com os fakes de manip_bt/test/.
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
        Node(
            package="caramelo_navigation",
            executable="mission_server",
            name="caramelo_mission_server",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
