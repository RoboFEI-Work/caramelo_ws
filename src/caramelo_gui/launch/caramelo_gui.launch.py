import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Sobe o agregador de saude (/diagnostics) e a GUI. O diagnostico e' um node
    # separado (usavel headless tambem — R4); a GUI so' o consome.
    use_diagnostics = LaunchConfiguration("use_diagnostics")

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("caramelo_diagnostics"),
                "launch", "diagnostics.launch.py",
            )
        ),
        condition=IfCondition(use_diagnostics),
    )

    # Sem "name=": o processo tem 3 nos internos (bridge, rviz, markers) e o
    # remap de nome unico os colidia ("Publisher already registered").
    gui = Node(
        package="caramelo_gui",
        executable="caramelo_gui",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_diagnostics",
            default_value="true",
            description="Sobe o caramelo_diagnostics junto com a GUI",
        ),
        diagnostics,
        gui,
    ])
