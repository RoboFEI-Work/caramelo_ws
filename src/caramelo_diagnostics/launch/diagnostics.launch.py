import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("caramelo_diagnostics"),
        "config", "diagnostics.yaml",
    )
    return LaunchDescription([
        Node(
            package="caramelo_diagnostics",
            executable="health_aggregator",
            name="caramelo_health_monitor",
            output="screen",
            parameters=[config],
        ),
    ])
