# Sobe o health_monitor (DiagnosticArray em /diagnostics: rede/Pi, controllers,
# lidar, imu, odometria, tf, bateria, nav2) com config/diagnostics.yaml.
# Quem usa: incluido por caramelo_gui.launch.py (a GUI consome os status).
# Como roda: ros2 launch caramelo_diagnostics diagnostics.launch.py
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
            executable="health_monitor",
            name="health_monitor",
            output="screen",
            parameters=[config],
        ),
    ])
