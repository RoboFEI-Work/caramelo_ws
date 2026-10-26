# Sobe o health_monitor (DiagnosticArray em /diagnostics: rede/Pi, controllers,
# lidar, imu, odometria, tf, bateria, nav2) com config/diagnostics.yaml.
# Quem usa: incluido por caramelo_gui.launch.py (a GUI consome os status).
# Como roda: ros2 launch caramelo_diagnostics diagnostics.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("caramelo_diagnostics"),
        "config", "diagnostics.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="true em simulacao. O monitor mede FREQUENCIA de topicos: "
                        "com o relogio errado ele reporta Hz errado e acusa "
                        "sensor parado que na verdade esta publicando.",
        ),
        Node(
            package="caramelo_diagnostics",
            executable="health_monitor",
            name="health_monitor",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        ),
    ])
