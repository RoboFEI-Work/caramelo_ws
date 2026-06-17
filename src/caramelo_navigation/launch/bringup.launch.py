import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")
    use_cmd_vel_relay = LaunchConfiguration("use_cmd_vel_relay")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    mecanum_reference_topic = LaunchConfiguration("mecanum_reference_topic")

    navigation_launch = os.path.join(
        get_package_share_directory("caramelo_navigation"),
        "launch",
        "navigation.launch.py",
    )

    default_rviz_config = os.path.join(
        get_package_share_directory("caramelo_localization"),
        "rviz",
        "global_localization.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="small_house",
            description="Map folder name inside caramelo_mapping/maps/",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Open RViz automatically",
        ),
        DeclareLaunchArgument(
            "navigation_start_delay",
            default_value="5.0",
            description="Seconds to wait before starting Nav2 after localization starts",
        ),
        DeclareLaunchArgument(
            "use_cmd_vel_relay",
            default_value="true",
            description="Convert Nav2 cmd_vel Twist into mecanum_controller TwistStamped reference",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Nav2 velocity command topic",
        ),
        DeclareLaunchArgument(
            "mecanum_reference_topic",
            default_value="/mecanum_controller/reference",
            description="Mecanum controller TwistStamped command topic",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Absolute path to RViz config file",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map_name": map_name,
                "navigation_start_delay": navigation_start_delay,
                "use_cmd_vel_relay": use_cmd_vel_relay,
                "cmd_vel_topic": cmd_vel_topic,
                "mecanum_reference_topic": mecanum_reference_topic,
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_navigation",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
