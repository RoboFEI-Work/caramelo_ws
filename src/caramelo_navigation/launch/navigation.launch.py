import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")
    use_cmd_vel_relay = LaunchConfiguration("use_cmd_vel_relay")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    mecanum_reference_topic = LaunchConfiguration("mecanum_reference_topic")
    lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]
    caramelo_navigation_pkg = get_package_share_directory("caramelo_navigation")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )
    
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house",
        description="Map folder name inside caramelo_mapping/maps/"
    )

    navigation_start_delay_arg = DeclareLaunchArgument(
        "navigation_start_delay",
        default_value="5.0",
        description="Seconds to wait before starting Nav2 after localization starts"
    )

    use_cmd_vel_relay_arg = DeclareLaunchArgument(
        "use_cmd_vel_relay",
        default_value="true",
        description="Convert Nav2 cmd_vel Twist into mecanum_controller TwistStamped reference"
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Nav2 velocity command topic"
    )

    mecanum_reference_topic_arg = DeclareLaunchArgument(
        "mecanum_reference_topic",
        default_value="/mecanum_controller/reference",
        description="Mecanum controller TwistStamped command topic"
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("caramelo_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_name': map_name
        }.items()
    )

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            os.path.join(
                caramelo_navigation_pkg,
                "config",
                "controller_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )
    
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(
                caramelo_navigation_pkg,
                "config",
                "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            os.path.join(
                caramelo_navigation_pkg,
                "config",
                "behavior_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )
    
    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(
                caramelo_navigation_pkg,
                "config",
                "bt_navigator.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(
                caramelo_navigation_pkg,
                "config",
                "smoother_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    cmd_vel_relay = Node(
        package="caramelo_utils",
        executable="twist_relay.py",
        name="nav2_twist_relay",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_twist_topic": cmd_vel_topic},
            {"output_twist_stamped_topic": mecanum_reference_topic},
            {"frame_id": "base_footprint"},
        ],
        condition=IfCondition(use_cmd_vel_relay),
    )

    delayed_navigation = TimerAction(
        period=navigation_start_delay,
        actions=[
            cmd_vel_relay,
            nav2_controller_server,
            nav2_planner_server,
            nav2_smoother_server,
            nav2_behaviors,
            nav2_bt_navigator,
            nav2_lifecycle_manager,
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        navigation_start_delay_arg,
        use_cmd_vel_relay_arg,
        cmd_vel_topic_arg,
        mecanum_reference_topic_arg,
        localization,
        delayed_navigation,
    ])
