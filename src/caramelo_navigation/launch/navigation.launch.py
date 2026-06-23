import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_costmap_resolution(context):
    requested_resolution = LaunchConfiguration("costmap_resolution").perform(context).strip()
    if requested_resolution.lower() != "map":
        return float(requested_resolution)

    map_name = LaunchConfiguration("map_name").perform(context)
    map_yaml_path = os.path.join(
        get_package_share_directory("caramelo_mapping"),
        "maps",
        map_name,
        "map.yaml",
    )

    try:
        with open(map_yaml_path, "r", encoding="utf-8") as map_yaml_file:
            map_data = yaml.safe_load(map_yaml_file) or {}
        return float(map_data.get("resolution", 0.20))
    except (OSError, TypeError, ValueError):
        return 0.20


def _make_costmap_resolution_params(resolution):
    params = {
        "local_costmap": {
            "local_costmap": {
                "ros__parameters": {
                    "resolution": resolution,
                },
            },
        },
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {
                    "resolution": resolution,
                },
            },
        },
    }

    params_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="caramelo_costmap_resolution_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )
    with params_file:
        yaml.safe_dump(params, params_file, sort_keys=False)
    return params_file.name


def _create_navigation_stack(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")
    use_cmd_vel_relay = LaunchConfiguration("use_cmd_vel_relay")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    mecanum_reference_topic = LaunchConfiguration("mecanum_reference_topic")

    lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]
    caramelo_navigation_pkg = get_package_share_directory("caramelo_navigation")
    costmap_resolution = _resolve_costmap_resolution(context)
    costmap_resolution_params = _make_costmap_resolution_params(costmap_resolution)

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "controller_server.yaml"),
            costmap_resolution_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "planner_server.yaml"),
            costmap_resolution_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "behavior_server.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "bt_navigator.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "smoother_server.yaml"),
            {"use_sim_time": use_sim_time},
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
            {"autostart": True},
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

    return [
        LogInfo(msg=f"Using costmap resolution: {costmap_resolution:.3f} m/cell"),
        delayed_navigation,
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("caramelo_localization"),
            "launch",
            "global_localization.launch.py",
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_name": map_name,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="small_house",
            description="Map folder name inside caramelo_mapping/maps/",
        ),
        DeclareLaunchArgument(
            "costmap_resolution",
            default_value="map",
            description="Use 'map' to read map.yaml resolution, or pass a numeric resolution in meters/cell",
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
        localization,
        OpaqueFunction(function=_create_navigation_stack),
    ])
