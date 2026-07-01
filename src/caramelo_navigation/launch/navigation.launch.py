import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    use_docking = LaunchConfiguration("use_docking")
    map_name = LaunchConfiguration("map_name")
    use_cmd_vel_relay = LaunchConfiguration("use_cmd_vel_relay")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    mecanum_reference_topic = LaunchConfiguration("mecanum_reference_topic")
    use_service_area_manager = LaunchConfiguration("use_service_area_manager")
    use_service_area_markers = LaunchConfiguration("use_service_area_markers")
    service_area_marker_topic = LaunchConfiguration("service_area_marker_topic")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "bt_navigator",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
    ]
    caramelo_navigation_pkg = get_package_share_directory("caramelo_navigation")
    docking_launch = os.path.join(caramelo_navigation_pkg, "launch", "docking_server.launch.py")
    bt_xml = os.path.join(caramelo_navigation_pkg, "behavior_tree", "caramelo_theta_dwb.xml")
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
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
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
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
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
            {"default_nav_to_pose_bt_xml": bt_xml},
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

    nav2_velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "velocity_smoother.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
        ],
    )

    nav2_collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "collision_monitor.yaml"),
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

    service_area_manager = Node(
        package="caramelo_navigation",
        executable="service_area_manager_node",
        name="service_area_manager_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"default_map_name": map_name},
        ],
        condition=IfCondition(use_service_area_manager),
    )

    service_area_markers = Node(
        package="caramelo_navigation",
        executable="service_area_markers_node",
        name="service_area_markers_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_name": map_name},
            {"marker_topic": service_area_marker_topic},
        ],
        condition=IfCondition(use_service_area_markers),
    )

    docking_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(docking_launch),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": use_sim_time,
            "base_frame": "base_footprint",
            "fixed_frame": "odom",
        }.items(),
        condition=IfCondition(use_docking),
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
            nav2_velocity_smoother,
            nav2_collision_monitor,
            nav2_lifecycle_manager,
            service_area_manager,
            service_area_markers,
            docking_server,
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
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("caramelo_localization"),
                "launch",
                "global_localization.launch.py",
            )
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
            default_value="sala_520",
            description="Map folder name inside caramelo_mapping/maps/",
        ),
        DeclareLaunchArgument(
            "costmap_resolution",
            default_value="0.02",
            description="Costmap resolution in meters/cell. Pass 'map' to use map.yaml resolution",
        ),
        DeclareLaunchArgument(
            "use_docking",
            default_value="false",
            description="Start Nav2 Docking Server using this map folder",
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
            description="Final velocity command topic after collision monitor",
        ),
        DeclareLaunchArgument(
            "mecanum_reference_topic",
            default_value="/mecanum_controller/reference",
            description="Mecanum controller TwistStamped command topic",
        ),
        DeclareLaunchArgument(
            "use_service_area_manager",
            default_value="true",
            description="Start Service Area service/action API",
        ),
        DeclareLaunchArgument(
            "use_service_area_markers",
            default_value="true",
            description="Publish service area MarkerArray for RViz",
        ),
        DeclareLaunchArgument(
            "service_area_marker_topic",
            default_value="/caramelo/service_areas/markers",
            description="MarkerArray topic for service areas",
        ),
        localization,
        OpaqueFunction(function=_create_navigation_stack),
    ])
