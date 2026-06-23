import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")
    map_resolution = LaunchConfiguration("map_resolution")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    scan_topic = LaunchConfiguration("scan_topic")
    slam_scan_topic = LaunchConfiguration("slam_scan_topic")
    use_scan_normalizer = LaunchConfiguration("use_scan_normalizer")
    fixed_scan_count = LaunchConfiguration("fixed_scan_count")
    scan_range_max = LaunchConfiguration("scan_range_max")

    ros_distro = os.environ["ROS_DISTRO"]
    lifecycle_nodes = ["map_saver_server", "slam_toolbox"]
    # Note: Both nodes are lifecycle-managed for better control

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    map_resolution_arg = DeclareLaunchArgument(
        "map_resolution",
        default_value="0.02",
        description="Default SLAM map resolution in meters/cell"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("caramelo_mapping"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to slam yaml file to load"
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("caramelo_mapping"),
            "rviz",
            "slam.rviz"
        ),
        description="Full path to RViz config file"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Open RViz automatically"
    )

    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="Raw LaserScan topic coming from the robot"
    )

    slam_scan_topic_arg = DeclareLaunchArgument(
        "slam_scan_topic",
        default_value="/scan_fixed",
        description="Fixed-size LaserScan topic consumed by slam_toolbox"
    )

    use_scan_normalizer_arg = DeclareLaunchArgument(
        "use_scan_normalizer",
        default_value="true",
        description="Republish raw scans on a fixed angular grid for slam_toolbox"
    )

    fixed_scan_count_arg = DeclareLaunchArgument(
        "fixed_scan_count",
        default_value="0",
        description="Fixed scan beam count. Use 0 to learn it from the first scan"
    )

    scan_range_max_arg = DeclareLaunchArgument(
        "scan_range_max",
        default_value="30.0",
        description="Maximum laser range in meters for the fixed scan"
    )

    scan_normalizer = Node(
        package="caramelo_mapping",
        executable="scan_normalizer.py",
        name="scan_normalizer",
        output="screen",
        parameters=[
            {"input_scan_topic": scan_topic},
            {"output_scan_topic": slam_scan_topic},
            {"target_count": ParameterValue(fixed_scan_count, value_type=int)},
            {"range_max": ParameterValue(scan_range_max, value_type=float)},
        ],
        condition=IfCondition(use_scan_normalizer),
    )
    
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},     # Conservative free threshold
            {"occupied_thresh_default": 0.65},  # Conservative occupied threshold
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",  # Synchronous mode for real-time
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"resolution": map_resolution},
            {"scan_topic": slam_scan_topic},
            {"max_laser_range": ParameterValue(scan_range_max, value_type=float)},
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 60.0}
        ],
    )


    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_slam",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_resolution_arg,
        slam_config_arg,
        rviz_config_arg,
        use_rviz_arg,
        scan_topic_arg,
        slam_scan_topic_arg,
        use_scan_normalizer_arg,
        fixed_scan_count_arg,
        scan_range_max_arg,
        scan_normalizer,
        nav2_map_saver,
        slam_toolbox,
        nav2_lifecycle_manager,
        rviz,
    ])