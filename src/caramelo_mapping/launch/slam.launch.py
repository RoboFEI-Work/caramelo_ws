import os
import subprocess
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _avisar_se_conflito(nos_conflitantes, este_modo, outro_modo):
    """Aviso RK-02: SLAM e AMCL publicam ambos map->odom; rodar os dois faz o TF
    piscar entre as duas estimativas. IMPORTANTE: apenas AVISA (nao aborta) — o
    `ros2 node list` pode mostrar nos zumbis de um daemon velho, e um raise aqui
    derrubava o launch inteiro antes de qualquer no subir."""
    try:
        saida = subprocess.run(
            ["ros2", "node", "list"], capture_output=True, text=True, timeout=4
        ).stdout
    except Exception:
        return
    ativos = set(saida.split())
    presentes = [no for no in nos_conflitantes if no in ativos]
    if presentes:
        print(
            f"\n[CARAMELO][AVISO RK-02] {outro_modo} parece estar rodando "
            f"({', '.join(presentes)}) enquanto sobe {este_modo}.\n"
            f"SLAM e AMCL publicam ambos map->odom — se for real (nao zumbi do "
            f"daemon), encerre {outro_modo}. Se for zumbi: `ros2 daemon stop`.\n",
            flush=True,
        )


def generate_launch_description():

    # RK-02: avisa (sem abortar) se a localizacao (AMCL) parecer ativa.
    _avisar_se_conflito(["/amcl"], "o mapeamento (SLAM)", "a localizacao (AMCL)")


    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")
    map_resolution = LaunchConfiguration("map_resolution")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    slam_scan_topic = LaunchConfiguration("slam_scan_topic")
    scan_range_max = LaunchConfiguration("scan_range_max")
    map_name = LaunchConfiguration("map_name")
    use_service_area_markers = LaunchConfiguration("use_service_area_markers")
    service_area_marker_topic = LaunchConfiguration("service_area_marker_topic")

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

    slam_scan_topic_arg = DeclareLaunchArgument(
        "slam_scan_topic",
        default_value="/scan",
        description="LaserScan topic consumed by slam_toolbox"
    )

    scan_range_max_arg = DeclareLaunchArgument(
        "scan_range_max",
        default_value="30.0",
        description="Maximum laser range in meters for the fixed scan"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="sala_520",
        description="Map folder used to load service_areas.yaml markers"
    )

    use_service_area_markers_arg = DeclareLaunchArgument(
        "use_service_area_markers",
        default_value="true",
        description="Publish service area markers during mapping"
    )

    service_area_marker_topic_arg = DeclareLaunchArgument(
        "service_area_marker_topic",
        default_value="/caramelo/service_areas/markers",
        description="MarkerArray topic for service areas"
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
            # slam_toolbox remains operational, but its bond heartbeat is
            # unreliable with the incoming hardware scan pipeline. Disabling
            # bond monitoring prevents lifecycle cleanup from erasing the map.
            {"bond_timeout": 0.0}
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

    return LaunchDescription([
        use_sim_time_arg,
        map_resolution_arg,
        slam_config_arg,
        rviz_config_arg,
        use_rviz_arg,
        slam_scan_topic_arg,
        scan_range_max_arg,
        map_name_arg,
        use_service_area_markers_arg,
        service_area_marker_topic_arg,
        nav2_map_saver,
        slam_toolbox,
        service_area_markers,
        nav2_lifecycle_manager,
        rviz,
    ])
