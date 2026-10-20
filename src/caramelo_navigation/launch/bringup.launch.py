# Bringup de navegacao do PC: navigation.launch.py (localizacao + Nav2 +
# docking + service areas) mais o RViz com a config de global_localization.
# Quem usa: a GUI ("Ligar navegacao") e caramelo_bringup/robot_manipulation.launch.py
# (com use_rviz:=false). Como roda: ros2 launch caramelo_navigation bringup.launch.py map_name:=<mapa>
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
    use_docking = LaunchConfiguration("use_docking")
    use_keepout = LaunchConfiguration("use_keepout")
    rviz_config = LaunchConfiguration("rviz_config")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")
    costmap_resolution = LaunchConfiguration("costmap_resolution")
    nav_profile = LaunchConfiguration("nav_profile")
    use_route_server = LaunchConfiguration("use_route_server")
    use_waypoint_follower = LaunchConfiguration("use_waypoint_follower")
    use_cmd_vel_relay = LaunchConfiguration("use_cmd_vel_relay")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    mecanum_reference_topic = LaunchConfiguration("mecanum_reference_topic")
    use_service_area_manager = LaunchConfiguration("use_service_area_manager")
    use_service_area_markers = LaunchConfiguration("use_service_area_markers")
    service_area_marker_topic = LaunchConfiguration("service_area_marker_topic")

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
            description="Usa o relogio simulado (/clock)",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="sala_520",
            description="Nome da pasta do mapa em caramelo_mapping/maps/",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Abre o RViz junto",
        ),
        DeclareLaunchArgument(
            "use_docking",
            default_value="true",
            description="Sobe o Docking Server (opennav_docking + dock_align_node) com o docking.yaml do mapa",
        ),
        DeclareLaunchArgument(
            "use_keepout",
            default_value="false",
            description="Ativa o Keepout Filter (parede virtual RK-04). Requer "
                        "maps/<mapa>/keepout_mask.{yaml,pgm}.",
        ),
        DeclareLaunchArgument(
            "navigation_start_delay",
            default_value="5.0",
            description="Segundos de espera entre subir a localizacao e subir o Nav2",
        ),
        DeclareLaunchArgument(
            "costmap_resolution",
            default_value="0.02",
            description="Resolucao dos costmaps em m/celula. 'map' usa a resolucao do map.yaml",
        ),
        DeclareLaunchArgument(
            "nav_profile",
            default_value="dwb",
            description="Perfil de navegacao (BT + controlador): dwb ou mppi",
        ),
        DeclareLaunchArgument(
            "use_route_server",
            default_value="true",
            description="Sobe o Route Server do Nav2 com o route_graph.geojson do mapa (so se tiver nos e arestas)",
        ),
        DeclareLaunchArgument(
            "use_waypoint_follower",
            default_value="true",
            description="Sobe o Waypoint Follower do Nav2",
        ),
        DeclareLaunchArgument(
            "use_cmd_vel_relay",
            default_value="true",
            description="Converte o Twist de cmd_vel do Nav2 em TwistStamped na referencia do mecanum_controller",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Topico final de velocidade (saida do collision_monitor)",
        ),
        DeclareLaunchArgument(
            "mecanum_reference_topic",
            default_value="/mecanum_controller/reference",
            description="Topico TwistStamped de comando do mecanum_controller",
        ),
        DeclareLaunchArgument(
            "use_service_area_manager",
            default_value="true",
            description="Sobe o service_area_manager_node (services list/validate/init + action save_pose)",
        ),
        DeclareLaunchArgument(
            "use_service_area_markers",
            default_value="true",
            description="Publica o MarkerArray das service areas para RViz/GUI",
        ),
        DeclareLaunchArgument(
            "service_area_marker_topic",
            default_value="/caramelo/service_areas/markers",
            description="Topico do MarkerArray das service areas",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Caminho absoluto da config do RViz",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map_name": map_name,
                "use_docking": use_docking,
                "use_keepout": use_keepout,
                "navigation_start_delay": navigation_start_delay,
                "costmap_resolution": costmap_resolution,
                "nav_profile": nav_profile,
                "use_route_server": use_route_server,
                "use_waypoint_follower": use_waypoint_follower,
                "use_cmd_vel_relay": use_cmd_vel_relay,
                "cmd_vel_topic": cmd_vel_topic,
                "mecanum_reference_topic": mecanum_reference_topic,
                "use_service_area_manager": use_service_area_manager,
                "use_service_area_markers": use_service_area_markers,
                "service_area_marker_topic": service_area_marker_topic,
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
