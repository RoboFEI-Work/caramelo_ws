# Bringup unificado do Caramelo: navegacao (caramelo_navigation) + manipulacao
# (manip_bringup/manip_stack). Roda no PC; o hardware (base + braco) sobe na Pi
# via raspberry_bringup, exceto em use_mock_hardware=true (braco mock no PC,
# sem navegacao real).
#
# use_gui: declarado mas SEM efeito por ora — GUI fora do escopo; abrir
# manualmente (ros2 launch caramelo_gui ...).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_nav = LaunchConfiguration("use_nav")
    map_name = LaunchConfiguration("map_name")
    use_camera = LaunchConfiguration("use_camera")
    use_apriltag = LaunchConfiguration("use_apriltag")
    use_audio = LaunchConfiguration("use_audio")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_manip = LaunchConfiguration("use_rviz_manip")
    use_docking = LaunchConfiguration("use_docking")
    nav_profile = LaunchConfiguration("nav_profile")
    use_keepout = LaunchConfiguration("use_keepout")
    use_sim_time = LaunchConfiguration("use_sim_time")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")
    costmap_resolution = LaunchConfiguration("costmap_resolution")
    use_route_server = LaunchConfiguration("use_route_server")
    use_waypoint_follower = LaunchConfiguration("use_waypoint_follower")
    use_cmd_vel_relay = LaunchConfiguration("use_cmd_vel_relay")
    use_service_area_manager = LaunchConfiguration("use_service_area_manager")
    use_service_area_markers = LaunchConfiguration("use_service_area_markers")

    nav_bringup_launch = os.path.join(
        get_package_share_directory("caramelo_navigation"),
        "launch",
        "bringup.launch.py",
    )

    manip_stack_launch = os.path.join(
        get_package_share_directory("manip_bringup"),
        "launch",
        "manip_stack.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Braco mock no PC (manip_stack); sem hardware real",
        ),
        DeclareLaunchArgument(
            "use_nav",
            default_value="true",
            description="Inclui a navegacao (caramelo_navigation bringup)",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="arena3_520",
            description="Nome da pasta do mapa em caramelo_mapping/maps/",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Inicia a RealSense no stack de manipulacao",
        ),
        # Repasses para o manip_stack (2026-08-10): sem eles nao dava para
        # ajustar camera nem as verificacoes de pega pela missao completa.
        DeclareLaunchArgument(
            "camera_profile",
            default_value="640x480x5",
            description=(
                "Perfil WxHxFPS da RealSense. 640x480x5 e o unico estavel em "
                "USB 2; com a camera em USB 3 use 640x480x15."
            ),
        ),
        DeclareLaunchArgument(
            "verify_grasp_effort",
            default_value="true",
            description=(
                "Confere pelo esforco se a garra realmente pegou o bloco. "
                "false = comportamento historico (segue mesmo pegando nada)."
            ),
        ),
        DeclareLaunchArgument(
            "max_pose_error_m",
            default_value="0.025",
            description=(
                "Erro maximo entre a pose pedida e onde a ponta parou. "
                "0.0 desliga a checagem (aceita falha silenciosa)."
            ),
        ),
        DeclareLaunchArgument(
            "use_apriltag",
            default_value="true",
            description="Inicia o apriltag_node no stack de manipulacao",
        ),
        DeclareLaunchArgument(
            "use_audio",
            default_value="true",
            description="Inicia o speech_node (Piper) no stack de manipulacao",
        ),
        DeclareLaunchArgument(
            "use_gui",
            default_value="false",
            description="GUI fora do escopo; abrir manualmente (sem efeito por ora)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Abre o RViz da navegacao",
        ),
        DeclareLaunchArgument(
            "use_rviz_manip",
            default_value="false",
            description="Abre o RViz da manipulacao",
        ),
        DeclareLaunchArgument(
            "use_docking",
            default_value="true",
            description="Inicia o Nav2 Docking Server com este mapa",
        ),
        DeclareLaunchArgument(
            "nav_profile",
            default_value="dwb",
            description="Perfil de navegacao: dwb ou mppi",
        ),
        DeclareLaunchArgument(
            "use_keepout",
            default_value="false",
            description="Ativa o Keepout Filter (parede virtual RK-04)",
        ),
        # --- Passthroughs do bringup.launch.py (mesmos defaults) ---
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Usa clock simulado se true",
        ),
        DeclareLaunchArgument(
            "navigation_start_delay",
            default_value="5.0",
            description="Segundos de espera antes do Nav2 apos a localizacao",
        ),
        DeclareLaunchArgument(
            "costmap_resolution",
            default_value="0.02",
            description="Resolucao do costmap em m/celula ('map' usa a do map.yaml)",
        ),
        DeclareLaunchArgument(
            "use_route_server",
            default_value="true",
            description="Inicia o Nav2 Route Server com o route_graph do mapa",
        ),
        DeclareLaunchArgument(
            "use_waypoint_follower",
            default_value="true",
            description="Inicia o Nav2 Waypoint Follower",
        ),
        DeclareLaunchArgument(
            "use_cmd_vel_relay",
            default_value="true",
            description="Converte cmd_vel Twist em TwistStamped p/ mecanum_controller",
        ),
        DeclareLaunchArgument(
            "use_service_area_manager",
            default_value="true",
            description="Inicia a API de service areas (service/action)",
        ),
        DeclareLaunchArgument(
            "use_service_area_markers",
            default_value="true",
            description="Publica MarkerArray das service areas p/ RViz",
        ),
        # --- Navegacao (opcional) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_bringup_launch),
            condition=IfCondition(use_nav),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map_name": map_name,
                "use_rviz": use_rviz,
                "use_docking": use_docking,
                "use_keepout": use_keepout,
                "navigation_start_delay": navigation_start_delay,
                "costmap_resolution": costmap_resolution,
                "nav_profile": nav_profile,
                "use_route_server": use_route_server,
                "use_waypoint_follower": use_waypoint_follower,
                "use_cmd_vel_relay": use_cmd_vel_relay,
                "use_service_area_manager": use_service_area_manager,
                "use_service_area_markers": use_service_area_markers,
            }.items(),
        ),
        # --- Manipulacao (sempre) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manip_stack_launch),
            launch_arguments={
                "use_mock_hardware": use_mock_hardware,
                "use_camera": use_camera,
                "use_apriltag": use_apriltag,
                "use_audio": use_audio,
                "use_rviz_manip": use_rviz_manip,
                "camera_profile": LaunchConfiguration("camera_profile"),
                "verify_grasp_effort": LaunchConfiguration("verify_grasp_effort"),
                "max_pose_error_m": LaunchConfiguration("max_pose_error_m"),
            }.items(),
        ),
    ])
