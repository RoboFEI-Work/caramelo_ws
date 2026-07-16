import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _abortar_se_conflito(nos_conflitantes, este_modo, outro_modo):
    """Trava estrutural RK-02: SLAM e AMCL publicam ambos map->odom; rodar os dois
    faz o TF piscar entre as duas estimativas. Se o modo conflitante ja estiver no
    ar, aborta este launch com mensagem clara. Best-effort: se a checagem falhar
    (sem daemon/grafo), nao bloqueia."""
    try:
        saida = subprocess.run(
            ["ros2", "node", "list"], capture_output=True, text=True, timeout=4
        ).stdout
    except Exception:
        return
    ativos = set(saida.split())
    presentes = [no for no in nos_conflitantes if no in ativos]
    if presentes:
        raise RuntimeError(
            f"\n[CARAMELO] Abortando {este_modo}: detectei {outro_modo} rodando "
            f"({', '.join(presentes)}).\nSLAM e AMCL publicam ambos map->odom; rodar "
            f"os dois faz o TF piscar (RK-02).\nEncerre {outro_modo} antes de subir "
            f"{este_modo}."
        )


def generate_launch_description():

    # RK-02: nao subir a localizacao se o mapeamento (SLAM) ja estiver ativo.
    _abortar_se_conflito(["/slam_toolbox"], "a localizacao (AMCL)", "o mapeamento (SLAM)")


    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
    )

    map_name_arg  = DeclareLaunchArgument(
        "map_name",
        default_value="sala_520",
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("caramelo_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )

    # Keepout Filter (RK-04): paredes virtuais invisiveis ao LiDAR (fita zebrada).
    # Default desligado; ligar com use_keepout:=true depois de pintar a mascara
    # maps/<mapa>/keepout_mask.pgm (ver docs/keepout_paredes_virtuais.md).
    use_keepout_arg = DeclareLaunchArgument(
        "use_keepout",
        default_value="false",
        description="Sobe os servidores do Keepout Filter (mascara de parede virtual)",
    )

    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")
    use_keepout = LaunchConfiguration("use_keepout")

    lifecycle_nodes = ["map_server", "amcl"]

    map_path = PathJoinSubstitution([
        get_package_share_directory("caramelo_mapping"),
        "maps",
        map_name,
        "map.yaml",
    ])

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
        ],
    )

    # --- Keepout Filter (RK-04) — so sobe com use_keepout:=true ---------------
    keepout_filter_config = os.path.join(
        get_package_share_directory("caramelo_localization"),
        "config",
        "keepout_filter.yaml",
    )
    keepout_mask_path = PathJoinSubstitution([
        get_package_share_directory("caramelo_mapping"),
        "maps",
        map_name,
        "keepout_mask.yaml",
    ])

    # Serve a mascara de keepout como OccupancyGrid.
    filter_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="filter_mask_server",
        output="screen",
        parameters=[
            keepout_filter_config,
            {"yaml_filename": keepout_mask_path},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_keepout),
    )

    # Publica o CostmapFilterInfo consumido pelo plugin KeepoutFilter dos costmaps.
    costmap_filter_info_server = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        output="screen",
        parameters=[
            keepout_filter_config,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_keepout),
    )

    # Lifecycle manager dedicado aos servidores de filtro (isolado do de
    # localizacao para nao precisar montar a lista condicionalmente).
    lifecycle_manager_costmap_filters = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap_filters",
        output="screen",
        parameters=[
            {"node_names": ["filter_mask_server", "costmap_filter_info_server"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
        ],
        condition=IfCondition(use_keepout),
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        amcl_config_arg,
        use_keepout_arg,
        nav2_amcl,
        nav2_map_server,
        nav2_lifecycle_manager,
        filter_mask_server,
        costmap_filter_info_server,
        lifecycle_manager_costmap_filters,
    ])
