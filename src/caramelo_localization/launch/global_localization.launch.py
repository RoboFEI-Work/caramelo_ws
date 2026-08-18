import os
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _avisar_se_conflito(nos_conflitantes, este_modo, outro_modo):
    """Aviso RK-02: SLAM e AMCL publicam ambos map->odom; rodar os dois faz o TF
    piscar entre as duas estimativas. IMPORTANTE: apenas AVISA (nao aborta) — o
    `ros2 node list` pode mostrar nos zumbis de um daemon velho, e um raise aqui
    derrubava o navigation.launch.py inteiro antes de qualquer no subir."""
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


def _maps_da_fonte(share_dir):
    """Arvore FONTE dos mapas deduzida do share instalado (install/ -> src/)."""
    for pai in Path(share_dir).parents:
        if pai.name == "install":
            return pai.parent / "src" / "caramelo_mapping" / "maps"
    return None


def _resolver_pasta_do_mapa(map_name):
    """Acha a pasta do mapa olhando a arvore FONTE ANTES do share instalado.

    Mesma ordem de _resolve_map_folder (caramelo_navigation/launch/navigation.launch.py)
    e de resolve_map_folder (caramelo_navigation/scripts/caramelo_docking_common.py):
    sem isso este launch era o unico lugar do sistema que so' enxergava o share.
    Sintoma real: mapa criado pela GUI em src/ sem rebuild -> o map_server nao
    conseguia configurar, o lifecycle_manager derrubava a subida INTEIRA da
    localizacao e o erro so' falava em transicao de estado, nunca em mapa.
    """
    share_dir = get_package_share_directory("caramelo_mapping")
    raizes = []
    fonte = _maps_da_fonte(share_dir)
    if fonte:
        raizes.append(fonte)
    raizes.append(Path(share_dir) / "maps")

    testados = []
    for raiz in raizes:
        candidato = raiz / map_name
        testados.append(candidato)
        if (candidato / "map.yaml").is_file():
            return candidato.resolve()

    texto = "\n  - ".join(str(caminho) for caminho in testados)
    raise RuntimeError(
        f"Nao encontrei o mapa '{map_name}' (falta o map.yaml). Procurei em:\n  - {texto}"
    )


def _nos_que_dependem_do_mapa(context, *_args, **_kwargs):
    """Nos que precisam do caminho REAL do mapa, resolvido em tempo de launch.

    Fica num OpaqueFunction porque map_name e um launch argument: so' da para
    montar o caminho depois que o contexto existe.
    """
    map_name = LaunchConfiguration("map_name").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_keepout = LaunchConfiguration("use_keepout")

    pasta_do_mapa = _resolver_pasta_do_mapa(map_name)
    map_path = str(pasta_do_mapa / "map.yaml")
    keepout_mask_path = str(pasta_do_mapa / "keepout_mask.yaml")

    keepout_filter_config = os.path.join(
        get_package_share_directory("caramelo_localization"),
        "config",
        "keepout_filter.yaml",
    )

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time},
        ],
    )

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

    avisos = [LogInfo(msg=f"[CARAMELO] Mapa da localizacao: {map_path}")]
    # Mesma armadilha do mapa: mascara ausente derruba a subida inteira dos
    # filtros e o erro nao diz que faltou o arquivo. Avisar antes ao menos
    # aponta o culpado (crie com scripts/init_keepout_mask.py).
    if use_keepout.perform(context).strip().lower() in ("1", "true", "yes", "on") \
            and not Path(keepout_mask_path).is_file():
        avisos.append(LogInfo(
            msg=f"[CARAMELO][AVISO] Paredes virtuais pedidas, mas {keepout_mask_path} "
                "nao existe. Gere a mascara antes (init_keepout_mask.py) ou suba "
                "com use_keepout:=false."))

    return avisos + [nav2_map_server, filter_mask_server]


def generate_launch_description():

    # RK-02: avisa (sem abortar) se o mapeamento (SLAM) parecer ativo.
    _avisar_se_conflito(["/slam_toolbox"], "a localizacao (AMCL)", "o mapeamento (SLAM)")


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

    # map_name nao aparece aqui: quem o usa e _nos_que_dependem_do_mapa, que
    # resolve o caminho de verdade com o contexto do launch.
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")
    use_keepout = LaunchConfiguration("use_keepout")

    lifecycle_nodes = ["map_server", "amcl"]

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
    # O filter_mask_server e o map_server moram em _nos_que_dependem_do_mapa:
    # os dois precisam do caminho REAL da pasta do mapa (fonte antes do share).
    keepout_filter_config = os.path.join(
        get_package_share_directory("caramelo_localization"),
        "config",
        "keepout_filter.yaml",
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
        # map_server + filter_mask_server: dependem do caminho do mapa, que so'
        # da para resolver com o contexto ja montado.
        OpaqueFunction(function=_nos_que_dependem_do_mapa),
        nav2_lifecycle_manager,
        costmap_filter_info_server,
        lifecycle_manager_costmap_filters,
    ])
