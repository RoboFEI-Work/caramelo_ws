# Navegacao Nav2 do Caramelo para um mapa de caramelo_mapping/maps/<map_name>.
# Inclui global_localization (AMCL + map_server [+ keepout]) e sobe
# controller/planner/smoother/bt_navigator/behavior/velocity_smoother/
# collision_monitor + lifecycle, o relay cmd_vel -> TwistStamped (caramelo_utils
# twist_relay), service_area_manager/markers e, com use_docking, o
# docking_server.launch.py. Quem usa: bringup.launch.py (GUI "Ligar navegacao")
# e caramelo_bringup/robot_manipulation.launch.py.
# Como roda: ros2 launch caramelo_navigation navigation.launch.py map_name:=<mapa> [nav_profile:=dwb|mppi] [lattice_set:=omni|nariz]
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _source_maps_from_share(share_dir):
    share_path = Path(share_dir)
    for parent in share_path.parents:
        if parent.name == "install":
            return parent.parent / "src" / "caramelo_mapping" / "maps"
    return None


def _resolve_map_folder(context):
    map_name = LaunchConfiguration("map_name").perform(context)
    share_dir = get_package_share_directory("caramelo_mapping")
    roots = []
    source_maps = _source_maps_from_share(share_dir)
    if source_maps:
        roots.append(source_maps)
    roots.append(Path(share_dir) / "maps")

    checked = []
    for root in roots:
        candidate = root / map_name
        checked.append(candidate)
        if candidate.exists() and (candidate / "map.yaml").exists():
            return candidate.resolve()

    checked_text = "\n  - ".join(str(path) for path in checked)
    raise RuntimeError(f"Nao encontrei a pasta do mapa '{map_name}' com map.yaml. Caminhos testados:\n  - {checked_text}")


def _resolve_costmap_resolution(context):
    requested_resolution = LaunchConfiguration("costmap_resolution").perform(context).strip()
    if requested_resolution.lower() != "map":
        return float(requested_resolution)

    map_yaml_path = _resolve_map_folder(context) / "map.yaml"

    try:
        with open(map_yaml_path, "r", encoding="utf-8") as map_yaml_file:
            map_data = yaml.safe_load(map_yaml_file) or {}
        return float(map_data.get("resolution", 0.20))
    except (OSError, TypeError, ValueError):
        return 0.20


# Conjuntos de primitivas do Smac lattice (2026-11-11), por valor de lattice_set.
# Cada entrada: arquivo em config/lattice/ e overrides do SmacLattice que o
# launch injeta por cima do planner_server.yaml (os valores que a bancada
# offline mediu JUNTO com o conjunto; sem eles o conjunto nao entrega o
# resultado). "omni" e' o default de producao e nao sobrescreve nada.
LATTICE_SETS = {
    "omni": {
        "arquivo": "caramelo_omni_2cm_16bins.json",
        "smac_overrides": {},
    },
    # nariz: frente + diagonais frente (custo x1.5) + laterais (custo x2.0),
    # sem re'. analytic_expansion_max_length 3.0 -> 0.8: a expansao analitica
    # e' uma reta ate o goal com yaw interpolado (vira lateral/re' no frame do
    # robo); limitada a 0.8 m deixa de responder por ~40% do plano.
    # rotation_penalty 1.0 -> 3.0: com a lateral mais cara, o giro no lugar
    # precisa custar mais para o A* nao trocar lateral por giro+frente+giro a
    # cada passo (bancada: 90% do plano de frente, +0.6% de comprimento).
    "nariz": {
        "arquivo": "caramelo_nariz_2cm_16bins.json",
        "smac_overrides": {
            # 0.8 -> 0.4 (2026-11-13, bancada 156 pares): frente 89,9 -> 95,4 %,
            # analitica 12,5 -> 4,8 %, t_max 0,34 -> 0,22 s; 0,2 dobraria o tempo.
            "analytic_expansion_max_length": 0.4,
            "rotation_penalty": 3.0,
        },
    },
}


def _resolve_lattice_set(context):
    lattice_set = LaunchConfiguration("lattice_set").perform(context).strip().lower()
    if lattice_set not in LATTICE_SETS:
        raise RuntimeError(
            f"lattice_set precisa ser um de {sorted(LATTICE_SETS)}; veio '{lattice_set}'.")
    return lattice_set, LATTICE_SETS[lattice_set]


def _make_navigation_override_params(map_name, resolution, lattice_filepath,
                                     route_graph_filepath, keepout_enabled=False,
                                     smac_overrides=None):
    # Parametros por costmap (local e global). So injetamos o Keepout Filter
    # (RK-04) quando use_keepout:=true, para nao gerar avisos de "esperando
    # CostmapFilterInfo" quando os servidores de mascara nao estao no ar.
    local_costmap_params = {"resolution": resolution}
    global_costmap_params = {"resolution": resolution}
    if keepout_enabled:
        keepout_plugin = {
            "plugin": "nav2_costmap_2d::KeepoutFilter",
            "enabled": True,
            "filter_info_topic": "/costmap_filter_info",
        }
        for costmap_params in (local_costmap_params, global_costmap_params):
            costmap_params["filters"] = ["keepout_filter"]
            costmap_params["keepout_filter"] = keepout_plugin

    # SmacLattice: caminho do lattice + overrides do conjunto escolhido
    # (lattice_set). Entram na copia do YAML, por cima do planner_server.yaml.
    smac_params = {"lattice_filepath": lattice_filepath}
    smac_params.update(smac_overrides or {})

    params = {
        "planner_server": {
            "ros__parameters": {
                "SmacLattice": smac_params,
            },
        },
        "local_costmap": {
            "local_costmap": {
                "ros__parameters": local_costmap_params,
            },
        },
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": global_costmap_params,
            },
        },
        "route_server": {
            "ros__parameters": {
                "graph_filepath": route_graph_filepath,
            },
        },
    }

    # Caminho FIXO por mapa (sobrescrito a cada launch): o tempfile de antes
    # deixava um arquivo novo em /tmp a cada 'ros2 launch'.
    params_path = Path("/tmp") / f"caramelo_navigation_overrides_{map_name}.yaml"
    with params_path.open("w", encoding="utf-8") as params_file:
        yaml.safe_dump(params, params_file, sort_keys=False)
    return str(params_path)


def _route_graph_counts(route_graph_filepath):
    try:
        with open(route_graph_filepath, "r", encoding="utf-8") as graph_file:
            graph = yaml.safe_load(graph_file) or {}
    except OSError:
        return 0, 0

    nodes = 0
    edges = 0
    for feature in graph.get("features", []):
        geometry = feature.get("geometry", {})
        if geometry.get("type") == "Point":
            nodes += 1
        else:
            edges += 1
    return nodes, edges


def _bt_xml_for_profile(context, package_share):
    profile = LaunchConfiguration("nav_profile").perform(context).strip().lower()
    if profile not in ("dwb", "mppi"):
        raise RuntimeError("nav_profile precisa ser 'dwb' ou 'mppi'.")
    # to_pose E through_poses: os DOIS precisam ser custom. O default de
    # through_poses do Nav2 usa Spin, que nao existe mais no behavior_server
    # (RK-05) — sem o override, o bt_navigator falhava ao ativar e o lifecycle
    # abortava o bringup inteiro ("navegacao inativa").
    # 2026-11-11: bt_replan escolhe COMO a BT de to_pose replaneja. "rate" =
    # arquivo de sempre (ComputePathToPose a 1 Hz); "invalid" = so' replaneja
    # se o goal mudou, o path ficou invalido ou passou 15 s (opcao da rodada 2
    # pos-bags; so' existe para dwb). through_poses nao muda.
    bt_replan = LaunchConfiguration("bt_replan").perform(context).strip().lower()
    if bt_replan not in ("rate", "invalid"):
        raise RuntimeError("bt_replan precisa ser 'rate' ou 'invalid'.")
    if bt_replan == "invalid":
        # 2026-11-13: com DWB forward_prune_distance/prune_distance 1.0 esta BT
        # (plano de ate' 15 s) pode dar "Resulting plan has 0 poses" (robo a mais
        # de 1 m do inicio do plano). Usar so' depois de rever o corte do plano.
        print("[navigation.launch] AVISO: bt_replan:=invalid nao e' recomendado com "
              "forward_prune_distance 1.0 (ver controller_server.yaml).")
    if bt_replan == "invalid" and profile != "dwb":
        raise RuntimeError("bt_replan:=invalid so' existe para nav_profile:=dwb.")
    sufixo = "_replan_if_invalid" if bt_replan == "invalid" else ""
    to_pose = os.path.join(
        package_share, "behavior_tree", f"caramelo_lattice_{profile}{sufixo}.xml")
    through = os.path.join(
        package_share, "behavior_tree", f"caramelo_lattice_through_{profile}.xml")
    return profile, to_pose, through


def _create_navigation_stack(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")
    use_docking = LaunchConfiguration("use_docking")
    map_name = LaunchConfiguration("map_name")
    use_route_server = LaunchConfiguration("use_route_server")
    use_waypoint_follower = LaunchConfiguration("use_waypoint_follower")
    use_cmd_vel_relay = LaunchConfiguration("use_cmd_vel_relay")
    use_speed_limit = LaunchConfiguration("use_speed_limit")
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
    nav_profile_value, bt_xml, bt_through_xml = _bt_xml_for_profile(
        context, caramelo_navigation_pkg)
    lattice_set_value, lattice_set = _resolve_lattice_set(context)
    lattice_filepath = os.path.join(
        caramelo_navigation_pkg,
        "config",
        "lattice",
        lattice_set["arquivo"],
    )
    map_folder = _resolve_map_folder(context)
    route_graph_filepath = str(map_folder / "route_graph.geojson")
    route_graph_nodes, route_graph_edges = _route_graph_counts(route_graph_filepath)
    start_route_server = (
        _as_bool(use_route_server.perform(context))
        and route_graph_nodes > 0
        and route_graph_edges > 0
    )
    start_waypoint_follower = _as_bool(use_waypoint_follower.perform(context))

    if start_route_server:
        lifecycle_nodes.append("route_server")
    if start_waypoint_follower:
        lifecycle_nodes.append("waypoint_follower")

    costmap_resolution = _resolve_costmap_resolution(context)
    keepout_enabled = _as_bool(LaunchConfiguration("use_keepout").perform(context))
    navigation_override_params = _make_navigation_override_params(
        map_name.perform(context),
        costmap_resolution,
        lattice_filepath,
        route_graph_filepath,
        keepout_enabled=keepout_enabled,
        smac_overrides=lattice_set["smac_overrides"],
    )

    # 2026-11-13: use_path_orientations true so' e' seguro com o RotationShim
    # corrigido do workspace (src/third_party). O do apt (1.3.13) compara yaw em
    # odom com yaw em map e gira o robo para o lado errado. Se o pacote resolvido
    # for o de /opt/ros (overlay nao compilado), forca false e avisa.
    # 2a condicao (2026-11-13, medido nos bags): o yaw do plano so' serve de
    # referencia se o lattice planeja de FRENTE. "nariz": 3% do plano de lado
    # (maior trecho 0,36 m), 0% de re'. "omni": 51% de lado (ate' 4,9 m
    # seguidos) e 10% de re' -> com true o robo seguia de re'/de lado as cegas
    # (LiDAR so' na frente) e batia na simulacao. Fora do "nariz", forca false.
    shim_prefix = get_package_prefix("nav2_rotation_shim_controller")
    shim_corrigido = not shim_prefix.startswith("/opt/ros")
    yaw_do_plano_ok = shim_corrigido and lattice_set_value == "nariz"
    shim_guard_params = [] if yaw_do_plano_ok else [
        {"DWBHolonomic.use_path_orientations": False}]

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "controller_server.yaml"),
            navigation_override_params,
            {"use_sim_time": use_sim_time},
            *shim_guard_params,
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
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "planner_server.yaml"),
            navigation_override_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
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
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "bt_navigator.yaml"),
            {"use_sim_time": use_sim_time},
            {"default_nav_to_pose_bt_xml": bt_xml},
            {"default_nav_through_poses_bt_xml": bt_through_xml},
        ],
    )

    route_and_waypoint_nodes = []
    if start_route_server:
        route_and_waypoint_nodes.append(
            Node(
                package="nav2_route",
                executable="route_server",
                name="route_server",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    os.path.join(caramelo_navigation_pkg, "config", "route_server.yaml"),
                    navigation_override_params,
                    {"use_sim_time": use_sim_time},
                ],
            )
        )

    if start_waypoint_follower:
        route_and_waypoint_nodes.append(
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    os.path.join(caramelo_navigation_pkg, "config", "waypoint_follower.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            )
        )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
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
        respawn=True,
        respawn_delay=2.0,
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
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            os.path.join(caramelo_navigation_pkg, "config", "collision_monitor.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # respawn=True nos servers (2026-11-07): se um server do Nav2 cair (crash,
    # kill por engano), o launch o sobe de novo em 2 s e o lifecycle manager,
    # ao ver o bond voltar dentro de bond_respawn_max_duration, refaz o
    # STARTUP de toda a pilha sozinho. Sem isso a queda de UM server deixava a
    # navegacao inteira inativa ate relancar tudo (aconteceu em 2026-11-07).
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"attempt_respawn_reconnection": True},
            {"bond_respawn_max_duration": 30.0},
        ],
    )

    cmd_vel_relay = Node(
        package="caramelo_utils",
        executable="twist_relay",
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

    # 2026-11-11: limite de velocidade por proximidade (pedido do operador:
    # "perto de objeto e em curva fechada, mais devagar"). O no' olha o menor
    # alcance do /scan e publica /speed_limit (percentual); o controller_server
    # aplica no DWB. Nunca abaixo de 70%: max_vel_theta 0,50 x 0,70 = 0,35 =
    # min_speed_theta, abaixo disso o DWB nao teria giro valido.
    speed_limit_node = Node(
        package="caramelo_navigation",
        executable="speed_limit_by_scan",
        name="speed_limit_by_scan",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_speed_limit),
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
            *route_and_waypoint_nodes,
            nav2_velocity_smoother,
            nav2_collision_monitor,
            nav2_lifecycle_manager,
            service_area_manager,
            service_area_markers,
            docking_server,
        ],
    )

    return [
        speed_limit_node,
        LogInfo(msg=f"Resolucao dos costmaps: {costmap_resolution:.3f} m/celula"),
        LogInfo(msg=f"Perfil Nav2: {nav_profile_value} (BT to_pose: {os.path.basename(bt_xml)})"),
        LogInfo(msg=(f"RotationShim: {shim_prefix}, lattice {lattice_set_value}: use_path_orientations do YAML"
                     if yaw_do_plano_ok else
                     f"AVISO RotationShim: use_path_orientations FORCADO false "
                     f"(shim corrigido: {shim_corrigido}; lattice_set: {lattice_set_value}; so' vale com o shim "
                     "do workspace E lattice_set:=nariz)")),
        LogInfo(msg=f"Primitivas do Smac lattice ({lattice_set_value}): {lattice_filepath}"),
        LogInfo(msg=f"Overrides do SmacLattice pelo lattice_set: {lattice_set['smac_overrides'] or 'nenhum'}"),
        LogInfo(msg=f"Grafo de rotas: {route_graph_filepath} ({route_graph_nodes} nos, {route_graph_edges} arestas)"),
        LogInfo(msg=f"Route server ligado: {start_route_server}"),
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
            "use_keepout": LaunchConfiguration("use_keepout"),
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
            description="Nome da pasta do mapa em caramelo_mapping/maps/",
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
        # 2026-11-13 (operador): speed_limit_by_scan DESLIGADO por padrao. O % do
        # /speed_limit tambem escala max_vel_theta (0,50 x 0,70 = 0,35 = min_speed_theta:
        # o DWB so' tinha o giro +-0,35) e na arena ficava ligado 99 % do tempo.
        # Religar: use_speed_limit:=true.
        DeclareLaunchArgument(
            "use_speed_limit",
            default_value="false",
            description="2026-11-11: sobe speed_limit_by_scan (reduz a velocidade maxima do DWB "
                        "para 70% quando o LiDAR ve algo a menos de 0,70 m; 100% acima de 0,85 m). "
                        "Desligar = use_speed_limit:=false.",
        ),
        DeclareLaunchArgument(
            "bt_replan",
            default_value="rate",
            description="Replanejamento da BT de NavigateToPose: rate (default, a 1 Hz) ou "
                        "invalid (2026-11-11, so' dwb: replaneja se o goal mudou, o path "
                        "ficou invalido ou passou 15 s; evita a troca de rota a cada "
                        "limpeza do costmap e os 'Start occupied' na staging). "
                        "Voltar = bt_replan:=rate.",
        ),
        # 2026-11-13: default omni -> nariz. O robo so' enxerga a FRENTE; no omni
        # 75 % do plano tinha rumo de lado/para tras e, com o shim seguindo o rumo
        # planejado, ele viraria a traseira cega para o movimento em 2/3 das pernas.
        # nariz: 93-95 % de frente, 0 falhas em 156 pares (bancada). Voltar: lattice_set:=omni.
        DeclareLaunchArgument(
            "lattice_set",
            default_value="nariz",
            description="Primitivas do Smac lattice: nariz (default desde 2026-11-13: sem re, diagonal "
                        "custo x1.5, lateral x2.0; o launch impoe analytic_expansion_max_length 0.4 e "
                        "rotation_penalty 3.0; bancada: 95% do plano de frente) ou omni (antigo default: "
                        "re, lateral e diagonais com o mesmo custo). Voltar = lattice_set:=omni.",
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
            "use_docking",
            default_value="true",
            description="Sobe o Docking Server (opennav_docking + dock_align_node) com o docking.yaml do mapa",
        ),
        DeclareLaunchArgument(
            "use_keepout",
            default_value="false",
            description="Ativa o Keepout Filter (RK-04): sobe os servidores de mascara "
                        "e injeta o plugin KeepoutFilter nos costmaps. Requer "
                        "maps/<mapa>/keepout_mask.{yaml,pgm} (ver ros2 run caramelo_navigation init_keepout_mask).",
        ),
        DeclareLaunchArgument(
            "navigation_start_delay",
            default_value="5.0",
            description="Segundos de espera entre subir a localizacao e subir o Nav2",
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
        localization,
        OpaqueFunction(function=_create_navigation_stack),
    ])
