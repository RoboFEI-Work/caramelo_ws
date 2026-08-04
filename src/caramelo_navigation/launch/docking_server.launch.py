import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_DOCK_IDS = ["START", "FINISH", "WS1", "WS2", "WS3", "WS4", "SH1", "PP1", "RT1"]

DEFAULT_CONTROLLER = {
    "k_phi": 2.0,
    "k_delta": 1.5,
    # 2026-08-03: firmware corrigido — piso 650 rpm (0.1215 m/s de corpo)
    # agora executa EXATO. v_min segue a regra 1.08x piso do controller_server
    # (comando sempre executavel com folga); v_max acima do v_min.
    "v_linear_min": 0.131,
    "v_linear_max": 0.16,
    "v_angular_max": 0.4,
    "slowdown_radius": 0.45,
    # False: o dock fica ENCOSTADO na bancada (obstaculo no costmap) — o
    # checker de colisao sempre disparava na aproximacao ("Collision detected")
    # e abortava. A protecao da navegacao normal continua no collision_monitor.
    "use_collision_detection": False,
    "costmap_topic": "local_costmap/costmap_raw",
    "footprint_topic": "local_costmap/published_footprint",
    "transform_tolerance": 0.2,
    "projection_time": 0.8,
    "simulation_time_step": 0.1,
    "dock_collision_threshold": 0.0,
}

DEFAULT_DOCK_PLUGINS = {
    "caramelo_front_dock": {
        "plugin": "opennav_docking::SimpleNonChargingDock",
        "docking_threshold": 0.05,
        "staging_x_offset": -0.70,
        "staging_yaw_offset": 0.0,
        "dock_direction": "forward",
        "rotate_to_dock": False,
        "use_external_detection_pose": False,
        "use_battery_status": False,
        "use_stall_detection": False,
    },
    "caramelo_precision_front_dock": {
        "plugin": "opennav_docking::SimpleNonChargingDock",
        "docking_threshold": 0.03,
        "staging_x_offset": -0.80,
        "staging_yaw_offset": 0.0,
        "dock_direction": "forward",
        "rotate_to_dock": False,
        "use_external_detection_pose": False,
        "use_battery_status": False,
        "use_stall_detection": False,
    },
    "caramelo_shelf_front_dock": {
        "plugin": "opennav_docking::SimpleNonChargingDock",
        "docking_threshold": 0.06,
        "staging_x_offset": -0.90,
        "staging_yaw_offset": 0.0,
        "dock_direction": "forward",
        "rotate_to_dock": False,
        "use_external_detection_pose": False,
        "use_battery_status": False,
        "use_stall_detection": False,
    },
}


def _default_dock_type(dock_id):
    if dock_id.startswith("SH"):
        return "caramelo_shelf_front_dock"
    if dock_id.startswith("PP") or dock_id.startswith("RT"):
        return "caramelo_precision_front_dock"
    return "caramelo_front_dock"


def _default_docking_database():
    return {
        "dock_plugins": DEFAULT_DOCK_PLUGINS,
        "docks": {
            dock_id: {
                "type": _default_dock_type(dock_id),
                "frame": "map",
                "pose": [0.0, 0.0, 0.0],
                "id": dock_id,
            }
            for dock_id in DEFAULT_DOCK_IDS
        },
    }


def _default_service_area(dock_id):
    if dock_id in ("START", "FINISH"):
        return {"kind": dock_id, "dock_id": dock_id, "approach": "front", "notes": f"Area {dock_id.lower()} da arena."}
    kind = ''.join(ch for ch in dock_id if ch.isalpha())
    data = {"kind": kind, "dock_id": dock_id, "approach": "front"}
    if kind == "WS":
        data.update({"full_name": "Workstation", "table_height_m": 0.10, "width_m": 0.80, "depth_m": 0.50, "precision": "normal"})
    elif kind == "SH":
        data.update({"full_name": "Shelf", "table_height_m": 0.10, "width_m": 0.80, "depth_m": 0.50, "precision": "normal"})
    elif kind == "PP":
        data.update({"full_name": "Precise Placement", "table_height_m": 0.10, "width_m": 0.80, "depth_m": 0.50, "precision": "high"})
    elif kind == "RT":
        data.update({"full_name": "Rotating Table", "table_height_m": 0.10, "diameter_m": 0.60, "precision": "high"})
    return data


def _default_service_areas():
    return {"service_areas": {dock_id: _default_service_area(dock_id) for dock_id in DEFAULT_DOCK_IDS}}


def _write_yaml_if_missing(path, data):
    if path.exists():
        return False
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=False, default_flow_style=False)
    return True


def _source_maps_from_share(share_dir):
    share_path = Path(share_dir)
    for parent in share_path.parents:
        if parent.name == "install":
            return parent.parent / "src" / "caramelo_mapping" / "maps"
    return None


def _resolve_map_folder(map_name, map_dir):
    roots = []
    if map_dir:
        roots.append(Path(map_dir).expanduser())
    share_dir = get_package_share_directory("caramelo_mapping")
    source_maps = _source_maps_from_share(share_dir)
    if source_maps:
        roots.append(source_maps)
    roots.append(Path(share_dir) / "maps")
    checked = []
    for root in roots:
        candidates = []
        if root.name == map_name:
            candidates.append(root)
        candidates.append(root / map_name)
        for candidate in candidates:
            checked.append(candidate)
            if candidate.exists() and (candidate / "map.yaml").exists():
                return candidate.resolve()
    checked_text = "\n  - ".join(str(path) for path in checked)
    raise RuntimeError(f"Nao encontrei a pasta do mapa '{map_name}' com map.yaml. Caminhos testados:\n  - {checked_text}")


def _load_docking_yaml(docking_path):
    service_areas_path = docking_path.with_name("service_areas.yaml")
    _write_yaml_if_missing(docking_path, _default_docking_database())
    _write_yaml_if_missing(service_areas_path, _default_service_areas())

    with docking_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if "docks" not in data:
        raise RuntimeError(f"Arquivo {docking_path} precisa conter a chave 'docks'.")
    return data


def _launch_setup(context, *args, **kwargs):
    map_name = LaunchConfiguration("map_name").perform(context)
    map_dir = LaunchConfiguration("map_dir").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    base_frame = LaunchConfiguration("base_frame").perform(context)
    fixed_frame = LaunchConfiguration("fixed_frame").perform(context)
    map_folder = _resolve_map_folder(map_name, map_dir)
    docking_path = map_folder / "docking.yaml"
    docking_data = _load_docking_yaml(docking_path)
    dock_plugins = docking_data.get("dock_plugins") or DEFAULT_DOCK_PLUGINS
    if not isinstance(dock_plugins, dict) or not dock_plugins:
        raise RuntimeError(f"Arquivo {docking_path} precisa conter 'dock_plugins' como dicionario.")

    params = {
        "use_sim_time": use_sim_time,
        "controller_frequency": 20.0,
        "initial_perception_timeout": 2.0,
        "wait_charge_timeout": 1.0,
        "dock_approach_timeout": 25.0,
        "undock_linear_tolerance": 0.15,
        "undock_angular_tolerance": 3.14,
        "max_retries": 1,
        "base_frame": base_frame,
        "fixed_frame": fixed_frame,
        "dock_backwards": False,
        "dock_prestaging_tolerance": 0.25,
        "enable_stamped_cmd_vel": False,
        "dock_database": str(docking_path),
        "dock_plugins": list(dock_plugins.keys()),
        "controller": DEFAULT_CONTROLLER,
    }
    params.update(dock_plugins)

    docking_server = Node(
        package="opennav_docking",
        executable="opennav_docking",
        name="docking_server",
        output="screen",
        parameters=[params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_docking",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["docking_server"]},
        ],
    )

    # Alinhamento fino holonomico (action /align_to_dock): validado no robo em
    # 2026-07-21 (erros ~1-2cm / ~1-2 graus). Pisos casados com o firmware novo
    # dos ESCs. Antes nenhum launch subia o node e o botao da GUI/mission_runner
    # dependia de rodar na mao.
    dock_align = Node(
        package="caramelo_navigation",
        executable="dock_align_node",
        name="dock_align_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            # 2026-07-27: pisos casados com o firmware ESC speed_min 650 rpm
            # (roda 2.43 rad/s -> corpo 0.121 m/s / 0.315 rad/s).
            # Piso fisico exato do ESC pos-fix do mapa (650rpm executa 650):
            # passo minimo do alinhamento fino 40% menor que os 0.19/0.49 antigos.
            {"min_body_vel": 0.1215},
            {"min_body_omega": 0.315},
            # Refino LiDAR: distancia frontal robo->face na pose final.
            # CALIBRADO 2026-07-23 na WS1 do arena3_520 (robo na pose ideal,
            # "refino face: dist=0.403 m, largura=0.60 m, residuo=3 mm").
            # Recalibrar se a pose ideal em relacao a mesa mudar.
            # 0.403 -> 0.393 (2026-08-03): medido na pose ideal de manipulacao
            # da WS1 (regua TF na face real; pose tambem re-gravada no banco).
            {"refine_desired_face_dist": 0.393},
        ],
    )

    # Gravador de pose de dock (topico /save_dock_pose): sem ele o botao
    # "Salvar pose como dock" da GUI publicava no vazio (sucesso falso).
    dock_recorder = Node(
        package="caramelo_navigation",
        executable="dock_pose_recorder_node",
        name="dock_pose_recorder_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_name": map_name},
        ],
    )

    return [docking_server, lifecycle_manager, dock_align, dock_recorder]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_name", description="Nome da pasta do mapa."),
        DeclareLaunchArgument("map_dir", default_value="", description="Diretorio raiz dos mapas ou pasta do mapa."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("fixed_frame", default_value="odom"),
        OpaqueFunction(function=_launch_setup),
    ])
