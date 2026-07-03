#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener

from caramelo_docking_common import normalize_dock_id, resolve_map_folder, yaw_from_quaternion
from caramelo_service_area_common import sync_docking_from_service_areas, update_service_area_pose


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="Salva a pose atual do robo em service_areas.yaml.")
    parser.add_argument("--map", "--map-name", dest="map_name", required=True, help="Nome da pasta do mapa.")
    parser.add_argument("--name", "--area-id", dest="area_id", required=True, help="START, FINISH, WS1, SH1, PP1, RT1...")
    parser.add_argument("--type", dest="area_type", required=True, help="start, finish, workstation, shelf, rotating_table ou precision_placement.")
    parser.add_argument("--height", type=float, default=0.10, help="Altura da mesa: 0.00, 0.05, 0.10 ou 0.15 m.")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    parser.add_argument("--frame", default="map", help="Frame global da pose salva.")
    parser.add_argument("--base-frame", default="base_footprint", help="Frame base preferencial do robo.")
    parser.add_argument("--timeout", type=float, default=3.0, help="Tempo maximo esperando TF.")
    parser.add_argument("--overwrite", "--yes", dest="overwrite", action="store_true", help="Sobrescreve uma area existente e cria backup.")
    parser.add_argument("--approach-offset", type=float, default=0.45, help="Distancia da approach_pose atras da pose final.")
    parser.add_argument("--linear-tolerance", type=float, default=0.03, help="Tolerancia linear da area.")
    parser.add_argument("--angular-tolerance", type=float, default=0.04, help="Tolerancia angular da area.")
    parser.add_argument("--width", type=float, default=0.80, help="Largura padrao da mesa em metros.")
    parser.add_argument("--depth", type=float, default=0.50, help="Profundidade padrao da mesa em metros.")
    parser.add_argument("--notes", default=None, help="Observacao humana para service_areas.yaml.")
    parser.add_argument("--no-sync-docking", action="store_true", help="Nao atualiza docking.yaml depois de salvar.")
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def _lookup_pose(node: Node, buffer: Buffer, frame: str, base_frame: str, timeout: float):
    tried = []
    for source_frame in (base_frame, "base_footprint", "base_link"):
        if source_frame in tried:
            continue
        tried.append(source_frame)
        try:
            transform = buffer.lookup_transform(frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=timeout))
            trans = transform.transform.translation
            rot = transform.transform.rotation
            yaw = yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)
            if source_frame == "base_link" and "base_footprint" in tried:
                node.get_logger().warn("base_footprint nao apareceu; salvando pose usando base_link.")
            return source_frame, {"x": trans.x, "y": trans.y, "yaw": yaw}
        except TransformException as exc:
            node.get_logger().warn(f"Falha buscando TF {frame} -> {source_frame}: {exc}")
    raise RuntimeError(f"Nao consegui obter TF de {frame} para {tried}.")


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = _parse_args(argv)
    try:
        area_id = normalize_dock_id(args.area_id)
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
    except Exception as exc:
        print(f"Erro nos argumentos: {exc}", file=sys.stderr)
        return 1

    rclpy.init(args=argv)
    node = Node("save_service_area_pose")
    buffer = Buffer()
    TransformListener(buffer, node)
    try:
        end_time = node.get_clock().now() + Duration(seconds=args.timeout)
        while rclpy.ok() and node.get_clock().now() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        source_frame, pose = _lookup_pose(node, buffer, args.frame, args.base_frame, args.timeout)
        service_path, service_backup = update_service_area_pose(
            map_folder=map_folder,
            map_name=args.map_name,
            area_id=area_id,
            pose=pose,
            area_type=args.area_type,
            frame_id=args.frame,
            approach_offset=args.approach_offset,
            linear_tolerance=args.linear_tolerance,
            angular_tolerance=args.angular_tolerance,
            station_width=args.width,
            station_depth=args.depth,
            station_height=args.height,
            notes=args.notes,
            overwrite=args.overwrite,
        )
        docking_result = None
        if not args.no_sync_docking:
            docking_result = sync_docking_from_service_areas(map_folder, make_backup=True)
    except Exception as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.destroy_node()
    rclpy.shutdown()
    print("Service Area salva com sucesso:")
    print(f"  map_name: {args.map_name}")
    print(f"  service_areas: {service_path}")
    if service_backup:
        print(f"  service_backup: {service_backup}")
    if docking_result:
        docking_path, docking_backup, count = docking_result
        print(f"  docking: {docking_path} ({count} docks sincronizados)")
        if docking_backup:
            print(f"  docking_backup: {docking_backup}")
    print(f"  area: {area_id}")
    print(f"  frame: {args.frame}")
    print(f"  base_frame_usado: {source_frame}")
    print(f"  final_robot_pose: [{pose['x']:.4f}, {pose['y']:.4f}, {pose['yaw']:.6f}]")
    print(f"  table: width={args.width:.2f}, depth={args.depth:.2f}, height={args.height:.2f}")
    print(f"  docking: approach_offset={args.approach_offset:.2f}, linear_tolerance={args.linear_tolerance:.3f}, angular_tolerance={args.angular_tolerance:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
