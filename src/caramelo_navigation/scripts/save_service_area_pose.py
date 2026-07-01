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
    parser.add_argument("--map-name", required=True, help="Nome da pasta do mapa.")
    parser.add_argument("--name", "--area-id", dest="area_id", required=True, help="START, FINISH, WS1, SH1, PP1, RT1...")
    parser.add_argument("--type", dest="area_type", default=None, help="START, FINISH, WS, SH, PP ou RT.")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    parser.add_argument("--frame", default="map", help="Frame global da pose salva.")
    parser.add_argument("--base-frame", default="base_link", help="Frame base preferencial do robo.")
    parser.add_argument("--timeout", type=float, default=3.0, help="Tempo maximo esperando TF.")
    parser.add_argument("--yes", action="store_true", help="Confirma sobrescrita de uma area existente.")
    parser.add_argument("--approach-offset", type=float, default=None, help="Distancia da pose de aproximacao atras da pose final.")
    parser.add_argument("--docking-offset", type=float, default=None, help="Offset tecnico usado pelo docking.")
    parser.add_argument("--linear-tolerance", type=float, default=None, help="Tolerancia linear da area.")
    parser.add_argument("--angular-tolerance", type=float, default=None, help="Tolerancia angular da area.")
    parser.add_argument("--station-width", type=float, default=None, help="Largura da estacao em metros.")
    parser.add_argument("--station-depth", type=float, default=None, help="Profundidade da estacao em metros.")
    parser.add_argument("--station-height", type=float, default=None, help="Altura da estacao em metros.")
    parser.add_argument("--free-width", type=float, default=None, help="Largura da area livre frontal em metros.")
    parser.add_argument("--free-depth", type=float, default=None, help="Profundidade da area livre frontal em metros.")
    parser.add_argument("--dock-type", default=None, help="Tipo tecnico de dock. Ex: caramelo_front_dock.")
    parser.add_argument("--notes", default=None, help="Observacao humana para service_areas.yaml.")
    parser.add_argument("--no-sync-docking", action="store_true", help="Nao atualiza docking.yaml depois de salvar.")
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def _lookup_pose(node: Node, buffer: Buffer, frame: str, base_frame: str, timeout: float):
    tried = []
    for source_frame in (base_frame, "base_footprint"):
        if source_frame in tried:
            continue
        tried.append(source_frame)
        try:
            transform = buffer.lookup_transform(frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=timeout))
            trans = transform.transform.translation
            rot = transform.transform.rotation
            yaw = yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)
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
            docking_offset=args.docking_offset,
            linear_tolerance=args.linear_tolerance,
            angular_tolerance=args.angular_tolerance,
            station_width=args.station_width,
            station_depth=args.station_depth,
            station_height=args.station_height,
            free_width=args.free_width,
            free_depth=args.free_depth,
            dock_type=args.dock_type,
            notes=args.notes,
            overwrite=args.yes,
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
    print(f"  pose: [{pose['x']:.4f}, {pose['y']:.4f}, {pose['yaw']:.6f}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
