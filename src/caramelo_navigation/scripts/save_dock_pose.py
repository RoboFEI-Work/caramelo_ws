#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener

from caramelo_docking_common import normalize_dock_id, resolve_map_folder, service_fields_from_args, update_dock_pose, yaw_from_quaternion


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="Salva a pose atual do robo como dock.")
    parser.add_argument("--map-name", required=True, help="Nome da pasta do mapa.")
    parser.add_argument("--dock-id", required=True, help="START, FINISH, WS1, SH1, PP1, RT1...")
    parser.add_argument("--dock-type", default=None, help="Tipo de dock. Ex: caramelo_front_dock.")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    parser.add_argument("--frame", default="map", help="Frame global da pose salva.")
    parser.add_argument("--base-frame", default="base_link", help="Frame base preferencial do robo.")
    parser.add_argument("--timeout", type=float, default=3.0, help="Tempo maximo esperando TF.")
    parser.add_argument("--update-service-area", action="store_true", help="Atualiza tambem service_areas.yaml.")
    parser.add_argument("--kind", default=None, help="WS, SH, PP, RT, START ou FINISH.")
    parser.add_argument("--height", type=float, default=None, help="Altura da area em metros.")
    parser.add_argument("--width", type=float, default=None, help="Largura da area em metros.")
    parser.add_argument("--depth", type=float, default=None, help="Profundidade da area em metros.")
    parser.add_argument("--diameter", type=float, default=None, help="Diametro da RT em metros.")
    parser.add_argument("--precision", default=None, help="normal ou high.")
    parser.add_argument("--notes", default=None, help="Observacoes semanticas.")
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
            return source_frame, [trans.x, trans.y, yaw]
        except TransformException as exc:
            node.get_logger().warn(f"Falha buscando TF {frame} -> {source_frame}: {exc}")
    raise RuntimeError(f"Nao consegui obter TF de {frame} para {tried}.")


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = _parse_args(argv)
    try:
        dock_id = normalize_dock_id(args.dock_id)
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
    except Exception as exc:
        print(f"Erro nos argumentos: {exc}", file=sys.stderr)
        return 1
    rclpy.init(args=argv)
    node = Node("save_dock_pose")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        end_time = node.get_clock().now() + Duration(seconds=args.timeout)
        while rclpy.ok() and node.get_clock().now() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        source_frame, pose = _lookup_pose(node, buffer, args.frame, args.base_frame, args.timeout)
        dock_path, dock_backup, service_path, service_backup = update_dock_pose(
            map_folder=map_folder,
            dock_id=dock_id,
            dock_type=args.dock_type,
            frame=args.frame,
            pose=pose,
            update_service_area=args.update_service_area,
            service_fields=service_fields_from_args(args),
        )
    except Exception as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.destroy_node()
    rclpy.shutdown()
    print("Dock salvo com sucesso:")
    print(f"  map_name: {args.map_name}")
    print(f"  file: {dock_path}")
    if dock_backup:
        print(f"  backup: {dock_backup}")
    if service_path:
        print(f"  service_areas: {service_path}")
        if service_backup:
            print(f"  service_backup: {service_backup}")
    print(f"  dock_id: {dock_id}")
    print(f"  type: {args.dock_type or 'auto'}")
    print(f"  frame: {args.frame}")
    print(f"  base_frame_usado: {source_frame}")
    print(f"  pose: [{pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.6f}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
