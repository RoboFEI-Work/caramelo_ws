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
    # Sem valor: mantem o que ja esta gravado. Com default numerico, TODA regravacao
    # de pose reescrevia o bloco table: -- arrastar uma prateleira de 0.15 no mapa
    # a rebaixava calada para 0.10, e width/depth voltavam ao padrao.
    parser.add_argument("--height", type=float, default=None, help="Altura da mesa: 0.00, 0.05, 0.10 ou 0.15 m. Sem isto, mantem a altura ja gravada (0.10 numa area nova).")
    parser.add_argument("--map-dir", default=None, help="Diretorio raiz dos mapas ou pasta do mapa.")
    parser.add_argument("--frame", default="map", help="Frame global da pose salva.")
    parser.add_argument(
        "--pose", nargs=3, default=None, metavar=("X", "Y", "YAW"),
        help="Grava ESTA pose em vez de perguntar ao robo onde ele esta. "
             "X e Y em metros, YAW em RADIANOS, no frame do mapa (\"map\"). "
             "E o caminho usado para posicionar a area clicando no mapa, sem "
             "levar o robo ate la. Sem a flag nada muda: a pose vem do robo.")
    parser.add_argument("--base-frame", default="base_footprint", help="Frame base preferencial do robo.")
    parser.add_argument("--timeout", type=float, default=3.0, help="Tempo maximo esperando TF.")
    parser.add_argument("--overwrite", "--yes", dest="overwrite", action="store_true", help="Sobrescreve uma area existente e cria backup.")
    parser.add_argument("--approach-offset", type=float, default=0.45, help="Distancia da approach_pose atras da pose final.")
    parser.add_argument("--linear-tolerance", type=float, default=0.03, help="Tolerancia linear da area.")
    parser.add_argument("--angular-tolerance", type=float, default=0.04, help="Tolerancia angular da area.")
    parser.add_argument("--width", type=float, default=None, help="Largura da mesa em metros. Sem isto, mantem a largura ja gravada (0.80 numa area nova).")
    parser.add_argument("--depth", type=float, default=None, help="Profundidade da mesa em metros. Sem isto, mantem a profundidade ja gravada (0.50 numa area nova).")
    parser.add_argument("--notes", default=None, help="Observacao humana para service_areas.yaml.")
    parser.add_argument("--no-sync-docking", action="store_true", help="Nao atualiza docking.yaml depois de salvar.")
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def _pose_informada(valores):
    """--pose X Y YAW -> {x, y, yaw}; None quando a flag nao foi passada.

    Valida aqui (e nao com type=float do argparse) para o erro sair em
    portugues e dizer QUAL dos tres valores nao e numero -- quem chama e a
    interface grafica por QProcess, e o texto do stderr vira a mensagem de tela.
    """
    if not valores:
        return None
    numeros = {}
    for chave, rotulo, bruto in zip(("x", "y", "yaw"), ("X", "Y", "YAW"), valores):
        try:
            numeros[chave] = float(bruto)
        except (TypeError, ValueError):
            raise ValueError(
                f"--pose espera 3 numeros (X Y YAW); '{bruto}' nao e um numero valido ({rotulo}).")
    return numeros


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


def _medida(valor):
    """Medida que o operador NAO passou fica como esta no arquivo."""
    return "mantido" if valor is None else f"{valor:.2f}"


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = _parse_args(argv)
    try:
        area_id = normalize_dock_id(args.area_id)
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
        pose_manual = _pose_informada(args.pose)
    except Exception as exc:
        print(f"Erro nos argumentos: {exc}", file=sys.stderr)
        return 1

    node = None
    try:
        if pose_manual is not None:
            # Com --pose NAO subimos no nem esperamos TF de proposito: marcar uma
            # area no mapa nao pode depender do robo estar ligado e localizado.
            source_frame, pose = "--pose (sem TF)", pose_manual
        else:
            rclpy.init(args=argv)
            node = Node("save_service_area_pose")
            buffer = Buffer()
            TransformListener(buffer, node)
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
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"Erro ao salvar a service area: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
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
    print(f"  table: width={_medida(args.width)}, depth={_medida(args.depth)}, height={_medida(args.height)}")
    print(f"  docking: approach_offset={args.approach_offset:.2f}, linear_tolerance={args.linear_tolerance:.3f}, angular_tolerance={args.angular_tolerance:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
