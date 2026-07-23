#!/usr/bin/env python3
"""Orquestrador do fluxo de estacoes (Fase 6 — autonomia SUPERIOR).

Fluxo do plano: START -> (para cada estacao: navegar -> dock -> [manipular] ->
undock) -> FINISH.

IMPORTANTE (armadilha documentada no plano): DockRobot NAO pode ser chamado de
dentro de uma BT do Nav2 (chamaria o Nav2 recursivamente). Por isso este
orquestrador e' um SCRIPT de autonomia superior, nao uma BT do bt_navigator.
Ele compoe actions ja existentes:

  DockRobot (/dock_robot, opennav_docking)  -> navega ate a staging pose do dock
                                                e faz o approach unicycle
  AlignToDock (/align_to_dock, nosso no)    -> alinhamento fino HOLONOMICO
  UndockRobot (/undock_robot, opennav_docking)

A navegacao ate cada estacao e' feita pelo proprio DockRobot
(navigate_to_staging_pose=true), entao nao reinventamos poses de aproximacao.

Depende de: dock poses reais gravadas em docking.yaml (hoje placeholders), do
Docking Server no ar e do dock_align_node. NAO validado em hardware.

NOTA (integracao manip -> caramelo): a missao COMPLETA (navegacao +
manipulacao) agora e' executada por run_mission + bt_yaml_executor (manip_bt).
Este script permanece como utilitario de teste de navegacao/docking puro e
como referencia A/B do no GoToWS.
"""
import argparse
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from caramelo_docking_common import (  # noqa: E402
    docking_file, load_yaml, normalize_dock_id, resolve_map_folder, default_dock_type,
)

from caramelo_msgs.action import AlignToDock  # noqa: E402

try:
    from nav2_msgs.action import DockRobot, UndockRobot
except ImportError:  # pragma: no cover
    from opennav_docking_msgs.action import DockRobot, UndockRobot  # type: ignore

from nav2_msgs.action import NavigateToPose


class MissionRunner(Node):
    def __init__(self):
        super().__init__("mission_runner")
        self._dock = ActionClient(self, DockRobot, "/dock_robot")
        self._undock = ActionClient(self, UndockRobot, "/undock_robot")
        self._align = ActionClient(self, AlignToDock, "align_to_dock")
        self._nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def navigate_to(self, dock_id, map_name, map_dir):
        """Navegacao PURA ate a pose do dock (sem approach/threshold de dock).

        Para START/FINISH: nao sao bancadas — basta chegar na zona com as
        tolerancias do goal checker (xy 0.10 / yaw 0.20). O dock() ficava
        ajustando milimetros que nao importam e estourava o staging_time.
        """
        import math
        try:
            data = load_yaml(docking_file(resolve_map_folder(map_name, map_dir)))
            pose = data["docks"][dock_id]["pose"]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[{dock_id}] pose nao encontrada: {exc}")
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose[0])
        goal.pose.pose.position.y = float(pose[1])
        goal.pose.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(pose[2]) / 2.0)
        self.get_logger().info(f"[{dock_id}] navegacao pura ate ({pose[0]:.2f}, {pose[1]:.2f})...")
        res = self._send_with_status(self._nav, goal, "/navigate_to_pose")
        return res is not None and res

    def _send_with_status(self, client, goal, name):
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Action server '{name}' nao respondeu.")
            return None
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"Goal '{name}' rejeitado.")
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapper = result_future.result()
        return wrapper is not None and wrapper.status == 4  # STATUS_SUCCEEDED

    # -- helpers de action (bloqueantes, no estilo dos scripts existentes) -----
    def _send(self, client, goal, name):
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Action server '{name}' nao respondeu.")
            return None
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"Goal '{name}' rejeitado.")
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    def dock(self, dock_id, max_staging_time):
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        goal.navigate_to_staging_pose = True
        goal.max_staging_time = float(max_staging_time)
        self.get_logger().info(f"[{dock_id}] dock (navega ate staging + approach)...")
        res = self._send(self._dock, goal, "/dock_robot")
        return bool(res and res.success)

    def align(self, dock_id, map_name, use_lidar_refine, timeout):
        goal = AlignToDock.Goal()
        goal.dock_id = dock_id
        goal.map_name = map_name
        goal.use_lidar_refine = use_lidar_refine
        goal.timeout = float(timeout)
        self.get_logger().info(f"[{dock_id}] alinhamento fino holonomico...")
        res = self._send(self._align, goal, "align_to_dock")
        if res and res.success:
            self.get_logger().info(
                f"[{dock_id}] alinhado (erro x={res.final_x_error:.3f} "
                f"y={res.final_y_error:.3f} yaw={res.final_yaw_error:.3f})")
            return True
        return False

    def undock(self, dock_type, max_undocking_time):
        goal = UndockRobot.Goal()
        goal.dock_type = dock_type or ""
        goal.max_undocking_time = float(max_undocking_time)
        self.get_logger().info(f"undock (dock_type='{goal.dock_type}')...")
        res = self._send(self._undock, goal, "/undock_robot")
        return bool(res and res.success)


def _dock_types(map_name, map_dir, dock_ids):
    """Mapeia cada dock_id ao seu dock_type (do docking.yaml, com fallback)."""
    types = {}
    try:
        data = load_yaml(docking_file(resolve_map_folder(map_name, map_dir)))
        docks = data.get("docks", {})
    except Exception:
        docks = {}
    for dock_id in dock_ids:
        entry = docks.get(dock_id, {})
        types[dock_id] = entry.get("type") or default_dock_type(dock_id)
    return types


def _parse_args(argv):
    p = argparse.ArgumentParser(description="Executa o fluxo de estacoes do Caramelo.")
    p.add_argument("--map-name", required=True, help="Pasta do mapa (docking.yaml).")
    p.add_argument("--map-dir", default=None)
    p.add_argument("--stations", nargs="+", required=True,
                   help="Ordem das estacoes, ex.: WS1 WS2 SH1 PP1")
    p.add_argument("--start", default=None, help="Dock inicial (ex.: START). Opcional.")
    p.add_argument("--finish", default=None, help="Dock final (ex.: FINISH). Opcional.")
    p.add_argument("--no-align", action="store_true", help="Nao rodar alinhamento holonomico.")
    p.add_argument("--use-lidar-refine", action="store_true", help="Refino da face no /scan.")
    p.add_argument("--no-undock", action="store_true", help="Nao dar undock apos cada estacao.")
    p.add_argument("--continue-on-error", action="store_true",
                   help="Segue para a proxima estacao mesmo se uma falhar.")
    p.add_argument("--max-staging-time", type=float, default=120.0)
    p.add_argument("--align-timeout", type=float, default=30.0)
    p.add_argument("--max-undocking-time", type=float, default=30.0)
    p.add_argument("--manip-delay", type=float, default=5.0,
                   help="Pausa em s na estacao simulando a manipulacao (0 desliga).")
    return p.parse_args(remove_ros_args(args=argv)[1:])


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = _parse_args(argv)
    try:
        stations = [normalize_dock_id(s) for s in args.stations]
        start = normalize_dock_id(args.start) if args.start else None
        finish = normalize_dock_id(args.finish) if args.finish else None
    except Exception as exc:  # noqa: BLE001
        print(f"Erro nos argumentos: {exc}", file=sys.stderr)
        return 1

    all_ids = ([start] if start else []) + stations + ([finish] if finish else [])
    types = _dock_types(args.map_name, args.map_dir, all_ids)

    rclpy.init(args=argv)
    node = MissionRunner()
    ok_geral = True
    try:
        # START (opcional): navegacao pura — nao e bancada, basta chegar na zona.
        if start:
            node.get_logger().info(f"=== START: {start} ===")
            if not node.navigate_to(start, args.map_name, args.map_dir):
                node.get_logger().error("Falha ao ir para START.")
                if not args.continue_on_error:
                    return 1

        for dock_id in stations:
            node.get_logger().info(f"=== Estacao: {dock_id} ===")
            etapa_ok = node.dock(dock_id, args.max_staging_time)
            if etapa_ok and not args.no_align:
                etapa_ok = node.align(dock_id, args.map_name, args.use_lidar_refine,
                                      args.align_timeout)
            if etapa_ok:
                # HOOK de manipulacao (Fase 8). Hoje so um marcador.
                node.get_logger().info(
                    f"[{dock_id}] manipulacao (placeholder — Fase 8): "
                    f"pausa de {args.manip_delay:.0f}s simulando a tarefa do braco.")
                import time as _time
                _time.sleep(max(0.0, args.manip_delay))
            if etapa_ok and not args.no_undock:
                etapa_ok = node.undock(types.get(dock_id, ""), args.max_undocking_time)

            if not etapa_ok:
                ok_geral = False
                node.get_logger().error(f"Estacao {dock_id} falhou.")
                if not args.continue_on_error:
                    return 1

        # FINISH: navegacao pura — nao e bancada, basta chegar na zona (operador
        # 2026-07-21: o dock() ficava ajustando milimetros e estourava o tempo
        # com o robo JA na zona do finish).
        if finish:
            node.get_logger().info(f"=== FINISH: {finish} ===")
            if not node.navigate_to(finish, args.map_name, args.map_dir):
                ok_geral = False
                node.get_logger().error("Falha ao ir para FINISH.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    node_ok = "concluido" if ok_geral else "com falhas"
    print(f"Fluxo de estacoes {node_ok}.")
    return 0 if ok_geral else 1


if __name__ == "__main__":
    raise SystemExit(main())
