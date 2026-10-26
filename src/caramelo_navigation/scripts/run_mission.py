#!/usr/bin/env python3
"""Ponto de entrada da MISSAO COMPLETA (navegacao + manipulacao) do Caramelo.

Fluxo (integracao manip -> caramelo):
  1. Resolve a pasta do mapa (docking.yaml, service_areas.yaml e
     ws_table_mapping.yaml precisam existir nela).
  2. Roda o task_planner (manip_bt) sobre o YAML da competicao, gerando o
     actions.yaml da missao em ~/.ros/caramelo_missions/.
  3. Mostra o plano gerado e pede confirmacao ao operador.
  4. Executa o bt_yaml_executor (manip_bt), que orquestra DockRobot,
     AlignToDock, UndockRobot e as actions de pick/place do braco.

Este script NAO usa rclpy diretamente: ele apenas orquestra os processos via
`ros2 run`, no mesmo espirito dos demais utilitarios do pacote. Ctrl+C e'
repassado como SIGINT ao executor, que faz o halt limpo da BT (nunca SIGKILL).

As tasks vivem em missions/ na raiz do workspace (fonte unica); --task e'
um caminho absoluto ou relativo ao CWD, sem fallback para share de pacote.

Exemplos (a partir da raiz do workspace):
  ros2 run caramelo_navigation run_mission --map-name arena2_520 --task missions/task_BMT.yaml --dry-run
  ros2 run caramelo_navigation run_mission --map-name arena2_520 --task missions/task_BMT.yaml --simulate-nav --yes
"""
import argparse
import datetime
import os
import signal
import subprocess
import sys
from pathlib import Path

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from caramelo_docking_common import (  # noqa: E402
    docking_file, graph_action_names, resolve_map_folder, service_areas_file,
    wait_for_action_servers,
)

MISSIONS_DIR = Path.home() / ".ros" / "caramelo_missions"


def _parse_args(argv):
    p = argparse.ArgumentParser(
        description="Executa a missao completa (nav + manipulacao) do Caramelo.")
    p.add_argument("--map-name", required=True,
                   help="Pasta do mapa (contem docking.yaml, service_areas.yaml, "
                        "ws_table_mapping.yaml).")
    p.add_argument("--map-dir", default=None,
                   help="Raiz alternativa dos mapas (opcional).")
    p.add_argument("--task", required=True,
                   help="YAML da competicao (ex.: missions/task_BMT.yaml). Caminho "
                        "absoluto ou relativo ao CWD; missions/ na raiz do "
                        "workspace e' a fonte unica (sem fallback para share).")
    p.add_argument("--dry-run", action="store_true",
                   help="So gera e mostra o plano; roda o executor com --dry-run.")
    p.add_argument("--simulate-nav", action="store_true",
                   help="Executor simula a navegacao (sem Nav2 no ar).")
    p.add_argument("--yes", action="store_true",
                   help="Nao pede confirmacao antes de executar.")
    p.add_argument("--finish", default="FINISH",
                   help='Dock final apos a missao (default: FINISH; "" desliga).')
    p.add_argument("--use-lidar-refine", action="store_true",
                   help="Repassa use_lidar_refine:=true ao executor (refino no /scan).")
    p.add_argument("--preflight-timeout", type=float, default=120.0,
                   help="Segundos maximos aguardando os servidores da missao "
                        "(move_group, pick/place, nav) antes de abortar.")
    return p.parse_args(argv[1:])


def _graph_actions() -> set:
    """Snapshot dos action servers visiveis no grafo (barato, nunca pendura)."""
    return graph_action_names(timeout_s=10.0)


def _preflight(args, actions=None) -> bool:
    """Espera os servidores da missao aparecerem no grafo, com progresso.

    Sem isso, o executor ficava pendurado EM SILENCIO esperando o move_group
    (minutos com o MoveIt subindo; para sempre com o stack desligado).
    IMPORTANTE: sonda so' o grafo (ros2 action list) — nunca `ros2 param get
    /move_group ...`, que pendura igual quando o MoveIt esta ocupado.
    `actions` = plano gerado (lista de dicts): so exige o /amt1_sort quando a
    missao tem uma acao amt1_sort (AMT1, 2026-08-28). A espera em si e' a
    wait_for_action_servers do caramelo_docking_common (compartilhada com o
    tour_docks).
    """
    exigidos = ["/move_action", "/pick_tag", "/place_tag"]
    if not args.simulate_nav:
        # /nudge_base (2026-08-28): ajuste lateral da fila de alcance, servido
        # pelo mesmo dock_align_node do /align_to_dock.
        exigidos += ["/navigate_to_pose", "/dock_robot", "/undock_robot",
                     "/align_to_dock", "/nudge_base"]
    if any(a.get("kind") == "amt1_sort" for a in (actions or [])):
        # amt1_sort_action_node (manip_task_execution) so faz falta na AMT1.
        exigidos.append("/amt1_sort")

    rotulos = {"/move_action": "move_group (MoveIt)",
               "/nudge_base": "nudge_base (dock_align_node)",
               "/amt1_sort": "amt1_sort (rotina AMT1)"}
    print("Verificando os servidores da missao (pre-flight)...")
    return wait_for_action_servers(
        exigidos, args.preflight_timeout, rotulos,
        hint="O stack esta no ar? -> ros2 launch caramelo_bringup "
             "robot_manipulation.launch.py map_name:=...",
        list_actions=_graph_actions)


def _resolve_task_yaml(task: str) -> Path:
    """Resolve o YAML da competicao: caminho literal (absoluto ou relativo ao CWD).

    missions/ na raiz do workspace e' a fonte unica das tasks; nao ha fallback
    para o share de pacote. Se o arquivo nao existir, o erro lista os
    missions/*.yaml visiveis a partir do CWD para o operador se orientar.
    """
    path = Path(task).expanduser()
    if path.is_file():
        return path.resolve()
    candidatos = sorted(Path("missions").glob("*.yaml"))
    if candidatos:
        dica = ("\nTasks disponiveis em missions/:\n  - "
                + "\n  - ".join(str(c) for c in candidatos))
    else:
        dica = ("\nNenhum missions/*.yaml a partir do CWD atual: rode a partir da "
                "raiz do workspace (ex.: --task missions/task_BMT.yaml) ou passe "
                "o caminho absoluto.")
    raise FileNotFoundError(f"YAML da tarefa nao encontrado: {path}{dica}")


def _validate_map_folder(map_folder: Path) -> None:
    """Garante que a pasta do mapa tem os 3 arquivos exigidos pela missao."""
    faltando = []
    if not docking_file(map_folder).exists():
        faltando.append("docking.yaml (rode init_map_docking / save_dock_pose)")
    if not service_areas_file(map_folder).exists():
        faltando.append("service_areas.yaml (rode init_service_areas)")
    if not (map_folder / "ws_table_mapping.yaml").exists():
        faltando.append("ws_table_mapping.yaml (mapeamento WS -> Mesa/dock_id)")
    if faltando:
        itens = "\n  - ".join(faltando)
        raise FileNotFoundError(
            f"Pasta do mapa {map_folder} esta incompleta. Faltam:\n  - {itens}")


def _resolve_apriltag_cfg() -> Path:
    """Config das tags AprilTag usada pelo task_planner (share do manip_bringup)."""
    try:
        from ament_index_python.packages import (
            PackageNotFoundError, get_package_share_directory,
        )
    except ImportError:
        raise FileNotFoundError(
            "ament_index_python nao esta disponivel; nao consigo localizar o "
            "config AprilTag do manip_bringup.")
    try:
        share = Path(get_package_share_directory("manip_bringup"))
    except PackageNotFoundError:
        raise FileNotFoundError(
            "Pacote manip_bringup nao encontrado. Ele ainda nao foi buildado? "
            "Rode o build do workspace de manipulacao e faca source do install.")
    cfg = share / "config" / "tags_36h11.yaml"
    if not cfg.exists():
        raise FileNotFoundError(f"Config AprilTag nao encontrado: {cfg}")
    return cfg


def _generate_actions(task_yaml: Path, apriltag_cfg: Path, map_folder: Path) -> Path:
    """Roda o task_planner e devolve o caminho do actions.yaml gerado."""
    MISSIONS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = MISSIONS_DIR / f"{stamp}_actions.yaml"
    cmd = [
        "ros2", "run", "manip_bt", "task_planner",
        str(task_yaml), str(out), str(apriltag_cfg),
        str(map_folder / "ws_table_mapping.yaml"),
    ]
    print(f"Gerando plano: {' '.join(cmd)}")
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        print("task_planner falhou:", file=sys.stderr)
        print(proc.stderr, file=sys.stderr)
        raise RuntimeError(f"task_planner retornou {proc.returncode}.")
    if proc.stdout.strip():
        print(proc.stdout.strip())
    return out


def _print_plan(actions_yaml: Path) -> list:
    """Imprime o plano gerado (numero da acao, kind e demais campos).

    Devolve a lista de acoes (dicts) — o pre-flight usa para saber quais
    servidores a missao exige (2026-08-28: /amt1_sort so na AMT1).
    """
    with actions_yaml.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    actions = data.get("actions") or []
    print(f"\n=== Plano da missao ({actions_yaml}) — {len(actions)} acoes ===")
    for i, action in enumerate(actions, start=1):
        kind = action.get("kind", "?")
        campos = ", ".join(
            f"{k}={v}" for k, v in action.items() if k != "kind")
        print(f"  {i:2d}. {kind:<9} {campos}")
    print()
    return actions


def _executor_cmd(actions_yaml: Path, map_folder: Path, args, dry_run: bool):
    cmd = ["ros2", "run", "manip_bt", "bt_yaml_executor", str(actions_yaml)]
    if dry_run:
        cmd.append("--dry-run")
    cmd += [
        "--ros-args",
        "-p", f"map_folder:={map_folder}",
        "-p", f"simulate_navigation:={'true' if args.simulate_nav else 'false'}",
        "-p", f"finish_dock_id:={args.finish}",
    ]
    if args.use_lidar_refine:
        cmd += ["-p", "use_lidar_refine:=true"]
    return cmd


def _run_executor(cmd) -> int:
    """Roda o executor repassando SIGINT (halt limpo da BT — nunca SIGKILL)."""
    print(f"Executando: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd)

    def _forward_sigint(signum, frame):  # noqa: ARG001
        if proc.poll() is None:
            print("\nSIGINT recebido — repassando ao executor para halt limpo...")
            proc.send_signal(signal.SIGINT)

    previous = signal.signal(signal.SIGINT, _forward_sigint)
    try:
        return proc.wait()
    finally:
        signal.signal(signal.SIGINT, previous)


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = _parse_args(argv)

    try:
        task_yaml = _resolve_task_yaml(args.task)
        map_folder = resolve_map_folder(args.map_name, args.map_dir)
        _validate_map_folder(map_folder)
        apriltag_cfg = _resolve_apriltag_cfg()
    except (FileNotFoundError, ValueError) as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 1

    print(f"Mapa:    {map_folder}")
    print(f"Tarefa:  {task_yaml} (resolvido de --task {args.task})")
    print(f"AprilTag: {apriltag_cfg}")

    try:
        actions_yaml = _generate_actions(task_yaml, apriltag_cfg, map_folder)
    except RuntimeError as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 1

    actions = _print_plan(actions_yaml)

    if args.dry_run:
        return _run_executor(_executor_cmd(actions_yaml, map_folder, args, dry_run=True))

    if not args.yes:
        resposta = input("Executar a missao? [s/N] ").strip().lower()
        if resposta not in ("s", "sim"):
            print("Missao cancelada pelo operador.")
            return 0

    if not _preflight(args, actions):
        return 1

    return _run_executor(_executor_cmd(actions_yaml, map_folder, args, dry_run=False))


if __name__ == "__main__":
    raise SystemExit(main())
