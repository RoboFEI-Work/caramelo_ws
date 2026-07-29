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

Exemplos:
  ros2 run caramelo_navigation run_mission --map-name arena2_520 --task BMT.yaml --dry-run
  ros2 run caramelo_navigation run_mission --map-name arena2_520 --task BMT.yaml --simulate-nav --yes
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
    docking_file, resolve_map_folder, service_areas_file,
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
                   help="YAML da competicao (ex.: BMT.yaml). Caminho relativo que "
                        "nao existir no CWD e' procurado no share de manip_bt "
                        "(behavior_tree_manip/).")
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
    try:
        proc = subprocess.run(
            ["ros2", "action", "list"], capture_output=True, text=True, timeout=10.0)
    except subprocess.TimeoutExpired:
        return set()
    if proc.returncode != 0:
        return set()
    return {line.strip() for line in proc.stdout.splitlines() if line.strip()}


def _preflight(args) -> bool:
    """Espera os servidores da missao aparecerem no grafo, com progresso.

    Sem isso, o executor ficava pendurado EM SILENCIO esperando o move_group
    (minutos com o MoveIt subindo; para sempre com o stack desligado).
    IMPORTANTE: sonda so' o grafo (ros2 action list) — nunca `ros2 param get
    /move_group ...`, que pendura igual quando o MoveIt esta ocupado.
    """
    import time as _time

    exigidos = ["/move_action", "/pick_tag", "/place_tag"]
    if not args.simulate_nav:
        exigidos += ["/navigate_to_pose", "/dock_robot", "/undock_robot", "/align_to_dock"]

    rotulos = {"/move_action": "move_group (MoveIt)"}
    print("Verificando os servidores da missao (pre-flight)...")
    inicio = _time.monotonic()
    pendentes = list(exigidos)
    ultimo_print = {}
    while pendentes:
        decorrido = _time.monotonic() - inicio
        if decorrido > args.preflight_timeout:
            faltam = ", ".join(rotulos.get(n, n) for n in pendentes)
            print(f"\nPre-flight FALHOU apos {decorrido:.0f}s. Ainda faltam: {faltam}",
                  file=sys.stderr)
            print("O stack esta no ar? -> ros2 launch caramelo_bringup "
                  "robot_manipulation.launch.py map_name:=...", file=sys.stderr)
            return False
        visiveis = _graph_actions()
        for nome in list(pendentes):
            if nome in visiveis:
                pendentes.remove(nome)
                print(f"  {rotulos.get(nome, nome)}... ok ({decorrido:.0f}s)")
            else:
                anterior = ultimo_print.get(nome, -10.0)
                if decorrido - anterior >= 10.0:
                    print(f"  aguardando {rotulos.get(nome, nome)}... ({decorrido:.0f}s)")
                    ultimo_print[nome] = decorrido
        if pendentes:
            _time.sleep(1.0)
    print("Pre-flight OK — todos os servidores no ar.\n")
    return True


def _resolve_task_yaml(task: str) -> Path:
    """Resolve o YAML da competicao (CWD primeiro, depois o share de manip_bt)."""
    path = Path(task).expanduser()
    if path.exists():
        return path.resolve()
    if not path.is_absolute():
        try:
            from ament_index_python.packages import (
                PackageNotFoundError, get_package_share_directory,
            )
        except ImportError:
            raise FileNotFoundError(
                f"YAML da tarefa '{task}' nao existe no CWD e ament_index_python "
                "nao esta disponivel para procurar no share de manip_bt.")
        try:
            share = Path(get_package_share_directory("manip_bt"))
        except PackageNotFoundError:
            raise FileNotFoundError(
                f"YAML da tarefa '{task}' nao existe no CWD e o pacote manip_bt "
                "nao foi encontrado (workspace buildado e com source feito?).")
        candidate = share / "behavior_tree_manip" / task
        if candidate.exists():
            return candidate.resolve()
        raise FileNotFoundError(
            f"YAML da tarefa '{task}' nao encontrado nem no CWD nem em {candidate}.")
    raise FileNotFoundError(f"YAML da tarefa nao encontrado: {path}")


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


def _print_plan(actions_yaml: Path) -> None:
    """Imprime o plano gerado (numero da acao, kind e demais campos)."""
    with actions_yaml.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    actions = data.get("actions") or []
    print(f"\n=== Plano da missao ({actions_yaml}) — {len(actions)} acoes ===")
    for i, action in enumerate(actions, start=1):
        kind = action.get("kind", "?")
        campos = ", ".join(
            f"{k}={v}" for k, v in action.items() if k != "kind")
        print(f"  {i:2d}. {kind:<6} {campos}")
    print()


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
    print(f"Tarefa:  {task_yaml}")
    print(f"AprilTag: {apriltag_cfg}")

    try:
        actions_yaml = _generate_actions(task_yaml, apriltag_cfg, map_folder)
    except RuntimeError as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 1

    _print_plan(actions_yaml)

    if args.dry_run:
        return _run_executor(_executor_cmd(actions_yaml, map_folder, args, dry_run=True))

    if not args.yes:
        resposta = input("Executar a missao? [s/N] ").strip().lower()
        if resposta not in ("s", "sim"):
            print("Missao cancelada pelo operador.")
            return 0

    if not _preflight(args):
        return 1

    return _run_executor(_executor_cmd(actions_yaml, map_folder, args, dry_run=False))


if __name__ == "__main__":
    raise SystemExit(main())
