#!/usr/bin/env python3
"""Logica compartilhada da missao completa (navegacao + manipulacao).

Extraido do run_mission.py sem mudanca de comportamento, para que o
run_mission (terminal) e o mission_server (action, usado pela GUI Qt e pelo
painel web) executem EXATAMENTE o mesmo caminho. Se os dois tivessem copias
separadas, uma missao disparada pela interface poderia divergir da que o
operador roda no terminal -- e a divergencia so' apareceria em prova.

Segue o padrao dos demais modulos de apoio do pacote (caramelo_docking_common,
caramelo_service_area_common): instalado com install(FILES), importado pelos
scripts vizinhos via sys.path.
"""
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

# Action servers exigidos antes de comecar. Os de navegacao caem fora quando o
# executor simula a navegacao.
PREFLIGHT_SEMPRE = ["/move_action", "/pick_tag", "/place_tag"]
PREFLIGHT_NAV = ["/navigate_to_pose", "/dock_robot", "/undock_robot", "/align_to_dock"]
PREFLIGHT_ROTULOS = {"/move_action": "move_group (MoveIt)"}

DEFAULT_PREFLIGHT_TIMEOUT = 120.0


def graph_actions() -> set:
    """Snapshot dos action servers visiveis no grafo (barato, nunca pendura)."""
    try:
        proc = subprocess.run(
            ["ros2", "action", "list"], capture_output=True, text=True, timeout=10.0)
    except subprocess.TimeoutExpired:
        return set()
    if proc.returncode != 0:
        return set()
    return {line.strip() for line in proc.stdout.splitlines() if line.strip()}


def preflight_required(simulate_nav: bool) -> list:
    return list(PREFLIGHT_SEMPRE) + ([] if simulate_nav else list(PREFLIGHT_NAV))


def preflight(simulate_nav: bool, timeout: float, on_progress=None,
              should_abort=None) -> tuple:
    """Espera os servidores da missao aparecerem no grafo.

    Sem isso, o executor ficava pendurado EM SILENCIO esperando o move_group
    (minutos com o MoveIt subindo; para sempre com o stack desligado).
    IMPORTANTE: sonda so' o grafo (ros2 action list) -- nunca `ros2 param get
    /move_group ...`, que pendura igual quando o MoveIt esta ocupado.

    on_progress(texto) recebe cada linha de progresso; e' o gancho que permite
    ao mission_server transformar o mesmo texto em feedback da action.
    should_abort() permite cancelar a espera de fora.

    Devolve (ok, mensagem_final).
    """
    import time as _time

    exigidos = preflight_required(simulate_nav)
    fala = on_progress or (lambda _texto: None)
    fala("Verificando os servidores da missao (pre-flight)...")

    inicio = _time.monotonic()
    pendentes = list(exigidos)
    ultimo_print = {}
    while pendentes:
        if should_abort is not None and should_abort():
            return False, "Pre-flight cancelado."
        decorrido = _time.monotonic() - inicio
        if decorrido > timeout:
            faltam = ", ".join(PREFLIGHT_ROTULOS.get(n, n) for n in pendentes)
            return False, (
                f"Pre-flight FALHOU apos {decorrido:.0f}s. Ainda faltam: {faltam}. "
                "O stack esta no ar? -> ros2 launch caramelo_bringup "
                "robot_manipulation.launch.py map_name:=...")
        visiveis = graph_actions()
        for nome in list(pendentes):
            if nome in visiveis:
                pendentes.remove(nome)
                fala(f"  {PREFLIGHT_ROTULOS.get(nome, nome)}... ok ({decorrido:.0f}s)")
            else:
                anterior = ultimo_print.get(nome, -10.0)
                if decorrido - anterior >= 10.0:
                    fala(f"  aguardando {PREFLIGHT_ROTULOS.get(nome, nome)}... "
                         f"({decorrido:.0f}s)")
                    ultimo_print[nome] = decorrido
        if pendentes:
            _time.sleep(1.0)
    fala("Pre-flight OK — todos os servidores no ar.")
    return True, "Pre-flight OK."


def resolve_task_yaml(task: str) -> Path:
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


def validate_map_folder(map_folder: Path) -> None:
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


def resolve_apriltag_cfg() -> Path:
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


def generate_actions(task_yaml: Path, apriltag_cfg: Path, map_folder: Path,
                     on_progress=None) -> Path:
    """Roda o task_planner e devolve o caminho do actions.yaml gerado."""
    fala = on_progress or (lambda _texto: None)
    MISSIONS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = MISSIONS_DIR / f"{stamp}_actions.yaml"
    cmd = [
        "ros2", "run", "manip_bt", "task_planner",
        str(task_yaml), str(out), str(apriltag_cfg),
        str(map_folder / "ws_table_mapping.yaml"),
    ]
    fala(f"Gerando plano: {' '.join(cmd)}")
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"task_planner retornou {proc.returncode}.\n{proc.stderr}")
    if proc.stdout.strip():
        fala(proc.stdout.strip())
    return out


def plan_rows(actions_yaml: Path) -> list:
    """Plano legivel: uma linha por acao, no formato que o operador ja conhece."""
    with actions_yaml.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    actions = data.get("actions") or []
    linhas = []
    for i, action in enumerate(actions, start=1):
        kind = action.get("kind", "?")
        campos = ", ".join(f"{k}={v}" for k, v in action.items() if k != "kind")
        linhas.append(f"  {i:2d}. {kind:<6} {campos}")
    return linhas


def task_id_of(task_yaml: Path) -> str:
    """task_id da tarefa da competicao (rotulo do MissionStatus). Nunca levanta."""
    try:
        with Path(task_yaml).open("r", encoding="utf-8") as handle:
            return str((yaml.safe_load(handle) or {}).get("task_id", "") or "")
    except Exception:
        return ""


def executor_cmd(actions_yaml: Path, map_folder: Path, *, dry_run: bool,
                 simulate_nav: bool, finish_dock_id: str, use_lidar_refine: bool,
                 task_id: str = "", skip_startup_home: bool = False) -> list:
    cmd = ["ros2", "run", "manip_bt", "bt_yaml_executor", str(actions_yaml)]
    if dry_run:
        cmd.append("--dry-run")
    # O `--finish ""` (documentado como "desliga o epilogo") gerava
    # `-p finish_dock_id:=`, que o rcl recusa com "Couldn't parse parameter
    # override rule" e derruba o executor antes de comecar. O override e' lido
    # como YAML, entao a string vazia precisa das aspas literais.
    finish_yaml = finish_dock_id if finish_dock_id else "''"
    cmd += [
        "--ros-args",
        "-p", f"map_folder:={map_folder}",
        "-p", f"simulate_navigation:={'true' if simulate_nav else 'false'}",
        "-p", f"finish_dock_id:={finish_yaml}",
    ]
    if use_lidar_refine:
        cmd += ["-p", "use_lidar_refine:=true"]
    if task_id:
        cmd += ["-p", f"task_id:={task_id}"]
    # O prologo "braco em home" e' um GoToNamedPose, que instancia o
    # MoveGroupInterface na construcao da arvore -- ou seja, exige o move_group
    # de pe' mesmo que a missao nao tenha nenhum passo de manipulacao. So'
    # desligamos quando pedido explicitamente; o default preserva o comportamento
    # de sempre (o robo pode bootar com o braco em pose qualquer).
    if skip_startup_home:
        cmd += ["-p", "startup_home:=false"]
    return cmd


def spawn_executor(cmd) -> subprocess.Popen:
    """Sobe o executor sem instalar handler de sinal.

    O run_executor troca o handler de SIGINT do processo, o que so' e' permitido
    na thread principal -- dentro do mission_server (que roda o goal numa thread
    do executor do rclpy) isso levantaria ValueError. Quem chama daqui cancela
    mandando SIGINT no Popen.
    """
    return subprocess.Popen(cmd)


def run_executor(cmd, on_progress=None) -> int:
    """Roda o executor repassando SIGINT (halt limpo da BT -- nunca SIGKILL)."""
    fala = on_progress or (lambda _texto: None)
    fala(f"Executando: {' '.join(cmd)}")
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


def prepare_mission(*, task, map_name, map_dir=None, actions_yaml=None,
                    on_progress=None):
    """Resolve caminhos, valida a pasta do mapa e gera (ou reusa) o plano.

    Devolve (task_yaml, map_folder, actions_yaml, linhas_do_plano).
    Levanta FileNotFoundError / ValueError / RuntimeError com mensagem pronta
    para o operador.
    """
    task_yaml = resolve_task_yaml(task)
    map_folder = resolve_map_folder(map_name, map_dir)
    validate_map_folder(map_folder)

    if actions_yaml:
        caminho = Path(actions_yaml).expanduser()
        if not caminho.exists():
            raise FileNotFoundError(f"actions.yaml informado nao existe: {caminho}")
    else:
        apriltag_cfg = resolve_apriltag_cfg()
        fala = on_progress or (lambda _texto: None)
        fala(f"AprilTag: {apriltag_cfg}")
        caminho = generate_actions(task_yaml, apriltag_cfg, map_folder,
                                   on_progress=on_progress)

    return task_yaml, map_folder, caminho, plan_rows(caminho)
