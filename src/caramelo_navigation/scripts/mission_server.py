#!/usr/bin/env python3
"""Servidor da action /caramelo/run_mission: a missao completa exposta por ROS.

Por que existe: a missao so' podia ser disparada pelo run_mission.py, que e'
interativo (pergunta "[s/N]") e imprime em stdout. Nem a GUI Qt (caramelo_gui)
nem o painel web (caramelo_web) conseguiam comecar, acompanhar ou abortar uma
missao sem terminal. Este servidor e' um cliente FINO do run_mission.py: importa
as funcoes dele (resolver task, validar a pasta do mapa, task_planner,
pre-flight, linha de comando do executor) e NAO reimplementa nenhuma regra --
o que roda pela interface e' exatamente o que roda no terminal.

Fluxo de um goal:
  PLANNING   resolve caminhos, valida a pasta do mapa, roda o task_planner
  (dry_run)  devolve as linhas do plano e termina; NAO sobe o executor
             (o --dry-run do bt_yaml_executor monta a arvore e espera o
             move_group: com o stack fora do ar ficaria pendurado)
  PREFLIGHT  espera os action servers aparecerem no grafo (run_mission._preflight)
  RUNNING    sobe o bt_yaml_executor; cada linha do log dele vira `message`
             do MissionStatus (sem indice de passo: o executor atual nao
             publica progresso por acao)
  cancelar   SIGINT no executor (halt limpo da arvore, nunca SIGKILL)

Um goal por vez: uma segunda missao concorrente disputaria o braco e a base.
Um servidor por grafo: se ja houver outro /caramelo/run_mission no ar (GUI e
painel web subidos juntos), este processo avisa e sai (single_instance:=true).

Publica MissionStatus em /caramelo/mission/status (transient_local, depth 64:
quem conecta no meio da missao recebe o historico recente).

Como roda: ros2 launch caramelo_navigation mission_server.launch.py
           (incluido pelos launches da GUI e do painel web)
"""
import argparse
import contextlib
import io
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from caramelo_msgs.action import RunMission
from caramelo_msgs.msg import MissionStatus

# run_mission.py mora ao lado (install(FILES) no mesmo lib/). Com
# --symlink-install o executavel e' um link: entram o diretorio do link e o do
# alvo, para o import funcionar dos dois jeitos.
for _pasta in (os.path.dirname(os.path.realpath(__file__)),
               os.path.dirname(os.path.abspath(__file__))):
    if _pasta not in sys.path:
        sys.path.insert(0, _pasta)
import run_mission as rm  # noqa: E402

ACTION_NAME = "/caramelo/run_mission"
STATUS_TOPIC = "/caramelo/mission/status"

# Prefixo do log do rclcpp: "[INFO] [1761234567.123456789] [bt_yaml_executor]: msg"
_PREFIXO_LOG = re.compile(r"^\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[[0-9.]+\] ")
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def status_qos() -> QoSProfile:
    """transient_local com historico fundo: a UI e' sempre late joiner."""
    return QoSProfile(
        depth=64,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


# ----------------------------------------------------------------- helpers puros
def task_id_of(task_yaml) -> str:
    """task_id da tarefa (rotulo do MissionStatus). Nunca levanta."""
    try:
        with Path(task_yaml).open("r", encoding="utf-8") as handle:
            return str((yaml.safe_load(handle) or {}).get("task_id", "") or "")
    except Exception:  # noqa: BLE001
        return ""


def finish_param(finish_dock_id: str) -> str:
    """Valor de --finish para o run_mission. Vazio vira "''" de proposito.

    `-p finish_dock_id:=` (string vazia crua) e' recusado pelo rcl ("Couldn't
    parse parameter override rule") e derruba o executor antes de comecar; o
    override e' lido como YAML, entao a string vazia precisa das aspas literais.
    """
    return finish_dock_id if finish_dock_id else "''"


def args_do_run_mission(goal) -> argparse.Namespace:
    """Namespace com os campos que run_mission._executor_cmd/_preflight leem."""
    return argparse.Namespace(
        simulate_nav=bool(goal.simulate_nav),
        finish=finish_param(goal.finish_dock_id),
        use_lidar_refine=bool(goal.use_lidar_refine),
        preflight_timeout=float(goal.preflight_timeout or 120.0),
    )


def executor_cmd(actions_yaml: Path, map_folder: Path, goal) -> list:
    """Linha de comando do executor: a do run_mission + o que so' a UI escolhe."""
    cmd = rm._executor_cmd(actions_yaml, map_folder, args_do_run_mission(goal), dry_run=False)
    if not goal.use_lidar_refine:
        # O run_mission so' sabe LIGAR o refino (flag); o default do executor
        # e' true, entao desligar precisa do override explicito.
        cmd += ["-p", "use_lidar_refine:=false"]
    if goal.skip_startup_home:
        cmd += ["-p", "startup_home:=false"]
    return cmd


def linhas_do_plano(actions_yaml: Path) -> tuple:
    """(linhas legiveis, lista de acoes) do plano, no formato do run_mission."""
    saida = io.StringIO()
    with contextlib.redirect_stdout(saida):
        actions = rm._print_plan(actions_yaml)
    linhas = [
        linha.rstrip() for linha in saida.getvalue().splitlines()
        if linha.strip() and not linha.startswith("===")
    ]
    return linhas, actions


def limpar_linha(linha: str) -> str:
    """Tira cor ANSI e o carimbo de tempo do rclcpp; mantem [no]: mensagem."""
    return _PREFIXO_LOG.sub("", _ANSI.sub("", linha)).strip()


class _Escrivao(io.TextIOBase):
    """Arquivo falso: cada linha impressa pelo run_mission vira um callback."""

    def __init__(self, on_line):
        super().__init__()
        self._on_line = on_line
        self._resto = ""

    def write(self, texto):
        self._resto += texto
        while "\n" in self._resto:
            linha, self._resto = self._resto.split("\n", 1)
            if linha.strip():
                self._on_line(linha.rstrip())
        return len(texto)

    def flush(self):
        if self._resto.strip():
            self._on_line(self._resto.rstrip())
        self._resto = ""


# ------------------------------------------------------------------------ no'
class MissionServer(Node):
    def __init__(self):
        super().__init__("caramelo_mission_server")
        self.declare_parameter("single_instance", True)

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False
        self._proc = None
        self._goal_handle = None
        self._mission_started = time.monotonic()
        self._task_id = ""
        self._mission_id = ""
        self._server = None

        # Ultima linha do executor ainda nao publicada (o timer abaixo publica
        # no maximo 4x/s, para o log do executor nao virar rajada no topico).
        self._linha_pendente = None
        self._status_pub = self.create_publisher(MissionStatus, STATUS_TOPIC, status_qos())
        self.create_timer(0.25, self._publicar_linha_pendente, callback_group=self._cb_group)

    def iniciar_servidor(self):
        self._server = ActionServer(
            self, RunMission, ACTION_NAME,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )
        self._publish(MissionStatus.STATE_IDLE, "Servidor de missao no ar.")
        self.get_logger().info(f"Servidor de missao no ar em {ACTION_NAME}.")

    def outro_servidor_no_ar(self) -> bool:
        """Ja existe /caramelo/run_mission servido por outro processo?"""
        servicos = dict(self.get_service_names_and_types())
        return f"{ACTION_NAME}/_action/send_goal" in servicos

    # ------------------------------------------------------------------ status
    def _publish(self, state, message, stage=""):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = self._mission_id
        msg.task_id = self._task_id
        msg.action_index = -1
        msg.action_total = 0
        msg.stage = stage
        msg.state = state
        msg.message = message
        msg.elapsed = time.monotonic() - self._mission_started
        try:
            self._status_pub.publish(msg)
        except Exception:  # noqa: BLE001 - publisher ja destruido no desligamento
            pass
        with self._lock:
            handle = self._goal_handle
        if handle is not None and handle.is_active:
            feedback = RunMission.Feedback()
            feedback.status = msg
            try:
                handle.publish_feedback(feedback)
            except Exception:  # noqa: BLE001
                pass

    def _progresso(self, state):
        """Gancho: cada linha impressa pelo run_mission vira log e MissionStatus."""
        def _fala(texto):
            self.get_logger().info(texto)
            self._publish(state, texto)
        return _fala

    def _linha_do_executor(self, linha: str):
        # Ecoa no terminal do servidor (o operador continua vendo o executor)
        # e guarda para o topico, que recebe a ultima linha a cada 0,25 s.
        print(linha, flush=True)
        with self._lock:
            self._linha_pendente = limpar_linha(linha)

    def _publicar_linha_pendente(self):
        with self._lock:
            linha = self._linha_pendente
            self._linha_pendente = None
            rodando = self._proc is not None and self._proc.poll() is None
        if linha and rodando:
            self._publish(MissionStatus.STATE_RUNNING, linha, stage="executor")

    # ------------------------------------------------------------- ciclo do goal
    def _on_goal(self, goal_request):
        del goal_request
        with self._lock:
            if self._busy:
                self.get_logger().warn("Goal recusado: ja existe uma missao em andamento.")
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        del goal_handle
        with self._lock:
            proc = self._proc
        if proc is not None and proc.poll() is None:
            self.get_logger().warn(
                "Cancelamento pedido: SIGINT no executor para halt limpo da arvore.")
            proc.send_signal(signal.SIGINT)
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        with self._lock:
            self._goal_handle = goal_handle
            self._proc = None
        self._mission_started = time.monotonic()
        self._mission_id = ""
        self._task_id = ""

        result = RunMission.Result()
        try:
            return self._run(goal, goal_handle, result)
        except Exception as exc:  # noqa: BLE001 - a falha tem que virar resultado
            self.get_logger().error(f"Missao falhou: {exc}")
            self._publish(MissionStatus.STATE_FAILED, str(exc))
            result.success = False
            result.message = str(exc)
            if goal_handle.is_active:
                goal_handle.abort()
            return result
        finally:
            with self._lock:
                self._busy = False
                self._proc = None
                self._goal_handle = None

    def _planejar(self, goal):
        """Mesmos passos do run_mission.main ate' o plano: devolve
        (task_yaml, map_folder, actions_yaml, linhas, actions)."""
        task_yaml = rm._resolve_task_yaml(goal.task_yaml)
        map_folder = rm.resolve_map_folder(goal.map_name, goal.map_dir or None)
        rm._validate_map_folder(map_folder)
        self._task_id = task_id_of(task_yaml)

        if goal.actions_yaml:
            actions_yaml = Path(goal.actions_yaml).expanduser()
            if not actions_yaml.is_file():
                raise FileNotFoundError(f"actions.yaml informado nao existe: {actions_yaml}")
            self._progresso(MissionStatus.STATE_PLANNING)(f"Reusando o plano {actions_yaml}")
        else:
            apriltag_cfg = rm._resolve_apriltag_cfg()
            fala = self._progresso(MissionStatus.STATE_PLANNING)
            fala(f"Mapa: {map_folder}")
            fala(f"Tarefa: {task_yaml}")
            escrivao = _Escrivao(fala)
            with contextlib.redirect_stdout(escrivao), contextlib.redirect_stderr(escrivao):
                actions_yaml = rm._generate_actions(task_yaml, apriltag_cfg, map_folder)
            escrivao.flush()

        linhas, actions = linhas_do_plano(actions_yaml)
        return task_yaml, map_folder, actions_yaml, linhas, actions

    def _run(self, goal, goal_handle, result):
        # --- planejamento ---
        self._publish(MissionStatus.STATE_PLANNING, "Preparando a missao...")
        _task_yaml, map_folder, actions_yaml, linhas, actions = self._planejar(goal)
        self._mission_id = actions_yaml.stem
        result.actions_yaml = str(actions_yaml)
        result.plan_rows = linhas
        self._publish(
            MissionStatus.STATE_PLANNING, f"Plano com {len(linhas)} acoes: {actions_yaml}")

        # --- dry-run: so' o plano, nada sobe ---
        if goal.dry_run:
            result.success = True
            result.message = (
                f"Plano gerado em {actions_yaml} ({len(linhas)} acoes). "
                "O executor ainda acrescenta o braco em home no inicio e a ida ao FINISH.")
            self._publish(MissionStatus.STATE_IDLE, "Dry-run concluido (nada se moveu).")
            goal_handle.succeed()
            return result

        if goal_handle.is_cancel_requested:
            return self._cancelado(goal_handle, result)

        # --- pre-flight (o do run_mission; a espera nao e' interrompivel) ---
        self._publish(MissionStatus.STATE_PREFLIGHT, "Verificando os servidores da missao...")
        escrivao = _Escrivao(self._progresso(MissionStatus.STATE_PREFLIGHT))
        with contextlib.redirect_stdout(escrivao), contextlib.redirect_stderr(escrivao):
            ok = rm._preflight(args_do_run_mission(goal), actions)
        escrivao.flush()
        if goal_handle.is_cancel_requested:
            return self._cancelado(goal_handle, result)
        if not ok:
            msg = ("Pre-flight falhou: faltam servidores da missao. O stack esta no ar? "
                   "ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=...")
            self._publish(MissionStatus.STATE_FAILED, msg)
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        # --- execucao ---
        cmd = executor_cmd(actions_yaml, map_folder, goal)
        self.get_logger().info(f"Executando: {' '.join(cmd)}")
        self._publish(MissionStatus.STATE_RUNNING, "Executor da missao iniciado.", stage="inicio")
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        with self._lock:
            self._proc = proc
        leitor = threading.Thread(
            target=self._ler_saida, args=(proc,), daemon=True, name="executor_stdout")
        leitor.start()
        codigo = proc.wait()
        leitor.join(timeout=2.0)

        # Exit codes do bt_yaml_executor: 0 SUCCESS, 1 FAILURE, 130 SIGINT.
        if codigo == 0:
            result.success = True
            result.message = "Missao concluida com sucesso."
            self._publish(MissionStatus.STATE_DONE, result.message, stage="fim")
            goal_handle.succeed()
        elif codigo == 130 or goal_handle.is_cancel_requested:
            return self._cancelado(goal_handle, result)
        else:
            result.success = False
            result.message = f"Executor retornou {codigo} (ver o log do mission_server)."
            self._publish(MissionStatus.STATE_FAILED, result.message, stage="fim")
            goal_handle.abort()
        return result

    def _cancelado(self, goal_handle, result):
        result.success = False
        result.message = "Missao abortada."
        self._publish(MissionStatus.STATE_ABORTED, result.message, stage="fim")
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _ler_saida(self, proc):
        try:
            for linha in proc.stdout:
                self._linha_do_executor(linha.rstrip("\n"))
        except ValueError:
            pass  # stdout fechado no desligamento

    def encerrar_executor(self):
        """Ctrl-C no servidor com missao em voo: repassa ao executor."""
        with self._lock:
            proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        self.get_logger().warn("Desligando com missao em voo: SIGINT no executor.")
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self.get_logger().error("Executor nao encerrou em 10 s; segue vivo.")


def main(argv=None):
    rclpy.init(args=argv)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        if node.get_parameter("single_instance").value:
            # Da' tempo ao discovery antes de perguntar ao grafo.
            fim = time.monotonic() + 2.0
            while rclpy.ok() and time.monotonic() < fim:
                executor.spin_once(timeout_sec=0.1)
            if node.outro_servidor_no_ar():
                node.get_logger().warn(
                    f"Ja existe um servidor em {ACTION_NAME} neste grafo; este processo sai "
                    "(single_instance:=true). Suba so' um mission_server por robo.")
                return 0
        node.iniciar_servidor()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C no terminal (KeyboardInterrupt) ou SIGINT vindo do ros2 launch
        # (rclpy derruba o contexto e o spin levanta ExternalShutdownException).
        pass
    finally:
        node.encerrar_executor()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
