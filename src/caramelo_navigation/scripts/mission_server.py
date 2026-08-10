#!/usr/bin/env python3
"""Servidor da action RunMission: a missao completa exposta por ROS.

Por que existe: ate' aqui a missao so' podia ser disparada pelo run_mission.py,
que e' interativo e imprime em stdout. Nem a GUI Qt nem o painel web conseguiam
comecar ou acompanhar uma missao sem parsear terminal. Com este servidor as duas
interfaces viram clientes do MESMO caminho que o operador usa no terminal --
task_planner -> plano -> pre-flight -> bt_yaml_executor -- porque toda a logica
mora em caramelo_mission_common.py, compartilhada com o run_mission.

Fluxo de um goal:
  PLANNING   resolve caminhos, valida a pasta do mapa, roda o task_planner
  (dry_run)  roda o executor com --dry-run e devolve o plano, sem mover o robo
  PREFLIGHT  espera os action servers aparecerem no grafo
  RUNNING    sobe o bt_yaml_executor; o feedback vem do proprio executor, que
             publica MissionStatus em /caramelo/mission/status
  cancelar   SIGINT no executor (halt limpo da arvore, nunca SIGKILL)

Um goal por vez: uma segunda missao concorrente disputaria o braco e a base.
"""
import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from caramelo_msgs.action import RunMission
from caramelo_msgs.msg import MissionStatus

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import caramelo_mission_common as mc  # noqa: E402

ACTION_NAME = "/caramelo/run_mission"
STATUS_TOPIC = "/caramelo/mission/status"


def status_qos() -> QoSProfile:
    """Mesma QoS do bt_yaml_executor.

    transient_local com historico fundo: o publicador do executor nasce junto com
    a missao, entao todo assinante e' late joiner. Com depth 1 o servidor (e
    qualquer UI) perderia o inicio de toda missao.
    """
    return QoSProfile(
        depth=64,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MissionServer(Node):
    def __init__(self):
        super().__init__("caramelo_mission_server")

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False
        self._proc = None
        self._goal_handle = None
        self._mission_started = time.monotonic()
        self._task_id = ""
        self._mission_id = ""

        self._status_pub = self.create_publisher(
            MissionStatus, STATUS_TOPIC, status_qos())
        # Assina o proprio topico para reencaminhar como feedback o que o
        # bt_yaml_executor publica -- assim o cliente da action ve os passos sem
        # precisar assinar nada alem da action.
        self.create_subscription(
            MissionStatus, STATUS_TOPIC, self._on_status, status_qos(),
            callback_group=self._cb_group)

        self._server = ActionServer(
            self, RunMission, ACTION_NAME,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f"Servidor de missao no ar em {ACTION_NAME}.")

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
        except Exception:  # noqa: BLE001
            # Durante o desligamento o publisher ja' foi destruido. Sem este
            # guard, a excecao de teardown substitui a mensagem de erro real no
            # traceback e esconde a causa da falha.
            pass

    def _on_status(self, msg: MissionStatus):
        with self._lock:
            handle = self._goal_handle
        if handle is None or not handle.is_active:
            return
        feedback = RunMission.Feedback()
        feedback.status = msg
        handle.publish_feedback(feedback)

    def _progress(self, state):
        """Gancho de progresso: cada linha vira log e MissionStatus."""
        def _fala(texto):
            self.get_logger().info(texto)
            self._publish(state, texto)
        return _fala

    # ------------------------------------------------------------- ciclo do goal
    def _on_goal(self, goal_request):
        with self._lock:
            if self._busy:
                self.get_logger().warn(
                    "Goal recusado: ja existe uma missao em andamento.")
                return GoalResponse.REJECT
            self._busy = True
        del goal_request
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        del goal_handle
        with self._lock:
            proc = self._proc
        if proc is not None and proc.poll() is None:
            self.get_logger().warn(
                "Cancelamento pedido — SIGINT no executor para halt limpo da arvore.")
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

    def _run(self, goal, goal_handle, result):
        # --- planejamento ---
        self._publish(MissionStatus.STATE_PLANNING, "Preparando a missao...")
        task_yaml, map_folder, actions_yaml, linhas = mc.prepare_mission(
            task=goal.task_yaml,
            map_name=goal.map_name,
            map_dir=goal.map_dir or None,
            actions_yaml=goal.actions_yaml or None,
            on_progress=self._progress(MissionStatus.STATE_PLANNING),
        )
        self._task_id = mc.task_id_of(task_yaml)
        self._mission_id = actions_yaml.stem
        result.actions_yaml = str(actions_yaml)
        result.plan_rows = linhas

        self._publish(
            MissionStatus.STATE_PLANNING,
            f"Plano com {len(linhas)} acoes: {actions_yaml}")

        cmd_kwargs = dict(
            dry_run=False,
            simulate_nav=goal.simulate_nav,
            finish_dock_id=goal.finish_dock_id,
            use_lidar_refine=goal.use_lidar_refine,
            task_id=self._task_id,
            skip_startup_home=goal.skip_startup_home,
        )

        # --- dry-run: devolve o plano sem mover o robo ---
        if goal.dry_run:
            import subprocess
            cmd = mc.executor_cmd(
                actions_yaml, map_folder, **{**cmd_kwargs, "dry_run": True})
            proc = subprocess.run(cmd, capture_output=True, text=True)
            ok = proc.returncode == 0
            result.success = ok
            result.message = proc.stdout if ok else (proc.stderr or proc.stdout)
            self._publish(
                MissionStatus.STATE_DONE if ok else MissionStatus.STATE_FAILED,
                "Dry-run concluido." if ok else "Dry-run falhou (ver mensagem).")
            goal_handle.succeed() if ok else goal_handle.abort()
            return result

        # --- pre-flight ---
        timeout = goal.preflight_timeout or mc.DEFAULT_PREFLIGHT_TIMEOUT
        ok, msg = mc.preflight(
            goal.simulate_nav, timeout,
            on_progress=self._progress(MissionStatus.STATE_PREFLIGHT),
            should_abort=lambda: goal_handle.is_cancel_requested,
        )
        if not ok:
            self._publish(MissionStatus.STATE_FAILED, msg)
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        # --- execucao ---
        cmd = mc.executor_cmd(actions_yaml, map_folder, **cmd_kwargs)
        self.get_logger().info(f"Executando: {' '.join(cmd)}")
        proc = mc.spawn_executor(cmd)
        with self._lock:
            self._proc = proc

        while proc.poll() is None:
            time.sleep(0.2)
        codigo = proc.returncode

        # Os exit codes vem do bt_yaml_executor: 0 SUCCESS, 1 FAILURE,
        # 130 interrompido por SIGINT (que e' como o cancelamento chega la').
        if codigo == 0:
            result.success = True
            result.message = "Missao concluida com sucesso."
            goal_handle.succeed()
        elif codigo == 130 or goal_handle.is_cancel_requested:
            result.success = False
            result.message = "Missao abortada."
            goal_handle.canceled()
        else:
            result.success = False
            result.message = f"Executor retornou {codigo} (ver /rosout)."
            goal_handle.abort()
        return result


def main(argv=None):
    rclpy.init(args=argv)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
