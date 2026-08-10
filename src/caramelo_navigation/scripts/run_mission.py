#!/usr/bin/env python3
"""Ponto de entrada da MISSAO COMPLETA (navegacao + manipulacao) do Caramelo.

Fluxo (inalterado do ponto de vista do operador):
  1. Resolve a pasta do mapa (docking.yaml, service_areas.yaml e
     ws_table_mapping.yaml precisam existir nela).
  2. Roda o task_planner (manip_bt) sobre o YAML da competicao, gerando o
     actions.yaml da missao em ~/.ros/caramelo_missions/.
  3. Mostra o plano gerado e pede confirmacao ao operador.
  4. Executa o bt_yaml_executor (manip_bt), que orquestra DockRobot,
     AlignToDock, UndockRobot e as actions de pick/place do braco.

O que mudou por dentro: a logica dos 4 passos foi extraida para
caramelo_mission_common.py e agora e' compartilhada com o mission_server (a
action /caramelo/run_mission, usada pela GUI Qt e pelo painel web). Assim uma
missao disparada pela interface percorre exatamente o mesmo caminho que a do
terminal.

O passo 4 e' delegado ao mission_server QUANDO ELE ESTIVER NO AR; se nao
estiver, este script executa localmente como sempre fez. Isso e' proposital:
nenhum fluxo de terminal existente quebra se o servidor nao subir.

Os passos 1-3 ficam locais de proposito -- o plano e a confirmacao precisam
acontecer antes de qualquer coisa se mover, e planejar e' barato. O plano ja
gerado e' repassado ao servidor, que nao replaneja.

Exemplos:
  ros2 run caramelo_navigation run_mission --map-name arena2_520 --task BMT.yaml --dry-run
  ros2 run caramelo_navigation run_mission --map-name arena2_520 --task BMT.yaml --simulate-nav --yes
"""
import argparse
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import caramelo_mission_common as mc  # noqa: E402

ACTION_NAME = "/caramelo/run_mission"
# Quanto esperar o mission_server aparecer antes de cair no modo local. Curto de
# proposito: se o servidor nao esta' no ar, o operador nao pode ficar parado.
SERVER_WAIT = 3.0


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
    p.add_argument("--preflight-timeout", type=float, default=mc.DEFAULT_PREFLIGHT_TIMEOUT,
                   help="Segundos maximos aguardando os servidores da missao "
                        "(move_group, pick/place, nav) antes de abortar.")
    p.add_argument("--local", action="store_true",
                   help="Nao usa o mission_server; executa sempre localmente.")
    return p.parse_args(argv[1:])


def _executar_local(args, actions_yaml, map_folder, task_id) -> int:
    """Caminho de sempre: pre-flight aqui e bt_yaml_executor como filho."""
    ok, msg = mc.preflight(
        args.simulate_nav, args.preflight_timeout, on_progress=print)
    if not ok:
        print(f"\n{msg}", file=sys.stderr)
        return 1
    cmd = mc.executor_cmd(
        actions_yaml, map_folder,
        dry_run=False, simulate_nav=args.simulate_nav,
        finish_dock_id=args.finish, use_lidar_refine=args.use_lidar_refine,
        task_id=task_id)
    return mc.run_executor(cmd, on_progress=print)


def _executar_no_servidor(args, actions_yaml, map_folder) -> int:
    """Delega ao mission_server. Devolve None se o servidor nao estiver no ar."""
    del map_folder  # o servidor reusa o actions_yaml; nao precisa da pasta
    try:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from caramelo_msgs.action import RunMission
    except ImportError as exc:
        print(f"(sem suporte a action: {exc}; executando localmente)")
        return None

    ja_iniciado = rclpy.ok()
    if not ja_iniciado:
        rclpy.init()
    node = Node("run_mission_client")
    cliente = ActionClient(node, RunMission, ACTION_NAME)

    def _encerrar():
        # O Ctrl-C tambem chega ao handler interno do rclpy, que derruba o
        # contexto por conta propria e em paralelo -- checar rclpy.ok() antes nao
        # basta, e' uma corrida. Sem estes guards o traceback do shutdown esconde
        # o motivo real da saida.
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if ja_iniciado:
            return
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass

    if not cliente.wait_for_server(timeout_sec=SERVER_WAIT):
        _encerrar()
        return None

    goal = RunMission.Goal()
    goal.task_yaml = args.task
    goal.map_name = args.map_name
    goal.map_dir = args.map_dir or ""
    goal.dry_run = False
    goal.simulate_nav = args.simulate_nav
    goal.use_lidar_refine = args.use_lidar_refine
    goal.finish_dock_id = args.finish
    goal.actions_yaml = str(actions_yaml)
    goal.preflight_timeout = args.preflight_timeout

    ultimo = {"linha": None}

    def _feedback(msg):
        st = msg.feedback.status
        if st.action_index >= 0:
            linha = (f"[{st.action_index + 1}/{st.action_total}] "
                     f"{st.action_kind} {st.action_target} — {st.stage}")
        else:
            linha = st.message
        if linha and linha != ultimo["linha"]:
            print(linha)
            ultimo["linha"] = linha

    print(f"Usando o mission_server em {ACTION_NAME}.")
    envio = cliente.send_goal_async(goal, feedback_callback=_feedback)
    rclpy.spin_until_future_complete(node, envio)
    handle = envio.result()
    if handle is None or not handle.accepted:
        print("Goal recusado pelo mission_server (ja existe missao em andamento?).",
              file=sys.stderr)
        _encerrar()
        return 1

    # Ctrl-C vira cancelamento da action; o servidor manda SIGINT no executor,
    # que faz o halt limpo da arvore. Mesmo contrato do modo local.
    def _cancelar(signum, frame):  # noqa: ARG001
        print("\nSIGINT recebido — cancelando a missao no servidor...")
        handle.cancel_goal_async()

    anterior = signal.signal(signal.SIGINT, _cancelar)
    interrompido = False
    resposta = None
    try:
        futuro = handle.get_result_async()
        rclpy.spin_until_future_complete(node, futuro)
        resposta = futuro.result()
    except (KeyboardInterrupt, ExternalShutdownException):
        # O Ctrl-C tambem derruba o contexto por dentro do rclpy; o cancelamento
        # ja' foi pedido no handler, entao aqui so' registramos a saida.
        interrompido = True
    finally:
        signal.signal(signal.SIGINT, anterior)
        _encerrar()

    if interrompido:
        print("Missao interrompida pelo operador.")
        return 130
    if resposta is None:
        print("Sem resultado do mission_server.", file=sys.stderr)
        return 1
    print(resposta.result.message)
    return 0 if resposta.result.success else 1


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = _parse_args(argv)

    try:
        task_yaml, map_folder, actions_yaml, linhas = mc.prepare_mission(
            task=args.task, map_name=args.map_name, map_dir=args.map_dir,
            on_progress=print)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 1

    print(f"Mapa:    {map_folder}")
    print(f"Tarefa:  {task_yaml}")

    print(f"\n=== Plano da missao ({actions_yaml}) — {len(linhas)} acoes ===")
    for linha in linhas:
        print(linha)
    print()

    task_id = mc.task_id_of(task_yaml)

    if args.dry_run:
        cmd = mc.executor_cmd(
            actions_yaml, map_folder,
            dry_run=True, simulate_nav=args.simulate_nav,
            finish_dock_id=args.finish, use_lidar_refine=args.use_lidar_refine,
            task_id=task_id)
        return mc.run_executor(cmd, on_progress=print)

    if not args.yes:
        resposta = input("Executar a missao? [s/N] ").strip().lower()
        if resposta not in ("s", "sim"):
            print("Missao cancelada pelo operador.")
            return 0

    if not args.local:
        codigo = _executar_no_servidor(args, actions_yaml, map_folder)
        if codigo is not None:
            return codigo
        print(f"mission_server nao encontrado em {ACTION_NAME}; executando localmente.")

    return _executar_local(args, actions_yaml, map_folder, task_id)


if __name__ == "__main__":
    raise SystemExit(main())
