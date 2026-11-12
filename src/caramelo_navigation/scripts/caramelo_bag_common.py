#!/usr/bin/env python3
"""Rosbag por corrida de missao (2026-11-11): lista de topicos, comando do
`ros2 bag record`, politica de disco e parada limpa do gravador.

Quem usa: run_mission.py (terminal) e mission_server.py (GUI / painel web)
chamam iniciar_bag() depois do pre-flight e parar_bag() num `finally`;
gravar_bag_tour.sh chama o modo CLI deste arquivo (`python3
caramelo_bag_common.py --task tour ...`) para gravar um tour ate o Ctrl-C.
Como roda: importado de lib/caramelo_navigation (install(FILES)); opt-out por
CARAMELO_BAG=0; pasta por CARAMELO_BAG_DIR (default ~/bags). Testado em
test/test_bag_common.py sem ROS e em test/test_bag_common_ros.py (fumaca,
ROS_DOMAIN_ID=99, so com CARAMELO_BAG_SMOKE=1).

Desenho (relatorio nav/rosbag.md, 2026-11-10): mcap + zstd_fast por chunk
(compressao incremental de menor custo), split de 500 MB, cache de 16 MB
(perde no maximo ~10 s numa queda) e --include-hidden-topics para o
feedback/status das actions. Desde 2026-11-11 (pedido do operador: historico
COMPLETO do robo — manipulacao, visao, controle, missao) o default e' gravar
TODOS os topicos (--all-topics) menos a lista negra EXCLUIR_REGEX (imagens
cruas, nuvens, costmap_raw/updates, visualizacao do DWB/MPPI); a lista
explicita TOPICOS_MIN continua existindo como modo enxuto (CARAMELO_BAG_TUDO=0). O recorder adota
o QoS que o publisher oferece: NAO se passa --qos-profile-overrides-path.
"""
from __future__ import annotations

import argparse
import datetime
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Optional, Sequence

# ------------------------------------------------------------------ constantes
BAG_DIR_DEFAULT = Path.home() / "bags"
LIMITE_TOTAL_GB = 40      # apaga as corridas mais antigas acima disto
MIN_LIVRE_GB = 10         # abaixo disto nao grava (a missao segue sem bag)
MAX_CORRIDAS = 40         # idem, por quantidade de pastas em ~/bags
MAX_BAG_SIZE = 524288000  # 500 MB por arquivo .mcap (split)
MAX_CACHE_SIZE = 16777216  # 16 MB (double buffer => ate 32 MB em RAM)
NODE_NAME = "caramelo_bag_recorder"
STORAGE_ID = "mcap"
STORAGE_PRESET = "zstd_fast"
ESPERA_PARADA_S = 30.0    # SIGINT -> fecha o metadata.yaml; depois SIGTERM
ESPERA_TERM_S = 10.0
ESPERA_DESCOBERTA_S = 2.0  # o record descobre os topicos antes do executor

# Lista MINIMA: cadeia de comando, estado, LiDAR normalizado, TF, plano local,
# costmap local e o que as actions reportam. ~60 MB/min brutos (estimativa).
TOPICOS_MIN: List[str] = [
    "/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel",
    "/mecanum_controller/reference", "/maxon/wheel_velocity",
    "/odom", "/odom/wheel", "/imu/data_raw", "/joint_states", "/amcl_pose",
    "/scan", "/tf", "/tf_static",
    "/plan", "/local_plan", "/local_costmap/costmap",
    "/local_costmap/published_footprint",
    "/collision_monitor_state", "/rosout", "/caramelo/mission/status",
    "/navigate_to_pose/_action/feedback", "/navigate_to_pose/_action/status",
    "/follow_path/_action/feedback", "/follow_path/_action/status",
    "/dock_robot/_action/feedback", "/dock_robot/_action/status",
    "/align_to_dock/_action/feedback", "/align_to_dock/_action/status",
]

# EXTRA (diagnostico pesado, --bag-extra): costmap global, particulas do AMCL,
# planos transformados, LiDAR cru de 360 graus e logs da BT. Dobra a banda.
TOPICOS_EXTRA: List[str] = [
    "/global_costmap/costmap", "/particle_cloud",
    "/transformed_global_plan", "/received_global_plan",
    "/scan_raw", "/behavior_tree_log", "/diagnostics", "/speed_limit",
]

# NUNCA: o Nav2 so CONSTROI estas mensagens quando ha assinante; o recorder
# viraria esse assinante dentro do laco de 20 Hz do controller (DWB monta
# 1680 trajetorias por ciclo para /marker e /evaluation; MPPI idem para
# /trajectories). */costmap_raw e */costmap_updates ficam vazios com
# always_send_full_costmap true. Imagem crua da camera = 276 MB/min.
TOPICOS_NUNCA: List[str] = [
    "/marker", "/evaluation", "/cost_cloud",
    "/trajectories", "/optimal_trajectory",
    "/local_costmap/costmap_raw", "/local_costmap/costmap_updates",
    "/local_costmap/costmap_raw_updates",
    "/global_costmap/costmap_raw", "/global_costmap/costmap_updates",
    "/global_costmap/costmap_raw_updates",
    "/camera/camera/color/image_raw", "/camera/camera/depth/image_rect_raw",
    "/manip/vision_debug/image",
]

# Camera do punho ANOTADA, so sob demanda (--bag-camera): um republish
# raw -> compressed (JPEG) e o record assina o comprimido. Assinar a imagem
# acorda o processamento do vision_debug_node (~10 ms/quadro).
TOPICO_CAMERA_RAW = "/manip/vision_debug/image"
TOPICO_CAMERA_COMPRIMIDA = "/manip/vision_debug/image/compressed"

# Modo TUDO (default desde 2026-11-11): --all-topics + esta regex de exclusao.
# Fora: imagens cruas e profundidade da RealSense (14 MB/s a 15 fps), nuvens,
# camadas internas dos costmaps (costmap_raw/updates: o /local_costmap/costmap
# ja' basta), visualizacao do DWB/MPPI (/marker, /evaluation, /cost_cloud,
# /trajectories), a imagem anotada CRUA da visao (a comprimida entra com
# --bag-camera), theora/compressedDepth e ruido de infra. Tudo o mais entra:
# juntas do braco, garra, MoveIt, actions de pick/place/dock/align, tags,
# deteccoes, fala, status da missao, diagnosticos.
# 2026-11-11 (bag 4, 962 MB em 38 min): tambem fora /bond (117k msgs), as
# estatisticas/introspecao do controller_manager (5 x 53k msgs), as camadas
# internas *_raw/obstacle_layer/static_layer dos costmaps (o /costmap ja' tem
# a soma), dynamic_joint_states (= joint_states), monitored_planning_scene
# e display_planned_path do MoveIt (pesados; o execute_trajectory e os
# controller_state ja' contam a historia) e marcadores de visualizacao.
EXCLUIR_REGEX = (
    r"(^/camera/|image_raw|/depth/|/points|costmap_raw|costmap_updates|/marker$"
    r"|/evaluation|/cost_cloud|/trajectories|/optimal_trajectory|/theora"
    r"|/compressedDepth|/vision_debug/image$|/parameter_events$|/rosout_agg$"
    r"|^/bond$|^/controller_manager/|_layer_raw$|/obstacle_layer$|/static_layer$"
    r"|/dynamic_joint_states$|/monitored_planning_scene$|/display_planned_path$"
    r"|/text_marker$|/collision_points_marker$|/service_areas/markers$)"
)
CAMERA_JPEG_QUALIDADE = 70

_NOME_RE = re.compile(r"[^A-Za-z0-9_.-]+")


@dataclass
class BagCorrida:
    """O que iniciar_bag() devolve e parar_bag() consome."""
    saida: Path                 # pasta do bag (criada pelo proprio record)
    procs: List[subprocess.Popen] = field(default_factory=list)
    log: Optional[Path] = None  # stdout/stderr do record (movido para a pasta)
    comando: List[str] = field(default_factory=list)


# ---------------------------------------------------------------- helpers puros
def bag_habilitado(env=None) -> bool:
    """CARAMELO_BAG=0 desliga a gravacao em todos os caminhos."""
    env = os.environ if env is None else env
    return str(env.get("CARAMELO_BAG", "1")).strip().lower() not in ("0", "false", "nao", "no", "off")


def bag_tudo_habilitado(env=None) -> bool:
    """CARAMELO_BAG_TUDO=0 volta ao modo enxuto (lista TOPICOS_MIN)."""
    env = os.environ if env is None else env
    return str(env.get("CARAMELO_BAG_TUDO", "1")).strip().lower() not in ("0", "false", "nao", "no", "off")


def bag_dir_padrao(env=None) -> Path:
    env = os.environ if env is None else env
    return Path(env.get("CARAMELO_BAG_DIR") or BAG_DIR_DEFAULT).expanduser()


def sanitizar(texto: str) -> str:
    """So [A-Za-z0-9_.-] no nome da pasta."""
    return _NOME_RE.sub("_", str(texto or "")).strip("_") or "x"


def nome_corrida(mission_id: str, task: str, mapa: str) -> str:
    """'<stamp>_<task>_<mapa>'. mission_id = stem do actions.yaml
    ('20261111_153000_actions' -> '20261111_153000')."""
    mid = str(mission_id or "")
    if mid.endswith("_actions"):
        mid = mid[: -len("_actions")]
    return "_".join(sanitizar(p) for p in (mid, task, mapa))


def montar_topicos(extra: bool = False, camera: bool = False,
                   topicos: Optional[Sequence[str]] = None) -> List[str]:
    """Lista final de topicos, sem duplicatas e sem nada da TOPICOS_NUNCA."""
    base = list(topicos) if topicos is not None else list(TOPICOS_MIN)
    if extra:
        base += TOPICOS_EXTRA
    if camera:
        base.append(TOPICO_CAMERA_COMPRIMIDA)
    vistos, saida = set(), []
    for t in base:
        if t in TOPICOS_NUNCA:
            raise ValueError(f"topico proibido na gravacao: {t}")
        if t not in vistos:
            vistos.add(t)
            saida.append(t)
    return saida


def montar_comando(saida: Path, topicos: Sequence[str],
                   custom_data: Optional[dict] = None, tudo: bool = False) -> List[str]:
    """`ros2 bag record ...` com as flags do rosbag2 0.26.11 (Jazzy).
    tudo=True: --all-topics + --exclude-regex EXCLUIR_REGEX (a lista `topicos`
    e' ignorada); tudo=False: --topics explicitos (ultimo bloco)."""
    cmd = [
        "ros2", "bag", "record",
        "--storage", STORAGE_ID,
        "--storage-preset-profile", STORAGE_PRESET,
        "--max-bag-size", str(MAX_BAG_SIZE),
        "--max-cache-size", str(MAX_CACHE_SIZE),
        "--include-hidden-topics",
        "--node-name", NODE_NAME,
        "--output", str(saida),
    ]
    if custom_data:
        cmd.append("--custom-data")
        cmd += [f"{k}={v}" for k, v in custom_data.items() if v not in (None, "")]
    if tudo:
        cmd += ["--all-topics", "--exclude-regex", EXCLUIR_REGEX]
    else:
        cmd += ["--topics", *topicos]
    return cmd


def comando_republish() -> List[str]:
    """republish raw -> compressed da camera anotada (image_transport Jazzy:
    parametros in_transport/out_transport, topicos in/out). O nome do
    parametro de qualidade segue o compressed_image_transport
    (<base>.compressed.jpeg_quality); conferir com `ros2 param list` se a
    qualidade nao for aplicada."""
    return [
        "ros2", "run", "image_transport", "republish", "--ros-args",
        "-r", "__node:=caramelo_bag_camera_republish",
        "-p", "in_transport:=raw", "-p", "out_transport:=compressed",
        "-r", f"in:={TOPICO_CAMERA_RAW}",
        "-r", f"out/compressed:={TOPICO_CAMERA_COMPRIMIDA}",
        "-p", f"out.compressed.jpeg_quality:={CAMERA_JPEG_QUALIDADE}",
    ]


def raiz_workspace() -> Optional[Path]:
    """Pasta com .git acima deste arquivo (symlink-install -> src/), senao
    ~/caramelo_ws; None se nenhuma tiver .git."""
    candidatos = list(Path(__file__).resolve().parents) + [Path.home() / "caramelo_ws"]
    for p in candidatos:
        if (p / ".git").exists():
            return p
    return None


def git_rev() -> str:
    raiz = raiz_workspace()
    if raiz is None:
        return "desconhecido"
    try:
        out = subprocess.run(["git", "-C", str(raiz), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=5.0)
        return out.stdout.strip() or "desconhecido"
    except (OSError, subprocess.TimeoutExpired):
        return "desconhecido"


def nav_profile_padrao(nav_profile: Optional[str], env=None) -> str:
    """Perfil Nav2 (dwb|mppi) so como metadado: o launch nao o expoe no grafo,
    entao vem do argumento ou de CARAMELO_NAV_PROFILE."""
    env = os.environ if env is None else env
    return str(nav_profile or env.get("CARAMELO_NAV_PROFILE") or "desconhecido")


def metadados_corrida(mission_id, task, mapa, actions_yaml, nav_profile=None,
                      extra=False, camera=False, topicos=None) -> dict:
    return {
        "mission_id": str(mission_id),
        "task": str(task),
        "mapa": str(mapa),
        "actions_yaml": str(actions_yaml),
        "nav_profile": nav_profile_padrao(nav_profile),
        "git_rev": git_rev(),
        "hora": datetime.datetime.now().isoformat(timespec="seconds"),
        "hostname": socket.gethostname(),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        "extra": bool(extra),
        "camera": bool(camera),
        "topicos": list(topicos or []),
    }


def escrever_corrida_yaml(saida: Path, meta: dict) -> Path:
    """corrida.yaml ao lado do metadata.yaml; escrito a mao (sem PyYAML) para
    nao depender de nada alem da stdlib."""
    saida.mkdir(parents=True, exist_ok=True)
    arq = saida / "corrida.yaml"
    linhas = ["# Metadados da corrida (caramelo_bag_common.py)"]
    for k, v in meta.items():
        if isinstance(v, (list, tuple)):
            linhas.append(f"{k}:")
            linhas += [f"  - {item}" for item in v]
        elif isinstance(v, bool):
            linhas.append(f"{k}: {'true' if v else 'false'}")
        else:
            texto = str(v).replace("'", "''")
            linhas.append(f"{k}: '{texto}'")
    arq.write_text("\n".join(linhas) + "\n", encoding="utf-8")
    return arq


# ------------------------------------------------------------------- disco
def tamanho_pasta(p: Path) -> int:
    total = 0
    for raiz, _dirs, arquivos in os.walk(p):
        for a in arquivos:
            try:
                total += (Path(raiz) / a).stat().st_size
            except OSError:
                pass
    return total


def listar_corridas(bag_dir: Path) -> List[Path]:
    """Subpastas de bag_dir (uma por corrida), da mais antiga para a mais nova."""
    if not bag_dir.is_dir():
        return []
    pastas = [p for p in bag_dir.iterdir() if p.is_dir()]
    return sorted(pastas, key=lambda p: p.stat().st_mtime)


def limpar_bags_antigas(bag_dir: Optional[Path] = None,
                        limite_bytes: int = LIMITE_TOTAL_GB * 2**30,
                        max_corridas: int = MAX_CORRIDAS,
                        preservar: Iterable[Path] = ()) -> List[Path]:
    """Apaga as corridas mais antigas enquanto (soma > limite) ou (n > max).
    Pastas com arquivo .keep e as de `preservar` nunca saem. Devolve o que
    apagou. Nunca levanta: limpeza e' melhor esforco."""
    bag_dir = bag_dir_padrao() if bag_dir is None else Path(bag_dir)
    intocaveis = {Path(p).resolve() for p in preservar}
    apagadas: List[Path] = []
    try:
        pastas = listar_corridas(bag_dir)
        tamanhos = {p: tamanho_pasta(p) for p in pastas}
        total = sum(tamanhos.values())
        n = len(pastas)
        for p in pastas:
            if total <= limite_bytes and n <= max_corridas:
                break
            if (p / ".keep").exists() or p.resolve() in intocaveis:
                continue
            try:
                shutil.rmtree(p)
            except OSError as exc:
                print(f"[bag] nao consegui apagar {p}: {exc}", file=sys.stderr)
                continue
            total -= tamanhos[p]
            n -= 1
            apagadas.append(p)
    except OSError as exc:
        print(f"[bag] limpeza de {bag_dir} falhou: {exc}", file=sys.stderr)
    return apagadas


def espaco_livre_bytes(bag_dir: Path) -> int:
    alvo = bag_dir if bag_dir.exists() else bag_dir.parent
    return shutil.disk_usage(alvo).free


# ----------------------------------------------------------- iniciar / parar
def iniciar_bag(mission_id, task, mapa, actions_yaml, extra: bool = False,
                camera: bool = False, nav_profile: Optional[str] = None,
                bag_dir: Optional[Path] = None,
                topicos: Optional[Sequence[str]] = None,
                espera_descoberta_s: float = ESPERA_DESCOBERTA_S,
                tudo: Optional[bool] = None) -> Optional[BagCorrida]:
    """Sobe o `ros2 bag record` da corrida e devolve BagCorrida (ou None se a
    gravacao esta desligada / sem disco / falhou). NUNCA levanta: a missao
    anda com ou sem bag. O processo nasce em sessao propria (Ctrl-C do
    terminal NAO chega nele): quem o para e' parar_bag()."""
    if not bag_habilitado():
        print("[bag] CARAMELO_BAG=0: corrida sem rosbag.")
        return None
    try:
        bag_dir = bag_dir_padrao() if bag_dir is None else Path(bag_dir)
        bag_dir.mkdir(parents=True, exist_ok=True)
        limpar_bags_antigas(bag_dir)
        livre = espaco_livre_bytes(bag_dir)
        if livre < MIN_LIVRE_GB * 2**30:
            print(f"[bag] so {livre / 2**30:.1f} GB livres em {bag_dir} "
                  f"(minimo {MIN_LIVRE_GB} GB): corrida sem rosbag.", file=sys.stderr)
            return None

        tudo = bag_tudo_habilitado() if tudo is None else bool(tudo)
        lista = montar_topicos(extra=extra, camera=camera, topicos=topicos)
        if tudo:  # metadado legivel; o record recebe --all-topics + regex
            lista = ["--all-topics", f"exclude-regex: {EXCLUIR_REGEX}"]
        nome = nome_corrida(mission_id, task, mapa)
        saida = bag_dir / nome
        if saida.exists():  # o record recusa pasta existente
            saida = bag_dir / f"{nome}_{int(time.time()) % 100000}"
        meta = metadados_corrida(mission_id, task, mapa, actions_yaml, nav_profile,
                                 extra, camera, lista)
        cmd = montar_comando(saida, lista, {
            "task": meta["task"], "map": meta["mapa"], "nav_profile": meta["nav_profile"],
            "git": meta["git_rev"], "actions": meta["actions_yaml"]}, tudo=tudo)

        # Log do record fora da pasta (que ainda nao existe); parar_bag() move.
        log = bag_dir / f"{nome}.record.log"
        corrida = BagCorrida(saida=saida, log=log, comando=cmd)
        arq_log = open(log, "w", encoding="utf-8")  # noqa: SIM115 - herdado pelos filhos
        try:
            if camera:
                corrida.procs.append(subprocess.Popen(
                    comando_republish(), stdout=arq_log, stderr=subprocess.STDOUT,
                    start_new_session=True))
            corrida.procs.append(subprocess.Popen(
                cmd, stdout=arq_log, stderr=subprocess.STDOUT, start_new_session=True))
        finally:
            arq_log.close()

        # Espera a pasta aparecer (o record a cria ao abrir o bag) e deixa a
        # descoberta inicial de topicos acontecer antes do executor partir.
        # Ctrl-C aqui (KeyboardInterrupt nao e' Exception): fecha o record
        # antes de propagar, senao ele ficaria orfao na sessao propria.
        rec = corrida.procs[-1]
        try:
            fim = time.monotonic() + max(espera_descoberta_s, 0.0) + 8.0
            while time.monotonic() < fim and not saida.exists() and rec.poll() is None:
                time.sleep(0.1)
            if rec.poll() is not None:
                print(f"[bag] ros2 bag record saiu com {rec.returncode} antes de abrir o bag; "
                      f"veja {log}. Corrida sem rosbag.", file=sys.stderr)
                parar_bag(corrida)
                return None
            time.sleep(max(espera_descoberta_s, 0.0))
        except KeyboardInterrupt:
            parar_bag(corrida)
            raise
        escrever_corrida_yaml(saida, meta)
        print(f"[bag] gravando em {saida} ("
              f"{'TODOS os topicos menos a lista negra' if tudo else f'{len(lista)} topicos'}"
              f"{', +extra' if extra else ''}{', +camera' if camera else ''}; "
              f"pid {rec.pid})")
        return corrida
    except Exception as exc:  # noqa: BLE001 - o bag nunca derruba a missao
        print(f"[bag] nao consegui iniciar a gravacao: {exc}", file=sys.stderr)
        return None


def _procs_de(corrida) -> List:
    if corrida is None:
        return []
    if isinstance(corrida, BagCorrida):
        return list(corrida.procs)
    return list(corrida)


def parar_bag(corrida, espera_s: float = ESPERA_PARADA_S,
              espera_term_s: float = ESPERA_TERM_S) -> Optional[Path]:
    """SIGINT no record (e no republish), espera ate `espera_s` para ele
    fechar o metadata.yaml; se nao sair, SIGTERM (tambem fecha o bag) e
    espera mais `espera_term_s`. NUNCA SIGKILL (perde cache e metadata).
    Idempotente. Devolve a pasta do bag."""
    procs = _procs_de(corrida)
    for p in reversed(procs):
        if p.poll() is None:
            try:
                p.send_signal(signal.SIGINT)
            except OSError:
                pass
    prazo = time.monotonic() + max(espera_s, 0.0)
    for p in procs:
        restante = max(prazo - time.monotonic(), 0.0)
        try:
            p.wait(timeout=restante)
        except subprocess.TimeoutExpired:
            print(f"[bag] pid {p.pid} nao saiu em {espera_s:.0f} s apos SIGINT; SIGTERM.",
                  file=sys.stderr)
            try:
                p.terminate()
                p.wait(timeout=max(espera_term_s, 0.0))
            except subprocess.TimeoutExpired:
                print(f"[bag] pid {p.pid} segue vivo apos SIGTERM; NAO uso SIGKILL "
                      "(feche-o a mao com kill -INT <pid>).", file=sys.stderr)
            except OSError:
                pass
    if not isinstance(corrida, BagCorrida):
        return None
    if corrida.log and corrida.log.exists() and corrida.saida.is_dir():
        try:
            shutil.move(str(corrida.log), str(corrida.saida / "record.log"))
        except OSError:
            pass
    return corrida.saida


def imprimir_info_bag(saida: Optional[Path], timeout_s: float = 30.0) -> None:
    """Caminho do bag e `ros2 bag info` (so le o metadata.yaml, nao toca o
    grafo). Nunca levanta."""
    if saida is None:
        return
    print(f"[bag] corrida gravada em {saida}")
    print(f"[bag] ler: ros2 bag info {saida}   |   "
          f"ros2 run caramelo_navigation bag_resumo {saida}")
    if not (Path(saida) / "metadata.yaml").exists():
        print(f"[bag] AVISO: sem metadata.yaml em {saida} (gravador nao fechou?). "
              f"Tente: ros2 bag reindex {saida} {STORAGE_ID}", file=sys.stderr)
        return
    try:
        out = subprocess.run(["ros2", "bag", "info", str(saida)], capture_output=True,
                             text=True, timeout=timeout_s)
        print((out.stdout or out.stderr).rstrip())
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"[bag] ros2 bag info falhou: {exc}", file=sys.stderr)


# --------------------------------------------------------------------- CLI
def _parse_args(argv):
    p = argparse.ArgumentParser(
        description="Grava um rosbag no padrao das corridas ate o Ctrl-C "
                    "(usado pelo gravar_bag_tour).")
    p.add_argument("--task", default="tour", help="rotulo (ex.: tour, ws6_3x)")
    p.add_argument("--mapa", default="", help="nome do mapa (so metadado)")
    p.add_argument("--extra", action="store_true", help="inclui TOPICOS_EXTRA")
    p.add_argument("--camera", action="store_true",
                   help="camera do punho anotada via republish comprimido")
    p.add_argument("--nav-profile", default=None, help="dwb|mppi (so metadado)")
    p.add_argument("--bag-dir", default=None, help="pasta raiz (default ~/bags)")
    return p.parse_args(argv[1:])


def main(argv=None) -> int:
    args = _parse_args(sys.argv if argv is None else argv)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    corrida = iniciar_bag(stamp, args.task, args.mapa or "sem_mapa", "",
                          extra=args.extra, camera=args.camera,
                          nav_profile=args.nav_profile,
                          bag_dir=Path(args.bag_dir) if args.bag_dir else None)
    if corrida is None:
        return 1
    print("[bag] comando:", " ".join(corrida.comando))
    print("[bag] Ctrl-C para encerrar e fechar o bag.")
    try:
        while corrida.procs[-1].poll() is None:
            time.sleep(0.5)
        print(f"[bag] o record saiu sozinho ({corrida.procs[-1].returncode}); "
              f"veja {corrida.log}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        imprimir_info_bag(parar_bag(corrida))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
