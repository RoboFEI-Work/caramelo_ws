#!/usr/bin/env python3
"""Gravacao completa de uma corrida (2026-11-13): pasta runs/run_<stamp>/ com
video do RViz, rosbag, logs, configuracao, metadata.json e analysis.md.

Pedido do operador: cada corrida grava sozinha video do RViz + bag + logs +
metadados, com relogio comum, para cruzar com o video do celular e achar no
bag/log o momento de um problema. Quem usa: run_mission.py e mission_server.py
(iniciar_run / parar_run no lugar de iniciar_bag / parar_bag) e o CLI
`gravar_run` (corrida manual ou tour, ate o Ctrl-C).

Pasta (raiz em CARAMELO_RUNS_DIR, default <workspace>/runs, fora do git):
  run_YYYYMMDD_HHMMSS/
    rviz_video.mp4   a janela do RViz que o robot_manipulation.launch.py abre
                     (use_rviz:=true, global_localization.rviz), do jeito que o
                     operador a ve, gravada com ffmpeg x11grab por id de janela
                     (VAAPI da Intel; cai para libx264); hora local HH:MM:SS.mmm
                     carimbada em cada quadro; MP4 fragmentado (abre mesmo se
                     cortado). O gravador NUNCA abre nem fecha RViz.
    rosbag/          caramelo_bag_common.iniciar_bag (mesma lista/politica)
    logs/            trecho de ~/.ros/log escrito durante a corrida, rosout.log
                     (do bag, pela analise), saidas do ffmpeg/viz/analise
    config/          yamls/BTs/launch da navegacao, amcl, mapa, task, actions,
                     git (rev + status + diff), params ao vivo (ros2 param dump)
    metadata.json    escrito no inicio e reescrito no fim (resultado)
    analysis.md      analisar_run.py, em segundo plano (nice 19) depois do fim
Sincronizacao: bag e rosout usam o relogio do PC (Pi via chrony); o video tem
a hora local carimbada e metadata.json guarda o epoch do 1o quadro (o x11grab
carimba com o relogio de parede), entao t_video = epoch - video.inicio_epoch.
Custo medido em 2026-11-13: ffmpeg ~10-15 % de 1 nucleo a 10 fps (de 12); o
no' de diagnostico ~3 %, so' le TF/rosout/plan.
Janela: titulo com CARAMELO_VIDEO_JANELA (default "global_localization.rviz");
sem ela e com um unico RViz aberto, grava esse. Desligar: CARAMELO_VIDEO=0,
CARAMELO_BAG=0 (sem bag), CARAMELO_ANALISE=0 (sem analysis.md automatico).
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import caramelo_bag_common as bagc  # noqa: E402

VIDEO_FPS = 10
JANELA_PADRAO = "global_localization.rviz"  # rviz_config default do bringup da navegacao
VIDEO_MAX_S = 3 * 3600                  # orfao (run_mission morto a SIGKILL) para sozinho
FONTE = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
VAAPI_DEV = "/dev/dri/renderD128"       # Intel; renderD129 e' a NVIDIA (sem VAAPI)
LOG_MAX_BYTES = 20 * 2**20               # por arquivo de ~/.ros/log copiado
NOS_PARAMS = (                           # ros2 param dump no inicio (o que valeu)
    "controller_server", "planner_server", "bt_navigator", "behavior_server",
    "smoother_server", "velocity_smoother", "collision_monitor", "amcl",
    "local_costmap/local_costmap", "global_costmap/global_costmap", "docking_server",
    "dock_align_node", "localization_watchdog", "speed_limit_by_scan", "map_server",
)
_RE_JANELA = re.compile(r'^\s*(0x[0-9a-f]+) "(.*)": \("rviz2" "rviz2"\)')
_RE_INICIO = re.compile(r"Input #0, x11grab.*?start: ([0-9]+\.[0-9]+)", re.S)


def _ligado(var: str, env=None) -> bool:
    env = os.environ if env is None else env
    return str(env.get(var, "1")).strip().lower() not in ("0", "false", "nao", "no", "off")


def runs_dir_padrao(env=None) -> Path:
    env = os.environ if env is None else env
    if env.get("CARAMELO_RUNS_DIR"):
        return Path(env["CARAMELO_RUNS_DIR"]).expanduser()
    return (bagc.raiz_workspace() or Path.home() / "caramelo_ws") / "runs"


def nome_run(agora: Optional[datetime.datetime] = None) -> str:
    return "run_" + (agora or datetime.datetime.now()).strftime("%Y%m%d_%H%M%S")


# ------------------------------------------------------------------ video
def _prefixo() -> Path:
    try:
        from ament_index_python.packages import get_package_prefix
        return Path(get_package_prefix("caramelo_navigation"))
    except Exception:  # noqa: BLE001
        return Path(__file__).resolve().parents[2]


def _executavel(nome: str) -> List[str]:
    """Script do pacote: lib/caramelo_navigation/<nome> instalado, senao o .py de src."""
    for p in (_prefixo() / "lib" / "caramelo_navigation" / nome,
              Path(__file__).resolve().parent / f"{nome}.py"):
        if p.exists():
            return [str(p)] if os.access(p, os.X_OK) else [sys.executable, str(p)]
    return ["ros2", "run", "caramelo_navigation", nome]


def achar_janela(trecho_titulo: str, saida_xwininfo: Optional[str] = None) -> Optional[str]:
    """Id da janela-cliente do rviz2 do robo: a que tem o trecho no titulo
    ('<config>.rviz - RViz'); sem nenhuma assim e com um unico RViz aberto, esse."""
    if saida_xwininfo is None:
        try:
            saida_xwininfo = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True,
                                            text=True, timeout=5.0).stdout
        except (OSError, subprocess.TimeoutExpired):
            return None
    janelas = [m.groups() for m in map(_RE_JANELA.match, saida_xwininfo.splitlines()) if m]
    for wid, titulo in janelas:
        if trecho_titulo in titulo:
            return wid
    return janelas[0][0] if len(janelas) == 1 else None


def meia_noite_epoch(agora: Optional[float] = None) -> int:
    d = datetime.datetime.fromtimestamp(agora or time.time())
    return int(d.replace(hour=0, minute=0, second=0, microsecond=0).timestamp())


def comando_ffmpeg(janela: str, saida: Path, display: str, rotulo: str,
                   meia_noite: int, vaapi: bool, fps: int = VIDEO_FPS) -> List[str]:
    """x11grab da janela; -copyts deixa o pts = epoch do quadro ate o drawtext,
    que escreve a hora local (pts - meia-noite) com milissegundos; depois o
    setpts zera a base do arquivo. MP4 fragmentado: legivel mesmo cortado."""
    texto = (f"drawtext=fontfile={FONTE}:text='%{{pts\\:hms\\:-{meia_noite}}}  {rotulo}'"
             ":x=10:y=8:fontsize=24:fontcolor=yellow:box=1:boxcolor=black@0.7")
    filtros = [texto, "setpts=PTS-STARTPTS", "scale=trunc(iw/2)*2:trunc(ih/2)*2"]
    cmd = ["ffmpeg", "-hide_banner", "-nostdin", "-loglevel", "info", "-y",
           "-f", "x11grab", "-framerate", str(fps), "-window_id", janela, "-i", display,
           "-copyts", "-t", str(VIDEO_MAX_S)]
    if vaapi:
        cmd += ["-vaapi_device", VAAPI_DEV, "-vf", ",".join(filtros + ["format=nv12", "hwupload"]),
                "-c:v", "h264_vaapi", "-qp", "26"]
    else:
        cmd += ["-vf", ",".join(filtros + ["format=yuv420p"]),
                "-c:v", "libx264", "-preset", "ultrafast", "-crf", "28"]
    cmd += ["-g", str(2 * fps), "-movflags", "+frag_keyframe+empty_moov+default_base_moof",
            str(saida)]
    return cmd


def epoch_inicio_video(log_ffmpeg: str) -> Optional[float]:
    """Epoch do 1o quadro: o x11grab carimba com o relogio de parede e o
    ffmpeg imprime 'Input #0, x11grab ... start: <epoch>'."""
    achados = _RE_INICIO.findall(log_ffmpeg or "")
    return float(achados[-1]) if achados else None  # ultima tentativa (VAAPI -> x264)


# ------------------------------------------------------------------ logs
def tamanhos_logs(log_dir: Path) -> Dict[str, int]:
    """Tamanho atual de cada arquivo de ~/.ros/log (para copiar so' o que a
    corrida escreveu)."""
    tam = {}
    for raiz, _d, arqs in os.walk(log_dir):
        for a in arqs:
            p = os.path.join(raiz, a)
            try:
                tam[p] = os.stat(p).st_size
            except OSError:
                pass
    return tam


def copiar_logs_da_corrida(log_dir: Path, tam_inicio: Dict[str, int], inicio_epoch: float,
                           destino: Path, max_bytes: int = LOG_MAX_BYTES) -> List[Path]:
    """Copia, de cada arquivo escrito desde o inicio, so' os bytes novos
    (arquivo novo: inteiro), limitado aos ultimos max_bytes."""
    copiados = []
    for raiz, _d, arqs in os.walk(log_dir):
        for a in arqs:
            src = Path(raiz) / a
            try:
                st = src.stat()
            except OSError:
                continue
            if st.st_mtime < inicio_epoch - 1.0:
                continue
            de = tam_inicio.get(str(src), 0)
            if st.st_size <= de:
                continue
            de = max(de, st.st_size - max_bytes)
            dst = destino / src.relative_to(log_dir)
            dst.parent.mkdir(parents=True, exist_ok=True)
            with open(src, "rb") as f_in, open(dst, "wb") as f_out:
                f_in.seek(de)
                shutil.copyfileobj(f_in, f_out)
            copiados.append(dst)
    return copiados


# ------------------------------------------------------------------ config
def _git(raiz: Path, *args, timeout=10.0) -> str:
    try:
        return subprocess.run(["git", "-C", str(raiz), *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def foto_config(destino: Path, task_yaml=None, actions_yaml=None, map_folder=None) -> dict:
    """Copia a configuracao da navegacao e o estado do git. Devolve o bloco
    'git' do metadata.json."""
    destino.mkdir(parents=True, exist_ok=True)
    raiz = bagc.raiz_workspace()
    if raiz is not None:
        for rel in ("src/caramelo_navigation/config", "src/caramelo_navigation/behavior_tree",
                    "src/caramelo_navigation/launch", "src/caramelo_localization/config"):
            if (raiz / rel).is_dir():
                shutil.copytree(raiz / rel, destino / rel.split("/", 1)[1], dirs_exist_ok=True,
                                ignore=shutil.ignore_patterns("__pycache__", "*.bak"))
    if map_folder and Path(map_folder).is_dir():
        for y in Path(map_folder).glob("*.yaml"):
            (destino / "mapa").mkdir(exist_ok=True)
            shutil.copy2(y, destino / "mapa" / y.name)
    for f in (task_yaml, actions_yaml):
        if f and Path(f).is_file():
            shutil.copy2(f, destino / Path(f).name)
    git = {"rev": "desconhecido", "branch": "", "sujo": None, "alterados": []}
    if raiz is not None:
        status = _git(raiz, "status", "--short")
        git = {"rev": _git(raiz, "rev-parse", "--short", "HEAD").strip() or "desconhecido",
               "branch": _git(raiz, "rev-parse", "--abbrev-ref", "HEAD").strip(),
               "sujo": bool(status.strip()),
               "alterados": [l[3:] for l in status.splitlines() if l.strip()]}
        (destino / "git_status.txt").write_text(
            f"rev {git['rev']}  branch {git['branch']}\n\n{status}", encoding="utf-8")
        (destino / "git_diff.patch").write_text(
            _git(raiz, "diff", "HEAD", "--", "src", "missions", timeout=20.0), encoding="utf-8")
    return git


def _dump_params(destino: Path) -> None:
    """ros2 param dump dos nos da navegacao (o valor que vale, com overrides
    de launch). Melhor esforco, em thread, prioridade baixa."""
    destino.mkdir(parents=True, exist_ok=True)
    try:
        nos = subprocess.run(["nice", "-n", "15", "ros2", "node", "list"], capture_output=True,
                             text=True, timeout=20.0).stdout.split()
    except (OSError, subprocess.TimeoutExpired):
        return
    for no in NOS_PARAMS:
        if "/" + no not in nos:
            continue
        try:
            out = subprocess.run(["nice", "-n", "15", "ros2", "param", "dump", "/" + no],
                                 capture_output=True, text=True, timeout=20.0)
            (destino / (no.replace("/", "__") + ".yaml")).write_text(
                out.stdout or out.stderr, encoding="utf-8")
        except (OSError, subprocess.TimeoutExpired):
            pass


# ------------------------------------------------------------------ corrida
@dataclass
class Run:
    pasta: Path
    inicio_epoch: float
    meta: dict
    bag: Optional[bagc.BagCorrida] = None
    procs: Dict[str, subprocess.Popen] = field(default_factory=dict)
    tam_logs: Dict[str, int] = field(default_factory=dict)
    params: Optional[threading.Thread] = None


def _popen(cmd, log: Path, **kw) -> subprocess.Popen:
    with open(log, "a", encoding="utf-8") as f:  # o filho herda o descritor
        return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True, **kw)


def _parar_proc(p: Optional[subprocess.Popen], espera_s=8.0) -> None:
    """SIGINT, espera, SIGTERM. Nunca SIGKILL (ffmpeg perderia o fim do mp4)."""
    if p is None or p.poll() is not None:
        return
    try:
        p.send_signal(signal.SIGINT)
        p.wait(timeout=espera_s)
    except subprocess.TimeoutExpired:
        try:
            p.terminate()
            p.wait(timeout=espera_s)
        except (subprocess.TimeoutExpired, OSError):
            print(f"[run] pid {p.pid} nao saiu; feche com kill -INT {p.pid}", file=sys.stderr)
    except OSError:
        pass


def escrever_metadata(run: Run) -> None:
    try:
        (run.pasta / "metadata.json").write_text(
            json.dumps(run.meta, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
    except OSError as exc:
        print(f"[run] metadata.json falhou: {exc}", file=sys.stderr)


def _iniciar_video(run: Run, rotulo: str) -> None:
    """Grava a janela do RViz do robo (aberto pelo operador com o launch)."""
    v = run.meta["video"]
    display = os.environ.get("DISPLAY", "")
    logs = run.pasta / "logs"
    if not display:
        v["motivo"] = "sem DISPLAY (sessao remota?)"
        return
    trecho = os.environ.get("CARAMELO_VIDEO_JANELA") or JANELA_PADRAO
    janela = achar_janela(trecho)
    if janela is None:
        v["motivo"] = (f"RViz do robo nao achado (titulo com '{trecho}'): lance o "
                       "robot_manipulation.launch.py com use_rviz:=true")
        return
    saida = run.pasta / "rviz_video.mp4"
    for vaapi in ((True, False) if os.path.exists(VAAPI_DEV) else (False,)):
        cmd = comando_ffmpeg(janela, saida, display, rotulo, meia_noite_epoch(), vaapi)
        p = _popen(cmd, logs / "ffmpeg.log")
        time.sleep(2.5)
        if p.poll() is None:
            run.procs["ffmpeg"] = p
            v.update(ok=True, janela=janela, encoder="h264_vaapi" if vaapi else "libx264",
                     fps=VIDEO_FPS, comando=cmd, motivo="",
                     inicio_epoch=epoch_inicio_video((logs / "ffmpeg.log").read_text(
                         errors="replace")))
            return
    v["motivo"] = "ffmpeg saiu logo ao abrir (ver logs/ffmpeg.log)"


def iniciar_run(mission_id: str, task: str, mapa: str, actions_yaml=None, task_yaml=None,
                map_folder=None, extra=False, camera=False, nav_profile=None,
                runs_dir: Optional[Path] = None) -> Optional[Run]:
    """Abre a pasta da corrida e sobe bag + no' de diagnostico + RViz + video.
    NUNCA levanta: a missao anda com o que der para gravar."""
    try:
        raiz = Path(runs_dir) if runs_dir else runs_dir_padrao()
        raiz.mkdir(parents=True, exist_ok=True)
        (raiz / "COLCON_IGNORE").touch()  # colcon nao varre os bags
        bagc.limpar_bags_antigas(raiz)
        agora = datetime.datetime.now()
        pasta = raiz / nome_run(agora)
        if pasta.exists():
            pasta = raiz / f"{nome_run(agora)}_{os.getpid()}"
        (pasta / "logs").mkdir(parents=True)
        run = Run(pasta=pasta, inicio_epoch=agora.timestamp(), meta={})
        run.tam_logs = tamanhos_logs(Path.home() / ".ros" / "log")
        run.meta = {
            "run_id": pasta.name, "inicio": agora.isoformat(timespec="milliseconds"),
            "inicio_epoch": agora.timestamp(), "fim": None, "duracao_s": None,
            "task": str(task), "task_yaml": str(task_yaml or ""), "mapa": str(mapa),
            "map_folder": str(map_folder or ""), "mission_id": str(mission_id),
            "actions_yaml": str(actions_yaml or ""),
            "nav_profile": bagc.nav_profile_padrao(nav_profile),
            "host": socket.gethostname(), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "git": {}, "resultado": {"codigo": None, "texto": "em andamento"},
            "bag": {"ok": False, "pasta": "rosbag"},
            "video": {"ok": False, "arquivo": "rviz_video.mp4", "inicio_epoch": None,
                      "motivo": "desligado (CARAMELO_VIDEO=0)"},
            "viz": {"ok": False},
            "sincronizacao": ("bag e rosout: relogio do PC (Pi via chrony); video: hora local "
                              "carimbada no quadro e t_video = epoch - video.inicio_epoch"),
        }
        run.meta["git"] = foto_config(pasta / "config", task_yaml, actions_yaml, map_folder)
        escrever_metadata(run)

        run.procs["viz"] = _popen(_executavel("run_diag_viz"), pasta / "logs" / "run_diag_viz.log")
        run.meta["viz"]["ok"] = True

        run.bag = bagc.iniciar_bag(mission_id, task, mapa, actions_yaml or "", extra=extra,
                                   camera=camera, nav_profile=nav_profile,
                                   saida=pasta / "rosbag")
        run.meta["bag"].update(ok=run.bag is not None,
                               comando=run.bag.comando if run.bag else None)
        if _ligado("CARAMELO_VIDEO"):
            _iniciar_video(run, f"{pasta.name} {task}")
        run.params = threading.Thread(target=_dump_params, args=(pasta / "config" / "params",),
                                      daemon=True, name="param_dump")
        run.params.start()
        escrever_metadata(run)
        v = run.meta["video"]
        print(f"[run] gravando em {pasta} (bag {'ok' if run.bag else 'NAO'}, video "
              f"{('ok, ' + v.get('encoder', '')) if v['ok'] else 'NAO: ' + v['motivo']})")
        if v["ok"]:
            print(f"[run] gravando a janela {v['janela']} do RViz do robo: pode ficar atras de "
                  "outras janelas; NAO minimize nem redimensione ate o fim da corrida.")
        return run
    except Exception as exc:  # noqa: BLE001 - gravar nunca derruba a missao
        print(f"[run] nao consegui iniciar a gravacao da corrida: {exc}", file=sys.stderr)
        return None


def texto_resultado(codigo) -> str:
    return {0: "sucesso", 1: "falha", 130: "abortada (Ctrl-C)", None: "desconhecido"}.get(
        codigo, f"codigo {codigo}")


def parar_run(run: Optional[Run], resultado=None) -> Optional[Path]:
    """Fecha video, RViz, no' de diagnostico e bag; copia os logs; grava o
    metadata.json final e dispara a analise em segundo plano. Idempotente e
    nunca levanta (roda no finally de quem chama)."""
    if run is None or run.meta.get("fim"):
        return None if run is None else run.pasta
    try:
        return _parar_run(run, resultado)
    except Exception as exc:  # noqa: BLE001
        print(f"[run] erro ao fechar a corrida {run.pasta}: {exc}", file=sys.stderr)
        return run.pasta


def _parar_run(run: Run, resultado) -> Path:
    fim = time.time()
    _parar_proc(run.procs.get("ffmpeg"), espera_s=10.0)
    _parar_proc(run.procs.get("viz"), espera_s=6.0)  # o RViz e' do operador: nao mexe
    if run.bag is not None:
        bagc.parar_bag(run.bag)
    try:
        log_ff = (run.pasta / "logs" / "ffmpeg.log").read_text(errors="replace")
    except OSError:
        log_ff = ""
    run.meta["video"]["inicio_epoch"] = epoch_inicio_video(log_ff) or run.meta["video"]["inicio_epoch"]
    try:
        n = len(copiar_logs_da_corrida(Path.home() / ".ros" / "log", run.tam_logs,
                                       run.inicio_epoch, run.pasta / "logs" / "ros_log"))
    except OSError as exc:
        n = f"falhou: {exc}"
    run.meta.update(fim=datetime.datetime.fromtimestamp(fim).isoformat(timespec="milliseconds"),
                    duracao_s=round(fim - run.inicio_epoch, 1), logs_ros_copiados=n)
    run.meta["resultado"] = {"codigo": resultado, "texto": texto_resultado(resultado)}
    escrever_metadata(run)
    print(f"[run] corrida em {run.pasta}")
    if run.bag is not None:
        bagc.imprimir_info_bag(run.bag.saida)
    if _ligado("CARAMELO_ANALISE") and run.bag is not None:
        try:
            _popen(["nice", "-n", "19", *_executavel("analisar_run"), str(run.pasta)],
                   run.pasta / "logs" / "analise.log")
            print(f"[run] relatorio em 1-3 min: {run.pasta / 'analysis.md'}")
        except OSError as exc:
            print(f"[run] analise nao subiu: {exc}", file=sys.stderr)
    return run.pasta


# --------------------------------------------------------------------- CLI
def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Grava uma corrida (video do RViz + bag + logs + "
                                "config + relatorio) ate o Ctrl-C.")
    p.add_argument("--task", default="manual", help="rotulo (ex.: manual, tour, ws6_3x)")
    p.add_argument("--mapa", default="", help="pasta ou nome do mapa (copia o docking.yaml etc.)")
    p.add_argument("--extra", action="store_true", help="bag modo enxuto + TOPICOS_EXTRA")
    p.add_argument("--camera", action="store_true", help="camera do punho anotada no bag")
    p.add_argument("--nav-profile", default=None, help="dwb|mppi (so' metadado)")
    p.add_argument("--bag-dir", "--runs-dir", dest="runs_dir", default=None,
                   help="raiz das corridas (default <workspace>/runs)")
    a = p.parse_args((sys.argv if argv is None else argv)[1:])
    # Ctrl-C e SIGTERM fecham a corrida limpa, mesmo se o shell herdou SIGINT
    # ignorado (processo em segundo plano): o handler Python tambem faz os
    # filhos (ffmpeg, bag, no' de diagnostico) nascerem com SIGINT padrao.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    map_folder = None
    if a.mapa:
        cand = Path(a.mapa).expanduser()
        if not cand.is_dir() and bagc.raiz_workspace():
            cand = bagc.raiz_workspace() / "src/caramelo_mapping/maps" / a.mapa
        map_folder = cand if cand.is_dir() else None
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    run = iniciar_run(stamp, a.task, Path(a.mapa).name if a.mapa else "sem_mapa",
                      map_folder=map_folder, extra=a.extra, camera=a.camera,
                      nav_profile=a.nav_profile, runs_dir=a.runs_dir)
    if run is None:
        return 1
    print("[run] Ctrl-C para encerrar a corrida.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        parar_run(run, resultado=None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
