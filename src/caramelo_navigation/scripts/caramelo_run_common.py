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
    videos/<vista>.mp4  um video por RViz ESCONDIDO (2026-11-13, pedido do
                     operador: a tela do robot_manipulation fica limpa; a analise
                     vem destes). Vistas em rviz/diag_<vista>.rviz: geral,
                     colisao, controlador, global (+expansoes), custos.
                     PADRAO = DEPOIS da corrida: o bag e' reproduzido (--clock,
                     ROS_DOMAIN_ID 88 so' localhost, so' topicos de visualizacao)
                     num X virtual (Xvfb) com os RViz em use_sim_time, nice 19.
                     Ao vivo (CARAMELO_VIDEO_AO_VIVO=1) custou 4,56 nucleos e
                     derrubou o laco do controller a 8-9 Hz na corrida 17h12 de
                     13/11: so' com pedido explicito. Gravacao por id de janela
                     (x11grab + VAAPI; cai para libx264), hora local HH:MM:SS.mmm
                     DA CORRIDA carimbada em cada quadro; MP4 fragmentado. O RViz
                     do operador nunca e' aberto, gravado nem fechado por aqui.
    rosbag/          caramelo_bag_common.iniciar_bag (mesma lista/politica)
    logs/            cpu.csv (CPU por processo a cada 2 s), trecho de ~/.ros/log, rosout.log
                     (do bag, pela analise), saidas do Xvfb/rviz/ffmpeg/viz/analise
    config/          yamls/BTs/launch da navegacao, amcl, mapa, task, actions,
                     git (rev + status + diff), params/ (ros2 param dump DEPOIS
                     do fim: no inicio derrubava o laco do controller a 6-8 Hz)
    metadata.json    escrito no inicio e reescrito no fim (resultado)
    analysis.md      analisar_run.py, em segundo plano (nice 19) depois do fim;
                     um processo so' faz param dump -> analise -> videos -> analise
Sincronizacao: bag e rosout usam o relogio do PC (Pi via chrony); o video tem
a hora local carimbada e metadata.json guarda o epoch do 1o quadro (o x11grab
carimba com o relogio de parede), entao t_video = epoch - video.inicio_epoch.
Custo: metadata.json guarda video.cpu_nucleos (CPU media dos RViz escondidos +
ffmpeg, em nucleos) para decidir quantas vistas cabem. Requer `sudo apt install
xvfb`; sem ele a corrida grava o resto e metadata.json diz por que.
Escolher vistas: CARAMELO_VIDEO_VISTAS="geral,colisao". Desligar: CARAMELO_VIDEO=0,
CARAMELO_BAG=0 (sem bag), CARAMELO_ANALISE=0 (sem analysis.md automatico),
CARAMELO_PARAMS=0 (sem ros2 param dump no fim).
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
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import caramelo_bag_common as bagc  # noqa: E402

VIDEO_FPS = 10
VISTAS = ("geral", "colisao", "controlador", "global", "custos")  # rviz/diag_<vista>.rviz
VISTA_W, VISTA_H = 960, 540              # cada vista; o X virtual tem 3 x 2 lugares
XVFB_TELA = (3 * VISTA_W, 2 * VISTA_H)   # sem gerenciador de janelas: nada se sobrepoe
XVFB_DISPLAY_BASE = 87
DOMINIO_RENDER = "88"                    # replay isolado (so' localhost), nunca o 67 do robo
TOPICOS_RENDER = (                       # so' visualizacao: nada de comando chega a ninguem
    "/map", "/tf", "/tf_static", "/robot_description", "/global_costmap/costmap",
    "/local_costmap/costmap", "/local_costmap/published_footprint", "/plan", "/local_plan",
    "/transformed_global_plan", "/unsmoothed_plan", "/diag/trilha_real", "/diag/plano_anterior",
    "/diag/eventos", "/scan", "/scan_raw", "/collision_monitor/footprint_approach",
    "/docking_trajectory", "/expansions", "/amcl_pose",
)
VIDEO_MAX_S = 3 * 3600                  # orfao (run_mission morto a SIGKILL) para sozinho
FONTE = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
VAAPI_DEV = "/dev/dri/renderD128"       # Intel; renderD129 e' a NVIDIA (sem VAAPI)
LOG_MAX_BYTES = 20 * 2**20               # por arquivo de ~/.ros/log copiado
NOS_PARAMS = (                           # ros2 param dump no fim (o que valeu)
    "controller_server", "planner_server", "bt_navigator", "behavior_server",
    "smoother_server", "velocity_smoother", "collision_monitor", "amcl",
    "local_costmap/local_costmap", "global_costmap/global_costmap", "docking_server",
    "dock_align_node", "localization_watchdog", "speed_limit_by_scan", "map_server",
)
_RE_JANELA = re.compile(r'^\s*(0x[0-9a-f]+) "(.*)": \("rviz2" "rviz2"\)')
_RE_INICIO = re.compile(r"Input #0, x11grab.*?start: ([0-9]+\.[0-9]+)", re.S)


def _ligado(var: str, env=None, padrao: str = "1") -> bool:
    env = os.environ if env is None else env
    return str(env.get(var, padrao)).strip().lower() not in ("0", "false", "nao", "no", "off")


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


def vista_do_mapa(map_folder: Optional[Path], largura=VISTA_W, altura=VISTA_H):
    """(x, y, escala px/m) da vista de cima que enquadra a parte conhecida do
    mapa na janela. Sem mapa legivel: centro (0,0), 60 px/m."""
    try:
        import numpy as np
        import yaml
        meta = yaml.safe_load((Path(map_folder) / "map.yaml").read_text())
        res = float(meta["resolution"])
        ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
        dados = (Path(map_folder) / meta["image"]).read_bytes()
        campos, pos = [], 0
        while len(campos) < 4:  # P5 / largura / altura / maxval, pulando comentarios
            while dados[pos:pos + 1].isspace():
                pos += 1
            if dados[pos:pos + 1] == b"#":
                pos = dados.index(b"\n", pos) + 1
                continue
            fim = pos
            while not dados[fim:fim + 1].isspace():
                fim += 1
            campos.append(dados[pos:fim])
            pos = fim
        w, h = int(campos[1]), int(campos[2])
        img = np.frombuffer(dados[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w)
        lin, col = np.nonzero(img != 205)  # 205 = desconhecido no trinary
        if lin.size == 0:
            lin, col = np.array([0, h - 1]), np.array([0, w - 1])
        c0, c1, l0, l1 = col.min(), col.max() + 1, lin.min(), lin.max() + 1
        x = ox + (c0 + c1) / 2 * res
        y = oy + (h - (l0 + l1) / 2) * res  # linha 0 do PGM = topo do mapa
        vw, vh = largura, max(altura - 60, 100)  # menu + barra de ferramentas
        escala = 0.94 * min(vw / max((c1 - c0) * res, 0.5), vh / max((l1 - l0) * res, 0.5))
        return round(float(x), 3), round(float(y), 3), round(float(escala), 1)
    except Exception:  # noqa: BLE001 - vista padrao basta
        return 0.0, 0.0, 60.0


def escrever_vista(nome: str, destino: Path, slot: int, vista_mapa) -> Path:
    """Copia rviz/diag_<nome>.rviz trocando os @...@: posicao da janela no X
    virtual (slot 0..5 numa grade 3 x 2) e a vista do mapa inteiro."""
    txt = _modelo_rviz(nome).read_text(encoding="utf-8")
    for k, v in (("WIN_X", (slot % 3) * VISTA_W), ("WIN_Y", (slot // 3) * VISTA_H),
                 ("WIDTH", VISTA_W), ("HEIGHT", VISTA_H), ("VIEW_X", vista_mapa[0]),
                 ("VIEW_Y", vista_mapa[1]), ("VIEW_SCALE", vista_mapa[2])):
        txt = txt.replace(f"@{k}@", str(v))
    destino.parent.mkdir(parents=True, exist_ok=True)
    destino.write_text(txt, encoding="utf-8")
    return destino


def _modelo_rviz(nome: str) -> Path:
    arq = f"diag_{nome}.rviz"
    for p in (Path(__file__).resolve().parent.parent / "rviz" / arq,  # src/ (symlink resolvido)
              _prefixo() / "share" / "caramelo_navigation" / "rviz" / arq):
        if p.exists():
            return p
    raise FileNotFoundError(f"{arq} nao encontrado (rebuild do caramelo_navigation?)")


def vistas_escolhidas(env=None) -> List[str]:
    env = os.environ if env is None else env
    pedidas = [v.strip() for v in str(env.get("CARAMELO_VIDEO_VISTAS", "")).split(",") if v.strip()]
    return [v for v in pedidas if v in VISTAS] or list(VISTAS)


def _executavel(nome: str) -> List[str]:
    """Script do pacote: lib/caramelo_navigation/<nome> instalado, senao o .py de src."""
    for p in (_prefixo() / "lib" / "caramelo_navigation" / nome,
              Path(__file__).resolve().parent / f"{nome}.py"):
        if p.exists():
            return [str(p)] if os.access(p, os.X_OK) else [sys.executable, str(p)]
    return ["ros2", "run", "caramelo_navigation", nome]


def achar_janela(trecho_titulo: str, saida_xwininfo: Optional[str] = None,
                 display: Optional[str] = None) -> Optional[str]:
    """Id da janela-cliente do rviz2 cujo titulo ('<config>.rviz - RViz')
    contem o trecho; None se nao houver."""
    if saida_xwininfo is None:
        cmd = ["xwininfo", "-root", "-tree"] + (["-display", display] if display else [])
        try:
            saida_xwininfo = subprocess.run(cmd, capture_output=True, text=True,
                                            timeout=5.0).stdout
        except (OSError, subprocess.TimeoutExpired):
            return None
    for m in map(_RE_JANELA.match, saida_xwininfo.splitlines()):
        if m and trecho_titulo in m.group(2):
            return m.group(1)
    return None


def cpu_s(pid: int) -> float:
    """CPU (usuario + sistema) gasta pelo processo ate agora, em segundos."""
    try:
        campos = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return (int(campos[11]) + int(campos[12])) / os.sysconf("SC_CLK_TCK")
    except (OSError, IndexError, ValueError):
        return 0.0


def meia_noite_epoch(agora: Optional[float] = None) -> int:
    d = datetime.datetime.fromtimestamp(agora or time.time())
    return int(d.replace(hour=0, minute=0, second=0, microsecond=0).timestamp())


def comando_ffmpeg(janela: str, saida: Path, display: str, rotulo: str,
                   offset_s: float, vaapi: bool, fps: int = VIDEO_FPS) -> List[str]:
    """x11grab da janela; -copyts deixa o pts = epoch do quadro ate o drawtext,
    que escreve pts + offset_s como HH:MM:SS.mmm (ao vivo: offset = -meia-noite
    = hora local; no replay: desloca para a hora DA CORRIDA); depois o setpts
    zera a base do arquivo. MP4 fragmentado: legivel mesmo cortado."""
    texto = (f"drawtext=fontfile={FONTE}:text='%{{pts\\:hms\\:{offset_s:.3f}}}  {rotulo}'"
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
    de launch). Melhor esforco, em processo separado (nice 19) DEPOIS do fim."""
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


def _subir_xvfb(run: Run) -> Optional[str]:
    """X virtual so' das vistas escondidas; devolve ':N' ou None (motivo no meta)."""
    v = run.meta["video"]
    if shutil.which("Xvfb") is None:
        v["motivo"] = "Xvfb nao instalado: sudo apt install xvfb (corrida segue sem video)"
        return None
    n = XVFB_DISPLAY_BASE
    while Path(f"/tmp/.X11-unix/X{n}").exists() or Path(f"/tmp/.X{n}-lock").exists():
        n += 1
    tela = f"{XVFB_TELA[0]}x{XVFB_TELA[1]}x24"
    run.procs["xvfb"] = _popen(["Xvfb", f":{n}", "-screen", "0", tela, "-nolisten", "tcp",
                                "-nocursor"], run.pasta / "logs" / "xvfb.log")
    fim = time.monotonic() + 5.0
    while time.monotonic() < fim and not Path(f"/tmp/.X11-unix/X{n}").exists():
        time.sleep(0.1)
    if run.procs["xvfb"].poll() is not None or not Path(f"/tmp/.X11-unix/X{n}").exists():
        v["motivo"] = "Xvfb nao subiu (ver logs/xvfb.log)"
        return None
    return f":{n}"


def _subir_vistas(run: Run, display: str, map_folder, env_extra=None, args_extra=(),
                  nice: str = "15") -> None:
    """Um rviz2 por vista no X virtual (GL por software, 1 thread)."""
    env = dict(os.environ, DISPLAY=display, LIBGL_ALWAYS_SOFTWARE="1", LP_NUM_THREADS="1",
               **(env_extra or {}))
    vista_mapa = vista_do_mapa(map_folder)
    for slot, nome in enumerate(vistas_escolhidas()):
        cfg = escrever_vista(nome, run.pasta / "config" / "rviz" / f"diag_{nome}.rviz", slot,
                             vista_mapa)
        run.procs[f"rviz_{nome}"] = _popen(
            ["nice", "-n", nice, "rviz2", "-d", str(cfg), "--ros-args",
             "-r", f"__node:=rviz_diag_{nome}", *args_extra],
            run.pasta / "logs" / f"rviz_{nome}.log", env=env)
        run.meta["video"]["vistas"][nome] = {"ok": False, "arquivo": f"videos/{nome}.mp4",
                                             "config": f"config/rviz/diag_{nome}.rviz"}


def _gravar_vistas(run: Run, display: str, rotulo: str, offset_s: Optional[float] = None,
                   nice: str = "15") -> None:
    """Acha a janela de cada vista e sobe um ffmpeg por vista (VAAPI, senao x264)."""
    offset_s = -meia_noite_epoch() if offset_s is None else offset_s
    v = run.meta["video"]
    logs, saida_dir = run.pasta / "logs", run.pasta / "videos"
    saida_dir.mkdir(exist_ok=True)
    pendentes, fim = dict.fromkeys(v["vistas"]), time.monotonic() + 30.0
    while pendentes and time.monotonic() < fim:
        time.sleep(0.5)
        for nome in list(pendentes):
            janela = achar_janela(f"diag_{nome}.rviz", display=display)
            if janela:
                v["vistas"][nome]["janela"] = janela
                del pendentes[nome]
    time.sleep(1.5)  # deixa cada RViz desenhar o 1o quadro com o mapa
    encoders = (True, False) if os.path.exists(VAAPI_DEV) else (False,)
    for nome, info in v["vistas"].items():
        if "janela" not in info:
            info["motivo"] = "janela nao apareceu em 30 s (ver logs/rviz_*.log)"
            continue
        for vaapi in encoders:
            cmd = comando_ffmpeg(info["janela"], saida_dir / f"{nome}.mp4", display,
                                 f"{rotulo} [{nome}]", offset_s, vaapi)
            log = logs / f"ffmpeg_{nome}.log"
            proc = _popen(["nice", "-n", nice, *cmd], log)
            time.sleep(1.0)
            if proc.poll() is None:
                run.procs[f"ffmpeg_{nome}"] = proc
                info.update(ok=True, encoder="h264_vaapi" if vaapi else "libx264",
                            inicio_epoch=epoch_inicio_video(log.read_text(errors="replace")))
                encoders = (vaapi,)  # o que funcionou vale para as outras vistas
                break
        else:
            info["motivo"] = "ffmpeg saiu logo ao abrir (ver logs/ffmpeg_*.log)"
    oks = [i for i in v["vistas"].values() if i.get("ok")]
    v.update(ok=bool(oks), display=display, fps=VIDEO_FPS,
             inicio_epoch=min((i["inicio_epoch"] for i in oks if i.get("inicio_epoch")),
                              default=None),
             motivo="" if oks else "nenhuma vista gravando (ver logs/)")


def _parar_procs(procs: Dict[str, subprocess.Popen]) -> None:
    for k in [k for k in procs if k.startswith("ffmpeg_")]:
        _parar_proc(procs[k], espera_s=10.0)
    for k in [k for k in procs if k.startswith("rviz_")] + ["play", "viz", "cpu", "xvfb"]:
        _parar_proc(procs.get(k), espera_s=6.0)


def _retomar_play(prazo_epoch: float) -> bool:
    """Chama /rosbag2_player/resume no instante combinado (domínio do replay)."""
    import rclpy
    from rosbag2_interfaces.srv import Resume
    rclpy.init()
    no = rclpy.create_node("caramelo_render_resume")
    try:
        cli = no.create_client(Resume, "/rosbag2_player/resume")
        if not cli.wait_for_service(timeout_sec=15.0):
            return False
        time.sleep(max(prazo_epoch - time.time(), 0.0))
        fut = cli.call_async(Resume.Request())
        rclpy.spin_until_future_complete(no, fut, timeout_sec=5.0)
        return fut.done()
    finally:
        no.destroy_node()
        rclpy.shutdown()


def renderizar_offline(pasta: Path, inicio_s: float = 0.0, max_s: Optional[float] = None) -> dict:
    """Videos das vistas a partir do bag, DEPOIS da corrida (custo zero durante a
    navegacao). Replay em ROS_DOMAIN_ID 88 so' localhost e so' TOPICOS_RENDER.
    Atualiza metadata.json (video.*) e devolve o bloco video."""
    pasta = Path(pasta)
    meta = json.loads((pasta / "metadata.json").read_text())
    run = Run(pasta=pasta, inicio_epoch=meta.get("inicio_epoch", time.time()), meta=meta)
    v = meta["video"] = {"ok": False, "pasta": "videos", "inicio_epoch": None, "vistas": {},
                         "modo": "replay do bag depois da corrida", "motivo": ""}
    bag = pasta / "rosbag"
    try:
        import yaml
        info = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
        t0 = info["starting_time"]["nanoseconds_since_epoch"] / 1e9 + inicio_s
        dur = info["duration"]["nanoseconds"] / 1e9 - inicio_s
    except (OSError, KeyError, TypeError) as exc:
        v["motivo"] = f"sem bag legivel: {exc}"
        escrever_metadata(run)
        return v
    env = {"ROS_DOMAIN_ID": DOMINIO_RENDER, "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}
    os.environ.update(env)  # o resume (rclpy) deste processo tambem fica isolado
    try:
        display = _subir_xvfb(run)
        if not display:
            return v
        _subir_vistas(run, display, meta.get("map_folder") or None, env_extra=env,
                      args_extra=["-p", "use_sim_time:=true"], nice="19")
        cmd_play = ["nice", "-n", "19", "ros2", "bag", "play", str(bag), "--clock",
                    "--start-paused", "--start-offset", str(inicio_s), "--topics", *TOPICOS_RENDER]
        if max_s:
            cmd_play += ["--playback-duration", str(max_s)]
        run.procs["play"] = _popen(cmd_play, pasta / "logs" / "bag_play.log", env=dict(os.environ))
        n = len(vistas_escolhidas())
        retomar = time.time() + 35.0 + 1.5 * n  # janelas (ate 30 s) + 1 ffmpeg/s
        offset = t0 - retomar - meia_noite_epoch(t0)  # carimbo = hora da corrida
        _gravar_vistas(run, display, f"{meta.get('run_id', pasta.name)} {meta.get('task', '')}",
                       offset_s=offset, nice="19")
        if not _retomar_play(retomar):
            v.update(ok=False, motivo="replay nao retomou (ver logs/bag_play.log)")
        else:
            try:
                run.procs["play"].wait(timeout=(max_s or dur) + 120.0)
            except subprocess.TimeoutExpired:
                pass
        gasto = sum(cpu_s(p.pid) for k, p in run.procs.items() if k.startswith(("rviz_", "ffmpeg_")))
        v["cpu_nucleos_render"] = round(gasto / max(time.time() - retomar, 1.0), 2)
    finally:
        _parar_procs(run.procs)
    for nome, i in v["vistas"].items():  # t_video 0 <-> instante da corrida
        try:
            w0 = epoch_inicio_video((pasta / "logs" / f"ffmpeg_{nome}.log").read_text(errors="replace"))
        except OSError:
            w0 = None
        i["inicio_epoch"] = (w0 - retomar + t0) if w0 else None
    epochs = [i["inicio_epoch"] for i in v["vistas"].values() if i.get("inicio_epoch")]
    v["inicio_epoch"] = min(epochs) if epochs else None
    escrever_metadata(run)
    return v


def nome_processo(pid: int) -> str:
    """Nome curto: o __node:= de um no' ROS, senao o script/executavel."""
    try:
        args = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    except OSError:
        return "?"
    args = [a.decode(errors="replace") for a in args if a]
    for a in args:
        if a.startswith("__node:="):
            return a.split("=", 1)[1]
    if not args:
        return "?"
    exe = os.path.basename(args[0])
    if exe.startswith("python") and len(args) > 1:
        return os.path.basename(args[1] if not args[1].startswith("-") else args[-1])
    return exe


def amostrar_cpu(saida: Path, periodo: float = 2.0) -> None:
    """CPU por processo a cada `periodo` s (so' os >= 1 %) em CSV; roda ate ser
    morto. Custo ~0,5 % de 1 nucleo. Para achar quem disputa CPU com o controller."""
    hz = os.sysconf("SC_CLK_TCK")
    ant, t_ant, nomes = {}, None, {}
    with open(saida, "w", encoding="utf-8") as f:
        f.write("epoch,pid,nome,cpu_pct\n")
        while True:
            t, atual = time.time(), {}
            for d in os.listdir("/proc"):
                if d.isdigit():
                    try:
                        c = Path(f"/proc/{d}/stat").read_text().rsplit(")", 1)[1].split()
                        atual[int(d)] = int(c[11]) + int(c[12])
                    except (OSError, IndexError, ValueError):
                        pass
            if t_ant is not None:
                for pid, tic in atual.items():
                    pct = (tic - ant.get(pid, tic)) / hz / (t - t_ant) * 100.0
                    if pct >= 1.0:
                        nome = nomes.setdefault(pid, nome_processo(pid)).replace(",", ";")
                        f.write(f"{t:.1f},{pid},{nome},{pct:.1f}\n")
                f.flush()
            ant, t_ant = atual, t
            time.sleep(periodo)


def pos_corrida(pasta: Path) -> None:
    """Depois do fim, num processo so' (nice 19): params -> analise -> videos -> analise."""
    pasta = Path(pasta)
    if _ligado("CARAMELO_PARAMS"):
        _dump_params(pasta / "config" / "params")
    analise = _executavel("analisar_run") + [str(pasta)]
    if _ligado("CARAMELO_ANALISE"):
        subprocess.run(analise, check=False)
    if _ligado("CARAMELO_VIDEO") and not _ligado("CARAMELO_VIDEO_AO_VIVO", padrao="0"):
        renderizar_offline(pasta)
        if _ligado("CARAMELO_ANALISE"):
            subprocess.run(analise, check=False)  # agora com t_video


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
            "video": {"ok": False, "pasta": "videos", "inicio_epoch": None, "vistas": {},
                      "motivo": "desligado (CARAMELO_VIDEO=0)"},
            "viz": {"ok": False},
            "sincronizacao": ("bag e rosout: relogio do PC (Pi via chrony); video: hora local "
                              "carimbada no quadro e t_video = epoch - video.inicio_epoch"),
        }
        run.meta["git"] = foto_config(pasta / "config", task_yaml, actions_yaml, map_folder)
        escrever_metadata(run)

        display = None
        if _ligado("CARAMELO_VIDEO") and _ligado("CARAMELO_VIDEO_AO_VIVO", padrao="0"):
            display = _subir_xvfb(run)
            if display:
                _subir_vistas(run, display, map_folder)  # boot junto com a descoberta do bag
        run.procs["viz"] = _popen(_executavel("run_diag_viz"), pasta / "logs" / "run_diag_viz.log")
        run.procs["cpu"] = _popen([sys.executable, os.path.abspath(__file__), "--amostrar-cpu",
                                   str(pasta / "logs" / "cpu.csv")], pasta / "logs" / "cpu.log")
        run.meta["viz"]["ok"] = True

        run.bag = bagc.iniciar_bag(mission_id, task, mapa, actions_yaml or "", extra=extra,
                                   camera=camera, nav_profile=nav_profile,
                                   saida=pasta / "rosbag")
        run.meta["bag"].update(ok=run.bag is not None,
                               comando=run.bag.comando if run.bag else None)
        if display:
            _gravar_vistas(run, display, f"{pasta.name} {task}")
        escrever_metadata(run)
        v = run.meta["video"]
        vistas_ok = [n for n, i in v["vistas"].items() if i.get("ok")]
        if not display and _ligado("CARAMELO_VIDEO"):
            v["motivo"] = "videos depois da corrida (replay do bag)"
        print(f"[run] gravando em {pasta} (bag {'ok' if run.bag else 'NAO'}, videos "
              f"{', '.join(vistas_ok) if vistas_ok else v['motivo']})")
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
    dur = max(fim - run.inicio_epoch, 1e-3)
    gasto = sum(cpu_s(p.pid) for k, p in run.procs.items()
                if k.startswith(("rviz_", "ffmpeg_")) or k == "xvfb")
    run.meta["video"]["cpu_nucleos"] = round(gasto / dur, 2)  # media da corrida
    _parar_procs(run.procs)
    if run.bag is not None:
        bagc.parar_bag(run.bag)
    for nome, info in run.meta["video"]["vistas"].items():
        try:
            info["inicio_epoch"] = epoch_inicio_video(
                (run.pasta / "logs" / f"ffmpeg_{nome}.log").read_text(errors="replace")) \
                or info.get("inicio_epoch")
        except OSError:
            pass
    epochs = [i["inicio_epoch"] for i in run.meta["video"]["vistas"].values() if i.get("inicio_epoch")]
    run.meta["video"]["inicio_epoch"] = min(epochs) if epochs else run.meta["video"].get("inicio_epoch")
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
    try:  # params, analise e videos num processo so', depois do fim (nice 19)
        _popen(["nice", "-n", "19", sys.executable, os.path.abspath(__file__), "--pos-corrida",
                str(run.pasta)], run.pasta / "logs" / "pos_corrida.log")
        print(f"[run] relatorio em 1-3 min e videos em ~{int(run.meta['duracao_s'] or 0) // 60 + 2} min: "
              f"{run.pasta}")
    except OSError as exc:
        print(f"[run] pos-corrida nao subiu: {exc}", file=sys.stderr)
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
    p.add_argument("--dump-params", default=None, help=argparse.SUPPRESS)  # uso interno (parar_run)
    p.add_argument("--pos-corrida", default=None, help=argparse.SUPPRESS)  # uso interno (parar_run)
    p.add_argument("--amostrar-cpu", default=None, help=argparse.SUPPRESS)  # uso interno (iniciar_run)
    p.add_argument("--renderizar", default=None,
                   help="refaz os videos de uma corrida a partir do bag (pasta da corrida)")
    a = p.parse_args((sys.argv if argv is None else argv)[1:])
    if a.dump_params:
        _dump_params(Path(a.dump_params))
        return 0
    if a.amostrar_cpu:
        try:
            amostrar_cpu(Path(a.amostrar_cpu))
        except KeyboardInterrupt:
            pass
        return 0
    if a.pos_corrida:
        pos_corrida(Path(a.pos_corrida))
        return 0
    if a.renderizar:
        print(json.dumps(renderizar_offline(Path(a.renderizar)), indent=1, default=str))
        return 0
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
