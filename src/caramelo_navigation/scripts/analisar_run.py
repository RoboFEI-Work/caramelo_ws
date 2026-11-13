#!/usr/bin/env python3
"""Relatorio tecnico de uma corrida (2026-11-13): <run>/analysis.md.

Le o bag da corrida uma vez (rosbag2_py, sem grafo ROS) e escreve:
  analysis.md       linha do tempo, pernas de navegacao, anomalias com hora
                    local / t_bag / t_video, logs WARN+ agrupados, relogio,
                    hipoteses (sempre marcadas como NAO confirmadas)
  logs/rosout.log   /rosout inteiro em texto, com hora local
  logs/eventos.csv  todos os eventos da analise (para cruzar com o video)
Separacao pedida pelo operador: "Fatos" = medido no bag; "Hipoteses" = regra
que liga fatos a uma causa possivel + o que olhar para confirmar. Nada aqui
declara causa comprovada.
Quem usa: caramelo_run_common.parar_run() (nice 19, segundo plano) ou a mao:
  ros2 run caramelo_navigation analisar_run runs/run_20261113_153000
  ros2 run caramelo_navigation analisar_run ~/bags/<corrida antiga>   (sem video)
"""
from __future__ import annotations

import csv
import datetime
import json
import math
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

# Limiares (fatos): mudar aqui muda so' o relatorio, nunca o robo.
CMD_MOV_V, CMD_MOV_W = 0.05, 0.20     # comando "querendo andar"
ODOM_PARADO_V, ODOM_PARADO_W = 0.02, 0.05
TRAVA_MIN_S = 1.5                     # comando sem movimento por mais que isto
SEMCMD_V, SEMCMD_W, SEMCMD_MIN_S = 0.15, 0.50, 0.3  # movimento com comando ~0
OSC_W, OSC_JANELA_S, OSC_TROCAS = 0.15, 4.0, 4
AMCL_SALTO_M, AMCL_SALTO_RAD = 0.15, math.radians(10)
PERTO_M = 0.10                        # LiDAR fica no ressalto da frente do footprint
DESVIO_PLANO_M = 0.30
MUDANCA_ROTA_M = 0.25
PISO_V, PISO_W = 0.1215, 0.315        # piso do ESC no PC (README / contexto.md)
STATUS_FIM = {4: "sucesso", 5: "cancelado", 6: "abortado"}
ACOES_LINHA = ("navigate_to_pose", "align_to_dock", "undock_robot", "dock_robot", "nudge_base",
               "pick_tag", "place_tag", "amt1_sort", "escape_free_space", "backup", "wait",
               "spin", "drive_on_heading")
ACOES_RECOVERY = ("escape_free_space", "backup", "wait", "spin")
ACOES_SO_STATUS = ("follow_path",)  # lidas para medir o laco, fora da linha do tempo
CM_ACAO = {1: "STOP", 2: "SLOWDOWN", 3: "APPROACH", 4: "LIMIT"}
NIVEL = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
CATEGORIAS = (  # (categoria, regex no "no: msg")
    ("tf", r"extrapolat|lookup would require|tf_|transform|timed out waiting for transform"),
    ("planejamento", r"^planner_server|start occupied|goal occupied|no valid path|failed to create plan|smoother"),
    ("controle", r"^controller_server|failed to make progress|no valid trajector|control loop missed|velocity_smoother|collision_monitor"),
    ("localizacao", r"^amcl|localization_watchdog|amcl fugiu"),
    ("docking", r"dock_align|docking_server|undock|nudge"),
    ("manipulacao", r"pick|place|move_group|moveit|gripper|amt1|arm_controller"),
    ("sensores", r"scan|lidar|laser|realsense|camera|imu"),
    ("missao", r"bt_navigator|bt_yaml_executor|mission"),
)
TOPICOS = ["/cmd_vel_nav", "/mecanum_controller/reference", "/odom", "/amcl_pose", "/scan",
           "/plan", "/collision_monitor_state", "/rosout",
           "/navigate_to_pose/_action/feedback"]


# ------------------------------------------------------------------ utilidades puras
def hora(t: float) -> str:
    return datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]


class Relogio:
    """Converte epoch em 'HH:MM:SS.mmm | +t_bag | t_video'."""

    def __init__(self, t0: float, video0=None):
        self.t0, self.v0 = t0, video0

    def bag(self, t):
        return f"+{t - self.t0:.1f}"

    def video(self, t):
        if self.v0 is None or t < self.v0:
            return "-"
        s = t - self.v0
        return f"{int(s // 60):02d}:{s % 60:04.1f}"

    def linha(self, t):
        return f"{hora(t)} | {self.bag(t)} | {self.video(t)}"


def ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def amostrar(pts, n):
    if len(pts) <= n:
        return list(pts)
    passo = (len(pts) - 1) / (n - 1)
    return [pts[round(i * passo)] for i in range(n)]


def desvio_medio(a, b):
    if not a or not b:
        return float("inf")
    aa, bb = amostrar(a, 40), amostrar(b, 80)
    return sum(min(math.hypot(x - u, y - v) for u, v in bb) for x, y in aa) / len(aa)


def dist_ponto_polilinha(p, pts):
    if not pts:
        return float("inf")
    if len(pts) == 1:
        return math.hypot(p[0] - pts[0][0], p[1] - pts[0][1])
    melhor = float("inf")
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - x1) * dx + (p[1] - y1) * dy) / L2))
        melhor = min(melhor, math.hypot(p[0] - x1 - u * dx, p[1] - y1 - u * dy))
    return melhor


def janelas(amostras, cond, min_s, gap_s=0.3):
    """amostras [(t, ...)] ordenadas; devolve [(t_ini, t_fim)] onde cond(amostra)
    vale continuamente por >= min_s (buracos > gap_s quebram a janela)."""
    saida, ini, ult = [], None, None
    for a in amostras:
        t = a[0]
        if cond(a):
            if ini is not None and t - ult > gap_s:  # buraco: fecha a janela anterior
                if ult - ini >= min_s:
                    saida.append((ini, ult))
                ini = None
            if ini is None:
                ini = t
            ult = t
        else:
            if ini is not None and ult - ini >= min_s:
                saida.append((ini, ult))
            ini = ult = None
    if ini is not None and ult - ini >= min_s:
        saida.append((ini, ult))
    return saida


def valor_em(serie, t):
    """Ultima amostra de serie [(t, ...)] com tempo <= t (busca binaria)."""
    lo, hi = 0, len(serie)
    while lo < hi:
        mid = (lo + hi) // 2
        if serie[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    return serie[lo - 1] if lo else None


def fatia(serie, t0, t1):
    return [a for a in serie if t0 <= a[0] <= t1]


def tempo_em(cmd, t0, t1, cond, max_dt=0.25):
    """Segundos (sample-and-hold) em que cond(vx, vy, wz) vale no comando."""
    tot = 0.0
    seg = fatia(cmd, t0, t1)
    for a, b in zip(seg, seg[1:]):
        if cond(a[1], a[2], a[3]):
            tot += min(b[0] - a[0], max_dt)
    return tot


def intervalos_actions(status):
    """status: {acao: [(t, [(uuid, st), ...])]} -> [(acao, uuid, t_ini, t_fim, resultado)]."""
    saida = []
    for acao, msgs in status.items():
        ini, fim = {}, {}
        for t, lista in msgs:
            for uuid, st in lista:
                if st in (1, 2) and uuid not in ini:
                    ini[uuid] = t
                if st in STATUS_FIM and uuid not in fim:
                    fim[uuid] = (t, STATUS_FIM[st])
                    ini.setdefault(uuid, t)
        for uuid, t_ini in ini.items():
            t_fim, res = fim.get(uuid, (None, "sem fim no bag"))
            saida.append((acao, uuid, t_ini, t_fim, res))
    return sorted(saida, key=lambda x: x[2])


def oscilacoes(cmd, idx, limiar=OSC_W, janela_s=OSC_JANELA_S, trocas=OSC_TROCAS):
    """Janelas com >= trocas inversoes de sinal do eixo idx (|v| > limiar) em janela_s."""
    viradas, sinal = [], 0
    for a in cmd:
        v = a[idx]
        if abs(v) > limiar:
            s = 1 if v > 0 else -1
            if sinal and s != sinal:
                viradas.append(a[0])
            sinal = s
    eventos, i = [], 0
    for j, t in enumerate(viradas):
        while viradas[i] < t - janela_s:
            i += 1
        if j - i + 1 >= trocas:
            if eventos and viradas[i] <= eventos[-1][1] + 0.5:
                eventos[-1] = (eventos[-1][0], t)
            else:
                eventos.append((viradas[i], t))
    return eventos


def saltos_amcl(amcl, odom):
    """Atualizacoes do AMCL que pulam mais que a odometria explica."""
    saida = []
    for a, b in zip(amcl, amcl[1:]):
        if b[0] - a[0] > 3.0:
            continue
        oa, ob = valor_em(odom, a[0]), valor_em(odom, b[0])
        d_odom = math.hypot(ob[1] - oa[1], ob[2] - oa[2]) if oa and ob else 0.0
        y_odom = abs(ang(ob[3] - oa[3])) if oa and ob else 0.0
        dxy = math.hypot(b[1] - a[1], b[2] - a[2]) - d_odom
        dyaw = abs(ang(b[3] - a[3])) - y_odom
        if dxy > AMCL_SALTO_M or dyaw > AMCL_SALTO_RAD:
            saida.append((b[0], dxy, math.degrees(dyaw)))
    return saida


def normalizar(msg: str) -> str:
    return re.sub(r"[-+]?\d+(\.\d+)?", "#", msg.strip())[:110]


def categoria(no: str, msg: str) -> str:
    texto = f"{no}: {msg}".lower()
    for cat, rx in CATEGORIAS:
        if re.search(rx, texto):
            return cat
    return "outros"


# ------------------------------------------------------------------ leitura do bag
def achar_bag(pasta: Path) -> Path:
    for p in (pasta / "rosbag", pasta):
        if (p / "metadata.yaml").exists() or list(p.glob("*.mcap")):
            return p
    raise FileNotFoundError(f"nenhum bag em {pasta}")


def ler_bag(pasta_bag: Path) -> dict:
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage = "mcap"
    try:
        storage = yaml.safe_load((pasta_bag / "metadata.yaml").read_text())[
            "rosbag2_bagfile_information"]["storage_identifier"]
    except (OSError, KeyError, TypeError):
        pass
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(pasta_bag), storage_id=storage),
           rosbag2_py.ConverterOptions("", ""))
    tipos = {t.name: t.type for t in r.get_all_topics_and_types()}
    status_tops = [t for t in tipos if t.endswith("/_action/status")
                   and t.split("/_action")[0].lstrip("/") in ACOES_LINHA + ACOES_SO_STATUS]
    quero = [t for t in TOPICOS if t in tipos] + status_tops
    r.set_filter(rosbag2_py.StorageFilter(topics=quero))
    cls = {t: get_message(tipos[t]) for t in quero}
    d = {k: [] for k in ("cmd", "ref", "odom", "amcl", "scan", "plan", "cm", "rosout", "fb",
                         "atraso_odom", "atraso_scan")}
    d["status"] = defaultdict(list)
    d["t0"] = d["t1"] = None
    yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))  # noqa: E731
    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9  # noqa: E731
    while r.has_next():
        top, dados, tns = r.read_next()
        t = tns * 1e-9
        d["t0"] = t if d["t0"] is None else d["t0"]
        d["t1"] = t
        m = deserialize_message(dados, cls[top])
        if top == "/cmd_vel_nav":
            d["cmd"].append((t, m.linear.x, m.linear.y, m.angular.z))
        elif top == "/mecanum_controller/reference":
            tw = m.twist
            d["ref"].append((t, tw.linear.x, tw.linear.y, tw.angular.z))
        elif top == "/odom":
            p, tw = m.pose.pose, m.twist.twist
            d["odom"].append((t, p.position.x, p.position.y, yaw(p.orientation),
                              tw.linear.x, tw.linear.y, tw.angular.z))
            d["atraso_odom"].append(t - stamp(m.header))
        elif top == "/amcl_pose":
            p, c = m.pose.pose, m.pose.covariance
            d["amcl"].append((t, p.position.x, p.position.y, yaw(p.orientation),
                              math.sqrt(max(c[0], 0) + max(c[7], 0))))
        elif top == "/scan":
            piso = max(0.05, m.range_min)
            d["scan"].append((t, min((x for x in m.ranges if piso < x < 1e3), default=9.9)))
            d["atraso_scan"].append(t - stamp(m.header))
        elif top == "/plan":
            pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
            if pts:
                L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
                d["plan"].append((t, amostrar(pts, 120), L))
        elif top == "/collision_monitor_state":
            d["cm"].append((t, int(m.action_type), m.polygon_name))
        elif top == "/rosout":
            d["rosout"].append((t, int(m.level), m.name, m.msg))
        elif top == "/navigate_to_pose/_action/feedback":
            d["fb"].append((t, bytes(m.goal_id.uuid).hex()[:8], int(m.feedback.number_of_recoveries)))
        elif top in status_tops:
            d["status"][top.split("/_action")[0].lstrip("/")].append(
                (t, [(bytes(s.goal_info.goal_id.uuid).hex()[:8], int(s.status))
                     for s in m.status_list]))
    return d


# ------------------------------------------------------------------ analise
def estacoes(rosout):
    """(t, dock) de cada goal do align_to_dock: o '=== Estacao' do GoToWS nao vai
    para o /rosout (logger sem no'), mas o dock_align_node loga o dock na entrada."""
    rx = re.compile(r"^\[([A-Za-z0-9_]+)\] \[inst\] entrada do goal")
    return [(t, m.group(1)) for t, _l, no, msg in rosout
            if no.startswith("dock_align") for m in [rx.search(msg)] if m]


def analisar(d: dict) -> dict:
    t0, t1 = d["t0"], d["t1"]
    acoes = intervalos_actions(d["status"])
    est = estacoes(d["rosout"])
    ev = []  # (t, tipo, categoria, texto)

    # FollowPath executando: o BT manda um goal novo a cada replan (~1 s), entao
    # une os intervalos colados (buraco < 0,5 s) antes de medir o laco.
    seguindo = []
    for ac, _u, fa, fb, _r in acoes:
        if ac == "follow_path":
            fb = fb or t1
            if seguindo and fa - seguindo[-1][1] < 0.5:
                seguindo[-1] = (seguindo[-1][0], max(seguindo[-1][1], fb))
            else:
                seguindo.append((fa, fb))

    # --- pernas de navegacao
    pernas = []
    for acao, uuid, a, b, res in acoes:
        if acao != "navigate_to_pose":
            continue
        b = b or t1
        rot = next((e for e in est if a <= e[0] <= b + 15.0), None)  # dock logo depois da perna
        planos = [p for p in d["plan"] if a <= p[0] <= b]
        mudancas = []
        for p0, p1 in zip(planos, planos[1:]):
            mesmo = math.hypot(p0[1][-1][0] - p1[1][-1][0], p0[1][-1][1] - p1[1][-1][1]) < 0.3
            dv = desvio_medio(p1[1], p0[1])
            if mesmo and dv > MUDANCA_ROTA_M:
                mudancas.append((p1[0], dv))
        desv_max, t_desv = 0.0, None
        for s in fatia(d["amcl"], a, b):
            pl = valor_em(planos, s[0])
            if pl:
                dd = dist_ponto_polilinha((s[1], s[2]), pl[1])
                if dd > desv_max:
                    desv_max, t_desv = dd, s[0]
        od = fatia(d["odom"], a, b)
        dist = sum(math.hypot(q[1] - p[1], q[2] - p[2]) for p, q in zip(od, od[1:]))
        sc = fatia(d["scan"], a, b)
        sc_min = min(sc, key=lambda s: s[1]) if sc else None
        dur = max(b - a, 1e-6)
        giro = tempo_em(d["cmd"], a, b, lambda vx, vy, w: abs(w) > 0.05 and math.hypot(vx, vy) < 0.02)
        lat = tempo_em(d["cmd"], a, b, lambda vx, vy, w: abs(vy) > abs(vx) and math.hypot(vx, vy) > 0.02)
        parado = tempo_em(d["cmd"], a, b, lambda vx, vy, w: math.hypot(vx, vy) < 0.02 and abs(w) < 0.05)
        hz = []  # msgs/s de /cmd_vel_nav nos segundos inteiros com o FollowPath executando
        for fa, fb in seguindo:
            if fa < b and fb > a:
                cont = defaultdict(int)
                for c in fatia(d["cmd"], max(fa, a), min(fb, b)):
                    cont[int(c[0])] += 1
                hz += [n for seg, n in cont.items() if max(fa, a) <= seg and seg + 1 <= min(fb, b)]
        hz.sort()
        recov = max((f[2] for f in d["fb"] if f[1] == uuid), default=0)
        pernas.append(dict(
            n=len(pernas) + 1, uuid=uuid, ini=a, fim=b, res=res, dur=dur,
            estacao=rot[1] if rot else "-", dist=dist,
            giro=100 * giro / dur, lat=100 * lat / dur, parado=100 * parado / dur,
            planos=len(planos), mudancas=mudancas, recov=recov, desv=desv_max, t_desv=t_desv,
            scan_min=sc_min, hz_med=statistics.median(hz) if hz else 0, hz_min=hz[0] if hz else 0))
        for tm, dv in mudancas:
            ev.append((tm, "mudanca_rota", "planejamento",
                       f"replan mudou a rota (desvio medio {dv:.2f} m) na perna {len(pernas)}"))
        if desv_max > DESVIO_PLANO_M:
            ev.append((t_desv, "desvio_plano", "controle",
                       f"robo a {desv_max:.2f} m do plano global (perna {len(pernas)})"))
        if res != "sucesso":
            ev.append((b, "nav_" + res.replace(" ", "_"), "missao",
                       f"navigate_to_pose {res} (perna {len(pernas)}, {pernas[-1]['estacao']})"))

    # --- comando sem movimento / movimento sem comando (comando que chega as rodas)
    ref, odom = d["ref"], d["odom"]
    junto = []
    for o in odom:
        r = valor_em(ref, o[0])
        if r is not None and o[0] - r[0] < 0.5:
            junto.append((o[0], r[1], r[2], r[3], o[4], o[5], o[6]))
    trav = janelas(junto, lambda a: (math.hypot(a[1], a[2]) >= CMD_MOV_V or abs(a[3]) >= CMD_MOV_W)
                   and math.hypot(a[4], a[5]) < ODOM_PARADO_V and abs(a[6]) < ODOM_PARADO_W,
                   TRAVA_MIN_S)
    for a, b in trav:
        seg = [x for x in junto if a <= x[0] <= b]
        v = max(math.hypot(x[1], x[2]) for x in seg)
        w = max(abs(x[3]) for x in seg)
        abaixo = v < PISO_V and w < PISO_W
        ev.append((a, "travado", "controle",
                   f"comando sem movimento por {b - a:.1f} s (cmd ate {v:.2f} m/s, {w:.2f} rad/s"
                   f"{'; ABAIXO do piso do ESC' if abaixo else ''})"))
    semcmd = janelas(junto, lambda a: math.hypot(a[1], a[2]) < 0.01 and abs(a[3]) < 0.02
                     and (math.hypot(a[4], a[5]) > SEMCMD_V or abs(a[6]) > SEMCMD_W),
                     SEMCMD_MIN_S, gap_s=0.1)
    for a, b in semcmd:
        seg = [x for x in junto if a <= x[0] <= b]
        ev.append((a, "mov_sem_cmd", "controle",
                   f"robo se moveu sem comando por {b - a:.1f} s (odom ate "
                   f"{max(math.hypot(x[4], x[5]) for x in seg):.2f} m/s, "
                   f"{max(abs(x[6]) for x in seg):.2f} rad/s)"))

    # --- oscilacao do controlador
    for idx, eixo in ((3, "giro"), (2, "lateral")):
        for a, b in oscilacoes(d["cmd"], idx):
            ev.append((a, "oscilacao", "controle",
                       f"oscilacao de {eixo} no /cmd_vel_nav por {b - a:.1f} s"))

    # --- localizacao
    for t, dxy, dyaw in saltos_amcl(d["amcl"], odom):
        ev.append((t, "salto_amcl", "localizacao",
                   f"AMCL pulou {max(dxy, 0):.2f} m / {max(dyaw, 0):.0f} graus alem da odometria"))
    # --- perto de obstaculo: so' navegando (no docking encostar a 6 cm da mesa e' o normal)
    for p in pernas:
        for a, b in janelas(fatia(d["scan"], p["ini"], p["fim"]), lambda s: s[1] < PERTO_M,
                            0.0, gap_s=0.5):
            ev.append((a, "perto", "sensores",
                       f"LiDAR a {min(s[1] for s in fatia(d['scan'], a, b)):.3f} m de algo "
                       f"navegando ({b - a:.1f} s, perna {p['n']})"))
    # --- collision monitor
    ult = 0
    for t, acao, pol in d["cm"]:
        if acao != ult and acao in CM_ACAO:
            ev.append((t, "collision_monitor", "controle", f"collision_monitor {CM_ACAO[acao]} ({pol})"))
        ult = acao
    # --- recoveries e actions da missao
    for acao, uuid, a, b, res in acoes:
        if acao in ACOES_RECOVERY:
            ev.append((a, "recovery", "controle", f"recovery {acao} ({res})"))
    # --- logs
    grupos = {}
    for t, lvl, no, msg in d["rosout"]:
        if lvl < 30:
            continue
        k = (lvl, no, normalizar(msg))
        g = grupos.setdefault(k, {"n": 0, "ini": t, "fim": t, "cat": categoria(no, msg), "ex": msg})
        g["n"] += 1
        g["fim"] = t
    for (lvl, no, _), g in grupos.items():
        ev.append((g["ini"], "log", g["cat"], f"{NIVEL.get(lvl, lvl)} {no}: {g['ex'][:120]}"
                   + (f" (x{g['n']})" if g["n"] > 1 else "")))
    ev.sort(key=lambda e: e[0])

    atraso = {}
    for k in ("atraso_odom", "atraso_scan"):
        xs = sorted(d[k])
        if xs:
            atraso[k] = (xs[len(xs) // 2], xs[int(0.99 * (len(xs) - 1))], xs[0])
    return dict(t0=t0, t1=t1, acoes=acoes, pernas=pernas, eventos=ev, grupos=grupos,
                atraso=atraso, estacoes=est,
                resets=[e for e in d["rosout"] if "AMCL fugiu" in e[3]])


def hipoteses(r: dict) -> list:
    """Regras conservadoras: (titulo, fatos que apoiam, como confirmar)."""
    ev = r["eventos"]
    por = defaultdict(list)
    for e in ev:
        por[e[1]].append(e)
    hs = []
    abaixo = [e for e in por["travado"] if "ABAIXO" in e[3]]
    if abaixo:
        hs.append(("Comando abaixo do piso do ESC: a roda nao gira", abaixo,
                   "olhar /joint_states (velocidade das rodas) e /mecanum_controller/reference "
                   "na janela; roda parada com referencia < 0,1215 m/s confirma"))
    acima = [e for e in por["travado"] if "ABAIXO" not in e[3]]
    if acima:
        hs.append(("Comando acima do piso sem movimento: robo encostado/preso, driver ou "
                   "timeout da referencia", acima,
                   "ver o video (contato fisico?), /scan minimo e gaps em "
                   "/mecanum_controller/reference (> 150 ms) nessa hora"))
    if por["mov_sem_cmd"]:
        hs.append(("Movimento sem comando (tranco do driver/ESC, tarefa T17)", por["mov_sem_cmd"],
                   "trancos.py do bag_forense com /joint_states; encoder rapido com referencia "
                   "zero confirma que e' abaixo do PC"))
    loc = por["salto_amcl"] + [(t, "reset", "localizacao", m) for t, _l, _n, m in r["resets"]]
    if loc:
        hs.append(("Localizacao: o AMCL corrigiu/pulou mais que a odometria explica", loc,
                   "plot_janela.py do bag_forense na janela (scan sobre o mapa nas duas poses); "
                   "obstaculo nao mapeado a vista reforca"))
    ocup = [e for e in ev if e[1] == "log" and re.search(r"(start|goal) occupied", e[3], re.I)]
    if ocup:
        hs.append(("Pose dentro de celula letal do costmap global (pose deslocada ou inflacao)",
                   ocup, "costmap global + /amcl_pose nesse instante; conferir se houve salto do AMCL antes"))
    instavel = [p for p in r["pernas"] if len(p["mudancas"]) >= 3]
    if instavel:
        hs.append(("Replanejamento instavel (rota muda varias vezes na mesma perna)",
                   [(p["mudancas"][0][0], "", "", f"perna {p['n']}: {len(p['mudancas'])} mudancas de rota")
                    for p in instavel],
                   "costmap global/local nessas horas: obstaculo dinamico, limpeza do costmap "
                   "(recovery) ou pose oscilando"))
    lentas = [p for p in r["pernas"] if p["hz_med"] and p["hz_med"] < 15]
    missed = [e for e in ev if e[1] == "log" and "control loop missed" in e[3].lower()]
    if lentas or missed:
        hs.append(("Laco do controller abaixo de 20 Hz (CPU ou DWB pesado)",
                   [(p["ini"], "", "", f"perna {p['n']}: mediana {p['hz_med']} Hz, min {p['hz_min']} Hz")
                    for p in lentas] + missed,
                   "CPU do PC durante a corrida (top/htop) e contagem de 'Control loop missed'"))
    giro = [p for p in r["pernas"] if p["giro"] > 40 and p["dur"] > 5]
    if giro:
        hs.append(("Giro parado domina a perna (RotationShim / alinhamento, tarefa T15)",
                   [(p["ini"], "", "", f"perna {p['n']}: {p['giro']:.0f} % girando em {p['dur']:.0f} s")
                    for p in giro],
                   "/cmd_vel_nav com vx=vy=0 e /plan curto ou com rumo trocando (plan_rumo.py)"))
    if por["desvio_plano"]:
        hs.append(("Robo longe do plano global (controle nao seguiu ou pose pulou)",
                   por["desvio_plano"],
                   "se coincide com salto do AMCL, e' localizacao; senao, /local_plan e costmap local"))
    if por["perto"]:
        hs.append(("Risco de colisao: algo a menos de 10 cm do LiDAR", por["perto"],
                   "video nessa hora; o LiDAR so' ve os 180 graus da frente (/scan_raw tem os 360)"))
    if por["oscilacao"]:
        hs.append(("Oscilacao do controlador (vaivem de giro ou lateral)", por["oscilacao"],
                   "criticos do DWB / RotationShim nessa hora; /local_plan trocando de lado"))
    return hs


# ------------------------------------------------------------------ saida
def _md_tab(cab, linhas):
    out = ["| " + " | ".join(cab) + " |", "|" + "|".join("---" for _ in cab) + "|"]
    out += ["| " + " | ".join(str(c).replace("|", "/") for c in l) + " |" for l in linhas]
    return "\n".join(out)


def escrever(pasta: Path, d: dict, r: dict, meta: dict) -> Path:
    rel = Relogio(r["t0"], (meta.get("video") or {}).get("inicio_epoch"))
    L = [f"# Analise da corrida {meta.get('run_id', pasta.name)}", "",
         f"Gerado {datetime.datetime.now():%Y-%m-%d %H:%M} por `analisar_run.py` a partir do bag. "
         "Secoes 1-5 sao **fatos medidos**; a secao 6 traz **hipoteses nao confirmadas**. "
         "Nenhuma causa aqui esta comprovada.", ""]
    git = meta.get("git") or {}
    vid = meta.get("video") or {}
    L += [_md_tab(["Campo", "Valor"], [
        ("Task", meta.get("task", "-")), ("Mapa", meta.get("mapa", "-")),
        ("Resultado", (meta.get("resultado") or {}).get("texto", "-")),
        ("Inicio / fim do bag", f"{hora(r['t0'])} / {hora(r['t1'])} ({r['t1'] - r['t0']:.0f} s)"),
        ("Codigo", f"{git.get('rev', '-')} ({'com alteracoes nao commitadas' if git.get('sujo') else 'limpo'})"),
        ("Perfil nav", meta.get("nav_profile", "-")),
        ("Video", f"rviz_video.mp4, 1o quadro {hora(vid['inicio_epoch'])}" if vid.get("inicio_epoch")
         else f"sem video ({vid.get('motivo', 'nao gravado')})"),
    ]), "",
        "Como achar um momento: a **hora local** e' o carimbo amarelo do video e o relogio do log; "
        "**t_bag** e' o segundo desde o inicio do bag (`ros2 bag play rosbag --start-offset <t_bag>`); "
        "**t_video** (mm:ss) e' a posicao no player.", ""]

    L += ["## 1. Linha do tempo", ""]
    linhas = []
    for t, dock in r["estacoes"]:
        linhas.append((t, f"dock_align: estacao {dock}"))
    for acao, uuid, a, b, res in r["acoes"]:
        if acao in ACOES_RECOVERY or acao in ACOES_SO_STATUS:
            continue
        linhas.append((a, f"{acao} inicio"))
        if b:
            linhas.append((b, f"{acao} {res} ({b - a:.1f} s)"))
    L += [_md_tab(["hora", "t_bag", "t_video", "evento"],
                  [(hora(t), rel.bag(t), rel.video(t), txt) for t, txt in sorted(linhas)]), ""]

    L += ["## 2. Pernas de navegacao (navigate_to_pose)", "",
          "giro = vx=vy=0 com giro comandado; lateral = |vy|>|vx|; parado = comando zero; "
          "laco = msgs/s de /cmd_vel_nav (alvo 20).", ""]
    L += [_md_tab(["#", "estacao", "inicio", "t_video", "dur s", "resultado", "dist m", "giro %",
                   "lateral %", "parado %", "planos", "mudancas rota", "recov", "desvio max m",
                   "LiDAR min m", "laco Hz med/min"],
                  [(p["n"], p["estacao"], hora(p["ini"]), rel.video(p["ini"]), f"{p['dur']:.1f}",
                    p["res"], f"{p['dist']:.2f}", f"{p['giro']:.0f}", f"{p['lat']:.0f}",
                    f"{p['parado']:.0f}", p["planos"], len(p["mudancas"]), p["recov"],
                    f"{p['desv']:.2f}" + (f" @{hora(p['t_desv'])}" if p["t_desv"] else ""),
                    f"{p['scan_min'][1]:.2f}" if p["scan_min"] else "-",
                    f"{p['hz_med']:.0f}/{p['hz_min']}") for p in r["pernas"]]), ""]

    L += ["## 3. Anomalias detectadas (fatos)", ""]
    tipos = defaultdict(list)
    for e in r["eventos"]:
        if e[1] != "log":
            tipos[e[1]].append(e)
    if not tipos:
        L += ["Nenhuma anomalia pelos limiares da analise.", ""]
    for tipo, evs in sorted(tipos.items(), key=lambda kv: kv[1][0][0]):
        L += [f"**{tipo}** ({len(evs)}, categoria {evs[0][2]})", ""]
        L += [f"- {rel.linha(e[0])} - {e[3]}" for e in evs[:15]]
        if len(evs) > 15:
            L += [f"- ... mais {len(evs) - 15} (ver logs/eventos.csv)"]
        L += [""]

    L += ["## 4. Logs WARN/ERROR agrupados", "",
          "Numeros trocados por # para agrupar; texto completo em logs/rosout.log.", ""]
    gs = sorted(r["grupos"].items(), key=lambda kv: (-kv[0][0], -kv[1]["n"]))
    L += [_md_tab(["nivel", "categoria", "no", "mensagem", "n", "primeira", "ultima"],
                  [(NIVEL.get(k[0], k[0]), g["cat"], k[1], g["ex"][:110], g["n"],
                    hora(g["ini"]), hora(g["fim"])) for k, g in gs[:60]]), ""]
    if len(gs) > 60:
        L += [f"... mais {len(gs) - 60} grupos.", ""]

    L += ["## 5. Relogio e TF", "",
          "atraso = recebimento no PC - header.stamp (relogio da Pi). Negativo = relogio da Pi "
          "adiantado; p99 alto = TF/odom chegando tarde (o costmap usa transform_tolerance 0,6 s).", ""]
    L += [_md_tab(["topico", "mediana ms", "p99 ms", "minimo ms"],
                  [(k.replace("atraso_", "/"), f"{v[0] * 1e3:.0f}", f"{v[1] * 1e3:.0f}",
                    f"{v[2] * 1e3:.0f}") for k, v in r["atraso"].items()]), ""]

    L += ["## 6. Hipoteses (NAO confirmadas) e como confirmar", ""]
    hs = hipoteses(r)
    if not hs:
        L += ["Nenhuma regra disparou.", ""]
    for i, (tit, fatos, conf) in enumerate(hs, 1):
        L += [f"**H{i}. {tit}**", ""]
        L += [f"- fato: {rel.linha(f[0])} - {f[3]}" for f in fatos[:8]]
        if len(fatos) > 8:
            L += [f"- ... mais {len(fatos) - 8}"]
        L += [f"- para confirmar: {conf}", ""]

    L += ["## 7. Limites desta analise", "",
          "- Travamento e' medido por /odom (EKF de rodas + IMU): roda patinando contra um "
          "obstaculo parece movimento. Video e /scan desempatam.",
          "- Pose no mapa = /amcl_pose (atualiza a cada 5 cm / 0,05 rad), nao a TF continua.",
          "- Trajetorias invalidas do DWB nao sao gravadas (/marker e /evaluation pesam no laco "
          "de 20 Hz); expansoes do planner so' existem com debug_visualizations do Smac ligado.",
          "- Limiares no topo de analisar_run.py; mudar la' muda so' o relatorio.", ""]
    saida = pasta / "analysis.md"
    saida.write_text("\n".join(L), encoding="utf-8")
    return saida


def escrever_logs(pasta: Path, d: dict, r: dict, meta: dict) -> None:
    logs = pasta / "logs"
    logs.mkdir(exist_ok=True)
    rel = Relogio(r["t0"], (meta.get("video") or {}).get("inicio_epoch"))
    with open(logs / "rosout.log", "w", encoding="utf-8") as f:
        for t, lvl, no, msg in d["rosout"]:
            f.write(f"{hora(t)} {rel.bag(t):>8} {NIVEL.get(lvl, lvl):5} [{no}] {msg}\n")
    with open(logs / "eventos.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["hora", "t_bag", "t_video", "tipo", "categoria", "texto"])
        for t, tipo, cat, txt in r["eventos"]:
            w.writerow([hora(t), rel.bag(t), rel.video(t), tipo, cat, txt])


def main(argv=None) -> int:
    argv = sys.argv if argv is None else argv
    if len(argv) < 2:
        print(__doc__)
        return 2
    pasta = Path(argv[1]).expanduser().resolve()
    try:
        meta = json.loads((pasta / "metadata.json").read_text())
    except (OSError, ValueError):
        meta = {"run_id": pasta.name}
    bag = achar_bag(pasta)
    print(f"[analise] lendo {bag} ...", flush=True)
    d = ler_bag(bag)
    if d["t0"] is None:
        print("[analise] bag vazio.", file=sys.stderr)
        return 1
    r = analisar(d)
    escrever_logs(pasta, d, r, meta)
    saida = escrever(pasta, d, r, meta)
    print(f"[analise] {saida} ({len(r['pernas'])} pernas, {len(r['eventos'])} eventos)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
