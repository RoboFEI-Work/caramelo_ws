#!/usr/bin/env python3
"""Resumo offline de um bag de corrida (rosbag2_py, sem grafo ROS).

O que imprime: duracao, tamanho, mensagens e bytes por topico; taxa real de
/cmd_vel_nav (o laco de 20 Hz do controller, que vive perdendo ciclo);
offset header.stamp x recv_timestamp em /odom e /scan (relogio da Pi x PC,
criterio chrony < 5 ms); e, por goal do /navigate_to_pose, a fracao do tempo
em movimento com |vy| > |vx| em /cmd_vel_nav (metrica do "caranguejo").
Quem usa: operador/agente depois da corrida; test_bag_common_ros.py.
Como roda: ros2 run caramelo_navigation bag_resumo ~/bags/<corrida> [--json]
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICO_CMD = "/cmd_vel_nav"
TOPICOS_OFFSET = ("/odom", "/scan")
TOPICO_STATUS_NAV = "/navigate_to_pose/_action/status"
# action_msgs/GoalStatus
STATUS_EXECUTING = 2
STATUS_TERMINAIS = (4, 5, 6)  # SUCCEEDED, CANCELED, ABORTED
NOMES_STATUS = {4: "sucesso", 5: "cancelado", 6: "abortado"}
VEL_MINIMA = 1e-3  # abaixo disto o robo esta parado (nao conta no caranguejo)


# ------------------------------------------------------------------ leitura
def storage_id_de(pasta: Path) -> str:
    meta = pasta / "metadata.yaml"
    try:
        dados = yaml.safe_load(meta.read_text(encoding="utf-8")) or {}
        return str(dados.get("rosbag2_bagfile_information", {}).get("storage_identifier")
                   or "mcap")
    except (OSError, yaml.YAMLError):
        return "mcap"


def _segundos(duracao) -> float:
    """rclpy.duration.Duration (rosbag2_py Jazzy), timedelta ou ns inteiros."""
    if hasattr(duracao, "nanoseconds"):
        return float(duracao.nanoseconds) / 1e9
    if hasattr(duracao, "total_seconds"):
        return float(duracao.total_seconds())
    try:
        return float(duracao) / 1e9
    except (TypeError, ValueError):
        return float("nan")


def ler_metadata(pasta: Path, storage_id: str):
    return rosbag2_py.Info().read_metadata(str(pasta), storage_id)


def abrir_leitor(pasta: Path, storage_id: str):
    leitor = rosbag2_py.SequentialReader()
    leitor.open(rosbag2_py.StorageOptions(uri=str(pasta), storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    return leitor


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _twist_de(msg):
    """geometry_msgs/Twist ou TwistStamped -> (vx, vy, wz)."""
    tw = msg.twist if hasattr(msg, "twist") else msg
    return float(tw.linear.x), float(tw.linear.y), float(tw.angular.z)


# --------------------------------------------------------------- estatistica
def _percentil(valores, q: float) -> float:
    if not valores:
        return float("nan")
    ordenados = sorted(valores)
    k = min(len(ordenados) - 1, max(0, int(round(q * (len(ordenados) - 1)))))
    return float(ordenados[k])


def estat_ms(valores_ns) -> dict:
    ms = [v / 1e6 for v in valores_ns]
    if not ms:
        return {"n": 0}
    return {"n": len(ms), "mediana": statistics.median(ms), "p05": _percentil(ms, 0.05),
            "p95": _percentil(ms, 0.95), "min": min(ms), "max": max(ms)}


def taxa_cmd(tempos_ns) -> dict:
    """Taxa media e intervalos do /cmd_vel_nav (recv_timestamp do PC)."""
    if len(tempos_ns) < 2:
        return {"n": len(tempos_ns)}
    ts = sorted(tempos_ns)
    intervalos = [(b - a) / 1e6 for a, b in zip(ts, ts[1:])]
    dur = (ts[-1] - ts[0]) / 1e9
    return {
        "n": len(ts), "duracao_s": dur,
        "taxa_hz": (len(ts) - 1) / dur if dur > 0 else float("nan"),
        "intervalo_mediana_ms": statistics.median(intervalos),
        "intervalo_p95_ms": _percentil(intervalos, 0.95),
        "intervalos_acima_100ms": sum(1 for i in intervalos if i > 100.0),
    }


def intervalos_de_goals(eventos_status) -> list:
    """eventos_status: lista de (t_recv_ns, goal_id_hex, status). Devolve
    [{goal, inicio_ns, fim_ns, status}] na ordem de inicio. Sem status
    EXECUTING visto, o goal comeca no primeiro evento dele."""
    goals = {}
    ordem = []
    for t, gid, st in eventos_status:
        g = goals.get(gid)
        if g is None:
            g = {"goal": gid, "inicio_ns": t, "fim_ns": None, "status": None}
            goals[gid] = g
            ordem.append(gid)
        if st == STATUS_EXECUTING and g["fim_ns"] is None and g["status"] is None:
            g["inicio_ns"] = min(g["inicio_ns"], t)
        if st in STATUS_TERMINAIS and g["fim_ns"] is None:
            g["fim_ns"] = t
            g["status"] = NOMES_STATUS.get(st, str(st))
    return [goals[g] for g in ordem]


def caranguejo_por_goal(cmds, goals, fim_bag_ns) -> list:
    """cmds: lista ordenada de (t_ns, vx, vy, wz). Para cada goal, tempo em
    movimento (|vx|+|vy| > VEL_MINIMA) e fracao desse tempo com |vy| > |vx|.
    Cada comando vale ate o seguinte (soma de dt)."""
    if not goals:
        if not cmds:
            return []
        goals = [{"goal": "bag_inteiro", "inicio_ns": cmds[0][0], "fim_ns": fim_bag_ns,
                  "status": "-"}]
    saida = []
    for g in goals:
        ini, fim = g["inicio_ns"], g["fim_ns"] if g["fim_ns"] is not None else fim_bag_ns
        mov = lat = 0.0
        soma_v = 0.0
        n = 0
        for (t, vx, vy, _wz), prox in zip(cmds, cmds[1:] + [None]):
            if t < ini or t >= fim:
                continue
            dt = ((prox[0] if prox else fim) - t) / 1e9
            dt = max(0.0, min(dt, (fim - t) / 1e9))
            v = abs(vx) + abs(vy)
            n += 1
            soma_v += math.hypot(vx, vy)
            if v > VEL_MINIMA:
                mov += dt
                if abs(vy) > abs(vx):
                    lat += dt
        saida.append({
            "goal": g["goal"][:8] if len(g["goal"]) == 32 else g["goal"],
            "status": g["status"] or "em_voo",
            "duracao_s": (fim - ini) / 1e9, "n_cmd": n,
            "tempo_movimento_s": mov,
            "fracao_lateral": (lat / mov) if mov > 0 else float("nan"),
            "vel_media_m_s": (soma_v / n) if n else float("nan"),
        })
    return saida


# ------------------------------------------------------------------ resumo
def resumir(pasta: Path) -> dict:
    pasta = Path(pasta).expanduser().resolve()
    storage_id = storage_id_de(pasta)
    meta = ler_metadata(pasta, storage_id)
    resumo = {
        "pasta": str(pasta), "storage": storage_id,
        "duracao_s": _segundos(meta.duration), "tamanho_bytes": int(meta.bag_size),
        "mensagens": int(meta.message_count), "arquivos": list(meta.relative_file_paths),
        "custom_data": dict(getattr(meta, "custom_data", {}) or {}),
        "topicos": {}, "cmd_vel_nav": {}, "offset_ms": {}, "goals": [],
    }
    for ti in meta.topics_with_message_count:
        resumo["topicos"][ti.topic_metadata.name] = {
            "tipo": ti.topic_metadata.type, "msgs": int(ti.message_count), "bytes": 0}

    leitor = abrir_leitor(pasta, storage_id)
    tipos = {t.name: t.type for t in leitor.get_all_topics_and_types()}
    classes = {}
    for nome in (TOPICO_CMD, TOPICO_STATUS_NAV, *TOPICOS_OFFSET):
        if nome in tipos:
            try:
                classes[nome] = get_message(tipos[nome])
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                print(f"[bag_resumo] sem tipo para {nome} ({tipos[nome]}): {exc}",
                      file=sys.stderr)

    bytes_por_topico = defaultdict(int)
    cmds, eventos, offsets = [], [], defaultdict(list)
    t_primeiro, t_ultimo = None, 0
    while leitor.has_next():
        topico, dado, t = leitor.read_next()  # t = recv_timestamp (relogio do PC)
        bytes_por_topico[topico] += len(dado)
        t_primeiro = t if t_primeiro is None else min(t_primeiro, t)
        t_ultimo = max(t_ultimo, t)
        cls = classes.get(topico)
        if cls is None:
            continue
        msg = deserialize_message(dado, cls)
        if topico == TOPICO_CMD:
            cmds.append((t, *_twist_de(msg)))
        elif topico == TOPICO_STATUS_NAV:
            for st in msg.status_list:
                gid = bytes(st.goal_info.goal_id.uuid).hex()
                eventos.append((t, gid, int(st.status)))
        elif topico in TOPICOS_OFFSET:
            offsets[topico].append(t - _stamp_ns(msg.header.stamp))

    for nome, nb in bytes_por_topico.items():
        resumo["topicos"].setdefault(nome, {"tipo": tipos.get(nome, "?"), "msgs": 0})
        resumo["topicos"][nome]["bytes"] = int(nb)
    if math.isnan(resumo["duracao_s"]) and t_primeiro is not None:
        resumo["duracao_s"] = (t_ultimo - t_primeiro) / 1e9  # reserva: pelos recv_timestamps
    cmds.sort(key=lambda c: c[0])
    resumo["cmd_vel_nav"] = taxa_cmd([c[0] for c in cmds])
    resumo["offset_ms"] = {k: estat_ms(v) for k, v in offsets.items()}
    resumo["goals"] = caranguejo_por_goal(cmds, intervalos_de_goals(eventos), t_ultimo)
    return resumo


def _fmt(v, casas=1):
    if isinstance(v, float):
        return "nan" if math.isnan(v) else f"{v:.{casas}f}"
    return str(v)


def imprimir(r: dict) -> None:
    print(f"Bag:      {r['pasta']}  ({r['storage']}, {len(r['arquivos'])} arquivo(s))")
    print(f"Duracao:  {_fmt(r['duracao_s'])} s   Tamanho: {r['tamanho_bytes'] / 2**20:.1f} MiB"
          f"   Mensagens: {r['mensagens']}")
    if r["custom_data"]:
        print("Corrida:  " + "  ".join(f"{k}={v}" for k, v in r["custom_data"].items()))
    print("\nTopicos (mensagens / bytes serializados / taxa media):")
    dur = r["duracao_s"] if r["duracao_s"] and r["duracao_s"] > 0 else float("nan")
    for nome, info in sorted(r["topicos"].items(), key=lambda kv: -kv[1].get("bytes", 0)):
        n = info.get("msgs", 0)
        print(f"  {nome:<45} {n:>8}  {info.get('bytes', 0) / 2**20:>8.2f} MiB"
              f"  {_fmt(n / dur if dur == dur else float('nan'))} Hz  {info.get('tipo', '')}")
    c = r["cmd_vel_nav"]
    print(f"\n{TOPICO_CMD} (laco do controller):")
    if c.get("n", 0) < 2:
        print("  sem mensagens suficientes")
    else:
        print(f"  {c['n']} msgs em {_fmt(c['duracao_s'])} s -> {_fmt(c['taxa_hz'], 2)} Hz medios;"
              f" intervalo mediana {_fmt(c['intervalo_mediana_ms'])} ms, p95 "
              f"{_fmt(c['intervalo_p95_ms'])} ms; {c['intervalos_acima_100ms']} intervalos > 100 ms")
    print("\nOffset recv (PC) - header.stamp (Pi), em ms (chrony: esperado |mediana| < 5):")
    if not r["offset_ms"]:
        print("  sem /odom nem /scan no bag")
    for nome, e in r["offset_ms"].items():
        if e.get("n", 0) == 0:
            continue
        print(f"  {nome:<8} n={e['n']:<7} mediana {_fmt(e['mediana'], 2)}  p05 {_fmt(e['p05'], 2)}"
              f"  p95 {_fmt(e['p95'], 2)}  min {_fmt(e['min'], 2)}  max {_fmt(e['max'], 2)}")
    print("\nCaranguejo por goal (fracao do tempo em movimento com |vy| > |vx| em "
          f"{TOPICO_CMD}):")
    if not r["goals"]:
        print("  sem comandos de velocidade no bag")
    for g in r["goals"]:
        print(f"  goal {g['goal']:<11} {g['status']:<10} {_fmt(g['duracao_s']):>7} s"
              f"  mov {_fmt(g['tempo_movimento_s']):>7} s  lateral {_fmt(g['fracao_lateral'] * 100):>5} %"
              f"  v media {_fmt(g['vel_media_m_s'], 3)} m/s  ({g['n_cmd']} cmds)")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Resumo offline de um bag de corrida do Caramelo.")
    p.add_argument("bag", help="pasta do bag (com metadata.yaml)")
    p.add_argument("--json", action="store_true", help="imprime o resumo em JSON")
    args = p.parse_args(argv)
    pasta = Path(args.bag).expanduser()
    if not (pasta / "metadata.yaml").exists():
        print(f"Erro: {pasta} nao tem metadata.yaml (bag nao fechou? tente "
              f"`ros2 bag reindex {pasta} mcap`).", file=sys.stderr)
        return 1
    r = resumir(pasta)
    if args.json:
        print(json.dumps(r, indent=1, ensure_ascii=True, default=str))
    else:
        imprimir(r)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
