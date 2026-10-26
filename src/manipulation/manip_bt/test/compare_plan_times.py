#!/usr/bin/env python3
"""Compara o tempo estimado guloso x otimizado do task_planner nas tasks oficiais.

Roda o BINARIO task_planner (sem ROS no ar) sobre missions/tasks geradas/*.yaml
com a matriz de distancias do mapa, duas vezes por task: com
CARAMELO_PLANNER_MODE=greedy (varias sementes, porque o guloso sorteia
empates: mostra min/max) e no modo normal (busca por tempo). Imprime uma
tabela Markdown para colar em docs/planejador_por_tempo.md.

Como rodar (da raiz do workspace, com o install sourceado ou apontando o
binario do build):
    TASK_PLANNER_BIN=build/manip_bt/task_planner python3 \
        src/manipulation/manip_bt/test/compare_plan_times.py --map-name arenalegal_2026
"""
import argparse
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

import yaml

HERE = pathlib.Path(__file__).resolve().parent
WS_ROOT = HERE.parents[3]
TASKS_DIR = WS_ROOT / "missions" / "tasks geradas"
TAGS_YAML = WS_ROOT / "src" / "manipulation" / "manip_bringup" / "config" / "tags_36h11.yaml"
MAPS_DIR = WS_ROOT / "src" / "caramelo_mapping" / "maps"

RE_GREEDY = re.compile(
    r"Tempo estimado guloso: (\d+) s \((\d+) visita\(s\), (\d+) troca\(s\), ([\d.]+) m\)")
RE_OPT = re.compile(
    r"otimizado: (\d+) s \((\d+) visita\(s\), (\d+) troca\(s\), ([\d.]+) m\)( = guloso)? "
    r"\[([^\]]*)\]")


def find_planner():
    env = os.environ.get("TASK_PLANNER_BIN")
    if env and pathlib.Path(env).is_file():
        return env
    for candidate in (WS_ROOT / "build" / "manip_bt" / "task_planner",
                      WS_ROOT / "install" / "manip_bt" / "lib" / "manip_bt" / "task_planner"):
        if candidate.is_file():
            return str(candidate)
    found = shutil.which("task_planner")
    if found:
        return found
    sys.exit("task_planner nao encontrado: exporte TASK_PLANNER_BIN ou builde o manip_bt")


def run_planner(planner, task, ws_map, out, env_extra):
    env = dict(os.environ)
    env.update(env_extra)
    proc = subprocess.run([planner, str(task), str(out), str(TAGS_YAML), str(ws_map)],
                          capture_output=True, text=True, env=env, timeout=120)
    if proc.returncode != 0:
        raise RuntimeError(f"{task.name}: task_planner rc={proc.returncode}\n{proc.stderr}")
    return proc.stdout + proc.stderr


def gotos(out):
    with open(out, encoding="utf-8") as fh:
        actions = yaml.safe_load(fh)["actions"]
    return " > ".join(a["ws"] for a in actions if a["kind"] == "goto")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--map-name", default="arenalegal_2026")
    parser.add_argument("--seeds", type=int, default=20, help="rodadas do guloso (sementes 1..N)")
    parser.add_argument("--tasks", nargs="*", default=None, help="arquivos (default: todos)")
    args = parser.parse_args(argv)

    planner = find_planner()
    ws_map = MAPS_DIR / args.map_name / "ws_table_mapping.yaml"
    if not (MAPS_DIR / args.map_name / "dock_distances.yaml").exists():
        sys.exit(f"sem dock_distances.yaml em {MAPS_DIR / args.map_name}: rode compute_dock_distances")
    tasks = [pathlib.Path(t) for t in args.tasks] if args.tasks else sorted(TASKS_DIR.glob("*.yaml"))

    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        tmp = pathlib.Path(tmp)
        for task in tasks:
            greedy = []
            for seed in range(1, args.seeds + 1):
                out = tmp / f"{task.stem}_g{seed}.yaml"
                log = run_planner(planner, task, ws_map, out,
                                  {"CARAMELO_PLANNER_MODE": "greedy", "CARAMELO_PLANNER_SEED": str(seed)})
                m = RE_GREEDY.search(log)
                if m:
                    greedy.append((int(m.group(1)), int(m.group(3)), float(m.group(4)), gotos(out)))
            out = tmp / f"{task.stem}_opt.yaml"
            log = run_planner(planner, task, ws_map, out, {"CARAMELO_PLANNER_MODE": "auto"})
            m = RE_OPT.search(log)
            if not greedy or not m:
                rows.append((task.stem, "-", "-", "-", "-", "-", "sem transporte (AMT) ou sem matriz"))
                continue
            g_times = [g[0] for g in greedy]
            g_best = min(greedy, key=lambda g: g[0])
            opt = (int(m.group(1)), int(m.group(3)), float(m.group(4)), m.group(6))
            rows.append((task.stem, f"{min(g_times)}-{max(g_times)}", f"{g_best[1]} / {g_best[2]:.1f}",
                         str(opt[0]), f"{opt[1]} / {opt[2]:.1f}",
                         f"{100.0 * (max(g_times) - opt[0]) / max(g_times):.0f}% / "
                         f"{100.0 * (min(g_times) - opt[0]) / min(g_times):.0f}%",
                         f"{opt[3]}; rota: {gotos(out)}"))

    print("| Task | Guloso (s) min-max | Guloso: trocas / m (melhor) | Otimizado (s) | "
          "Otimizado: trocas / m | Ganho vs pior / melhor guloso | Busca; rota otimizada |")
    print("|---|---|---|---|---|---|---|")
    for r in rows:
        print("| " + " | ".join(r) + " |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
