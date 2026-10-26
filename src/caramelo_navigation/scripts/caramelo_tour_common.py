#!/usr/bin/env python3
"""Logica PURA do tour_docks (sem ROS): ordem de visita dos docks, leitura
das entradas do docking.yaml/service_areas.yaml, pose de staging, erro da
pose final no frame do dock, frases da voz e relatorio (tabela no terminal
e markdown). Importado por tour_docks.py (instalado em lib/caramelo_navigation)
e testado em test/test_tour_docks.py sem o ROS no ar."""

from __future__ import annotations

import datetime
import math
import re
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Ordem de visita por tipo (rulebook): WS, depois SH, PP, RT; ST (area de
# empilhamento, formato 2026) por ultimo. START nunca entra (e onde o robo
# esta); FINISH so entra como ultimo ponto, via `finish`.
KIND_ORDER = ("WS", "SH", "PP", "RT", "ST")
_DOCK_RE = re.compile(r"^([A-Z]+)([0-9]*)$")
# Mesmo fallback do MissionMapConfig (manip_bt) e do dock_align_node quando o
# dock_plugins nao tem staging_x_offset para o type do dock.
DEFAULT_STAGING_OFFSET = -0.45

Pose2D = Tuple[float, float, float]


# ----------------------------------------------------------------- ordem
def split_dock_id(dock_id: str) -> Tuple[str, int]:
    """'WS10' -> ('WS', 10); 'START' -> ('START', 0)."""
    match = _DOCK_RE.match((dock_id or "").strip().upper())
    if not match:
        raise ValueError(f"dock_id invalido: {dock_id!r}")
    kind, number = match.group(1), match.group(2)
    return kind, int(number) if number else 0


def dock_sort_key(dock_id: str) -> Tuple[int, int, str]:
    """Chave de ordenacao: tipo (WS < SH < PP < RT < ST < outros) e numero
    NUMERICO (WS10 depois de WS9, nao antes de WS2)."""
    kind, number = split_dock_id(dock_id)
    rank = KIND_ORDER.index(kind) if kind in KIND_ORDER else len(KIND_ORDER)
    return rank, number, kind


def parse_dock_list(text: Optional[str]) -> List[str]:
    """'ws1, WS3,,' -> ['WS1', 'WS3']; None/'' -> []."""
    if not text:
        return []
    return [item.strip().upper() for item in text.split(",") if item.strip()]


def plan_tour(dock_ids: Iterable[str], only: Optional[Sequence[str]] = None,
              skip: Optional[Sequence[str]] = None, finish: Optional[str] = "FINISH") -> List[str]:
    """Ordem de visita: WS1..WSn, SH*, PP*, RT* (numerica dentro de cada
    tipo) e `finish` por ultimo ('' = nao vai a lugar nenhum no fim).
    START nunca e visitado. `only` restringe (mantendo a ordem numerica) e
    `skip` remove; ids fora de `dock_ids` em only/skip/finish dao ValueError."""
    known = {str(d).strip().upper() for d in dock_ids}
    finish = (finish or "").strip().upper()
    only = [d.strip().upper() for d in (only or [])]
    skip = [d.strip().upper() for d in (skip or [])]
    for label, ids in (("--only", only), ("--skip", skip)):
        unknown = [d for d in ids if d not in known]
        if unknown:
            raise ValueError(
                f"{label}: dock(s) fora do docking.yaml: {', '.join(unknown)}")
    if finish and finish not in known:
        raise ValueError(f"--finish: dock '{finish}' nao esta no docking.yaml")
    candidates = [d for d in known if split_dock_id(d)[0] in KIND_ORDER]
    if only:
        candidates = [d for d in candidates if d in only]
    candidates = [d for d in candidates if d not in skip and d != finish]
    ordered = sorted(candidates, key=dock_sort_key)
    if finish:
        ordered.append(finish)
    return ordered


# ------------------------------------------------------------- geometria
def normalize_angle(angle: float) -> float:
    """Leva o angulo para [-pi, pi] (robusto perto de +-pi e para |a| > 2pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_error_in_dock_frame(robot: Pose2D, dock: Pose2D) -> Pose2D:
    """Onde o robo PAROU em relacao ao ponto do dock, expresso no frame do
    dock: (long, lat, yaw).
      long + = passou do ponto (mais perto da mesa); - = ficou antes;
      lat  + = a esquerda do ponto (eixo +y do dock); - = a direita;
      yaw  + = girado para a esquerda (anti-horario) em relacao ao yaw do
               dock, normalizado em [-pi, pi].
    (Convencao oposta ao amcl_errors do dock_align_node, que e' alvo - robo
    no frame do ROBO; aqui e' robo - alvo no frame do DOCK, que e' o que o
    operador quer saber para corrigir a pose gravada.)"""
    rx, ry, ryaw = robot
    dx, dy, dyaw = dock
    ex, ey = rx - dx, ry - dy
    c, s = math.cos(dyaw), math.sin(dyaw)
    longitudinal = c * ex + s * ey
    lateral = -s * ex + c * ey
    return longitudinal, lateral, normalize_angle(ryaw - dyaw)


def staging_pose(dock: Pose2D, staging_x_offset: float,
                 staging_yaw_offset: float = 0.0) -> Pose2D:
    """Mesma conta do SimpleNonChargingDock::getStagingPose e do
    MissionMapConfig::stagingPose: desloca ao longo do yaw do dock (offset
    negativo = para tras) e soma o yaw offset."""
    x, y, yaw = dock
    return (x + staging_x_offset * math.cos(yaw),
            y + staging_x_offset * math.sin(yaw),
            yaw + staging_yaw_offset)


# ---------------------------------------------------------------- docks
@dataclass
class DockEntry:
    """Uma entrada de docking.yaml + o use_docking do service_areas.yaml."""
    dock_id: str
    dock_type: str
    pose: Pose2D
    frame: str = "map"
    staging_x_offset: float = DEFAULT_STAGING_OFFSET
    staging_yaw_offset: float = 0.0
    staging_from_yaml: bool = False
    use_docking: bool = True

    @property
    def pose_is_placeholder(self) -> bool:
        return all(abs(v) < 1e-9 for v in self.pose)

    @property
    def staging(self) -> Pose2D:
        return staging_pose(self.pose, self.staging_x_offset, self.staging_yaw_offset)


def default_use_docking(dock_id: str) -> bool:
    """Mesmo fallback do MissionMapConfig::useDocking: tudo menos START/FINISH."""
    return dock_id not in ("START", "FINISH")


def load_dock_entries(docking_data: dict,
                      service_areas_data: Optional[dict] = None) -> Dict[str, DockEntry]:
    """Monta as entradas a partir dos dicts do docking.yaml (docks +
    dock_plugins: staging pelo type, como o MissionMapConfig) e, se houver,
    do service_areas.yaml (service_areas.<id>.docking.use_docking)."""
    plugins = docking_data.get("dock_plugins") or {}
    if not isinstance(plugins, dict):
        plugins = {}
    areas = (service_areas_data or {}).get("service_areas") or {}
    if not isinstance(areas, dict):
        areas = {}
    entries: Dict[str, DockEntry] = {}
    for raw_id, entry in (docking_data.get("docks") or {}).items():
        dock_id = str(raw_id).strip().upper()
        if not isinstance(entry, dict):
            raise ValueError(f"dock '{dock_id}': entrada do docking.yaml nao e um dicionario")
        pose = entry.get("pose") or [0.0, 0.0, 0.0]
        if len(pose) < 3:
            raise ValueError(f"dock '{dock_id}': pose precisa de [x, y, yaw]")
        dock_type = str(entry.get("type") or "")
        plugin = plugins.get(dock_type)
        item = DockEntry(
            dock_id=dock_id,
            dock_type=dock_type,
            pose=(float(pose[0]), float(pose[1]), float(pose[2])),
            frame=str(entry.get("frame") or "map"),
            use_docking=default_use_docking(dock_id),
        )
        if isinstance(plugin, dict) and "staging_x_offset" in plugin:
            item.staging_x_offset = float(plugin["staging_x_offset"])
            item.staging_yaw_offset = float(plugin.get("staging_yaw_offset", 0.0))
            item.staging_from_yaml = True
        area = areas.get(dock_id) or areas.get(raw_id)
        docking_cfg = area.get("docking") if isinstance(area, dict) else None
        if isinstance(docking_cfg, dict) and "use_docking" in docking_cfg:
            item.use_docking = bool(docking_cfg["use_docking"])
        entries[dock_id] = item
    return entries


# ------------------------------------------------------------ resultados
# Status de um dock no relatorio.
STATUS_OK = "ok"
STATUS_WARN = "ok_aviso"      # chegou, mas o align ressalvou o yaw
STATUS_FAIL = "falhou"
STATUS_CANCELED = "cancelado"
STATUS_SKIPPED = "pulado"

# Fluxos de chegada (o mesmo nome aparece no relatorio).
FLOW_STAGING_ALIGN = "staging+align"    # GoToWS padrao: Nav2 ate a staging, align faz o approach
FLOW_OPENNAV_ALIGN = "opennav+align"    # rollback use_opennav_approach=true
FLOW_OPENNAV_ONLY = "opennav"           # idem com skip_align (--no-align)
FLOW_NAV_ONLY = "nav_pura"              # use_docking=false (START/FINISH)


@dataclass
class DockReport:
    """Linha do relatorio: o que aconteceu num dock."""
    dock_id: str
    flow: str = ""
    status: str = STATUS_SKIPPED
    failed_stage: str = ""
    message: str = ""
    t_undock: Optional[float] = None
    t_nav: Optional[float] = None       # staging nav, /dock_robot ou nav pura
    t_align: Optional[float] = None
    err_long: Optional[float] = None    # m, frame do dock (pose_error_in_dock_frame)
    err_lat: Optional[float] = None
    err_yaw: Optional[float] = None     # rad
    robot_pose: Optional[Pose2D] = None
    staging_err: Optional[Pose2D] = None   # erro na staging apos o Nav2 (long, lat, yaw)
    align_message: str = ""
    align_x: Optional[float] = None     # final_x_error reportado pelo align (front - stop)
    align_y: Optional[float] = None
    align_yaw: Optional[float] = None   # yaw da face pelo LiDAR (ou AMCL se yaw_nao_medido)

    @property
    def reached(self) -> bool:
        return self.status in (STATUS_OK, STATUS_WARN)

    @property
    def t_total(self) -> float:
        return sum(t for t in (self.t_undock, self.t_nav, self.t_align) if t is not None)

    @property
    def err_xy(self) -> Optional[float]:
        if self.err_long is None or self.err_lat is None:
            return None
        return math.hypot(self.err_long, self.err_lat)


def align_status(message: str) -> str:
    """Sucesso do align: 'ok', ou 'ok_aviso' quando a message vem com a
    ressalva do dock_align_node ('yaw_fora_da_tolerancia (x graus)' ou
    'yaw_nao_medido' na frente)."""
    text = (message or "").strip()
    if text.startswith("yaw_fora_da_tolerancia") or text.startswith("yaw_nao_medido"):
        return STATUS_WARN
    return STATUS_OK


# ------------------------------------------------------------------- voz
def speech_ready(dock_id: str, err_long: Optional[float], err_lat: Optional[float],
                 err_yaw: Optional[float]) -> str:
    """'!Dock WS3 pronto, erro 3 centimetros e 2 graus' (ASCII: o speech_node
    devolve os acentos). Sem medida de pose: '!Dock WS3 pronto, sem medida da pose'."""
    if err_long is None or err_lat is None or err_yaw is None:
        return f"!Dock {dock_id} pronto, sem medida da pose"
    cm = int(round(math.hypot(err_long, err_lat) * 100.0))
    deg = int(round(abs(math.degrees(err_yaw))))
    cm_txt = "1 centimetro" if cm == 1 else f"{cm} centimetros"
    deg_txt = "1 grau" if deg == 1 else f"{deg} graus"
    return f"!Dock {dock_id} pronto, erro {cm_txt} e {deg_txt}"


def speech_failed(dock_id: str, reason: str) -> str:
    return f"!Dock {dock_id} falhou: {reason}"


def speech_summary(n_reached: int, n_total: int) -> str:
    return f"!Tour terminado: {n_reached} de {n_total} docks ok"


# ------------------------------------------------------------- relatorio
def report_filename(now: datetime.datetime) -> str:
    """tour_<AAAA-MM-DD_HHMM>.md"""
    return f"tour_{now:%Y-%m-%d_%H%M}.md"


def _fmt_s(value: Optional[float]) -> str:
    return "-" if value is None else f"{value:.1f}"


def _fmt_cm(value: Optional[float]) -> str:
    return "-" if value is None else f"{value * 100.0:+.1f}"


def _fmt_deg(value: Optional[float]) -> str:
    return "-" if value is None else f"{math.degrees(value):+.1f}"


def _fmt_pose(pose: Optional[Pose2D]) -> str:
    if pose is None:
        return "-"
    return f"({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):+.1f} graus)"


def _fmt_err3(err: Optional[Pose2D]) -> str:
    if err is None:
        return "-"
    return f"{_fmt_cm(err[0])} / {_fmt_cm(err[1])} cm, {_fmt_deg(err[2])} graus"


def summarize(reports: Sequence[DockReport]) -> dict:
    """Totais do tour: contagens por status, tempo total e piores erros."""
    counts = {s: 0 for s in (STATUS_OK, STATUS_WARN, STATUS_FAIL, STATUS_CANCELED, STATUS_SKIPPED)}
    for rep in reports:
        counts[rep.status] = counts.get(rep.status, 0) + 1
    reached = [r for r in reports if r.reached]
    worst_xy = max((r for r in reached if r.err_xy is not None),
                   key=lambda r: r.err_xy, default=None)
    worst_yaw = max((r for r in reached if r.err_yaw is not None),
                    key=lambda r: abs(r.err_yaw), default=None)
    return {
        "n_total": len(reports),
        "n_reached": len(reached),
        "counts": counts,
        "total_s": sum(r.t_total for r in reports),
        "worst_xy": (worst_xy.dock_id, worst_xy.err_xy) if worst_xy else None,
        "worst_yaw": (worst_yaw.dock_id, worst_yaw.err_yaw) if worst_yaw else None,
    }


_COLUMNS = ("Dock", "Status", "Undock(s)", "Nav(s)", "Align(s)", "Long(cm)",
            "Lat(cm)", "Yaw(graus)", "Mensagem")


def _row(rep: DockReport) -> List[str]:
    return [rep.dock_id, rep.status, _fmt_s(rep.t_undock), _fmt_s(rep.t_nav),
            _fmt_s(rep.t_align), _fmt_cm(rep.err_long), _fmt_cm(rep.err_lat),
            _fmt_deg(rep.err_yaw), rep.message or ""]


def format_table(reports: Sequence[DockReport]) -> str:
    """Tabela de largura fixa para o terminal (uma linha por dock + totais)."""
    rows = [list(_COLUMNS)] + [_row(r) for r in reports]
    widths = [max(len(str(row[i])) for row in rows) for i in range(len(_COLUMNS))]
    lines = []
    for n, row in enumerate(rows):
        cells = []
        for i, cell in enumerate(row):
            # Numeros alinhados a direita, texto a esquerda.
            cells.append(str(cell).rjust(widths[i]) if 2 <= i <= 7 else str(cell).ljust(widths[i]))
        lines.append("  ".join(cells).rstrip())
        if n == 0:
            lines.append("  ".join("-" * w for w in widths))
    summary = summarize(reports)
    lines.append("")
    lines.append(summary_line(summary))
    return "\n".join(lines)


def summary_line(summary: dict) -> str:
    counts = summary["counts"]
    parts = [f"{summary['n_reached']} de {summary['n_total']} docks ok"]
    if counts.get(STATUS_WARN):
        parts.append(f"{counts[STATUS_WARN]} com aviso")
    if counts.get(STATUS_FAIL):
        parts.append(f"{counts[STATUS_FAIL]} falhou")
    if counts.get(STATUS_CANCELED):
        parts.append(f"{counts[STATUS_CANCELED]} cancelado")
    if counts.get(STATUS_SKIPPED):
        parts.append(f"{counts[STATUS_SKIPPED]} pulado")
    text = "Totais: " + ", ".join(parts) + f"; tempo em movimento {summary['total_s'] / 60.0:.1f} min"
    if summary["worst_xy"]:
        dock_id, err = summary["worst_xy"]
        text += f"; maior erro XY: {dock_id} ({err * 100.0:.1f} cm)"
    if summary["worst_yaw"]:
        dock_id, err = summary["worst_yaw"]
        text += f"; maior erro de yaw: {dock_id} ({math.degrees(err):+.1f} graus)"
    return text + "."


def format_report_md(reports: Sequence[DockReport], *, map_name: str, map_folder: str,
                     started: datetime.datetime, finished: datetime.datetime,
                     command: str, flow_text: str) -> str:
    """Relatorio markdown: cabecalho, tabela por dock, totais, detalhes e
    legenda (como ler os erros)."""
    summary = summarize(reports)
    minutes = max(0.0, (finished - started).total_seconds()) / 60.0
    out = [
        f"# Tour dos docks - {map_name}",
        "",
        f"- Data: {started:%Y-%m-%d %H:%M} -> {finished:%H:%M} ({minutes:.0f} min)",
        f"- Mapa: `{map_folder}`",
        f"- Fluxo: {flow_text}",
        f"- Comando: `{command}`",
        f"- {summary_line(summary)}",
        "",
        "| " + " | ".join(_COLUMNS) + " |",
        "|" + "|".join("---" for _ in _COLUMNS) + "|",
    ]
    for rep in reports:
        cells = [c.replace("|", "/") for c in _row(rep)]
        out.append("| " + " | ".join(cells) + " |")
    out += [
        "",
        "## Detalhes",
        "",
        "| Dock | Fluxo | Pose final (x, y, yaw) | Staging: long / lat, yaw | Align reportou: x / y cm, yaw_face | Etapa que falhou |",
        "|---|---|---|---|---|---|",
    ]
    for rep in reports:
        align = "-"
        if rep.align_x is not None or rep.align_y is not None or rep.align_yaw is not None:
            align = (f"{_fmt_cm(rep.align_x)} / {_fmt_cm(rep.align_y)} cm, "
                     f"{_fmt_deg(rep.align_yaw)} graus")
            if rep.align_message:
                align += f" ({rep.align_message.replace('|', '/')})"
        out.append(
            f"| {rep.dock_id} | {rep.flow or '-'} | {_fmt_pose(rep.robot_pose)} | "
            f"{_fmt_err3(rep.staging_err)} | {align} | {rep.failed_stage or '-'} |")
    out += [
        "",
        "## Como ler",
        "",
        "- Erros no frame do DOCK, da pose final do robo (TF map -> base_footprint, AMCL) "
        "contra a `pose` do docking.yaml: Long + = passou do ponto (mais perto da mesa); "
        "Lat + = parou a esquerda do ponto; Yaw + = girado para a esquerda (anti-horario). "
        "Erro sistematico no mesmo sentido em todas as visitas = pose do dock mal gravada; "
        "erro que varia = Nav2/AMCL/align instavel naquele ponto.",
        "- Staging: o mesmo erro, medido na pre-docking (pose do dock + staging_x_offset) "
        "logo apos o Nav2, antes do align.",
        "- Align reportou: `final_x_error` = front - stop pelo LiDAR (+ = longe da mesa), "
        "`final_y_error` = lateral pelo AMCL, `final_yaw_error` = yaw da face pelo LiDAR "
        "(ou pelo AMCL quando `yaw_nao_medido`). Status `ok_aviso` = alinhou, mas com "
        "`yaw_fora_da_tolerancia` ou `yaw_nao_medido` na mensagem.",
        "- Tempos: Undock = /undock_robot; Nav = Nav2 ate a staging (ou /dock_robot, ou "
        "navegacao pura no FINISH); Align = align_to_dock (approach + alinhamento).",
    ]
    return "\n".join(out) + "\n"
