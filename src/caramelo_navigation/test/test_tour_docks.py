"""Testes da logica PURA do tour_docks (caramelo_tour_common) e da espera de
action servers do pre-flight (caramelo_docking_common), sem ROS ativo.

Carrega os modulos direto de scripts/ (mesmo truque de test_nudge_math.py).
caramelo_tour_common so usa a stdlib; caramelo_docking_common puxa yaml e
ament_index_python (ambiente sourceado), mas nao inicializa nada.
"""
import datetime
import importlib.util
import io
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, "..", "scripts")


def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(SCRIPTS, name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    # Registrado em sys.modules ANTES de executar: os dataclasses com
    # `from __future__ import annotations` resolvem os tipos pelo modulo.
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


tc = _load("caramelo_tour_common")
dc = _load("caramelo_docking_common")

# Docks do arenalegal_2026 + extras para a ordenacao (WS10, RT2, ST1).
ARENA = ["START", "FINISH", "WS1", "WS2", "WS3", "WS4", "WS5", "WS6", "SH1", "PP1"]
EXTRA = ARENA + ["WS10", "WS7", "RT1", "RT2", "PP2", "SH2", "ST1"]


# ----------------------------------------------------------------- ordem
def test_ordem_numerica_ws10_depois_de_ws9():
    ids = ["WS10", "WS9", "WS1", "WS2"]
    assert tc.plan_tour(ids, finish="") == ["WS1", "WS2", "WS9", "WS10"]


def test_ordem_por_tipo_e_finish_por_ultimo_sem_start():
    plan = tc.plan_tour(EXTRA)
    assert plan == ["WS1", "WS2", "WS3", "WS4", "WS5", "WS6", "WS7", "WS10",
                    "SH1", "SH2", "PP1", "PP2", "RT1", "RT2", "ST1", "FINISH"]
    assert "START" not in plan


def test_ordem_do_mapa_real():
    assert tc.plan_tour(ARENA) == ["WS1", "WS2", "WS3", "WS4", "WS5", "WS6", "SH1", "PP1", "FINISH"]


def test_finish_vazio_nao_vai_a_lugar_nenhum():
    plan = tc.plan_tour(ARENA, finish="")
    assert plan[-1] == "PP1" and "FINISH" not in plan
    assert tc.plan_tour(ARENA, finish=None)[-1] == "PP1"


def test_only_mantem_a_ordem_numerica():
    assert tc.plan_tour(ARENA, only=["ws3", "WS1"]) == ["WS1", "WS3", "FINISH"]
    assert tc.plan_tour(ARENA, only=["SH1"], finish="") == ["SH1"]


def test_skip_remove_e_combina_com_only():
    assert tc.plan_tour(ARENA, skip=["SH1", "ws2"]) == ["WS1", "WS3", "WS4", "WS5", "WS6", "PP1", "FINISH"]
    assert tc.plan_tour(ARENA, only=["WS1", "WS2"], skip=["WS2"]) == ["WS1", "FINISH"]


def test_finish_pode_ser_outro_dock_e_sai_do_meio():
    assert tc.plan_tour(ARENA, finish="WS2") == ["WS1", "WS3", "WS4", "WS5", "WS6", "SH1", "PP1", "WS2"]


@pytest.mark.parametrize("kwargs", [
    {"only": ["WS9"]},
    {"skip": ["RT1"]},
    {"finish": "FIM"},
])
def test_ids_desconhecidos_dao_erro(kwargs):
    with pytest.raises(ValueError):
        tc.plan_tour(ARENA, **kwargs)


def test_only_start_nao_visita_start():
    # START nunca e visitado, mesmo pedido: sobra so o FINISH.
    assert tc.plan_tour(ARENA, only=["START"]) == ["FINISH"]


def test_parse_dock_list():
    assert tc.parse_dock_list("ws1, WS3,,sh1 ") == ["WS1", "WS3", "SH1"]
    assert tc.parse_dock_list("") == []
    assert tc.parse_dock_list(None) == []


def test_split_dock_id():
    assert tc.split_dock_id("ws10") == ("WS", 10)
    assert tc.split_dock_id("START") == ("START", 0)
    with pytest.raises(ValueError):
        tc.split_dock_id("WS-1")


# ------------------------------------------------------------- geometria
@pytest.mark.parametrize("angle, expected", [
    (0.0, 0.0),
    (math.pi, math.pi),
    (-math.pi, -math.pi),
    (math.pi + 0.1, -math.pi + 0.1),
    (-math.pi - 0.1, math.pi - 0.1),
    (2 * math.pi, 0.0),
    (3 * math.pi, math.pi),
    (6.186, 6.186 - 2 * math.pi),          # PP1: 3.10 - (-3.086)
    (-6.2, -6.2 + 2 * math.pi),
])
def test_normalize_angle(angle, expected):
    got = tc.normalize_angle(angle)
    assert got == pytest.approx(expected, abs=1e-9)
    assert -math.pi <= got <= math.pi


# dock yaw (graus), deslocamento do robo no frame do MAPA (dx, dy), erro
# esperado (long, lat) no frame do dock. Em cada yaw a "frente" do dock
# aponta para um lado diferente do mapa; a projecao tem de devolver a
# mesma leitura.
@pytest.mark.parametrize("yaw_deg, dx, dy, lon, lat", [
    (0.0, 0.05, 0.0, 0.05, 0.0),        # frente = +x: passou 5 cm
    (0.0, 0.0, 0.02, 0.0, 0.02),        # esquerda = +y
    (90.0, 0.0, 0.05, 0.05, 0.0),       # frente = +y
    (90.0, -0.02, 0.0, 0.0, 0.02),      # esquerda = -x
    (180.0, -0.05, 0.0, 0.05, 0.0),     # frente = -x
    (180.0, 0.0, -0.02, 0.0, 0.02),     # esquerda = -y
    (-90.0, 0.0, -0.05, 0.05, 0.0),     # frente = -y
    (-90.0, 0.02, 0.0, 0.0, 0.02),      # esquerda = +x
    (0.0, -0.03, -0.01, -0.03, -0.01),  # ficou antes e a direita
])
def test_erro_no_frame_do_dock(yaw_deg, dx, dy, lon, lat):
    dock = (1.3, -0.7, math.radians(yaw_deg))
    robot = (dock[0] + dx, dock[1] + dy, dock[2])
    got = tc.pose_error_in_dock_frame(robot, dock)
    assert got[0] == pytest.approx(lon, abs=1e-9)
    assert got[1] == pytest.approx(lat, abs=1e-9)
    assert got[2] == pytest.approx(0.0, abs=1e-9)


def test_erro_decomposicao_em_yaw_arbitrario():
    """Robo em dock + a*x_hat + b*y_hat (eixos do dock) devolve (a, b)."""
    yaw = math.radians(37.0)
    a, b = 0.07, -0.22
    dock = (-3.0, 5.5, yaw)
    robot = (dock[0] + a * math.cos(yaw) - b * math.sin(yaw),
             dock[1] + a * math.sin(yaw) + b * math.cos(yaw), yaw)
    lon, lat, _ = tc.pose_error_in_dock_frame(robot, dock)
    assert lon == pytest.approx(a, abs=1e-9)
    assert lat == pytest.approx(b, abs=1e-9)


@pytest.mark.parametrize("dock_yaw, robot_yaw, expected_deg", [
    (3.1, -3.1, math.degrees(2 * math.pi - 6.2)),     # dos dois lados de +-pi: 4,8 graus, nao -355
    (-3.1, 3.1, -math.degrees(2 * math.pi - 6.2)),
    (-3.086312, 3.10, math.degrees(6.186312 - 2 * math.pi)),   # PP1 do arenalegal_2026
    (3.129658, -3.12, math.degrees(-6.249658 + 2 * math.pi)),  # WS4
    (math.pi, -math.pi, 0.0),
    (0.5, 0.6, math.degrees(0.1)),
    (0.6, 0.5, -math.degrees(0.1)),
])
def test_erro_de_yaw_perto_de_pi(dock_yaw, robot_yaw, expected_deg):
    _, _, eyaw = tc.pose_error_in_dock_frame((0.0, 0.0, robot_yaw), (0.0, 0.0, dock_yaw))
    assert math.degrees(eyaw) == pytest.approx(expected_deg, abs=1e-6)
    assert abs(eyaw) <= math.pi


@pytest.mark.parametrize("yaw_deg, offset, expected", [
    (0.0, -0.25, (0.75, 0.0)),
    (90.0, -0.25, (1.0, -0.25)),
    (180.0, -0.45, (1.45, 0.0)),
    (-90.0, -0.4, (1.0, 0.4)),
])
def test_staging_pose(yaw_deg, offset, expected):
    sx, sy, syaw = tc.staging_pose((1.0, 0.0, math.radians(yaw_deg)), offset)
    assert (sx, sy) == pytest.approx(expected, abs=1e-9)
    assert syaw == pytest.approx(math.radians(yaw_deg))
    assert tc.staging_pose((1.0, 0.0, 0.0), -0.25, 0.1)[2] == pytest.approx(0.1)


def test_staging_fica_atras_do_dock_e_no_eixo():
    """Da staging ao dock o erro longitudinal e' exatamente -offset, lateral 0."""
    dock = (0.7762, -0.4228, 0.086555)  # WS1
    staging = tc.staging_pose(dock, -0.25)
    lon, lat, eyaw = tc.pose_error_in_dock_frame(staging, dock)
    assert lon == pytest.approx(-0.25, abs=1e-9)
    assert lat == pytest.approx(0.0, abs=1e-9)
    assert eyaw == pytest.approx(0.0, abs=1e-9)


# ----------------------------------------------------------------- docks
DOCKING = {
    "dock_plugins": {
        "caramelo_ws1_dock": {"staging_x_offset": -0.25, "staging_yaw_offset": 0.0},
        "caramelo_finish_dock": {"staging_x_offset": -0.45},
    },
    "docks": {
        "START": {"type": "caramelo_start_dock", "frame": "map", "pose": [-0.8, 0.04, -0.008]},
        "FINISH": {"type": "caramelo_finish_dock", "frame": "map", "pose": [0.8, -6.9, -1.49]},
        "WS1": {"type": "caramelo_ws1_dock", "frame": "map", "pose": [0.78, -0.42, 0.087]},
        "ws2": {"type": "caramelo_sem_plugin", "pose": [0.0, 0.0, 0.0]},
    },
}
AREAS = {"service_areas": {
    "START": {"docking": {"use_docking": False}},
    "FINISH": {"docking": {"use_docking": False}},
    "WS1": {"docking": {"use_docking": True}},
}}


def test_load_dock_entries_staging_e_use_docking():
    entries = tc.load_dock_entries(DOCKING, AREAS)
    assert set(entries) == {"START", "FINISH", "WS1", "WS2"}
    ws1 = entries["WS1"]
    assert ws1.staging_x_offset == -0.25 and ws1.staging_from_yaml
    assert ws1.use_docking is True and not ws1.pose_is_placeholder
    assert ws1.staging == pytest.approx(tc.staging_pose(ws1.pose, -0.25))
    ws2 = entries["WS2"]  # id normalizado, sem plugin: fallback -0.45, placeholder
    assert ws2.staging_x_offset == tc.DEFAULT_STAGING_OFFSET and not ws2.staging_from_yaml
    assert ws2.pose_is_placeholder
    assert entries["FINISH"].use_docking is False


def test_load_dock_entries_sem_service_areas_usa_o_default():
    entries = tc.load_dock_entries(DOCKING, None)
    assert entries["START"].use_docking is False
    assert entries["FINISH"].use_docking is False
    assert entries["WS1"].use_docking is True
    # service_areas pode LIGAR o docking do FINISH (arena que encosta no fim).
    on = tc.load_dock_entries(DOCKING, {"service_areas": {"FINISH": {"docking": {"use_docking": True}}}})
    assert on["FINISH"].use_docking is True


def test_load_dock_entries_pose_incompleta_da_erro():
    with pytest.raises(ValueError):
        tc.load_dock_entries({"docks": {"WS1": {"pose": [1.0, 2.0]}}})


# ------------------------------------------------------------------- voz
def test_frases_da_voz():
    assert tc.speech_ready("WS3", 0.03, 0.0, math.radians(2.0)) == \
        "!Dock WS3 pronto, erro 3 centimetros e 2 graus"
    assert tc.speech_ready("WS3", 0.03, 0.04, math.radians(-2.4)) == \
        "!Dock WS3 pronto, erro 5 centimetros e 2 graus"      # hipotenusa, modulo
    assert tc.speech_ready("WS1", 0.01, 0.0, math.radians(1.0)) == \
        "!Dock WS1 pronto, erro 1 centimetro e 1 grau"           # singular
    assert tc.speech_ready("WS3", None, None, None) == "!Dock WS3 pronto, sem medida da pose"
    assert tc.speech_failed("WS3", "alinhamento timeout") == "!Dock WS3 falhou: alinhamento timeout"
    assert tc.speech_summary(7, 9) == "!Tour terminado: 7 de 9 docks ok"
    for text in (tc.speech_ready("WS3", 0.03, 0.0, 0.0), tc.speech_failed("SH1", "undock")):
        assert text.startswith("!") and text.isascii()


@pytest.mark.parametrize("message, status", [
    ("alinhado (tentativa 1/1)", "ok"),
    ("yaw_fora_da_tolerancia (-8.0 graus); alinhado (tentativa 1/1)", "ok_aviso"),
    ("yaw_nao_medido; alinhado (tentativa 1/1)", "ok_aviso"),
    ("", "ok"),
])
def test_status_do_align_pela_mensagem(message, status):
    assert tc.align_status(message) == status


# ------------------------------------------------------------- relatorio
def _reports():
    ok = tc.DockReport("WS1", flow=tc.FLOW_STAGING_ALIGN, status=tc.STATUS_OK,
                       message="alinhado (tentativa 1/1)", t_nav=25.2, t_align=41.0,
                       err_long=0.012, err_lat=-0.008, err_yaw=math.radians(1.5),
                       robot_pose=(0.79, -0.43, 0.1), staging_err=(-0.01, 0.02, 0.01),
                       align_x=0.004, align_y=-0.006, align_yaw=math.radians(0.8),
                       align_message="alinhado (tentativa 1/1)")
    warn = tc.DockReport("WS3", flow=tc.FLOW_STAGING_ALIGN, status=tc.STATUS_WARN,
                         message="yaw_fora_da_tolerancia (-8.0 graus); alinhado (tentativa 1/1)",
                         t_undock=4.0, t_nav=30.0, t_align=60.0,
                         err_long=0.041, err_lat=0.003, err_yaw=math.radians(-7.5))
    fail = tc.DockReport("SH1", flow=tc.FLOW_STAGING_ALIGN, status=tc.STATUS_FAIL,
                         failed_stage="alinhamento", message="alinhamento: falhou - a | b",
                         t_undock=3.5, t_nav=20.0, t_align=95.0)
    fin = tc.DockReport("FINISH", flow=tc.FLOW_NAV_ONLY, status=tc.STATUS_OK,
                        message="navegacao pura (sem dock)", t_nav=40.0,
                        err_long=-0.05, err_lat=0.02, err_yaw=math.radians(3.0))
    return [ok, warn, fail, fin]


def test_report_filename():
    now = datetime.datetime(2026, 10, 26, 18, 7)
    assert tc.report_filename(now) == "tour_2026-10-26_1807.md"


def test_dock_report_propriedades():
    ok, _warn, fail, _fin = _reports()
    assert ok.reached and ok.t_total == pytest.approx(66.2)
    assert ok.err_xy == pytest.approx(math.hypot(0.012, -0.008))
    assert not fail.reached and fail.err_xy is None
    assert fail.t_total == pytest.approx(118.5)


def test_summarize():
    s = tc.summarize(_reports())
    assert s["n_total"] == 4 and s["n_reached"] == 3
    assert s["counts"]["ok"] == 2 and s["counts"]["ok_aviso"] == 1 and s["counts"]["falhou"] == 1
    assert s["worst_xy"][0] == "FINISH"          # 5,4 cm > 4,1 cm (so' os que chegaram)
    assert s["worst_yaw"][0] == "WS3"            # -7,5 graus em modulo
    assert s["total_s"] == pytest.approx(66.2 + 94.0 + 118.5 + 40.0)
    assert tc.summarize([])["worst_xy"] is None


def test_format_table_terminal():
    text = tc.format_table(_reports())
    lines = text.splitlines()
    assert lines[0].startswith("Dock") and "Yaw(graus)" in lines[0]
    assert set(lines[1]) <= {"-", " "}
    assert any(line.startswith("WS1") and "+1.2" in line and "-0.8" in line and "+1.5" in line
               for line in lines)
    sh1 = next(line for line in lines if line.startswith("SH1"))
    assert sh1.count(" -") >= 3 and "falhou" in sh1     # erros nao medidos = "-"
    assert "95" in sh1
    assert lines[-1].startswith("Totais: 3 de 4 docks ok, 1 com aviso, 1 falhou")
    assert "maior erro XY: FINISH (5.4 cm)" in lines[-1]
    assert "maior erro de yaw: WS3 (-7.5 graus)" in lines[-1]


def test_format_report_md():
    started = datetime.datetime(2026, 10, 26, 18, 7)
    finished = started + datetime.timedelta(minutes=22)
    md = tc.format_report_md(
        _reports(), map_name="arenalegal_2026", map_folder="/x/maps/arenalegal_2026",
        started=started, finished=finished, command="tour_docks --map-name arenalegal_2026",
        flow_text="undock -> Nav2 ate a staging -> align_to_dock")
    assert md.startswith("# Tour dos docks - arenalegal_2026\n")
    assert "- Data: 2026-10-26 18:07 -> 18:29 (22 min)" in md
    assert "- Comando: `tour_docks --map-name arenalegal_2026`" in md
    assert "| WS1 | ok | - | 25.2 | 41.0 | +1.2 | -0.8 | +1.5 | alinhado (tentativa 1/1) |" in md
    assert "| FINISH | ok | - | 40.0 | - | -5.0 | +2.0 | +3.0 | navegacao pura (sem dock) |" in md
    assert "| SH1 | falhou | 3.5 | 20.0 | 95.0 | - | - | - | alinhamento: falhou - a / b |" in md  # '|' escapado
    assert "Totais: 3 de 4 docks ok, 1 com aviso, 1 falhou" in md
    assert "## Detalhes" in md and "## Como ler" in md
    assert "| WS1 | staging+align | (0.790, -0.430, +5.7 graus) | -1.0 / +2.0 cm, +0.6 graus | " \
           "+0.4 / -0.6 cm, +0.8 graus (alinhado (tentativa 1/1)) | - |" in md
    assert "| SH1 | staging+align | - | - | - | alinhamento |" in md
    assert "Long + = passou do ponto" in md
    assert md.endswith("\n")


def test_format_report_md_vazio_nao_quebra():
    now = datetime.datetime(2026, 1, 1, 0, 0)
    md = tc.format_report_md([], map_name="m", map_folder="/m", started=now, finished=now,
                             command="tour_docks", flow_text="f")
    assert "Totais: 0 de 0 docks ok" in md


# ------------------------------------------------------------ pre-flight
class _FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


def test_wait_for_action_servers_aparecem_aos_poucos():
    clock = _FakeClock()
    snapshots = iter([set(), {"/undock_robot"}, {"/undock_robot"},
                      {"/undock_robot", "/navigate_to_pose", "/align_to_dock"}])
    out, err = io.StringIO(), io.StringIO()
    ok = dc.wait_for_action_servers(
        ["/navigate_to_pose", "/undock_robot", "/align_to_dock"], 60.0,
        {"/align_to_dock": "align_to_dock (dock_align_node)"},
        list_actions=lambda: next(snapshots), sleep=clock.sleep, clock=clock, out=out, err=err)
    assert ok is True
    text = out.getvalue()
    assert "/undock_robot... ok (1s)" in text
    assert "align_to_dock (dock_align_node)... ok (3s)" in text
    assert text.rstrip().endswith("Pre-flight OK — todos os servidores no ar.")
    assert err.getvalue() == ""
    assert clock.t == pytest.approx(3.0)


def test_wait_for_action_servers_timeout_lista_o_que_falta():
    clock = _FakeClock()
    out, err = io.StringIO(), io.StringIO()
    ok = dc.wait_for_action_servers(
        ["/navigate_to_pose", "/dock_robot"], 25.0, {"/dock_robot": "docking server"},
        hint="suba o stack",
        list_actions=lambda: {"/navigate_to_pose"}, sleep=clock.sleep, clock=clock,
        out=out, err=err)
    assert ok is False
    assert "Pre-flight FALHOU apos 26s. Ainda faltam: docking server" in err.getvalue()
    assert "suba o stack" in err.getvalue()
    # "aguardando" a cada 10 s, nao a cada segundo.
    assert out.getvalue().count("aguardando docking server") == 3


def test_wait_for_action_servers_sem_nada_a_esperar():
    out = io.StringIO()
    assert dc.wait_for_action_servers([], 1.0, list_actions=lambda: set(), out=out) is True
