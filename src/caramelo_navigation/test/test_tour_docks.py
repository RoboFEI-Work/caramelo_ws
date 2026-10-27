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
    assert tc.speech_summary(2, 9, 1, 6) == "!Tour terminado: 2 de 9 docks ok, 1 com aviso, 6 reprovadas"
    assert tc.speech_summary(2, 9, 0, 1) == "!Tour terminado: 2 de 9 docks ok, 1 reprovada"
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
    assert s["n_ok"] == 2 and s["n_warn"] == 1 and s["n_fail_pose"] == 0 and s["fail_pose_ids"] == []
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
    assert lines[-1].startswith("Totais: 2 ok / 1 com aviso / 0 reprovadas de 4 docks, 1 falhou")
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
    assert "Totais: 2 ok / 1 com aviso / 0 reprovadas de 4 docks, 1 falhou" in md
    assert "## Detalhes" in md and "## Como ler" in md
    # yaw_face marcado pela fonte (lidar/amcl); parada "-" quando nao esperou.
    assert "| WS1 | staging+align | (0.790, -0.430, +5.7 graus) | -1.0 / +2.0 cm, +0.6 graus | - | " \
           "+0.4 / -0.6 cm, +0.8 graus (lidar) (alinhado (tentativa 1/1)) | - | - |" in md
    assert "| SH1 | staging+align | - | - | - | - | - | alinhamento |" in md
    assert "Long + = passou do ponto" in md
    # Legenda de sinais e das tolerancias usadas (2026-10-26, I6/I7).
    assert "convencao OPOSTA" in md and "yaw_face + = o robo precisa girar CCW" in md
    assert "|yaw| > 5.2 graus ou |lat| > 10 cm" in md
    assert md.endswith("\n")


def test_format_report_md_vazio_nao_quebra():
    now = datetime.datetime(2026, 1, 1, 0, 0)
    md = tc.format_report_md([], map_name="m", map_folder="/m", started=now, finished=now,
                             command="tour_docks", flow_text="f")
    assert "Totais: 0 ok / 0 com aviso / 0 reprovadas de 0 docks" in md


# ------------------------------------------- classificacao pelos numeros
# Os 16 results reais dos dois tours de 2026-10-26 (tour_2026-10-26_1945.md e
# _1952.md em caramelo_mapping/maps/arenalegal_2026): Long/Lat/Yaw do tour
# (AMCL, frame do dock) e x/y/yaw do align (LiDAR para o yaw, AMCL quando
# yaw_nao_medido). Serve de regressao para F6/C7 do diagnostico.
_OK_MSG = "alinhado (tentativa 1/1)"
_NM_MSG = "yaw_nao_medido; alinhado (tentativa 1/1)"


def _yfora(deg):
    return f"yaw_fora_da_tolerancia ({deg:+.1f} graus); alinhado (tentativa 1/1)"


# (tour, dock, long cm, lat cm, yaw graus, align x cm, align y cm, align yaw graus, message)
REAL_RESULTS = [
    ("T1", "WS1", -3.4, -23.3, -31.3, -0.2, +24.0, +23.9, _NM_MSG),
    ("T1", "WS2", +6.0, -14.3, +3.8, +0.7, +13.9, -7.6, _yfora(-7.6)),
    ("T1", "WS3", +2.1, +15.9, +8.6, +0.1, -15.8, -13.0, _NM_MSG),
    ("T1", "WS4", +15.9, +1.7, -5.7, -0.4, -2.5, +3.9, _OK_MSG),
    ("T1", "WS5", +6.0, +19.0, -0.2, -0.0, -18.0, -2.6, _OK_MSG),
    ("T1", "WS6", -2.7, +1.4, -8.6, -0.5, -1.0, +11.4, _yfora(+11.4)),
    ("T1", "SH1", +2.8, -20.6, +1.9, +1.5, +20.3, -2.2, _OK_MSG),
    ("T1", "PP1", +4.3, -17.0, -20.6, -0.3, +16.7, +19.9, _NM_MSG),
    ("T2", "WS1", +0.9, -6.9, -22.1, +0.1, +7.2, +12.9, _yfora(+12.9)),
    ("T2", "WS2", +4.6, +5.6, +2.3, +0.6, -4.1, -5.2, _yfora(-5.2)),
    ("T2", "WS3", +1.7, +17.5, +11.9, +0.1, -17.4, -15.9, _NM_MSG),
    ("T2", "WS4", +11.9, -8.5, -4.8, +0.5, +7.4, +0.6, _OK_MSG),
    ("T2", "WS5", +3.2, -2.3, -1.5, +0.1, +0.5, -3.2, _OK_MSG),
    ("T2", "WS6", +2.1, -0.5, -11.0, -0.3, +0.1, +10.5, _yfora(+10.5)),
    ("T2", "SH1", +3.7, -18.7, +16.0, +1.8, +17.6, -9.6, _yfora(-9.6)),
    ("T2", "PP1", +6.8, +9.0, +1.9, +0.3, -9.7, -6.0, _yfora(-6.0)),
]


def _real_report(row):
    tour, dock, lon, lat, yaw, ax, ay, ayaw, msg = row
    return tc.DockReport(dock, flow=tc.FLOW_STAGING_ALIGN, message=msg, align_message=msg,
                         err_long=lon / 100.0, err_lat=lat / 100.0, err_yaw=math.radians(yaw),
                         align_x=ax / 100.0, align_y=ay / 100.0, align_yaw=math.radians(ayaw))


# Pelos NUMEROS (yaw_tol 0,09 rad = 5,157 graus; y_max 0,10 m) reprovam 14 de
# 16. O plano (secao 11, I7) listava 12 e esperava WS4-T1 e WS2-T2 aprovados,
# mas: WS4-T1 tem yaw AMCL -5,7 graus (0,0995 rad > 0,09) e WS2-T2 tem
# yaw_face -5,2 graus que o proprio dock_align_node declarou fora
# (yaw_fora_da_tolerancia). Aprovados de verdade: so WS4-T2 e WS5-T2.
REAL_EXPECTED_FAIL = {
    ("T1", "WS1"), ("T1", "WS2"), ("T1", "WS3"), ("T1", "WS4"), ("T1", "WS5"),
    ("T1", "WS6"), ("T1", "SH1"), ("T1", "PP1"),
    ("T2", "WS1"), ("T2", "WS2"), ("T2", "WS3"), ("T2", "WS6"), ("T2", "SH1"), ("T2", "PP1"),
}


def test_classificacao_dos_16_results_reais():
    failed = set()
    for row in REAL_RESULTS:
        status, why = tc.classify_dock(_real_report(row))
        if status == tc.STATUS_FAIL_POSE:
            failed.add((row[0], row[1]))
            assert why
        else:
            assert status == tc.STATUS_OK and why == ""
    assert failed == REAL_EXPECTED_FAIL
    assert len(failed) == 14


def test_classificacao_real_motivos():
    by = {(r[0], r[1]): r for r in REAL_RESULTS}
    # WS1-T1: yaw_nao_medido com AMCL fora reprova pelas DUAS fontes AMCL.
    status, why = tc.classify_dock(_real_report(by[("T1", "WS1")]))
    assert status == "reprovada"
    assert "lat AMCL -23.3 > 10.0 cm" in why and "yaw AMCL -31.3 > 5.2 graus" in why
    assert "lat align +24.0 > 10.0 cm" in why and "yaw align/amcl +23.9 > 5.2 graus" in why
    # SH1-T1: so o lateral (yaw bom nas duas fontes), align disse "alinhado".
    status, why = tc.classify_dock(_real_report(by[("T1", "SH1")]))
    assert status == "reprovada" and "yaw" not in why
    assert "lat AMCL -20.6 > 10.0 cm" in why and "lat align +20.3 > 10.0 cm" in why
    # WS6-T1: lateral bom, yaw fora pelo LiDAR (+11.4) e pelo AMCL (-8.6).
    status, why = tc.classify_dock(_real_report(by[("T1", "WS6")]))
    assert "yaw align/lidar +11.4 > 5.2 graus" in why and "yaw AMCL -8.6 > 5.2 graus" in why
    assert "lat" not in why
    # WS4-T2 e WS5-T2 passam limpos.
    assert tc.classify_dock(_real_report(by[("T2", "WS4")])) == ("ok", "")
    assert tc.classify_dock(_real_report(by[("T2", "WS5")])) == ("ok", "")


def test_classificacao_regras_de_borda():
    base = dict(flow=tc.FLOW_STAGING_ALIGN, err_long=0.0)
    # yaw_nao_medido dentro das tolerancias: aviso (a face nao foi medida), nao reprova.
    rep = tc.DockReport("WS9", message=_NM_MSG, align_message=_NM_MSG, err_lat=0.02,
                        err_yaw=math.radians(2.0), align_y=-0.02, align_yaw=math.radians(-2.0), **base)
    assert tc.classify_dock(rep) == (tc.STATUS_WARN, "")
    # yaw_nao_medido com |yaw AMCL| > tol reprova (nao e mais "aviso").
    rep.align_yaw = math.radians(-6.0)
    assert tc.classify_dock(rep)[0] == tc.STATUS_FAIL_POSE
    # Exatamente no limite nao reprova (> e estrito).
    rep = tc.DockReport("WS9", message=_OK_MSG, align_message=_OK_MSG, err_lat=0.10,
                        err_yaw=0.09, align_y=-0.10, align_yaw=-0.09, **base)
    assert tc.classify_dock(rep) == (tc.STATUS_OK, "")
    rep.err_lat = 0.1001
    assert tc.classify_dock(rep)[0] == tc.STATUS_FAIL_POSE
    # Tolerancias relaxadas pela CLI (--yaw-tol-deg 15 --y-max 0.25) aprovam a WS6.
    ws6 = _real_report(next(r for r in REAL_RESULTS if r[:2] == ("T1", "WS6")))
    assert tc.classify_dock(ws6, yaw_tol=math.radians(15.0), y_max=0.25) == (tc.STATUS_WARN, "")
    # Sem medida (None) a fonte e ignorada; sem NENHUMA medida e aviso, nao ok.
    rep = tc.DockReport("WS9", message=_OK_MSG, align_message=_OK_MSG, flow=tc.FLOW_STAGING_ALIGN)
    assert tc.classify_dock(rep) == (tc.STATUS_WARN, "sem medida de pose")
    # Navegacao pura (FINISH) nao e docking: ok mesmo com -5.9 graus (tolerancia do Nav2).
    fin = tc.DockReport("FINISH", flow=tc.FLOW_NAV_ONLY, err_lat=0.033, err_yaw=math.radians(-5.9))
    assert tc.classify_dock(fin) == (tc.STATUS_OK, "")
    # --no-align (opennav): so os numeros do tour (AMCL) decidem.
    opn = tc.DockReport("WS2", flow=tc.FLOW_OPENNAV_ONLY, err_lat=-0.143, err_yaw=0.01)
    assert tc.classify_dock(opn) == (tc.STATUS_FAIL_POSE, "lat AMCL -14.3 > 10.0 cm")


def test_fonte_do_yaw_do_align():
    assert tc.align_yaw_source(_NM_MSG) == "amcl"
    assert tc.align_yaw_source(_yfora(-7.6)) == "lidar"
    assert tc.align_yaw_source(_OK_MSG) == "lidar"
    assert tc.align_yaw_source("") == "lidar"


def test_reprovada_conta_como_encostada_mas_nao_como_ok():
    """A reprovada ENCOSTOU (o proximo dock precisa de undock), mas nao
    entra em 'docks ok' nem no resumo como ok."""
    reps = [_real_report(r) for r in REAL_RESULTS if r[0] == "T1"]
    for rep in reps:
        rep.status, rep.pose_verdict = tc.classify_dock(rep)
    fin = tc.DockReport("FINISH", flow=tc.FLOW_NAV_ONLY, status=tc.STATUS_OK,
                        message="navegacao pura (sem dock)", t_nav=16.2,
                        err_long=-0.033, err_lat=0.033, err_yaw=math.radians(-5.9))
    reps.append(fin)
    s = tc.summarize(reps)
    assert s["n_total"] == 9 and s["n_reached"] == 9
    assert s["n_ok"] == 1 and s["n_warn"] == 0 and s["n_fail_pose"] == 8
    assert s["fail_pose_ids"] == ["WS1", "WS2", "WS3", "WS4", "WS5", "WS6", "SH1", "PP1"]
    assert all(r.reached for r in reps)
    line = tc.summary_line(s)
    assert line.startswith("Totais: 1 ok / 0 com aviso / 8 reprovadas (WS1, WS2, WS3, WS4, WS5, WS6, SH1, PP1) de 9 docks")
    assert "maior erro de yaw: WS1 (-31.3 graus)" in line
    assert tc.tour_exit_code(s) == 1
    md = tc.format_report_md(reps, map_name="m", map_folder="/m",
                             started=datetime.datetime(2026, 10, 26, 19, 45),
                             finished=datetime.datetime(2026, 10, 26, 19, 49),
                             command="tour_docks", flow_text="f")
    assert "| WS1 | reprovada |" in md
    assert "+23.9 graus (amcl)" in md and "+11.4 graus (lidar)" in md
    assert "| lat AMCL -20.6 > 10.0 cm; lat align +20.3 > 10.0 cm | - |" in md   # SH1


def test_exit_code_do_tour():
    def _sum(**statuses):
        reps = [tc.DockReport(f"D{i}", flow=tc.FLOW_STAGING_ALIGN, status=st)
                for i, st in enumerate(statuses.values())]
        return tc.summarize(reps)
    assert tc.tour_exit_code(_sum(a=tc.STATUS_OK, b=tc.STATUS_OK)) == 0
    assert tc.tour_exit_code(_sum(a=tc.STATUS_OK, b=tc.STATUS_WARN)) == 0      # aviso nao derruba
    assert tc.tour_exit_code(_sum(a=tc.STATUS_OK, b=tc.STATUS_FAIL_POSE)) == 1
    assert tc.tour_exit_code(_sum(a=tc.STATUS_OK, b=tc.STATUS_FAIL)) == 1
    assert tc.tour_exit_code(_sum(a=tc.STATUS_OK, b=tc.STATUS_CANCELED)) == 1
    assert tc.tour_exit_code(_sum(a=tc.STATUS_OK, b=tc.STATUS_SKIPPED)) == 1
    assert tc.tour_exit_code(tc.summarize([])) == 0


# --------------------------------------------------- espera de parada
class _OdomSim:
    """Amostras sinteticas de /odom a `rate` Hz: gira a 0,3 rad/s ate
    `t_stop` (None = nunca para), depois 0. spin_once avanca o relogio e
    entrega a amostra ao detector, como o rclpy.spin_once faria no robo."""

    def __init__(self, det, t_stop, rate=20.0, dead=False):
        self.det, self.t_stop, self.dt, self.dead = det, t_stop, 1.0 / rate, dead
        self.t = 0.0

    def clock(self):
        return self.t

    def spin_once(self):
        self.t += self.dt
        if self.dead:
            return
        moving = self.t_stop is None or self.t < self.t_stop
        self.det.feed(self.t, 0.0, 0.0, 0.3 if moving else 0.0)


def test_espera_de_parada_para_em_0_8_s_e_libera_em_1_3_s():
    det = tc.StopDetector(tc.StopLimits(), t_reset=0.0)
    sim = _OdomSim(det, t_stop=0.8)
    stopped, waited, state = tc.wait_for_stop(det, sim.clock, sim.spin_once)
    assert stopped is True and state == tc.STOP_STOPPED
    # Primeira amostra quieta em 0,80 s + janela 0,5 s = 1,30 s (+ um passo de 50 ms).
    assert 1.3 <= waited <= 1.35
    assert det.samples == pytest.approx(waited * 20.0, abs=1.0)


def test_espera_de_parada_nunca_para_libera_em_3_s_com_estado_movendo():
    # So a funcao pura: o WARN de wait_base_stopped (tour_docks.py) precisa de rclpy.
    det = tc.StopDetector(tc.StopLimits(), t_reset=0.0)
    sim = _OdomSim(det, t_stop=None)
    stopped, waited, state = tc.wait_for_stop(det, sim.clock, sim.spin_once)
    assert stopped is False and state == tc.STOP_MOVING
    assert waited == pytest.approx(3.0, abs=0.05)
    assert det.last_ang == pytest.approx(0.3) and det.last_lin == 0.0


def test_espera_de_parada_sem_odom():
    det = tc.StopDetector(tc.StopLimits(timeout_s=2.0), t_reset=0.0)
    sim = _OdomSim(det, t_stop=0.0, dead=True)
    stopped, waited, state = tc.wait_for_stop(det, sim.clock, sim.spin_once)
    assert stopped is False and state == tc.STOP_NO_ODOM
    assert waited == pytest.approx(2.0, abs=0.05) and det.samples == 0


def test_detector_de_parada_reinicia_a_janela_e_exige_amostra_fresca():
    det = tc.StopDetector(tc.StopLimits(), t_reset=10.0)
    assert det.state(10.0) == tc.STOP_NO_ODOM
    # Amostra quieta ANTERIOR ao reset conta a partir do reset, nao de antes.
    det.feed(9.9, 0.0, 0.0, 0.0)
    assert det.quiet_since == 10.0
    for t in (10.1, 10.2, 10.3, 10.4):
        det.feed(t, 0.01, 0.0, 0.02)
    assert det.state(10.4) == tc.STOP_MOVING          # 0,4 s < janela 0,5 s
    det.feed(10.5, 0.0, 0.0, 0.0)
    assert det.state(10.5) == tc.STOP_STOPPED
    det.feed(10.6, 0.0, 0.025, 0.0)                   # |v| 0,025 > 0,02: reinicia
    assert det.quiet_since is None and det.state(10.6) == tc.STOP_MOVING
    det.feed(10.7, 0.0, 0.0, 0.049)
    assert det.quiet_since == 10.7
    # A janela e medida ate a ULTIMA AMOSTRA, como no StopDetector C++ do
    # pose_gate.hpp: uma amostra quieta e 0,5 s de silencio do /odom NAO e
    # "parado" (revisao 2026-10-26), e preciso amostra quieta em 11,2.
    assert det.state(11.2) == tc.STOP_MOVING
    det.feed(11.2, 0.0, 0.0, 0.0)
    assert det.state(11.2) == tc.STOP_STOPPED
    assert det.state(11.7) == tc.STOP_STOPPED         # ainda fresca (0,5 s)
    assert det.state(11.8) == tc.STOP_NO_ODOM         # ultima amostra com 0,6 s > 0,5 s
    det.feed(11.3, float("nan"), 0.0, 0.0)            # nao finito nunca e quieto
    assert det.quiet_since is None


def test_detector_de_parada_uma_amostra_quieta_e_silencio_nao_e_parado():
    # Reproducao da revisao de 2026-10-26: antes, feed(1.0, 0) + state(1.5) dava "parado".
    det = tc.StopDetector(tc.StopLimits(), t_reset=0.0)
    det.feed(1.0, 0.0, 0.0, 0.0)
    assert det.state(1.5) == tc.STOP_MOVING
    det = tc.StopDetector(tc.StopLimits(), t_reset=0.0)
    det.feed(0.9, 0.0, 0.0, 0.3)
    det.feed(1.0, 0.0, 0.0, 0.0)
    assert det.state(1.5) == tc.STOP_MOVING


def test_summary_line_singular_de_reprovada():
    reps = [tc.DockReport("WS1", flow=tc.FLOW_STAGING_ALIGN, status=tc.STATUS_FAIL_POSE),
            tc.DockReport("WS2", flow=tc.FLOW_STAGING_ALIGN, status=tc.STATUS_OK)]
    line = tc.summary_line(tc.summarize(reps))
    assert line.startswith("Totais: 1 ok / 0 com aviso / 1 reprovada (WS1) de 2 docks")


def test_deriva_do_yaw_na_staging_e_coluna_de_parada():
    rep = tc.DockReport("WS6", flow=tc.FLOW_STAGING_ALIGN, status=tc.STATUS_OK,
                        staging_err_moving=(0.0, 0.0, math.radians(3.0)),
                        staging_err=(0.0, 0.0, math.radians(9.7)), t_stop=0.9, stop_ok=True)
    assert math.degrees(rep.staging_yaw_drift) == pytest.approx(6.7)
    assert tc._fmt_stop(rep) == "0.9 s, +6.7 graus"
    rep.stop_ok = False
    assert tc._fmt_stop(rep) == "! 0.9 s, +6.7 graus"
    rep.staging_err_moving = None
    assert rep.staging_yaw_drift is None and tc._fmt_stop(rep) == "! 0.9 s, - graus"
    assert tc._fmt_stop(tc.DockReport("X")) == "-"
    # Deriva atravessa +-pi sem virar -355 graus.
    rep = tc.DockReport("PP1", staging_err_moving=(0, 0, 3.10), staging_err=(0, 0, -3.10))
    assert rep.staging_yaw_drift == pytest.approx(2 * math.pi - 6.2)


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
