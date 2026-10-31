"""Testes do detector de mesa pela camera (caramelo_dock_camera), sem ROS.

Renderiza o depth da D455 por raio (chao + mesa em caixa) com a camera na
pose `home` medida no robo em 2026-10-27 (x 0,264, z 0,480, 23 graus para
baixo, imagem de cabeca para baixo) e mede com um TF ERRADO em pitch
(5,45 graus, o erro medido no chao): o detector tem de achar yaw e centro da
mesa mesmo assim.
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import caramelo_dock_camera as cam  # noqa: E402

K = np.array([[390.67, 0.0, 322.38], [0.0, 390.67, 240.0], [0.0, 0.0, 1.0]])
W, H = 640, 480
# TF base<-optico lido no robo (braco em home): colunas = eixos x, y, z opticos.
R_HOME = np.array([[0.002, 0.389, 0.921],
                   [1.000, 0.000, -0.002],
                   [0.000, 0.921, -0.389]])
T_HOME = np.array([0.264, 0.011, 0.480])


def _orthonormal(r):
    u, _s, vt = np.linalg.svd(r)
    return u @ vt


def _rot_y(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def render(table_x, table_y, table_yaw_deg, width=0.80, depth=0.50, h=0.10,
           r_true=None, t=T_HOME, extra=(), extra_t=()):
    """Depth (mm, uint16) visto pela camera VERDADEIRA. A mesa: face da
    frente a table_x (no ponto do seu centro), centro lateral table_y, normal
    girada table_yaw_deg (CCW). extra = caixas (x0, x1, y0, y1, h) no base;
    extra_t = idem no frame da MESA (x a partir da face, y do centro)."""
    r = _orthonormal(R_HOME if r_true is None else r_true)
    vs, us = np.mgrid[0:H, 0:W]
    dirs_opt = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1],
                         np.ones_like(us, dtype=float)], axis=-1).reshape(-1, 3)
    dirs = dirs_opt @ r.T
    best = np.full(len(dirs), np.inf)
    # chao
    with np.errstate(divide="ignore", invalid="ignore"):
        s = -t[2] / dirs[:, 2]
    best = np.where((s > 0), np.minimum(best, s), best)
    ya = math.radians(table_yaw_deg)
    c, sn = math.cos(ya), math.sin(ya)

    def to_table(p):
        q = p - np.array([table_x, table_y, 0.0])
        return np.stack([q[..., 0] * c + q[..., 1] * sn,
                         -q[..., 0] * sn + q[..., 1] * c, q[..., 2]], axis=-1)

    boxes = [(0.0, depth, -width / 2, width / 2, h, True)]
    boxes += [(x0, x1, y0, y1, hh, False) for x0, x1, y0, y1, hh in extra]
    boxes += [(x0, x1, y0, y1, hh, True) for x0, x1, y0, y1, hh in extra_t]
    for x0, x1, y0, y1, hh, in_table_frame in boxes:
        o = to_table(t) if in_table_frame else t
        dd = (np.stack([dirs[:, 0] * c + dirs[:, 1] * sn, -dirs[:, 0] * sn + dirs[:, 1] * c,
                        dirs[:, 2]], axis=-1) if in_table_frame else dirs)
        lo = np.array([x0, y0, 0.0])
        hi = np.array([x1, y1, hh])
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo - o) / dd
            t2 = (hi - o) / dd
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tmax >= tmin) & (tmin > 0)
        best = np.where(hit, np.minimum(best, tmin), best)
    zc = best * dirs_opt[:, 2]
    zc[~np.isfinite(zc)] = 0.0
    return np.clip(zc * 1000.0, 0, 65535).astype(np.uint16).reshape(H, W)


def measure(depth_img, r_tf=None, cfg=None):
    cfg = cfg or cam.CamCfg()
    r = _orthonormal(R_HOME if r_tf is None else r_tf)
    m, why = cam.measure_depth_frame(depth_img, K, r, T_HOME, cfg)
    assert m is not None, why
    return m


# TF com o erro medido: a camera de verdade esta 5,45 graus mais inclinada
# do que o TF diz (sinal escolhido para o chao "subir" com x, como no robo).
R_TRUE_TILT = _rot_y(5.45) @ R_HOME


@pytest.mark.parametrize("yaw_deg,y_off", [(0.0, 0.0), (8.0, 0.05), (-12.0, -0.07), (4.0, 0.10)])
def test_yaw_e_centro_no_staging(yaw_deg, y_off):
    img = render(0.63, y_off, yaw_deg)
    m = measure(img)
    assert m.ok, m.reason
    assert math.degrees(m.face_yaw) == pytest.approx(yaw_deg, abs=1.0)
    assert m.c_foot is not None, m.center_reason
    # c_foot = desvio do centro da face pelo pe' da perpendicular
    ya = math.radians(yaw_deg)
    d = 0.63 * math.cos(ya) + y_off * math.sin(ya)
    v_c = -0.63 * math.sin(ya) + y_off * math.cos(ya)
    assert m.c_foot == pytest.approx(v_c, abs=0.015)
    assert m.c_axis == pytest.approx(v_c + d * math.tan(ya), abs=0.02)
    assert m.front == pytest.approx(d / math.cos(ya), abs=0.02)


def test_tf_inclinado_nao_engana():
    """Camera real 5,45 graus mais inclinada que o TF: o chao 'sobe' com x,
    mas o detector corrige pelo plano e mede yaw/centro certos."""
    img = render(0.63, 0.04, 6.0, r_true=R_TRUE_TILT)
    m = measure(img)
    assert m.ok, m.reason
    assert m.tilt_deg == pytest.approx(5.45, abs=0.5)
    assert math.degrees(m.face_yaw) == pytest.approx(6.0, abs=1.2)
    assert m.c_foot == pytest.approx(-0.63 * math.sin(math.radians(6)) + 0.04 * math.cos(math.radians(6)), abs=0.02)
    assert m.table_h == pytest.approx(0.10, abs=0.015)


def test_objetos_no_tampo_nao_atrapalham():
    cubes = [(0.75, 0.80, -0.20, -0.15, 0.15), (0.80, 0.85, 0.10, 0.15, 0.15)]
    m = measure(render(0.65, 0.0, 3.0, extra=cubes))
    assert m.ok, m.reason
    assert math.degrees(m.face_yaw) == pytest.approx(3.0, abs=1.0)
    assert m.c_foot == pytest.approx(-0.65 * math.sin(math.radians(3)), abs=0.015)


def test_perto_demais_usa_laterais():
    """Face a 0,50 m: a borda da frente sai do campo de visao; yaw e centro
    vem das laterais."""
    m = measure(render(0.50, 0.03, 5.0))
    assert m.ok, m.reason
    assert m.yaw_src in ("lados", "frente+lados")
    assert math.degrees(m.face_yaw) == pytest.approx(5.0, abs=1.5)
    assert m.c_foot == pytest.approx(-0.50 * math.sin(math.radians(5)) + 0.03 * math.cos(math.radians(5)), abs=0.02)


def test_mesa_vizinha_nao_entra():
    """Outra mesa a 0,30 m do lado direito: a largura medida continua 0,80."""
    neighbor = [(0.60, 1.10, -1.20, -0.70, 0.10)]
    m = measure(render(0.63, 0.0, 0.0, extra=neighbor))
    assert m.ok, m.reason
    assert m.width == pytest.approx(0.80, abs=0.04)
    assert m.c_foot == pytest.approx(0.0, abs=0.015)


def test_mesa_mais_estreita_que_o_yaml():
    """Robo 2026-10-27: mesa real 0,61 m x 0,15 m, yaml dizendo 0,80."""
    m = measure(render(0.70, 0.05, 5.0, width=0.61, h=0.15, r_true=R_TRUE_TILT))
    assert m.ok, m.reason
    assert m.width == pytest.approx(0.61, abs=0.02)
    ya = math.radians(5.0)
    assert m.c_foot == pytest.approx(-0.70 * math.sin(ya) + 0.05 * math.cos(ya), abs=0.015)


@pytest.mark.parametrize("width,depth,h", [
    (1.20, 0.70, 0.15),   # larga e funda
    (0.50, 0.40, 0.05),   # estreita e baixa
    (0.90, 0.60, 0.10),
])
def test_qualquer_mesa_sem_dimensao_configurada(width, depth, h):
    m = measure(render(0.65, -0.04, -6.0, width=width, depth=depth, h=h, r_true=R_TRUE_TILT))
    assert m.ok, m.reason
    assert math.degrees(m.face_yaw) == pytest.approx(-6.0, abs=1.2)
    assert m.table_h == pytest.approx(h, abs=0.02)
    assert m.c_foot is not None, m.center_reason
    assert m.width == pytest.approx(width, abs=0.03)
    ya = math.radians(-6.0)
    assert m.c_foot == pytest.approx(-0.65 * math.sin(ya) - 0.04 * math.cos(ya), abs=0.02)


def test_uma_lateral_so_com_largura_ja_medida():
    """Robo deslocado: a lateral direita sai da imagem. Sem largura medida
    antes, nao inventa centro; com a largura do proprio docking, centra."""
    img = render(0.45, -0.35, 0.0, width=1.0, depth=0.6, h=0.12)
    m = measure(img)
    assert m.ok and m.c_foot is None and "largura ainda nao medida" in m.center_reason
    cfg = cam.CamCfg()
    cfg.expected_width = 1.0
    m = measure(img, cfg=cfg)
    assert m.c_foot == pytest.approx(-0.35, abs=0.02)


def test_fundo_em_degrau_so_informa():
    """Tampo em L (peca grudada atras de um lado): o fundo em degrau aparece
    em info['retangulo'][2] (< 0,8) mas nao reprova — no tour de 27/10 os
    objetos no tampo davam o mesmo sinal em mesas boas."""
    stuck = [(0.63, 1.40, 0.30, 0.60, 0.10)]   # encostada no lado esquerdo, mais funda
    m = measure(render(0.63, 0.0, 0.0, width=0.6, depth=0.5, h=0.10, extra=stuck))
    assert m.ok
    assert m.info["retangulo"][2] < 0.8


def test_parede_logo_atras_da_mesa():
    """Robo 2026-10-27: parede encostada no fundo da mesa. A fatia dela na
    altura do tampo nao pode virar 'frente' (era escolhida em metade dos
    frames, front 1,17 em vez de 0,73)."""
    wall = [(1.17, 1.20, -1.2, 1.2, 0.60)]
    for y in (0.0, 0.03):
        m = measure(render(0.73, y, -3.0, width=0.61, depth=0.44, h=0.15,
                           r_true=R_TRUE_TILT, extra=wall))
        assert m.ok, m.reason
        assert m.front == pytest.approx(0.73, abs=0.03)
        assert m.width == pytest.approx(0.61, abs=0.03)
        assert math.degrees(m.face_yaw) == pytest.approx(-3.0, abs=1.0)


# Paredes brancas como o tampo (2026-10-27, operador): atras, de um lado,
# do outro, ou no canto. No frame da mesa 0,61 x 0,44 x 0,15.
_WALL_BACK = (0.44, 0.47, -1.5, 1.5, 0.60)
_WALL_LEFT = (-0.60, 0.47, 0.305, 0.335, 0.60)
_WALL_RIGHT = (-0.60, 0.47, -0.335, -0.305, 0.60)


@pytest.mark.parametrize("walls", [
    [_WALL_BACK], [_WALL_LEFT], [_WALL_RIGHT], [_WALL_BACK, _WALL_LEFT],
    [_WALL_BACK, _WALL_RIGHT]], ids=["atras", "esquerda", "direita", "canto_esq", "canto_dir"])
@pytest.mark.parametrize("yaw_deg,y_off", [(0.0, 0.0), (-5.0, 0.06), (7.0, -0.05)])
def test_paredes_atras_e_nas_laterais(walls, yaw_deg, y_off):
    img = render(0.68, y_off, yaw_deg, width=0.61, depth=0.44, h=0.15,
                 r_true=R_TRUE_TILT, extra_t=walls)
    m = measure(img)
    assert m.ok, m.reason
    ya = math.radians(yaw_deg)
    assert math.degrees(m.face_yaw) == pytest.approx(yaw_deg, abs=1.2)
    assert m.c_foot is not None, m.center_reason
    assert m.width == pytest.approx(0.61, abs=0.03)
    v_c = -0.68 * math.sin(ya) + y_off * math.cos(ya)
    assert m.c_foot == pytest.approx(v_c, abs=0.015)


def test_mesa_cortada_pelo_alcance_nao_inventa_lateral():
    """Mesa longe (1,3 m) atravessando o limite de 1,6 m: o corte nao vira
    lateral nem largura."""
    m = measure(render(1.30, 0.0, -20.0, width=1.0, depth=0.6, h=0.14))
    assert m.c_foot is None or m.width is None or m.width > 0.9


def test_sem_mesa():
    m = measure(render(5.0, 0.0, 0.0))
    assert not m.ok
    assert "tampo" in m.reason


def test_camera_fora_da_pose():
    ok, _ = cam.camera_view_ok(_orthonormal(R_HOME))
    assert ok
    olhando_reto = _orthonormal(np.array([[0, 0, 1.0], [1.0, 0, 0], [0, 1.0, 0]]))
    ok, why = cam.camera_view_ok(olhando_reto)
    assert not ok and "eixo optico" in why
