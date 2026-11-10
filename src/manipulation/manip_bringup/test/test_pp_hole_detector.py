"""Testes do NUCLEO (sem ROS) do detector de buracos da mesa de precisao.

Cobrem a segmentacao (marrom sobre tampo branco, anel branco obrigatorio,
rejeicao de blob cortado), a projecao pixel -> plano do tampo e o rastreio
de varios buracos com mediana.
"""
import importlib.util
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, "..", "scripts")


def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(SCRIPTS, name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    # pp_hole_detector_node importa container_detector_node pelo nome: o
    # registro em sys.modules faz o papel do diretorio compartilhado de
    # lib/manip_bringup onde os dois sao instalados.
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


cd = _load("container_detector_node")
pp = _load("pp_hole_detector_node")

BRANCO = (250, 250, 250)


def _marrom(h=15, s=120, v=120):
    px = np.uint8([[[h, s, v]]])
    return [int(c) for c in cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]]


def _square(img, center, side, bgr):
    x, y = int(center[0]), int(center[1])
    half = int(side // 2)
    cv2.rectangle(img, (x - half, y - half), (x + half, y + half), bgr, thickness=-1)


def test_three_holes_on_white_table():
    cfg = pp.HoleConfig()
    img = np.full((480, 640, 3), BRANCO, dtype=np.uint8)
    centers = [(180, 380), (320, 380), (460, 380)]   # longe dos poligonos da garra (topo)
    for c in centers:
        _square(img, c, 60, _marrom())
    blobs, _hsv, _masks = pp.segment_holes(img, cfg, holding=True)
    ok = [b for b in blobs if b.reason is None]
    assert len(ok) == 3, [(b.reason, b.center) for b in blobs]
    got = sorted(b.center for b in ok)
    for (gx, gy), (ex, ey) in zip(got, sorted(centers)):
        assert math.hypot(gx - ex, gy - ey) < 3.0, (got, centers)
    for b in ok:
        assert b.ring_white_frac > 0.9


def test_hole_without_white_ring_rejected():
    # Mesmo marrom, mas cercado de CINZA (V abaixo de white_v_min): e' o chao
    # da arena / fora da mesa, nao uma cavidade do tampo.
    cfg = pp.HoleConfig()
    img = np.full((480, 640, 3), (110, 110, 110), dtype=np.uint8)
    _square(img, (320, 380), 60, _marrom())
    blobs, _hsv, _masks = pp.segment_holes(img, cfg, holding=True)
    assert blobs, "a mancha marrom tem que existir"
    assert all(b.reason is not None for b in blobs)
    assert any(b.reason.startswith("anel_branco") for b in blobs), [b.reason for b in blobs]


def test_hole_cut_by_border_rejected():
    cfg = pp.HoleConfig()
    img = np.full((480, 640, 3), BRANCO, dtype=np.uint8)
    _square(img, (640, 380), 60, _marrom())   # metade fora da imagem
    blobs, _hsv, _masks = pp.segment_holes(img, cfg, holding=True)
    assert blobs
    assert all(b.reason == "cortado" for b in blobs), [b.reason for b in blobs]


def test_ring_check_can_be_disabled():
    cfg = pp.HoleConfig()
    cfg.ring_enabled = False
    img = np.full((480, 640, 3), (110, 110, 110), dtype=np.uint8)
    _square(img, (320, 380), 60, _marrom())
    blobs, _hsv, _masks = pp.segment_holes(img, cfg, holding=True)
    assert any(b.reason is None for b in blobs)


def _caster_looking_down(cfg, cam_xyz=(0.0, 0.30, 0.30)):
    K = np.array([600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0])
    caster = cd.PlaneCaster(K, [], cfg)
    # Otico: z para a frente, x direita, y baixo. Camera olhando RETO para
    # baixo com x alinhado ao x da base: colunas = eixos da camera na base.
    R = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    caster.set_camera_pose(np.array(cam_xyz), R)
    return caster


def test_hole_xy_ray_cast_center():
    cfg = pp.HoleConfig()
    caster = _caster_looking_down(cfg)
    # Cavidade de 5 cm a 0,30 m: lado projetado = 0.05 * 600 / 0.30 = 100 px.
    cx, cy, half = 320.0, 240.0, 50.0
    blob = pp.HoleBlob(
        center=(cx, cy),
        box=np.array([[cx - half, cy - half], [cx + half, cy - half],
                      [cx + half, cy + half], [cx - half, cy + half]]),
        long_mid=((cx - half, cy), (cx + half, cy)),
        short_mid=((cx, cy - half), (cx, cy + half)),
        area_px=100.0 * 100.0,
        solidity=1.0, aspect=1.0, rectangularity=1.0, ring_white_frac=1.0,
        contour=np.zeros((4, 1, 2), dtype=np.int32))
    res, why = pp.hole_xy(caster, blob, 0.0, cfg)
    assert res is not None, why
    x, y, long_len, short_len, area_ratio = res
    assert abs(x - 0.0) < 1e-6 and abs(y - 0.30) < 1e-6
    assert abs(long_len - 0.05) < 0.002 and abs(short_len - 0.05) < 0.002
    assert 0.9 < area_ratio < 1.1


def test_hole_xy_rejects_wrong_size():
    cfg = pp.HoleConfig()
    caster = _caster_looking_down(cfg)
    cx, cy, half = 320.0, 240.0, 150.0   # 15 cm "cavidade": grande demais
    blob = pp.HoleBlob(
        center=(cx, cy),
        box=np.array([[cx - half, cy - half], [cx + half, cy - half],
                      [cx + half, cy + half], [cx - half, cy + half]]),
        long_mid=((cx - half, cy), (cx + half, cy)),
        short_mid=((cx, cy - half), (cx, cy + half)),
        area_px=300.0 * 300.0,
        solidity=1.0, aspect=1.0, rectangularity=1.0, ring_white_frac=1.0,
        contour=np.zeros((4, 1, 2), dtype=np.int32))
    res, why = pp.hole_xy(caster, blob, 0.0, cfg)
    assert res is None and why.startswith("lado_longo"), (res, why)


def test_tracks_two_holes_median_and_prune():
    tr = pp.HoleTracks(match_dist_m=0.03, n=5, window_s=2.0, min_samples=2)
    # Duas cavidades com ruido de +-2 mm; uma unica amostra avulsa longe.
    for i, t in enumerate([0.0, 0.3, 0.6]):
        dx = (i - 1) * 0.002
        tr.push(t, [(0.10 + dx, 0.30), (0.20 - dx, 0.30)])
    tr.push(0.9, [(0.50, 0.40)])
    est = tr.estimates(1.0)
    assert len(est) == 2, est   # a avulsa tem 1 amostra < min_samples
    xs = sorted(e[0] for e in est)
    assert abs(xs[0] - 0.10) < 0.003 and abs(xs[1] - 0.20) < 0.003
    # Janela vence: 3 s depois nada sobrevive.
    assert tr.estimates(4.0) == []
