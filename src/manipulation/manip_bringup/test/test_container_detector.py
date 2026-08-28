"""Testes do NUCLEO (sem ROS) do detector de containers por cor.

Cobrem a segmentacao (mascaras HSV, filtros 2D, ROI do cubo na garra), a
projecao pixel -> plano (ray-cast com camera inclinada) e a suavizacao.
"""
import importlib.util
import math
import os

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "scripts", "container_detector_node.py")


def _load():
    spec = importlib.util.spec_from_file_location("container_detector_node", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


cd = _load()


def _hsv_bgr(h, s=200, v=200):
    px = np.uint8([[[h, s, v]]])
    return [int(c) for c in cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]]


def _draw_rect(img, center, size, angle_deg, bgr):
    rect = ((float(center[0]), float(center[1])), (float(size[0]), float(size[1])), float(angle_deg))
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.fillPoly(img, [box], bgr)


def test_red_container_with_gripper_regions_masked():
    cfg = cd.DetectorConfig()
    img = np.full((480, 640, 3), 200, np.uint8)          # mesa clara
    _draw_rect(img, (300, 340), (170, 105), 20, _hsv_bgr(3))   # container vermelho (H=3)
    _draw_rect(img, (300, 130), (60, 60), 0, _hsv_bgr(176))    # cubo vermelho na garra (H=176)
    _draw_rect(img, (30, 40), (60, 80), 0, _hsv_bgr(2))        # dedo vermelho no canto
    per_color, _hsv, _masks = cd.segment_image(img, cfg)
    reds = per_color["vermelho"]
    accepted = [b for b in reds if b.reason is None]
    assert len(accepted) == 1, [(b.center, b.reason) for b in reds]
    b = accepted[0]
    assert abs(b.center[0] - 300) < 3 and abs(b.center[1] - 340) < 3
    assert 1.45 < b.aspect < 1.75
    assert len(reds) == 1                                       # garra e cubo apagados da mascara
    assert not [b for b in per_color["azul"] if b.reason is None]


def test_blue_container_ignores_small_speck():
    cfg = cd.DetectorConfig()
    img = np.full((480, 640, 3), 200, np.uint8)
    _draw_rect(img, (250, 340), (170, 105), -35, _hsv_bgr(110))
    cv2.circle(img, (520, 100), 6, _hsv_bgr(110), -1)          # ruido azul pequeno
    per_color, _hsv, _masks = cd.segment_image(img, cfg)
    blues = [b for b in per_color["azul"] if b.reason is None]
    assert len(blues) == 1
    assert abs(blues[0].center[0] - 250) < 3 and abs(blues[0].center[1] - 340) < 3


def test_cube_rect_only_when_holding():
    cfg = cd.DetectorConfig()
    img = np.full((480, 640, 3), 200, np.uint8)
    _draw_rect(img, (300, 200), (170, 105), 0, _hsv_bgr(3))    # container no meio do retangulo do cubo
    hold, _h, _m = cd.segment_image(img, cfg, holding=True)
    free, _h, _m = cd.segment_image(img, cfg, holding=False)
    b_hold = [b for b in hold["vermelho"] if b.reason is None][0]
    b_free = [b for b in free["vermelho"] if b.reason is None][0]
    assert b_hold.partial and not b_free.partial              # segurando: parte apagada -> parcial
    assert abs(b_free.center[0] - 300) < 3 and abs(b_free.center[1] - 200) < 3
    assert abs(b_hold.center[0] - 300) < 3 and 200 < b_hold.center[1] < 240   # centroide desce, mas dentro
    cfg.cube_rect_only_when_holding = False
    always, _h, _m = cd.segment_image(img, cfg, holding=False)
    assert [b for b in always["vermelho"] if b.reason is None][0].partial


def test_fit_gripper_polys_finds_top_corner_blobs():
    cfg = cd.DetectorConfig()
    img = np.full((480, 640, 3), 200, np.uint8)
    cv2.fillPoly(img, [np.array([[0, 0], [80, 0], [90, 130], [0, 90]])], _hsv_bgr(3))     # dedo esquerdo
    cv2.fillPoly(img, [np.array([[560, 0], [640, 0], [640, 60], [540, 130]])], _hsv_bgr(3))  # dedo direito
    polys = cd.fit_gripper_polys(img, cfg, margin_px=10)
    assert len(polys) == 2 and all(top for _txt, top in polys)
    parsed = cd.parse_polys([txt for txt, _top in polys])
    assert all(len(p) >= 3 for p in parsed)
    xs = [x for p in parsed for x, _y in p]
    assert min(xs) < 0.02 and max(xs) > 0.98


def test_square_cube_outside_roi_rejected_by_aspect():
    cfg = cd.DetectorConfig()
    img = np.full((480, 640, 3), 200, np.uint8)
    _draw_rect(img, (150, 150), (60, 60), 10, _hsv_bgr(3))
    per_color, _hsv, _masks = cd.segment_image(img, cfg)
    reds = per_color["vermelho"]
    assert reds and all(b.reason == "proporcao" for b in reds)


def _camera_looking_down(tilt_deg):
    """Camera a 0,295 m acima da base, optical frame (z para frente), inclinada."""
    # Optical: x direita, y baixo, z frente. Queremos z ~ -Z_base (olhando para baixo),
    # com inclinacao tilt em torno do eixo x da camera. Construimos R (cam->base).
    t = math.radians(tilt_deg)
    # base: X direita, Y frente, Z cima. cam z -> (0, sin t, -cos t); cam x -> (1,0,0); cam y = z × x
    zc = np.array([0.0, math.sin(t), -math.cos(t)])
    xc = np.array([1.0, 0.0, 0.0])
    yc = np.cross(zc, xc)
    R = np.column_stack([xc, yc, zc])
    o = np.array([-0.007, 0.280, 0.295])
    return o, R


def test_plane_cast_round_trip():
    cfg = cd.DetectorConfig()
    cfg.wall_bias_correction = False
    K = np.array([[385.0, 0, 320.0], [0, 385.0, 240.0], [0, 0, 1]])
    caster = cd.PlaneCaster(K, [0, 0, 0, 0, 0], cfg)
    o, R = _camera_looking_down(10.0)
    caster.set_camera_pose(o, R)
    plane_z = 0.041
    cx, cy, yaw = 0.05, 0.30, math.radians(25.0)
    L, W = cfg.container_long_m, cfg.container_short_m
    # cantos do retangulo no plano -> pixels
    corners_base = []
    for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        dx, dy = sx * L / 2.0, sy * W / 2.0
        px = cx + dx * math.cos(yaw) - dy * math.sin(yaw)
        py = cy + dx * math.sin(yaw) + dy * math.cos(yaw)
        corners_base.append(np.array([px, py, plane_z]))

    def project(p):
        pc = R.T @ (p - o)
        return (K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2])

    pix = np.array([project(p) for p in corners_base])
    img = np.full((480, 640, 3), 200, np.uint8)
    cv2.fillPoly(img, [pix.astype(np.int32)], _hsv_bgr(3))
    per_color, _hsv, _masks = cd.segment_image(img, cfg)
    blob = [b for b in per_color["vermelho"] if b.reason is None][0]
    pose, why = caster.container_pose(blob, plane_z)
    assert pose is not None, why
    assert abs(pose.x - cx) < 0.004 and abs(pose.y - cy) < 0.004
    assert abs(cd.wrap_half_pi(pose.yaw - yaw)) < math.radians(2.0)
    assert abs(pose.long_len - L) < 0.012 and abs(pose.short_len - W) < 0.012


def test_plane_cast_rejects_behind_arm():
    cfg = cd.DetectorConfig()
    K = np.array([[385.0, 0, 320.0], [0, 385.0, 240.0], [0, 0, 1]])
    caster = cd.PlaneCaster(K, None, cfg)
    o, R = _camera_looking_down(0.0)
    caster.set_camera_pose(o, R)
    # ray para cima nunca cruza o plano abaixo
    o2 = o.copy()
    R2 = R.copy()
    R2[:, 2] = -R2[:, 2]
    caster.set_camera_pose(o2, R2)
    assert caster.cast(320, 240, 0.041) is None


def test_smoother_median_and_yaw_wrap():
    sm = cd.PoseSmoother(n=5, window_s=2.0, jump_reset_m=0.04, min_samples=2)
    for i, (x, y, yaw_deg) in enumerate([(0.10, 0.30, 85), (0.11, 0.31, -88), (0.10, 0.30, 87)]):
        sm.push(float(i) * 0.3, x, y, math.radians(yaw_deg))
    est = sm.estimate(1.0)
    assert est is not None
    x, y, yaw, n = est
    assert n == 3 and abs(x - 0.10) < 1e-9 and abs(y - 0.30) < 1e-9
    # 85, 92 (=-88 mod 180) e 87 -> mediana 87
    assert abs(math.degrees(yaw) - 87) < 1e-6 or abs(math.degrees(yaw) + 93) < 1e-6
    # salto grande reseta
    sm.push(1.2, 0.30, 0.50, 0.0)
    assert len(sm.samples) == 1


def test_config_from_dict_parses_colors_and_rects():
    cfg = cd.config_from_dict({
        "colors": ["vermelho", "verde"],
        "vermelho": {"h_ranges": [0, 8, 172, 179], "s_min": 120},
        "verde": {"frame": "ct_verde", "h_ranges": [40, 80]},
        "exclusion_rects_norm": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
    })
    assert cfg.colors["vermelho"].h_ranges == [(0, 8), (172, 179)]
    assert cfg.colors["vermelho"].s_min == 120
    assert cfg.colors["verde"].frame == "ct_verde"
    assert len(cfg.exclusion_rects) == 2
    with pytest.raises(ValueError):
        cd.parse_h_ranges([0, 10, 170])


def test_partial_container_at_border_uses_visible_centroid():
    cfg = cd.DetectorConfig()
    img = np.full((480, 640, 3), 200, np.uint8)
    # container vermelho com metade fora do quadro (esquerda): visivel x in [0, 90]
    cv2.rectangle(img, (-90, 150), (90, 330), _hsv_bgr(3), -1)
    per_color, _hsv, _masks = cd.segment_image(img, cfg)
    reds = [b for b in per_color["vermelho"] if b.reason is None]
    assert len(reds) == 1 and reds[0].partial
    cx, cy = reds[0].center
    assert 0 < cx < 90 and 150 < cy < 330          # centroide da parte visivel
    assert abs(cx - 45) < 4 and abs(cy - 240) < 4
    cfg.accept_partial = False
    per_color, _hsv, _masks = cd.segment_image(img, cfg)
    assert all(b.reason == "borda_imagem" for b in per_color["vermelho"])


def test_triangle_and_elongated_full_blobs_rejected():
    cfg = cd.DetectorConfig()
    cfg.exclusion_polys = []                                  # sem mascaras: queremos ver os blobs
    img = np.full((480, 640, 3), 200, np.uint8)
    cv2.fillPoly(img, [np.array([[300, 300], [460, 300], [380, 440]])], _hsv_bgr(3))   # triangulo (dedo solto)
    cv2.rectangle(img, (40, 400), (300, 440), _hsv_bgr(110), -1)                      # faixa 6.5:1 (lado de cubo)
    per_color, _hsv, _masks = cd.segment_image(img, cfg, holding=False)
    reds = [b for b in per_color["vermelho"] if b.area_px > 1000]
    assert reds and reds[0].reason == "retangularidade", [(b.reason, b.rectangularity) for b in reds]
    blues = [b for b in per_color["azul"] if b.area_px > 1000]
    assert blues and blues[0].reason == "proporcao_max"


def test_partial_sliver_rejected_by_expected_area():
    cfg = cd.DetectorConfig()
    K = np.array([[385.0, 0, 320.0], [0, 385.0, 240.0], [0, 0, 1]])
    caster = cd.PlaneCaster(K, None, cfg)
    o, R = _camera_looking_down(10.0)
    caster.set_camera_pose(o, R)
    img = np.full((480, 640, 3), 200, np.uint8)
    cv2.rectangle(img, (0, 180), (30, 280), _hsv_bgr(3), -1)      # filete na borda esquerda (~10% da area esperada)
    per_color, _hsv, _masks = cd.segment_image(img, cfg, holding=False)
    blob = [b for b in per_color["vermelho"] if b.reason is None][0]
    assert blob.partial
    pose, why = caster.container_pose(blob, 0.041)
    assert pose is None and why.startswith("area_parcial"), why


def test_overlapping_exclusion_polys_have_no_hole():
    cfg = cd.DetectorConfig()
    cfg.exclusion_polys = [[(0.0, 0.0), (0.3, 0.0), (0.3, 0.3), (0.0, 0.3)]]
    cfg.exclusion_polys_holding = [[(0.1, 0.1), (0.5, 0.1), (0.5, 0.5), (0.1, 0.5)]]
    seg = cd.ContainerSegmenter(cfg)
    seg.holding = True
    m = seg.exclusion_mask((100, 100))
    assert m[20, 20] == 255 and m[5, 5] == 255 and m[40, 40] == 255 and m[80, 80] == 0


def test_partial_blob_ignores_side_limits_and_full_blob_uses_upper_tolerance():
    cfg = cd.DetectorConfig()
    K = np.array([[385.0, 0, 320.0], [0, 385.0, 240.0], [0, 0, 1]])
    caster = cd.PlaneCaster(K, None, cfg)
    o, R = _camera_looking_down(10.0)
    caster.set_camera_pose(o, R)
    # Container "inflado" (paredes visiveis) cortado pela borda de baixo: 0,20 x 0,18 m no plano.
    img = np.full((480, 640, 3), 200, np.uint8)
    cv2.rectangle(img, (170, 250), (470, 520), _hsv_bgr(3), -1)
    per_color, _hsv, _masks = cd.segment_image(img, cfg, holding=False)
    blob = [b for b in per_color["vermelho"] if b.reason is None][0]
    assert blob.partial
    pose, why = caster.container_pose(blob, 0.041)
    assert pose is not None, why
    # Inteiro e um pouco inflado (1,3x): aceito com o limite superior de 2x...
    img2 = np.full((480, 640, 3), 200, np.uint8)
    _draw_rect(img2, (320, 320), (310, 190), 0, _hsv_bgr(3))
    per2, _h, _m = cd.segment_image(img2, cfg, holding=False)
    blob2 = [b for b in per2["vermelho"] if b.reason is None][0]
    assert not blob2.partial
    pose2, why2 = caster.container_pose(blob2, 0.041)
    assert pose2 is not None, why2
    # ... e rejeitado se o limite superior for apertado (1,1x; o blob mede ~1,2x).
    cfg.dims_tolerance_upper_frac = 0.1
    pose3, why3 = caster.container_pose(blob2, 0.041)
    assert pose3 is None and why3.startswith("lado_"), why3


def _project_rect(K, o, R, cx, cy, yaw, L, W, plane_z):
    corners = []
    for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        dx, dy = sx * L / 2.0, sy * W / 2.0
        px = cx + dx * math.cos(yaw) - dy * math.sin(yaw)
        py = cy + dx * math.sin(yaw) + dy * math.cos(yaw)
        p = np.array([px, py, plane_z])
        pc = R.T @ (p - o)
        corners.append((K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2]))
    return np.array(corners)


def test_yaw_reliability_and_memory():
    cfg = cd.DetectorConfig()
    cfg.wall_bias_correction = False
    K = np.array([[385.0, 0, 320.0], [0, 385.0, 240.0], [0, 0, 1]])
    caster = cd.PlaneCaster(K, [0, 0, 0, 0, 0], cfg)
    o, R = _camera_looking_down(10.0)
    caster.set_camera_pose(o, R)
    plane_z = 0.041
    # alongado (0,17 x 0,105): yaw confiavel
    img = np.full((480, 640, 3), 200, np.uint8)
    cv2.fillPoly(img, [_project_rect(K, o, R, 0.0, 0.30, math.radians(30), 0.17, 0.105, plane_z).astype(np.int32)], _hsv_bgr(3))
    blob = [b for b in cd.segment_image(img, cfg, holding=False)[0]["vermelho"] if b.reason is None][0]
    pose, why = caster.container_pose(blob, plane_z)
    assert pose is not None and pose.yaw_reliable and not pose.yaw_from_memory
    assert abs(cd.wrap_half_pi(pose.yaw - math.radians(30))) < math.radians(3)
    # pouco alongado (0,17 x 0,14, razao 1,2 < 1,3): passa no 2D mas o yaw NAO e confiavel; com dica usa a memoria
    img2 = np.full((480, 640, 3), 200, np.uint8)
    cv2.fillPoly(img2, [_project_rect(K, o, R, 0.0, 0.30, math.radians(10), 0.17, 0.14, plane_z).astype(np.int32)], _hsv_bgr(3))
    blob2 = [b for b in cd.segment_image(img2, cfg, holding=False)[0]["vermelho"] if b.reason is None][0]
    pose2, _ = caster.container_pose(blob2, plane_z)
    assert pose2 is not None and not pose2.yaw_reliable and not pose2.yaw_from_memory
    pose3, _ = caster.container_pose(blob2, plane_z, yaw_hint=math.radians(70))
    assert pose3.yaw_from_memory and abs(cd.wrap_half_pi(pose3.yaw - math.radians(70))) < 1e-9


def test_partial_center_extended_towards_cut_side():
    cfg = cd.DetectorConfig()
    cfg.wall_bias_correction = False
    K = np.array([[385.0, 0, 320.0], [0, 385.0, 240.0], [0, 0, 1]])
    caster = cd.PlaneCaster(K, [0, 0, 0, 0, 0], cfg)
    o, R = _camera_looking_down(0.0)
    caster.set_camera_pose(o, R)
    plane_z = 0.041
    L, W = cfg.container_long_m, cfg.container_short_m
    # container com o eixo longo ao longo de X, centro deslocado para fora da
    # imagem pela ESQUERDA: so' ~60% do lado longo aparece.
    yaw = 0.0
    cx_true, cy_true = -0.19, 0.28
    poly = _project_rect(K, o, R, cx_true, cy_true, yaw, L, W, plane_z)
    img = np.full((480, 640, 3), 200, np.uint8)
    cv2.fillPoly(img, [poly.astype(np.int32)], _hsv_bgr(3))
    per, _h, masks = cd.segment_image(img, cfg, holding=False)
    blob = [b for b in per["vermelho"] if b.reason is None][0]
    assert blob.partial
    excl = masks["__garra__"]
    # sem extensao: centroide da parte visivel (puxado para a direita)
    cfg.partial_extend_enable = False
    p0, why0 = caster.container_pose(blob, plane_z, yaw_hint=yaw, excl_mask=excl, scale=cfg.process_scale)
    assert p0 is not None, why0
    assert p0.x - cx_true > 0.02
    # com extensao: volta para perto do centro real
    cfg.partial_extend_enable = True
    p1, why1 = caster.container_pose(blob, plane_z, yaw_hint=yaw, excl_mask=excl, scale=cfg.process_scale)
    assert p1 is not None, why1
    assert p1.shift_m > 0.01
    assert abs(p1.x - cx_true) < 0.012 and abs(p1.y - cy_true) < 0.012, (p1.x, p1.y)
