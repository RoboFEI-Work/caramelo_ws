"""Testes do NUCLEO (sem ROS) da visao de depuracao (vision_debug_node.py).

Cobrem: projecao base -> pixel com a mesma camera sintetica do mapa da mesa
(0,31 m acima do tampo, inclinada 84 graus como pegar_obj), pintura da grade
por classe, cantos do container no frame base, desenho das tags, barra de
status e o gate por idade.
"""
import importlib.util
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "scripts", "vision_debug_node.py")
SURFACE_SCRIPT = os.path.join(HERE, "..", "scripts", "workstation_surface_node.py")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


vd = _load("vision_debug_node", SCRIPT)

W, H = 640, 480
K = np.array([[380.0, 0.0, W / 2.0], [0.0, 380.0, H / 2.0], [0.0, 0.0, 1.0]])
PLANE_Z = 0.0


def _camera(origin=(0.0, 0.16, 0.31), pitch_deg=84.0):
    """Mesma montagem de pose do run_offline do workstation_surface_node (--camera)."""
    a = math.radians(pitch_deg)
    zax = np.array([0.0, math.cos(a), -math.sin(a)])
    xax = np.array([-1.0, 0.0, 0.0])
    yax = np.cross(zax, xax)
    R = np.stack([xax, yax, zax], axis=1)
    return np.array(origin, dtype=float), R


def _pinhole(origin, R, p):
    pc = R.T @ (np.asarray(p, dtype=float) - origin)
    return K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2]


def test_cell_codes_match_surface_node():
    ws = _load("workstation_surface_node", SURFACE_SCRIPT)
    assert (vd.CELL_UNKNOWN, vd.CELL_TABLE_FREE, vd.CELL_OFF_TABLE, vd.CELL_OBSTACLE) == \
        (ws.CELL_UNKNOWN, ws.CELL_TABLE_FREE, ws.CELL_OFF_TABLE, ws.CELL_OBSTACLE)


def test_grid_cell_centers():
    X, Y = vd.grid_cell_centers(-0.01, 0.18, 0.01, 2, 2)
    assert X.shape == (2, 2) and Y.shape == (2, 2)
    assert abs(X[0, 0] + 0.005) < 1e-9 and abs(X[0, 1] - 0.005) < 1e-9
    assert abs(Y[0, 0] - 0.185) < 1e-9 and abs(Y[1, 0] - 0.195) < 1e-9


def test_project_matches_pinhole_and_same_as_surface_projection():
    origin, R = _camera()
    pts = np.array([[0.0, 0.19, PLANE_Z], [0.10, 0.25, PLANE_Z], [-0.08, 0.15, PLANE_Z]])
    uv = vd.project_base_points(K, None, origin, R, pts, W, H)
    assert np.isfinite(uv).all()
    for p, (u, v) in zip(pts, uv):
        eu, ev = _pinhole(origin, R, p)
        assert abs(u - eu) < 0.5 and abs(v - ev) < 0.5
    # A mesma conta do mapa da mesa (SurfaceGridFusion.project_cells).
    ws = _load("workstation_surface_node", SURFACE_SCRIPT)
    cfg = ws.SurfaceConfig(grid_x_min=-0.1, grid_x_max=0.1, grid_y_min=0.15, grid_y_max=0.25, resolution=0.01)
    fusion = ws.SurfaceGridFusion(cfg)
    ref = fusion.project_cells(K, None, origin, R, PLANE_Z, W, H).reshape(-1, 2)
    X, Y = vd.grid_cell_centers(cfg.grid_x_min, cfg.grid_y_min, cfg.resolution, fusion.width, fusion.height)
    mine = vd.project_base_points(K, None, origin, R,
                                  np.stack([X.ravel(), Y.ravel(), np.full(X.size, PLANE_Z)], axis=1), W, H)
    both = np.isfinite(ref).all(axis=1) & np.isfinite(mine).all(axis=1)
    assert both.sum() > 100
    assert np.abs(ref[both] - mine[both]).max() < 0.01


def test_pinhole_project_matches_cv2_with_distortion():
    import cv2
    rng = np.random.default_rng(7)
    Pc = np.stack([rng.uniform(-0.3, 0.3, 500), rng.uniform(-0.2, 0.2, 500), rng.uniform(0.2, 0.8, 500)], axis=1)
    for D in (None, np.zeros(5), np.array([-0.05, 0.06, 0.001, -0.002]), np.array([-0.05, 0.06, 0.001, -0.002, 0.01]),
              np.array([-0.05, 0.06, 0.001, -0.002, 0.01, 0.02, -0.01, 0.005]),
              np.array([-0.05, 0.06, 0.001, -0.002, 0.01, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0])):
        ref, _ = cv2.projectPoints(Pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K,
                                   np.zeros(5) if D is None else D)
        mine = vd.pinhole_project(K, D, Pc)
        assert np.abs(mine - ref.reshape(-1, 2)).max() < 1e-4, D


def test_project_invalid_points_are_nan():
    origin, R = _camera()
    pts = np.array([
        [0.0, -0.50, PLANE_Z],     # atras da camera (ela olha para +Y)
        [0.0, 0.19, 0.50],         # acima da camera
        [0.0, 0.19, 0.31],         # na altura da camera (d.z = 0)
        [0.40, 0.19, PLANE_Z],     # fora da imagem (40 cm a direita = u < 0, imagem espelhada)
        [2.0, 0.19, PLANE_Z],      # fora do cone de 55 graus (distorcao dobraria)
        [0.0, 0.19, PLANE_Z],      # valido
    ])
    uv = vd.project_base_points(K, None, origin, R, pts, W, H)
    assert np.isnan(uv[:5]).all()
    assert np.isfinite(uv[5]).all()
    # Sem limites de imagem o ponto da direita projeta (fora, mas finito); o do cone nao.
    uv2 = vd.project_base_points(K, None, origin, R, pts[3:5])
    assert np.isfinite(uv2[0]).all() and uv2[0, 0] < 0
    assert np.isnan(uv2[1]).all()


def test_project_base_points_d455_distortion_has_no_fold():
    """Mesma guarda do workstation_surface_node: com o D real da D455 nenhum
    ponto alem do cone (diagonal da imagem = 46 graus) ganha pixel."""
    ws = _load("workstation_surface_node", SURFACE_SCRIPT)
    assert vd.MAX_HALF_ANGLE_DEG == ws.MAX_RAY_HALF_ANGLE_DEG
    D = np.array([-0.0561, 0.0652, -0.0003, 0.0005, -0.0208])
    assert vd.distortion_fold_tan(D) == ws.distortion_fold_tan(D)
    assert math.tan(math.radians(vd.MAX_HALF_ANGLE_DEG)) < vd.distortion_fold_tan(D) < math.tan(math.radians(62))
    origin, R = _camera(origin=(-0.011, 0.164, 0.311), pitch_deg=84.4)
    X, Y = vd.grid_cell_centers(-0.6, 0.05, 0.01, 120, 70)
    pts = np.stack([X.ravel(), Y.ravel(), np.full(X.size, 0.083)], axis=1)
    uv = vd.project_base_points(K, D, origin, R, pts, W, H)
    uv0 = vd.project_base_points(K, None, origin, R, pts, W, H)
    ok, ok0 = np.isfinite(uv[:, 0]), np.isfinite(uv0[:, 0])
    Pc = (R.T @ (pts - origin).T).T
    ang = np.degrees(np.arccos(np.clip(Pc[:, 2] / np.linalg.norm(Pc, axis=1), -1.0, 1.0)))
    assert ok.sum() > 500 and ang[ok].max() < 47.0, ang[ok].max()
    assert abs(int(ok.sum()) - int(ok0.sum())) < 0.05 * ok0.sum()
    # Sem limites de imagem, um ponto a 65 graus (dobrado pelo polinomio) continua NaN.
    far = np.array([[1.2, 0.19, 0.083]])
    pcf = R.T @ (far[0] - origin)
    assert math.degrees(math.acos(pcf[2] / np.linalg.norm(pcf))) > 60.0
    assert np.isnan(vd.project_base_points(K, D, origin, R, far)).all()
    # Distorcao fortissima (dobra a 39 graus): a guarda de D barra antes do cone.
    strong = np.array([-0.5, 0.0, 0.0, 0.0, 0.0])
    uvs = vd.project_base_points(K, strong, origin, R, pts, W, H)
    oks = np.isfinite(uvs[:, 0])
    assert oks.any() and ang[oks].max() <= math.degrees(math.atan(0.95 * vd.distortion_fold_tan(strong))) + 1e-6


def test_paint_grid_colors_by_class_and_leaves_unknown():
    origin, R = _camera()
    X, Y = vd.grid_cell_centers(-0.01, 0.18, 0.01, 2, 2)
    pts = np.stack([X.ravel(), Y.ravel(), np.full(4, PLANE_Z)], axis=1)
    uv = vd.project_base_points(K, None, origin, R, pts, W, H)
    assert np.isfinite(uv).all()
    codes = np.array([vd.CELL_TABLE_FREE, vd.CELL_OBSTACLE, vd.CELL_OFF_TABLE, vd.CELL_UNKNOWN], dtype=np.int8)
    img = np.full((H, W, 3), 100, np.uint8)
    out = vd.paint_grid(img, uv, codes, alpha=0.35)
    assert out is img

    def px(i):
        u, v = int(round(uv[i, 0])), int(round(uv[i, 1]))
        return tuple(int(c) for c in img[v, u])
    b, g, r = px(0)
    assert g > 100 and r < 100 and b < 100, px(0)            # livre: verde
    b, g, r = px(1)
    assert r > 100 and g < 100 and b < 100, px(1)            # objeto: vermelho
    b, g, r = px(2)
    assert b == g == r and b > 100, px(2)                    # fora da mesa: cinza (mais claro que o fundo)
    assert px(3) == (100, 100, 100), px(3)                   # desconhecido: intocado
    # Celulas vizinhas a 1 cm ficam ~12 px afastadas: o splat cobre o espaco entre elas.
    um = (uv[0] + uv[1]) / 2.0
    assert tuple(img[int(round(um[1])), int(round(um[0]))]) != (100, 100, 100)


def test_paint_grid_all_nan_or_unknown_is_noop():
    img = np.full((H, W, 3), 200, np.uint8)
    uv = np.full((4, 2), np.nan, np.float32)
    vd.paint_grid(img, uv, np.zeros(4, np.int8))
    assert (img == 200).all()
    vd.paint_grid(img, np.array([[10.0, 10.0]]), np.array([vd.CELL_UNKNOWN]))
    assert (img == 200).all()


def test_container_corners_base():
    yaw = math.radians(30.0)
    T = vd.pose_to_matrix(0.10, 0.30, 0.04, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    c = vd.container_corners_base(T, 0.170, 0.105)
    assert c.shape == (4, 3)
    assert np.allclose(c.mean(axis=0), (0.10, 0.30, 0.04))
    assert np.allclose(c[:, 2], 0.04)
    e_long = c[0] - c[1]
    e_short = c[1] - c[2]
    assert abs(np.linalg.norm(e_long) - 0.170) < 1e-9
    assert abs(np.linalg.norm(e_short) - 0.105) < 1e-9
    assert abs(np.linalg.norm(c[2] - c[3]) - 0.170) < 1e-9
    # Lado longo = eixo X do frame, girado pelo yaw.
    assert np.allclose(e_long / np.linalg.norm(e_long), (math.cos(yaw), math.sin(yaw), 0.0))
    bottom = vd.container_corners_base(T, 0.170, 0.105, dz=-0.070)
    assert np.allclose(bottom[:, 2], 0.04 - 0.070)
    assert np.allclose(bottom[:, :2], c[:, :2])


def test_draw_tags_paints_outline():
    img = np.zeros((H, W, 3), np.uint8)
    dets = [{"id": 7, "hamming": 0, "decision_margin": 55.0, "centre": (150.0, 150.0),
             "corners": [(100.0, 100.0), (200.0, 100.0), (200.0, 200.0), (100.0, 200.0)]}]
    out = vd.draw_tags(img, dets)
    assert out is img
    assert tuple(img[100, 150]) == vd.COLOR_TAG            # aresta de cima
    assert tuple(img[150, 200]) == vd.COLOR_TAG            # aresta direita
    assert tuple(img[150, 150]) == vd.COLOR_TAG            # ponto no centro
    assert (img[150, 110] == 0).all()                      # interior nao pintado
    assert np.count_nonzero(img[:100, 100:200]) > 0        # rotulo 'tag_7' acima
    img2 = np.zeros((H, W, 3), np.uint8)
    dets[0]["hamming"] = 1
    vd.draw_tags(img2, dets)
    assert tuple(img2[100, 150]) == vd.COLOR_TAG_HAMMING
    # Entradas degeneradas nao quebram.
    vd.draw_tags(img2, [{"id": 1, "corners": [(float("nan"), 0.0)] * 4}, {"id": 2, "corners": []}])
    vd.draw_tags(img2, [])


def test_draw_containers_paints_box():
    img = np.zeros((H, W, 3), np.uint8)
    top = np.array([[300.0, 300.0], [400.0, 300.0], [400.0, 360.0], [300.0, 360.0]])
    bottom = top + (0.0, 40.0)
    vd.draw_containers(img, [{"name": "ct_vermelho", "color": (0, 0, 255), "top": top, "bottom": bottom, "dist_m": 0.31}])
    assert tuple(img[300, 350]) == (0, 0, 255)             # borda (traco 2)
    b, g, r = (int(c) for c in img[400, 350])
    assert r > 200 and g == 0 and b == 0                   # fundo (traco 1 com antialias)
    assert np.count_nonzero(img[361:399, 300]) > 0         # aresta vertical
    # Canto NaN (fora do cone) = nao desenha, sem erro.
    img2 = np.zeros((H, W, 3), np.uint8)
    top_nan = top.copy()
    top_nan[0] = np.nan
    vd.draw_containers(img2, [{"name": "ct_azul", "color": (255, 0, 0), "top": top_nan, "bottom": None}])
    assert not img2.any()


def test_draw_status_bar():
    img = np.full((H, W, 3), 255, np.uint8)
    out = vd.draw_status(img, ["PLACE | 9.8 fps 11 ms", "tags: 3, 7 | containers: vermelho 0.31 m"])
    assert out.shape == (H, W, 3)
    assert tuple(img[2, W - 2]) != (255, 255, 255)         # barra escurecida
    assert tuple(img[H - 2, W - 2]) == (255, 255, 255)     # resto intocado
    assert vd.draw_status(img, []) is img


def test_surface_status_prefix_and_grid_origin():
    """Prefixo latched do workstation_surface_node -> origem do mapa na barra."""
    ok = "depth=ok alem=12% acima=3% retas=1 quadros=3/3 | frames=3 mesa=sim (62% branco) | bordas: x | garra=vazia"
    st = vd.parse_surface_status(ok)
    assert st["depth"] == "ok" and st["alem"] == "12%" and st["acima"] == "3%" and st["retas"] == "1"
    assert st["quadros"] == "3/3" and "frames" not in st            # so' o prefixo (antes do ' | ')
    assert vd.grid_origin_from_status(ok) == "depth"
    # Quadro atual descartado mas a grade ja levou profundidade antes: ainda 'depth'.
    assert vd.grid_origin_from_status("depth=descartada(38%) alem=20% acima=18% retas=0 quadros=2/5 | x") == "depth"
    # Nenhum quadro com profundidade: 'cor'.
    assert vd.grid_origin_from_status("depth=sem alem=0% acima=0% retas=1 quadros=0/4 | x") == "cor"
    assert vd.grid_origin_from_status("depth=off alem=0% acima=0% retas=0 quadros=0/1") == "cor"
    assert vd.grid_origin_from_status("depth=descartada(90%) alem=50% acima=40% retas=0 quadros=0/1") == "cor"
    # Sem o campo quadros, decide depth=ok; sem prefixo (status de erro / vazio): '?'.
    assert vd.grid_origin_from_status("depth=ok alem=5% acima=0%") == "depth"
    assert vd.grid_origin_from_status("depth=velha(0.80s) alem=0% acima=0%") == "cor"
    assert vd.grid_origin_from_status("sem camera_info") == "?"
    assert vd.grid_origin_from_status("") == "?" and vd.parse_surface_status(None) == {}
    assert vd.grid_origin_from_status("depth=ok quadros=x/y") == "depth"     # quadros ilegivel: cai no depth=


def test_is_fresh_gate():
    assert vd.is_fresh(10.0, 10.3, 0.5)
    assert not vd.is_fresh(10.0, 10.6, 0.5)
    assert not vd.is_fresh(10.0, 10.5, 0.5)
    assert not vd.is_fresh(0.0, 10.0, 0.5)         # sem stamp = velho
    assert vd.is_fresh(10.5, 10.0, 0.5)            # no futuro (relogios desencontrados) = fresco


def test_quat_to_rot_and_pose_to_matrix():
    assert np.allclose(vd.quat_to_rot(0.0, 0.0, 0.0, 1.0), np.eye(3))
    R = vd.quat_to_rot(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))   # yaw 90
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), (0.0, 1.0, 0.0))
    T = vd.pose_to_matrix(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    assert np.allclose(T[:3, 3], (1.0, 2.0, 3.0)) and np.allclose(T[:3, :3], np.eye(3)) and T[3, 3] == 1.0


# ---------------------------------------------------------------------------
# Poligonos e vetores (2026-10-21)
# ---------------------------------------------------------------------------
def test_grid_class_contours_follow_cell_borders_and_holes():
    import cv2
    w, h = 30, 20
    codes = np.full((h, w), vd.CELL_UNKNOWN, np.int8)
    codes[2:18, 2:28] = vd.CELL_TABLE_FREE
    codes[8:12, 10:16] = vd.CELL_OBSTACLE          # buraco no livre
    codes[0:2, :] = vd.CELL_OFF_TABLE
    polys = vd.grid_class_contours(codes.ravel(), w, h)
    assert set(polys) == {vd.CELL_TABLE_FREE, vd.CELL_OBSTACLE, vd.CELL_OFF_TABLE}
    free = polys[vd.CELL_TABLE_FREE]
    assert len(free) == 2                           # contorno externo + buraco
    outer = max(free, key=lambda p: cv2.contourArea(p.astype(np.float32)))
    assert outer.shape[0] == 4
    # Segue a BORDA das celulas (+1/4 de celula): celulas 2..27 -> 1.25..27.75.
    assert abs(outer[:, 0].min() - 1.25) < 1e-6 and abs(outer[:, 0].max() - 27.75) < 1e-6
    assert abs(outer[:, 1].min() - 1.25) < 1e-6 and abs(outer[:, 1].max() - 17.75) < 1e-6
    obst = polys[vd.CELL_OBSTACLE][0]
    assert abs(obst[:, 0].min() - 9.25) < 1e-6 and abs(obst[:, 0].max() - 15.75) < 1e-6
    # Recorte por celula valida: metade direita invalida -> o livre acaba na celula 14.
    valid = np.ones((h, w), bool)
    valid[:, 15:] = False
    polys_v = vd.grid_class_contours(codes.ravel(), w, h, valid.ravel())
    outer_v = max(polys_v[vd.CELL_TABLE_FREE], key=lambda p: cv2.contourArea(p.astype(np.float32)))
    assert abs(outer_v[:, 0].max() - 14.75) < 1e-6
    assert vd.grid_class_contours(np.full(w * h, vd.CELL_UNKNOWN, np.int8), w, h) == {}
    xy = vd.cells_to_base(np.array([[0.0, 0.0], [1.0, 2.0]]), -0.6, 0.05, 0.01)
    assert np.allclose(xy, [[-0.595, 0.055], [-0.585, 0.075]])


def test_subdivide_polygon():
    sq = np.array([[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]])
    out = vd.subdivide_polygon(sq, 0.01, closed=True)
    assert out.shape == (40, 2)                      # 4 arestas de 10 cm a cada 1 cm
    d = np.linalg.norm(np.diff(np.vstack([out, out[:1]]), axis=0), axis=1)
    assert np.allclose(d, 0.01)
    assert np.allclose(out[0], sq[0]) and np.allclose(out[10], sq[1])
    op = vd.subdivide_polygon(sq[:2], 0.03, closed=False)
    assert op.shape[0] == 5 and np.allclose(op[-1], sq[1])      # 4 trechos de 2,5 cm + ponto final
    assert vd.subdivide_polygon(sq, 0.0).shape == (4, 2)
    assert vd.subdivide_polygon(sq[:1], 0.01).shape == (1, 2)


def test_paint_grid_polygons_fills_classes_with_priority():
    origin, R = _camera()
    free = np.array([[-0.05, 0.15], [0.05, 0.15], [0.05, 0.25], [-0.05, 0.25]])
    obst = np.array([[-0.01, 0.19], [0.01, 0.19], [0.01, 0.21], [-0.01, 0.21]])
    px = {vd.CELL_TABLE_FREE: [vd.project_base_polygon(K, None, origin, R, free, PLANE_Z)],
          vd.CELL_OBSTACLE: [vd.project_base_polygon(K, None, origin, R, obst, PLANE_Z)]}
    assert px[vd.CELL_TABLE_FREE][0].shape[0] == 40 and np.isfinite(px[vd.CELL_TABLE_FREE][0]).all()
    img = np.full((H, W, 3), 100, np.uint8)
    assert vd.paint_grid_polygons(img, px, alpha=0.35) is img

    def at(x, y):
        u, v = vd.project_base_points(K, None, origin, R, np.array([[x, y, PLANE_Z]]))[0]
        return tuple(int(c) for c in img[int(round(v)), int(round(u))])
    b, g, r = at(-0.03, 0.17)
    assert g > 100 and r < 100 and b < 100, (b, g, r)       # livre: verde
    b, g, r = at(0.0, 0.20)
    assert r > 100 and g < 100, (b, g, r)                    # objeto por cima do livre
    assert at(0.0, 0.30) == (100, 100, 100)                  # fora dos poligonos: intocado
    # Vertice NaN descartado sem erro (sobram 2 pontos: nada); vazio = no-op.
    img2 = np.full((H, W, 3), 100, np.uint8)
    vd.paint_grid_polygons(img2, {vd.CELL_TABLE_FREE: [np.array([[np.nan, np.nan], [10.0, 10.0], [20.0, 10.0]])]})
    vd.paint_grid_polygons(img2, {})
    assert (img2 == 100).all()


def _marker(ns, i, pts):
    return {"ns": ns, "id": i, "points": [(x, y, 0.0) for (x, y) in pts]}


def _rect_markers(front=True, left=True, right=True, back=False):
    ms = []
    i = 0
    if front:   # y = 0.16, fora = -Y (lado do robo)
        ms += [_marker("edge", i, [(-0.30, 0.16), (0.30, 0.16)]), _marker("edge_normal", i, [(0.0, 0.16), (0.0, 0.11)])]
        i += 1
    if right:   # x = 0.40, fora = +X
        ms += [_marker("edge", i, [(0.40, 0.20), (0.40, 0.60)]), _marker("edge_normal", i, [(0.40, 0.40), (0.45, 0.40)])]
        i += 1
    if left:
        ms += [_marker("edge", i, [(-0.40, 0.60), (-0.40, 0.20)]), _marker("edge_normal", i, [(-0.40, 0.40), (-0.45, 0.40)])]
        i += 1
    if back:
        ms += [_marker("edge", i, [(0.30, 0.66), (-0.30, 0.66)]), _marker("edge_normal", i, [(0.0, 0.66), (0.0, 0.71)])]
        i += 1
    return ms


def test_edge_lines_from_markers_and_kinds():
    lines = vd.edge_lines_from_markers(_rect_markers(back=True))
    assert [ln["kind"] for ln in lines] == ["frente", "direita", "esquerda", "fundo"]
    f = lines[0]
    assert np.allclose(f["n"], (0.0, -1.0)) and abs(f["length"] - 0.60) < 1e-9
    assert f["a"] * 0.0 + f["b"] * 0.10 + f["c"] > 0           # lado do robo = fora
    assert f["a"] * 0.0 + f["b"] * 0.30 + f["c"] < 0           # tampo = dentro
    r = lines[1]
    assert r["a"] * 0.50 + r["b"] * 0.40 + r["c"] > 0 and r["a"] * 0.0 + r["b"] * 0.40 + r["c"] < 0
    assert vd.edge_color("frente") == vd.COLOR_EDGE_FRONT and vd.edge_color("esquerda") == vd.COLOR_EDGE_SIDE
    # Reta sem seta descartada; seta sem reta ignorada; marcador degenerado nao quebra.
    assert vd.edge_lines_from_markers([
        _marker("edge", 7, [(0, 0), (1, 0)]), _marker("edge_normal", 9, [(0, 0), (0, 1)]),
        _marker("edge", 8, [(0, 0)]), {"ns": "edge", "id": 1}]) == []


def test_manipulation_zone_polygon():
    lines = vd.edge_lines_from_markers(_rect_markers())
    z = vd.manipulation_zone_polygon(lines, 0.20, 0.02, 0.02)
    assert z is not None and z.shape == (4, 2)
    assert abs(z[:, 1].min() - 0.18) < 1e-9 and abs(z[:, 1].max() - 0.36) < 1e-9     # 2 a 20 cm da frente
    assert abs(z[:, 0].min() + 0.38) < 1e-9 and abs(z[:, 0].max() - 0.38) < 1e-9     # 2 cm das laterais
    # So' a frente: limites laterais = pontas do trecho VISTO (+-0.30).
    z2 = vd.manipulation_zone_polygon(vd.edge_lines_from_markers(_rect_markers(left=False, right=False)))
    assert abs(z2[:, 0].min() + 0.30) < 1e-9 and abs(z2[:, 0].max() - 0.30) < 1e-9
    assert abs(z2[:, 1].min() - 0.18) < 1e-9 and abs(z2[:, 1].max() - 0.36) < 1e-9
    assert vd.manipulation_zone_polygon(vd.edge_lines_from_markers(_rect_markers(front=False))) is None
    assert vd.manipulation_zone_polygon([]) is None


def test_draw_vectors_paint_something():
    origin, R = _camera()
    lines = vd.edge_lines_from_markers(_rect_markers(left=False, right=False))
    img = np.zeros((H, W, 3), np.uint8)
    vd.draw_edge_lines(img, K, None, origin, R, lines)
    assert np.count_nonzero((img == vd.COLOR_EDGE_FRONT).all(axis=2)) > 50
    zone = vd.manipulation_zone_polygon(lines)
    img2 = np.zeros((H, W, 3), np.uint8)
    vd.draw_zone(img2, K, None, origin, R, zone, PLANE_Z)
    n_zone = int(np.count_nonzero((img2 == vd.COLOR_ZONE).all(axis=2)))
    assert n_zone > 50
    # Tracejado pinta menos que o traco continuo.
    img3 = np.zeros((H, W, 3), np.uint8)
    uv = vd.project_base_polygon(K, None, origin, R, zone, PLANE_Z)
    vd.draw_base_polyline(img3, uv, vd.COLOR_ZONE, 2, closed=True, dashed=False)
    assert np.count_nonzero((img3 == vd.COLOR_ZONE).all(axis=2)) > n_zone
    # Polilinha com NaN no meio: so' pula aquele trecho.
    img4 = np.zeros((H, W, 3), np.uint8)
    vd.draw_base_polyline(img4, np.array([[10.0, 10.0], [100.0, 10.0], [np.nan, np.nan], [100.0, 100.0]]),
                          (255, 255, 255), 1, closed=False)
    assert img4[10, 50].all() and not img4[55, 100].any()
    assert vd.draw_base_polyline(img4, np.zeros((1, 2)), (1, 1, 1)) is img4
