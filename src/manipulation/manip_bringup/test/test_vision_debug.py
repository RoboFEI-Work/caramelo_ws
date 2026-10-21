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
        [2.0, 0.19, PLANE_Z],      # fora do cone de 70 graus (distorcao dobraria)
        [0.0, 0.19, PLANE_Z],      # valido
    ])
    uv = vd.project_base_points(K, None, origin, R, pts, W, H)
    assert np.isnan(uv[:5]).all()
    assert np.isfinite(uv[5]).all()
    # Sem limites de imagem o ponto da direita projeta (fora, mas finito); o do cone nao.
    uv2 = vd.project_base_points(K, None, origin, R, pts[3:5])
    assert np.isfinite(uv2[0]).all() and uv2[0, 0] < 0
    assert np.isnan(uv2[1]).all()


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
