"""Testes do NUCLEO (sem ROS) do mapa da mesa (workstation_surface_node.py).

Cobrem: segmentacao (tampo branco, objetos coloridos, fora da mesa, garra),
projecao celula -> pixel com uma camera sintetica inclinada e a fusao de
duas vistas na mesma grade. Os codigos das celulas tem que bater com
surface_place_planner.hpp.
"""
import importlib.util
import math
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "scripts", "workstation_surface_node.py")


def _load():
    spec = importlib.util.spec_from_file_location("workstation_surface_node", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ws = _load()


def _no_gripper_cfg(**kw):
    cfg = ws.SurfaceConfig(**kw)
    cfg.exclusion_polys = []
    cfg.exclusion_polys_holding = []
    cfg.exclusion_rects = []
    return cfg


def test_cell_codes_match_planner():
    assert ws.CELL_UNKNOWN == -1 and ws.CELL_TABLE_FREE == 0
    assert ws.CELL_OFF_TABLE == 70 and ws.CELL_OBSTACLE == 100


def test_segment_table_with_cube_and_floor():
    cfg = _no_gripper_cfg()
    img = np.full((480, 640, 3), (40, 90, 40), np.uint8)          # chao verde escuro
    cv2.rectangle(img, (60, 80), (580, 440), (225, 225, 225), -1)  # tampo branco
    cv2.rectangle(img, (300, 240), (340, 280), (200, 60, 20), -1)  # cubo azul 40 px
    cv2.rectangle(img, (310, 250), (330, 270), (240, 240, 240), -1)  # etiqueta branca no cubo
    res = ws.segment_frame(img, cfg, holding=False)
    assert res.table_found
    lbl = cv2.resize(res.label, (640, 480), interpolation=cv2.INTER_NEAREST)
    assert lbl[20, 20] == ws.LBL_NONWHITE        # chao
    assert lbl[460, 600] == ws.LBL_NONWHITE
    assert lbl[150, 150] == ws.LBL_WHITE         # tampo
    assert lbl[260, 320] == ws.LBL_NONWHITE      # centro do cubo: etiqueta FECHADA vira objeto
    assert lbl[245, 305] == ws.LBL_NONWHITE      # canto do cubo
    # Chao (fora) + cubo: 2 manchas nao brancas.
    assert res.n_nonwhite_blobs == 2
    # O cubo (+dilatacao) nao engole a mesa.
    frac_white = np.count_nonzero(lbl == ws.LBL_WHITE) / lbl.size
    assert 0.55 < frac_white < 0.62


def test_gripper_regions_become_unknown():
    cfg = ws.SurfaceConfig()   # com os poligonos default dos dedos (cantos de cima)
    img = np.full((480, 640, 3), (225, 225, 225), np.uint8)
    cv2.rectangle(img, (0, 0), (70, 100), (30, 30, 200), -1)       # dedo vermelho no canto
    res = ws.segment_frame(img, cfg, holding=True)
    lbl = cv2.resize(res.label, (640, 480), interpolation=cv2.INTER_NEAREST)
    assert lbl[30, 30] == ws.LBL_UNKNOWN
    assert lbl[100, 320] == ws.LBL_UNKNOWN      # cubo carregado (centro-alto) tambem apagado
    assert lbl[400, 320] == ws.LBL_WHITE
    # Nada do vermelho do dedo virou "nao branco".
    assert np.count_nonzero(lbl == ws.LBL_NONWHITE) == 0


def test_no_table_is_flagged():
    cfg = _no_gripper_cfg()
    img = np.full((480, 640, 3), (40, 90, 40), np.uint8)
    res = ws.segment_frame(img, cfg, holding=False)
    assert not res.table_found
    assert np.count_nonzero(res.label == ws.LBL_WHITE) == 0


def test_small_white_patch_is_not_table():
    cfg = _no_gripper_cfg()
    img = np.full((480, 640, 3), (40, 90, 40), np.uint8)
    cv2.rectangle(img, (60, 80), (580, 440), (225, 225, 225), -1)   # tampo
    cv2.rectangle(img, (600, 450), (630, 475), (235, 235, 235), -1)  # etiqueta branca no chao
    res = ws.segment_frame(img, cfg, holding=False)
    lbl = cv2.resize(res.label, (640, 480), interpolation=cv2.INTER_NEAREST)
    assert lbl[462, 615] == ws.LBL_NONWHITE


def _grid_masks(cfg, white_fn):
    fusion = ws.SurfaceGridFusion(cfg)
    white = white_fn(fusion.cell_x, fusion.cell_y)
    nonwhite = ~white
    return fusion, white, nonwhite


def test_edge_lines_found_for_rectangular_table():
    cfg = _no_gripper_cfg()
    fusion, white, nonwhite = _grid_masks(
        cfg, lambda X, Y: (X > -0.40) & (X < 0.40) & (Y > 0.16) & (Y < 0.66))
    lines = ws.detect_edge_lines(white, nonwhite, fusion.cell_x, fusion.cell_y, cfg)
    assert len(lines) == 4, [ln.describe() for ln in lines]
    # Cada reta: lado positivo = fora. Testa pontos de dentro/fora.
    for ln in lines:
        assert ln.signed(0.0, 0.40) < 0.0          # centro da mesa = dentro
    outs = [(0.0, 0.10), (0.0, 0.72), (-0.50, 0.40), (0.50, 0.40)]
    for (x, y) in outs:
        assert any(ln.signed(x, y) > 0.0 for ln in lines), (x, y)


def test_edge_lines_ignore_cube_and_container():
    cfg = _no_gripper_cfg()
    # Mesa inteira vista como branca; um cubo (4 cm) e um container (17 x 10 cm) nao brancos.
    def white_fn(X, Y):
        on_table = (X > -0.40) & (X < 0.40) & (Y > 0.16) & (Y < 0.66)
        cube = (np.abs(X - 0.0) < 0.0205) & (np.abs(Y - 0.30) < 0.0205)
        cont = (np.abs(X + 0.20) < 0.085) & (np.abs(Y - 0.45) < 0.0525)
        return on_table & ~cube & ~cont
    fusion, white, nonwhite = _grid_masks(cfg, white_fn)
    lines = ws.detect_edge_lines(white, nonwhite, fusion.cell_x, fusion.cell_y, cfg)
    # So' as 4 bordas da mesa; nenhuma reta passa pelo cubo/container.
    assert len(lines) == 4, [ln.describe() for ln in lines]
    for ln in lines:
        assert abs(ln.signed(0.0, 0.30)) > 0.05 and abs(ln.signed(-0.20, 0.45)) > 0.05
    cells = fusion.cells.__func__  # noqa: F841  (so' garante que o metodo existe)


def test_aligned_containers_do_not_make_an_edge():
    """Dois containers com as faces alinhadas (reta de ~40 cm) com tampo branco
    ALEM deles: nao e' borda (fotos reais de 25/08)."""
    cfg = _no_gripper_cfg()

    def white_fn(X, Y):
        on_table = (X > -0.40) & (X < 0.40) & (Y > 0.16) & (Y < 0.66)
        c1 = (np.abs(X + 0.12) < 0.085) & (Y > 0.20) & (Y < 0.30)
        c2 = (np.abs(X - 0.12) < 0.085) & (Y > 0.20) & (Y < 0.30)
        return on_table & ~c1 & ~c2
    fusion = ws.SurfaceGridFusion(cfg)
    X, Y = fusion.cell_x, fusion.cell_y
    seen = (np.abs(X) < 0.30) & (Y > 0.17) & (Y < 0.40)    # so' a vista central, sem bordas da mesa
    white = seen & white_fn(X, Y)
    nonwhite = seen & ~white_fn(X, Y)
    lines = ws.detect_edge_lines(white, nonwhite, X, Y, cfg)
    assert lines == [], [ln.describe() for ln in lines]
    fusion.votes_white[white] = 1
    fusion.votes_nonwhite[nonwhite] = 1
    cells = fusion.cells()
    i = int((0.12 - cfg.grid_x_min) / cfg.resolution)
    j = int((0.25 - cfg.grid_y_min) / cfg.resolution)
    assert cells[j, i] == ws.CELL_OBSTACLE
    assert np.count_nonzero(cells == ws.CELL_OFF_TABLE) == 0


def test_cells_classification_with_lines():
    cfg = _no_gripper_cfg()
    fusion = ws.SurfaceGridFusion(cfg)
    X, Y = fusion.cell_x, fusion.cell_y
    on_table = (X > -0.40) & (X < 0.40) & (Y > 0.16) & (Y < 0.66)
    cube = (np.abs(X - 0.10) < 0.0205) & (np.abs(Y - 0.30) < 0.0205)
    seen = (np.abs(X) < 0.55) & (Y < 0.72)
    fusion.votes_white[seen & on_table & ~cube] = 1
    fusion.votes_nonwhite[seen & (~on_table | cube)] = 1
    fusion.frames = 1
    cells = fusion.cells()
    assert len(fusion.lines) >= 3

    def cell(x, y):
        i = int((x - cfg.grid_x_min) / cfg.resolution)
        j = int((y - cfg.grid_y_min) / cfg.resolution)
        return cells[j, i]
    assert cell(0.0, 0.30) == ws.CELL_TABLE_FREE
    assert cell(0.10, 0.30) == ws.CELL_OBSTACLE       # cubo: nao branco sem reta = objeto
    assert cell(0.0, 0.10) == ws.CELL_OFF_TABLE       # antes da frente
    assert cell(0.0, 0.70) == ws.CELL_OFF_TABLE       # alem do fundo
    assert cell(0.50, 0.40) == ws.CELL_OFF_TABLE      # alem da direita
    assert cell(0.58, 0.40) == ws.CELL_UNKNOWN        # nunca visto


class _SyntheticCamera:
    """Camera pinhole olhando para baixo e para a frente, como a do punho em pegar_obj."""

    def __init__(self, origin, pitch_down_deg):
        self.K = np.array([[380.0, 0.0, 320.0], [0.0, 380.0, 240.0], [0.0, 0.0, 1.0]])
        self.W, self.H = 640, 480
        self.origin = np.array(origin, dtype=float)
        # Frame optico: z para frente, x direita, y para baixo. Queremos z
        # apontando para -Z do mundo (baixo) inclinado para +Y (frente).
        a = math.radians(pitch_down_deg)
        z = np.array([0.0, math.cos(a), -math.sin(a)])
        x = np.array([-1.0, 0.0, 0.0])      # imagem espelhada como na camera real (esq. da imagem = +X)
        y = np.cross(z, x)
        self.R = np.stack([x, y, z], axis=1)   # colunas = eixos da camera no mundo

    def project(self, p):
        pc = self.R.T @ (np.asarray(p) - self.origin)
        if pc[2] <= 0:
            return None
        u = self.K[0, 0] * pc[0] / pc[2] + self.K[0, 2]
        v = self.K[1, 1] * pc[1] / pc[2] + self.K[1, 2]
        return u, v


def test_projection_roundtrip_single_cell():
    cfg = _no_gripper_cfg(grid_x_min=-0.3, grid_x_max=0.3, grid_y_min=0.05, grid_y_max=0.45, resolution=0.01)
    fusion = ws.SurfaceGridFusion(cfg)
    cam = _SyntheticCamera(origin=(0.0, 0.16, 0.30), pitch_down_deg=80.0)
    plane_z = 0.0
    uv = fusion.project_cells(cam.K, None, cam.origin, cam.R, plane_z, cam.W, cam.H)
    # Celula (x=0.10, y=0.25): compara com a projecao direta.
    i = int((0.10 - cfg.grid_x_min) / cfg.resolution)
    j = int((0.25 - cfg.grid_y_min) / cfg.resolution)
    exp = cam.project((fusion.cell_x[j, i], fusion.cell_y[j, i], plane_z))
    assert exp is not None
    assert np.isfinite(uv[j, i]).all()
    assert abs(uv[j, i, 0] - exp[0]) < 0.5 and abs(uv[j, i, 1] - exp[1]) < 0.5
    # Celula longe demais para a frente (abaixo da borda inferior da imagem) e' NaN.
    cfg2 = _no_gripper_cfg(grid_x_min=-0.3, grid_x_max=0.3, grid_y_min=0.05, grid_y_max=0.65, resolution=0.01)
    fusion2 = ws.SurfaceGridFusion(cfg2)
    uv2 = fusion2.project_cells(cam.K, None, cam.origin, cam.R, plane_z, cam.W, cam.H)
    j_far = int((0.60 - cfg2.grid_y_min) / cfg2.resolution)
    assert not np.isfinite(uv2[j_far, i]).all()


def _render_table(cam, plane_z, table, cubes, W=640, H=480):
    """Imagem sintetica: para cada pixel, ray-cast no plano e pinta tampo/cubo/chao."""
    img = np.full((H, W, 3), (40, 90, 40), np.uint8)
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    d = np.stack([(us - cam.K[0, 2]) / cam.K[0, 0], (vs - cam.K[1, 2]) / cam.K[1, 1], np.ones_like(us, dtype=float)], axis=-1)
    dw = d @ cam.R.T
    t = (plane_z - cam.origin[2]) / dw[..., 2]
    ok = (dw[..., 2] < 0) & (t > 0)
    px = cam.origin[0] + t * dw[..., 0]
    py = cam.origin[1] + t * dw[..., 1]
    on_table = ok & (px > table[0]) & (px < table[1]) & (py > table[2]) & (py < table[3])
    img[on_table] = (225, 225, 225)
    for (cx, cy, side) in cubes:
        on_cube = on_table & (np.abs(px - cx) < side / 2) & (np.abs(py - cy) < side / 2)
        img[on_cube] = (200, 60, 20)
    return img


def test_fuse_two_views_builds_consistent_grid():
    cfg = _no_gripper_cfg(grid_x_min=-0.6, grid_x_max=0.6, grid_y_min=0.05, grid_y_max=0.75, resolution=0.01)
    fusion = ws.SurfaceGridFusion(cfg)
    plane_z = 0.0
    table = (-0.40, 0.40, 0.16, 0.66)         # x_min, x_max, y_front, y_back
    cubes = [(0.0, 0.30, 0.041), (0.25, 0.40, 0.041)]
    # Vista 1: centro (como pegar_obj). Vista 2: deslocada para a direita.
    for origin in ((0.0, 0.16, 0.30), (0.22, 0.20, 0.34)):
        cam = _SyntheticCamera(origin=origin, pitch_down_deg=80.0)
        img = _render_table(cam, plane_z, table, cubes)
        res = ws.segment_frame(img, cfg, holding=False)
        assert res.table_found
        uv = fusion.project_cells(cam.K, None, cam.origin, cam.R, plane_z, cam.W, cam.H)
        fusion.fuse(res.label, cfg.process_scale, uv)
    cells = fusion.cells()
    assert fusion.frames == 2

    def cell(x, y):
        i = int((x - cfg.grid_x_min) / cfg.resolution)
        j = int((y - cfg.grid_y_min) / cfg.resolution)
        return cells[j, i]
    assert cell(0.0, 0.30) == ws.CELL_OBSTACLE          # cubo 1
    assert cell(0.25, 0.40) == ws.CELL_OBSTACLE         # cubo 2 (visto pela 2a camera)
    assert cell(-0.10, 0.25) == ws.CELL_TABLE_FREE      # tampo livre
    assert cell(0.10, 0.22) == ws.CELL_TABLE_FREE
    assert cell(0.0, 0.12) == ws.CELL_OFF_TABLE         # antes da frente da mesa (reta vista)
    assert cell(-0.55, 0.30) in (ws.CELL_OFF_TABLE, ws.CELL_OBSTACLE, ws.CELL_UNKNOWN)
    assert any("frente" in ln.describe() for ln in fusion.lines), [ln.describe() for ln in fusion.lines]
    # A frente e' uma linha reta em y~0.16: acima dela tampo, abaixo fora.
    for x in (-0.15, 0.0, 0.15):
        assert cell(x, 0.14) == ws.CELL_OFF_TABLE
        assert cell(x, 0.19) == ws.CELL_TABLE_FREE
    # Objetos (+ chao sem reta) nunca engolem o tampo: ha bastante livre.
    free = np.count_nonzero(cells == ws.CELL_TABLE_FREE)
    assert free > 1500   # ~0,15 m2 de tampo livre visto


def test_config_from_dict_and_polys():
    cfg = ws.config_from_dict({
        "white_v_min": "90", "resolution": 0.02,
        "exclusion_polys_norm": ["0,0 0.1,0 0.1,0.1"],
        "exclusion_polys_holding_norm": [""],
        "exclusion_rects_norm": [0.3, 0.0, 0.6, 0.4],
    })
    assert cfg.white_v_min == 90 and cfg.resolution == 0.02
    assert len(cfg.exclusion_polys) == 1 and cfg.exclusion_polys_holding == []
    assert cfg.exclusion_rects == [(0.3, 0.0, 0.6, 0.4)]
    try:
        ws.config_from_dict({"resolution": -1})
        assert False, "resolucao negativa aceita"
    except ValueError:
        pass
