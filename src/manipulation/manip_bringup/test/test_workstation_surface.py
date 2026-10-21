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


# camera_info REAL da cor da D455 em 640x480 (plumb_bob, k3 < 0): o polinomio
# radial deixa de crescer a ~59 graus do eixo e dobra pontos de fora do FOV
# para dentro da imagem.
D455_D = np.array([-0.0561, 0.0652, -0.0003, 0.0005, -0.0208])


def test_distortion_fold_tan():
    assert ws.distortion_fold_tan(None) == float("inf")
    assert ws.distortion_fold_tan(np.zeros(5)) == float("inf")
    assert ws.distortion_fold_tan(np.array([0.1, 0.2, 0.3])) == float("inf")      # tamanho fora de 4/5/8: sem guarda
    r = ws.distortion_fold_tan(D455_D)
    assert math.tan(math.radians(56)) < r < math.tan(math.radians(62)), math.degrees(math.atan(r))
    assert ws.distortion_fold_tan(np.array([-0.5, 0.0, 0.0, 0.0, 0.0])) < math.tan(math.radians(45))   # forte: dobra cedo
    # O cone fixo fica ABAIXO da dobra da D455 e acima da diagonal da imagem (46 graus).
    assert 46.0 < ws.MAX_RAY_HALF_ANGLE_DEG < math.degrees(math.atan(r))


def test_project_cells_with_d455_distortion_has_no_folded_cells():
    """Bug de 21/10: com o D real, cv2.projectPoints devolvia pixels 'dentro'
    da imagem para 1000+ celulas a > 59 graus do eixo (fora do FOV), que
    herdavam o rotulo de pixels do centro e faziam retas laterais falsas."""
    cfg = _no_gripper_cfg()
    fusion = ws.SurfaceGridFusion(cfg)
    cam = _SyntheticCamera(origin=(-0.011, 0.164, 0.311), pitch_down_deg=84.4)   # pegar_obj real (docs)
    cam.K = np.array([[385.0, 0.0, 320.0], [0.0, 385.0, 240.0], [0.0, 0.0, 1.0]])
    scale = cfg.process_scale
    w, h = int(cam.W * scale), int(cam.H * scale)
    for plane_z in (0.083, -0.029):
        uv_d = fusion.project_cells(cam.K, D455_D, cam.origin, cam.R, plane_z, cam.W, cam.H)
        uv_0 = fusion.project_cells(cam.K, None, cam.origin, cam.R, plane_z, cam.W, cam.H)
        seen_d = np.isfinite(uv_d[..., 0])
        seen_0 = np.isfinite(uv_0[..., 0])
        P = np.stack([fusion.cell_x.ravel(), fusion.cell_y.ravel(), np.full(fusion.cell_x.size, plane_z)], axis=1)
        Pc = (cam.R.T @ (P - cam.origin).T).T
        ang = np.degrees(np.arccos(np.clip(Pc[:, 2] / np.linalg.norm(Pc, axis=1), -1.0, 1.0))).reshape(seen_d.shape)
        assert seen_d.sum() > 500
        # Nenhuma celula alem do cone recebe pixel; a diagonal da imagem e' 46 graus.
        assert ang[seen_d].max() <= ws.MAX_RAY_HALF_ANGLE_DEG + 1e-6
        assert ang[seen_d].max() < 47.0, ang[seen_d].max()
        # Com e sem D veem praticamente as mesmas celulas (barrel ~1%: so' a borda muda).
        assert abs(int(seen_d.sum()) - int(seen_0.sum())) < 0.05 * seen_0.sum(), (seen_d.sum(), seen_0.sum())
        # pixel -> raio (expected_plane_range, undistortPoints) e celula -> pixel
        # (project_cells, projectPoints) sao o mesmo modelo: z optico concorda
        # em TODAS as celulas vistas (dobradas discordavam ate 6,7 cm).
        exp = ws.expected_plane_range(cam.K, D455_D, cam.origin, cam.R, plane_z, w, h, scale)
        us = np.clip(np.round(uv_d[..., 0][seen_d] * scale).astype(int), 0, w - 1)
        vs = np.clip(np.round(uv_d[..., 1][seen_d] * scale).astype(int), 0, h - 1)
        err = np.abs(exp[vs, us] - Pc[:, 2].reshape(seen_d.shape)[seen_d])
        assert np.isfinite(err).all() and err.max() < 0.01, err.max()
    # Distorcao fortissima (dobra a 39 graus): a guarda extra barra antes do cone.
    strong = np.array([-0.5, 0.0, 0.0, 0.0, 0.0])
    uv_s = fusion.project_cells(cam.K, strong, cam.origin, cam.R, 0.083, cam.W, cam.H)
    seen_s = np.isfinite(uv_s[..., 0])
    P = np.stack([fusion.cell_x.ravel(), fusion.cell_y.ravel(), np.full(fusion.cell_x.size, 0.083)], axis=1)
    Pc = (cam.R.T @ (P - cam.origin).T).T
    ang = np.degrees(np.arccos(np.clip(Pc[:, 2] / np.linalg.norm(Pc, axis=1), -1.0, 1.0))).reshape(seen_s.shape)
    assert seen_s.any() and ang[seen_s].max() <= math.degrees(math.atan(0.95 * ws.distortion_fold_tan(strong))) + 1e-6


def test_depth_wait_pending_rule():
    assert ws.depth_wait_pending("sem", 0.2, 1.0)                 # profundidade ainda nao chegou: espera
    assert ws.depth_wait_pending("velha(0.80s)", 0.9, 1.0)        # desencontrada: espera
    assert not ws.depth_wait_pending("sem", 1.2, 1.0)             # graca vencida: so' cor
    assert not ws.depth_wait_pending("off", 0.1, 1.0)             # depth_enable=false
    assert not ws.depth_wait_pending("enc=rgb8", 0.1, 1.0)        # encoding errado nao e' atraso
    assert not ws.depth_wait_pending("erro(ValueError)", 0.1, 1.0)
    assert not ws.depth_wait_pending(None, 0.1, 1.0)
    assert not ws.depth_wait_pending("sem", None, 1.0)            # inativo
    assert not ws.depth_wait_pending("sem", 0.1, 0.0)             # graca desligada


def _render_table(cam, plane_z, table, cubes, W=640, H=480, floor=(40, 90, 40)):
    """Imagem sintetica: para cada pixel, ray-cast no plano e pinta tampo/cubo/chao."""
    img = np.full((H, W, 3), floor, np.uint8)
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


# ---------------------------------------------------------------------------
# Profundidade: mesa x chao (2026-10-21)
# ---------------------------------------------------------------------------
def test_expected_plane_range_matches_raycast():
    cam = _SyntheticCamera(origin=(0.0, 0.16, 0.30), pitch_down_deg=80.0)
    scale = 0.5
    w, h = int(cam.W * scale), int(cam.H * scale)
    exp = ws.expected_plane_range(cam.K, None, cam.origin, cam.R, 0.0, w, h, scale)
    assert exp.shape == (h, w) and exp.dtype == np.float32

    def direct(cam, us, vs):
        u = (us + 0.5) / scale - 0.5
        v = (vs + 0.5) / scale - 0.5
        dc = np.array([(u - cam.K[0, 2]) / cam.K[0, 0], (v - cam.K[1, 2]) / cam.K[1, 1], 1.0])
        dw = cam.R @ dc
        return (0.0 - cam.origin[2]) / dw[2] if dw[2] < 0 else float("nan")
    for (us, vs) in ((w // 2, h // 2), (10, 10), (w - 5, h - 5), (3, h // 3)):
        assert abs(exp[vs, us] - direct(cam, us, vs)) < 1e-4, (us, vs)
    # Coerente com project_cells: a celula (x, y) cai num pixel cuja profundidade
    # esperada e' o z optico da propria celula.
    cfg = _no_gripper_cfg()
    fusion = ws.SurfaceGridFusion(cfg)
    uv = fusion.project_cells(cam.K, None, cam.origin, cam.R, 0.0, cam.W, cam.H)
    i = int((0.05 - cfg.grid_x_min) / cfg.resolution)
    j = int((0.30 - cfg.grid_y_min) / cfg.resolution)
    assert np.isfinite(uv[j, i]).all()
    us, vs = int(round(uv[j, i, 0] * scale)), int(round(uv[j, i, 1] * scale))
    pc = cam.R.T @ (np.array([fusion.cell_x[j, i], fusion.cell_y[j, i], 0.0]) - cam.origin)
    assert abs(exp[vs, us] - pc[2]) < 0.01
    # Camera quase horizontal: raios que nao descem ate o plano = NaN (e so' eles).
    cam2 = _SyntheticCamera(origin=(0.0, 0.16, 0.30), pitch_down_deg=20.0)
    exp2 = ws.expected_plane_range(cam2.K, None, cam2.origin, cam2.R, 0.0, w, h, scale)
    assert np.isnan(exp2).any() and np.isfinite(exp2).any()
    for (us, vs) in ((w // 2, 0), (w // 2, h - 1), (0, h // 2)):
        d = direct(cam2, us, vs)
        assert (np.isnan(exp2[vs, us]) and math.isnan(d)) or abs(exp2[vs, us] - d) < 1e-4
    # Distorcao nula explicita = mesmo resultado.
    exp3 = ws.expected_plane_range(cam.K, np.zeros(5), cam.origin, cam.R, 0.0, w, h, scale)
    assert np.allclose(exp, exp3, equal_nan=True)


def test_classify_depth_rules():
    cfg = _no_gripper_cfg()   # depth_min_valid 0.25, beyond 0.03, above 0.02, open 3
    h, w = 40, 60
    exp = np.full((h, w), 0.50, np.float32)
    depth = np.full((h, w), 0.50, np.float32)      # igual a esperada: sem decisao
    depth[:, :20] = 0.0                              # invalido (D455: 0)
    depth[:, 20:30] = 0.70                           # alem do plano (chao)
    depth[:, 30:40] = 0.40                           # aquem: algo acima do tampo
    depth[:, 40:45] = 0.20                           # perto demais (zona cega): invalido
    depth[:, 45:50] = 0.52                           # dentro da folga de 3 cm: sem decisao
    depth[5, 55] = 2.0                               # flying pixel isolado: some na abertura
    exp[:, 50:52] = np.nan                           # raio sem plano: sem decisao
    depth[:, 50:52] = 0.9
    lbl = ws.classify_depth(depth, exp, cfg)
    assert lbl.shape == (h, w) and lbl.dtype == np.uint8
    assert (lbl[:, :20] == ws.LBL_UNKNOWN).all()
    assert (lbl[:, 20:30] == ws.LBL_BEYOND).all()
    assert (lbl[:, 30:40] == ws.LBL_NONWHITE).all()
    assert (lbl[:, 40:52] == ws.LBL_UNKNOWN).all()
    assert (lbl[:, 52:] == ws.LBL_UNKNOWN).all()
    # Sem abertura o flying pixel fica.
    assert ws.classify_depth(depth, exp, _no_gripper_cfg(depth_open_px=1))[5, 55] == ws.LBL_BEYOND
    try:
        ws.classify_depth(depth[:10], exp, cfg)
        assert False, "tamanhos diferentes aceitos"
    except ValueError:
        pass


def test_depth_image_to_m():
    raw = np.array([[0, 1000], [450, 65535]], np.uint16)
    m = ws.depth_image_to_m(raw, "16UC1")
    assert m.dtype == np.float32 and m[0, 1] == 1.0 and abs(m[1, 0] - 0.45) < 1e-6 and m[0, 0] == 0.0
    assert ws.depth_image_to_m(np.array([[0.5]], np.float32), "32FC1")[0, 0] == 0.5
    assert ws.depth_image_to_m(raw, "rgb8") is None


def test_segment_frame_depth_overrides_color():
    cfg = _no_gripper_cfg()
    img = np.full((480, 640, 3), (225, 225, 225), np.uint8)       # chao CLARO: tudo "branco" por cor
    h, w = 240, 320
    dl = np.full((h, w), ws.LBL_UNKNOWN, np.uint8)
    dl[150:, :] = ws.LBL_BEYOND                                      # profundidade: parte de baixo e' chao
    dl[40:70, 100:140] = ws.LBL_NONWHITE                            # algo acima do tampo (branco)
    res = ws.segment_frame(img, cfg, holding=False, depth_label=dl)
    assert res.depth_used and res.table_found
    assert res.label[200, 160] == ws.LBL_BEYOND
    assert res.label[20, 160] == ws.LBL_WHITE
    assert res.label[55, 120] == ws.LBL_NONWHITE
    assert 0.5 < res.table_frac < 0.62                               # so' o que NAO e' chao conta como tampo
    assert abs(res.depth_beyond_frac - 90.0 / 240.0) < 0.03
    assert not res.depth_above_rejected
    # Sem profundidade o chao claro vira "tampo": o bug visto no robo.
    res0 = ws.segment_frame(img, cfg, holding=False)
    assert res0.label[200, 160] == ws.LBL_WHITE and not res0.depth_used
    # depth_enable=false ou rotulo todo "sem decisao" = identico ao sem profundidade.
    res_off = ws.segment_frame(img, _no_gripper_cfg(depth_enable=False), holding=False, depth_label=dl)
    assert np.array_equal(res_off.label, res0.label) and not res_off.depth_used
    res_u = ws.segment_frame(img, cfg, holding=False, depth_label=np.zeros((h, w), np.uint8))
    assert np.array_equal(res_u.label, res0.label)
    # Rotulo em outra resolucao e' reamostrado.
    res_r = ws.segment_frame(img, cfg, holding=False, depth_label=cv2.resize(dl, (640, 480), interpolation=cv2.INTER_NEAREST))
    assert np.array_equal(res_r.label, res.label)
    # "Acima" em quase toda a imagem = plano errado: regra ignorada, flag ligada.
    res2 = ws.segment_frame(img, cfg, holding=False, depth_label=np.full((h, w), ws.LBL_NONWHITE, np.uint8))
    assert res2.depth_above_rejected and res2.label[120, 160] == ws.LBL_WHITE


def _render_depth(cam, table, table_z, floor_z, W=640, H=480):
    """Profundidade sintetica (m, z optico como o D455): tampo INVALIDO (0, zona
    cega) e chao valido, cubos sobre o tampo tambem invalidos."""
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    d = np.stack([(us - cam.K[0, 2]) / cam.K[0, 0], (vs - cam.K[1, 2]) / cam.K[1, 1], np.ones_like(us, dtype=float)], axis=-1)
    dw = d @ cam.R.T
    with np.errstate(divide="ignore", invalid="ignore"):
        t_tab = (table_z - cam.origin[2]) / dw[..., 2]
        t_fl = (floor_z - cam.origin[2]) / dw[..., 2]
    ok = dw[..., 2] < 0
    px = cam.origin[0] + t_tab * dw[..., 0]
    py = cam.origin[1] + t_tab * dw[..., 1]
    on_table = ok & (px > table[0]) & (px < table[1]) & (py > table[2]) & (py < table[3])
    depth = np.zeros((H, W), np.float32)
    floor = ok & ~on_table
    depth[floor] = t_fl[floor]
    return depth


def test_fusion_depth_marks_floor_off_table_and_finds_front():
    cfg = _no_gripper_cfg()
    plane_z = 0.0
    table = (-0.40, 0.40, 0.16, 0.66)
    cam = _SyntheticCamera(origin=(0.0, 0.16, 0.30), pitch_down_deg=80.0)
    # Chao CLARO (mesma cor do tampo): so' por cor tudo e' "mesa". Mesa de 15 cm.
    img = _render_table(cam, plane_z, table, [(0.0, 0.30, 0.041)], floor=(225, 225, 225))
    depth = _render_depth(cam, table, plane_z, plane_z - 0.15)
    scale = cfg.process_scale
    w, h = int(cam.W * scale), int(cam.H * scale)
    exp = ws.expected_plane_range(cam.K, None, cam.origin, cam.R, plane_z, w, h, scale)
    small = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
    dl = ws.classify_depth(small, exp, cfg)
    assert np.count_nonzero(dl == ws.LBL_BEYOND) > 0.05 * dl.size
    assert np.count_nonzero(dl == ws.LBL_NONWHITE) == 0        # tampo invalido: nada "acima"

    def run(depth_label):
        fusion = ws.SurfaceGridFusion(cfg)
        res = ws.segment_frame(img, cfg, holding=False, depth_label=depth_label)
        uv = fusion.project_cells(cam.K, None, cam.origin, cam.R, plane_z, cam.W, cam.H)
        fusion.fuse(res.label, scale, uv)
        return fusion, fusion.cells()

    def cell(cells, x, y):
        i = int((x - cfg.grid_x_min) / cfg.resolution)
        j = int((y - cfg.grid_y_min) / cfg.resolution)
        return cells[j, i]
    # Sem profundidade: o chao claro vira tampo livre e nao ha frente (bug do robo).
    f0, c0 = run(None)
    assert cell(c0, 0.0, 0.10) == ws.CELL_TABLE_FREE
    assert not any("frente" in ln.describe() for ln in f0.lines)
    assert f0.summary()["beyond"] == 0
    # Com profundidade: chao = fora da mesa DIRETO e a frente aparece na RANSAC.
    f1, c1 = run(dl)
    assert cell(c1, 0.0, 0.10) == ws.CELL_OFF_TABLE
    assert cell(c1, 0.0, 0.30) == ws.CELL_OBSTACLE           # cubo
    assert cell(c1, -0.15, 0.25) == ws.CELL_TABLE_FREE
    assert f1.summary()["beyond"] > 0
    assert any("frente" in ln.describe() for ln in f1.lines), [ln.describe() for ln in f1.lines]
    for x in (-0.15, 0.0, 0.15):
        assert cell(c1, x, 0.13) == ws.CELL_OFF_TABLE
        assert cell(c1, x, 0.20) == ws.CELL_TABLE_FREE
    # Celula "alem" ganha de um voto branco de outra vista (profundidade nao atravessa mesa).
    f1.votes_white[f1.votes_beyond > 0] += 1
    assert cell(f1.cells(update_lines=False), 0.0, 0.10) == ws.CELL_OFF_TABLE


def test_fusion_white_without_depth_cannot_refute_beyond():
    """Depois da graca um quadro sem profundidade (perdida/velha) e' fundido
    so' por cor: o chao claro vota branco. Esses votos NAO podem virar o
    'alem' de um quadro com profundidade; so' brancos de quadros com
    profundidade disputam (maioria, empate = alem)."""
    cfg = _no_gripper_cfg()
    plane_z = 0.0
    table = (-0.40, 0.40, 0.16, 0.66)
    cam = _SyntheticCamera(origin=(0.0, 0.16, 0.30), pitch_down_deg=80.0)
    img = _render_table(cam, plane_z, table, [], floor=(225, 225, 225))     # chao claro
    depth = _render_depth(cam, table, plane_z, plane_z - 0.15)
    scale = cfg.process_scale
    w, h = int(cam.W * scale), int(cam.H * scale)
    exp = ws.expected_plane_range(cam.K, None, cam.origin, cam.R, plane_z, w, h, scale)
    dl = ws.classify_depth(cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST), exp, cfg)
    uv = ws.SurfaceGridFusion(cfg).project_cells(cam.K, None, cam.origin, cam.R, plane_z, cam.W, cam.H)

    def cell(cells, x, y):
        i = int((x - cfg.grid_x_min) / cfg.resolution)
        j = int((y - cfg.grid_y_min) / cfg.resolution)
        return cells[j, i]
    with_depth = ws.segment_frame(img, cfg, holding=False, depth_label=dl)
    no_depth = ws.segment_frame(img, cfg, holding=False, depth_label=None)
    assert with_depth.depth_used and not no_depth.depth_used
    # 1 quadro com profundidade + 3 so' por cor: o chao continua FORA.
    f = ws.SurfaceGridFusion(cfg)
    f.fuse(with_depth.label, scale, uv, depth_used=True)
    for _ in range(3):
        f.fuse(no_depth.label, scale, uv, depth_used=False)
    c = f.cells()
    assert cell(c, 0.0, 0.10) == ws.CELL_OFF_TABLE and cell(c, 0.0, 0.30) == ws.CELL_TABLE_FREE
    assert int(f.votes_white[f.votes_beyond > 0].max()) == 3 and int(f.votes_white_depth[f.votes_beyond > 0].max()) == 0
    # 1 'alem' contra 2 brancos de quadros COM profundidade (garbage num quadro): maioria = livre.
    g = ws.SurfaceGridFusion(cfg)
    g.fuse(with_depth.label, scale, uv, depth_used=True)
    for _ in range(2):
        g.fuse(no_depth.label, scale, uv, depth_used=True)
    assert cell(g.cells(update_lines=False), 0.0, 0.10) == ws.CELL_TABLE_FREE
    # Empate (1 x 1) = alem.
    e = ws.SurfaceGridFusion(cfg)
    e.fuse(with_depth.label, scale, uv, depth_used=True)
    e.fuse(no_depth.label, scale, uv, depth_used=True)
    assert cell(e.cells(update_lines=False), 0.0, 0.10) == ws.CELL_OFF_TABLE


def test_depth_config_keys():
    cfg = ws.config_from_dict({"depth_enable": "false", "depth_beyond_plane_m": "0.04", "depth_open_px": 5})
    assert cfg.depth_enable is False and cfg.depth_beyond_plane_m == 0.04 and cfg.depth_open_px == 5
    d = ws.SurfaceConfig()
    assert d.depth_enable is True and d.depth_min_valid_m == 0.25
    assert d.depth_beyond_plane_m == 0.03 and d.depth_above_plane_m == 0.02


# ---------------------------------------------------------------------------
# Poligonos
# ---------------------------------------------------------------------------
def test_table_polygon_from_cells_rectangle():
    cfg = _no_gripper_cfg()
    fusion = ws.SurfaceGridFusion(cfg)
    X, Y = fusion.cell_x, fusion.cell_y
    cells = np.full(X.shape, ws.CELL_UNKNOWN, np.int8)
    cells[(np.abs(X) < 0.55) & (Y < 0.72)] = ws.CELL_OFF_TABLE
    on = (X > -0.40) & (X < 0.40) & (Y > 0.16) & (Y < 0.66)
    cells[on] = ws.CELL_TABLE_FREE
    cells[on & (np.abs(X - 0.1) < 0.02) & (np.abs(Y - 0.3) < 0.02)] = ws.CELL_OBSTACLE    # cubo: faz parte do tampo
    cells[(np.abs(X + 0.5) < 0.02) & (np.abs(Y - 0.5) < 0.02)] = ws.CELL_TABLE_FREE       # mancha solta: ignorada
    poly = ws.table_polygon_from_cells(cells, X, Y)
    assert len(poly) == 4, poly
    P = np.array(poly)
    assert abs(P[:, 0].min() + 0.40) < 0.015 and abs(P[:, 0].max() - 0.40) < 0.015
    assert abs(P[:, 1].min() - 0.16) < 0.015 and abs(P[:, 1].max() - 0.66) < 0.015
    assert ws.table_polygon_from_cells(np.full(X.shape, ws.CELL_OFF_TABLE, np.int8), X, Y) == []
    assert ws.table_polygon_from_cells(np.full(X.shape, ws.CELL_UNKNOWN, np.int8), X, Y) == []
