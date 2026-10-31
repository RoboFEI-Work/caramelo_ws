"""Mesa vista pela RealSense do punho para o docking (2026-10-27).

Pedido do operador: "chegou no pre-docking, ativa a camera, ve a mesa e alinha
no centro dela". Na pose `home` (a de navegacao) a D455 fica a ~0,48 m do
chao, x ~0,26 m, olhando para frente ~23 graus para baixo: do staging (face a
~0,63 m) ve o tampo inteiro (0,80 x 0,50 m) com folga; perto da parada final
(face a ~0,40 m) a borda da frente sai do campo de visao. Por isso a camera
manda no STAGING e o LiDAR continua dono da distancia final.

NENHUMA dimensao fixa (2026-10-27, operador: "as mesas mudam de largura,
comprimento e altura de arena para arena"): a mesa e' reconhecida pelo
FORMATO — plano horizontal acima do chao (3 a 20 cm) cujo contorno e' um
retangulo (borda da frente reta, laterais paralelas a normal, tampo
preenchendo o retangulo das bordas). Largura, profundidade e altura saem
MEDIDAS; a largura so' e' reaproveitada dentro do mesmo docking
(expected_width, preenchido pelo no' depois de uma medida com as duas
laterais) para centrar quando uma lateral sai da imagem.

Matematica PURA (numpy), sem ROS, testavel com nuvens sinteticas:
  depth_to_points  -> nuvem no frame optico (+ pixel de cada ponto)
  measure_table    -> nuvem ja' no base_footprint -> TableMeasure

Por que nao confiar na altura do TF: medido 2026-10-27 com o braco em `home`,
o chao projetado pelo TF sobe 5,45 graus em pitch (roll 0,4). Em vez de
cortar por z, o detector acha os planos HORIZONTAIS no proprio depth (RANSAC
com normal perto de +z), corrige a inclinacao pelo plano do chao (ou do
tampo, que e' paralelo) e so' entao mede bordas no plano.

Convencoes (as mesmas do LiDAR em dock_align_node.face_center_from_points):
  face_yaw = angulo da normal da face (apontando para dentro da mesa) no
             frame do robo; > 0 = girar CCW (+z) esquadra.
  c_axis   = desvio do centro da face em relacao ao ponto onde o eixo +x do
             robo fura a face (o que a reta acerta SEM esquadrar), + = esquerda.
  c_foot   = desvio em relacao ao pe' da perpendicular (o que a reta acerta
             DEPOIS de esquadrar). Identidade: c_foot = c_axis - d*tan(yaw).
"""
import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage as ndi


@dataclass
class CamCfg:
    stride: int = 4                 # 1 de cada `stride` pixels em u e v
    depth_scale: float = 0.001      # 16UC1 em mm
    depth_min: float = 0.30         # D455: abaixo disso e' lixo
    depth_max: float = 2.0
    roi_x: tuple = (0.42, 1.6)      # bico do robo (footprint 0,395) .. fundo da mesa no staging (~1,15)
    roi_abs_y: float = 1.2
    roi_z: tuple = (-0.25, 0.50)    # largo de proposito (TF com ate ~15 graus de erro)
    plane_thr: float = 0.012
    plane_max_tilt_deg: float = 15.0
    plane_min_pts: int = 500        # tampo no staging: ~5 mil pontos (stride 4)
    ransac_iters: int = 120
    floor_max_abs_z0: float = 0.05  # plano do chao: |z sob a camera| <= isto
    table_h_range: tuple = (0.03, 0.20)  # mesas @Work: 5-15 cm (+ folga de ruido)
    grid: float = 0.02
    min_cells: int = 250            # 0,10 m2 (mesa no staging ~1000 celulas; na parada ~700)
    bin: float = 0.02
    border_px: int = 8              # ponto de borda a menos disso da imagem = campo de visao, nao mesa
    min_front_bins: int = 10
    min_front_span: float = 0.25
    min_side_bins: int = 5
    edge_thr: float = 0.015
    # Largura JA' MEDIDA neste docking (o no' preenche); None = desconhecida.
    # So' serve para centrar com UMA lateral visivel e para estranhar uma
    # largura nova muito diferente (vizinha grudada / tampo cortado).
    expected_width: float = None
    width_tol: float = 0.06
    min_width: float = 0.20         # sanidade (menos que isto e' recorte, nao mesa)
    max_width: float = 2.50
    # Formato: fracao das celulas OBSERVADAS dentro do retangulo das bordas
    # que e' tampo (objetos/sombras abrem buracos), e fracao do tampo FORA
    # do retangulo (contorno nao retangular / vizinha grudada).
    rect_min_fill: float = 0.75
    rect_max_outside: float = 0.08
    # Fundo do tampo (quando visivel): fracao dos bins a +-far_tol da mediana.
    # SO' INFORMATIVO (info["retangulo"][2] e imagem de debug): no tour de
    # 27/10 objetos no tampo escondiam pedacos do fundo e reprovavam mesas
    # boas como "fundo em degrau".
    far_tol: float = 0.03
    far_min_straight: float = 0.80
    yaw_sides_max_diff_deg: float = 6.0
    yaw_offset: float = 0.0         # calibracao (rad), somada ao face_yaw
    y_offset: float = 0.0           # calibracao (m), somada aos centros
    # Normal local: vizinhos a +-normal_step amostras da grade (stride 4 ->
    # ~2 cm a 0,7 m). Ponto cuja normal passa de normal_max_tilt_deg da
    # vertical e' PAREDE/face vertical: fica fora dos planos e do tampo.
    # 50 graus: ruido da normal ~7 graus, parede 90, quina tampo/face ~45.
    normal_step: int = 2
    normal_max_tilt_deg: float = 50.0
    seed: int = 7


@dataclass
class TableMeasure:
    ok: bool = False                # yaw medido (o minimo para usar a camera)
    reason: str = ""
    face_yaw: float = None
    yaw_src: str = ""               # "frente" | "lados" | "frente+lados"
    front: float = None             # distancia ao longo do +x do robo ate a face (y=0)
    d: float = None                 # distancia da origem a reta da face (normal)
    c_axis: float = None
    c_foot: float = None
    width: float = None
    sides: str = ""                 # "ambos" | "esquerda" | "direita" | ""
    center_reason: str = ""
    table_h: float = None
    tilt_deg: float = None
    n_front: int = 0
    info: dict = field(default_factory=dict)
    # Dados para a imagem de debug (caramelo_dock_camera_debug): pixels do
    # tampo/parede, pontos das bordas no base corrigido, retas ajustadas.
    dbg: dict = field(default_factory=dict)


def depth_to_points(depth, k, cfg):
    """depth (HxW uint16/float) + K (3x3) -> (pts_opt Nx3, pix Nx2 [u, v],
    normais Nx3 no frame optico). A normal local sai dos vizinhos na grade
    (+-normal_step amostras): tampo/chao apontam para cima, PAREDE (atras ou
    nas laterais da mesa, sempre branca como o tampo — a cor nao importa,
    o depth e' infravermelho) aponta para o lado. Sem vizinhos validos = 0."""
    h, w = depth.shape[:2]
    s = max(1, int(cfg.stride))
    vs, us = np.mgrid[0:h:s, 0:w:s]
    z = depth[0:h:s, 0:w:s].astype(np.float64) * cfg.depth_scale
    m = (z > cfg.depth_min) & (z < cfg.depth_max)
    x = (us - k[0][2]) / k[0][0] * z
    y = (vs - k[1][2]) / k[1][1] * z
    grid = np.stack([x, y, z], axis=-1)
    grid[~m] = np.nan
    o = max(1, int(cfg.normal_step))

    def one_sided(axis):
        # Diferenca para o lado CONTINUO (vizinho mais perto em 3D): na quina
        # do tampo a central misturava tampo e chao e a borda serrilhava.
        fwd = np.full_like(grid, np.nan)
        bwd = np.full_like(grid, np.nan)
        if axis == 1:
            fwd[:, :-o] = grid[:, o:] - grid[:, :-o]
            bwd[:, o:] = grid[:, o:] - grid[:, :-o]
        else:
            fwd[:-o, :] = grid[o:, :] - grid[:-o, :]
            bwd[o:, :] = grid[o:, :] - grid[:-o, :]
        nf = np.linalg.norm(fwd, axis=-1)
        nb = np.linalg.norm(bwd, axis=-1)
        use_b = np.isnan(nf) | (~np.isnan(nb) & (nb < nf))
        return np.where(use_b[..., None], bwd, fwd)

    du = one_sided(1)
    dv = one_sided(0)
    nrm = np.cross(du, dv)
    nn = np.linalg.norm(nrm, axis=-1, keepdims=True)
    with np.errstate(invalid="ignore", divide="ignore"):
        nrm = nrm / nn
    nrm[~np.isfinite(nrm).all(axis=-1)] = 0.0
    return grid[m], np.stack([us[m], vs[m]], axis=1), nrm[m]


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def camera_view_ok(rot_base_opt):
    """A camera esta olhando para frente e para baixo (pose de navegacao)?
    Eixo optico (+z do frame optico) no base: x > 0,5 e descendo 5..65 graus.
    Fora disso (braco em outra pose) a medida nao vale."""
    ax = rot_base_opt[:, 2]
    pitch_down = math.degrees(math.atan2(-ax[2], math.hypot(ax[0], ax[1])))
    if ax[0] < 0.5 or not (5.0 <= pitch_down <= 65.0):
        return False, f"eixo optico ({ax[0]:+.2f}, {ax[1]:+.2f}, {ax[2]:+.2f}), " \
                      f"{pitch_down:.0f} graus para baixo"
    return True, ""


def measure_depth_frame(depth, k, rot, pos, cfg):
    """Frame de depth 16UC1 + K + TF base<-optico (rot 3x3, pos) ->
    (TableMeasure, "") ou (None, motivo). Usado pelo dock_align_node e pelo
    dock_camera_probe (mesma conta nos dois)."""
    ok, why = camera_view_ok(rot)
    if not ok:
        return None, "camera fora da pose de navegacao: " + why
    pts, pix, nrm = depth_to_points(depth, k, cfg)
    pos = np.asarray(pos, dtype=float)
    return measure_table(pts @ rot.T + pos, pix, depth.shape, cfg, cam_pos=pos,
                         nrm=nrm @ rot.T, view=useful_view(depth)), ""


def _fit_plane(pts):
    c = pts.mean(axis=0)
    w, v = np.linalg.eigh(np.cov((pts - c).T))
    n = v[:, 0]
    if n[2] < 0:
        n = -n
    return n, float(n @ c)


def _ransac_planes(pts, cfg, rng):
    """Planos quase horizontais, do maior para o menor (no maximo 3)."""
    planes = []
    rest = np.ones(len(pts), dtype=bool)
    cos_tilt = math.cos(math.radians(cfg.plane_max_tilt_deg))
    for _ in range(3):
        idx = np.flatnonzero(rest)
        if len(idx) < cfg.plane_min_pts:
            break
        q = pts[idx]
        tri = rng.integers(0, len(q), size=(cfg.ransac_iters, 3))
        a, b, c = q[tri[:, 0]], q[tri[:, 1]], q[tri[:, 2]]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n, axis=1)
        good = nn > 1e-9
        n[good] /= nn[good, None]
        n[n[:, 2] < 0] *= -1.0
        good &= n[:, 2] >= cos_tilt
        if not good.any():
            break
        n, a = n[good], a[good]
        d = np.einsum("ij,ij->i", n, a)
        # contagem em lotes: iters x N cabe folgado (120 x 30k)
        cnt = (np.abs(q @ n.T - d) < cfg.plane_thr).sum(axis=0)
        best = int(np.argmax(cnt))
        if cnt[best] < cfg.plane_min_pts:
            break
        inl = np.abs(q @ n[best] - d[best]) < cfg.plane_thr
        nr, dr = _fit_plane(q[inl])
        inl = np.abs(q @ nr - dr) < cfg.plane_thr
        nr, dr = _fit_plane(q[inl])
        if nr[2] < cos_tilt:
            rest[idx[inl]] = False
            continue
        planes.append((nr, dr, int(inl.sum())))
        rest[idx[np.abs(q @ nr - dr) < 1.5 * cfg.plane_thr]] = False
    return planes


def _rot_to_z(n):
    """Rotacao minima que leva n em +z (Rodrigues)."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s = np.linalg.norm(v)
    c = float(n @ z)
    if s < 1e-9:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))








def _line_ransac(t, s, thr, rng, iters=150):
    """s = a + b*t robusto. Devolve (a, b, mascara de inliers) ou None."""
    n = len(t)
    if n < 2:
        return None
    # Vetorizado (2026-10-27: o laco por par era ~20% do tempo do frame).
    pairs = rng.integers(0, n, size=(iters, 2))
    i, j = pairs[:, 0], pairs[:, 1]
    ok = t[i] != t[j]
    if not ok.any():
        return None
    i, j = i[ok], j[ok]
    bs = (s[j] - s[i]) / (t[j] - t[i])
    as_ = s[i] - bs * t[i]
    res = np.abs(s[None, :] - (as_[:, None] + bs[:, None] * t[None, :])) < thr
    best = res[int(np.argmax(res.sum(axis=1)))]
    if best.sum() < 2:
        return None
    b, a = np.polyfit(t[best], s[best], 1)
    inl = np.abs(s - (a + b * t)) < thr
    if inl.sum() >= 2:
        b, a = np.polyfit(t[inl], s[inl], 1)
    return float(a), float(b), inl


def useful_view(depth, min_frac=0.3):
    """Retangulo de pixels (u0, u1, v0, v1) onde o depth EXISTE: a D455 tem
    uma faixa lateral fixa sem profundidade (~50 px, probe 27/10) — um tampo
    encostado nela esta CORTADO, nao terminando. Colunas/linhas com dado em
    >= min_frac do frame; sombras finas de bordas nao mudam isso."""
    valid = depth > 0
    cols = np.flatnonzero(valid.mean(axis=0) >= min_frac)
    rows = np.flatnonzero(valid.mean(axis=1) >= min_frac)
    h, w = depth.shape[:2]
    if len(cols) == 0 or len(rows) == 0:
        return 0, w - 1, 0, h - 1
    return int(cols[0]), int(cols[-1]), int(rows[0]), int(rows[-1])


def _in_image(pix, view, border):
    u0, u1, v0, v1 = view
    return ((pix[:, 0] >= u0 + border) & (pix[:, 0] <= u1 - border)
            & (pix[:, 1] >= v0 + border) & (pix[:, 1] <= v1 - border))


def _extremes(t, s, pix_ok, bin_w, take_max):
    """Por bin de t: o ponto extremo em s (max ou min). Devolve (t, s, ok)
    do extremo de cada bin com >= 3 pontos; ok = pixel longe da borda da
    imagem (borda da imagem = campo de visao, nao borda da mesa)."""
    if len(t) == 0:
        return np.array([]), np.array([]), np.array([], dtype=bool)
    keys = np.floor(t / bin_w).astype(int)
    order = np.lexsort((-s if take_max else s, keys))
    k_sorted = keys[order]
    first = np.ones(len(order), dtype=bool)
    first[1:] = k_sorted[1:] != k_sorted[:-1]
    counts = np.diff(np.append(np.flatnonzero(first), len(order)))
    sel = order[first][counts >= 3]
    return t[sel], s[sel], pix_ok[sel]


def _rectangularity(xy, allxy, yaw, u_lo, v_lo, v_hi, grid):
    """Quao retangular e' o tampo: (preenchimento, fracao fora). Celulas no
    frame da face (u ao longo da normal, v tangente). Preenchimento = celulas
    de tampo / celulas OBSERVADAS (qualquer ponto da nuvem) dentro do
    retangulo [u_lo, fundo medido] x [v_lo, v_hi] — o que a camera nao ve
    (fora do campo, sombra) nao conta contra."""
    c, s = math.cos(yaw), math.sin(yaw)

    def cells(p):
        u = p[:, 0] * c + p[:, 1] * s
        v = -p[:, 0] * s + p[:, 1] * c
        return u, v, set(zip(np.floor(u / grid).astype(int), np.floor(v / grid).astype(int)))

    u, _v, table = cells(xy)
    _ua, _va, seen = cells(allxy)
    u_hi = float(np.percentile(u, 98))
    i0, i1 = int(math.floor(u_lo / grid)) + 1, int(math.floor(u_hi / grid)) - 1
    j0, j1 = int(math.floor(v_lo / grid)) + 1, int(math.floor(v_hi / grid)) - 1
    inside = {(i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)}
    seen_in = inside & (seen | table)
    fill = len(inside & table) / max(1, len(seen_in))
    margin = {(i, j) for i in range(i0 - 2, i1 + 3) for j in range(j0 - 2, j1 + 3)}
    outside = len(table - margin) / max(1, len(table))
    return fill, outside


def _far_edge_straightness(xy, pix_ok, yaw, cfg):
    """Fracao dos bins do FUNDO do tampo (maior u por bin de v) a +-far_tol
    da mediana; None se o fundo nao aparece (bins na borda da imagem ou no
    limite de alcance da regiao nao contam)."""
    c, s = math.cos(yaw), math.sin(yaw)
    u = xy[:, 0] * c + xy[:, 1] * s
    v = -xy[:, 0] * s + xy[:, 1] * c
    ok = pix_ok & (xy[:, 0] < cfg.roi_x[1] - 0.05)
    tv, tu, tok = _extremes(v, u, ok, cfg.bin, take_max=True)
    tu = tu[tok]
    if len(tu) < 2 * cfg.min_side_bins:
        return None
    return float(np.mean(np.abs(tu - np.median(tu)) <= cfg.far_tol))


def measure_table(pts, pix, shape, cfg, cam_pos=(0.0, 0.0, 0.0), rng=None, nrm=None,
                  view=None):
    """Nuvem no base_footprint (Nx3) + pixel de cada ponto -> TableMeasure.
    cam_pos = posicao da camera no base (do mesmo TF): o erro de inclinacao
    gira a nuvem em torno DELA, entao a correcao tambem (girar em torno da
    origem do robo deslocava a face 5 cm com 5,45 graus de erro).
    nrm = normais no base (depth_to_points girada); None = sem filtro de
    parede (so' para nuvens sem grade)."""
    rng = rng or np.random.default_rng(cfg.seed)
    out = TableMeasure()
    roi = ((pts[:, 0] >= cfg.roi_x[0]) & (pts[:, 0] <= cfg.roi_x[1])
           & (np.abs(pts[:, 1]) <= cfg.roi_abs_y)
           & (pts[:, 2] >= cfg.roi_z[0]) & (pts[:, 2] <= cfg.roi_z[1]))
    pts, pix = pts[roi], pix[roi]
    if nrm is not None:
        # Superficie horizontal (tampo, chao): |n_z| alto. Parede fora.
        horiz = np.abs(nrm[roi][:, 2]) >= math.cos(math.radians(cfg.normal_max_tilt_deg))
        out.info["parede_pts"] = int((~horiz).sum())
    else:
        horiz = np.ones(len(pts), dtype=bool)
    if horiz.sum() < cfg.plane_min_pts:
        out.reason = f"poucos pontos horizontais na regiao ({int(horiz.sum())})"
        return out

    planes = _ransac_planes(pts[horiz], cfg, rng)
    if not planes:
        out.reason = "nenhum plano horizontal"
        return out
    cp = np.asarray(cam_pos, dtype=float)
    # Altura de cada plano SOB a camera: invariante ao erro de inclinacao.
    z0 = [(d - n[0] * cp[0] - n[1] * cp[1]) / n[2] for n, d, _ in planes]
    floor = [i for i, z in enumerate(z0) if abs(z) <= cfg.floor_max_abs_z0]
    fi = max(floor, key=lambda i: planes[i][2]) if floor else None
    zf = z0[fi] if fi is not None else 0.0
    tables = [i for i, z in enumerate(z0)
              if i != fi and cfg.table_h_range[0] <= z - zf <= cfg.table_h_range[1]]
    out.info["planos"] = [(round(z, 3), planes[i][2]) for i, z in enumerate(z0)]
    if not tables:
        out.reason = "sem tampo entre %.2f e %.2f m acima do chao (planos z0=%s)" % (
            cfg.table_h_range[0], cfg.table_h_range[1],
            ", ".join(f"{z:+.3f}" for z in z0))
        return out
    ti = max(tables, key=lambda i: planes[i][2])
    out.table_h = z0[ti] - zf
    n_ref = planes[fi][0] if fi is not None else planes[ti][0]
    out.tilt_deg = math.degrees(math.acos(min(1.0, n_ref[2])))
    rot = _rot_to_z(n_ref)

    allxy = ((pts - cp) @ rot.T + cp)[:, :2]
    tn, td, _ = planes[ti]
    on_table = horiz & (np.abs(pts @ tn - td) < 1.25 * cfg.plane_thr)
    tp = (pts[on_table] - cp) @ rot.T + cp
    tpix = pix[on_table]
    out.dbg["wall_pix"] = pix[~horiz]
    out.dbg["all_xy"] = allxy[::3]
    xy = tp[:, :2]

    # Tampo = componente conexa (grade de grid m, fechada 1 celula) que
    # contem o eixo do robo; sem ela, a mais proxima do eixo.
    # Formato (robo 2026-10-27): a fatia de uma PAREDE na altura do tampo cai
    # no plano da mesa como um filete de 1-2 celulas, encostado no fundo e
    # mais largo que ela — e a frente era ajustada nele. Fechamento (tapa
    # furos de tag/depth) e depois ABERTURA (filete fino some; tampo e' area).
    # Grade densa + scipy.ndimage (2026-10-27: a versao por conjuntos era
    # ~40% do tempo do frame); borda com folga para a erosao nao "comer".
    ij = np.floor(xy / cfg.grid).astype(int)
    lo = ij.min(axis=0) - 3
    gi, gj = ij[:, 0] - lo[0], ij[:, 1] - lo[1]
    occ = np.zeros((gi.max() + 4, gj.max() + 4), dtype=bool)
    occ[gi, gj] = True
    st = np.ones((3, 3), dtype=bool)
    area = ndi.binary_erosion(ndi.binary_dilation(occ, st), st, border_value=0)
    area = ndi.binary_dilation(ndi.binary_erosion(area, st, border_value=0), st)
    labels, n_lab = ndi.label(area, structure=st)
    sizes = np.bincount(labels.ravel(), minlength=n_lab + 1)
    best = None
    for k in np.flatnonzero(sizes >= cfg.min_cells):
        if k == 0:
            continue
        cx, cy = np.nonzero(labels == k)
        ys = (cy + lo[1]) * cfg.grid
        gap = 0.0 if ys.min() <= 0.0 <= ys.max() + cfg.grid else min(abs(ys.min()), abs(ys.max()))
        key = (gap, (cx.min() + lo[0]) * cfg.grid)
        if best is None or key < best[0]:
            best = (key, k, int(sizes[k]))
    if best is None:
        out.reason = "tampo sem componente com area de mesa"
        return out
    keep = labels[gi, gj] == best[1]
    xy, tpix = xy[keep], tpix[keep]
    out.dbg["table_pix"] = tpix
    out.dbg["table_xy"] = xy
    # Extremo na borda da imagem OU no limite da regiao de busca (alcance
    # roi_x, lateral roi_abs_y) e' corte, nao borda da mesa (probe 27/10:
    # mesa a 1,3 m cortada em 1,6 m virou "lateral", largura 0,43).
    if view is None:
        view = (0, shape[1] - 1, 0, shape[0] - 1)
    pix_ok = (_in_image(tpix, view, cfg.border_px)
              & (xy[:, 0] < cfg.roi_x[1] - 0.05) & (np.abs(xy[:, 1]) < cfg.roi_abs_y - 0.05))
    out.info["celulas"] = best[2]

    # ---- Borda da frente: menor x por bin de y ----
    yaw_front = None
    fy, fx, fok = _extremes(xy[:, 1], xy[:, 0], pix_ok, cfg.bin, take_max=False)
    fy, fx = fy[fok], fx[fok]
    # A frente e' a borda MAIS PROXIMA com comprimento (nao a com mais
    # pontos): ate 3 retas por RANSAC, tirando os inliers de cada uma.
    fit = None
    left = np.ones(len(fy), dtype=bool)
    for _ in range(3):
        if left.sum() < cfg.min_front_bins:
            break
        f = _line_ransac(fy[left], fx[left], cfg.edge_thr, rng)
        if f is None:
            break
        inl = np.zeros(len(fy), dtype=bool)
        inl[np.flatnonzero(left)[f[2]]] = True
        left &= ~inl
        span = float(fy[inl].max() - fy[inl].min())
        if inl.sum() < cfg.min_front_bins or span < cfg.min_front_span:
            continue
        if fit is None or np.median(fx[inl]) < np.median(fx[fit[2]]):
            fit = (f[0], f[1], inl)
    if fit is not None:
        a, b, inl = fit
        span = float(fy[inl].max() - fy[inl].min()) if inl.any() else 0.0
        out.n_front = int(inl.sum())
        out.dbg["front_xy"] = np.stack([fx[inl], fy[inl]], axis=1)
        if out.n_front >= cfg.min_front_bins and span >= cfg.min_front_span:
            yaw_front = -math.atan(b)
            out.d = a / math.sqrt(1.0 + b * b)
        else:
            out.info["frente"] = f"{out.n_front} bins, {span:.2f} m"
    else:
        out.info["frente"] = f"{len(fy)} bins validos"

    # ---- Laterais: extremos em v por bin de u, no frame alinhado ----
    def sides_at(yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        u = xy[:, 0] * c + xy[:, 1] * s
        v = -xy[:, 0] * s + xy[:, 1] * c
        u0 = out.d if out.d is not None else float(np.percentile(u, 2))
        # Fundo do tampo MEDIDO (ou o fim do que a camera ve), nao fixo.
        u_far = float(np.percentile(u, 98))
        band = (u >= u0 + 0.04) & (u <= u_far - 0.04)
        res = {}
        for name, take_max in (("esquerda", True), ("direita", False)):
            tu, sv, ok = _extremes(u[band], v[band], pix_ok[band], cfg.bin, take_max)
            tu, sv = tu[ok], sv[ok]
            if len(tu) < cfg.min_side_bins:
                continue
            f = _line_ransac(tu, sv, cfg.edge_thr, rng)
            if f is None or f[2].sum() < cfg.min_side_bins:
                continue
            res[name] = (float(np.median(sv[f[2]])), math.atan(f[1]), int(f[2].sum()),
                         np.stack([tu[f[2]], sv[f[2]]], axis=1))
        return res, u0

    yaw0 = yaw_front if yaw_front is not None else 0.0
    sides, _u0 = sides_at(yaw0)
    yaw_sides = None
    if sides:
        yaw_sides = yaw0 + float(np.mean([v[1] for v in sides.values()]))
        if yaw_front is None:
            sides, _u0 = sides_at(yaw_sides)
            if sides:
                yaw_sides = yaw_sides + float(np.mean([v[1] for v in sides.values()]))

    if yaw_front is not None and yaw_sides is not None:
        diff = math.degrees(abs(yaw_front - yaw_sides))
        out.info["yaw_frente_lados"] = (round(math.degrees(yaw_front), 1),
                                        round(math.degrees(yaw_sides), 1))
        if diff > cfg.yaw_sides_max_diff_deg:
            # Tour 27/10 (WS1/WS3/WS6): frente batia com o LiDAR e as laterais
            # davam o dobro. A frente e' a borda longa e inteira: vale ela.
            yaw, out.yaw_src = yaw_front, "frente (laterais discordam)"
        else:
            yaw, out.yaw_src = (2.0 * yaw_front + yaw_sides) / 3.0, "frente+lados"
    elif yaw_front is not None:
        yaw, out.yaw_src = yaw_front, "frente"
    elif yaw_sides is not None:
        yaw, out.yaw_src = yaw_sides, "lados"
    else:
        out.reason = "nem a borda da frente nem as laterais (%s)" % out.info.get("frente", "")
        return out

    # Remede as laterais no yaw final (bordas paralelas ao eixo u).
    sides, u0 = sides_at(yaw)
    d = out.d if out.d is not None else u0
    v_c = None
    if "esquerda" in sides and "direita" in sides:
        out.width = sides["esquerda"][0] - sides["direita"][0]
        out.sides = "ambos"
        vl, vr = sides["esquerda"][0], sides["direita"][0]
        fill, outside = _rectangularity(xy, allxy, yaw, d, vr, vl, cfg.grid)
        straight = _far_edge_straightness(xy, pix_ok, yaw, cfg)
        out.info["retangulo"] = (round(fill, 2), round(outside, 2),
                                 None if straight is None else round(straight, 2))
        if not (cfg.min_width <= out.width <= cfg.max_width):
            out.center_reason = f"largura {out.width:.2f} m implausivel"
        elif fill < cfg.rect_min_fill or outside > cfg.rect_max_outside:
            out.center_reason = (f"formato nao retangular (preenche {fill:.0%}, "
                                 f"{outside:.0%} do tampo fora)")
        elif (cfg.expected_width is not None
              and abs(out.width - cfg.expected_width) > cfg.width_tol):
            out.center_reason = (f"largura {out.width:.2f} m diferente da medida antes "
                                 f"({cfg.expected_width:.2f}) — vizinha ou tampo cortado?")
        else:
            v_c = 0.5 * (vl + vr)
    elif sides and cfg.expected_width is None:
        out.sides = next(iter(sides))
        out.center_reason = f"so' a lateral {out.sides} visivel e largura ainda nao medida"
    elif "esquerda" in sides:
        out.sides = "esquerda"
        v_c = sides["esquerda"][0] - 0.5 * cfg.expected_width
    elif "direita" in sides:
        out.sides = "direita"
        v_c = sides["direita"][0] + 0.5 * cfg.expected_width
    else:
        out.center_reason = "nenhuma lateral visivel"

    yaw += cfg.yaw_offset
    out.face_yaw = yaw
    out.front = d / math.cos(yaw)
    if v_c is not None:
        v_c += cfg.y_offset
        out.c_foot = v_c
        out.c_axis = v_c + d * math.tan(yaw)
    out.ok = True
    out.dbg.update(yaw=yaw - cfg.yaw_offset, d=d, u_far=float(np.percentile(
        xy[:, 0] * math.cos(yaw) + xy[:, 1] * math.sin(yaw), 98)),
        sides={k: v[3] for k, v in sides.items()})
    if "esquerda" in sides:
        out.dbg["vl"] = sides["esquerda"][0]
    if "direita" in sides:
        out.dbg["vr"] = sides["direita"][0]
    if v_c is not None:
        out.dbg["v_c"] = v_c - cfg.y_offset
    return out
