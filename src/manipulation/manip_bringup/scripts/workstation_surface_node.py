#!/usr/bin/env python3
"""Mapa top-down da MESA (workstation) com a RealSense do punho (2026-10-19).

Por que existe: o place em mesa comum escolhia entre 3 poses fixas (MesaX,
MesaX.1, MesaX.2) e so' conhecia obstaculos com AprilTag. O rulebook
(RoboCup@Work 4.4) pede: objetos a >= 2 cm uns dos outros e da borda da
service area, >= 10 cm de uma parede, dentro de 20 cm a partir da frente da
mesa. Para cumprir isso em QUALQUER mesa, o robo precisa VER o tampo: onde
ele acaba (bordas), o que ja esta em cima (com ou sem tag) e o que esta livre.

Como: a mesa e' branca e plana. A profundidade do D455 nao alcanca daqui
(0,2-0,4 m, zona cega), entao cada pixel e' projetado por RAY-CAST no plano do
TAMPO (cota publicada pelo place em /manip/table_plane_z, frame
manip_base_link) — a mesma ideia do container_detector. O resultado e' uma
GRADE regular no plano da mesa (nav_msgs/OccupancyGrid, 1 cm por celula):
    -1  desconhecido (fora da imagem, atras da garra, raio rasante)
     0  tampo branco LIVRE
    70  FORA da mesa (chao, parede, nada branco)
   100  OBJETO sobre a mesa (mancha nao branca dentro do tampo)
Varias vistas (pegar_obj, tag_direita, tag_esquerda...) sao FUNDIDAS na mesma
grade por votos — a base nao se move durante a observacao. Objetos sem tag,
containers, sombras escuras: tudo vira obstaculo. Como o topo de um cubo de
41 mm e' projetado no plano do tampo, a mancha fica um pouco MAIOR e mais
LONGE da camera do que o pe do objeto — conservador para a folga de 2 cm.

Como se decide o que e' BORDA da mesa e o que e' OBJETO (2026-10-19, apos as
fotos reais): por cor so' da para separar BRANCO (tampo) de NAO-BRANCO; um
container no canto da imagem e o chao sao ambos nao-brancos. Entao a
fronteira branco/nao-branco e' levada ao plano da mesa e procuram-se RETAS
LONGAS (RANSAC, >= 20 cm, cobertura >= 50 %): a mesa e' um retangulo, suas
bordas sao retas longas; cubos (4 cm) e containers (17 cm) nao sao. So' o
que fica ALEM de uma reta dessas (do lado em que predomina o nao-branco)
vira "fora da mesa"; todo o resto nao-branco e' OBJETO (proibido do mesmo
jeito, mas nunca vira "borda"). Sem reta vista, a frente vem do docking
(fallback no place). Cortes pela borda da IMAGEM nao viram borda (fora da
imagem = desconhecido). Dedos/cubo na garra apagados pelos poligonos do
container_detector (camera e garra solidarias).

PROFUNDIDADE: mesa x chao (2026-10-21, apos o teste no robo). So' por cor o
CHAO claro projetado no plano do tampo virava "tampo livre" e, sem borda
detectada, o place aceitava pontos a menos de 2 cm da borda real. O D455
publica a profundidade ALINHADA a cor (aligned_depth_to_color, 16UC1 em mm,
0 = invalido). O tampo a 0,3-0,4 m costuma vir INVALIDO (zona cega), mas o
chao e tudo o que esta alem da mesa vem VALIDO e MAIS LONGE que o plano do
tampo. Entao, por pixel, compara-se a profundidade medida com a profundidade
ESPERADA do raio ate o plano (expected_plane_range):
    medida > esperada + depth_beyond_plane_m  -> LBL_BEYOND (fora da mesa),
                                                 seja qual for a cor;
    medida < esperada - depth_above_plane_m   -> LBL_NONWHITE (algo acima do
                                                 tampo), mesmo que branco;
    invalida / ausente / sem depth no ar      -> decisao por cor, como antes.
Logo apos a ativacao o primeiro quadro de cor costuma chegar ANTES da
primeira profundidade: fundir so' por cor nesse instante pintaria o chao de
"tampo livre" (exatamente o bug). Por isso, com depth_enable, o no ESPERA a
profundidade ate depth_grace_s (1 s) antes de cair para "so' cor".
Na fusao LBL_BEYOND vota em votes_beyond; celula com votes_beyond >=
votes_white_depth (brancos SO' de quadros que tinham profundidade: um quadro
fundido so' por cor nao pode refutar "alem") vira CELL_OFF_TABLE DIRETO (sem
depender das retas) e entra como "fora" na fronteira usada pela RANSAC, que
assim acha a borda REAL da mesa.
Sem profundidade o comportamento e' identico ao anterior.

PROJECAO celula -> pixel (project_cells): alem do cone de
MAX_RAY_HALF_ANGLE_DEG (55 graus) do eixo optico a celula NAO esta na imagem,
mesmo que cv2.projectPoints devolva um pixel "dentro": com o D real da D455
(k3 < 0) o polinomio radial deixa de crescer a ~59 graus e DOBRA para dentro
da imagem pontos fora do campo de visao, que herdariam o rotulo de pixels do
centro (1000+ celulas fantasmas e retas laterais falsas em 21/10).

POLIGONOS: alem da grade, publica o contorno do tampo visto (PolygonStamped
em .../table_polygon, maior componente livre|objeto simplificado) e as retas
de borda (MarkerArray em .../edge_lines: LINE_LIST ns "edge" + ARROW ns
"edge_normal" apontando para FORA) — so' quando ha assinante.

Ativacao: so' enquanto /manip/surface_observe (Bool, latched) for true; a
subida de false->true ZERA a fusao. always_on=true para calibrar.

Calibracao offline (sem robo):
  workstation_surface_node.py --image foto.png [--config x.yaml] [--out overlay.png]
                              [--probe u,v] [--gripper-open]
                              [--camera x,y,z,pitch --plane-z z --depth depth.png
                               --intrinsics fx,fy,cx,cy]   # depth.png: 16 bits em mm
Quem consome a grade: mtc_place_action_node (surface_place_planner).
"""
import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# Codigos das celulas — iguais a surface_place_planner.hpp (SurfaceCell).
CELL_UNKNOWN = -1
CELL_TABLE_FREE = 0
CELL_OFF_TABLE = 70
CELL_OBSTACLE = 100

# Rotulos por pixel/celula de UMA vista (antes da classificacao por retas).
LBL_UNKNOWN = 0
LBL_WHITE = 1       # tampo branco
LBL_NONWHITE = 2    # objeto OU fora da mesa — decidido depois, pelas retas
LBL_BEYOND = 3      # profundidade ALEM do plano do tampo: fora da mesa (chao), sem depender das retas

# Namespaces dos marcadores de borda (edge_lines): lidos pelo vision_debug_node.
EDGE_MARKER_NS = "edge"               # LINE_LIST com os 2 pontos da reta
EDGE_NORMAL_MARKER_NS = "edge_normal"  # ARROW do meio da reta para FORA da mesa

# Cone da projecao celula -> pixel (project_cells), em graus do eixo optico.
# O FOV da D455 cor e' 90 x 65 graus (diagonal ~50; em 640x480 com fx=385 a
# diagonal da imagem fica a 46 graus): 55 nao perde nada da imagem e fica
# ABAIXO do raio em que a distorcao da D455 dobra (~59 graus). Mesmo valor
# em vision_debug_node.py (MAX_HALF_ANGLE_DEG).
MAX_RAY_HALF_ANGLE_DEG = 55.0


# Dedos vermelhos da garra ABERTA (RealSense 640x480, 25/08) — copia dos
# defaults do container_detector_node.py; o yaml manda.
DEFAULT_GRIPPER_POLYS: List[List[Tuple[float, float]]] = [
    [(0.0, 0.0), (0.138, 0.0), (0.157, 0.094), (0.155, 0.304), (0.0, 0.225)],
    [(0.841, 0.0), (1.0, 0.0), (1.0, 0.140), (0.867, 0.277), (0.838, 0.279),
     (0.795, 0.113), (0.789, 0.023)],
]
DEFAULT_GRIPPER_POLYS_HOLDING: List[List[Tuple[float, float]]] = [
    [(0.791, 0.023), (0.791, 0.077), (0.773, 0.133), (0.722, 0.196), (0.616, 0.260),
     (0.594, 0.250), (0.528, 0.037), (0.534, 0.000), (0.762, 0.000)],
    [(0.217, 0.219), (0.206, 0.235), (0.184, 0.235), (0.048, 0.150), (0.005, 0.113),
     (0.000, 0.002), (0.209, 0.000)],
    [(0.150, 0.000), (0.660, 0.000), (0.660, 0.250), (0.600, 0.360), (0.570, 0.390),
     (0.300, 0.385), (0.260, 0.355), (0.150, 0.240)],
]


@dataclass
class SurfaceConfig:
    # --- grade no plano da mesa (manip_base_link; +X direita do robo, +Y frente) ---
    grid_x_min: float = -0.60
    grid_x_max: float = 0.60
    grid_y_min: float = 0.05
    grid_y_max: float = 0.75
    resolution: float = 0.01
    # --- imagem ---
    process_scale: float = 0.5
    # Branco = pouca saturacao e brilho alto (OpenCV: S/V 0-255).
    white_s_max: int = 70
    white_v_min: int = 100
    # Manchas brancas menores que isto (fracao da imagem processada) NAO sao
    # tampo (etiqueta branca num cubo, reflexo): viram nao-branco.
    table_component_min_frac: float = 0.02
    # Fracao minima da imagem branca para dizer "vi a mesa" nesta vista.
    table_min_frac: float = 0.05
    # Morfologia (px, na imagem ja escalada).
    white_open_px: int = 3
    obstacle_open_px: int = 3      # tira pontinhos/sombra fina do nao-branco
    obstacle_close_px: int = 5     # fecha o desenho da tag dentro do cubo
    # Buraco branco FECHADO por nao-branco menor que isto (fracao da imagem)
    # e' parte do objeto (etiqueta/tag): vira nao-branco.
    hole_max_frac: float = 0.03
    obstacle_dilate_px: int = 2    # margem de seguranca em pixels
    # --- garra (fracao da imagem; mesmos nomes do container_detector.yaml) ---
    exclusion_rects: List[Tuple[float, float, float, float]] = field(
        default_factory=lambda: [(0.30, 0.00, 0.62, 0.42)])
    exclusion_polys: List[List[Tuple[float, float]]] = field(
        default_factory=lambda: [list(p) for p in DEFAULT_GRIPPER_POLYS])
    exclusion_polys_holding: List[List[Tuple[float, float]]] = field(
        default_factory=lambda: [list(p) for p in DEFAULT_GRIPPER_POLYS_HOLDING])
    exclusion_dilate_px: int = 4
    gripper_joint: str = "manip_joint6"
    gripper_holding_above: float = -0.12
    # --- projecao ---
    max_ray_length_m: float = 1.2
    # Raio rasante (angulo do raio com o plano) abaixo disto = desconhecido:
    # a projecao fica imprecisa e sensivel a altura.
    min_ray_elevation_deg: float = 18.0
    # Pixels a menos que isto da borda da imagem nao contam (undistort/borda).
    image_border_px: int = 2
    # --- fusao (maioria de votos por celula; empate = nao-branco, conservador) ---
    free_min_votes: int = 1
    # --- retas de borda (no plano da mesa, metros) ---
    edge_inlier_m: float = 0.012       # ponto a menos que isto da reta = inlier
    edge_min_length_m: float = 0.20    # extensao minima da reta (cubo 0,04; container 0,17; mesa >= 0,5)
    edge_min_coverage: float = 0.5     # fracao dos trechos de 2 cm da extensao com inlier
    edge_max_lines: int = 4
    edge_ransac_iters: int = 300
    edge_outside_min_frac: float = 0.7 # do lado "de fora" >= 70% nao-branco; do lado "de dentro" <= 30%
    edge_band_m: float = 0.06          # faixa de cada lado da reta usada nessa contagem
    # Alem de uma borda REAL nao ha mais tampo: se, do lado de fora, ate
    # edge_outside_check_m da reta, mais que edge_outside_max_white_frac das
    # celulas vistas forem brancas, a "borda" e' na verdade a face de um
    # objeto (dois containers alinhados fizeram uma reta de 39 cm nas fotos
    # de 25/08) — rejeitada; o nao-branco fica como OBJETO.
    edge_outside_check_m: float = 0.25
    edge_outside_max_white_frac: float = 0.10
    # --- profundidade (D455 alinhada a cor; ver docstring "PROFUNDIDADE") ---
    depth_enable: bool = True
    depth_min_valid_m: float = 0.25    # abaixo disto o D455 nao mede (zona cega): invalido
    # 3 cm: a mesa de 5 cm do rulebook tem o chao so 5 cm abaixo do tampo e com a
    # camera quase vertical a diferenca de alcance e ~5 cm; 6 cm deixava a regra
    # inerte. Ruido do D455 a 0,5-1 m ~0,5-1 cm.
    depth_beyond_plane_m: float = 0.03 # medida > esperada + isto = alem do plano (fora da mesa)
    depth_above_plane_m: float = 0.02  # medida < esperada - isto = acima do tampo (objeto)
    depth_open_px: int = 3             # abertura (px escalados) que tira "flying pixels" das bordas
    # Se mais que isto dos pixels conhecidos viessem "acima do tampo", o plano
    # e' que esta errado (nao a mesa inteira coberta): a regra "acima" e'
    # ignorada nesse quadro e o status avisa. "Alem" nao tem esse teto.
    depth_above_max_frac: float = 0.5


@dataclass
class FrameResult:
    label: np.ndarray            # HxW (imagem escalada) com LBL_*
    table_found: bool            # branco >= table_min_frac da imagem
    table_frac: float
    n_nonwhite_blobs: int
    white_stats: Dict[str, Tuple[int, int, int]]
    depth_used: bool = False     # houve rotulo de profundidade neste quadro
    depth_beyond_frac: float = 0.0   # fracao dos pixels conhecidos marcados "alem do plano"
    depth_above_frac: float = 0.0    # idem "acima do tampo" (antes do teto depth_above_max_frac)
    depth_above_rejected: bool = False


@dataclass
class EdgeLine:
    """Reta a*x + b*y + c = 0 (normalizada) no plano da mesa; o lado POSITIVO
    e' o de FORA da mesa."""
    a: float
    b: float
    c: float
    n_inliers: int
    length_m: float
    cx: float
    cy: float

    def signed(self, x, y):
        return self.a * x + self.b * y + self.c

    def describe(self) -> str:
        ang = math.degrees(math.atan2(-self.a, self.b))   # direcao da reta vs +X
        kind = "frente/fundo" if abs(abs(ang) % 180.0 - 0.0) < 30 or abs(abs(ang) % 180.0 - 180.0) < 30 else \
            ("lateral" if abs(abs(ang) % 180.0 - 90.0) < 30 else "obliqua")
        return f"{kind} por ({self.cx:+.2f},{self.cy:.2f}) {self.length_m*100:.0f}cm n={self.n_inliers}"


def parse_polys(values: Sequence[str]) -> List[List[Tuple[float, float]]]:
    polys: List[List[Tuple[float, float]]] = []
    for item in values:
        txt = str(item).strip()
        if not txt:
            continue
        pts: List[Tuple[float, float]] = []
        for tok in txt.replace(";", " ").split():
            xy = tok.split(",")
            if len(xy) != 2:
                raise ValueError(f"poligono: vertice invalido '{tok}' em '{txt}'")
            x, y = float(xy[0]), float(xy[1])
            if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
                raise ValueError(f"poligono: vertice fora de [0,1]: '{tok}'")
            pts.append((x, y))
        if len(pts) < 3:
            raise ValueError(f"poligono com menos de 3 vertices: '{txt}'")
        polys.append(pts)
    return polys


def parse_rects(values: Sequence[float]) -> List[Tuple[float, float, float, float]]:
    vals = [float(v) for v in values]
    if len(vals) % 4 != 0:
        raise ValueError(f"exclusion_rects_norm precisa de grupos de 4: {vals}")
    return [tuple(vals[i:i + 4]) for i in range(0, len(vals), 4)]  # type: ignore[misc]


def exclusion_mask(cfg: SurfaceConfig, w: int, h: int, holding: bool) -> np.ndarray:
    """255 onde a garra/cubo carregado aparecem (w x h = imagem escalada)."""
    m = np.zeros((h, w), dtype=np.uint8)
    polys = list(cfg.exclusion_polys)
    if holding:
        polys += list(cfg.exclusion_polys_holding)
        for (x0, y0, x1, y1) in cfg.exclusion_rects:
            polys.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
    for poly in polys:
        pts = np.round(np.array([[x * w, y * h] for (x, y) in poly])).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(m, [pts], 255)   # um por vez (sobrepostos abrem buraco)
    k = int(cfg.exclusion_dilate_px)
    if k > 0:
        m = cv2.dilate(m, np.ones((2 * k + 1, 2 * k + 1), np.uint8))
    return m


def segment_frame(bgr_full: np.ndarray, cfg: SurfaceConfig, holding: bool = True,
                  depth_label: Optional[np.ndarray] = None) -> FrameResult:
    """Imagem -> rotulos por pixel (escalados): branco (tampo) / nao-branco /
    alem do plano / desconhecido.

    depth_label: opcional, rotulos de classify_depth (LBL_BEYOND / LBL_NONWHITE /
    LBL_UNKNOWN) na resolucao processada; onde ha decisao por profundidade ela
    manda sobre a cor (alem do plano nunca e' tampo; acima do tampo nunca e'
    livre). None = so' cor (comportamento original)."""
    scale = float(cfg.process_scale)
    small = cv2.resize(bgr_full, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) \
        if scale != 1.0 else bgr_full
    h, w = small.shape[:2]
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    excl = exclusion_mask(cfg, w, h, holding)
    b = int(cfg.image_border_px)
    known = np.full((h, w), 255, dtype=np.uint8)
    known[excl > 0] = 0
    if b > 0:
        known[:b, :] = 0
        known[-b:, :] = 0
        known[:, :b] = 0
        known[:, -b:] = 0

    white = cv2.inRange(hsv, (0, 0, int(cfg.white_v_min)), (179, int(cfg.white_s_max), 255))
    white[known == 0] = 0
    dl = None
    depth_beyond_frac = depth_above_frac = 0.0
    depth_above_rejected = False
    if depth_label is not None and bool(cfg.depth_enable):
        dl = np.asarray(depth_label, dtype=np.uint8)
        if dl.shape != (h, w):
            dl = cv2.resize(dl, (w, h), interpolation=cv2.INTER_NEAREST)
        dl = np.where(known > 0, dl, np.uint8(LBL_UNKNOWN)).astype(np.uint8)
        n_known = max(1, int(np.count_nonzero(known)))
        depth_beyond_frac = float(np.count_nonzero(dl == LBL_BEYOND)) / n_known
        depth_above_frac = float(np.count_nonzero(dl == LBL_NONWHITE)) / n_known
        if depth_above_frac > float(cfg.depth_above_max_frac):
            dl[dl == LBL_NONWHITE] = LBL_UNKNOWN     # plano errado, nao mesa coberta
            depth_above_rejected = True
        white[dl != LBL_UNKNOWN] = 0                 # alem ou acima do plano: nao e' tampo
    k_open = int(cfg.white_open_px)
    if k_open > 1:
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((k_open, k_open), np.uint8))
    sel = hsv[white > 0]
    wstats: Dict[str, Tuple[int, int, int]] = {}
    if sel.size:
        for i, name in enumerate(("H", "S", "V")):
            ch = sel[:, i]
            wstats[name] = (int(np.percentile(ch, 5)), int(np.percentile(ch, 50)), int(np.percentile(ch, 95)))
    # Manchas brancas pequenas (etiqueta, reflexo) nao sao tampo.
    n_lbl, lbls, stats, _ = cv2.connectedComponentsWithStats(white, connectivity=8)
    min_area = cfg.table_component_min_frac * w * h
    for k in range(1, n_lbl):
        if stats[k, cv2.CC_STAT_AREA] < min_area:
            white[lbls == k] = 0
    table_frac = float(np.count_nonzero(white)) / float(w * h)

    nonwhite = cv2.bitwise_and(known, cv2.bitwise_not(white))
    ko = int(cfg.obstacle_open_px)
    if ko > 1:
        nonwhite = cv2.morphologyEx(nonwhite, cv2.MORPH_OPEN, np.ones((ko, ko), np.uint8))
    kc = int(cfg.obstacle_close_px)
    if kc > 1:
        closed = cv2.morphologyEx(nonwhite, cv2.MORPH_CLOSE, np.ones((kc, kc), np.uint8))
        nonwhite = cv2.bitwise_and(closed, known)
    # Buracos brancos FECHADOS por nao-branco (tag/etiqueta no cubo): componentes
    # de "nao nao-branco" que nao tocam a borda da imagem e sao pequenos.
    rest = cv2.bitwise_not(nonwhite)
    n_r, r_lbls, r_stats, _ = cv2.connectedComponentsWithStats(rest, connectivity=4)
    max_hole = cfg.hole_max_frac * w * h
    for k in range(1, n_r):
        x0, y0, bw, bh, area = r_stats[k]
        touches = x0 <= 0 or y0 <= 0 or x0 + bw >= w or y0 + bh >= h
        if not touches and area <= max_hole:
            nonwhite[r_lbls == k] = 255
    n_blobs = max(0, cv2.connectedComponentsWithStats(nonwhite, connectivity=8)[0] - 1)
    kd = int(cfg.obstacle_dilate_px)
    if kd > 0:
        nonwhite = cv2.dilate(nonwhite, np.ones((2 * kd + 1, 2 * kd + 1), np.uint8))
        nonwhite = cv2.bitwise_and(nonwhite, known)
    label = np.full((h, w), LBL_UNKNOWN, dtype=np.uint8)
    label[(white > 0) & (known > 0)] = LBL_WHITE
    label[nonwhite > 0] = LBL_NONWHITE
    if dl is not None:
        label[dl == LBL_BEYOND] = LBL_BEYOND       # por cima da dilatacao dos objetos
    return FrameResult(label, table_frac >= cfg.table_min_frac, table_frac, n_blobs, wstats,
                       depth_used=dl is not None, depth_beyond_frac=depth_beyond_frac,
                       depth_above_frac=depth_above_frac, depth_above_rejected=depth_above_rejected)


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def distortion_fold_tan(D: Optional[np.ndarray], r_max: float = 4.0) -> float:
    """Raio normalizado (tan do angulo ao eixo optico) a partir do qual o modelo
    radial plumb_bob / rational_polynomial de D (k1 k2 p1 p2 [k3 [k4 k5 k6]])
    deixa de ser CRESCENTE: alem dele cv2.projectPoints devolve pixels que
    voltam para dentro da imagem (dobra). inf se nao ha dobra ate r_max, se D
    e' nulo ou se o tamanho de D nao e' 4/5/8. A parte tangencial e' ignorada."""
    Dv = np.asarray(D, dtype=np.float64).ravel() if (D is not None and len(D)) else None
    if Dv is None or not np.any(Dv) or Dv.size not in (4, 5, 8):
        return float("inf")
    k = np.zeros(8)
    k[:Dv.size] = Dv
    k1, k2, _p1, _p2, k3, k4, k5, k6 = k
    r = np.linspace(0.0, float(r_max), 4001)
    r2 = r * r
    num = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    den = 1.0 + r2 * (k4 + r2 * (k5 + r2 * k6))
    with np.errstate(divide="ignore", invalid="ignore"):
        f = r * num / den
    bad = ~np.isfinite(f[1:]) | (np.diff(f) <= 0.0)
    if not np.any(bad):
        return float("inf")
    return float(r[int(np.argmax(bad))])


def depth_wait_pending(depth_why: Optional[str], since_active_s: Optional[float], grace_s: float) -> bool:
    """True enquanto vale ESPERAR a profundidade em vez de fundir so' por cor:
    ela esta ligada mas ainda nao chegou ("sem") ou esta desencontrada da cor
    ("velha(...)"), e a ativacao foi ha menos de grace_s. Vencida a graca (ou
    depth_enable=false, encoding errado...), o no funde so' por cor."""
    if depth_why is None or since_active_s is None or float(grace_s) <= 0.0:
        return False
    if not (depth_why == "sem" or depth_why.startswith("velha")):
        return False
    return float(since_active_s) < float(grace_s)


def expected_plane_range(K: np.ndarray, D: Optional[np.ndarray], cam_origin: np.ndarray, cam_rot: np.ndarray,
                         plane_z: float, w: int, h: int, scale: float = 1.0) -> np.ndarray:
    """Profundidade ESPERADA de cada pixel da imagem processada (h x w = imagem
    cheia x scale) se o raio fosse ate o plano do tampo z = plane_z.

    Devolve (h, w) float32 com o z no frame OPTICO do ponto onde o raio cruza o
    plano — a mesma grandeza que o D455 publica (profundidade = z, nao a
    distancia euclidiana). NaN onde o raio nao desce ate o plano (olha para
    cima / paralelo) ou o plano fica atras da camera. K/D = camera_info da
    cor (a profundidade alinhada tem a mesma geometria); cam_rot: colunas =
    eixos da camera no frame base (base <- camera_optical)."""
    w, h = int(w), int(h)
    s = float(scale)
    # Centro do pixel escalado i em coordenadas da imagem cheia: (i + 0.5) / s - 0.5.
    us = (np.arange(w, dtype=np.float64) + 0.5) / s - 0.5
    vs = (np.arange(h, dtype=np.float64) + 0.5) / s - 0.5
    U, V = np.meshgrid(us, vs)
    Kf = np.asarray(K, dtype=np.float64)
    Dv = np.asarray(D, dtype=np.float64).ravel() if (D is not None and len(D)) else None
    if Dv is not None and np.any(Dv):
        pts = np.stack([U.ravel(), V.ravel()], axis=1).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(pts, Kf, Dv).reshape(-1, 2)
        x, y = norm[:, 0], norm[:, 1]
    else:
        x = (U.ravel() - Kf[0, 2]) / Kf[0, 0]
        y = (V.ravel() - Kf[1, 2]) / Kf[1, 1]
    # Raio no frame optico com z = 1: o t da intersecao E' a profundidade.
    R = np.asarray(cam_rot, dtype=np.float64)
    dz_world = R[2, 0] * x + R[2, 1] * y + R[2, 2]          # componente z (base) do raio
    dh = float(plane_z) - float(np.asarray(cam_origin, dtype=np.float64)[2])
    with np.errstate(divide="ignore", invalid="ignore"):
        t = dh / dz_world
    t = np.where((dz_world < -1e-9) & (t > 0.0), t, np.nan)
    return t.reshape(h, w).astype(np.float32)


def classify_depth(depth_m: np.ndarray, expected_m: np.ndarray, cfg: SurfaceConfig) -> np.ndarray:
    """Rotulo por pixel SO' pela profundidade (h, w) uint8:
        LBL_BEYOND   medida valida e > esperada + depth_beyond_plane_m (chao, alem da borda)
        LBL_NONWHITE medida valida e < esperada - depth_above_plane_m (algo sobre o tampo)
        LBL_UNKNOWN  sem decisao (0/NaN, perto demais, raio sem plano, ou entre as duas folgas)
    depth_m em metros (0 ou NaN = invalido); expected_m de expected_plane_range.
    Uma abertura de depth_open_px tira os "flying pixels" das bordas dos objetos."""
    depth = np.asarray(depth_m, dtype=np.float32)
    exp = np.asarray(expected_m, dtype=np.float32)
    if depth.shape != exp.shape:
        raise ValueError(f"classify_depth: profundidade {depth.shape} x esperada {exp.shape}")
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(depth) & (depth >= float(cfg.depth_min_valid_m)) & np.isfinite(exp)
        beyond = valid & (depth > exp + float(cfg.depth_beyond_plane_m))
        above = valid & (depth < exp - float(cfg.depth_above_plane_m))
    k = int(cfg.depth_open_px)
    if k > 1:
        kern = np.ones((k, k), np.uint8)
        beyond = cv2.morphologyEx(beyond.astype(np.uint8), cv2.MORPH_OPEN, kern) > 0
        above = cv2.morphologyEx(above.astype(np.uint8), cv2.MORPH_OPEN, kern) > 0
    out = np.full(depth.shape, LBL_UNKNOWN, dtype=np.uint8)
    out[above] = LBL_NONWHITE
    out[beyond] = LBL_BEYOND
    return out


def depth_image_to_m(arr: np.ndarray, encoding: str = "16UC1", depth_scale: float = 0.001) -> Optional[np.ndarray]:
    """Imagem de profundidade crua -> metros float32 (0 = invalido, mantido).
    16UC1 (D455: mm) x depth_scale; 32FC1 ja em metros. Outro encoding: None."""
    enc = str(encoding or "").upper()
    if enc in ("16UC1", "MONO16") or (enc == "" and arr.dtype == np.uint16):
        return arr.astype(np.float32) * float(depth_scale)
    if enc == "32FC1" or arr.dtype == np.float32:
        return np.asarray(arr, dtype=np.float32)
    return None


def detect_edge_lines(white: np.ndarray, nonwhite: np.ndarray, cell_x: np.ndarray, cell_y: np.ndarray,
                      cfg: SurfaceConfig, rng: Optional[np.random.Generator] = None) -> List[EdgeLine]:
    """Retas longas na fronteira branco/nao-branco da grade (metros).

    white/nonwhite: mascaras booleanas (height, width) da grade fundida
    (nonwhite inclui as celulas "alem do plano" pela profundidade: elas sao
    "fora" com certeza e ajudam a reta a cair na borda REAL).
    Devolve ate edge_max_lines retas, com o lado positivo = fora da mesa.
    """
    rng = rng or np.random.default_rng(12345)
    k3 = np.ones((3, 3), np.uint8)
    nw_d = cv2.dilate(nonwhite.astype(np.uint8), k3) > 0
    boundary = white & nw_d
    pts = np.stack([cell_x[boundary], cell_y[boundary]], axis=1)
    lines: List[EdgeLine] = []
    if pts.shape[0] < 10:
        return lines
    wx, wy = cell_x[white], cell_y[white]
    nx, ny = cell_x[nonwhite], cell_y[nonwhite]
    remaining = pts.copy()
    for _ in range(int(cfg.edge_max_lines)):
        n = remaining.shape[0]
        if n < 10:
            break
        best = None
        for _it in range(int(cfg.edge_ransac_iters)):
            i, j = rng.integers(0, n, size=2)
            if i == j:
                continue
            p, q = remaining[i], remaining[j]
            d = q - p
            L = float(np.hypot(d[0], d[1]))
            if L < 0.08:
                continue
            a, b = -d[1] / L, d[0] / L            # normal unitaria
            c = -(a * p[0] + b * p[1])
            dist = np.abs(a * remaining[:, 0] + b * remaining[:, 1] + c)
            inl = dist <= cfg.edge_inlier_m
            cnt = int(np.count_nonzero(inl))
            if best is None or cnt > best[0]:
                best = (cnt, a, b, c, inl)
        if best is None or best[0] < 10:
            break
        cnt, a, b, c, inl = best
        # Reajuste por minimos quadrados (PCA) nos inliers.
        P = remaining[inl]
        mean = P.mean(axis=0)
        u, sv, vt = np.linalg.svd(P - mean, full_matrices=False)
        d = vt[0]
        a, b = -d[1], d[0]
        c = -(a * mean[0] + b * mean[1])
        dist = np.abs(a * remaining[:, 0] + b * remaining[:, 1] + c)
        inl = dist <= cfg.edge_inlier_m
        P = remaining[inl]
        t = (P - mean) @ d
        span = float(t.max() - t.min()) if P.shape[0] else 0.0
        remaining = remaining[~inl]          # consome os pontos mesmo se a reta for rejeitada
        if span < cfg.edge_min_length_m or P.shape[0] < 10:
            continue
        bins = np.floor((t - t.min()) / 0.02).astype(int)
        coverage = float(len(np.unique(bins))) / float(max(1, int(span / 0.02) + 1))
        if coverage < cfg.edge_min_coverage:
            continue
        # Lado de fora: faixa de ate edge_band_m de cada lado, so' na extensao da reta (+15 cm).
        band = float(cfg.edge_band_m)
        ext = span / 2.0 + 0.15

        def side_counts(xs, ys):
            sd = a * xs + b * ys + c
            tt = (xs - mean[0]) * d[0] + (ys - mean[1]) * d[1]
            in_ext = np.abs(tt) <= ext
            pos = int(np.count_nonzero(in_ext & (sd > 0.01) & (sd <= band)))
            neg = int(np.count_nonzero(in_ext & (sd < -0.01) & (sd >= -band)))
            return pos, neg
        w_pos, w_neg = side_counts(wx, wy)
        n_pos, n_neg = side_counts(nx, ny)
        frac_pos = n_pos / float(max(1, n_pos + w_pos))
        frac_neg = n_neg / float(max(1, n_neg + w_neg))
        if frac_pos >= cfg.edge_outside_min_frac and frac_neg <= 1.0 - cfg.edge_outside_min_frac:
            sign = 1.0
        elif frac_neg >= cfg.edge_outside_min_frac and frac_pos <= 1.0 - cfg.edge_outside_min_frac:
            sign = -1.0
        else:
            continue   # fronteira ambigua (sombra? objeto comprido?) — nao e' borda
        # Alem de uma borda de verdade nao ha tampo: branco do lado de fora
        # (ate edge_outside_check_m) = face de objeto, nao borda.
        far = float(cfg.edge_outside_check_m)
        sd_w = sign * (a * wx + b * wy + c)
        sd_n = sign * (a * nx + b * ny + c)
        tw = (wx - mean[0]) * d[0] + (wy - mean[1]) * d[1]
        tn = (nx - mean[0]) * d[0] + (ny - mean[1]) * d[1]
        w_out = int(np.count_nonzero((np.abs(tw) <= ext) & (sd_w > 0.01) & (sd_w <= far)))
        n_out = int(np.count_nonzero((np.abs(tn) <= ext) & (sd_n > 0.01) & (sd_n <= far)))
        if w_out > cfg.edge_outside_max_white_frac * max(1, w_out + n_out):
            continue
        lines.append(EdgeLine(sign * a, sign * b, sign * c, int(P.shape[0]), span, float(mean[0]), float(mean[1])))
    return lines


class SurfaceGridFusion:
    """Grade no plano da mesa + votos branco/nao-branco por celula, fundindo
    varias vistas; a classificacao final (livre / objeto / fora) usa as retas
    de borda detectadas na grade fundida."""

    def __init__(self, cfg: SurfaceConfig):
        self.cfg = cfg
        self.width = max(1, int(round((cfg.grid_x_max - cfg.grid_x_min) / cfg.resolution)))
        self.height = max(1, int(round((cfg.grid_y_max - cfg.grid_y_min) / cfg.resolution)))
        xs = cfg.grid_x_min + (np.arange(self.width) + 0.5) * cfg.resolution
        ys = cfg.grid_y_min + (np.arange(self.height) + 0.5) * cfg.resolution
        self.cell_x, self.cell_y = np.meshgrid(xs, ys)   # shape (height, width)
        self.reset()

    def reset(self) -> None:
        shape = (self.height, self.width)
        self.votes_white = np.zeros(shape, dtype=np.int32)
        self.votes_nonwhite = np.zeros(shape, dtype=np.int32)
        self.votes_beyond = np.zeros(shape, dtype=np.int32)   # profundidade alem do plano
        self.votes_white_depth = np.zeros(shape, dtype=np.int32)   # brancos de quadros COM profundidade
        self.frames = 0
        self.last_stamp = None
        self.lines: List[EdgeLine] = []

    def project_cells(self, K: np.ndarray, D: Optional[np.ndarray], cam_origin: np.ndarray,
                      cam_rot: np.ndarray, plane_z: float, img_w: int, img_h: int):
        """Para cada celula, o pixel (u, v) da camera FULL-RES que a ve; NaN se
        a celula esta fora da imagem, atras da camera, longe demais, vista de
        raspao ou fora do cone de MAX_RAY_HALF_ANGLE_DEG (onde a distorcao de D
        dobraria o ponto para dentro da imagem)."""
        X = self.cell_x.ravel()
        Y = self.cell_y.ravel()
        Z = np.full_like(X, float(plane_z))
        P = np.stack([X, Y, Z], axis=1)                       # (N, 3) no frame base
        d = P - cam_origin.reshape(1, 3)
        dist = np.linalg.norm(d, axis=1)
        with np.errstate(invalid="ignore", divide="ignore"):
            elev = np.degrees(np.arcsin(np.clip(np.abs(d[:, 2]) / np.maximum(dist, 1e-9), 0.0, 1.0)))
        Pc = (cam_rot.T @ d.T).T                              # (N, 3) no frame da camera (optico: z para frente)
        # Cone: fora de MAX_RAY_HALF_ANGLE_DEG (e alem do raio em que o
        # polinomio de D dobra) o ponto nao esta na imagem, mesmo que
        # cv2.projectPoints devolva um pixel "dentro" dela. dist == |Pc|.
        cos_cone = math.cos(math.radians(MAX_RAY_HALF_ANGLE_DEG))
        r_fold = distortion_fold_tan(D)
        with np.errstate(invalid="ignore", divide="ignore"):
            r_norm = np.hypot(Pc[:, 0], Pc[:, 1]) / np.maximum(Pc[:, 2], 1e-9)
        ok = (Pc[:, 2] > 0.02) & (dist <= self.cfg.max_ray_length_m) & (elev >= self.cfg.min_ray_elevation_deg) \
            & (d[:, 2] < 0.0) \
            & (Pc[:, 2] >= dist * cos_cone) & (r_norm <= 0.95 * r_fold)   # camera acima do plano; dentro do cone
        uv = np.full((X.shape[0], 2), np.nan, dtype=np.float32)
        if np.any(ok):
            pts = Pc[ok].reshape(-1, 1, 3).astype(np.float64)
            rvec = np.zeros(3)
            tvec = np.zeros(3)
            Dv = D if (D is not None and len(D)) else np.zeros(5)
            proj, _ = cv2.projectPoints(pts, rvec, tvec, K.astype(np.float64), Dv.astype(np.float64))
            proj = proj.reshape(-1, 2)
            inside = (proj[:, 0] >= 0) & (proj[:, 0] <= img_w - 1) & (proj[:, 1] >= 0) & (proj[:, 1] <= img_h - 1)
            sub = np.full_like(proj, np.nan, dtype=np.float64)
            sub[inside] = proj[inside]
            uv[ok] = sub.astype(np.float32)
        return uv.reshape(self.height, self.width, 2)

    def fuse(self, label_small: np.ndarray, scale: float, uv_full: np.ndarray, stamp=None,
             depth_used: bool = False) -> np.ndarray:
        """Amostra os rotulos (imagem escalada) nas celulas e soma votos.
        depth_used: o quadro tinha rotulo de profundidade (so' esses brancos
        podem disputar com "alem"). Devolve o rotulo por celula desta vista (LBL_*)."""
        map_x = uv_full[:, :, 0] * float(scale)
        map_y = uv_full[:, :, 1] * float(scale)
        nan = ~np.isfinite(map_x) | ~np.isfinite(map_y)
        map_x = np.where(nan, -1.0, map_x).astype(np.float32)
        map_y = np.where(nan, -1.0, map_y).astype(np.float32)
        sampled = cv2.remap(label_small, map_x, map_y, interpolation=cv2.INTER_NEAREST,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=int(LBL_UNKNOWN))
        sampled = sampled.astype(np.uint8)
        sampled[nan] = LBL_UNKNOWN
        self.votes_white += (sampled == LBL_WHITE)
        self.votes_nonwhite += (sampled == LBL_NONWHITE)
        self.votes_beyond += (sampled == LBL_BEYOND)
        if depth_used:
            self.votes_white_depth += (sampled == LBL_WHITE)
        self.frames += 1
        self.last_stamp = stamp
        return sampled

    def masks(self):
        """(seen, white, nonwhite, beyond) booleanos (height, width), disjuntos
        entre si (menos seen). Alem do plano ganha de branco em empate (a
        profundidade nao atravessa mesa) e so' brancos de quadros COM
        profundidade votam contra ele: um quadro fundido so' por cor (depth
        perdida/velha depois da graca) nao refuta "alem". Branco x nao-branco:
        empate = nao-branco."""
        seen = (self.votes_white + self.votes_nonwhite + self.votes_beyond) > 0
        beyond = seen & (self.votes_beyond > 0) & (self.votes_beyond >= self.votes_white_depth)
        nonwhite = seen & ~beyond & (self.votes_nonwhite >= self.votes_white)      # empate = nao-branco
        white = seen & ~beyond & ~nonwhite & (self.votes_white >= self.cfg.free_min_votes)
        return seen, white, nonwhite, beyond

    def cells(self, update_lines: bool = True) -> np.ndarray:
        """int8 (height, width) com os codigos CELL_*."""
        seen, white, nonwhite, beyond = self.masks()
        if update_lines:
            self.lines = detect_edge_lines(white, nonwhite | beyond, self.cell_x, self.cell_y, self.cfg)
        out = np.full((self.height, self.width), CELL_UNKNOWN, dtype=np.int8)
        out[nonwhite] = CELL_OBSTACLE
        out[white] = CELL_TABLE_FREE
        out[beyond] = CELL_OFF_TABLE          # direto: nao depende das retas
        for ln in self.lines:
            outside = seen & (ln.signed(self.cell_x, self.cell_y) > 0.0)
            out[outside] = CELL_OFF_TABLE
        return out

    def summary(self) -> Dict[str, int]:
        c = self.cells(update_lines=False)
        return {
            "free": int(np.count_nonzero(c == CELL_TABLE_FREE)),
            "obstacle": int(np.count_nonzero(c == CELL_OBSTACLE)),
            "off": int(np.count_nonzero(c == CELL_OFF_TABLE)),
            "unknown": int(np.count_nonzero(c == CELL_UNKNOWN)),
            "frames": int(self.frames),
            "lines": len(self.lines),
            "beyond": int(np.count_nonzero(self.votes_beyond > 0)),
        }


def table_polygon_from_cells(cells: np.ndarray, cell_x: np.ndarray, cell_y: np.ndarray,
                             epsilon_m: float = 0.015, min_cells: int = 50) -> List[Tuple[float, float]]:
    """Contorno do TAMPO VISTO como poligono (metros, frame da grade): o maior
    componente de celulas TABLE_FREE | OBSTACLE, contorno externo simplificado
    (cv2.approxPolyDP, tolerancia epsilon_m). O contorno segue a borda das
    celulas (a mascara e' ampliada 2x e dilatada 1 px: sobra 1/4 de celula para
    fora). Lista vazia se nao ha tampo (menos de min_cells celulas)."""
    cells = np.asarray(cells)
    h, w = cells.shape
    on = ((cells == CELL_TABLE_FREE) | (cells == CELL_OBSTACLE)).astype(np.uint8)
    if int(np.count_nonzero(on)) < int(min_cells) or w < 2 or h < 2:
        return []
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(on, connectivity=8)
    if n < 2:
        return []
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    m = (lbl == k).astype(np.uint8)
    res_x = float(cell_x[0, 1] - cell_x[0, 0])
    res_y = float(cell_y[1, 0] - cell_y[0, 0])
    up = cv2.resize(m, (2 * w, 2 * h), interpolation=cv2.INTER_NEAREST)
    up = cv2.dilate(up, np.ones((3, 3), np.uint8))
    cs, _ = cv2.findContours(up, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs:
        return []
    c = max(cs, key=cv2.contourArea)
    eps_px = float(epsilon_m) / (max(abs(res_x), 1e-9) / 2.0)
    approx = cv2.approxPolyDP(c, eps_px, True).reshape(-1, 2)
    if approx.shape[0] < 3:
        return []
    x0 = float(cell_x[0, 0]) - res_x / 2.0
    y0 = float(cell_y[0, 0]) - res_y / 2.0
    return [(x0 + (float(px) + 0.5) * res_x / 2.0, y0 + (float(py) + 0.5) * res_y / 2.0) for px, py in approx]


def draw_overlay(bgr_full: np.ndarray, scale: float, res: FrameResult, cfg: SurfaceConfig,
                 holding: bool, status_lines: List[str]) -> np.ndarray:
    out = bgr_full.copy()
    H, W = out.shape[:2]
    lbl = cv2.resize(res.label, (W, H), interpolation=cv2.INTER_NEAREST)
    tint = np.zeros_like(out)
    tint[lbl == LBL_WHITE] = (0, 200, 0)
    tint[lbl == LBL_NONWHITE] = (0, 0, 255)
    tint[lbl == LBL_BEYOND] = (255, 80, 0)          # azul: alem do plano (chao) pela profundidade
    tint[lbl == LBL_UNKNOWN] = (128, 128, 128)
    out = cv2.addWeighted(out, 0.65, tint, 0.35, 0)
    h, w = res.label.shape[:2]
    excl = exclusion_mask(cfg, w, h, holding)
    cs, _ = cv2.findContours(excl, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cs:
        cv2.polylines(out, [(c.astype(float) / scale).astype(np.int32)], True, (0, 255, 255), 1)
    y = 16
    for line in status_lines:
        cv2.putText(out, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        y += 16
    return out


def grid_to_image(cells: np.ndarray, scale: int = 4, lines: Optional[List[EdgeLine]] = None,
                  fusion: Optional["SurfaceGridFusion"] = None) -> np.ndarray:
    """Grade -> imagem BGR (y crescendo para CIMA = fundo da mesa no topo, robo embaixo;
    x crescendo para a DIREITA = direita do robo). Retas de borda em amarelo."""
    h, w = cells.shape
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[cells == CELL_UNKNOWN] = (90, 90, 90)
    img[cells == CELL_TABLE_FREE] = (235, 235, 235)
    img[cells == CELL_OFF_TABLE] = (60, 30, 30)
    img[cells == CELL_OBSTACLE] = (0, 0, 220)
    img = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
    if lines and fusion is not None:
        res = fusion.cfg.resolution
        for ln in lines:
            d = np.array([ln.b, -ln.a])
            for sgn in (-1.0, 1.0):
                p0 = np.array([ln.cx, ln.cy])
                p1 = p0 + sgn * d * (ln.length_m / 2.0)
                u0 = int((p0[0] - fusion.cfg.grid_x_min) / res * scale)
                v0 = int((p0[1] - fusion.cfg.grid_y_min) / res * scale)
                u1 = int((p1[0] - fusion.cfg.grid_x_min) / res * scale)
                v1 = int((p1[1] - fusion.cfg.grid_y_min) / res * scale)
                cv2.line(img, (u0, v0), (u1, v1), (0, 255, 255), 2)
    return cv2.flip(img, 0)


def load_config_yaml(path: str) -> SurfaceConfig:
    import yaml
    with open(path, "r", encoding="utf-8") as handle:
        root = yaml.safe_load(handle) or {}
    params = root.get("workstation_surface", {}).get("ros__parameters", root)
    return config_from_dict(params)


def _coerce(kind, value):
    if kind is bool:
        if isinstance(value, str):
            v = value.strip().lower()
            if v in ("true", "1", "yes", "on"):
                return True
            if v in ("false", "0", "no", "off", ""):
                return False
            raise ValueError(f"valor booleano invalido: {value!r}")
        return bool(value)
    if kind is int:
        return int(float(value))
    return kind(value)


SIMPLE_KEYS = [
    "grid_x_min", "grid_x_max", "grid_y_min", "grid_y_max", "resolution", "process_scale",
    "white_s_max", "white_v_min", "table_component_min_frac", "table_min_frac",
    "white_open_px", "obstacle_open_px", "obstacle_close_px", "hole_max_frac", "obstacle_dilate_px",
    "exclusion_dilate_px", "gripper_joint", "gripper_holding_above",
    "max_ray_length_m", "min_ray_elevation_deg", "image_border_px",
    "free_min_votes", "edge_inlier_m", "edge_min_length_m", "edge_min_coverage", "edge_max_lines",
    "edge_ransac_iters", "edge_outside_min_frac", "edge_band_m",
    "edge_outside_check_m", "edge_outside_max_white_frac",
    "depth_enable", "depth_min_valid_m", "depth_beyond_plane_m", "depth_above_plane_m",
    "depth_open_px", "depth_above_max_frac",
]


def config_from_dict(p: dict) -> SurfaceConfig:
    cfg = SurfaceConfig()
    for k in SIMPLE_KEYS:
        if k in p:
            setattr(cfg, k, _coerce(type(getattr(cfg, k)), p[k]))
    if "exclusion_rects_norm" in p:
        cfg.exclusion_rects = parse_rects(p["exclusion_rects_norm"])
    if "exclusion_polys_norm" in p:
        cfg.exclusion_polys = parse_polys(p["exclusion_polys_norm"])
    if "exclusion_polys_holding_norm" in p:
        cfg.exclusion_polys_holding = parse_polys(p["exclusion_polys_holding_norm"])
    if cfg.resolution <= 0.0 or cfg.grid_x_max <= cfg.grid_x_min or cfg.grid_y_max <= cfg.grid_y_min:
        raise ValueError("grade invalida (resolution <= 0 ou limites trocados)")
    return cfg


def load_gripper_polys_from_yaml(path: str, cfg: SurfaceConfig) -> SurfaceConfig:
    """Reaproveita os poligonos da garra do container_detector.yaml (fonte unica)."""
    import yaml
    with open(path, "r", encoding="utf-8") as handle:
        root = yaml.safe_load(handle) or {}
    p = root.get("container_detector", {}).get("ros__parameters", root)
    if "exclusion_polys_norm" in p:
        cfg.exclusion_polys = parse_polys(p["exclusion_polys_norm"])
    if "exclusion_polys_holding_norm" in p:
        cfg.exclusion_polys_holding = parse_polys(p["exclusion_polys_holding_norm"])
    if "exclusion_rects_norm" in p:
        cfg.exclusion_rects = parse_rects(p["exclusion_rects_norm"])
    if "gripper_joint" in p:
        cfg.gripper_joint = str(p["gripper_joint"])
    if "gripper_holding_above" in p:
        cfg.gripper_holding_above = float(p["gripper_holding_above"])
    return cfg


# ============================================================================
# CLI offline
# ============================================================================
def run_offline(args) -> int:
    cfg = load_config_yaml(args.config) if args.config else SurfaceConfig()
    if args.gripper_yaml:
        cfg = load_gripper_polys_from_yaml(args.gripper_yaml, cfg)
    bgr = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if bgr is None:
        print(f"nao consegui abrir {args.image}", file=sys.stderr)
        return 2
    holding = not args.gripper_open
    H, W = bgr.shape[:2]
    # Camera sintetica (x y z pitch_deg em manip_base_link) + intrinsecos: usados
    # pela projecao na grade e pela profundidade esperada.
    cam_origin = cam_rot = K = None
    if args.camera:
        x, y, z, pitch = [float(v) for v in args.camera.split(",")]
        a = math.radians(pitch)
        zax = np.array([0.0, math.cos(a), -math.sin(a)])
        xax = np.array([-1.0, 0.0, 0.0])
        yax = np.cross(zax, xax)
        cam_rot = np.stack([xax, yax, zax], axis=1)
        cam_origin = np.array([x, y, z])
        if args.intrinsics:
            fx, fy, cx, cy = [float(v) for v in args.intrinsics.split(",")]
        else:
            fx, fy, cx, cy = 380.0, 380.0, W / 2.0, H / 2.0
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    depth_label = None
    if args.depth:
        if K is None:
            print("--depth precisa de --camera (pose + plano para a profundidade esperada)", file=sys.stderr)
            return 2
        raw = cv2.imread(args.depth, cv2.IMREAD_UNCHANGED)
        if raw is None:
            print(f"nao consegui abrir {args.depth}", file=sys.stderr)
            return 2
        if raw.ndim == 3:
            raw = raw[:, :, 0]
        depth_m = depth_image_to_m(raw, "16UC1" if raw.dtype == np.uint16 else "32FC1", args.depth_scale)
        if depth_m.shape != (H, W):
            depth_m = cv2.resize(depth_m, (W, H), interpolation=cv2.INTER_NEAREST)
        s = float(cfg.process_scale)
        w_s, h_s = int(round(W * s)), int(round(H * s))
        small = cv2.resize(depth_m, (w_s, h_s), interpolation=cv2.INTER_NEAREST)
        exp = expected_plane_range(K, None, cam_origin, cam_rot, args.plane_z, w_s, h_s, s)
        depth_label = classify_depth(small, exp, cfg)
        valid = np.isfinite(small) & (small >= cfg.depth_min_valid_m)
        print(f"profundidade {args.depth}: {np.count_nonzero(valid) * 100.0 / small.size:.0f}% valida "
              f"(>= {cfg.depth_min_valid_m} m); mediana valida "
              f"{(np.median(small[valid]) if valid.any() else float('nan')):.3f} m; "
              f"esperada no centro {exp[h_s // 2, w_s // 2]:.3f} m")
    t0 = time.perf_counter()
    res = segment_frame(bgr, cfg, holding=holding, depth_label=depth_label)
    dt = (time.perf_counter() - t0) * 1000.0
    print(f"imagem {W}x{H}, process_scale {cfg.process_scale}, {dt:.1f} ms")
    print("garra: " + ("SEGURANDO cubo" if holding else "vazia (--gripper-open)"))
    print(f"branco: S<={cfg.white_s_max} V>={cfg.white_v_min}; HSV p5/p50/p95 no branco: {res.white_stats}")
    print(f"mesa {'VISTA' if res.table_found else 'NAO vista'}: {res.table_frac*100:.1f}% da imagem branca "
          f"(minimo {cfg.table_min_frac*100:.0f}%), {res.n_nonwhite_blobs} mancha(s) nao branca(s)")
    if res.depth_used:
        print(f"profundidade: alem do plano {res.depth_beyond_frac*100:.1f}% | acima do tampo "
              f"{res.depth_above_frac*100:.1f}%" + (" (IGNORADA: plano errado?)" if res.depth_above_rejected else ""))
    lbl = res.label
    tot = lbl.size
    for name, code in (("tampo branco", LBL_WHITE), ("nao branco", LBL_NONWHITE),
                       ("alem do plano", LBL_BEYOND), ("desconhecido", LBL_UNKNOWN)):
        print(f"  {name:13s}: {np.count_nonzero(lbl == code) * 100.0 / tot:5.1f}%")
    if K is not None:
        # Projecao com a camera sintetica: grade + retas + poligono do tampo.
        fusion = SurfaceGridFusion(cfg)
        uv = fusion.project_cells(K, None, cam_origin, cam_rot, args.plane_z, W, H)
        fusion.fuse(res.label, cfg.process_scale, uv, depth_used=res.depth_used)
        cells = fusion.cells()
        print("grade:", fusion.summary())
        for ln in fusion.lines:
            print("  reta:", ln.describe())
        poly = table_polygon_from_cells(cells, fusion.cell_x, fusion.cell_y)
        print("poligono do tampo: " + (", ".join(f"({px:+.2f},{py:.2f})" for px, py in poly) if poly else "nenhum"))
        if args.out:
            cv2.imwrite(args.out.replace(".png", "_grid.png"), grid_to_image(cells, 4, fusion.lines, fusion))
    if args.probe:
        u, v = [int(x) for x in args.probe.split(",")]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        print(f"probe ({u},{v}): HSV = {tuple(int(x) for x in hsv[v, u])}  (H 0-179)")
    if args.out:
        status = [f"{dt:.1f} ms", f"mesa {res.table_frac*100:.0f}% manchas {res.n_nonwhite_blobs}"]
        cv2.imwrite(args.out, draw_overlay(bgr, cfg.process_scale, res, cfg, holding, status))
        print(f"overlay salvo em {args.out}")
    return 0


# ============================================================================
# No ROS
# ============================================================================
def ros_main(argv=None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
    from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_msgs.msg import Bool, ColorRGBA, Float64, String
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import Point, Point32, PolygonStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    from cv_bridge import CvBridge
    from rclpy.time import Time
    from rclpy.duration import Duration

    def stamp_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    class WorkstationSurface(Node):
        def __init__(self):
            super().__init__("workstation_surface")
            _dyn = ParameterDescriptor(dynamic_typing=True)

            def dp(name, default):
                return self.declare_parameter(name, default, _dyn)
            dp("rate_hz", 3.0)
            dp("always_on", False)
            dp("image_topic", "/camera/camera/color/image_raw")
            dp("camera_info_topic", "/camera/camera/color/camera_info")
            # Profundidade ALINHADA a cor (align_depth.enable no launch): mesma
            # geometria/camera_info da cor. depth_enable e as folgas estao em SIMPLE_KEYS.
            dp("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
            dp("depth_max_age_s", 0.5)        # |stamp cor - stamp depth| maior que isto = ignora a profundidade
            dp("depth_grace_s", 1.0)          # apos ativar, espera a profundidade ate isto antes de fundir so' por cor
            dp("observe_topic", "/manip/surface_observe")
            dp("plane_z_topic", "/manip/table_plane_z")
            dp("grid_topic", "/manip/workstation_surface/grid")
            dp("base_frame", "manip_base_link")
            dp("camera_frame", "")
            dp("plane_z_default", -0.029)      # tampo de mesa de 15 cm em manip_base_link (sem FK)
            dp("tf_timeout_s", 0.0)
            dp("reject_when_camera_moving", True)
            dp("camera_motion_max_m", 0.005)
            dp("camera_motion_max_deg", 1.0)
            dp("debug_image", True)
            dp("save_debug_dir", "")
            dp("save_debug_every_s", 1.0)
            dp("joint_states_topic", "/joint_states")
            # Poligonos da garra: por default lidos do container_detector.yaml
            # (fonte unica; camera e garra sao as mesmas). "" = usa os daqui.
            dp("gripper_mask_yaml", "")
            defaults = SurfaceConfig()
            for k in SIMPLE_KEYS:
                dp(k, getattr(defaults, k))
            dp("exclusion_rects_norm", [0.30, 0.00, 0.62, 0.42])
            dp("exclusion_polys_norm", [" ".join(f"{x:.3f},{y:.3f}" for x, y in poly)
                                        for poly in DEFAULT_GRIPPER_POLYS])
            dp("exclusion_polys_holding_norm", [" ".join(f"{x:.3f},{y:.3f}" for x, y in poly)
                                                for poly in DEFAULT_GRIPPER_POLYS_HOLDING])
            try:
                self.cfg = self._build_config()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"parametros invalidos no arranque ({exc}) — usando defaults.")
                self.cfg = SurfaceConfig()
            self.add_on_set_parameters_callback(self._on_params)
            if hasattr(self, "add_post_set_parameters_callback"):
                self.add_post_set_parameters_callback(self._on_params_applied)
            self._pending_cfg: Optional[SurfaceConfig] = None

            self.bridge = CvBridge()
            self.K: Optional[np.ndarray] = None
            self.D: Optional[np.ndarray] = None
            self.img_size = (0, 0)
            self.camera_frame_from_info = ""
            self.plane_z: Optional[float] = None
            self.fusion = SurfaceGridFusion(self.cfg)
            self.last_processed = 0.0
            self.last_cam_pose = None
            self.image_sub = None
            self.depth_sub = None
            self.depth_raw: Optional[bytes] = None      # ultima profundidade serializada (raw=True)
            self.active_since: Optional[float] = None   # monotonic da ultima ativacao (graca da profundidade)
            self._exp_cache = None                      # (chave, profundidade esperada) da ultima pose
            self.observe = False
            self.gripper_holding = True
            self.gripper_pos = None
            self.warned_tf = 0.0
            self.warned_no_info = False
            self.warned_no_plane = False
            self.warned_depth_enc = False
            self._last_save_wall = 0.0

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                CameraInfo, self.get_parameter("camera_info_topic").value, self._on_camera_info,
                QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           history=QoSHistoryPolicy.KEEP_LAST))
            self.create_subscription(Bool, self.get_parameter("observe_topic").value, self._on_observe, latched)
            self.create_subscription(Float64, self.get_parameter("plane_z_topic").value, self._on_plane_z, latched)
            self.create_subscription(JointState, self.get_parameter("joint_states_topic").value,
                                     self._on_joint_states, 10)
            self.grid_pub = self.create_publisher(OccupancyGrid, self.get_parameter("grid_topic").value, latched)
            # Vetores/poligonos (so' publicados com assinante): contorno do tampo
            # visto e retas de borda, para o vision_debug e o RViz.
            self.poly_pub = self.create_publisher(PolygonStamped, "/manip/workstation_surface/table_polygon", latched)
            self.lines_pub = self.create_publisher(MarkerArray, "/manip/workstation_surface/edge_lines", latched)
            self.status_pub = self.create_publisher(String, "/manip/workstation_surface/status", 5)
            self.debug_pub = self.create_publisher(Image, "/manip/workstation_surface/debug_image", 1)
            self.grid_img_pub = self.create_publisher(Image, "/manip/workstation_surface/grid_image", 1)
            self.get_logger().info(
                "workstation_surface pronto: grade %dx%d @ %.3f m (x %.2f..%.2f, y %.2f..%.2f); ativo %s."
                % (self.fusion.width, self.fusion.height, self.cfg.resolution, self.cfg.grid_x_min,
                   self.cfg.grid_x_max, self.cfg.grid_y_min, self.cfg.grid_y_max,
                   "sempre" if self.get_parameter("always_on").value else "so durante a observacao"))
            self._update_active()

        # ---------------- parametros ----------------
        def _config_dict(self, overrides=None) -> dict:
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            keys = SIMPLE_KEYS + ["exclusion_rects_norm", "exclusion_polys_norm", "exclusion_polys_holding_norm"]
            p = {k: g(k) for k in keys}
            for k, v in (overrides or {}).items():
                if k in p:
                    p[k] = v
            return p

        def _build_config(self) -> SurfaceConfig:
            cfg = config_from_dict(self._config_dict())
            path = str(self.get_parameter("gripper_mask_yaml").value or "").strip()
            if path:
                try:
                    cfg = load_gripper_polys_from_yaml(os.path.expanduser(path), cfg)
                    self.get_logger().info(f"poligonos da garra lidos de {path}")
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"nao li os poligonos da garra de {path} ({exc}) — usando os parametros daqui.")
            return cfg

        def _on_params(self, params):
            overrides = {prm.name: prm.value for prm in params}
            try:
                cfg = config_from_dict(self._config_dict(overrides))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"parametro rejeitado: {exc}")
                return SetParametersResult(successful=False, reason=str(exc))
            self._pending_cfg = cfg
            if not hasattr(self, "add_post_set_parameters_callback"):
                self._apply_cfg(cfg)
            return SetParametersResult(successful=True)

        def _on_params_applied(self, params):  # noqa: ARG002
            if self._pending_cfg is not None:
                self._apply_cfg(self._pending_cfg)
                self._pending_cfg = None
            if any(prm.name == "always_on" for prm in params):
                self._update_active()

        def _apply_cfg(self, cfg: SurfaceConfig):
            regrid = (cfg.resolution != self.cfg.resolution or cfg.grid_x_min != self.cfg.grid_x_min or
                      cfg.grid_x_max != self.cfg.grid_x_max or cfg.grid_y_min != self.cfg.grid_y_min or
                      cfg.grid_y_max != self.cfg.grid_y_max)
            self.cfg = cfg
            if regrid:
                self.fusion = SurfaceGridFusion(cfg)
            else:
                self.fusion.cfg = cfg
            self.get_logger().info("parametros recarregados.")

        # ---------------- ativacao ----------------
        def _on_observe(self, msg: Bool):
            want = bool(msg.data)
            if want and not self.observe:
                self.fusion.reset()
                self.last_cam_pose = None
                self.active_since = time.monotonic()
                self.get_logger().info("observacao iniciada: grade zerada.")
            elif not want and self.observe:
                s = self.fusion.summary()
                self.get_logger().info(f"observacao encerrada: {s}")
            self.observe = want
            self._update_active()

        def _update_active(self):
            want = self.observe or bool(self.get_parameter("always_on").value)
            sensor = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST)
            if want and self.image_sub is None:
                self.image_sub = self.create_subscription(
                    Image, self.get_parameter("image_topic").value, self._on_image, sensor)
                if bool(self.cfg.depth_enable):
                    # raw=True: so' desserializa a profundidade do quadro processado (3 Hz),
                    # nao os 15 Hz que chegam.
                    self.depth_sub = self.create_subscription(
                        Image, self.get_parameter("depth_topic").value, self._on_depth_raw, sensor, raw=True)
                self.active_since = time.monotonic()
                self.get_logger().info("ativo: assinando a imagem" +
                                       (" e a profundidade alinhada." if self.depth_sub is not None else "."))
            elif not want and self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
                if self.depth_sub is not None:
                    self.destroy_subscription(self.depth_sub)
                    self.depth_sub = None
                self.depth_raw = None
                self.active_since = None
                self.get_logger().info("inativo: imagem desassinada (0%% CPU).")

        # ---------------- entradas ----------------
        def _on_camera_info(self, msg: CameraInfo):
            K = np.array(msg.k, dtype=float).reshape(3, 3)
            if self.K is None or not np.allclose(K, self.K):
                self.K = K
                self.D = np.array(msg.d, dtype=float) if len(msg.d) else None
                self.img_size = (int(msg.width), int(msg.height))
                self.camera_frame_from_info = msg.header.frame_id
                self.get_logger().info(
                    f"camera_info: fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f} "
                    f"{msg.width}x{msg.height} frame={msg.header.frame_id}")

        def _on_plane_z(self, msg: Float64):
            changed = self.plane_z is None or abs(float(msg.data) - self.plane_z) > 0.003
            self.plane_z = float(msg.data)
            if changed:
                self.fusion.reset()
            self.get_logger().info(f"plano do tampo recebido: z={self.plane_z:.3f} ({self.get_parameter('base_frame').value})")

        def _on_joint_states(self, msg: JointState):
            try:
                idx = list(msg.name).index(self.cfg.gripper_joint)
            except ValueError:
                return
            if idx < len(msg.position):
                self.gripper_pos = float(msg.position[idx])
                self.gripper_holding = self.gripper_pos > float(self.cfg.gripper_holding_above)

        def _on_depth_raw(self, raw: bytes):
            self.depth_raw = raw

        # ---------------- profundidade ----------------
        def _expected_depth(self, origin, rot, plane_z, w, h, scale):
            """expected_plane_range com cache: a camera fica parada durante a
            observacao, entao a mesma pose/plano reaproveita a conta."""
            key = (tuple(np.round(origin, 4)), tuple(np.round(rot.ravel(), 4)), round(float(plane_z), 4), w, h, scale)
            if self._exp_cache is None or self._exp_cache[0] != key:
                self._exp_cache = (key, expected_plane_range(self.K, self.D, origin, rot, plane_z, w, h, scale))
            return self._exp_cache[1]

        def _depth_labels(self, color_stamp, origin, rot, plane_z, W, H):
            """Rotulos de profundidade (resolucao processada) para o quadro de cor
            com esse stamp, ou (None, motivo)."""
            if not bool(self.cfg.depth_enable):
                return None, "off"
            if self.depth_raw is None:
                return None, "sem"
            try:
                dmsg = deserialize_message(self.depth_raw, Image)
            except Exception as exc:  # noqa: BLE001
                return None, f"erro({exc.__class__.__name__})"
            age = abs(stamp_s(color_stamp) - stamp_s(dmsg.header.stamp))
            if age > float(self.get_parameter("depth_max_age_s").value):
                return None, f"velha({age:.2f}s)"
            raw = self.bridge.imgmsg_to_cv2(dmsg, desired_encoding="passthrough")
            depth_m = depth_image_to_m(raw, dmsg.encoding)
            if depth_m is None:
                if not self.warned_depth_enc:
                    self.warned_depth_enc = True
                    self.get_logger().warn(f"profundidade com encoding {dmsg.encoding!r} (esperava 16UC1/32FC1) — ignorada.")
                return None, f"enc={dmsg.encoding}"
            if depth_m.ndim == 3:
                depth_m = depth_m[:, :, 0]
            if depth_m.shape != (H, W):
                # Alinhada a cor deveria ter o mesmo tamanho; redimensiona por NEAREST (0 continua 0).
                depth_m = cv2.resize(depth_m, (W, H), interpolation=cv2.INTER_NEAREST)
            scale = float(self.cfg.process_scale)
            w_s, h_s = int(round(W * scale)), int(round(H * scale))
            small = cv2.resize(depth_m, (w_s, h_s), interpolation=cv2.INTER_NEAREST) if scale != 1.0 else depth_m
            exp = self._expected_depth(origin, rot, plane_z, w_s, h_s, scale)
            return classify_depth(small, exp, self.cfg), "ok"

        # ---------------- vetores / poligonos ----------------
        def _publish_polygon(self, cells, stamp, plane_z):
            pts = table_polygon_from_cells(cells, self.fusion.cell_x, self.fusion.cell_y)
            msg = PolygonStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.get_parameter("base_frame").value
            msg.polygon.points = [Point32(x=float(x), y=float(y), z=float(plane_z)) for (x, y) in pts]
            self.poly_pub.publish(msg)

        def _publish_edge_lines(self, lines, stamp, plane_z):
            """MarkerArray: DELETEALL + por reta um LINE_LIST (ns 'edge', os 2
            extremos) e um ARROW (ns 'edge_normal') do meio para FORA; frente/
            fundo em amarelo, laterais em ciano. Texto com describe()."""
            frame = self.get_parameter("base_frame").value
            arr = MarkerArray()
            clear = Marker()
            clear.header.frame_id = frame
            clear.header.stamp = stamp
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            z = float(plane_z) + 0.005
            for i, ln in enumerate(lines):
                d = np.array([ln.b, -ln.a])
                p0 = np.array([ln.cx, ln.cy]) - d * ln.length_m / 2.0
                p1 = np.array([ln.cx, ln.cy]) + d * ln.length_m / 2.0
                mid = np.array([ln.cx, ln.cy])
                lateral = abs(ln.a) > abs(ln.b)
                color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0) if lateral else ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                seg = Marker()
                seg.header.frame_id = frame
                seg.header.stamp = stamp
                seg.ns = EDGE_MARKER_NS
                seg.id = i
                seg.type = Marker.LINE_LIST
                seg.action = Marker.ADD
                seg.pose.orientation.w = 1.0
                seg.scale.x = 0.01
                seg.color = color
                seg.points = [Point(x=float(p0[0]), y=float(p0[1]), z=z), Point(x=float(p1[0]), y=float(p1[1]), z=z)]
                seg.text = ln.describe()
                arr.markers.append(seg)
                arrow = Marker()
                arrow.header.frame_id = frame
                arrow.header.stamp = stamp
                arrow.ns = EDGE_NORMAL_MARKER_NS
                arrow.id = i
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.orientation.w = 1.0
                arrow.scale.x = 0.01
                arrow.scale.y = 0.02
                arrow.scale.z = 0.02
                arrow.color = color
                tip = mid + np.array([ln.a, ln.b]) * 0.05
                arrow.points = [Point(x=float(mid[0]), y=float(mid[1]), z=z), Point(x=float(tip[0]), y=float(tip[1]), z=z)]
                arr.markers.append(arrow)
            self.lines_pub.publish(arr)

        def _camera_pose(self, stamp):
            base = self.get_parameter("base_frame").value
            cam = self.get_parameter("camera_frame").value or self.camera_frame_from_info or "camera_color_optical_frame"
            timeout = Duration(seconds=max(0.0, float(self.get_parameter("tf_timeout_s").value)))
            used_latest = False
            try:
                tf = self.tf_buffer.lookup_transform(base, cam, stamp, timeout)
            except (LookupException, ConnectivityException, ExtrapolationException):
                try:
                    tf = self.tf_buffer.lookup_transform(base, cam, Time(), timeout)
                    used_latest = True
                except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                    now = time.monotonic()
                    if now - self.warned_tf > 5.0:
                        self.warned_tf = now
                        self.get_logger().warn(f"sem TF {base} <- {cam}: {exc}")
                    return None, None, used_latest
            t = tf.transform.translation
            q = tf.transform.rotation
            return np.array([t.x, t.y, t.z]), quat_to_rot(q.x, q.y, q.z, q.w), used_latest

        # ---------------- pipeline ----------------
        def _on_image(self, msg: Image):
            rate = float(self.get_parameter("rate_hz").value)
            now_mono = time.monotonic()
            if rate > 0 and now_mono - self.last_processed < 1.0 / rate:
                return
            self.last_processed = now_mono
            try:
                self._process(msg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"erro no processamento: {exc!r}")

        def _process(self, msg: Image):
            t0 = time.perf_counter()
            if self.K is None:
                if not self.warned_no_info:
                    self.warned_no_info = True
                    self.get_logger().warn("sem camera_info ainda — nao processo.")
                self._publish_status("sem camera_info")
                return
            stamp = Time.from_msg(msg.header.stamp)
            origin, rot, used_latest = self._camera_pose(stamp)
            if origin is None:
                self._publish_status("sem TF da camera")
                return
            if bool(self.get_parameter("reject_when_camera_moving").value) and self.last_cam_pose is not None:
                o0, r0 = self.last_cam_pose
                dpos = float(np.linalg.norm(origin - o0))
                dang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r0.T @ rot) - 1.0) / 2.0))))
                if dpos > float(self.get_parameter("camera_motion_max_m").value) or \
                        dang > float(self.get_parameter("camera_motion_max_deg").value):
                    self.last_cam_pose = (origin, rot)
                    self._publish_status(f"camera movendo (d={dpos*100:.1f}cm, {dang:.1f}deg) — frame ignorado")
                    return
            self.last_cam_pose = (origin, rot)

            plane_z = self.plane_z
            src = "topic"
            if plane_z is None:
                plane_z = float(self.get_parameter("plane_z_default").value)
                src = "default"
                if not self.warned_no_plane:
                    self.warned_no_plane = True
                    self.get_logger().warn(f"sem {self.get_parameter('plane_z_topic').value} — usando plane_z_default={plane_z:.3f}")
            if origin[2] <= plane_z + 0.03:
                self._publish_status(f"camera abaixo/na altura do plano (z_cam={origin[2]:.3f}, plano={plane_z:.3f}) — frame ignorado")
                return

            H, W = int(msg.height), int(msg.width)
            depth_label, depth_why = self._depth_labels(msg.header.stamp, origin, rot, plane_z, W, H)
            if depth_label is None:
                since = None if self.active_since is None else time.monotonic() - self.active_since
                grace = float(self.get_parameter("depth_grace_s").value)
                if depth_wait_pending(depth_why, since, grace):
                    # O primeiro quadro de cor chega antes da primeira profundidade:
                    # fundir so' por cor agora pintaria o chao de "tampo livre".
                    self.last_processed = 0.0        # tenta de novo ja no proximo quadro
                    self._publish_status(f"aguardando profundidade ({depth_why}, {since:.1f}s de {grace:.1f}s) "
                                         "— quadro nao fundido")
                    return
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            holding = bool(self.gripper_holding)
            res = segment_frame(bgr, self.cfg, holding=holding, depth_label=depth_label)
            uv = self.fusion.project_cells(self.K, self.D, origin, rot, plane_z, W, H)
            view = self.fusion.fuse(res.label, float(self.cfg.process_scale), uv, stamp=msg.header.stamp,
                                    depth_used=res.depth_used)
            n_seen = int(np.count_nonzero(view != LBL_UNKNOWN))
            cells = self.fusion.cells(update_lines=True)
            summary = self.fusion.summary()
            self._publish_grid(cells, msg.header.stamp)
            if self.poly_pub.get_subscription_count() > 0:
                self._publish_polygon(cells, msg.header.stamp, plane_z)
            if self.lines_pub.get_subscription_count() > 0:
                self._publish_edge_lines(self.fusion.lines, msg.header.stamp, plane_z)
            dt_ms = (time.perf_counter() - t0) * 1000.0
            lines_txt = "; ".join(ln.describe() for ln in self.fusion.lines) or "nenhuma"
            if res.depth_used:
                depth_txt = (f"depth=ok alem={res.depth_beyond_frac*100:.0f}% acima={res.depth_above_frac*100:.0f}%"
                             + ("(IGNORADA: plano errado?)" if res.depth_above_rejected else ""))
            else:
                depth_txt = f"depth={depth_why}" + (" (so cor)" if depth_why in ("sem",) or depth_why.startswith("velha") else "")
            status = (f"frames={summary['frames']} mesa={'sim' if res.table_found else 'NAO'} "
                      f"({res.table_frac*100:.0f}% branco) manchas={res.n_nonwhite_blobs} "
                      f"vista={n_seen}cel | grade livre={summary['free']} obst={summary['obstacle']} "
                      f"fora={summary['off']} desc={summary['unknown']} | bordas: {lines_txt} | {depth_txt} | "
                      f"garra={'segurando' if holding else 'vazia'} "
                      f"cam z={origin[2]:.3f} plano={plane_z:.3f}({src}) tf={'latest' if used_latest else 'stamp'} {dt_ms:.0f}ms")
            self._publish_status(status)
            want_pub = bool(self.get_parameter("debug_image").value)
            if want_pub and self.debug_pub.get_subscription_count() > 0:
                overlay = draw_overlay(bgr, self.cfg.process_scale, res, self.cfg, holding, [status[:90], status[90:180]])
                out = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                out.header = msg.header
                self.debug_pub.publish(out)
            if want_pub and self.grid_img_pub.get_subscription_count() > 0:
                gi = grid_to_image(cells, 4, self.fusion.lines, self.fusion)
                out = self.bridge.cv2_to_imgmsg(gi, encoding="bgr8")
                out.header = msg.header
                self.grid_img_pub.publish(out)
            save_dir = str(self.get_parameter("save_debug_dir").value or "").strip()
            now_wall = time.time()
            if save_dir and now_wall - self._last_save_wall >= float(self.get_parameter("save_debug_every_s").value):
                self._save_debug(save_dir, bgr, res, holding, status, now_wall)

        def _publish_grid(self, cells, stamp):
            from nav_msgs.msg import OccupancyGrid
            msg = OccupancyGrid()
            msg.header.stamp = stamp
            msg.header.frame_id = self.get_parameter("base_frame").value
            msg.info.resolution = float(self.cfg.resolution)
            msg.info.width = int(self.fusion.width)
            msg.info.height = int(self.fusion.height)
            msg.info.origin.position.x = float(self.cfg.grid_x_min)
            msg.info.origin.position.y = float(self.cfg.grid_y_min)
            msg.info.origin.position.z = float(self.plane_z if self.plane_z is not None else
                                               self.get_parameter("plane_z_default").value)
            msg.info.origin.orientation.w = 1.0
            msg.data = cells.ravel().astype(np.int8).tolist()
            self.grid_pub.publish(msg)

        def _save_debug(self, save_dir, bgr, res, holding, status, now_wall):
            try:
                d = os.path.expanduser(save_dir)
                os.makedirs(d, exist_ok=True)
                ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(now_wall)) + f"_{int((now_wall % 1) * 1000):03d}"
                cv2.imwrite(os.path.join(d, f"{ts}_raw.png"), bgr)
                cv2.imwrite(os.path.join(d, f"{ts}_overlay.png"),
                            draw_overlay(bgr, self.cfg.process_scale, res, self.cfg, holding, [status[:90]]))
                cv2.imwrite(os.path.join(d, f"{ts}_grid.png"),
                            grid_to_image(self.fusion.cells(update_lines=False), 4, self.fusion.lines, self.fusion))
                with open(os.path.join(d, "log.txt"), "a", encoding="utf-8") as fh:
                    fh.write(f"{ts} | {status}\n")
                self._last_save_wall = now_wall
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"nao consegui gravar debug em {save_dir}: {exc}", throttle_duration_sec=10.0)

        def _publish_status(self, text: str):
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=argv)
    node = WorkstationSurface()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()
    return 0


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if any(a == "--image" or a.startswith("--image=") for a in argv):
        parser = argparse.ArgumentParser(description="Calibracao offline do mapa da mesa.")
        parser.add_argument("--image", required=True)
        parser.add_argument("--config", default=None, help="workstation_surface.yaml (default: valores internos)")
        parser.add_argument("--gripper-yaml", default=None, help="container_detector.yaml para os poligonos da garra")
        parser.add_argument("--out", default=None, help="salva overlay PNG (verde=tampo, vermelho=objeto, azul=fora, cinza=desconhecido)")
        parser.add_argument("--probe", default=None, help="u,v: imprime o HSV desse pixel")
        parser.add_argument("--gripper-open", action="store_true", help="foto com a garra vazia")
        parser.add_argument("--camera", default=None,
                            help="x,y,z,pitch_deg de uma camera sintetica (manip_base_link) para projetar na grade")
        parser.add_argument("--plane-z", dest="plane_z", type=float, default=-0.029, help="cota do tampo (com --camera)")
        parser.add_argument("--depth", default=None,
                            help="PNG 16 bits (mm) da profundidade alinhada a cor, mesmo quadro de --image (exige --camera)")
        parser.add_argument("--depth-scale", dest="depth_scale", type=float, default=0.001, help="m por unidade do PNG (D455: 0.001)")
        parser.add_argument("--intrinsics", default=None, help="fx,fy,cx,cy da camera_info (default 380,380,W/2,H/2)")
        return run_offline(parser.parse_args(argv))
    return ros_main(argv)


if __name__ == "__main__":
    sys.exit(main())
