#!/usr/bin/env python3
"""Visao de DEPURACAO: a camera do punho com o que o robo entendeu por cima (2026-10-21).

Por que existe: o operador so' via a imagem crua da RealSense; o apriltag_ros
nao desenha nada e os overlays dos detectores (container, mesa) so' existem
durante o place. Este no junta tudo numa UNICA imagem ao vivo, para conferir
"o que a camera esta vendo" durante um pick/place sem abrir 3 ferramentas:
  a) mapa da mesa (/manip/workstation_surface/grid) projetado de volta na
     imagem — verde = tampo livre, vermelho = objeto, cinza = fora da mesa,
     desconhecido = nada (mesma conta de SurfaceGridFusion.project_cells).
     Pintado por POLIGONOS (paint_mode=polygons, 2026-10-21): contorno de
     cada classe na grade (cv2.findContours + approxPolyDP), arestas
     subdivididas a cada 1 cm no plano antes de projetar (perspectiva certa),
     cv2.fillPoly com alpha; celulas fora/atras da camera ficam de fora. O
     modo por pontos (splat, paint_mode=points) continua como fallback.
     Por cima: contorno do tampo visto (.../table_polygon, branco), as retas
     de borda (.../edge_lines: frente amarela, fundo magenta, laterais ciano,
     seta = lado de FORA) e a ZONA DE MANIPULACAO do rulebook (tracejado
     laranja: 2 a 20 cm da frente, 2 cm das laterais) quando ha frente vista;
  b) containers por cor (TF manip_base_link -> ct_vermelho / ct_azul): caixa
     nominal 170 x 105 x 70 mm projetada (borda = traco grosso, fundo = fino);
  c) AprilTags (/detections): contorno, centro, 'tag_<id>' e decision_margin;
     amarelo = hamming > 0 (leitura com bit corrigido, menos confiavel);
  d) barra de status: PICK / PLACE / observando mesa, ids vistos, containers,
     idade e ORIGEM do mapa ('mapa: depth' = a grade levou profundidade em
     algum quadro; 'mapa: cor' = so' cor, lido do prefixo 'depth=.. quadros=a/b'
     do status latched do no de superficie), ultima frase falada
     (/manip/speech) e o fps real.
Tudo e' gated por IDADE (stamp): tag some ao sair da vista, container e mapa
somem quando o detector para de publicar — nunca fica desenho fantasma.

Custo ZERO sem espectador: imagem, camera_info, deteccoes e TF so' sao
assinados enquanto alguem assina a imagem anotada (conferido 1x/s): sem
espectador o no so' tem os Bool latched e a fala — nada de trafego de
imagem nem de TF (listener de TF em Python custa alguns % de CPU), e o
perception_gate nao conta este no como "espectador" da camera. A imagem
chega crua (raw=True) e e' desserializada so' ao processar; o callback
devolve na hora se o espectador sumiu — mesmo criterio do debug_image dos
detectores. Com espectador, processa no maximo rate_hz quadros/s (~10 ms
cada, medido). So' LE: nao publica em topico de comando, nao chama servico
nem action, nao mexe em TF.

e) DOCKING (2026-10-27, pedido do operador: "o mesmo viewer para as duas
   funcoes, trocando quando precisa"): enquanto o dock_align_node publica a
   imagem de debug do docking por camera (/dock_align/camera_debug), ELA
   sai neste mesmo topico no lugar da visao do pick/place; dock_hold_s sem
   quadro do docking e volta sozinho. Assinada so' com espectador, como o
   resto (e o dock_align_node so' desenha se alguem assina).

Quem usa: o operador (rqt_image_view /manip/vision_debug/image, ou
use_vision_viewer:=true no launch). Nada no robo depende deste no.
Como roda: manip_stack.launch.py (use_vision_debug, default true) ou
  ros2 run manip_bringup vision_debug_node.py
  ros2 param set /vision_debug grid_alpha 0.5      # ajustes ao vivo
Nucleo puro (sem ROS) testado em test/test_vision_debug.py.
"""
import math
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# Codigos das celulas — iguais a workstation_surface_node.py / surface_place_planner.hpp.
CELL_UNKNOWN = -1
CELL_TABLE_FREE = 0
CELL_OFF_TABLE = 70
CELL_OBSTACLE = 100

# Cores (BGR).
COLOR_FREE = (0, 200, 0)
COLOR_OBSTACLE = (0, 0, 255)
COLOR_OFF_TABLE = (150, 150, 150)
COLOR_TAG = (0, 255, 0)
COLOR_TAG_HAMMING = (0, 255, 255)
COLOR_TEXT = (255, 255, 255)
# Frames dos containers por cor (container_detector.yaml: vermelho.frame / azul.frame).
CONTAINER_COLORS: Dict[str, Tuple[int, int, int]] = {
    "ct_vermelho": (0, 0, 255),
    "ct_azul": (255, 100, 0),
}
# (codigo, rotulo uint8, cor): a ORDEM e' a prioridade de pintura onde os
# splats de celulas vizinhas se sobrepoem (objeto por cima de tudo).
_PAINT_ORDER = (
    (CELL_OFF_TABLE, 1, COLOR_OFF_TABLE),
    (CELL_TABLE_FREE, 2, COLOR_FREE),
    (CELL_OBSTACLE, 3, COLOR_OBSTACLE),
)
# Vetores/poligonos publicados pelo workstation_surface_node.
COLOR_TABLE_POLY = (255, 255, 255)
COLOR_EDGE_FRONT = (0, 255, 255)     # amarelo: frente da mesa (fora = lado do robo)
COLOR_EDGE_BACK = (255, 0, 255)      # magenta: fundo
COLOR_EDGE_SIDE = (255, 255, 0)      # ciano: laterais
COLOR_ZONE = (0, 165, 255)           # laranja: zona de manipulacao (rulebook)
EDGE_MARKER_NS = "edge"              # iguais ao workstation_surface_node.py
EDGE_NORMAL_MARKER_NS = "edge_normal"
PAINT_MODES = ("polygons", "points")
# Cone em que a projecao e' confiavel (graus do eixo optico): com o D real da
# D455 (k3 < 0) o polinomio radial deixa de crescer a ~59 graus e
# "dobra" para dentro da imagem pontos que estao FORA do campo de visao. O
# FOV da D455 cor e' 90 x 65 graus (diagonal ~50; 46 em 640x480 com fx=385),
# entao 55 nao perde nada. Mesmo valor de MAX_RAY_HALF_ANGLE_DEG no
# workstation_surface_node.py; distortion_fold_tan(D) e' a guarda extra.
MAX_HALF_ANGLE_DEG = 55.0


# ============================================================================
# Nucleo puro (sem ROS)
# ============================================================================
def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose_to_matrix(tx: float, ty: float, tz: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Translacao + quaternion -> matriz homogenea 4x4."""
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
    T[:3, 3] = (tx, ty, tz)
    return T


def is_fresh(stamp_s: float, now_s: float, max_age_s: float) -> bool:
    """Gate por idade: True se (now - stamp) < max_age. Stamp <= 0 (sem relogio)
    e' sempre velho; stamp no futuro (relogios Pi/PC desencontrados) conta
    como fresco — melhor mostrar do que esconder por 100 ms de desvio."""
    if stamp_s <= 0.0:
        return False
    return (now_s - stamp_s) < float(max_age_s)


def parse_surface_status(text: str) -> Dict[str, str]:
    """Prefixo do status do workstation_surface_node ('depth=ok alem=12%
    acima=3% retas=1 quadros=2/3 | ...') -> {chave: valor} dos tokens
    chave=valor antes do primeiro ' | '. {} sem prefixo."""
    head = str(text or "").split(" | ", 1)[0]
    out: Dict[str, str] = {}
    for tok in head.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            if k:
                out[k] = v
    return out


def grid_origin_from_status(text: str) -> str:
    """'depth' se a grade levou profundidade em algum quadro (quadros=a/b com
    a > 0; sem 'quadros', depth=ok), 'cor' se so' cor, '?' sem status."""
    st = parse_surface_status(text)
    if "quadros" in st:
        try:
            return "depth" if int(st["quadros"].split("/", 1)[0]) > 0 else "cor"
        except ValueError:
            pass
    if "depth" in st:
        return "depth" if st["depth"] == "ok" else "cor"
    return "?"


def grid_cell_centers(origin_x: float, origin_y: float, res: float, width: int, height: int):
    """Centros das celulas de uma OccupancyGrid (origem = canto da celula [0,0]).
    Devolve X, Y com shape (height, width); data[j * width + i] <-> (X[j, i], Y[j, i])."""
    xs = origin_x + (np.arange(int(width)) + 0.5) * float(res)
    ys = origin_y + (np.arange(int(height)) + 0.5) * float(res)
    return np.meshgrid(xs, ys)


def distortion_fold_tan(D: Optional[np.ndarray], r_max: float = 4.0) -> float:
    """Copia de workstation_surface_node.distortion_fold_tan: raio normalizado
    (tan do angulo ao eixo) a partir do qual o modelo radial de D deixa de ser
    crescente (dobra); inf sem dobra, D nulo ou tamanho fora de 4/5/8."""
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


def pinhole_project(K: np.ndarray, D: Optional[np.ndarray], Pc: np.ndarray) -> np.ndarray:
    """Pontos (N, 3) no frame OPTICO (z > 0) -> pixels (N, 2), modelo plumb_bob /
    rational_polynomial do OpenCV (D com 4, 5 ou 8 coeficientes k1 k2 p1 p2
    [k3 [k4 k5 k6]]) em numpy puro: mesmo resultado de cv2.projectPoints
    (testado a 1e-4 px) por ~0,1 ms em vez de ~5 ms (ele monta o jacobiano
    mesmo sem ser pedido). Outros tamanhos de D caem no cv2.projectPoints."""
    Kf = np.asarray(K, dtype=np.float64)
    Dv = np.asarray(D, dtype=np.float64).ravel() if (D is not None and len(D)) else np.zeros(5)
    P = np.asarray(Pc, dtype=np.float64).reshape(-1, 3)
    if np.any(Dv) and Dv.size not in (4, 5, 8):
        proj, _ = cv2.projectPoints(P.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), Kf, Dv)
        return proj.reshape(-1, 2)
    x = P[:, 0] / P[:, 2]
    y = P[:, 1] / P[:, 2]
    if np.any(Dv):
        k = np.zeros(8)
        k[:Dv.size] = Dv
        k1, k2, p1, p2, k3, k4, k5, k6 = k
        r2 = x * x + y * y
        radial = (1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))) / (1.0 + r2 * (k4 + r2 * (k5 + r2 * k6)))
        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x, y = xd, yd
    return np.stack([Kf[0, 0] * x + Kf[0, 1] * y + Kf[0, 2], Kf[1, 1] * y + Kf[1, 2]], axis=1)


def project_base_points(K: np.ndarray, D: Optional[np.ndarray], cam_origin: np.ndarray, cam_rot: np.ndarray,
                        pts_base: np.ndarray, img_w: int = 0, img_h: int = 0,
                        min_depth: float = 0.02) -> np.ndarray:
    """Pontos (N, 3) no frame base -> pixels (N, 2) float32, NaN onde invalido.

    Mesma conta de SurfaceGridFusion.project_cells: d = P - cam_origin;
    Pc = R_cam^T d (frame optico: z para frente); projecao com K e D
    (pinhole_project = cv2.projectPoints em numpy, mais rapido).
    Invalido = atras da camera (Pc.z <= min_depth), ACIMA da camera (d.z >= 0;
    tudo que se projeta aqui esta no plano da mesa, abaixo dela), fora do cone
    de MAX_HALF_ANGLE_DEG, alem do raio em que a distorcao de D dobra
    (distortion_fold_tan) ou, se img_w/img_h > 0, fora da imagem.
    cam_rot/cam_origin = transform base <- camera_optical (colunas = eixos da
    camera no base)."""
    P = np.asarray(pts_base, dtype=np.float64).reshape(-1, 3)
    d = P - np.asarray(cam_origin, dtype=np.float64).reshape(1, 3)
    Pc = (np.asarray(cam_rot, dtype=np.float64).T @ d.T).T
    norm = np.linalg.norm(Pc, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        r_norm = np.hypot(Pc[:, 0], Pc[:, 1]) / np.maximum(Pc[:, 2], 1e-9)
    ok = (Pc[:, 2] > float(min_depth)) & (d[:, 2] < 0.0) \
        & (Pc[:, 2] >= norm * math.cos(math.radians(MAX_HALF_ANGLE_DEG))) \
        & (r_norm <= 0.95 * distortion_fold_tan(D))
    uv = np.full((P.shape[0], 2), np.nan, dtype=np.float32)
    if np.any(ok):
        proj = pinhole_project(K, D, Pc[ok])
        if img_w > 0 and img_h > 0:
            inside = (proj[:, 0] >= 0) & (proj[:, 0] <= img_w - 1) & (proj[:, 1] >= 0) & (proj[:, 1] <= img_h - 1)
            proj = np.where(inside[:, None], proj, np.nan)
        uv[ok] = proj.astype(np.float32)
    return uv


def container_corners_base(T_base_ct: np.ndarray, long_m: float, short_m: float, dz: float = 0.0) -> np.ndarray:
    """4 cantos (4, 3) do retangulo nominal do container no frame base.
    Centrado no frame ct_*, X = lado longo, Y = lado curto, no plano z = dz do
    frame (0 = borda/rim; -container_height_m = fundo). Ordem: +X+Y, -X+Y,
    -X-Y, +X-Y (poligono fechado)."""
    hl, hs = float(long_m) / 2.0, float(short_m) / 2.0
    local = np.array([
        [hl, hs, dz, 1.0],
        [-hl, hs, dz, 1.0],
        [-hl, -hs, dz, 1.0],
        [hl, -hs, dz, 1.0],
    ])
    return (np.asarray(T_base_ct, dtype=np.float64) @ local.T).T[:, :3]


def _auto_radius(uv: np.ndarray) -> int:
    """Metade da distancia mediana entre centros CONSECUTIVOS (vizinhos na
    mesma linha da grade; a volta de linha e' 1 em `width` e nao mexe na
    mediana), +1 para fechar as frestas. Entre 1 e 12 px."""
    d = np.linalg.norm(np.diff(uv, axis=0), axis=1)
    d = d[np.isfinite(d)]
    if d.size == 0:
        return 3
    # 0.75 do espacamento cobre a diagonal entre 4 vizinhos (0.71); com 0.5
    # a tinta saia pontilhada perto da camera.
    return int(min(12, max(1, math.ceil(float(np.median(d)) * 0.75) + 1)))


def paint_grid(img: np.ndarray, uv: np.ndarray, codes: np.ndarray, alpha: float = 0.35,
               radius: int = 0) -> np.ndarray:
    """Tinge a imagem (in place; devolve a mesma) nas celulas projetadas.

    uv: (N, 2) pixels dos centros (NaN = nao desenha); codes: (N,) CELL_*.
    Desconhecido (-1) nao pinta. "Splat por indice": cada centro vira 1 pixel
    numa mascara por classe, dilatada ate' cobrir o espaco entre vizinhos;
    uma LUT pinta e um addWeighted mistura so' onde ha rotulo.
    Medido (640x480, grade 120x70 = 8400 celulas, ~1800 visiveis, numpy+cv2):
      cv2.circle por celula      15 ms (cresce com o numero de celulas)
      fillPoly dos 4 cantos      23 ms (exato, mas caro)
      splat + dilate + LUT      ~2 ms (cresce so' com a imagem)  <- este
    (indexacao booleana do numpy no lugar do copyTo/max do OpenCV custava
    18 ms: por isso tudo aqui e' operacao do OpenCV.)
    radius=0: automatico (0,75 x mediana da distancia entre vizinhos + 1)."""
    H, W = img.shape[:2]
    uv = np.asarray(uv, dtype=np.float32).reshape(-1, 2)
    codes = np.asarray(codes).ravel()
    if codes.shape[0] != uv.shape[0]:
        raise ValueError(f"paint_grid: {uv.shape[0]} pixels para {codes.shape[0]} celulas")
    valid = np.isfinite(uv).all(axis=1) & (codes != CELL_UNKNOWN)
    if not np.any(valid):
        return img
    r = int(radius) if radius > 0 else _auto_radius(uv)
    ui = np.clip(np.round(uv[valid, 0]).astype(np.int32), 0, W - 1)
    vi = np.clip(np.round(uv[valid, 1]).astype(np.int32), 0, H - 1)
    cc = codes[valid]
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    label = np.zeros((H, W), dtype=np.uint8)
    for code, idx, _color in _PAINT_ORDER:
        sel = cc == code
        if not np.any(sel):
            continue
        m = np.zeros((H, W), dtype=np.uint8)
        m[vi[sel], ui[sel]] = idx
        m = cv2.dilate(m, kernel)            # maximo local: espalha o idx no raio
        label = cv2.max(label, m)            # idx crescente = prioridade (objeto por cima)
    return _blend_label(img, label, alpha)


def _paint_lut() -> np.ndarray:
    lut = np.zeros((256, 1, 3), dtype=np.uint8)
    for _code, idx, color in _PAINT_ORDER:
        lut[idx] = color
    return lut


def _blend_label(img: np.ndarray, label: np.ndarray, alpha: float) -> np.ndarray:
    """Tinge img (in place) onde label != 0, com a cor da LUT por rotulo."""
    tint = cv2.LUT(cv2.merge([label, label, label]), _paint_lut())
    blended = cv2.addWeighted(img, 1.0 - float(alpha), tint, float(alpha), 0.0)
    out = cv2.copyTo(blended, label, img)    # so' onde ha rotulo (label != 0 e' a mascara)
    if out is not img:                       # binding sem escrita in place: copia de volta
        np.copyto(img, out)
    return img


# ---------------------------------------------------------------------------
# Poligonos / vetores
# ---------------------------------------------------------------------------
def grid_class_contours(codes: np.ndarray, width: int, height: int, valid: Optional[np.ndarray] = None,
                        epsilon_cells: float = 0.75) -> Dict[int, List[np.ndarray]]:
    """Contornos de cada classe da grade, em COORDENADAS DE CELULA (i, j) float
    (centro da celula [j, i] = (i, j)): {codigo: [(N, 2), ...]}, contornos
    externos E buracos juntos (cv2.fillPoly numa chamada so' deixa o buraco
    vazio). valid (height*width,) booleano: celulas cujo centro projeta na
    imagem; as outras ficam de fora (recorte por celula valida). A mascara e'
    ampliada 2x e dilatada 1 px para o contorno seguir a BORDA das celulas
    (sobra 1/4 de celula) e nao os centros; approxPolyDP com epsilon_cells."""
    width, height = int(width), int(height)
    grid = np.asarray(codes).reshape(height, width)
    vmask = None if valid is None else np.asarray(valid, dtype=bool).reshape(height, width)
    out: Dict[int, List[np.ndarray]] = {}
    for code, _idx, _color in _PAINT_ORDER:
        m = grid == code
        if vmask is not None:
            m = m & vmask
        if not np.any(m):
            continue
        up = cv2.resize(m.astype(np.uint8), (2 * width, 2 * height), interpolation=cv2.INTER_NEAREST)
        up = cv2.dilate(up, np.ones((3, 3), np.uint8))
        cs, _ = cv2.findContours(up, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        polys: List[np.ndarray] = []
        for c in cs:
            if c.shape[0] < 3:
                continue
            a = cv2.approxPolyDP(c, float(epsilon_cells) * 2.0, True).reshape(-1, 2).astype(np.float64)
            if a.shape[0] >= 3:
                polys.append((a + 0.5) / 2.0 - 0.5)     # px da imagem 2x -> celulas
        if polys:
            out[code] = polys
    return out


def cells_to_base(pts_cells: np.ndarray, origin_x: float, origin_y: float, res: float) -> np.ndarray:
    """(N, 2) em coordenadas de celula (i, j) -> (N, 2) metros no frame da grade."""
    p = np.asarray(pts_cells, dtype=np.float64).reshape(-1, 2)
    return np.stack([float(origin_x) + (p[:, 0] + 0.5) * float(res),
                     float(origin_y) + (p[:, 1] + 0.5) * float(res)], axis=1)


def subdivide_polygon(pts: np.ndarray, step_m: float, closed: bool = True) -> np.ndarray:
    """Insere pontos ao longo das arestas para nenhum trecho passar de step_m
    (reta no plano -> curva na imagem com distorcao; e o recorte por celula
    valida fica fiel). Fechado: a ultima aresta volta ao primeiro ponto (o
    primeiro nao e' repetido no fim). Aberto: termina no ultimo ponto."""
    P = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    n = P.shape[0]
    if n < 2 or step_m <= 0.0:
        return P
    pieces = []
    last = n if closed else n - 1
    for k in range(last):
        a = P[k]
        b = P[(k + 1) % n]
        m = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / float(step_m))))
        t = (np.arange(m, dtype=np.float64) / m)[:, None]
        pieces.append(a[None, :] + t * (b - a)[None, :])
    out = np.concatenate(pieces, axis=0)
    if not closed:
        out = np.vstack([out, P[-1:]])
    return out


def project_base_polygon(K: np.ndarray, D: Optional[np.ndarray], cam_origin: np.ndarray, cam_rot: np.ndarray,
                         pts_xy: np.ndarray, z: float, step_m: float = 0.01, closed: bool = True) -> np.ndarray:
    """Poligono/polilinha (N, 2) no plano z do frame base -> pixels (M, 2)
    float32 com as arestas subdivididas a cada step_m; NaN onde o vertice nao
    projeta (atras/acima da camera, fora do cone). Sem recorte pela imagem:
    o OpenCV clipa ao desenhar."""
    P = subdivide_polygon(pts_xy, step_m, closed)
    if P.shape[0] == 0:
        return np.zeros((0, 2), np.float32)
    P3 = np.column_stack([P, np.full(P.shape[0], float(z))])
    return project_base_points(K, D, cam_origin, cam_rot, P3)


def paint_grid_polygons(img: np.ndarray, polys_px: Dict[int, Sequence[np.ndarray]], alpha: float = 0.35) -> np.ndarray:
    """Tinge a imagem (in place) com os poligonos projetados de cada classe:
    {codigo: [(M, 2) px, ...]} (vertices NaN descartados). Mesma LUT/alpha/
    prioridade de paint_grid; fillPoly com 1/8 px (shift=3)."""
    H, W = img.shape[:2]
    label = np.zeros((H, W), dtype=np.uint8)
    painted = False
    for code, idx, _color in _PAINT_ORDER:
        cs = []
        for p in polys_px.get(code, ()):
            p = np.asarray(p, dtype=np.float64).reshape(-1, 2)
            p = p[np.isfinite(p).all(axis=1)]
            if p.shape[0] >= 3:
                cs.append(np.round(np.clip(p, -1e5, 1e5) * 8.0).astype(np.int32).reshape(-1, 1, 2))
        if cs:
            cv2.fillPoly(label, cs, int(idx), cv2.LINE_8, 3)
            painted = True
    if not painted:
        return img
    return _blend_label(img, label, alpha)


def draw_base_polyline(img: np.ndarray, uv: np.ndarray, color: Tuple[int, int, int], thickness: int = 2,
                       closed: bool = True, dashed: bool = False, dash: int = 2) -> np.ndarray:
    """Desenha a polilinha (M, 2) px (de project_base_polygon): trecho com
    vertice NaN e' pulado; dashed = desenha `dash` trechos e pula `dash`
    (trechos de step_m no plano: tracos de tamanho fisico igual)."""
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    if uv.shape[0] < 2:
        return img
    if closed:
        uv = np.vstack([uv, uv[:1]])
    for k in range(uv.shape[0] - 1):
        if dashed and (k // max(1, int(dash))) % 2 == 1:
            continue
        a, b = uv[k], uv[k + 1]
        if not (np.isfinite(a).all() and np.isfinite(b).all()):
            continue
        cv2.line(img, (int(round(a[0])), int(round(a[1]))), (int(round(b[0])), int(round(b[1]))),
                 color, int(thickness), cv2.LINE_AA)
    return img


def edge_kind(n: np.ndarray) -> str:
    """Tipo da borda pela normal de FORA (manip_base_link: +X direita do robo,
    +Y para a frente): frente (fora = -Y, lado do robo), fundo (+Y),
    esquerda (-X), direita (+X)."""
    if abs(float(n[1])) >= abs(float(n[0])):
        return "frente" if float(n[1]) < 0.0 else "fundo"
    return "direita" if float(n[0]) > 0.0 else "esquerda"


def edge_lines_from_markers(markers: Sequence[dict]) -> List[dict]:
    """MarkerArray do workstation_surface_node (ja como dicts {ns, id, points
    [(x, y, z), ...]}, so' os ADD) -> retas: {p0, p1, mid, n, a, b, c, z,
    length, kind}; a*x + b*y + c = 0 normalizada com o lado POSITIVO = fora da
    mesa (sentido da seta ns 'edge_normal' do mesmo id). Reta sem seta e'
    descartada (sem o lado de fora nao serve para a zona)."""
    segs: Dict[int, np.ndarray] = {}
    normals: Dict[int, np.ndarray] = {}
    for m in markers:
        pts = np.asarray(m.get("points", ()), dtype=np.float64).reshape(-1, 3)
        if pts.shape[0] < 2 or not np.isfinite(pts[:2]).all():
            continue
        ns = str(m.get("ns", ""))
        if ns == EDGE_MARKER_NS:
            segs[int(m.get("id", 0))] = pts[:2]
        elif ns == EDGE_NORMAL_MARKER_NS:
            normals[int(m.get("id", 0))] = pts[:2]
    out: List[dict] = []
    for i, seg in sorted(segs.items()):
        if i not in normals:
            continue
        p0, p1 = seg[0, :2], seg[1, :2]
        d = p1 - p0
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            continue
        d = d / L
        n = np.array([-d[1], d[0]])
        nv = normals[i][1, :2] - normals[i][0, :2]
        if float(np.dot(nv, n)) < 0.0:
            n = -n
        mid = (p0 + p1) / 2.0
        a, b = float(n[0]), float(n[1])
        c = -(a * float(mid[0]) + b * float(mid[1]))
        out.append({"p0": p0, "p1": p1, "mid": mid, "n": n, "a": a, "b": b, "c": c,
                    "z": float(seg[0, 2]), "length": L, "kind": edge_kind(n)})
    return out


def _intersect(l1, l2) -> Optional[np.ndarray]:
    a1, b1, c1 = l1
    a2, b2, c2 = l2
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-9:
        return None
    return np.array([(b1 * c2 - b2 * c1) / det, (a2 * c1 - a1 * c2) / det])


def manipulation_zone_polygon(lines: Sequence[dict], depth_m: float = 0.20, edge_clearance_m: float = 0.02,
                              front_offset_m: float = 0.02) -> Optional[np.ndarray]:
    """Zona de manipulacao do rulebook (4.4) a partir das retas de borda
    (edge_lines_from_markers): faixa de front_offset_m a depth_m para DENTRO da
    frente (a mais longa), limitada de cada lado pela lateral detectada
    (recuada edge_clearance_m) ou, sem lateral daquele lado, pela ponta do
    trecho VISTO da frente. (4, 2) metros: frente-esq, frente-dir, fundo-dir,
    fundo-esq; None sem frente ou zona degenerada."""
    fronts = [ln for ln in lines if ln.get("kind") == "frente"]
    if not fronts:
        return None
    f = max(fronts, key=lambda ln: ln["length"])
    a, b, c = f["a"], f["b"], f["c"]
    # Deslocar a*x + b*y + c = 0 em s para o lado NEGATIVO (dentro): c + s.
    front_in = (a, b, c + float(front_offset_m))
    back_in = (a, b, c + float(depth_m))
    d = np.array([b, -a])                     # tangente da frente
    bounds = []
    for sgn in (-1.0, 1.0):
        end = f["mid"] + sgn * d * f["length"] / 2.0
        side = [ln for ln in lines if ln.get("kind") in ("esquerda", "direita")
                and float(np.dot(ln["n"], sgn * d)) > 0.5]
        if side:
            lat = max(side, key=lambda ln: ln["length"])
            bounds.append((lat["a"], lat["b"], lat["c"] + float(edge_clearance_m)))
        else:
            nn = sgn * d
            bounds.append((float(nn[0]), float(nn[1]), -float(np.dot(nn, end))))
    corners = [_intersect(front_in, bounds[0]), _intersect(front_in, bounds[1]),
               _intersect(back_in, bounds[1]), _intersect(back_in, bounds[0])]
    if any(p is None for p in corners):
        return None
    poly = np.array(corners)
    if float(np.dot(poly[1] - poly[0], d)) <= 0.0:    # laterais cruzadas: zona vazia
        return None
    return poly


def edge_color(kind: str) -> Tuple[int, int, int]:
    return {"frente": COLOR_EDGE_FRONT, "fundo": COLOR_EDGE_BACK}.get(kind, COLOR_EDGE_SIDE)


def draw_edge_lines(img: np.ndarray, K, D, cam_origin, cam_rot, lines: Sequence[dict],
                    step_m: float = 0.01) -> np.ndarray:
    """Retas de borda como vetores: traco grosso + seta de 5 cm para FORA + tipo."""
    for ln in lines:
        color = edge_color(ln.get("kind", ""))
        z = float(ln.get("z", 0.0))
        uv = project_base_polygon(K, D, cam_origin, cam_rot, np.vstack([ln["p0"], ln["p1"]]), z, step_m, closed=False)
        draw_base_polyline(img, uv, color, 3, closed=False)
        tip = ln["mid"] + ln["n"] * 0.05
        arrow = project_base_points(K, D, cam_origin, cam_rot,
                                    np.array([[ln["mid"][0], ln["mid"][1], z], [tip[0], tip[1], z]]))
        if np.isfinite(arrow).all():
            p, q = arrow
            cv2.arrowedLine(img, (int(round(p[0])), int(round(p[1]))), (int(round(q[0])), int(round(q[1]))),
                            color, 2, cv2.LINE_AA, tipLength=0.4)
            x = int(np.clip(q[0] + 4, 2, max(2, img.shape[1] - 90)))
            y = int(np.clip(q[1] + 4, 14, img.shape[0] - 4))
            _text(img, f"{ln.get('kind', '?')} {ln['length'] * 100:.0f}cm", (x, y), 0.42, color)
    return img


def draw_zone(img: np.ndarray, K, D, cam_origin, cam_rot, zone_xy: np.ndarray, z: float,
              step_m: float = 0.01) -> np.ndarray:
    """Zona de manipulacao: poligono tracejado laranja + rotulo."""
    uv = project_base_polygon(K, D, cam_origin, cam_rot, zone_xy, z, step_m, closed=True)
    draw_base_polyline(img, uv, COLOR_ZONE, 2, closed=True, dashed=True, dash=2)
    fin = uv[np.isfinite(uv).all(axis=1)]
    if fin.shape[0]:
        p = fin[np.argmax(fin[:, 1])]      # vertice mais baixo na imagem (perto do robo)
        x = int(np.clip(p[0] - 40, 2, max(2, img.shape[1] - 120)))
        y = int(np.clip(p[1] - 6, 14, img.shape[0] - 4))
        _text(img, "zona 2-20 cm", (x, y), 0.42, COLOR_ZONE)
    return img


def _text(img: np.ndarray, s: str, org: Tuple[int, int], scale: float = 0.45,
          color: Tuple[int, int, int] = COLOR_TEXT, thickness: int = 1) -> None:
    """Texto com contorno preto (legivel sobre tampo branco ou chao escuro)."""
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def _int_pts(pts: np.ndarray) -> np.ndarray:
    """(N, 2) float -> (N, 1, 2) int32 para o OpenCV (clip: fora da imagem e' clipado por ele)."""
    return np.round(np.clip(pts, -1e5, 1e5)).astype(np.int32).reshape(-1, 1, 2)


def draw_containers(img: np.ndarray, boxes: Sequence[dict]) -> np.ndarray:
    """boxes: {name, color, top (4,2), bottom (4,2) | None, dist_m | None}.
    Borda (top) com traco 2; fundo (bottom) com traco 1 + 4 arestas verticais."""
    for box in boxes:
        color = tuple(int(c) for c in box.get("color", (255, 255, 255)))
        top = np.asarray(box["top"], dtype=np.float64).reshape(-1, 2)
        if top.shape[0] != 4 or not np.isfinite(top).all():
            continue
        pts_top = _int_pts(top)
        bottom = box.get("bottom")
        if bottom is not None:
            bottom = np.asarray(bottom, dtype=np.float64).reshape(-1, 2)
            if bottom.shape[0] == 4 and np.isfinite(bottom).all():
                pts_bot = _int_pts(bottom)
                cv2.polylines(img, [pts_bot], True, color, 1, cv2.LINE_AA)
                for a, b in zip(pts_top.reshape(-1, 2), pts_bot.reshape(-1, 2)):
                    cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), color, 1, cv2.LINE_AA)
        cv2.polylines(img, [pts_top], True, color, 2, cv2.LINE_AA)
        cx, cy = top.mean(axis=0)
        label = str(box.get("name", ""))
        if box.get("dist_m") is not None:
            label += f" {float(box['dist_m']):.2f} m"
        # Caixa meio fora da imagem (comum de perto): rotulo preso dentro dela.
        x = int(np.clip(cx - 40, 2, max(2, img.shape[1] - 150)))
        y = int(np.clip(cy + 5, 14, img.shape[0] - 4))
        _text(img, label, (x, y), 0.5, color)
    return img


def draw_tags(img: np.ndarray, dets: Sequence[dict]) -> np.ndarray:
    """dets: {id, hamming, decision_margin, centre (x, y), corners [(x, y)] x 4},
    pixels da imagem FULL. Contorno verde (amarelo se hamming > 0), ponto no
    centro, 'tag_<id>' e a margem de decisao."""
    for det in dets:
        corners = np.asarray(det.get("corners", ()), dtype=np.float64).reshape(-1, 2)
        if corners.shape[0] < 3 or not np.isfinite(corners).all():
            continue
        color = COLOR_TAG_HAMMING if int(det.get("hamming", 0) or 0) > 0 else COLOR_TAG
        cv2.polylines(img, [_int_pts(corners)], True, color, 2, cv2.LINE_AA)
        centre = det.get("centre")
        cx, cy = (float(centre[0]), float(centre[1])) if centre is not None else tuple(corners.mean(axis=0))
        if math.isfinite(cx) and math.isfinite(cy):
            cv2.circle(img, (int(round(cx)), int(round(cy))), 3, color, -1, cv2.LINE_AA)
        x0 = int(np.clip(corners[:, 0].min(), 0, img.shape[1] - 1))
        y0 = int(np.clip(corners[:, 1].min(), 0, img.shape[0] - 1))
        _text(img, f"tag_{int(det.get('id', -1))}", (x0, max(12, y0 - 6)), 0.5, color)
        margin = det.get("decision_margin")
        if margin is not None:
            y1 = int(np.clip(corners[:, 1].max(), 0, img.shape[0] - 1))
            _text(img, f"m={float(margin):.0f}", (x0, min(img.shape[0] - 3, y1 + 14)), 0.38, color)
    return img


def draw_status(img: np.ndarray, lines: Sequence[str], alpha: float = 0.55) -> np.ndarray:
    """Barra escura translucida no topo com uma linha de texto por item."""
    lines = [str(s) for s in lines if s]
    if not lines:
        return img
    H, W = img.shape[:2]
    line_h = 17
    h = int(min(H, 8 + line_h * len(lines)))
    img[:h] = cv2.addWeighted(img[:h], 1.0 - float(alpha), np.zeros_like(img[:h]), float(alpha), 0.0)
    y = 14
    for line in lines:
        cv2.putText(img, line[:120], (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_TEXT, 1, cv2.LINE_AA)
        y += line_h
    return img


# ============================================================================
# No ROS
# ============================================================================
def ros_main(argv=None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from rclpy.duration import Duration
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Bool, String
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PolygonStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from apriltag_msgs.msg import AprilTagDetectionArray
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    from cv_bridge import CvBridge

    def stamp_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    class VisionDebug(Node):
        def __init__(self):
            super().__init__("vision_debug")
            dp = self.declare_parameter
            dp("image_topic", "/camera/camera/color/image_raw")
            dp("camera_info_topic", "/camera/camera/color/camera_info")
            dp("detections_topic", "/detections")
            dp("grid_topic", "/manip/workstation_surface/grid")
            dp("base_frame", "manip_base_link")
            dp("camera_frame", "")            # "" = frame_id do camera_info
            dp("publish_topic", "/manip/vision_debug/image")
            dp("dock_debug_topic", "/dock_align/camera_debug")
            dp("dock_hold_s", 2.0)
            dp("rate_hz", 10.0)
            dp("detections_max_age_s", 0.5)
            dp("tf_max_age_s", 1.0)
            dp("grid_max_age_s", 60.0)   # o place usa o mapa latched ate a base andar; a barra mostra a idade
            dp("grid_alpha", 0.35)
            # Pintura do mapa: polygons (contornos por classe, fillPoly) | points (splat por celula).
            dp("paint_mode", "polygons")
            dp("poly_step_m", 0.01)           # subdivisao das arestas no plano antes de projetar
            dp("edge_lines_topic", "/manip/workstation_surface/edge_lines")
            dp("table_polygon_topic", "/manip/workstation_surface/table_polygon")
            dp("surface_status_topic", "/manip/workstation_surface/status")   # latched; prefixo depth=.. quadros=a/b
            # Zona de manipulacao (rulebook 4.4): 2 a 20 cm da frente, 2 cm das laterais.
            dp("zone_depth_m", 0.20)
            dp("zone_edge_clearance_m", 0.02)
            dp("zone_front_offset_m", 0.02)
            dp("container_long_m", 0.170)     # iguais a config/container_detector.yaml
            dp("container_short_m", 0.105)
            dp("container_height_m", 0.070)
            dp("pick_active_topic", "/manip/pick_active")
            dp("place_active_topic", "/manip/place_active")
            dp("surface_observe_topic", "/manip/surface_observe")
            dp("speech_topic", "/manip/speech_status")

            self.bridge = CvBridge()
            self.K: Optional[np.ndarray] = None
            self.D: Optional[np.ndarray] = None
            self.camera_frame_from_info = ""
            self.dets = None                  # ultimo AprilTagDetectionArray
            self.grid = None                  # ultimo OccupancyGrid
            self.grid_pts: Optional[np.ndarray] = None   # (N, 3) centros no frame da grade
            self.grid_codes: Optional[np.ndarray] = None
            self._grid_key = None
            self.edge_lines: List[dict] = []  # retas de borda (edge_lines_from_markers)
            self.edge_stamp = None
            self.table_poly: Optional[np.ndarray] = None   # (N, 2) contorno do tampo visto
            self.table_poly_z = 0.0
            self.table_stamp = None
            self.pick_active = False
            self.place_active = False
            self.surface_observe = False
            self.surface_status = ""          # ultimo status do no de superficie (origem do mapa)
            self.speech = ""
            self.last_pub_mono = 0.0
            self.fps = 0.0
            self._warned_tf = 0.0
            self._warned_grid_frame = 0.0
            self.next_allowed = 0.0
            # Assinaturas "caras" (imagem, camera_info, deteccoes, TF): so' com
            # espectador — criadas/destruidas em _check_viewer (1 Hz).
            self.image_sub = None
            self.info_sub = None
            self.dock_sub = None
            self.dock_last_mono = -1e9
            self.det_sub = None
            self.tf_buffer = Buffer()
            self.tf_listener = None

            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            self.pub = self.create_publisher(Image, g("publish_topic"), 1)
            self.create_timer(1.0, self._check_viewer)
            self.create_subscription(OccupancyGrid, g("grid_topic"), self._on_grid, latched)
            # Vetores/poligonos: pequenos, latched e so' mudam a 3 Hz durante a
            # observacao — assinados sempre (o no de superficie so' os publica
            # porque este assinante existe).
            self.create_subscription(MarkerArray, g("edge_lines_topic"), self._on_edge_lines, latched)
            self.create_subscription(PolygonStamped, g("table_polygon_topic"), self._on_table_polygon, latched)
            self.create_subscription(Bool, g("pick_active_topic"), self._on_pick_active, latched)
            self.create_subscription(Bool, g("place_active_topic"), self._on_place_active, latched)
            self.create_subscription(Bool, g("surface_observe_topic"), self._on_surface_observe, latched)
            self.create_subscription(String, g("surface_status_topic"), self._on_surface_status, latched)
            self.create_subscription(String, g("speech_topic"), self._on_speech, 10)
            self.get_logger().info(
                f"vision_debug pronto: {g('image_topic')} + tags/containers/mapa -> {g('publish_topic')} "
                f"(so' processa com espectador, ate' {float(g('rate_hz')):.0f} Hz).")

        # ---------------- entradas ----------------
        def _on_camera_info(self, msg: CameraInfo):
            K = np.array(msg.k, dtype=float).reshape(3, 3)
            if self.K is None or not np.allclose(K, self.K):
                self.K = K
                self.D = np.array(msg.d, dtype=float) if len(msg.d) else None
                self.camera_frame_from_info = msg.header.frame_id
                self.get_logger().info(
                    f"camera_info: fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} {msg.width}x{msg.height} frame={msg.header.frame_id}")

        def _on_detections(self, msg: AprilTagDetectionArray):
            self.dets = msg

        def _on_grid(self, msg: OccupancyGrid):
            info = msg.info
            key = (round(info.origin.position.x, 5), round(info.origin.position.y, 5), round(info.origin.position.z, 5),
                   round(info.resolution, 6), int(info.width), int(info.height))
            if key != self._grid_key:
                X, Y = grid_cell_centers(info.origin.position.x, info.origin.position.y, info.resolution,
                                         info.width, info.height)
                Z = np.full(X.size, float(info.origin.position.z))
                self.grid_pts = np.stack([X.ravel(), Y.ravel(), Z], axis=1)
                self._grid_key = key
            self.grid_codes = np.asarray(msg.data, dtype=np.int8)
            self.grid = msg

        def _on_edge_lines(self, msg: MarkerArray):
            adds = [{"ns": m.ns, "id": int(m.id), "points": [(p.x, p.y, p.z) for p in m.points]}
                    for m in msg.markers if m.action == Marker.ADD]
            self.edge_lines = edge_lines_from_markers(adds)
            stamps = [m.header.stamp for m in msg.markers]
            self.edge_stamp = stamps[0] if stamps else None

        def _on_table_polygon(self, msg: PolygonStamped):
            pts = msg.polygon.points
            self.table_poly = np.array([[p.x, p.y] for p in pts], dtype=np.float64) if len(pts) >= 3 else None
            self.table_poly_z = float(pts[0].z) if pts else 0.0
            self.table_stamp = msg.header.stamp

        def _on_pick_active(self, msg: Bool):
            self.pick_active = bool(msg.data)

        def _on_place_active(self, msg: Bool):
            self.place_active = bool(msg.data)

        def _on_surface_observe(self, msg: Bool):
            self.surface_observe = bool(msg.data)

        def _on_surface_status(self, msg: String):
            self.surface_status = str(msg.data)

        def _on_speech(self, msg: String):
            # /manip/speech_status: "falando: X" | "ocioso" | "descartada (...): X".
            # Guarda so' o que o robo de fato esta' falando (a ultima frase fica
            # na tela ate' a proxima). Se apontado para /manip/speech cru, tira
            # o prefixo de prioridade ('!' alta, '~' baixa).
            data = str(msg.data)
            if data.startswith("falando: "):
                self.speech = data[len("falando: "):]
            elif data == "ocioso" or data.startswith("descartada"):
                return
            else:
                self.speech = data.lstrip("!~").strip()

        # ---------------- TF ----------------
        def _lookup(self, target: str, source: str, stamp):
            """TransformStamped target <- source ou None (sem excecao)."""
            try:
                return self.tf_buffer.lookup_transform(target, source, stamp, Duration(seconds=0.0))
            except (LookupException, ConnectivityException, ExtrapolationException):
                return None

        def _camera_pose(self, stamp):
            """(origin, rot, used_latest) de base <- camera_optical no stamp da
            imagem (fallback: ultimo TF disponivel), ou None."""
            base = self.get_parameter("base_frame").value
            cam = self.get_parameter("camera_frame").value or self.camera_frame_from_info or "camera_color_optical_frame"
            tf = self._lookup(base, cam, stamp)
            used_latest = False
            if tf is None:
                tf = self._lookup(base, cam, Time())
                used_latest = True
            if tf is None:
                now = time.monotonic()
                if now - self._warned_tf > 10.0:
                    self._warned_tf = now
                    self.get_logger().warn(f"sem TF {base} <- {cam}")
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            return np.array([t.x, t.y, t.z]), quat_to_rot(q.x, q.y, q.z, q.w), used_latest

        # ---------------- pipeline ----------------
        def _check_viewer(self):
            """Assina imagem/camera_info/deteccoes/TF so' enquanto alguem assina a
            imagem anotada; sem espectador destroi tudo (custo zero)."""
            want = self.pub.get_subscription_count() > 0
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            sensor = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST)
            if want and self.image_sub is None:
                self.tf_buffer.clear()        # nada de ct_* velho da vez passada
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.info_sub = self.create_subscription(
                    CameraInfo, g("camera_info_topic"), self._on_camera_info,
                    QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                               history=QoSHistoryPolicy.KEEP_LAST))
                # BEST_EFFORT casa com o publisher RELIABLE do apriltag_ros e com um best-effort.
                self.det_sub = self.create_subscription(
                    AprilTagDetectionArray, g("detections_topic"), self._on_detections, sensor)
                # raw=True: a imagem chega serializada (bytes) e so' e' desserializada
                # ao processar (o limitador de taxa descarta sem custo).
                self.image_sub = self.create_subscription(
                    Image, g("image_topic"), self._on_image_raw, sensor, raw=True)
                self.dock_sub = self.create_subscription(
                    Image, g("dock_debug_topic"), self._on_dock_debug, 1)
                self.fps = 0.0
                self.last_pub_mono = 0.0
                self.next_allowed = 0.0
                self.get_logger().info("espectador conectado: assinando imagem, camera_info, deteccoes e TF.")
            elif not want and self.image_sub is not None:
                for sub in (self.image_sub, self.info_sub, self.det_sub, self.dock_sub):
                    if sub is not None:
                        self.destroy_subscription(sub)
                self.image_sub = self.info_sub = self.det_sub = self.dock_sub = None
                if self.tf_listener is not None:
                    self.tf_listener.unregister()
                    self.tf_listener = None
                self.dets = None
                self.get_logger().info("sem espectador: tudo desassinado (custo zero).")

        def _on_dock_debug(self, msg):
            """Docking por camera em curso: o quadro dele vai direto para o
            espectador (ja' vem desenhado) e segura a visao do pick/place."""
            self.dock_last_mono = time.monotonic()
            self.pub.publish(msg)

        def _on_image_raw(self, raw: bytes):
            if self.pub.get_subscription_count() == 0:
                return                        # espectador sumiu: nao processa nada
            if time.monotonic() - self.dock_last_mono < float(self.get_parameter("dock_hold_s").value):
                return                        # docking usando o espectador
            rate = float(self.get_parameter("rate_hz").value)
            now_mono = time.monotonic()
            # Limitador por "proximo instante permitido" (nao por intervalo desde
            # o ultimo quadro): com a camera na mesma taxa que rate_hz o jitter
            # rejeitava metade dos quadros (medido 4,6 Hz em vez de 10).
            if rate > 0:
                if now_mono < self.next_allowed:
                    return
                self.next_allowed = max(self.next_allowed + 1.0 / rate, now_mono)
            try:
                self._process(deserialize_message(raw, Image))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"erro no overlay: {exc!r}", throttle_duration_sec=5.0)

        def _process(self, msg: Image):
            t0 = time.perf_counter()
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            H, W = bgr.shape[:2]
            # Referencia de idade = stamp da imagem (mesmo relogio das deteccoes,
            # da grade e dos TFs ct_*, que carregam o stamp da imagem que os
            # gerou; a latencia da camera cancela). Sem stamp: relogio do no.
            ref_s = stamp_s(msg.header.stamp)
            if ref_s <= 0.0:
                ref_s = self.get_clock().now().nanoseconds * 1e-9
            stamp = Time.from_msg(msg.header.stamp)
            cam = self._camera_pose(stamp) if self.K is not None else None
            notes: List[str] = []
            if self.K is None:
                notes.append("sem camera_info")
            elif cam is None:
                notes.append("sem TF da camera")
            elif cam[2]:
                notes.append("tf=latest")

            # a) mapa da mesa
            grid_txt = "mapa: --"
            if self.grid is not None and self.grid_pts is not None:
                age = ref_s - stamp_s(self.grid.header.stamp)
                if not is_fresh(stamp_s(self.grid.header.stamp), ref_s, float(self.get_parameter("grid_max_age_s").value)):
                    grid_txt = f"mapa: velho ({age:.0f} s)"
                elif cam is None:
                    grid_txt = f"mapa: {age:.1f} s (sem projecao)"
                elif self.grid.header.frame_id and self.grid.header.frame_id != self.get_parameter("base_frame").value:
                    grid_txt = f"mapa: frame {self.grid.header.frame_id} != base"
                    now = time.monotonic()
                    if now - self._warned_grid_frame > 30.0:
                        self._warned_grid_frame = now
                        self.get_logger().warn(grid_txt)
                else:
                    uv = project_base_points(self.K, self.D, cam[0], cam[1], self.grid_pts, W, H)
                    alpha = float(self.get_parameter("grid_alpha").value)
                    mode = str(self.get_parameter("paint_mode").value)
                    if mode == "points":
                        paint_grid(bgr, uv, self.grid_codes, alpha)
                    else:
                        info = self.grid.info
                        step = float(self.get_parameter("poly_step_m").value)
                        valid = np.isfinite(uv).all(axis=1)
                        polys = grid_class_contours(self.grid_codes, info.width, info.height, valid)
                        px = {code: [project_base_polygon(
                            self.K, self.D, cam[0], cam[1],
                            cells_to_base(p, info.origin.position.x, info.origin.position.y, info.resolution),
                            info.origin.position.z, step) for p in lst] for code, lst in polys.items()}
                        paint_grid_polygons(bgr, px, alpha)
                    codes = self.grid_codes
                    # Origem da grade: 'depth' se algum quadro fundido levou
                    # profundidade, 'cor' se so' cor (prefixo do status latched).
                    grid_txt = (f"mapa: {grid_origin_from_status(self.surface_status)} {age:.1f} s  "
                                f"livre={int(np.count_nonzero(codes == CELL_TABLE_FREE))} "
                                f"obj={int(np.count_nonzero(codes == CELL_OBSTACLE))} "
                                f"fora={int(np.count_nonzero(codes == CELL_OFF_TABLE))} cel")

            # a2) vetores: contorno do tampo, retas de borda e zona de manipulacao
            if cam is not None:
                max_age = float(self.get_parameter("grid_max_age_s").value)
                step = float(self.get_parameter("poly_step_m").value)
                if self.table_poly is not None and self.table_stamp is not None \
                        and is_fresh(stamp_s(self.table_stamp), ref_s, max_age):
                    uv = project_base_polygon(self.K, self.D, cam[0], cam[1], self.table_poly, self.table_poly_z, step)
                    draw_base_polyline(bgr, uv, COLOR_TABLE_POLY, 2, closed=True)
                if self.edge_lines and self.edge_stamp is not None and is_fresh(stamp_s(self.edge_stamp), ref_s, max_age):
                    draw_edge_lines(bgr, self.K, self.D, cam[0], cam[1], self.edge_lines, step)
                    zone = manipulation_zone_polygon(
                        self.edge_lines, float(self.get_parameter("zone_depth_m").value),
                        float(self.get_parameter("zone_edge_clearance_m").value),
                        float(self.get_parameter("zone_front_offset_m").value))
                    kinds = ", ".join(ln["kind"] for ln in self.edge_lines)
                    if zone is not None:
                        draw_zone(bgr, self.K, self.D, cam[0], cam[1], zone, float(self.edge_lines[0]["z"]), step)
                        grid_txt += f" | bordas: {kinds} | zona ok"
                    else:
                        grid_txt += f" | bordas: {kinds} | sem frente"

            # b) containers por cor (TF base <- ct_*, so' com idade < tf_max_age_s)
            seen_ct: List[str] = []
            if cam is not None:
                base = self.get_parameter("base_frame").value
                long_m = float(self.get_parameter("container_long_m").value)
                short_m = float(self.get_parameter("container_short_m").value)
                height_m = float(self.get_parameter("container_height_m").value)
                max_age = float(self.get_parameter("tf_max_age_s").value)
                boxes = []
                for frame, color in CONTAINER_COLORS.items():
                    tf = self._lookup(base, frame, Time())
                    if tf is None or not is_fresh(stamp_s(tf.header.stamp), ref_s, max_age):
                        continue
                    t, q = tf.transform.translation, tf.transform.rotation
                    T = pose_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)
                    top = project_base_points(self.K, self.D, cam[0], cam[1],
                                              container_corners_base(T, long_m, short_m, 0.0))
                    bottom = project_base_points(self.K, self.D, cam[0], cam[1],
                                                 container_corners_base(T, long_m, short_m, -height_m))
                    dist = float(np.linalg.norm(T[:3, 3] - cam[0]))
                    boxes.append({"name": frame, "color": color, "top": top, "bottom": bottom, "dist_m": dist})
                    seen_ct.append(f"{frame[3:]} {dist:.2f} m")
                draw_containers(bgr, boxes)

            # c) tags
            ids: List[int] = []
            if self.dets is not None and is_fresh(stamp_s(self.dets.header.stamp), ref_s,
                                                  float(self.get_parameter("detections_max_age_s").value)):
                dets = [{
                    "id": int(d.id), "hamming": int(d.hamming), "decision_margin": float(d.decision_margin),
                    "centre": (d.centre.x, d.centre.y), "corners": [(c.x, c.y) for c in d.corners],
                } for d in self.dets.detections]
                draw_tags(bgr, dets)
                ids = sorted(int(d.id) for d in self.dets.detections)

            # d) barra de status
            now_mono = time.monotonic()
            if self.last_pub_mono > 0.0 and now_mono > self.last_pub_mono:
                inst = 1.0 / (now_mono - self.last_pub_mono)
                self.fps = inst if self.fps <= 0.0 else 0.8 * self.fps + 0.2 * inst
            self.last_pub_mono = now_mono
            dt_ms = (time.perf_counter() - t0) * 1000.0
            state = "PICK" if self.pick_active else ("PLACE" if self.place_active else "ocioso")
            if self.surface_observe:
                state += " | observando mesa"
            lines = [
                f"{state} | {self.fps:.1f} fps {dt_ms:.0f} ms" + (" | " + ", ".join(notes) if notes else ""),
                f"tags: {', '.join(str(i) for i in ids) if ids else 'nenhuma'} | "
                f"containers: {', '.join(seen_ct) if seen_ct else 'nenhum'}",
                grid_txt,
            ]
            st = parse_surface_status(self.surface_status)
            if "depth" in st:
                lines.append("superficie: " + self.surface_status.split(" | ", 1)[0])
            if self.speech:
                txt = self.speech.replace("\n", " ")
                lines.append('fala: "' + (txt[:70] + "..." if len(txt) > 70 else txt) + '"')
            draw_status(bgr, lines)

            out = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            out.header = msg.header
            self.pub.publish(out)

    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=argv)
    node = VisionDebug()
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
    return ros_main(sys.argv[1:] if argv is None else argv)


if __name__ == "__main__":
    sys.exit(main())
