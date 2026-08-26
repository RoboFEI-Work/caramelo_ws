#!/usr/bin/env python3
"""Detector de containers (vermelho/azul) por COR com a RealSense do punho.

Por que existe (2026-08-25): os containers da arena nao tem mais AprilTag; o
place precisa do centro do container da cor certa em `manip_base_link` (o
frame da IK custom) para soltar o cubo dentro. A profundidade do D455 nao
alcanca a mesa daqui (0,20-0,38 m, dentro da zona cega ~0,4 m), entao o
centro sai por RAY-CAST: o pixel central da mancha colorida e' projetado pela
camera ate cruzar o plano da BORDA do container, cuja cota o place publica em
`/manip/container_plane_z` (z em manip_base_link; sem ela usa o default).

Fluxo por frame (so' enquanto `/manip/place_active` for true, ou always_on):
  imagem -> HSV -> mascara por cor (vermelho = 2 faixas de matiz) -> morfologia
  -> contornos -> filtros (area, ROI do cubo na garra, solidez, proporcao,
  area esperada, dimensoes projetadas, alvo a frente do braco) -> minAreaRect
  -> ray-cast do centro e dos pontos medios dos lados -> (x, y, yaw) -> mediana
  de N amostras -> TF manip_base_link -> ct_vermelho | ct_azul (X = lado longo,
  stamp da imagem). Nunca republica estimativa velha: o place rejeita TF com
  mais de 1 s (max_tag_age_sec), e' essa a protecao contra fantasma.

Calibracao sem robo (mesma segmentacao do no, so' a parte de imagem):
  container_detector_node.py --image foto.jpg [--config x.yaml] [--out overlay.png]
                             [--probe u,v]
Tudo e' parametro ROS (config/container_detector.yaml), reajustavel ao vivo.
"""
import argparse
import collections
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


# ============================================================================
# Nucleo SEM ROS (importavel pelo pytest e pelo CLI offline)
# ============================================================================
@dataclass
class ColorSpec:
    name: str
    frame: str
    enabled: bool = True
    h_ranges: List[Tuple[int, int]] = field(default_factory=list)
    s_min: int = 90
    s_max: int = 255
    v_min: int = 60
    v_max: int = 255


DEFAULT_COLORS = {
    "vermelho": ColorSpec("vermelho", "ct_vermelho", True, [(0, 10), (170, 179)], 90, 255, 60, 255),
    "azul": ColorSpec("azul", "ct_azul", True, [(95, 130)], 90, 255, 50, 255),
}


@dataclass
class DetectorConfig:
    process_scale: float = 0.5
    min_area_px: float = 300.0
    min_solidity: float = 0.75
    min_aspect_ratio: float = 1.10
    max_aspect_ratio: float = 2.60     # container 1.62; em perspectiva ate' ~2.1 (medido); cubo de lado 4.4
    min_rectangularity: float = 0.70   # area / area do minAreaRect (container 0.83-0.91; triangulo 0.5)
    partial_min_area_frac: float = 0.25  # parcial: pelo menos 25% da area esperada (cubo na borda = 10-20%, filete = 5%)
    # Regioes da garra apagadas da mascara (fracao da imagem). Retangulos
    # (x0 y0 x1 y1) e poligonos ("x,y x,y x,y ..." — os dedos sao triangulares).
    exclusion_rects: List[Tuple[float, float, float, float]] = field(
        default_factory=lambda: [(0.30, 0.00, 0.62, 0.42)])
    exclusion_polys: List[List[Tuple[float, float]]] = field(
        default_factory=lambda: [list(p) for p in DEFAULT_GRIPPER_POLYS])
    exclusion_polys_holding: List[List[Tuple[float, float]]] = field(default_factory=list)
    # Retangulos (cubo carregado) so valem com a garra FECHADA (joint_states);
    # sem joint_states = assume fechada. Poligonos (dedos) valem sempre.
    cube_rect_only_when_holding: bool = True
    gripper_joint: str = "manip_joint6"
    gripper_holding_above: float = -0.12   # aberta = -0.2265, fechada = 0.170 (SRDF)
    border_margin_px: int = 3          # blob encostado na borda da imagem = objeto cortado
    accept_partial: bool = True        # cortado: usa o centroide da parte VISIVEL (sempre dentro da abertura)
    morph_open_px: int = 3
    morph_close_px: int = 5
    morph_close_iters: int = 2
    container_long_m: float = 0.170
    container_short_m: float = 0.105
    container_height_m: float = 0.070
    dims_tolerance_frac: float = 0.5
    area_expected_min_frac: float = 0.2
    area_expected_max_frac: float = 5.0
    min_target_y_m: float = 0.10
    max_target_radius_m: float = 0.60
    max_ray_length_m: float = 1.0
    wall_bias_correction: bool = True
    wall_bias_max_m: float = 0.02
    yaw_axis_offset_deg: float = 0.0
    colors: Dict[str, ColorSpec] = field(default_factory=lambda: dict(DEFAULT_COLORS))


@dataclass
class Blob:
    """Candidato a container numa imagem (coordenadas em pixels FULL-RES)."""
    center: Tuple[float, float]
    box: np.ndarray                       # 4x2
    long_mid: Tuple[Tuple[float, float], Tuple[float, float]]
    short_mid: Tuple[Tuple[float, float], Tuple[float, float]]
    area_px: float                        # em px full-res
    solidity: float
    aspect: float
    contour: np.ndarray                   # em px escalados (para desenhar)
    reason: Optional[str] = None          # None = aceito pelos filtros 2D
    partial: bool = False                 # encosta na borda da imagem: so' a parte visivel
    rectangularity: float = 0.0           # area / area do minAreaRect


def parse_h_ranges(values: Sequence[int]) -> List[Tuple[int, int]]:
    vals = [int(v) for v in values]
    if len(vals) % 2 != 0:
        raise ValueError(f"h_ranges precisa de pares [lo, hi, ...]: {vals}")
    return [(vals[i], vals[i + 1]) for i in range(0, len(vals), 2)]


# Dedos VERMELHOS da garra ABERTA, medidos na RealSense 640x480 em 25/08
# (convex hull do blob + 10 px de margem, normalizado). Recalibrar com
#   container_detector_node.py --fit-gripper foto_garra_sobre_mesa_branca.png
DEFAULT_GRIPPER_POLYS: List[List[Tuple[float, float]]] = [
    [(0.0, 0.0), (0.138, 0.0), (0.157, 0.094), (0.155, 0.304), (0.0, 0.225)],   # dedo esquerdo
    [(0.841, 0.0), (1.0, 0.0), (1.0, 0.140), (0.867, 0.277), (0.838, 0.279),
     (0.795, 0.113), (0.789, 0.023)],                                            # dedo direito
]


def parse_polys(values: Sequence[str]) -> List[List[Tuple[float, float]]]:
    """Cada item: "x,y x,y x,y ..." (>= 3 vertices, fracao da imagem)."""
    polys: List[List[Tuple[float, float]]] = []
    for item in values:
        txt = str(item).strip()
        if not txt:
            continue
        pts: List[Tuple[float, float]] = []
        for tok in txt.replace(";", " ").split():
            xy = tok.split(",")
            if len(xy) != 2:
                raise ValueError(f"exclusion_polys_norm: vertice invalido '{tok}' em '{txt}'")
            x, y = float(xy[0]), float(xy[1])
            if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
                raise ValueError(f"exclusion_polys_norm: vertice fora de [0,1]: '{tok}'")
            pts.append((x, y))
        if len(pts) < 3:
            raise ValueError(f"exclusion_polys_norm: poligono com menos de 3 vertices: '{txt}'")
        polys.append(pts)
    return polys


def exclusion_polygons_px(cfg: "DetectorConfig", w: int, h: int,
                          include_rects: bool = True) -> List[np.ndarray]:
    """Retangulos + poligonos da garra em pixels (int32, Nx1x2) para uma imagem w x h."""
    out: List[np.ndarray] = []
    for (x0, y0, x1, y1) in (cfg.exclusion_rects if include_rects else []):
        out.append(np.array([[x0 * w, y0 * h], [x1 * w, y0 * h], [x1 * w, y1 * h], [x0 * w, y1 * h]]))
    polys = list(cfg.exclusion_polys) + (list(cfg.exclusion_polys_holding) if include_rects else [])
    for poly in polys:
        out.append(np.array([[x * w, y * h] for (x, y) in poly]))
    return [np.round(p).astype(np.int32).reshape(-1, 1, 2) for p in out]


def parse_rects(values: Sequence[float]) -> List[Tuple[float, float, float, float]]:
    vals = [float(v) for v in values]
    if len(vals) % 4 != 0:
        raise ValueError(f"exclusion_rects_norm precisa de grupos de 4: {vals}")
    return [tuple(vals[i:i + 4]) for i in range(0, len(vals), 4)]  # type: ignore[misc]


def rect_axes(box: np.ndarray):
    """Pontos medios dos lados: retorna (long_mid(2 pts), short_mid(2 pts), long_len, short_len).

    long_mid = extremidades do EIXO longo (pontos medios dos lados curtos).
    """
    p = [np.asarray(box[i], dtype=float) for i in range(4)]
    e0 = np.linalg.norm(p[1] - p[0])
    e1 = np.linalg.norm(p[2] - p[1])
    m01 = (p[0] + p[1]) / 2.0
    m12 = (p[1] + p[2]) / 2.0
    m23 = (p[2] + p[3]) / 2.0
    m30 = (p[3] + p[0]) / 2.0
    if e0 >= e1:
        # lados 0-1 e 2-3 sao os longos -> eixo longo liga os medios dos curtos (1-2, 3-0)
        return (tuple(m12), tuple(m30)), (tuple(m01), tuple(m23)), e0, e1
    return (tuple(m01), tuple(m23)), (tuple(m12), tuple(m30)), e1, e0


class ContainerSegmenter:
    def __init__(self, cfg: DetectorConfig):
        self.cfg = cfg

    def build_mask(self, hsv: np.ndarray, spec: ColorSpec) -> np.ndarray:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in spec.h_ranges:
            part = cv2.inRange(
                hsv, (int(lo), int(spec.s_min), int(spec.v_min)),
                (int(hi), int(spec.s_max), int(spec.v_max)))
            mask = cv2.bitwise_or(mask, part)
        # Regioes fixas da garra (dedos VERMELHOS nos cantos superiores, cubo
        # carregado no centro-alto): apagadas ANTES da morfologia, senao o dedo
        # se funde com um container da mesma cor e puxa o centro (campo 25/08).
        mask[self.exclusion_mask(mask.shape[:2]) > 0] = 0
        k_open = max(1, int(self.cfg.morph_open_px))
        k_close = max(1, int(self.cfg.morph_close_px))
        if k_open > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((k_open, k_open), np.uint8))
        if k_close > 1:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, np.ones((k_close, k_close), np.uint8),
                iterations=max(1, int(self.cfg.morph_close_iters)))
        return mask

    def exclusion_mask(self, shape_hw) -> np.ndarray:
        """Mascara (255 = garra) no tamanho pedido; cacheada por tamanho/config."""
        h, w = int(shape_hw[0]), int(shape_hw[1])
        holding = bool(getattr(self, "holding", True))
        key = (h, w, id(self.cfg), holding)
        cached = getattr(self, "_excl_cache", None)
        if cached is not None and cached[0] == key:
            return cached[1]
        m = np.zeros((h, w), dtype=np.uint8)
        polys = exclusion_polygons_px(
            self.cfg, w, h, include_rects=(holding or not self.cfg.cube_rect_only_when_holding))
        for poly in polys:
            # um por vez: fillPoly com varios poligonos SOBREPOSTOS abre buraco na intersecao
            cv2.fillPoly(m, [poly], 255)
        self._excl_cache = (key, m)
        return m

    def _touches_exclusion(self, cnt: np.ndarray, shape_hw) -> bool:
        """Blob encosta (ate 3 px) numa regiao apagada -> parte pode estar sob a garra."""
        excl = self.exclusion_mask(shape_hw)
        if not np.any(excl):
            return False
        x, y, w, h = cv2.boundingRect(cnt)
        pad = 4
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(excl.shape[1], x + w + pad), min(excl.shape[0], y + h + pad)
        if not np.any(excl[y0:y1, x0:x1]):
            return False
        local = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
        cv2.drawContours(local, [cnt - np.array([[[x0, y0]]])], -1, 255, thickness=cv2.FILLED)
        local = cv2.dilate(local, np.ones((7, 7), np.uint8))
        return bool(np.any(cv2.bitwise_and(local, excl[y0:y1, x0:x1])))

    def find_blobs(self, mask: np.ndarray, scale: float, full_shape) -> List[Blob]:
        """Contornos externos -> candidatos ordenados por area (maior primeiro)."""
        full_h, full_w = full_shape[:2]
        inv = 1.0 / float(scale)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs: List[Blob] = []
        for cnt in contours:
            area_s = float(cv2.contourArea(cnt))
            if area_s <= 0.0:
                continue
            hull = cv2.convexHull(cnt)
            hull_area = float(cv2.contourArea(hull))
            solidity = area_s / hull_area if hull_area > 0 else 0.0
            rect = cv2.minAreaRect(cnt)
            (cx, cy), (rw, rh), _ang = rect
            box_s = cv2.boxPoints(rect)
            long_mid, short_mid, long_px, short_px = rect_axes(box_s)
            aspect = (long_px / short_px) if short_px > 1e-6 else 999.0
            rectangularity = area_s / max(float(rw) * float(rh), 1e-6)
            scale_pt = lambda pt: (float(pt[0]) * inv, float(pt[1]) * inv)  # noqa: E731
            blob = Blob(
                center=(cx * inv, cy * inv),
                box=np.asarray(box_s, dtype=float) * inv,
                long_mid=(scale_pt(long_mid[0]), scale_pt(long_mid[1])),
                short_mid=(scale_pt(short_mid[0]), scale_pt(short_mid[1])),
                area_px=area_s * inv * inv,
                solidity=solidity,
                aspect=aspect,
                contour=cnt,
                rectangularity=rectangularity,
            )
            if area_s < self.cfg.min_area_px:
                blob.reason = "area_min"
            else:
                bx, by, bw, bh = cv2.boundingRect(cnt)
                m = max(0, int(self.cfg.border_margin_px))
                sh, sw = mask.shape[:2]
                touches_border = bx <= m or by <= m or bx + bw >= sw - m or by + bh >= sh - m
                # Encosta numa regiao apagada (garra): tambem e' parcial.
                if not touches_border and self._touches_exclusion(cnt, mask.shape[:2]):
                    touches_border = True
                if touches_border and self.cfg.accept_partial:
                    # Parte visivel de uma abertura convexa: o centroide dos
                    # pixels cai DENTRO da abertura (pedido do operador 25/08:
                    # nunca soltar fora; perto da parede e' aceitavel).
                    mom = cv2.moments(cnt)
                    if mom["m00"] > 0:
                        blob.center = (mom["m10"] / mom["m00"] * inv, mom["m01"] / mom["m00"] * inv)
                    blob.partial = True
                if touches_border and not self.cfg.accept_partial:
                    blob.reason = "borda_imagem"
                elif not blob.partial and solidity < self.cfg.min_solidity:
                    blob.reason = "solidez"
                elif not blob.partial and aspect < self.cfg.min_aspect_ratio:
                    blob.reason = "proporcao"
                elif not blob.partial and aspect > self.cfg.max_aspect_ratio:
                    blob.reason = "proporcao_max"
                elif not blob.partial and rectangularity < self.cfg.min_rectangularity:
                    blob.reason = "retangularidade"
            blobs.append(blob)
        blobs.sort(key=lambda b: b.area_px, reverse=True)
        return blobs

    @staticmethod
    def hsv_stats(hsv: np.ndarray, mask: np.ndarray) -> Dict[str, Tuple[int, int, int]]:
        sel = hsv[mask > 0]
        if sel.size == 0:
            return {}
        out = {}
        for i, name in enumerate(("H", "S", "V")):
            ch = sel[:, i]
            out[name] = (int(np.percentile(ch, 5)), int(np.percentile(ch, 50)),
                         int(np.percentile(ch, 95)))
        return out


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@dataclass
class ContainerPose:
    x: float
    y: float
    yaw: float
    long_len: float
    short_len: float
    dist: float          # camera -> centro (m)
    area_ratio: float    # area do blob / area esperada
    partial: bool = False


class PlaneCaster:
    """Pixel -> reta pela camera -> intersecao com o plano z = plane_z (frame base)."""

    def __init__(self, K: np.ndarray, D: Sequence[float], cfg: DetectorConfig):
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        self.D = np.asarray(D, dtype=float).reshape(-1) if D is not None and len(D) else None
        self.cfg = cfg
        self.o: Optional[np.ndarray] = None   # origem da camera no frame base
        self.R: Optional[np.ndarray] = None   # rotacao camera -> base

    def set_camera_pose(self, origin: np.ndarray, rotation: np.ndarray) -> None:
        self.o = np.asarray(origin, dtype=float).reshape(3)
        self.R = np.asarray(rotation, dtype=float).reshape(3, 3)

    def ray(self, u: float, v: float) -> np.ndarray:
        pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
        if self.D is not None and np.any(self.D != 0.0):
            und = cv2.undistortPoints(pts, self.K, self.D)  # normalizado (x', y')
            xn, yn = float(und[0, 0, 0]), float(und[0, 0, 1])
        else:
            xn = (u - self.K[0, 2]) / self.K[0, 0]
            yn = (v - self.K[1, 2]) / self.K[1, 1]
        r = np.array([xn, yn, 1.0])
        return r / np.linalg.norm(r)

    def cast(self, u: float, v: float, plane_z: float) -> Optional[np.ndarray]:
        if self.o is None or self.R is None:
            return None
        r = self.R @ self.ray(u, v)
        if r[2] > -0.05:            # tem que olhar para baixo
            return None
        t = (plane_z - self.o[2]) / r[2]
        if t < 0.05 or t > self.cfg.max_ray_length_m:
            return None
        return self.o + t * r

    def container_pose(self, blob: Blob, plane_z: float) -> Tuple[Optional[ContainerPose], str]:
        """Devolve (pose, motivo_de_rejeicao). pose None quando rejeitado."""
        c = self.cast(blob.center[0], blob.center[1], plane_z)
        if c is None:
            return None, "sem_plano"
        l1 = self.cast(*blob.long_mid[0], plane_z)
        l2 = self.cast(*blob.long_mid[1], plane_z)
        s1 = self.cast(*blob.short_mid[0], plane_z)
        s2 = self.cast(*blob.short_mid[1], plane_z)
        if l1 is None or l2 is None or s1 is None or s2 is None:
            return None, "sem_plano"
        long_len = float(np.linalg.norm(l2 - l1))
        short_len = float(np.linalg.norm(s2 - s1))
        cfg = self.cfg
        dist = float(np.linalg.norm(c - self.o))
        fx, fy = self.K[0, 0], self.K[1, 1]
        area_exp = cfg.container_long_m * cfg.container_short_m * fx * fy / max(dist * dist, 1e-6)
        area_ratio = blob.area_px / area_exp if area_exp > 0 else 0.0
        tol = cfg.dims_tolerance_frac
        if area_ratio > cfg.area_expected_max_frac:
            return None, f"area_esperada({area_ratio:.2f}x)"
        if long_len > cfg.container_long_m * (1 + tol):
            return None, f"lado_longo({long_len:.3f}m)"
        if short_len > cfg.container_short_m * (1 + tol):
            return None, f"lado_curto({short_len:.3f}m)"
        if blob.partial and area_ratio < cfg.partial_min_area_frac:
            # Pedaco pequeno demais (filete de dedo/cubo na borda de uma mascara).
            return None, f"area_parcial({area_ratio:.2f}x)"
        if not blob.partial:
            # Inteiro: tambem exige tamanho MINIMO (parcial pode ser qualquer pedaco).
            if area_ratio < cfg.area_expected_min_frac:
                return None, f"area_esperada({area_ratio:.2f}x)"
            if long_len < cfg.container_long_m * (1 - tol):
                return None, f"lado_longo({long_len:.3f}m)"
            if short_len < cfg.container_short_m * (1 - tol):
                return None, f"lado_curto({short_len:.3f}m)"
        x, y = float(c[0]), float(c[1])
        if cfg.wall_bias_correction and not blob.partial:
            # A parede de fora voltada para a camera entra na mancha e puxa o
            # centro na direcao da camera: empurra de volta (ate wall_bias_max_m).
            corners = [self.cast(float(p[0]), float(p[1]), plane_z) for p in blob.box]
            corners = [k for k in corners if k is not None]
            if corners:
                foot = self.o[:2]
                r_near = min(float(np.linalg.norm(k[:2] - foot)) for k in corners)
                h = float(self.o[2] - plane_z)
                if h > 0.02:
                    delta = min(cfg.wall_bias_max_m, r_near * cfg.container_height_m / (2.0 * h))
                    d = np.array([x, y]) - foot
                    nrm = float(np.linalg.norm(d))
                    if nrm > 1e-6:
                        x += delta * d[0] / nrm
                        y += delta * d[1] / nrm
        if y < cfg.min_target_y_m:
            return None, f"atras_do_braco(y={y:.2f})"
        if math.hypot(x, y) > cfg.max_target_radius_m:
            return None, f"longe(r={math.hypot(x, y):.2f})"
        yaw = math.atan2(l2[1] - l1[1], l2[0] - l1[0]) + math.radians(cfg.yaw_axis_offset_deg)
        yaw = wrap_half_pi(yaw)
        return ContainerPose(x, y, yaw, long_len, short_len, dist, area_ratio, blob.partial), ""


def wrap_half_pi(yaw: float) -> float:
    """Eixo sem sentido (linha): reduz a (-pi/2, pi/2]."""
    while yaw <= -math.pi / 2.0:
        yaw += math.pi
    while yaw > math.pi / 2.0:
        yaw -= math.pi
    return yaw


class PoseSmoother:
    def __init__(self, n: int = 5, window_s: float = 2.0, jump_reset_m: float = 0.04,
                 min_samples: int = 2):
        self.n = max(1, int(n))
        self.window_s = float(window_s)
        self.jump_reset_m = float(jump_reset_m)
        self.min_samples = max(1, int(min_samples))
        self.samples: collections.deque = collections.deque()

    def reset(self) -> None:
        self.samples.clear()

    def push(self, t: float, x: float, y: float, yaw: float) -> None:
        if self.samples:
            est = self.estimate(t, force=True)
            if est is not None and math.hypot(x - est[0], y - est[1]) > self.jump_reset_m:
                self.samples.clear()
        self.samples.append((t, x, y, yaw))
        while len(self.samples) > self.n:
            self.samples.popleft()
        while self.samples and t - self.samples[0][0] > self.window_s:
            self.samples.popleft()

    def estimate(self, now: float, force: bool = False):
        while self.samples and now - self.samples[0][0] > self.window_s:
            self.samples.popleft()
        if not self.samples or (not force and len(self.samples) < self.min_samples):
            return None
        xs = sorted(s[1] for s in self.samples)
        ys = sorted(s[2] for s in self.samples)
        ref = self.samples[-1][3]
        yaws = []
        for s in self.samples:
            y = s[3]
            while y - ref > math.pi / 2.0:
                y -= math.pi
            while y - ref < -math.pi / 2.0:
                y += math.pi
            yaws.append(y)
        yaws.sort()
        m = len(xs) // 2
        return xs[m], ys[m], wrap_half_pi(yaws[m]), len(self.samples)


def draw_overlay(bgr_full: np.ndarray, scale: float, per_color, cfg: DetectorConfig,
                 status_lines: List[str], holding: bool = True) -> np.ndarray:
    out = bgr_full.copy()
    h, w = out.shape[:2]
    for poly in exclusion_polygons_px(cfg, w, h, include_rects=(holding or not cfg.cube_rect_only_when_holding)):
        cv2.polylines(out, [poly], True, (0, 255, 255), 1)
    inv = 1.0 / scale
    for name, blobs in per_color.items():
        for i, b in enumerate(blobs):
            ok = b.reason is None
            color = (0, 255, 0) if ok and i == 0 else (0, 0, 255)
            cnt = (b.contour.astype(float) * inv).astype(np.int32)
            cv2.drawContours(out, [cnt], -1, color, 1)
            cv2.polylines(out, [b.box.astype(np.int32)], True, color, 1)
            cx, cy = int(b.center[0]), int(b.center[1])
            cv2.drawMarker(out, (cx, cy), color, cv2.MARKER_CROSS, 12, 1)
            l1, l2 = b.long_mid
            cv2.line(out, (int(l1[0]), int(l1[1])), (int(l2[0]), int(l2[1])), color, 1)
            label = f"{name}" + (" parcial" if b.partial else "") + ("" if ok else f" x {b.reason}")
            cv2.putText(out, label, (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    y = 16
    for line in status_lines:
        cv2.putText(out, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        y += 16
    return out


def segment_image(bgr_full: np.ndarray, cfg: DetectorConfig, holding: bool = True):
    """Parte de imagem do pipeline: devolve (per_color blobs, hsv_small, masks).
    holding=False (garra aberta/vazia) desliga os retangulos do cubo carregado."""
    scale = float(cfg.process_scale)
    small = cv2.resize(bgr_full, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) \
        if scale != 1.0 else bgr_full
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    seg = ContainerSegmenter(cfg)
    seg.holding = holding
    per_color = {}
    masks = {}
    for name, spec in cfg.colors.items():
        if not spec.enabled:
            continue
        mask = seg.build_mask(hsv, spec)
        masks[name] = mask
        per_color[name] = seg.find_blobs(mask, scale, bgr_full.shape)
    return per_color, hsv, masks


def load_config_yaml(path: str) -> DetectorConfig:
    import yaml  # local: o nucleo nao depende dele
    with open(path, "r", encoding="utf-8") as handle:
        root = yaml.safe_load(handle) or {}
    params = root.get("container_detector", {}).get("ros__parameters", root)
    return config_from_dict(params)


def _coerce(kind, value):
    """bool/int/float robustos a string ("false", "3.0") — dynamic_typing deixa passar."""
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


def config_from_dict(p: dict) -> DetectorConfig:
    cfg = DetectorConfig()
    simple = ["process_scale", "min_area_px", "min_solidity", "min_aspect_ratio",
              "max_aspect_ratio", "min_rectangularity", "partial_min_area_frac",
              "border_margin_px", "accept_partial",
              "cube_rect_only_when_holding", "gripper_joint", "gripper_holding_above",
              "morph_open_px", "morph_close_px", "morph_close_iters",
              "container_long_m", "container_short_m", "container_height_m",
              "dims_tolerance_frac", "area_expected_min_frac", "area_expected_max_frac",
              "min_target_y_m", "max_target_radius_m", "max_ray_length_m",
              "wall_bias_correction", "wall_bias_max_m", "yaw_axis_offset_deg"]
    for k in simple:
        if k in p:
            setattr(cfg, k, _coerce(type(getattr(cfg, k)), p[k]))
    if "exclusion_rects_norm" in p:
        cfg.exclusion_rects = parse_rects(p["exclusion_rects_norm"])
    if "exclusion_polys_norm" in p:
        cfg.exclusion_polys = parse_polys(p["exclusion_polys_norm"])
    if "exclusion_polys_holding_norm" in p:
        cfg.exclusion_polys_holding = parse_polys(p["exclusion_polys_holding_norm"])
    names = p.get("colors", list(DEFAULT_COLORS.keys()))
    colors = {}
    for name in names:
        base = DEFAULT_COLORS.get(name, ColorSpec(name, f"ct_{name}", True, [(0, 179)]))
        sub = p.get(name, {}) or {}
        colors[name] = ColorSpec(
            name=name,
            frame=str(sub.get("frame", base.frame)),
            enabled=_coerce(bool, sub.get("enabled", base.enabled)),
            h_ranges=parse_h_ranges(sub["h_ranges"]) if "h_ranges" in sub else list(base.h_ranges),
            s_min=_coerce(int, sub.get("s_min", base.s_min)),
            s_max=_coerce(int, sub.get("s_max", base.s_max)),
            v_min=_coerce(int, sub.get("v_min", base.v_min)),
            v_max=_coerce(int, sub.get("v_max", base.v_max)))
    cfg.colors = colors
    return cfg


# ============================================================================
# CLI offline (calibracao com fotos)
# ============================================================================
def fit_gripper_polys(bgr: np.ndarray, cfg: DetectorConfig, margin_px: int = 10,
                      color: str = "vermelho") -> List[Tuple[str, bool]]:
    """Poligonos (normalizados, com margem) dos blobs da cor da garra numa foto
    tirada com a garra sobre fundo neutro (mesa branca, SEM container dessa cor).
    Retorna [(texto para exclusion_polys_norm, encosta_no_topo)]."""
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    spec = cfg.colors[color]
    mask = np.zeros((h, w), dtype=np.uint8)
    for lo, hi in spec.h_ranges:
        mask |= cv2.inRange(hsv, (int(lo), int(spec.s_min), int(spec.v_min)),
                            (int(hi), int(spec.s_max), int(spec.v_max)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out: List[Tuple[str, bool]] = []
    k = 2 * max(0, int(margin_px)) + 1
    for cnt in sorted(contours, key=cv2.contourArea, reverse=True):
        if cv2.contourArea(cnt) < 200:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        m = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(m, [cv2.convexHull(cnt)], -1, 255, thickness=cv2.FILLED)
        if k > 1:
            m = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hull = cv2.convexHull(max(cs, key=cv2.contourArea))
        poly = cv2.approxPolyDP(hull, 3.0, True).reshape(-1, 2)
        txt = " ".join(f"{min(1.0, max(0.0, px / w)):.3f},{min(1.0, max(0.0, py / h)):.3f}"
                       for px, py in poly)
        out.append((txt, bool(poly[:, 1].min() <= 1)))
    return out


def run_offline(args) -> int:
    cfg = load_config_yaml(args.config) if args.config else DetectorConfig()
    bgr = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if bgr is None:
        print(f"nao consegui abrir {args.image}", file=sys.stderr)
        return 2
    if getattr(args, "fit_gripper", False):
        print("# Poligonos da garra (cole em exclusion_polys_norm; 'topo' = encosta na borda de cima):")
        for txt, top in fit_gripper_polys(bgr, cfg, args.margin_px, color=args.fit_color):
            print(f'      - "{txt}"   # {"topo" if top else "solto"}')
        return 0
    h, w = bgr.shape[:2]
    print(f"imagem {w}x{h}, process_scale {cfg.process_scale}")
    t0 = time.perf_counter()
    holding = not getattr(args, "gripper_open", False)
    per_color, hsv, masks = segment_image(bgr, cfg, holding=holding)
    dt_ms = (time.perf_counter() - t0) * 1000.0
    print("garra: " + ("SEGURANDO cubo (retangulo do cubo ativo)" if holding else "vazia (--gripper-open)"))
    if args.probe:
        u, v = [int(x) for x in args.probe.split(",")]
        hsv_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        print(f"probe ({u},{v}): HSV = {tuple(int(x) for x in hsv_full[v, u])}  (H 0-179)")
    # picos de matiz da imagem inteira (ajuda a achar as faixas)
    hsv_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    sat = hsv_full[:, :, 1] > 80
    if np.any(sat):
        hist = np.bincount(hsv_full[:, :, 0][sat].ravel(), minlength=180)
        peaks = np.argsort(hist)[::-1][:6]
        print("picos de matiz (pixels saturados):", ", ".join(f"H={int(p)}:{int(hist[p])}" for p in peaks))
    status = [f"{dt_ms:.1f} ms"]
    for name, blobs in per_color.items():
        spec = cfg.colors[name]
        stats = ContainerSegmenter.hsv_stats(hsv, masks[name])
        print(f"\n[{name}] faixas H {spec.h_ranges} S>={spec.s_min} V>={spec.v_min}: "
              f"{len(blobs)} blob(s); HSV p5/p50/p95 na mascara: {stats}")
        for i, b in enumerate(blobs[:6]):
            print(f"  #{i}: centro=({b.center[0]:.0f},{b.center[1]:.0f}) area={b.area_px:.0f}px "
                  f"solidez={b.solidity:.2f} proporcao={b.aspect:.2f} retang={b.rectangularity:.2f} "
                  f"{'ACEITO (2D)' if b.reason is None else 'rejeitado: ' + b.reason}")
        ok = [b for b in blobs if b.reason is None]
        status.append(f"{name}: {'ok' if ok else 'nao visto'}")
    if args.out:
        cv2.imwrite(args.out, draw_overlay(bgr, cfg.process_scale, per_color, cfg, status, holding))
        print(f"\noverlay salvo em {args.out}")
    return 0


# ============================================================================
# No ROS
# ============================================================================
def ros_main(argv=None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
    from rcl_interfaces.msg import SetParametersResult
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_msgs.msg import Bool, Float64, String
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from tf2_ros import Buffer, TransformListener, TransformBroadcaster
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    from cv_bridge import CvBridge
    from rclpy.time import Time
    from rclpy.duration import Duration

    class ContainerDetector(Node):
        def __init__(self):
            super().__init__("container_detector")
            # --- parametros ---
            # dynamic_typing: um operador escrevendo `300` em vez de `300.0` no
            # yaml nao pode derrubar o no na arena.
            from rcl_interfaces.msg import ParameterDescriptor
            _dyn = ParameterDescriptor(dynamic_typing=True)

            def dp(name, default):
                return self.declare_parameter(name, default, _dyn)
            dp("rate_hz", 3.0)
            dp("always_on", False)
            dp("process_scale", 0.5)
            dp("image_topic", "/camera/camera/color/image_raw")
            dp("camera_info_topic", "/camera/camera/color/camera_info")
            dp("place_active_topic", "/manip/place_active")
            dp("plane_z_topic", "/manip/container_plane_z")
            dp("debug_image", True)
            dp("base_frame", "manip_base_link")
            dp("camera_frame", "")
            dp("plane_z_default", 0.041)
            dp("tf_timeout_s", 0.0)
            dp("reject_when_camera_moving", True)
            dp("camera_motion_max_m", 0.005)
            dp("camera_motion_max_deg", 1.0)
            dp("container_long_m", 0.170)
            dp("container_short_m", 0.105)
            dp("container_height_m", 0.070)
            dp("dims_tolerance_frac", 0.5)
            dp("area_expected_min_frac", 0.2)
            dp("area_expected_max_frac", 5.0)
            dp("min_area_px", 300.0)
            dp("min_solidity", 0.75)
            dp("min_aspect_ratio", 1.10)
            dp("max_aspect_ratio", 2.60)
            dp("min_rectangularity", 0.70)
            dp("partial_min_area_frac", 0.25)
            dp("wall_bias_correction", True)
            dp("wall_bias_max_m", 0.02)
            dp("exclusion_rects_norm", [0.30, 0.00, 0.62, 0.42])
            dp("exclusion_polys_holding_norm", [""])
            dp("exclusion_polys_norm", [" ".join(f"{x:.3f},{y:.3f}" for x, y in poly)
                                        for poly in DEFAULT_GRIPPER_POLYS])
            dp("border_margin_px", 3)
            dp("accept_partial", True)
            dp("cube_rect_only_when_holding", True)
            dp("gripper_joint", "manip_joint6")
            dp("gripper_holding_above", -0.12)
            dp("joint_states_topic", "/joint_states")
            dp("min_target_y_m", 0.10)
            dp("max_target_radius_m", 0.60)
            dp("max_ray_length_m", 1.0)
            dp("morph_open_px", 3)
            dp("morph_close_px", 5)
            dp("morph_close_iters", 2)
            dp("smoothing_n", 5)
            dp("smoothing_window_s", 2.0)
            dp("smoothing_jump_reset_m", 0.04)
            dp("min_samples", 2)
            dp("yaw_axis_offset_deg", 0.0)
            dp("colors", list(DEFAULT_COLORS.keys()))
            for name in self.get_parameter("colors").value:
                base = DEFAULT_COLORS.get(name, ColorSpec(name, f"ct_{name}", True, [(0, 179)]))
                dp(f"{name}.enabled", base.enabled)
                dp(f"{name}.frame", base.frame)
                dp(f"{name}.h_ranges", [v for pair in base.h_ranges for v in pair])
                dp(f"{name}.s_min", base.s_min)
                dp(f"{name}.s_max", base.s_max)
                dp(f"{name}.v_min", base.v_min)
                dp(f"{name}.v_max", base.v_max)
            try:
                self.cfg = self._build_config()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"parametros invalidos no arranque ({exc}) — usando defaults internos.")
                self.cfg = DetectorConfig()
            # Validacao ANTES de aceitar (senao 'ros2 param set' diz sucesso e a
            # config fica congelada); aplicacao depois que o rclpy gravar.
            self.add_on_set_parameters_callback(self._on_params)
            if hasattr(self, "add_post_set_parameters_callback"):
                self.add_post_set_parameters_callback(self._on_params_applied)
            self._pending_cfg: Optional[DetectorConfig] = None

            self.bridge = CvBridge()
            self.caster: Optional[PlaneCaster] = None
            self.camera_frame_from_info = ""
            self.plane_z: Optional[float] = None
            self.smoothers: Dict[str, PoseSmoother] = {}
            self._new_smoothers()
            self.last_processed = 0.0
            self.last_cam_pose = None
            self.image_sub = None
            self.active = False
            self.place_active = False
            self.warned_no_plane = False
            self.warned_no_info = False
            self.warned_tf = 0.0

            # spin_thread=False + timeout 0: com rclpy.spin(node) o listener nao
            # ganha thread propria (o no so pode estar num executor) e um
            # lookup com timeout bloquearia o unico executor sem processar /tf.
            # Sem espera: tenta no stamp da imagem, senao usa o ultimo TF (braco
            # parado durante a deteccao — diferenca desprezivel).
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.tf_broadcaster = TransformBroadcaster(self)

            latched = QoSProfile(
                depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                CameraInfo, self.get_parameter("camera_info_topic").value,
                self._on_camera_info, QoSProfile(
                    depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST))
            self.create_subscription(
                Bool, self.get_parameter("place_active_topic").value, self._on_place_active, latched)
            self.create_subscription(
                Float64, self.get_parameter("plane_z_topic").value, self._on_plane_z, latched)
            self.gripper_holding = True          # sem joint_states: assume segurando (seguro)
            self.gripper_pos = None
            self.create_subscription(
                JointState, self.get_parameter("joint_states_topic").value, self._on_joint_states, 10)
            self.status_pub = self.create_publisher(String, "/manip/container_detector/status", 5)
            self.pose_pubs = {
                name: self.create_publisher(PoseStamped, f"/manip/container_detector/pose_{name}", 5)
                for name in self.cfg.colors
            }
            self.debug_pub = self.create_publisher(Image, "/manip/container_detector/debug_image", 1)
            self.get_logger().info(
                "container_detector pronto: cores %s; ativo %s; plano default %.3f."
                % (list(self.cfg.colors.keys()),
                   "sempre" if self.get_parameter("always_on").value else "so durante o place",
                   self.get_parameter("plane_z_default").value))
            self._update_active()

        # ---------------- parametros ----------------
        def _build_config(self) -> DetectorConfig:
            return config_from_dict(self._config_dict())

        def _new_smoothers(self):
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            self.smoothers = {
                name: PoseSmoother(g("smoothing_n"), g("smoothing_window_s"),
                                   g("smoothing_jump_reset_m"), g("min_samples"))
                for name in self.cfg.colors
            }

        def _config_dict(self, overrides=None) -> dict:
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            keys = ["process_scale", "min_area_px", "min_solidity", "min_aspect_ratio",
                    "max_aspect_ratio", "min_rectangularity", "partial_min_area_frac",
                    "border_margin_px", "accept_partial",
                    "cube_rect_only_when_holding", "gripper_joint", "gripper_holding_above",
                    "morph_open_px", "morph_close_px", "morph_close_iters",
                    "container_long_m", "container_short_m", "container_height_m",
                    "dims_tolerance_frac", "area_expected_min_frac", "area_expected_max_frac",
                    "min_target_y_m", "max_target_radius_m", "max_ray_length_m",
                    "wall_bias_correction", "wall_bias_max_m", "yaw_axis_offset_deg",
                    "exclusion_rects_norm", "exclusion_polys_norm", "exclusion_polys_holding_norm", "colors"]
            p = {k: g(k) for k in keys}
            ov = overrides or {}
            for k in keys:
                if k in ov:
                    p[k] = ov[k]
            for name in p["colors"]:
                sub = {}
                for k in ("enabled", "frame", "h_ranges", "s_min", "s_max", "v_min", "v_max"):
                    full = f"{name}.{k}"
                    if full in ov:
                        sub[k] = ov[full]
                    elif self.has_parameter(full):
                        sub[k] = g(full)
                p[name] = sub
            return p

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

        def _apply_cfg(self, cfg: DetectorConfig):
            self.cfg = cfg
            if self.caster is not None:
                self.caster.cfg = cfg
            self._new_smoothers()
            self.get_logger().info("parametros recarregados.")

        # ---------------- ativacao ----------------
        def _on_place_active(self, msg: Bool):
            self.place_active = bool(msg.data)
            self._update_active()

        def _update_active(self):
            want = self.place_active or bool(self.get_parameter("always_on").value)
            if want and self.image_sub is None:
                self.image_sub = self.create_subscription(
                    Image, self.get_parameter("image_topic").value, self._on_image,
                    QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                               history=QoSHistoryPolicy.KEEP_LAST))
                for sm in self.smoothers.values():
                    sm.reset()
                self.warned_no_plane = False
                self.get_logger().info("ativo: assinando a imagem.")
            elif not want and self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
                self.get_logger().info("inativo: imagem desassinada (0%% CPU).")
            self.active = want

        # ---------------- entradas ----------------
        def _on_camera_info(self, msg: CameraInfo):
            if self.caster is None or self.caster.K[0, 0] != msg.k[0]:
                self.caster = PlaneCaster(np.array(msg.k), list(msg.d), self.cfg)
                self.camera_frame_from_info = msg.header.frame_id
                self.get_logger().info(
                    f"camera_info: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} cx={msg.k[2]:.1f} "
                    f"cy={msg.k[5]:.1f} frame={msg.header.frame_id} D={list(msg.d)[:5]}")
            self.caster.cfg = self.cfg

        def _on_plane_z(self, msg: Float64):
            changed = self.plane_z is None or abs(float(msg.data) - self.plane_z) > 0.005
            self.plane_z = float(msg.data)
            if changed:
                for sm in self.smoothers.values():
                    sm.reset()
            self.get_logger().info(f"plano da borda recebido: z={self.plane_z:.3f} (manip_base_link)")

        def _camera_pose(self, stamp):
            base = self.get_parameter("base_frame").value
            cam = self.get_parameter("camera_frame").value or self.camera_frame_from_info \
                or "camera_color_optical_frame"
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
            stamp = Time.from_msg(msg.header.stamp)
            t_img = stamp.nanoseconds * 1e-9
            rate = float(self.get_parameter("rate_hz").value)
            now_mono = time.monotonic()
            if rate > 0 and now_mono - self.last_processed < 1.0 / rate:
                return
            self.last_processed = now_mono
            try:
                self._process(msg, stamp, t_img)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"erro no processamento: {exc!r}")

        def _on_joint_states(self, msg: JointState):
            try:
                idx = list(msg.name).index(self.cfg.gripper_joint)
            except ValueError:
                return
            if idx < len(msg.position):
                self.gripper_pos = float(msg.position[idx])
                self.gripper_holding = self.gripper_pos > float(self.cfg.gripper_holding_above)

        def _process(self, msg: Image, stamp, t_img: float):
            t0 = time.perf_counter()
            status = []
            if self.caster is None:
                if not self.warned_no_info:
                    self.warned_no_info = True
                    self.get_logger().warn("sem camera_info ainda — nao detecto.")
                self._publish_status("sem camera_info")
                return
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
                    for sm in self.smoothers.values():
                        sm.reset()
                    self._publish_status(f"camera movendo (d={dpos*100:.1f}cm, {dang:.1f}deg)")
                    return
            self.last_cam_pose = (origin, rot)
            self.caster.set_camera_pose(origin, rot)

            plane_z = self.plane_z
            plane_src = "topic"
            if plane_z is None:
                plane_z = float(self.get_parameter("plane_z_default").value)
                plane_src = "default"
                if not self.warned_no_plane:
                    self.warned_no_plane = True
                    self.get_logger().warn(
                        f"sem /manip/container_plane_z — usando plane_z_default={plane_z:.3f}")

            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            holding = bool(self.gripper_holding)
            per_color, hsv, masks = segment_image(bgr, self.cfg, holding=holding)
            status.append("garra: " + ("segurando" if holding else "vazia")
                          + (f" (j6={self.gripper_pos:+.3f})" if self.gripper_pos is not None else " (sem joint_states)"))
            now_wall = time.time()
            base = self.get_parameter("base_frame").value
            for name, blobs in per_color.items():
                spec = self.cfg.colors[name]
                accepted = [b for b in blobs if b.reason is None]
                if len(accepted) >= 2 and accepted[1].area_px > 0.5 * accepted[0].area_px:
                    self.get_logger().warn(
                        f"{name}: {len(accepted)} blobs parecidos, usando o maior", throttle_duration_sec=5.0)
                pose = None
                reason = "nao visto" if not blobs else \
                    ", ".join(sorted({b.reason for b in blobs if b.reason}))[:60]
                for b in accepted:
                    pose, why = self.caster.container_pose(b, plane_z)
                    if pose is not None:
                        break
                    b.reason = why
                    reason = why
                sm = self.smoothers[name]
                if pose is None:
                    status.append(f"{name}: {reason or 'nao visto'}")
                    continue
                sm.push(t_img, pose.x, pose.y, pose.yaw)
                est = sm.estimate(t_img)
                if est is None:
                    status.append(f"{name}: acumulando ({len(sm.samples)})")
                    continue
                x, y, yaw, n = est
                self._publish_tf(base, spec.frame, x, y, plane_z, yaw, msg.header.stamp)
                ps = PoseStamped()
                ps.header.stamp = msg.header.stamp
                ps.header.frame_id = base
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, plane_z
                ps.pose.orientation.z = math.sin(yaw / 2.0)
                ps.pose.orientation.w = math.cos(yaw / 2.0)
                self.pose_pubs[name].publish(ps)
                status.append(
                    f"{name}{' PARCIAL' if pose.partial else ''}: x={x:.3f} y={y:.3f} "
                    f"yaw={math.degrees(yaw):.0f}deg n={n} area={pose.area_ratio:.2f}xA_esp "
                    f"lados={pose.long_len:.3f}/{pose.short_len:.3f}")
            dt_ms = (time.perf_counter() - t0) * 1000.0
            lat = now_wall - t_img
            if lat > 0.5:
                self.get_logger().warn(
                    f"imagem com {lat:.2f}s de atraso (latencia/relogio)", throttle_duration_sec=10.0)
            status.append(f"plane_z={plane_z:.3f}({plane_src}) tf={'latest' if used_latest else 'stamp'} {dt_ms:.1f}ms")
            self._publish_status(" | ".join(status))
            if bool(self.get_parameter("debug_image").value) and self.debug_pub.get_subscription_count() > 0:
                overlay = draw_overlay(bgr, self.cfg.process_scale, per_color, self.cfg, status, holding)
                out = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                out.header = msg.header
                self.debug_pub.publish(out)

        def _publish_tf(self, base, child, x, y, z, yaw, stamp):
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = base
            tf.child_frame_id = child
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = float(z)
            tf.transform.rotation.z = math.sin(yaw / 2.0)
            tf.transform.rotation.w = math.cos(yaw / 2.0)
            self.tf_broadcaster.sendTransform(tf)

        def _publish_status(self, text: str):
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=argv)
    node = ContainerDetector()
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
        parser = argparse.ArgumentParser(description="Calibracao offline do detector de containers.")
        parser.add_argument("--image", required=True)
        parser.add_argument("--config", default=None, help="container_detector.yaml (default: valores internos)")
        parser.add_argument("--out", default=None, help="salva overlay PNG")
        parser.add_argument("--probe", default=None, help="u,v: imprime o HSV desse pixel")
        parser.add_argument("--fit-gripper", action="store_true",
                            help="imprime os poligonos (garra sobre fundo neutro) para exclusion_polys_norm")
        parser.add_argument("--margin-px", type=int, default=10, help="margem dos poligonos (--fit-gripper)")
        parser.add_argument("--fit-color", default="vermelho", help="cor a contornar em --fit-gripper (vermelho|azul)")
        parser.add_argument("--gripper-open", action="store_true",
                            help="foto com a garra vazia: nao apaga o retangulo do cubo carregado")
        return run_offline(parser.parse_args(argv))
    return ros_main(argv)


if __name__ == "__main__":
    sys.exit(main())
