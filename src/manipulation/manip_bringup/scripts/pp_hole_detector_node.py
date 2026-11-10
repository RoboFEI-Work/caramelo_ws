#!/usr/bin/env python3
"""Detector do BURACO (cavidade) da mesa de precision placement por COR.

Por que existe (2026-11-06, pedido do operador): o tampo da PP e' BRANCO e a
cavidade onde o cubo encaixa aparece MARROM (fundo de madeira). Hoje o alvo
do slot sai da tag adesiva no tampo + offsets fixos (slot_place_*_offset_m /
slot_center_offset_m), que dependem de como a tag foi colada. A mudanca de
cor branco->marrom da' o CENTRO REAL da cavidade: este no' acha as manchas
marrons cercadas de branco e publica os centros em manip_base_link; o place
corrige o alvo xy do slot pelo buraco mais proximo (vale para o place normal
de PP e para a AMT1 — a correcao fica no mtc_place_action_node).

Mesma receita do container_detector (D455 do punho: profundidade cega a
< 0,4 m, entao RAY-CAST do pixel no plano do TAMPO, cuja cota o place publica
em /manip/pp_plane_z; sem ela usa plane_z_default):
  imagem -> HSV -> mascara MARROM -> morfologia -> contornos -> filtros
  (area, solidez, proporcao ~quadrada, lado esperado, ANEL BRANCO em volta)
  -> ray-cast do centro -> rastreio por proximidade + mediana por buraco ->
  PoseArray /manip/pp_hole_detector/holes (+ TFs pp_buraco_<i> para o RViz).

O anel branco e' o filtro que importa: so' aceita mancha marrom CERCADA de
tampo branco — o chao da arena (tambem marrom) e sombras fora da mesa nao
passam. Buraco cortado pela borda da imagem/garra e' rejeitado por padrao
(centro da parte visivel nao e' o centro da cavidade).

Calibracao sem robo (so' a parte de imagem, mesma segmentacao do no'):
  pp_hole_detector_node.py --image foto.jpg [--config x.yaml] [--out o.png]
                           [--probe u,v]
Tudo e' parametro ROS (config/pp_hole_detector.yaml), reajustavel ao vivo.
"""
import argparse
import collections
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# Nucleo compartilhado com o detector de containers (instalados lado a lado
# em lib/manip_bringup: o diretorio do proprio script esta' no sys.path).
from container_detector_node import (  # noqa: E402
    DEFAULT_GRIPPER_POLYS,
    PlaneCaster,
    _coerce,
    exclusion_polygons_px,
    parse_h_ranges,
    parse_polys,
    parse_rects,
    quat_to_rot,
    rect_axes,
)


# ============================================================================
# Nucleo SEM ROS (importavel pelo pytest e pelo CLI offline)
# ============================================================================
@dataclass
class HoleConfig:
    process_scale: float = 0.5
    # Cor do buraco (fundo de madeira/MDF visto pela RealSense). H 0-179.
    h_ranges: List[Tuple[int, int]] = field(default_factory=lambda: [(5, 25)])
    s_min: int = 50
    s_max: int = 255
    v_min: int = 40
    v_max: int = 230
    # Anel BRANCO em volta da mancha: fracao minima de pixels "tampo branco"
    # (S baixo, V alto) num anel de ring_px alem do contorno. A tag adesiva
    # ao lado do buraco come parte do anel — por isso 0.45 e nao 0.8.
    ring_enabled: bool = True
    ring_px: int = 6                    # espessura do anel em px ESCALADOS
    ring_white_min_frac: float = 0.45
    white_s_max: int = 70
    white_v_min: int = 140
    # Forma (cavidade ~quadrada vista de cima)
    min_area_px: float = 150.0          # em px escalados
    min_solidity: float = 0.85
    max_aspect_ratio: float = 1.8
    min_rectangularity: float = 0.60
    border_margin_px: int = 3
    accept_partial: bool = False        # cortado: centro visivel != centro da cavidade
    morph_open_px: int = 3
    morph_close_px: int = 5
    morph_close_iters: int = 1
    # Geometria 3D (lado nominal da cavidade; filtros frouxos)
    hole_side_m: float = 0.05
    side_tolerance_frac: float = 0.5    # lado em [nominal*(1-tol), nominal*(1+tol)]
    area_expected_min_frac: float = 0.3
    area_expected_max_frac: float = 4.0
    min_target_y_m: float = 0.10
    max_target_radius_m: float = 0.60
    max_ray_length_m: float = 1.0       # exigido pelo PlaneCaster
    # Garra na imagem — MESMOS nomes do container (reusa exclusion_polygons_px)
    exclusion_rects: List[Tuple[float, float, float, float]] = field(
        default_factory=lambda: [(0.30, 0.00, 0.62, 0.42)])
    exclusion_polys: List[List[Tuple[float, float]]] = field(
        default_factory=lambda: [list(p) for p in DEFAULT_GRIPPER_POLYS])
    exclusion_polys_holding: List[List[Tuple[float, float]]] = field(default_factory=list)
    cube_rect_only_when_holding: bool = True
    gripper_joint: str = "manip_joint6"
    gripper_holding_above: float = -0.12


@dataclass
class HoleBlob:
    """Candidato a cavidade numa imagem (coordenadas em pixels FULL-RES)."""
    center: Tuple[float, float]
    box: np.ndarray                     # 4x2 full-res
    long_mid: Tuple[Tuple[float, float], Tuple[float, float]]
    short_mid: Tuple[Tuple[float, float], Tuple[float, float]]
    area_px: float                      # full-res
    solidity: float
    aspect: float
    rectangularity: float
    ring_white_frac: float
    contour: np.ndarray                 # px escalados (para desenhar)
    reason: Optional[str] = None        # None = aceito pelos filtros 2D
    partial: bool = False


def _exclusion_mask(cfg: HoleConfig, shape_hw, holding: bool) -> np.ndarray:
    h, w = int(shape_hw[0]), int(shape_hw[1])
    m = np.zeros((h, w), dtype=np.uint8)
    include_rects = holding or not cfg.cube_rect_only_when_holding
    for poly in exclusion_polygons_px(cfg, w, h, include_rects=include_rects):
        # um por vez: fillPoly com poligonos sobrepostos abre buraco na intersecao
        cv2.fillPoly(m, [poly], 255)
    return m


def segment_holes(bgr_full: np.ndarray, cfg: HoleConfig, holding: bool = True):
    """Parte de imagem do pipeline: (blobs, hsv_small, masks).

    masks: marrom (mascara final), branco (tampo), __garra__ (exclusao).
    """
    scale = float(cfg.process_scale)
    small = cv2.resize(bgr_full, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) \
        if scale != 1.0 else bgr_full
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    excl = _exclusion_mask(cfg, hsv.shape[:2], holding)

    brown = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in cfg.h_ranges:
        part = cv2.inRange(
            hsv, (int(lo), int(cfg.s_min), int(cfg.v_min)),
            (int(hi), int(cfg.s_max), int(cfg.v_max)))
        brown = cv2.bitwise_or(brown, part)
    brown_raw = brown.copy()            # antes da garra/morfologia: o anel desconta isto
    brown[excl > 0] = 0
    k_open = max(1, int(cfg.morph_open_px))
    k_close = max(1, int(cfg.morph_close_px))
    if k_open > 1:
        brown = cv2.morphologyEx(brown, cv2.MORPH_OPEN, np.ones((k_open, k_open), np.uint8))
    if k_close > 1:
        brown = cv2.morphologyEx(
            brown, cv2.MORPH_CLOSE, np.ones((k_close, k_close), np.uint8),
            iterations=max(1, int(cfg.morph_close_iters)))

    # Tampo branco: saturacao baixa + brilho alto.
    white = cv2.inRange(hsv, (0, 0, int(cfg.white_v_min)), (179, int(cfg.white_s_max), 255))

    inv = 1.0 / scale
    sh, sw = brown.shape[:2]
    contours, _ = cv2.findContours(brown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs: List[HoleBlob] = []
    ring_k = 2 * max(1, int(cfg.ring_px)) + 1
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

        # Anel branco: dilatacao do blob menos o blob, descontando qualquer
        # marrom cru (buracos vizinhos colados nao contam como "nao branco").
        blob_fill = np.zeros((sh, sw), dtype=np.uint8)
        cv2.drawContours(blob_fill, [cnt], -1, 255, thickness=cv2.FILLED)
        ring = cv2.dilate(blob_fill, np.ones((ring_k, ring_k), np.uint8))
        ring = cv2.subtract(ring, blob_fill)
        ring[brown_raw > 0] = 0
        ring[excl > 0] = 0
        ring_n = int(np.count_nonzero(ring))
        ring_white = float(np.count_nonzero(cv2.bitwise_and(ring, white))) / ring_n \
            if ring_n > 0 else 0.0

        scale_pt = lambda pt: (float(pt[0]) * inv, float(pt[1]) * inv)  # noqa: E731
        blob = HoleBlob(
            center=(cx * inv, cy * inv),
            box=np.asarray(box_s, dtype=float) * inv,
            long_mid=(scale_pt(long_mid[0]), scale_pt(long_mid[1])),
            short_mid=(scale_pt(short_mid[0]), scale_pt(short_mid[1])),
            area_px=area_s * inv * inv,
            solidity=solidity,
            aspect=aspect,
            rectangularity=rectangularity,
            ring_white_frac=ring_white,
            contour=cnt,
        )
        bx, by, bw, bh = cv2.boundingRect(cnt)
        m = max(0, int(cfg.border_margin_px))
        touches = bx <= m or by <= m or bx + bw >= sw - m or by + bh >= sh - m
        if not touches and np.any(excl):
            grown = cv2.dilate(blob_fill, np.ones((7, 7), np.uint8))
            touches = bool(np.any(cv2.bitwise_and(grown, excl)))
        blob.partial = touches
        if area_s < cfg.min_area_px:
            blob.reason = "area_min"
        elif touches and not cfg.accept_partial:
            blob.reason = "cortado"
        elif solidity < cfg.min_solidity:
            blob.reason = "solidez"
        elif aspect > cfg.max_aspect_ratio:
            blob.reason = "proporcao"
        elif rectangularity < cfg.min_rectangularity:
            blob.reason = "retangularidade"
        elif cfg.ring_enabled and ring_white < cfg.ring_white_min_frac:
            blob.reason = f"anel_branco({ring_white:.2f})"
        blobs.append(blob)
    blobs.sort(key=lambda b: b.area_px, reverse=True)
    masks = {"marrom": brown, "branco": white, "__garra__": excl}
    return blobs, hsv, masks


def hole_xy(caster: PlaneCaster, blob: HoleBlob, plane_z: float, cfg: HoleConfig):
    """Centro da cavidade no frame base via ray-cast: ((x, y, long, short,
    area_ratio), motivo). Primeiro elemento None quando rejeitado."""
    c = caster.cast(blob.center[0], blob.center[1], plane_z)
    if c is None:
        return None, "sem_plano"
    l1 = caster.cast(*blob.long_mid[0], plane_z)
    l2 = caster.cast(*blob.long_mid[1], plane_z)
    s1 = caster.cast(*blob.short_mid[0], plane_z)
    s2 = caster.cast(*blob.short_mid[1], plane_z)
    if l1 is None or l2 is None or s1 is None or s2 is None:
        return None, "sem_plano"
    long_len = float(np.linalg.norm(l2 - l1))
    short_len = float(np.linalg.norm(s2 - s1))
    tol = cfg.side_tolerance_frac
    if long_len > cfg.hole_side_m * (1 + tol):
        return None, f"lado_longo({long_len:.3f}m)"
    if short_len < cfg.hole_side_m * (1 - tol):
        return None, f"lado_curto({short_len:.3f}m)"
    dist = float(np.linalg.norm(c - caster.o))
    fx, fy = caster.K[0, 0], caster.K[1, 1]
    area_exp = cfg.hole_side_m * cfg.hole_side_m * fx * fy / max(dist * dist, 1e-6)
    area_ratio = blob.area_px / area_exp if area_exp > 0 else 0.0
    if area_ratio < cfg.area_expected_min_frac or area_ratio > cfg.area_expected_max_frac:
        return None, f"area_esperada({area_ratio:.2f}x)"
    x, y = float(c[0]), float(c[1])
    if y < cfg.min_target_y_m:
        return None, f"atras_do_braco(y={y:.2f})"
    if math.hypot(x, y) > cfg.max_target_radius_m:
        return None, f"longe(r={math.hypot(x, y):.2f})"
    return (x, y, long_len, short_len, area_ratio), ""


class HoleTracks:
    """Varios buracos anonimos: casa cada deteccao com o rastro mais proximo
    (< match_dist_m) e publica a MEDIANA por rastro com min_samples amostras.
    Troca de plano/braco em movimento: reset() (quem chama decide)."""

    def __init__(self, match_dist_m: float = 0.03, n: int = 5, window_s: float = 2.5,
                 min_samples: int = 2):
        self.match_dist_m = float(match_dist_m)
        self.n = max(1, int(n))
        self.window_s = float(window_s)
        self.min_samples = max(1, int(min_samples))
        self.tracks: List[collections.deque] = []   # cada rastro: deque de (t, x, y)

    def reset(self) -> None:
        self.tracks.clear()

    def _center(self, tr) -> Tuple[float, float]:
        xs = sorted(s[1] for s in tr)
        ys = sorted(s[2] for s in tr)
        m = len(xs) // 2
        return xs[m], ys[m]

    def push(self, t: float, detections: List[Tuple[float, float]]) -> None:
        # rastros velhos caem antes do casamento
        self.tracks = [tr for tr in self.tracks if tr and t - tr[-1][0] <= self.window_s]
        used = set()
        for (x, y) in detections:
            best, best_d = None, self.match_dist_m
            for i, tr in enumerate(self.tracks):
                if i in used:
                    continue
                cx, cy = self._center(tr)
                d = math.hypot(x - cx, y - cy)
                if d < best_d:
                    best, best_d = i, d
            if best is None:
                tr = collections.deque(maxlen=self.n)
                self.tracks.append(tr)
                best = len(self.tracks) - 1
            self.tracks[best].append((t, x, y))
            used.add(best)

    def estimates(self, t: float) -> List[Tuple[float, float, int]]:
        out = []
        for tr in self.tracks:
            while tr and t - tr[0][0] > self.window_s:
                tr.popleft()
            if len(tr) >= self.min_samples:
                cx, cy = self._center(tr)
                out.append((cx, cy, len(tr)))
        out.sort(key=lambda e: (e[0], e[1]))   # ordem estavel para os TFs
        return out


def draw_overlay(bgr_full: np.ndarray, scale: float, blobs: List[HoleBlob],
                 cfg: HoleConfig, status_lines: List[str], holding: bool = True) -> np.ndarray:
    out = bgr_full.copy()
    h, w = out.shape[:2]
    include_rects = holding or not cfg.cube_rect_only_when_holding
    for poly in exclusion_polygons_px(cfg, w, h, include_rects=include_rects):
        cv2.polylines(out, [poly], True, (0, 255, 255), 1)
    inv = 1.0 / scale
    for b in blobs:
        ok = b.reason is None
        color = (0, 255, 0) if ok else (0, 0, 255)
        cnt = (b.contour.astype(float) * inv).astype(np.int32)
        cv2.drawContours(out, [cnt], -1, color, 1)
        cx, cy = int(b.center[0]), int(b.center[1])
        cv2.drawMarker(out, (cx, cy), color, cv2.MARKER_CROSS, 12, 1)
        label = f"anel={b.ring_white_frac:.2f}" + ("" if ok else f" x {b.reason}")
        cv2.putText(out, label, (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    y = 16
    for line in status_lines:
        cv2.putText(out, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        y += 16
    return out


def config_from_dict(p: dict) -> HoleConfig:
    cfg = HoleConfig()
    simple = ["process_scale", "s_min", "s_max", "v_min", "v_max",
              "ring_enabled", "ring_px", "ring_white_min_frac", "white_s_max", "white_v_min",
              "min_area_px", "min_solidity", "max_aspect_ratio", "min_rectangularity",
              "border_margin_px", "accept_partial",
              "morph_open_px", "morph_close_px", "morph_close_iters",
              "hole_side_m", "side_tolerance_frac",
              "area_expected_min_frac", "area_expected_max_frac",
              "min_target_y_m", "max_target_radius_m", "max_ray_length_m",
              "cube_rect_only_when_holding", "gripper_joint", "gripper_holding_above"]
    for k in simple:
        if k in p:
            setattr(cfg, k, _coerce(type(getattr(cfg, k)), p[k]))
    if "h_ranges" in p:
        cfg.h_ranges = parse_h_ranges(p["h_ranges"])
    if "exclusion_rects_norm" in p:
        cfg.exclusion_rects = parse_rects(p["exclusion_rects_norm"])
    if "exclusion_polys_norm" in p:
        cfg.exclusion_polys = parse_polys(p["exclusion_polys_norm"])
    if "exclusion_polys_holding_norm" in p:
        cfg.exclusion_polys_holding = parse_polys(p["exclusion_polys_holding_norm"])
    return cfg


def load_config_yaml(path: str) -> HoleConfig:
    import yaml  # local: o nucleo nao depende dele
    with open(path, "r", encoding="utf-8") as handle:
        root = yaml.safe_load(handle) or {}
    params = root.get("pp_hole_detector", {}).get("ros__parameters", root)
    return config_from_dict(params)


# ============================================================================
# CLI offline (calibracao com fotos)
# ============================================================================
def run_offline(args) -> int:
    cfg = load_config_yaml(args.config) if args.config else HoleConfig()
    bgr = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if bgr is None:
        print(f"nao consegui abrir {args.image}", file=sys.stderr)
        return 2
    h, w = bgr.shape[:2]
    print(f"imagem {w}x{h}, process_scale {cfg.process_scale}")
    holding = not getattr(args, "gripper_open", False)
    t0 = time.perf_counter()
    blobs, _hsv, _masks = segment_holes(bgr, cfg, holding=holding)
    dt_ms = (time.perf_counter() - t0) * 1000.0
    if args.probe:
        u, v = [int(x) for x in args.probe.split(",")]
        hsv_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        print(f"probe ({u},{v}): HSV = {tuple(int(x) for x in hsv_full[v, u])}  (H 0-179)")
    print(f"\nmarrom H {cfg.h_ranges} S {cfg.s_min}-{cfg.s_max} V {cfg.v_min}-{cfg.v_max}; "
          f"anel branco S<={cfg.white_s_max} V>={cfg.white_v_min} "
          f"min_frac {cfg.ring_white_min_frac}: {len(blobs)} blob(s) em {dt_ms:.1f} ms")
    for i, b in enumerate(blobs[:10]):
        print(f"  #{i}: centro=({b.center[0]:.0f},{b.center[1]:.0f}) area={b.area_px:.0f}px "
              f"solidez={b.solidity:.2f} proporcao={b.aspect:.2f} retang={b.rectangularity:.2f} "
              f"anel={b.ring_white_frac:.2f} "
              f"{'ACEITO (2D)' if b.reason is None else 'rejeitado: ' + b.reason}")
    if args.out:
        status = [f"{dt_ms:.1f} ms", f"{sum(1 for b in blobs if b.reason is None)} aceito(s)"]
        cv2.imwrite(args.out, draw_overlay(bgr, cfg.process_scale, blobs, cfg, status, holding))
        print(f"\noverlay salvo em {args.out}")
    return 0


# ============================================================================
# No ROS
# ============================================================================
def ros_main(argv=None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
    from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_msgs.msg import Bool, Float64, String
    from geometry_msgs.msg import Pose, PoseArray, TransformStamped
    from tf2_ros import Buffer, TransformListener, TransformBroadcaster
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    from cv_bridge import CvBridge
    from rclpy.time import Time
    from rclpy.duration import Duration

    class PpHoleDetector(Node):
        def __init__(self):
            super().__init__("pp_hole_detector")
            # dynamic_typing: operador escrevendo `300` em vez de `300.0` no
            # yaml nao pode derrubar o no' na arena (mesma regra do container).
            _dyn = ParameterDescriptor(dynamic_typing=True)

            def dp(name, default):
                return self.declare_parameter(name, default, _dyn)
            dp("rate_hz", 3.0)
            dp("always_on", False)
            dp("image_topic", "/camera/camera/color/image_raw")
            dp("camera_info_topic", "/camera/camera/color/camera_info")
            dp("place_active_topic", "/manip/place_active")
            dp("plane_z_topic", "/manip/pp_plane_z")
            dp("debug_image", True)
            dp("save_debug_dir", "")       # mesmo aviso do container: enche disco, so' diagnostico
            dp("save_debug_every_s", 1.0)
            dp("base_frame", "manip_base_link")
            dp("camera_frame", "")
            # Tampo da mesa de 15 cm em manip_base_link (borda do container
            # 0.041 - altura 0.070). So' vale sem o place publicando o plano.
            dp("plane_z_default", -0.029)
            dp("tf_timeout_s", 0.0)
            dp("reject_when_camera_moving", True)
            dp("camera_motion_max_m", 0.005)
            dp("camera_motion_max_deg", 1.0)
            dp("publish_tf", True)
            dp("joint_states_topic", "/joint_states")
            # nucleo (HoleConfig)
            dp("process_scale", 0.5)
            dp("h_ranges", [5, 25])
            dp("s_min", 50)
            dp("s_max", 255)
            dp("v_min", 40)
            dp("v_max", 230)
            dp("ring_enabled", True)
            dp("ring_px", 6)
            dp("ring_white_min_frac", 0.45)
            dp("white_s_max", 70)
            dp("white_v_min", 140)
            dp("min_area_px", 150.0)
            dp("min_solidity", 0.85)
            dp("max_aspect_ratio", 1.8)
            dp("min_rectangularity", 0.60)
            dp("border_margin_px", 3)
            dp("accept_partial", False)
            dp("morph_open_px", 3)
            dp("morph_close_px", 5)
            dp("morph_close_iters", 1)
            dp("hole_side_m", 0.05)
            dp("side_tolerance_frac", 0.5)
            dp("area_expected_min_frac", 0.3)
            dp("area_expected_max_frac", 4.0)
            dp("min_target_y_m", 0.10)
            dp("max_target_radius_m", 0.60)
            dp("max_ray_length_m", 1.0)
            dp("exclusion_rects_norm", [0.30, 0.00, 0.62, 0.42])
            dp("exclusion_polys_norm", [" ".join(f"{x:.3f},{y:.3f}" for x, y in poly)
                                        for poly in DEFAULT_GRIPPER_POLYS])
            dp("exclusion_polys_holding_norm", [""])
            dp("cube_rect_only_when_holding", True)
            dp("gripper_joint", "manip_joint6")
            dp("gripper_holding_above", -0.12)
            # rastreio
            dp("track_match_dist_m", 0.03)
            dp("smoothing_n", 5)
            dp("smoothing_window_s", 2.5)
            dp("min_samples", 2)

            try:
                self.cfg = self._build_config()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"parametros invalidos no arranque ({exc}) — usando defaults internos.")
                self.cfg = HoleConfig()
            self.add_on_set_parameters_callback(self._on_params)
            if hasattr(self, "add_post_set_parameters_callback"):
                self.add_post_set_parameters_callback(self._on_params_applied)
            self._pending_cfg: Optional[HoleConfig] = None

            self.bridge = CvBridge()
            self.caster: Optional[PlaneCaster] = None
            self.camera_frame_from_info = ""
            self.plane_z: Optional[float] = None
            self.tracks = self._new_tracks()
            self.last_processed = 0.0
            self.last_cam_pose = None
            self.image_sub = None
            self.place_active = False
            self.gripper_holding = True    # sem joint_states: assume segurando (seguro)
            self.gripper_pos = None
            self.warned_no_plane = False
            self.warned_no_info = False
            self.warned_tf = 0.0

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
            self.create_subscription(
                JointState, self.get_parameter("joint_states_topic").value, self._on_joint_states, 10)
            self.status_pub = self.create_publisher(String, "/manip/pp_hole_detector/status", 5)
            self.holes_pub = self.create_publisher(PoseArray, "/manip/pp_hole_detector/holes", 5)
            self.debug_pub = self.create_publisher(Image, "/manip/pp_hole_detector/debug_image", 1)
            self.get_logger().info(
                "pp_hole_detector pronto: marrom H %s; anel branco %s; ativo %s; plano default %.3f."
                % (self.cfg.h_ranges,
                   "exigido" if self.cfg.ring_enabled else "DESLIGADO",
                   "sempre" if self.get_parameter("always_on").value else "so durante o place",
                   self.get_parameter("plane_z_default").value))
            self._update_active()

        # ---------------- parametros ----------------
        def _build_config(self) -> HoleConfig:
            return config_from_dict(self._config_dict())

        def _new_tracks(self) -> HoleTracks:
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            return HoleTracks(g("track_match_dist_m"), g("smoothing_n"),
                              g("smoothing_window_s"), g("min_samples"))

        def _config_dict(self, overrides=None) -> dict:
            g = lambda k: self.get_parameter(k).value  # noqa: E731
            keys = ["process_scale", "h_ranges", "s_min", "s_max", "v_min", "v_max",
                    "ring_enabled", "ring_px", "ring_white_min_frac", "white_s_max",
                    "white_v_min", "min_area_px", "min_solidity", "max_aspect_ratio",
                    "min_rectangularity", "border_margin_px", "accept_partial",
                    "morph_open_px", "morph_close_px", "morph_close_iters",
                    "hole_side_m", "side_tolerance_frac",
                    "area_expected_min_frac", "area_expected_max_frac",
                    "min_target_y_m", "max_target_radius_m", "max_ray_length_m",
                    "exclusion_rects_norm", "exclusion_polys_norm",
                    "exclusion_polys_holding_norm", "cube_rect_only_when_holding",
                    "gripper_joint", "gripper_holding_above"]
            p = {k: g(k) for k in keys}
            for k, v in (overrides or {}).items():
                if k in keys:
                    p[k] = v
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
            if any(prm.name == "always_on" for prm in params) and hasattr(self, "image_sub"):
                self._update_active()

        def _apply_cfg(self, cfg: HoleConfig):
            self.cfg = cfg
            if self.caster is not None:
                self.caster.cfg = cfg
            self.tracks = self._new_tracks()
            self.get_logger().info("parametros recarregados.")

        # ---------------- ativacao / entradas ----------------
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
                self.tracks.reset()
                self.warned_no_plane = False
                self.get_logger().info("ativo: assinando a imagem.")
            elif not want and self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
                self.get_logger().info("inativo: imagem desassinada (0%% CPU).")

        def _on_camera_info(self, msg: CameraInfo):
            if self.caster is None or self.caster.K[0, 0] != msg.k[0]:
                self.caster = PlaneCaster(np.array(msg.k), list(msg.d), self.cfg)
                self.camera_frame_from_info = msg.header.frame_id
                self.get_logger().info(
                    f"camera_info: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} frame={msg.header.frame_id}")
            self.caster.cfg = self.cfg

        def _on_plane_z(self, msg: Float64):
            changed = self.plane_z is None or abs(float(msg.data) - self.plane_z) > 0.005
            self.plane_z = float(msg.data)
            if changed:
                self.tracks.reset()
                self.get_logger().info(
                    f"plano do tampo recebido: z={self.plane_z:.3f} (manip_base_link)")

        def _on_joint_states(self, msg: JointState):
            try:
                idx = list(msg.name).index(self.cfg.gripper_joint)
            except ValueError:
                return
            if idx < len(msg.position):
                self.gripper_pos = float(msg.position[idx])
                self.gripper_holding = self.gripper_pos > float(self.cfg.gripper_holding_above)

        def _camera_pose(self, stamp):
            base = self.get_parameter("base_frame").value
            cam = self.get_parameter("camera_frame").value or self.camera_frame_from_info \
                or "camera_color_optical_frame"
            timeout = Duration(seconds=max(0.0, float(self.get_parameter("tf_timeout_s").value)))
            try:
                tf = self.tf_buffer.lookup_transform(base, cam, stamp, timeout)
            except (LookupException, ConnectivityException, ExtrapolationException):
                try:
                    tf = self.tf_buffer.lookup_transform(base, cam, Time(), timeout)
                except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                    now = time.monotonic()
                    if now - self.warned_tf > 5.0:
                        self.warned_tf = now
                        self.get_logger().warn(f"sem TF {base} <- {cam}: {exc}")
                    return None, None
            t = tf.transform.translation
            q = tf.transform.rotation
            return np.array([t.x, t.y, t.z]), quat_to_rot(q.x, q.y, q.z, q.w)

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

        def _process(self, msg: Image, stamp, t_img: float):
            t0 = time.perf_counter()
            if self.caster is None:
                if not self.warned_no_info:
                    self.warned_no_info = True
                    self.get_logger().warn("sem camera_info ainda — nao detecto.")
                self._publish_status("sem camera_info")
                return
            origin, rot = self._camera_pose(stamp)
            if origin is None:
                self._publish_status("sem TF da camera")
                return
            if bool(self.get_parameter("reject_when_camera_moving").value) and \
                    self.last_cam_pose is not None:
                o0, r0 = self.last_cam_pose
                dpos = float(np.linalg.norm(origin - o0))
                dang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r0.T @ rot) - 1.0) / 2.0))))
                if dpos > float(self.get_parameter("camera_motion_max_m").value) or \
                        dang > float(self.get_parameter("camera_motion_max_deg").value):
                    self.last_cam_pose = (origin, rot)
                    self.tracks.reset()
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
                        f"sem /manip/pp_plane_z — usando plane_z_default={plane_z:.3f}")

            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            holding = bool(self.gripper_holding)
            blobs, _hsv, _masks = segment_holes(bgr, self.cfg, holding=holding)
            detections: List[Tuple[float, float]] = []
            rejects: List[str] = []
            for b in blobs:
                if b.reason is not None:
                    rejects.append(b.reason)
                    continue
                res, why = hole_xy(self.caster, b, plane_z, self.cfg)
                if res is None:
                    b.reason = why
                    rejects.append(why)
                    continue
                detections.append((res[0], res[1]))
            self.tracks.push(t_img, detections)
            estimates = self.tracks.estimates(t_img)

            base = self.get_parameter("base_frame").value
            pa = PoseArray()
            pa.header.stamp = msg.header.stamp
            pa.header.frame_id = base
            tfs = []
            for i, (x, y, _n) in enumerate(estimates):
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = x, y, plane_z
                pose.orientation.w = 1.0
                pa.poses.append(pose)
                if bool(self.get_parameter("publish_tf").value):
                    tf = TransformStamped()
                    tf.header.stamp = msg.header.stamp
                    tf.header.frame_id = base
                    tf.child_frame_id = f"pp_buraco_{i}"
                    tf.transform.translation.x = float(x)
                    tf.transform.translation.y = float(y)
                    tf.transform.translation.z = float(plane_z)
                    tf.transform.rotation.w = 1.0
                    tfs.append(tf)
            self.holes_pub.publish(pa)
            if tfs:
                self.tf_broadcaster.sendTransform(tfs)

            dt_ms = (time.perf_counter() - t0) * 1000.0
            status = [f"garra: {'segurando' if holding else 'vazia'}",
                      f"{len(detections)} visto(s) agora, {len(estimates)} estavel(is): "
                      + "; ".join(f"[{x:.3f} {y:.3f}] n={n}" for x, y, n in estimates)]
            if rejects and not estimates:
                status.append("rejeitados: " + ", ".join(sorted(set(rejects)))[:80])
            status.append(f"plane_z={plane_z:.3f}({plane_src}) {dt_ms:.1f}ms")
            self._publish_status(" | ".join(status))

            want_pub = bool(self.get_parameter("debug_image").value) and \
                self.debug_pub.get_subscription_count() > 0
            save_dir = str(self.get_parameter("save_debug_dir").value or "").strip()
            now_wall = time.time()
            want_save = bool(save_dir) and (now_wall - getattr(self, "_last_save_wall", 0.0)) >= \
                float(self.get_parameter("save_debug_every_s").value)
            if want_pub or want_save:
                overlay = draw_overlay(bgr, self.cfg.process_scale, blobs, self.cfg, status, holding)
                if want_pub:
                    out = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                    out.header = msg.header
                    self.debug_pub.publish(out)
                if want_save:
                    try:
                        d = os.path.expanduser(save_dir)
                        os.makedirs(d, exist_ok=True)
                        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(now_wall)) \
                            + f"_{int((now_wall % 1) * 1000):03d}"
                        cv2.imwrite(os.path.join(d, f"{ts}_raw.png"), bgr)
                        cv2.imwrite(os.path.join(d, f"{ts}_overlay.png"), overlay)
                        with open(os.path.join(d, "log.txt"), "a", encoding="utf-8") as fh:
                            fh.write(f"{ts} | {' | '.join(status)}\n")
                        self._last_save_wall = now_wall
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().warn(
                            f"nao consegui gravar debug em {save_dir}: {exc}",
                            throttle_duration_sec=10.0)

        def _publish_status(self, text: str):
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=argv)
    node = PpHoleDetector()
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
        parser = argparse.ArgumentParser(description="Calibracao offline do detector de buracos da PP.")
        parser.add_argument("--image", required=True)
        parser.add_argument("--config", default=None, help="pp_hole_detector.yaml (default: valores internos)")
        parser.add_argument("--out", default=None, help="salva overlay PNG")
        parser.add_argument("--probe", default=None, help="u,v: imprime o HSV desse pixel")
        parser.add_argument("--gripper-open", action="store_true",
                            help="foto com a garra vazia: nao apaga o retangulo do cubo carregado")
        return run_offline(parser.parse_args(argv))
    return ros_main(argv)


if __name__ == "__main__":
    sys.exit(main())
