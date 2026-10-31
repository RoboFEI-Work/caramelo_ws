"""Imagem de debug do docking por camera (2026-10-27, pedido do operador).

"Pelo rqt_image_view a gente poder ver os algoritmos da visao" — uma imagem
BGR com dois paineis, publicada pelo dock_align_node (em cada frame medido
durante o docking) e pelo dock_camera_probe:

  ESQUERDA: o depth da D455 (claro = perto), JA' GIRADO 180 graus (a camera
            fica de cabeca para baixo no braco) com o TAMPO usado em verde e
            o que foi descartado como PAREDE/face vertical em vermelho.
  DIREITA:  vista de CIMA no base_footprint corrigido (robo embaixo, olhando
            para cima; grade de 10 cm): cinza = tudo que a camera ve, verde =
            tampo, amarelo = borda da frente (pontos e reta), ciano =
            laterais, branco = eixo do robo, magenta = centro da face e a
            normal (para onde o robo devia apontar). Texto: yaw, centro,
            largura, altura, e o motivo quando rejeita.

So' desenho (cv2 + numpy); a conta esta em caramelo_dock_camera.
"""
import math

import cv2
import numpy as np

_X_RANGE = (0.30, 1.70)
_Y_HALF = 0.70
_SIDE = 480


def _top_px(x, y):
    s = _SIDE / (_X_RANGE[1] - _X_RANGE[0])
    col = (_Y_HALF - np.asarray(y)) / (2 * _Y_HALF) * _SIDE
    row = (_X_RANGE[1] - np.asarray(x)) * s
    return np.round(col).astype(int), np.round(row).astype(int)


def _uv_to_xy(u, v, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return u * c - v * s, u * s + v * c


def _dots(img, xy, color, r=1):
    if xy is None or len(xy) == 0:
        return
    cols, rows = _top_px(xy[:, 0], xy[:, 1])
    ok = (cols >= 0) & (cols < _SIDE) & (rows >= 0) & (rows < _SIDE)
    if r <= 1:
        img[rows[ok], cols[ok]] = color
        return
    for c, r_ in zip(cols[ok], rows[ok]):
        cv2.circle(img, (int(c), int(r_)), r, color, -1)


def _line(img, p0, p1, color, thick=2):
    c0, r0 = _top_px(*p0)
    c1, r1 = _top_px(*p1)
    cv2.line(img, (int(c0), int(r0)), (int(c1), int(r1)), color, thick, cv2.LINE_AA)


def _wrap(lines, width=52):
    out = []
    for t in lines:
        while len(t) > width:
            cut = t.rfind(" ", 0, width)
            cut = cut if cut > 10 else width
            out.append(t[:cut])
            t = "  " + t[cut:].lstrip()
        out.append(t)
    return out


def _text(img, lines, org, color=(255, 255, 255), scale=0.45):
    x, y = org
    for t in _wrap(lines):
        cv2.putText(img, t, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, t, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
        y += int(22 * scale / 0.45)


def draw(depth, m, label="", why="", stride=4):
    """depth HxW (uint16 mm), m = TableMeasure ou None -> imagem BGR."""
    h, w = depth.shape[:2]
    # ---- painel do depth ----
    d = depth.astype(np.float32) * 0.001
    gray = np.where(d > 0, np.clip(255.0 * (1.0 - d / 2.0), 30, 255), 0).astype(np.uint8)
    left = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    dbg = m.dbg if m is not None else {}
    # Mascara na grade do stride e um resize "nearest" (2026-10-27: um
    # cv2.rectangle por ponto custava ~60 ms por frame durante o docking).
    s = max(1, stride)
    for key, color in (("wall_pix", (0, 0, 230)), ("table_pix", (0, 200, 0))):
        pix = dbg.get(key)
        if pix is None or len(pix) == 0:
            continue
        mask = np.zeros(((h + s - 1) // s, (w + s - 1) // s), dtype=np.uint8)
        mask[np.asarray(pix[:, 1]) // s, np.asarray(pix[:, 0]) // s] = 1
        mask = cv2.resize(mask, (mask.shape[1] * s, mask.shape[0] * s),
                          interpolation=cv2.INTER_NEAREST)[:h, :w].astype(bool)
        left[mask] = color
    left = cv2.rotate(left, cv2.ROTATE_180)
    if w != _SIDE or h != _SIDE:
        left = cv2.resize(left, (int(round(w * _SIDE / h)), _SIDE))
    _text(left, ["depth (girado 180)", "verde = tampo   vermelho = parede/vertical"], (8, 20))

    # ---- vista de cima ----
    top = np.full((_SIDE, _SIDE, 3), 25, np.uint8)
    for gx in np.arange(math.ceil(_X_RANGE[0] * 10) / 10, _X_RANGE[1], 0.1):
        _line(top, (gx, -_Y_HALF), (gx, _Y_HALF), (45, 45, 45), 1)
    for gy in np.arange(-0.6, 0.61, 0.1):
        _line(top, (_X_RANGE[0], gy), (_X_RANGE[1], gy), (45, 45, 45), 1)
    _dots(top, dbg.get("all_xy"), (110, 110, 110))
    _dots(top, dbg.get("table_xy"), (0, 170, 0))
    _line(top, (_X_RANGE[0], 0.0), (_X_RANGE[1], 0.0), (230, 230, 230), 1)
    _dots(top, dbg.get("front_xy"), (0, 230, 255), r=2)

    yaw = dbg.get("yaw")
    if yaw is not None:
        dd = dbg["d"]
        vl, vr = dbg.get("vl"), dbg.get("vr")
        v_lo = vr if vr is not None else -0.45
        v_hi = vl if vl is not None else 0.45
        _line(top, _uv_to_xy(dd, v_lo, yaw), _uv_to_xy(dd, v_hi, yaw), (0, 230, 255), 2)
        for name, pts in dbg.get("sides", {}).items():
            if len(pts):
                xs, ys = _uv_to_xy(pts[:, 0], pts[:, 1], yaw)
                _dots(top, np.stack([xs, ys], axis=1), (255, 255, 0), r=2)
        for v in (vl, vr):
            if v is not None:
                _line(top, _uv_to_xy(dd, v, yaw), _uv_to_xy(dbg["u_far"], v, yaw), (255, 255, 0), 2)
        if dbg.get("v_c") is not None:
            cx, cy = _uv_to_xy(dd, dbg["v_c"], yaw)
            ex, ey = _uv_to_xy(dd - 0.25, dbg["v_c"], yaw)
            _line(top, (ex, ey), (cx, cy), (255, 0, 255), 2)
            cc, rr = _top_px(cx, cy)
            cv2.circle(top, (int(cc), int(rr)), 6, (255, 0, 255), 2)

    # ---- texto ----
    lines = [label] if label else []
    if m is not None and m.ok:
        lines.append(f"yaw {math.degrees(m.face_yaw):+.1f} graus ({m.yaw_src})")
        lines.append("centro: " + ("? (" + m.center_reason + ")" if m.c_foot is None else
                                   f"c_axis {m.c_axis * 100:+.1f}  c_foot {m.c_foot * 100:+.1f} cm"))
        lines.append(f"largura {'?' if m.width is None else '%.2f' % m.width} m  "
                     f"frente {m.front:.2f} m  h {m.table_h:.2f} m  incl {m.tilt_deg:.1f}")
        rect = m.info.get("retangulo")
        if rect:
            lines.append(f"retangulo: preenche {rect[0]:.0%} fora {rect[1]:.0%} "
                         f"fundo reto {'?' if rect[2] is None else '%.0f%%' % (rect[2] * 100)}")
        color = (120, 255, 120) if m.c_foot is not None else (0, 220, 255)
    else:
        lines.append("SEM MESA: " + (why or (m.reason if m is not None else "?")))
        color = (80, 80, 255)
    _text(top, lines, (8, 20), color=color)
    _text(top, ["vista de cima (robo embaixo), grade 10 cm"], (8, _SIDE - 10),
          color=(170, 170, 170), scale=0.4)
    return np.hstack([left, top])
