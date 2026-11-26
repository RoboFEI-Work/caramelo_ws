#!/usr/bin/env python3
"""Mesa de precisao (PP) refeita do zero: furo e cubo medidos pela camera. Sem ROS.

2026-11-16 (branch mudanca_pp, pedido do operador): "fazer a verificacao do furo
e o seu centro" e descer com o cubo em cima do furo, recalculando a cada parada.

Tudo e' medido na propria imagem, com a TAG DO SLOT como regua (a tag e o furo
estao no mesmo plano, o tampo):
  1. medir_furo: a homografia dos 4 cantos da tag leva a imagem para uma vista
     de cima em milimetros; nela o furo e' o quadrado marrom/escuro colado na
     tag. Sai o CENTRO DO FURO em mm no frame da tag (medido em 16/11 no slot
     104: (+0,3; -45,8) mm, +-0,5 mm entre quadros e corridas; furo 45,0 x 44,2).
  2. erro_do_cubo: a pose da camera em relacao ao tampo (PnP da tag) e o ponto
     do FUNDO DO CUBO no frame da camera (constante: camera e garra sao rigidas)
     dao o pe' do cubo no tampo; a diferenca para o centro do furo e' quanto o
     cubo tem de andar, ja' em "lado" e "frente" do braco.

Nao usa TF, nem a calibracao camera->braco, nem offset fixo tag->furo: o vetor
tag->furo e' medido na hora, enquanto o furo esta' inteiro a' vista, e guardado
para quando o cubo o tapar (a tag do slot continua visivel).

Por que o fundo do cubo e nao o TCP: o que entra no furo e' o fundo do cubo, e
a 2 cm do tampo um erro de 1 grau na inclinacao so' desloca esse ponto 0,3 mm.
"""

import math
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np


@dataclass
class FuroCfg:
    lado_tag_mm: float = 31.8          # quadrado preto da tag do slot (regua da imagem)
    px_mm: float = 4.0                 # resolucao da vista retificada
    # Janela onde o furo e' procurado, em mm no frame da tag (furo em -y).
    janela_mm: tuple = (-45.0, 45.0, -110.0, -14.0)
    # Fundo marrom (H 0-179) e sombra das paredes do furo.
    h_marrom: tuple = (3, 28)
    s_marrom_min: int = 45
    v_escuro_max: int = 85
    lado_furo_mm: float = 44.6         # so' para decidir "furo inteiro"
    lado_tol_mm: float = 4.0
    area_min_mm2: float = 250.0
    # Fundo do cubo no frame OPTICO da camera (mm). Calibrado em 16/11 pelos tres
    # places em que o cubo entrou (18h01, 18h26, 18h56): com o cubo em cima do
    # centro do furo, 15 mm acima do tampo, o ponto caiu em (-13,5 -32,4 132,2),
    # (-13,0 -31,8 131,2) e (-13,1 -32,2 131,5). Refazer se a camera ou a garra
    # forem trocadas (scripts de bancada: ver docs/pp_furo.md).
    fundo_cubo_cam_mm: tuple = (-13.25, -32.20, 131.78)
    # Angulo da tag na imagem (canto 0 -> canto 1) com as faces do cubo alinhadas.
    angulo_ref_graus: float = 180.0
    # 2026-11-26 (PP da arena: fundo do furo CLARO, furo ~46-49 mm e ~10 mm
    # SEPARADO da tag; a mascara de cor dava "sem_furo" e a orientacao saia
    # aleatoria). metodo: "cor" (so' o de sempre), "borda" (so' o novo) ou
    # "auto" (cor; se ela nao acha o furo INTEIRO, borda).
    metodo: str = "auto"
    # Busca pela borda: quadrado alinhado a' tag cujas 4 bordas tem o maior
    # contraste (gradiente), qualquer que seja a cor do fundo.
    borda_lado_mm: tuple = (42.0, 52.0)
    borda_faixa_mm: float = 2.0        # espessura da faixa de cada borda
    borda_contraste_min: float = 8.0   # pior borda / gradiente mediano da janela (furos reais: >= 20; falsos: <= 4,4)
    borda_gap_tag_mm: float = 4.0      # o furo comeca pelo menos isto alem do quadrado preto
    # Interior liso: o topo do cubo (tag colada) tem textura, o fundo do furo
    # nao. Sem isto o CUBO era achado como furo nos quadros com o cubo por cima.
    borda_interior_max: float = 1.5    # gradiente medio do miolo / mediano da janela
    borda_miolo_mm: float = 5.0        # o miolo ignora esta faixa junto das bordas


def homografia_tag(cantos, lado_mm):
    """mm no frame da tag (origem no centro; +x do canto 0 para o 1, +y do 1 para o 2) -> px."""
    m = np.float32([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * (lado_mm / 2.0)
    return cv2.getPerspectiveTransform(m, np.float32(cantos)).astype(np.float64)


def _retificar(bgr, cantos, cfg):
    x0, x1, y0, y1 = cfg.janela_mm
    a = np.array([[1 / cfg.px_mm, 0, x0], [0, 1 / cfg.px_mm, y0], [0, 0, 1]])
    m = homografia_tag(cantos, cfg.lado_tag_mm) @ a
    w, h = int(round((x1 - x0) * cfg.px_mm)), int(round((y1 - y0) * cfg.px_mm))
    return cv2.warpPerspective(bgr, np.linalg.inv(m), (w, h), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=(255, 255, 255))


def _mascara(ret, cfg):
    hsv = cv2.cvtColor(ret, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    marrom = (h >= cfg.h_marrom[0]) & (h <= cfg.h_marrom[1]) & (s >= cfg.s_marrom_min)
    m = (marrom | (v <= cfg.v_escuro_max)).astype(np.uint8) * 255
    k = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)


def _orientar(cantos, bgr, cfg):
    """Giro (0..3) da ordem dos cantos que poe o furo em -y do frame da tag: o lado com mais marrom."""
    c = np.asarray(cantos, float).reshape(4, 2)
    return int(np.argmax([np.count_nonzero(_mascara(_retificar(bgr, np.roll(c, k, axis=0), cfg), cfg))
                          for k in range(4)]))


def _quadrado_por_borda(ret, cfg):
    """Melhor quadrado (alinhado a' tag) pelas bordas na vista retificada.

    Devolve (score, pior_lado_rel, cx_px, cy_px, lado_px) ou None. score e
    pior_lado_rel sao relativos ao gradiente mediano da janela (textura do tampo).
    """
    g = cv2.GaussianBlur(cv2.cvtColor(ret, cv2.COLOR_BGR2GRAY).astype(np.float32), (3, 3), 0)
    gx = np.abs(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3))   # bordas verticais (esq./dir.)
    gy = np.abs(cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))   # bordas horizontais (cima/baixo)
    fundo = float(np.median(gx + gy)) / 2.0 + 1.0
    ix = cv2.integral(gx).astype(np.float64)
    iy = cv2.integral(gy).astype(np.float64)
    ig = cv2.integral((gx + gy) / 2.0).astype(np.float64)
    m = int(round(cfg.borda_miolo_mm * cfg.px_mm))
    h, w = g.shape
    px = cfg.px_mm
    b = max(1, int(round(cfg.borda_faixa_mm * px)))
    x0, _x1, y0, _y1 = cfg.janela_mm
    # A borda de baixo do furo (lado da tag) nao pode entrar na tag nem no
    # contorno branco/azul dela: so' alem de lado_tag/2 + gap.
    v_max = int(((-cfg.lado_tag_mm / 2.0 - cfg.borda_gap_tag_mm) - y0) * px)
    melhor = None

    def soma(ii, r0, r1, c0, c1):
        # soma do retangulo [r0,r1) x [c0,c1) para arrays de r0/c0 (mesma forma)
        return ii[r1, c1] - ii[r0, c1] - ii[r1, c0] + ii[r0, c0]

    for lado_mm in np.arange(cfg.borda_lado_mm[0], cfg.borda_lado_mm[1] + 1e-6, 1.0):
        s = int(round(lado_mm * px))
        v_hi = min(h - b - 1, v_max) - s           # topo maximo (fundo do quadrado <= v_max)
        u_hi = w - b - 1 - s
        if v_hi <= b or u_hi <= b:
            continue
        vv, uu = np.mgrid[b:v_hi:2, b:u_hi:2]       # passo de 0,5 mm (4 px/mm): sobra na folga do furo
        hb = b // 2
        esq = soma(ix, vv, vv + s, uu - hb, uu - hb + b) / (s * b)
        dirt = soma(ix, vv, vv + s, uu + s - hb, uu + s - hb + b) / (s * b)
        cima = soma(iy, vv - hb, vv - hb + b, uu, uu + s) / (s * b)
        baixo = soma(iy, vv + s - hb, vv + s - hb + b, uu, uu + s) / (s * b)
        pior = np.minimum(np.minimum(esq, dirt), np.minimum(cima, baixo))
        score = (esq + dirt + cima + baixo) / 4.0 + pior     # premia as 4 bordas juntas
        miolo = soma(ig, vv + m, vv + s - m, uu + m, uu + s - m) / float((s - 2 * m) ** 2)
        score = np.where(miolo <= cfg.borda_interior_max * fundo, score, -1.0)
        # centro do furo perto do eixo da tag (|x| <= 25 mm, como no metodo da cor)
        cx_mm = (uu + s / 2.0) / px + x0
        score = np.where(np.abs(cx_mm) <= 25.0, score, -1.0)
        i = int(np.argmax(score))
        sc = float(score.flat[i])
        if sc > 0.0 and (melhor is None or sc > melhor[0]):
            r, c = np.unravel_index(i, score.shape)
            melhor = (sc, float(pior[r, c]), uu[r, c] + s / 2.0, vv[r, c] + s / 2.0, float(s))
    if melhor is None:
        return None
    sc, pior, cx, cy, s = melhor
    return sc / fundo, pior / fundo, cx, cy, s


def _medir_furo_borda(bgr, c0, cfg, giro_cantos):
    """Furo pelas bordas (fundo de qualquer cor). Mesmo formato de saida do medir_furo."""
    ks = range(4) if giro_cantos is None else [int(giro_cantos)]
    melhor = None
    for k in ks:
        cantos = np.roll(c0, k, axis=0)
        q = _quadrado_por_borda(_retificar(bgr, cantos, cfg), cfg)
        if q is not None and (melhor is None or q[0] > melhor[1][0]):
            melhor = (k, q, cantos)
    out = {"ok": False, "motivo": "sem_furo", "completo": False,
           "giro_cantos": 0 if giro_cantos is None else int(giro_cantos), "metodo": "borda"}
    if melhor is None:
        return out
    k, (sc, pior, cx, cy, s), cantos = melhor
    x0, _x1, y0, _y1 = cfg.janela_mm
    cx_mm, cy_mm = cx / cfg.px_mm + x0, cy / cfg.px_mm + y0
    lado = s / cfg.px_mm
    completo = pior >= cfg.borda_contraste_min
    p = homografia_tag(cantos, cfg.lado_tag_mm) @ np.array([cx_mm, cy_mm, 1.0])
    out.update(ok=True, motivo="" if completo else "borda_fraca", completo=bool(completo),
               giro_cantos=k, centro_mm=(float(cx_mm), float(cy_mm)), lado_mm=(lado, lado),
               giro_graus=0.0, centro_px=(float(p[0] / p[2]), float(p[1] / p[2])),
               contraste=(float(sc), float(pior)))
    return out


def medir_furo(bgr, cantos_tag, cfg: Optional[FuroCfg] = None, giro_cantos: Optional[int] = None):
    """Furo do slot desta tag: pela cor, pela borda ou os dois (cfg.metodo)."""
    cfg = cfg or FuroCfg()
    c0 = np.asarray(cantos_tag, float).reshape(4, 2)
    if cfg.metodo == "borda":
        return _medir_furo_borda(bgr, c0, cfg, giro_cantos)
    f = _medir_furo_cor(bgr, c0, cfg, giro_cantos)
    if cfg.metodo == "auto" and not (f["ok"] and f["completo"]):
        # Na cor a orientacao vem do "lado com mais marrom"; sem marrom ela e'
        # aleatoria, entao a borda escolhe a sua (salvo se a memoria ja fixou).
        fb = _medir_furo_borda(bgr, c0, cfg, giro_cantos)
        if fb["ok"] and fb["completo"]:
            return fb
    return f


def _medir_furo_cor(bgr, cantos_tag, cfg, giro_cantos):
    """Furo do slot desta tag, medido na vista retificada pela propria tag.

    Devolve dict com ok/motivo e, se ok:
      centro_mm (x, y)  centro no frame da tag    lado_mm (lx, ly)
      giro_graus        bordas do furo em relacao a' tag
      completo          as 4 bordas sao do furo (nada tapando): so' entao o centro vale
      centro_px         centro na imagem          giro_cantos   ordem dos cantos usada
    """
    c0 = np.asarray(cantos_tag, float).reshape(4, 2)
    k = _orientar(c0, bgr, cfg) if giro_cantos is None else int(giro_cantos)
    cantos = np.roll(c0, k, axis=0)
    masc = _mascara(_retificar(bgr, cantos, cfg), cfg)
    n, lab, st, _ = cv2.connectedComponentsWithStats(masc, 8)
    x0, _x1, y0, y1 = cfg.janela_mm
    out = {"ok": False, "motivo": "sem_furo", "completo": False, "giro_cantos": k, "metodo": "cor"}
    melhor = None
    for i in range(1, n):
        area = st[i, cv2.CC_STAT_AREA] / cfg.px_mm ** 2
        base = (st[i, cv2.CC_STAT_TOP] + st[i, cv2.CC_STAT_HEIGHT]) / cfg.px_mm + y0
        cx = (st[i, cv2.CC_STAT_LEFT] + st[i, cv2.CC_STAT_WIDTH] / 2.0) / cfg.px_mm + x0
        # o furo comeca colado na tag (borda de baixo da janela) e fica na frente dela
        if area < cfg.area_min_mm2 or base < y1 - 14.0 or abs(cx) > 25.0:
            continue
        if melhor is None or area > melhor[1]:
            melhor = (i, area)
    if melhor is None:
        return out
    comp = (lab == melhor[0]).astype(np.uint8)
    cnt = max(cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
    (rx, ry), (rw, rh), ang = cv2.minAreaRect(cnt)
    ang = (ang + 45.0) % 90.0 - 45.0            # bordas do furo: giro em (-45, 45]
    box = cv2.boxPoints(((rx, ry), (rw, rh), ang))
    a = math.radians(ang)
    ex, ey = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    lx, ly = float(np.ptp(box @ ex)) / cfg.px_mm, float(np.ptp(box @ ey)) / cfg.px_mm
    cx_mm, cy_mm = rx / cfg.px_mm + x0, ry / cfg.px_mm + y0
    completo = (abs(lx - cfg.lado_furo_mm) <= cfg.lado_tol_mm and abs(ly - cfg.lado_furo_mm) <= cfg.lado_tol_mm
                and st[melhor[0], cv2.CC_STAT_TOP] > 1)
    p = homografia_tag(cantos, cfg.lado_tag_mm) @ np.array([cx_mm, cy_mm, 1.0])
    out.update(ok=True, motivo="" if completo else "parcial", completo=bool(completo),
               centro_mm=(float(cx_mm), float(cy_mm)), lado_mm=(lx, ly), giro_graus=float(ang),
               centro_px=(float(p[0] / p[2]), float(p[1] / p[2])))
    return out


def pose_da_tag(cantos, k_cam, cfg):
    """PnP da tag: (R, t, n) com x_camera = R x_tag + t (mm) e n = normal do tampo para a camera."""
    obj = np.float32([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]]) * (cfg.lado_tag_mm / 2.0)
    _ok, rvecs, tvecs, _err = cv2.solvePnPGeneric(
        obj, np.float32(cantos), np.asarray(k_cam, float).reshape(3, 3), None, flags=cv2.SOLVEPNP_IPPE)
    r, _ = cv2.Rodrigues(rvecs[0])           # a de menor erro de reprojecao vem primeiro
    t = np.asarray(tvecs[0], float).reshape(3)
    n = r[:, 2] if r[:, 2] @ t < 0 else -r[:, 2]
    return r, t, n


def erro_do_cubo(cantos, k_cam, vetor_furo_mm, q5, cfg: Optional[FuroCfg] = None):
    """Quanto o CUBO tem de andar para o fundo dele ficar em cima do centro do furo.

    cantos: da tag do slot, ja' na ordem do medir_furo (furo em -y).
    vetor_furo_mm: centro do furo no frame da tag (do medir_furo com completo=True).
    q5: junta do punho (rad). A camera gira com o punho: vista dela, a direcao
        radial do braco esta' em (q5 - pi) e a lateral em (q5 - pi/2) — relacao
        conferida contra 20 movimentos reais em 16/11.
    Devolve dict: lat_mm (+ = j1 crescendo), rad_mm (+ = para longe da base),
    alt_mm (fundo do cubo acima do tampo), giro_graus (quanto a garra tem de girar,
    + = anti-horario visto de cima) e pe_mm (onde o cubo esta', no frame da tag).
    """
    cfg = cfg or FuroCfg()
    cantos = np.asarray(cantos, float).reshape(4, 2)
    r, t, n = pose_da_tag(cantos, k_cam, cfg)
    fundo = np.asarray(cfg.fundo_cubo_cam_mm, float)
    alt = float((fundo - t) @ n)
    pe = r.T @ (fundo - alt * n - t)              # pe' do cubo no tampo, frame da tag
    e = np.array([vetor_furo_mm[0] - pe[0], vetor_furo_mm[1] - pe[1]])
    # Direcoes do braco no frame da tag, pela imagem: a_u/a_v = direcoes do tampo
    # que aparecem como +u/+v na imagem perto do cubo.
    hm = homografia_tag(cantos, cfg.lado_tag_mm)

    def px(p):
        w = hm @ np.array([p[0], p[1], 1.0])
        return w[:2] / w[2]

    p0 = px(pe[:2])
    jac = np.column_stack([px(pe[:2] + [1.0, 0.0]) - p0, px(pe[:2] + [0.0, 1.0]) - p0])
    a_u = np.linalg.solve(jac, [1.0, 0.0])
    a_v = np.linalg.solve(jac, [0.0, 1.0])
    a_u, a_v = a_u / np.linalg.norm(a_u), a_v / np.linalg.norm(a_v)
    rad = math.cos(q5 - math.pi) * (-a_u) + math.sin(q5 - math.pi) * a_v
    lat = math.cos(q5 - math.pi / 2.0) * (-a_u) + math.sin(q5 - math.pi / 2.0) * a_v
    d = cantos[1] - cantos[0]
    ang = math.degrees(math.atan2(d[1], d[0]))
    giro = -((ang - cfg.angulo_ref_graus + 180.0) % 360.0 - 180.0)
    return {"lat_mm": float(e @ lat), "rad_mm": float(e @ rad), "alt_mm": alt, "giro_graus": giro,
            "pe_mm": (float(pe[0]), float(pe[1]))}


@dataclass
class Memoria:
    """O que se sabe do slot desde que o furo foi visto inteiro (vale enquanto a base nao andar)."""
    vetor_mm: Optional[tuple] = None      # centro do furo no frame da tag
    giro_cantos: Optional[int] = None
    lado_mm: Optional[tuple] = None
    n: int = 0
    _amostras: list = field(default_factory=list)


def medir(bgr, k_cam, cantos_tag, q5, mem: Memoria, cfg: Optional[FuroCfg] = None):
    """Uma medida completa num quadro: furo (se estiver a' vista) + erro do cubo.

    furo = "visto" (inteiro neste quadro: atualiza a memoria), "pela_tag" (tapado:
    usa o vetor tag->furo guardado) ou "nao" (nunca foi visto: sem erro do cubo).
    """
    cfg = cfg or FuroCfg()
    f = medir_furo(bgr, cantos_tag, cfg, mem.giro_cantos)
    out = {"furo": "nao", "ok": False}
    if f["ok"] and f["completo"]:
        mem._amostras.append(f["centro_mm"])
        mem.vetor_mm = tuple(float(v) for v in np.median(np.array(mem._amostras), axis=0))
        mem.giro_cantos, mem.lado_mm, mem.n = f["giro_cantos"], f["lado_mm"], len(mem._amostras)
        out["furo"] = "visto"
    elif mem.vetor_mm is not None:
        out["furo"] = "pela_tag"
    if mem.vetor_mm is None:
        out["motivo"] = "furo_nunca_visto"
        return out
    cantos = np.roll(np.asarray(cantos_tag, float).reshape(4, 2), mem.giro_cantos, axis=0)
    out.update(erro_do_cubo(cantos, k_cam, mem.vetor_mm, q5, cfg))
    out.update(ok=True, vetor_mm=mem.vetor_mm)
    return out
