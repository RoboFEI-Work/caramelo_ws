#!/usr/bin/env python3
"""Visao do alinhamento cubo -> furo da mesa de precisao (PP), SEM ROS.

Por que existe (2026-11-14, arquitetura aprovada pelo operador): o place da PP
passa a ser OLHAR-MOVER-OLHAR com o cubo na garra. O erro e' medido NA MESMA
IMAGEM entre a aresta visivel do cubo e a borda do furo-alvo, entao a escala da
tag, a calibracao da camera e a queda do braco viram erro comum e saem da conta.
No teste de 14/11 (runs/run_20261114_112951) a tag do slot-alvo ficou tapada
pelo cubo e NENHUMA tag foi detectada na descida final: por isso a medida nao
depende de tag (elas so' ajudam na perspectiva e a vetar o furo do vizinho).

O que a camera do punho ve com o cubo na garra (640x480): garra e cubo
vermelhos no topo-centro, em posicao quase fixa (camera e garra sao rigidas);
a aresta de baixo da face frontal do cubo e, logo adiante dela (para baixo na
imagem), o furo marrom do tampo branco, com a parte de cima tapada pelo cubo.

O que este modulo mede (medir(), funcoes puras numpy+cv2):
  aresta A-B do cubo (px)  x  borda DISTANTE P-Q do furo (a que fica adiante
  do cubo, a unica inteira quando o cubo tapa o furo):
    lateral = (meio de PQ - meio de AB) ao longo da aresta   (+ = furo para B)
    avanco  = idem ao longo da normal (+ = furo adiante do cubo)
    yaw     = angulo de PQ em relacao a AB
  em px e em mm no plano do tampo. NAO e' o erro de controle: a aresta do cubo
  esta ACIMA do tampo (paralaxe), entao o valor certo de lateral/avanco com o
  cubo alinhado nao e' zero e muda com a altura. O laco deve subtrair a
  REFERENCIA ENSINADA (mesma medida com o cubo colocado certo, mesma altura).
  O yaw nao tem paralaxe (reta paralela ao tampo continua paralela).

A segmentacao do furo e' a do pp_hole_detector_node (segment_holes); aqui so'
entram a aresta do cubo, o refino subpixel das bordas e a conta do erro.

Conferir um quadro sem robo:
  pp_align_vision.py --image quadro.jpg [--out overlay.png]
"""
import dataclasses
import math
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

# Instalado ao lado em lib/manip_bringup (mesmo esquema do pp_hole_detector_node).
from pp_hole_detector_node import HoleConfig, segment_holes  # noqa: E402


@dataclass
class AlignParams:
    # Cubo/garra: vermelho pelo canal a* do Lab (8 bits, 128 = neutro). Medido em
    # 14/11: vermelho p5 >= 149, marrom do furo p95 <= 141, tampo <= 132.
    red_a_min: int = 146
    # 2026-11-14 (corrida 12h07): ha' cubos AZUIS; so' com o vermelho o modulo
    # dizia "sem_cubo" (2 de 3 places). Azul = b* baixo: medido b* 106-115 na face
    # do cubo azul contra 138-143 no marrom do furo e 130-132 no tampo.
    blue_b_max: int = 118
    # Onde a aresta de baixo do cubo pode estar (x0, y0, x1, y1 normalizados).
    # Camera e garra sao rigidas: nas duas pegas de 14/11 a aresta ficou em
    # x = 214..343, y = 178..184 px (640x480). A janela separa "cubo na garra" de
    # vermelho em outro lugar; cubo recem-solto debaixo da garra ainda passa.
    edge_roi: Tuple[float, float, float, float] = (0.20, 0.27, 0.72, 0.50)
    edge_min_px: float = 90.0           # comprimento da aresta (130 px em 14/11)
    edge_max_px: float = 180.0
    edge_max_slope: float = 0.20        # |dy/dx| da aresta (pega torta ate ~11 graus)
    # Furo: mesma segmentacao do detector; so' muda o que o cubo exige (ver _cfg_furo).
    hole: HoleConfig = field(default_factory=HoleConfig)
    # Lado da abertura. 45 mm = medida por homografia da auditoria (44,2-45,4 mm);
    # NAO foi medido com regua. So' entra na conversao px -> mm.
    hole_side_mm: float = 45.0
    # Furo-alvo: centro do blob a no maximo isto do meio da aresta, em comprimentos
    # de aresta. O vizinho fica a ~2,5 arestas de lado na descida final.
    gate_lateral: float = 0.8
    gate_avanco: float = 1.6
    # Tags dos slots (uma ao lado de cada furo). tag_size_mm = 0: o tamanho da tag
    # esta em disputa (0.041 x ~33 mm, doc secao 10.1), entao a tag so' da' a
    # PERSPECTIVA e a unidade vem do lado do furo. > 0 usa a tag como metrica.
    slot_tag_ids: Tuple[int, ...] = (101, 102, 103, 104, 105, 106, 107)
    tag_size_mm: float = 0.0
    # Centro do furo no frame da tag (_h_tag), em lados de tag. Medido em 14/11 com
    # 306 pares tag-furo de garra vazia e braco parado: mediana (0.02, 1.55); por
    # slot o y vai de 1.46 a 1.69 (adesivos colados desiguais), dai a tolerancia.
    hole_in_tag: Tuple[float, float] = (0.02, 1.55)
    hole_in_tag_tol: float = 0.6
    # Refino subpixel das bordas
    min_degrau: float = 6.0             # degrau minimo furo x tampo (niveis de cinza por px)
    min_degrau_cubo: float = 1.5        # idem no a* para a aresta do cubo (rampa: ~3/px sobre o furo)
    min_frac_borda: float = 0.5         # fracao dos perfis que tem de cair na reta


@dataclass
class AlignResult:
    ok: bool = False
    motivos: List[str] = field(default_factory=list)   # por que nao mediu / avisos
    tem_cubo: bool = False
    aresta: Optional[np.ndarray] = None     # 2x2 px: A, B (cubo acima de A->B na imagem)
    furo: Optional[np.ndarray] = None       # 4x2 px: perto-A, perto-B, Q (distante-B), P (distante-A)
    bordas_ok: List[bool] = field(default_factory=list)  # perto, lado B, distante, lado A
    lateral_px: float = float("nan")
    avanco_px: float = float("nan")
    yaw_px_deg: float = float("nan")
    lateral_mm: float = float("nan")
    avanco_mm: float = float("nan")
    yaw_deg: float = float("nan")
    mm_por_px: float = float("nan")         # escala local na borda distante
    fonte_escala: str = ""                  # tag_<id>, tag_<id>+furo, furo_4cantos, furo_aresta
    razao_furo_tag: float = float("nan")    # lado do furo em lados de tag (confere o tamanho da tag)
    confianca: float = 0.0                  # 0..1: fracao de perfis bons da pior borda usada


def _normal(d: np.ndarray) -> np.ndarray:
    """Normal a 90 graus de d no sentido x->y da imagem (aresta para a direita => para baixo)."""
    return np.array([-d[1], d[0]], dtype=float)


def _reta_subpixel(canal: np.ndarray, a, b, normal, sinal: float, raio: int, passo: int,
                   min_degrau: float):
    """Reta da borda entre a e b: perfis ao longo da normal, primeiro degrau forte
    com o sinal pedido (subpixel por parabola), ajuste Huber.

    Devolve (ponto, direcao, fracao de perfis sobre a reta) ou None.
    O PRIMEIRO degrau forte (e nao o maior) evita grudar na moldura azul do
    adesivo da tag, que fica a poucos px da borda do furo. passo = meia base da
    derivada: 1 para borda nitida (furo x tampo); 3 para a aresta do cubo sobre
    o furo, que e' uma rampa de ~7 px (medido em 14/11, ate' com o braco parado).
    """
    a, b, normal = (np.asarray(v, dtype=float) for v in (a, b, normal))
    d = b - a
    comp = float(np.linalg.norm(d))
    if comp < 6.0:
        return None
    n_am = max(8, int(comp * 0.7 / 3.0))
    base = a[None] + np.linspace(0.15, 0.85, n_am)[:, None] * d[None]   # longe dos cantos
    s = np.arange(-raio - passo, raio + passo + 1, dtype=float)
    px = base[:, None, :] + s[None, :, None] * normal[None, None, :]
    h, w = canal.shape[:2]
    dentro = ((px[..., 0] >= 0) & (px[..., 0] <= w - 1) & (px[..., 1] >= 0) & (px[..., 1] <= h - 1)).all(axis=1)
    prof = cv2.remap(canal, px[..., 0].astype(np.float32), px[..., 1].astype(np.float32),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).astype(float)
    g = sinal * (prof[:, 2 * passo:] - prof[:, :-2 * passo]) / (2.0 * passo)   # derivada em s[passo:-passo]
    pts = []
    for i in range(n_am):
        gi = g[i]
        if not dentro[i] or gi.max() < min_degrau:
            continue
        j = int(np.argmax(gi >= max(min_degrau, 0.5 * gi.max())))
        while j + 1 < len(gi) and gi[j + 1] > gi[j]:
            j += 1
        off = 0.0
        if 0 < j < len(gi) - 1:
            den = gi[j - 1] - 2.0 * gi[j] + gi[j + 1]
            if den < 0:
                off = 0.5 * (gi[j - 1] - gi[j + 1]) / den
        pts.append(base[i] + (s[j + passo] + off) * normal)
    if len(pts) < 4:
        return None
    pts = np.asarray(pts, dtype=np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    res = np.abs((pts[:, 0] - x0) * vy - (pts[:, 1] - y0) * vx)
    bons = pts[res <= 1.0]
    if len(bons) < 4:
        return None
    vx, vy, x0, y0 = cv2.fitLine(bons, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    return np.array([x0, y0], dtype=float), np.array([vx, vy], dtype=float), len(bons) / float(n_am)


def _cruza(r1, r2) -> Optional[np.ndarray]:
    """Intersecao de duas retas (ponto, direcao)."""
    (p1, d1), (p2, d2) = r1[:2], r2[:2]
    m = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]], dtype=float)
    if abs(np.linalg.det(m)) < 1e-9:
        return None
    k = np.linalg.solve(m, p2 - p1)
    return p1 + k[0] * d1


def aresta_do_cubo(bgr: np.ndarray, p: AlignParams):
    """Aresta de baixo do cubo na garra: (A, B, fracao de perfis bons, perfil) ou None.

    A silhueta vermelha (garra + cubo) termina, embaixo, na aresta inferior da
    face frontal do cubo: e' o trecho reto mais longo do contorno inferior do
    vermelho dentro da janela edge_roi. Os cantos A e B saem do cruzamento com
    as laterais inclinadas da face frontal. perfil = (xs, ys) do contorno
    inferior, para montar a exclusao do furo.
    """
    h, w = bgr.shape[:2]
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.int16)
    # "cor de cubo" = vermelho (a* alto) OU azul (b* baixo); o marrom do furo e o
    # tampo branco nao passam em nenhum dos dois. lab_a vira o canal do degrau
    # para as duas cores: a*-128 no vermelho, 128-b* no azul.
    red = cv2.morphologyEx(((lab[..., 1] >= p.red_a_min) | (lab[..., 2] <= p.blue_b_max)).astype(np.uint8),
                           cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    lab_a = np.clip(np.maximum(lab[..., 1] - 128, 128 - lab[..., 2]) + 128, 0, 255).astype(np.uint8)
    x0, y0, x1, y1 = (int(round(v)) for v in (p.edge_roi[0] * w, p.edge_roi[1] * h,
                                               p.edge_roi[2] * w, p.edge_roi[3] * h))
    sub = red[y0:y1, x0:x1]
    cols = np.flatnonzero(sub.any(axis=0))
    if len(cols) < p.edge_min_px:
        return None
    ys = (y1 - 1 - np.argmax(sub[::-1, cols], axis=0)).astype(float)   # ultima linha vermelha
    xs = (x0 + cols).astype(float)

    # Reta com mais colunas a <= 1.5 px (pares de colunas a cada 6 px).
    best, best_n = None, 0
    cand = np.arange(0, len(xs), 6)
    for i in cand:
        for j in cand[cand > i]:
            dx = xs[j] - xs[i]
            if dx < 40:
                continue
            m = (ys[j] - ys[i]) / dx
            if abs(m) > p.edge_max_slope:
                continue
            n_in = int(np.count_nonzero(np.abs(ys - (ys[i] + m * (xs - xs[i]))) <= 1.5))
            if n_in > best_n:
                best, best_n = (xs[i], ys[i], m), n_in
    if best is None:
        return None
    bx, by, m = best
    k_in = np.flatnonzero(np.abs(ys - (by + m * (xs - bx))) <= 1.5)
    canal = cv2.GaussianBlur(lab_a, (5, 5), 0)
    A0, B0 = (np.array([xs[k], by + m * (xs[k] - bx)]) for k in (k_in[0], k_in[-1]))
    t = (B0 - A0) / np.linalg.norm(B0 - A0)
    # Subpixel: a* cai de vermelho (dentro) para tampo/furo (fora, normal para baixo).
    reta = _reta_subpixel(canal, A0, B0, _normal(t), -1.0, 5, 3, p.min_degrau_cubo)
    if reta is None:
        return None
    m = reta[1][1] / reta[1][0]
    bx, by = reta[0]
    # Fim do trecho reto, do meio para cada lado: 4 colunas seguidas fora da reta.
    # Tolerancia de 4 px porque sobre o furo a aresta aparece ~2 px mais baixa do
    # que sobre o tampo (rampa vermelho->marrom); com menos, o degrau na parede
    # do furo cortava a aresta (quadros 00376/00377: canto A 7-25 px para dentro).
    sobe = (by + m * (xs - bx)) - ys          # quanto o contorno esta' acima da reta
    fins = []
    for lado in (-1, 1):
        k, seguidas = int(k_in[len(k_in) // 2]), 0
        while 0 <= k + lado < len(xs) and abs(xs[k + lado] - xs[k]) <= 3 and seguidas < 4:
            k += lado
            seguidas = seguidas + 1 if abs(sobe[k]) > 4.0 else 0
        fins.append(k - seguidas * lado)
    ka, kb = fins
    if not (p.edge_min_px <= xs[kb] - xs[ka] <= p.edge_max_px):
        return None
    if by + m * (xs[kb] - bx) >= y1 - 2 or by + m * (xs[ka] - bx) >= y1 - 2:
        return None                      # vermelho continua abaixo da janela: nao e' a aresta
    # Cantos: cruzamento com as laterais inclinadas da face frontal. A lateral e'
    # o contorno que sobe de 4 a 14 px logo alem do trecho reto, ate' o primeiro
    # salto (depois dele e' o dedo da garra).
    cantos = []
    for lado, kc in ((-1, ka), (1, kb)):
        ks = [kc]
        while 0 <= ks[-1] + lado < len(xs) and len(ks) <= 45 \
                and abs(xs[ks[-1] + lado] - xs[ks[-1]]) <= 2 and abs(ys[ks[-1] + lado] - ys[ks[-1]]) <= 3:
            ks.append(ks[-1] + lado)
        ks = [k for k in ks if 4.0 <= sobe[k] <= 14.0]
        fim_reto = np.array([xs[kc], by + m * (xs[kc] - bx)])
        canto = None
        if len(ks) >= 6:
            c0, c1 = (np.array([xs[k], ys[k]]) for k in (ks[0], ks[-1]))
            dl = (c1 - c0) / np.linalg.norm(c1 - c0)
            nl = _normal(dl) if np.dot(_normal(dl), _normal(t)) > 0 else -_normal(dl)   # para fora do cubo
            lat = _reta_subpixel(canal, c0 - 0.2 * (c1 - c0), c1 + 0.2 * (c1 - c0), nl, -1.0, 4, 2,
                                 p.min_degrau_cubo)
            canto = _cruza(reta, lat if lat is not None else (c0, dl))
        if canto is None or np.linalg.norm(canto - fim_reto) > 12:
            # shortcut: sem lateral visivel o canto e' o fim do trecho reto (+-4 px);
            # rever se uma pega diferente esconder a face frontal do cubo.
            canto = fim_reto
        cantos.append(canto)
    A, B = cantos
    if B[0] < A[0]:
        A, B = B, A
    return A, B, reta[2], (xs, ys)


def _cfg_furo(p: AlignParams, perfil, shape_hw) -> HoleConfig:
    """Config do segment_holes para furo sob o cubo: a exclusao vira a silhueta
    REAL do cubo (contorno inferior do vermelho), nao os poligonos fixos do YAML,
    que cortariam a faixa do furo colada na aresta. Furo cortado pelo cubo e' o
    caso normal aqui (accept_partial) e a faixa visivel e' comprida (proporcao)."""
    h, w = shape_hw
    xs, ys = perfil
    passo = max(1, len(xs) // 40)
    poly = [(xs[0] / w, 0.0)] + [(x / w, (y + 1.0) / h) for x, y in zip(xs[::passo], ys[::passo])] \
        + [(xs[-1] / w, (ys[-1] + 1.0) / h), (xs[-1] / w, 0.0)]
    return dataclasses.replace(
        p.hole, exclusion_rects=[], exclusion_polys=[], exclusion_polys_holding=[poly],
        accept_partial=True, max_aspect_ratio=30.0,
        # v_min 40 do YAML perde o fundo do furo na sombra do cubo (V 21-39 em 14/11)
        v_min=min(p.hole.v_min, 20))


def _tag_do_furo(centro, dets, p: AlignParams) -> Optional[int]:
    """Id da tag de slot cujo furo (hole_in_tag, em lados de tag) cai em centro."""
    for d in dets:
        if int(d["id"]) not in p.slot_tag_ids:
            continue
        ht = _h_tag(d)
        q = cv2.perspectiveTransform(np.asarray(centro, dtype=np.float32).reshape(1, 1, 2), ht).ravel()
        if math.hypot(q[0] - p.hole_in_tag[0], q[1] - p.hole_in_tag[1]) <= p.hole_in_tag_tol:
            return int(d["id"])
    return None


def _h_tag(det) -> np.ndarray:
    """Homografia imagem -> frame da tag em LADOS DE TAG (centro 0, x/y de -0.5 a 0.5),
    na ordem de cantos do apriltag_ros (/detections)."""
    quad = np.float32([[-0.5, 0.5], [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5]])
    return cv2.getPerspectiveTransform(np.asarray(det["corners"], dtype=np.float32), quad)


def _quad_do_furo(cinza: np.ndarray, box: np.ndarray, p: AlignParams):
    """Refina as 4 bordas do furo (escuro dentro, claro fora) a partir do retangulo
    do blob: (cantos 4x2, fracao de perfis bons por borda k = canto k -> k+1)."""
    quad = np.asarray(box, dtype=float)
    fr = [0.0] * 4
    for raio in (8, 4):                  # 2a passada parte dos cantos ja corrigidos
        c = quad.mean(axis=0)
        retas = []
        for k in range(4):
            a, b = quad[k], quad[(k + 1) % 4]
            d = (b - a) / max(np.linalg.norm(b - a), 1e-9)
            n = _normal(d)
            if np.dot(n, (a + b) / 2.0 - c) < 0:
                n = -n
            r = _reta_subpixel(cinza, a, b, n, 1.0, raio, 1, p.min_degrau)
            fr[k] = r[2] if r is not None else 0.0
            retas.append(r if r is not None else (a, d, 0.0))   # borda tapada: fica a do blob
        novo = [_cruza(retas[k - 1], retas[k]) for k in range(4)]
        if any(v is None for v in novo):
            break
        quad = np.asarray(novo, dtype=float)
    return quad, fr


def medir(bgr: np.ndarray, dets: List[dict], p: Optional[AlignParams] = None,
          alvo_id: Optional[int] = None, aresta: Optional[np.ndarray] = None) -> AlignResult:
    """Mede aresta do cubo x furo-alvo num quadro BGR.

    dets: [{"id": int, "corners": [[x, y] * 4]}] de /detections (pode ser []).
    alvo_id: tag do slot-alvo, se conhecida; com ela visivel o furo-alvo e' o
    dela, e o furo de OUTRA tag visivel e' vetado.
    aresta: A, B (2x2 px) ja' conhecidos da pega. O cubo e' rigido com a camera,
    entao a aresta nao muda entre quadros: a mediana de varios quadros sobre o
    tampo e' melhor que a leitura de um quadro so' (cantos: +-0,5 a 1,4 px por
    quadro em 14/11, e piores com o furo logo abaixo do canto).
    """
    p = p or AlignParams()
    res = AlignResult()
    ar = aresta_do_cubo(bgr, p)
    if ar is None:
        res.motivos.append("sem_cubo")
        return res
    A, B, fr_cubo, perfil = ar
    if aresta is not None:
        A, B = (np.asarray(v, dtype=float) for v in aresta)
    res.tem_cubo, res.aresta = True, np.stack([A, B])
    comp = float(np.linalg.norm(B - A))
    t = (B - A) / comp
    n = _normal(t)
    Mc = (A + B) / 2.0

    blobs, _hsv, _masks = segment_holes(bgr, _cfg_furo(p, perfil, bgr.shape[:2]), holding=True)
    cands = []
    for b in blobs:
        if b.reason is not None:
            continue
        v = np.asarray(b.center) - Mc
        lat, av = float(np.dot(v, t)), float(np.dot(v, n))
        dono = _tag_do_furo(b.center, dets, p)
        cands.append((b, lat, av, dono))
    alvo = None
    if alvo_id is not None:
        alvo = next((c for c in cands if c[3] == alvo_id), None)
    if alvo is None:
        perto = [c for c in cands if abs(c[1]) <= p.gate_lateral * comp and 0 < c[2] <= p.gate_avanco * comp]
        alvo = min(perto, key=lambda c: c[2], default=None)
    if alvo is None:
        res.motivos.append("sem_furo")
        return res
    if alvo_id is not None and alvo[3] is not None and alvo[3] != alvo_id:
        res.motivos.append(f"furo_do_vizinho(tag_{alvo[3]})")
        return res

    cinza = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    quad, fr = _quad_do_furo(cinza, alvo[0].box, p)
    # Reordena em relacao ao cubo: borda distante = a de maior avanco.
    meio = [(quad[k] + quad[(k + 1) % 4]) / 2.0 for k in range(4)]
    kd = int(np.argmax([np.dot(mk - Mc, n) for mk in meio]))
    c0, c1 = quad[kd], quad[(kd + 1) % 4]
    if np.dot(c1 - c0, t) > 0:          # P do lado de A, Q do lado de B
        P, Q, kP, kQ = c0, c1, (kd - 1) % 4, (kd + 1) % 4
        perto_A, perto_B = quad[(kd - 1) % 4], quad[(kd + 2) % 4]
    else:
        P, Q, kP, kQ = c1, c0, (kd + 1) % 4, (kd - 1) % 4
        perto_A, perto_B = quad[(kd + 2) % 4], quad[(kd - 1) % 4]
    res.furo = np.stack([perto_A, perto_B, Q, P])
    ok_b = [f >= p.min_frac_borda for f in fr]
    res.bordas_ok = [ok_b[(kd + 2) % 4], ok_b[kQ], ok_b[kd], ok_b[kP]]
    if not ok_b[kd]:
        res.motivos.append("borda_distante_tapada")
        return res
    if not (ok_b[kP] and ok_b[kQ]):
        # sem as duas laterais nao ha' canto real para o meio da borda
        res.motivos.append("lateral_do_furo_tapada")
        return res

    Mh = (P + Q) / 2.0
    res.lateral_px = float(np.dot(Mh - Mc, t))
    res.avanco_px = float(np.dot(Mh - Mc, n))
    u = (Q - P) / np.linalg.norm(Q - P)
    res.yaw_px_deg = math.degrees(math.atan2(t[0] * u[1] - t[1] * u[0], float(np.dot(t, u))))

    # Plano do tampo: perspectiva da tag de slot mais proxima, senao do proprio furo.
    def aplica(hm, pts):
        return cv2.perspectiveTransform(np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2), hm).reshape(-1, 2)

    slot = [d for d in dets if int(d["id"]) in p.slot_tag_ids]
    if slot:
        d = min(slot, key=lambda d: np.linalg.norm(np.mean(d["corners"], axis=0) - Mh))
        ht = _h_tag(d)
        pq = aplica(ht, [P, Q])
        res.razao_furo_tag = float(np.linalg.norm(pq[1] - pq[0]))
        if p.tag_size_mm > 0:
            k, res.fonte_escala = p.tag_size_mm, f"tag_{int(d['id'])}"
        else:
            k, res.fonte_escala = p.hole_side_mm / res.razao_furo_tag, f"tag_{int(d['id'])}+furo"
        hm = np.diag([k, k, 1.0]) @ ht
    elif all(ok_b):
        s = p.hole_side_mm
        hm = cv2.getPerspectiveTransform(res.furo.astype(np.float32),
                                         np.float32([[0, 0], [s, 0], [s, s], [0, s]]))
        res.fonte_escala = "furo_4cantos"
    else:
        k = p.hole_side_mm / float(np.linalg.norm(Q - P))
        hm = np.diag([k, k, 1.0])
        res.fonte_escala = "furo_aresta"
    mA, mB, mP, mQ, mMc, mN = aplica(hm, [A, B, P, Q, Mc, Mc + 10.0 * n])
    tm = (mB - mA) / np.linalg.norm(mB - mA)
    nm = mN - mMc
    nm = nm - np.dot(nm, tm) * tm
    nm = nm / np.linalg.norm(nm)
    mMh = (mP + mQ) / 2.0
    res.lateral_mm = float(np.dot(mMh - mMc, tm))
    res.avanco_mm = float(np.dot(mMh - mMc, nm))
    um = mQ - mP
    mao = 1.0 if (tm[0] * nm[1] - tm[1] * nm[0]) > 0 else -1.0     # homografia pode espelhar
    res.yaw_deg = mao * math.degrees(math.atan2(tm[0] * um[1] - tm[1] * um[0], float(np.dot(tm, um))))
    res.mm_por_px = float(np.linalg.norm(mQ - mP) / np.linalg.norm(Q - P))
    res.confianca = float(min(1.0, fr_cubo, fr[kd], fr[kP], fr[kQ]))
    res.ok = True
    return res


def desenhar(bgr: np.ndarray, res: AlignResult) -> np.ndarray:
    """Overlay de depuracao: aresta do cubo (amarelo), furo (verde = borda vista,
    vermelho = tapada), meios ligados (ciano) e os numeros."""
    out = bgr.copy()
    pt = lambda v: (int(round(v[0])), int(round(v[1])))  # noqa: E731
    if res.aresta is not None:
        cv2.line(out, pt(res.aresta[0]), pt(res.aresta[1]), (0, 255, 255), 2)
    if res.furo is not None:
        for k in range(4):
            cor = (0, 255, 0) if res.bordas_ok and res.bordas_ok[k] else (0, 0, 255)
            cv2.line(out, pt(res.furo[k]), pt(res.furo[(k + 1) % 4]), cor, 1)
    if res.ok:
        cv2.line(out, pt(res.aresta.mean(axis=0)), pt(res.furo[2:].mean(axis=0)), (255, 255, 0), 1)
        linhas = [f"lat {res.lateral_px:+.1f}px {res.lateral_mm:+.1f}mm  av {res.avanco_px:+.1f}px "
                  f"{res.avanco_mm:+.1f}mm  yaw {res.yaw_deg:+.1f}",
                  f"{res.mm_por_px:.3f} mm/px ({res.fonte_escala})  conf {res.confianca:.2f}"]
    else:
        linhas = ["sem medida: " + ", ".join(res.motivos)]
    for i, linha in enumerate(linhas):
        cv2.putText(out, linha, (6, out.shape[0] - 26 + 16 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(out, linha, (6, out.shape[0] - 26 + 16 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return out



# ---------------------------------------------------------------------------
# 2026-11-14: medida PELOS FUROS VIZINHOS. Com o cubo quase em cima do furo-alvo
# sobra so' uma lasca do furo (rejeitada por solidez em 2 de 4 places de 14/11),
# justamente o regime do ajuste fino. Os furos vizinhos (a um passo, ~109 mm)
# aparecem INTEIROS dos lados: eles dao a regua e a perspectiva do tampo, e o
# centro do furo-alvo sai da geometria da mesa (config/pp_table.yaml), mesmo
# com ele tapado. Tudo na mesma imagem: nao usa tag, TF nem calibracao.
# ---------------------------------------------------------------------------
def carregar_mesa(caminho: str) -> dict:
    """Le o pp_table.yaml (arquivo de calibracao da mesa, editavel na competicao)."""
    import yaml
    with open(caminho, "r", encoding="utf-8") as fh:
        mesa = yaml.safe_load(fh)["pp_table"]
    for chave in ("pitch_mm", "hole_side_mm", "cube_side_mm"):
        if not float(mesa[chave]) > 0.0:
            raise ValueError(f"pp_table.{chave} tem que ser > 0")
    return mesa


def erro_cubo_no_plano(vizinhos, A, B, mesa: dict) -> Optional[dict]:
    """Geometria pura. vizinhos = [(k, quad 4x2 px)]: furo a k passos do alvo
    (k != 0), cantos em qualquer ordem. A, B = aresta de baixo do cubo (px).
    Frame "mesa" (mm): origem no centro do FURO-ALVO, x ao longo da fileira no
    sentido A->B, y no sentido do avanco (do cubo para fora, para baixo na imagem).
    Devolve quanto o cubo tem de andar para ficar centrado no furo: erro_x_mm,
    erro_y_mm, yaw_deg (giro da aresta em relacao a' fileira), e o residuo da
    homografia. NAO corrige paralaxe: com o cubo acima do tampo a aresta aparece
    deslocada (proporcional a' altura); o laco subtrai a referencia ensinada."""
    A, B = np.asarray(A, float), np.asarray(B, float)
    t = (B - A) / np.linalg.norm(B - A)
    n = _normal(t)
    passo, lado = float(mesa["pitch_mm"]), float(mesa["hole_side_mm"])
    modelo, imagem = [], []
    for k, quad in vizinhos:
        quad = np.asarray(quad, float)
        c = quad.mean(axis=0)
        for canto in quad:
            u, v = np.dot(canto - c, t), np.dot(canto - c, n)
            modelo.append((k * passo + math.copysign(lado / 2.0, u), math.copysign(lado / 2.0, v)))
            imagem.append(canto)
    if len(modelo) < 4:
        return None
    modelo, imagem = np.array(modelo, np.float64), np.array(imagem, np.float64)
    H, _ = cv2.findHomography(modelo, imagem, 0)          # mesa (mm) -> imagem (px)
    if H is None:
        return None
    reproj = cv2.perspectiveTransform(modelo.reshape(-1, 1, 2), H)[:, 0]
    residuo = float(np.sqrt(np.mean(np.sum((reproj - imagem) ** 2, axis=1))))
    Am, Bm = cv2.perspectiveTransform(np.array([[A], [B]], np.float64), np.linalg.inv(H))[:, 0]
    e = (Bm - Am) / np.linalg.norm(Bm - Am)
    nm = np.array([-e[1], e[0]])
    if nm[1] < 0:
        nm = -nm                                          # normal para o lado do avanco (+y)
    centro = (Am + Bm) / 2.0 - nm * float(mesa["cube_side_mm"]) / 2.0   # cubo fica do lado -y da aresta
    return {"erro_x_mm": float(-centro[0]), "erro_y_mm": float(-centro[1]),
            "yaw_deg": float(math.degrees(math.atan2(e[1], e[0]))),
            "aresta_mm": float(np.linalg.norm(Bm - Am)), "residuo_px": residuo,
            "n_vizinhos": len(vizinhos)}


def medir_por_vizinhos(bgr: np.ndarray, mesa: dict, p: Optional[AlignParams] = None,
                       aresta: Optional[np.ndarray] = None) -> dict:
    """Erro cubo -> furo-alvo num quadro BGR usando os furos vizinhos inteiros.
    Devolve {"ok": bool, "motivos": [...], + campos de erro_cubo_no_plano}."""
    p = p or AlignParams()
    out = {"ok": False, "motivos": []}
    ar = aresta_do_cubo(bgr, p)
    if ar is None:
        out["motivos"].append("sem_cubo")
        return out
    A, B, _fr, perfil = ar
    if aresta is not None:
        A, B = (np.asarray(v, dtype=float) for v in aresta)
    t = (B - A) / np.linalg.norm(B - A)
    Mc = (A + B) / 2.0
    blobs, _hsv, _masks = segment_holes(bgr, _cfg_furo(p, perfil, bgr.shape[:2]), holding=True)
    cinza = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    passo, lado = float(mesa["pitch_mm"]), float(mesa["hole_side_mm"])
    vizinhos = []
    for b in blobs:
        if b.reason is not None or b.partial:
            continue                                      # so' furo inteiro serve de regua
        quad, fr = _quad_do_furo(cinza, b.box, p)
        if min(fr) < p.min_frac_borda:
            continue
        lados = [np.linalg.norm(quad[k] - quad[(k + 1) % 4]) for k in range(4)]
        if max(lados) > 1.35 * min(lados):
            continue                                      # nao e' um quadrado: cubo/sombra em cima
        mm_px = lado / float(np.mean(lados))              # escala local so' para contar os passos
        k = int(round(float(np.dot(quad.mean(axis=0) - Mc, t)) * mm_px / passo))
        if k != 0 and abs(k) <= 2:
            vizinhos.append((k, quad))
    if not vizinhos:
        out["motivos"].append("sem_vizinho")
        return out
    res = erro_cubo_no_plano(vizinhos, A, B, mesa)
    if res is None:
        out["motivos"].append("homografia")
        return out
    out.update(res)
    # A aresta medida no plano tem de ter o tamanho do cubo: confere regua + aresta.
    if abs(res["aresta_mm"] - float(mesa["cube_side_mm"])) > 0.25 * float(mesa["cube_side_mm"]):
        out["motivos"].append(f"aresta_{res['aresta_mm']:.0f}mm")
        return out
    out["ok"] = True
    return out


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Mede aresta do cubo x furo num quadro (sem tags).")
    ap.add_argument("--image", required=True)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    img = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if img is None:
        sys.exit(f"nao consegui abrir {args.image}")
    r = medir(img, [])
    print({k: v for k, v in dataclasses.asdict(r).items() if k not in ("aresta", "furo")})
    if args.out:
        cv2.imwrite(args.out, desenhar(img, r))
