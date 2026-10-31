#!/usr/bin/env python3
"""Alinhamento fino HOLONOMICO a um dock e ajuste lateral da base (fluxo v5).

Serve duas actions (caramelo_msgs) no mesmo no', nunca as duas ao mesmo tempo:
  - align_to_dock: a partir do pre-docking (staging do opennav_docking), faz
    (A) UM ajuste lateral pelo centro da face no /scan, (B) RETO no piso do
    ESC guiado pelo muro LiDAR ate a distancia de parada (corte descontando o
    coast), com UMA pausa a ~refine_refit_front_m para refazer o fit da face
    isolada e esquadrar pelo LiDAR, (C) retoque lateral na mesa e (D)
    validacao final de yaw pela face (2026-10-21): fora da tolerancia, recua
    ate a distancia de giro, esquadra pela face e refaz reto+lateral uma vez;
    ainda fora => 'yaw_fora_da_tolerancia' (sucesso com WARN, ou falha com
    final_yaw_strict). 2026-10-26 (tours 19:45/19:52, WS1/WS2/WS3/WS6
    tortas): espera a base PARAR antes de medir; esquadro pelo LiDAR no
    staging ANTES do lateral e centro pelo pe' da perpendicular; esquadro
    por AMCL DESLIGADO por default (amcl_square_enabled) e, ligado, so' sem
    face nenhuma, base parada, |erro| <= amcl_square_max_err (0,20 rad) e
    sem veto do LiDAR; o lateral do staging centra pelo eixo (c_axis) e so'
    loga o pe' da perpendicular (c_foot);
    tentativas = approach_retries+1 (0 no launch = uma so'); so' recua na
    falha se parou perigosamente perto. Clientes: GoToWS (manip_bt) e GUI.
    2026-10-27 (camera_*): a RealSense do punho (braco em home) mede yaw e
    centro da mesa (caramelo_dock_camera.py) e MANDA no staging (corrige por
    odometria, remedindo); na pausa do reto, no lateral da mesa e no yaw
    final vale a fusao camera+LiDAR. Fase de feedback "camera" (parado
    medindo). Sem medida da camera = o fluxo LiDAR acima.
  - nudge_base: deslocamento LATERAL puro (+ = esquerda) em malha fechada por
    ODOMETRIA, com as mesmas guardas (muro, faixa lateral, scan velho,
    watchdog de progresso). Clientes: NudgeBase BT (manip_bt nudge_base_bt,
    via reach_queue/bt_yaml_executor) e amt1_sort_action_node.
Publica TwistStamped em reference_topic, /manip/speech (aviso antes de mover) e
/manip/base_shift_total (latched: total lateral desde o ultimo align); assina
scan_topic e odom_topic. Parametros-chave: min_body_vel/omega (piso do ESC),
floor_start_margin (partida do repouso), hard_min_face_dist (muro), *_coast_*,
lateral_min_front, nudge_*. Valores vigentes: launch/docking_server.launch.py;
historico e medicoes: docs/docking_alinhamento_holonomico.md.
"""
import collections
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float64, String
import tf2_ros

# Helpers compartilhados de docking (carregam docking.yaml por pasta de mapa).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from caramelo_docking_common import (  # noqa: E402
    docking_file, load_yaml, resolve_map_folder, service_areas_file)
import caramelo_dock_camera as dock_cam  # noqa: E402
try:  # imagem de debug (cv2); sem cv2 o docking segue sem a imagem
    import caramelo_dock_camera_debug as dock_cam_dbg  # noqa: E402
except ImportError:  # pragma: no cover
    dock_cam_dbg = None
import numpy as np  # noqa: E402

from caramelo_msgs.action import AlignToDock, NudgeBase  # noqa: E402


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quat(z, w, x=0.0, y=0.0):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _lateral_displacement(x0, y0, yaw0, x, y):
    """Projecao do deslocamento (x0,y0)->(x,y) no eixo +y INICIAL do robo
    (yaw0 no frame da odometria), em m: + = andou para a ESQUERDA (eixo +y
    de base_footprint). Funcao pura, testavel sem ROS — e a medida de curso
    do nudge_base (2026-08-28)."""
    return -math.sin(yaw0) * (x - x0) + math.cos(yaw0) * (y - y0)


def _longitudinal_displacement(x0, y0, yaw0, x, y):
    """Projecao no eixo +x INICIAL do robo (+ = frente). So para o log do
    nudge: um lateral puro deveria dar ~0 aqui."""
    return math.cos(yaw0) * (x - x0) + math.sin(yaw0) * (y - y0)


def _nudge_accepts(dy, travelled, slack):
    """Regra de sucesso do nudge: andou (com o sinal certo) pelo menos
    |dy| - slack. Pura, testavel."""
    sign = 1.0 if dy > 0.0 else -1.0
    return travelled * sign >= abs(dy) - slack


def _nudge_request_valid(dy, timeout):
    """Goal bem formado (achado 4): NaN/inf em dy passariam nas checagens de
    faixa (NaN nunca e menor nem maior que nada) e o curso so acabaria por
    timeout; timeout NaN/inf idem. Pura, testavel."""
    return math.isfinite(dy) and math.isfinite(timeout)


def _front_after_lateral(front, dy, face_yaw):
    """Pior caso do front DEPOIS de andar |dy| de lado ao longo de uma face
    cuja normal faz face_yaw com o eixo x do robo: o bico avanca
    |dy|*sin(|face_yaw|) (0,25 m a 5 graus = 2,2 cm; os pousos ficam a
    0,365-0,39 m com hard_min 0,35). Usa |face_yaw| de proposito: nao
    depende do sentido do lateral (conservador). Pura, testavel (achado 1)."""
    return front - abs(dy) * math.sin(abs(face_yaw))


def _nudge_corrected(raw_lateral, gain):
    """Lateral medido pela odometria x ganho de escorregamento (achado 2).
    O /odom do EKF integra so a velocidade das rodas (odom0_config funde
    vx, vy, vyaw) e nao ve o escorregamento lateral (~18% no campo 21/08):
    o bruto SUPERESTIMA o curso. gain = real/odom, calibrado com trena
    (esperado ~0,82); 1.0 = sem correcao. Pura, testavel."""
    return raw_lateral * gain


def _nudge_cut_reached(mag, travelled, sign, coast, v_cmd, odom_age, period):
    """Criterio de corte do curso lateral (achado 5): o que falta
    (mag - travelled*sign) ja cabe no coast do ESC MAIS o que a base anda, a
    velocidade COMANDADA v_cmd (piso x margem de partida), enquanto a medida
    envelhece (odom_age) e ate a proxima decisao (period) — a mesma conta da
    reta do align (lag = v*(idade + periodo)). Pura, testavel."""
    return mag - travelled * sign <= coast + v_cmd * (odom_age + period)


def _floor_start_speed(floor, margin):
    """Velocidade de PARTIDA a partir do repouso = piso x (1 + margin).
    O piso de corpo (0.1215 m/s / 0.315 rad/s) convertido em roda pelo
    mecanum_drive_controller da Pi (raio 0.05, projecao 0.3855) da'
    2.4299999999999997 rad/s — UM ULP ABAIXO do piso do ESC (2.43). O
    driver (maxon_motors_node) tem histerese: LIGAR exige |cmd| >= piso;
    so' DESLIGA abaixo de 0.4 x piso. Logo quem parte do repouso exatamente
    no piso nunca move as rodas (campo 2026-10-19: nudge "andou +0.000 m");
    ja' em movimento (docking: parte com kp*erro > piso) o piso exato segue
    valendo. O giro idem: 0.315 x 0.3855 / 0.05 = 2.42865 < 2.43.
    Margem negativa ou NaN/inf reproduziria o bug em silencio (partida
    ABAIXO do piso): vale 0 (piso exato). Pura, testavel."""
    margin = float(margin)
    if not math.isfinite(margin) or margin < 0.0:
        margin = 0.0
    return float(floor) * (1.0 + margin)


def _nudge_no_progress(elapsed_s, travelled_m, min_s, min_m):
    """Watchdog de progresso do curso lateral (2026-10-19): comandando ha'
    pelo menos min_s e |travelled| < min_m = a base nao respondeu (roda
    sub-piso, outro publicador na referencia, reference_timeout do
    controller por relogio PC x Pi). Pura, testavel."""
    return elapsed_s >= min_s and abs(travelled_m) < min_m


class _OdomSettle:
    """Assentamento por ODOMETRIA do nudge (achado 3): num lateral puro o
    front NAO muda, entao o _settle do docking (front estavel em 3 scans)
    devolveria enquanto o ESC ainda retem a referencia (~0,5 s) e a medida
    sairia com a base em movimento. Assentado = o lateral ficou dentro de
    eps da amostra que abriu a janela por pelo menos min_s (>= retencao do
    ESC); qualquer amostra fora de eps reabre a janela nela. Comparar com a
    ancora (e nao so com a amostra anterior) evita que um rastejo de 2-3 mm
    por amostra a 50 Hz passe por parado. Pura, testavel:
    update(t, lateral) -> True quando assentou."""

    def __init__(self, eps, min_s):
        self.eps = float(eps)
        self.min_s = float(min_s)
        self.anchor_t = None
        self.anchor_lat = None

    def update(self, t, lateral):
        if self.anchor_lat is None or abs(lateral - self.anchor_lat) >= self.eps:
            self.anchor_t = t
            self.anchor_lat = lateral
            return False
        return t - self.anchor_t >= self.min_s


def _fit_line_tls(points):
    """Ajuste de reta por minimos quadrados totais (2x2) sem numpy.

    Retorna (cx, cy, nx, ny) = ponto medio + vetor NORMAL unitario da reta,
    ou None se os pontos forem poucos/degenerados. Usado para estimar a face
    plana da mesa (distancia e orientacao)."""
    n = len(points)
    if n < 8:
        return None
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    # Autovetor do menor autovalor da covariancia = direcao NORMAL da reta.
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    # theta e' a direcao principal (ao longo da reta); a normal e' perpendicular.
    nx = -math.sin(theta)
    ny = math.cos(theta)
    return cx, cy, nx, ny


# ---------------------------------------------------------------------------
# Revisao 2026-10-21 (run da WS1: docking "pronto" com yaw_face -19,7 graus).
# Helpers puros das decisoes de yaw do docking, testaveis sem ROS
# (test/test_nudge_math.py).

# Resultado de um fit da face: ok=False traz o motivo em `reason`.
_FaceFit = collections.namedtuple(
    "_FaceFit", "ok reason dist yaw width residual npts")

# Custo estimado (s) do ciclo de correcao final de yaw (recuo ~0,2 m +
# refit + giro + reto + lateral + medida): so' corrige se ainda sobrar
# isto do align_timeout (o executor passa 90 s; o docking normal leva ~7).
_YAW_CORRECTION_EST_S = 20.0


def yaw_needs_correction(face_yaw, tol):
    """|yaw da face| acima da tolerancia. Sem medida (None/NaN) nao ha o
    que corrigir: False — quem chama trata 'yaw_nao_medido' a parte."""
    if face_yaw is None or not math.isfinite(face_yaw):
        return False
    return abs(face_yaw) > tol


def refit_trigger(front, refit_front_m, already_done, coast=0.0, lag=0.0):
    """Hora de pausar o reto para refazer o fit da face: UMA vez por
    aproximacao, com medicao do muro e refit_front_m > 0 (<= 0 desliga).
    O corte desconta o coast do ESC e o atraso da medida (lag), como o
    corte final da reta, para o robo PARAR em ~refit_front_m — acima de
    rotate_min_front, senao nao ha como esquadrar depois do refit (com
    coast 0,10 uma pausa em 0,55 lido pararia em 0,45 < 0,46)."""
    if already_done or front is None or refit_front_m <= 0.0:
        return False
    return front - lag - coast <= refit_front_m


def face_fit_reject_reason(residual, width, min_w, max_w, max_residual):
    """Motivo de rejeicao de um fit da face ('' = aceito). Largura acima
    de max_w vem PRIMEIRO: e' mesa + vizinho (run da WS1: 0,98 m a 0,68 m
    da face, residuo 61 mm) e nesse caso o residuo alto e' consequencia —
    o chamador usa o motivo para NAO esquadrar pelo AMCL no staging."""
    if width > max_w:
        return "face larga demais (vizinho?)"
    if residual > max_residual:
        return "residuo alto (face nao-plana?)"
    if width < min_w:
        return "face estreita demais (perna/objeto?)"
    return ""


def face_fit_acceptable(residual, width, min_w, max_w, max_residual=0.02):
    """Fit confiavel: residuo <= max_residual e largura em [min_w, max_w]."""
    return face_fit_reject_reason(residual, width, min_w, max_w, max_residual) == ""


def final_yaw_decision(face_yaw, tol, corrected_once, strict, min_correctable=0.0):
    """Decisao no fim do docking, com o yaw da face MEDIDO (nao None):
      'ok'                dentro da tolerancia;
      'corrigir'          fora, ainda sem correcao neste goal e acima do que
                          o giro consegue corrigir (min_correctable = coast
                          do giro + folga: abaixo disso o _cut_drive nem
                          parte, e recuar seria tempo perdido);
      'aceitar_com_aviso' fora e sem como corrigir, strict desligado
                          (competicao: nao abortar a estacao);
      'falhar'            idem com strict ligado."""
    if not yaw_needs_correction(face_yaw, tol):
        return "ok"
    if not corrected_once and abs(face_yaw) > min_correctable:
        return "corrigir"
    return "falhar" if strict else "aceitar_com_aviso"


# ---------------------------------------------------------------------------
# Revisao 2026-10-26 (tours 19:45/19:52: WS1/WS2/WS3/WS6 tortas e fora do
# centro; docs/diagnostico_docking_tour_2026-10-26.md, fatos F12-F15/F20,
# causas C1-C3). Helpers puros das decisoes novas do align, testaveis sem
# ROS (test/test_nudge_math.py).

# Ciclos de "correcao final de yaw" (recuo + refit + reto + lateral) por
# goal. F12: 6/6 correcoes dos tours foram inocuas porque o refit(correcao)
# reprovou e o ciclo so' recuou e voltou; agora um ciclo sem giro NAO marca
# corrected_once (I8), e este teto evita repetir o recuo cego ate o timeout.
# Mais de 1 so' compensa depois de o refit aceitar faces estreitas (R1).
_YAW_CORRECTION_MAX_CYCLES = 1

# Folga acima do coast de giro (yaw_coast_est) abaixo da qual o _cut_drive em
# z nem parte: yaw_min_corr = yaw_coast_est + _YAW_MIN_CORR_MARGIN.
_YAW_MIN_CORR_MARGIN = 0.02
# Giro maximo pelo AMCL (rad) quando amcl_square_enabled=true. Revisao de
# 2026-10-26: com 0,12 a janela (yaw_min_corr, max_err] era VAZIA
# (yaw_coast_est 0,10 + 0,02 = 0,12) e o esquadro ligado nunca rodava.
# 0,20 (~11,5 graus) deixa a janela 6,9..11,5 graus; acima disso o dado do
# AMCL, que ja' esteve 7-8 graus enviesado na chegada (F14), nao manda.
_AMCL_SQUARE_MAX_ERR_DEFAULT = 0.20


def amcl_square_window(yaw_coast, max_err):
    """Janela de giro do esquadro por AMCL: (yaw_min_corr, max_err]. Devolve
    (yaw_min_corr, aberta). Fechada (max_err <= yaw_min_corr) = o esquadro
    por AMCL nunca gira, mesmo ligado — o no' avisa no init. Pura."""
    yaw_min_corr = float(yaw_coast) + _YAW_MIN_CORR_MARGIN
    return yaw_min_corr, float(max_err) > yaw_min_corr


def correction_counted(status):
    """I8 (F12): a correcao final de yaw so' conta como feita (corrected_once)
    se o refit_square('correcao') devolveu 'girou' — recuo + reto sem giro
    nao corrige nada. Pura, testavel."""
    return status == "girou"


def merge_refine_results(bank_target, first, second):
    """I2 (F13, WS1-T2): o refino do staging roda duas vezes e a 2a chamada
    sobrescrevia `confident` e o alvo — um 2o fit rejeitado (residuo 21 mm)
    jogava fora o alvo que a 1a acabara de refinar (-7,6 graus aplicados) e
    disparava o esquadro por AMCL contra ele. Regra: confiante se QUALQUER
    uma foi; alvo = o da ULTIMA confiante; nenhuma = o do banco.
    first/second = (alvo, confiante). Pura, testavel."""
    t1, c1 = first
    t2, c2 = second
    if c2:
        return t2, True
    if c1:
        return t1, True
    return bank_target, False


def amcl_square_veto(eyaw0, lidar_yaws, agree_tol, yaw_tol, max_eyaw):
    """I1 (F13/C1): motivo para NAO esquadrar pelo AMCL ('' = pode girar).
    eyaw0 = erro de yaw pelo AMCL (alvo - robo, rad; + = girar CCW), ja'
    lido com a base parada; lidar_yaws = yaw_rel da face pelo LiDAR nas
    leituras do staging (os fits REJEITADOS tambem trazem o yaw).
      - sem eyaw0 (AMCL/TF ausente): nao gira;
      - duas ultimas leituras do LiDAR concordando (agree_tol): se
        |yaw_rel| <= yaw_tol o robo JA' esta' esquadrado pela face, e se o
        sinal discorda de eyaw0 o AMCL pede o sentido ERRADO (WS6 nos dois
        tours: -9,7/-9,4 CW contra +5/+6 CCW do LiDAR) — nao gira (o veto
        do LiDAR vem ANTES do limite: e' o motivo que interessa no log);
      - |eyaw0| > max_eyaw (amcl_square_max_err, 0,20 rad): giro grande
        demais para um dado que
        ja' esteve 7-8 graus enviesado na chegada (F14) — nao gira.
    Leituras discordantes ou menos de duas: sem veto pelo LiDAR (quem
    segura esse caso e' a guarda 'so' sem face nenhuma' do chamador).
    Pura, testavel."""
    if eyaw0 is None or not math.isfinite(eyaw0):
        return "sem erro de yaw pelo AMCL"
    yaws = [y for y in (lidar_yaws or []) if y is not None and math.isfinite(y)]
    if len(yaws) >= 2:
        y1, y2 = yaws[-2], yaws[-1]
        if abs(_normalize_angle(y1 - y2)) <= agree_tol:
            yaw_rel = 0.5 * (y1 + y2)
            if abs(yaw_rel) <= yaw_tol:
                return (f"LiDAR ve a face a {math.degrees(yaw_rel):+.1f} graus "
                        f"(dentro da tolerancia {math.degrees(yaw_tol):.1f})")
            if (yaw_rel > 0.0) != (eyaw0 > 0.0):
                return (f"LiDAR pede o sentido oposto ({math.degrees(yaw_rel):+.1f} "
                        f"graus vs AMCL {math.degrees(eyaw0):+.1f})")
    if abs(eyaw0) > max_eyaw:
        return (f"|erro| {math.degrees(abs(eyaw0)):.1f} graus > limite "
                f"{math.degrees(max_eyaw):.1f}")
    return ""


def median_if_tight(vals, min_n, max_spread):
    """I1c: mediana de >= min_n amostras com dispersao (max-min) <=
    max_spread; senao None (erro do AMCL ainda mudando = nao gira). Pura."""
    vals = sorted(v for v in vals if v is not None and math.isfinite(v))
    if len(vals) < min_n or vals[-1] - vals[0] > max_spread:
        return None
    return vals[len(vals) // 2]


def yaw_readings_stable(yaws, n, eps):
    """I3c (F14/C2): as ultimas n leituras do yaw da face (fit leve) dentro
    de eps (max-min). Menos de n leituras = ainda nao. Pura."""
    if n <= 0 or len(yaws) < n:
        return False
    tail = yaws[-n:]
    return max(tail) - min(tail) <= eps


class _StillWindow:
    """Base PARADA por odometria (I1c/I3c; F14: a TF ainda gira 6,7-7,8
    graus nos 0,4 s depois de o Nav2 declarar chegada): |v| <= v_max e
    |wz| <= w_max continuamente por min_s. update(t, v, wz) -> True quando
    fechou; uma amostra em movimento reabre a janela. Pura, testavel."""

    def __init__(self, v_max, w_max, min_s):
        self.v_max = float(v_max)
        self.w_max = float(w_max)
        self.min_s = float(min_s)
        self.since = None

    def update(self, t, v, wz):
        if abs(v) > self.v_max or abs(wz) > self.w_max:
            self.since = None
            return False
        if self.since is None:
            self.since = t
        return t - self.since >= self.min_s


def lateral_cut_reached(err, sign, coast, v_cmd, scan_age, period):
    """R6 (F15/C3): corte do curso lateral pelo ponto de parada PREVISTO,
    como _nudge_cut_reached e a reta: o que falta (err*sign) ja' cabe no
    coast MAIS o que a base anda, a v_cmd, enquanto a medida envelhece
    (scan_age) e ate a proxima decisao (period). Antes cortava so' em
    err <= coast e o ESC retinha a referencia ~0,5 s: sobrecurso ADITIVO de
    5-10 cm (WS3: demanda +14,1 -> remedida -7,6 cm). Pura, testavel."""
    return err * sign <= coast + v_cmd * (scan_age + period)


def plan_rot_pulse(remaining, c, k, t_min, t_max, tol, max_over=1.5):
    """Giro em PULSOS (2026-10-27): duracao do proximo pulso para girar
    `remaining` rad com o modelo giro(T) = c + k*T (c = o que a base anda de
    rampa/coast mesmo num pulso minimo; k = rad por segundo de pulso).
    Devolve (T, "") ou (None, motivo): dentro da tolerancia, ou ate o pulso
    minimo passaria do ponto (giro minimo > max_over * |remaining| = o
    residuo do outro lado sairia maior que ~50% do erro). Pura, testavel."""
    mag = abs(remaining)
    if mag <= tol:
        return None, "dentro"
    d_min = c + k * t_min
    if d_min > max_over * mag:
        return None, "pulso minimo grande demais"
    t = (mag - c) / k if k > 1e-6 else t_max
    return min(t_max, max(t_min, t)), ""


def update_rot_model(c, k, t, rot, t_min, k_min=0.05):
    """Aprende o modelo do pulso com o giro MEDIDO `rot` (rad, ja' no
    sentido do pulso; >0 = girou para onde devia). Pulso MINIMO: o que
    girou alem de k*t e' rampa/coast -> ajusta c (o tamanho do menor giro
    possivel); pulso maior -> ajusta k. Media 50/50 com o anterior: um
    pulso fora da curva nao estraga o modelo. Pura."""
    if rot <= 0.0:
        return c, k
    if t <= t_min + 1e-6 or rot <= c:
        return max(0.0, 0.5 * c + 0.5 * (rot - k * t)), k
    return c, max(k_min, 0.5 * k + 0.5 * (rot - c) / t)


def refit_square_decision(first, second, agree_tol, yaw_tol, yaw_min_corr,
                          front, rotate_min):
    """Decisao do refit_square (I8/F12): (yaw_rel|None, status, motivo).
    status 'girar' = esquadrar por yaw_rel; os outros NAO giram:
    'sem_fits' (fit rejeitado ou sem 2a leitura), 'discordam', 'dentro'
    (|yaw_rel| <= yaw_tol), 'coast' (abaixo do que o giro corrige), 'perto'
    (front < rotate_min: o bico raspa a mesa). Pura, testavel."""
    if first is None or not first.ok:
        return None, "sem_fits", "sem scan" if first is None else first.reason
    if second is None or not second.ok:
        return None, "sem_fits", "sem /scan novo" if second is None else second.reason
    if abs(_normalize_angle(first.yaw - second.yaw)) > agree_tol:
        return None, "discordam", (
            f"leituras discordam ({math.degrees(first.yaw):+.1f} vs "
            f"{math.degrees(second.yaw):+.1f} graus, tol {math.degrees(agree_tol):.1f})")
    yaw_rel = 0.5 * (first.yaw + second.yaw)
    if not yaw_needs_correction(yaw_rel, yaw_tol):
        return yaw_rel, "dentro", f"dentro da tolerancia ({math.degrees(yaw_tol):.1f})"
    if abs(yaw_rel) <= yaw_min_corr:
        return yaw_rel, "coast", (
            f"fora da tolerancia mas abaixo do coast de giro "
            f"({math.degrees(yaw_min_corr):.1f})")
    if front is None or front < rotate_min - 1e-6:
        return yaw_rel, "perto", (
            f"fora da tolerancia mas front={'?' if front is None else '%.3f' % front} "
            f"< rotate_min_front {rotate_min:.2f}")
    return yaw_rel, "girar", ""


def parse_dock_entry(data, dock_id, docking_path=""):
    """R3: metadados de um dock do docking.yaml -> (x, y, yaw, frame,
    desired_face_dist|None, staging_offset, refine_half_width_y|None).
    `refine_half_width_y` e' OPCIONAL por dock (2026-10-26, F20: na WS6 um
    bloco/parede a 0,42 m a esquerda do eixo cai dentro da janela global de
    0,45 m e o refino do staging reprovava por residuo 46-53 mm; 0,35 deixa
    o bloco fora). Ausente, <= 0 ou invalido = None (o no' usa o parametro
    global). O opennav_docking le' o mesmo arquivo e ignora chaves extras
    (utils.hpp parseDockFile: so' frame/type/pose/id). Pura, testavel."""
    docks = data.get("docks", {}) or {}
    entry = docks.get(dock_id.strip().upper())
    if entry is None:
        raise KeyError(f"dock '{dock_id}' nao esta em {docking_path}")
    pose = entry.get("pose", [0.0, 0.0, 0.0])
    face_dist = entry.get("desired_face_dist")
    face_dist = float(face_dist) if face_dist is not None else None
    staging_offset = 0.45
    plugin = (data.get("dock_plugins", {}) or {}).get(entry.get("type", ""), {})
    if isinstance(plugin, dict) and "staging_x_offset" in plugin:
        staging_offset = abs(float(plugin["staging_x_offset"]))
    half_w = entry.get("refine_half_width_y")
    try:
        half_w = float(half_w) if half_w is not None else None
    except (TypeError, ValueError):
        half_w = None
    if half_w is not None and not (math.isfinite(half_w) and half_w > 0.0):
        half_w = None
    return (float(pose[0]), float(pose[1]), float(pose[2]),
            entry.get("frame", "map"), face_dist, staging_offset, half_w)


def face_center_from_points(pts, front, hy, edge_margin, max_residual, wmin, wmax):
    """Centro da face a partir dos pontos do scan JA' no base_frame
    (matematica de _face_center_from_scan, extraida em 2026-10-26 para
    teste; o docstring do metodo explica as guardas). Devolve
    (c_axis, c_foot, largura, '') ou (None, None, None, motivo):
      c_axis = desvio do centro da face em relacao ao ponto onde o EIXO do
               robo (+x) fura a face — o que a reta acerta SEM esquadrar;
      c_foot = desvio em relacao ao PE' DA PERPENDICULAR do centro do robo
               a reta da face — o que a reta acerta DEPOIS de um esquadro
               (I4, F15/C3: com yaw theta e face a d, c_axis carrega
               d*tan(theta): 0,65 m a 7 graus = 8 cm; c_foot nao).
    Identidade: c_foot = c_axis - d*tan(face_yaw). +y = esquerda."""
    deep = [p for p in pts
            if front - 0.03 <= p[0] <= front + 0.25 and abs(p[1]) <= hy]
    band = [p for p in deep if p[0] <= front + 0.10]
    fit = _fit_line_tls(band)
    if fit is None:
        return None, None, None, "fit falhou (poucos pontos na banda)"
    cx, cy, nx, ny = fit
    if nx < 0:
        nx, ny = -nx, -ny
    inl = [p for p in deep if abs((p[0] - cx) * nx + (p[1] - cy) * ny) <= 0.025]
    if len(inl) < 30:
        return None, None, None, "menos de 30 inliers"
    rms = math.sqrt(
        sum(((p[0] - cx) * nx + (p[1] - cy) * ny) ** 2 for p in inl) / len(inl))
    if rms > max_residual:
        return None, None, None, f"residuo alto ({rms * 1000:.0f} mm, {len(inl)} pts)"
    tang = (-ny, nx)
    # Coordenada ao longo da reta e a lateral (y) de cada inlier, ordenadas.
    along = sorted(
        ((p[0] - cx) * tang[0] + (p[1] - cy) * tang[1], p[1]) for p in inl)
    # Segmento que contem o eixo do robo (py cruza zero).
    segs = []
    cur = [along[0]]
    for prev, item in zip(along, along[1:]):
        if item[0] - prev[0] > 0.05:
            segs.append(cur)
            cur = []
        cur.append(item)
    segs.append(cur)
    seg = None
    for candidate in segs:
        ys = [q[1] for q in candidate]
        if min(ys) <= 0.0 <= max(ys):
            seg = candidate
            break
    if seg is None:
        return None, None, None, "nenhum segmento contem o eixo do robo"
    width = seg[-1][0] - seg[0][0]
    if not (wmin <= width <= wmax):
        return None, None, None, f"largura {width:.2f} fora de [{wmin:.2f},{wmax:.2f}]"
    # (Sem comparar com a largura do refino do staging: aquela janela de
    # +-0.45 m CORTA uma mesa de 0.80 — e' por isso que os logs de campo
    # sempre mostravam 0.62-0.65 m. Estimadores diferentes nao se comparam.)
    y_lo = min(q[1] for q in seg)
    y_hi = max(q[1] for q in seg)
    if y_hi > hy - edge_margin or y_lo < -(hy - edge_margin):
        # borda encostada no limite da janela: cortada
        return None, None, None, (
            f"borda na janela (y {y_lo:+.2f}..{y_hi:+.2f}, limite {hy - edge_margin:.2f})")
    mid_along = 0.5 * (seg[0][0] + seg[-1][0])
    if abs(nx) < 1e-6:
        return None, None, None, "face paralela ao eixo"
    # Ponto onde o EIXO do robo (direcao +x) cruza a face: (t, 0).
    t_hit = (cx * nx + cy * ny) / nx
    along_hit = (t_hit - cx) * tang[0] + (0.0 - cy) * tang[1]
    # Pe' da perpendicular do centro do robo (origem) a reta: d*(nx, ny).
    d = cx * nx + cy * ny
    along_foot = (d * nx - cx) * tang[0] + (d * ny - cy) * tang[1]
    return mid_along - along_hit, mid_along - along_foot, width, ""


class DockAlignNode(Node):
    def __init__(self):
        super().__init__("dock_align_node")

        # --- Parametros ---
        self.declare_parameter("reference_topic", "/mecanum_controller/reference")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("control_frequency", 20.0)
        # Limites coerentes com o Nav2 (controller_server.yaml). 0.13 (v4,
        # 2026-08-21): a reta do staging e' curta (~0.15-0.25 m) e o coast do
        # ESC cresce com a velocidade.
        self.declare_parameter("max_vel_x", 0.13)
        # Piso fisico do ESC pos-fix do firmware (2026-08-03): 650rpm = 0.1215
        # m/s / 0.315 rad/s de corpo, executado EXATO (mapa ancorado 1570us).
        self.declare_parameter("min_body_vel", 0.1215)
        self.declare_parameter("min_body_omega", 0.315)
        # Partida do REPOUSO (2026-10-19, campo: nudge "andou +0.000 m"): o
        # piso em roda da' 2.4299999999999997 rad/s, um ULP abaixo do piso
        # do ESC (2.43), e o driver so' LIGA com |cmd| >= piso (histerese:
        # desliga abaixo de 0.4 x piso). Toda primitiva que parte do repouso
        # comanda piso x (1 + margem); ver _floor_start_speed.
        self.declare_parameter("floor_start_margin", 0.07)
        self.declare_parameter("kp_linear", 1.2)
        self.declare_parameter("default_xy_tolerance", 0.03)
        # 0.09 rad (~5 graus, 2026-08-18): o yaw da face oscila +-4 graus
        # entre fits; exigir menos vira retoque infinito ate o timeout, e o
        # pick compensa o residuo pela camera/tag.
        self.declare_parameter("default_yaw_tolerance", 0.09)
        # 90 s (v4, 2026-08-21): cobre o approach inteiro (staging->dock,
        # ~45 cm) mais o recuo final.
        self.declare_parameter("default_timeout", 90.0)

        # ------- MURO DE SEGURANCA por LiDAR (2026-08-18, pedido do operador:
        # "o robo NUNCA pode bater na mesa"). Distancia frontal medida a CADA
        # ciclo por minimo robusto do corredor frontal — nao o fit TLS, cujos
        # guards descartam vista parcial justamente perto da mesa.
        self.declare_parameter("wall_enabled", True)
        # Distancia de parada frontal. -1 = usa desired_face_dist do dock
        # (docking.yaml, por mesa) e, na falta, refine_desired_face_dist
        # global; tudo <0 = muro so aplica o hard_min.
        self.declare_parameter("stop_face_dist", -1.0)
        # Banda de histerese do HOLD: a janela de sucesso [stop-0.01,
        # stop+band+0.01] = 3,5 cm fica acima do ruido do percentil (+-3 mm).
        self.declare_parameter("wall_hysteresis_band", 0.015)
        # Abaixo disso: para tudo (hard_stop). O que toca a mesa e' o BICO do
        # LiDAR (ponta ~0.40 m do centro; footprint 0.395 em planner_server.yaml
        # e controller_server.yaml): hard_min fica ACIMA dele. 0.42 = 2,5 cm de
        # folga do contato real (campo 2026-08-21: encostou com o muro lendo 0.398).
        self.declare_parameter("hard_min_face_dist", 0.42)
        # Idade medida pela HORA DE CHEGADA do scan (imune a relogio da Pi).
        # 0.2 s: a 0.12 m/s cada 0.1 s e' 1,2 cm de atraso.
        self.declare_parameter("wall_max_scan_age", 0.2)
        self.declare_parameter("wall_min_points", 5)
        # Corredor frontal = meia-largura do chassi 0.195 + 1.5cm de margem.
        self.declare_parameter("wall_corridor_half_width", 0.21)
        self.declare_parameter("wall_max_range", 1.5)
        # k-esimo menor x (fracao dos pontos do corredor): rejeita speckle
        # isolado sem esconder obstaculo real.
        self.declare_parameter("wall_percentile", 0.10)
        # Tentativas extras apos hard_stop/timeout (recua ate o pre-docking e
        # tenta de novo). 0 (v5, 2026-08-21, pedido do operador: SEM
        # vai-e-vem): uma tentativa so'; na falha a missao pula a mesa.
        self.declare_parameter("approach_retries", 0)

        # ------- Coast do ESC e distancias minimas das primitivas (v4/v5,
        # 2026-08-21; o fluxo por pulsos/standoff do v4 saiu no v5).
        # O ESC mantem a ultima referencia ~0,5 s depois do neutro (TURNOFF):
        # qualquer movimento continuo e' CORTADO quando falta exatamente este
        # coast, e o ESC termina. MEDIDO 2026-08-21 (pulse_test.py cont, robo
        # real): x andou 15 cm p/ 12,2 comandados => coast ~3 cm (0.04 c/ margem);
        # giro 20-30 graus p/ 18 => ~6 graus (0.10 rad); lateral andou 10 cm
        # p/ 12,2 (escorrega, sem coast) => 0.03.
        self.declare_parameter("esc_coast_dist", 0.04)
        # Mesma ideia para o giro (rad) e para o lateral (m). Abaixo disso nao
        # ha como corrigir — o residuo e' aceito (a camera do pick compensa).
        self.declare_parameter("yaw_coast_est", 0.10)
        self.declare_parameter("center_coast_est", 0.03)
        # Raio do canto do bico ~0.38-0.40 m: girar so com front >= isto;
        # lateral so com front >= lateral_min_front.
        self.declare_parameter("rotate_min_front", 0.46)
        # 0.36 (2026-08-24): lateral puro nao avanca o bico (campo 21/08) e,
        # com alvo de pouso 0,38 (pousos 0,37-0,39), o retoque lateral na mesa
        # precisa valer ate hard_min+0,01.
        self.declare_parameter("lateral_min_front", 0.36)
        # 2026-08-29 (campo, AMT1): o docking para com o bico a ~0,35-0,39 m
        # e o nudge era recusado (muito_perto_da_mesa). Em vez de recusar,
        # RECUA em linha reta (mesmo movimento do undock) ate ter folga e
        # so' entao anda de lado. Sem folga nem recuando: recusa como antes.
        self.declare_parameter("nudge_backoff_enabled", True)
        self.declare_parameter("nudge_backoff_margin", 0.03)   # folga acima de lateral_min_front
        self.declare_parameter("nudge_backoff_max", 0.12)      # recuo maximo (m)
        self.declare_parameter("nudge_backoff_coast", 0.06)    # coast da re (ESC retem ~0,5 s)
        # Assentamento apos cada comando: front estavel em N scans NOVOS
        # (eps) ou teto de tempo — so entao medir/comandar de novo.
        self.declare_parameter("settle_stable_scans", 3)
        self.declare_parameter("settle_stable_eps", 0.003)
        self.declare_parameter("settle_max_s", 1.5)
        # Sem medicao do muro: scan velho (STALE) ou corredor vazio (BLIND).
        # Nos dois casos o robo NAO anda; passado o teto, a tentativa termina
        # (recuo+retry). Mesa fora do plano do LiDAR NAO e' dockavel por aqui.
        self.declare_parameter("stale_abort_s", 1.0)
        self.declare_parameter("blind_wait_s", 1.0)
        # Centragem na face (lateral_adjust): estimador endurecido + curso vy
        # cortado pelo centro medido.
        self.declare_parameter("center_enabled", True)
        self.declare_parameter("center_half_width_y", 0.65)
        self.declare_parameter("center_edge_margin", 0.08)
        # 0.48 (campo 2026-08-22): a prateleira mede 0.49.
        self.declare_parameter("center_width_min", 0.48)
        self.declare_parameter("center_width_max", 0.95)
        self.declare_parameter("center_max_residual", 0.012)
        self.declare_parameter("center_samples", 5)
        self.declare_parameter("center_spread_max", 0.02)
        self.declare_parameter("center_deadband", 0.025)
        self.declare_parameter("center_max_travel", 0.15)
        # Faixa lateral (alem do corredor) vigiada antes de cada curso vy.
        self.declare_parameter("side_clear_half_width", 0.50)
        # Recuo final (apos esgotar tentativas/timeout) com orcamento proprio.
        self.declare_parameter("final_retreat_grace_s", 15.0)
        # Sanidade do pre-docking (campo 22/08, WS5): o align so comeca se o
        # robo estiver a menos disto do staging gravado — pega AMCL perdido e
        # pose de dock desatualizada ANTES de andar rumo a face errada.
        self.declare_parameter("staging_sanity_max_m", 0.6)

        # ------- nudge_base (2026-08-28, fila de alcance): deslocamento
        # LATERAL puro pedido pela arvore quando o braco nao alcanca um alvo
        # na mesa. Malha fechada por ODOMETRIA (/odom do EKF), nao pelo
        # LiDAR — a face reta nao observa o lateral. Curso maximo por
        # chamada e passo minimo executavel (campo 2026-08-21: o lateral
        # escorrega ~18% e nao tem coast; passo minimo ~10 cm no piso).
        self.declare_parameter("nudge_max_travel", 0.25)
        self.declare_parameter("nudge_min_travel", 0.08)
        self.declare_parameter("nudge_default_timeout", 15.0)
        # Folga aceita no curso: pedido 0.20 e andou 0.17 = sucesso (o
        # deslocamento REAL e publicado de qualquer jeito).
        self.declare_parameter("nudge_accept_slack", 0.03)
        # Odometria mais velha que isto = cega: nao comeca / corta o curso.
        # 0.2 (revisao, achado 5): a idade entra no corte como atraso da
        # medida; mais velha que isto a conta ja nao vale.
        self.declare_parameter("nudge_odom_max_age", 0.2)
        # Exige medicao do muro (face no plano do LiDAR) para andar de lado;
        # false SO para mesas invisiveis ao LiDAR — ai nao ha muro nenhum.
        self.declare_parameter("nudge_require_scan", True)
        self.declare_parameter("odom_topic", "/odom")
        # Fala "Vou me ajustar para a esquerda/direita" ANTES de mover (o
        # operador afasta objetos) e espera este tanto antes do 1o comando.
        self.declare_parameter("speech_enabled", True)
        self.declare_parameter("nudge_speech_lead_s", 1.0)
        # ---- revisao 2026-08-28 (achados da revisao adversarial) ----
        # Achado 1: num lateral ao longo de uma face NAO perpendicular o bico
        # avanca |dy|*sin(face_yaw). A pre-checagem projeta isso; durante o
        # curso, front caindo mais que isto em relacao ao inicio = muro.
        # Ruido do front (percentil) e ~settle_stable_eps; subir se disparar
        # a toa.
        self.declare_parameter("nudge_front_drift_max", 0.02)
        # Achado 2: o /odom do EKF integra so a velocidade das rodas e nao
        # ve o escorregamento lateral (~18% no campo). Ganho real/odom
        # aplicado ao lateral medido (corte, aceite, feedback e total
        # publicado). CALIBRAR com trena: pedir 0.20, medir o real, ganho =
        # real / odom bruto do log de fim (esperado ~0.82). 1.0 = sem
        # correcao (o aceite entao conta com a folga nudge_accept_slack).
        self.declare_parameter("nudge_odom_lateral_gain", 1.0)
        # Achado 3: assentamento por ODOMETRIA (o front nao muda num lateral
        # puro): lateral dentro de eps por min_s (>= retencao do ESC ~0,5 s),
        # teto max_s; so depois mede o curso e decide.
        self.declare_parameter("nudge_settle_eps", 0.003)
        self.declare_parameter("nudge_settle_min_s", 0.8)
        self.declare_parameter("nudge_settle_max_s", 3.0)
        # Watchdog de progresso (2026-10-19): comandando ha' nudge_no_progress_s
        # sem a odometria andar nudge_no_progress_min_m = aborta com
        # 'sem_progresso' e loga o diagnostico (publicadores na referencia,
        # relogio PC x Pi, piso do ESC) em vez de esperar o teto e devolver
        # 'curso_incompleto' sem explicacao.
        self.declare_parameter("nudge_no_progress_s", 1.0)
        self.declare_parameter("nudge_no_progress_min_m", 0.01)

        # ------- Refino de yaw sob suspeita (operador 2026-08-18: "as vezes
        # piora"): so aplica a correcao quando 2 fits confiantes consecutivos
        # concordam; e da para desligar de vez por parametro.
        self.declare_parameter("refine_correct_yaw", True)
        self.declare_parameter("refine_yaw_agree_tol", 0.035)

        # Janela frontal (m) onde procuramos a face da mesa para refino.
        self.declare_parameter("refine_front_min_x", 0.10)
        self.declare_parameter("refine_front_max_x", 1.20)
        self.declare_parameter("refine_half_width_y", 0.45)
        # Guardas anti-falso-positivo do refino (2026-07-23):
        # residuo RMS maximo do ajuste de reta (face plana de verdade) e largura
        # minima da face detectada (rejeita perna de mesa/objeto isolado).
        self.declare_parameter("refine_max_residual", 0.02)
        # 0.50 (2026-08-03): fits saudaveis medem 0.61-0.65 (face real 0.80 m);
        # um fit de 0.39 m (segmento parcial visto de perto) passou nos guards
        # e girou o alvo 16.7 graus. Abaixo de 0.50 = vista degradada.
        self.declare_parameter("refine_min_face_width", 0.50)
        # Distancia frontal DESEJADA robo->face na pose final (calibrar no robo:
        # posicione o robo na pose perfeita e leia o face_dist do log). Valor
        # negativo = ainda nao calibrado -> o refino corrige SO o yaw.
        self.declare_parameter("refine_desired_face_dist", -1.0)
        # (v4) Sem re-refino periodico: depois do refino inicial o controle e'
        # todo pela face no LiDAR (front/yaw/centro), nao pelo alvo do mapa.

        # ------- Revisao 2026-10-21 (run da WS1: "pronto" com yaw_face
        # -19,7 graus). Historico em docs/docking_alinhamento_holonomico.md.
        # Portao de 2 leituras: quando o 1o fit e' confiavel, espera um /scan
        # NOVO ate este tanto e refaz o fit NO MESMO GOAL (antes a 2a leitura
        # so' vinha no goal seguinte: "yaw MANTIDO (portao)" com -15 graus).
        self.declare_parameter("refine_second_read_max_s", 0.6)
        # Refit perto da mesa: o reto pausa (uma vez) quando vai PARAR a
        # ~esta distancia da face (corte descontando coast/lag), refaz o fit
        # numa banda rasa ao redor do muro (face isolada dos vizinhos) e
        # esquadra pelo LiDAR se front >= rotate_min_front. <= 0 desliga.
        self.declare_parameter("refine_refit_front_m", 0.55)
        # Largura MAXIMA de um fit confiavel: a mesa tem 0,80 m; 0,98 e' mesa
        # + vizinho (WS1 a 0,68 m). Rejeitado como 'face larga demais' e, no
        # staging, SEM esquadro por AMCL (fica para o refit perto da mesa).
        self.declare_parameter("refine_max_face_width", 0.90)
        # Validacao final: |yaw_face| > tolerancia apos o lateral(mesa) =>
        # recua ate rotate_min_front+0,05, esquadra pela face, refaz reto e
        # lateral (UMA vez por goal). false = so' mede e avisa.
        self.declare_parameter("final_yaw_correction", True)
        # true: ainda fora apos a correcao => goal FALHA (abort). false
        # (competicao): sucede com WARN e message 'yaw_fora_da_tolerancia'.
        self.declare_parameter("final_yaw_strict", False)
        # Dispersao maxima (rad) entre as leituras de yaw da face na medida
        # final (mediana de center_samples scans); acima = 'yaw_nao_medido'.
        # 0,07 rad (4 graus): o yaw entre fits oscila +-4 graus (2026-08-18).
        self.declare_parameter("final_yaw_spread_max", 0.07)

        # ------- Revisao 2026-10-26 (tours 19:45/19:52; docs/
        # diagnostico_docking_tour_2026-10-26.md). Esquadro por AMCL
        # DESLIGADO por default (I1, F13/C1): na WS6 girou -9,7/-9,4 graus CW
        # nos dois tours contra um LiDAR que pedia +5 CCW; na WS1-T2 -11,9
        # contra o alvo recem-refinado. false = so' loga o que teria feito.
        self.declare_parameter("amcl_square_enabled", False)
        # Ligado: giro maximo pelo AMCL (rad). Acima disso nao gira e loga —
        # o yaw da TF ja' esteve 7-8 graus enviesado na chegada (F14).
        # PRECISA ser > yaw_coast_est + 0,02 (yaw_min_corr), senao a janela
        # de giro fica vazia e o esquadro ligado nunca roda (ver
        # _AMCL_SQUARE_MAX_ERR_DEFAULT e o WARN abaixo).
        self.declare_parameter("amcl_square_max_err", _AMCL_SQUARE_MAX_ERR_DEFAULT)
        # Espera de parada por /odom antes de medir (I3c, F14/C2: a TF ainda
        # gira +6,7..+7,8 graus nos 0,4 s depois do "Reached the goal" do
        # Nav2): |v| <= still_v_max e |wz| <= still_w_max por still_min_s,
        # teto still_max_s. O yaw da face do fit leve (settle_stable_scans
        # dentro de still_yaw_eps) e' so' ADVISORIO com /odom (vai para o
        # log) e vira o criterio apenas sem /odom: o fit leve oscila +-4
        # graus entre scans (final_yaw_spread_max, 2026-08-18), entao 0,5
        # grau era inatingivel e a espera batia no teto em todo docking.
        # 0,035 rad = 2 graus (= refine_yaw_agree_tol).
        self.declare_parameter("still_v_max", 0.02)
        self.declare_parameter("still_w_max", 0.05)
        self.declare_parameter("still_min_s", 0.5)
        self.declare_parameter("still_max_s", 3.0)
        self.declare_parameter("still_yaw_eps", 0.035)

        # ------- Camera no docking (2026-10-27, pedido do operador) -------
        # A RealSense do punho, com o braco na pose de navegacao (home), ve o
        # tampo inteiro do staging (caramelo_dock_camera.py). Ligada, ela
        # MANDA: no staging mede yaw + centro da mesa e corrige por odometria
        # (mede -> move -> remede, ate camera_max_steps movimentos); na pausa
        # do reto e na validacao final o yaw/centro sao a FUSAO camera+LiDAR
        # (peso camera_weight para a camera quando concordam; discordando,
        # vale a camera). Sem medida da camera: o fluxo LiDAR de sempre.
        # A assinatura do depth so' existe durante o goal.
        self.declare_parameter("camera_enabled", True)
        self.declare_parameter("camera_depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info")
        # Tipos de service area (service_areas.yaml) onde a camera NAO e'
        # usada: a prateleira tem tampo em cima tapando o nivel de baixo.
        self.declare_parameter("camera_skip_types", ["shelf"])
        # 2026-10-27 (tour: cada medida levava ~4 s): 3 frames, teto 1,5 s.
        self.declare_parameter("camera_frames", 3)          # frames por medida (mediana)
        self.declare_parameter("camera_wait_s", 1.5)        # teto por medida
        self.declare_parameter("camera_stride", 4)          # 1 de cada 4 pixels
        self.declare_parameter("camera_yaw_spread_max", 0.035)    # 2 graus entre frames
        self.declare_parameter("camera_center_spread_max", 0.03)
        self.declare_parameter("camera_max_steps", 3)       # movimentos no staging
        # Tolerancias da camera; o giro ainda fica limitado pelo coast
        # (yaw_coast_est + 0,02): abaixo disso o giro nao corrige.
        # "Bom o bastante" (operador 27/10: o manipulador tolera um pouco de
        # angulo/centro; correr atras de 2 graus/2 cm custava dezenas de s).
        self.declare_parameter("camera_yaw_tol", 0.07)      # 4 graus
        self.declare_parameter("camera_center_tol", 0.04)   # 4 cm
        # Abaixo disto (front do LiDAR) a borda da frente sai da imagem e o yaw
        # so' pelas laterais errava ate 20 graus (WS1/WS3 27/10, disparando
        # recuo + correcao a toa): perto da mesa vale so' o LiDAR.
        self.declare_parameter("camera_min_front", 0.50)
        self.declare_parameter("camera_max_lateral", 0.30)  # demanda lateral maior = medida suspeita
        # Fusao camera+LiDAR: concordando dentro de *_agree, media com peso
        # camera_weight para a camera; discordando, so' a camera.
        self.declare_parameter("camera_weight", 0.8)
        self.declare_parameter("camera_lidar_yaw_agree", 0.06)
        self.declare_parameter("camera_lidar_center_agree", 0.04)
        # Correcao final de yaw com a camera: recua ate aqui (a borda da
        # frente volta ao campo de visao) em vez de rotate_min_front + 0,05.
        self.declare_parameter("camera_view_front", 0.60)
        # Calibracao (somadas a medida): robo docado PERFEITO pela trena,
        # ler "[cam]" no log; offset = -(o que a camera mediu).
        self.declare_parameter("camera_yaw_offset", 0.0)
        self.declare_parameter("camera_y_offset", 0.0)
        # Imagem de debug (rqt_image_view): depth com tampo/parede + vista de
        # cima com as bordas, o centro e a decisao. So' desenha se alguem
        # assina o topico.
        # ---- Giro em PULSOS (2026-10-27, operador: "ele gira sempre a mesma
        # quantidade e nunca se ajusta, so' entorta") ----
        # O ESC (FOC, piso 650 rpm, rampas no firmware) faz cada giro continuo
        # sair um degrau de ~12-20 graus. Em pulsos: pulso de T s -> para ->
        # mede pelo LiDAR quanto girou -> aprende giro(T) = c + k*T -> proximo
        # pulso pelo que falta. "continuo" = o corte antigo (_cut_drive).
        self.declare_parameter("rotate_mode", "pulsos")
        self.declare_parameter("rot_pulse_c0", 0.05)      # rad (~3 graus) de partida
        self.declare_parameter("rot_pulse_k0", 0.30)      # rad por s de pulso
        self.declare_parameter("rot_pulse_t_min", 0.15)
        self.declare_parameter("rot_pulse_t_max", 0.80)
        self.declare_parameter("rot_pulse_max", 6)        # pulsos por giro
        self.declare_parameter("rot_pulse_tol", 0.035)    # 2 graus: para aqui
        self.declare_parameter("camera_debug_image", True)
        self.declare_parameter("camera_debug_topic", "/dock_align/camera_debug")
        # 2026-10-26: janela de giro do esquadro por AMCL fechada = ligado
        # mas inoperante; avisa no init (antes era silencioso).
        _ymin, _open = amcl_square_window(
            self.get_parameter("yaw_coast_est").value,
            self.get_parameter("amcl_square_max_err").value)
        if not _open:
            self.get_logger().warn(
                f"amcl_square_max_err ({float(self.get_parameter('amcl_square_max_err').value):.2f} rad) "
                f"<= yaw_coast_est + {_YAW_MIN_CORR_MARGIN:.2f} ({_ymin:.2f}): a janela de giro e' "
                "vazia — esquadro por AMCL efetivamente DESLIGADO mesmo com "
                "amcl_square_enabled=true")

        self._ref_topic = self.get_parameter("reference_topic").value
        self._global_frame = self.get_parameter("global_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._cb_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(TwistStamped, self._ref_topic, 10)
        self._last_scan = None
        # SensorDataQoS (2026-08-18): compativel com driver reliable OU
        # best-effort; a assinatura antiga (reliable/depth 10) quebraria em
        # silencio se o driver publicasse best-effort.
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value,
            self._scan_cb, qos_profile_sensor_data, callback_group=self._cb_group,
        )
        # Cache da TF base<-frame_do_scan (LiDAR parafusado = estatica).
        self._scan_tf = None
        self._scan_tf_frame = None
        # Ultimo yaw de face confiante (frame do MAPA) p/ o portao de
        # consistencia do refino de yaw.
        self._last_face_yaw_map = None
        # desired_face_dist por dock (docking.yaml), carregado por goal.
        self._dock_face_dist = None
        # Scan: hora de CHEGADA (idade imune ao relogio da Pi) e contador de
        # scans novos (uma medida por scan).
        self._last_scan_rx_ns = 0
        self._scan_seq = 0
        self._stamp_warned = False
        # floor_start_margin invalido (negativo/NaN) e' avisado uma vez.
        self._margin_warned = False
        # Menor x fora do corredor, por lado (esq py>0, dir py<0): checagem
        # lateral antes de cada curso vy.
        self._side_min = (None, None)
        self._wall_age = None
        self._center_reject = ""
        # Largura da face medida no refino do staging (coerencia do estimador
        # de centro perto da mesa).
        self._last_face_width = None
        # 2026-10-21: yaw da face do fit leve do muro no ULTIMO _check_guards
        # (erro vivo do esquadro pelo LiDAR) e flag 'face larga demais' do
        # refino do staging (inibia o esquadro por AMCL; desde 2026-10-26 o
        # esquadro por AMCL so' roda sem face NENHUMA — fica como registro).
        # Ambos por goal.
        self._live_face_yaw = None
        self._refine_wide_face = False
        # 2026-10-26: yaw_rel de TODOS os fits do refino do staging neste
        # goal (rejeitados inclusive — veto do esquadro por AMCL, I1);
        # janela lateral do refino por dock (R3, None = parametro global);
        # motivo da ultima reprovacao do fit leve do muro e o par
        # (c_axis, c_foot) da ultima medida de centro (instrumentacao I6/I4).
        self._refine_fit_yaws = []
        self._dock_refine_half_width = None
        self._wall_fit_reject = ""
        self._center_last = (None, None)
        self._center_medians = (None, None)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Camera no docking (2026-10-27): assinaturas criadas por goal
        # (_cam_start/_cam_stop); depth = (msg, hora de CHEGADA ns).
        self._cam_sub = None
        self._cam_info_sub = None
        self._cam_depth = None
        self._cam_seq = 0
        self._cam_k = None
        self._cam_reject = ""
        self._cam_goal_enabled = False
        self._cam_dock_id = ""
        # Modelo do giro em pulsos (c, k): aprendido e mantido entre goals
        # (a base e' a mesma); None = ainda os valores iniciais dos params.
        self._rot_model = None
        self._cam_dbg_pub = self.create_publisher(
            Image, self.get_parameter("camera_debug_topic").value, 1)
        # Largura MEDIDA pela camera neste goal (duas laterais visiveis):
        # nenhuma dimensao de mesa e' configurada (variam por arena).
        self._cam_goal_width = None

        # ------- nudge_base (2026-08-28) -------
        # Odometria do EKF: (x, y, yaw, hora de CHEGADA ns) — idade pela
        # chegada, como o scan (relogio da Pi nao e confiavel).
        self._odom = None
        # Twist do /odom (vx, vy, wz, hora de CHEGADA ns): espera de parada
        # antes de medir (2026-10-26, I3c) e instrumentacao.
        self._odom_twist = None
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._odom_cb, qos_profile_sensor_data, callback_group=self._cb_group,
        )
        self._speech_pub = self.create_publisher(String, "/manip/speech", 10)
        # AUDITORIA 31/08 (vazamento): create_rate por goal deixava um Timer
        # de 20 Hz vivo para SEMPRE no caminho do align (o do nudge era
        # destruido). Um Rate persistente por frequencia serve aos dois.
        self._rate_cache = None
        # Total lateral acumulado desde o ultimo align_to_dock (+ = esquerda).
        # LATCHED (transient_local, depth 1): o place node pode assinar
        # depois e ainda recebe o ultimo valor. SO este no publica aqui.
        self._shift_pub = self.create_publisher(
            Float64, "/manip/base_shift_total",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._base_shift_total = 0.0
        self._publish_shift_total()
        # Lock de MOVIMENTO: align_to_dock e nudge_base publicam na mesma
        # referencia e o executor e multi-thread — nunca os dois juntos.
        # Aquisicao NAO bloqueante: ocupado = rejeita/aborta, nunca enfileira.
        self._motion_lock = threading.Lock()
        # Pose de odometria no inicio do nudge em curso (p/ o feedback) e o
        # ganho de escorregamento em uso (feedback = mesma medida do result).
        self._nudge_origin = None
        self._nudge_gain = 1.0

        self._action = ActionServer(
            self, AlignToDock, "align_to_dock",
            execute_callback=self._execute,
            goal_callback=self._motion_goal_cb,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self._nudge_action = ActionServer(
            self, NudgeBase, "nudge_base",
            execute_callback=self._execute_nudge,
            goal_callback=self._motion_goal_cb,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"dock_align_node pronto. Publica em '{self._ref_topic}', "
            f"frames {self._global_frame}->{self._base_frame}; "
            f"nudge_base por odometria em '{self.get_parameter('odom_topic').value}'."
        )

    def _scan_cb(self, msg):
        self._last_scan = msg
        self._last_scan_rx_ns = self.get_clock().now().nanoseconds
        self._scan_seq += 1

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Tupla atribuida de uma vez (leitura atomica pelas threads do nudge).
        rx_ns = self.get_clock().now().nanoseconds
        self._odom = (p.x, p.y, _yaw_from_quat(q.z, q.w, q.x, q.y), rx_ns)
        tw = msg.twist.twist
        self._odom_twist = (tw.linear.x, tw.linear.y, tw.angular.z, rx_ns)

    def _odom_sample(self, max_age):
        """(x, y, yaw, idade_s, rx_ns) da ultima odometria se chegou ha
        menos de max_age s; senao None (nunca recebida ou velha = cega). A
        idade entra no corte do curso (achado 5)."""
        odom = self._odom
        if odom is None:
            return None
        age = (self.get_clock().now().nanoseconds - odom[3]) * 1e-9
        if age > max_age:
            return None
        return odom[0], odom[1], odom[2], age, odom[3]

    def _odom_pose(self, max_age):
        """(x, y, yaw) da odometria fresca, ou None (ver _odom_sample)."""
        sample = self._odom_sample(max_age)
        return None if sample is None else sample[:3]

    def _motion_goal_cb(self, _goal):
        """Goal de align_to_dock OU nudge_base: rejeita se a base ja esta em
        movimento por outro goal (os dois publicam na mesma referencia)."""
        if self._motion_lock.locked():
            self.get_logger().warn(
                "goal rejeitado: outro movimento (align/nudge) em curso — ocupado")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _publish_shift_total(self):
        msg = Float64()
        msg.data = float(self._base_shift_total)
        self._shift_pub.publish(msg)

    def _set_shift_total(self, value):
        self._base_shift_total = float(value)
        self._publish_shift_total()
        self.get_logger().info(
            f"/manip/base_shift_total = {self._base_shift_total:+.3f} m")

    def _speak(self, text):
        if not bool(self.get_parameter("speech_enabled").value):
            return
        msg = String()
        msg.data = text
        self._speech_pub.publish(msg)

    # ------------------------------------------------------------------ TF
    def _robot_pose(self):
        """Pose (x, y, yaw) do robo no global_frame via TF, ou None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return t.x, t.y, _yaw_from_quat(q.z, q.w, q.x, q.y)

    # --------------------------------------------------------------- Docks
    def _load_dock_pose(self, dock_id, map_name, map_dir):
        """Pose do dock + metadados: (x, y, yaw, desired_face_dist|None,
        staging_offset, refine_half_width_y|None). desired_face_dist e' a
        chave OPCIONAL por mesa (2026-08-18); staging_offset vem do
        dock_plugins pelo type do dock e dimensiona o recuo do retry
        (fallback 0.45); refine_half_width_y e' a janela lateral do refino
        por dock (2026-10-26, R3). Leitura em parse_dock_entry (pura)."""
        map_folder = resolve_map_folder(map_name, map_dir or None)
        path = docking_file(map_folder)
        x, y, yaw, frame, face_dist, staging_offset, half_w = parse_dock_entry(
            load_yaml(path), dock_id, str(path))
        if frame != self._global_frame:
            self.get_logger().warn(
                f"dock '{dock_id}' esta no frame '{frame}', "
                f"mas o no usa '{self._global_frame}'.")
        return x, y, yaw, face_dist, staging_offset, half_w

    # ------------------------------------------------- Camera no docking
    def _cam_setup_goal(self, dock_id, map_name, map_dir):
        """Por goal: camera ligada para este dock? (param + tipo da service
        area). Zera a largura medida (nada de dimensao do yaml)."""
        self._cam_goal_width = None
        self._cam_dock_id = dock_id
        self._cam_goal_enabled = bool(self.get_parameter("camera_enabled").value)
        if not self._cam_goal_enabled:
            return
        area_type = ""
        try:
            data = load_yaml(service_areas_file(resolve_map_folder(map_name, map_dir or None)))
            area = ((data or {}).get("service_areas") or {}).get(dock_id) or {}
            area_type = str(area.get("type", ""))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"[{dock_id}] camera: service_areas.yaml ilegivel ({exc}); sem filtro de tipo")
        skip = list(self.get_parameter("camera_skip_types").value)
        if area_type and area_type in skip:
            self._cam_goal_enabled = False
            self.get_logger().info(
                f"[{dock_id}] camera desligada neste dock (tipo '{area_type}' em "
                "camera_skip_types) — so' LiDAR")

    def _cam_start(self):
        if self._cam_sub is not None:
            return
        self._cam_depth = None
        # RELIABLE (2026-10-27): o depth tem 600 KB e em best-effort chegavam
        # 2 frames em 6 s (fragmentos perdidos) — cada medida esperava o teto.
        # Reliable: ~10 frames/s. O driver da RealSense publica reliable.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._cam_sub = self.create_subscription(
            Image, self.get_parameter("camera_depth_topic").value,
            self._cam_depth_cb, qos, callback_group=self._cb_group)
        self._cam_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._cam_info_cb, qos, callback_group=self._cb_group)

    def _cam_stop(self):
        for sub in (self._cam_sub, self._cam_info_sub):
            if sub is not None:
                self.destroy_subscription(sub)
        self._cam_sub = None
        self._cam_info_sub = None
        self._cam_depth = None

    def _cam_depth_cb(self, msg):
        self._cam_depth = (msg, self.get_clock().now().nanoseconds)
        self._cam_seq += 1

    def _cam_info_cb(self, msg):
        self._cam_k = np.array(msg.k, dtype=float).reshape(3, 3)

    def _cam_cfg(self):
        cfg = dock_cam.CamCfg()
        cfg.stride = max(1, int(self.get_parameter("camera_stride").value))
        cfg.expected_width = self._cam_goal_width
        cfg.yaw_offset = float(self.get_parameter("camera_yaw_offset").value)
        cfg.y_offset = float(self.get_parameter("camera_y_offset").value)
        return cfg

    def _cam_frame_measure(self, msg, cfg, label=""):
        """Um frame de depth -> (TableMeasure, "") ou (None, motivo)."""
        if msg.encoding not in ("16UC1", "mono16"):
            return None, f"encoding '{msg.encoding}' (esperado 16UC1)"
        if self._cam_k is None:
            return None, "sem camera_info"
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, msg.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            return None, f"sem TF {self._base_frame}<-{msg.header.frame_id} ({exc})"
        q = tf.transform.rotation
        t = tf.transform.translation
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.step // 2)[:, :msg.width]
        m, why = dock_cam.measure_depth_frame(
            depth, self._cam_k, dock_cam.quat_to_rot(q.x, q.y, q.z, q.w),
            (t.x, t.y, t.z), cfg)
        self._cam_publish_debug(msg, depth, m, why, label, cfg)
        return m, why

    def _cam_publish_debug(self, msg, depth, m, why, label, cfg):
        if (dock_cam_dbg is None or not bool(self.get_parameter("camera_debug_image").value)
                or self._cam_dbg_pub.get_subscription_count() == 0):
            return
        try:
            img = dock_cam_dbg.draw(depth, m, label, why, cfg.stride)
        except Exception as exc:  # noqa: BLE001 - debug nunca derruba o docking
            self.get_logger().warn(f"[cam] imagem de debug falhou: {exc}", throttle_duration_sec=10.0)
            return
        out = Image()
        out.header = msg.header
        out.height, out.width = img.shape[:2]
        out.encoding = "bgr8"
        out.step = out.width * 3
        out.data = img.tobytes()
        self._cam_dbg_pub.publish(out)

    def _measure_camera(self, goal_handle, rate, deadline, attempt_end, where):
        """Mediana de camera_frames medidas da mesa em frames que CHEGARAM
        depois do inicio da medida (base parada). Devolve dict (yaw, src,
        c_axis, c_foot, front, width, sides; centro None se nao confiou) ou
        None com o motivo em self._cam_reject."""
        n = max(1, int(self.get_parameter("camera_frames").value))
        wait_s = float(self.get_parameter("camera_wait_s").value)
        yaw_spread = float(self.get_parameter("camera_yaw_spread_max").value)
        c_spread = float(self.get_parameter("camera_center_spread_max").value)
        cfg = self._cam_cfg()
        self._cam_start()
        start_ns = self.get_clock().now().nanoseconds
        last_seq = self._cam_seq
        got = []
        reject = collections.Counter()
        frames = 0
        while len(got) < n:
            if (self.get_clock().now().nanoseconds - start_ns) * 1e-9 > wait_s:
                break
            self._check_guards(goal_handle, deadline, attempt_end)
            self._publish_cmd(0.0, 0.0, 0.0)
            depth = self._cam_depth
            if self._cam_seq != last_seq and depth is not None and depth[1] >= start_ns:
                last_seq = self._cam_seq
                frames += 1
                m, why = self._cam_frame_measure(
                    depth[0], cfg, f"[{self._cam_dock_id}] {where} frame {frames}")
                if m is None:
                    reject[why] += 1
                    if why.startswith("camera fora"):
                        break
                elif not m.ok:
                    reject[m.reason] += 1
                else:
                    got.append(m)
            rate.sleep()
        rej_txt = ", ".join(f"{k} x{v}" for k, v in reject.most_common(3)) or "nenhuma"
        if frames == 0:
            self._cam_reject = (f"nenhum frame de '{self.get_parameter('camera_depth_topic').value}' "
                                f"em {wait_s:.1f} s")
            return None
        if len(got) < max(2, (n + 1) // 2):
            self._cam_reject = f"{len(got)}/{frames} frames com a mesa; rejeicoes: {rej_txt}"
            return None

        def med(vals):
            vals = sorted(vals)
            return vals[len(vals) // 2]

        yaws = [m.face_yaw for m in got]
        if max(yaws) - min(yaws) > yaw_spread:
            self._cam_reject = (f"yaw disperso entre frames ({math.degrees(min(yaws)):+.1f}.."
                                f"{math.degrees(max(yaws)):+.1f} graus)")
            return None
        def mode(vals):
            return collections.Counter(vals).most_common(1)[0][0]

        res = {"yaw": med(yaws), "src": mode([m.yaw_src for m in got]),
               "front": med([m.front for m in got]), "c_axis": None, "c_foot": None,
               "width": None, "sides": "", "center_why": ""}
        cen = [m for m in got if m.c_axis is not None]
        if len(cen) >= max(2, (n + 1) // 2):
            axes = [m.c_axis for m in cen]
            if max(axes) - min(axes) <= c_spread:
                res["c_axis"] = med(axes)
                res["c_foot"] = med([m.c_foot for m in cen])
                widths = [m.width for m in cen if m.width is not None]
                res["width"] = med(widths) if widths else None
                res["sides"] = mode([m.sides for m in cen])
                if res["sides"] == "ambos" and res["width"] is not None \
                        and self._cam_goal_width is None:
                    self._cam_goal_width = res["width"]
            else:
                res["center_why"] = (f"centro disperso ({min(axes) * 100:+.1f}.."
                                     f"{max(axes) * 100:+.1f} cm)")
        else:
            whys = collections.Counter(m.center_reason for m in got if m.c_axis is None)
            res["center_why"] = whys.most_common(1)[0][0] if whys else "?"
        g0 = got[0]
        self.get_logger().info(
            f"[cam] {where}: {len(got)}/{frames} frames, yaw={math.degrees(res['yaw']):+.1f} graus "
            f"({res['src']}), c_axis="
            f"{'?' if res['c_axis'] is None else '%+.1f' % (res['c_axis'] * 100)} c_foot="
            f"{'?' if res['c_foot'] is None else '%+.1f' % (res['c_foot'] * 100)} cm "
            f"({res['sides'] or res['center_why']}), largura="
            f"{'?' if res['width'] is None else '%.2f' % res['width']} m, front="
            f"{res['front']:.3f} m, mesa h={g0.table_h:.3f} m, inclinacao corrigida "
            f"{g0.tilt_deg:.1f} graus")
        return res

    # --------------------------------------------------------- Refino LiDAR
    def _scan_points_in_base(self, scan):
        """Pontos validos do scan TRANSFORMADOS para o base_frame (2D) pela
        TF real (base <- frame do scan): o LiDAR e' montado de cabeca para
        baixo (RPY 180/0/180) e o crop do scan_normalizer poe a frente fisica
        em ~180 graus, entao os angulos crus do scan nao servem (2026-07-23).
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, scan.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(f"refino: sem TF {self._base_frame}<-{scan.header.frame_id}: {exc}")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # Linhas X e Y da matriz de rotacao (aplicadas a pontos no plano do laser).
        r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        r01 = 2.0 * (q.x * q.y - q.z * q.w)
        r10 = 2.0 * (q.x * q.y + q.z * q.w)
        r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)

        pts = []
        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment
            if not math.isfinite(r) or r <= scan.range_min or r >= scan.range_max:
                continue
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            pts.append((r00 * lx + r01 * ly + t.x, r10 * lx + r11 * ly + t.y))
        return pts

    # ------------------------------------------------- Muro de seguranca
    def _wall_tf(self, scan_frame):
        """TF base<-scan cacheada (LiDAR parafusado): (r00,r01,r10,r11,tx,ty)."""
        if self._scan_tf is not None and self._scan_tf_frame == scan_frame:
            return self._scan_tf
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, scan_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(
                f"muro: sem TF {self._base_frame}<-{scan_frame}: {exc}")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        self._scan_tf = (
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            2.0 * (q.x * q.y - q.z * q.w),
            2.0 * (q.x * q.y + q.z * q.w),
            1.0 - 2.0 * (q.x * q.x + q.z * q.z),
            t.x, t.y,
        )
        self._scan_tf_frame = scan_frame
        return self._scan_tf

    def _front_wall_distance(self):
        """Distancia frontal REAL + angulo da face no corredor do chassi.

        Distancia = minimo robusto (k-esimo menor x) — de proposito NAO e' o
        fit TLS completo: o muro precisa funcionar com vista parcial e com
        QUALQUER obstaculo (mao, caixa, quina), justamente perto da mesa.
        face_yaw = fit TLS LEVE nos mesmos pontos (guards relaxados — e'
        sinal de CONTROLE relativo, nao reposicionamento de alvo): o angulo
        da normal da face no frame do robo, que a reta final usa para
        "andar reto e so aparar o angulo" (pedido do operador 2026-08-18).

        Retorna (dist|None, n_pts, scan_age_s, face_yaw|None):
          dist None + n_pts == -1  -> sem medicao (scan ausente/velho/sem TF)
          dist None + n_pts >= 0   -> corredor vazio no plano do LiDAR (BLIND)
        """
        scan = self._last_scan
        self._side_min = (None, None)
        # 2026-10-26 (I6): os retornos antecipados tambem registram o motivo,
        # senao _measure_face_yaw agregava o motivo do fit ANTERIOR (ex.
        # 'residuo x3' quando a causa era 'scan velho').
        self._wall_fit_reject = ""
        if scan is None:
            self._wall_fit_reject = "sem scan"
            return None, -1, float("inf"), None
        # Idade pela hora de CHEGADA (v4): o carimbo vem do relogio da Pi e ja
        # esteve 24 dias atrasado (2026-08-21). Avisa uma vez se divergir.
        now_ns = self.get_clock().now().nanoseconds
        scan_age = (now_ns - self._last_scan_rx_ns) * 1e-9
        stamp_age = (now_ns - Time.from_msg(scan.header.stamp).nanoseconds) * 1e-9
        if abs(stamp_age - scan_age) > 0.3 and not self._stamp_warned:
            self._stamp_warned = True
            self.get_logger().warn(
                f"/scan: carimbo difere da chegada em {stamp_age - scan_age:+.2f} s "
                "(relogio Pi x PC?) — usando a hora de chegada.")
        if scan_age > self.get_parameter("wall_max_scan_age").value:
            self._wall_fit_reject = "scan velho"
            return None, -1, scan_age, None
        tfm = self._wall_tf(scan.header.frame_id)
        if tfm is None:
            self._wall_fit_reject = "sem TF"
            return None, -1, scan_age, None
        r00, r01, r10, r11, ox, oy = tfm

        max_range = self.get_parameter("wall_max_range").value
        half_w = self.get_parameter("wall_corridor_half_width").value
        side_w = self.get_parameter("side_clear_half_width").value
        # Pre-crop por range ANTES de transformar (custo Python): folga = o
        # deslocamento do LiDAR em relacao ao base_frame.
        range_cut = max_range + math.hypot(ox, oy) + 0.05

        pts = []
        left_min = right_min = None
        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment
            if (not math.isfinite(r) or r <= scan.range_min
                    or r >= scan.range_max or r > range_cut):
                continue
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            px = r00 * lx + r01 * ly + ox
            if not (0.05 <= px <= max_range):
                continue
            py = r10 * lx + r11 * ly + oy
            apy = abs(py)
            if apy <= half_w:
                pts.append((px, py))
            elif apy <= side_w:
                # Faixa lateral: o que esta ao lado do corredor, a frente do
                # bico (px >= 0.32 e' o que o LiDAR consegue ver).
                if py > 0.0:
                    left_min = px if left_min is None else min(left_min, px)
                else:
                    right_min = px if right_min is None else min(right_min, px)
        self._side_min = (left_min, right_min)

        n = len(pts)
        if n < int(self.get_parameter("wall_min_points").value):
            self._wall_fit_reject = f"corredor vazio ({n} pts)"
            return None, n, scan_age, None
        xs = sorted(p[0] for p in pts)
        k = max(1, math.ceil(self.get_parameter("wall_percentile").value * n))
        front = xs[k - 1]

        # Fit leve para o angulo da face: so pontos perto do plano frontal
        # (rejeita fundo/objetos atras da mesa), guards de controle.
        face_yaw = None
        band_pts = [p for p in pts if p[0] <= front + 0.10]
        fit = _fit_line_tls(band_pts)
        # 2026-10-26 (I6): motivo da reprovacao do fit leve fica em
        # _wall_fit_reject (agregado por _measure_face_yaw) e sai em DEBUG
        # aqui — a 20 Hz um INFO por scan inundaria o log.
        reject = ""
        if fit is None or len(band_pts) < 30:
            reject = f"npts {len(band_pts)}<30"
        else:
            cx, cy, nx, ny = fit
            if nx < 0:
                nx, ny = -nx, -ny
            residual = math.sqrt(
                sum(((p[0] - cx) * nx + (p[1] - cy) * ny) ** 2
                    for p in band_pts) / len(band_pts))
            # Guard de extensao (missao 2026-08-18/WS2): fit estreito num
            # canto/objeto passava no residuo e cuspia yaw de lixo — a reta
            # parava a cada segundo para "aparar" ruido.
            along = [-(p[0] - cx) * ny + (p[1] - cy) * nx for p in band_pts]
            extent = max(along) - min(along)
            yaw = math.atan2(ny, nx)
            if residual > 0.025:
                reject = f"residuo {residual * 1000:.0f}mm>25"
            elif extent < 0.25:
                reject = f"extensao {extent:.2f}<0.25"
            elif abs(yaw) > math.radians(35):
                reject = f"yaw {math.degrees(yaw):+.0f}>35"
            else:
                face_yaw = yaw
        self._wall_fit_reject = reject
        if reject:
            self.get_logger().debug(
                f"[inst] fit leve rejeitado: {reject} (front {front:.3f}, n {n})")
        return front, n, scan_age, face_yaw

    def _side_clear(self, direction, front, margin=0.05):
        """True se nada na faixa lateral do lado do movimento esta mais perto
        que a face (caixa/objeto protruso junto ao bico). Cego para o que
        esta AO LADO do chassi (px < 0.32) e abaixo do plano do LiDAR — por
        isso os cursos laterais sao curtos e vigiados a cada ciclo."""
        side = self._side_min[0] if direction > 0.0 else self._side_min[1]
        return side is None or front is None or side > front - margin

    def _face_center_from_scan(self, front, foot=False):
        """(desvio lateral do centro da face em m, +y = esquerda; largura) ou None.

        Endurecido (campo 18/08: o centro pulava -7 -> +17 cm em 2 s porque
        max/min eram dominados por 1 ponto perdido):
          (1) banda em x ao redor do `front` do muro;
          (2) fit TLS + so INLIERS a <= 2,5 cm da reta;
          (3) segmentacao por GAP ao longo da reta: fica o segmento que
              contem o eixo do robo (py=0) — senao None (nunca "o maior":
              centraria na mesa vizinha);
          (4) residuo pequeno, largura plausivel E coerente com a largura
              medida no refino do staging, e as DUAS bordas longe do limite
              da janela lateral (borda cortada = centro enviesado).
        Matematica em face_center_from_points (pura, 2026-10-26). O desvio
        devolvido e' c_foot (pe' da perpendicular: o que a reta acerta
        DEPOIS de um esquadro, I4/F15) com foot=True, ou c_axis (onde o
        EIXO do robo fura a face) com foot=False; os dois ficam em
        _center_last para o log. Duas bandas (campo 22/08: WS2/WS6 tinham
        coisas 15-25 cm ATRAS da face e o fit unico saia com residuo de
        6-13 cm): a reta usa so a banda RASA (+10 cm); a funda entra so
        como fonte de inliers para largura/bordas.
        """
        scan = self._last_scan
        if scan is None or front is None:
            self._center_reject = "sem scan/front"
            return None
        pts = self._scan_points_in_base(scan)
        if pts is None:
            self._center_reject = "sem TF"
            return None
        c_axis, c_foot, width, reason = face_center_from_points(
            pts, front,
            self.get_parameter("center_half_width_y").value,
            self.get_parameter("center_edge_margin").value,
            self.get_parameter("center_max_residual").value,
            self.get_parameter("center_width_min").value,
            self.get_parameter("center_width_max").value)
        if reason:
            self._center_reject = reason
            return None
        self._center_last = (c_axis, c_foot)
        return (c_foot if foot else c_axis), width

    def _resolve_stop_dist(self):
        """Distancia de parada frontal: param > por-dock > global > desligado."""
        stop = self.get_parameter("stop_face_dist").value
        if stop is None or stop < 0.0:
            stop = self._dock_face_dist if self._dock_face_dist is not None \
                else self.get_parameter("refine_desired_face_dist").value
        return stop if stop is not None and stop >= 0.0 else None

    def _wait_new_scan(self, max_s, rate=None, goal_handle=None):
        """Espera um /scan NOVO (contador de chegada) por ate max_s. True se
        chegou. Respeita cancel do goal; sem rate dorme 20 ms por volta."""
        seq0 = self._scan_seq
        start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok() and self._scan_seq == seq0:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False
            if (self.get_clock().now().nanoseconds - start_ns) * 1e-9 > max_s:
                return False
            if rate is not None:
                rate.sleep()
            else:
                time.sleep(0.02)
        return self._scan_seq != seq0

    def _fit_face(self, x_lo, x_hi, tag="refino"):
        """Fit TLS da face nos pontos do /scan ATUAL com x em [x_lo, x_hi] e
        |y| <= refine_half_width_y, com as guardas anti-falso-positivo
        (idade do scan, largura maxima/residuo/largura minima — nesta
        ordem, ver face_fit_reject_reason —, inclinacao e distancia).
        Devolve _FaceFit (ok=False traz o motivo). Loga SEMPRE a linha
        'refino face: ...' (instrumento de calibracao do
        refine_desired_face_dist). Marca _refine_wide_face quando a
        rejeicao foi 'face larga demais' (mesa + vizinho)."""
        scan = self._last_scan
        if scan is None:
            self.get_logger().warn(f"{tag}: nenhum /scan recebido ainda.")
            return _FaceFit(False, "sem scan", None, None, None, None, 0)
        # Guard de idade (2026-08-03): um /scan CONGELADO (LiDAR/USB) faria o
        # alvo seguir o robo — runaway rumo a mesa. Scan velho -> sem
        # confianca. Idade pela hora de CHEGADA (v4), como no muro.
        scan_age = (self.get_clock().now().nanoseconds - self._last_scan_rx_ns) * 1e-9
        if scan_age > 0.5:
            self.get_logger().warn(
                f"{tag}: /scan com {scan_age:.1f}s de idade; descartado.")
            return _FaceFit(False, "scan velho", None, None, None, None, 0)
        # Janela lateral: por dock (docking.yaml, R3 2026-10-26) > global.
        hy = self._dock_refine_half_width
        if hy is None:
            hy = self.get_parameter("refine_half_width_y").value
        max_residual = self.get_parameter("refine_max_residual").value
        min_width = self.get_parameter("refine_min_face_width").value
        max_width = float(self.get_parameter("refine_max_face_width").value)

        all_pts = self._scan_points_in_base(scan)
        if all_pts is None:
            return _FaceFit(False, "sem TF", None, None, None, None, 0)
        pts = [p for p in all_pts if x_lo <= p[0] <= x_hi and abs(p[1]) <= hy]

        fit = _fit_line_tls(pts)
        if fit is None:
            self.get_logger().warn(
                f"{tag}: pontos insuficientes na janela frontal ({len(pts)} < 8).")
            return _FaceFit(False, "poucos pontos", None, None, None, None, len(pts))
        cx, cy, nx, ny = fit
        # Normal apontando para FRENTE do robo (da base em direcao a face).
        if nx < 0:
            nx, ny = -nx, -ny
        face_dist = cx * nx + cy * ny
        face_yaw = math.atan2(ny, nx)  # orientacao da normal no frame do robo
        # 2026-10-26 (I1b): o yaw de TODO fit do refino do staging, aceito
        # ou nao, fica guardado para vetar o esquadro por AMCL (F13: na WS6
        # os fits rejeitados por residuo diziam +5/+6 e o AMCL girou -9,7).
        if tag.startswith("refino"):
            self._refine_fit_yaws.append(face_yaw)

        # Guardas anti-falso-positivo: face plana (residuo), larga o bastante
        # (nao e' perna de mesa/objeto) mas nao demais (mesa + vizinho),
        # pouco inclinada e a distancia sensata.
        residual_rms = math.sqrt(
            sum(((p[0] - cx) * nx + (p[1] - cy) * ny) ** 2 for p in pts) / len(pts))
        tang = (-ny, nx)
        along = [(p[0] - cx) * tang[0] + (p[1] - cy) * tang[1] for p in pts]
        face_width = max(along) - min(along)

        # Log SEMPRE (e' o instrumento de calibracao do refine_desired_face_dist).
        self.get_logger().info(
            f"{tag} face: dist={face_dist:.3f} m, yaw_rel={math.degrees(face_yaw):.1f} deg, "
            f"largura={face_width:.2f} m, residuo={residual_rms*1000:.0f} mm, pts={len(pts)}, "
            f"janela +-{hy:.2f}")

        reason = face_fit_reject_reason(
            residual_rms, face_width, min_width, max_width, max_residual)
        if reason:
            if reason.startswith("face larga"):
                self._refine_wide_face = True
            self.get_logger().warn(f"{tag}: {reason}; descartando.")
            return _FaceFit(False, reason, face_dist, face_yaw, face_width,
                            residual_rms, len(pts))
        if abs(_normalize_angle(face_yaw)) > math.radians(30) or not (0.05 < face_dist < x_hi):
            self.get_logger().warn(f"{tag}: face torta/distante demais; descartando.")
            return _FaceFit(False, "face torta/distante demais", face_dist, face_yaw,
                            face_width, residual_rms, len(pts))
        return _FaceFit(True, "", face_dist, face_yaw, face_width, residual_rms, len(pts))

    def _refit_face_band(self, front, tag):
        """Fit da face numa banda RASA ao redor do muro ([front-0,03,
        front+0,30]): perto da mesa (<= ~0,7 m) a face aparece isolada dos
        vizinhos e do que ha' atras (a janela global do refino, ate 1,20 m,
        e' o que pegava mesa + vizinho no staging). +0,30 cobre uma face de
        0,9 m vista a ate ~18 graus."""
        return self._fit_face(front - 0.03, front + 0.30, tag=tag)

    def _refine_target_from_scan(self, target, rate=None, goal_handle=None):
        """Refina a pose alvo pela FACE da mesa no /scan.

        Corrige YAW (esquadra com a normal da face) e, se
        refine_desired_face_dist estiver calibrado (>=0), tambem X (distancia
        perpendicular a face). Y (lateral) continua por conta da localizacao
        (problema da abertura, ADR-05) — no pick, a camera fecha a lateral.

        Retorna (target_refinado, confiante: bool). Sem confianca, o chamador
        usa a pose do banco (comportamento identico ao antigo).
        """
        fx_min = self.get_parameter("refine_front_min_x").value
        fx_max = self.get_parameter("refine_front_max_x").value
        # Distancia desejada: mesma resolucao do muro (por-dock > global).
        desired_dist = self._resolve_stop_dist()
        desired_dist = desired_dist if desired_dist is not None else -1.0

        fit = self._fit_face(fx_min, fx_max)
        if not fit.ok:
            return target, False
        face_dist, face_yaw = fit.dist, fit.yaw

        # Largura da face vista daqui (staging): referencia de coerencia do
        # estimador de centro perto da mesa (v4).
        self._last_face_width = fit.width

        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn("refino: sem TF do robo; descartando.")
            return target, False
        rx, ry, ryaw = pose

        tx, ty, tyaw = target
        # Yaw alvo: esquadrar com a normal da face (direcao dela no mapa) —
        # MAS so quando habilitado E dois fits confiantes consecutivos
        # concordarem (2026-08-18: operador viu o refino PIORAR o yaw as
        # vezes; um fit isolado nunca mais gira o alvo sozinho).
        face_yaw_map = _normalize_angle(ryaw + face_yaw)
        apply_yaw = bool(self.get_parameter("refine_correct_yaw").value)
        if apply_yaw:
            agree_tol = self.get_parameter("refine_yaw_agree_tol").value
            if (self._last_face_yaw_map is None or
                    abs(_normalize_angle(face_yaw_map - self._last_face_yaw_map))
                    > agree_tol):
                # 2a leitura IMEDIATA (2026-10-21): antes o portao so' fechava
                # no goal seguinte — apos o esquadro por AMCL havia UMA leitura
                # por goal e o yaw ficava "MANTIDO (portao)" com -15 graus na
                # cara (run da WS1). Espera um /scan NOVO e refaz o fit com as
                # MESMAS guardas; aplica so' se os dois fits confiaveis
                # concordarem dentro de agree_tol.
                max_s = float(self.get_parameter("refine_second_read_max_s").value)
                second = None
                if self._wait_new_scan(max_s, rate, goal_handle):
                    second = self._fit_face(fx_min, fx_max, tag="refino(2a leitura)")
                if second is not None and second.ok:
                    pose2 = self._robot_pose()
                    ryaw2 = pose2[2] if pose2 is not None else ryaw
                    yaw2_map = _normalize_angle(ryaw2 + second.yaw)
                    if abs(_normalize_angle(yaw2_map - face_yaw_map)) <= agree_tol:
                        self.get_logger().info(
                            "refino yaw: 2a leitura concorda "
                            f"({math.degrees(face_yaw_map):.1f} vs {math.degrees(yaw2_map):.1f} deg "
                            "no mapa) — aplico no mesmo goal.")
                        face_dist, face_yaw = second.dist, second.yaw
                        face_yaw_map = yaw2_map
                        self._last_face_width = second.width
                    else:
                        self.get_logger().info(
                            "refino yaw: 2a leitura DISCORDA "
                            f"({math.degrees(face_yaw_map):.1f} vs {math.degrees(yaw2_map):.1f} deg "
                            f"no mapa, tol {math.degrees(agree_tol):.1f}) — yaw mantido.")
                        apply_yaw = False
                else:
                    why = "sem /scan novo" if second is None else second.reason
                    self.get_logger().info(
                        f"refino yaw: sem 2a leitura confiavel em {max_s:.1f} s ({why}) — "
                        f"aguardando proxima (esta: {math.degrees(face_yaw_map):.1f} deg no mapa).")
                    apply_yaw = False
        self._last_face_yaw_map = face_yaw_map
        refined_yaw = face_yaw_map if apply_yaw else tyaw

        refined_tx, refined_ty = tx, ty
        if desired_dist >= 0.0:
            # Plano da face no mapa: p . n_map = c_face. Move o alvo do banco ao
            # longo da normal para que a distancia final robo->face = calibrada.
            n_map = (math.cos(ryaw + face_yaw), math.sin(ryaw + face_yaw))
            c_face = rx * n_map[0] + ry * n_map[1] + face_dist
            desired_proj = c_face - desired_dist
            shift = desired_proj - (tx * n_map[0] + ty * n_map[1])
            refined_tx = tx + shift * n_map[0]
            refined_ty = ty + shift * n_map[1]
            self.get_logger().info(
                f"refino aplicado: X ajustado {shift*100:+.1f} cm ao longo da normal "
                f"(alvo dist={desired_dist:.3f} m); yaw "
                f"{'esquadrado %+.1f deg' % math.degrees(face_yaw) if apply_yaw else 'MANTIDO (portao/desligado)'}. "
                "Y mantido do banco.")
        else:
            self.get_logger().info(
                "refino aplicado: distancia nao calibrada (X do banco); yaw "
                f"{'esquadrado %+.1f deg' % math.degrees(face_yaw) if apply_yaw else 'MANTIDO (portao/desligado)'}.")
        return (refined_tx, refined_ty, refined_yaw), True

    # ------------------------------------------------------------- Controle
    def _publish_cmd(self, vx, vy, wz):
        # Eixo PURO por construcao (v4): o piso do ESC e' por roda; comando
        # misto deixa rodas sub-piso e o driver arredonda — o movimento sai
        # torto. Qualquer mistura aqui e' bug de logica.
        assert sum(1 for v in (vx, vy, wz) if abs(v) > 1e-9) <= 1, (vx, vy, wz)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self._cmd_pub.publish(msg)

    def _start_speeds(self):
        """(v, w) de PARTIDA do repouso = piso x (1 + floor_start_margin).
        Ver _floor_start_speed (2026-10-19): no piso exato a roda fica um
        ULP abaixo do piso do ESC e nao liga. Margem negativa/NaN e' avisada
        UMA vez e vale 0 (nunca partir abaixo do piso)."""
        margin = float(self.get_parameter("floor_start_margin").value)
        if not math.isfinite(margin) or margin < 0.0:
            if not self._margin_warned:
                self._margin_warned = True
                self.get_logger().warn(
                    f"floor_start_margin={margin} invalido (negativo/NaN): uso 0 — "
                    "partida no piso exato pode nao ligar as rodas (campo 2026-10-19)")
            margin = 0.0
        return (_floor_start_speed(float(self.get_parameter("min_body_vel").value), margin),
                _floor_start_speed(float(self.get_parameter("min_body_omega").value), margin))

    # ------------------------------------------- Primitivas de movimento (v5)
    class _AttemptAbort(Exception):
        """Encerra a tentativa de dentro das primitivas (hard_stop, cancel,
        anomalia). `outcome` e' o mesmo vocabulario de _run_attempt."""

        def __init__(self, outcome, info=None):
            super().__init__(outcome)
            self.outcome = outcome
            self.info = info

    def _check_guards(self, goal_handle, deadline, attempt_end):
        """Cancel/timeouts/hard_min — chamado a cada ciclo de QUALQUER
        primitiva. Devolve (front, npts, face_yaw) do muro."""
        if goal_handle.is_cancel_requested:
            self._publish_cmd(0.0, 0.0, 0.0)
            raise self._AttemptAbort("canceled")
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s > deadline:
            self._publish_cmd(0.0, 0.0, 0.0)
            raise self._AttemptAbort("timeout", "timeout no alinhamento")
        if now_s > attempt_end:
            self._publish_cmd(0.0, 0.0, 0.0)
            raise self._AttemptAbort(
                "soft_timeout", "tentativa estourou o orcamento")
        front, npts, age, face_yaw = self._front_wall_distance()
        self._wall_age = age
        self._live_face_yaw = face_yaw
        if front is not None:
            self._saw_face_this_goal = True
        hard_min = self.get_parameter("hard_min_face_dist").value
        if (bool(self.get_parameter("wall_enabled").value)
                and front is not None and front < hard_min):
            self._publish_cmd(0.0, 0.0, 0.0)
            raise self._AttemptAbort(
                "hard_stop", f"muro: face a {front:.3f}m < hard_min {hard_min:.3f}m")
        return front, npts, face_yaw

    def _settle(self, goal_handle, rate, deadline, attempt_end, phase_name):
        """Para (publicando zero) ate o front ficar ESTAVEL em N scans novos
        ou ate o teto de tempo. Devolve (front, npts, face_yaw_mediana).
        front None = sem medicao (STALE/BLIND) apos o teto."""
        n_stable = int(self.get_parameter("settle_stable_scans").value)
        eps = self.get_parameter("settle_stable_eps").value
        max_s = self.get_parameter("settle_max_s").value
        start_s = self.get_clock().now().nanoseconds * 1e-9
        last_seq = -1
        hist = []
        yaws = []
        front = npts = None
        while rclpy.ok():
            front, npts, face_yaw = self._check_guards(goal_handle, deadline, attempt_end)
            self._publish_cmd(0.0, 0.0, 0.0)
            self._feedback(goal_handle, phase_name, 0.0, 0.0, 0.0)
            if self._scan_seq != last_seq and front is not None:
                last_seq = self._scan_seq
                hist.append(front)
                if face_yaw is not None:
                    yaws.append(face_yaw)
                hist = hist[-n_stable:]
                yaws = yaws[-5:]
                if len(hist) == n_stable and max(hist) - min(hist) <= eps:
                    break
            if self.get_clock().now().nanoseconds * 1e-9 - start_s > max_s:
                break
            rate.sleep()
        face_yaw_f = sorted(yaws)[len(yaws) // 2] if yaws else None
        return front, npts, face_yaw_f

    def _cut_drive(self, goal_handle, rate, deadline, attempt_end, axis, sign,
                   error_fn, coast, max_s, phase_name, lag_v=0.0):
        """Movimento CONTINUO no piso do eixo, cortado quando o erro vivo
        (error_fn() -> valor com sinal, ou None) fica <= coast — o ESC retem a
        referencia ~0,5 s e termina o curso. Sem medicao por 3 scans novos ou
        inversao de sinal: corta. Devolve o resultado do _settle.
        lag_v > 0 (2026-10-26, R6/F15): corta pelo ponto de parada PREVISTO,
        coast + lag_v*(idade do scan + periodo), como a reta e o nudge
        (lateral_cut_reached); 0 = corte cru em err <= coast (giro: R4)."""
        # Parte do repouso: piso x (1 + margem) nos 3 eixos, senao a roda
        # fica um ULP abaixo do piso do ESC e nao liga (2026-10-19).
        min_v, min_w = self._start_speeds()
        period = 1.0 / float(self.get_parameter("control_frequency").value)
        start_s = self.get_clock().now().nanoseconds * 1e-9
        miss = 0
        last_seq = -1
        odom0 = self._odom
        reason = "max_s"
        last_err = None
        now_s = start_s   # o log do corte usa now_s mesmo se o laco nao rodar (shutdown)
        while rclpy.ok():
            front, _n, _fy = self._check_guards(goal_handle, deadline, attempt_end)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - start_s > max_s:
                reason = "max_s"
                break
            if self._scan_seq != last_seq:
                last_seq = self._scan_seq
                err = error_fn(front)
                if err is None:
                    miss += 1
                    if miss >= 3:
                        reason = "3 scans sem medida"
                        break
                else:
                    miss = 0
                    last_err = err
                    age = self._wall_age
                    if age is None or not math.isfinite(age):
                        age = 0.0
                    if lateral_cut_reached(err, sign, coast, lag_v, age, period):
                        # falta so o coast (+ lag): corta e deixa o ESC terminar
                        reason = "erro<=coast+lag" if lag_v > 0.0 else "erro<=coast"
                        break
            if axis == "x":
                self._publish_cmd(math.copysign(min_v, sign), 0.0, 0.0)
            elif axis == "y":
                self._publish_cmd(0.0, math.copysign(min_v, sign), 0.0)
            else:
                self._publish_cmd(0.0, 0.0, math.copysign(min_w, sign))
            self._feedback(goal_handle, phase_name, 0.0, 0.0, 0.0)
            rate.sleep()
        self._publish_cmd(0.0, 0.0, 0.0)
        # 2026-10-26 (I6): motivo do corte, ultimo erro, idade do scan e o
        # curso por odometria no corte e depois do settle (= coast real).
        err_txt = "?" if last_err is None else (
            f"{math.degrees(last_err):+.1f} graus" if axis == "z" else f"{last_err * 100:+.1f} cm")
        age_txt = "?" if self._wall_age is None or not math.isfinite(self._wall_age) \
            else f"{self._wall_age:.2f} s"
        self.get_logger().info(
            f"[inst] corte({phase_name}/{axis}): motivo={reason}, ultimo_erro={err_txt}, "
            f"idade_scan={age_txt}, curso_odom={self._odom_course(odom0, axis)} "
            f"({now_s - start_s:.1f} s)")
        res = self._settle(goal_handle, rate, deadline, attempt_end, phase_name)
        self.get_logger().info(
            f"[inst] apos settle({phase_name}/{axis}): curso_odom="
            f"{self._odom_course(odom0, axis)} (inclui o coast do ESC)")
        return res

    def _rotate_pulses(self, goal_handle, rate, deadline, attempt_end, target, tag,
                       phase="esquadrando"):
        """Gira `target` rad (+ = CCW) em PULSOS com medida entre eles: o
        giro de cada pulso e' medido PARADO pela variacao do yaw da face no
        LiDAR (mediana do _settle; sem face, odometria). Devolve
        (girado, restante, (front, npts, face_yaw) do ultimo settle)."""
        p = self.get_parameter
        if self._rot_model is None:
            self._rot_model = (float(p("rot_pulse_c0").value), float(p("rot_pulse_k0").value))
        t_min = float(p("rot_pulse_t_min").value)
        t_max = float(p("rot_pulse_t_max").value)
        tol = float(p("rot_pulse_tol").value)
        n_max = max(1, int(p("rot_pulse_max").value))
        _v, w_start = self._start_speeds()
        remaining = target
        done = 0.0
        res = self._settle(goal_handle, rate, deadline, attempt_end, phase)
        for i in range(n_max):
            c, k = self._rot_model
            t, why = plan_rot_pulse(remaining, c, k, t_min, t_max, tol)
            if t is None:
                self.get_logger().info(
                    f"{tag} giro em pulsos: falta {math.degrees(remaining):+.1f} graus — "
                    + ("dentro da tolerancia" if why == "dentro" else
                       f"o menor pulso (~{math.degrees(c + k * t_min):.1f} graus) passaria do ponto; paro"))
                break
            yaw_b = res[2]
            o_b = self._odom_pose(0.5)
            sign = 1.0 if remaining > 0 else -1.0
            t0 = self.get_clock().now().nanoseconds * 1e-9
            while self.get_clock().now().nanoseconds * 1e-9 - t0 < t:
                self._check_guards(goal_handle, deadline, attempt_end)
                self._publish_cmd(0.0, 0.0, math.copysign(w_start, sign))
                self._feedback(goal_handle, phase, 0.0, 0.0, remaining)
                rate.sleep()
            self._publish_cmd(0.0, 0.0, 0.0)
            self._wait_still(goal_handle, rate, deadline, attempt_end, phase)
            res = self._settle(goal_handle, rate, deadline, attempt_end, phase)
            yaw_a = res[2]
            o_a = self._odom_pose(0.5)
            if yaw_b is not None and yaw_a is not None:
                rot, src = yaw_b - yaw_a, "lidar"
            elif o_b is not None and o_a is not None:
                rot, src = _normalize_angle(o_a[2] - o_b[2]), "odometria"
            else:
                self.get_logger().warn(f"{tag} giro em pulsos: sem como medir o giro — paro")
                break
            done += rot
            remaining -= rot
            self._rot_model = update_rot_model(c, k, t, rot * sign, t_min)
            self.get_logger().info(
                f"{tag} pulso {i + 1}: {t:.2f} s -> girou {math.degrees(rot):+.1f} graus ({src}); "
                f"falta {math.degrees(remaining):+.1f}; modelo c={math.degrees(self._rot_model[0]):.1f} "
                f"graus k={math.degrees(self._rot_model[1]):.1f} graus/s")
        return done, remaining, res

    def _odom_course(self, odom0, axis):
        """Curso por odometria desde odom0 no eixo da primitiva, como texto
        para o log (2026-10-26, I6): x = longitudinal, y = lateral
        (_lateral_displacement, + = esquerda), z = giro. '?' sem odometria."""
        odom = self._odom
        if odom0 is None or odom is None:
            return "?"
        x0, y0, yaw0 = odom0[0], odom0[1], odom0[2]
        if axis == "x":
            return f"{_longitudinal_displacement(x0, y0, yaw0, odom[0], odom[1]) * 100:+.1f} cm"
        if axis == "y":
            return f"{_lateral_displacement(x0, y0, yaw0, odom[0], odom[1]) * 100:+.1f} cm"
        return f"{math.degrees(_normalize_angle(odom[2] - yaw0)):+.1f} graus"

    def _wait_still(self, goal_handle, rate, deadline, attempt_end, phase_name,
                    v_max=None, w_max=None, min_s=None, max_s=None):
        """2026-10-26 (I3c, F14/C2): publica zero e espera a base PARAR antes
        de medir/decidir: /odom com |v| <= still_v_max e |wz| <= still_w_max
        por still_min_s; teto still_max_s — estourou, avisa e segue. O Nav2
        declara 'Reached the goal' com a base ainda girando (TF +6,7..+7,8
        graus nos 0,4 s seguintes) e o refino e o esquadro partiam com o
        erro mudando ~15 graus/s. A estabilidade do yaw da face do fit leve
        (ultimos settle_stable_scans dentro de still_yaw_eps) e' ADVISORIA
        quando ha' /odom (so' vai para o log: o fit leve oscila +-4 graus
        entre scans e um gate por ele batia no teto em todo docking) e vira
        o criterio apenas SEM /odom. Devolve
        (esperou_s, odom_parado, yaw_estavel, yaw_inicial, yaw_final)."""
        p = self.get_parameter
        v_max = float(p("still_v_max").value) if v_max is None else float(v_max)
        w_max = float(p("still_w_max").value) if w_max is None else float(w_max)
        min_s = float(p("still_min_s").value) if min_s is None else float(min_s)
        max_s = float(p("still_max_s").value) if max_s is None else float(max_s)
        yaw_eps = float(p("still_yaw_eps").value)
        n_stable = max(1, int(p("settle_stable_scans").value))
        win = _StillWindow(v_max, w_max, min_s)
        t0 = self.get_clock().now().nanoseconds * 1e-9
        yaws = []
        scans = 0
        last_seq = -1
        last_tw_ns = None
        still = False
        yaw_ok = False
        done = False
        while rclpy.ok():
            _front, _n, face_yaw = self._check_guards(goal_handle, deadline, attempt_end)
            self._publish_cmd(0.0, 0.0, 0.0)
            self._feedback(goal_handle, phase_name, 0.0, 0.0, 0.0)
            t = self.get_clock().now().nanoseconds * 1e-9
            tw = self._odom_twist
            if tw is not None and tw[3] != last_tw_ns:
                last_tw_ns = tw[3]
                still = win.update(t, math.hypot(tw[0], tw[1]), tw[2])
            if self._scan_seq != last_seq:
                last_seq = self._scan_seq
                scans += 1
                if face_yaw is not None:
                    yaws.append(face_yaw)
                # Sem yaw da face em n scans: nada a estabilizar pelo LiDAR.
                yaw_ok = (yaw_readings_stable(yaws, n_stable, yaw_eps)
                          or (scans >= n_stable and not yaws))
            # Com /odom o gate e' a odometria (o yaw so' informa); sem /odom
            # o yaw da face e' o unico sinal de parada que existe.
            done = still if self._odom_twist is not None else yaw_ok
            if done:
                break
            if t - t0 > max_s:
                break
            rate.sleep()
        waited = self.get_clock().now().nanoseconds * 1e-9 - t0
        y0 = yaws[0] if yaws else None
        y1 = yaws[-1] if yaws else None
        tail = yaws[-n_stable:] if yaws else []
        spread_txt = "?" if not tail else f"{math.degrees(max(tail) - min(tail)):.1f}"
        yaw_txt = "sem yaw da face" if y0 is None else (
            f"{math.degrees(y0):+.1f} -> {math.degrees(y1):+.1f} graus "
            f"(delta {math.degrees(y1 - y0):+.1f}, {len(yaws)} leituras, "
            f"espalh ult{len(tail)}={spread_txt} graus vs eps {math.degrees(yaw_eps):.1f})")
        line = (f"[inst] parada({phase_name}): esperou {waited:.2f} s, odom parado="
                f"{still}{'' if self._odom_twist is not None else ' (sem /odom)'}, "
                f"yaw_face estavel={yaw_ok}: {yaw_txt}")
        if done:
            self.get_logger().info(line)
        else:
            self.get_logger().warn(line + f" — teto {max_s:.1f} s estourado, sigo assim mesmo")
        return waited, still, yaw_ok, y0, y1

    def _measure_center(self, goal_handle, rate, deadline, attempt_end, foot=False):
        """Mediana de N medidas do centro da face em scans DISTINTOS; None se
        o estimador nao confiou ou as medidas divergiram. foot: ver
        _face_center_from_scan (I4). As medianas de c_axis e c_foot ficam
        em _center_medians para o log (2026-10-26)."""
        n = int(self.get_parameter("center_samples").value)
        spread_max = self.get_parameter("center_spread_max").value
        vals = []
        widths = []
        axis_vals = []
        foot_vals = []
        self._center_medians = (None, None)
        last_seq = -1
        tries = 0
        start_s = self.get_clock().now().nanoseconds * 1e-9
        while len(vals) < n and tries < 4 * n:
            if self.get_clock().now().nanoseconds * 1e-9 - start_s > 3.0:
                break  # sem scans novos (STALE): nao esperar o orcamento inteiro
            front, _npts, _fy = self._check_guards(goal_handle, deadline, attempt_end)
            self._publish_cmd(0.0, 0.0, 0.0)
            if self._scan_seq != last_seq:
                last_seq = self._scan_seq
                tries += 1
                res = self._face_center_from_scan(front, foot=foot)
                if res is not None:
                    vals.append(res[0])
                    widths.append(res[1])
                    axis_vals.append(self._center_last[0])
                    foot_vals.append(self._center_last[1])
            rate.sleep()
        if axis_vals:
            axis_vals.sort()
            foot_vals.sort()
            self._center_medians = (axis_vals[len(axis_vals) // 2],
                                    foot_vals[len(foot_vals) // 2])
        if len(vals) < max(3, n - 1):
            self.get_logger().info(
                f"centro: {len(vals)}/{n} medidas validas — ultima rejeicao: {self._center_reject}")
            return None
        vals.sort()
        widths.sort()
        if vals[-1] - vals[0] > spread_max or widths[-1] - widths[0] > 2.0 * spread_max:
            self.get_logger().info(
                f"centro: medidas dispersas (centro {vals[0]*100:+.1f}..{vals[-1]*100:+.1f} cm, "
                f"largura {widths[0]:.2f}..{widths[-1]:.2f})")
            return None
        return vals[len(vals) // 2]

    def _measure_face_yaw(self, goal_handle, rate, deadline, attempt_end):
        """Yaw da face no fim do docking (2026-10-21): mediana do face_yaw do
        fit leve do muro (_front_wall_distance, o mesmo que o 'pronto:'
        sempre logou) em ate center_samples scans DISTINTOS, parado.
        Devolve (yaw, "") ou (None, motivo) quando faltam fits (< 3) ou
        eles dispersam mais que final_yaw_spread_max — ai' o nó reporta
        'yaw_nao_medido' em vez de girar as cegas pelo AMCL."""
        n = int(self.get_parameter("center_samples").value)
        spread_max = float(self.get_parameter("final_yaw_spread_max").value)
        vals = []
        reject = collections.Counter()
        last_seq = -1
        tries = 0
        start_s = self.get_clock().now().nanoseconds * 1e-9
        while len(vals) < n and tries < 4 * n:
            if self.get_clock().now().nanoseconds * 1e-9 - start_s > 3.0:
                break  # sem scans novos (STALE): nao esperar o orcamento inteiro
            _front, _npts, face_yaw = self._check_guards(goal_handle, deadline, attempt_end)
            self._publish_cmd(0.0, 0.0, 0.0)
            if self._scan_seq != last_seq:
                last_seq = self._scan_seq
                tries += 1
                if face_yaw is not None:
                    vals.append(face_yaw)
                else:
                    reject[self._wall_fit_reject or "sem muro"] += 1
            rate.sleep()
        # 2026-10-26 (I6): uma linha por chamada com os motivos agregados
        # (residuo/extensao/npts/35 graus) — hoje so' saia "x/5 fits".
        rej_txt = ", ".join(f"{k} x{v}" for k, v in reject.most_common()) or "nenhuma"
        self.get_logger().info(
            f"[inst] yaw_final: {len(vals)}/{tries} fits leves da face; rejeicoes: {rej_txt}")
        if len(vals) < max(3, n - 1):
            return None, f"{len(vals)}/{n} fits confiaveis da face"
        vals.sort()
        if vals[-1] - vals[0] > spread_max:
            return None, (f"leituras dispersas ({math.degrees(vals[0]):+.1f}.."
                          f"{math.degrees(vals[-1]):+.1f} graus > "
                          f"{math.degrees(spread_max):.1f})")
        return vals[len(vals) // 2], ""

    def _run_attempt(self, goal_handle, bank_target, g, xy_tol, yaw_tol,
                     deadline, attempt_budget, rate, attempt, attempts,
                     staging_offset):
        """v5 (pedido do operador 2026-08-21, 'o pre-docking esta otimo'):
        do staging, (A0, 2026-10-26) espera a base parar, refina e esquadra
        pelo LiDAR ANTES de centrar, (A) lateral pelo centro da face (pe' da
        perpendicular), (B) RETO no piso guiado
        pelo muro ate a distancia de parada (corte descontando o coast
        medido), com UMA pausa de refit/esquadro pelo LiDAR a
        ~refine_refit_front_m, (C) retoque lateral na mesa e (D) validacao
        final de yaw pela face (2026-10-21): fora da tolerancia recua ate
        rotate_min_front+0,05, esquadra pela face e refaz (B)+(C) uma vez.
        Sem retry de aproximacao — se nao der, para onde esta (o recuo so
        acontece na falha, em _execute_align, para nao deixar o robo colado
        na mesa).

        Retorna (outcome, info); "success" -> (ex, ey, eyaw, note) com note
        "" | 'yaw_fora_da_tolerancia (x graus)' | 'yaw_nao_medido';
        "yaw_out" (final_yaw_strict) -> info = a mesma nota.
        """
        max_vx = self.get_parameter("max_vel_x").value
        kp = self.get_parameter("kp_linear").value
        min_v = self.get_parameter("min_body_vel").value
        period = 1.0 / float(self.get_parameter("control_frequency").value)
        band = self.get_parameter("wall_hysteresis_band").value
        coast = self.get_parameter("esc_coast_dist").value
        center_coast = self.get_parameter("center_coast_est").value
        center_enabled = bool(self.get_parameter("center_enabled").value)
        center_deadband = self.get_parameter("center_deadband").value
        center_max_travel = self.get_parameter("center_max_travel").value
        lateral_min = self.get_parameter("lateral_min_front").value
        stale_abort_s = self.get_parameter("stale_abort_s").value
        blind_wait_s = self.get_parameter("blind_wait_s").value
        hard_min = self.get_parameter("hard_min_face_dist").value

        target = bank_target
        tx, ty, tyaw = target
        confident = False
        self._refine_wide_face = False
        self._refine_fit_yaws = []

        start_s = self.get_clock().now().nanoseconds * 1e-9
        attempt_end = start_s + attempt_budget
        tag = f"[{g.dock_id}]"

        def log(msg):
            self.get_logger().info(f"{tag} {msg}")

        def now():
            return self.get_clock().now().nanoseconds * 1e-9

        def amcl_errors():
            # Le' tx/ty/tyaw do escopo externo NA CHAMADA: segue o alvo
            # refinado depois de cada refino.
            pose = self._robot_pose()
            if pose is None:
                return 0.0, 0.0, 0.0
            rx, ry, ryaw = pose
            dx, dy = tx - rx, ty - ry
            return (math.cos(ryaw) * dx + math.sin(ryaw) * dy,
                    -math.sin(ryaw) * dx + math.cos(ryaw) * dy,
                    _normalize_angle(tyaw - ryaw))

        def inst(where):
            """2026-10-26 (I6): uma linha '[inst]' com AMCL (x, y, yaw do
            alvo no frame do robo), LiDAR (front e yaw da face do fit leve)
            e twist do /odom nas transicoes do docking, para o proximo tour
            separar 'AMCL enviesado' de 'base ainda girando' (F14)."""
            ex, ey, eyaw = amcl_errors()
            front, _n, age, fy = self._front_wall_distance()
            tw = self._odom_twist
            tw_txt = "sem /odom" if tw is None else (
                f"v={math.hypot(tw[0], tw[1]):.3f} m/s wz={tw[2]:+.3f} rad/s "
                f"idade={(now() - tw[3] * 1e-9):.2f} s")
            log(f"[inst] {where}: amcl x={ex * 100:+.1f} y={ey * 100:+.1f} cm "
                f"yaw={math.degrees(eyaw):+.1f} graus | lidar front="
                f"{'?' if front is None else '%.3f' % front} yaw_face="
                f"{'?' if fy is None else '%+.1f' % math.degrees(fy)} graus "
                f"idade={'?' if not math.isfinite(age) else '%.2f' % age} s | odom {tw_txt}")

        inst("entrada do goal")
        # 2026-10-26 (I3c, F14/C2): o Nav2 declara chegada com a base ainda
        # girando (TF +6,7..+7,8 graus nos 0,4 s seguintes; yaw_rel do LiDAR
        # derivando -4,6 -> -9,7 em 0,33 s na WS1-T2). Antes de QUALQUER
        # medida: zero na referencia e espera /odom parado (o yaw da face
        # do fit leve so' vai para o log; teto still_max_s; estourou, avisa
        # e segue).
        # Cancel/hard_stop aqui (antes do try principal) encerram a tentativa.
        try:
            self._wait_still(goal_handle, rate, deadline, attempt_end, "parando")
        except self._AttemptAbort as ab:
            self._publish_cmd(0.0, 0.0, 0.0)
            return ab.outcome, ab.info

        if g.use_lidar_refine:
            self._feedback(goal_handle, "refinando", 0.0, 0.0, 0.0)
            first = self._refine_target_from_scan(target, rate, goal_handle)
            for _ in range(max(1, int(0.3 / period))):
                rate.sleep()
            second = self._refine_target_from_scan(first[0], rate, goal_handle)
            # I2 (F13, WS1-T2): a 2a chamada NAO sobrescreve a 1a — confiante
            # se qualquer uma foi; alvo = o da ultima confiante.
            target, confident = merge_refine_results(bank_target, first, second)
            tx, ty, tyaw = target
            if not confident:
                self.get_logger().warn(
                    "refino por LiDAR sem confianca (2 leituras); usando pose do banco.")
            elif not second[1]:
                log("refino: 2a leitura sem confianca — mantenho o alvo refinado da 1a")
            inst("apos refino")
        tx, ty, tyaw = target

        stop = self._resolve_stop_dist()
        if stop is None:
            return "soft_timeout", "sem distancia de parada (stop_face_dist)"
        if stop < hard_min + 0.03 - 1e-6:
            self.get_logger().warn(
                f"stop {stop:.3f} a menos de 3 cm do hard_min {hard_min:.3f}: "
                f"parada ajustada para {hard_min + 0.03:.3f}")
            stop = hard_min + 0.03
        window_lo = stop - 0.02
        window_hi = stop + band + max(0.04, xy_tol)

        # Sanidade: estou mesmo no pre-docking gravado? (dock + staging_offset
        # ao longo do yaw do dock). Longe demais = mesa errada/AMCL perdido:
        # abortar PARADO e' mais seguro que dockar na face errada.
        sanity = self.get_parameter("staging_sanity_max_m").value
        pose_now = self._robot_pose()
        if pose_now is not None and sanity > 0.0:
            sx = tx - staging_offset * math.cos(tyaw)
            sy = ty - staging_offset * math.sin(tyaw)
            d_staging = math.hypot(pose_now[0] - sx, pose_now[1] - sy)
            if d_staging > sanity:
                return "soft_timeout", (
                    f"fora do pre-docking: estou a {d_staging:.2f} m do staging "
                    f"gravado (max {sanity:.2f}) — mesa errada ou AMCL perdido; "
                    "nao vou me mover")

        rotate_min = self.get_parameter("rotate_min_front").value
        yaw_coast = self.get_parameter("yaw_coast_est").value
        agree_tol = self.get_parameter("refine_yaw_agree_tol").value
        refit_front = float(self.get_parameter("refine_refit_front_m").value)
        second_read_s = float(self.get_parameter("refine_second_read_max_s").value)
        # O giro so' corrige acima do coast (+0,02 rad de folga, como o
        # esquadro por AMCL sempre fez): abaixo disso o _cut_drive nem parte.
        yaw_min_corr, _ = amcl_square_window(yaw_coast, 0.0)
        # Giro em pulsos (2026-10-27): corrige abaixo do coast do giro
        # continuo; quem decide se o pulso cabe e' o _rotate_pulses.
        pulses = self.get_parameter("rotate_mode").value == "pulsos"
        yaw_min_rot = 0.0 if pulses else yaw_min_corr

        # Esquadro por AMCL: so quando o refino NAO confiou (mesa diagonal =
        # Nav2 chega torto e a vista de esguelha reprova o fit). Gira longe da
        # mesa (rotate_min_front) e refaz o refino ja de frente.
        # 2026-10-21: nunca quando a rejeicao foi 'face larga demais' (WS1:
        # AMCL girou +12,3 contra uma face que o LiDAR mediu a -15).
        # 2026-10-26 (I1, F13/C1): DESLIGADO por default (amcl_square_enabled)
        # — na WS6 girou -9,7/-9,4 graus CW nos dois tours contra fits
        # rejeitados por RESIDUO que diziam +5/+6 CCW (a guarda de 21/10 so'
        # cobria 'face larga'), e na WS1-T2 -11,9 contra o alvo recem-
        # refinado. Ligado, so' roda: (a) sem face NENHUMA no refino (poucos
        # pontos/sem scan), nunca quando achou e rejeitou por residuo/largura;
        # (c) com a base parada por /odom e a mediana de >= 5 amostras do erro
        # com dispersao <= 0,02 rad; (d) |erro| <= amcl_square_max_err; (b)
        # sem veto do LiDAR (amcl_square_veto: sinal oposto ou ja' dentro da
        # tolerancia). Desligado, loga o que teria feito.
        # PENDENTE (I1(e) do diagnostico): se o refino pos-giro reprovar, o
        # giro NAO e' desfeito pela odometria ("sigo pelo banco", como
        # antes). So' vale implementar depois de medir o coast real de giro
        # (teste T5); enquanto o default for False nao tem efeito.
        amcl_enabled = bool(self.get_parameter("amcl_square_enabled").value)
        amcl_max_err = float(self.get_parameter("amcl_square_max_err").value)
        # Cancel/hard_stop/timeout aqui (antes do try principal) encerram a
        # tentativa com o mesmo vocabulario de outcome.
        try:
            if g.use_lidar_refine and not confident:
                lidar_yaws = list(self._refine_fit_yaws)
                if self._live_face_yaw is not None:
                    lidar_yaws.append(self._live_face_yaw)
                lidar_txt = ", ".join(f"{math.degrees(y):+.1f}" for y in lidar_yaws) or "nenhum"
                if not amcl_enabled:
                    log(f"esquadro por AMCL desligado (amcl_square_enabled=false): o AMCL "
                        f"pede {math.degrees(amcl_errors()[2]):+.1f} graus; LiDAR yaw_rel "
                        f"[{lidar_txt}] graus — nao giro")
                elif self._refine_fit_yaws:
                    log("esquadro por AMCL pulado: o refino ACHOU a face e rejeitou o fit "
                        f"(LiDAR yaw_rel [{lidar_txt}] graus) — nao giro pelo AMCL contra "
                        "uma face medida; fica para o refit perto da mesa")
                else:
                    _w, still, _yok, _y0, _y1 = self._wait_still(
                        goal_handle, rate, deadline, attempt_end, "parando",
                        w_max=0.03, min_s=0.3)
                    samples = []
                    t_s = now()
                    while len(samples) < 5 and now() - t_s < 1.0:
                        self._check_guards(goal_handle, deadline, attempt_end)
                        self._publish_cmd(0.0, 0.0, 0.0)
                        samples.append(amcl_errors()[2])
                        rate.sleep()
                    eyaw0 = median_if_tight(samples, 5, 0.02)
                    f0, _n0, _fy0 = self._check_guards(goal_handle, deadline, attempt_end)
                    if self._live_face_yaw is not None:
                        lidar_yaws.append(self._live_face_yaw)
                    veto = amcl_square_veto(eyaw0, lidar_yaws, agree_tol, yaw_tol, amcl_max_err)
                    if not still:
                        log("esquadro por AMCL pulado: base nao parou pelo /odom")
                    elif eyaw0 is None:
                        log("esquadro por AMCL pulado: erro do AMCL ainda mudando "
                            f"({len(samples)} amostras, dispersao > 0,02 rad)")
                    elif veto:
                        log(f"esquadro por AMCL pulado: {veto}")
                    elif abs(eyaw0) <= yaw_min_corr:
                        log(f"esquadro por AMCL pulado: |erro| {math.degrees(abs(eyaw0)):.1f} graus "
                            f"abaixo do coast de giro ({math.degrees(yaw_min_corr):.1f}) — "
                            "o giro nao corrige")
                    elif f0 is None or f0 < rotate_min:
                        log("esquadro por AMCL pulado: front="
                            f"{'?' if f0 is None else '%.3f' % f0} < rotate_min_front "
                            f"{rotate_min:.2f} (bico raspa a mesa)")
                    else:
                        inst("antes do esquadro por AMCL")
                        log(f"esquadro por AMCL: {math.degrees(eyaw0):+.1f} graus (refino sem face)")
                        self._cut_drive(goal_handle, rate, deadline, attempt_end,
                                        "z", 1.0 if eyaw0 > 0 else -1.0,
                                        lambda _fr: amcl_errors()[2],
                                        yaw_coast, abs(eyaw0) / 0.315 + 2.0, "esquadrando")
                        inst("depois do esquadro por AMCL")
                        target, confident = self._refine_target_from_scan(target, rate, goal_handle)
                        if confident:
                            tx, ty, tyaw = target
                            log("refino apos esquadro: ok")
                        else:
                            log("refino apos esquadro: ainda sem confianca — sigo pelo banco")
                        inst("apos refino pos-esquadro")
        except self._AttemptAbort as ab:
            self._publish_cmd(0.0, 0.0, 0.0)
            return ab.outcome, ab.info

        def lateral_adjust(where, foot=False):
            """UM ajuste lateral pelo centro da face, se confiavel e seguro.
            foot=True (2026-10-26, I4): centra pelo PE' DA PERPENDICULAR
            (o que a reta acerta DEPOIS de um esquadro); False, pelo ponto
            onde o eixo fura a face. Loga os dois."""
            front, npts, face_yaw = self._check_guards(
                goal_handle, deadline, attempt_end)
            if not center_enabled or front is None:
                return front, npts, face_yaw
            if front < lateral_min - 1e-6:
                log(f"lateral({where}): front={front:.3f} < {lateral_min:.2f} — pulando")
                return front, npts, face_yaw
            c = self._measure_center(goal_handle, rate, deadline, attempt_end, foot=foot)
            ca, cf = self._center_medians
            log(f"[inst] lateral({where}): c_axis="
                f"{'?' if ca is None else '%+.1f' % (ca * 100)} c_foot="
                f"{'?' if cf is None else '%+.1f' % (cf * 100)} cm "
                f"(uso {'c_foot' if foot else 'c_axis'}; yaw_face="
                f"{'?' if face_yaw is None else '%+.1f' % math.degrees(face_yaw)} graus)")
            if c is None:
                log(f"lateral({where}): centro sem confianca ({self._center_reject}) — nao mexo")
            elif abs(c) <= max(center_deadband, center_coast):
                log(f"lateral({where}): centrado ({c*100:+.1f} cm)")
            elif abs(c) > center_max_travel + center_coast:
                log(f"lateral({where}): desvio {c*100:+.1f} cm maior que o curso — nao mexo")
            elif not self._side_clear(1.0 if c > 0 else -1.0, front):
                log(f"lateral({where}): obstaculo na faixa lateral — nao mexo")
            else:
                c_hist = []

                def center_err(front_now):
                    res = self._face_center_from_scan(front_now, foot=foot)
                    if res is None:
                        return None
                    c_hist.append(res[0])
                    del c_hist[:-3]
                    return sorted(c_hist)[len(c_hist) // 2]

                before_c = c
                # R6 (F15/C3, 2026-10-26): corte pelo ponto de parada
                # PREVISTO (lag_v = velocidade comandada): o corte cru em
                # err <= coast deixava 5-10 cm de sobrecurso aditivo.
                front, npts, face_yaw = self._cut_drive(
                    goal_handle, rate, deadline, attempt_end,
                    "y", 1.0 if c > 0 else -1.0, center_err, center_coast,
                    (abs(c) + 0.05) / min_v + 1.0, "centrando",
                    lag_v=self._start_speeds()[0])
                res = self._measure_center(goal_handle, rate, deadline, attempt_end, foot=foot)
                log(f"lateral({where}): {before_c*100:+.1f} -> "
                    f"{'?' if res is None else '%+.1f' % (res*100)} cm")
            return front, npts, face_yaw

        def square_by_lidar(where, yaw_rel, front):
            """Giro pelo yaw da FACE no LiDAR (nao pelo AMCL): _cut_drive em z
            com o erro vivo = face_yaw do fit leve do muro (_check_guards),
            corte no coast do giro. Devolve o _settle (front, npts, yaw)."""
            t0 = now()
            log(f"esquadro por LiDAR ({where}): yaw_rel {math.degrees(yaw_rel):+.1f} graus "
                f"a front={front:.3f}")
            if self.get_parameter("rotate_mode").value == "pulsos":
                _d, _r, (front, npts, fy) = self._rotate_pulses(
                    goal_handle, rate, deadline, attempt_end, yaw_rel, f"{tag} LiDAR({where})")
            else:
                front, npts, fy = self._cut_drive(
                    goal_handle, rate, deadline, attempt_end,
                    "z", 1.0 if yaw_rel > 0 else -1.0,
                    lambda _fr: self._live_face_yaw,
                    yaw_coast, abs(yaw_rel) / 0.315 + 2.0, "esquadrando")
            log(f"esquadro por LiDAR ({where}): yaw_face apos o giro "
                f"{'?' if fy is None else '%+.1f' % math.degrees(fy)} graus, "
                f"front={'?' if front is None else '%.3f' % front} ({now() - t0:.1f} s)")
            return front, npts, fy

        def refit_square(where):
            """Para (zero), assenta, tira 2 fits CONFIAVEIS e concordantes da
            face numa banda rasa ao redor do muro (face isolada) e, se
            |yaw_rel| > yaw_tol e front >= rotate_min_front, esquadra pelo
            LiDAR. Nunca gira perto demais (bico raspa a mesa). Devolve
            (front, npts, face_yaw, status) do muro; status e' o de
            refit_square_decision e vale 'girou' SO' se o giro aconteceu
            (2026-10-26, I8/F12: antes o chamador marcava a correcao como
            feita mesmo com os fits rejeitados)."""
            t0 = now()
            self._publish_cmd(0.0, 0.0, 0.0)
            front, npts, face_yaw = self._settle(
                goal_handle, rate, deadline, attempt_end, "refinando")
            if front is None:
                log(f"refit({where}): sem medicao do muro apos parar — sigo sem esquadrar")
                return front, npts, face_yaw, "sem_muro"
            first = self._refit_face_band(front, f"refit({where})")
            second = None
            if first.ok and self._wait_new_scan(second_read_s, rate, goal_handle):
                second = self._refit_face_band(front, f"refit({where}, 2a leitura)")
            yaw_rel, status, why = refit_square_decision(
                first, second, agree_tol, yaw_tol, yaw_min_rot, front, rotate_min)
            # I6: yaw da banda rasa (fit TLS) ao lado do fit leve do corredor
            # (_live_face_yaw, o que fecha a malha do giro e o 'pronto:').
            live = self._live_face_yaw
            log(f"[inst] refit({where}): yaw_rel banda="
                f"{'?' if yaw_rel is None else '%+.1f' % math.degrees(yaw_rel)} | fit leve="
                f"{'?' if live is None else '%+.1f' % math.degrees(live)} graus, "
                f"front={front:.3f}, status={status}")
            if yaw_rel is not None:
                self._last_face_width = second.width
            if status == "sem_fits":
                log(f"refit({where}): sem 2 fits confiaveis ({why}) — sigo sem "
                    f"esquadrar ({now() - t0:.1f} s)")
                return front, npts, face_yaw, status
            if status == "discordam":
                log(f"refit({where}): {why} — sigo sem esquadrar ({now() - t0:.1f} s)")
                return front, npts, face_yaw, status
            yaw_txt = f"yaw_rel {math.degrees(yaw_rel):+.1f} graus"
            if status == "dentro":
                log(f"refit({where}): {yaw_txt} {why} ({now() - t0:.1f} s)")
                return front, npts, face_yaw, status
            if status == "coast":
                log(f"refit({where}): {yaw_txt} {why} — nao giro")
                return front, npts, face_yaw, status
            if status == "perto":
                log(f"refit({where}): {yaw_txt} {why} — NAO giro (bico raspa a mesa)")
                return front, npts, face_yaw, status
            inst(f"antes do esquadro por LiDAR ({where})")
            front, npts, fy = square_by_lidar(where, yaw_rel, front)
            inst(f"depois do esquadro por LiDAR ({where})")
            return front, npts, fy, "girou"

        def approach(allow_refit):
            """(B) RETO ate a janela, guiado pelo muro. Com allow_refit, pausa
            UMA vez quando vai parar em ~refine_refit_front_m (gatilho
            descontando coast e lag) para o refit/esquadro pelo LiDAR.
            Devolve o _settle final (front, npts, face_yaw)."""
            waited_since = None
            refit_done = not allow_refit
            while rclpy.ok():
                front, npts, face_yaw = self._check_guards(
                    goal_handle, deadline, attempt_end)
                ex, ey, eyaw = amcl_errors()
                now_s = now()
                if front is None:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    waited_since = waited_since or now_s
                    self._feedback(
                        goal_handle,
                        "sem_scan" if npts < 0 else "face_invisivel",
                        ex, ey, eyaw)
                    limit = stale_abort_s if npts < 0 else blind_wait_s
                    if now_s - waited_since > limit:
                        raise self._AttemptAbort("soft_timeout", (
                            "STALE: /scan velho — nao avanco as cegas"
                            if npts < 0 else
                            "BLIND: face fora do plano do LiDAR"))
                    rate.sleep()
                    continue
                waited_since = None
                lag = min_v * ((self._wall_age or 0.0) + period)
                if refit_trigger(front, refit_front, refit_done, coast=coast, lag=lag):
                    refit_done = True
                    self._publish_cmd(0.0, 0.0, 0.0)
                    log(f"reto: pausa para refit a front={front:.3f} "
                        f"(alvo {refit_front:.2f} + coast {coast:.2f})")
                    # Camera manda (2026-10-27); sem medida dela, o LiDAR.
                    if not cam_used["v"] or camera_align("reto")[0] == "sem_camera":
                        refit_square("reto")
                    continue
                if front - lag <= stop + coast:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    log(f"reto: corte a front={front:.3f} (alvo {stop:.3f} + coast {coast:.2f})")
                    break
                vx = max(min_v, min(max_vx, kp * (front - stop)))
                self._feedback(goal_handle, "aproximando", ex, ey, eyaw)
                self._publish_cmd(vx, 0.0, 0.0)
                rate.sleep()

            front, npts, face_yaw = self._settle(
                goal_handle, rate, deadline, attempt_end, "assentando")
            log(f"reto: parou em front={front if front is None else round(front, 3)} "
                f"(janela [{window_lo:.3f}, {window_hi:.3f}])")
            inst("parou do reto")
            return front, npts, face_yaw

        # ---------- Camera no docking (2026-10-27) ----------
        cam_steps = max(0, int(self.get_parameter("camera_max_steps").value))
        # Em pulsos a base corrige abaixo do coast do giro continuo: o limiar
        # vira a tolerancia da camera (o _rotate_pulses recusa o pulso que
        # passaria do ponto).
        cam_yaw_thr = float(self.get_parameter("camera_yaw_tol").value) \
            if self.get_parameter("rotate_mode").value == "pulsos" \
            else max(float(self.get_parameter("camera_yaw_tol").value), yaw_min_corr)
        cam_c_thr = max(float(self.get_parameter("camera_center_tol").value), center_coast)
        cam_max_lat = float(self.get_parameter("camera_max_lateral").value)
        cam_w = float(self.get_parameter("camera_weight").value)
        cam_yaw_agree = float(self.get_parameter("camera_lidar_yaw_agree").value)
        cam_c_agree = float(self.get_parameter("camera_lidar_center_agree").value)
        # "v" = a camera ja' mediu a mesa neste goal: dali em diante cada
        # ponto de decisao (reto, mesa, yaw final) tenta a camera primeiro.
        cam_used = {"v": False}

        def fuse(cam_v, lidar_v, agree):
            """Camera pesa mais: concordando, media ponderada; discordando
            (ou sem LiDAR), so' a camera."""
            if lidar_v is None:
                return cam_v, "so' camera"
            if abs(cam_v - lidar_v) <= agree:
                return cam_w * cam_v + (1.0 - cam_w) * lidar_v, "camera+lidar"
            return cam_v, "lidar discorda: camera"

        def odom_drive(axis, demand, phase):
            """Curso LATERAL por ODOMETRIA (a camera nao fecha malha a 10 Hz):
            anda `demand` e remede depois. Giro: rotate_tracked."""
            o0 = self._odom_pose(0.5)
            if o0 is None:
                log(f"camera: sem /odom fresco — nao mexo ({phase})")
                return False

            def err(_front):
                o = self._odom_pose(0.5)
                if o is None:
                    return None
                if axis == "z":
                    return demand - _normalize_angle(o[2] - o0[2])
                return demand - _lateral_displacement(o0[0], o0[1], o0[2], o[0], o[1])
            sign = 1.0 if demand > 0 else -1.0
            if axis == "z":
                self._cut_drive(goal_handle, rate, deadline, attempt_end, "z", sign, err,
                                yaw_coast, abs(demand) / 0.315 + 2.0, phase)
            else:
                self._cut_drive(goal_handle, rate, deadline, attempt_end, "y", sign, err,
                                center_coast, (abs(demand) + 0.05) / min_v + 1.0, phase,
                                lag_v=self._start_speeds()[0])
            return True

        def rotate_tracked(demand, phase):
            """Giro de `demand` cortado pela variacao MEDIDA do yaw da face no
            LiDAR (fit leve, a cada scan): rotacao = yaw0 - yaw(t). Tour de
            27/10: a odometria subestimou o giro real em 30-70% e cada passo
            da camera girava ~2x o pedido (vaivem de +-11 graus na WS2/WS6);
            o esquadro do LiDAR, que fecha a malha assim, parava a +-1,5.
            Sem face no LiDAR, cai na odometria. Devolve a fonte ou None."""
            live0 = self._live_face_yaw
            o0 = self._odom_pose(0.5)
            if live0 is None and o0 is None:
                log(f"camera: sem face no LiDAR nem /odom fresco — nao giro ({phase})")
                return None

            def err(_front):
                ly = self._live_face_yaw
                if live0 is not None and ly is not None:
                    return demand - (live0 - ly)
                o = self._odom_pose(0.5)
                if o is None or o0 is None:
                    return None
                return demand - _normalize_angle(o[2] - o0[2])
            self._cut_drive(goal_handle, rate, deadline, attempt_end, "z",
                            1.0 if demand > 0 else -1.0, err, yaw_coast,
                            abs(demand) / 0.315 + 2.0, phase)
            return "lidar" if live0 is not None else "odometria"

        def camera_align(where):
            """Mede a mesa pela camera (fundida com o LiDAR) e corrige: primeiro
            o yaw (giro cortado pelo LiDAR, se front >= rotate_min_front; se
            passar do ponto, nao gira de novo), depois
            o centro (c_axis); remede apos cada movimento, ate camera_max_steps
            movimentos. Devolve (status, (front, npts, face_yaw)); status:
            'alinhado' (yaw e centro dentro), 'esquadrado' (yaw dentro, sem
            centro confiavel), 'parcial' (nao fechou), 'sem_camera' (nada
            medido nem mexido) ou 'off'."""
            if not self._cam_goal_enabled:
                return "off", self._check_guards(goal_handle, deadline, attempt_end)
            moved = False
            rot_sign_prev = None
            rot_blocked = False
            lat_blocked = False
            for step in range(cam_steps + 1):
                front, npts, lidar_yaw = self._settle(
                    goal_handle, rate, deadline, attempt_end, "camera")
                self._feedback(goal_handle, "camera", 0.0, 0.0, 0.0)
                m = self._measure_camera(goal_handle, rate, deadline, attempt_end,
                                         f"{where}/{step}")
                if m is None:
                    log(f"camera({where}): sem medida ({self._cam_reject})"
                        + ("" if moved else " — sigo pelo LiDAR"))
                    return ("parcial" if moved else "sem_camera"), (front, npts, lidar_yaw)
                cam_used["v"] = True
                yaw, ysrc = fuse(m["yaw"], lidar_yaw, cam_yaw_agree)
                c = csrc = None
                lidar_c = None
                if m["c_axis"] is not None:
                    if front is not None and front >= lateral_min:
                        res = self._face_center_from_scan(front, foot=False)
                        lidar_c = None if res is None else res[0]
                    c, csrc = fuse(m["c_axis"], lidar_c, cam_c_agree)
                log(f"camera({where}/{step}): yaw cam {math.degrees(m['yaw']):+.1f} lidar "
                    f"{'?' if lidar_yaw is None else '%+.1f' % math.degrees(lidar_yaw)} -> "
                    f"{math.degrees(yaw):+.1f} graus ({ysrc}); centro cam "
                    f"{'?' if m['c_axis'] is None else '%+.1f' % (m['c_axis'] * 100)} lidar "
                    f"{'?' if lidar_c is None else '%+.1f' % (lidar_c * 100)} -> "
                    f"{'?' if c is None else '%+.1f' % (c * 100)} cm"
                    f"{'' if c is None else ' (' + csrc + ')'}; front lidar "
                    f"{'?' if front is None else '%.3f' % front} cam {m['front']:.3f}")
                if (rot_sign_prev is not None and not rot_blocked
                        and abs(yaw) > cam_yaw_thr and (yaw > 0) != (rot_sign_prev > 0)):
                    # Passou do ponto: girar de volta so' repete o vaivem
                    # (o giro minimo da base e' maior que o erro).
                    rot_blocked = True
                    log(f"camera({where}): o giro passou do ponto (agora "
                        f"{math.degrees(yaw):+.1f} graus do outro lado) — nao giro de novo")

                def done_status():
                    # "Bom o bastante" (operador 27/10: o manipulador tolera um
                    # pouco de angulo/centro): o que nao da' mais para melhorar
                    # (giro menor que o pulso minimo, lateral abaixo do passo)
                    # conta como feito.
                    yaw_ok = abs(yaw) <= cam_yaw_thr or rot_blocked
                    c_ok = c is not None and (abs(c) <= cam_c_thr or lat_blocked)
                    st = ("alinhado" if c_ok else "esquadrado") if yaw_ok else "parcial"
                    log(f"camera({where}): {st} (yaw {math.degrees(yaw):+.1f} graus, limiar "
                        f"{math.degrees(cam_yaw_thr):.1f}{', sem giro possivel' if rot_blocked else ''}"
                        f"; centro {'?' if c is None else '%+.1f' % (c * 100)} cm, limiar "
                        f"{cam_c_thr * 100:.1f}{', sem passo possivel' if lat_blocked else ''})")
                    return st, (front, npts, lidar_yaw)

                need_rot = abs(yaw) > cam_yaw_thr and not rot_blocked
                need_lat = c is not None and abs(c) > cam_c_thr and not lat_blocked
                if not need_rot and not need_lat:
                    return done_status()
                if step == cam_steps:
                    log(f"camera({where}): {cam_steps} movimento(s) e ainda fora — sigo")
                    break
                if need_rot and (front is None or front < rotate_min):
                    log(f"camera({where}): yaw {math.degrees(yaw):+.1f} graus fora mas front="
                        f"{'?' if front is None else '%.3f' % front} < rotate_min_front "
                        f"{rotate_min:.2f} — nao giro aqui")
                    rot_blocked = True
                elif need_rot:
                    inst(f"antes do giro pela camera ({where})")
                    if self.get_parameter("rotate_mode").value == "pulsos":
                        log(f"camera({where}): giro {math.degrees(yaw):+.1f} graus em pulsos")
                        done, _rest, _res = self._rotate_pulses(
                            goal_handle, rate, deadline, attempt_end, yaw,
                            f"{tag} camera({where})")
                        if abs(done) >= 1e-3:
                            moved = True
                            continue   # remede depois de girar
                        # Nenhum pulso cabe: decide o centro com ESTA medida
                        # (antes media de novo so' para descobrir o mesmo).
                        rot_blocked = True
                    else:
                        log(f"camera({where}): giro {math.degrees(yaw):+.1f} graus (corte pelo LiDAR)")
                        src = rotate_tracked(yaw, "esquadrando")
                        if src is None:
                            break
                        if src != "lidar":
                            log(f"camera({where}): giro cortado pela {src} (sem face no LiDAR)")
                        rot_sign_prev = yaw
                        moved = True
                        continue
                need_lat = c is not None and abs(c) > cam_c_thr and not lat_blocked
                if not need_lat:
                    return done_status()
                if abs(c) > cam_max_lat:
                    log(f"camera({where}): centro {c * 100:+.1f} cm > camera_max_lateral "
                        f"{cam_max_lat * 100:.0f} — medida suspeita, nao mexo")
                    break
                if front is None or front < lateral_min:
                    log(f"camera({where}): front="
                        f"{'?' if front is None else '%.3f' % front} < lateral_min_front — "
                        "nao mexo de lado")
                    break
                if not self._side_clear(1.0 if c > 0 else -1.0, front):
                    log(f"camera({where}): obstaculo na faixa lateral — nao mexo")
                    break
                log(f"camera({where}): lateral {c * 100:+.1f} cm por odometria")
                o0 = self._odom_pose(0.5)
                if not odom_drive("y", c, "centrando"):
                    break
                o1 = self._odom_pose(0.5)
                if o0 is not None and o1 is not None and abs(
                        _lateral_displacement(o0[0], o0[1], o0[2], o1[0], o1[1])) < 0.01:
                    # Tour 27/10 (WS3/WS5): pedidos de 3-4 cm andaram 0,0 cm
                    # tres vezes seguidas (abaixo do passo minimo da base).
                    lat_blocked = True
                    log(f"camera({where}): o lateral nao andou (abaixo do passo minimo) — "
                        "nao repito")
                    return done_status()
                moved = True
            return "parcial", self._check_guards(goal_handle, deadline, attempt_end)

        cam_min_front = float(self.get_parameter("camera_min_front").value)

        def cam_close(where):
            front_now = self._check_guards(goal_handle, deadline, attempt_end)[0]
            if front_now is not None and front_now < cam_min_front:
                log(f"camera({where}): front={front_now:.3f} < camera_min_front "
                    f"{cam_min_front:.2f} — perto demais, so' LiDAR")
                return True
            return False

        def lateral_mesa():
            """(C) com a camera, se ela ja' mediu neste goal e esta' longe o
            bastante da mesa; senao LiDAR."""
            if cam_used["v"] and not cam_close("mesa"):
                st, res = camera_align("mesa")
                if st != "sem_camera":
                    return res
            return lateral_adjust("mesa")

        def final_yaw():
            """(D) yaw final: LiDAR (mediana do fit leve) fundido com a camera."""
            yaw_l, why = self._measure_face_yaw(goal_handle, rate, deadline, attempt_end)
            if not cam_used["v"] or cam_close("final"):
                return yaw_l, why
            m = self._measure_camera(goal_handle, rate, deadline, attempt_end, "final")
            if m is None:
                log(f"yaw final: camera sem medida ({self._cam_reject}) — so' LiDAR")
                return yaw_l, why
            yaw, src = fuse(m["yaw"], yaw_l, cam_yaw_agree)
            log(f"yaw final: cam {math.degrees(m['yaw']):+.1f} lidar "
                f"{'?' if yaw_l is None else '%+.1f' % math.degrees(yaw_l)} -> "
                f"{math.degrees(yaw):+.1f} graus ({src})")
            return yaw, ""

        correct_enabled = bool(self.get_parameter("final_yaw_correction").value)
        strict = bool(self.get_parameter("final_yaw_strict").value)

        try:
            # ---------- (A0) esquadro pelo LiDAR no STAGING (2026-10-26, I4) -
            # C3/F15: o lateral media o centro com o robo torto e a demanda
            # carregava front*tan(yaw) (WS3 +16/+17,5 cm nos dois tours,
            # SH1-T2 -18,7); o giro vinha DEPOIS e nao desfazia a translacao.
            # Agora esquadra antes (so' com refino confiante e front >=
            # rotate_min; refit_square ja' exige 2 fits concordantes e
            # |yaw_rel| > yaw_min_corr — sem giro, nada muda). O lateral do
            # staging centra SEMPRE pelo ponto onde o eixo fura a face
            # (c_axis): apos 'girou' c_axis ~ c_foot; nos demais status
            # (dentro/coast/perto/sem_fits) nao vai haver giro e o robo anda
            # pelo proprio eixo, entao c_axis e' o ponto que a reta acerta.
            # Usar c_foot "apostando" que o refit do reto gira seria pior
            # justamente nas docks com refino rejeitado, onde o refit da
            # banda rasa tambem reprova (F11: WS2 residuo, WS6 face estreita)
            # — ali c_foot carregaria d*tan(yaw) (WS2 +6..+8 cm, WS6 ~11 cm).
            # c_foot fica so' no log [inst] ate um tour mostrar o contrario.
            # ---------- (Acam) CAMERA no staging (2026-10-27) ----------------
            # Ve o tampo inteiro daqui: esquadra e centra por ela. So' cai no
            # (A0)+(A) do LiDAR o que a camera nao resolveu.
            cam_st, _res = camera_align("staging")
            if cam_st == "alinhado":
                log("staging pela camera: esquadro e lateral do LiDAR pulados")
            else:
                if cam_st == "esquadrado":
                    log("staging: yaw pela camera, centro pelo LiDAR")
                else:
                    front0, _n0, _fy0 = self._check_guards(goal_handle, deadline, attempt_end)
                    square_possible = (g.use_lidar_refine and front0 is not None
                                       and front0 >= rotate_min)
                    if square_possible and confident:
                        refit_square("staging")
                    elif g.use_lidar_refine:
                        log("esquadro no staging pulado: " + (
                            "refino sem confianca" if not confident else
                            f"front={'?' if front0 is None else '%.3f' % front0} "
                            f"< rotate_min_front {rotate_min:.2f}"))

                # ---------- (A) ajuste lateral no STAGING (longe = mais seguro)
                lateral_adjust("staging", foot=False)

            # ---------- (B) RETO ate a janela (com pausa de refit) -----------
            # Staging fechado pela camera: sem a pausa no meio da reta (a reta
            # e' curta; a pausa custava uma medida a mais por docking).
            front, npts, face_yaw = approach(
                allow_refit=cam_st not in ("alinhado", "esquadrado"))

            # ---------- (C) retoque lateral NA MESA, se ainda estiver torto --
            # (lateral puro nao avanca o bico; confirmado em campo 21/08)
            front, npts, face_yaw = lateral_mesa()

            # ---------- (D) validacao final de YAW pela face (2026-10-21) ----
            # Antes o sucesso so' validava a janela frontal e a WS1 saiu
            # "pronto" com yaw_face -19,7 graus. Agora: mede (mediana de
            # fits confiaveis); fora da tolerancia, recua ate a distancia
            # de giro, esquadra pela face e refaz reto+lateral UMA vez.
            corrected_once = False
            correction_cycles = 0
            while True:
                if front is None:
                    return "soft_timeout", "sem medicao do muro apos a chegada"
                if front > window_hi + 0.03:
                    return "soft_timeout", (
                        f"parou longe demais (front {front:.3f} > {window_hi + 0.03:.3f})")
                ex, ey, eyaw = amcl_errors()
                # 2026-10-26: o Y pelo AMCL vai para o log (na WS3 de 23/10 o
                # result levou +30,4 cm e nenhuma linha daqui mostrava isso).
                x_txt = (f"erro X {((front - stop) * 100):+.1f} cm, "
                         f"Y(AMCL) {ey * 100:+.1f} cm")
                yaw_m, why = final_yaw()
                if yaw_m is None:
                    self.get_logger().warn(
                        f"{tag} yaw_nao_medido: {why}; AMCL diz {math.degrees(eyaw):+.1f} graus "
                        f"e Y {ey * 100:+.1f} cm (so' para o log — nao giro pelo AMCL; "
                        "o GoToWS valida contra os limites duros)")
                    log(f"pronto: front={front:.3f} yaw_face=? graus ({x_txt}) — yaw_nao_medido")
                    return "success", (front - stop, ey, eyaw, "yaw_nao_medido")
                now_s = now()
                has_time = min(deadline, attempt_end) - now_s >= _YAW_CORRECTION_EST_S
                can_correct = (correct_enabled and not corrected_once
                               and correction_cycles < _YAW_CORRECTION_MAX_CYCLES
                               and has_time)
                decision = final_yaw_decision(
                    yaw_m, yaw_tol, not can_correct, strict, min_correctable=yaw_min_rot)
                yaw_deg = math.degrees(yaw_m)
                if decision == "ok":
                    log(f"pronto: front={front:.3f} yaw_face={yaw_deg:+.1f} graus ({x_txt})")
                    return "success", (front - stop, ey, yaw_m, "")
                if decision == "corrigir":
                    t0 = now_s
                    target_front = rotate_min + 0.05
                    if cam_used["v"]:
                        # a borda da frente volta ao campo de visao da camera
                        target_front = max(target_front, float(
                            self.get_parameter("camera_view_front").value))
                    self.get_logger().warn(
                        f"{tag} yaw_face {yaw_deg:+.1f} graus > tolerancia "
                        f"{math.degrees(yaw_tol):.1f}: recuo ate front {target_front:.2f}, "
                        "esquadro pela face do LiDAR e refaco reto+lateral (uma vez; "
                        f"restam {deadline - now_s:.0f} s)")
                    self._feedback(goal_handle, "recuando", ex, ey, eyaw)
                    if self._retreat(goal_handle, staging_offset, rate,
                                     min(deadline, attempt_end), period,
                                     target_front=target_front) == "canceled":
                        raise self._AttemptAbort("canceled")
                    correction_cycles += 1
                    st = None
                    if cam_used["v"]:
                        cam_c, _res = camera_align("correcao")
                        if cam_c in ("alinhado", "esquadrado"):
                            # a camera esquadrou (ou confirmou o esquadro)
                            st = "girou"
                    if st is None:
                        _f, _n, _fy, st = refit_square("correcao")
                    front, npts, face_yaw = approach(allow_refit=False)
                    front, npts, face_yaw = lateral_mesa()
                    # I8 (F12, 2026-10-26): so' conta como corrigido se o
                    # esquadro ACONTECEU; 6/6 correcoes dos tours recuaram e
                    # voltaram sem girar (refit rejeitado) e a flag
                    # legitimava o mesmo yaw.
                    if correction_counted(st):
                        corrected_once = True
                    else:
                        self.get_logger().warn(
                            f"{tag} correcao sem esquadro (refit {st}: fits rejeitados/"
                            f"sem giro) — ciclo {correction_cycles}/"
                            f"{_YAW_CORRECTION_MAX_CYCLES}; nao conto como corrigido")
                    log(f"correcao final de yaw custou {now() - t0:.1f} s "
                        f"(restam {deadline - now():.0f} s do align_timeout)")
                    continue
                note = f"yaw_fora_da_tolerancia ({yaw_deg:+.1f} graus)"
                if corrected_once:
                    cause = "ja' corrigido uma vez neste goal"
                elif correction_cycles >= _YAW_CORRECTION_MAX_CYCLES:
                    cause = (f"{correction_cycles} ciclo(s) de correcao SEM esquadro "
                             "(refit rejeitado) — o yaw nao foi corrigido")
                elif not correct_enabled:
                    cause = "final_yaw_correction=false"
                elif not has_time:
                    cause = f"sem tempo para corrigir (restam {deadline - now_s:.0f} s)"
                else:
                    cause = (f"abaixo do coast de giro ({math.degrees(yaw_min_corr):.1f} graus): "
                             "o giro nao corrige")
                if decision == "falhar":
                    self.get_logger().error(
                        f"{tag} {note}: tolerancia {math.degrees(yaw_tol):.1f} graus, "
                        f"{cause}; final_yaw_strict=true — abortando")
                    return "yaw_out", note
                self.get_logger().warn(
                    f"{tag} ========== {note.upper()} ==========\n"
                    f"{tag} tolerancia {math.degrees(yaw_tol):.1f} graus, {cause}; "
                    "SIGO (final_yaw_strict=false) — o pick/place vai trabalhar com a "
                    "mesa torta; conferir a pose do dock no mapa.")
                log(f"pronto: front={front:.3f} yaw_face={yaw_deg:+.1f} graus ({x_txt}) — {note}")
                return "success", (front - stop, ey, yaw_m, note)
        except self._AttemptAbort as ab:
            self._publish_cmd(0.0, 0.0, 0.0)
            return ab.outcome, ab.info

        self._publish_cmd(0.0, 0.0, 0.0)
        return "shutdown", None

    def _execute(self, goal_handle):
        """Entrada do align_to_dock: toma o lock de movimento (ocupado se um
        nudge esta em curso) e ZERA o total lateral acumulado — um novo
        docking e uma nova referencia para a memoria de slots do place
        (2026-08-28). O trabalho de verdade esta em _execute_align."""
        if not self._motion_lock.acquire(blocking=False):
            return self._abort(goal_handle, AlignToDock.Result(),
                               "ocupado: outro movimento (nudge_base) em curso")
        try:
            self._set_shift_total(0.0)
            return self._execute_align(goal_handle)
        finally:
            self._publish_cmd(0.0, 0.0, 0.0)
            self._cam_stop()
            self._motion_lock.release()

    def _shared_rate(self, period):
        """Rate persistente (auditoria 31/08): nao criar/vazar um por goal."""
        hz = 1.0 / float(period)
        if self._rate_cache is None or abs(self._rate_cache[0] - hz) > 1e-6:
            if self._rate_cache is not None:
                try:
                    self.destroy_rate(self._rate_cache[1])
                except Exception:  # noqa: BLE001
                    pass
            self._rate_cache = (hz, self.create_rate(hz))
        return self._rate_cache[1]

    def _execute_align(self, goal_handle):
        """Orquestrador: N tentativas de aproximacao; entre elas, RECUO ate a
        folga de pre-docking. v5: uma tentativa por padrao (approach_retries 0
        e' aceitavel); apos falhar TUDO, recua ate o staging antes de abortar
        para nao deixar o robo colado na mesa."""
        g = goal_handle.request
        result = AlignToDock.Result()

        xy_tol = g.xy_tolerance or self.get_parameter("default_xy_tolerance").value
        yaw_tol = g.yaw_tolerance or self.get_parameter("default_yaw_tolerance").value
        timeout = g.timeout or self.get_parameter("default_timeout").value
        period = 1.0 / float(self.get_parameter("control_frequency").value)
        rate = self._shared_rate(period)

        try:
            bx, by, byaw, face_dist, staging_offset, half_w = self._load_dock_pose(
                g.dock_id, g.map_name, g.map_dir)
        except Exception as exc:  # noqa: BLE001
            return self._abort(goal_handle, result, f"falha ao carregar dock: {exc}")
        bank_target = (bx, by, byaw)
        self._dock_face_dist = face_dist
        self._cam_setup_goal(g.dock_id, g.map_name, g.map_dir)
        # Janela lateral do refino por dock (R3, 2026-10-26); None = global.
        self._dock_refine_half_width = half_w
        if half_w is not None:
            self.get_logger().info(
                f"[{g.dock_id}] refine_half_width_y por dock: +-{half_w:.2f} m "
                f"(global {self.get_parameter('refine_half_width_y').value:.2f})")
        self._last_face_yaw_map = None  # portao de consistencia zera por goal
        self._last_face_width = None    # largura de referencia e' por mesa
        self._live_face_yaw = None
        self._refine_wide_face = False
        # So recuar se a face foi VISTA neste goal: re cega a partir de uma
        # pose desconhecida (campo 2026-08-21: 2x 0,35 m de re sem sensor
        # traseiro) e' pior que ficar parado.
        self._saw_face_this_goal = False

        attempts = max(1, int(self.get_parameter("approach_retries").value) + 1)
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        attempt_budget = max(40.0, timeout / attempts)
        grace = float(self.get_parameter("final_retreat_grace_s").value)

        last_reason = "sem tentativa"
        for attempt in range(1, attempts + 1):
            outcome, info = self._run_attempt(
                goal_handle, bank_target, g, xy_tol, yaw_tol,
                deadline, attempt_budget, rate, attempt, attempts,
                staging_offset)

            if outcome == "success":
                # note (2026-10-21): "" ou 'yaw_fora_da_tolerancia (x graus)' /
                # 'yaw_nao_medido' — vai na frente da message.
                ex, ey, eyaw, note = info
                result.success = True
                result.message = f"alinhado (tentativa {attempt}/{attempts})"
                if note:
                    result.message = f"{note}; {result.message}"
                result.final_x_error = ex
                result.final_y_error = ey
                result.final_yaw_error = eyaw
                goal_handle.succeed()
                return result
            if outcome == "canceled":
                self._publish_cmd(0.0, 0.0, 0.0)
                goal_handle.canceled()
                result.success = False
                result.message = "cancelado"
                return result
            if outcome == "shutdown":
                self._publish_cmd(0.0, 0.0, 0.0)
                return self._abort(goal_handle, result, "no encerrado")

            last_reason = info
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if outcome in ("hard_stop", "soft_timeout") and attempt < attempts \
                    and now_s < deadline and self._saw_face_this_goal:
                self.get_logger().warn(
                    f"[{g.dock_id}] tentativa {attempt}/{attempts} falhou "
                    f"({info}); recuando para tentar de novo.")
                self._feedback(goal_handle, "recuando", 0.0, 0.0, 0.0)
                if self._retreat(goal_handle, staging_offset, rate,
                                 deadline, period) == "canceled":
                    goal_handle.canceled()
                    result.success = False
                    result.message = "cancelado durante o recuo"
                    return result
                continue
            break

        # Falhou tudo: RECUA ate o staging ANTES de abortar (nao deixar o robo
        # colado na mesa — o proximo Nav2 giraria no lugar contra ela). Mas SO
        # se a face chegou a ser vista: sem referencia, ficar parado.
        if not self._saw_face_this_goal:
            self._publish_cmd(0.0, 0.0, 0.0)
            return self._abort(
                goal_handle, result,
                f"alinhamento falhou sem nunca ver a face ({last_reason}) — "
                "sem recuo (o robo nao esta de frente para a mesa?)")
        # Pedido do operador (2026-08-21): SEM re, a menos que tenha parado
        # perigosamente perto (a proxima navegacao giraria contra a mesa).
        front_now, _n, _a, _f = self._front_wall_distance()
        if front_now is not None and front_now >= 0.44:
            self._publish_cmd(0.0, 0.0, 0.0)
            return self._abort(
                goal_handle, result,
                f"alinhamento falhou ({last_reason}) — parado a front="
                f"{front_now:.3f} (sem recuo)")
        self.get_logger().warn(
            f"[{g.dock_id}] alinhamento falhou ({last_reason}); recuando ate o "
            "staging antes de reportar.")
        self._feedback(goal_handle, "recuando", 0.0, 0.0, 0.0)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self._retreat(goal_handle, staging_offset, rate,
                         max(deadline, now_s) + grace, period) == "canceled":
            goal_handle.canceled()
            result.success = False
            result.message = "cancelado durante o recuo final"
            return result

        self._publish_cmd(0.0, 0.0, 0.0)
        return self._abort(
            goal_handle, result,
            f"alinhamento falhou apos {attempt} tentativa(s): {last_reason}")

    def _retreat(self, goal_handle, staging_offset, rate, deadline, period,
                 target_front=None):
        """Re' pura ate a folga de pre-docking (vx<0 afasta da mesa — sempre
        seguro). Alvo: front_dist >= stop + staging_offset — ou target_front
        (2026-10-21: a correcao final de yaw recua so' ate rotate_min_front
        + 0,05, nao ate o staging); fallback por integracao do comando
        quando a face nao e' visivel. Melhor esforco: timeout local nao
        aborta o goal (a proxima tentativa parte de onde estiver)."""
        stop_dist = self._resolve_stop_dist()
        base = stop_dist if stop_dist is not None else 0.35
        travel_est = staging_offset
        if target_front is None:
            target_front = base + staging_offset
        else:
            f0, _n, _a, _f = self._front_wall_distance()
            if f0 is not None:
                travel_est = max(0.05, target_front - f0)
        # Parte do repouso acima do piso (2026-10-19): o 0.13 fixo de antes
        # ja' fazia isso na pratica (2.6 rad/s na roda); fica como minimo.
        speed = max(self._start_speeds()[0], 0.13)
        max_time = (travel_est / speed) * 2.0 + 2.0
        start_s = self.get_clock().now().nanoseconds * 1e-9
        traveled = 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_cmd(0.0, 0.0, 0.0)
                return "canceled"
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - start_s > max_time or now_s > deadline:
                break
            front, _npts, _age, _fyaw = self._front_wall_distance()
            if front is not None and front >= target_front:
                break
            if front is None and traveled >= travel_est:
                break
            self._publish_cmd(-speed, 0.0, 0.0)
            traveled += speed * period
            rate.sleep()

        self._publish_cmd(0.0, 0.0, 0.0)
        self.get_logger().info(
            f"recuo concluido (~{traveled:.2f}m comandados, alvo front {target_front:.2f}).")
        return "ok"

    def _feedback(self, goal_handle, phase, ex, ey, eyaw):
        # As primitivas (_settle/_cut_drive/_check_guards) servem as duas
        # actions: despacha pelo tipo do goal (2026-08-28).
        if isinstance(goal_handle.request, NudgeBase.Goal):
            fb = NudgeBase.Feedback()
            fb.travelled = float(self._nudge_travelled_now())
            goal_handle.publish_feedback(fb)
            return
        fb = AlignToDock.Feedback()
        fb.phase = phase
        fb.x_error = ex
        fb.y_error = ey
        fb.yaw_error = eyaw
        goal_handle.publish_feedback(fb)

    # ------------------------------------------------ nudge_base (2026-08-28)
    def _nudge_travelled_now(self):
        """Curso lateral (ja com o ganho de escorregamento) desde o inicio
        do nudge em curso pela ULTIMA odometria recebida (mesmo velha — e
        so feedback/log)."""
        origin = self._nudge_origin
        odom = self._odom
        if origin is None or odom is None:
            return 0.0
        return _nudge_corrected(
            _lateral_displacement(origin[0], origin[1], origin[2],
                                  odom[0], odom[1]),
            self._nudge_gain)

    def _nudge_progress(self, origin):
        """(lateral BRUTO da odometria, longitudinal, dyaw) desde `origin`
        pela ultima odometria recebida (para o resultado e o log). O ganho
        de escorregamento e aplicado por quem chama."""
        odom = self._odom
        if odom is None:
            return 0.0, 0.0, 0.0
        x0, y0, yaw0 = origin
        return (_lateral_displacement(x0, y0, yaw0, odom[0], odom[1]),
                _longitudinal_displacement(x0, y0, yaw0, odom[0], odom[1]),
                _normalize_angle(odom[2] - yaw0))

    def _execute_nudge(self, goal_handle):
        """Servidor da action nudge_base: deslocamento LATERAL puro (+ =
        esquerda) em malha fechada por odometria, com as guardas do docking.
        O total acumulado vai para /manip/base_shift_total mesmo em curso
        parcial/abortado — a base se moveu de fato e o place precisa saber."""
        g = goal_handle.request
        result = NudgeBase.Result()
        result.success = False
        result.travelled = 0.0

        def fail(reason, travelled=0.0, canceled=False):
            result.success = False
            result.reason = reason
            result.travelled = float(travelled)
            self.get_logger().warn(
                f"nudge_base: {reason} (dy={g.dy:+.3f} m, andou {travelled:+.3f} m)")
            if canceled:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        if not self._motion_lock.acquire(blocking=False):
            return fail("ocupado")
        try:
            period = 1.0 / float(self.get_parameter("control_frequency").value)
            rate = self._shared_rate(period)
            return self._run_nudge(goal_handle, g, result, fail, rate)
        finally:
            # Zero final SO se a fase de movimento comecou (goal rejeitado
            # pelas guardas nao toca a referencia compartilhada).
            if self._nudge_origin is not None:
                self._publish_cmd(0.0, 0.0, 0.0)
            self._nudge_origin = None
            self._motion_lock.release()

    def _run_nudge(self, goal_handle, g, result, fail, rate):
        dy = float(g.dy)
        mag = abs(dy)
        sign = 1.0 if dy > 0.0 else -1.0
        min_travel = float(self.get_parameter("nudge_min_travel").value)
        max_travel = float(self.get_parameter("nudge_max_travel").value)
        slack = float(self.get_parameter("nudge_accept_slack").value)
        odom_max_age = float(self.get_parameter("nudge_odom_max_age").value)
        require_scan = bool(self.get_parameter("nudge_require_scan").value)
        lateral_min = float(self.get_parameter("lateral_min_front").value)
        hard_min = float(self.get_parameter("hard_min_face_dist").value)
        wall_enabled = bool(self.get_parameter("wall_enabled").value)
        coast = float(self.get_parameter("center_coast_est").value)
        min_v = float(self.get_parameter("min_body_vel").value)
        period = 1.0 / float(self.get_parameter("control_frequency").value)
        gain = float(self.get_parameter("nudge_odom_lateral_gain").value)
        drift_max = float(self.get_parameter("nudge_front_drift_max").value)
        timeout = float(g.timeout) if g.timeout > 0.0 \
            else float(self.get_parameter("nudge_default_timeout").value)
        lead = float(self.get_parameter("nudge_speech_lead_s").value)

        def log(msg):
            self.get_logger().info(f"[nudge {dy:+.2f}] {msg}")

        # ---- (0) pedido bem formado (achado 4): NaN/inf passariam nas
        # comparacoes de faixa abaixo e o curso so acabaria por timeout.
        if not _nudge_request_valid(dy, float(g.timeout)):
            return fail("pedido_invalido")

        # ---- (1) validacao do pedido: nada abaixo do passo executavel nem
        # acima do curso maximo por chamada (a arvore quantiza antes).
        if mag < min_travel - 1e-6:
            return fail("abaixo_do_passo_minimo")
        if mag > max_travel + 1e-6:
            return fail("acima_do_curso_maximo")

        # ---- (2) odometria fresca: sem ela nao ha malha — nao anda.
        origin = self._odom_pose(odom_max_age)
        if origin is None:
            return fail("sem_odometria")

        # ---- (3) muro e faixa lateral, como no lateral_adjust do docking.
        front, npts, scan_age, face_yaw = self._front_wall_distance()
        if front is None:
            if require_scan:
                log("sem medicao do muro: "
                    + ("/scan ausente, velho ou sem TF" if npts < 0
                       else "face fora do plano do LiDAR (BLIND)")
                    + f" (idade {scan_age:.2f} s) — nao ando as cegas")
                return fail("sem_scan")
            self.get_logger().warn(
                "nudge_base: SEM muro (nudge_require_scan=false) — "
                "o lateral vai so pela odometria.")
        else:
            if wall_enabled and front < hard_min:
                return fail("muro")
            # Achado 1: a face pode nao estar perpendicular ao robo; num
            # lateral de |dy| ao longo dela o bico avanca |dy|*sin(face_yaw)
            # (0,25 m a 5 graus = 2,2 cm). Sem angulo confiavel nao da para
            # prever o avanco: com require_scan e recusa (sem_scan).
            if face_yaw is None:
                if require_scan:
                    log(f"face a {front:.3f} m SEM angulo confiavel (fit TLS "
                        "reprovado) — nao sei quanto o bico avanca num "
                        "lateral; nao ando")
                    return fail("sem_scan")
                self.get_logger().warn(
                    "nudge_base: face sem angulo confiavel "
                    "(nudge_require_scan=false) — assumo face perpendicular.")
                face_yaw = 0.0
            front_proj = _front_after_lateral(front, mag, face_yaw)
            if (front_proj < lateral_min - 1e-6
                    and bool(self.get_parameter("nudge_backoff_enabled").value)):
                margin = float(self.get_parameter("nudge_backoff_margin").value)
                need = (lateral_min + margin) - front_proj
                bmax = float(self.get_parameter("nudge_backoff_max").value)
                if need <= bmax:
                    # Alvo de FRENTE = front atual + o que falta na PROJECAO
                    # (campo 29/08: passar lateral_min+margem como alvo so'
                    # recuava 1 cm — a falta vem do yaw da face, nao da
                    # frente). E o passo nunca fica abaixo do coast: o
                    # _cut_drive cortaria antes do 1o comando e a base nem
                    # sairia do lugar (recuo pedido 5,7 cm < coast 6 cm).
                    coast_b = float(self.get_parameter("nudge_backoff_coast").value)
                    step = max(need, coast_b + 0.01)
                    target_front = front + step
                    log(f"perto da mesa (front {front:.3f}, projetado "
                        f"{front_proj:.3f} < {lateral_min:.2f}): recuo "
                        f"{step:.3f} m antes do lateral (alvo front {target_front:.3f})")
                    self._speak("Recuando antes de ir de lado")
                    # prazo proprio do recuo (o deadline do lateral so e
                    # calculado mais abaixo)
                    backoff_deadline = self.get_clock().now().nanoseconds * 1e-9 + float(timeout)
                    front_before = front
                    try:
                        self._nudge_backoff(goal_handle, rate, backoff_deadline,
                                            target_front)
                    except self._AttemptAbort as ab:
                        self._publish_cmd(0.0, 0.0, 0.0)
                        reason = {"canceled": "cancelado", "hard_stop": "muro",
                                  "timeout": "timeout", "soft_timeout": "timeout"}.get(
                            ab.outcome, "muro")
                        log(f"recuo abortado: {ab.outcome} ({ab.info})")
                        return fail(reason)
                    front, npts, scan_age, face_yaw = self._front_wall_distance()
                    if front is None:
                        log("sem medicao do muro apos o recuo")
                        return fail("sem_scan")
                    # O recuo (_cut_drive em x) nao tem watchdog por odometria:
                    # front inalterado apos o curso = a base nao respondeu
                    # (padrao dos logs de 16/09 e 15/10). Mesmo diagnostico
                    # do watchdog do lateral.
                    if abs(front - front_before) < 0.01:
                        self.get_logger().warn(
                            f"nudge_base: recuo sem efeito (base nao respondeu?): front "
                            f"{front_before:.3f} -> {front:.3f} apos recuo de {step:.3f} m; "
                            f"conferir publicadores em {self._ref_topic} (count_publishers="
                            f"{self.count_publishers(self._ref_topic)}), relogio PC x Pi "
                            f"(reference_timeout 0.15 s no mecanum_controller) e o piso do "
                            f"ESC (roda >= 2.43 rad/s)")
                    if face_yaw is None:
                        face_yaw = 0.0
                    front_proj = _front_after_lateral(front, mag, face_yaw)
                    log(f"apos o recuo: front {front:.3f} (projetado {front_proj:.3f})")
                else:
                    log(f"precisaria recuar {need:.3f} m > nudge_backoff_max {bmax:.2f}")
            if front_proj < lateral_min - 1e-6:
                log(f"front={front:.3f} (projetado {front_proj:.3f} apos "
                    f"{mag:.2f} m a {math.degrees(face_yaw):+.1f} graus) "
                    f"< lateral_min_front {lateral_min:.2f}")
                return fail("muito_perto_da_mesa")
            if not self._side_clear(sign, front):
                side = self._side_min[0] if sign > 0 else self._side_min[1]
                log(f"faixa lateral {'esq' if sign > 0 else 'dir'} ocupada a "
                    f"{side:.3f} m (face a {front:.3f})")
                return fail("obstaculo_lateral")

        # ---- (4) avisa ANTES de mover (fora do collision_monitor: o
        # operador afasta objetos), respeitando cancel durante a espera.
        self._nudge_gain = gain
        self._nudge_origin = origin
        self._speak("Ajustando para a " + ("esquerda" if sign > 0 else "direita"))
        start_s = self.get_clock().now().nanoseconds * 1e-9
        deadline = start_s + timeout
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 - start_s < lead:
            if goal_handle.is_cancel_requested:
                return fail("cancelado", canceled=True)
            rate.sleep()

        # Teto de tempo do curso com a velocidade de PARTIDA real (piso x
        # margem) — a mesma que _nudge_drive comanda e desconta no corte.
        v_start, _w = self._start_speeds()
        max_s = (mag + 0.05) / v_start + 1.0
        log(f"inicio: front={'?' if front is None else '%.3f' % front} "
            f"face_yaw={'?' if front is None else '%+.1f' % math.degrees(face_yaw)} "
            f"coast={coast:.3f} ganho_odom={gain:.2f} max_s={max_s:.1f} "
            f"timeout={timeout:.0f}")
        # Diagnostico de partida (2026-10-19): velocidade REAL comandada
        # (piso x margem) e quem mais esta' na referencia — um segundo
        # publicador (Nav2, teleop) zerando o topico anula o nudge; zero
        # assinantes = o mecanum_controller nao esta' ouvindo.
        log(f"comando: vy={math.copysign(v_start, sign):+.4f} m/s (piso {min_v:.4f} x "
            f"{1.0 + float(self.get_parameter('floor_start_margin').value):.2f}) em "
            f"{self._ref_topic}: publicadores={self.count_publishers(self._ref_topic)} "
            f"(este no' incluso), assinantes={self.count_subscribers(self._ref_topic)}")

        # ---- (5) curso continuo no piso, cortado pela odometria (com a
        # idade da medida descontada); depois assenta por ODOMETRIA — o
        # front nao muda num lateral, o _settle do docking devolveria com a
        # base ainda rodando (achado 3). Qualquer guarda vira _AttemptAbort.
        outcome, info = "ok", None
        drive_exit = None  # motivo de saida do laco do curso (diagnostico)
        settled = False
        try:
            drive_exit = self._nudge_drive(goal_handle, rate, deadline, sign, mag, origin,
                                           coast, max_s, require_scan, odom_max_age,
                                           gain, period, front, drift_max)
            settled = self._nudge_settle(goal_handle, rate, deadline, origin)
        except self._AttemptAbort as ab:
            outcome, info = ab.outcome, ab.info
            if drive_exit is None:
                drive_exit = "sem_progresso" if ab.outcome == "no_progress" else ab.outcome
            # Abortado a base ainda escorrega o coast: espera parar com a
            # referencia em zero (sem guardas — ja dispararam) para que o
            # total publicado seja a posicao REAL, nao a de meio do coast.
            settled = self._nudge_settle(goal_handle, rate, deadline, origin,
                                         guarded=False)
        self._publish_cmd(0.0, 0.0, 0.0)

        # ---- (6) medida final e total acumulado — SEMPRE, porque a base se
        # moveu de fato mesmo quando o curso ficou incompleto. O ganho de
        # escorregamento (achado 2) vale para o aceite e para o total.
        if self._odom_pose(odom_max_age) is None:
            self.get_logger().warn(
                "nudge_base: odometria velha na medida final — usando a ultima recebida.")
        raw, longi, dyaw = self._nudge_progress(origin)
        travelled = _nudge_corrected(raw, gain)
        self._set_shift_total(self._base_shift_total + travelled)
        log(f"fim ({outcome}{'' if info is None else ': ' + str(info)}; saida do "
            f"curso: {drive_exit}): lateral "
            f"{travelled * 100:+.1f} cm (odom bruto {raw * 100:+.1f} cm x ganho "
            f"{gain:.2f}; pedido {dy * 100:+.1f}), longitudinal "
            f"{longi * 100:+.1f} cm, yaw {math.degrees(dyaw):+.1f} graus, "
            f"assentou={'sim' if settled else 'NAO (teto)'}")

        if outcome == "canceled":
            return fail("cancelado", travelled, canceled=True)
        if outcome == "hard_stop":
            return fail("muro", travelled)
        if outcome in ("timeout", "soft_timeout"):
            return fail("timeout", travelled)
        if outcome == "side_blocked":
            return fail("obstaculo_lateral", travelled)
        if outcome == "shutdown":
            return fail("cancelado", travelled, canceled=True)
        if outcome == "no_progress":
            # Razao NOVA (2026-10-19): a base nao saiu do lugar. As demais
            # razoes nao mudam de texto (amt1_sort_action_node compara
            # strings exatas).
            return fail("sem_progresso", travelled)
        if _nudge_accepts(dy, travelled, slack):
            result.success = True
            result.reason = ""
            result.travelled = float(travelled)
            goal_handle.succeed()
            return result
        return fail("curso_incompleto", travelled)

    def _nudge_backoff(self, goal_handle, rate, deadline, target_front):
        """Re em linha reta (eixo x puro, como o undock) ate o muro ler
        >= target_front (o corte desconta o coast da re; o ESC retem a
        referencia ~0,5 s). Guardas do docking valem (scan velha/cega,
        cancel, prazo); muro nao dispara porque a frente so' cresce."""
        coast = float(self.get_parameter("nudge_backoff_coast").value)
        # O _cut_drive ja' parte acima do piso (2026-10-19); o teto de tempo
        # usa a mesma velocidade.
        v_start, _w = self._start_speeds()
        max_s = (float(self.get_parameter("nudge_backoff_max").value) + coast) / max(v_start, 1e-3) + 2.0

        def err_fn(front):
            return None if front is None else (front - target_front)

        # sign=-1 => corta quando (target_front - front) <= coast
        self._cut_drive(goal_handle, rate, deadline, deadline, "x", -1.0,
                        err_fn, coast, max_s, "nudge_backoff")
        self._publish_cmd(0.0, 0.0, 0.0)

    def _nudge_drive(self, goal_handle, rate, deadline, sign, mag, origin,
                     coast, max_s, require_scan, odom_max_age, gain, period,
                     front0, drift_max):
        """Laco do _cut_drive para o lateral por ODOMETRIA. Diferencas de
        proposito: o erro e avaliado a CADA ciclo (o _cut_drive so avalia em
        scan novo — sem scan ele andaria ate max_s sem nunca medir); sem
        medicao do muro (require_scan) ou odometria velha = corta na hora;
        a faixa lateral e vigiada a cada ciclo; o front nao pode cair mais
        que drift_max em relacao ao inicio (face inclinada = o bico avanca,
        achado 1b) — muro; o corte desconta a idade da odometria (achado
        5) e usa o lateral ja corrigido pelo ganho (achado 2). Guardas de
        cancel/timeout/hard_min em _check_guards, como nas outras
        primitivas. front0 = front da pre-checagem (None sem scan).
        Devolve o motivo de saida do laco (teto_de_tempo | corte | sem_scan
        | odometria_velha | shutdown) para o log de fim; abortos (muro,
        faixa lateral, sem_progresso, guardas) sobem como _AttemptAbort."""
        # Parte do repouso: piso x (1 + floor_start_margin). No piso exato a
        # roda fica um ULP abaixo do 2.43 rad/s do ESC e nunca liga (campo
        # 2026-10-19: "andou +0.000 m"). O corte desconta o lag com a MESMA
        # velocidade comandada.
        v_cmd, _w = self._start_speeds()
        no_prog_s = float(self.get_parameter("nudge_no_progress_s").value)
        no_prog_m = float(self.get_parameter("nudge_no_progress_min_m").value)
        start_s = self.get_clock().now().nanoseconds * 1e-9
        x0, y0, yaw0 = origin
        exit_reason = "shutdown"
        while rclpy.ok():
            front, _npts, _fy = self._check_guards(goal_handle, deadline, deadline)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - start_s > max_s:
                self.get_logger().warn("nudge_base: teto de tempo do curso — corto")
                exit_reason = "teto_de_tempo"
                break
            if require_scan and front is None:
                self.get_logger().warn(
                    "nudge_base: muro sem medicao no meio do curso (STALE/BLIND) — corto")
                exit_reason = "sem_scan"
                break
            if front is not None:
                if front0 is None:
                    front0 = front
                elif front < front0 - drift_max:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    raise self._AttemptAbort(
                        "hard_stop",
                        f"muro: bico avancou {(front0 - front) * 100:.1f} cm no "
                        f"lateral (front {front:.3f} < {front0:.3f} - "
                        f"{drift_max:.3f}; face inclinada?)")
            if not self._side_clear(sign, front):
                self._publish_cmd(0.0, 0.0, 0.0)
                raise self._AttemptAbort(
                    "side_blocked", "obstaculo entrou na faixa lateral durante o curso")
            sample = self._odom_sample(odom_max_age)
            if sample is None:
                self.get_logger().warn("nudge_base: odometria velha no meio do curso — corto")
                exit_reason = "odometria_velha"
                break
            raw = _lateral_displacement(x0, y0, yaw0, sample[0], sample[1])
            travelled = _nudge_corrected(raw, gain)
            if _nudge_cut_reached(mag, travelled, sign, coast, v_cmd,
                                  sample[3], period):
                # Falta so o coast (+ o que anda ate a medida chegar):
                # corta e deixa o ESC terminar.
                self.get_logger().info(
                    f"nudge_base: corte a {travelled * 100:+.1f} cm (odom bruto "
                    f"{raw * 100:+.1f} cm, idade {sample[3] * 1000:.0f} ms)")
                exit_reason = "corte"
                break
            # Watchdog de progresso (2026-10-19): comandando ha' no_prog_s e
            # a odometria nao saiu do lugar = a base nao respondeu. Aborta
            # ja' com o diagnostico, em vez de esperar o teto e devolver
            # 'curso_incompleto' sem explicacao.
            elapsed = now_s - start_s
            if _nudge_no_progress(elapsed, travelled, no_prog_s, no_prog_m):
                self._publish_cmd(0.0, 0.0, 0.0)
                self.get_logger().warn(
                    f"nudge_base: base nao respondeu ao comando (vy="
                    f"{math.copysign(v_cmd, sign):+.4f} m/s por {elapsed:.1f} s, odom "
                    f"bruto {raw * 100:+.1f} cm): conferir publicadores em "
                    f"{self._ref_topic} (count_publishers="
                    f"{self.count_publishers(self._ref_topic)}), relogio PC x Pi "
                    f"(reference_timeout 0.15 s no mecanum_controller) e o piso do "
                    f"ESC (roda >= 2.43 rad/s)")
                raise self._AttemptAbort(
                    "no_progress", f"sem progresso em {elapsed:.1f} s")
            self._publish_cmd(0.0, math.copysign(v_cmd, sign), 0.0)
            self._feedback(goal_handle, "deslocando", 0.0, 0.0, 0.0)
            rate.sleep()
        self._publish_cmd(0.0, 0.0, 0.0)
        return exit_reason

    def _nudge_settle(self, goal_handle, rate, deadline, origin, guarded=True):
        """Assenta por ODOMETRIA (achado 3): publica zero ate o lateral
        ficar dentro de nudge_settle_eps por nudge_settle_min_s (criterio em
        _OdomSettle), teto nudge_settle_max_s. Devolve True se assentou,
        False no teto (sem odometria nova tambem cai no teto). guarded=False
        = so espera parar, sem guardas nem feedback (depois de um abort, quando
        elas ja dispararam) — a referencia e zero, nada anda."""
        eps = float(self.get_parameter("nudge_settle_eps").value)
        min_s = float(self.get_parameter("nudge_settle_min_s").value)
        max_s = float(self.get_parameter("nudge_settle_max_s").value)
        tracker = _OdomSettle(eps, min_s)
        x0, y0, yaw0 = origin
        start_s = self.get_clock().now().nanoseconds * 1e-9
        last_rx = -1
        while rclpy.ok():
            if guarded:
                self._check_guards(goal_handle, deadline, deadline)
            self._publish_cmd(0.0, 0.0, 0.0)
            if guarded:
                self._feedback(goal_handle, "assentando", 0.0, 0.0, 0.0)
            odom = self._odom
            if odom is not None and odom[3] != last_rx:
                last_rx = odom[3]
                lat = _lateral_displacement(x0, y0, yaw0, odom[0], odom[1])
                if tracker.update(odom[3] * 1e-9, lat):
                    return True
            if self.get_clock().now().nanoseconds * 1e-9 - start_s > max_s:
                self.get_logger().warn(
                    f"nudge_base: assentamento por odometria estourou o teto "
                    f"({max_s:.1f} s) — meco assim mesmo")
                return False
            rate.sleep()
        return False

    def _abort(self, goal_handle, result, message):
        self.get_logger().warn(message)
        result.success = False
        result.message = message
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DockAlignNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
