#!/usr/bin/env python3
"""Nucleo (sem ROS) do vigia de localizacao (2026-11-11).

Ideia: a odometria do Caramelo (rodas + giroscopio fundidos no EKF da Pi) e'
confiavel em janelas curtas — no bag das 22h03 o giro integrado do giroscopio
e o do EKF diferem 2 graus em 150 s, enquanto o AMCL "perdeu" 89 graus e
atravessou a parede central do mapa. Entao: a pose do AMCL so' pode se afastar
de (ultima pose confiavel ⊕ deslocamento da odometria desde entao) por uma
folga pequena; se passar da folga por `strikes` atualizacoes seguidas, o vigia
manda o AMCL reiniciar na pose propagada pela odometria (/initialpose).
Tudo em 2D (x, y, yaw). Testado offline nos bags (loc_watchdog_offline.py).
"""
import math
from dataclasses import dataclass, field


def ang(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a, b):
    """a ⊕ b: aplica b (x,y,yaw) no frame de a."""
    ax, ay, at = a
    bx, by, bt = b
    return (ax + math.cos(at) * bx - math.sin(at) * by,
            ay + math.sin(at) * bx + math.cos(at) * by,
            ang(at + bt))


def inverse(a):
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return (-(c * ax + s * ay), -(-s * ax + c * ay), ang(-at))


def between(a, b):
    """a ⊖ b: transformacao T tal que T ⊕ b = a (ex.: map->odom = amcl ⊖ odom)."""
    return compose(a, inverse(b))


@dataclass
class Params:
    folga_xy: float = 0.10          # m: folga fixa por verificacao
    folga_xy_por_metro: float = 0.08  # fracao da distancia odometrica desde a ultima aceitacao
    folga_yaw: float = math.radians(8.0)   # rad fixo (8 graus: cobre o atraso do AMCL num giro rapido; 5 disparava nos trancos do ESC)
    folga_yaw_por_rad: float = 0.08        # fracao do giro odometrico desde a ultima aceitacao
    strikes_para_reset: int = 2      # atualizacoes seguidas fora da folga
    max_intervalo_s: float = 30.0    # sem /amcl_pose por mais que isto, re-ancora sem acusar
    min_intervalo_reset_s: float = 2.0


@dataclass
class Estado:
    t_ok: float = -1.0               # instante da ultima aceitacao
    map_odom: tuple = None           # map->odom confiavel (x, y, yaw)
    odom_ok: tuple = None            # pose odom na ultima aceitacao
    dist_odom: float = 0.0           # distancia odometrica acumulada desde t_ok
    giro_odom: float = 0.0           # giro odometrico acumulado desde t_ok (abs)
    odom_prev: tuple = None
    strikes: int = 0
    t_ultimo_reset: float = -1e9
    resets: list = field(default_factory=list)


class Vigia:
    def __init__(self, p: Params = None):
        self.p = p or Params()
        self.e = Estado()

    def odom(self, t, pose):
        """Alimenta a odometria (odom->base) a cada mensagem; acumula distancia e giro."""
        if self.e.odom_prev is not None:
            dx = pose[0] - self.e.odom_prev[0]
            dy = pose[1] - self.e.odom_prev[1]
            self.e.dist_odom += math.hypot(dx, dy)
            self.e.giro_odom += abs(ang(pose[2] - self.e.odom_prev[2]))
        self.e.odom_prev = pose

    def ancorar(self, t, amcl, odom):
        self.e.t_ok = t
        self.e.map_odom = between(amcl, odom)
        self.e.odom_ok = odom
        self.e.dist_odom = 0.0
        self.e.giro_odom = 0.0
        self.e.strikes = 0

    def amcl(self, t, amcl, odom):
        """Uma atualizacao do AMCL (pose no mapa) com a pose odom do mesmo instante.
        Devolve None (ok), ou a pose (x, y, yaw) no mapa em que o AMCL deve ser
        reiniciado."""
        e, p = self.e, self.p
        if e.map_odom is None or (t - e.t_ok) > p.max_intervalo_s:
            self.ancorar(t, amcl, odom)
            return None
        esperado = compose(e.map_odom, odom)
        dxy = math.hypot(amcl[0] - esperado[0], amcl[1] - esperado[1])
        dyaw = abs(ang(amcl[2] - esperado[2]))
        folga_xy = p.folga_xy + p.folga_xy_por_metro * e.dist_odom
        folga_yaw = p.folga_yaw + p.folga_yaw_por_rad * e.giro_odom
        self.ultimo = (dxy, dyaw, folga_xy, folga_yaw, esperado)
        if dxy <= folga_xy and dyaw <= folga_yaw:
            self.ancorar(t, amcl, odom)   # segue a correcao lenta do AMCL
            return None
        e.strikes += 1
        if e.strikes < p.strikes_para_reset or (t - e.t_ultimo_reset) < p.min_intervalo_reset_s:
            return None
        # reset: a verdade assumida e' a odometria desde a ultima aceitacao
        e.t_ultimo_reset = t
        e.resets.append((t, dxy, dyaw, esperado, amcl))
        self.ancorar(t, esperado, odom)
        return esperado
