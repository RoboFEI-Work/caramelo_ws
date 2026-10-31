"""Giro em pulsos do dock_align_node (2026-10-27), sem ROS ativo.

Operador: "ele gira sempre a mesma quantidade e nunca se ajusta". O plano
do pulso escolhe a duracao pelo que falta e recusa o pulso que passaria do
ponto; o modelo giro(T) = c + k*T aprende com o giro medido.
"""
import importlib.util
import math
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "scripts", "dock_align_node.py")
_spec = importlib.util.spec_from_file_location("dock_align_node", SCRIPT)
dan = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dan)

D = math.radians


def test_dentro_da_tolerancia_nao_pulsa():
    t, why = dan.plan_rot_pulse(D(1.5), D(3), D(17), 0.15, 0.8, D(2))
    assert t is None and why == "dentro"


def test_pulso_proporcional_ao_que_falta():
    t_small, _ = dan.plan_rot_pulse(D(8), D(3), D(17), 0.15, 0.8, D(2))
    t_big, _ = dan.plan_rot_pulse(D(15), D(3), D(17), 0.15, 0.8, D(2))
    assert 0.15 <= t_small < t_big <= 0.8
    assert t_small == pytest.approx((8 - 3) / 17, abs=1e-6)


def test_sinal_nao_importa_para_a_duracao():
    a, _ = dan.plan_rot_pulse(D(10), D(3), D(17), 0.15, 0.8, D(2))
    b, _ = dan.plan_rot_pulse(D(-10), D(3), D(17), 0.15, 0.8, D(2))
    assert a == b


def test_recusa_pulso_que_passaria_do_ponto():
    # giro minimo ~ 6 + 17*0.15 = 8,6 graus para corrigir 4: passaria do ponto
    t, why = dan.plan_rot_pulse(D(4), D(6), D(17), 0.15, 0.8, D(2))
    assert t is None and "minimo" in why


def test_modelo_aprende_k_e_c():
    c, k = D(3), D(17)
    # pulso de 0,4 s girou 15 graus: k sobe
    c2, k2 = dan.update_rot_model(c, k, 0.4, D(15), 0.15)
    assert k2 > k and c2 == c
    # pulso MINIMO girou 10,8 graus (robo, 27/10): o menor giro e' maior -> c sobe
    c3, k3 = dan.update_rot_model(c, k, 0.15, D(10.8), 0.15)
    assert c3 > c and k3 == k
    # pulso girou menos que o c: c desce
    c4, k4 = dan.update_rot_model(c, k, 0.30, D(1), 0.15)
    assert c4 < c and k4 == k
    # nao girou (ou girou ao contrario): nao mexe
    assert dan.update_rot_model(c, k, 0.3, 0.0, 0.15) == (c, k)
