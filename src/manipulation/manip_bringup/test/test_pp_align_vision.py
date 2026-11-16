"""Testes da visao do alinhamento cubo -> furo da PP (sem ROS), 2026-11-14.

Cobrem a geometria pura (erro do cubo no plano do tampo a partir de um furo
vizinho) e a medida em quadros REAIS da camera do punho gravados em 14/11
(corridas run_20261114_112951 e run_20261114_120744): cubo vermelho e cubo
azul na pre-pose, garra vazia e descida sem furo vizinho inteiro.
"""
import math
import os
import sys

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import pp_align_vision as V  # noqa: E402

DADOS = os.path.join(HERE, "data", "pp_align")
MESA = V.carregar_mesa(os.path.join(HERE, "..", "config", "pp_table.yaml"))


def _img(nome):
    img = cv2.imread(os.path.join(DADOS, nome))
    assert img is not None, nome
    return img


def test_mesa_tem_as_medidas_da_regua():
    assert MESA["pitch_mm"] == pytest.approx(109.0)
    assert 40.0 < MESA["hole_side_mm"] < 46.0
    assert len(MESA["slot_tags"]) == len(MESA["slot_correction_mm"]) == 7


def test_geometria_devolve_o_deslocamento_do_cubo():
    # mesa -> imagem: 2,5 px/mm, girada 7 graus, origem (furo-alvo) em (300, 260)
    s, ang, o = 2.5, math.radians(7.0), np.array([300.0, 260.0])
    R = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
    px = lambda p: o + s * (R @ np.asarray(p, float))  # noqa: E731
    lado, passo, cubo = MESA["hole_side_mm"], MESA["pitch_mm"], MESA["cube_side_mm"]
    vizinho = [px((passo + a * lado / 2, b * lado / 2)) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    # cubo com o centro em (+6, -15) mm e girado 3 graus em relacao a' fileira
    c, g = np.array([6.0, -15.0]), math.radians(3.0)
    e, n = np.array([math.cos(g), math.sin(g)]), np.array([-math.sin(g), math.cos(g)])
    A, B = px(c + n * cubo / 2 - e * cubo / 2), px(c + n * cubo / 2 + e * cubo / 2)
    r = V.erro_cubo_no_plano([(1, vizinho)], A, B, MESA)
    assert r["erro_x_mm"] == pytest.approx(-6.0, abs=0.05)
    assert r["erro_y_mm"] == pytest.approx(15.0, abs=0.05)
    assert r["yaw_deg"] == pytest.approx(3.0, abs=0.05)
    assert r["aresta_mm"] == pytest.approx(cubo, abs=0.05)


@pytest.mark.parametrize("a, b", [("vermelho_prepose_a.jpg", "vermelho_prepose_b.jpg"),
                                  ("azul_prepose_a.jpg", "azul_prepose_b.jpg")])
def test_mede_cubo_vermelho_e_azul_na_prepose(a, b):
    ra, rb = (V.medir_por_vizinhos(_img(n), MESA) for n in (a, b))
    for r in (ra, rb):
        assert r["ok"], r["motivos"]
        assert r["n_vizinhos"] >= 1
        # em 14/11 o cubo parou ~18 mm antes do furo (o no' leu 18-19 mm na verificacao)
        assert 12.0 < r["erro_y_mm"] < 26.0
        assert abs(r["erro_x_mm"]) < 20.0 and abs(r["yaw_deg"]) < 6.0
    # braco parado entre os dois quadros: a medida tem de repetir
    assert abs(ra["erro_x_mm"] - rb["erro_x_mm"]) < 2.5
    assert abs(ra["erro_y_mm"] - rb["erro_y_mm"]) < 2.5


def test_garra_vazia_diz_sem_cubo():
    r = V.medir_por_vizinhos(_img("garra_vazia.jpg"), MESA)
    assert not r["ok"] and r["motivos"] == ["sem_cubo"]


def test_descida_sem_furo_vizinho_inteiro_nao_inventa_medida():
    r = V.medir_por_vizinhos(_img("azul_descida_sem_vizinho.jpg"), MESA)
    assert not r["ok"] and r["motivos"]
