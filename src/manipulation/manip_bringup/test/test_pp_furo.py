"""Testes do pp_furo (furo e cubo da mesa de precisao pela camera), sem ROS.

Tudo contra QUADROS REAIS da D455 do punho gravados em 16/11 (test/data/pp_furo):
garra vazia de perto (furo inteiro), cubo em cima do furo nos places que deram
certo, e cubo fora do lugar nos places que deram errado.
"""
import json
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import pp_furo as F  # noqa: E402

DADOS = os.path.join(os.path.dirname(__file__), "data", "pp_furo")
META = json.load(open(os.path.join(DADOS, "quadros.json"), encoding="utf-8"))["quadros"]
VETOR_104 = (0.36, -45.8)      # centro do furo do slot 104 no frame da tag (mm), medido em 16/11


def quadro(nome):
    m = META[nome]
    return cv2.imread(os.path.join(DADOS, nome + ".jpg")), m, {int(k): np.array(v) for k, v in m["tags"].items()}


@pytest.mark.parametrize("nome", ["perto_vazia_a", "perto_vazia_b"])
def test_furo_inteiro_tem_centro_e_tamanho(nome):
    bgr, _m, tags = quadro(nome)
    f = F.medir_furo(bgr, tags[104])
    assert f["ok"] and f["completo"], f
    assert abs(f["centro_mm"][0] - VETOR_104[0]) < 0.8 and abs(f["centro_mm"][1] - VETOR_104[1]) < 0.8, f
    assert abs(f["lado_mm"][0] - 45.0) < 1.5 and abs(f["lado_mm"][1] - 44.2) < 1.5, f
    assert abs(f["giro_graus"]) < 2.5
    inteiros = 0
    for vizinho in (103, 105):          # furo vizinho: vale quando esta' inteiro no quadro
        g = F.medir_furo(bgr, tags[vizinho])
        if g["completo"]:
            inteiros += 1
            assert abs(g["centro_mm"][0]) < 2.0 and -48.0 < g["centro_mm"][1] < -44.0, g
    assert inteiros >= 1                # cortado pela borda da imagem nao conta como medida


@pytest.mark.parametrize("nome", ["prepose_bom_a", "prepose_bom_b"])
def test_cubo_em_cima_do_furo_nos_places_bons(nome):
    bgr, m, tags = quadro(nome)
    f = F.medir_furo(bgr, tags[104])
    assert not f["completo"]                       # o cubo tapa o furo: nao serve de medida
    e = F.erro_do_cubo(tags[104], m["k"], VETOR_104, m["q5"])
    assert abs(e["lat_mm"]) < 1.0 and abs(e["rad_mm"]) < 1.0, e
    assert 11.0 < e["alt_mm"] < 19.0 and abs(e["giro_graus"]) < 0.7, e


def test_cubo_fora_no_place_da_vermelha():
    # 18h30: o place parou ~18 mm de lado e ~17 mm a' frente do furo, 3 graus girado.
    _bgr, m, tags = quadro("prepose_fora")
    e = F.erro_do_cubo(tags[104], m["k"], VETOR_104, m["q5"])
    assert abs(e["lat_mm"] - 17.7) < 1.5 and abs(e["rad_mm"] + 16.8) < 1.5, e
    assert abs(e["giro_graus"] - 3.1) < 0.7, e


def test_erro_de_posicao_nao_se_confunde_com_giro():
    # Depois da malha antiga: imagem "centrada" com o cubo girado 20 graus. O fundo
    # do cubo continuava ~19 mm de lado — foi por isso que caiu fora.
    _bgr, m, tags = quadro("prepose_girado")
    e = F.erro_do_cubo(tags[104], m["k"], VETOR_104, m["q5"])
    assert abs(e["lat_mm"] - 18.7) < 1.5 and abs(e["giro_graus"] + 19.8) < 1.0, e
    _bgr, m, tags = quadro("prepose_malha_ruim")
    e = F.erro_do_cubo(tags[104], m["k"], VETOR_104, m["q5"])
    assert abs(e["lat_mm"] + 16.3) < 1.5 and 5.0 < e["alt_mm"] < 11.0, e    # e 7 mm mais baixo (flecha)


def test_medir_guarda_o_furo_visto_e_segue_pela_tag():
    mem = F.Memoria()
    bgr, m, tags = quadro("prepose_bom_a")
    r = F.medir(bgr, m["k"], tags[104], m["q5"], mem)
    assert not r["ok"] and r["furo"] == "nao" and r["motivo"] == "furo_nunca_visto"
    bgr, m, tags = quadro("perto_vazia_a")
    r = F.medir(bgr, m["k"], tags[104], m["q5"], mem)
    assert r["ok"] and r["furo"] == "visto" and mem.n == 1
    assert abs(mem.vetor_mm[0] - VETOR_104[0]) < 0.8 and abs(mem.vetor_mm[1] - VETOR_104[1]) < 0.8
    bgr, m, tags = quadro("prepose_bom_a")           # agora com o cubo tapando: usa o vetor guardado
    r = F.medir(bgr, m["k"], tags[104], m["q5"], mem)
    assert r["ok"] and r["furo"] == "pela_tag" and mem.n == 1
    assert abs(r["lat_mm"]) < 1.2 and abs(r["rad_mm"]) < 1.2, r


def test_ordem_dos_cantos_nao_importa():
    bgr, m, tags = quadro("perto_vazia_b")
    ref = F.medir_furo(bgr, tags[104])
    for k in (1, 2, 3):
        f = F.medir_furo(bgr, np.roll(tags[104], k, axis=0))
        assert f["completo"] and np.allclose(f["centro_mm"], ref["centro_mm"], atol=0.05), (k, f)


# 2026-11-26: PP da ARENA (foto de celular do operador): fundo do furo CLARO e
# furo ~10 mm afastado da tag. A mascara de cor dava "sem_furo"; a busca pela
# borda (metodo auto) tem de achar o furo inteiro alem da tag.
ARENA = json.load(open(os.path.join(DADOS, "arena_foto_celular.json"), encoding="utf-8"))


@pytest.mark.parametrize("tag", ["103", "104", "105", "106"])
def test_arena_furo_claro_achado_pela_borda(tag):
    bgr = cv2.imread(os.path.join(DADOS, "arena_foto_celular.png"))
    cantos = np.array(ARENA["tags"][tag])
    cor = F.medir_furo(bgr, cantos, F.FuroCfg(metodo="cor"))
    assert not (cor["ok"] and cor["completo"])          # o problema da arena
    f = F.medir_furo(bgr, cantos, F.FuroCfg())          # auto
    assert f["ok"] and f["completo"] and f["metodo"] == "borda"
    cx, cy = f["centro_mm"]
    assert abs(cx) < 4.0 and -60.0 < cy < -52.0         # ~55 mm alem da tag (laboratorio: 45,8)
    assert 44.0 <= f["lado_mm"][0] <= 52.0


@pytest.mark.parametrize("nome", sorted(META))
def test_auto_nunca_aceita_furo_que_a_cor_recusou_errado(nome):
    """No laboratorio o auto so' pode concordar com a cor ou achar o furo no lugar certo."""
    bgr, _m, tags = quadro(nome)
    for _tid, cantos in tags.items():
        cor = F.medir_furo(bgr, cantos, F.FuroCfg(metodo="cor"))
        auto = F.medir_furo(bgr, cantos, F.FuroCfg())
        if cor["ok"] and cor["completo"]:
            assert auto == cor
        elif auto["ok"] and auto["completo"]:
            cx, cy = auto["centro_mm"]
            assert abs(cx) < 2.0 and abs(cy - VETOR_104[1]) < 2.0 and auto["giro_cantos"] == 0
