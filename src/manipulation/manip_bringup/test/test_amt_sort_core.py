"""Testes da AMT refeita (amt_sort_core), sem ROS: quem esta' em qual slot e o plano de trocas."""
import itertools
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import amt_sort_core as A  # noqa: E402

SLOTS = [101, 102, 103, 104, 105, 106, 107]


def quadrado(cx, cy, lado, ang=0.0):
    c, s, h = np.cos(ang), np.sin(ang), lado / 2.0
    return [[cx + c * a - s * b, cy + s * a + c * b] for a, b in ((-h, -h), (h, -h), (h, h), (-h, h))]


def mesa(blocos, desvio_m=0.0, tapadas=(), largura=640, altura=480):
    """Vista REALISTA de pegar_obj: camera 39 cm acima do tampo, 13 cm a' frente do
    braco, olhando para a frente e para baixo; slots a 107 mm um do outro, tag do
    slot a 38 cm, bloco 46 mm antes dela com a tag 4 cm acima do tampo (paralaxe).
    desvio_m = base deslocada de lado (+ = esquerda). So' entra o que cabe na imagem.
    Como no robo (16/11 21h47): o bloco TAPA a tag do proprio slot, e a imagem e'
    espelhada (o slot de id maior aparece a' esquerda)."""
    cam = np.array([0.0, 0.13, 0.39])
    t = np.radians(35.0)
    zc = np.array([0.0, np.sin(t), -np.cos(t)])
    xc = np.array([1.0, 0.0, 0.0])
    yc = np.cross(zc, xc)

    def tag(centro, lado):
        px = []
        for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            d = centro + np.array([a * lado / 2.0, b * lado / 2.0, 0.0]) - cam
            px.append([320.0 - 386.0 * (d @ xc) / (d @ zc), 386.0 * (d @ yc) / (d @ zc) + 240.0])
        px = np.array(px)
        return px.tolist() if np.all((px > 2) & (px < [largura - 2, altura - 2])) else None

    tags = {}
    for i, s in enumerate(SLOTS):
        x = (i - 3) * 0.107 + desvio_m
        if s not in tapadas and blocos.get(s) is None:
            tags[s] = tag(np.array([x, 0.38, 0.0]), 0.0318)
        if blocos.get(s) is not None:
            tags[blocos[s]] = tag(np.array([x, 0.334, 0.040]), 0.0318)
    return {k: v for k, v in tags.items() if v is not None}


def test_arranjo_com_medidas_reais_de_16_11():
    # Vista de verificacao do place bom (18h01, bag): bloco tag 1 no slot 104.
    tags = {1: quadrado(168.9, 191.6, 58.2, np.radians(1.6)),
            103: quadrado(256.9, 45.7, 46.4, np.radians(92.3)),
            104: quadrado(251.2, 202.6, 47.7, np.radians(91.3)),
            105: quadrado(245.4, 368.5, 48.5, np.radians(91.6))}
    no_slot, vazios, soltos = A.arranjo_do_quadro(tags, SLOTS)
    assert no_slot == {1: 104} and soltos == []
    assert vazios == []                                    # vizinhos perto da borda: nao afirma
    # 18h56: o bloco tag 26 tapava a borda da tag 104 (so' 103 e 105 detectadas).
    tags = {26: quadrado(345.1, 168.9, 55.4, np.radians(-92.7)),
            103: quadrado(408.6, 30.3, 46.3, np.radians(91.5)),
            105: quadrado(402.5, 356.3, 48.3, np.radians(91.4))}
    no_slot, vazios, _ = A.arranjo_do_quadro(tags, SLOTS)
    assert no_slot == {26: 104}


REAIS = json.load(open(os.path.join(os.path.dirname(__file__), "data", "amt", "deteccoes_214652.json"),
                      encoding="utf-8"))["quadros"]
# Como a mesa estava em 16/11 21h47 (conferido no bag): 107 vazio, o resto abaixo.
MESA_REAL = {106: 1, 105: 4, 104: 6, 103: 2, 102: 3, 101: 5, 107: None}


def _fila(quadros):
    f = A.Fileira(SLOTS)
    for q in quadros:
        f.juntar({int(k): v for k, v in q["tags"].items()})
    return f


def test_fileira_com_as_deteccoes_reais_da_corrida_que_falhou():
    # Nessa corrida nenhuma tag de slot OCUPADO apareceu (o bloco tapa): a regra antiga nao via nada.
    sem_slot = [q for q in REAIS if not any(int(k) in SLOTS for k in q["tags"])]
    assert len(sem_slot) == len(REAIS) - 1                 # so' 1 dos 41 quadros tem tag de slot (a 107)
    assert all(A.arranjo_do_quadro({int(k): v for k, v in q["tags"].items()}, SLOTS)[0] == {} for q in sem_slot)
    est, ancorado = _fila(REAIS).estado()
    assert est == MESA_REAL and ancorado                   # a tag 107 (slot vazio) apareceu num quadro
    # So' os quadros parados em pegar_obj: 5 blocos, sem ancora -> hipotese "comeca no 1o slot"
    parado = [q for q in REAIS if abs(q["q"][1] + 0.376) < 0.01]
    est, ancorado = _fila(parado).estado()
    assert est == {s: c for s, c in MESA_REAL.items() if c not in (1, None)} and not ancorado
    assert A.completar(est, [1, 2, 3, 4, 5, 6], SLOTS) == est     # falta o bloco 1: nao conclui
    assert _fila(parado[:1]).estado() == ({}, False)       # um quadro so' nao vale


def test_fileira_com_tag_de_slot_ocupado_a_vista_17_11():
    # Poses laterais (mais altas): a tag do slot OCUPADO aparece colada no bloco. Em 17/11 16h12
    # ela entrava na fila como slot vazio e a AMT leu {105: 3, 102: 6, ...} e planejou errado.
    verdade = {101: 2, 102: 1, 103: 6, 104: 3, 105: 4, 106: 5, 107: None}
    corridas = json.load(open(os.path.join(os.path.dirname(__file__), "data", "amt", "deteccoes_1117.json"),
                              encoding="utf-8"))["corridas"]
    for quadros in corridas.values():
        assert _fila(quadros).estado() == (verdade, True)
        # so' a olhada para a esquerda: 4 slots, cada um pela tag colada no bloco dele
        est, ancorado = _fila([q for q in quadros if q["j1"] > 2.0]).estado()
        assert est == {s: c for s, c in verdade.items() if s <= 104} and ancorado
        # so' de pegar_obj as tags ficam tapadas: sem ancora, quem chama nao pode confiar
        assert not _fila([q for q in quadros if 1.4 < q["j1"] < 1.7]).estado()[1]
    # tag de slot na borda sem bloco ao lado (103@601 de tag_direita): o bloco ficou fora da imagem
    est, _ = _fila([q for q in corridas["161638"] if q["j1"] < 1.1]).estado()
    assert 103 not in est and est[107] is None


def test_fileira_emenda_vistas_e_acha_buraco_no_meio():
    blocos = {101: 5, 102: 2, 103: None, 104: 1, 105: 4, 106: 3, 107: 6}
    f = A.Fileira(SLOTS)
    for desvio in (0.0, 0.0, 0.15, 0.15, -0.15, -0.15):     # centro, base a' esquerda e a' direita
        f.juntar(mesa(blocos, desvio))
    est, ancorado = f.estado()
    assert est == blocos and ancorado                      # tag 103 (vazio) a' vista + fila de 7
    # sem ver a tag do slot vazio (tapada por algo): a fila de 7 posicoes ja' ancora sozinha
    f = A.Fileira(SLOTS)
    for desvio in (0.0, 0.0, 0.15, 0.15, -0.15, -0.15):
        f.juntar(mesa(blocos, desvio, tapadas=(103,)))
    assert f.estado() == (blocos, True)


def test_fileira_ignora_bloco_fora_da_fila():
    blocos = {103: 2, 104: 1, 105: 3}
    f = A.Fileira(SLOTS)
    for _ in range(2):
        tags = mesa(blocos)
        tags[9] = quadrado(300, 30, 30)                    # bloco na garra / no chao: longe da fila
        f.juntar(tags)
    est, _anc = f.estado()
    assert 9 not in est.values() and est[104] == 1 and est[103] == 2 and est[105] == 3
    assert est[102] is None and est[106] is None           # tags dos slots vazios vizinhos a' vista
    assert A.Fileira(SLOTS).estado() == ({}, False)


def test_plano_ordena_todas_as_permutacoes():
    ids = [3, 8, 12, 21, 26, 29]                           # ids quaisquer, como vem do arquivo
    alvo = A.alvo_por_ordem(SLOTS, sorted(ids))
    assert alvo == {101: 3, 102: 8, 103: 12, 104: 21, 105: 26, 106: 29}
    pior = 0
    for perm in itertools.permutations(ids):
        estado = dict(zip(SLOTS, list(perm) + [None]))
        ops, sobra = A.planejar(estado, alvo, capacidade=3)
        fim, bordo = A.simular(estado, ops, capacidade=3)
        assert sobra == [] and bordo == [] and A.ordem(fim, SLOTS) == sorted(ids), (perm, ops)
        fora = sum(1 for s, c in estado.items() if c is not None and alvo.get(s) != c)
        assert len(ops) == 2 * fora                        # so' mexe em quem esta' fora do lugar
        pior = max(pior, len(ops))
    assert pior == 12


def test_plano_com_dois_potes_e_com_um():
    alvo = A.alvo_por_ordem(SLOTS, [1, 2, 3])
    estado = {101: 2, 102: 3, 103: 1, 104: None}
    ops, sobra = A.planejar(estado, alvo, capacidade=2)
    fim, bordo = A.simular(estado, ops, capacidade=2)
    assert sobra == [] and bordo == [] and A.ordem(fim, SLOTS) == [1, 2, 3]
    ops, sobra = A.planejar(estado, alvo, capacidade=1)    # ciclo fechado nao roda com 1 pote
    assert ops == [] and sobra == [1, 2, 3]


def test_plano_usa_slot_vazio_e_respeita_o_que_nao_foi_visto():
    # bloco no slot 107 cujo lugar (101) esta' vazio: 1 pick + 1 place
    alvo = A.alvo_por_ordem(SLOTS, [1, 2])
    ops, sobra = A.planejar({101: None, 102: 2, 107: 1}, alvo)
    assert ops == [("pick", 1, 107), ("place", 1, 101)] and sobra == []
    # slot-alvo do bloco 1 nao foi visto: ele fica onde esta'
    ops, sobra = A.planejar({102: 2, 107: 1}, alvo)
    assert ops == [] and sobra == []
    # bloco que ja' veio a bordo (tentativa anterior) e' solto primeiro
    ops, sobra = A.planejar({101: None, 102: 2}, alvo, a_bordo=[1])
    assert ops == [("place", 1, 101)] and sobra == []
    assert A.alvo_por_ordem(SLOTS, [1, 2, 3], "right_to_left") == {107: 1, 106: 2, 105: 3}


def test_plano_nao_deixa_bloco_a_bordo_quando_o_ciclo_nao_fecha():
    # O bloco 9 (fora do arquivo) ocupa o lugar do 1: nada pode ser tirado para "abrir vaga".
    alvo = A.alvo_por_ordem(SLOTS, [1, 2])
    estado = {101: 9, 102: 1, 103: 2}
    ops, sobra = A.planejar(estado, alvo)
    _fim, bordo = A.simular(estado, ops)
    assert bordo == [] and sobra == [1, 2] and ops == []
    # idem quando o slot-alvo de quem bloqueia nao foi visto
    ops, sobra = A.planejar({102: 1, 103: 2, 104: 3}, A.alvo_por_ordem(SLOTS, [1, 2, 3]))
    assert A.simular({102: 1, 103: 2, 104: 3}, ops)[1] == [], ops


def test_eliminacao_so_com_todos_os_blocos_localizados():
    est = {101: 5, 102: 2, 103: 6, 104: 1, 105: 4, 106: 3}
    assert A.completar(est, [1, 2, 3, 4, 5, 6], SLOTS)[107] is None
    assert 107 not in A.completar({101: 5}, [5, 9], SLOTS)  # falta o 9: nao conclui nada
