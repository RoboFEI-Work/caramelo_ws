"""Testes da matematica PURA do nudge_base (dock_align_node), sem ROS ativo.

Importa o script como modulo (mesmo truque de
manip_bringup/test/test_container_detector.py): o import puxa rclpy e
caramelo_msgs (o ambiente precisa estar sourceado), mas nao inicializa nada.

Convencao (2026-08-28): base_footprint +y = ESQUERDA; a odometria da (x, y,
yaw) no frame odom; o curso lateral e a projecao do deslocamento no eixo +y
INICIAL do robo (yaw0) — + = andou para a esquerda.
"""
import importlib.util
import math
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "scripts", "dock_align_node.py")


def _load():
    spec = importlib.util.spec_from_file_location("dock_align_node", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


dan = _load()
lat = dan._lateral_displacement
lon = dan._longitudinal_displacement


# yaw0 (graus), deslocamento no frame odom (dx, dy), lateral esperado (m).
# Em cada yaw o "para a esquerda" do robo aponta para um lado diferente do
# frame odom; a projecao tem de devolver sempre +0.20.
@pytest.mark.parametrize("yaw_deg, dx, dy, expected", [
    (0.0, 0.0, 0.20, 0.20),      # frente = +x odom: esquerda = +y odom
    (90.0, -0.20, 0.0, 0.20),    # frente = +y odom: esquerda = -x odom
    (-90.0, 0.20, 0.0, 0.20),    # frente = -y odom: esquerda = +x odom
    (180.0, 0.0, -0.20, 0.20),   # frente = -x odom: esquerda = -y odom
    (0.0, 0.0, -0.15, -0.15),    # direita = negativo
    (90.0, 0.15, 0.0, -0.15),
])
def test_lateral_displacement_por_yaw(yaw_deg, dx, dy, expected):
    yaw0 = math.radians(yaw_deg)
    x0, y0 = 1.3, -0.7  # origem qualquer: so o delta importa
    assert lat(x0, y0, yaw0, x0 + dx, y0 + dy) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("yaw_deg", [0.0, 90.0, -90.0, 180.0, 37.0, -123.0])
def test_movimento_puro_em_x_nao_conta_como_lateral(yaw_deg):
    """Andar so para a frente/re (eixo x do robo) da lateral 0 e
    longitudinal = o que andou, em qualquer yaw."""
    yaw0 = math.radians(yaw_deg)
    x0, y0 = 0.4, 2.1
    for d in (0.30, -0.12):
        x = x0 + d * math.cos(yaw0)
        y = y0 + d * math.sin(yaw0)
        assert lat(x0, y0, yaw0, x, y) == pytest.approx(0.0, abs=1e-9)
        assert lon(x0, y0, yaw0, x, y) == pytest.approx(d, abs=1e-9)


def test_decomposicao_em_yaw_arbitrario():
    """Deslocamento a*x_hat + b*y_hat (eixos iniciais do robo) tem de
    devolver exatamente (lon=a, lat=b)."""
    yaw0 = math.radians(37.0)
    a, b = 0.07, -0.22
    x_hat = (math.cos(yaw0), math.sin(yaw0))
    y_hat = (-math.sin(yaw0), math.cos(yaw0))
    x0, y0 = -3.0, 5.5
    x = x0 + a * x_hat[0] + b * y_hat[0]
    y = y0 + a * x_hat[1] + b * y_hat[1]
    assert lat(x0, y0, yaw0, x, y) == pytest.approx(b, abs=1e-9)
    assert lon(x0, y0, yaw0, x, y) == pytest.approx(a, abs=1e-9)


def test_sem_movimento_da_zero():
    assert lat(1.0, 2.0, 0.9, 1.0, 2.0) == 0.0
    assert lon(1.0, 2.0, 0.9, 1.0, 2.0) == 0.0


@pytest.mark.parametrize("dy, travelled, slack, ok", [
    (0.20, 0.20, 0.03, True),     # exato
    (0.20, 0.17, 0.03, True),     # dentro da folga (escorregou ~18%: campo)
    (0.20, 0.165, 0.03, False),   # abaixo da folga = curso_incompleto
    (0.20, 0.24, 0.03, True),     # passou do pedido tambem conta
    (-0.20, -0.18, 0.03, True),   # direita: sinal negativo
    (-0.20, -0.16, 0.03, False),
    (-0.20, 0.18, 0.03, False),   # andou para o lado errado
    (0.10, 0.0, 0.03, False),     # nao saiu do lugar
])
def test_regra_de_aceite(dy, travelled, slack, ok):
    assert dan._nudge_accepts(dy, travelled, slack) is ok


# ---------------------------------------------------------------------------
# Revisao 2026-08-28 (achados da revisao adversarial): funcoes puras novas.
front_after = dan._front_after_lateral
corr = dan._nudge_corrected
cut = dan._nudge_cut_reached
valid = dan._nudge_request_valid
Settle = dan._OdomSettle


# Achado 4: NaN/inf no goal.
@pytest.mark.parametrize("dy, timeout, ok", [
    (0.20, 0.0, True),
    (-0.10, 20.0, True),
    (float("nan"), 0.0, False),
    (0.20, float("nan"), False),
    (float("inf"), 0.0, False),
    (0.20, float("inf"), False),
])
def test_pedido_invalido_nan_inf(dy, timeout, ok):
    assert valid(dy, timeout) is ok


def test_nan_passaria_nas_checagens_de_faixa():
    """Motivo da guarda: NaN nao e menor que o minimo nem maior que o
    maximo — sem a validacao o curso comecaria e so acabaria por timeout."""
    dy = float("nan")
    assert not (abs(dy) < 0.08) and not (abs(dy) > 0.25)


# Achado 1: front projetado apos o lateral numa face inclinada.
@pytest.mark.parametrize("front, dy, yaw_deg, expected", [
    (0.38, 0.25, 0.0, 0.38),                                      # perpendicular
    (0.38, 0.25, 5.0, 0.38 - 0.25 * math.sin(math.radians(5))),   # 2,2 cm
    (0.38, -0.25, 5.0, 0.38 - 0.25 * math.sin(math.radians(5))),  # direita igual
    (0.38, 0.25, -5.0, 0.38 - 0.25 * math.sin(math.radians(5))),  # sinal do yaw
    (0.40, 0.10, 30.0, 0.35),                                     # 0,10*sin30 = 0,05
])
def test_front_projetado_apos_lateral(front, dy, yaw_deg, expected):
    assert front_after(front, dy, math.radians(yaw_deg)) == pytest.approx(expected, abs=1e-9)


def test_precheck_face_inclinada_reprova_perto_da_mesa():
    """Pouso a 0,375 m (janela 0,365-0,39): 0,25 m perpendicular passa no
    lateral_min_front 0,36; a 5 graus o bico avancaria 2,2 cm -> reprova.
    Um passo curto (0,10 m) na mesma face ainda passa."""
    lateral_min = 0.36
    assert front_after(0.375, 0.25, 0.0) >= lateral_min
    assert front_after(0.375, 0.25, math.radians(5.0)) < lateral_min
    assert front_after(0.375, 0.10, math.radians(5.0)) >= lateral_min


# Achado 2: ganho de escorregamento lateral.
@pytest.mark.parametrize("raw, gain, expected", [
    (0.20, 1.0, 0.20),
    (0.20, 0.82, 0.164),
    (-0.20, 0.82, -0.164),
    (0.0, 0.82, 0.0),
])
def test_ganho_de_escorregamento(raw, gain, expected):
    assert corr(raw, gain) == pytest.approx(expected, abs=1e-9)


def test_ganho_entra_no_aceite():
    """Odom bruta 0,20 com ganho 0,82 = 0,164 real: abaixo da folga
    (0,17) -> curso_incompleto; com 1.0 o mesmo bruto seria aceito."""
    assert dan._nudge_accepts(0.20, corr(0.20, 1.0), 0.03) is True
    assert dan._nudge_accepts(0.20, corr(0.20, 0.82), 0.03) is False
    assert dan._nudge_accepts(0.20, corr(0.21, 0.82), 0.03) is True   # 0,172


# Achado 5: corte descontando a idade da odometria (lag = v*(idade+periodo)).
COAST, MIN_V = 0.03, 0.1215


@pytest.mark.parametrize("mag, travelled, sign, age, period, ok", [
    (0.20, 0.17, 1.0, 0.0, 0.0, True),      # falta exatamente o coast
    (0.20, 0.16, 1.0, 0.0, 0.0, False),     # falta 4 cm, sem atraso: segue
    (0.20, 0.16, 1.0, 0.2, 0.05, True),     # mesmo 4 cm, mas anda 3 cm ate medir
    (0.20, 0.13, 1.0, 0.2, 0.05, False),    # 7 cm > 3 + 3,04
    (0.20, -0.17, -1.0, 0.0, 0.0, True),    # direita
    (0.20, 0.17, -1.0, 0.0, 0.0, False),    # lado errado
    (0.20, 0.25, 1.0, 0.0, 0.0, True),      # passou: corta
    (0.10, 0.0, 1.0, 0.0, 0.05, False),     # inicio do curso
])
def test_criterio_de_corte_com_idade(mag, travelled, sign, age, period, ok):
    assert cut(mag, travelled, sign, COAST, MIN_V, age, period) is ok


def test_corte_lag_igual_ao_da_reta_do_align():
    """A conta e a mesma da reta (front - v*(idade+periodo) <= stop + coast)."""
    age, period = 0.15, 0.05
    lag = MIN_V * (age + period)
    remaining = COAST + lag
    assert cut(0.20, 0.20 - remaining + 1e-9, 1.0, COAST, MIN_V, age, period) is True
    assert cut(0.20, 0.20 - remaining - 1e-6, 1.0, COAST, MIN_V, age, period) is False


# Achado 3: assentamento por odometria.
EPS, MIN_S, DT = 0.003, 0.8, 0.02  # odom a 50 Hz


def test_assentamento_nao_confunde_rastejo_com_parado():
    """No piso (0,1215 m/s) a 50 Hz o lateral muda 2,4 mm por amostra —
    MENOS que eps entre consecutivas. A ancora da janela pega: nunca
    assenta enquanto anda."""
    s = Settle(EPS, MIN_S)
    t, lat = 0.0, 0.0
    for _ in range(150):  # 3 s andando
        t += DT
        lat += 0.1215 * DT
        assert s.update(t, lat) is False


def test_assentamento_espera_min_s_depois_de_parar():
    s = Settle(EPS, MIN_S)
    t, lat = 0.0, 0.0
    for _ in range(50):
        t += DT
        lat += 0.1215 * DT
        s.update(t, lat)
    t_stop = t  # ultima amostra em movimento
    settled_at = None
    for _ in range(100):
        t += DT
        if s.update(t, lat):
            settled_at = t
            break
    assert settled_at is not None
    # A janela fecha min_s depois da ancora; a ancora pode ser ate 2 amostras
    # ANTES da parada (elas ja estao a < eps da posicao final — e isso que
    # "parado" quer dizer), nunca depois da 1a amostra parada.
    since_stop = settled_at - t_stop
    assert MIN_S - 2 * DT - 1e-9 <= since_stop <= MIN_S + DT + 1e-9


def test_assentamento_reinicia_no_salto():
    s = Settle(EPS, MIN_S)
    t = 0.0
    for _ in range(30):  # 0,6 s parado
        t += DT
        assert s.update(t, 0.10) is False
    t += DT
    assert s.update(t, 0.11) is False  # salto de 1 cm: janela reaberta aqui
    t_open = t
    settled_at = None
    for _ in range(100):
        t += DT
        if s.update(t, 0.11):
            settled_at = t
            break
    assert settled_at is not None
    assert settled_at - t_open == pytest.approx(MIN_S, abs=DT + 1e-9)


def test_assentamento_tolera_ruido_abaixo_de_eps():
    s = Settle(EPS, MIN_S)
    t = 0.0
    settled = False
    for i in range(80):
        t += DT
        settled = s.update(t, 0.10 + (0.001 if i % 2 else -0.001)) or settled
    assert settled


def test_assentamento_sem_amostra_nova_nao_assenta():
    """So a 1a amostra: nao ha janela fechada (o no cai no teto)."""
    s = Settle(EPS, MIN_S)
    assert s.update(0.0, 0.10) is False
