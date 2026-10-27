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


# Achado 5: corte descontando a idade da odometria (lag = v_cmd*(idade+periodo));
# v_cmd e' a velocidade COMANDADA no curso (piso x margem de partida).
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
    assert cut(mag, travelled, sign, COAST, v_cmd=MIN_V, odom_age=age, period=period) is ok


def test_corte_lag_igual_ao_da_reta_do_align():
    """A conta e a mesma da reta (front - v*(idade+periodo) <= stop + coast)."""
    age, period = 0.15, 0.05
    lag = MIN_V * (age + period)
    remaining = COAST + lag
    assert cut(0.20, 0.20 - remaining + 1e-9, 1.0, COAST, v_cmd=MIN_V, odom_age=age, period=period) is True
    assert cut(0.20, 0.20 - remaining - 1e-6, 1.0, COAST, v_cmd=MIN_V, odom_age=age, period=period) is False


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


# ---------------------------------------------------------------------------
# Revisao 2026-10-19: "o robo fala que vai se mover mas nao se move".
# O piso de corpo convertido em roda pelo mecanum_drive_controller da Pi
# (wheels_radius 0.05, projecao 0.3855) fica UM ULP abaixo do piso do ESC
# (min_wheel_rad_per_sec = 2.43) — e o driver so' LIGA com |cmd| >= piso.
start = dan._floor_start_speed
no_progress = dan._nudge_no_progress

WHEEL_R = 0.05        # caramelo_controllers.yaml: wheels_radius
PROJ = 0.3855         # sum_of_robot_center_projection_on_X_Y_axis
ESC_FLOOR = 2.43      # maxon_motors_node: min_wheel_rad_per_sec
MIN_W, MARGIN = 0.315, 0.07   # MIN_V = 0.1215 ja' definido acima


def test_piso_exato_fica_abaixo_do_esc():
    """O caso que falhava no campo: 0.1215/0.05 = 2.4299999999999997 < 2.43
    em float; o giro idem (0.315*0.3855/0.05 = 2.42865)."""
    assert MIN_V / WHEEL_R < ESC_FLOOR
    assert MIN_W * PROJ / WHEEL_R < ESC_FLOOR


def test_partida_com_margem_passa_do_piso():
    assert start(MIN_V, MARGIN) / WHEEL_R >= ESC_FLOOR
    assert start(MIN_W, MARGIN) * PROJ / WHEEL_R >= ESC_FLOOR


@pytest.mark.parametrize("floor, margin, expected", [
    (0.1215, 0.07, 0.1215 * 1.07),
    (0.315, 0.07, 0.315 * 1.07),
    (0.1215, 0.0, 0.1215),     # sem margem = piso exato (como antes)
    (0.0, 0.07, 0.0),
])
def test_partida_formula(floor, margin, expected):
    assert start(floor, margin) == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize("margin", [-0.05, -1.0, float("nan"), float("-inf"), float("inf")])
def test_partida_margem_invalida_vira_zero(margin):
    """Margem negativa/NaN/inf no launch reproduziria o bug de campo em
    silencio (partida ABAIXO do piso ou NaN no comando): o helper trata
    como 0 (piso exato) e o no' avisa uma vez."""
    assert start(MIN_V, margin) == pytest.approx(MIN_V, abs=1e-12)
    assert start(MIN_W, margin) == pytest.approx(MIN_W, abs=1e-12)


def test_partida_no_patamar_do_retreat_antigo():
    """O _retreat usava 0.13 fixo (2.6 rad/s na roda) e por isso andava;
    a margem default poe as outras primitivas no mesmo patamar."""
    assert start(MIN_V, MARGIN) == pytest.approx(0.13, abs=1e-3)
    assert 0.13 / WHEEL_R >= ESC_FLOOR


@pytest.mark.parametrize("elapsed, travelled, ok", [
    (0.0, 0.0, False),        # acabou de comandar
    (0.99, 0.0, False),       # antes do prazo
    (1.0, 0.0, True),         # prazo exato, parado
    (1.0, 0.009, True),       # abaixo do minimo
    (1.0, -0.009, True),      # modulo
    (1.0, 0.01, False),       # minimo exato = andou
    (1.0, -0.02, False),      # andou (lado errado, mas andou)
    (5.3, 0.0, True),         # o caso do log de 2026-10-19
    (2.0, 0.05, False),
])
def test_watchdog_sem_progresso(elapsed, travelled, ok):
    assert no_progress(elapsed, travelled, 1.0, 0.01) is ok


# ---------------------------------------------------------------------------
# Revisao 2026-10-21: docking "pronto" com yaw_face -19,7 graus na WS1.
# Helpers puros das decisoes de yaw do align_to_dock (fora da classe).
needs = dan.yaw_needs_correction
refit = dan.refit_trigger
fit_ok = dan.face_fit_acceptable
fit_why = dan.face_fit_reject_reason
decide = dan.final_yaw_decision

YAW_TOL = 0.09          # default_yaw_tolerance (5,2 graus)
YAW_COAST = 0.10        # yaw_coast_est: abaixo de coast+0,02 o giro nem parte
MIN_CORR = YAW_COAST + 0.02


@pytest.mark.parametrize("yaw_deg, tol, ok", [
    (-19.7, YAW_TOL, True),      # o caso da WS1
    (2.6, YAW_TOL, False),       # WS5 na mesma run
    (5.0, YAW_TOL, False),       # 0,087 rad < 0,09
    (5.3, YAW_TOL, True),        # 0,0925 rad > 0,09
    (-5.3, YAW_TOL, True),       # modulo
    (0.0, YAW_TOL, False),
])
def test_yaw_precisa_correcao(yaw_deg, tol, ok):
    assert needs(math.radians(yaw_deg), tol) is ok


@pytest.mark.parametrize("yaw", [None, float("nan"), float("inf"), float("-inf")])
def test_yaw_sem_medida_nao_corrige(yaw):
    """Sem fit confiavel nao ha o que corrigir: o no' reporta
    'yaw_nao_medido' em vez de girar pelo AMCL."""
    assert needs(yaw, YAW_TOL) is False


# Gatilho do refit no reto: desconta coast (0,10 no launch) e lag, como o
# corte final, para o robo PARAR em ~refit_front_m (acima de rotate_min 0,46).
COAST_X, LAG = 0.10, 0.1215 * 0.15


@pytest.mark.parametrize("front, done, ok", [
    (0.70, False, False),         # 0,70 - 0,018 - 0,10 = 0,582 > 0,55: segue
    (0.678, False, False),        # staging da WS1 (lido 0,678): 0,560 > 0,55
    (0.67, False, False),         # 0,552 > 0,55
    (0.66, False, True),          # 0,66 - 0,018 - 0,10 = 0,542 <= 0,55: pausa
    (0.65, False, True),
    (0.50, False, True),          # ja' perto: pausa mesmo assim (nao gira < 0,46)
    (0.65, True, False),          # uma vez so' por aproximacao
    (None, False, False),         # sem medicao do muro
])
def test_gatilho_do_refit(front, done, ok):
    assert refit(front, 0.55, done, coast=COAST_X, lag=LAG) is ok


def test_gatilho_do_refit_para_acima_de_rotate_min():
    """Com coast 0,10 o robo para ~coast depois do gatilho: pausa lida em
    0,55 (sem descontar) pararia em 0,45 < rotate_min_front 0,46 e o
    esquadro nunca aconteceria; descontando, o gatilho dispara em 0,65 e o
    robo para em ~0,55."""
    first_trigger = next(f / 1000 for f in range(900, 300, -1)
                         if refit(f / 1000, 0.55, False, coast=COAST_X, lag=0.0))
    assert first_trigger == pytest.approx(0.65, abs=1e-3)
    assert first_trigger - COAST_X >= 0.46
    # Sem descontar (assinatura de 3 argumentos) e' o front cru.
    assert refit(0.55, 0.55, False) is True
    assert refit(0.551, 0.55, False) is False


@pytest.mark.parametrize("refit_front", [0.0, -1.0])
def test_gatilho_do_refit_desligado(refit_front):
    assert refit(0.50, refit_front, False, coast=COAST_X, lag=LAG) is False


# Guardas do fit: largura maxima PRIMEIRO (mesa + vizinho), depois residuo,
# depois largura minima. Valores da run: WS1 0,98 m / 61 mm; WS5 0,87 / 43;
# fit bom 0,64 / 13 mm.
FIT_MIN_W, FIT_MAX_W, FIT_MAX_RES = 0.50, 0.90, 0.02   # refine_min/max_face_width, refine_max_residual


@pytest.mark.parametrize("residual, width, ok, why", [
    (0.013, 0.64, True, ""),                                   # fit bom (WS1 apos giro)
    (0.061, 0.98, False, "face larga demais (vizinho?)"),      # WS1 staging
    (0.010, 0.95, False, "face larga demais (vizinho?)"),      # largo mesmo com residuo bom
    (0.043, 0.87, False, "residuo alto (face nao-plana?)"),    # WS5 staging (< 0,90)
    (0.010, 0.39, False, "face estreita demais (perna/objeto?)"),  # caso de 2026-08-03
    (0.020, 0.50, True, ""),                                   # bordas inclusivas
    (0.020, 0.90, True, ""),
    (0.0201, 0.90, False, "residuo alto (face nao-plana?)"),
])
def test_guardas_do_fit(residual, width, ok, why):
    assert fit_ok(residual, width, FIT_MIN_W, FIT_MAX_W, FIT_MAX_RES) is ok
    assert fit_why(residual, width, FIT_MIN_W, FIT_MAX_W, FIT_MAX_RES) == why


def test_guarda_largura_default_do_residuo():
    """max_residual default = refine_max_residual (0,02)."""
    assert fit_ok(0.02, 0.64, FIT_MIN_W, FIT_MAX_W) is True
    assert fit_ok(0.021, 0.64, FIT_MIN_W, FIT_MAX_W) is False


# Decisao final.
@pytest.mark.parametrize("yaw_deg, corrected, strict, expected", [
    (2.6, False, False, "ok"),                       # WS5: dentro
    (-19.7, False, False, "corrigir"),               # WS1: 1a vez -> recua e esquadra
    (-19.7, False, True, "corrigir"),                # strict nao muda a 1a decisao
    (-19.7, True, False, "aceitar_com_aviso"),       # ja' corrigiu: competicao segue
    (-19.7, True, True, "falhar"),                   # strict: aborta
    (-6.0, False, False, "corrigir"),                # 0,105 rad > 0,09 (sem min_correctable)
    (0.0, True, True, "ok"),
])
def test_decisao_final(yaw_deg, corrected, strict, expected):
    # min_correctable=0 (default): qualquer yaw fora da tolerancia e' corrigivel.
    assert decide(math.radians(yaw_deg), YAW_TOL, corrected, strict) == expected


@pytest.mark.parametrize("yaw_deg, strict, expected", [
    (-6.0, False, "aceitar_com_aviso"),   # 0,105 rad: fora da tol (0,09) mas <= coast+0,02 (0,12)
    (-6.0, True, "falhar"),
    (-7.0, False, "corrigir"),            # 0,122 rad > 0,12: o giro parte
    (-19.7, False, "corrigir"),
    (5.0, True, "ok"),                    # dentro da tol: strict irrelevante
])
def test_decisao_final_abaixo_do_coast_nao_recua(yaw_deg, strict, expected):
    """Entre a tolerancia e o coast do giro o _cut_drive nem parte: recuar
    seria tempo perdido — aceita com aviso (ou falha com strict)."""
    assert decide(math.radians(yaw_deg), YAW_TOL, False, strict,
                  min_correctable=MIN_CORR) == expected


def test_decisao_final_sequencia_da_ws1():
    """A run: pronto com -19,7 graus. Agora: corrigir; se depois da correcao
    ainda medir -8 graus, aceita com aviso (strict off) — nunca 'ok'."""
    first = decide(math.radians(-19.7), YAW_TOL, False, False, min_correctable=MIN_CORR)
    assert first == "corrigir"
    second = decide(math.radians(-8.0), YAW_TOL, True, False, min_correctable=MIN_CORR)
    assert second == "aceitar_com_aviso"
    assert decide(math.radians(-2.0), YAW_TOL, True, False, min_correctable=MIN_CORR) == "ok"


# ---------------------------------------------------------------------------
# Runs de 22-23/10 (docs/investigacao_incidente_2026-10.md): o que o
# dock_align decidiu no fim de cada docking, reproduzido nas funcoes puras.
# ---------------------------------------------------------------------------

def test_decisao_final_sequencia_da_ws6_2026_10_23():
    """WS6 23/10 19:08: +7,6 graus -> corrigir (recuo + esquadro); depois da
    correcao mediu -12,7 graus -> aceitar_com_aviso (strict desligado): o
    goal SUCEDE com 'yaw_fora_da_tolerancia' e so' o gate do GoToWS (limite
    duro 0,20 rad = 11,5 graus) segura a manipulacao."""
    first = decide(math.radians(7.6), YAW_TOL, False, False, min_correctable=MIN_CORR)
    assert first == "corrigir"
    second = decide(math.radians(-12.7), YAW_TOL, True, False, min_correctable=MIN_CORR)
    assert second == "aceitar_com_aviso"
    assert abs(math.radians(-12.7)) > 0.20          # estoura o limite duro do GoToWS
    assert abs(math.radians(8.7)) <= 0.20           # WS6 18:49: dentro do limite duro
    assert decide(math.radians(8.7), YAW_TOL, True, True, min_correctable=MIN_CORR) == "falhar"


def test_yaw_nao_medido_nao_gira_pelo_amcl():
    """WS3 23/10 18:51 e PP1 22/10 15:42: 0/5 e 1/5 fits da face no fim ->
    face_yaw None -> nada a corrigir aqui (o AMCL, +18,3 / +15,5 graus, vai
    so' para o result); quem decide e' o gate do GoToWS."""
    assert needs(None, YAW_TOL) is False
    assert needs(float("nan"), YAW_TOL) is False
    for amcl_deg in (18.3, 15.5):
        assert abs(math.radians(amcl_deg)) > 0.20


# ---------------------------------------------------------------------------
# Revisao 2026-10-26: tours 19:45/19:52 (WS1/WS2/WS3/WS6 tortas e fora do
# centro; docs/diagnostico_docking_tour_2026-10-26.md). Helpers puros novos
# do align_to_dock e os casos do plano (secao 11: I1, I2, I3c, I4, I8, R3, R6).
# ---------------------------------------------------------------------------
import re        # noqa: E402
import tempfile  # noqa: E402
from pathlib import Path  # noqa: E402

merge = dan.merge_refine_results
veto = dan.amcl_square_veto
tight = dan.median_if_tight
yaw_stable = dan.yaw_readings_stable
Still = dan._StillWindow
lat_cut = dan.lateral_cut_reached
refit_decide = dan.refit_square_decision
parse_dock = dan.parse_dock_entry
center_pts = dan.face_center_from_points
FaceFit = dan._FaceFit

AGREE_TOL = 0.035       # refine_yaw_agree_tol (2,0 graus)
AMCL_MAX = dan._AMCL_SQUARE_MAX_ERR_DEFAULT   # amcl_square_max_err (0,20)
BANK = (1.0, 2.0, 0.5)
REFINED_1 = (1.02, 2.01, 0.37)   # 1a leitura confiante (WS1-T2: -7,6 graus aplicados)
REFINED_2 = (1.03, 2.00, 0.36)


# I2 — sequencia (ok, rejeitado) mantem confident e o alvo refinado.
def test_i2_segunda_leitura_rejeitada_mantem_o_alvo_da_primeira():
    """WS1-T2 (F13): a 1a chamada do refino aplicou -7,6 graus e a 2a
    (residuo 21 mm) sobrescrevia confident=False, jogando fora o alvo e
    disparando o esquadro por AMCL contra ele."""
    target, ok = merge(BANK, (REFINED_1, True), (REFINED_1, False))
    assert ok is True
    assert target == REFINED_1


def test_i2_duas_confiantes_usa_a_ultima():
    target, ok = merge(BANK, (REFINED_1, True), (REFINED_2, True))
    assert ok is True and target == REFINED_2


def test_i2_so_a_segunda_confiante():
    target, ok = merge(BANK, (BANK, False), (REFINED_2, True))
    assert ok is True and target == REFINED_2


def test_i2_nenhuma_confiante_volta_ao_banco():
    """So' aqui o WARN 'usando pose do banco' e' verdadeiro."""
    target, ok = merge(BANK, (BANK, False), (BANK, False))
    assert ok is False and target == BANK


# I1 — veto do esquadro por AMCL.
def test_i1_ws6_amcl_cw_contra_lidar_ccw_e_pulado_por_sinal():
    """WS6 nos dois tours: AMCL pedia -9,7 graus (CW) e os fits do LiDAR
    (rejeitados por residuo) diziam +5/+6 (CCW). Duas leituras concordantes
    com o sinal oposto ao do AMCL = nao gira (e o motivo logado e' o sinal,
    nao o limite)."""
    why = veto(math.radians(-9.7), [math.radians(6.3), math.radians(5.9)],
               AGREE_TOL, YAW_TOL, AMCL_MAX)
    assert "sentido oposto" in why
    # Leituras +5,0/+4,2 (media 4,6 < 5,2): veto por 'ja' dentro da tolerancia'.
    assert "dentro da tolerancia" in veto(
        math.radians(-9.7), [math.radians(5.0), math.radians(4.2)], AGREE_TOL, YAW_TOL, AMCL_MAX)


def test_i1_limite_de_giro_pelo_amcl():
    """|eyaw0| = 0,25 rad > 0,20: pulado por limite, mesmo sem LiDAR; 0,15
    (dentro da janela) e o proprio limite passam."""
    why = veto(0.25, [], AGREE_TOL, YAW_TOL, AMCL_MAX)
    assert "limite" in why
    assert veto(-0.25, None, AGREE_TOL, YAW_TOL, AMCL_MAX) != ""
    assert veto(0.15, [], AGREE_TOL, YAW_TOL, AMCL_MAX) == ""
    assert veto(AMCL_MAX, [], AGREE_TOL, YAW_TOL, AMCL_MAX) == ""   # no limite: passa


def test_i1_janela_de_giro_do_amcl_nao_e_vazia():
    """Revisao 2026-10-26: com amcl_square_max_err 0,12 e yaw_coast_est 0,10
    o giro exigia |e| > 0,12 (yaw_min_corr) e o veto descartava |e| > 0,12 —
    o caminho ligado era codigo morto. Com os defaults atuais (no' e launch)
    existe erro em (yaw_min_corr, max_err] sem veto; a janela fechada e'
    detectada por amcl_square_window (WARN no init)."""
    node_src = open(SCRIPT, encoding="utf-8").read()
    launch = os.path.join(HERE, "..", "launch", "docking_server.launch.py")
    launch_src = open(launch, encoding="utf-8").read()
    m = re.search(r'\{"yaw_coast_est":\s*([0-9.]+)\}', launch_src)
    assert m, "launch sem yaw_coast_est"
    yaw_coast_launch = float(m.group(1))
    assert re.search(r'declare_parameter\("yaw_coast_est",\s*%s\)' % re.escape("%.2f" % yaw_coast_launch),
                     node_src)
    assert re.search(r'declare_parameter\("amcl_square_max_err",\s*_AMCL_SQUARE_MAX_ERR_DEFAULT\)',
                     node_src)
    ymin, aberta = dan.amcl_square_window(yaw_coast_launch, AMCL_MAX)
    assert ymin == pytest.approx(MIN_CORR) and aberta
    e = 0.5 * (ymin + AMCL_MAX)              # meio da janela
    assert ymin < e <= AMCL_MAX
    assert veto(e, [], AGREE_TOL, YAW_TOL, AMCL_MAX) == ""
    assert veto(-e, [math.radians(-7.0), math.radians(-6.5)], AGREE_TOL, YAW_TOL, AMCL_MAX) == ""
    # O bug original: 0,12 com yaw_coast 0,10 => fechada.
    assert dan.amcl_square_window(0.10, 0.12) == (pytest.approx(0.12), False)
    assert dan.amcl_square_window(0.10, 0.121)[1] is True


def test_i1_lidar_dentro_da_tolerancia_nao_gira():
    """LiDAR ve a face a +2/+3 graus (dentro de 5,2): o robo ja' esta'
    esquadrado pela face; o AMCL pedindo +8 nao manda."""
    why = veto(math.radians(8.0), [math.radians(2.0), math.radians(3.0)],
               AGREE_TOL, YAW_TOL, AMCL_MAX)
    assert "dentro da tolerancia" in why


def test_i1_lidar_concordando_no_sentido_deixa_girar():
    """-6 graus (0,105 rad <= 0,12) com o LiDAR no mesmo sentido: pode."""
    assert veto(math.radians(-6.0), [math.radians(-7.0), math.radians(-6.5)],
                AGREE_TOL, YAW_TOL, AMCL_MAX) == ""


def test_i1_leituras_discordantes_nao_vetam():
    """WS6-T1 real: +5,0 e +2,8 (2,2 graus > agree_tol): sem veto pelo
    sinal — quem segura esse caso e' a guarda 'so' sem face nenhuma' do
    chamador (o refino ACHOU a face e rejeitou por residuo), nao o veto:
    -9,7 graus (0,169 rad) cabe no limite de 0,20. Acima do limite (-13
    graus) o veto segura mesmo com leituras discordantes."""
    assert veto(math.radians(-6.0), [math.radians(5.0), math.radians(2.8)],
                AGREE_TOL, YAW_TOL, AMCL_MAX) == ""
    assert veto(math.radians(-6.0), [math.radians(5.0)], AGREE_TOL, YAW_TOL, AMCL_MAX) == ""
    assert veto(math.radians(-9.7), [math.radians(5.0), math.radians(2.8)],
                AGREE_TOL, YAW_TOL, AMCL_MAX) == ""
    assert "limite" in veto(math.radians(-13.0), [math.radians(5.0), math.radians(2.8)],
                            AGREE_TOL, YAW_TOL, AMCL_MAX)


def test_i1_sem_amcl_nao_gira():
    assert veto(None, [], AGREE_TOL, YAW_TOL, AMCL_MAX) != ""
    assert veto(float("nan"), [], AGREE_TOL, YAW_TOL, AMCL_MAX) != ""


def test_i1_default_desligado_no_no_e_no_launch():
    """TESTE DE FONTE (regex, nao de comportamento): amcl_square_enabled
    nasce False no no' e e' passado False no launch. Guarda minima contra
    alguem religar o esquadro por AMCL sem passar pelo diagnostico."""
    node_src = open(SCRIPT, encoding="utf-8").read()
    assert re.search(r'declare_parameter\("amcl_square_enabled",\s*False\)', node_src)
    launch = os.path.join(HERE, "..", "launch", "docking_server.launch.py")
    launch_src = open(launch, encoding="utf-8").read()
    assert re.search(r'\{"amcl_square_enabled":\s*False\}', launch_src)


# I1c — mediana so' com >= 5 amostras estaveis.
def test_i1c_mediana_so_com_amostras_estaveis():
    assert tight([0.10, 0.11, 0.10, 0.12, 0.11], 5, 0.02) == pytest.approx(0.11)
    assert tight([0.10, 0.11, 0.10, 0.12], 5, 0.02) is None            # poucas
    assert tight([0.05, 0.08, 0.10, 0.12, 0.14], 5, 0.02) is None      # mudando (F14)
    assert tight([0.1, None, 0.1, float("nan"), 0.1, 0.1, 0.1], 5, 0.02) == pytest.approx(0.1)


# I3c — base parada por odometria e yaw da face estavel.
def test_i3c_janela_de_parada_fecha_so_depois_de_min_s():
    s = Still(0.02, 0.05, 0.5)
    t = 0.0
    # Nav2 declarou chegada, base ainda girando 0,25 rad/s por 0,4 s (F14).
    for _ in range(20):
        t += 0.02
        assert s.update(t, 0.0, 0.25) is False
    t_stop = t
    settled = None
    for _ in range(100):
        t += 0.02
        if s.update(t, 0.0, 0.0):
            settled = t
            break
    assert settled is not None
    assert settled - t_stop == pytest.approx(0.5, abs=0.021)


def test_i3c_amostra_em_movimento_reabre_a_janela():
    s = Still(0.02, 0.05, 0.5)
    t = 0.0
    for _ in range(20):
        t += 0.02
        s.update(t, 0.0, 0.0)
    t += 0.02
    assert s.update(t, 0.03, 0.0) is False    # 3 cm/s > 2 cm/s
    t_open = t
    for _ in range(100):
        t += 0.02
        if s.update(t, 0.0, 0.0):
            break
    assert t - t_open == pytest.approx(0.5, abs=0.021)


def test_i3c_simulador_de_odom_que_para_em_0_8s():
    """Plano I3: odom que para em 0,8 s => a janela de 0,5 s fecha em ~1,3 s."""
    s = Still(0.02, 0.05, 0.5)
    t, closed = 0.0, None
    while t < 3.0 and closed is None:
        t += 0.02
        wz = 0.3 if t < 0.8 else 0.0
        if s.update(t, 0.0, wz):
            closed = t
    assert closed == pytest.approx(1.3, abs=0.03)


def test_i3c_yaw_da_face_estavel():
    eps = math.radians(0.5)
    deriva = [math.radians(d) for d in (-4.6, -7.6, -9.7)]     # WS1-T2 em 0,33 s
    assert yaw_stable(deriva, 3, eps) is False
    parado = [math.radians(d) for d in (-9.7, -9.9, -9.6)]
    assert yaw_stable(parado, 3, eps) is True
    assert yaw_stable(parado[:2], 3, eps) is False               # poucas leituras
    assert yaw_stable(deriva + parado, 3, eps) is True           # so' as ultimas n
    # Revisao 2026-10-26: still_yaw_eps default 0,035 rad (2 graus) — o fit
    # leve oscila +-4 graus entre scans (final_yaw_spread_max) e 0,5 grau era
    # inatingivel; com /odom o criterio e' so' advisorio. A deriva da WS1-T2
    # (5,1 graus em 3 scans) continua reprovando com 2 graus; ruido tipico
    # de 1,5 grau passa.
    eps2 = 0.035
    assert yaw_stable(deriva, 3, eps2) is False
    assert yaw_stable([math.radians(d) for d in (-9.7, -11.2, -10.0)], 3, eps2) is True
    node_src = open(SCRIPT, encoding="utf-8").read()
    assert re.search(r'declare_parameter\("still_yaw_eps",\s*0\.035\)', node_src)


# I4 — centro pelo pe' da perpendicular (scan sintetico).
HY, EDGE, MAX_RES, WMIN, WMAX = 0.65, 0.08, 0.012, 0.48, 0.95   # center_* do no'


def _face_points(dist, yaw_deg, half_width=0.40, offset_along=0.0, step=0.004):
    """Face plana a `dist` m do centro do robo (pe' da perpendicular), normal
    girada yaw_deg no frame do robo, centro da face deslocado offset_along
    ao longo da reta (+ = esquerda). Devolve (pontos em base_frame, front
    do muro = menor x no corredor |y| <= 0,21)."""
    th = math.radians(yaw_deg)
    nx, ny = math.cos(th), math.sin(th)
    tx, ty = -ny, nx
    pts = []
    k = -half_width
    while k <= half_width + 1e-9:
        s = k + offset_along
        pts.append((dist * nx + s * tx, dist * ny + s * ty))
        k += step
    corridor = [p[0] for p in pts if abs(p[1]) <= 0.21]
    return pts, min(corridor)


@pytest.mark.parametrize("yaw_deg", [7.0, -7.0])
def test_i4_robo_girado_centrado_no_pe_da_perpendicular(yaw_deg):
    """Face a 0,65 m, robo girado +-7 graus e fisicamente centrado no pe'
    da perpendicular: c_foot ~ 0 e c_axis ~ +-0,65*tan(7) = +-7,98 cm —
    o que o lateral(staging) demandava a mais antes do esquadro (F15: WS3
    +16/+17,5 cm; previsao d*tan(yaw) bateu com o AMCL em 3 casos). Face
    de 0,70 m: a banda funda [front-0,03, front+0,25] corta ~3 cm da ponta
    mais proxima de uma face de 0,80 a 7 graus (propriedade do estimador,
    nao o que se testa aqui)."""
    pts, front = _face_points(0.65, yaw_deg, half_width=0.35)
    c_axis, c_foot, width, why = center_pts(pts, front, HY, EDGE, MAX_RES, WMIN, WMAX)
    assert why == ""
    assert width == pytest.approx(0.70, abs=0.01)
    assert c_foot == pytest.approx(0.0, abs=0.003)
    expected = 0.65 * math.tan(math.radians(yaw_deg))
    assert abs(expected) == pytest.approx(0.0798, abs=0.001)
    assert c_axis == pytest.approx(expected, abs=0.003)
    # Identidade c_foot = c_axis - d*tan(yaw).
    assert c_foot == pytest.approx(c_axis - 0.65 * math.tan(math.radians(yaw_deg)), abs=0.003)


def test_i4_sem_yaw_os_dois_coincidem():
    pts, front = _face_points(0.65, 0.0, offset_along=0.06)
    c_axis, c_foot, width, why = center_pts(pts, front, HY, EDGE, MAX_RES, WMIN, WMAX)
    assert why == ""
    assert c_axis == pytest.approx(c_foot, abs=1e-6)
    assert c_axis == pytest.approx(0.06, abs=0.003)     # centro 6 cm a esquerda


def test_i4_deslocamento_real_aparece_nos_dois():
    """Robo girado +7 e o centro da face 5 cm a DIREITA do pe': c_foot
    = -5 cm (o que precisa andar depois de esquadrar), c_axis = -5 + 8."""
    pts, front = _face_points(0.65, 7.0, half_width=0.35, offset_along=-0.05)
    c_axis, c_foot, _w, why = center_pts(pts, front, HY, EDGE, MAX_RES, WMIN, WMAX)
    assert why == ""
    assert c_foot == pytest.approx(-0.05, abs=0.003)
    assert c_axis == pytest.approx(-0.05 + 0.0798, abs=0.003)


def test_i4_guardas_continuam_valendo():
    pts, front = _face_points(0.65, 0.0, half_width=0.15)     # 0,30 m: perna/objeto
    assert center_pts(pts, front, HY, EDGE, MAX_RES, WMIN, WMAX)[3].startswith("largura")
    assert center_pts([], 0.65, HY, EDGE, MAX_RES, WMIN, WMAX)[3].startswith("fit falhou")


# R6 — corte do lateral pelo ponto de parada previsto.
def test_r6_corte_previsto_com_lag():
    """A 0,13 m/s, scan com 0,1 s de idade e periodo 0,05 s, coast 0,03: o
    corte ocorre com erro ~0,03 + 0,13*0,15 = 0,0495 m (antes: so' em 0,03,
    e o ESC retinha a referencia ~0,5 s = 5-10 cm de sobrecurso)."""
    v, age, period, coast = 0.13, 0.1, 0.05, 0.03
    limit = coast + v * (age + period)
    assert limit == pytest.approx(0.0495, abs=1e-9)
    assert lat_cut(0.049, 1.0, coast, v, age, period) is True
    assert lat_cut(0.051, 1.0, coast, v, age, period) is False
    assert lat_cut(-0.049, -1.0, coast, v, age, period) is True    # direita
    assert lat_cut(-0.049, 1.0, coast, v, age, period) is True     # passou do centro: corta
    assert lat_cut(0.14, 1.0, coast, v, age, period) is False      # WS3: demanda +14,1, longe


def test_r6_sem_lag_e_o_corte_antigo():
    assert lat_cut(0.03, 1.0, 0.03, 0.0, 0.1, 0.05) is True
    assert lat_cut(0.031, 1.0, 0.03, 0.0, 0.1, 0.05) is False


def test_r6_mesma_conta_do_nudge():
    """lateral_cut_reached(err) == _nudge_cut_reached(mag, travelled) com
    err = mag - travelled*sign."""
    for travelled in (0.10, 0.15, 0.17, 0.19):
        err = 0.20 - travelled
        assert lat_cut(err, 1.0, COAST, MIN_V, 0.1, 0.05) is cut(
            0.20, travelled, 1.0, COAST, v_cmd=MIN_V, odom_age=0.1, period=0.05)


# I8 — refit_square devolve se girou.
def _fit(ok, yaw_deg, reason="", width=0.64, residual=0.005):
    return FaceFit(ok, reason, 0.64, math.radians(yaw_deg), width, residual, 120)


def test_i8_refit_sem_fits_nao_gira():
    """F12: 5/6 correcoes finais tiveram o refit rejeitado e o ciclo so'
    recuou e voltou; corrected_once nao pode ser marcado."""
    yaw, status, why = refit_decide(
        _fit(False, 10.8, "face estreita demais (perna/objeto?)"), None,
        AGREE_TOL, YAW_TOL, MIN_CORR, 0.64, 0.46)
    assert yaw is None and status == "sem_fits" and "estreita" in why
    yaw, status, why = refit_decide(_fit(True, 10.8), None, AGREE_TOL, YAW_TOL, MIN_CORR, 0.64, 0.46)
    assert status == "sem_fits" and "sem /scan novo" in why
    yaw, status, _ = refit_decide(_fit(True, 10.8), _fit(False, 10.0, "residuo alto"),
                                  AGREE_TOL, YAW_TOL, MIN_CORR, 0.64, 0.46)
    assert status == "sem_fits"
    assert refit_decide(None, None, AGREE_TOL, YAW_TOL, MIN_CORR, 0.64, 0.46)[1] == "sem_fits"


def test_i8_refit_com_fits_concordantes_gira():
    yaw, status, why = refit_decide(_fit(True, 10.8), _fit(True, 11.4),
                                    AGREE_TOL, YAW_TOL, MIN_CORR, 0.64, 0.46)
    assert status == "girar" and why == ""
    assert yaw == pytest.approx(math.radians(11.1))


@pytest.mark.parametrize("y1, y2, front, status", [
    (10.8, 14.0, 0.64, "discordam"),     # 3,2 graus > 2,0
    (3.0, 3.5, 0.64, "dentro"),          # dentro de 5,2
    (6.0, 6.5, 0.64, "coast"),           # 0,109 rad <= 0,12: o giro nem parte
    (10.8, 11.4, 0.45, "perto"),         # front < rotate_min 0,46: bico raspa
    (10.8, 11.4, None, "perto"),
])
def test_i8_refit_outros_motivos_nao_giram(y1, y2, front, status):
    yaw, st, why = refit_decide(_fit(True, y1), _fit(True, y2),
                                AGREE_TOL, YAW_TOL, MIN_CORR, front, 0.46)
    assert st == status and st != "girar"
    assert (yaw is None) == (status == "discordam")


def test_i8_correction_counted_so_com_giro():
    """Decisao pura 'conta como corrigido' (corrected_once): so' o status
    'girou' do refit_square('correcao'); todos os outros (fits rejeitados,
    discordam, dentro, coast, perto, sem_muro) deixam a correcao em aberto."""
    assert dan.correction_counted("girou") is True
    for st in ("sem_fits", "discordam", "dentro", "coast", "perto", "sem_muro", None, ""):
        assert dan.correction_counted(st) is False


def test_i8_corrected_once_so_quando_girou():
    """TESTE DE FONTE (regex, nao de comportamento): guarda de regressao no
    laco final de _run_attempt — a unica atribuicao corrected_once = True
    fica sob `if correction_counted(st):` (st = status do
    refit_square("correcao")); antes era incondicional (F12). A decisao em
    si e' testada em test_i8_correction_counted_so_com_giro."""
    node_src = open(SCRIPT, encoding="utf-8").read()
    assert node_src.count("corrected_once = True") == 1
    assert re.search(r'if correction_counted\(st\):\n\s+corrected_once = True', node_src)
    assert re.search(r'_f, _n, _fy, st = refit_square\("correcao"\)', node_src)
    assert dan._YAW_CORRECTION_MAX_CYCLES >= 1


# R3 — refine_half_width_y por dock no docking.yaml.
YAML_BASE = """
dock_plugins:
  caramelo_ws6_dock:
    plugin: opennav_docking::SimpleNonChargingDock
    staging_x_offset: -0.25
docks:
  WS6:
    type: caramelo_ws6_dock
    frame: map
    pose: [-0.1623, -5.7086, -1.617509]
    id: WS6
{extra}
  WS1:
    type: caramelo_ws6_dock
    frame: map
    pose: [1.0, 2.0, 0.5]
    id: WS1
    desired_face_dist: 0.40
"""


def _parse(extra, dock):
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "docking.yaml"
        path.write_text(YAML_BASE.format(extra=extra), encoding="utf-8")
        return parse_dock(dan.load_yaml(path), dock, str(path))


def test_r3_campo_presente_devolve_o_override():
    x, y, yaw, frame, face_dist, staging, half_w = _parse("    refine_half_width_y: 0.35", "WS6")
    assert (x, y, yaw) == (-0.1623, -5.7086, -1.617509)
    assert frame == "map" and face_dist is None
    assert staging == pytest.approx(0.25)
    assert half_w == pytest.approx(0.35)


def test_r3_campo_ausente_devolve_none_e_o_resto_igual():
    """Sem o campo o no' usa o parametro global (0,45): None aqui."""
    x, y, yaw, frame, face_dist, staging, half_w = _parse("", "WS6")
    assert half_w is None and staging == pytest.approx(0.25)
    _x, _y, _yaw, _f, face_dist, _s, half_w = _parse("", "ws1")   # id normalizado
    assert face_dist == pytest.approx(0.40) and half_w is None


@pytest.mark.parametrize("value", ["0", "-0.3", "abc", ".nan"])
def test_r3_valor_invalido_cai_no_global(value):
    assert _parse(f"    refine_half_width_y: {value}", "WS6")[6] is None


def test_r3_dock_inexistente():
    with pytest.raises(KeyError):
        _parse("", "WS9")


def test_r3_yaml_real_da_arena_tem_o_override_da_ws6():
    """O docking.yaml da arenalegal_2026 (fora deste pacote) carrega 0,35
    na WS6 e nada nas outras; o opennav so' le' type/frame/pose/id."""
    real = Path(HERE) / ".." / ".." / "caramelo_mapping" / "maps" / "arenalegal_2026" / "docking.yaml"
    if not real.exists():
        pytest.skip("mapa arenalegal_2026 fora do workspace")
    data = dan.load_yaml(real)
    assert parse_dock(data, "WS6")[6] == pytest.approx(0.35)
    assert parse_dock(data, "WS1")[6] is None
