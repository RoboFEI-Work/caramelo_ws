"""Corrida completa (caramelo_run_common + analisar_run) sem ROS no ar (2026-11-13).

Cobre: achar a janela do RViz do robo no xwininfo, as cores/topicos da
global_localization.rviz, comando do ffmpeg (VAAPI e x264),
epoch do 1o quadro, recorte dos logs da corrida, pasta + metadata.json com
bag/video desligados, e as regras puras do analisar_run.
Roda com: python3 -m pytest src/caramelo_navigation/test/test_run_common.py -q
"""
import importlib
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

rc = importlib.import_module("caramelo_run_common")
an = importlib.import_module("analisar_run")


def _mapa(tmp_path, w=100, h=50, conhecido=(10, 60, 5, 45)):
    """PGM trinary 100x50 a 0,1 m: so' colunas 10..59 e linhas 5..44 conhecidas."""
    c0, c1, l0, l1 = conhecido
    px = bytearray([205] * (w * h))
    for lin in range(l0, l1):
        for col in range(c0, c1):
            px[lin * w + col] = 254
    (tmp_path / "map.pgm").write_bytes(b"P5\n# comentario\n%d %d\n255\n" % (w, h) + bytes(px))
    (tmp_path / "map.yaml").write_text("image: map.pgm\nresolution: 0.1\norigin: [-1.0, -2.0, 0]\n")
    return tmp_path


def test_achar_janela_do_rviz_do_robo():
    moldura = ('     0x6014bd "/s/global_localization.rviz - RViz": ("mutter-x11-frames" '
               '"mutter-x11-frames")  10x10+0+0  +0+0\n')
    nav = ('        0x1000106 "/s/global_localization.rviz - RViz": ("rviz2" "rviz2")  '
           '1850x1016+0+0  +70+69\n')
    manip = ('        0x2000106 "/s/manip_moveit_rasp.rviz - RViz": ("rviz2" "rviz2")  '
             '1850x1016+0+0  +0+0\n')
    # com dois RViz, so' o da navegacao (pelo titulo); nunca a moldura do mutter
    assert rc.achar_janela("global_localization.rviz", moldura + nav + manip) == "0x1000106"
    # titulo diferente e um unico RViz aberto: grava esse
    assert rc.achar_janela("global_localization.rviz", moldura + manip) == "0x2000106"
    # titulo diferente e dois RViz: nao chuta
    assert rc.achar_janela("outro.rviz", nav + manip) is None
    assert rc.achar_janela("global_localization.rviz", "") is None


def test_config_do_rviz_do_robo_tem_as_cores_e_nada_pesado():
    import yaml
    cfg = (SCRIPTS.parents[1] / "caramelo_localization" / "rviz" / "global_localization.rviz")
    txt = cfg.read_text()
    assert "Marker Topic:" not in txt  # rviz2 Jazzy ignora a chave: o display nao assina nada
    topicos = {}

    def walk(ds):
        for d in ds:
            if "Displays" in d:
                walk(d["Displays"])
            t = d.get("Topic")
            if isinstance(t, dict):
                topicos[t["Value"]] = d
    walk(yaml.safe_load(txt)["Visualization Manager"]["Displays"])
    esperado = {"/plan": "0; 90; 255", "/local_plan": "0; 230; 0",
                "/diag/trilha_real": "255; 220; 0", "/diag/plano_anterior": "140; 170; 255",
                "/expansions": "255; 140; 0"}
    for top, cor in esperado.items():
        assert topicos[top]["Color"] == cor, top
    for top in ("/plan", "/local_plan", "/diag/trilha_real", "/diag/plano_anterior"):
        assert topicos[top]["Pose Style"] == "None" and topicos[top]["Line Style"] == "Billboards"
    assert "/diag/eventos" in topicos and "/caramelo/service_areas/markers" in topicos
    for proibido in ("/marker", "/evaluation", "/trajectories", "/cost_cloud"):
        assert proibido not in topicos


def test_comando_ffmpeg_vaapi_e_x264(tmp_path):
    v = rc.comando_ffmpeg("0x1", tmp_path / "v.mp4", ":1", "run_x", 1000, vaapi=True)
    assert v[v.index("-window_id") + 1] == "0x1" and "-copyts" in v and "-nostdin" in v
    assert "h264_vaapi" in v and rc.VAAPI_DEV in v
    filtro = v[v.index("-vf") + 1]
    assert "pts\\:hms\\:-1000" in filtro and filtro.index("drawtext") < filtro.index("setpts")
    assert "+frag_keyframe+empty_moov+default_base_moof" in v
    x = rc.comando_ffmpeg("0x1", tmp_path / "v.mp4", ":1", "run_x", 1000, vaapi=False)
    assert "libx264" in x and "h264_vaapi" not in x and "-vaapi_device" not in x


def test_epoch_inicio_pega_a_ultima_tentativa():
    log = ("Input #0, x11grab, from ':1':\n  Duration: N/A, start: 100.500000, bitrate: 1\n"
           "erro vaapi\nInput #0, x11grab, from ':1':\n  Duration: N/A, start: 103.250000,\n")
    assert rc.epoch_inicio_video(log) == 103.25
    assert rc.epoch_inicio_video("nada") is None


def test_logs_copia_so_o_que_a_corrida_escreveu(tmp_path):
    log = tmp_path / "log"
    (log / "launch_a").mkdir(parents=True)
    velho, cresce = log / "velho.log", log / "launch_a" / "launch.log"
    velho.write_text("antes\n")
    cresce.write_text("antes\n")
    tam = rc.tamanhos_logs(log)
    inicio = time.time()
    os.utime(velho, (inicio - 100, inicio - 100))
    with open(cresce, "a") as f:
        f.write("durante\n")
    (log / "novo.log").write_text("novo\n")
    copiados = rc.copiar_logs_da_corrida(log, tam, inicio, tmp_path / "dst")
    nomes = sorted(str(p.relative_to(tmp_path / "dst")) for p in copiados)
    assert nomes == ["launch_a/launch.log", "novo.log"]
    assert (tmp_path / "dst" / "launch_a" / "launch.log").read_text() == "durante\n"


def test_iniciar_e_parar_run_com_tudo_desligado(tmp_path, monkeypatch):
    monkeypatch.setenv("CARAMELO_VIDEO", "0")
    monkeypatch.setenv("CARAMELO_ANALISE", "0")
    monkeypatch.setattr(rc.bagc, "iniciar_bag", lambda *a, **k: None)
    monkeypatch.setattr(rc, "_executavel", lambda nome: [sys.executable, "-c",
                                                          "import time; time.sleep(30)"])
    monkeypatch.setattr(rc, "_dump_params", lambda destino: None)
    monkeypatch.setattr(rc, "tamanhos_logs", lambda d: {})
    monkeypatch.setattr(rc, "copiar_logs_da_corrida", lambda *a, **k: [])
    task = tmp_path / "task_BTT1.yaml"
    task.write_text("x: 1\n")
    (tmp_path / "m").mkdir()
    run = rc.iniciar_run("20261113_150000_actions", "task_BTT1", "arena", task_yaml=task,
                         map_folder=_mapa(tmp_path / "m"), runs_dir=tmp_path / "runs")
    assert run is not None and run.pasta.name.startswith("run_")
    assert (tmp_path / "runs" / "COLCON_IGNORE").exists()
    assert (run.pasta / "config" / "task_BTT1.yaml").exists()
    meta = json.loads((run.pasta / "metadata.json").read_text())
    assert meta["resultado"]["texto"] == "em andamento" and meta["video"]["ok"] is False
    viz = run.procs["viz"]
    assert rc.parar_run(run, resultado=0) == run.pasta
    assert viz.poll() is not None  # no' de diagnostico fechado por SIGINT
    meta = json.loads((run.pasta / "metadata.json").read_text())
    assert meta["resultado"] == {"codigo": 0, "texto": "sucesso"} and meta["duracao_s"] >= 0
    assert rc.parar_run(run, resultado=1) == run.pasta  # idempotente: nao reescreve
    assert json.loads((run.pasta / "metadata.json").read_text())["resultado"]["codigo"] == 0


def test_parar_run_nunca_levanta(tmp_path, monkeypatch):
    run = rc.Run(pasta=tmp_path, inicio_epoch=time.time(), meta={})  # meta sem chaves
    assert rc.parar_run(run) == tmp_path
    assert rc.parar_run(None) is None


# ------------------------------------------------------------------ analisar_run
def test_janelas_respeita_duracao_e_buracos():
    amostras = [(t / 10, 1) for t in range(0, 30)] + [(5.0 + t / 10, 1) for t in range(0, 5)]
    js = an.janelas(amostras, lambda a: a[1] == 1, 1.0, gap_s=0.3)
    assert js == [(0.0, 2.9)]  # o trecho de 0,4 s depois do buraco nao chega a 1 s


def test_oscilacoes_conta_inversoes_de_sinal():
    cmd = [(i * 0.5, 0, 0, 0.3 * (-1) ** i) for i in range(12)]  # troca a cada 0,5 s
    assert an.oscilacoes(cmd, 3) and an.oscilacoes(cmd, 3)[0][0] == 0.5
    calmo = [(i * 0.5, 0, 0, 0.3) for i in range(12)]
    assert an.oscilacoes(calmo, 3) == []


def test_saltos_amcl_desconta_a_odometria():
    odom = [(0.0, 0, 0, 0, 0, 0, 0), (1.0, 0.5, 0, 0, 0, 0, 0)]
    andou_junto = [(0.0, 0, 0, 0, 0.1), (1.0, 0.5, 0, 0, 0.1)]
    pulou = [(0.0, 0, 0, 0, 0.1), (1.0, 0.9, 0, 0, 0.1)]
    assert an.saltos_amcl(andou_junto, odom) == []
    assert len(an.saltos_amcl(pulou, odom)) == 1


def test_intervalos_actions_e_resultado():
    st = {"navigate_to_pose": [(1.0, [("aa", 2)]), (5.0, [("aa", 4), ("bb", 2)]),
                               (9.0, [("bb", 6)])]}
    iv = an.intervalos_actions(st)
    assert iv == [("navigate_to_pose", "aa", 1.0, 5.0, "sucesso"),
                  ("navigate_to_pose", "bb", 5.0, 9.0, "abortado")]


def test_distancia_a_polilinha():
    assert math.isclose(an.dist_ponto_polilinha((1, 1), [(0, 0), (2, 0)]), 1.0)
    assert math.isclose(an.dist_ponto_polilinha((3, 0), [(0, 0), (2, 0)]), 1.0)


def test_hipoteses_separam_piso_do_esc():
    r = {"eventos": [(1.0, "travado", "controle", "comando sem movimento ... ABAIXO do piso do ESC)"),
                     (2.0, "travado", "controle", "comando sem movimento (cmd ate 0.30 m/s)")],
         "resets": [], "pernas": []}
    titulos = [h[0] for h in an.hipoteses(r)]
    assert any("piso do ESC" in t for t in titulos)
    assert any("acima do piso" in t for t in titulos)
