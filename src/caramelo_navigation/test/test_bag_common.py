"""Rosbag por corrida (caramelo_bag_common) sem ROS no ar.

Cobre a montagem do comando do `ros2 bag record` (flags do rosbag2 0.26.11,
lista MINIMA/EXTRA/camera, nada da TOPICOS_NUNCA), o nome da pasta, a
politica de limpeza de ~/bags com pastas falsas em tmp (limite de tamanho,
de quantidade, .keep), o opt-out (CARAMELO_BAG=0), a recusa por disco baixo
e a parada SIGINT -> SIGTERM sem SIGKILL com processos falsos.
Roda com: python3 -m pytest src/caramelo_navigation/test/test_bag_common.py -q
"""
import importlib
import os
import signal
import subprocess
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

bag = importlib.import_module("caramelo_bag_common")


# ------------------------------------------------------------- comando/listas
def test_listas_nao_se_misturam():
    assert not set(bag.TOPICOS_MIN) & set(bag.TOPICOS_NUNCA)
    assert not set(bag.TOPICOS_EXTRA) & set(bag.TOPICOS_NUNCA)
    assert not set(bag.TOPICOS_MIN) & set(bag.TOPICOS_EXTRA)
    assert len(bag.TOPICOS_MIN) == len(set(bag.TOPICOS_MIN))
    # os que importam para o diagnostico do caranguejo e do laco
    for t in ("/cmd_vel_nav", "/cmd_vel", "/odom", "/scan", "/tf", "/tf_static",
              "/navigate_to_pose/_action/status", "/local_costmap/costmap"):
        assert t in bag.TOPICOS_MIN
    # os de visualizacao do controller nunca entram
    for t in ("/marker", "/evaluation", "/trajectories", "/cost_cloud"):
        assert t in bag.TOPICOS_NUNCA


def test_montar_topicos_min_extra_camera():
    minimo = bag.montar_topicos()
    assert minimo == bag.TOPICOS_MIN
    extra = bag.montar_topicos(extra=True)
    assert set(bag.TOPICOS_EXTRA) <= set(extra) and len(extra) == len(set(extra))
    cam = bag.montar_topicos(camera=True)
    assert cam[-1] == bag.TOPICO_CAMERA_COMPRIMIDA
    assert bag.TOPICO_CAMERA_RAW not in cam
    with pytest.raises(ValueError):
        bag.montar_topicos(topicos=["/odom", "/marker"])


def test_montar_comando_tem_as_flags_do_rosbag2(tmp_path):
    saida = tmp_path / "20261111_153000_task_BMT_arena2_520"
    cmd = bag.montar_comando(saida, ["/odom", "/scan"], {"task": "task_BMT", "git": "abc123",
                                                         "vazio": ""})
    assert cmd[:3] == ["ros2", "bag", "record"]

    def valor(flag):
        return cmd[cmd.index(flag) + 1]

    assert valor("--storage") == "mcap"
    assert valor("--storage-preset-profile") == "zstd_fast"
    assert valor("--max-bag-size") == "524288000"
    assert valor("--max-cache-size") == "16777216"
    assert valor("--node-name") == "caramelo_bag_recorder"
    assert valor("--output") == str(saida)
    assert "--include-hidden-topics" in cmd
    assert "-a" not in cmd and "--all" not in cmd
    assert "--qos-profile-overrides-path" not in cmd
    i = cmd.index("--custom-data")
    assert cmd[i + 1:i + 3] == ["task=task_BMT", "git=abc123"]
    assert not any(c.startswith("vazio=") for c in cmd)
    # --topics e' o ULTIMO bloco (nargs=+ engoliria o que viesse depois)
    assert cmd[cmd.index("--topics") + 1:] == ["/odom", "/scan"]


def test_nome_corrida_tira_o_sufixo_actions_e_sanitiza():
    assert bag.nome_corrida("20261111_153000_actions", "task_BMT", "arena2_520") == \
        "20261111_153000_task_BMT_arena2_520"
    assert bag.nome_corrida("x", "tarefa com espaco/barra", "mapa") == \
        "x_tarefa_com_espaco_barra_mapa"
    assert bag.nome_corrida("", "", "") == "x_x_x"


def test_corrida_yaml_tem_os_metadados(tmp_path, monkeypatch):
    monkeypatch.setattr(bag, "git_rev", lambda: "deadbee")
    monkeypatch.setenv("CARAMELO_NAV_PROFILE", "mppi")
    meta = bag.metadados_corrida("20261111_153000_actions", "task_BMT", "arena2_520",
                                 tmp_path / "a.yaml", None, extra=True, camera=False,
                                 topicos=["/odom"])
    arq = bag.escrever_corrida_yaml(tmp_path / "corrida", meta)
    texto = arq.read_text(encoding="utf-8")
    import yaml
    lido = yaml.safe_load(texto)
    assert lido["task"] == "task_BMT" and lido["mapa"] == "arena2_520"
    assert lido["nav_profile"] == "mppi" and lido["git_rev"] == "deadbee"
    assert lido["extra"] is True and lido["camera"] is False
    assert lido["topicos"] == ["/odom"]
    assert lido["hostname"] and lido["hora"]


# ------------------------------------------------------------------ limpeza
def _corrida_falsa(raiz: Path, nome: str, mb: int, idade_s: int, keep=False) -> Path:
    p = raiz / nome
    p.mkdir(parents=True)
    (p / f"{nome}_0.mcap").write_bytes(b"\0" * (mb * 2**20))
    (p / "metadata.yaml").write_text("x\n")
    if keep:
        (p / ".keep").touch()
    import time
    t = time.time() - idade_s
    os.utime(p, (t, t))
    return p


def test_limpeza_por_tamanho_preserva_keep_e_apaga_as_mais_antigas(tmp_path):
    a = _corrida_falsa(tmp_path, "a_velha", 3, 4000)
    k = _corrida_falsa(tmp_path, "b_keep", 3, 3000, keep=True)
    c = _corrida_falsa(tmp_path, "c_media", 3, 2000)
    d = _corrida_falsa(tmp_path, "d_nova", 3, 1000)
    (tmp_path / "solto.record.log").write_text("nao e pasta\n")
    # 12 MB no total, limite 7 MB: sai a_velha (9), depois c_media (6) - keep fica
    apagadas = bag.limpar_bags_antigas(tmp_path, limite_bytes=7 * 2**20, max_corridas=100)
    assert apagadas == [a, c]
    assert k.exists() and d.exists()
    assert (tmp_path / "solto.record.log").exists()


def test_limpeza_por_quantidade_e_preservar(tmp_path):
    pastas = [_corrida_falsa(tmp_path, f"r{i}", 0, 1000 * (10 - i)) for i in range(6)]
    # 6 pastas, max 3; r1 preservada explicitamente (a corrida atual)
    apagadas = bag.limpar_bags_antigas(tmp_path, limite_bytes=10**12, max_corridas=3,
                                       preservar=[pastas[1]])
    assert apagadas == [pastas[0], pastas[2], pastas[3]]
    assert pastas[1].exists() and pastas[4].exists() and pastas[5].exists()


def test_limpeza_sem_nada_para_fazer(tmp_path):
    assert bag.limpar_bags_antigas(tmp_path / "nao_existe") == []
    _corrida_falsa(tmp_path, "unica", 1, 10)
    assert bag.limpar_bags_antigas(tmp_path, limite_bytes=10**9, max_corridas=5) == []


# ------------------------------------------------------- opt-out / disco baixo
class _PopenProibido:
    def __init__(self, *a, **k):
        raise AssertionError("ros2 bag record nao devia ter sido chamado")


def test_opt_out_por_env(monkeypatch, tmp_path):
    monkeypatch.setenv("CARAMELO_BAG", "0")
    monkeypatch.setattr(bag.subprocess, "Popen", _PopenProibido)
    assert bag.iniciar_bag("m", "t", "mapa", tmp_path / "a.yaml", bag_dir=tmp_path) is None
    assert not bag.bag_habilitado({"CARAMELO_BAG": "false"})
    assert bag.bag_habilitado({})
    assert bag.bag_habilitado({"CARAMELO_BAG": "1"})


def test_disco_baixo_nao_grava_e_nao_levanta(monkeypatch, tmp_path, capsys):
    monkeypatch.delenv("CARAMELO_BAG", raising=False)
    monkeypatch.setattr(bag.subprocess, "Popen", _PopenProibido)
    monkeypatch.setattr(bag, "espaco_livre_bytes", lambda _p: 2 * 2**30)
    assert bag.iniciar_bag("m", "t", "mapa", tmp_path / "a.yaml", bag_dir=tmp_path / "bags") is None
    assert "GB livres" in capsys.readouterr().err
    assert (tmp_path / "bags").is_dir()


# --------------------------------------------------- iniciar/parar com fakes
class _ProcFalso:
    """Imita o Popen do record: cria a pasta de saida ao 'abrir o bag',
    sai no SIGINT (ou so no SIGTERM se teimoso). kill() e' proibido."""

    def __init__(self, cmd, *a, teimoso=False, **k):
        self.cmd = cmd
        self.pid = 4242
        self.sinais = []
        self.teimoso = teimoso
        self.returncode = None
        if "--output" in cmd:
            Path(cmd[cmd.index("--output") + 1]).mkdir(parents=True, exist_ok=True)

    def poll(self):
        return self.returncode

    def send_signal(self, sig):
        self.sinais.append(sig)
        if sig == signal.SIGINT and not self.teimoso:
            self.returncode = 0

    def terminate(self):
        self.sinais.append(signal.SIGTERM)
        self.returncode = 0

    def kill(self):
        raise AssertionError("SIGKILL e' proibido no record")

    def wait(self, timeout=None):
        if self.returncode is None:
            raise subprocess.TimeoutExpired(self.cmd, timeout)
        return self.returncode


def test_iniciar_e_parar_com_processo_falso(monkeypatch, tmp_path):
    monkeypatch.delenv("CARAMELO_BAG", raising=False)
    monkeypatch.setattr(bag.subprocess, "Popen", _ProcFalso)
    monkeypatch.setattr(bag, "espaco_livre_bytes", lambda _p: 100 * 2**30)
    monkeypatch.setattr(bag, "git_rev", lambda: "abc1234")
    corrida = bag.iniciar_bag("20261111_153000_actions", "task_BMT", "arena2_520",
                              tmp_path / "a.yaml", bag_dir=tmp_path / "bags",
                              nav_profile="dwb", espera_descoberta_s=0.0)
    assert corrida is not None
    assert corrida.saida == tmp_path / "bags" / "20261111_153000_task_BMT_arena2_520"
    assert (corrida.saida / "corrida.yaml").exists()
    assert corrida.log.exists()
    assert "nav_profile=dwb" in corrida.comando and "git=abc1234" in corrida.comando
    assert len(corrida.procs) == 1  # sem camera, sem republish

    saida = bag.parar_bag(corrida, espera_s=0.1, espera_term_s=0.1)
    assert saida == corrida.saida
    assert corrida.procs[0].sinais == [signal.SIGINT]
    assert (corrida.saida / "record.log").exists() and not corrida.log.exists()
    # idempotente
    assert bag.parar_bag(corrida, espera_s=0.1) == corrida.saida


def test_camera_sobe_republish_antes_do_record(monkeypatch, tmp_path):
    monkeypatch.delenv("CARAMELO_BAG", raising=False)
    monkeypatch.setattr(bag.subprocess, "Popen", _ProcFalso)
    monkeypatch.setattr(bag, "espaco_livre_bytes", lambda _p: 100 * 2**30)
    monkeypatch.setattr(bag, "git_rev", lambda: "abc1234")  # subprocess.run usa Popen
    corrida = bag.iniciar_bag("m", "t", "mapa", tmp_path / "a.yaml", camera=True,
                              bag_dir=tmp_path / "bags", espera_descoberta_s=0.0)
    assert [p.cmd[:3] for p in corrida.procs] == [["ros2", "run", "image_transport"],
                                                  ["ros2", "bag", "record"]]
    # modo enxuto: topico listado; modo TUDO (default 2026-11-11): entra pelo
    # --all-topics e a regex de exclusao nao pode pega-lo.
    import re as _re
    if "--all-topics" in corrida.comando:
        assert not _re.search(bag.EXCLUIR_REGEX, bag.TOPICO_CAMERA_COMPRIMIDA)
    else:
        assert bag.TOPICO_CAMERA_COMPRIMIDA in corrida.comando
    bag.parar_bag(corrida, espera_s=0.1)
    # para na ordem inversa: record primeiro, republish depois
    assert all(p.sinais == [signal.SIGINT] for p in corrida.procs)


def test_parar_bag_cai_para_sigterm_e_nunca_sigkill():
    teimoso = _ProcFalso(["ros2", "bag", "record"], teimoso=True)
    bag.parar_bag([teimoso], espera_s=0.05, espera_term_s=0.05)
    assert teimoso.sinais == [signal.SIGINT, signal.SIGTERM]
    assert bag.parar_bag(None) is None
    assert bag.parar_bag([]) is None


def test_record_que_morre_cedo_vira_none(monkeypatch, tmp_path, capsys):
    class _Morto(_ProcFalso):
        def __init__(self, cmd, *a, **k):
            super().__init__(cmd, *a, **k)
            self.returncode = 2
            if "--output" in cmd:
                Path(cmd[cmd.index("--output") + 1]).rmdir()

    monkeypatch.delenv("CARAMELO_BAG", raising=False)
    monkeypatch.setattr(bag.subprocess, "Popen", _Morto)
    monkeypatch.setattr(bag, "espaco_livre_bytes", lambda _p: 100 * 2**30)
    monkeypatch.setattr(bag, "git_rev", lambda: "abc1234")
    assert bag.iniciar_bag("m", "t", "mapa", tmp_path / "a.yaml", bag_dir=tmp_path / "bags",
                           espera_descoberta_s=0.0) is None
    assert "saiu com 2" in capsys.readouterr().err


# ---------------------------------------------------- bag_resumo (funcoes puras)
resumo = pytest.importorskip("bag_resumo", reason="bag_resumo importa rosbag2_py/rclpy")


def test_intervalos_de_goals_e_caranguejo():
    g1, g2 = "a" * 32, "b" * 32
    s = 1_000_000_000
    eventos = [
        (0 * s, g1, resumo.STATUS_EXECUTING),
        (5 * s, g1, 4),                       # sucesso em t=5
        (5 * s, g2, resumo.STATUS_EXECUTING),
        (9 * s, g2, 6),                       # abortado em t=9
    ]
    goals = resumo.intervalos_de_goals(eventos)
    assert [(g["goal"], g["inicio_ns"], g["fim_ns"], g["status"]) for g in goals] == [
        (g1, 0, 5 * s, "sucesso"), (g2, 5 * s, 9 * s, "abortado")]

    # goal 1: 5 s andando de lado; goal 2: 2 s em frente, 2 s parado
    cmds = [(t * s, 0.0, 0.2, 0.0) for t in range(0, 5)]
    cmds += [(5 * s, 0.2, 0.0, 0.0), (6 * s, 0.2, 0.01, 0.0), (7 * s, 0.0, 0.0, 0.0),
             (8 * s, 0.0, 0.0, 0.0)]
    r = resumo.caranguejo_por_goal(cmds, goals, 9 * s)
    assert r[0]["goal"] == "aaaaaaaa" and r[0]["fracao_lateral"] == pytest.approx(1.0)
    assert r[0]["tempo_movimento_s"] == pytest.approx(5.0)
    assert r[1]["fracao_lateral"] == pytest.approx(0.0)
    assert r[1]["tempo_movimento_s"] == pytest.approx(2.0)
    assert r[1]["duracao_s"] == pytest.approx(4.0)
    # sem status: o bag inteiro vira um goal so
    todo = resumo.caranguejo_por_goal(cmds, [], 9 * s)
    assert len(todo) == 1 and todo[0]["goal"] == "bag_inteiro"
    assert todo[0]["fracao_lateral"] == pytest.approx(5.0 / 7.0)


def test_taxa_cmd_e_estat_ms():
    ts = [i * 50_000_000 for i in range(21)]  # 20 Hz por 1 s
    del ts[10:12]                             # dois ciclos perdidos: um buraco de 150 ms
    t = resumo.taxa_cmd(ts)
    assert t["n"] == 19 and t["taxa_hz"] == pytest.approx(18.0)
    assert t["intervalo_mediana_ms"] == pytest.approx(50.0)
    assert t["intervalos_acima_100ms"] == 1
    assert resumo.taxa_cmd([1]) == {"n": 1}
    e = resumo.estat_ms([1_000_000, 2_000_000, 3_000_000])
    assert e["n"] == 3 and e["mediana"] == pytest.approx(2.0) and e["max"] == pytest.approx(3.0)
    assert resumo.estat_ms([]) == {"n": 0}


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))


def test_montar_comando_modo_tudo(tmp_path):
    """2026-11-11: modo TUDO = --all-topics + --exclude-regex; nada de --topics."""
    import re
    saida = tmp_path / "corrida"
    cmd = bag.montar_comando(saida, ["/odom"], {"task": "t"}, tudo=True)
    assert "--all-topics" in cmd and "--topics" not in cmd
    assert cmd[cmd.index("--exclude-regex") + 1] == bag.EXCLUIR_REGEX
    assert "--include-hidden-topics" in cmd
    rx = re.compile(bag.EXCLUIR_REGEX)
    # a lista negra antiga continua excluida pela regex
    for t in bag.TOPICOS_NUNCA:
        assert rx.search(t), t
    # o que precisamos continua entrando
    for t in bag.TOPICOS_MIN + bag.TOPICOS_EXTRA + [bag.TOPICO_CAMERA_COMPRIMIDA,
                                                     "/joint_states", "/arm_controller/controller_state",
                                                     "/pick_tag/_action/feedback", "/detections",
                                                     "/manip/speech", "/particle_cloud",
                                                     "/local_costmap/costmap", "/global_costmap/costmap",
                                                     "/execute_trajectory/_action/feedback"]:
        assert not rx.search(t), t
    # ruido pesado visto no bag 4 de 11/11 fica fora
    for t in ["/bond", "/controller_manager/statistics/full", "/local_costmap/obstacle_layer_raw",
              "/global_costmap/static_layer", "/dynamic_joint_states", "/monitored_planning_scene",
              "/display_planned_path", "/collision_monitor/collision_points_marker"]:
        assert rx.search(t), t


def test_bag_tudo_habilitado_env():
    assert bag.bag_tudo_habilitado({}) is True
    assert bag.bag_tudo_habilitado({"CARAMELO_BAG_TUDO": "0"}) is False
    assert bag.bag_tudo_habilitado({"CARAMELO_BAG_TUDO": "off"}) is False
