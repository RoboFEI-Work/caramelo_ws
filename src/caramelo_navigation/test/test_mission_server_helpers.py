"""Helpers puros do mission_server (sem grafo ROS no ar).

Cobre o que a GUI e o painel web dependem e que nao passa por action: a
citacao do --finish vazio, a limpeza das linhas do executor, o escrivao que
transforma o stdout do run_mission em callbacks, as linhas do plano e a linha
de comando do executor montada sobre run_mission._executor_cmd.
Roda com: python3 -m pytest src/caramelo_navigation/test/test_mission_server_helpers.py -q
(precisa do workspace com caramelo_msgs buildado e sourceado: o modulo importa rclpy).
"""
import importlib
import sys
import types
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

ms = importlib.import_module("mission_server")


def _goal(**extra):
    base = dict(
        task_yaml="missions/task_BMT.yaml", map_name="arena", map_dir="",
        dry_run=False, simulate_nav=False, use_lidar_refine=True,
        finish_dock_id="FINISH", skip_startup_home=False, actions_yaml="",
        preflight_timeout=0.0)
    base.update(extra)
    return types.SimpleNamespace(**base)


def test_finish_vazio_vira_aspas_literais():
    assert ms.finish_param("FINISH") == "FINISH"
    assert ms.finish_param("") == "''"


def test_args_do_run_mission_usa_default_do_preflight():
    args = ms.args_do_run_mission(_goal(preflight_timeout=0.0, simulate_nav=True))
    assert args.simulate_nav is True
    assert args.preflight_timeout == 120.0
    assert args.finish == "FINISH"
    assert ms.args_do_run_mission(_goal(preflight_timeout=7.5)).preflight_timeout == 7.5


def test_executor_cmd_segue_run_mission_e_acrescenta_so_o_que_a_ui_escolhe(tmp_path):
    cmd = ms.executor_cmd(tmp_path / "a.yaml", tmp_path, _goal())
    assert cmd[:4] == ["ros2", "run", "manip_bt", "bt_yaml_executor"]
    assert "--dry-run" not in cmd
    assert f"map_folder:={tmp_path}" in cmd
    assert "simulate_navigation:=false" in cmd
    assert "finish_dock_id:=FINISH" in cmd
    assert "startup_home:=false" not in cmd
    assert "use_lidar_refine:=false" not in cmd

    cmd = ms.executor_cmd(
        tmp_path / "a.yaml", tmp_path,
        _goal(skip_startup_home=True, use_lidar_refine=False, finish_dock_id="",
              simulate_nav=True))
    assert "startup_home:=false" in cmd
    assert "use_lidar_refine:=false" in cmd
    assert "finish_dock_id:=''" in cmd
    assert "simulate_navigation:=true" in cmd


def test_limpar_linha_tira_ansi_e_carimbo_do_rclcpp():
    bruta = "\x1b[32m[INFO] [1761234567.123456789] [bt_yaml_executor]: Arvore terminou com SUCESSO\x1b[0m"
    assert ms.limpar_linha(bruta) == "[bt_yaml_executor]: Arvore terminou com SUCESSO"
    assert ms.limpar_linha("  texto sem prefixo  ") == "texto sem prefixo"


def test_escrivao_entrega_linha_por_linha_e_o_resto_no_flush():
    linhas = []
    escrivao = ms._Escrivao(linhas.append)
    escrivao.write("a\nb")
    assert linhas == ["a"]
    escrivao.write("\n\n")
    assert linhas == ["a", "b"]
    escrivao.write("fim sem quebra")
    escrivao.flush()
    assert linhas == ["a", "b", "fim sem quebra"]


def test_linhas_do_plano_no_formato_do_run_mission(tmp_path):
    actions = tmp_path / "actions.yaml"
    actions.write_text(
        "actions:\n"
        "  - {kind: goto, ws: WS_1}\n"
        "  - {kind: pick, tag: tag_07}\n"
        "  - {kind: place, tag: tag_07, ws: WS_4}\n",
        encoding="utf-8")
    linhas, lista = ms.linhas_do_plano(actions)
    assert len(lista) == 3
    assert [l.strip()[:2] for l in linhas] == ["1.", "2.", "3."]
    assert "goto" in linhas[0] and "ws=WS_1" in linhas[0]


def test_task_id_of_nunca_levanta(tmp_path):
    task = tmp_path / "t.yaml"
    task.write_text("task_id: BMT\nname: x\n", encoding="utf-8")
    assert ms.task_id_of(task) == "BMT"
    assert ms.task_id_of(tmp_path / "nao_existe.yaml") == ""
    (tmp_path / "lista.yaml").write_text("- 1\n- 2\n", encoding="utf-8")
    assert ms.task_id_of(tmp_path / "lista.yaml") == ""


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
