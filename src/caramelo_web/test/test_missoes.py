"""Descoberta das tasks do painel (caramelo_web/missoes.py), sem ROS.

Garante a regra do repositorio: so' missions/ e missions/tasks geradas/ na raiz
do workspace, so' YAML com task_id, oficiais (tasks geradas) primeiro, mesmo
task_id em arquivos diferentes nao some da lista, e valida() so' aceita o que
listar() ofereceu.
Roda com: python3 -m pytest src/caramelo_web/test -q
"""
from pathlib import Path

import pytest

from caramelo_web import missoes


@pytest.fixture
def ws(tmp_path, monkeypatch):
    raiz = tmp_path / "missions"
    (raiz / "tasks geradas").mkdir(parents=True)
    (raiz / "exemplos").mkdir()
    (raiz / "task_BMT.yaml").write_text(
        "task_id: BMT\nname: Teste da BMT\nduration_min: 6\n"
        "active_service_areas: [WS_1, WS_4]\nobjects: [{id: 21}, {id: 7}]\n",
        encoding="utf-8")
    (raiz / "tasks geradas" / "BMT_seed974164_equipe.yaml").write_text(
        "task_id: BMT\nname: Basic Manipulation Test\nduration_min: 6\n"
        "active_service_areas: [WS_1, WS_5]\nobjects: [{id: 9}]\n",
        encoding="utf-8")
    (raiz / "tasks geradas" / "ATT1_seed1_equipe.yaml").write_text(
        "task_id: ATT1\nactive_service_areas: [{id: WS_1}, {id: SH_1}]\n",
        encoding="utf-8")
    (raiz / "nao_e_task.yaml").write_text("frame_id: map\n", encoding="utf-8")
    (raiz / "quebrado.yaml").write_text("a: [\n", encoding="utf-8")
    (raiz / "exemplos" / "acoes.yaml").write_text("task_id: NAO\nactions: []\n",
                                                  encoding="utf-8")
    monkeypatch.setattr(missoes, "raiz_do_workspace", lambda: tmp_path)
    return tmp_path


def test_lista_oficiais_primeiro_e_so_yaml_com_task_id(ws):
    lista = missoes.listar()
    assert [m["arquivo"] for m in lista] == [
        "tasks geradas/ATT1_seed1_equipe.yaml",
        "tasks geradas/BMT_seed974164_equipe.yaml",
        "task_BMT.yaml",
    ]
    # Dois arquivos com o mesmo task_id (oficial e teste a mao) continuam os dois.
    assert [m["task_id"] for m in lista].count("BMT") == 2
    bmt = next(m for m in lista if m["arquivo"] == "task_BMT.yaml")
    assert bmt["rotulo"] == "BMT — Teste da BMT  (task_BMT.yaml)"
    assert bmt["areas"] == ["WS_1", "WS_4"]
    assert bmt["objetos"] == 2
    att = next(m for m in lista if m["task_id"] == "ATT1")
    assert att["areas"] == ["WS_1", "SH_1"]
    assert att["rotulo"] == "ATT1  (tasks geradas/ATT1_seed1_equipe.yaml)"


def test_valida_so_aceita_o_que_foi_listado(ws):
    lista = missoes.listar()
    assert missoes.valida(lista[0]["caminho"])
    assert not missoes.valida(str(ws / "missions" / "nao_e_task.yaml"))
    assert not missoes.valida("/etc/passwd")


def test_sem_pasta_missions_lista_vazia(tmp_path, monkeypatch):
    monkeypatch.setattr(missoes, "raiz_do_workspace", lambda: tmp_path)
    assert missoes.listar() == []


def test_raiz_do_workspace_deduzida_da_arvore_dos_mapas(tmp_path, monkeypatch):
    mapas = tmp_path / "src" / "caramelo_mapping" / "maps"
    mapas.mkdir(parents=True)
    (tmp_path / "missions").mkdir()
    monkeypatch.setattr(missoes.arenas, "pasta_dos_mapas", lambda: mapas)
    assert missoes.raiz_do_workspace() == tmp_path
    # Sem missions/ ao lado de src/, cai para ~/caramelo_ws.
    outro = tmp_path / "outro_ws" / "src" / "caramelo_mapping" / "maps"
    outro.mkdir(parents=True)
    monkeypatch.setattr(missoes.arenas, "pasta_dos_mapas", lambda: outro)
    assert missoes.raiz_do_workspace() == Path.home() / "caramelo_ws"
