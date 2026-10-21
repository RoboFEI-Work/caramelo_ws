#!/usr/bin/env python3
"""Teste do task_planner sobre os YAML oficiais do @Work Commander (edicao 2026).

Roda o BINARIO task_planner (sem ROS no ar) sobre cada arquivo de
``test/fixtures/formato2026/`` (copias de ``missions/tasks geradas/``) com o
``tags_36h11.yaml`` do manip_bringup e o ``ws_table_mapping.yaml`` do mapa
arenalegal_2026, e confere a estrutura das acoes geradas. Tambem confere que
os fixtures antigos (BMT/ATT1 com constraints) continuam traduzindo e que uma
pilha ja existente em ``start_state.<ST>.stack`` (cubo ocupando a tag de
referencia ou a base) e' desfeita antes do place (simulacao fisica da pilha,
varias rodadas porque o planner sorteia empates).

Como rodar (da raiz do workspace, com o install sourceado):
    python3 -m pytest src/manipulation/manip_bt/test/test_task_planner_formato2026.py -q
ou pelo colcon: ``colcon test --packages-select manip_bt --ctest-args -R formato2026``.

Onde achar o binario, nesta ordem: variavel TASK_PLANNER_BIN (o CMake aponta
para o build), ``ros2 pkg prefix manip_bt``/lib/manip_bt/task_planner, PATH.
TASK_PLANNER_TAGS_YAML e TASK_PLANNER_WS_MAP sobrescrevem os outros dois
arquivos (default: os do src).
"""

import os
import pathlib
import shutil
import subprocess

import pytest
import yaml

HERE = pathlib.Path(__file__).resolve().parent
FIXTURES = HERE / "fixtures" / "formato2026"
OLD_FIXTURES = HERE / "fixtures"
WS_ROOT = HERE.parents[3]
TAGS_YAML = pathlib.Path(
    os.environ.get(
        "TASK_PLANNER_TAGS_YAML",
        WS_ROOT / "src" / "manipulation" / "manip_bringup" / "config" / "tags_36h11.yaml"))
WS_MAP = pathlib.Path(
    os.environ.get(
        "TASK_PLANNER_WS_MAP",
        WS_ROOT / "src" / "caramelo_mapping" / "maps" / "arenalegal_2026"
        / "ws_table_mapping.yaml"))

MAX_ONBOARD = 3


def _find_planner():
    env = os.environ.get("TASK_PLANNER_BIN")
    if env and pathlib.Path(env).is_file():
        return env
    try:
        prefix = subprocess.run(
            ["ros2", "pkg", "prefix", "manip_bt"], capture_output=True, text=True,
            check=False, timeout=30).stdout.strip()
        if prefix:
            candidate = pathlib.Path(prefix) / "lib" / "manip_bt" / "task_planner"
            if candidate.is_file():
                return str(candidate)
    except (OSError, subprocess.SubprocessError):
        pass
    found = shutil.which("task_planner")
    if found:
        return found
    pytest.skip("task_planner nao encontrado (TASK_PLANNER_BIN, ros2 pkg prefix ou PATH)")


@pytest.fixture(scope="module")
def planner():
    """Caminho do binario task_planner."""
    return _find_planner()


def _run(planner, task_yaml, tmp_path, ws_map=WS_MAP, tags=TAGS_YAML):
    """Roda o planner; devolve (returncode, stdout+stderr, actions ou None)."""
    out = tmp_path / (pathlib.Path(task_yaml).stem + "_actions.yaml")
    proc = subprocess.run(
        [planner, str(task_yaml), str(out), str(tags), str(ws_map)],
        capture_output=True, text=True, check=False, timeout=60)
    log = proc.stdout + proc.stderr
    actions = None
    if proc.returncode == 0:
        with open(out, encoding="utf-8") as fh:
            actions = yaml.safe_load(fh)["actions"]
    return proc.returncode, log, actions


def _plan(planner, name, tmp_path):
    rc, log, actions = _run(planner, FIXTURES / name, tmp_path)
    assert rc == 0, log
    assert "formato 2026 do @Work Commander detectado" in log, log
    assert "Translation failed" not in log, log
    return actions, log


def _check_invariants(actions):
    """Confere as regras gerais do plano.

    goto antes de pick/place, pick antes do place da mesma tag, no maximo 3 a
    bordo e ws da acao = ws do ultimo goto.
    """
    current_ws = None
    onboard = []
    picked = set()
    for i, act in enumerate(actions):
        kind = act["kind"]
        if kind == "goto":
            current_ws = act["ws"]
            assert act["mesa"], act
        elif kind in ("pick", "place", "amt1_sort"):
            assert current_ws is not None, f"[{i}] {kind} sem goto antes"
            assert act["ws"] == current_ws, f"[{i}] {act} fora da estacao {current_ws}"
            assert act["table_pose"], act
        if kind == "pick":
            assert act["tag_frame"] not in picked, f"[{i}] pick repetido {act}"
            picked.add(act["tag_frame"])
            onboard.append(act["tag_frame"])
            assert len(onboard) <= MAX_ONBOARD, f"[{i}] mais de {MAX_ONBOARD} a bordo"
        elif kind == "place":
            assert act["tag_frame"] in onboard, f"[{i}] place sem pick antes: {act}"
            onboard.remove(act["tag_frame"])
    assert not onboard, f"sobrou a bordo no fim: {onboard}"


def _simulate_stacks(actions, start_stacks):
    """Simula fisicamente as pilhas: devolve {ws: [tags base..topo]} no fim.

    Falha se um pick tira cubo que tem outro em cima, se um place na tag_200
    acontece com a tag ainda ocupada, ou se um place sobre tag_X acontece com
    X fora do topo da pilha.
    """
    stacks = {ws: [f"tag_{i}" for i in ids] for ws, ids in start_stacks.items()}
    for i, act in enumerate(actions):
        if act["kind"] == "pick":
            stack = stacks.get(act["ws"], [])
            if act["tag_frame"] in stack:
                assert stack[-1] == act["tag_frame"], \
                    f"[{i}] pick {act['tag_frame']} com {stack} (outro cubo em cima)"
                stack.pop()
        elif act["kind"] == "place" and act.get("stack_on"):
            stack = stacks.setdefault(act["ws"], [])
            base = act["stack_on"]
            if base == "tag_200":
                assert not stack, f"[{i}] place {act['tag_frame']} na tag_200 ocupada por {stack}"
            else:
                assert stack and stack[-1] == base, \
                    f"[{i}] place {act['tag_frame']} sobre {base} que nao e' o topo de {stack}"
            stack.append(act["tag_frame"])
    return stacks


def _plan_text(planner, text, tmp_path, name="caso.yaml"):
    task = tmp_path / name
    task.write_text(text, encoding="utf-8")
    return _run(planner, task, tmp_path)


STACK_HEADER = """task_id: ATT1
name: Advanced Transportation Test I
active_service_areas: [ST, WS_1, WS_5]
stacking_areas: {ST: {reference_tag: 200}}
"""

# (a) ocupante da tag_200 sai para outra mesa; (b) nova base entra EMBAIXO
# do ocupante (troca); (c) cubo entra no MEIO de uma pilha existente;
# (d) dois ocupantes, pilha final com os tres; (e) base e topo trocam de lugar.
START_STACK_CASES = {
    "ocupante_sai": (STACK_HEADER + """objects: [{id: 14}, {id: 30}]
start_state:
  ST: {obj_ids: [14], stack: [14]}
  WS_1: {obj_ids: [], table: []}
  WS_5: {obj_ids: [30], table: [30]}
finish_state:
  ST: {obj_ids: [30], stack: [30]}
  WS_1: {obj_ids: [14], table: [14]}
  WS_5: {obj_ids: [], table: []}
""", {"ST": [14]}, {"ST": ["tag_30"]}),
    "troca_base": (STACK_HEADER + """objects: [{id: 3}, {id: 14}, {id: 30}]
start_state:
  ST: {obj_ids: [14], stack: [14]}
  WS_1: {obj_ids: [3], table: [3]}
  WS_5: {obj_ids: [30], table: [30]}
finish_state:
  ST: {obj_ids: [30, 14], stack: [30, 14]}
  WS_1: {obj_ids: [3], table: [3]}
  WS_5: {obj_ids: [], table: []}
""", {"ST": [14]}, {"ST": ["tag_30", "tag_14"]}),
    "entra_no_meio": (STACK_HEADER + """objects: [{id: 3}, {id: 14}, {id: 30}]
start_state:
  ST: {obj_ids: [3, 14], stack: [3, 14]}
  WS_1: {obj_ids: [], table: []}
  WS_5: {obj_ids: [30], table: [30]}
finish_state:
  ST: {obj_ids: [3, 14, 30], stack: [3, 30, 14]}
  WS_1: {obj_ids: [], table: []}
  WS_5: {obj_ids: [], table: []}
""", {"ST": [3, 14]}, {"ST": ["tag_3", "tag_30", "tag_14"]}),
    "dois_ocupantes": (STACK_HEADER + """objects: [{id: 7}, {id: 14}, {id: 30}]
start_state:
  ST: {obj_ids: [14, 7], stack: [14, 7]}
  WS_1: {obj_ids: [], table: []}
  WS_5: {obj_ids: [30], table: [30]}
finish_state:
  ST: {obj_ids: [7, 14, 30], stack: [30, 14, 7]}
  WS_1: {obj_ids: [], table: []}
  WS_5: {obj_ids: [], table: []}
""", {"ST": [14, 7]}, {"ST": ["tag_30", "tag_14", "tag_7"]}),
    "inverte_pilha": (STACK_HEADER + """objects: [{id: 3}, {id: 14}]
start_state:
  ST: {obj_ids: [3, 14], stack: [3, 14]}
  WS_1: {obj_ids: [], table: []}
  WS_5: {obj_ids: [], table: []}
finish_state:
  ST: {obj_ids: [3, 14], stack: [14, 3]}
  WS_1: {obj_ids: [], table: []}
  WS_5: {obj_ids: [], table: []}
""", {"ST": [3, 14]}, {"ST": ["tag_14", "tag_3"]}),
}


def _places(actions, **where):
    return [a for a in actions if a["kind"] == "place" and
            all(a.get(k) == v for k, v in where.items())]


def _picks(actions, **where):
    return [a for a in actions if a["kind"] == "pick" and
            all(a.get(k) == v for k, v in where.items())]


def test_inputs_exist():
    assert FIXTURES.is_dir(), FIXTURES
    assert len(list(FIXTURES.glob("*.yaml"))) == 7, list(FIXTURES.glob("*.yaml"))
    assert TAGS_YAML.is_file(), TAGS_YAML
    assert WS_MAP.is_file(), WS_MAP


def test_bmt(planner, tmp_path):
    actions, _ = _plan(planner, "BMT_seed974164_equipe.yaml", tmp_path)
    _check_invariants(actions)
    places = _places(actions)
    assert len(places) == 3, places
    assert all(p["ws"] == "WS_1" for p in places), places
    assert sorted(p["tag_frame"] for p in places) == ["tag_13", "tag_17", "tag_26"]
    assert not any("container_color" in p for p in places), places
    assert not any("stack_on" in p or "at_tag" in p for p in places), places
    picks = _picks(actions)
    assert all(p["ws"] == "WS_5" for p in picks), picks
    assert "tag_9" not in [p["tag_frame"] for p in picks]   # fica na WS_5


def test_btt1_btt2(planner, tmp_path):
    for name, n in (("BTT1_seed967429_equipe.yaml", 3), ("BTT2_seed597609_equipe.yaml", 5)):
        actions, _ = _plan(planner, name, tmp_path)
        _check_invariants(actions)
        assert len(_picks(actions)) == n, name
        assert len(_places(actions)) == n, name
        assert not any(
            k in a for a in actions for k in ("container_color", "stack_on", "at_tag")), name


def test_att2(planner, tmp_path):
    actions, _ = _plan(planner, "ATT2_seed343421_equipe.yaml", tmp_path)
    _check_invariants(actions)
    places = _places(actions)
    assert len(places) == 8, places
    blue = _places(actions, container_color="BLUE")
    red = _places(actions, container_color="RED")
    assert sorted(p["tag_frame"] for p in blue) == ["tag_2", "tag_5"], blue
    assert all(p["ws"] == "WS_2" for p in blue), blue
    assert sorted(p["tag_frame"] for p in red) == ["tag_3", "tag_7"], red
    assert all(p["ws"] == "WS_5" for p in red), red
    slot = _places(actions, ws="PP")
    assert {p["tag_frame"]: p["at_tag"] for p in slot} == {"tag_8": "tag_102",
                                                           "tag_21": "tag_107"}, slot
    assert all(p["table_pose"] == "Mesa10" for p in slot), slot   # PP -> PP_1
    assert not any("stack_on" in p or "container_color" in p for p in slot), slot
    shelf = _places(actions, ws="SH_1")
    assert sorted(p["tag_frame"] for p in shelf) == ["tag_20", "tag_9"], shelf
    assert all(p["shelf_level"] == "TOP" and p["table_pose"] == "MesaSh" for p in shelf)
    gotos = [a for a in actions if a["kind"] == "goto" and a["ws"] == "PP"]
    assert gotos and all(g["mesa"] == "Mesa10" for g in gotos), gotos


def test_att1_stack(planner, tmp_path):
    actions, log = _plan(planner, "ATT1_seed706244_equipe.yaml", tmp_path)
    _check_invariants(actions)
    stack = _places(actions, ws="ST")
    assert [(p["tag_frame"], p["stack_on"]) for p in stack] == [
        ("tag_14", "tag_200"), ("tag_30", "tag_14")], stack
    assert all(p["table_pose"] == "Mesa15" for p in stack), stack   # ST -> ST_1
    assert "2 empilhamento(s)" in log, log
    shelf = _places(actions, ws="SH_1")
    assert [(p["tag_frame"], p["shelf_level"]) for p in shelf] == [("tag_27", "TOP")], shelf
    assert [p["tag_frame"] for p in _places(actions, container_color="RED")] == ["tag_7"]
    assert [p["tag_frame"] for p in _places(actions, container_color="BLUE")] == ["tag_8"]
    plain = _places(actions, ws="WS_3")
    assert {p["tag_frame"] for p in plain} == {"tag_3", "tag_7"}, plain
    assert [p for p in plain if p["tag_frame"] == "tag_3"][0].get("container_color") is None


def test_amt(planner, tmp_path):
    actions, log = _plan(planner, "AMT_seed164474_equipe.yaml", tmp_path)
    assert [a["kind"] for a in actions] == ["goto", "amt1_sort"], actions
    assert actions[0]["ws"] == "PP" and actions[0]["mesa"] == "Mesa10", actions
    sort = actions[1]
    assert sort["ws"] == "PP" and sort["table_pose"] == "Mesa10", sort
    assert sort["expected_tags"] == [1, 2, 3, 4, 5, 6], sort
    assert sort["target_order"] == [1, 2, 3, 4, 5, 6], sort
    assert sort["direction"] == "left_to_right" and sort["max_onboard"] == 3, sort
    assert "alvo explicito por slot [1,2,3,4,5,6]" in log, log


def test_amt_custom_slot_order(planner, tmp_path):
    """Alvo por slot fora da ordem crescente vira target_order na ordem dos slot_tags."""
    src = (FIXTURES / "AMT_seed164474_equipe.yaml").read_text(encoding="utf-8")
    custom = src.replace(
        "slots: {101: 1, 102: 2, 103: 3, 104: 4, 105: 5, 106: 6, 107: null}",
        "slots: {101: null, 102: 6, 103: 1, 104: 4, 105: 2, 106: 5, 107: 3}")
    assert custom != src
    task = tmp_path / "amt_custom.yaml"
    task.write_text(custom, encoding="utf-8")
    rc, log, actions = _run(planner, task, tmp_path)
    assert rc == 0, log
    assert actions[1]["target_order"] == [6, 1, 4, 2, 5, 3], actions


def test_final(planner, tmp_path):
    actions, log = _plan(planner, "FINAL_seed198117_equipe.yaml", tmp_path)
    _check_invariants(actions)
    assert len(_picks(actions)) == 10 and len(_places(actions)) == 10, actions
    stack = _places(actions, ws="ST")
    assert [(p["tag_frame"], p["stack_on"]) for p in stack] == [
        ("tag_27", "tag_200"), ("tag_16", "tag_27")], stack
    slot = _places(actions, ws="PP")
    assert {p["tag_frame"]: p["at_tag"] for p in slot} == {"tag_29": "tag_102",
                                                           "tag_22": "tag_105"}, slot
    assert sorted(p["tag_frame"] for p in _places(actions, container_color="BLUE")) == [
        "tag_14", "tag_3"]
    assert sorted(p["tag_frame"] for p in _places(actions, container_color="RED")) == [
        "tag_13", "tag_17"]
    shelf_picks = _picks(actions, ws="SH_1")
    assert {p["tag_frame"]: p["shelf_level"] for p in shelf_picks} == {
        "tag_13": "BOTTOM", "tag_22": "BOTTOM"}, shelf_picks
    shelf_places = _places(actions, ws="SH_1")
    assert {p["tag_frame"]: p["shelf_level"] for p in shelf_places} == {
        "tag_26": "TOP", "tag_4": "TOP"}, shelf_places
    assert "10 objeto(s) a transportar" in log, log


@pytest.mark.parametrize("case", sorted(START_STACK_CASES))
def test_start_stack_occupant_is_picked_before_place(planner, tmp_path, case):
    """Pilha INICIAL ocupando a tag_200/base: o ocupante sai antes do place.

    Revisao de 21/10: antes o planner soltava a base na tag_200 (ou um topo
    sobre a base) com o cubo da pilha inicial ainda la. Varias rodadas: o
    planner sorteia empates de rota.
    """
    text, start_stacks, expected = START_STACK_CASES[case]
    for i in range(6):
        rc, log, actions = _plan_text(planner, text, tmp_path, f"{case}_{i}.yaml")
        assert rc == 0, log
        assert "esperam a pilha inicial sair" in log, log
        _check_invariants(actions)
        assert _simulate_stacks(actions, start_stacks) == expected, actions
    # Quem ocupa a tag_200 e' pego ANTES do place na tag_200 (caso base).
    if case == "ocupante_sai":
        kinds = [(a["kind"], a["tag_frame"]) for a in actions if a["kind"] in ("pick", "place")]
        assert kinds.index(("pick", "tag_14")) < kinds.index(("place", "tag_30")), kinds


def test_start_stack_occupant_not_picked_is_a_clear_error(planner, tmp_path):
    """Ocupante da tag_200 sem destino (nao sera pego) -> erro, nao plano errado."""
    text = STACK_HEADER + """objects: [{id: 14}, {id: 30}]
start_state:
  ST: {obj_ids: [14], stack: [14]}
  WS_1: {obj_ids: [], table: []}
  WS_5: {obj_ids: [30], table: [30]}
finish_state:
  ST: {obj_ids: [30], stack: [30]}
  WS_1: {obj_ids: [], table: []}
  WS_5: {obj_ids: [], table: []}
"""
    rc, log, _ = _plan_text(planner, text, tmp_path)
    assert rc == 1, log
    assert "ocupa esse lugar e nao sera pego" in log, log


def test_missing_station_mapping_is_a_clear_error(planner, tmp_path):
    """ST sem ST_1 no ws_table_mapping -> erro dizendo o que acrescentar."""
    with open(WS_MAP, encoding="utf-8") as fh:
        mapping = yaml.safe_load(fh)
    mapping["ws_to_table_pose"].pop("ST_1", None)
    mapping["ws_to_dock_id"].pop("ST_1", None)
    broken = tmp_path / "ws_table_mapping_sem_st.yaml"
    broken.write_text(yaml.safe_dump(mapping), encoding="utf-8")
    rc, log, _ = _run(planner, FIXTURES / "ATT1_seed706244_equipe.yaml", tmp_path, ws_map=broken)
    assert rc == 1, log
    assert "ST_1: Mesa15" in log and "ws_table_mapping.yaml" in log, log


def test_missing_slot_tag_frame_is_a_clear_error(planner, tmp_path):
    """tags_36h11.yaml sem tag_102 -> erro pedindo para acrescentar o id."""
    with open(TAGS_YAML, encoding="utf-8") as fh:
        tags = yaml.safe_load(fh)
    tag = tags["apriltag"]["ros__parameters"]["tag"]
    idx = tag["ids"].index(102)
    for key in ("ids", "frames", "sizes"):
        del tag[key][idx]
    broken = tmp_path / "tags_sem_102.yaml"
    broken.write_text(yaml.safe_dump(tags), encoding="utf-8")
    rc, log, _ = _run(planner, FIXTURES / "ATT2_seed343421_equipe.yaml", tmp_path, tags=broken)
    assert rc == 1, log
    assert "sem frame de tag" in log and "102" in log, log


def test_old_fixtures_still_translate(planner, tmp_path):
    """Formato antigo (constraints inside / sem sub-blocos): fluxo de sempre."""
    for name in ("BMT.yaml", "ATT1.yaml"):
        rc, log, actions = _run(
            planner, OLD_FIXTURES / name, tmp_path, ws_map=OLD_FIXTURES / "ws_table_mapping.yaml")
        assert rc == 0, log
        assert "formato 2026" not in log, log
        assert "AMT1" not in log, log
        _check_invariants(actions)
        assert _picks(actions) and _places(actions), name
    rc, log, actions = _run(planner, OLD_FIXTURES / "BMT.yaml", tmp_path,
                            ws_map=OLD_FIXTURES / "ws_table_mapping.yaml")
    assert rc == 0, log
    assert sorted(p["tag_frame"] for p in _places(actions, container_color="BLUE")) == [
        "tag_2", "tag_4"]
    assert sorted(p["tag_frame"] for p in _places(actions, container_color="RED")) == [
        "tag_1", "tag_7"]


def test_old_amt1_mission_has_no_target_order(planner, tmp_path):
    """missions/task_AMT1_pp1.yaml (formato antigo): amt1_sort sem target_order."""
    mission = WS_ROOT / "missions" / "task_AMT1_pp1.yaml"
    if not mission.is_file():
        pytest.skip(f"{mission} ausente")
    rc, log, actions = _run(planner, mission, tmp_path)
    assert rc == 0, log
    assert [a["kind"] for a in actions] == ["goto", "amt1_sort"], actions
    assert "target_order" not in actions[1], actions
    assert actions[1]["expected_tags"] == [1, 2, 3, 4, 5, 6], actions
    assert "AMT1: ordenar 6 tags na PP_1 (Mesa10, left_to_right, max 3 a bordo)" in log, log
