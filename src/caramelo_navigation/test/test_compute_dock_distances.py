"""Testes do compute_dock_distances num mapa sintetico pequeno, sem ROS ativo.

Importa o script como modulo (mesmo truque do test_nudge_math.py); o import
nao puxa nada de ROS (caramelo_docking_common so e' importado no main com
--map-name). Mapa: sala 2 x 1 m a 0,05 m/celula com uma parede vertical no
meio e uma passagem embaixo - o caminho entre os dois lados tem que
contornar a parede (bem maior que a reta).

Como rodar (da raiz do workspace):
    python3 -m pytest src/caramelo_navigation/test/test_compute_dock_distances.py -q
"""
import importlib.util
import math
import os

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "scripts", "compute_dock_distances.py")


def _load():
    spec = importlib.util.spec_from_file_location("compute_dock_distances", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


cdd = _load()

RES = 0.05
COLS, ROWS = 40, 20          # 2,0 m x 1,0 m
ORIGIN = [0.0, 0.0, 0.0]


def _write_pgm(path, image):
    rows, cols = image.shape
    with open(path, "wb") as fh:
        fh.write(f"P5\n# teste\n{cols} {rows}\n255\n".encode("ascii"))
        fh.write(image.astype(np.uint8).tobytes())


def _synthetic_map(tmp_path, with_pgm=True, gap=True):
    """Mapa com parede vertical no meio e passagem embaixo.

    Parede em x = 1,0 m (coluna 20) do topo ate 0,20 m do chao (passagem
    embaixo quando gap=True); bordas ocupadas. Docks: START a esquerda, WS1 a
    direita, FINISH a direita embaixo.
    """
    image = np.full((ROWS, COLS), 254, dtype=np.uint8)   # livre
    image[0, :] = 0
    image[-1, :] = 0
    image[:, 0] = 0
    image[:, -1] = 0
    wall_rows = ROWS - 4 if gap else ROWS          # linhas 0..15 (y de 1,0 ate 0,20)
    image[0:wall_rows, 20] = 0
    if with_pgm:
        _write_pgm(tmp_path / "map.pgm", image)
    (tmp_path / "map.yaml").write_text(yaml.safe_dump({
        "image": "map.pgm", "mode": "trinary", "resolution": RES, "origin": ORIGIN,
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196}), encoding="utf-8")
    staging = {"staging_x_offset": -0.10, "staging_yaw_offset": 0.0}
    (tmp_path / "docking.yaml").write_text(yaml.safe_dump({
        "dock_plugins": {
            "caramelo_start_dock": dict(staging), "caramelo_finish_dock": dict(staging),
            "caramelo_front_dock": dict(staging)},
        "docks": {
            # pose do dock + staging recuado 0,10 m ao longo do yaw
            "START": {"type": "caramelo_start_dock", "frame": "map",
                      "pose": [0.50, 0.70, 0.0], "id": "START"},
            "WS1": {"type": "caramelo_front_dock", "frame": "map",
                    "pose": [1.60, 0.70, math.pi], "id": "WS1"},       # staging em x=1,70
            "FINISH": {"type": "caramelo_finish_dock", "frame": "map",
                       "pose": [1.50, 0.30, 0.0], "id": "FINISH"},
            "ST1": {"type": "caramelo_front_dock", "frame": "map",
                    "pose": [0.0, 0.0, 0.0], "id": "ST1"},            # placeholder
        }}), encoding="utf-8")
    return tmp_path


def test_load_pgm_binary_and_ascii(tmp_path):
    image = np.arange(12, dtype=np.uint8).reshape(3, 4) * 20
    _write_pgm(tmp_path / "bin.pgm", image)
    assert np.array_equal(cdd.load_pgm(tmp_path / "bin.pgm"), image)
    ascii_text = "P2\n4 3\n255\n" + " ".join(str(v) for v in image.flatten()) + "\n"
    (tmp_path / "ascii.pgm").write_text(ascii_text)
    assert np.array_equal(cdd.load_pgm(tmp_path / "ascii.pgm"), image)


def test_classify_cells_trinary():
    image = np.array([[0, 205, 254]], dtype=np.uint8)
    occupied, unknown = cdd.classify_cells(image, {"negate": 0, "occupied_thresh": 0.65,
                                                   "free_thresh": 0.196})
    assert occupied.tolist() == [[True, False, False]]
    assert unknown.tolist() == [[False, True, False]]


def test_inflate_disk():
    obstacles = np.zeros((7, 7), dtype=bool)
    obstacles[3, 3] = True
    out = cdd.inflate(obstacles, 2)
    assert out[3, 1] and out[1, 3] and out[5, 3] and out[3, 5]      # raio 2 nos eixos
    assert out[2, 2] and out[4, 4]                                   # diagonal 1
    assert not out[1, 1] and not out[5, 5]                           # diagonal 2 (2,83 > 2)
    assert not out[0, 3] and not out[3, 0]
    assert np.array_equal(cdd.inflate(obstacles, 0), obstacles)


def test_world_to_cell_top_row_is_max_y():
    assert cdd.world_to_cell(0.0, 0.0, ORIGIN, RES, ROWS) == (ROWS - 1, 0)
    assert cdd.world_to_cell(1.0, 1.0 - 1e-9, ORIGIN, RES, ROWS) == (0, 20)
    assert cdd.world_to_cell(0.5, 0.7, ORIGIN, RES, ROWS) == (ROWS - 1 - 14, 10)


def test_nearest_free_cell_snaps_out_of_obstacle():
    free = np.ones((5, 5), dtype=bool)
    free[2, 2] = False
    free[2, 3] = False
    assert cdd.nearest_free_cell(free, (2, 2), 3) in ((1, 2), (3, 2), (2, 1))
    assert cdd.nearest_free_cell(free, (1, 1), 3) == (1, 1)
    assert cdd.nearest_free_cell(np.zeros((3, 3), dtype=bool), (1, 1), 2) is None


def test_dijkstra_straight_line_and_diagonal():
    free = np.ones((5, 5), dtype=bool)
    d = cdd.dijkstra_8(free, (0, 0), [(0, 4), (4, 4)], 0.1)
    assert d[(0, 4)] == pytest.approx(0.4)
    assert d[(4, 4)] == pytest.approx(4 * math.sqrt(2) * 0.1)
    # Parede completa: alvo inalcancavel fica fora do resultado.
    free[:, 2] = False
    assert cdd.dijkstra_8(free, (0, 0), [(0, 4)], 0.1) == {}


def test_path_goes_around_wall_and_is_symmetric(tmp_path):
    folder = _synthetic_map(tmp_path)
    result = cdd.compute_distances(folder, inflation_m=0.05)
    assert result["method"] == "astar8"
    assert sorted(result["docks"]) == ["FINISH", "START", "WS1"]
    assert result["skipped_docks"] == ["ST1"]
    assert result["euclidean_pairs"] == []
    dist = result["distances_m"]
    straight = math.hypot(1.70 - 0.40, 0.0)            # staging START x=0,40, WS1 x=1,70
    # Contorna a parede por baixo: desce 0,6 m, cruza, sobe 0,6 m - com passos
    # diagonais da ~1,8 m (contra 1,3 m em linha reta).
    assert dist["START"]["WS1"] > straight + 0.4
    assert dist["START"]["WS1"] < 2.5
    assert dist["START"]["WS1"] == dist["WS1"]["START"]
    assert dist["START"]["START"] == 0.0
    # FINISH esta abaixo da parede: caminho curto.
    assert dist["WS1"]["FINISH"] < dist["START"]["WS1"]
    # Inflacao maior fecha a passagem (0,20 m): par cai na euclidiana.
    closed = cdd.compute_distances(folder, inflation_m=0.15)
    assert ["START", "WS1"] in closed["euclidean_pairs"]
    assert closed["distances_m"]["START"]["WS1"] == pytest.approx(straight, abs=1e-6)


def test_euclidean_fallback_without_pgm(tmp_path):
    folder = _synthetic_map(tmp_path, with_pgm=False)
    result = cdd.compute_distances(folder)
    assert result["method"] == "euclidean"
    assert result["distances_m"]["START"]["WS1"] == pytest.approx(1.30)
    assert result["notes"]


def test_write_and_reload_yaml(tmp_path):
    folder = _synthetic_map(tmp_path)
    result = cdd.compute_distances(folder, inflation_m=0.05)
    out = cdd.write_dock_distances(folder, result)
    assert out == folder / "dock_distances.yaml"
    with out.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert data["schema"] == 1
    assert data["map"] == folder.name
    assert data["distances_m"]["START"]["WS1"] == result["distances_m"]["START"]["WS1"]
    assert "compute_dock_distances" in out.read_text(encoding="utf-8").splitlines()[1]


def test_main_dry_run_and_write(tmp_path, capsys):
    folder = _synthetic_map(tmp_path)
    assert cdd.main(["--map-folder", str(folder), "--dry-run"]) == 0
    assert not (folder / "dock_distances.yaml").exists()
    assert "START" in capsys.readouterr().out
    assert cdd.main(["--map-folder", str(folder), "--inflation", "0.05"]) == 0
    assert (folder / "dock_distances.yaml").exists()
    assert cdd.main(["--map-folder", str(tmp_path / "nao_existe")]) == 1
