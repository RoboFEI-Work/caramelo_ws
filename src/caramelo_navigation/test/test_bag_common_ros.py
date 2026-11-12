"""Fumaca REAL do rosbag por corrida (caramelo_bag_common + bag_resumo).

Sobe um no' rclpy de teste publicando /cmd_vel_nav (Twist, 20 Hz, vy > vx) e
/odom (Odometry carimbado com o relogio local), grava ~5 s com iniciar_bag()
numa pasta temporaria, para com parar_bag() (SIGINT) e confere metadata.yaml,
corrida.yaml, record.log e a saida JSON do bag_resumo.
So' roda com CARAMELO_BAG_SMOKE=1 (pula no colcon test) e FORCA
ROS_DOMAIN_ID=99: nunca encosta no grafo do robo. Os unicos processos que
cria e para sao o `ros2 bag record` (pid guardado no BagCorrida) e o
bag_resumo (subprocess.run).
Roda com: CARAMELO_BAG_SMOKE=1 python3 -m pytest src/caramelo_navigation/test/test_bag_common_ros.py -q -s
"""
import importlib
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

pytestmark = pytest.mark.skipif(
    os.environ.get("CARAMELO_BAG_SMOKE") != "1",
    reason="fumaca real do rosbag: exporte CARAMELO_BAG_SMOKE=1 (roda em ROS_DOMAIN_ID=99)")


def test_grava_5s_em_dominio_99_e_le_com_bag_resumo(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "99")
    monkeypatch.delenv("CARAMELO_BAG", raising=False)
    bag = importlib.import_module("caramelo_bag_common")

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = rclpy.create_node("caramelo_bag_teste_pub")
    pub_cmd = node.create_publisher(Twist, "/cmd_vel_nav", 10)
    pub_odom = node.create_publisher(Odometry, "/odom", 10)

    def tick():
        tw = Twist()
        tw.linear.x, tw.linear.y = 0.05, 0.20  # "caranguejo" de proposito
        pub_cmd.publish(tw)
        od = Odometry()
        od.header.stamp = node.get_clock().now().to_msg()
        od.header.frame_id = "odom"
        pub_odom.publish(od)

    node.create_timer(0.05, tick)
    fio = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    fio.start()

    bag_dir = tmp_path / "bags"
    corrida = None
    try:
        corrida = bag.iniciar_bag("20990101_000000_actions", "fumaca", "mapa_teste",
                                  tmp_path / "actions.yaml", bag_dir=bag_dir,
                                  nav_profile="teste")
        assert corrida is not None, "iniciar_bag devolveu None (disco? ros2 bag record?)"
        rec = corrida.procs[-1]
        print(f"\n[fumaca] record pid {rec.pid} em {corrida.saida}")
        time.sleep(5.0)
        assert rec.poll() is None, f"record morreu cedo: {corrida.log.read_text()}"
    finally:
        saida = bag.parar_bag(corrida) if corrida is not None else None
        node.destroy_node()
        rclpy.shutdown()
        fio.join(timeout=5.0)

    assert saida is not None and saida.is_dir()
    assert saida.name == "20990101_000000_fumaca_mapa_teste"
    assert (saida / "metadata.yaml").exists(), "record nao fechou o bag (metadata.yaml)"
    assert (saida / "corrida.yaml").exists()
    assert (saida / "record.log").exists()
    assert list(saida.glob("*.mcap")), "sem arquivo .mcap"
    for p in corrida.procs:
        assert p.poll() is not None, f"pid {p.pid} ainda vivo"

    # ros2 bag info so' le o metadata.yaml
    info = subprocess.run(["ros2", "bag", "info", str(saida)], capture_output=True,
                          text=True, timeout=60)
    assert info.returncode == 0, info.stderr
    assert "mcap" in info.stdout and "/cmd_vel_nav" in info.stdout
    print(info.stdout)

    resumo = subprocess.run([sys.executable, str(SCRIPTS / "bag_resumo.py"), str(saida), "--json"],
                            capture_output=True, text=True, timeout=120)
    assert resumo.returncode == 0, resumo.stderr
    dados = json.loads(resumo.stdout)
    assert dados["topicos"]["/cmd_vel_nav"]["msgs"] >= 40
    assert dados["topicos"]["/odom"]["msgs"] >= 40
    assert dados["custom_data"].get("task") == "fumaca"
    assert dados["custom_data"].get("nav_profile") == "teste"
    assert 15.0 <= dados["cmd_vel_nav"]["taxa_hz"] <= 25.0
    # mesmo relogio: offset recv - stamp pequeno e positivo
    off = dados["offset_ms"]["/odom"]
    assert off["n"] >= 40 and -5.0 < off["mediana"] < 50.0
    assert len(dados["goals"]) == 1 and dados["goals"][0]["fracao_lateral"] > 0.95

    legivel = subprocess.run([sys.executable, str(SCRIPTS / "bag_resumo.py"), str(saida)],
                             capture_output=True, text=True, timeout=120)
    assert legivel.returncode == 0, legivel.stderr
    print(legivel.stdout)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q", "-s"]))
