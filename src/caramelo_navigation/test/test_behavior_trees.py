"""BTs do perfil DWB (2026-11-13): o FollowPath recebe o path cru do lattice.

O RotationShim usa o yaw do plano (use_path_orientations true); qualquer
smoother do Nav2 1.3.13 reescreve esse yaw com a tangente. Este teste falha se
alguem recolocar o SmoothPath ou usar um no que o Nav2 instalado nao conhece.
"""
import glob
import os
import xml.etree.ElementTree as ET

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))
BTS = sorted(glob.glob(os.path.join(AQUI, "..", "behavior_tree", "caramelo_lattice_*dwb*.xml")))
NOS_NAV2 = "/opt/ros/jazzy/share/nav2_behavior_tree/nav2_tree_nodes.xml"
ESTRUTURA = {"root", "BehaviorTree", "Sequence", "Fallback", "ReactiveSequence",
             "ReactiveFallback", "Inverter"}


@pytest.mark.parametrize("arquivo", BTS, ids=os.path.basename)
def test_bt_dwb_entrega_o_path_do_lattice(arquivo):
    nos = list(ET.parse(arquivo).getroot().iter())
    tags = {n.tag for n in nos}
    assert "SmoothPath" not in tags
    follow = [n for n in nos if n.tag == "FollowPath"]
    assert len(follow) == 1 and follow[0].get("path") == "{path}"
    planner = [n for n in nos if n.tag.startswith("ComputePath")]
    assert planner and all(n.get("path") == "{path}" for n in planner)
    # memoria de obstaculo: so' 1 limpeza total do global (ultimo recurso)
    assert sum(n.tag == "ClearEntireCostmap" for n in nos) == 1
    if os.path.exists(NOS_NAV2):
        conhecidos = {n.get("ID") for n in ET.parse(NOS_NAV2).getroot().iter() if n.get("ID")}
        assert tags - ESTRUTURA - conhecidos == set()


def test_ha_bts_dwb():
    assert len(BTS) == 4
