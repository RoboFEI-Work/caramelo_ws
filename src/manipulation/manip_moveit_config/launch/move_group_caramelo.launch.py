"""move_group para o robo COMBINADO Caramelo (base movel + manipulador).

Usa a URDF combinada do pacote caramelo_description (a mesma que a Pi publica
em /robot_description) e a SRDF config/caramelo.srdf deste pacote. Os demais
YAMLs (kinematics, joint_limits, moveit_controllers, ...) continuam vindo do
manip_moveit_config, iguais aos da bancada standalone.
"""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    urdf_xacro = os.path.join(
        get_package_share_directory("caramelo_description"),
        "urdf",
        "robots",
        "robot.urdf.xacro",
    )
    # O MoveItConfigsBuilder engole silenciosamente URDF inexistente (só loga um
    # warning e segue sem robot_description) — checar antes e falhar alto.
    if not os.path.isfile(urdf_xacro):
        raise RuntimeError(
            "URDF combinada do Caramelo nao encontrada: {}\n"
            "Compile/instale o pacote caramelo_description e faca source do "
            "workspace correspondente antes de lancar este launch.".format(urdf_xacro)
        )

    moveit_config = (
        MoveItConfigsBuilder("manip", package_name="manip_moveit_config")
        .robot_description(
            file_path=urdf_xacro,
            mappings={
                "use_manipulator": "true",
                "use_realsense": "true",
                "use_mock_components": "true",
                # primitive: colisao por box/cilindro. Em modo package a colisao
                # usava as malhas STL completas (1,36 MILHAO de triangulos, 82%
                # nas 4 rodas) — dava "too many vertices" e deixava o MoveIt
                # inteiro lento. Mesma arvore de links/joints; SRDF inalterada.
                "visual_mode": "primitive",
                "mesh_uri_prefix": "package://caramelo_description",
            },
        )
        .robot_description_semantic(file_path="config/caramelo.srdf")
        .to_moveit_configs()
    )

    # Capability p/ MoveIt Task Constructor: generate_move_group_launch declara o
    # LaunchArgument "capabilities" com default = move_group_capabilities["capabilities"];
    # o builder do Jazzy nao tem setter, entao setamos o campo direto no config —
    # assim a capability entra por padrao e o caller ainda pode sobrescrever via
    # <arg name="capabilities" .../> (como manip_pc.launch.xml faz com move_group.launch.py).
    moveit_config.move_group_capabilities["capabilities"] = (
        "move_group/ExecuteTaskSolutionCapability"
    )

    return generate_move_group_launch(moveit_config)
