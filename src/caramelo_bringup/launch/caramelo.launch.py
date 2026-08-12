"""Inicializacao completa do lado PC do Caramelo — um comando, nada de terminal.

    ros2 launch caramelo_bringup caramelo.launch.py

Sobe navegacao (Nav2 + AMCL + docking), o servidor de missao e a interface
grafica. Em prova, e' o unico comando que alguem digita: dali em diante tudo
acontece na tela.

O MAPA NAO E' ARGUMENTO. Ele vem de ~/.config/caramelo/robot_state.yaml, gravado
pela propria GUI quando o operador escolhe a arena. O robo volta a ligar no
ambiente em que estava, e trocar de mapa e' uma escolha na tela -- nao alguem
lembrando o nome certo do mapa num terminal, sob pressao.
(Contrato do arquivo: caramelo_gui/src/bridge/robot_state.hpp.)

Para a simulacao, use o launch que acrescenta o Gazebo por baixo deste:
    ros2 launch caramelo_simulation caramelo_sim.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROBOT_STATE = os.path.join(
    os.path.expanduser("~"), ".config", "caramelo", "robot_state.yaml")

# Usado apenas quando o robo nunca teve mapa definido. Nao e' um default
# silencioso: o launch avisa em voz alta e a GUI abre pedindo a arena.
MAPA_FALLBACK = "arena3_520"


def mapa_persistido() -> tuple:
    """Devolve (nome_do_mapa, aviso). aviso vazio quando veio do arquivo."""
    try:
        with open(ROBOT_STATE, "r", encoding="utf-8") as handle:
            nome = str((yaml.safe_load(handle) or {}).get("map_name", "") or "")
        if nome:
            return nome, ""
    except FileNotFoundError:
        pass
    except Exception as exc:  # noqa: BLE001
        return MAPA_FALLBACK, f"{ROBOT_STATE} ilegivel ({exc}); usando {MAPA_FALLBACK}."
    return MAPA_FALLBACK, (
        f"Nenhum mapa gravado em {ROBOT_STATE}; subindo com {MAPA_FALLBACK}. "
        "Escolha a arena na tela Mapas para gravar a escolha no robo.")


def generate_launch_description():
    mapa, aviso = mapa_persistido()

    nav_bringup = os.path.join(
        get_package_share_directory("caramelo_navigation"), "launch", "bringup.launch.py")
    mission_server = os.path.join(
        get_package_share_directory("caramelo_navigation"), "launch",
        "mission_server.launch.py")
    gui = os.path.join(
        get_package_share_directory("caramelo_gui"), "launch", "caramelo_gui.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration("use_gui")
    use_nav = LaunchConfiguration("use_nav")
    use_mission_server = LaunchConfiguration("use_mission_server")
    map_name = LaunchConfiguration("map_name")

    acoes = [
        DeclareLaunchArgument(
            "map_name", default_value=mapa,
            description="Arena. O default vem do robot_state.yaml; so passe isto "
                        "para forcar um mapa sem gravar a escolha."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("use_nav", default_value="true"),
        DeclareLaunchArgument("use_mission_server", default_value="true"),
    ]
    if aviso:
        acoes.append(LogInfo(msg=aviso))
    acoes.append(LogInfo(msg=["Caramelo subindo com o mapa: ", map_name]))

    acoes += [
        # Navegacao completa: Nav2 + AMCL (global_localization) + docking.
        # O AMCL sobe SEMPRE -- navegar com mapa e' o modo normal do robo, nao
        # uma opcao que alguem precisa lembrar de ligar.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_bringup),
            condition=IfCondition(use_nav),
            launch_arguments={
                "map_name": map_name,
                "use_sim_time": use_sim_time,
                "use_rviz": "false",   # a GUI tem o mapa embutido
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_server),
            condition=IfCondition(use_mission_server),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gui),
            condition=IfCondition(use_gui),
            # Repassar use_sim_time aqui nao e' detalhe: sem isto a GUI roda em
            # tempo de parede e o resto do sistema em tempo de simulacao. Ver o
            # comentario em caramelo_gui/launch/caramelo_gui.launch.py.
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
    ]

    return LaunchDescription(acoes)
