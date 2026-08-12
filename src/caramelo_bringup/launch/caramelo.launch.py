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


def pasta_dos_mapas() -> str:
    """Arvore FONTE dos mapas, a mesma que a GUI usa (nao o share instalado)."""
    try:
        share = get_package_share_directory("caramelo_mapping")
    except Exception:  # noqa: BLE001
        return os.path.join(os.path.expanduser("~"), "caramelo_ws", "src",
                            "caramelo_mapping", "maps")
    idx = share.find("/install/")
    if idx > 0:
        fonte = os.path.join(share[:idx], "src", "caramelo_mapping", "maps")
        if os.path.isdir(fonte):
            return fonte
    return os.path.join(share, "maps")


def arenas_disponiveis() -> list:
    raiz = pasta_dos_mapas()
    if not os.path.isdir(raiz):
        return []
    return sorted(
        nome for nome in os.listdir(raiz)
        if not nome.startswith(".") and os.path.isfile(os.path.join(raiz, nome, "map.yaml")))


def mapa_persistido() -> tuple:
    """Devolve (nome_do_mapa, aviso). aviso vazio quando esta tudo certo.

    Confere se a arena EXISTE neste workspace. O nome fica gravado em
    ~/.config/caramelo (global), mas os mapas vivem dentro de cada workspace --
    o mapa 'warehouse', por exemplo, so' existe no de simulacao. Sem esta
    checagem, o map_server nao consegue configurar, o lifecycle_manager aborta a
    subida inteira e a unica pista e' um erro de transicao de estado que nao diz
    nada sobre mapa. Foi exatamente assim que a navegacao "nao subiu".
    """
    disponiveis = arenas_disponiveis()
    nome = ""
    try:
        with open(ROBOT_STATE, "r", encoding="utf-8") as handle:
            nome = str((yaml.safe_load(handle) or {}).get("map_name", "") or "")
    except FileNotFoundError:
        pass
    except Exception as exc:  # noqa: BLE001
        nome = ""
        aviso_leitura = f"{ROBOT_STATE} ilegivel ({exc})."
        if disponiveis:
            return disponiveis[0], f"{aviso_leitura} Subindo com {disponiveis[0]}."
        return MAPA_FALLBACK, aviso_leitura

    if nome and nome in disponiveis:
        return nome, ""

    if nome:
        escolhido = disponiveis[0] if disponiveis else MAPA_FALLBACK
        return escolhido, (
            f"A arena gravada no robo ('{nome}') NAO existe em {pasta_dos_mapas()}. "
            f"Arenas disponiveis: {', '.join(disponiveis) or 'nenhuma'}. "
            f"Subindo com '{escolhido}'. Escolha a arena certa na tela Mapas.")

    escolhido = disponiveis[0] if disponiveis else MAPA_FALLBACK
    return escolhido, (
        f"Nenhuma arena gravada em {ROBOT_STATE}; subindo com '{escolhido}'. "
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
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")

    acoes = [
        DeclareLaunchArgument(
            "map_name", default_value=mapa,
            description="Arena. O default vem do robot_state.yaml; so passe isto "
                        "para forcar um mapa sem gravar a escolha."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("use_nav", default_value="true"),
        DeclareLaunchArgument("use_mission_server", default_value="true"),
        DeclareLaunchArgument(
            "navigation_start_delay", default_value="5.0",
            description="Segundos antes de subir o Nav2. Em simulacao vale mais: "
                        "o Gazebo precisa spawnar o robo e ativar os controllers "
                        "antes, senao o Nav2 sobe cego e reclama de extrapolacao."),
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
                "navigation_start_delay": navigation_start_delay,
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
