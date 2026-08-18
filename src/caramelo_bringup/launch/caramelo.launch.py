"""Inicializacao completa do lado PC do Caramelo — um comando, nada de terminal.

    ros2 launch caramelo_bringup caramelo.launch.py

O QUE SOBE (nesta ordem, tudo ligavel/desligavel por argumento):
  - navegacao (use_nav, default true): Nav2 + AMCL + docking + service areas,
    via caramelo_navigation/bringup.launch.py;
  - manipulacao (use_manipulation, default true): move_group do MoveIt,
    commander, pick/place MTC, camera e AprilTag, via
    manip_bringup/manip_stack.launch.py;
  - servidor de missao (use_mission_server, default true);
  - interface grafica (use_gui, default true).

A manipulacao NAO era subida aqui e isso quebrava a missao: o pre-flight
(caramelo_navigation/scripts/caramelo_mission_common.py, PREFLIGHT_SEMPRE)
exige SEMPRE os servidores de move/pick/place. A missao aceitava o pedido,
entrava em pre-flight e ficava esperando para sempre servidores que ninguem
tinha subido — na tela, "a missao travou sem motivo".

Ajustes de navegacao que antes so' existiam editando arquivo agora sao
argumentos: use_keepout, use_docking, nav_profile e costmap_resolution.

Em prova, e' o unico comando que alguem digita: dali em diante tudo acontece
na tela.

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


def caminho_do_manip_stack() -> str:
    """Launch da manipulacao, ou "" se o pacote nao estiver neste workspace.

    Devolver "" em vez de estourar mantem a promessa do "um comando": num PC so'
    de navegacao (ou com a manipulacao ainda nao compilada), o robo sobe e
    navega, e o operador ve um aviso claro em vez de um traceback de import.
    """
    try:
        return os.path.join(
            get_package_share_directory("manip_bringup"), "launch", "manip_stack.launch.py")
    except Exception:  # noqa: BLE001
        return ""


def tem_paredes_virtuais(mapa: str) -> bool:
    """A arena tem keepout_mask.pgm, ou seja: alguem PINTOU paredes virtuais.

    Isto decide o default de use_keepout, e a razao e' um estrago silencioso.
    O default era "false" nos tres launches, e nenhuma tela passava o
    argumento. O operador passava dez minutos pintando a fita do chao e o
    degrau, a ferramenta confirmava "as paredes passam a valer na proxima vez
    que a navegacao for iniciada", e o robo passava por cima -- porque a
    mascara existia no disco e ninguem a carregava.

    O contrario tambem era ruim: ligar keepout por padrao numa arena SEM
    mascara faz o costmap procurar um arquivo que nao existe. Por isso a
    decisao olha o disco em vez de escolher um lado fixo: pintou, vale.
    """
    try:
        return (pasta_dos_mapas() / mapa / "keepout_mask.pgm").is_file()
    except Exception:  # noqa: BLE001
        return False


def generate_launch_description():
    mapa, aviso = mapa_persistido()

    nav_bringup = os.path.join(
        get_package_share_directory("caramelo_navigation"), "launch", "bringup.launch.py")
    mission_server = os.path.join(
        get_package_share_directory("caramelo_navigation"), "launch",
        "mission_server.launch.py")
    gui = os.path.join(
        get_package_share_directory("caramelo_gui"), "launch", "caramelo_gui.launch.py")

    manip_stack = caminho_do_manip_stack()

    # Paredes virtuais: pintou, vale. Ver tem_paredes_virtuais().
    tem_paredes = tem_paredes_virtuais(mapa)
    paredes_por_padrao = "true" if tem_paredes else "false"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration("use_gui")
    use_nav = LaunchConfiguration("use_nav")
    use_manipulation = LaunchConfiguration("use_manipulation")
    use_mission_server = LaunchConfiguration("use_mission_server")
    map_name = LaunchConfiguration("map_name")
    navigation_start_delay = LaunchConfiguration("navigation_start_delay")
    use_keepout = LaunchConfiguration("use_keepout")
    use_docking = LaunchConfiguration("use_docking")
    nav_profile = LaunchConfiguration("nav_profile")
    costmap_resolution = LaunchConfiguration("costmap_resolution")

    acoes = [
        LogInfo(msg=(
            f"Paredes virtuais LIGADAS: a arena '{mapa}' tem uma mascara "
            f"pintada, entao ela vale nesta subida."
            if tem_paredes else
            f"Sem paredes virtuais: a arena '{mapa}' nao tem nenhuma pintada. "
            f"Se voce pintou e esta lendo isto, a mascara nao foi salva."
        )),
        DeclareLaunchArgument(
            "map_name", default_value=mapa,
            description="Arena. O default vem do robot_state.yaml; so passe isto "
                        "para forcar um mapa sem gravar a escolha."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("use_nav", default_value="true"),
        DeclareLaunchArgument(
            "use_manipulation", default_value="true",
            description="Sobe o braco (MoveIt + pick/place + camera). A missao "
                        "EXIGE estes servidores no pre-flight: com isto em false "
                        "ela aceita o pedido e fica esperando para sempre."),
        DeclareLaunchArgument("use_mission_server", default_value="true"),
        DeclareLaunchArgument(
            "navigation_start_delay", default_value="5.0",
            description="Segundos antes de subir o Nav2. Em simulacao vale mais: "
                        "o Gazebo precisa spawnar o robo e ativar os controllers "
                        "antes, senao o Nav2 sobe cego e reclama de extrapolacao."),
        # Ajustes da navegacao repassados ao bringup: antes so' dava para mudar
        # editando caramelo_navigation/launch/bringup.launch.py na mao.
        DeclareLaunchArgument(
            "use_keepout", default_value=paredes_por_padrao,
            description="Liga as paredes virtuais (Keepout Filter RK-04). Exige "
                        "maps/<mapa>/keepout_mask.{yaml,pgm}."),
        DeclareLaunchArgument(
            "use_docking", default_value="true",
            description="Sobe o servidor de encaixe nas estacoes deste mapa."),
        DeclareLaunchArgument(
            "nav_profile", default_value="dwb",
            description="Perfil de navegacao: dwb ou mppi."),
        DeclareLaunchArgument(
            "costmap_resolution", default_value="0.02",
            description="Resolucao dos costmaps em metros/celula. 'map' usa a "
                        "resolucao do proprio mapa."),
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
                "use_keepout": use_keepout,
                "use_docking": use_docking,
                "nav_profile": nav_profile,
                "costmap_resolution": costmap_resolution,
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

    # Manipulacao: sem ela a missao trava no pre-flight (ver o docstring).
    if manip_stack:
        acoes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manip_stack),
            condition=IfCondition(use_manipulation),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ))
    else:
        acoes.append(LogInfo(msg=(
            "O braco nao esta instalado neste computador (pacote manip_bringup "
            "ausente). O robo vai navegar normalmente, mas missoes com pegar e "
            "colocar objetos nao vao comecar.")))

    return LaunchDescription(acoes)
