import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Sobe o agregador de saude (/diagnostics) e a GUI. O diagnostico e' um node
    # separado (usavel headless tambem — R4); a GUI so' o consome.
    use_diagnostics = LaunchConfiguration("use_diagnostics")
    use_sim_time = LaunchConfiguration("use_sim_time")

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("caramelo_diagnostics"),
                "launch", "diagnostics.launch.py",
            )
        ),
        condition=IfCondition(use_diagnostics),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Sem "name=": o processo tem 3 nos internos (bridge, rviz, markers) e o
    # remap de nome unico os colidia ("Publisher already registered").
    gui = Node(
        package="caramelo_gui",
        executable="caramelo_gui",
        output="screen",
        # use_sim_time e' OBRIGATORIO em simulacao e vale para os tres nos do
        # processo (bridge, rviz, markers) porque o launch grava o override sob
        # o curinga /**.
        #
        # Sem isso a GUI roda em tempo de PAREDE enquanto o resto do sistema roda
        # em tempo de SIMULACAO. O sintoma nao parece de relogio: o filtro de TF
        # do RViz nunca casa, loga "Message Filter dropping message ... queue is
        # full" em rajada, o processo satura a CPU, o EKF passa a perder deadline
        # e o lifecycle_manager derruba o map_server por bond timeout. Um sistema
        # nesse estado tambem nao responde ao SIGINT a tempo, e o ros2 launch
        # precisa escalar para SIGTERM/SIGKILL no encerramento.
        parameters=[{"use_sim_time": use_sim_time}],
        # Silencia o DEBUG do RViz embutido: antes de localizar (sem TF
        # map->base) ele logava "Unable to transform marker message" por
        # marcador POR FRAME e inundava o terminal. E benigno e some ao
        # localizar; em warn o terminal fica legivel.
        arguments=[
            "--ros-args",
            "--log-level", "rviz_common:=warn",
            "--log-level", "rviz_rendering:=warn",
            "--log-level", "rviz_default_plugins:=warn",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_diagnostics",
            default_value="true",
            description="Sobe o caramelo_diagnostics junto com a GUI",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="true em simulacao: a GUI precisa do MESMO relogio do resto",
        ),
        diagnostics,
        gui,
    ])
