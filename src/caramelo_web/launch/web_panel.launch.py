"""Painel web de engenharia: tres processos, um comando.

    ros2 launch caramelo_web web_panel.launch.py

  8080  caramelo_web_server   pagina, API REST e WebSocket de estado
  9090  rosbridge_websocket   transporte generico de topicos/services/actions
  8081  web_video_server      MJPEG da camera, SOB DEMANDA

Acesso de outro computador da rede: http://<ip-do-pc>:8080

A porta 8000 e' evitada de proposito -- ja' esta ocupada pelo mesh server de
STLs da Raspberry.

A camera vem DESLIGADA por padrao. Num Wi-Fi de competicao, video continuo tira
banda da navegacao; e o painel e' ferramenta de desenvolvimento, nao de prova.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    porta = LaunchConfiguration("porta")
    host = LaunchConfiguration("host")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rosbridge = LaunchConfiguration("use_rosbridge")
    use_camera = LaunchConfiguration("use_camera")

    return LaunchDescription([
        DeclareLaunchArgument("porta", default_value="8080"),
        DeclareLaunchArgument(
            "host", default_value="0.0.0.0",
            description="0.0.0.0 aceita conexao de outro computador da rede"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rosbridge", default_value="true"),
        DeclareLaunchArgument(
            "use_camera", default_value="false",
            description="MJPEG na 8081. Desligado por padrao: video continuo "
                        "tira banda da navegacao."),

        Node(
            package="caramelo_web",
            executable="caramelo_web_server",
            name="caramelo_web",
            output="screen",
            arguments=["--host", host, "--port", porta],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            condition=IfCondition(use_rosbridge),
            parameters=[{"use_sim_time": use_sim_time, "port": 9090}],
        ),
        Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            output="screen",
            condition=IfCondition(use_camera),
            parameters=[{"use_sim_time": use_sim_time, "port": 8081}],
        ),
    ])
