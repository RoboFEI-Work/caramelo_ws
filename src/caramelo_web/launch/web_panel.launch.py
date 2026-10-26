"""Painel web de engenharia: tres processos, um comando.

    ros2 launch caramelo_web web_panel.launch.py

  8080  caramelo_web_server   pagina, API REST e WebSocket de estado
  9090  rosbridge_websocket   transporte generico -- DESLIGADO por padrao
  8081  web_video_server      MJPEG da camera, SOB DEMANDA

Acesso de outro computador da rede: o proprio servidor IMPRIME o link completo
(com a chave de acesso) assim que sobe. Sem a chave o painel abre so' para
olhar: nao comanda o robo, nao abre terminal e nao espelha a tela.

Para fixar a chave (util para deixar um favorito ou um QR colado no robo):

    ros2 launch caramelo_web web_panel.launch.py token:=umaChaveQualquer

A porta 8000 e' evitada de proposito -- ja' esta ocupada pelo mesh server de
STLs da Raspberry.

A camera vem DESLIGADA por padrao. Num Wi-Fi de competicao, video continuo tira
banda da navegacao; e o painel e' ferramenta de desenvolvimento, nao de prova.

O rosbridge tambem vem DESLIGADO, e a razao aqui nao e' banda -- e' o robo.
Ele publica o grafo ROS inteiro na 9090 sem chave nenhuma: qualquer topico
gravavel, para quem estiver na mesma rede. Um deles e'
/mecanum_controller/reference, que entra DEPOIS do twist_mux e do
collision_monitor -- ou seja, sem limite de velocidade e sem parada por
obstaculo. Como o firmware do ESC le' ausencia de PWM como re' a fundo e a base
nao tem botao de emergencia de hardware, deixar isso ligado por padrao e' risco
fisico, nao de dados. O painel nao precisa dele: fala com a propria API, que
passa pelos caminhos com trava.
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
    token = LaunchConfiguration("token")

    return LaunchDescription([
        DeclareLaunchArgument("porta", default_value="8080"),
        DeclareLaunchArgument(
            "host", default_value="0.0.0.0",
            description="0.0.0.0 aceita conexao de outro computador da rede"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "use_rosbridge", default_value="false",
            description="Transporte generico do grafo ROS na 9090. Deixe "
                        "desligado a menos que va' usar uma ferramenta de "
                        "terceiros que fale rosbridge; o painel nao usa."),
        DeclareLaunchArgument(
            "token", default_value="",
            description="Fixa a chave de acesso do painel. Vazio sorteia uma "
                        "nova a cada arranque (recomendado)."),
        DeclareLaunchArgument(
            "use_camera", default_value="false",
            description="MJPEG na 8081. Desligado por padrao: video continuo "
                        "tira banda da navegacao."),

        Node(
            package="caramelo_web",
            executable="caramelo_web_server",
            name="caramelo_web",
            output="screen",
            arguments=["--host", host, "--port", porta, "--token", token],
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
