# Stack de manipulacao no PC (porta do manip_pc.launch.xml para Python).
#
# SEMPRE sobe: move_group (com ExecuteTaskSolutionCapability p/ MTC) + manip_commander.
# Opcionais: camera RealSense, AprilTag, audio (Piper), pick/place MTC, RViz.
#
# use_mock_hardware=true: sobe RSP + ros2_control com mock_components NO PC
# (braco simulado, SEM base mecanum/navegacao). Em modo real (default) o
# RSP/ros2_control roda na Pi (raspberry_bringup) e aqui NAO sobe nada disso.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Composicao ---
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_camera = LaunchConfiguration("use_camera")
    use_apriltag = LaunchConfiguration("use_apriltag")
    camera_profile = LaunchConfiguration("camera_profile")
    use_audio = LaunchConfiguration("use_audio")
    use_pick_place = LaunchConfiguration("use_pick_place")
    use_rviz_manip = LaunchConfiguration("use_rviz_manip")
    launch_pick_telemetry = LaunchConfiguration("launch_pick_telemetry")

    manip_bringup_share = get_package_share_directory("manip_bringup")
    moveit_config_share = get_package_share_directory("manip_moveit_config")
    manip_audio_share = get_package_share_directory("manip_audio")

    # Estado dos containers e lock do braco: compartilhados entre pick e place.
    container_state_file = os.path.join(
        os.path.expanduser("~"), ".config", "caramelo", "container_states.yaml"
    )
    manipulator_lock_file = "/tmp/caramelo_manip_action.lock"

    apriltag_params_file = os.path.join(
        manip_bringup_share, "config", "tags_36h11.yaml"
    )

    declared_args = [
        # --- Composicao ---
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Sobe RSP + ros2_control mock do braco no PC (sem base mecanum)",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Inicia a RealSense (rs_launch.py) sem publicar TF",
        ),
        DeclareLaunchArgument(
            "camera_profile",
            # 640x480x5: unico modo estavel em porta USB 2 (2026-08-07: em 2.1
            # a 15fps dava "Incomplete video frame"/"Frame Corrupted" e o driver
            # reiniciava em loop, derrubando ate a navegacao). Em USB 3 passe
            # camera_profile:=640x480x15 (ou mais).
            default_value="640x480x5",
            description="Perfil WxHxFPS dos streams color e depth da RealSense",
        ),
        DeclareLaunchArgument(
            "use_apriltag",
            default_value="true",
            description="Inicia o apriltag_node na imagem colorida da RealSense",
        ),
        DeclareLaunchArgument(
            "use_audio",
            default_value="true",
            description="Inicia o speech_node (Piper, efeito dog)",
        ),
        DeclareLaunchArgument(
            "use_pick_place",
            default_value="true",
            description="Inicia mtc_pick_action_node e mtc_place_action_node",
        ),
        DeclareLaunchArgument(
            "use_rviz_manip",
            default_value="false",
            description="Abre RViz com o config de manipulacao",
        ),
        DeclareLaunchArgument(
            "launch_pick_telemetry",
            default_value="false",
            description="Inicia o pick_telemetry_plotter",
        ),
        # --- Audio (defaults identicos ao manip_pc.launch.xml) ---
        DeclareLaunchArgument("piper_executable", default_value="auto"),
        DeclareLaunchArgument(
            "piper_model",
            default_value=os.path.join(
                manip_audio_share, "models", "pt_BR-jeff", "pt_BR-jeff-medium.onnx"
            ),
        ),
        DeclareLaunchArgument("speech_player_executable", default_value="auto"),
        # --- IK/grasp (defaults identicos ao manip_pc.launch.xml) ---
        DeclareLaunchArgument("verify_grasp_effort", default_value="true"),
        DeclareLaunchArgument("grasp_min_effort_nm", default_value="0.15"),
        DeclareLaunchArgument("grasp_min_effort_increase_nm", default_value="0.05"),
        DeclareLaunchArgument("grasp_retry_attempts", default_value="2"),
        DeclareLaunchArgument("switch_ik_after_failed_grasp", default_value="true"),
        DeclareLaunchArgument("attempt_1_ik_solver", default_value="pick_ik/PickIkPlugin"),
        DeclareLaunchArgument(
            "attempt_2_ik_solver",
            default_value="kdl_kinematics_plugin/KDLKinematicsPlugin",
        ),
        DeclareLaunchArgument("attempt_3_ik_solver", default_value="pick_ik/PickIkPlugin"),
        DeclareLaunchArgument("attempt_3_ik_solve_type", default_value="Distance"),
        DeclareLaunchArgument("attempt_3_ik_position_only", default_value="false"),
        DeclareLaunchArgument("attempt_3_ik_position_threshold", default_value="0.010"),
        DeclareLaunchArgument("attempt_3_ik_orientation_threshold", default_value="0.90"),
        DeclareLaunchArgument("attempt_3_goal_position_tolerance", default_value="0.010"),
        DeclareLaunchArgument("attempt_3_goal_orientation_tolerance", default_value="0.80"),
        DeclareLaunchArgument("skip_failed_pick_after_retries", default_value="true"),
        DeclareLaunchArgument("skip_missing_place_tag", default_value="true"),
        DeclareLaunchArgument("use_camera_alignment_retry", default_value="true"),
        DeclareLaunchArgument("fallback_ik_solver", default_value="pick_ik/PickIkPlugin"),
        DeclareLaunchArgument("fallback_ik_mode", default_value="local"),
        DeclareLaunchArgument("fallback_ik_solve_type", default_value="Distance"),
        DeclareLaunchArgument("fallback_ik_position_only", default_value="true"),
        DeclareLaunchArgument("reset_container_states", default_value="true"),
    ]

    # --- move_group (sempre) ---
    # Mesmo mecanismo do manip_pc.launch.xml: o launch gerado pelo
    # generate_move_group_launch declara o arg "capabilities" e o injeta como
    # parametro do move_group; basta repassar como launch argument.
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "move_group_caramelo.launch.py")
        ),
        launch_arguments={
            "capabilities": "move_group/ExecuteTaskSolutionCapability",
        }.items(),
    )

    # --- Commander (sempre; exec "commmander" com tres m, como no pacote) ---
    commander = Node(
        package="manip_commander",
        executable="commmander",
    )

    # --- Pick/Place MTC ---
    # URDF primitive LOCAL como parametro: sem ele, o RDFLoader dos nos MTC cai
    # no topico /robot_description da Pi (malhas via http://raspberrypi:8000 —
    # ~68 MB de STL baixados e parseados VARIAS vezes por no; era a demora de
    # minutos na largada). Parametro tem precedencia sobre topico. A SRDF
    # continua vindo do topico /robot_description_semantic do move_group.
    primitive_robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("caramelo_description"),
                "urdf", "robots", "robot.urdf.xacro",
            ]),
            " use_manipulator:=true",
            " use_realsense:=true",
            " use_mock_components:=true",
            " visual_mode:=primitive",
        ]),
        value_type=str,
    )

    # SRDF LOCAL como parametro (2026-08-07, achado do teste mock): quando o
    # move_group cai, o RDFLoader dos nos MTC esperava a SRDF via topico por
    # 10s e o construtor do MoveGroupInterface ABORTAVA O PROCESSO (SIGABRT em
    # thread interna). Com a SRDF local, o modelo monta sempre e a falta do
    # move_group vira falha limpa da sonda/timeout de action server.
    with open(
        os.path.join(moveit_config_share, "config", "caramelo.srdf"),
        "r", encoding="utf-8",
    ) as srdf_file:
        robot_description_semantic_content = srdf_file.read()

    # respawn (2026-08-07, achado do teste mock): se o no de pick/place morrer
    # (ex.: SIGABRT interno do MoveIt), renasce sozinho em vez de deixar a
    # missao sem action server ate reiniciar o stack na mao.
    pick_action = Node(
        respawn=True,
        respawn_delay=3.0,
        package="manip_task_execution",
        executable="mtc_pick_action_node",
        output="screen",
        condition=IfCondition(use_pick_place),
        parameters=[{
            "reset_container_states_on_start": ParameterValue(
                LaunchConfiguration("reset_container_states"), value_type=bool),
            "verify_grasp_effort": ParameterValue(
                LaunchConfiguration("verify_grasp_effort"), value_type=bool),
            "grasp_min_effort_nm": ParameterValue(
                LaunchConfiguration("grasp_min_effort_nm"), value_type=float),
            "grasp_min_effort_increase_nm": ParameterValue(
                LaunchConfiguration("grasp_min_effort_increase_nm"), value_type=float),
            "grasp_retry_attempts": ParameterValue(
                LaunchConfiguration("grasp_retry_attempts"), value_type=int),
            "switch_ik_after_failed_grasp": ParameterValue(
                LaunchConfiguration("switch_ik_after_failed_grasp"), value_type=bool),
            "attempt_1_ik_solver": LaunchConfiguration("attempt_1_ik_solver"),
            "attempt_2_ik_solver": LaunchConfiguration("attempt_2_ik_solver"),
            "attempt_3_ik_solver": LaunchConfiguration("attempt_3_ik_solver"),
            "attempt_3_ik_solve_type": LaunchConfiguration("attempt_3_ik_solve_type"),
            "attempt_3_ik_position_only": ParameterValue(
                LaunchConfiguration("attempt_3_ik_position_only"), value_type=bool),
            "attempt_3_ik_position_threshold": ParameterValue(
                LaunchConfiguration("attempt_3_ik_position_threshold"), value_type=float),
            "attempt_3_ik_orientation_threshold": ParameterValue(
                LaunchConfiguration("attempt_3_ik_orientation_threshold"), value_type=float),
            "attempt_3_goal_position_tolerance": ParameterValue(
                LaunchConfiguration("attempt_3_goal_position_tolerance"), value_type=float),
            "attempt_3_goal_orientation_tolerance": ParameterValue(
                LaunchConfiguration("attempt_3_goal_orientation_tolerance"), value_type=float),
            "fallback_ik_solver": LaunchConfiguration("fallback_ik_solver"),
            "fallback_ik_mode": LaunchConfiguration("fallback_ik_mode"),
            "fallback_ik_solve_type": LaunchConfiguration("fallback_ik_solve_type"),
            "fallback_ik_position_only": ParameterValue(
                LaunchConfiguration("fallback_ik_position_only"), value_type=bool),
            "skip_failed_pick_after_retries": ParameterValue(
                LaunchConfiguration("skip_failed_pick_after_retries"), value_type=bool),
            "use_camera_alignment_retry": ParameterValue(
                LaunchConfiguration("use_camera_alignment_retry"), value_type=bool),
            "container_state_file": container_state_file,
            "manipulator_lock_file": manipulator_lock_file,
            "robot_description": primitive_robot_description,
            "robot_description_semantic": robot_description_semantic_content,
        }],
    )

    place_action = Node(
        respawn=True,
        respawn_delay=3.0,
        package="manip_task_execution",
        executable="mtc_place_action_node",
        output="screen",
        condition=IfCondition(use_pick_place),
        parameters=[{
            "skip_missing_place_tag": ParameterValue(
                LaunchConfiguration("skip_missing_place_tag"), value_type=bool),
            "container_state_file": container_state_file,
            "manipulator_lock_file": manipulator_lock_file,
            "robot_description": primitive_robot_description,
            "robot_description_semantic": robot_description_semantic_content,
        }],
    )

    # --- Camera RealSense ---
    # publish_tf:=false: o TF da camera vem do URDF do Caramelo (RSP), nao do driver.
    # GroupAction scoped+forwarding=False: o rs_launch.py itera as launch
    # configurations do contexto e emitia ~58 warnings "Parameter 'X' is not
    # supported" para cada arg NOSSO vazado; o escopo entrega a ele so' os
    # launch_arguments abaixo. A condicao fica no GRUPO (dentro do escopo o
    # use_camera nao existe mais).
    realsense = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"
                    ])
                ),
                launch_arguments={
                    "camera_name": "camera",
                    "align_depth.enable": "true",
                    "pointcloud.enable": "false",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "publish_tf": "false",
                    # 2026-08-07: resolucao/fps reduzidos (default era
                    # 1280x720@30). O pick acontece com o robo PARADO a ~30cm
                    # da tag (>100px mesmo em 480p); full-res so servia para o
                    # apriltag comer 125% de CPU e afogar o Nav2. O perfil vem
                    # do arg camera_profile (USB2 vs USB3 — ver declaracao).
                    "rgb_camera.color_profile": camera_profile,
                    "depth_module.depth_profile": camera_profile,
                }.items(),
            ),
        ],
        scoped=True,
        forwarding=False,
        # forwarding=False isola o escopo: configs de fora nao entram. O
        # camera_profile precisa ser injetado explicitamente aqui dentro.
        launch_configurations={"camera_profile": camera_profile},
        condition=IfCondition(use_camera),
    )

    # --- AprilTag (config vendorado no manip_bringup) ---
    # nice +15: percepcao cede CPU para o Nav2 (controller a 20Hz) durante a
    # navegacao; no pick/place o robo esta parado e ha CPU de sobra. Sem isso
    # o controller_server caiu a 2-8Hz e o staging abortava (2026-08-03).
    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        prefix="nice -n 15",
        output="screen",
        condition=IfCondition(use_apriltag),
        remappings=[
            ("image_rect", "/camera/camera/color/image_raw"),
            ("camera_info", "/camera/camera/color/camera_info"),
        ],
        parameters=[
            {"qos_profile": "sensor_data"},
            apriltag_params_file,
        ],
    )

    # --- Audio (Piper com voz de cachorro) ---
    speech = Node(
        package="manip_audio",
        executable="speech_node",
        output="screen",
        condition=IfCondition(use_audio),
        parameters=[{
            "backend": "piper",
            "piper_executable": LaunchConfiguration("piper_executable"),
            "piper_model": LaunchConfiguration("piper_model"),
            "player_executable": LaunchConfiguration("speech_player_executable"),
            "voice_effect": "dog",
        }],
    )

    pick_telemetry = Node(
        package="manip_task_execution",
        executable="pick_telemetry_plotter",
        output="screen",
        condition=IfCondition(launch_pick_telemetry),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(use_rviz_manip),
        arguments=[
            "-d",
            os.path.join(manip_bringup_share, "config", "manip_moveit_rasp.rviz"),
        ],
    )

    # --- Mock do braco no PC (use_mock_hardware=true) ---
    # Mesmo xacro do robo real, mas com mock_components e meshes via package://
    # (no PC nao ha mesh_server http da Pi).
    mock_robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("caramelo_description"),
                "urdf", "robots", "robot.urdf.xacro",
            ]),
            " use_manipulator:=true",
            " use_realsense:=true",
            " use_mock_components:=true",
            " visual_mode:=package",
            " mesh_uri_prefix:=package://caramelo_description",
        ]),
        value_type=str,
    )

    mock_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        condition=IfCondition(use_mock_hardware),
        parameters=[{"robot_description": mock_robot_description}],
    )

    # Padrao Jazzy (igual ao hardware_bringup.launch.py da Pi): o
    # controller_manager assina o topico /robot_description publicado pelo RSP;
    # nao precisa de remap nem de param robot_description.
    mock_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=IfCondition(use_mock_hardware),
        parameters=[
            os.path.join(
                manip_bringup_share, "config", "caramelo_mock_controllers.yaml"
            ),
        ],
    )

    mock_jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(use_mock_hardware),
        arguments=["joint_state_broadcaster"],
    )

    mock_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(use_mock_hardware),
        arguments=["arm_controller"],
    )

    mock_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(use_mock_hardware),
        arguments=["gripper_controller"],
    )

    # A RealSense entra por IncludeLaunchDescription (sem como aplicar prefix):
    # renice apos o driver subir. Mesma justificativa do nice do apriltag.
    renice_camera = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "pgrep -f realsense2_camera_node | xargs -r renice +15 -p",
                ],
                output="log",
                condition=IfCondition(use_camera),
            ),
        ],
    )

    return LaunchDescription(declared_args + [
        move_group,
        commander,
        pick_action,
        place_action,
        realsense,
        renice_camera,
        apriltag,
        speech,
        pick_telemetry,
        rviz,
        mock_rsp,
        mock_control_node,
        mock_jsb_spawner,
        mock_arm_spawner,
        mock_gripper_spawner,
    ])
