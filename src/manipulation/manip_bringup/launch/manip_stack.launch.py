# Stack de manipulacao no PC (porta em Python do launch XML antigo, removido).
#
# SEMPRE sobe: move_group (com ExecuteTaskSolutionCapability p/ MTC).
# Opcionais: commander (use_commander, ferramenta de bancada), camera RealSense,
# AprilTag, voz (Kokoro/Piper/espeak), pick/place MTC, RViz.
#
# use_mock_hardware=true: sobe RSP + ros2_control com mock_components NO PC
# (braco simulado, SEM base mecanum/navegacao). Em modo real (default) o
# RSP/ros2_control roda na Pi (raspberry_bringup) e aqui NAO sobe nada disso.
import os

import yaml
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
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Composicao ---
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_camera = LaunchConfiguration("use_camera")
    use_apriltag = LaunchConfiguration("use_apriltag")
    use_perception_gate = LaunchConfiguration("use_perception_gate")
    camera_profile = LaunchConfiguration("camera_profile")
    use_audio = LaunchConfiguration("use_audio")
    use_pick_place = LaunchConfiguration("use_pick_place")
    use_rviz_manip = LaunchConfiguration("use_rviz_manip")
    use_commander = LaunchConfiguration("use_commander")
    launch_pick_telemetry = LaunchConfiguration("launch_pick_telemetry")
    use_container_detector = LaunchConfiguration("use_container_detector")
    container_detector_always_on = LaunchConfiguration("container_detector_always_on")
    use_surface_detector = LaunchConfiguration("use_surface_detector")
    surface_detector_always_on = LaunchConfiguration("surface_detector_always_on")
    use_vision_debug = LaunchConfiguration("use_vision_debug")
    use_vision_viewer = LaunchConfiguration("use_vision_viewer")

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
    # Multi-place: o no de place enumera as tags que podem estar
    # na mesa a partir da MESMA lista que o apriltag publica — uma fonte so.
    with open(apriltag_params_file, "r", encoding="utf-8") as handle:
        _apriltag_yaml = yaml.safe_load(handle) or {}
    slot_obstacle_tag_frames = [
        str(frame)
        for frame in (
            _apriltag_yaml.get("apriltag", {})
            .get("ros__parameters", {})
            .get("tag", {})
            .get("frames", [])
        )
    ]
    # Containers por cor: os frames publicados pelo
    # container_detector entram como obstaculo dos places normais (0,13 m
    # via clearanceForFrame, prefixo ct_). Sem detector, o place ignora
    # frames sem TF.
    slot_obstacle_tag_frames += ["ct_vermelho", "ct_azul"]

    declared_args = [
        DeclareLaunchArgument(
            "use_container_detector",
            default_value="true",
            description=(
                "Detector de containers vermelho/azul por cor (container_detector_node.py). "
                "So processa imagem enquanto ha um place ativo."
            ),
        ),
        DeclareLaunchArgument(
            "container_detector_always_on",
            default_value="false",
            description="Detector processa sempre (calibracao/bag), nao so durante o place",
        ),
        # --- Superficie livre ---
        DeclareLaunchArgument(
            "use_surface_detector",
            default_value="true",
            description=(
                "Mapa top-down da mesa pela RealSense (workstation_surface_node.py): o place "
                "escolhe um ponto livre que respeita o rulebook (bordas, objetos, parede). "
                "So processa imagem durante a observacao do place."
            ),
        ),
        DeclareLaunchArgument(
            "surface_detector_always_on",
            default_value="false",
            description="Mapa da mesa processa sempre (calibracao/bag), nao so na observacao",
        ),
        # --- Visao de depuracao ---
        DeclareLaunchArgument(
            "use_vision_debug",
            default_value="true",
            description=(
                "Imagem da camera anotada (tags, containers, mapa da mesa, status) em "
                "/manip/vision_debug/image (vision_debug_node.py); custo zero sem espectador."
            ),
        ),
        DeclareLaunchArgument(
            "use_vision_viewer",
            default_value="false",
            description="Abre o rqt_image_view ja em /manip/vision_debug/image",
        ),
        DeclareLaunchArgument(
            "place_surface_planner",
            default_value="true",
            description=(
                "false = kill-switch: o place volta as 3 poses fixas (MesaX/.1/.2) sem olhar o "
                "tampo. Tambem: ros2 param set /place_action_server surface_place_enabled false"
            ),
        ),
        DeclareLaunchArgument(
            "place_surface_wall_lidar",
            default_value="false",
            description="Detecta parede ao lado da mesa pelo /scan (opt-in; so acrescenta paredes)",
        ),
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
            # 640x480x5: unico modo estavel em porta USB 2 (a 15 fps o driver
            # reiniciava em loop com frames corrompidos, medido 2026-08-07).
            # Em USB 3 passe camera_profile:=640x480x15 (ou mais).
            default_value="640x480x5",
            description="Perfil WxHxFPS dos streams color e depth da RealSense",
        ),
        DeclareLaunchArgument(
            "use_apriltag",
            default_value="true",
            description="Inicia o apriltag_node na imagem colorida da RealSense",
        ),
        DeclareLaunchArgument(
            "use_perception_gate",
            # Default FALSE: o liga-desliga ja travou o driver da RealSense e
            # o motivo original do gate (banda do USB 2) sumiu com a camera no
            # USB 3. Opt-in; ver o docstring do perception_gate_node.py.
            default_value="false",
            description=(
                "Camera streama SO durante pick/place (libera CPU/USB na "
                "navegacao). EXPERIMENTAL: ja travou o driver uma vez."
            ),
        ),
        DeclareLaunchArgument(
            "perception_off_delay",
            default_value="20.0",
            description=(
                "Segundos de ociosidade antes de desligar os streams — cobre "
                "a janela entre o pick e o place do mesmo objeto"
            ),
        ),
        DeclareLaunchArgument(
            "use_audio",
            default_value="true",
            description="Inicia o speech_node (voz em /manip/speech)",
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
        DeclareLaunchArgument(
            "use_commander",
            default_value="true",
            description=(
                "Sobe o commander (manip_commander): ferramenta de bancada para mover "
                "o braco por topico (ros2 topic pub). false na competicao."
            ),
        ),
        # --- Voz ---
        DeclareLaunchArgument(
            "speech_backend",
            default_value="kokoro",
            description=(
                "Motor de voz: kokoro (natural; venv ~/caramelo_tts_venv, ver "
                "manip_audio/scripts/instalar_kokoro.sh), piper ou espeak. Sem o venv "
                "o speech_node cai para o piper sozinho."
            ),
        ),
        DeclareLaunchArgument(
            "kokoro_voice",
            default_value="pm_alex",
            description="Voz pt-BR do Kokoro: pm_alex, pm_santa (masc.) ou pf_dora (fem.)",
        ),
        DeclareLaunchArgument("piper_executable", default_value="auto"),
        DeclareLaunchArgument(
            "piper_model",
            default_value=os.path.join(
                manip_audio_share, "models", "pt_BR-faber", "pt_BR-faber-medium.onnx"
            ),
            description="Modelo .onnx do Piper (pt_BR-faber ou pt_BR-jeff, em manip_audio/models)",
        ),
        DeclareLaunchArgument("speech_player_executable", default_value="auto"),
        DeclareLaunchArgument(
            "voice_effect",
            default_value="none",
            description="Efeito na voz via ffplay: none, dog, dog_deep ou cartoon_dog",
        ),
        # --- IK/grasp ---
        DeclareLaunchArgument("verify_grasp_effort", default_value="true"),
        # Erro maximo (m) entre a pose pedida e onde a ponta parou. 0.0 DESLIGA
        # a checagem (missao demo aceitando falha silenciosa).
        DeclareLaunchArgument("max_pose_error_m", default_value="0.025"),
        # --- Multi-place na mesa comum ---
        # false = pose unica legada (kill-switch de arena). Tambem aceita
        # `ros2 param set /place_action_server slot_selection_enabled false`
        # sem relaunch (o no rele os slot_* a cada goal) — mas esse param set
        # vale so ate o no respawnar (respawn=True); para desligar de forma
        # persistente use este arg.
        DeclareLaunchArgument("place_slot_selection", default_value="true"),
        # Folga centro a centro entre o ponto de soltar e cada tag na mesa.
        DeclareLaunchArgument("place_slot_min_clearance_m", default_value="0.07"),
        # Olhar a mesa de garra vazia (movimento extra) alem de em pegar_obj.
        DeclareLaunchArgument("place_slot_observe_empty_gripper_first", default_value="false"),
        # XY livre pela IK custom quando nenhuma pose fixa tem folga. OFF ate
        # ser validado no robo.
        DeclareLaunchArgument("place_slot_fallback_free_xy", default_value="false"),
        DeclareLaunchArgument("grasp_min_effort_nm", default_value="0.15"),
        DeclareLaunchArgument("grasp_min_effort_increase_nm", default_value="0.05"),
        # >= 2 e OBRIGATORIO para o ladder da mesa ter os 3 degraus:
        # tentativa 1 = IK custom bottom, 2 = IK custom frente,
        # 3 = MoveIt/pick_ik (ultimo recurso).
        DeclareLaunchArgument("grasp_retry_attempts", default_value="2"),
        # So os attempt_3_* agem (degrau MoveIt/pick_ik da tentativa 3): as
        # tentativas 1-2 usam a IK custom e nem chamam a IK do MoveIt, e trocar
        # o kinematics_solver em runtime nao afeta o plugin ja carregado pelo
        # move_group. Os antigos switch_ik_after_failed_grasp, attempt_1/2_* e
        # fallback_ik_* eram inertes e foram removidos daqui (o no de pick
        # continua com os defaults de codigo).
        DeclareLaunchArgument("attempt_3_ik_solver", default_value="pick_ik/PickIkPlugin"),
        DeclareLaunchArgument("attempt_3_ik_solve_type", default_value="Distance"),
        DeclareLaunchArgument("attempt_3_ik_position_only", default_value="false"),
        DeclareLaunchArgument("attempt_3_ik_position_threshold", default_value="0.010"),
        DeclareLaunchArgument("attempt_3_ik_orientation_threshold", default_value="0.90"),
        DeclareLaunchArgument("attempt_3_goal_position_tolerance", default_value="0.010"),
        DeclareLaunchArgument("attempt_3_goal_orientation_tolerance", default_value="0.80"),
        DeclareLaunchArgument("skip_failed_pick_after_retries", default_value="true"),
        DeclareLaunchArgument("skip_missing_place_tag", default_value="true"),
        # Default TRUE: todo start de stack zera os containers (fluxo real:
        # 1 launch por rodada). Num relaunch POS-CRASH no meio de missao com
        # bloco a bordo, subir com reset_container_states:=false para
        # preservar o estado.
        DeclareLaunchArgument("reset_container_states", default_value="true"),
    ]

    # --- move_group (sempre) ---
    # Mesmo mecanismo do launch XML antigo (removido): o launch gerado pelo
    # generate_move_group_launch declara o arg "capabilities" e o injeta como
    # parametro do move_group; basta repassar como launch argument.
    #
    # O warning repetido
    #   "Unable to transform object from frame 'tag_N' to planning frame"
    # e BENIGNO: o PlanningSceneMonitor varre TODOS os frames do TF a cada
    # request de plan, e os frames tag_N ficam stale porque o apriltag so
    # publica com a tag visivel. Nao e bug e nao se corrige aqui — na pratica
    # serve de termometro: warning sumindo/aparecendo em rajada = dropout da
    # camera.
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "move_group_caramelo.launch.py")
        ),
        launch_arguments={
            "capabilities": "move_group/ExecuteTaskSolutionCapability",
        }.items(),
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

    # SRDF LOCAL como parametro: quando o
    # move_group cai, o RDFLoader dos nos MTC esperava a SRDF via topico por
    # 10s e o construtor do MoveGroupInterface ABORTAVA O PROCESSO (SIGABRT em
    # thread interna). Com a SRDF local, o modelo monta sempre e a falta do
    # move_group vira falha limpa da sonda/timeout de action server.
    with open(
        os.path.join(moveit_config_share, "config", "caramelo.srdf"),
        "r", encoding="utf-8",
    ) as srdf_file:
        robot_description_semantic_content = srdf_file.read()

    # --- Commander (use_commander; ferramenta de bancada, nao faz parte da missao) ---
    # URDF/SRDF locais como parametro (mesmo tratamento dos nos MTC): sem isso
    # o commander corria contra o robot_state_publisher da Pi, morria no boot
    # ("Could not find parameter robot_description") e /go_to_named_target
    # ficava sem assinante em silencio.
    commander = Node(
        package="manip_commander",
        executable="commander",
        condition=IfCondition(use_commander),
        respawn=True,
        respawn_delay=5.0,
        parameters=[{
            "robot_description": primitive_robot_description,
            "robot_description_semantic": robot_description_semantic_content,
        }],
    )


    # respawn: se o no de pick/place morrer
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
            # reset_container_states_on_start NAO e mais passado ao no (fica
            # no default false do codigo): com respawn=True, o reset por
            # processo zerava o yaml com blocos fisicamente a bordo. O reset
            # por rodada e o one-shot reset_container_states_once abaixo.
            "verify_grasp_effort": ParameterValue(
                LaunchConfiguration("verify_grasp_effort"), value_type=bool),
            "max_pose_error_m": ParameterValue(
                LaunchConfiguration("max_pose_error_m"), value_type=float),
            "grasp_min_effort_nm": ParameterValue(
                LaunchConfiguration("grasp_min_effort_nm"), value_type=float),
            "grasp_min_effort_increase_nm": ParameterValue(
                LaunchConfiguration("grasp_min_effort_increase_nm"), value_type=float),
            "grasp_retry_attempts": ParameterValue(
                LaunchConfiguration("grasp_retry_attempts"), value_type=int),
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
            "skip_failed_pick_after_retries": ParameterValue(
                LaunchConfiguration("skip_failed_pick_after_retries"), value_type=bool),
            "container_state_file": container_state_file,
            "manipulator_lock_file": manipulator_lock_file,
            "robot_description": primitive_robot_description,
            "robot_description_semantic": robot_description_semantic_content,
        }],
    )

    # Reset dos containers UMA vez por rodada (start do launch), nunca no
    # (re)start de processo: o respawn do no de pick re-executaria o reset e
    # zeraria o yaml com blocos a bordo.
    reset_container_states_once = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("reset_container_states")),
        name="reset_container_states_once",
        cmd=[
            "python3", "-c",
            (
                "import os\n"
                "p = r'" + container_state_file + "'\n"
                "os.makedirs(os.path.dirname(p), exist_ok=True)\n"
                "linhas = ['containers:\\n']\n"
                "for c in ('container1', 'container2', 'container3'):\n"
                "    linhas += ['  %s:\\n' % c, '    occupied: false\\n',"
                " '    tag_frame: \"\"\\n', '    status: empty\\n']\n"
                "open(p, 'w').writelines(linhas)\n"
                "print('containers resetados: ' + p)\n"
            ),
        ],
        output="screen",
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
            # Os mesmos args de esforco do pick valem para o place; sem isso o
            # place ficaria nos defaults do codigo.
            "verify_grasp_effort": ParameterValue(
                LaunchConfiguration("verify_grasp_effort"), value_type=bool),
            "grasp_min_effort_nm": ParameterValue(
                LaunchConfiguration("grasp_min_effort_nm"), value_type=float),
            "grasp_min_effort_increase_nm": ParameterValue(
                LaunchConfiguration("grasp_min_effort_increase_nm"), value_type=float),
            # Multi-place.
            "slot_selection_enabled": ParameterValue(
                LaunchConfiguration("place_slot_selection"), value_type=bool),
            "slot_min_clearance_m": ParameterValue(
                LaunchConfiguration("place_slot_min_clearance_m"), value_type=float),
            "slot_observe_empty_gripper_first": ParameterValue(
                LaunchConfiguration("place_slot_observe_empty_gripper_first"), value_type=bool),
            "slot_fallback_free_xy_enabled": ParameterValue(
                LaunchConfiguration("place_slot_fallback_free_xy"), value_type=bool),
            "slot_obstacle_tag_frames": slot_obstacle_tag_frames,
            "max_pose_error_m": ParameterValue(
                LaunchConfiguration("max_pose_error_m"), value_type=float),
            # Superficie livre.
            "surface_place_enabled": ParameterValue(
                LaunchConfiguration("place_surface_planner"), value_type=bool),
            "surface_wall_lidar_enabled": ParameterValue(
                LaunchConfiguration("place_surface_wall_lidar"), value_type=bool),
        }],
    )

    # --- AMT1: ordenacao das tags da mesa de precision placement ---
    # Servidor /amt1_sort. Cliente de /pick_tag e /place_tag (nao pega nem
    # solta sozinho); so move o braco para OBSERVAR a mesa, por isso recebe
    # os mesmos URDF/SRDF locais, o mesmo lock e o mesmo yaml dos containers.
    # tag_frames = so as tag_* da lista do apriltag (os ct_* do detector de
    # containers nao sao cubos).
    amt1_sort_action = Node(
        respawn=True,
        respawn_delay=3.0,
        package="manip_task_execution",
        executable="amt1_sort_action_node",
        output="screen",
        condition=IfCondition(use_pick_place),
        parameters=[{
            "container_state_file": container_state_file,
            "manipulator_lock_file": manipulator_lock_file,
            "robot_description": primitive_robot_description,
            "robot_description_semantic": robot_description_semantic_content,
            "tag_frames": [
                frame for frame in slot_obstacle_tag_frames if frame.startswith("tag_")
            ],
            # Altura do cubo: tampo = z da tag - cube_height_m (o place soma
            # stack_place_z_offset ao soltar).
            "cube_height_m": 0.042,
            # Busca de tag que falta: gira
            # SO a j1 ("j1"); "base" = anda de lado; "j1_then_base" = os dois.
            "search_mode": "j1",
            "search_j1_offsets_deg": [-30.0, 30.0, -60.0, 60.0],
            # Observa de tag_direita e depois tag_esquerda (j2=0 no SRDF nas
            # duas; validado em teste 2026-08-31).
            "observe_poses": ["tag_direita", "tag_esquerda"],
            "reobserve_poses": ["tag_direita", "tag_esquerda"],
            # Precisao do place: re-registro local antes de cada
            # place e passe de verificacao/ajuste (tolerancia 15 mm).
            "place_local_reregister": True,
            "verify_after_sort": True,
            "verify_tol_m": 0.015,
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
                    # Resolucao/fps reduzidos (default do driver e 1280x720@30):
                    # o pick acontece com o robo PARADO a ~30cm da tag (>100px
                    # mesmo em 480p); full-res so servia para o apriltag comer
                    # 125% de CPU e afogar o Nav2. O perfil vem do arg
                    # camera_profile (USB2 vs USB3 — ver declaracao).
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
        # Percepcao morta no meio de uma missao deixava o pick cego
        # ate o fim da rodada.
        respawn=True,
        respawn_delay=3.0,
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

    # --- Detector de containers por cor ---
    # Segmentacao HSV + ray-cast no plano da borda (a profundidade do D455
    # nao alcanca a mesa daqui). Publica TF manip_base_link -> ct_vermelho /
    # ct_azul so enquanto ve o container e so durante o place (/manip/
    # place_active). nice +15 como o apriltag. Cores em config/container_detector.yaml.
    container_detector = Node(
        package="manip_bringup",
        executable="container_detector_node.py",
        name="container_detector",
        output="screen",
        prefix="nice -n 15",
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(use_container_detector),
        parameters=[
            os.path.join(manip_bringup_share, "config", "container_detector.yaml"),
            {"always_on": ParameterValue(container_detector_always_on, value_type=bool)},
        ],
    )

    # --- Mapa da mesa (superficie livre) ---
    # Segmenta o tampo branco e projeta cada celula de 1 cm no plano do tampo
    # (cota em /manip/table_plane_z, do place). Publica nav_msgs/OccupancyGrid
    # em /manip/workstation_surface/grid (frame manip_base_link) enquanto
    # /manip/surface_observe for true. Poligonos da garra = os do
    # container_detector.yaml (fonte unica). nice +15 como o apriltag.
    workstation_surface = Node(
        package="manip_bringup",
        executable="workstation_surface_node.py",
        name="workstation_surface",
        output="screen",
        prefix="nice -n 15",
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(use_surface_detector),
        parameters=[
            os.path.join(manip_bringup_share, "config", "workstation_surface.yaml"),
            {
                "always_on": ParameterValue(surface_detector_always_on, value_type=bool),
                "gripper_mask_yaml": os.path.join(
                    manip_bringup_share, "config", "container_detector.yaml"),
            },
        ],
    )

    # --- Visao de depuracao ---
    # Uma imagem so' com tudo que a percepcao entendeu da camera do punho:
    # tags (/detections), containers por cor (TF ct_*), mapa da mesa (grade
    # projetada de volta) e barra de status. So assina a imagem enquanto
    # alguem assina /manip/vision_debug/image (sem espectador: custo zero e
    # nao conta como "viewer" para o perception_gate); com espectador, ate
    # 10 Hz. nice +15 como os outros nos de imagem.
    vision_debug = Node(
        package="manip_bringup",
        executable="vision_debug_node.py",
        name="vision_debug",
        output="screen",
        prefix="nice -n 15",
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(use_vision_debug),
    )

    # Espectador pronto (opcional): rqt_image_view ja aberto no topico anotado.
    vision_viewer = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        output="screen",
        condition=IfCondition(use_vision_viewer),
        arguments=["/manip/vision_debug/image"],
    )

    # --- Gate da percepcao ---
    # A camera so streama durante pick/place. Desacoplado: sem este no, tudo
    # se comporta como antes (camera sempre ligada). Ver o docstring do script.
    perception_gate = Node(
        package="manip_bringup",
        executable="perception_gate_node.py",
        name="perception_gate",
        output="screen",
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(use_perception_gate),
        parameters=[{
            "enabled": ParameterValue(
                LaunchConfiguration("use_perception_gate"), value_type=bool),
            "off_delay_s": ParameterValue(
                LaunchConfiguration("perception_off_delay"), value_type=float),
            # Assinantes "de fabrica" do image_raw: 1 quando o apriltag roda,
            # 0 sem ele. Acima disso = alguem (RViz) quer ver a imagem, e o
            # gate mantem a camera ligada.
            "viewer_subscriber_baseline": ParameterValue(
                PythonExpression(
                    ["(1 if '", LaunchConfiguration("use_apriltag"),
                     "' == 'true' else 0) + (1 if '",
                     LaunchConfiguration("use_container_detector"),
                     "' == 'true' and '",
                     LaunchConfiguration("container_detector_always_on"),
                     "' == 'true' else 0)"]),
                value_type=int),
        }],
    )

    # --- Voz (fila com prioridade; nunca bloqueia os nos, nunca corta frase) ---
    speech = Node(
        package="manip_audio",
        executable="speech_node",
        output="screen",
        condition=IfCondition(use_audio),
        parameters=[{
            "backend": LaunchConfiguration("speech_backend"),
            "kokoro_voice": LaunchConfiguration("kokoro_voice"),
            "piper_executable": LaunchConfiguration("piper_executable"),
            "piper_model": LaunchConfiguration("piper_model"),
            "player_executable": LaunchConfiguration("speech_player_executable"),
            "voice_effect": LaunchConfiguration("voice_effect"),
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
    # Era um one-shot de 12 s: o driver respawnado voltava com
    # prioridade normal e roubava CPU do Nav2 sem ninguem perceber. Agora e um
    # loop leve (uma varredura a cada 10 s; renice em pid ja nicado e no-op).
    renice_camera = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "while true; do pgrep -f realsense2_camera_node "
                    "| xargs -r renice +15 -p >/dev/null 2>&1; sleep 10; done",
                ],
                output="log",
                condition=IfCondition(use_camera),
            ),
        ],
    )

    return LaunchDescription(declared_args + [
        move_group,
        commander,
        reset_container_states_once,
        pick_action,
        place_action,
        amt1_sort_action,
        realsense,
        perception_gate,
        renice_camera,
        apriltag,
        container_detector,
        workstation_surface,
        vision_debug,
        vision_viewer,
        speech,
        pick_telemetry,
        rviz,
        mock_rsp,
        mock_control_node,
        mock_jsb_spawner,
        mock_arm_spawner,
        mock_gripper_spawner,
    ])
