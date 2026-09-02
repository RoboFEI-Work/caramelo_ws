#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <future>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
#include <algorithm>
#include <limits>
#include <array>
#include <map>
#include <vector>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "manip_task_execution/container_state_store.hpp"
#include "manip_task_execution/custom_ik.hpp"
#include "manip_task_execution/manipulator_execution_lock.hpp"
#include "manip_task_execution/reach_shift.hpp"
#include "my_robot_msgs/action/place_tag.hpp"

class PlaceActionServer : public rclcpp::Node
{
public:

    using PlaceTag =
        my_robot_msgs::action::PlaceTag;

    using GoalHandlePlaceTag =
        rclcpp_action::ServerGoalHandle<PlaceTag>;

    using MoveGroupInterface =
        moveit::planning_interface::MoveGroupInterface;

    PlaceActionServer()
        : Node("place_action_server")
    {
        const auto lock_file = this->declare_parameter<std::string>(
            "manipulator_lock_file",
            "/tmp/caramelo_manip_action.lock");
        execution_lock_ =
            std::make_unique<manip_task_execution::ManipulatorExecutionLock>(lock_file);
        if (!execution_lock_->valid()) {
            throw std::runtime_error(
                "Failed to open manipulator lock file '" + lock_file + "': " +
                execution_lock_->error());
        }

        // Mesma protecao do pick: tag precisa ter sido vista AGORA (TF velha
        // com robo/braco movido = alvo fantasma; tambem evitaria que uma tag
        // velha "fingisse presenca" no skip_missing_place_tag).
        max_tag_age_sec_ = this->declare_parameter<double>("max_tag_age_sec", 1.0);

        const auto default_container_state_file = getDefaultContainerStatePath();
        container_state_file_ = this->declare_parameter<std::string>(
            "container_state_file",
            default_container_state_file);
        container_place_z_offset_ =
            this->declare_parameter<double>("container_place_z_offset", 0.1);
        // 2026-08-24 — EMPILHAR (pedido do operador: tag_7 em cima da tag_4).
        // PlaceTag.goal.stack_on = frame da tag BASE ja na mesa. O TCP vai a
        // (x, y) da base e z = z_base + stack_place_z_offset (altura do cubo
        // ~42 mm + folga de queda; CALIBRAR no robo), passando por uma
        // pre-pose stack_pre_lift_m acima e descendo na vertical; depois de
        // soltar sobe de volta. Escada de inclinacao como no pick (alcance).
        // Base nao vista / IK sem solucao -> place normal no table_pose
        // (stack_fallback_to_table) em vez de abortar com o bloco na garra.
        stack_place_z_offset_ =
            this->declare_parameter<double>("stack_place_z_offset", 0.05);
        stack_pre_lift_m_ = this->declare_parameter<double>("stack_pre_lift_m", 0.05);
        stack_fallback_to_table_ =
            this->declare_parameter<bool>("stack_fallback_to_table", true);
        stack_arrival_tolerance_m_ =
            this->declare_parameter<double>("stack_arrival_tolerance_m", 0.03);
        stack_tilt_ladder_deg_ = this->declare_parameter<std::vector<double>>(
            "stack_tilt_ladder_deg", std::vector<double>{15.0, 30.0});
        // 2026-08-25 — CONTAINER POR COR na MESA (NAO confundir com os potes a
        // bordo container1..3 / container_state_store). PlaceTag.goal.
        // container_color = RED|BLUE. O container_detector publica os frames
        // ct_vermelho/ct_azul (centro no plano da BORDA, X = lado longo)
        // enquanto o place esta ativo; este no publica a cota da borda em
        // /manip/container_plane_z a partir da FK da pose nomeada da mesa.
        // Alvo do TCP = borda + cubo + folga (fundo do cubo 3 cm acima da
        // borda, decisao do operador). Nao visto / sem IK -> place normal.
        container_frame_red_ = this->declare_parameter<std::string>(
            "container_frame_red", "ct_vermelho");
        container_frame_blue_ = this->declare_parameter<std::string>(
            "container_frame_blue", "ct_azul");
        container_rim_height_m_ =
            this->declare_parameter<double>("container_rim_height_m", 0.07);
        container_drop_clearance_m_ =
            this->declare_parameter<double>("container_drop_clearance_m", 0.01);
        container_close_gripper_on_exit_ =
            this->declare_parameter<bool>("container_close_gripper_on_exit", true);
        container_slot_offsets_long_m_ = this->declare_parameter<std::vector<double>>(
            "container_slot_offsets_long_m", std::vector<double>{0.03, -0.03, 0.0});
        container_slot_offsets_short_m_ = this->declare_parameter<std::vector<double>>(
            "container_slot_offsets_short_m", std::vector<double>{0.0, 0.0, 0.0});
        container_exit_gripper_position_ =
            this->declare_parameter<double>("container_exit_gripper_position", 0.170);
        cube_height_m_ = this->declare_parameter<double>("cube_height_m", 0.042);
        container_detect_timeout_sec_ =
            this->declare_parameter<double>("container_detect_timeout_sec", 3.0);
        container_arrival_tolerance_m_ =
            this->declare_parameter<double>("container_arrival_tolerance_m", 0.03);
        container_arrival_z_tolerance_m_ =
            this->declare_parameter<double>("container_arrival_z_tolerance_m", 0.06);
        container_tilt_ladder_deg_ = this->declare_parameter<std::vector<double>>(
            "container_tilt_ladder_deg", std::vector<double>{15.0, 30.0});
        container_fallback_to_table_ =
            this->declare_parameter<bool>("container_fallback_to_table", true);
        // 2026-08-28 (fila de alcance): container/pilha VISTOS mas sem IK em
        // nenhum candidato x inclinacao => com final_attempt=false o place sai
        // CEDO: devolve o cubo ao container de bordo (yaml continua occupied)
        // e responde unreachable=true com uma sugestao de deslocamento
        // lateral da base verificada pela mesma IK (reach_shift), SEM cair na
        // mesa. false = comportamento antigo (fallback silencioso na mesa).
        // Mesmos nomes/defaults do no de pick.
        unreachable_bailout_enabled_ = this->declare_parameter<bool>(
            "unreachable_bailout_enabled", true);
        // |dy| candidatos (m, base_footprint), crescentes; o piso do ESC nao
        // executa passos < ~0,10 m e o curso maximo e decisao do operador.
        unreachable_shift_candidates_m_ = this->declare_parameter<std::vector<double>>(
            "unreachable_shift_candidates_m",
            std::vector<double>{0.10, 0.15, 0.20, 0.25});
        // A base para um pouco antes do pedido: a busca testa o alvo com
        // esse desconto para nao sugerir um passo que fique 1 cm curto.
        unreachable_shift_margin_m_ = this->declare_parameter<double>(
            "unreachable_shift_margin_m", 0.02);
        container_plane_topic_ = this->declare_parameter<std::string>(
            "container_plane_topic", "/manip/container_plane_z");
        // Container fora do quadro de pegar_obj (mesa de 80 cm, camera a ~0,33 m
        // ve so' +-20 cm): procura tambem de tag_direita/tag_esquerda antes de
        // desistir (campo 25/08: bins cortados na borda sao rejeitados).
        container_search_lateral_ =
            this->declare_parameter<bool>("container_search_lateral", true);
        container_search_timeout_sec_ =
            this->declare_parameter<double>("container_search_timeout_sec", 1.5);
        skip_missing_place_tag_ =
            this->declare_parameter<bool>("skip_missing_place_tag", true);
        // Auditoria 2026-08-07, item 2.4: verificacao de objeto preso por
        // esforco tambem no PLACE (garra fechando no vazio dentro do
        // container retornava sucesso fantasma). Limiares iniciais = os do
        // pick; CALIBRAR no robo real.
        verify_grasp_effort_ =
            this->declare_parameter<bool>("verify_grasp_effort", true);
        // 2026-08-31 (pedido do operador): a verificacao de esforco ao pegar
        // o cubo NO CONTAINER durante o place ("garra_vazia_no_container",
        // que ja reprovou pega valida com um dedo so) sai por default.
        // Religavel por parametro; a checagem da DEVOLUCAO (RETURN-CUBE)
        // continua sob verify_grasp_effort.
        verify_container_grasp_ =
            this->declare_parameter<bool>("place_verify_container_grasp", false);
        grasp_min_effort_nm_ =
            this->declare_parameter<double>("grasp_min_effort_nm", 0.15);
        grasp_min_effort_increase_nm_ =
            this->declare_parameter<double>(
                "grasp_min_effort_increase_nm", 0.05);
        grasp_effort_sample_duration_ =
            this->declare_parameter<double>(
                "grasp_effort_sample_duration", 0.8);
        grasp_effort_max_age_ =
            this->declare_parameter<double>("grasp_effort_max_age", 0.4);
        dynamic_joint_state_subscription_ =
            this->create_subscription<control_msgs::msg::DynamicJointState>(
                "/dynamic_joint_states",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &PlaceActionServer::onDynamicJointState,
                    this,
                    std::placeholders::_1));
        container_state_store_ =
            std::make_unique<manip_task_execution::ContainerStateStore>(container_state_file_);
        declarePlanningDefaults();

        tf_buffer_ =
            std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        speech_enabled_ = this->declare_parameter<bool>("speech_enabled", true);
        speech_publisher_ =
            this->create_publisher<std_msgs::msg::String>("/manip/speech", 10);

        // Item 3.10 (gate da percepcao): espelho do /manip/pick_active — o
        // gate liga a camera enquanto qualquer um dos dois estiver ativo.
        place_active_publisher_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/manip/place_active",
                rclcpp::QoS(1).transient_local().reliable());
        publishPlaceActive(false);
        // Cota (z em manip_base_link) da BORDA do container da mesa de
        // destino, para o container_detector fazer o ray-cast no plano certo.
        container_plane_z_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            container_plane_topic_, rclcpp::QoS(1).transient_local().reliable());
        // 2026-08-28 (fila de alcance): deslocamento lateral ACUMULADO da base
        // desde o ultimo align_to_dock (+ = esquerda, base_footprint), que so
        // o dock_align_node publica (latched; zera a cada align_to_dock). A
        // memoria de slots e gravada no frame da pose de docking (offset 0):
        // ler = x + offset, gravar = x - offset (ver containerSlotCandidates,
        // addMemoryObstacles, recordSlotUsed). O yaml NUNCA e reescrito.
        base_shift_total_subscription_ =
            this->create_subscription<std_msgs::msg::Float64>(
                "/manip/base_shift_total",
                rclcpp::QoS(1).transient_local().reliable(),
                [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
                    base_shift_total_.store(msg->data, std::memory_order_relaxed);
                });

        // 2026-08-12: entregar na prateleira exige um waypoint obrigatorio
        // (`pirocao`) antes e depois da pose de mesa — sem ele o braco bate
        // na estante. Mesmo vocabulario/parametro usados no no de pick.
        shelf_table_poses_ = this->declare_parameter<std::vector<std::string>>(
            "shelf_table_poses", std::vector<std::string>{"MesaSh"});

        // Frame da base do braco — a IK custom trabalha nele (2026-08-17,
        // entrega em alvo TF pela IK propria). Espelho do
        // shelf_ik_reference_frame do no de pick.
        ik_reference_frame_ = this->declare_parameter<std::string>(
            "ik_reference_frame", "manip_base_link");

        // ---- Multi-place na mesa comum (2026-08-21) ----
        // Antes de soltar, olha a mesa e escolhe, entre as poses MesaX /
        // MesaX.1 / MesaX.2 do SRDF, a mais distante das tags presentes
        // (rulebook 5.1.3: o bloco e o robo nao podem tocar outros objetos
        // nem containers). Todos os slot_* sao relidos a cada goal
        // (reloadSlotParameters) para `ros2 param set` valer na arena.
        this->declare_parameter<bool>("slot_selection_enabled", true);
        this->declare_parameter<std::vector<std::string>>(
            "slot_table_poses",
            std::vector<std::string>{"Mesa0", "Mesa5", "Mesa10", "Mesa15"});
        this->declare_parameter<std::vector<std::string>>(
            "slot_selection_ws_prefixes", std::vector<std::string>{"WS"});
        this->declare_parameter<std::vector<std::string>>(
            "slot_excluded_poses", std::vector<std::string>{});
        // Frames que contam como obstaculo na mesa. Default = a lista de
        // manip_bringup/config/tags_36h11.yaml (o launch repassa a mesma).
        this->declare_parameter<std::vector<std::string>>(
            "slot_obstacle_tag_frames",
            std::vector<std::string>{
                "tag_1", "tag_2", "tag_3", "tag_4", "tag_5", "tag_6", "tag_7",
                "ct_10", "ct_11", "ct_12", "ct_13", "ct_14", "ct_15", "ct_16",
                "tag_20", "tag_21", "tag_22", "tag_23", "tag_24", "tag_25",
                "tag_26", "tag_27", "tag_28", "tag_42"});
        this->declare_parameter<double>("slot_observe_dwell_sec", 1.0);
        this->declare_parameter<double>("slot_observe_max_dz_m", 0.15);
        this->declare_parameter<double>("slot_observe_min_radial_m", 0.10);
        // 0.07 centro a centro = 2,8 cm de vao entre cubos de 42 mm (o padrao
        // dos arbitros e 2 cm); 0.05 deixaria 0,8 cm.
        this->declare_parameter<double>("slot_min_clearance_m", 0.07);
        // Container 2B: 135 x 160 mm externos com a tag no fundo.
        this->declare_parameter<double>("slot_container_clearance_m", 0.13);
        this->declare_parameter<double>("slot_margin_cap_m", 0.20);
        this->declare_parameter<double>("slot_tie_epsilon_m", 0.01);
        // Chegada no slot nomeado: o controlador aceita 0.30 rad por junta e
        // o creep sob carga (~0.13 rad em j3) vale ~2,6 cm — 4 cm separa
        // "chegou" de "parou longe" sem reprovar toda entrega.
        this->declare_parameter<double>("slot_named_pose_error_m", 0.04);
        this->declare_parameter<double>("max_pose_error_m", 0.025);
        this->declare_parameter<bool>("slot_memory_enabled", true);
        this->declare_parameter<double>("slot_memory_ttl_sec", 900.0);
        this->declare_parameter<bool>("slot_fallback_free_xy_enabled", false);
        this->declare_parameter<double>("slot_fallback_grid_step_m", 0.02);
        this->declare_parameter<double>("slot_fallback_wrist_offset_rad", 0.0);
        this->declare_parameter<bool>("slot_observe_empty_gripper_first", false);
        // Abaixo desta margem os corpos se sobrepoem (cubo sobre cubo);
        // -1.0 = nunca recusar (decisao do operador: "coloca no menos ruim").
        // Para nunca empilhar: -0.025.
        this->declare_parameter<double>("slot_overlap_refuse_margin_m", -1.0);
        reloadSlotParameters();
        RCLCPP_WARN(
            get_logger(),
            "[SLOT] slot_selection_enabled=%s (um `ros2 param set` vale so ate o no "
            "respawnar; kill-switch persistente = place_slot_selection:=false no launch)",
            slot_selection_enabled_ ? "true" : "false");

        camera_info_topic_ = this->declare_parameter<std::string>(
            "camera_info_topic", "/camera/camera/color/camera_info");
        perception_warmup_timeout_ = this->declare_parameter<double>(
            "perception_warmup_timeout", 6.0);
        camera_info_subscription_ =
            this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_,
                rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {
                    camera_info_count_.fetch_add(1, std::memory_order_relaxed);
                    camera_info_last_ns_.store(
                        std::chrono::steady_clock::now()
                            .time_since_epoch().count(),
                        std::memory_order_relaxed);
                });

        action_server_ =
            rclcpp_action::create_server<PlaceTag>(
                this,
                "/place_tag",

                std::bind(
                    &PlaceActionServer::handle_goal,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),

                std::bind(
                    &PlaceActionServer::handle_cancel,
                    this,
                    std::placeholders::_1),

                std::bind(
                    &PlaceActionServer::handle_accepted,
                    this,
                    std::placeholders::_1));
    }

private:
    static std::string getDefaultContainerStatePath()
    {
        const char * home = std::getenv("HOME");
        if (home != nullptr && home[0] != '\0') {
            return std::string(home) + "/.config/caramelo/container_states.yaml";
        }
        return "container_states.yaml";
    }

    rclcpp_action::Server<PlaceTag>::SharedPtr
        action_server_;
    std::string container_state_file_;
    std::unique_ptr<manip_task_execution::ContainerStateStore> container_state_store_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr place_active_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_subscription_;
    std::atomic<std::uint64_t> camera_info_count_{0};
    std::atomic<std::int64_t> camera_info_last_ns_{0};
    std::vector<std::string> shelf_table_poses_;
    std::string ik_reference_frame_;

    // ---- Multi-place (2026-08-21): parametros relidos a cada goal ----
    bool slot_selection_enabled_{true};
    std::vector<std::string> slot_table_poses_;
    std::vector<std::string> slot_selection_ws_prefixes_;
    std::vector<std::string> slot_excluded_poses_;
    std::vector<std::string> slot_obstacle_tag_frames_;
    double slot_observe_dwell_sec_{1.0};
    double slot_observe_max_dz_m_{0.15};
    double slot_observe_min_radial_m_{0.10};
    double slot_min_clearance_m_{0.07};
    double slot_container_clearance_m_{0.13};
    double slot_margin_cap_m_{0.20};
    double slot_tie_epsilon_m_{0.01};
    double slot_named_pose_error_m_{0.04};
    double max_pose_error_m_{0.025};
    bool slot_memory_enabled_{true};
    double slot_memory_ttl_sec_{900.0};
    bool slot_fallback_free_xy_enabled_{false};
    double slot_fallback_grid_step_m_{0.02};
    double slot_fallback_wrist_offset_rad_{0.0};
    bool slot_observe_empty_gripper_first_{false};
    double slot_overlap_refuse_margin_m_{-1.0};

    /// Obstaculo na mesa (tag vista agora ou slot usado antes), no frame da
    /// base do braco.
    struct TableObstacle
    {
        std::string frame;
        Eigen::Vector3d p{0.0, 0.0, 0.0};
        double clearance{0.07};
        std::string origin;   ///< "tf" | "memoria"
        double age_sec{0.0};
        rclcpp::Time stamp;
    };

    /// Pose nomeada candidata (MesaX / MesaX.N) com a FK do TCP.
    struct TableSlotCandidate
    {
        std::string name;
        std::array<double, 5> q{};
        Eigen::Vector3d p{0.0, 0.0, 0.0};
        bool is_center{false};
        double margin{0.0};
    };

    /// Onde o bloco foi (ou vai ser) solto — alimenta a memoria de slots.
    struct SlotDecision
    {
        std::string slot;          ///< pose nomeada ou "free_xy"
        std::string table_pose;    ///< nome-base da mesa
        Eigen::Vector3d p{0.0, 0.0, 0.0};
        double margin{0.0};
    };

    // Estado por goal (zerado em execute()).
    std::optional<rclcpp::Time> pegar_obj_arrival_time_;
    std::map<std::string, TableObstacle> early_obstacles_;
    std::optional<SlotDecision> last_slot_decision_;

    // Empilhamento (2026-08-24), ver placeStackedOnFrame.
    double stack_place_z_offset_{0.05};
    double stack_pre_lift_m_{0.05};
    bool stack_fallback_to_table_{true};
    double stack_arrival_tolerance_m_{0.03};
    std::vector<double> stack_tilt_ladder_deg_;
    /// Juntas da pre-pose acima da pilha do ciclo atual (subida apos soltar).
    std::optional<std::array<double, 5>> stack_lift_q_;

    // Container por cor na mesa (2026-08-25), ver placeIntoContainer.
    std::string container_frame_red_;
    std::string container_frame_blue_;
    double container_rim_height_m_{0.07};
    double container_drop_clearance_m_{0.03};
    double cube_height_m_{0.042};
    double container_detect_timeout_sec_{3.0};
    double container_arrival_tolerance_m_{0.03};   // erro em XY (o que decide se cai dentro)
    double container_arrival_z_tolerance_m_{0.06}; // erro em Z: o braco estendido afunda ~2,5 cm (medido 25/08)
    std::vector<double> container_tilt_ladder_deg_;
    bool container_fallback_to_table_{true};
    std::string container_plane_topic_;
    bool container_search_lateral_{true};
    double container_search_timeout_sec_{1.5};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr container_plane_z_pub_;
    /// Cota da borda publicada para o goal atual (autoridade do Z do drop).
    std::optional<double> container_plane_z_;
    /// O braco saiu de pegar_obj durante placeIntoContainer (fallback recolhe).
    bool container_arm_moved_{false};
    bool container_exit_pending_{false};   // soltou no container: fechar garra, subir j2/j3, depois o resto
    bool container_close_gripper_on_exit_{true};
    // Varios blocos no mesmo container: candidatos de deslocamento do ponto
    // de soltura no eixo LONGO/curto do container (m). O escolhido e' o mais
    // longe dos blocos ja soltos la (memoria de slots).
    std::vector<double> container_slot_offsets_long_m_{0.03, -0.03, 0.0};
    std::vector<double> container_slot_offsets_short_m_{0.0, 0.0, 0.0};
    double container_exit_gripper_position_{0.170};  // manip_joint6/7: fechada 0.170, aberta -0.2265

    // ---- Fila de alcance (2026-08-28) ----
    bool unreachable_bailout_enabled_{true};
    std::vector<double> unreachable_shift_candidates_m_{0.10, 0.15, 0.20, 0.25};
    double unreachable_shift_margin_m_{0.02};
    // Estado POR GOAL (zerado em execute()): o container/pilha foi VISTO e
    // nenhum candidato x inclinacao teve IK antes de qualquer movimento.
    bool last_failure_unreachable_{false};
    // Sugestao de deslocamento da base (base_footprint, + = esquerda; 0 =
    // de lado nao resolve), so calculada no caminho de saida cedo.
    double last_suggested_shift_m_{0.0};
    // Alvos 3D (manip_base_link) que a escada nao alcancou, na ordem de
    // preferencia (container: candidatos de slot; pilha: 1 alvo). O 1o vira
    // target_x/target_y do result e decide a direcao da busca.
    std::vector<Eigen::Vector3d> last_unreachable_targets_;
    /// O braco saiu de pegar_obj durante placeStackedOnFrame (pre-alinhamento
    /// lateral) — a devolucao do cubo passa por pegar_obj antes.
    bool stack_arm_moved_{false};
    /// Deslocamento lateral acumulado da base (m, + = esquerda), do
    /// /manip/base_shift_total (dock_align_node). 0 sem publicacao.
    std::atomic<double> base_shift_total_{0.0};
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr base_shift_total_subscription_;

    std::string camera_info_topic_;
    double perception_warmup_timeout_{6.0};
    double container_place_z_offset_;
    std::unique_ptr<manip_task_execution::ManipulatorExecutionLock> execution_lock_;
    bool speech_enabled_{true};
    bool skip_missing_place_tag_{true};
    bool verify_grasp_effort_{true};
    bool verify_container_grasp_{false};
    double grasp_min_effort_nm_{0.15};
    double grasp_min_effort_increase_nm_{0.05};
    double grasp_effort_sample_duration_{0.8};
    double grasp_effort_max_age_{0.4};
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
        dynamic_joint_state_subscription_;
    std::mutex effort_mutex_;
    double motor6_effort_{0.0};
    double motor7_effort_{0.0};
    std::chrono::steady_clock::time_point effort_update_time_;
    std::uint64_t effort_update_sequence_{0};
    bool effort_available_{false};
    // Item 2.5: causa da ultima falha (vira fail_reason no result).
    std::string last_place_failure_reason_;
    std::atomic_bool cancel_requested_{false};
    std::mutex active_interfaces_mutex_;
    std::shared_ptr<MoveGroupInterface> active_arm_;
    std::shared_ptr<MoveGroupInterface> active_gripper_;

    class ExecutionGuard
    {
    public:
        explicit ExecutionGuard(PlaceActionServer & server)
        : server_(server)
        {
        }

        ~ExecutionGuard()
        {
            release();
        }

        // Libera os recursos UMA unica vez por goal (verificacao adversarial
        // 2026-08-10): a 2a chamada soltava o flock ja readquirido pelo goal
        // seguinte (a flag do lock e compartilhada entre goals e entre os
        // nos pick/place).
        void release()
        {
            if (!released_) {
                released_ = true;
                server_.releaseExecutionResources();
            }
        }

    private:
        PlaceActionServer & server_;
        bool released_{false};
    };

    bool cancellationRequested() const
    {
        return cancel_requested_.load();
    }

    static std::string spokenTargetName(std::string target)
    {
        constexpr char tag_prefix[] = "tag_";
        if (target.rfind(tag_prefix, 0) == 0) {
            target.erase(0, sizeof(tag_prefix) - 1);
            target = "tag " + target;
        } else if (target.rfind("ct_", 0) == 0 && target.size() > 3) {
            target = "container " + target.substr(3);   // ct_vermelho -> container vermelho
        } else if (target.rfind("ct", 0) == 0 && target.size() > 2) {
            target = "container " + target.substr(2);
        }
        for (char & character : target) {
            if (character == '_') {
                character = ' ';
            }
        }
        return target;
    }

    void speak(const std::string & text)
    {
        if (!speech_enabled_ || text.empty()) {
            return;
        }

        std_msgs::msg::String message;
        message.data = text;
        speech_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "[SPEECH] %s", text.c_str());
    }

    void setActiveInterfaces(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper)
    {
        std::lock_guard<std::mutex> lock(active_interfaces_mutex_);
        active_arm_ = arm;
        active_gripper_ = gripper;
    }

    void clearActiveInterfaces()
    {
        std::lock_guard<std::mutex> lock(active_interfaces_mutex_);
        active_arm_.reset();
        active_gripper_.reset();
    }

    void releaseExecutionResources()
    {
        publishPlaceActive(false);
        // AUDITORIA 31/08 (espelho do pick, item 2.6): NAO zerar os MGIs —
        // manter um vivo preserva o CurrentStateMonitor aquecido e evita, a
        // CADA goal, re-parse do URDF (MBs) + ~10 endpoints DDS criados e
        // destruidos que TODOS os peers (Pi incluida) precisam rastrear.
        cancel_requested_.store(false);
        execution_lock_->release();
    }

    void publishPlaceActive(bool active)
    {
        std_msgs::msg::Bool message;
        message.data = active;
        place_active_publisher_->publish(message);
    }

    // Item 3.10: gemeo do waitForCameraStream do pick — ver comentario la.
    void waitForCameraStream(const std::string & cycle_name)
    {
        if (perception_warmup_timeout_ <= 0.0 ||
            this->count_publishers(camera_info_topic_) == 0)
        {
            return;
        }

        const auto now_ns =
            std::chrono::steady_clock::now().time_since_epoch().count();
        const auto last_ns = camera_info_last_ns_.load(std::memory_order_relaxed);
        constexpr std::int64_t kFreshNs = 500'000'000;  // 0,5 s
        if (last_ns != 0 && (now_ns - last_ns) < kFreshNs) {
            return;
        }

        const auto baseline = camera_info_count_.load(std::memory_order_relaxed);
        const auto start = std::chrono::steady_clock::now();
        const auto deadline =
            start + std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(perception_warmup_timeout_));

        while (std::chrono::steady_clock::now() < deadline) {
            if (cancellationRequested()) {
                return;
            }
            if (camera_info_count_.load(std::memory_order_relaxed) >= baseline + 2) {
                RCLCPP_INFO(
                    get_logger(),
                    "[%s] camera voltou a streamar em %.1fs",
                    cycle_name.c_str(),
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - start).count());
                return;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }

        RCLCPP_WARN(
            get_logger(),
            "[%s] camera nao entregou frame em %.1fs (%s) — seguindo assim mesmo.",
            cycle_name.c_str(),
            perception_warmup_timeout_,
            camera_info_topic_.c_str());
    }

    void stopActiveMotion()
    {
        std::shared_ptr<MoveGroupInterface> arm;
        std::shared_ptr<MoveGroupInterface> gripper;
        {
            std::lock_guard<std::mutex> lock(active_interfaces_mutex_);
            arm = active_arm_;
            gripper = active_gripper_;
        }
        if (arm) {
            arm->stop();
        }
        if (gripper) {
            gripper->stop();
        }
    }

    bool sleepInterruptibly(const std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            if (cancellationRequested()) {
                return false;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
    }

    void declarePlanningDefaults()
    {
        if (!this->has_parameter("ompl.planning_plugins")) {
            this->declare_parameter<std::vector<std::string>>(
                "ompl.planning_plugins",
                std::vector<std::string>{"ompl_interface/OMPLPlanner"});
        }
        if (!this->has_parameter("ompl.planning_plugin")) {
            this->declare_parameter<std::string>(
                "ompl.planning_plugin",
                "ompl_interface/OMPLPlanner");
        }
        if (!this->has_parameter("ompl.request_adapters")) {
            this->declare_parameter<std::vector<std::string>>(
                "ompl.request_adapters",
                std::vector<std::string>{
                    "default_planning_request_adapters/ResolveConstraintFrames",
                    "default_planning_request_adapters/ValidateWorkspaceBounds",
                    "default_planning_request_adapters/CheckStartStateBounds",
                    "default_planning_request_adapters/CheckStartStateCollision"});
        }
        if (!this->has_parameter("ompl.response_adapters")) {
            this->declare_parameter<std::vector<std::string>>(
                "ompl.response_adapters",
                std::vector<std::string>{
                    "default_planning_response_adapters/ValidateSolution",
                    "default_planning_response_adapters/DisplayMotionPath"});
        }
        if (!this->has_parameter("ompl.start_state_max_bounds_error")) {
            this->declare_parameter<double>("ompl.start_state_max_bounds_error", 0.1);
        }

        if (!this->has_parameter("robot_description_kinematics.arm.kinematics_solver")) {
            this->declare_parameter<std::string>(
                "robot_description_kinematics.arm.kinematics_solver",
                "pick_ik/PickIkPlugin");
        }
        if (!this->has_parameter("robot_description_kinematics.arm.kinematics_solver_timeout")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.kinematics_solver_timeout",
                0.2);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.mode")) {
            this->declare_parameter<std::string>(
                "robot_description_kinematics.arm.mode",
                "global");
        }
        if (!this->has_parameter("robot_description_kinematics.arm.position_scale")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.position_scale",
                1.0);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.rotation_scale")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.rotation_scale",
                0.03);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.position_threshold")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.position_threshold",
                0.001);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.orientation_threshold")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.orientation_threshold",
                0.08);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.cost_threshold")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.cost_threshold",
                0.001);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.minimal_displacement_weight")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.minimal_displacement_weight",
                0.02);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.gd_step_size")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.gd_step_size",
                0.0008);
        }

        if (!this->has_parameter("arm.kinematics_solver")) {
            this->declare_parameter<std::string>("arm.kinematics_solver", "pick_ik/PickIkPlugin");
        }
        if (!this->has_parameter("arm.kinematics_solver_timeout")) {
            this->declare_parameter<double>("arm.kinematics_solver_timeout", 0.2);
        }
        if (!this->has_parameter("arm.mode")) {
            this->declare_parameter<std::string>("arm.mode", "global");
        }
        if (!this->has_parameter("arm.position_scale")) {
            this->declare_parameter<double>("arm.position_scale", 1.0);
        }
        if (!this->has_parameter("arm.rotation_scale")) {
            this->declare_parameter<double>("arm.rotation_scale", 0.03);
        }
        if (!this->has_parameter("arm.position_threshold")) {
            this->declare_parameter<double>("arm.position_threshold", 0.001);
        }
        if (!this->has_parameter("arm.orientation_threshold")) {
            this->declare_parameter<double>("arm.orientation_threshold", 0.30);
        }
        if (!this->has_parameter("arm.cost_threshold")) {
            this->declare_parameter<double>("arm.cost_threshold", 0.001);
        }
        if (!this->has_parameter("arm.minimal_displacement_weight")) {
            this->declare_parameter<double>("arm.minimal_displacement_weight", 0.02);
        }
        if (!this->has_parameter("arm.gd_step_size")) {
            this->declare_parameter<double>("arm.gd_step_size", 0.0008);
        }

        std::vector<std::string> planning_plugins;
        if (!this->get_parameter("ompl.planning_plugins", planning_plugins) ||
            planning_plugins.empty() || planning_plugins.front().empty()) {
            this->set_parameter(
                rclcpp::Parameter(
                    "ompl.planning_plugins",
                    std::vector<std::string>{"ompl_interface/OMPLPlanner"}));
        }

        std::string planning_plugin;
        if (!this->get_parameter("ompl.planning_plugin", planning_plugin) ||
            planning_plugin.empty()) {
            this->set_parameter(
                rclcpp::Parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner"));
        }

        std::string kinematics_solver;
        if (!this->get_parameter(
                "robot_description_kinematics.arm.kinematics_solver",
                kinematics_solver) ||
            kinematics_solver.empty()) {
            this->set_parameter(
                rclcpp::Parameter(
                    "robot_description_kinematics.arm.kinematics_solver",
                    "pick_ik/PickIkPlugin"));
        }

        if (!this->get_parameter("arm.kinematics_solver", kinematics_solver) ||
            kinematics_solver.empty()) {
            this->set_parameter(
                rclcpp::Parameter("arm.kinematics_solver", "pick_ik/PickIkPlugin"));
        }
    }

    void publish_stage(
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle,
        const std::string & stage)
    {
        auto feedback = std::make_shared<PlaceTag::Feedback>();
        feedback->current_stage = stage;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "[PLACE] stage=%s", stage.c_str());
    }

    // Auditoria 2026-08-07, item 1.2: sem prazo, o construtor do
    // MoveGroupInterface espera o action server do move_group PARA SEMPRE.
    // 15s folgados; no estouro devolve nullptr e o chamador falha o goal.
    std::shared_ptr<MoveGroupInterface> makeInterfaceWithTimeout(
        const std::string & group)
    {
        // Sonda ANTES de construir (teste mock 2026-08-07): sem move_group no
        // ar, o construtor do MGI aborta o processo inteiro (SIGABRT em thread
        // interna do MoveIt, fora do alcance do try/catch).
        auto probe = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
            shared_from_this(), "move_action");
        if (!probe->wait_for_action_server(std::chrono::seconds(15))) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "move_group ('/move_action') indisponivel em 15s — nao "
                "construindo MoveGroupInterface('" << group << "').");
            return nullptr;
        }
        try {
            return std::make_shared<MoveGroupInterface>(
                shared_from_this(), group,
                std::shared_ptr<tf2_ros::Buffer>(),
                rclcpp::Duration::from_seconds(15.0));
        } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "MoveGroupInterface('" << group << "') indisponivel em 15s: "
                    << e.what());
            return nullptr;
        }
    }

    // ---- Deadlines client-side (auditoria 2026-08-07, item 1.1) ----
    // Espelho exato dos wrappers do pick: plan/execute do MoveGroupInterface
    // bloqueiam SEM timeout; resposta perdida no DDS = espera infinita (e no
    // place a thread pendurada segura o flock do braco, matando pick E place).
    // Prazos largos: plan 25s; execute = duracao planejada x3 + 3s + 10s.
    template<typename Fn>
    bool runWithDeadline(Fn && fn, double timeout_s, const std::string & label)
    {
        auto prom = std::make_shared<std::promise<bool>>();
        auto fut = prom->get_future();
        std::thread(
            [prom, fn = std::forward<Fn>(fn)]() mutable {
                bool ok = false;
                try {
                    ok = fn();
                } catch (...) {
                }
                try {
                    prom->set_value(ok);
                } catch (...) {
                }
            }).detach();
        if (fut.wait_for(std::chrono::duration<double>(timeout_s)) ==
            std::future_status::ready)
        {
            return fut.get();
        }
        RCLCPP_FATAL_STREAM(
            get_logger(),
            "DEADLINE de " << timeout_s << "s estourado em '" << label
                << "'. Olhe o log do move_group ANTES de culpar a rede: "
                   "(a) 'Controller is taking too long to execute trajectory' "
                   "= o braco pode ter CHEGADO e o joint_trajectory_controller "
                   "nao declarou sucesso (tolerancia/goal_time do controlador "
                   "na Pi) — acao saudavel cortada, ajuste as constraints; "
                   "(b) silencio no move_group = chamada realmente pendurada "
                   "(perda DDS), e estouros repetidos pedem reiniciar o no.");
        return false;
    }

    static double plannedTrajectoryDurationSec(const MoveGroupInterface::Plan & plan)
    {
        const auto & pts = plan.trajectory.joint_trajectory.points;
        if (pts.empty()) {
            return 0.0;
        }
        return rclcpp::Duration(pts.back().time_from_start).seconds();
    }

    bool planWithDeadline(
        const std::shared_ptr<MoveGroupInterface> & iface,
        MoveGroupInterface::Plan & plan_out,
        const std::string & label)
    {
        auto plan_box = std::make_shared<MoveGroupInterface::Plan>();
        const bool ok = runWithDeadline(
            [iface, plan_box]() {
                return iface->plan(*plan_box) ==
                       moveit::core::MoveItErrorCode::SUCCESS;
            },
            25.0, label + " [plan]");
        if (ok) {
            plan_out = *plan_box;
        } else {
            iface->stop();
        }
        return ok;
    }

    bool executeWithDeadline(
        const std::shared_ptr<MoveGroupInterface> & iface,
        const MoveGroupInterface::Plan & plan,
        const std::string & label)
    {
        const double timeout_s =
            plannedTrajectoryDurationSec(plan) * 3.0 + 3.0 + 10.0;
        auto plan_box = std::make_shared<MoveGroupInterface::Plan>(plan);
        const bool ok = runWithDeadline(
            [iface, plan_box]() {
                return iface->execute(*plan_box) ==
                       moveit::core::MoveItErrorCode::SUCCESS;
            },
            timeout_s, label + " [execute]");
        if (!ok) {
            iface->stop();
        }
        return ok;
    }

    bool planAndExecute(
        const std::shared_ptr<MoveGroupInterface> & iface,
        const std::string & label)
    {
        if (cancellationRequested()) {
            return false;
        }

        MoveGroupInterface::Plan plan;

        if (!planWithDeadline(iface, plan, label)) {
            RCLCPP_ERROR_STREAM(get_logger(), "Planning failed: " << label);
            return false;
        }

        if (cancellationRequested()) {
            return false;
        }

        const bool exec_ok = executeWithDeadline(iface, plan, label);
        if (cancellationRequested()) {
            return false;
        }
        if (!exec_ok) {
            RCLCPP_ERROR_STREAM(get_logger(), "Execution failed: " << label);
            return false;
        }

        return true;
    }

    /// Move o braco para um alvo em espaco de juntas (5 valores) planejando
    /// pelo MoveIt — mantem checagem de colisao, deadlines e retries.
    /// Espelho do moveToJointTarget validado no no de pick (2026-08-17).
    bool moveToJointTarget(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::array<double, 5> & q,
        const std::string & label)
    {
        if (cancellationRequested()) {
            return false;
        }
        if (!arm->getCurrentState(2.0)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Sem estado atual do braco antes de '%s'.", label.c_str());
            return false;
        }

        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setJointValueTarget(std::vector<double>(q.begin(), q.end()));
        if (planAndExecute(arm, label)) {
            return true;
        }

        // Loga o erro por junta para o teste apontar o culpado na hora
        // (mesma telemetria do pick, 2026-08-13).
        const std::vector<double> atual = arm->getCurrentJointValues();
        for (std::size_t i = 0; i < q.size() && i < atual.size(); ++i) {
            const double erro = atual[i] - q[i];
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] j%zu alvo=%.4f atual=%.4f erro=%+.4f rad%s",
                label.c_str(), i + 1, q[i], atual[i], erro,
                std::abs(erro) > 0.10 ? "  <-- FORA da tolerancia 0.10" : "");
        }
        return false;
    }

    static bool isContainerTarget(const std::string & target)
    {
        if (target.size() <= 2 || target[0] != 'c' || target[1] != 't') {
            return false;
        }

        for (std::size_t i = 2; i < target.size(); ++i) {
            if (!std::isdigit(static_cast<unsigned char>(target[i]))) {
                return false;
            }
        }

        return true;
    }

    static bool isTagTarget(const std::string & target)
    {
        return target.rfind("tag_", 0) == 0;
    }

    static bool isTfPlaceTarget(const std::string & target)
    {
        return isContainerTarget(target) || isTagTarget(target);
    }

    /// Pose de mesa que fica numa prateleira (entrega com waypoint).
    bool isShelfPlaceTarget(const std::string & target) const
    {
        return std::find(
            shelf_table_poses_.begin(), shelf_table_poses_.end(),
            target) != shelf_table_poses_.end();
    }

    /// Waypoint obrigatorio da prateleira. Falhar aqui e falhar a entrega:
    /// seguir direto para a pose de mesa bate na estante.
    bool goToShelfWaypoint(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & label)
    {
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget(kShelfWaypointPose);
        return planAndExecute(arm, label);
    }

    static constexpr const char * kShelfWaypointPose = "pirocao";

    geometry_msgs::msg::TransformStamped getTagTransform(
        const std::string & reference_frame,
        const std::string & tag_frame) const
    {
        return tf_buffer_->lookupTransform(
            reference_frame,
            tag_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(0.5));
    }

    bool waitForTagTransform(
        const std::string & reference_frame,
        const std::string & tag_frame,
        geometry_msgs::msg::TransformStamped & out_tf,
        const std::chrono::milliseconds timeout,
        const std::chrono::milliseconds retry_period,
        const std::string & cycle_name)
    {
        const auto start = std::chrono::steady_clock::now();
        tf2::TransformException last_ex("unknown TF error");
        double stale_age_sec = -1.0;

        while (std::chrono::steady_clock::now() - start < timeout) {
            if (cancellationRequested()) {
                RCLCPP_WARN(
                    get_logger(),
                    "[%s] canceled while waiting for TF",
                    cycle_name.c_str());
                return false;
            }
            try {
                out_tf = getTagTransform(reference_frame, tag_frame);
                // Transform existir nao basta: precisa ser deteccao RECENTE.
                const double age_sec =
                    (this->get_clock()->now() - rclcpp::Time(out_tf.header.stamp)).seconds();
                if (age_sec <= max_tag_age_sec_) {
                    return true;
                }
                stale_age_sec = age_sec;
            } catch (const tf2::TransformException & ex) {
                last_ex = ex;
            }

            if (!sleepInterruptibly(retry_period)) {
                return false;
            }
        }

        if (stale_age_sec >= 0.0) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "[" << cycle_name << "] Tag " << tag_frame
                    << " vista pela ultima vez ha " << stale_age_sec
                    << " s (max " << max_tag_age_sec_
                    << " s) — a camera NAO esta vendo a tag agora.");
        } else {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "[" << cycle_name << "] Timed out waiting TF "
                    << reference_frame << " <- " << tag_frame
                    << " after " << timeout.count() << " ms. Last error: "
                    << last_ex.what());
        }
        return false;
    }

    double max_tag_age_sec_{1.0};

    // moveToTarget (setPoseTarget + fallback setApproximateJointValueTarget)
    // REMOVIDO em 2026-08-17: o fallback aproximado e o anti-padrao que ja
    // fechou garra no vazio no pick. A entrega em alvo TF agora usa a IK
    // custom abaixo — pedido do operador ("todas as IKs viram a custom").

    /// Aproximacao final da entrega em alvo TF (ct*/tag_*) pela IK custom
    /// (kDown, j5 alinhado ao yaw do frame alvo). Tentativa UNICA, sem
    /// fallback para o MoveIt: falhou, o goal falha com o bloco na garra
    /// (contrato falha_com_bloco_na_garra do chamador).
    /// `target_tf` PRECISA estar no frame da base do braco
    /// (ik_reference_frame_) e ja vir com o container_place_z_offset_ somado.
    bool approachPlaceTargetCustomIk(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const geometry_msgs::msg::TransformStamped & target_tf,
        const std::string & label)
    {
        const Eigen::Vector3d target(
            target_tf.transform.translation.x,
            target_tf.transform.translation.y,
            target_tf.transform.translation.z);

        std::array<double, 5> q{};
        if (!manip_task_execution::solveIk(
                target,
                manip_task_execution::ToolDirection::kDown,
                0.0,
                q))
        {
            RCLCPP_ERROR(
                get_logger(),
                "[PLACE] IK propria nao achou solucao para [%.3f %.3f %.3f] "
                "em %s (%s).",
                target.x(), target.y(), target.z(),
                ik_reference_frame_.c_str(), label.c_str());
            speak("Não consegui posicionar o braço para a entrega");
            return false;
        }

        // O TCP esta no eixo do joint5: trocar q5 pos-solve nao move a
        // ponta, so alinha a linha dos dedos ao yaw do alvo.
        const double target_yaw = manip_task_execution::projectedFrameYaw(
            Eigen::Quaterniond(
                target_tf.transform.rotation.w,
                target_tf.transform.rotation.x,
                target_tf.transform.rotation.y,
                target_tf.transform.rotation.z).toRotationMatrix());
        q[4] = manip_task_execution::computeWristForTagYaw(target_yaw, q[0]);

        RCLCPP_INFO(
            get_logger(),
            "[PLACE] IK custom: alvo [%.3f %.3f %.3f] yaw=%.4f -> j "
            "[%.4f %.4f %.4f %.4f %.4f]",
            target.x(), target.y(), target.z(), target_yaw,
            q[0], q[1], q[2], q[3], q[4]);

        return moveToJointTarget(arm, q, label);
    }

    // =====================================================================
    // Multi-place na mesa comum (2026-08-21)
    //
    // Fluxo: o braco chega a pegar_obj com o bloco -> olha a mesa (TF das
    // tags conhecidas, so as vistas AGORA) + memoria dos slots que o proprio
    // robo ja usou nesta estacao -> pontua as poses nomeadas MesaX/MesaX.N
    // pela distancia a cada obstaculo -> vai para a melhor. Sem folga em
    // nenhuma: (flag) XY livre pela IK custom, senao "menos ruim" com aviso
    // (decisao do operador). A escolha e a UNICA protecao: a garra nao tem
    // geometria de colisao perto do TCP e a planning scene esta vazia — por
    // isso todo re-alvo apos falha parcial passa ANTES por pegar_obj.
    // =====================================================================

    void reloadSlotParameters()
    {
        slot_selection_enabled_ = this->get_parameter("slot_selection_enabled").as_bool();
        slot_table_poses_ = this->get_parameter("slot_table_poses").as_string_array();
        slot_selection_ws_prefixes_ =
            this->get_parameter("slot_selection_ws_prefixes").as_string_array();
        slot_excluded_poses_ = this->get_parameter("slot_excluded_poses").as_string_array();
        slot_obstacle_tag_frames_ =
            this->get_parameter("slot_obstacle_tag_frames").as_string_array();
        slot_observe_dwell_sec_ = this->get_parameter("slot_observe_dwell_sec").as_double();
        slot_observe_max_dz_m_ = this->get_parameter("slot_observe_max_dz_m").as_double();
        slot_observe_min_radial_m_ =
            this->get_parameter("slot_observe_min_radial_m").as_double();
        slot_min_clearance_m_ = this->get_parameter("slot_min_clearance_m").as_double();
        slot_container_clearance_m_ =
            this->get_parameter("slot_container_clearance_m").as_double();
        slot_margin_cap_m_ = this->get_parameter("slot_margin_cap_m").as_double();
        slot_tie_epsilon_m_ = this->get_parameter("slot_tie_epsilon_m").as_double();
        slot_named_pose_error_m_ = this->get_parameter("slot_named_pose_error_m").as_double();
        max_pose_error_m_ = this->get_parameter("max_pose_error_m").as_double();
        slot_memory_enabled_ = this->get_parameter("slot_memory_enabled").as_bool();
        slot_memory_ttl_sec_ = this->get_parameter("slot_memory_ttl_sec").as_double();
        slot_fallback_free_xy_enabled_ =
            this->get_parameter("slot_fallback_free_xy_enabled").as_bool();
        slot_fallback_grid_step_m_ =
            this->get_parameter("slot_fallback_grid_step_m").as_double();
        slot_fallback_wrist_offset_rad_ =
            this->get_parameter("slot_fallback_wrist_offset_rad").as_double();
        slot_observe_empty_gripper_first_ =
            this->get_parameter("slot_observe_empty_gripper_first").as_bool();
        slot_overlap_refuse_margin_m_ =
            this->get_parameter("slot_overlap_refuse_margin_m").as_double();
    }

    static std::string normalizedWs(std::string ws)
    {
        const auto not_space = [](unsigned char c) {return !std::isspace(c);};
        ws.erase(ws.begin(), std::find_if(ws.begin(), ws.end(), not_space));
        ws.erase(std::find_if(ws.rbegin(), ws.rend(), not_space).base(), ws.end());
        for (char & c : ws) {
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
        return ws;
    }

    /// A escolha de slot so vale para mesa comum de uma WS_*: nunca para
    /// container/tag (TF), prateleira, precision placement (PP_*) nem para
    /// um goal que ja pede um slot explicito ("Mesa15.2").
    bool slotSelectionApplies(
        const std::string & table_pose,
        const std::string & ws_raw,
        std::string * why) const
    {
        const auto reject = [&](const std::string & reason) {
                if (why) {*why = reason;}
                return false;
            };
        if (!slot_selection_enabled_) {
            return reject("slot_selection_enabled=false");
        }
        if (isTfPlaceTarget(table_pose)) {
            return reject("alvo TF (container/tag)");
        }
        if (isShelfPlaceTarget(table_pose)) {
            return reject("prateleira");
        }
        if (std::find(slot_table_poses_.begin(), slot_table_poses_.end(), table_pose) ==
            slot_table_poses_.end())
        {
            return reject("table_pose '" + table_pose + "' fora de slot_table_poses");
        }
        const std::string ws = normalizedWs(ws_raw);
        if (ws.empty()) {
            return reject("goal sem 'ws' (legado)");
        }
        for (const auto & prefix : slot_selection_ws_prefixes_) {
            if (ws.rfind(normalizedWs(prefix), 0) == 0) {
                return true;
            }
        }
        return reject("ws '" + ws + "' nao e mesa comum (prefixos: WS)");
    }

    static double wrapAngle(double a)
    {
        while (a > M_PI) {a -= 2.0 * M_PI;}
        while (a < -M_PI) {a += 2.0 * M_PI;}
        return a;
    }

    /// Pedido do operador 2026-08-21: no place em mesa o motor 2 (ombro) e o
    /// ULTIMO a se mover — primeiro j1/j3/j4/j5 com o ombro parado (o braco se
    /// arruma por cima da mesa), depois so o j2 desce ate o alvo. Sem estado
    /// atual, cai no movimento unico.
    bool moveToJointTargetJoint2Last(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::array<double, 5> & q,
        const std::string & label)
    {
        const auto state = arm->getCurrentState(2.0);
        std::vector<double> cur;
        if (state) {
            state->copyJointGroupPositions(arm->getName(), cur);
        }
        if (cur.size() < 5) {
            RCLCPP_WARN(get_logger(), "[%s] sem estado atual — movimento unico (j2 junto).", label.c_str());
            return moveToJointTarget(arm, q, label);
        }
        std::array<double, 5> first = q;
        first[1] = cur[1];
        if (std::abs(first[1] - q[1]) > 1e-3) {
            if (!moveToJointTarget(arm, first, label + " [j2 parado]")) {
                return false;
            }
        }
        return moveToJointTarget(arm, q, label + " [j2 por ultimo]");
    }

    /// Desloca `target` (x,y) para um dos candidatos de
    /// container_slot_offsets_{long,short}_m, expressos no eixo longo/curto do
    /// container (yaw do frame). Sem bloco anterior na memoria: candidato 0.
    /// Com blocos: o candidato mais longe do bloco mais proximo — robusto ao
    /// sinal do yaw (pode virar 180 graus entre deteccoes) e a onde o 1o caiu.
    std::vector<Eigen::Vector2d> containerSlotCandidates(
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle,
        const std::string & frame,
        double frame_yaw,
        const Eigen::Vector3d & target)
    {
        const auto & offs_l = container_slot_offsets_long_m_;
        const auto & offs_s = container_slot_offsets_short_m_;
        if (offs_l.empty()) {
            return {Eigen::Vector2d(target.x(), target.y())};
        }
        std::vector<Eigen::Vector2d> prior;
        const std::string ws = normalizedWs(goal_handle->get_goal()->ws);
        // Memoria gravada no frame da pose de docking; a base ja pode ter
        // andado de lado (fila de alcance, 2026-08-28): ponto fixo na mesa
        // aparece em x + offset no manip_base_link de agora.
        const double offset = base_shift_total_.load(std::memory_order_relaxed);
        if (slot_memory_enabled_ && !ws.empty()) {
            std::vector<manip_task_execution::TableSlotRecord> records;
            std::string err;
            if (container_state_store_->getTableSlotsUsed(
                    ws, slot_memory_ttl_sec_, this->get_clock()->now().seconds(), &records, &err))
            {
                for (const auto & rec : records) {
                    if (rec.slot == frame && rec.frame == ik_reference_frame_) {
                        prior.emplace_back(rec.x + offset, rec.y);
                    }
                }
            } else {
                RCLCPP_WARN(get_logger(), "[CONTAINER] memoria de slots ilegivel (%s) — solto no candidato 0.", err.c_str());
            }
        }
        const double c = std::cos(frame_yaw);
        const double sn = std::sin(frame_yaw);
        std::vector<Eigen::Vector2d> cands;
        for (std::size_t k = 0; k < offs_l.size(); ++k) {
            const double dl = offs_l[k];
            const double ds = k < offs_s.size() ? offs_s[k] : 0.0;
            cands.emplace_back(target.x() + dl * c - ds * sn, target.y() + dl * sn + ds * c);
        }
        std::size_t best = 0;
        double best_score = -1.0;
        if (!prior.empty()) {
            for (std::size_t k = 0; k < cands.size(); ++k) {
                double score = std::numeric_limits<double>::infinity();
                for (const auto & p : prior) {
                    score = std::min(score, (cands[k] - p).norm());
                }
                if (score > best_score) {
                    best_score = score;
                    best = k;
                }
            }
        }
        RCLCPP_INFO(
            get_logger(),
            "[CONTAINER] %zu bloco(s) ja soltos em %s (memoria): candidato %zu/%zu "
            "(%+.3f longo, %+.3f curto) -> [%.3f %.3f]%s",
            prior.size(), frame.c_str(), best + 1, cands.size(),
            offs_l[best], best < offs_s.size() ? offs_s[best] : 0.0,
            cands[best].x(), cands[best].y(),
            prior.empty() ? "" : (", " + std::to_string(best_score * 100.0).substr(0, 4) + " cm do bloco mais proximo").c_str());
        // Ordem: preferido, depois os outros por distancia ao bloco mais
        // proximo (maior primeiro) — se o preferido nao tiver IK (28/08: pote
        // longe a direita, +3 cm caia fora do alcance), tenta os seguintes.
        std::vector<std::pair<double, std::size_t>> order;
        for (std::size_t k = 0; k < cands.size(); ++k) {
            double score = 0.0;
            if (!prior.empty()) {
                score = std::numeric_limits<double>::infinity();
                for (const auto & p : prior) {
                    score = std::min(score, (cands[k] - p).norm());
                }
            }
            order.emplace_back(k == best ? std::numeric_limits<double>::infinity() : score, k);
        }
        std::stable_sort(order.begin(), order.end(),
                         [](const auto & a, const auto & b) { return a.first > b.first; });
        std::vector<Eigen::Vector2d> ranked;
        for (const auto & [score, k] : order) {
            (void)score;
            ranked.push_back(cands[k]);
        }
        return ranked;
    }

    /// Como moveToJointTargetJoint2Last, mas segurando ombro (j2) E cotovelo
    /// (j3): primeiro base/punho (j1, j4, j5) com o braco na altura atual, so'
    /// depois j2+j3 juntos. Evita que o cotovelo abaixe o TCP enquanto a base
    /// ainda gira (esbarrou na parede do container em 25/08).
    bool moveToJointTargetJoints23Last(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::array<double, 5> & q,
        const std::string & label)
    {
        const auto state = arm->getCurrentState(2.0);
        std::vector<double> cur;
        if (state) {
            state->copyJointGroupPositions(arm->getName(), cur);
        }
        if (cur.size() < 5) {
            RCLCPP_WARN(get_logger(), "[%s] sem estado atual — movimento unico (j2/j3 juntos).", label.c_str());
            return moveToJointTarget(arm, q, label);
        }
        std::array<double, 5> first = q;
        first[1] = cur[1];
        first[2] = cur[2];
        if (std::abs(first[1] - q[1]) > 1e-3 || std::abs(first[2] - q[2]) > 1e-3) {
            if (!moveToJointTarget(arm, first, label + " [j2/j3 parados]")) {
                return false;
            }
        }
        return moveToJointTarget(arm, q, label + " [j2/j3 por ultimo]");
    }

    /// Inverso de moveToJointTargetJoints23Last: primeiro ombro+cotovelo (j2,
    /// j3) com base/punho parados (sobe na vertical), depois j1/j4/j5.
    bool moveToJointTargetJoints23First(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::array<double, 5> & q,
        const std::string & label)
    {
        const auto state = arm->getCurrentState(2.0);
        std::vector<double> cur;
        if (state) {
            state->copyJointGroupPositions(arm->getName(), cur);
        }
        if (cur.size() < 5) {
            RCLCPP_WARN(get_logger(), "[%s] sem estado atual — movimento unico.", label.c_str());
            return moveToJointTarget(arm, q, label);
        }
        std::array<double, 5> first{cur[0], q[1], q[2], cur[3], cur[4]};
        if (std::abs(first[1] - cur[1]) > 1e-3 || std::abs(first[2] - cur[2]) > 1e-3) {
            if (!moveToJointTarget(arm, first, label + " [so j2/j3]")) {
                return false;
            }
        }
        return moveToJointTarget(arm, q, label + " [resto]");
    }

    /// Juntas j1..j5 de uma pose nomeada do SRDF (false se faltar alguma).
    bool namedPoseJoints(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & name,
        std::array<double, 5> & q) const
    {
        static const std::array<const char *, 5> kJoints{
            "manip_joint1", "manip_joint2", "manip_joint3", "manip_joint4", "manip_joint5"};
        const std::map<std::string, double> jv = arm->getNamedTargetValues(name);
        for (std::size_t i = 0; i < kJoints.size(); ++i) {
            const auto it = jv.find(kJoints[i]);
            if (it == jv.end()) {
                return false;
            }
            q[i] = it->second;
        }
        return true;
    }

    /// Poses nomeadas do SRDF que pertencem a mesa `base` (base, base.1,
    /// base.2, ...), com a FK do TCP no frame da base do braco. Nunca decide
    /// pelo sufixo: na Mesa5 o .1 e a ESQUERDA e o .2 a direita, ao contrario
    /// das outras. Rejeita a familia "de costas" (q3<0 ou q1 longe do azimute
    /// do alvo), o mesmo criterio do require_forward da IK custom.
    std::vector<TableSlotCandidate> enumerateTableSlots(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & base,
        std::string * error) const
    {
        static const std::array<const char *, 5> kJoints{
            "manip_joint1", "manip_joint2", "manip_joint3", "manip_joint4", "manip_joint5"};

        std::vector<TableSlotCandidate> out;
        const std::vector<std::string> names = arm->getNamedTargets();
        for (const std::string & name : names) {
            bool belongs = (name == base);
            if (!belongs && name.size() > base.size() + 1 &&
                name.compare(0, base.size(), base) == 0 && name[base.size()] == '.')
            {
                belongs = std::all_of(
                    name.begin() + static_cast<long>(base.size()) + 1, name.end(),
                    [](unsigned char c) {return std::isdigit(c) != 0;});
            }
            if (!belongs) {
                continue;
            }
            if (std::find(slot_excluded_poses_.begin(), slot_excluded_poses_.end(), name) !=
                slot_excluded_poses_.end())
            {
                RCLCPP_WARN(get_logger(), "[SLOT] %s excluida por parametro.", name.c_str());
                continue;
            }

            const std::map<std::string, double> jv = arm->getNamedTargetValues(name);
            TableSlotCandidate c;
            c.name = name;
            c.is_center = (name == base);
            bool complete = true;
            for (std::size_t i = 0; i < kJoints.size(); ++i) {
                const auto it = jv.find(kJoints[i]);
                if (it == jv.end()) {
                    complete = false;
                    break;
                }
                c.q[i] = it->second;
            }
            if (!complete) {
                RCLCPP_WARN(
                    get_logger(), "[SLOT] %s sem as 5 juntas no SRDF — ignorada.",
                    name.c_str());
                continue;
            }

            Eigen::Matrix3d rot;
            manip_task_execution::forwardKinematics(
                c.q, manip_task_execution::ArmModel{}, c.p, rot);

            const double azimuth = std::atan2(c.p.y(), c.p.x());
            const double dq1 = wrapAngle(c.q[0] - azimuth);
            if (c.q[2] < 0.0 || std::abs(dq1) > M_PI / 2.0) {
                RCLCPP_WARN(
                    get_logger(),
                    "[SLOT] %s e da familia 'de costas' (q1-azimute=%.0f graus, q3=%.3f) — "
                    "ignorada (regravar a pose no SRDF).",
                    name.c_str(), dq1 * 180.0 / M_PI, c.q[2]);
                continue;
            }
            out.push_back(c);
        }

        // Centro primeiro (desempate), depois ordem alfabetica estavel.
        std::stable_sort(
            out.begin(), out.end(),
            [](const TableSlotCandidate & a, const TableSlotCandidate & b) {
                if (a.is_center != b.is_center) {return a.is_center;}
                return a.name < b.name;
            });

        const bool has_center = !out.empty() && out.front().is_center;
        if (!has_center || out.size() < 2) {
            if (error) {
                *error = "mesa '" + base + "': " + std::to_string(out.size()) +
                    " pose(s) valida(s)" + (has_center ? "" : ", sem a pose central");
            }
            return {};
        }
        return out;
    }

    double clearanceForFrame(const std::string & frame) const
    {
        const bool container = frame.rfind("ct_", 0) == 0 || isContainerTarget(frame);
        return container ? slot_container_clearance_m_ : slot_min_clearance_m_;
    }

    /// Olha a mesa pelo TF: para cada frame conhecido, a transform mais
    /// recente SEM esperar (TimePointZero, sem timeout) e so se foi vista
    /// agora (max_tag_age_sec_). Janela curta para nao depender de um unico
    /// quadro. `not_before` descarta deteccoes de antes de o braco parar
    /// (com 0,3 s de tolerancia: tag vista na chegada e depois ocluida pelo
    /// bloco ainda vale). Devolve false so em cancelamento.
    bool observeTableObstacles(
        const std::string & carried_tag,
        const Eigen::Vector3d & center_p,
        const std::optional<rclcpp::Time> & not_before,
        std::map<std::string, TableObstacle> & out)
    {
        waitForCameraStream("PLACE slot");

        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(std::max(0.1, slot_observe_dwell_sec_)));

        while (true) {
            const rclcpp::Time now = this->get_clock()->now();
            for (const std::string & frame : slot_obstacle_tag_frames_) {
                if (frame.empty() || frame == carried_tag) {
                    continue;
                }
                if (!tf_buffer_->canTransform(ik_reference_frame_, frame, tf2::TimePointZero)) {
                    continue;
                }
                geometry_msgs::msg::TransformStamped tf;
                try {
                    tf = tf_buffer_->lookupTransform(
                        ik_reference_frame_, frame, tf2::TimePointZero);
                } catch (const tf2::TransformException &) {
                    continue;
                }
                const rclcpp::Time stamp(tf.header.stamp);
                const double age = (now - stamp).seconds();
                if (age > max_tag_age_sec_) {
                    continue;  // tag velha no buffer = fantasma
                }
                if (not_before &&
                    stamp < (*not_before - rclcpp::Duration::from_seconds(0.3)))
                {
                    continue;
                }
                const Eigen::Vector3d p(
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);
                if (std::abs(p.z() - center_p.z()) > slot_observe_max_dz_m_) {
                    continue;  // prateleira, parede, outro nivel
                }
                if (p.y() < slot_observe_min_radial_m_) {
                    continue;  // encostado no eixo do braco: nao e mesa
                }
                auto it = out.find(frame);
                if (it != out.end() && it->second.origin == "tf" && !(stamp > it->second.stamp)) {
                    continue;
                }
                TableObstacle ob;
                ob.frame = frame;
                ob.p = p;
                ob.clearance = clearanceForFrame(frame);
                ob.origin = "tf";
                ob.age_sec = age;
                ob.stamp = stamp;
                out[frame] = ob;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                break;
            }
            if (!sleepInterruptibly(std::chrono::milliseconds(100))) {
                return false;
            }
        }
        return true;
    }

    /// Slots que o proprio robo ja usou nesta estacao (yaml, sobrevive ao
    /// respawn) viram obstaculos virtuais — cobre o caso da tag do 1o bloco
    /// nao ser vista na 2a passada. +2 cm de folga pelo erro de re-docking.
    void addMemoryObstacles(
        const std::string & ws,
        std::map<std::string, TableObstacle> & out) const
    {
        if (!slot_memory_enabled_ || ws.empty()) {
            return;
        }
        std::vector<manip_task_execution::TableSlotRecord> records;
        std::string err;
        const double now_sec = this->get_clock()->now().seconds();
        if (!container_state_store_->getTableSlotsUsed(
                ws, slot_memory_ttl_sec_, now_sec, &records, &err))
        {
            RCLCPP_WARN(get_logger(), "[SLOT] memoria de slots ilegivel (%s) — seguindo so com a camera.", err.c_str());
            return;
        }
        std::size_t n = 0;
        // Memoria no frame da pose de docking -> x + deslocamento lateral
        // acumulado da base (fila de alcance, 2026-08-28).
        const double offset = base_shift_total_.load(std::memory_order_relaxed);
        for (const auto & rec : records) {
            if (rec.frame != ik_reference_frame_) {
                continue;
            }
            const std::string key = "memoria:" + rec.tag_frame + "@" + rec.slot;
            TableObstacle ob;
            ob.frame = key;
            ob.p = Eigen::Vector3d(rec.x + offset, rec.y, rec.z);
            // Slot "ct_vermelho"/"ct_azul" = bloco solto DENTRO de um container:
            // folga de container (0,13), nao a do cubo.
            ob.clearance = clearanceForFrame(
                rec.slot.rfind("ct_", 0) == 0 ? rec.slot : rec.tag_frame) + 0.02;
            ob.origin = "memoria";
            ob.age_sec = now_sec - rec.stamp_sec;
            out[key] = ob;
            ++n;
        }
        if (n > 0) {
            RCLCPP_INFO(get_logger(), "[SLOT] %zu slot(s) ja usado(s) em %s entraram como obstaculo.", n, ws.c_str());
        }
    }

    double marginAt(
        const Eigen::Vector3d & p,
        const std::map<std::string, TableObstacle> & obstacles) const
    {
        double margin = slot_margin_cap_m_;
        for (const auto & kv : obstacles) {
            const double d = std::hypot(p.x() - kv.second.p.x(), p.y() - kv.second.p.y());
            margin = std::min(margin, d - kv.second.clearance);
        }
        return margin;
    }

    /// Indice do melhor candidato (maior margem); empate dentro de epsilon
    /// fica com quem vem antes na lista (centro primeiro). `skip` exclui um
    /// indice (para achar o 2o melhor).
    int bestCandidate(const std::vector<TableSlotCandidate> & c, int skip = -1) const
    {
        int best = -1;
        for (int i = 0; i < static_cast<int>(c.size()); ++i) {
            if (i == skip) {continue;}
            if (best < 0 || c[static_cast<std::size_t>(i)].margin >
                c[static_cast<std::size_t>(best)].margin + slot_tie_epsilon_m_)
            {
                best = i;
            }
        }
        return best;
    }

    std::string sideOf(const Eigen::Vector3d & p) const
    {
        // X do frame da base do braco: +X = direita do robo (convencao do
        // tag_direita em moveToPlaceTarget).
        if (p.x() > 0.05) {return "a direita";}
        if (p.x() < -0.05) {return "a esquerda";}
        return "no centro";
    }

    /// Onde a ponta realmente parou, pela FK das juntas atuais (mesmo modelo
    /// da IK; nao depende do TF tcp da Pi). Devolve o erro 3D ou -1 sem estado.
    double tcpErrorByFk(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const Eigen::Vector3d & target,
        Eigen::Vector3d * actual = nullptr) const
    {
        // getCurrentState espera ate 2 s por um joint_states fresco (mesma
        // espera do moveToJointTarget); sem estado, devolve -1 e o chamador
        // decide (sempre com log) o que fazer.
        const auto state = arm->getCurrentState(2.0);
        if (!state) {
            return -1.0;
        }
        std::vector<double> jv;
        state->copyJointGroupPositions(arm->getName(), jv);
        if (jv.size() < 5) {
            return -1.0;
        }
        std::array<double, 5> q{};
        std::copy_n(jv.begin(), 5, q.begin());
        Eigen::Vector3d p;
        Eigen::Matrix3d rot;
        manip_task_execution::forwardKinematics(q, manip_task_execution::ArmModel{}, p, rot);
        if (actual) {*actual = p;}
        return (p - target).norm();
    }

    /// Vai a uma pose nomeada de slot e confere a chegada. Fora da tolerancia:
    /// reenvia o MESMO alvo uma vez; ainda fora = WARN e segue (nunca aborta
    /// com o bloco na garra). Devolve false so se plan/execute falhou.
    bool goToNamedSlot(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const TableSlotCandidate & slot,
        const std::map<std::string, TableObstacle> & obstacles,
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle)
    {
        publish_stage(goal_handle, "going_table_slot");
        if (!moveToJointTargetJoint2Last(arm, slot.q, "go slot " + slot.name)) {
            return false;
        }

        publish_stage(goal_handle, "verifying_slot_arrival");
        Eigen::Vector3d actual = slot.p;
        for (int attempt = 0; attempt < 2; ++attempt) {
            if (!sleepInterruptibly(std::chrono::milliseconds(300))) {
                return false;
            }
            const double err = tcpErrorByFk(arm, slot.p, &actual);
            if (err < 0.0) {
                if (attempt == 0) {
                    RCLCPP_WARN(get_logger(), "[SLOT] sem joint_states para conferir a chegada em %s — tentando de novo.", slot.name.c_str());
                    continue;
                }
                RCLCPP_ERROR(get_logger(), "[SLOT] sem joint_states em %s — soltando SEM conferir a chegada.", slot.name.c_str());
                speak("Atencao: nao consegui conferir a posicao da ponta antes de soltar");
                return true;
            }
            RCLCPP_INFO(
                get_logger(),
                "[SLOT] %s: ponta em [%.3f %.3f %.3f], alvo [%.3f %.3f %.3f], erro %.1f cm (max %.1f)",
                slot.name.c_str(), actual.x(), actual.y(), actual.z(),
                slot.p.x(), slot.p.y(), slot.p.z(), err * 100.0,
                slot_named_pose_error_m_ * 100.0);
            if (err <= slot_named_pose_error_m_) {
                return true;
            }
            if (attempt == 0) {
                RCLCPP_WARN(get_logger(), "[SLOT] fora da tolerancia — reenviando %s uma vez.", slot.name.c_str());
                publish_stage(goal_handle, "going_table_slot_retry");
                arm->setStartStateToCurrentState();
                arm->setNamedTarget(slot.name);
                if (!planAndExecute(arm, "re-go slot " + slot.name)) {
                    return false;
                }
            }
        }
        // Fora da tolerancia duas vezes: a margem que vale e a da posicao
        // REAL da ponta. Slot que era livre e virou ocupado = nao soltar aqui
        // (o chamador sobe a pegar_obj e tenta o proximo).
        const double real_margin = marginAt(actual, obstacles);
        if (slot.margin >= 0.0 && real_margin < 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "[SLOT] %s: ponta parou com margem real %+.1f cm (< 0) — nao solto aqui.",
                slot.name.c_str(), real_margin * 100.0);
            return false;
        }
        RCLCPP_WARN(
            get_logger(),
            "[SLOT] %s ainda fora da tolerancia apos reenvio (margem real %+.1f cm) — soltando assim mesmo.",
            slot.name.c_str(), real_margin * 100.0);
        speak("Atencao: a ponta parou um pouco fora do ponto de entrega");
        return true;
    }

    /// Ultimo recurso: XY livre dentro do envelope das poses ensinadas (bbox
    /// dos candidatos; Z da pose central, folga validada sobre o tampo).
    /// Aceita se melhorar a margem do melhor slot em >= 2 cm, qualquer sinal,
    /// e se a IK custom (frontal) for factivel — testando os melhores pontos
    /// em ordem, porque perto da borda do envelope pode nao haver solucao.
    bool planFreeXy(
        const std::vector<TableSlotCandidate> & candidates,
        const std::map<std::string, TableObstacle> & obstacles,
        double best_named_margin,
        std::array<double, 5> * q_out,
        Eigen::Vector3d * p_out,
        double * margin_out) const
    {
        // Envelope = o ARCO das poses ensinadas: anel radial [r_min, r_max]
        // (+3 cm de banda) e faixa azimutal entre os candidatos extremos. Um
        // bbox retangular incluiria pontos ate 11 cm mais perto/longe do robo
        // do que qualquer pose validada.
        double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
        double rmin = 1e9, rmax = -1e9, azmin = 1e9, azmax = -1e9;
        for (const auto & c : candidates) {
            xmin = std::min(xmin, c.p.x()); xmax = std::max(xmax, c.p.x());
            ymin = std::min(ymin, c.p.y()); ymax = std::max(ymax, c.p.y());
            const double r = std::hypot(c.p.x(), c.p.y());
            const double az = std::atan2(c.p.y(), c.p.x());
            rmin = std::min(rmin, r); rmax = std::max(rmax, r);
            azmin = std::min(azmin, az); azmax = std::max(azmax, az);
        }
        constexpr double kRadialBand = 0.03;
        const double z = candidates.front().p.z();   // centro (lista ordenada)
        const double step = std::max(0.005, slot_fallback_grid_step_m_);

        struct Pt {Eigen::Vector3d p; double margin;};
        std::vector<Pt> grid;
        for (double x = xmin; x <= xmax + 1e-9; x += step) {
            for (double y = ymin; y <= ymax + 1e-9; y += step) {
                const double r = std::hypot(x, y);
                const double az = std::atan2(y, x);
                if (r < rmin - kRadialBand || r > rmax + kRadialBand ||
                    az < azmin || az > azmax)
                {
                    continue;
                }
                const Eigen::Vector3d p(x, y, z);
                const double m = marginAt(p, obstacles);
                if (m >= best_named_margin + 0.02) {
                    grid.push_back({p, m});
                }
            }
        }
        if (grid.empty()) {
            return false;
        }
        std::sort(grid.begin(), grid.end(), [](const Pt & a, const Pt & b) {return a.margin > b.margin;});

        constexpr std::size_t kMaxIkTries = 10;
        for (std::size_t i = 0; i < grid.size() && i < kMaxIkTries; ++i) {
            std::array<double, 5> q{};
            if (!manip_task_execution::solveIk(
                    grid[i].p, manip_task_execution::ToolDirection::kDown, 0.0, q))
            {
                continue;
            }
            // Linha dos dedos como nas poses ensinadas (q5 do centro) mais
            // o offset de teste; trocar q5 nao move a ponta.
            q[4] = candidates.front().q[4] + slot_fallback_wrist_offset_rad_;
            *q_out = q;
            *p_out = grid[i].p;
            *margin_out = grid[i].margin;
            return true;
        }
        return false;
    }

    /// Caminho completo da mesa comum com escolha de slot. Devolve false
    /// apenas em falha de movimento com o bloco na garra (o chamador marca
    /// falha_com_bloco_na_garra).
    bool placeWithSlotSelection(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & table_pose,
        const std::string & ws,
        const std::string & carried_tag,
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle)
    {
        std::string enum_error;
        std::vector<TableSlotCandidate> candidates =
            enumerateTableSlots(arm, table_pose, &enum_error);
        if (candidates.empty()) {
            RCLCPP_WARN(
                get_logger(),
                "[SLOT] %s — caindo na pose unica legada '%s'.",
                enum_error.c_str(), table_pose.c_str());
            speak("Indo para a pose de entrega " + spokenTargetName(table_pose));
            std::array<double, 5> q{};
            if (namedPoseJoints(arm, table_pose, q)) {
                return moveToJointTargetJoint2Last(arm, q, "go " + table_pose);
            }
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(table_pose);
            return planAndExecute(arm, "go " + table_pose);
        }
        for (const auto & c : candidates) {
            RCLCPP_INFO(
                get_logger(),
                "[SLOT] candidato %-10s %s x=%+.3f y=%.3f z=%+.3f (%s)",
                c.name.c_str(), c.is_center ? "[centro]" : "        ",
                c.p.x(), c.p.y(), c.p.z(), sideOf(c.p).c_str());
        }

        // 1) Olhar a mesa (+ o que ja foi visto de garra vazia, + memoria).
        publish_stage(goal_handle, "observing_table");
        speak("Olhando a mesa antes de soltar");
        std::map<std::string, TableObstacle> obstacles = early_obstacles_;
        if (!observeTableObstacles(
                carried_tag, candidates.front().p, pegar_obj_arrival_time_, obstacles))
        {
            return false;  // cancelado
        }
        addMemoryObstacles(ws, obstacles);

        // 2) Pontuar.
        publish_stage(goal_handle, "selecting_slot");
        for (const auto & kv : obstacles) {
            const auto & ob = kv.second;
            RCLCPP_INFO(
                get_logger(),
                "[SLOT] obstaculo %-24s x=%+.3f y=%.3f z=%+.3f idade=%.1fs folga=%.2f (%s)",
                ob.frame.c_str(), ob.p.x(), ob.p.y(), ob.p.z(), ob.age_sec,
                ob.clearance, ob.origin.c_str());
        }
        for (auto & c : candidates) {
            c.margin = marginAt(c.p, obstacles);
        }
        const int best_i = bestCandidate(candidates);
        const int second_i = bestCandidate(candidates, best_i);
        const TableSlotCandidate & best = candidates[static_cast<std::size_t>(best_i)];
        for (const auto & c : candidates) {
            RCLCPP_INFO(
                get_logger(), "[SLOT] margem %-10s %+.1f cm%s",
                c.name.c_str(), c.margin * 100.0, (&c == &best) ? "  <-- escolhida" : "");
        }

        const std::size_t n_obs = obstacles.size();
        if (n_obs == 0) {
            speak("Mesa livre, entregando no centro");
        } else if (best.margin >= 0.0) {
            speak("Vi " + std::to_string(n_obs) + " objeto" + (n_obs > 1 ? "s" : "") +
                " na mesa, entregando " + sideOf(best.p));
        }

        // 3) Pose escolhida (e, se falhar, a 2a melhor — sempre subindo a
        //    pegar_obj antes). Cada tentativa passa pelo mesmo tratamento de
        //    "sem folga" (XY livre por flag / menos ruim com aviso / recusa
        //    opcional por sobreposicao fisica).
        if (tryPlaceAtSlot(arm, best, candidates, obstacles, table_pose, goal_handle)) {
            return true;
        }
        if (cancellationRequested() || last_place_failure_reason_ == "sem_espaco_na_mesa") {
            return false;
        }
        if (second_i < 0) {
            return false;
        }
        const TableSlotCandidate & second = candidates[static_cast<std::size_t>(second_i)];
        RCLCPP_WARN(
            get_logger(),
            "[SLOT] falhou em %s — subindo a pegar_obj e tentando %s (margem %+.1f cm).",
            best.name.c_str(), second.name.c_str(), second.margin * 100.0);
        publish_stage(goal_handle, "recovering_pegar_obj");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "recover pegar_obj before 2nd slot")) {
            return false;
        }
        return tryPlaceAtSlot(arm, second, candidates, obstacles, table_pose, goal_handle);
    }

    /// Leva o bloco ao slot `slot` (braco em pegar_obj). Se o slot nao tem
    /// folga: com a flag, tenta primeiro um XY livre que melhore a margem;
    /// senao (decisao do operador) vai ao "menos ruim" avisando — a menos que
    /// a margem indique sobreposicao fisica e slot_overlap_refuse_margin_m
    /// esteja ligado, caso em que recusa soltar (sem_espaco_na_mesa).
    /// Devolve false em falha de movimento (chamador decide o retry).
    bool tryPlaceAtSlot(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const TableSlotCandidate & slot,
        const std::vector<TableSlotCandidate> & candidates,
        const std::map<std::string, TableObstacle> & obstacles,
        const std::string & table_pose,
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle)
    {
        if (slot.margin < 0.0) {
            if (slot_fallback_free_xy_enabled_) {
                speak("Mesa cheia, calculando um ponto livre");
                std::array<double, 5> q{};
                Eigen::Vector3d p;
                double m = 0.0;
                if (planFreeXy(candidates, obstacles, slot.margin, &q, &p, &m)) {
                    RCLCPP_INFO(
                        get_logger(),
                        "[SLOT] XY livre [%+.3f %.3f %+.3f] margem %+.1f cm -> j [%.4f %.4f %.4f %.4f %.4f]",
                        p.x(), p.y(), p.z(), m * 100.0, q[0], q[1], q[2], q[3], q[4]);
                    publish_stage(goal_handle, "place_free_xy");
                    bool ok = moveToJointTargetJoint2Last(arm, q, "place free xy " + table_pose);
                    if (ok) {
                        if (!sleepInterruptibly(std::chrono::milliseconds(300))) {
                            return false;
                        }
                        const double err = tcpErrorByFk(arm, p);
                        if (err < 0.0) {
                            RCLCPP_WARN(get_logger(), "[SLOT] XY livre: sem joint_states para conferir a chegada — soltando sem verificacao.");
                            speak("Atencao: nao consegui conferir a posicao da ponta antes de soltar");
                        } else if (err > max_pose_error_m_) {
                            ok = false;
                            RCLCPP_WARN(get_logger(), "[SLOT] XY livre: ponta a %.1f cm do alvo (max %.1f).", err * 100.0, max_pose_error_m_ * 100.0);
                        }
                    }
                    if (ok) {
                        last_slot_decision_ = SlotDecision{"free_xy", table_pose, p, m};
                        return true;
                    }
                    // Recuperar SEMPRE por cima: subir a pegar_obj antes de
                    // qualquer outro alvo na mesa.
                    publish_stage(goal_handle, "place_free_xy_recover");
                    arm->setStartStateToCurrentState();
                    arm->setEndEffectorLink("tcp");
                    arm->setNamedTarget("pegar_obj");
                    if (!planAndExecute(arm, "recover pegar_obj after free xy")) {
                        return false;
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "[SLOT] nenhum XY livre melhora a margem em 2 cm com IK factivel.");
                }
            }
            if (slot.margin < slot_overlap_refuse_margin_m_) {
                // Margem tao negativa que os corpos se sobrepoem (cubo em
                // cima de cubo). Ligado por parametro; default desligado para
                // respeitar a decisao "coloca no menos ruim".
                RCLCPP_ERROR(
                    get_logger(),
                    "[SLOT] RECUSADO: %s com margem %+.1f cm (< %+.1f) = sobreposicao fisica — nao solto.",
                    slot.name.c_str(), slot.margin * 100.0, slot_overlap_refuse_margin_m_ * 100.0);
                speak("Sem espaco na mesa, nao vou soltar o bloco aqui");
                last_place_failure_reason_ = "sem_espaco_na_mesa";
                return false;
            }
            RCLCPP_WARN(
                get_logger(),
                "[SLOT] SEM FOLGA: %s com margem %+.1f cm — entregando no ponto menos ocupado (decisao do operador).",
                slot.name.c_str(), slot.margin * 100.0);
            speak("Atencao: sem folga na mesa, entregando no ponto menos ocupado");
        }

        if (!goToNamedSlot(arm, slot, obstacles, goal_handle)) {
            return false;
        }
        last_slot_decision_ = SlotDecision{slot.name, table_pose, slot.p, slot.margin};
        return true;
    }

    /// Grava na memoria onde o bloco foi solto (posicao FK real da ponta).
    void recordSlotUsed(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & ws,
        const std::string & carried_tag)
    {
        if (!last_slot_decision_ || !slot_memory_enabled_ || ws.empty()) {
            return;
        }
        Eigen::Vector3d actual = last_slot_decision_->p;
        (void)tcpErrorByFk(arm, last_slot_decision_->p, &actual);

        // Grava no frame da pose de docking: desconta o deslocamento lateral
        // acumulado da base (fila de alcance, 2026-08-28), para a leitura
        // (x + offset) bater mesmo depois de outro nudge ou de re-docking.
        const double offset = base_shift_total_.load(std::memory_order_relaxed);

        manip_task_execution::TableSlotRecord rec;
        rec.slot = last_slot_decision_->slot;
        rec.table_pose = last_slot_decision_->table_pose;
        rec.tag_frame = carried_tag;
        rec.frame = ik_reference_frame_;
        rec.x = actual.x() - offset;
        rec.y = actual.y();
        rec.z = actual.z();
        rec.stamp_sec = this->get_clock()->now().seconds();
        std::string err;
        if (!container_state_store_->addTableSlotUsed(ws, rec, &err)) {
            RCLCPP_WARN(get_logger(), "[SLOT] nao gravei a memoria de slot (%s) — entrega segue valida.", err.c_str());
            return;
        }
        RCLCPP_INFO(
            get_logger(),
            "[SLOT] memoria: %s usou %s em [%+.3f %.3f %+.3f] (%s; x real %+.3f, "
            "offset da base %+.3f)",
            ws.c_str(), rec.slot.c_str(), rec.x, rec.y, rec.z, carried_tag.c_str(),
            actual.x(), offset);
    }

    // =====================================================================
    // EMPILHAR (2026-08-24): soltar o objeto carregado em cima do objeto da
    // tag `base_frame`, ja apoiado na mesa da estacao.
    //
    // Convencao que fecha a conta (dados reais, ver memoria): o pick leva o
    // TCP ao plano da tag (face de cima do cubo) e o place solta com o TCP na
    // cota em que a tag de um cubo apoiado ficaria. Logo a cota da pilha e'
    // simplesmente "onde a tag do cubo de cima vai ficar":
    //   TCP = (x_base, y_base, z_base + stack_place_z_offset), ferramenta para
    //   baixo, dedos no yaw da tag base. Pre-pose stack_pre_lift_m acima,
    //   descida vertical (mesma inclinacao nas duas), subida apos soltar.
    // =====================================================================
    // kUnreachable (2026-08-28, fila de alcance): alvo VISTO e nenhum
    // candidato x inclinacao teve IK ANTES de qualquer movimento em direcao
    // ao alvo (os alvos ficam em last_unreachable_targets_). Falha de
    // CHEGADA (FK fora da tolerancia) continua kNoIk.
    enum class StackOutcome { kOk, kBaseNotSeen, kNoIk, kMoveFailed, kUnreachable };

    /// Escada de inclinacao da ferramenta em rad: 0 e depois cada valor
    /// (graus) de `ladder_deg` em (0, 90). E a MESMA lista que a busca de
    /// deslocamento da base (reach_shift) testa — o que resolve la resolve
    /// aqui (2026-08-28).
    static std::vector<double> tiltLadderRad(const std::vector<double> & ladder_deg)
    {
        std::vector<double> tilts{0.0};
        for (const double t : ladder_deg) {
            if (t > 0.0 && t < 90.0) {
                tilts.push_back(t * M_PI / 180.0);
            }
        }
        return tilts;
    }

    StackOutcome placeStackedOnFrame(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & base_frame,
        const std::string & table_pose,
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle,
        const std::vector<double> & place_joints = {})
    {
        // 2026-08-31 (pedido do operador): JUNTAS GRAVADAS na pega deste slot
        // — vai direto para elas (mesma configuracao em que a garra fechou),
        // sem IK do ponto e sem depender da camera. Pre-pose = mesmo ponto
        // stack_pre_lift_m acima (IK; sem solucao = descida direta). Chegada
        // por FK dispensada: juntas sao exatas.
        if (place_joints.size() >= 5) {
            std::array<double, 5> qj{
                place_joints[0], place_joints[1], place_joints[2], place_joints[3], place_joints[4]};
            Eigen::Vector3d fk_pos;
            Eigen::Matrix3d fk_rot;
            manip_task_execution::forwardKinematics(
                qj, manip_task_execution::ArmModel{}, fk_pos, fk_rot);
            const double tilt = std::acos(std::max(-1.0, std::min(1.0, -fk_rot.col(2).z())));
            std::array<double, 5> q_lift2{};
            bool lift_ok = manip_task_execution::solveIk(
                fk_pos + Eigen::Vector3d(0.0, 0.0, stack_pre_lift_m_), tilt, 0.0, q_lift2);
            if (lift_ok) {
                q_lift2[4] = qj[4];
            } else {
                RCLCPP_WARN(
                    get_logger(), "[STACK] juntas gravadas: sem IK para a pre-pose %.0f mm acima — descida direta.",
                    stack_pre_lift_m_ * 1000.0);
                q_lift2 = qj;
            }
            RCLCPP_INFO(
                get_logger(),
                "[STACK] place por JUNTAS gravadas em %s: alvo FK [%.3f %.3f %.3f] (%.0f graus) j "
                "[%.3f %.3f %.3f %.3f %.3f]",
                base_frame.c_str(), fk_pos.x(), fk_pos.y(), fk_pos.z(), tilt * 180.0 / M_PI,
                qj[0], qj[1], qj[2], qj[3], qj[4]);
            publish_stage(goal_handle, "stack_joints_pre_pose");
            speak("Colocando o bloco na posicao gravada");
            stack_arm_moved_ = true;
            if (!moveToJointTarget(arm, q_lift2, "stack joints pre-pose " + base_frame)) {
                return StackOutcome::kMoveFailed;
            }
            stack_lift_q_ = q_lift2;  // subida apos soltar
            publish_stage(goal_handle, "stack_joints_final");
            if (!moveToJointTarget(arm, qj, "stack joints replay " + base_frame)) {
                return StackOutcome::kMoveFailed;
            }
            last_slot_decision_ = SlotDecision{"stack:" + base_frame, table_pose, fk_pos, 0.0};
            return StackOutcome::kOk;
        }

        publish_stage(goal_handle, "stack_detecting_base");
        speak("Procurando o bloco de baixo " + spokenTargetName(base_frame));
        waitForCameraStream("PLACE");

        geometry_msgs::msg::TransformStamped base_tf;
        if (!waitForTagTransform(
                ik_reference_frame_, base_frame, base_tf,
                std::chrono::milliseconds(5000), std::chrono::milliseconds(200),
                "stack detect " + base_frame))
        {
            return StackOutcome::kBaseNotSeen;
        }
        RCLCPP_INFO(
            get_logger(), "[STACK] base %s <- %s: x=%.4f y=%.4f z=%.4f",
            ik_reference_frame_.c_str(), base_frame.c_str(),
            base_tf.transform.translation.x, base_tf.transform.translation.y,
            base_tf.transform.translation.z);

        // Mesmo pre-alinhamento lateral do place por TF (tag_direita/esquerda
        // sao poses altas: a camera no punho segue vendo a base por cima do
        // bloco carregado).
        constexpr double kTagXNearZero = 0.1;
        if (std::abs(base_tf.transform.translation.x) > kTagXNearZero) {
            publish_stage(goal_handle, "stack_pre_approach");
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(
                base_tf.transform.translation.x > 0.0 ? "tag_direita" : "tag_esquerda");
            stack_arm_moved_ = true;
            if (!planAndExecute(arm, "stack go tag_lateral")) {
                return StackOutcome::kMoveFailed;
            }
            if (!sleepInterruptibly(std::chrono::milliseconds(1000))) {
                return StackOutcome::kMoveFailed;
            }
            if (!waitForTagTransform(
                    ik_reference_frame_, base_frame, base_tf,
                    std::chrono::milliseconds(3000), std::chrono::milliseconds(200),
                    "stack final " + base_frame))
            {
                return StackOutcome::kBaseNotSeen;
            }
        }

        // Guarda: a base tem que estar A FRENTE do braco (na mesa), nao numa
        // parede/prateleira atras ou sob o robo.
        if (base_tf.transform.translation.y < 0.10) {
            RCLCPP_WARN(
                get_logger(),
                "[STACK] base %s em y=%.3f (atras/sob o braco) — nao empilho.",
                base_frame.c_str(), base_tf.transform.translation.y);
            return StackOutcome::kBaseNotSeen;
        }

        const Eigen::Vector3d target(
            base_tf.transform.translation.x,
            base_tf.transform.translation.y,
            base_tf.transform.translation.z + stack_place_z_offset_);
        const Eigen::Vector3d target_lift = target + Eigen::Vector3d(0.0, 0.0, stack_pre_lift_m_);
        const double base_yaw = manip_task_execution::projectedFrameYaw(
            Eigen::Quaterniond(
                base_tf.transform.rotation.w, base_tf.transform.rotation.x,
                base_tf.transform.rotation.y, base_tf.transform.rotation.z)
            .toRotationMatrix());

        // Escada de inclinacao (0, depois stack_tilt_ladder_deg): pre-pose e
        // pegada precisam resolver na MESMA inclinacao para a descida ser
        // vertical.
        std::array<double, 5> q_final{};
        std::array<double, 5> q_lift{};
        bool solved = false;
        double tilt_used = 0.0;
        for (const double tilt : tiltLadderRad(stack_tilt_ladder_deg_)) {
            if (manip_task_execution::solveIk(target, tilt, 0.0, q_final) &&
                manip_task_execution::solveIk(target_lift, tilt, 0.0, q_lift))
            {
                solved = true;
                tilt_used = tilt;
                break;
            }
        }
        if (!solved) {
            // Base VISTA, sem IK em nenhuma inclinacao, braco ainda longe da
            // pilha: fora de alcance (fila de alcance, 2026-08-28). O
            // chamador decide entre sair cedo e o fallback da mesa.
            RCLCPP_ERROR(
                get_logger(),
                "[STACK] IK propria nao achou solucao (com pre-pose) para "
                "[%.3f %.3f %.3f] em %s — fora de alcance.",
                target.x(), target.y(), target.z(), ik_reference_frame_.c_str());
            last_unreachable_targets_ = {target};
            return StackOutcome::kUnreachable;
        }
        q_final[4] = manip_task_execution::computeWristForTagYaw(base_yaw, q_final[0], tilt_used);
        q_lift[4] = manip_task_execution::computeWristForTagYaw(base_yaw, q_lift[0], tilt_used);
        RCLCPP_INFO(
            get_logger(),
            "[STACK] alvo [%.3f %.3f %.3f] (base z=%.3f + %.3f), ferramenta a %.0f graus, "
            "yaw base=%.3f -> pre-pose j [%.3f %.3f %.3f %.3f %.3f], final j "
            "[%.3f %.3f %.3f %.3f %.3f]",
            target.x(), target.y(), target.z(), base_tf.transform.translation.z,
            stack_place_z_offset_, tilt_used * 180.0 / M_PI, base_yaw,
            q_lift[0], q_lift[1], q_lift[2], q_lift[3], q_lift[4],
            q_final[0], q_final[1], q_final[2], q_final[3], q_final[4]);

        publish_stage(goal_handle, "stack_pre_pose");
        speak("Levando o bloco para cima da pilha");
        // Todas as juntas juntas (pedido do operador 2026-08-24) — sem o
        // "j2 por ultimo" dos slots.
        if (!moveToJointTarget(arm, q_lift, "stack pre-pose above " + base_frame)) {
            return StackOutcome::kMoveFailed;
        }
        stack_lift_q_ = q_lift;  // subida apos soltar

        publish_stage(goal_handle, "stack_final_approach");
        if (!moveToJointTarget(arm, q_final, "stack descend onto " + base_frame)) {
            return StackOutcome::kMoveFailed;
        }

        // Conferencia de chegada pela FK (como nos slots): longe demais do
        // alvo = soltar derrubaria a pilha. Sobe de volta e deixa o chamador
        // decidir (fallback para a mesa).
        Eigen::Vector3d actual = target;
        const double err = tcpErrorByFk(arm, target, &actual);
        // err < 0 = sem joint_states: nao da para conferir — em cima de uma
        // pilha, nao soltar as cegas (recua e cai no fallback da mesa).
        if (err < 0.0 || err > stack_arrival_tolerance_m_) {
            RCLCPP_WARN(
                get_logger(),
                "[STACK] chegada %s (tolerancia %.1f cm) — nao solto em cima da pilha.",
                err < 0.0 ? "sem joint_states para conferir"
                          : (std::to_string(err * 100.0).substr(0, 4) + " cm do alvo").c_str(),
                stack_arrival_tolerance_m_ * 100.0);
            (void)moveToJointTarget(arm, q_lift, "stack back up (arrival)");
            stack_lift_q_.reset();
            return StackOutcome::kNoIk;
        }

        last_slot_decision_ = SlotDecision{"stack:" + base_frame, table_pose, target, 0.0};
        RCLCPP_INFO(
            get_logger(), "[STACK] em posicao sobre %s (erro FK %.1f cm) — soltando.",
            base_frame.c_str(), err * 100.0);
        return StackOutcome::kOk;
    }

    // =====================================================================
    // CONTAINER POR COR (2026-08-25): soltar o objeto DENTRO do container
    // vermelho/azul que a camera ve na mesa de destino. NAO e' o pote a
    // bordo (container1..3). Fluxo espelhado em placeStackedOnFrame.
    // =====================================================================
    std::string containerFrameForColor(const std::string & color) const
    {
        if (color == "RED") {
            return container_frame_red_;
        }
        if (color == "BLUE") {
            return container_frame_blue_;
        }
        return "";
    }

    static std::string containerColorPt(const std::string & color)
    {
        if (color == "RED") {
            return "vermelho";
        }
        if (color == "BLUE") {
            return "azul";
        }
        return color;
    }

    /// Cota do TAMPO da mesa (manip_base_link) a partir da pose nomeada: o
    /// TCP de soltura fica na cota da face de cima de um cubo apoiado, logo
    /// tampo = FK.z - altura do cubo (mesma convencao do empilhamento).
    bool tabletopZForPose(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & table_pose,
        double * z_out) const
    {
        std::array<double, 5> q{};
        if (!namedPoseJoints(arm, table_pose, q)) {
            return false;
        }
        Eigen::Vector3d p;
        Eigen::Matrix3d rot;
        manip_task_execution::forwardKinematics(q, manip_task_execution::ArmModel{}, p, rot);
        *z_out = p.z() - cube_height_m_;
        return true;
    }

    /// Publica (latched) a cota da BORDA do container para o detector, antes
    /// de qualquer movimento do ciclo — o detector so precisa dela na hora da
    /// aproximacao, ~10 s depois.
    void publishContainerPlaneZ(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<const PlaceTag::Goal> & goal)
    {
        container_plane_z_.reset();
        // Publica para TODO goal em mesa comum (nao so' os com cor): o detector
        // tambem alimenta os obstaculos dos places normais e precisa do plano
        // da mesa ATUAL (o topico e' latched — sem isso ficaria com a mesa
        // anterior ou o default).
        if (isShelfPlaceTarget(goal->table_pose) || isTfPlaceTarget(goal->table_pose) ||
            normalizedWs(goal->ws).rfind("WS", 0) != 0)
        {
            if (!goal->container_color.empty()) {
                RCLCPP_WARN(
                    get_logger(),
                    "[CONTAINER] table_pose '%s' / ws '%s' nao e mesa comum — sem plano da "
                    "borda (container por cor so em WS_*).",
                    goal->table_pose.c_str(), goal->ws.c_str());
            }
            return;
        }
        double tabletop = 0.0;
        if (!tabletopZForPose(arm, goal->table_pose, &tabletop)) {
            RCLCPP_WARN(
                get_logger(),
                "[CONTAINER] sem juntas da pose '%s' — sem plano da borda; o detector "
                "usa o default dele.",
                goal->table_pose.c_str());
            return;
        }
        const double plane_z = tabletop + container_rim_height_m_;
        std_msgs::msg::Float64 msg;
        msg.data = plane_z;
        container_plane_z_pub_->publish(msg);
        container_plane_z_ = plane_z;
        RCLCPP_INFO(
            get_logger(),
            "[CONTAINER] plano da borda em %s: z=%.3f (tampo %.3f + borda %.3f) — "
            "publicado em %s.",
            ik_reference_frame_.c_str(), plane_z, tabletop, container_rim_height_m_,
            container_plane_topic_.c_str());
    }

    StackOutcome placeIntoContainer(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & color,
        const std::string & table_pose,
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle)
    {
        const std::string frame = containerFrameForColor(color);
        const std::string cor = containerColorPt(color);
        if (frame.empty()) {
            RCLCPP_WARN(
                get_logger(), "[CONTAINER] cor '%s' sem frame configurado — place normal.",
                color.c_str());
            return StackOutcome::kBaseNotSeen;
        }

        // Detector nao esta no ar (ninguem assina o plano da borda): nao
        // faz sentido esperar 3 s pelo TF — place normal direto.
        if (container_plane_z_pub_->get_subscription_count() == 0) {
            RCLCPP_WARN(
                get_logger(),
                "[CONTAINER] container_detector nao esta no ar (sem assinante em %s) — "
                "place normal.",
                container_plane_topic_.c_str());
            return StackOutcome::kBaseNotSeen;
        }

        publish_stage(goal_handle, "container_detecting");
        speak("Procurando o container " + cor);
        waitForCameraStream("PLACE");

        geometry_msgs::msg::TransformStamped tf;
        const auto detect_timeout = std::chrono::milliseconds(
            static_cast<int>(std::max(0.2, container_detect_timeout_sec_) * 1000.0));
        bool seen = waitForTagTransform(
            ik_reference_frame_, frame, tf, detect_timeout,
            std::chrono::milliseconds(200), "container detect " + frame);
        if (!seen && container_search_lateral_) {
            // Varredura lateral: o container pode estar fora do quadro de
            // pegar_obj (ou cortado na borda, que o detector rejeita).
            const auto search_timeout = std::chrono::milliseconds(
                static_cast<int>(std::max(0.5, container_search_timeout_sec_) * 1000.0));
            for (const char * pose : {"tag_direita", "tag_esquerda"}) {
                publish_stage(goal_handle, "container_search");
                RCLCPP_INFO(
                    get_logger(), "[CONTAINER] %s nao visto de pegar_obj — olhando de %s.",
                    frame.c_str(), pose);
                arm->setStartStateToCurrentState();
                arm->setEndEffectorLink("tcp");
                arm->setNamedTarget(pose);
                container_arm_moved_ = true;
                if (!planAndExecute(arm, std::string("container search ") + pose)) {
                    return StackOutcome::kMoveFailed;
                }
                if (!sleepInterruptibly(std::chrono::milliseconds(800))) {
                    return StackOutcome::kMoveFailed;
                }
                if (waitForTagTransform(
                        ik_reference_frame_, frame, tf, search_timeout,
                        std::chrono::milliseconds(200), std::string("container search ") + pose))
                {
                    seen = true;
                    break;
                }
            }
        }
        if (!seen) {
            return StackOutcome::kBaseNotSeen;
        }
        RCLCPP_INFO(
            get_logger(), "[CONTAINER] %s <- %s: x=%.4f y=%.4f z=%.4f",
            ik_reference_frame_.c_str(), frame.c_str(),
            tf.transform.translation.x, tf.transform.translation.y,
            tf.transform.translation.z);

        // Pre-alinhamento lateral (mesmo do empilhamento / place por TF).
        constexpr double kTagXNearZero = 0.1;
        if (std::abs(tf.transform.translation.x) > kTagXNearZero) {
            publish_stage(goal_handle, "container_pre_approach");
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(
                tf.transform.translation.x > 0.0 ? "tag_direita" : "tag_esquerda");
            container_arm_moved_ = true;
            if (!planAndExecute(arm, "container go tag_lateral")) {
                return StackOutcome::kMoveFailed;
            }
            if (!sleepInterruptibly(std::chrono::milliseconds(1000))) {
                return StackOutcome::kMoveFailed;
            }
            if (!waitForTagTransform(
                    ik_reference_frame_, frame, tf,
                    std::chrono::milliseconds(3000), std::chrono::milliseconds(200),
                    "container final " + frame))
            {
                return StackOutcome::kBaseNotSeen;
            }
        }

        if (tf.transform.translation.y < 0.10) {
            RCLCPP_WARN(
                get_logger(),
                "[CONTAINER] %s em y=%.3f (atras/sob o braco) — nao uso.",
                frame.c_str(), tf.transform.translation.y);
            return StackOutcome::kBaseNotSeen;
        }

        // Cota: o plano da borda vem da FK da mesa (autoridade); o z do TF e'
        // so conferencia (o detector ray-casta no plano que publicamos).
        double plane_z = tf.transform.translation.z;
        if (container_plane_z_) {
            if (std::abs(tf.transform.translation.z - *container_plane_z_) > 0.05) {
                RCLCPP_WARN(
                    get_logger(),
                    "[CONTAINER] z do frame (%.3f) difere do plano da borda (%.3f) em "
                    "mais de 5 cm — usando o plano da FK.",
                    tf.transform.translation.z, *container_plane_z_);
            }
            plane_z = *container_plane_z_;
        }
        Eigen::Vector3d target(
            tf.transform.translation.x,
            tf.transform.translation.y,
            plane_z + cube_height_m_ + container_drop_clearance_m_);
        const double frame_yaw = manip_task_execution::projectedFrameYaw(
            Eigen::Quaterniond(
                tf.transform.rotation.w, tf.transform.rotation.x,
                tf.transform.rotation.y, tf.transform.rotation.z)
            .toRotationMatrix());
        // Varios blocos no MESMO container (pedido do operador 25/08): soltar
        // sempre no centro empilha o 2o em cima do 1o (e a garra o re-pega ao
        // fechar). Cada bloco vai para o candidato mais LONGE dos que a
        // memoria de slots diz que ja soltamos neste container.
        const std::vector<Eigen::Vector2d> slot_cands =
            containerSlotCandidates(goal_handle, frame, frame_yaw, target);

        const std::vector<double> tilts = tiltLadderRad(container_tilt_ladder_deg_);
        // Pose de recuo VERTICAL (stack_pre_lift_m acima) resolvida junto, na
        // mesma inclinacao: se a chegada reprovar, sobe por ela antes de
        // qualquer plano em juntas para pegar_obj (revisao 25/08).
        Eigen::Vector3d target_lift = target + Eigen::Vector3d(0.0, 0.0, stack_pre_lift_m_);
        std::array<double, 5> q{};
        std::array<double, 5> q_lift{};
        bool solved = false;
        double tilt_used = 0.0;
        for (std::size_t ci = 0; ci < slot_cands.size() && !solved; ++ci) {
            target.x() = slot_cands[ci].x();
            target.y() = slot_cands[ci].y();
            target_lift = target + Eigen::Vector3d(0.0, 0.0, stack_pre_lift_m_);
            for (const double tilt : tilts) {
                if (manip_task_execution::solveIk(target, tilt, 0.0, q) &&
                    manip_task_execution::solveIk(target_lift, tilt, 0.0, q_lift))
                {
                    solved = true;
                    tilt_used = tilt;
                    break;
                }
            }
            if (!solved) {
                RCLCPP_WARN(
                    get_logger(),
                    "[CONTAINER] candidato %zu/%zu [%.3f %.3f] sem IK — tentando o proximo.",
                    ci + 1, slot_cands.size(), target.x(), target.y());
            } else if (ci > 0) {
                RCLCPP_WARN(
                    get_logger(),
                    "[CONTAINER] usando o candidato %zu/%zu [%.3f %.3f] (preferido sem IK).",
                    ci + 1, slot_cands.size(), target.x(), target.y());
            }
        }
        if (!solved) {
            // Container VISTO, sem IK em nenhum candidato x inclinacao, braco
            // ainda longe dele: fora de alcance (fila de alcance, 2026-08-28).
            // Guarda os candidatos como alvos 3D (ordem de preferencia
            // preservada) para a busca de deslocamento da base; o chamador
            // decide entre sair cedo e o fallback da mesa.
            RCLCPP_ERROR(
                get_logger(),
                "[CONTAINER] IK propria nao achou solucao (com recuo) em nenhum candidato perto de [%.3f %.3f %.3f] em %s — fora de alcance.",
                target.x(), target.y(), target.z(), ik_reference_frame_.c_str());
            last_unreachable_targets_.clear();
            for (const auto & cand : slot_cands) {
                last_unreachable_targets_.emplace_back(cand.x(), cand.y(), target.z());
            }
            return StackOutcome::kUnreachable;
        }
        // Linha dos dedos ao longo do lado LONGO do container (X do frame).
        q[4] = manip_task_execution::computeWristForTagYaw(frame_yaw, q[0], tilt_used);
        q_lift[4] = q[4];
        RCLCPP_INFO(
            get_logger(),
            "[CONTAINER] alvo [%.3f %.3f %.3f] (borda %.3f + cubo %.3f + folga %.3f), "
            "ferramenta a %.0f graus, yaw %.3f -> j [%.3f %.3f %.3f %.3f %.3f]",
            target.x(), target.y(), target.z(), plane_z, cube_height_m_,
            container_drop_clearance_m_, tilt_used * 180.0 / M_PI, frame_yaw,
            q[0], q[1], q[2], q[3], q[4]);

        publish_stage(goal_handle, "container_approach");
        speak("Levando o bloco para dentro do container " + cor);
        container_arm_moved_ = true;
        // Pedido do operador (25/08): so' descer depois que as outras juntas
        // estao certas. 1) base e punho (j1, j4, j5) com o braco na altura
        // atual; 2) j2+j3 ate' a pose de ELEVACAO (mesmo x,y, +stack_pre_lift_m
        // acima do alvo); 3) descida vertical curta ate' o alvo.
        if (!moveToJointTargetJoints23Last(arm, q_lift, "container above " + frame)) {
            return StackOutcome::kMoveFailed;
        }
        if (!moveToJointTarget(arm, q, "container descend " + frame)) {
            return StackOutcome::kMoveFailed;
        }

        Eigen::Vector3d actual = target;
        const double err = tcpErrorByFk(arm, target, &actual);
        // XY decide se o bloco cai dentro; Z so' muda a altura da queda — e o
        // braco estendido afunda ~2,5 cm sistematicamente (medido 25/08), por
        // isso tolerancia propria.
        const double err_xy = (err < 0.0) ? -1.0 : std::hypot(actual.x() - target.x(), actual.y() - target.y());
        const double err_z = (err < 0.0) ? 0.0 : std::abs(actual.z() - target.z());
        if (err < 0.0 || err_xy > container_arrival_tolerance_m_ || err_z > container_arrival_z_tolerance_m_) {
            RCLCPP_WARN(
                get_logger(),
                "[CONTAINER] chegada %s (tolerancia xy %.1f cm, z %.1f cm) — nao solto no container; "
                "subindo antes do fallback.",
                err < 0.0 ? "sem joint_states para conferir"
                          : (std::to_string(err_xy * 100.0).substr(0, 4) + " cm em xy, " +
                             std::to_string(err_z * 100.0).substr(0, 4) + " cm em z").c_str(),
                container_arrival_tolerance_m_ * 100.0, container_arrival_z_tolerance_m_ * 100.0);
            (void)moveToJointTarget(arm, q_lift, "container back up (arrival)");
            return StackOutcome::kNoIk;
        }

        last_slot_decision_ = SlotDecision{frame, table_pose, target, 0.0};
        stack_lift_q_ = q_lift;            // saida: sobe na vertical antes de recolher
        container_exit_pending_ = true;    // ... com a garra FECHADA (pedido do operador 25/08)
        RCLCPP_INFO(
            get_logger(),
            "[CONTAINER] em posicao sobre o container %s (erro FK %.1f cm: dx %+.1f dy %+.1f dz %+.1f cm) — soltando.",
            cor.c_str(), err * 100.0, (actual.x() - target.x()) * 100.0,
            (actual.y() - target.y()) * 100.0, (actual.z() - target.z()) * 100.0);
        return StackOutcome::kOk;
    }

    /// Fila de alcance (2026-08-28): o container/pilha foi VISTO e nenhum
    /// candidato x inclinacao teve IK antes de qualquer movimento
    /// (StackOutcome::kUnreachable). Com o bailout ligado e o goal ainda nao
    /// sendo a tentativa final da estacao, o place sai cedo: pergunta a
    /// reach_shift quantos cm de lado a base precisa andar (mesma IK, mesma
    /// escada de inclinacao e mesma elevacao stack_pre_lift_m que a
    /// aproximacao usa) e marca a falha como "alvo_fora_de_alcance" — SEM
    /// fallback de mesa e SEM soltar; o chamador (run_place_cycle) devolve o
    /// cubo ao container de bordo e execute() responde skipped/unreachable.
    /// Devolve false quando o caminho de sempre (fallback na mesa) deve
    /// seguir: tentativa final, bailout desligado ou contradicao da IK.
    bool tryUnreachableBailout(
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle,
        const std::vector<double> & tilt_ladder_deg,
        const std::string & log_tag,
        const std::string & spoken_prefix)
    {
        const bool final_attempt = goal_handle->get_goal()->final_attempt;
        if (!unreachable_bailout_enabled_ || final_attempt) {
            RCLCPP_WARN(
                get_logger(),
                "[%s] alvo VISTO fora de alcance; %s — seguindo o caminho de sempre "
                "(fallback na mesa).",
                log_tag.c_str(),
                final_attempt ? "tentativa FINAL da estacao" : "unreachable_bailout_enabled=false");
            return false;
        }
        if (last_unreachable_targets_.empty()) {
            // Nao deveria acontecer (kUnreachable sempre guarda os alvos).
            RCLCPP_WARN(
                get_logger(),
                "[%s] fora de alcance sem alvos guardados — seguindo o caminho de sempre.",
                log_tag.c_str());
            return false;
        }

        publish_stage(goal_handle, "unreachable_check");
        manip_task_execution::ReachShiftOptions opts;
        opts.candidates_m = unreachable_shift_candidates_m_;
        opts.tilts_rad = tiltLadderRad(tilt_ladder_deg);
        opts.q5_fixed = 0.0;
        opts.lift_m = stack_pre_lift_m_;
        opts.undershoot_margin_m = unreachable_shift_margin_m_;
        const manip_task_execution::ReachShiftResult shift =
            manip_task_execution::findBaseShiftForReach(last_unreachable_targets_, opts);
        const Eigen::Vector3d & first = last_unreachable_targets_.front();

        if (shift.reachable_now) {
            // Contradicao: a mesma IK, com a mesma escada e a mesma elevacao,
            // resolveu um dos alvos sem mover a base. Nao adia — segue o
            // fallback de sempre.
            RCLCPP_WARN(
                get_logger(),
                "[%s] reach_shift resolve o alvo [%.3f %.3f %.3f] SEM mover a base, "
                "mas a aproximacao nao achou IK. Seguindo o caminho de sempre.",
                log_tag.c_str(), first.x(), first.y(), first.z());
            return false;
        }

        last_failure_unreachable_ = true;
        last_suggested_shift_m_ = shift.shift_m;
        last_place_failure_reason_ = "alvo_fora_de_alcance";
        RCLCPP_WARN(
            get_logger(),
            "[%s] alvo VISTO em [%.3f %.3f %.3f] (%s, %zu candidato(s)) FORA DO ALCANCE. "
            "Sugestao de deslocamento da base: %+.2f m (%s; alvo deslocado "
            "[%.3f %.3f %.3f], inclinacao %.0f graus). Saindo cedo: sem fallback de "
            "mesa, devolvendo o bloco ao container de bordo (fila de alcance).",
            log_tag.c_str(), first.x(), first.y(), first.z(),
            ik_reference_frame_.c_str(), last_unreachable_targets_.size(),
            shift.shift_m,
            shift.shift_m == 0.0 ? "de lado nao resolve" :
            (shift.shift_m > 0.0 ? "esquerda" : "direita"),
            shift.shifted_target.x(), shift.shifted_target.y(), shift.shifted_target.z(),
            shift.tilt_rad * 180.0 / M_PI);
        speak(
            spoken_prefix + " esta fora do meu alcance" +
            (shift.shift_m != 0.0 ? ", vou precisar me mover" : ""));
        return true;
    }

    bool moveToPlaceTarget(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & table_pose,
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle)
    {
        stack_lift_q_.reset();
        container_exit_pending_ = false;
        stack_arm_moved_ = false;
        container_arm_moved_ = false;
        {
            const auto goal = goal_handle->get_goal();
            if (!goal->stack_on.empty()) {
                if (isShelfPlaceTarget(table_pose)) {
                    RCLCPP_WARN(
                        get_logger(),
                        "[STACK] empilhar na prateleira (%s) nao e suportado — place normal.",
                        table_pose.c_str());
                } else {
                    const StackOutcome out = placeStackedOnFrame(
                        arm, goal->stack_on, table_pose, goal_handle,
                        std::vector<double>(goal->place_joints.begin(), goal->place_joints.end()));
                    if (out == StackOutcome::kOk) {
                        return true;
                    }
                    // Fila de alcance (2026-08-28): base VISTA, sem IK antes
                    // de mover — sai cedo, sem fallback de mesa e sem soltar.
                    if (out == StackOutcome::kUnreachable &&
                        tryUnreachableBailout(
                            goal_handle, stack_tilt_ladder_deg_, "STACK",
                            "O bloco base esta fora do meu alcance"))
                    {
                        return false;
                    }
                    // AMT1 (2026-08-28): na precision placement (ws PP_*) o
                    // stack_on e um slot registrado (tag_amt1_slot_k) e o
                    // fallback na mesa (Mesa10) largaria o cubo em cima de
                    // outro. Base nao vista / chegada fora da tolerancia /
                    // fora de alcance sem bailout => NAO cai na mesa: sai
                    // como unreachable com sugestao 0 — o caminho existente
                    // (run_place_cycle) devolve o cubo ao container de
                    // bordo e execute() responde skipped+unreachable; o no
                    // AMT1 re-registra os slots e repete 1x. WS_* inalterado.
                    if ((out == StackOutcome::kBaseNotSeen || out == StackOutcome::kNoIk ||
                        out == StackOutcome::kUnreachable) &&
                        normalizedWs(goal->ws).rfind("PP", 0) == 0)
                    {
                        last_place_failure_reason_ =
                            out == StackOutcome::kBaseNotSeen ? "base_nao_vista" :
                            out == StackOutcome::kNoIk ? "chegada_fora_da_tolerancia" :
                            "alvo_fora_de_alcance";
                        last_failure_unreachable_ = true;
                        last_suggested_shift_m_ = 0.0;
                        // Chegada reprovada: o braco esta na pre-pose acima
                        // do slot — a devolucao passa por pegar_obj antes.
                        if (out == StackOutcome::kNoIk) {
                            stack_arm_moved_ = true;
                        }
                        RCLCPP_WARN(
                            get_logger(),
                            "[STACK] PP (%s): nao empilhei sobre %s (%s) e NAO vou soltar na mesa "
                            "— devolvendo o bloco ao container de bordo (unreachable, sugestao 0).",
                            goal->ws.c_str(), goal->stack_on.c_str(),
                            last_place_failure_reason_.c_str());
                        speak("Nao achei o lugar certo na mesa de precisao, vou guardar o bloco de volta");
                        return false;
                    }
                    if (out == StackOutcome::kMoveFailed || !stack_fallback_to_table_) {
                        last_place_failure_reason_ = "empilhar_falhou";
                        return false;
                    }
                    RCLCPP_WARN(
                        get_logger(),
                        "[STACK] nao consegui empilhar sobre %s (%s) — soltando na mesa "
                        "normalmente (%s).",
                        goal->stack_on.c_str(),
                        out == StackOutcome::kBaseNotSeen ? "base nao vista" : "sem IK/chegada",
                        table_pose.c_str());
                    speak("Nao consegui empilhar, vou soltar na mesa");
                    // Volta a pegar_obj antes do caminho normal (o braco pode
                    // estar em tag_direita/esquerda ou na pre-pose).
                    arm->setStartStateToCurrentState();
                    arm->setEndEffectorLink("tcp");
                    arm->setNamedTarget("pegar_obj");
                    if (!planAndExecute(arm, "stack fallback return pegar_obj")) {
                        last_place_failure_reason_ = "empilhar_falhou";
                        return false;
                    }
                }
            }
        }

        // Container por cor (2026-08-25): so em mesa comum WS_*, nunca junto
        // com empilhamento (o executor ja rejeita a combinacao).
        {
            const auto goal = goal_handle->get_goal();
            if (!goal->container_color.empty() && goal->stack_on.empty()) {
                const std::string cor = containerColorPt(goal->container_color);
                container_arm_moved_ = false;
                if (containerFrameForColor(goal->container_color).empty()) {
                    RCLCPP_WARN(
                        get_logger(),
                        "[CONTAINER] cor '%s' sem container fisico (RED|BLUE) — place normal.",
                        goal->container_color.c_str());
                } else if (isShelfPlaceTarget(table_pose) || isTfPlaceTarget(table_pose) ||
                    normalizedWs(goal->ws).rfind("WS", 0) != 0)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "[CONTAINER] container por cor so em mesa comum WS_* (mesa '%s', ws '%s') "
                        "— place normal.",
                        table_pose.c_str(), goal->ws.c_str());
                } else {
                    const StackOutcome out = placeIntoContainer(
                        arm, goal->container_color, table_pose, goal_handle);
                    if (out == StackOutcome::kOk) {
                        return true;
                    }
                    // Fila de alcance (2026-08-28): container VISTO, sem IK
                    // em nenhum candidato antes de mover — sai cedo, sem
                    // fallback de mesa e sem soltar (28/08: o azul a
                    // x=+0,20 m caia na mesa em silencio).
                    if (out == StackOutcome::kUnreachable &&
                        tryUnreachableBailout(
                            goal_handle, container_tilt_ladder_deg_, "CONTAINER",
                            "O container " + cor + " esta fora do meu alcance"))
                    {
                        return false;
                    }
                    if (out == StackOutcome::kMoveFailed || !container_fallback_to_table_) {
                        last_place_failure_reason_ = "container_falhou";
                        return false;
                    }
                    RCLCPP_WARN(
                        get_logger(),
                        "[CONTAINER] nao consegui soltar no container %s (%s) — soltando na "
                        "mesa normalmente (%s).",
                        cor.c_str(),
                        out == StackOutcome::kBaseNotSeen ? "nao visto" : "sem IK/chegada",
                        table_pose.c_str());
                    speak(
                        out == StackOutcome::kBaseNotSeen
                            ? "Nao vi o container " + cor + ", vou soltar na mesa"
                            : "Nao consegui chegar no container " + cor + ", vou soltar na mesa");
                    if (container_arm_moved_) {
                        arm->setStartStateToCurrentState();
                        arm->setEndEffectorLink("tcp");
                        arm->setNamedTarget("pegar_obj");
                        if (!planAndExecute(arm, "container fallback return pegar_obj")) {
                            last_place_failure_reason_ = "container_falhou";
                            return false;
                        }
                    }
                }
            }
        }

        // Place normal na mesa (poses nomeadas do SRDF / slots MesaX.N e
        // prateleira): NAO tem conceito de alcance — as poses sao fixas em
        // juntas e sempre resolvem. A fila de alcance (2026-08-28) so vale
        // para container por cor e pilha, cujos alvos vem da camera.
        if (!isTfPlaceTarget(table_pose)) {
            if (isShelfPlaceTarget(table_pose)) {
                publish_stage(goal_handle, "shelf_waypoint");
                speak("Passando pela pose intermediaria da prateleira");
                if (!goToShelfWaypoint(arm, "place shelf waypoint before " + table_pose)) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "[PLACE] nao cheguei ao waypoint %s — abortando antes "
                        "de %s para nao bater na prateleira.",
                        kShelfWaypointPose, table_pose.c_str());
                    return false;
                }
            }

            // Mesa comum de uma WS: escolha de slot por camera (2026-08-21).
            // Fora da guarda (shelf, PP, goal sem ws, slot explicito, flag
            // off) segue byte a byte o caminho legado abaixo.
            {
                const auto goal = goal_handle->get_goal();
                std::string why;
                if (slotSelectionApplies(table_pose, goal->ws, &why)) {
                    return placeWithSlotSelection(
                        arm, table_pose, normalizedWs(goal->ws), goal->tag_frame, goal_handle);
                }
                if (!isShelfPlaceTarget(table_pose)) {
                    RCLCPP_INFO(
                        get_logger(), "[SLOT] pose unica legada '%s' (%s).",
                        table_pose.c_str(), why.c_str());
                }
            }

            speak("Indo para a pose de entrega " + spokenTargetName(table_pose));
            if (!isShelfPlaceTarget(table_pose)) {
                // Mesa comum (pose unica): j2 por ultimo, como nos slots.
                std::array<double, 5> q{};
                if (namedPoseJoints(arm, table_pose, q)) {
                    return moveToJointTargetJoint2Last(arm, q, "go " + table_pose);
                }
            }
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(table_pose);
            return planAndExecute(arm, "go " + table_pose);
        }

        publish_stage(goal_handle, "detecting_place_tag");
        speak("Procurando o alvo de entrega " + spokenTargetName(table_pose));
        waitForCameraStream("PLACE");

        // 2026-08-17: TODO o fluxo TF do place roda no frame da base do
        // braco (ik_reference_frame_) — e o frame da IK custom, e o lookup
        // inicial ja era feito nele (o log antigo dizia base_footprint por
        // engano).
        geometry_msgs::msg::TransformStamped target_tf;
        if (!waitForTagTransform(
                ik_reference_frame_,
                table_pose,
                target_tf,
                std::chrono::milliseconds(5000),
                std::chrono::milliseconds(200),
                "place detect " + table_pose)) {
            speak("Nao encontrei o alvo de entrega " + spokenTargetName(table_pose));
            return false;
        }

        publish_stage(goal_handle, "place_pre_approach");
        speak("Alvo de entrega encontrado. Ajustando a aproximacao");

        constexpr double kTagXNearZero = 0.1;
        const double tag_x = target_tf.transform.translation.x;

        RCLCPP_INFO(
            get_logger(),
            "[PLACE] initial TF %s <- %s: x=%.4f y=%.4f z=%.4f",
            ik_reference_frame_.c_str(),
            table_pose.c_str(),
            target_tf.transform.translation.x,
            target_tf.transform.translation.y,
            target_tf.transform.translation.z);

        if (std::abs(tag_x) > kTagXNearZero) {
            //speak("Corrigindo a aproximacao lateral para a entrega");
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");

            if (tag_x > 0.0) {
                arm->setNamedTarget("tag_direita");
                if (!planAndExecute(arm, "place go tag_direita")) {
                    return false;
                }
            } else {
                arm->setNamedTarget("tag_esquerda");
                if (!planAndExecute(arm, "place go tag_esquerda")) {
                    return false;
                }
            }
        }

        if (!sleepInterruptibly(std::chrono::milliseconds(1000))) {
            return false;
        }

        if (!waitForTagTransform(
                ik_reference_frame_,
                table_pose,
                target_tf,
                std::chrono::milliseconds(3000),
                std::chrono::milliseconds(200),
                "place final " + table_pose)) {
            speak("Perdi o alvo de entrega antes da aproximacao final");
            return false;
        }

        RCLCPP_INFO(
            get_logger(),
            "[PLACE] final TF %s <- %s: x=%.4f y=%.4f z=%.4f",
            ik_reference_frame_.c_str(),
            table_pose.c_str(),
            target_tf.transform.translation.x,
            target_tf.transform.translation.y,
            target_tf.transform.translation.z);

        // O mount do braco e translacao + yaw puro (eixos Z paralelos ao
        // base_footprint), entao o offset em Z vale identico neste frame.
        target_tf.transform.translation.z += container_place_z_offset_;

        publish_stage(goal_handle, "place_final_approach");
        //speak("Fazendo a aproximacao final para entregar o bloco");
        return approachPlaceTargetCustomIk(
            arm, target_tf, "place above " + table_pose);
    }

    rclcpp_action::GoalResponse
    handle_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const PlaceTag::Goal> goal)
    {
        if (goal->tag_frame.empty() || goal->table_pose.empty()) {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting PLACE goal: tag_frame and table_pose are required");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!execution_lock_->tryAcquire()) {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting PLACE for '%s': manipulator is busy",
                goal->tag_frame.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        cancel_requested_.store(false);
        RCLCPP_INFO(
            get_logger(),
            "Received place goal tag=%s table=%s ws=%s stack_on=%s container_color=%s "
            "final_attempt=%s base_shift_total=%+.3f",
            goal->tag_frame.c_str(),
            goal->table_pose.c_str(),
            goal->ws.c_str(),
            goal->stack_on.empty() ? "-" : goal->stack_on.c_str(),
            goal->container_color.empty() ? "-" : goal->container_color.c_str(),
            goal->final_attempt ? "true" : "false",
            base_shift_total_.load(std::memory_order_relaxed));
        if (!goal->container_color.empty() &&
            goal->container_color != "RED" && goal->container_color != "BLUE")
        {
            RCLCPP_WARN(
                get_logger(),
                "container_color '%s' desconhecida (esperado RED|BLUE) — place na mesa.",
                goal->container_color.c_str());
        }
        if (!goal->container_color.empty() && !goal->stack_on.empty()) {
            RCLCPP_WARN(
                get_logger(),
                "container_color e stack_on juntos: empilhar tem precedencia, container ignorado.");
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(
        const std::shared_ptr<GoalHandlePlaceTag>)
    {
        RCLCPP_WARN(get_logger(), "PLACE cancellation requested");
        cancel_requested_.store(true);
        stopActiveMotion();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<GoalHandlePlaceTag> goal_handle)
    {
        std::thread{
            std::bind(
                &PlaceActionServer::execute,
                this,
                std::placeholders::_1),
            goal_handle
        }.detach();
    }

    void execute(
        const std::shared_ptr<GoalHandlePlaceTag> goal_handle)
    {
        ExecutionGuard execution_guard(*this);
        publishPlaceActive(true);
        const auto goal = goal_handle->get_goal();

        // Fila de alcance (2026-08-28): estado por goal, zerado ANTES de
        // qualquer saida (o cancel antes de comecar tambem preenche o result).
        last_failure_unreachable_ = false;
        last_suggested_shift_m_ = 0.0;
        last_unreachable_targets_.clear();
        stack_arm_moved_ = false;
        container_arm_moved_ = false;

        auto result =
            std::make_shared<PlaceTag::Result>();

        // Fila de alcance: os campos de alcance saem preenchidos em TODOS os
        // desfechos (0/false por default). unreachable so e verdadeiro sem
        // entrega — o executor adia o objeto quando o ve. target_x/y ficam
        // como diagnostico sempre que a aproximacao chegou a montar alvos.
        const auto fill_reach_fields = [this, &result](bool physical_success)
            {
                result->unreachable = !physical_success && last_failure_unreachable_;
                result->suggested_base_shift_m =
                    result->unreachable ?
                    static_cast<float>(last_suggested_shift_m_) : 0.0f;
                if (!last_unreachable_targets_.empty()) {
                    result->target_x = static_cast<float>(last_unreachable_targets_.front().x());
                    result->target_y = static_cast<float>(last_unreachable_targets_.front().y());
                } else {
                    result->target_x = 0.0f;
                    result->target_y = 0.0f;
                }
            };

        const auto finish_failure =
            [this, &goal_handle, &result, &execution_guard, &fill_reach_fields](
                const std::string & message)
            {
                result->success = false;
                result->fail_reason = last_place_failure_reason_;
                fill_reach_fields(false);
                execution_guard.release();
                if (cancellationRequested() || goal_handle->is_canceling()) {
                    result->message = "Place canceled: " + message;
                    goal_handle->canceled(result);
                } else {
                    result->message = message;
                    goal_handle->abort(result);
                }
            };
        const auto finish_skip =
            [this, &goal_handle, &result, &execution_guard, &fill_reach_fields](
                const std::string & message)
            {
                result->success = true;
                result->skipped = true;  // item 2.5: contrato explicito
                result->fail_reason = "tag_nao_esta_em_container";
                result->message = message;
                fill_reach_fields(true);
                publish_stage(goal_handle, "skipped_missing_container_tag");
                speak("Nao encontrei esse bloco no container. Vou seguir para a proxima tarefa");
                execution_guard.release();
                goal_handle->succeed(result);
            };

        if (cancellationRequested() || goal_handle->is_canceling()) {
            finish_failure("canceled before execution started");
            return;
        }

        last_place_failure_reason_.clear();
        reloadSlotParameters();
        pegar_obj_arrival_time_.reset();
        early_obstacles_.clear();
        last_slot_decision_.reset();
        container_plane_z_.reset();
        speak(
            "Iniciando a rotina de entregar a " +
            spokenTargetName(goal->tag_frame));

        // Auditoria 2026-08-07, item 1.2: resolver o container ANTES de
        // construir os MoveGroupInterfaces — o caminho de skip deixa de pagar
        // 1-2s de conexao e de segurar o flock do braco a toa.
        std::string container_pose;
        std::string lookup_error;
        if (!container_state_store_->findContainerByTag(
                goal->tag_frame,
                &container_pose,
                &lookup_error))
        {
            if (skip_missing_place_tag_) {
                finish_skip("Place skipped: tag '" + goal->tag_frame +
                    "' is not in any occupied container: " + lookup_error);
                return;
            }
            result->success = false;
            result->message = "Place failed: could not resolve container for tag '" +
                goal->tag_frame + "' from yaml: " + lookup_error;
            finish_failure(result->message);
            return;
        }
        speak("Objeto localizado no " + spokenTargetName(container_pose));

        auto arm = makeInterfaceWithTimeout("arm");
        auto gripper = makeInterfaceWithTimeout("gripper");
        if (!arm || !gripper) {
            result->success = false;
            result->message =
                "Place failed: move_group indisponivel (MoveGroupInterface nao "
                "conectou em 15s) — stack de manipulacao no ar?";
            finish_failure(result->message);
            return;
        }
        setActiveInterfaces(arm, gripper);
        publishContainerPlaneZ(arm, goal);

        arm->setPoseReferenceFrame("base_footprint");
        arm->setPlanningTime(15.0);
        arm->setNumPlanningAttempts(20);
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(1.0);
        gripper->setMaxVelocityScalingFactor(1.0);
        gripper->setMaxAccelerationScalingFactor(1.0);

        const bool success =
            run_place_cycle(
                arm,
                gripper,
                container_pose,
                goal->table_pose,
                goal_handle);

        // Fallback do contrato 2.5: nenhuma falha sai sem causa nomeada —
        // portas genericas de plan/execute viram "ik_ou_execucao_falhou".
        if (!success && last_place_failure_reason_.empty()) {
            last_place_failure_reason_ = "ik_ou_execucao_falhou";
        }
        // Fila de alcance (2026-08-28): saida cedo (alvo visto, sem IK, ainda
        // nao e a tentativa final, cubo ja devolvido ao container de bordo) —
        // vira skip com unreachable=true; o executor desloca a base e pede
        // este place de novo. Nunca chama setEmpty (o yaml continua occupied).
        const bool unreachable_bailout =
            !success && last_failure_unreachable_ &&
            unreachable_bailout_enabled_ && !goal->final_attempt;

        // Verificacao adversarial 2026-08-10: gravar o yaml ANTES de
        // responder um cancel tardio — entrega concluida com cancel na saida
        // deixava o container marcado 'occupied' com ele ja vazio.
        bool state_write_success = true;
        std::string state_write_error;
        if (success) {
            state_write_success =
                container_state_store_->setEmpty(container_pose, &state_write_error);
            if (!state_write_success) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to update container state file %s: %s",
                    container_state_file_.c_str(),
                    state_write_error.c_str());
            }
        }

        if (cancellationRequested() || goal_handle->is_canceling()) {
            if (!success) {
                finish_failure("canceled during place cycle");
                return;
            }
            result->success = true;
            result->message = "Place completed (cancel recebido apos a entrega)";
            fill_reach_fields(true);
            execution_guard.release();
            goal_handle->canceled(result);
            return;
        }

        // Item 2.9 (espelho do pick): sucesso FISICO manda — falha de I/O do
        // yaml vira WARN + flag na mensagem, nunca aborta entrega feita.
        std::string slot_suffix;
        if (success && last_slot_decision_) {
            char buf[96];
            std::snprintf(
                buf, sizeof(buf), " [slot=%s margem=%+.1fcm]",
                last_slot_decision_->slot.c_str(), last_slot_decision_->margin * 100.0);
            slot_suffix = buf;
        }
        result->success = success;
        fill_reach_fields(success);
        if (success && state_write_success) {
            result->message = "Place completed" + slot_suffix;
        } else if (success && !state_write_success) {
            result->message =
                "Place completed (AVISO: falha ao gravar container yaml: " +
                state_write_error + ")" + slot_suffix;
            RCLCPP_WARN(
                get_logger(),
                "Place fisicamente OK mas o yaml de containers nao gravou: %s",
                state_write_error.c_str());
        } else {
            result->fail_reason = last_place_failure_reason_;
            result->message = "Place failed (" + last_place_failure_reason_ + ")";
        }

        if (result->success)
        {
            publish_stage(goal_handle, "done");
            speak("Entrega concluida com sucesso");
            execution_guard.release();
            goal_handle->succeed(result);
        }
        else if (unreachable_bailout)
        {
            // Fila de alcance: nao e falha — o cubo esta de volta no
            // container de bordo e o executor desloca a base e volta.
            // success=true + skipped=true + unreachable=true (+ sugestao).
            publish_stage(goal_handle, "skipped_unreachable");
            result->success = true;
            result->skipped = true;
            result->fail_reason = last_place_failure_reason_;
            char shift_txt[32];
            std::snprintf(
                shift_txt, sizeof(shift_txt), "%+.2f", last_suggested_shift_m_);
            result->message =
                "Place skipped: alvo visto mas fora de alcance (bloco devolvido ao " +
                container_pose + "; sugestao base " + std::string(shift_txt) + " m)";
            execution_guard.release();
            goal_handle->succeed(result);
        }
        else
        {
            speak("Nao consegui entregar o bloco");
            finish_failure(result->message);
        }
    }


    void onDynamicJointState(
        const control_msgs::msg::DynamicJointState::SharedPtr message)
    {
        std::optional<double> motor6_effort;
        std::optional<double> motor7_effort;
        for (size_t i = 0; i < message->joint_names.size(); ++i) {
            if (i >= message->interface_values.size()) {
                break;
            }
            const auto & joint_name = message->joint_names[i];
            if (joint_name != "manip_joint6" && joint_name != "manip_joint7") {
                continue;
            }
            const auto & interface_value = message->interface_values[i];
            for (size_t j = 0; j < interface_value.interface_names.size(); ++j) {
                if (j >= interface_value.values.size()) {
                    break;
                }
                if (interface_value.interface_names[j] != "effort") {
                    continue;
                }
                if (joint_name == "manip_joint6") {
                    motor6_effort = interface_value.values[j];
                } else {
                    motor7_effort = interface_value.values[j];
                }
                break;
            }
        }
        if (!motor6_effort || !motor7_effort) {
            return;
        }
        std::lock_guard<std::mutex> lock(effort_mutex_);
        motor6_effort_ = *motor6_effort;
        motor7_effort_ = *motor7_effort;
        effort_update_time_ = std::chrono::steady_clock::now();
        ++effort_update_sequence_;
        effort_available_ = true;
    }

    struct GripperEffortSample
    {
        double motor6{0.0};
        double motor7{0.0};
        std::uint64_t sequence{0};
    };

    std::optional<GripperEffortSample> getFreshGripperEffort()
    {
        std::lock_guard<std::mutex> lock(effort_mutex_);
        if (!effort_available_) {
            return std::nullopt;
        }
        const auto age = std::chrono::steady_clock::now() - effort_update_time_;
        if (age > std::chrono::duration<double>(grasp_effort_max_age_)) {
            return std::nullopt;
        }
        return GripperEffortSample{
            std::abs(motor6_effort_),
            std::abs(motor7_effort_),
            effort_update_sequence_};
    }

    std::optional<GripperEffortSample> waitForFreshGripperEffort(
        const std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (cancellationRequested()) {
                return std::nullopt;
            }
            const auto sample = getFreshGripperEffort();
            if (sample) {
                return sample;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }
        return std::nullopt;
    }

    bool verifyGraspByEffort(
        const GripperEffortSample & baseline,
        const std::string & cycle_name)
    {
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::duration<double>(grasp_effort_sample_duration_);
        double motor6_sum = 0.0;
        double motor7_sum = 0.0;
        size_t sample_count = 0;
        std::uint64_t last_sequence = baseline.sequence;
        while (std::chrono::steady_clock::now() < deadline) {
            if (cancellationRequested()) {
                return false;
            }
            const auto sample = getFreshGripperEffort();
            if (sample && sample->sequence != last_sequence) {
                motor6_sum += sample->motor6;
                motor7_sum += sample->motor7;
                ++sample_count;
                last_sequence = sample->sequence;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }
        if (sample_count == 0) {
            RCLCPP_ERROR(
                get_logger(),
                "[%s] verificacao de garra falhou: sem amostras de esforco",
                cycle_name.c_str());
            return false;
        }
        const double m6_avg = motor6_sum / static_cast<double>(sample_count);
        const double m7_avg = motor7_sum / static_cast<double>(sample_count);
        const bool m6_loaded =
            m6_avg >= grasp_min_effort_nm_ &&
            (m6_avg - baseline.motor6) >= grasp_min_effort_increase_nm_;
        const bool m7_loaded =
            m7_avg >= grasp_min_effort_nm_ &&
            (m7_avg - baseline.motor7) >= grasp_min_effort_increase_nm_;
        RCLCPP_INFO(
            get_logger(),
            "[%s] place grasp effort: M6 base=%.3f avg=%.3f; M7 base=%.3f "
            "avg=%.3f; samples=%zu",
            cycle_name.c_str(), baseline.motor6, m6_avg,
            baseline.motor7, m7_avg, sample_count);
        return m6_loaded && m7_loaded;
    }

    /// Fila de alcance (2026-08-28): o alvo (container/pilha) esta fora do
    /// alcance e o cubo ainda esta na garra — devolve-o ao container de
    /// bordo de onde saiu, pelo caminho inverso da busca: pegar_obj (se o
    /// braco saiu de la) -> pre_container -> <container_pose> -> abre a
    /// garra -> pre_container -> pegar_obj. O yaml do container continua
    /// occupied (o setEmpty so roda com entrega concluida), entao o proximo
    /// PlaceTag deste bloco, depois do ajuste da base, volta a busca-lo la.
    /// false = o cubo pode ter ficado na garra ou o braco fora de pegar_obj:
    /// o chamador aborta como falha_com_bloco_na_garra (sem unreachable).
    bool returnCubeToOnboardContainer(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string & container_pose,
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle)
    {
        RCLCPP_WARN(
            get_logger(),
            "[PLACE] alvo fora de alcance — devolvendo o bloco ao %s (yaml continua occupied).",
            container_pose.c_str());
        speak("Vou devolver o bloco ao " + spokenTargetName(container_pose));
        arm->setMaxAccelerationScalingFactor(1.0);

        if (container_arm_moved_ || stack_arm_moved_) {
            // Braco em tag_direita/esquerda ou na pre-pose: volta ao caminho
            // validado (pegar_obj) antes de descer ao pote.
            publish_stage(goal_handle, "returning_cube_pegar_obj");
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget("pegar_obj");
            if (!planAndExecute(arm, "return cube: pegar_obj")) {
                return false;
            }
        }

        publish_stage(goal_handle, "returning_cube_pre_container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, "return cube: pre_container")) {
            return false;
        }

        publish_stage(goal_handle, "returning_cube_container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget(container_pose);
        if (!planAndExecute(arm, "return cube: " + container_pose)) {
            return false;
        }

        publish_stage(goal_handle, "returning_cube_opening_gripper");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, "return cube: open gripper")) {
            RCLCPP_ERROR(
                get_logger(),
                "[PLACE] a garra nao abriu dentro do %s — o bloco pode continuar na garra.",
                container_pose.c_str());
            return false;
        }
        // Conferencia so pelo valor ABSOLUTO do esforco (base zero, mesmo
        // padrao da saida do container da mesa): garra ainda carregada
        // depois de abrir = o bloco pode nao ter ficado no pote. Apenas
        // WARN — nada aqui reprova a devolucao.
        if (verify_grasp_effort_) {
            const auto sample = waitForFreshGripperEffort(std::chrono::milliseconds(600));
            if (sample) {
                GripperEffortSample zero = *sample;
                zero.motor6 = 0.0;
                zero.motor7 = 0.0;
                if (verifyGraspByEffort(zero, "RETURN-CUBE")) {
                    RCLCPP_WARN(
                        get_logger(),
                        "[PLACE] esforco da garra ainda alto depois de abrir no %s — o bloco "
                        "pode nao ter ficado no container de bordo.",
                        container_pose.c_str());
                }
            } else {
                RCLCPP_WARN(
                    get_logger(),
                    "[PLACE] sem telemetria de esforco para conferir a devolucao — seguindo.");
            }
        }

        publish_stage(goal_handle, "returning_cube_pre_container_out");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, "return cube: pre_container (saida)")) {
            return false;
        }

        publish_stage(goal_handle, "returning_cube_pegar_obj_final");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "return cube: pegar_obj (final)")) {
            return false;
        }
        RCLCPP_INFO(
            get_logger(),
            "[PLACE] bloco devolvido ao %s; braco em pegar_obj — pronto para o ajuste da base.",
            container_pose.c_str());
        return true;
    }

    bool run_place_cycle(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string& container_pose,
        const std::string& table_pose,
        const std::shared_ptr<GoalHandlePlaceTag>& goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        const std::string place_ws = normalizedWs(goal->ws);

        // Opcional (decisao do operador 2026-08-21): olhar a mesa de GARRA
        // VAZIA antes de ir ao container, para o caso de o bloco carregado
        // ocluir tags em pegar_obj. Custa um movimento extra; falha aqui
        // nao reprova nada.
        {
            std::string why;
            if (slot_observe_empty_gripper_first_ &&
                slotSelectionApplies(table_pose, goal->ws, &why))
            {
                publish_stage(goal_handle, "going_observe_pose");
                arm->setStartStateToCurrentState();
                arm->setEndEffectorLink("tcp");
                arm->setNamedTarget("pegar_obj");
                if (planAndExecute(arm, "go pegar_obj (observar mesa)")) {
                    std::string enum_error;
                    const auto candidates = enumerateTableSlots(arm, table_pose, &enum_error);
                    if (!candidates.empty()) {
                        const rclcpp::Time arrival = this->get_clock()->now();
                        if (!observeTableObstacles(
                                goal->tag_frame, candidates.front().p, arrival, early_obstacles_))
                        {
                            return false;  // cancelado
                        }
                        RCLCPP_INFO(
                            get_logger(), "[SLOT] observacao de garra vazia: %zu obstaculo(s).",
                            early_obstacles_.size());
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "[SLOT] nao cheguei a pegar_obj para observar — seguindo sem.");
                }
            }
        }

        publish_stage(goal_handle, "opening_gripper");
        //speak("Abrindo a garra para preparar a retirada do container");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, "open gripper")) {
            return false;
        }

        publish_stage(goal_handle, "going_pre_container");
        //speak("Indo para a pre pose do container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, "go pre_container")) {
            return false;
        }

        publish_stage(goal_handle, "going_container");
        //speak("Indo ate o " + spokenTargetName(container_pose));
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget(container_pose);
        if (!planAndExecute(arm, "go " + container_pose)) {
            return false;
        }

        std::optional<GripperEffortSample> effort_before_close;
        if (verify_grasp_effort_ && verify_container_grasp_) {
            effort_before_close = waitForFreshGripperEffort(
                std::chrono::milliseconds(600));
            if (!effort_before_close) {
                last_place_failure_reason_ = "sem_telemetria_garra";
                speak("Falha: sem telemetria de esforço da garra");
                return false;
            }
        }

        publish_stage(goal_handle, "closing_gripper");
        //speak("Fechando a garra no bloco dentro do container");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_close");
        if (!planAndExecute(gripper, "close gripper")) {
            last_place_failure_reason_ = "garra_nao_fechou";
            return false;
        }

        // Item 2.4: garra fechou no VAZIO dentro do container? Reabrir e
        // falhar com causa distinta — sem sucesso fantasma e sem setEmpty.
        if (verify_grasp_effort_ && verify_container_grasp_) {
            publish_stage(goal_handle, "verifying_grasp");
            speak("Verificando o bloco pela forca da garra");
            if (!verifyGraspByEffort(*effort_before_close, "PLACE")) {
                last_place_failure_reason_ = "garra_vazia_no_container";
                speak("Falha: a garra não encontrou o bloco no container");
                gripper->setStartStateToCurrentState();
                gripper->setNamedTarget("gripper_open");
                (void)planAndExecute(gripper, "reopen after empty container grasp");
                return false;
            }
            speak("A garra detectou o bloco");
        }

        publish_stage(goal_handle, "returning_pre_container");
        //speak("Retirando o bloco do container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, "return pre_container")) {
            last_place_failure_reason_ = "falha_com_bloco_na_garra";
            return false;
        }

        // Pedido do operador 2026-08-15: na PRATELEIRA o pegar_obj entre o
        // pre_container e o pirocao e desnecessario — vai direto
        // pre_container -> pirocao -> MesaSh. Mesa comum mantem o caminho
        // validado com pegar_obj.
        if (!isShelfPlaceTarget(table_pose)) {
            publish_stage(goal_handle, "going_pegar_obj");
            //speak("Indo para a pose de transporte");
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget("pegar_obj");
            if (!planAndExecute(arm, "go pegar_obj")) {
                last_place_failure_reason_ = "falha_com_bloco_na_garra";
                return false;
            }
            pegar_obj_arrival_time_ = this->get_clock()->now();
        }


        arm->setMaxAccelerationScalingFactor(1.0);
        
        publish_stage(goal_handle, "going_table");
        //speak("Levando o bloco para o destino");
        if (!moveToPlaceTarget(arm, table_pose, goal_handle)) {
            // Fila de alcance (2026-08-28): alvo visto fora de alcance — o
            // cubo volta ao container de bordo antes de o goal sair como
            // skipped/unreachable (execute()). Se a devolucao falhar, vira a
            // falha de sempre (abort com o bloco na garra), sem unreachable.
            if (last_failure_unreachable_) {
                if (!returnCubeToOnboardContainer(arm, gripper, container_pose, goal_handle)) {
                    last_failure_unreachable_ = false;
                    last_place_failure_reason_ = "falha_com_bloco_na_garra";
                }
                return false;
            }
            if (last_place_failure_reason_.empty()) {
                last_place_failure_reason_ = "falha_com_bloco_na_garra";
            }
            return false;
        }
        arm->setMaxAccelerationScalingFactor(1.0);
        

        publish_stage(goal_handle, "opening_gripper_final");
        //speak("Abrindo a garra para soltar o bloco no destino");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, "open gripper final")) {
            last_place_failure_reason_ = "garra_nao_abriu_no_destino";
            return false;
        }
        recordSlotUsed(arm, place_ws, goal->tag_frame);

        // Soltou DENTRO do container (pedido do operador 25/08): os dedos
        // abertos batiam na parede e empurravam o container ao recolher.
        // Fecha a garra, sobe na vertical (j2/j3 ate' a pose de elevacao) e
        // so' depois volta a pegar_obj — j2/j3 primeiro, o resto em seguida.
        // Melhor esforco: o bloco ja' foi entregue.
        if (container_exit_pending_) {
            container_exit_pending_ = false;
            if (container_close_gripper_on_exit_) {
                std::optional<GripperEffortSample> base;
                if (verify_grasp_effort_) {
                    base = waitForFreshGripperEffort(std::chrono::milliseconds(600));
                }
                publish_stage(goal_handle, "container_closing_gripper");
                gripper->setStartStateToCurrentState();
                const std::map<std::string, double> jt{
                    {"manip_joint6", container_exit_gripper_position_},
                    {"manip_joint7", container_exit_gripper_position_}};
                gripper->setJointValueTarget(jt);
                if (!planAndExecute(gripper, "close gripper after container release")) {
                    RCLCPP_WARN(get_logger(), "[CONTAINER] garra nao fechou apos soltar — saindo assim mesmo.");
                } else if (verify_grasp_effort_ && base) {
                    // Receio do operador (25/08): fechar pode pegar o bloco de
                    // volta. O esforco da garra denuncia: se carregou, reabre
                    // (o bloco cai de novo no container) e sai com a garra aberta.
                    publish_stage(goal_handle, "container_verifying_empty");
                    // So' o valor ABSOLUTO conta: a leitura de base logo apos
                    // abrir ainda pode trazer o esforco da pega anterior
                    // (1,09 Nm visto em 25/08) e mascarar uma re-pega.
                    GripperEffortSample zero = *base;
                    zero.motor6 = 0.0;
                    zero.motor7 = 0.0;
                    if (verifyGraspByEffort(zero, "CONTAINER-EXIT")) {
                        RCLCPP_WARN(
                            get_logger(),
                            "[CONTAINER] a garra pegou o bloco de volta ao fechar — reabrindo e "
                            "saindo com a garra aberta.");
                        speak("Peguei o bloco de volta, vou soltar de novo");
                        gripper->setStartStateToCurrentState();
                        gripper->setNamedTarget("gripper_open");
                        (void)planAndExecute(gripper, "reopen after regrasp in container");
                    }
                }
            }
            if (stack_lift_q_) {
                publish_stage(goal_handle, "container_lift_after_release");
                if (!moveToJointTarget(arm, *stack_lift_q_, "container lift after release")) {
                    RCLCPP_WARN(get_logger(), "[CONTAINER] subida vertical apos soltar falhou — recolhendo direto.");
                }
                stack_lift_q_.reset();
            }
            std::array<double, 5> q_rest{};
            if (namedPoseJoints(arm, "pegar_obj", q_rest)) {
                publish_stage(goal_handle, "returning_pegar_obj_final");
                if (!moveToJointTargetJoints23First(arm, q_rest, "return pegar_obj after container")) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Entrega no container concluida mas o retorno a pegar_obj falhou — "
                        "braco pode estar estendido sobre a mesa.");
                    speak("Entreguei o bloco, mas nao consegui recolher o braco");
                }
                return true;
            }
            // sem juntas de pegar_obj: cai no retorno generico abaixo
        }

        // Empilhou: sobe na vertical ate a pre-pose antes de recolher, para
        // nao arrastar a pilha com a garra aberta. Melhor esforco.
        if (stack_lift_q_) {
            publish_stage(goal_handle, "stack_lift_after_release");
            if (!moveToJointTarget(arm, *stack_lift_q_, "stack lift after release")) {
                RCLCPP_WARN(
                    get_logger(),
                    "[STACK] subida apos soltar falhou — recolhendo direto.");
            }
            stack_lift_q_.reset();
        }

        // Caminho inverso na prateleira (pedido do operador 2026-08-17):
        // sair pelo waypoint pirocao e PARAR nele — sem voltar a pegar_obj.
        // O proximo movimento da task (home/pre_container) parte do waypoint.
        // Melhor esforco — o bloco ja foi entregue, nada aqui reprova a
        // entrega.
        if (isShelfPlaceTarget(table_pose)) {
            publish_stage(goal_handle, "returning_shelf_waypoint_final");
            if (!goToShelfWaypoint(
                    arm, "place shelf waypoint after " + table_pose)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Entrega concluida mas o retorno ao waypoint %s falhou — "
                    "braco pode estar estendido sobre a prateleira.",
                    kShelfWaypointPose);
                speak("Entreguei o bloco, mas nao consegui recolher o braco");
            }
            return true;
        }

        publish_stage(goal_handle, "returning_pegar_obj_final");
        //speak("Voltando para a pose segura depois da entrega");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "return pegar_obj")) {
            // Verificacao adversarial 2026-08-10: a entrega JA aconteceu (a
            // garra abriu no destino) — reportar falha aqui corrompia o yaml
            // (container ficava 'occupied' vazio). Conta como sucesso e so
            // avisa que o braco pode ter ficado estendido.
            RCLCPP_ERROR(
                get_logger(),
                "Entrega concluida mas o retorno a pegar_obj falhou — braco "
                "pode estar estendido sobre a mesa.");
            speak("Entreguei o bloco, mas nao consegui recolher o braco");
        }

        return true;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PlaceActionServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
