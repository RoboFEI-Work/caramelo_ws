#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "manip_task_execution/container_state_store.hpp"
#include "manip_task_execution/custom_ik.hpp"
#include "manip_task_execution/manipulator_execution_lock.hpp"
#include "my_robot_msgs/action/pick_tag.hpp"

namespace mtc = moveit::task_constructor;

class PickActionServer : public rclcpp::Node
{
public:
    using PickTag = my_robot_msgs::action::PickTag;
    using GoalHandlePickTag = rclcpp_action::ServerGoalHandle<PickTag>;
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    PickActionServer()
    : Node("pick_action_server")
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

        // Idade maxima aceita da TF da tag: o robo e' MOVEL e a camera fica no
        // braco — uma tag vista ha segundos (buffer TF) vira alvo fantasma se a
        // base/braco se moveu. Exigir deteccao RECENTE ("tag vista AGORA").
        max_tag_age_sec_ = this->declare_parameter<double>("max_tag_age_sec", 1.0);

        const auto default_container_state_file = getDefaultContainerStatePath();
        container_state_file_ = this->declare_parameter<std::string>(
            "container_state_file",
            default_container_state_file);
        const bool reset_container_states_on_start =
            this->declare_parameter<bool>("reset_container_states_on_start", false);
        container_state_store_ =
            std::make_unique<manip_task_execution::ContainerStateStore>(container_state_file_);
        if (reset_container_states_on_start) {
            std::string reset_error;
            if (!container_state_store_->resetAllEmpty(
                    {"container1", "container2", "container3"},
                    &reset_error)) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to reset container state file %s: %s",
                    container_state_file_.c_str(),
                    reset_error.c_str());
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Reset container state file to empty: %s",
                    container_state_file_.c_str());
            }
        }

        // Ensure MTC PipelinePlanner can resolve OMPL params when this node is
        // started outside the generated MoveIt launch.
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
                    "default_planning_request_adapters/CheckStartStateCollision"
                });
        }
        if (!this->has_parameter("ompl.response_adapters")) {
            this->declare_parameter<std::vector<std::string>>(
                "ompl.response_adapters",
                std::vector<std::string>{
                    "default_planning_response_adapters/ValidateSolution",
                    "default_planning_response_adapters/DisplayMotionPath"
                });
        }
        if (!this->has_parameter("ompl.start_state_max_bounds_error")) {
            this->declare_parameter<double>(
                "ompl.start_state_max_bounds_error",
                0.1);
        }

        // Ensure MoveGroupInterface can instantiate IK for approximate targets
        // when this node is started standalone (without MoveIt launch params).
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
                0.002);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.orientation_threshold")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.orientation_threshold",
                0.30);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.solve_type")) {
            this->declare_parameter<std::string>(
                "robot_description_kinematics.arm.solve_type",
                "Speed");
        }
        if (!this->has_parameter("robot_description_kinematics.arm.position_only_ik")) {
            this->declare_parameter<bool>(
                "robot_description_kinematics.arm.position_only_ik",
                false);
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

        // Keep the top-level form as a compatibility fallback for existing
        // launch files that pass config/kinematics.yaml directly to this node.
        if (!this->has_parameter("arm.kinematics_solver")) {
            this->declare_parameter<std::string>(
                "arm.kinematics_solver",
                "pick_ik/PickIkPlugin");
        }
        if (!this->has_parameter("arm.kinematics_solver_timeout")) {
            this->declare_parameter<double>(
                "arm.kinematics_solver_timeout",
                0.2);
        }
        if (!this->has_parameter("arm.mode")) {
            this->declare_parameter<std::string>(
                "arm.mode",
                "global");
        }
        if (!this->has_parameter("arm.position_scale")) {
            this->declare_parameter<double>(
                "arm.position_scale",
                1.0);
        }
        if (!this->has_parameter("arm.rotation_scale")) {
            this->declare_parameter<double>(
                "arm.rotation_scale",
                0.03);
        }
        if (!this->has_parameter("arm.position_threshold")) {
            this->declare_parameter<double>(
                "arm.position_threshold",
                0.002);
        }
        if (!this->has_parameter("arm.orientation_threshold")) {
            this->declare_parameter<double>(
                "arm.orientation_threshold",
                0.30);
        }
        if (!this->has_parameter("arm.solve_type")) {
            this->declare_parameter<std::string>(
                "arm.solve_type",
                "Speed");
        }
        if (!this->has_parameter("arm.position_only_ik")) {
            this->declare_parameter<bool>(
                "arm.position_only_ik",
                false);
        }
        if (!this->has_parameter("arm.cost_threshold")) {
            this->declare_parameter<double>(
                "arm.cost_threshold",
                0.001);
        }
        if (!this->has_parameter("arm.minimal_displacement_weight")) {
            this->declare_parameter<double>(
                "arm.minimal_displacement_weight",
                0.02);
        }
        if (!this->has_parameter("arm.gd_step_size")) {
            this->declare_parameter<double>(
                "arm.gd_step_size",
                0.0008);
        }

        std::vector<std::string> planning_plugins;
        if (!this->get_parameter("ompl.planning_plugins", planning_plugins) ||
            planning_plugins.empty() || planning_plugins.front().empty())
        {
            this->set_parameter(
                rclcpp::Parameter(
                    "ompl.planning_plugins",
                    std::vector<std::string>{"ompl_interface/OMPLPlanner"}));
        }

        std::string planning_plugin;
        if (!this->get_parameter("ompl.planning_plugin", planning_plugin) || planning_plugin.empty()) {
            this->set_parameter(
                rclcpp::Parameter(
                    "ompl.planning_plugin",
                    "ompl_interface/OMPLPlanner"));
        }

        std::string kinematics_solver;
        if (!this->get_parameter(
                "robot_description_kinematics.arm.kinematics_solver",
                kinematics_solver) ||
            kinematics_solver.empty())
        {
            this->set_parameter(
                rclcpp::Parameter(
                    "robot_description_kinematics.arm.kinematics_solver",
                    "pick_ik/PickIkPlugin"));
        }

        if (!this->get_parameter("arm.kinematics_solver", kinematics_solver) || kinematics_solver.empty()) {
            this->set_parameter(
                rclcpp::Parameter(
                    "arm.kinematics_solver",
                    "pick_ik/PickIkPlugin"));
        }

        tf_buffer_ =
            std::make_shared<tf2_ros::Buffer>(
                this->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_);

        speech_enabled_ = this->declare_parameter<bool>("speech_enabled", true);
        speech_publisher_ =
            this->create_publisher<std_msgs::msg::String>("/manip/speech", 10);
        pick_active_publisher_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/manip/pick_active",
                rclcpp::QoS(1).transient_local().reliable());

        // Item 3.10 (gate da percepcao): com a camera desligada fora do
        // pick/place, o 1o frame demora ~2,6s a voltar e o estagio
        // detecting_tag so espera 900ms — sem esta sonda toda pega comecaria
        // gastando uma varredura a toa. Contador de camera_info (mensagem
        // minuscula, mesma taxa do color) = "a camera esta entregando frame".
        // Erro maximo aceito entre a pose pedida e onde a ponta realmente
        // parou (ver a verificacao em moveToTarget). 0 desliga a checagem.
        max_pose_error_m_ = this->declare_parameter<double>(
            "max_pose_error_m", 0.025);

        // Prateleira: quais table_pose disparam a sequencia de shelf. Fica em
        // parametro para nao espalhar string magica pelo codigo e para dar
        // conta de mais de uma altura de prateleira no futuro.
        shelf_table_poses_ = this->declare_parameter<std::vector<std::string>>(
            "shelf_table_poses", std::vector<std::string>{"MesaSh"});
        // Frame da base do braco — a IK propria trabalha nele, e entre ele e o
        // base_footprint ha translacao E um yaw de -1.57. Desde 2026-08-17 a
        // MESA COMUM tambem usa este frame (tentativas 1-2 do ladder custom);
        // o nome ficou por compatibilidade com launches salvos.
        shelf_ik_reference_frame_ = this->declare_parameter<std::string>(
            "shelf_ik_reference_frame", "manip_base_link");

        // Depuracao de campo do ladder da mesa: 0 = automatico (1 down,
        // 2 frente, 3+ MoveIt); 1/2/3 forca o mesmo degrau em TODAS as
        // tentativas (torna o teste do degrau 2/3 deterministico).
        force_table_ik_strategy_ = this->declare_parameter<int>(
            "force_table_ik_strategy", 0);
        // 2026-08-24 (pedido do operador: "sempre pegar por cima"): a pegada
        // top-down ESTRITA so alcanca ~0,365 m de raio (z ~0,04) e, alem
        // disso, o degrau 1 falhava e o pick caia no degrau 2 = pela FRENTE
        // (de lado). Antes de desistir do "por cima", o degrau 1 tenta a
        // ferramenta inclinada (graus a partir da vertical, radialmente para
        // fora) nesta escada: 0 e depois cada valor abaixo. Vazio = so 0.
        table_down_tilt_ladder_deg_ = this->declare_parameter<std::vector<double>>(
            "table_down_tilt_ladder_deg", std::vector<double>{15.0, 30.0});

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

        verify_grasp_effort_ =
            this->declare_parameter<bool>("verify_grasp_effort", true);
        grasp_min_effort_nm_ =
            this->declare_parameter<double>("grasp_min_effort_nm", 0.15);
        grasp_min_effort_increase_nm_ =
            this->declare_parameter<double>(
                "grasp_min_effort_increase_nm",
                0.05);
        grasp_effort_sample_duration_ =
            this->declare_parameter<double>(
                "grasp_effort_sample_duration",
                0.8);
        grasp_effort_max_age_ =
            this->declare_parameter<double>("grasp_effort_max_age", 0.4);
        grasp_retry_attempts_ =
            this->declare_parameter<int>("grasp_retry_attempts", 2);
        switch_ik_after_failed_grasp_ =
            this->declare_parameter<bool>("switch_ik_after_failed_grasp", true);
        primary_ik_profile_.solver =
            this->declare_parameter<std::string>(
                "attempt_1_ik_solver",
                "pick_ik/PickIkPlugin");
        second_ik_profile_.solver =
            this->declare_parameter<std::string>(
                "attempt_2_ik_solver",
                "kdl_kinematics_plugin/KDLKinematicsPlugin");
        third_ik_profile_.solver =
            this->declare_parameter<std::string>(
                "attempt_3_ik_solver",
                "pick_ik/PickIkPlugin");
        fallback_ik_profile_.solver = third_ik_profile_.solver;
        this->declare_parameter<std::string>(
            "primary_ik_solver",
            primary_ik_profile_.solver);
        fallback_ik_profile_.solver = this->declare_parameter<std::string>(
            "fallback_ik_solver",
            fallback_ik_profile_.solver);
        primary_ik_profile_.mode =
            this->declare_parameter<std::string>("attempt_1_ik_mode", "global");
        second_ik_profile_.mode =
            this->declare_parameter<std::string>("attempt_2_ik_mode", "global");
        third_ik_profile_.mode =
            this->declare_parameter<std::string>("attempt_3_ik_mode", "Speed");
        fallback_ik_profile_.mode = third_ik_profile_.mode;
        this->declare_parameter<std::string>("primary_ik_mode", primary_ik_profile_.mode);
        fallback_ik_profile_.mode =
            this->declare_parameter<std::string>("fallback_ik_mode", fallback_ik_profile_.mode);
        primary_ik_profile_.solve_type =
            this->declare_parameter<std::string>("attempt_1_ik_solve_type", "Speed");
        second_ik_profile_.solve_type =
            this->declare_parameter<std::string>("attempt_2_ik_solve_type", "Speed");
        third_ik_profile_.solve_type =
            this->declare_parameter<std::string>("attempt_3_ik_solve_type", "Distance");
        fallback_ik_profile_.solve_type =
            this->declare_parameter<std::string>("fallback_ik_solve_type", third_ik_profile_.solve_type);
        primary_ik_profile_.position_only_ik =
            this->declare_parameter<bool>("attempt_1_ik_position_only", false);
        second_ik_profile_.position_only_ik =
            this->declare_parameter<bool>("attempt_2_ik_position_only", false);
        third_ik_profile_.position_only_ik =
            this->declare_parameter<bool>("attempt_3_ik_position_only", false);
        fallback_ik_profile_.position_only_ik =
            this->declare_parameter<bool>("fallback_ik_position_only", third_ik_profile_.position_only_ik);
        primary_ik_profile_.rotation_scale =
            this->declare_parameter<double>("attempt_1_ik_rotation_scale", 0.03);
        second_ik_profile_.rotation_scale =
            this->declare_parameter<double>("attempt_2_ik_rotation_scale", 0.03);
        third_ik_profile_.rotation_scale =
            this->declare_parameter<double>("attempt_3_ik_rotation_scale", 0.01);
        fallback_ik_profile_.rotation_scale = third_ik_profile_.rotation_scale;
        primary_ik_profile_.position_threshold =
            this->declare_parameter<double>("attempt_1_ik_position_threshold", 0.002);
        second_ik_profile_.position_threshold =
            this->declare_parameter<double>("attempt_2_ik_position_threshold", 0.002);
        third_ik_profile_.position_threshold =
            this->declare_parameter<double>("attempt_3_ik_position_threshold", 0.002);
        fallback_ik_profile_.position_threshold = third_ik_profile_.position_threshold;
        primary_ik_profile_.orientation_threshold =
            this->declare_parameter<double>("attempt_1_ik_orientation_threshold", 0.30);
        second_ik_profile_.orientation_threshold =
            this->declare_parameter<double>("attempt_2_ik_orientation_threshold", 0.30);
        third_ik_profile_.orientation_threshold =
            this->declare_parameter<double>("attempt_3_ik_orientation_threshold", 0.30);
        fallback_ik_profile_.orientation_threshold = third_ik_profile_.orientation_threshold;
        primary_ik_profile_.goal_position_tolerance =
            this->declare_parameter<double>("attempt_1_goal_position_tolerance", 0.003);
        second_ik_profile_.goal_position_tolerance =
            this->declare_parameter<double>("attempt_2_goal_position_tolerance", 0.003);
        third_ik_profile_.goal_position_tolerance =
            this->declare_parameter<double>("attempt_3_goal_position_tolerance", 0.003);
        fallback_ik_profile_.goal_position_tolerance = third_ik_profile_.goal_position_tolerance;
        primary_ik_profile_.goal_orientation_tolerance =
            this->declare_parameter<double>("attempt_1_goal_orientation_tolerance", 0.20);
        second_ik_profile_.goal_orientation_tolerance =
            this->declare_parameter<double>("attempt_2_goal_orientation_tolerance", 0.20);
        third_ik_profile_.goal_orientation_tolerance =
            this->declare_parameter<double>("attempt_3_goal_orientation_tolerance", 0.20);
        fallback_ik_profile_.goal_orientation_tolerance =
            third_ik_profile_.goal_orientation_tolerance;
        primary_ik_profile_.minimal_displacement_weight =
            this->declare_parameter<double>(
                "attempt_1_ik_minimal_displacement_weight",
                0.02);
        second_ik_profile_.minimal_displacement_weight =
            this->declare_parameter<double>(
                "attempt_2_ik_minimal_displacement_weight",
                0.02);
        third_ik_profile_.minimal_displacement_weight =
            this->declare_parameter<double>(
                "attempt_3_ik_minimal_displacement_weight",
                0.20);
        fallback_ik_profile_.minimal_displacement_weight =
            third_ik_profile_.minimal_displacement_weight;
        primary_ik_profile_.gd_step_size =
            this->declare_parameter<double>("attempt_1_ik_gd_step_size", 0.0008);
        second_ik_profile_.gd_step_size =
            this->declare_parameter<double>("attempt_2_ik_gd_step_size", 0.0008);
        third_ik_profile_.gd_step_size =
            this->declare_parameter<double>("attempt_3_ik_gd_step_size", 0.0015);
        fallback_ik_profile_.gd_step_size = third_ik_profile_.gd_step_size;
        primary_ik_profile_.timeout =
            this->declare_parameter<double>("attempt_1_ik_timeout", 0.2);
        second_ik_profile_.timeout =
            this->declare_parameter<double>("attempt_2_ik_timeout", 0.4);
        third_ik_profile_.timeout =
            this->declare_parameter<double>("attempt_3_ik_timeout", 0.5);
        fallback_ik_profile_.timeout = third_ik_profile_.timeout;
        skip_failed_pick_after_retries_ =
            this->declare_parameter<bool>("skip_failed_pick_after_retries", true);

        effort_subscription_ =
            this->create_subscription<control_msgs::msg::DynamicJointState>(
                "/dynamic_joint_states",
                // Item 3.3: SensorDataQoS (best-effort) — telemetria a 100Hz
                // vinda da Pi por wifi nao deve criar backlog reliable no
                // enlace critico; a verificacao de garra so quer a amostra
                // MAIS NOVA. Casa com o QoS ja usado no place.
                rclcpp::SensorDataQoS(),
                std::bind(
                    &PickActionServer::onDynamicJointState,
                    this,
                    std::placeholders::_1));

        action_server_ =
            rclcpp_action::create_server<PickTag>(
            this,
            "/pick_tag",
            std::bind(
                &PickActionServer::handle_goal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            std::bind(
                &PickActionServer::handle_cancel,
                this,
                std::placeholders::_1),
            std::bind(
                &PickActionServer::handle_accepted,
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

    rclcpp_action::Server<PickTag>::SharedPtr action_server_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pick_active_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_subscription_;
    std::atomic<std::uint64_t> camera_info_count_{0};
    std::atomic<std::int64_t> camera_info_last_ns_{0};
    std::string camera_info_topic_;
    double perception_warmup_timeout_{6.0};
    double max_pose_error_m_{0.025};

    // Prateleira (2026-08-12). `shelf_table_poses_` e o GATILHO: se o
    // table_pose do goal estiver nesta lista, o ciclo usa a sequencia de
    // shelf em vez da aproximacao cartesiana de mesa comum.
    static constexpr const char * kShelfStartPose = "pegar_obj_sh";
    std::vector<std::string> shelf_table_poses_;
    std::string shelf_ik_reference_frame_;
    int force_table_ik_strategy_{0};
    std::vector<double> table_down_tilt_ladder_deg_;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
        effort_subscription_;
    bool speech_enabled_{true};
    bool verify_grasp_effort_{true};
    double grasp_min_effort_nm_{0.15};
    double grasp_min_effort_increase_nm_{0.05};
    double grasp_effort_sample_duration_{0.8};
    double grasp_effort_max_age_{0.4};
    int grasp_retry_attempts_{2};
    bool switch_ik_after_failed_grasp_{true};
    struct IkProfile
    {
        std::string solver{"pick_ik/PickIkPlugin"};
        std::string mode{"global"};
        std::string solve_type{"Speed"};
        bool position_only_ik{false};
        double rotation_scale{0.03};
        double position_threshold{0.002};
        double orientation_threshold{0.30};
        double goal_position_tolerance{0.003};
        double goal_orientation_tolerance{0.20};
        double minimal_displacement_weight{0.02};
        double gd_step_size{0.0008};
        double timeout{0.2};
    };
    IkProfile primary_ik_profile_;
    IkProfile second_ik_profile_{
        "kdl_kinematics_plugin/KDLKinematicsPlugin",
        "global",
        "Speed",
        false,
        0.03,
        0.002,
        0.30,
        0.003,
        0.20,
        0.02,
        0.0008,
        0.4};
    IkProfile third_ik_profile_{
        "pick_ik/PickIkPlugin",
        "Speed",
        "Distance",
        false,
        0.01,
        0.010,
        0.20,
        0.010,
        0.80,
        0.20,
        0.0015,
        0.5};
    IkProfile fallback_ik_profile_{
        "pick_ik/PickIkPlugin",
        "Speed",
        "Distance",
        false,
        0.01,
        0.010,
        0.90,
        0.010,
        0.80,
        0.20,
        0.0015,
        0.5};
    bool skip_failed_pick_after_retries_{true};
    double active_goal_position_tolerance_{0.003};
    double active_goal_orientation_tolerance_{0.20};
    std::string container_state_file_;
    std::unique_ptr<manip_task_execution::ContainerStateStore> container_state_store_;
    std::unique_ptr<manip_task_execution::ManipulatorExecutionLock> execution_lock_;
    std::atomic_bool cancel_requested_{false};
    // Item 2.5: causa da ultima falha do ciclo (vira fail_reason no result).
    std::string last_pick_failure_reason_;
    std::mutex active_interfaces_mutex_;
    std::shared_ptr<MoveGroupInterface> active_arm_;
    std::shared_ptr<MoveGroupInterface> active_gripper_;
    std::mutex effort_mutex_;
    double motor6_effort_{0.0};
    double motor7_effort_{0.0};
    std::chrono::steady_clock::time_point effort_update_time_;
    std::uint64_t effort_update_sequence_{0};
    bool effort_available_{false};

    class ExecutionGuard
    {
    public:
        explicit ExecutionGuard(PickActionServer & server)
        : server_(server)
        {
        }

        ~ExecutionGuard()
        {
            release();
        }

        // Libera os recursos UMA unica vez por goal: a 2a chamada (destrutor
        // depois de um finish explicito) soltava o flock ja readquirido pelo
        // goal seguinte e apagava o cancel dele (verificacao adversarial
        // 2026-08-10 — a flag do lock e compartilhada entre goals).
        void release()
        {
            if (!released_) {
                released_ = true;
                server_.releaseExecutionResources();
            }
        }

    private:
        PickActionServer & server_;
        bool released_{false};
    };

    bool cancellationRequested() const
    {
        return cancel_requested_.load();
    }

    static std::string spokenTagName(std::string tag_frame)
    {
        constexpr char prefix[] = "tag_";
        if (tag_frame.rfind(prefix, 0) == 0) {
            tag_frame.erase(0, sizeof(prefix) - 1);
        }
        for (char & character : tag_frame) {
            if (character == '_') {
                character = ' ';
            }
        }
        return tag_frame;
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

    void publishPickActive(bool active)
    {
        std_msgs::msg::Bool message;
        message.data = active;
        pick_active_publisher_->publish(message);
    }

    void applyIkProfile(const IkProfile & profile, const std::string & profile_name)
    {
        // Auditoria 2026-08-07, item 2.7: a "troca de solver" entre tentativas
        // era PLACEBO — o plugin de IK e cacheado no carregamento e o
        // parametro kinematics_solver setado aqui nunca chega ao move_group
        // (e o mode="Speed" do attempt_3 era invalido e ignorado em silencio).
        // Ficou so o que AGE de verdade por tentativa: os parametros numericos
        // dinamicos do pick_ik (valem no fallback local de IK aproximada) e as
        // goal tolerances do MotionPlanRequest.
        bool params_ok = true;
        const auto set_ik_params =
            [this, &profile, &params_ok](const std::string & prefix)
            {
                for (const auto & p : {
                        rclcpp::Parameter(
                            prefix + ".rotation_scale", profile.rotation_scale),
                        rclcpp::Parameter(
                            prefix + ".position_threshold",
                            profile.position_threshold),
                        rclcpp::Parameter(
                            prefix + ".orientation_threshold",
                            profile.orientation_threshold),
                        rclcpp::Parameter(
                            prefix + ".minimal_displacement_weight",
                            profile.minimal_displacement_weight),
                        rclcpp::Parameter(
                            prefix + ".gd_step_size", profile.gd_step_size)})
                {
                    if (!this->set_parameter(p).successful) {
                        params_ok = false;
                    }
                }
            };

        set_ik_params("robot_description_kinematics.arm");
        set_ik_params("arm");
        if (!params_ok) {
            RCLCPP_WARN(
                this->get_logger(),
                "applyIkProfile(%s): nem todos os parametros de IK aplicaram.",
                profile_name.c_str());
        }

        RCLCPP_WARN(
            this->get_logger(),
            "IK profile %s: rotation_scale=%.3f position_threshold=%.3f "
            "orientation_threshold=%.3f minimal_displacement_weight=%.3f "
            "gd_step_size=%.4f | goal tol pos=%.3f ori=%.3f",
            profile_name.c_str(),
            profile.rotation_scale,
            profile.position_threshold,
            profile.orientation_threshold,
            profile.minimal_displacement_weight,
            profile.gd_step_size,
            profile.goal_position_tolerance,
            profile.goal_orientation_tolerance);
        active_goal_position_tolerance_ = profile.goal_position_tolerance;
        active_goal_orientation_tolerance_ = profile.goal_orientation_tolerance;
    }

    const IkProfile & ikProfileForAttempt(int attempt) const
    {
        if (attempt <= 1) {
            return primary_ik_profile_;
        }
        if (attempt == 2) {
            return second_ik_profile_;
        }
        return third_ik_profile_;
    }

    std::string ikProfileNameForAttempt(int attempt) const
    {
        return "attempt_" + std::to_string(attempt);
    }

    // 2026-08-17 — ladder de IK da MESA COMUM (pedido do operador): tentativa
    // 1 = IK custom com a ferramenta para BAIXO (j5 alinhado ao yaw da tag;
    // desde 2026-08-24 com escada de inclinacao 0/15/30 graus dentro da
    // mesma tentativa — ver table_down_tilt_ladder_deg), tentativa 2 = IK
    // custom pela FRENTE (j5 fixo), tentativa 3+ = caminho antigo do
    // MoveIt/pick_ik, que ja funciona, como ultimo recurso.
    enum class TableIkStrategy
    {
        kCustomDown,
        kCustomForward,
        kMoveItPose,
    };

    TableIkStrategy tableStrategyForAttempt(int attempt) const
    {
        const int effective =
            force_table_ik_strategy_ > 0 ? force_table_ik_strategy_ : attempt;
        if (effective <= 1) {
            return TableIkStrategy::kCustomDown;
        }
        if (effective == 2) {
            return TableIkStrategy::kCustomForward;
        }
        return TableIkStrategy::kMoveItPose;
    }

    void configureArmInterface(const std::shared_ptr<MoveGroupInterface> & arm)
    {
        arm->setPoseReferenceFrame("base_footprint");
        arm->setPlanningTime(15.0);
        arm->setNumPlanningAttempts(20);
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(1.0);
    }

    // Auditoria 2026-08-07, item 1.2: sem prazo, o construtor do
    // MoveGroupInterface espera o action server do move_group PARA SEMPRE.
    // 15s folgados; no estouro (throw do MoveIt) devolve nullptr e o
    // chamador converte em falha de goal explicita.
    std::shared_ptr<MoveGroupInterface> makeInterfaceWithTimeout(
        const std::string & group)
    {
        // Sonda ANTES de construir (teste mock 2026-08-07): sem move_group no
        // ar, o construtor do MGI aborta o processo inteiro (SIGABRT em thread
        // interna do MoveIt, fora do alcance do try/catch). Se /move_action
        // nao existe, falha limpa sem tocar no construtor.
        auto probe = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
            shared_from_this(), "move_action");
        if (!probe->wait_for_action_server(std::chrono::seconds(15))) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
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
                this->get_logger(),
                "MoveGroupInterface('" << group << "') indisponivel em 15s: "
                    << e.what());
            return nullptr;
        }
    }

    std::shared_ptr<MoveGroupInterface> createArmInterface(bool fallback_profile)
    {
        applyIkProfile(
            fallback_profile ? fallback_ik_profile_ : primary_ik_profile_,
            fallback_profile ? "fallback" : "primary");
        auto arm = makeInterfaceWithTimeout("arm");
        if (arm) {
            configureArmInterface(arm);
        }
        return arm;
    }

    std::shared_ptr<MoveGroupInterface> createArmInterfaceForAttempt(int attempt)
    {
        applyIkProfile(ikProfileForAttempt(attempt), ikProfileNameForAttempt(attempt));
        auto arm = makeInterfaceWithTimeout("arm");
        if (arm) {
            configureArmInterface(arm);
        }
        return arm;
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
            if (joint_name != "manip_joint6" &&
                joint_name != "manip_joint7") {
                continue;
            }

            const auto & interface_value = message->interface_values[i];
            for (size_t j = 0;
                j < interface_value.interface_names.size();
                ++j) {
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

        const auto age =
            std::chrono::steady_clock::now() - effort_update_time_;
        if (age > std::chrono::duration<double>(grasp_effort_max_age_)) {
            return std::nullopt;
        }

        return GripperEffortSample{
            std::abs(motor6_effort_),
            std::abs(motor7_effort_),
            effort_update_sequence_
        };
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
                this->get_logger(),
                "[%s] grasp verification failed: no fresh effort samples",
                cycle_name.c_str());
            return false;
        }

        const double motor6_average =
            motor6_sum / static_cast<double>(sample_count);
        const double motor7_average =
            motor7_sum / static_cast<double>(sample_count);
        const double motor6_increase =
            motor6_average - baseline.motor6;
        const double motor7_increase =
            motor7_average - baseline.motor7;

        const bool motor6_loaded =
            motor6_average >= grasp_min_effort_nm_ &&
            motor6_increase >= grasp_min_effort_increase_nm_;
        const bool motor7_loaded =
            motor7_average >= grasp_min_effort_nm_ &&
            motor7_increase >= grasp_min_effort_increase_nm_;

        RCLCPP_INFO(
            this->get_logger(),
            "[%s] grasp effort: M6 baseline=%.3f avg=%.3f delta=%.3f Nm; "
            "M7 baseline=%.3f avg=%.3f delta=%.3f Nm; samples=%zu",
            cycle_name.c_str(),
            baseline.motor6,
            motor6_average,
            motor6_increase,
            baseline.motor7,
            motor7_average,
            motor7_increase,
            sample_count);

        return motor6_loaded && motor7_loaded;
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
        publishPickActive(false);
        // Auditoria 2026-08-07, item 2.6: NAO zerar active_arm_/gripper_ —
        // manter um MGI vivo preserva o CurrentStateMonitor COMPARTILHADO
        // aquecido (a subscription de joint_states nao esfria entre goals;
        // era a causa do "Failed to fetch current robot state" no 1o uso).
        cancel_requested_.store(false);
        execution_lock_->release();
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

    void publish_stage(
        const std::shared_ptr<GoalHandlePickTag> & goal_handle,
        const std::string & stage)
    {
        auto feedback = std::make_shared<PickTag::Feedback>();
        feedback->current_stage = stage;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "[ACTION] stage=%s", stage.c_str());
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const PickTag::Goal> goal)
    {
        if (goal->tag_frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rejecting PICK goal: tag_frame is empty");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!execution_lock_->tryAcquire()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejecting PICK for '%s': manipulator is busy",
                goal->tag_frame.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        cancel_requested_.store(false);
        RCLCPP_INFO(
            this->get_logger(),
            "Received goal tag=%s",
            goal->tag_frame.c_str());
        last_pick_failure_reason_.clear();
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickTag>)
    {
        RCLCPP_WARN(this->get_logger(), "PICK cancellation requested");
        cancel_requested_.store(true);
        stopActiveMotion();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePickTag> goal_handle)
    {
        std::thread(
            std::bind(
                &PickActionServer::execute,
                this,
                std::placeholders::_1),
            goal_handle).detach();
    }

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

    // Item 3.10 (gate da percepcao): espera a camera voltar a entregar frames.
    // Custo ZERO quando ela ja esta streamando ou quando nao ha camera no
    // grafo (mock / use_camera:=false). Nunca reprova o ciclo — no pior caso
    // segue e deixa a deteccao (com varredura e retries) decidir.
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
            return;  // camera ja entregando: nada a esperar
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
                    this->get_logger(),
                    "[%s] camera voltou a streamar em %.1fs",
                    cycle_name.c_str(),
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - start).count());
                return;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }

        RCLCPP_WARN(
            this->get_logger(),
            "[%s] camera nao entregou frame em %.1fs (%s) — seguindo assim "
            "mesmo; a deteccao tem varredura e retries.",
            cycle_name.c_str(),
            perception_warmup_timeout_,
            camera_info_topic_.c_str());
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
                    this->get_logger(),
                    "[%s] canceled while waiting for TF",
                    cycle_name.c_str());
                return false;
            }
            try {
                out_tf = getTagTransform(reference_frame, tag_frame);
                // Transform existir nao basta: precisa ser deteccao RECENTE
                // (robo movel + camera no braco => TF velha = alvo fantasma).
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
                this->get_logger(),
                "[" << cycle_name << "] Tag " << tag_frame
                    << " vista pela ultima vez ha " << stale_age_sec
                    << " s (max " << max_tag_age_sec_
                    << " s) — a camera NAO esta vendo a tag agora.");
        } else {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "[" << cycle_name << "] Timed out waiting TF "
                    << reference_frame << " <- " << tag_frame
                    << " after " << timeout.count() << " ms. Last error: "
                    << last_ex.what());
        }
        return false;
    }

    double max_tag_age_sec_{1.0};

    // ---- Deadlines client-side (auditoria 2026-08-07, item 1.1) ----
    // MoveGroupInterface::plan()/execute() bloqueiam em `while(!done) sleep(1ms)`
    // SEM timeout e sem checar rclcpp::ok(): resposta perdida no DDS = chamada
    // pendurada PARA SEMPRE (2 travamentos observados em campo em 07/08).
    // Prazos LARGOS que nunca cancelam acao saudavel: plan = 15s de planning
    // + 10s de margem; execute = duracao planejada x3 + 3s (espelho do monitor
    // server-side) + 10s de margem. No estouro: FATAL + stop() + falha; a
    // thread orfa fica no busy-wait de 1ms do MoveIt (custo desprezivel) e
    // escreve num Plan proprio (nunca no do chamador — sem use-after-free).
    // Estouros repetidos = reiniciar o no.
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
            this->get_logger(),
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
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Planning failed: " << label);
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
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Execution failed: " << label);
            return false;
        }

        return true;
    }

    // Varredura de busca da tag (2026-08-07): o robo pode docar com alguns
    // cm/graus de erro e a tag cair fora do FOV da camera (visto na missao
    // 3-objetos: WS1 "um pouco torta" = picks pulados sem nem ver as tags).
    // Gira a junta 1 em passos alternados a partir da pose atual, re-testando
    // a deteccao em cada parada. Achou -> segue o ciclo (o alinhamento XY da
    // camera centraliza depois). Nao achou -> volta ao centro e falha.
    bool sweepForTag(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & tag_frame,
        geometry_msgs::msg::TransformStamped & out_tf,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle)
    {
        const std::vector<double> offsets = {-0.30, 0.30, -0.60, 0.60};
        std::vector<double> joints = arm->getCurrentJointValues();
        if (joints.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "[%s] varredura abortada: sem estado atual do braco",
                cycle_name.c_str());
            return false;
        }
        const double center = joints[0];
        speak("Vou procurar a tag " + spokenTagName(tag_frame));
        int step = 0;
        for (const double offset : offsets) {
            if (cancellationRequested()) {
                return false;
            }
            // Verificacao adversarial 2026-08-10: a varredura inteira rodava
            // MUDA dentro do gap do estagio detecting_tag — o operador nao via
            // nada acontecer e o watchdog de 120s do BT contava o tempo todo
            // como "sem progresso". Cada passo agora publica.
            publish_stage(
                goal_handle,
                "sweeping_for_tag_" + std::to_string(++step) + "_of_" +
                std::to_string(static_cast<int>(offsets.size())));
            joints[0] = center + offset;
            arm->setJointValueTarget(joints);
            if (!planAndExecute(arm, cycle_name + " sweep")) {
                continue;  // passo invalido (limite/colisao): tenta o proximo
            }
            if (waitForTagTransform(
                    "base_footprint",
                    tag_frame,
                    out_tf,
                    std::chrono::milliseconds(900),
                    std::chrono::milliseconds(100),
                    cycle_name + " sweep_detect"))
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[%s] tag %s encontrada na varredura (offset %+.2f rad)",
                    cycle_name.c_str(), tag_frame.c_str(), offset);
                return true;
            }
        }
        publish_stage(goal_handle, "sweep_returning_to_center");
        joints[0] = center;
        arm->setJointValueTarget(joints);
        if (!planAndExecute(arm, cycle_name + " sweep_return")) {
            // Verificacao adversarial 2026-08-10: a falha do retorno era
            // descartada — a tentativa seguinte recomecava da pose GIRADA e
            // varria o lado errado achando que estava no centro.
            RCLCPP_WARN(
                this->get_logger(),
                "[%s] varredura nao conseguiu voltar ao centro — a proxima "
                "tentativa comeca da pose girada.",
                cycle_name.c_str());
        }
        return false;
    }

    // Confere ONDE a ponta realmente parou depois de executar. Extraido de
    // moveToTarget (2026-08-12) para o ramo de prateleira usar a mesma
    // protecao: sem isto o ciclo fecha a garra em qualquer lugar que o braco
    // tenha alcancado. `note` so enriquece a mensagem de erro.
    bool verifyTcpArrival(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & eef_link,
        const geometry_msgs::msg::Point & target_position,
        const std::string & label,
        const std::string & note = "")
    {
        if (max_pose_error_m_ <= 0.0) {
            return true;
        }

        geometry_msgs::msg::PoseStamped achieved;
        try {
            achieved = arm->getCurrentPose(eef_link);
        } catch (const std::exception & ex) {
            RCLCPP_WARN_STREAM(
                this->get_logger(),
                "Nao consegui ler a pose atingida em " << label << " ("
                    << ex.what() << ") — seguindo sem verificar.");
            return true;
        }

        const double dx = achieved.pose.position.x - target_position.x;
        const double dy = achieved.pose.position.y - target_position.y;
        const double dz = achieved.pose.position.z - target_position.z;
        const double err = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (err > max_pose_error_m_) {
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] o braco PAROU A %.1f cm do alvo (limite %.1f cm)%s. "
                "Alvo [%.3f %.3f %.3f], atingido [%.3f %.3f %.3f]. "
                "Falhando em vez de fechar a garra no vazio.",
                label.c_str(), err * 100.0, max_pose_error_m_ * 100.0,
                note.c_str(),
                target_position.x, target_position.y, target_position.z,
                achieved.pose.position.x, achieved.pose.position.y,
                achieved.pose.position.z);
            speak("Falha: o braco nao chegou na pose de pega");
            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[%s] chegou a %.1f cm do alvo%s.",
            label.c_str(), err * 100.0, note.c_str());
        return true;
    }

    // ---------------------------------------------------------------------
    // PRATELEIRA (shelf) — 2026-08-12
    //
    // Alcancar uma tag numa prateleira nao e a mesma coisa que numa mesa: o
    // braco precisa subir "por fora" antes de estender, senao bate. Por isso
    // este caminho NAO usa a IK do MoveIt (setPoseTarget), e sim a IK propria
    // (custom_ik.hpp) em duas fases de espaco de juntas, definidas pelo
    // operador:
    //   fase 1 — j1/j4 da IK, j2 = 0, j3 = kShelfPhase1Joint3, j5 = +1.5708
    //   fase 2 — j1..j4 da IK, j5 continua +1.5708
    // O retorno faz o caminho inverso.
    static constexpr double kShelfPhase1Joint3 = 2.3038;
    static constexpr double kShelfWristJoint5 = 1.5708;
    // Punho da tentativa 2 da mesa (kForward), pedido do operador 2026-08-17;
    // 2026-08-21: -1.5708 (pedido do operador). A shelf continua +1.5708.
    static constexpr double kTableForwardWristJoint5 = -1.5708;

    enum class CustomIkOutcome { kOk, kNoTransform, kNoSolution };

    /// Resolve uma tag pela IK propria (generico: shelf E mesa). `q_out` sai
    /// em ordem j1..j5. O alvo e convertido para o frame da BASE DO BRACO —
    /// nao basta subtrair o X do mount: ha tambem um yaw de -1.57
    /// (manip_mount_rpy). `label` so muda os logs ("shelf"/"mesa").
    /// `target_yaw_out` (opcional): yaw do eixo X do frame alvo projetado no
    /// plano XY da base do braco — usado para alinhar o j5 aos dedos.
    /// `tilt_from_vertical`: direcao da ferramenta como inclinacao continua a
    /// partir da vertical (0 = por cima, pi/4 = kMiddle, pi/2 = frente).
    CustomIkOutcome solveCustomIkForTag(
        const std::string & tag_frame,
        double tilt_from_vertical,
        double q5_fixed,
        const char * label,
        const std::string & cycle_name,
        std::array<double, 5> & q_out,
        double * target_yaw_out = nullptr)
    {
        geometry_msgs::msg::TransformStamped tf_arm;
        try {
            tf_arm = tf_buffer_->lookupTransform(
                shelf_ik_reference_frame_, tag_frame,
                tf2::TimePointZero, tf2::durationFromSec(0.5));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] %s: sem TF %s <- %s (%s)",
                cycle_name.c_str(), label, shelf_ik_reference_frame_.c_str(),
                tag_frame.c_str(), ex.what());
            return CustomIkOutcome::kNoTransform;
        }

        const Eigen::Vector3d target(
            tf_arm.transform.translation.x,
            tf_arm.transform.translation.y,
            tf_arm.transform.translation.z);

        if (!manip_task_execution::solveIk(
                target,
                tilt_from_vertical,
                q5_fixed,
                q_out))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[%s] %s: IK propria nao achou solucao para [%.3f %.3f %.3f] "
                "em %s (ferramenta a %.0f graus da vertical).",
                cycle_name.c_str(), label, target.x(), target.y(), target.z(),
                shelf_ik_reference_frame_.c_str(),
                tilt_from_vertical * 180.0 / M_PI);
            return CustomIkOutcome::kNoSolution;
        }

        if (target_yaw_out) {
            *target_yaw_out = manip_task_execution::projectedFrameYaw(
                Eigen::Quaterniond(
                    tf_arm.transform.rotation.w,
                    tf_arm.transform.rotation.x,
                    tf_arm.transform.rotation.y,
                    tf_arm.transform.rotation.z).toRotationMatrix());
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[%s] %s IK: alvo [%.3f %.3f %.3f] -> j "
            "[%.4f %.4f %.4f %.4f %.4f]",
            cycle_name.c_str(), label, target.x(), target.y(), target.z(),
            q_out[0], q_out[1], q_out[2], q_out[3], q_out[4]);
        return CustomIkOutcome::kOk;
    }

    /// Wrapper da shelf — comportamento identico ao historico (kMiddle,
    /// punho +1.5708, logs "shelf").
    bool solveShelfIk(
        const std::string & tag_frame,
        const std::string & cycle_name,
        std::array<double, 5> & q_out)
    {
        return solveCustomIkForTag(
            tag_frame,
            manip_task_execution::tiltFromVertical(
                manip_task_execution::ToolDirection::kMiddle),
            kShelfWristJoint5,
            "shelf",
            cycle_name,
            q_out) == CustomIkOutcome::kOk;
    }

    /// Move o braco para um alvo em espaco de juntas (5 valores) planejando
    /// pelo MoveIt — mantem checagem de colisao, deadlines e retries.
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

        // 2026-08-13: as fases da prateleira morriam com GOAL_TOLERANCE_
        // VIOLATED e o log do PC nao dizia QUAL junta ficou fora. Loga o erro
        // por junta para o proximo teste apontar o culpado na hora.
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

    /// Sequencia de aproximacao da prateleira (fases 1 e 2).
    bool approachShelfTarget(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & tag_frame,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle,
        std::array<double, 5> & q_ik_out)
    {
        if (!solveShelfIk(tag_frame, cycle_name, q_ik_out)) {
            return false;
        }

        // Decisao do operador 2026-08-15: a tag e lida UMA unica vez (na IK
        // acima). A partir do momento em que o braco COMECA a se mover
        // (fase 1), nao ha mais NENHUMA releitura de tag/camera nem
        // verificacao de chegada — os alvos sao juntas exatas da nossa IK
        // (nao ha IK aproximada aqui) e a protecao contra "pegar o vazio"
        // fica com a verificacao de esforco da garra, que ja roda depois.
        publish_stage(goal_handle, "shelf_phase1");
        const std::array<double, 5> phase1{
            q_ik_out[0], 0.0, kShelfPhase1Joint3, q_ik_out[3], kShelfWristJoint5};
        if (!moveToJointTarget(arm, phase1, cycle_name + " shelf fase1")) {
            return false;
        }

        publish_stage(goal_handle, "shelf_phase2");
        const std::array<double, 5> phase2{
            q_ik_out[0], q_ik_out[1], q_ik_out[2], q_ik_out[3], kShelfWristJoint5};
        return moveToJointTarget(arm, phase2, cycle_name + " shelf fase2");
    }

    /// Caminho inverso: desfaz a fase 2, depois a fase 1, e volta a pose de
    /// partida da prateleira — com o bloco na garra.
    bool retreatFromShelf(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::array<double, 5> & q_ik,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle)
    {
        publish_stage(goal_handle, "shelf_retreat");

        const std::array<double, 5> phase1{
            q_ik[0], 0.0, kShelfPhase1Joint3, q_ik[3], kShelfWristJoint5};
        if (!moveToJointTarget(arm, phase1, cycle_name + " shelf retorno fase1")) {
            return false;
        }

        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget(kShelfStartPose);
        return planAndExecute(arm, cycle_name + " shelf retorno " + kShelfStartPose);
    }

    // -----------------------------------------------------------------------
    // MESA COMUM pela IK custom (2026-08-17) — tentativas 1 (kDown, j5 pelo
    // yaw da tag) e 2 (kForward, j5 fixo). Mesmo contrato da shelf: a tag e
    // lida UMA vez (no lookup da IK), sem releitura nem verifyTcpArrival
    // depois que o braco comeca a se mover — as juntas sao exatas e a
    // protecao contra "pegar o vazio" e a verificacao de esforco da garra.
    bool approachTableTargetCustomIk(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & tag_frame,
        TableIkStrategy strategy,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle)
    {
        const bool down = strategy == TableIkStrategy::kCustomDown;
        publish_stage(
            goal_handle,
            down ? "final_approach_custom_down" : "final_approach_custom_forward");

        std::array<double, 5> q{};
        double tag_yaw = 0.0;
        double tilt_used = 0.0;
        CustomIkOutcome outcome = CustomIkOutcome::kNoSolution;

        if (down) {
            // Escada "por cima": vertical estrita e depois inclinacoes
            // crescentes (table_down_tilt_ladder_deg). So cai para a pegada
            // frontal (degrau 2) se NENHUMA delas alcancar.
            std::vector<double> tilts_deg{0.0};
            for (const double t : table_down_tilt_ladder_deg_) {
                if (t > 0.0 && t < 90.0) {
                    tilts_deg.push_back(t);
                }
            }
            for (const double tilt_deg : tilts_deg) {
                const double tilt = tilt_deg * M_PI / 180.0;
                outcome = solveCustomIkForTag(
                    tag_frame, tilt, 0.0, "mesa", cycle_name, q, &tag_yaw);
                if (outcome != CustomIkOutcome::kNoSolution) {
                    tilt_used = tilt;
                    break;
                }
            }
        } else {
            outcome = solveCustomIkForTag(
                tag_frame,
                manip_task_execution::tiltFromVertical(
                    manip_task_execution::ToolDirection::kForward),
                kTableForwardWristJoint5,
                "mesa",
                cycle_name,
                q,
                nullptr);
        }

        if (outcome == CustomIkOutcome::kNoTransform) {
            speak("Falha: perdi a tag na aproximação");
            return false;
        }
        if (outcome == CustomIkOutcome::kNoSolution) {
            speak("Falha: sem solução de I K para a tag " + spokenTagName(tag_frame));
            return false;
        }

        if (down) {
            // O TCP fica no eixo do joint5: trocar q5 pos-solve nao move a
            // ponta, so gira a linha dos dedos para o yaw da tag (formula
            // generalizada para a ferramenta inclinada).
            q[4] = manip_task_execution::computeWristForTagYaw(tag_yaw, q[0], tilt_used);
            RCLCPP_INFO(
                this->get_logger(),
                "[%s] mesa: pegada por cima a %.0f graus da vertical; j5 pela "
                "orientacao da tag: yaw=%.4f -> q5=%.4f",
                cycle_name.c_str(), tilt_used * 180.0 / M_PI, tag_yaw, q[4]);
        }

        speak("Encontrei uma solução de I K para a tag " + spokenTagName(tag_frame));
        return moveToJointTarget(
            arm, q,
            cycle_name + (down ? " mesa custom down" : " mesa custom forward"));
    }

    bool moveToTarget(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const geometry_msgs::msg::TransformStamped & tf,
        const std::string & eef_link,
        const std::string & label,
        bool use_orientation_constraint,
        bool enforce_pitch_pi,
        const std::string & ik_success_speech = "",
        const std::string & ik_failure_speech = "")
    {
        if (cancellationRequested()) {
            return false;
        }

        // Guarda de estado (auditoria 2026-08-07, item 2.8c): getCurrentPose
        // abaixo devolve IDENTIDADE em silencio sem estado atual — melhor
        // falhar explicito aqui do que planejar para um alvo corrompido.
        if (!arm->getCurrentState(2.0)) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Sem estado atual do braco em moveToTarget (" << label
                    << ") — abortando a tentativa.");
            return false;
        }

        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink(eef_link);

        const double x = tf.transform.translation.x;
        const double y = tf.transform.translation.y;
        const double z = tf.transform.translation.z;

        // Yaw RADIAL: azimute do EIXO DA JUNTA 1 (x=+0.217 m do base_footprint,
        // manip_mount_xyz do robot.urdf.xacro) ate o alvo. E o yaw natural
        // deste braco de 5 juntas com pegada top-down (a junta 5 gira a garra
        // livre em torno do eixo vertical). Decisao do operador 2026-08-07:
        // a orientacao da TAG (ruidosa em 480p/USB2 e fonte de "unable to
        // sample goal states") NAO entra na pose de grasp em NENHUM caminho.
        constexpr double kJoint1AxisX = 0.217;
        const double yaw_radial = std::atan2(y, x - kJoint1AxisX);

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        if (use_orientation_constraint) {
            tf2::Quaternion desired_q;
            if (enforce_pitch_pi) {
                desired_q.setRPY(0.0, M_PI, yaw_radial);
            } else {
                const auto current_pose = arm->getCurrentPose(eef_link).pose;
                tf2::Quaternion current_q;
                tf2::fromMsg(current_pose.orientation, current_q);
                double current_roll = 0.0;
                double current_pitch = 0.0;
                double current_yaw = 0.0;
                tf2::Matrix3x3(current_q).getRPY(
                    current_roll,
                    current_pitch,
                    current_yaw);
                (void)current_yaw;
                // Item 2.8b: aqui tambem o yaw vem do radial, nunca da tag.
                desired_q.setRPY(current_roll, 0.0, yaw_radial);
            }
            desired_q.normalize();
            target_pose.orientation = tf2::toMsg(desired_q);
        } else {
            target_pose.orientation = arm->getCurrentPose(eef_link).pose.orientation;
        }

        MoveGroupInterface::Plan plan;

        arm->setGoalPositionTolerance(active_goal_position_tolerance_);
        arm->setGoalOrientationTolerance(
            use_orientation_constraint ? active_goal_orientation_tolerance_ : M_PI);

        arm->clearPoseTargets();
        arm->setPoseTarget(target_pose, eef_link);

        bool success = planWithDeadline(arm, plan, label + " setPoseTarget");
        bool used_approximate_ik = false;

        if (!success) {
            RCLCPP_WARN_STREAM(
                this->get_logger(),
                "Plan falhou com setPoseTarget em " << label
                                                                                         << ". Tentando IK aproximada.");

            // Guarda anti-travamento (2026-08-07): a IK aproximada precisa do
            // estado atual; sem ele o fluxo ficava pendurado INDEFINIDAMENTE
            // (visto: 6 min mudo apos "Failed to fetch current robot state").
            // Sem estado em 2 s -> falha limpa e o ladder de retry/skip segue.
            if (!arm->getCurrentState(2.0)) {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Sem estado atual do braco para IK aproximada em " << label
                        << " — abortando a tentativa.");
                speak(ik_failure_speech);
                return false;
            }

            arm->clearPoseTargets();
            arm->setApproximateJointValueTarget(target_pose, eef_link);

            success = planWithDeadline(arm, plan, label + " IK aproximada");
            used_approximate_ik = true;
        }

        if (!success) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Planning failed: " << label);
            speak(ik_failure_speech);
            return false;
        }

        speak(ik_success_speech);

        if (cancellationRequested()) {
            return false;
        }

        const bool exec_ok = executeWithDeadline(arm, plan, label);

        if (cancellationRequested()) {
            return false;
        }

        if (!exec_ok) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Execution failed: " << label);
            return false;
        }

        // 2026-08-10 — VERIFICACAO DE CHEGADA (bug observado no robo: "so
        // fechou a garra, nao se aproximou do bloco"). Quando setPoseTarget
        // falha, o fallback setApproximateJointValueTarget devolve a solucao
        // alcancavel MAIS PROXIMA — que pode estar a dezenas de cm do alvo.
        // O codigo executava isso, retornava true, e o ciclo fechava a garra
        // NO VAZIO. Agora conferimos ONDE a ponta parou; longe demais = falha
        // retentavel, nao pega fantasma. Tambem cobre erro de rastreio do
        // controlador (tolerancia de 0.10 rad por junta vira cm na ponta).
        return verifyTcpArrival(
            arm, eef_link, target_pose.position, label,
            used_approximate_ik ? " [via IK APROXIMADA]" : "");
    }

    // createApproachTask/executeTask REMOVIDOS (auditoria 2026-08-07, item
    // 1.3): a task MTC era um CurrentState->MoveTo("pegar_obj") REDUNDANTE
    // (o fluxo ja leva o braco a pegar_obj com recovery logo antes), a falha
    // era sempre engolida pelo fallback, e task.execute() do MTC espera o
    // result SEM timeout — resposta perdida no DDS = server "ocupado" para
    // sempre (raiz dos travamentos e do bug "server ocupado" de 04/08).

    // Auditoria 2026-08-07, item 2.2: o desfecho do transporte distingue
    // falha COM o bloco ainda na garra (nunca pode virar skip "sucesso" — o
    // gripper_open do proximo goal derrubaria o bloco no chassi) de falha
    // DEPOIS de soltar (bloco JA no container = sucesso de fato).
    enum class TransferOutcome { kOk, kFailHolding, kFailAfterRelease };

    TransferOutcome transferGraspedObjectToContainer(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string & container_pose,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle,
        bool is_shelf)
    {
        // Prateleira (pedido 2026-08-17): NUNCA passar por pegar_obj com o
        // bloco vindo da estante — o retorno ja deixou o braco em
        // pegar_obj_sh; segue direto ao pre_container.
        if (!is_shelf) {
            publish_stage(goal_handle, "returning_to_pegar_obj");
            speak("Bloco seguro. Voltando para a pose de transporte");
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget("pegar_obj");
            if (!planAndExecute(arm, cycle_name + " return pegar_obj")) {
                return TransferOutcome::kFailHolding;
            }
        }

        publish_stage(goal_handle, "going_pre_container");
        //speak("Indo para a pre pose do container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, cycle_name + " pre_container")) {
            return TransferOutcome::kFailHolding;
        }

        publish_stage(goal_handle, "going_container");
        speak("Levando o bloco para o " + container_pose);
        arm->setStartStateToCurrentState();
        arm->setNamedTarget(container_pose);
        if (!planAndExecute(arm, cycle_name + " " + container_pose)) {
            return TransferOutcome::kFailHolding;
        }

        publish_stage(goal_handle, "opening_gripper");
        //speak("Abrindo a garra para soltar o bloco");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, cycle_name + " open gripper")) {
            return TransferOutcome::kFailHolding;
        }

        publish_stage(goal_handle, "going pre_container_final");
        //speak("Saindo do container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, cycle_name + " pre_container final")) {
            return TransferOutcome::kFailAfterRelease;
        }
        return TransferOutcome::kOk;
    }

    bool runTransferWithRetry(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string & container_pose,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle,
        bool is_shelf,
        bool & object_still_grasped)
    {
        TransferOutcome transfer = transferGraspedObjectToContainer(
            arm, gripper, container_pose, cycle_name, goal_handle, is_shelf);
        if (transfer == TransferOutcome::kFailHolding) {
            speak("Falha no transporte com o bloco na garra. Tentando o transporte de novo");
            transfer = transferGraspedObjectToContainer(
                arm, gripper, container_pose,
                cycle_name + " transfer_retry", goal_handle, is_shelf);
        }
        if (transfer == TransferOutcome::kFailHolding) {
            // Item 2.2b: NUNCA vira skip — o gripper_open do proximo goal
            // derrubaria o bloco no chassi. Aborta o goal com causa explicita.
            object_still_grasped = true;
            speak("Falha: o bloco continua preso na garra");
            last_pick_failure_reason_ = "object_still_grasped";
            return false;
        }
        if (transfer == TransferOutcome::kFailAfterRelease) {
            // Item 2.2a: bloco JA depositado no container — sucesso de fato
            // (so a saida falhou); contabilizar ocupacao normalmente.
            RCLCPP_WARN(
                this->get_logger(),
                "[%s] saida do container falhou, mas o bloco JA foi solto no "
                "%s — contabilizando como sucesso.",
                cycle_name.c_str(), container_pose.c_str());
        }
        return true;
    }

    bool alignCameraToTagXY(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & tag_frame,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle,
        const std::string & cycle_name)
    {
        publish_stage(goal_handle, "camera_xy_alignment");
        //speak("Alinhando a camera em X e Y pela tag " + spokenTagName(tag_frame));

        geometry_msgs::msg::TransformStamped tag_tf;
        if (!waitForTagTransform(
                "manip_base_link",
                tag_frame,
                tag_tf,
                std::chrono::milliseconds(900),
                std::chrono::milliseconds(100),
                cycle_name + " camera_xy_alignment")) {
            speak("Nao consegui alinhar a camera porque nao encontrei a tag");
            return false;
        }

        constexpr double kTagXNearZero = 0.1;
        constexpr double kTagYNearZero = 0.7;
        const double tag_x = tag_tf.transform.translation.x;
        const double tag_y = tag_tf.transform.translation.y;

        std::string target;
        if (std::abs(tag_x) > kTagXNearZero) {
            if (tag_x > 0.0) {
                target = std::abs(tag_y) > kTagYNearZero ?
                    "tag_direita_cima" :
                    "tag_direita";
            } else {
                target = std::abs(tag_y) > kTagYNearZero ?
                    "tag_esquerda_cima" :
                    "tag_esquerda";
            }
        } else if (std::abs(tag_y) > kTagYNearZero) {
            target = "tag_cima";
        }

        if (target.empty()) {
            speak("A camera ja esta alinhada com a tag");
            return true;
        }

        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget(target);
        if (!planAndExecute(arm, cycle_name + " " + target)) {
            speak("Falhei ao alinhar a camera com a tag");
            return false;
        }

        return sleepInterruptibly(std::chrono::milliseconds(120));
    }

    // runCameraAlignmentRetry REMOVIDO (auditoria 2026-08-07, item 3.4):
    // era codigo morto — definido e nunca chamado; prometia um retry que
    // nao existia (parametro use_camera_alignment_retry sem efeito).

    bool run_pick_cycle(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string & tag_frame,
        const std::string & container_pose,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle,
        int attempt,
        bool is_shelf,
        bool & retryable_failure,
        bool & object_still_grasped)
    {
        // Auditoria 2026-08-07, item 2.3: TODA falha PRE-grasp e retentavel
        // (o ladder de 3 tentativas cobre o caso mais comum dos logs — falha
        // de IK/plan — que antes pulava a tag na 1a). Cada porta de falha
        // NOMEIA a causa em voz+log (pedido do operador). Falha pos-grasp
        // (transfer com bloco na garra) NAO seta o flag.
        retryable_failure = false;
        publish_stage(goal_handle, "detecting_tag");
        speak("Procurando a tag " + spokenTagName(tag_frame));
        waitForCameraStream(cycle_name);

        geometry_msgs::msg::TransformStamped tag_tf;
        if (!waitForTagTransform(
                "base_footprint",
                tag_frame,
                tag_tf,
                std::chrono::milliseconds(900),
                std::chrono::milliseconds(100),
                cycle_name + " detect_tag")
            && !sweepForTag(arm, tag_frame, tag_tf, cycle_name, goal_handle))
        {
            retryable_failure = true;
            speak("Falha: não encontrei a tag " + spokenTagName(tag_frame));
            last_pick_failure_reason_ = "tag_nao_encontrada";
            return false;
        }

        speak("Identifiquei a tag " + spokenTagName(tag_frame));
        // Pedido do operador 2026-08-15: na PRATELEIRA nao ha pre-aproximacao
        // — o alinhamento XY da camera foi pensado para a vista de cima da
        // mesa; em frente a estante ele so move o braco a toa perto da
        // estrutura. Deteccao e varredura continuam valendo nos dois casos.
        if (!is_shelf) {
            publish_stage(goal_handle, "pre_approach");
            if (!alignCameraToTagXY(
                    arm,
                    tag_frame,
                    goal_handle,
                    cycle_name + " pre_approach_xy")) {
                retryable_failure = true;
                speak("Falha: não consegui alinhar a câmera com a tag");
                last_pick_failure_reason_ = "align_camera_falhou";
                return false;
            }
        }

        if (!sleepInterruptibly(std::chrono::milliseconds(120))) {
            return false;  // cancelamento: nao retentar
        }

        if (!waitForTagTransform(
                "base_footprint",
                tag_frame,
                tag_tf,
                std::chrono::milliseconds(700),
                std::chrono::milliseconds(100),
                cycle_name + " final_approach")) {
            retryable_failure = true;
            speak("Falha: perdi a tag na aproximação");
            last_pick_failure_reason_ = "tag_perdida_na_aproximacao";
            return false;
        }

        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(0.8);

        // PRATELEIRA: sequencia propria em espaco de juntas, com a IK custom.
        // MESA COMUM: caminho cartesiano de sempre, intocado.
        std::array<double, 5> shelf_q{};
        if (is_shelf) {
            if (!approachShelfTarget(arm, tag_frame, cycle_name, goal_handle, shelf_q)) {
                retryable_failure = true;
                speak("Falha: nao consegui alcancar a tag na prateleira");
                last_pick_failure_reason_ = "shelf_ik_ou_execucao_falhou";
                arm->setMaxVelocityScalingFactor(1.0);
                arm->setMaxAccelerationScalingFactor(1.0);
                return false;
            }
        } else {
            const TableIkStrategy strategy = tableStrategyForAttempt(attempt);
            if (strategy == TableIkStrategy::kMoveItPose) {
                // Degrau 3+: caminho antigo do MoveIt/pick_ik, identico ao
                // attempt 3 historico (enforce_pitch_pi era attempt < 3).
                publish_stage(goal_handle, "final_approach");
                //speak("Fazendo a aproximacao final da tag");
                if (!moveToTarget(
                        arm,
                        tag_tf,
                        "tcp",
                        cycle_name + " tcp final",
                        true,
                        false,
                        "Encontrei uma solução de I K para a tag " + spokenTagName(tag_frame),
                        "Falha: sem solução de I K para a tag " + spokenTagName(tag_frame))) {
                    retryable_failure = true;
                    last_pick_failure_reason_ = "ik_ou_execucao_falhou";
                    return false;
                }
            } else if (!approachTableTargetCustomIk(
                    arm, tag_frame, strategy, cycle_name, goal_handle))
            {
                retryable_failure = true;
                last_pick_failure_reason_ = "ik_custom_ou_execucao_falhou";
                arm->setMaxVelocityScalingFactor(1.0);
                arm->setMaxAccelerationScalingFactor(1.0);
                return false;
            }
        }
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(1.0);

        // Depois da fase 2 a ponta esta DENTRO do vao da prateleira. Qualquer
        // saida daqui para frente — falha ou sucesso — tem que desfazer o
        // caminho primeiro: planejar de dentro do vao direto para uma pose
        // nomeada (retry, home, pre_container) arrasta o braco pela estante.
        const auto leave_shelf_if_needed = [&]() {
            if (is_shelf) {
                (void)retreatFromShelf(arm, shelf_q, cycle_name, goal_handle);
            }
        };

        std::optional<GripperEffortSample> effort_before_close;
        if (verify_grasp_effort_) {
            effort_before_close = waitForFreshGripperEffort(
                std::chrono::milliseconds(600));
            if (!effort_before_close) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Cannot verify grasp: gripper effort telemetry is unavailable");
                retryable_failure = true;
                speak("Falha: sem telemetria de esforço da garra");
            last_pick_failure_reason_ = "sem_telemetria_garra";
                leave_shelf_if_needed();
                return false;
            }
        }

        publish_stage(goal_handle, "closing_gripper");
        //speak("Fechando a garra no bloco");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_close");
        if (!planAndExecute(gripper, cycle_name + " close gripper")) {
            retryable_failure = true;
            speak("Falha: não consegui fechar a garra");
            last_pick_failure_reason_ = "garra_nao_fechou";
            // Verificacao adversarial 2026-08-10: o close falha por tolerancia
            // JUSTAMENTE quando o bloco esta entre os dedos — reabrir antes do
            // retry, senao o braco sobe carregando o bloco pincado e o proximo
            // gripper_open derruba o bloco no chassi.
            gripper->setStartStateToCurrentState();
            gripper->setNamedTarget("gripper_open");
            (void)planAndExecute(gripper, cycle_name + " reopen after failed close");
            leave_shelf_if_needed();
            return false;
        }

        if (verify_grasp_effort_) {
            publish_stage(goal_handle, "verifying_grasp");
            speak("Verificando o bloco pela forca da garra");
            if (!verifyGraspByEffort(*effort_before_close, cycle_name)) {
                retryable_failure = true;
                speak("Falha: a garra não detectou o bloco");
            last_pick_failure_reason_ = "garra_vazia";
                RCLCPP_WARN(
                    this->get_logger(),
                    "[%s] opening gripper after failed grasp verification",
                    cycle_name.c_str());
                gripper->setStartStateToCurrentState();
                gripper->setNamedTarget("gripper_open");
                (void)planAndExecute(
                    gripper,
                    cycle_name + " reopen after failed grasp");
                leave_shelf_if_needed();
                return false;
            }
            speak("A garra detectou o bloco");
        }

        // Caminho inverso com o bloco na garra: fase 1 -> pegar_obj_sh.
        if (is_shelf &&
            !retreatFromShelf(arm, shelf_q, cycle_name, goal_handle))
        {
            // O bloco JA esta na garra: nao pode virar retry (o proximo ciclo
            // abriria a garra em cima da prateleira) nem skip.
            object_still_grasped = true;
            retryable_failure = false;
            speak("Falha: nao consegui sair da prateleira com o bloco");
            last_pick_failure_reason_ = "shelf_retorno_falhou";
            return false;
        }

        return runTransferWithRetry(
            arm,
            gripper,
            container_pose,
            cycle_name,
            goal_handle,
            is_shelf,
            object_still_grasped);
    }

    void execute(const std::shared_ptr<GoalHandlePickTag> goal_handle)
    {
        ExecutionGuard execution_guard(*this);
        publishPickActive(true);
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PickTag::Result>();

        const auto finish_failure =
            [this, &goal_handle, &result, &execution_guard](const std::string & message)
            {
                result->success = false;
                result->fail_reason = last_pick_failure_reason_;
                execution_guard.release();
                if (cancellationRequested() || goal_handle->is_canceling()) {
                    result->message = "Pick canceled: " + message;
                    goal_handle->canceled(result);
                } else {
                    result->message = message;
                    goal_handle->abort(result);
                }
            };

        if (cancellationRequested() || goal_handle->is_canceling()) {
            finish_failure("canceled before execution started");
            return;
        }

        //speak("Iniciando a rotina de pegar a tag " + spokenTagName(goal->tag_frame));

        std::string container_pose;
        std::string lookup_error;
        if (!container_state_store_->findFirstEmptyContainer(&container_pose, &lookup_error)) {
            result->success = false;
            result->message =
                "Pick failed: could not resolve an empty container from yaml: " + lookup_error;
            finish_failure(result->message);
            return;
        }
        //speak("Container livre selecionado: " + container_pose);

        auto arm = createArmInterface(false);
        auto gripper = makeInterfaceWithTimeout("gripper");
        if (!arm || !gripper) {
            finish_failure(
                "Pick failed: move_group indisponivel (MoveGroupInterface nao "
                "conectou em 15s) — stack de manipulacao no ar?");
            return;
        }
        setActiveInterfaces(arm, gripper);

        // Item 2.6b: warmup UNICO do monitor de estado no inicio do goal —
        // a 1a leitura paga a assinatura de joint_states; as demais sao quentes.
        if (!arm->getCurrentState(5.0)) {
            finish_failure(
                "Pick failed: sem joint_states em 5s (bringup da Pi no ar?)");
            return;
        }

        gripper->setMaxVelocityScalingFactor(1.0);
        gripper->setMaxAccelerationScalingFactor(1.0);

        publish_stage(goal_handle, "opening_gripper");
        //speak("Abrindo a garra antes de pegar");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, "open gripper")) {
            finish_failure("Pick failed while opening gripper");
            return;
        }

        // 2026-08-12: a prateleira parte de uma pose propria e roda a
        // sequencia de duas fases com a IK custom. Mesa comum = caminho
        // validado de sempre, sem nenhum desvio.
        const bool is_shelf = std::find(
            shelf_table_poses_.begin(), shelf_table_poses_.end(),
            goal->table_pose) != shelf_table_poses_.end();
        const std::string start_pose = is_shelf ? kShelfStartPose : "pegar_obj";
        RCLCPP_INFO(
            this->get_logger(),
            "PICK de %s: table_pose='%s' -> %s (pose de partida: %s)",
            goal->tag_frame.c_str(),
            goal->table_pose.empty() ? "<vazio>" : goal->table_pose.c_str(),
            is_shelf ? "PRATELEIRA" : "mesa comum",
            start_pose.c_str());

        publish_stage(goal_handle, "going_pegar_obj");
        //speak("Indo para a pose inicial de pegar");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget(start_pose);
        if (!planAndExecute(arm, start_pose + " initial")) {
            RCLCPP_WARN(
                this->get_logger(),
                "Initial %s move failed. Retrying from home to avoid residual state between sequential picks.",
                start_pose.c_str());

            arm->setStartStateToCurrentState();
            arm->setNamedTarget("home");
            if (!planAndExecute(arm, "home recovery before " + start_pose)) {
                finish_failure("Pick failed while recovering to home before " + start_pose);
                return;
            }

            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(start_pose);
            if (!planAndExecute(arm, start_pose + " initial after home recovery")) {
                finish_failure("Pick failed while moving to " + start_pose + " after home recovery");
                return;
            }
        }

        // approach_task MTC removido (auditoria 2026-08-07, item 1.3) — o
        // braco ja esta em pegar_obj pelo bloco acima; segue direto ao ciclo.
        if (!sleepInterruptibly(std::chrono::milliseconds(200))) {
            finish_failure("canceled before the pick cycle");
            return;
        }

        const int max_pick_attempts =
            grasp_retry_attempts_ < 0 ? 1 : grasp_retry_attempts_ + 1;
        bool cycle_success = false;
        bool retryable_failure = false;
        bool object_still_grasped = false;

        for (int attempt = 1; attempt <= max_pick_attempts; ++attempt) {
            const std::string cycle_name =
                max_pick_attempts == 1 ?
                "ACTION_CYCLE" :
                "ACTION_CYCLE_ATTEMPT_" + std::to_string(attempt);

            // Item 2.6c: REUTILIZA o MGI entre tentativas — a recriacao nao
            // trocava solver (placebo do 2.7) e custava 1-2s + monitor frio.
            // So os parametros dinamicos do perfil mudam por tentativa.
            applyIkProfile(
                ikProfileForAttempt(attempt), ikProfileNameForAttempt(attempt));
            setActiveInterfaces(arm, gripper);

            if (max_pick_attempts > 1) {
                speak("Tentativa de pegar numero " + std::to_string(attempt));
            }

            cycle_success = run_pick_cycle(
                arm,
                gripper,
                goal->tag_frame,
                container_pose,
                cycle_name,
                goal_handle,
                attempt,
                is_shelf,
                retryable_failure,
                object_still_grasped);

            if (cycle_success) {
                break;
            }

            if (object_still_grasped) {
                break;  // bloco na garra: nunca retentar/pular — abortar
            }

            if (cancellationRequested() || goal_handle->is_canceling()) {
                break;
            }

            if (!retryable_failure || attempt >= max_pick_attempts) {
                break;
            }

            if (attempt < max_pick_attempts) {
                publish_stage(goal_handle, "preparing_next_pick_attempt");
                // Mesa: 1->2 troca a ESTRATEGIA (down -> frente), nao as
                // tolerancias; a frase antiga so vale entrando no degrau
                // MoveIt (3+). Shelf mantem a frase historica.
                if (!is_shelf &&
                    tableStrategyForAttempt(attempt + 1) !=
                        TableIkStrategy::kMoveItPose)
                {
                    speak("Vou tentar de outro jeito na próxima tentativa");
                } else {
                    speak("Vou relaxar as tolerancias para a proxima tentativa");
                }
            }

            //speak("Vou abrir a garra e tentar pegar novamente");
            publish_stage(goal_handle, "retrying_grasp");
            RCLCPP_WARN(
                this->get_logger(),
                "Pick attempt %d/%d falhou (retentavel). Tentando de novo.",
                attempt,
                max_pick_attempts);

            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(start_pose);
            if (!planAndExecute(arm, "return " + start_pose + " before grasp retry")) {
                finish_failure("Pick failed while preparing grasp retry");
                return;
            }

            if (!sleepInterruptibly(std::chrono::milliseconds(100))) {
                finish_failure("canceled before grasp retry");
                return;
            }
        }

        // Item 2.2b (ANTES do cancel generico): bloco preso na garra e a
        // informacao mais importante do desfecho — NAO recolher (estado
        // fisico incerto), NAO pular: abortar com causa explicita para o
        // operador intervir. finish_failure ja resolve canceled vs abort.
        if (object_still_grasped) {
            finish_failure(
                "object_still_grasped: transporte falhou 2x com o bloco na "
                "garra — intervencao manual necessaria.");
            return;
        }

        const bool success = cycle_success;

        // Verificacao adversarial 2026-08-10: gravar o container ANTES de
        // responder um cancel tardio — coleta concluida com cancel chegando
        // na saida deixava o yaml 'empty' e o proximo pick soltava um 2o
        // bloco no mesmo container.
        bool state_write_success = true;
        std::string state_write_error;
        if (success) {
            state_write_success = container_state_store_->setOccupied(
                container_pose,
                goal->tag_frame,
                &state_write_error);
            if (!state_write_success) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to update container state file %s: %s",
                    container_state_file_.c_str(),
                    state_write_error.c_str());
            }
        }

        if (cancellationRequested() || goal_handle->is_canceling()) {
            if (!success) {
                finish_failure("canceled during pick cycle");
                return;
            }
            result->success = true;
            result->message = "Pick completed (cancel recebido apos a coleta)";
            execution_guard.release();
            goal_handle->canceled(result);
            return;
        }

        // Pedido do operador (2026-08-07): apos a falha FINAL, voltar o braco
        // para a pose de transporte (melhor esforco) — a missao nunca segue
        // com o braco em posicao imprevisivel — e anunciar o desfecho.
        if (!cycle_success && arm) {
            RCLCPP_ERROR(
                this->get_logger(),
                "PICK de %s FALHOU em %d tentativa(s). Recolhendo o braco "
                "para %s antes de seguir.",
                goal->tag_frame.c_str(), max_pick_attempts, start_pose.c_str());
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(start_pose);
            (void)planAndExecute(arm, "recolher apos falha final do pick");
        }

        // Item 2.9: sucesso FISICO manda — falha de I/O do yaml vira WARN +
        // flag na mensagem, nunca aborta uma coleta ja realizada.
        result->success = success;
        if (success && state_write_success) {
            result->message = "Pick completed";
        } else if (success && !state_write_success) {
            result->message =
                "Pick completed (AVISO: falha ao gravar container yaml: " +
                state_write_error + ")";
            RCLCPP_WARN(
                this->get_logger(),
                "Pick fisicamente OK mas o yaml de containers nao gravou: %s",
                state_write_error.c_str());
        } else {
            result->message = "Pick failed";
        }

        if (result->success) {
            publish_stage(goal_handle, "done");
            speak("Coleta concluida com sucesso");
            execution_guard.release();
            goal_handle->succeed(result);
        } else if (!cycle_success && skip_failed_pick_after_retries_) {
            publish_stage(goal_handle, "skipped_after_pick_retries");
            result->success = true;
            result->skipped = true;  // item 2.5: contrato explicito
            result->fail_reason = last_pick_failure_reason_;
            result->message = "Pick skipped after all retry attempts failed (" +
                last_pick_failure_reason_ + ")";
            speak("Nao consegui pegar o bloco. Vou seguir para o proximo objetivo");
            execution_guard.release();
            goal_handle->succeed(result);
        } else {
            result->fail_reason = last_pick_failure_reason_;
            speak("Nao consegui pegar o bloco");
            finish_failure(result->message);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PickActionServer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
