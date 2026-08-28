// amt1_sort_action_node (2026-08-28) — rotina AMT1 (Advanced Manipulation
// Task 1): ordenar os cubos com AprilTag da mesa de precision placement em
// ordem crescente de id, da esquerda para a direita do robo, usando os
// containers de bordo como buffer.
//
// Servidor /amt1_sort (my_robot_msgs/Amt1Sort). NAO move o braco para pegar
// ou soltar: e CLIENTE de /pick_tag e /place_tag (os nos MTC existentes) e
// de nudge_base (dock_align_node, deslocamento lateral da base). O unico
// movimento proprio e o de OBSERVACAO (poses nomeadas pegar_obj /
// tag_esquerda / tag_direita via MoveGroupInterface) e a volta a pose de
// transporte — com o flock do braco so durante esses movimentos.
//
// Fluxo: capacidade (containers vazios) -> observa a mesa (TF das tags de 2-3
// poses, a amostra mais perto do eixo optico) -> slots por x
// (amt1_sort_planner::assignSlots) -> frames tag_amt1_slot_k filhos de odom
// (10 Hz, stamp = agora: o place aceita stack_on so' com TF fresco) -> plano
// por ciclos (planSort) -> picks/places com final_attempt=false; alvo visto
// fora de alcance => nudge_base(dy) + re-registro dos slots (delta medio das
// tags ainda na mesa) e repete a op.
//
// Frames: manip_base_link +X = direita do robo, +Y = frente; base_footprint
// +y = esquerda; a base anda dy para a esquerda => o x_manip de um alvo fixo
// vira x + dy.
//
// Politica de terminacao: succeed() sempre que o braco esta na pose de
// transporte (mesmo com success=false — o executor le success/fail_reason);
// canceled() no cancel; abort() SO com o braco em estado desconhecido. Sem
// braco (observe_move_enabled=false, bancada) o estado e sempre conhecido.
//
// Helpers copiados do mtc_place_action_node (makeInterfaceWithTimeout,
// deadlines de plan/execute, sleepInterruptibly, speak,
// declarePlanningDefaults, lock de execucao) SEM refatorar aquele no.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "manip_task_execution/amt1_sort_planner.hpp"
#include "manip_task_execution/container_state_store.hpp"
#include "manip_task_execution/custom_ik.hpp"
#include "manip_task_execution/manipulator_execution_lock.hpp"

#include "caramelo_msgs/action/nudge_base.hpp"
#include "my_robot_msgs/action/amt1_sort.hpp"
#include "my_robot_msgs/action/pick_tag.hpp"
#include "my_robot_msgs/action/place_tag.hpp"

namespace amt1 = manip_task_execution::amt1;

class Amt1SortActionServer : public rclcpp::Node
{
public:
    using Amt1Sort = my_robot_msgs::action::Amt1Sort;
    using GoalHandleAmt1Sort = rclcpp_action::ServerGoalHandle<Amt1Sort>;
    using PickTag = my_robot_msgs::action::PickTag;
    using PlaceTag = my_robot_msgs::action::PlaceTag;
    using NudgeBase = caramelo_msgs::action::NudgeBase;
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    Amt1SortActionServer()
        : Node("amt1_sort_action_server")
    {
        const auto lock_file = this->declare_parameter<std::string>(
            "manipulator_lock_file", "/tmp/caramelo_manip_action.lock");
        execution_lock_ =
            std::make_unique<manip_task_execution::ManipulatorExecutionLock>(lock_file);
        if (!execution_lock_->valid()) {
            throw std::runtime_error(
                "Failed to open manipulator lock file '" + lock_file + "': " +
                execution_lock_->error());
        }

        container_state_file_ = this->declare_parameter<std::string>(
            "container_state_file", getDefaultContainerStatePath());
        container_state_store_ =
            std::make_unique<manip_task_execution::ContainerStateStore>(container_state_file_);
        container_names_ = this->declare_parameter<std::vector<std::string>>(
            "container_names", std::vector<std::string>{"container1", "container2", "container3"});

        // ---- Observacao ----
        // false = bancada sem MoveIt: nao constroi o MoveGroupInterface e le
        // as TFs parado (as tags precisam estar no TF de qualquer forma).
        observe_move_enabled_ = this->declare_parameter<bool>("observe_move_enabled", true);
        observe_poses_ = this->declare_parameter<std::vector<std::string>>(
            "observe_poses", std::vector<std::string>{"pegar_obj", "tag_esquerda", "tag_direita"});
        reobserve_poses_ = this->declare_parameter<std::vector<std::string>>(
            "reobserve_poses", std::vector<std::string>{"tag_esquerda", "tag_direita"});
        observe_dwell_s_ = this->declare_parameter<double>("observe_dwell_s", 1.5);
        // true = para de visitar poses assim que todas as esperadas foram vistas.
        observe_stop_early_ = this->declare_parameter<bool>("observe_stop_early", false);
        transport_pose_ = this->declare_parameter<std::string>("transport_pose", "pegar_obj");
        std::vector<std::string> default_frames;
        for (int id = 0; id <= 30; ++id) {
            default_frames.push_back("tag_" + std::to_string(id));
        }
        default_frames.push_back("tag_42");
        tag_frames_ = this->declare_parameter<std::vector<std::string>>("tag_frames", default_frames);
        anchor_frame_ = this->declare_parameter<std::string>("anchor_frame", "odom");
        ik_reference_frame_ = this->declare_parameter<std::string>(
            "ik_reference_frame", "manip_base_link");
        camera_optical_frame_ = this->declare_parameter<std::string>(
            "camera_optical_frame", "camera_color_optical_frame");
        max_tag_age_sec_ = this->declare_parameter<double>("max_tag_age_sec", 1.0);
        observe_min_radial_m_ = this->declare_parameter<double>("observe_min_radial_m", 0.10);
        observe_max_dz_m_ = this->declare_parameter<double>("observe_max_dz_m", 0.15);
        slot_tie_eps_m_ = this->declare_parameter<double>("slot_tie_eps_m", 0.01);
        slot_min_spacing_m_ = this->declare_parameter<double>("slot_min_spacing_m", 0.04);
        // Tampo = z da tag - altura do cubo (o place soma stack_place_z_offset).
        cube_height_m_ = this->declare_parameter<double>("cube_height_m", 0.042);
        slot_frame_prefix_ = this->declare_parameter<std::string>(
            "slot_frame_prefix", "tag_amt1_slot_");
        broadcast_rate_hz_ = this->declare_parameter<double>("broadcast_rate_hz", 10.0);
        min_seen_tags_ = this->declare_parameter<int>("min_seen_tags", 2);
        allow_partial_ = this->declare_parameter<bool>("allow_partial", true);

        // ---- Alcance / nudge ----
        nudge_enabled_ = this->declare_parameter<bool>("nudge_enabled", true);
        max_nudges_ = this->declare_parameter<int>("max_nudges", 6);
        min_shift_m_ = this->declare_parameter<double>("min_shift_m", 0.10);
        max_shift_m_ = this->declare_parameter<double>("max_shift_m", 0.25);
        max_total_shift_m_ = this->declare_parameter<double>("max_total_shift_m", 0.35);
        nudge_timeout_s_ = this->declare_parameter<double>("nudge_timeout_s", 20.0);
        reregister_after_nudge_ = this->declare_parameter<bool>("reregister_after_nudge", true);
        reregister_dwell_s_ = this->declare_parameter<double>("reregister_dwell_s", 1.0);
        reregister_min_anchors_ = this->declare_parameter<int>("reregister_min_anchors", 2);
        reregister_max_spread_m_ = this->declare_parameter<double>("reregister_max_spread_m", 0.02);
        retry_not_seen_ = this->declare_parameter<int>("retry_not_seen", 1);
        op_timeout_s_ = this->declare_parameter<double>("op_timeout_s", 300.0);
        heartbeat_s_ = this->declare_parameter<double>("heartbeat_s", 5.0);
        server_wait_s_ = this->declare_parameter<double>("server_wait_s", 10.0);
        lock_wait_s_ = this->declare_parameter<double>("lock_wait_s", 10.0);
        speech_enabled_ = this->declare_parameter<bool>("speech_enabled", true);

        declarePlanningDefaults();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        speech_publisher_ = this->create_publisher<std_msgs::msg::String>("/manip/speech", 10);
        active_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/manip/amt1_active", rclcpp::QoS(1).transient_local().reliable());
        publishActive(false);

        base_shift_total_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/manip/base_shift_total",
            rclcpp::QoS(1).transient_local().reliable(),
            [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
                base_shift_total_.store(msg->data, std::memory_order_relaxed);
            });

        pick_client_ = rclcpp_action::create_client<PickTag>(this, "/pick_tag");
        place_client_ = rclcpp_action::create_client<PlaceTag>(this, "/place_tag");
        // Nome RELATIVO de proposito, como no dock_align_node / NudgeBaseBT.
        nudge_client_ = rclcpp_action::create_client<NudgeBase>(this, "nudge_base");

        const double rate = std::max(1.0, broadcast_rate_hz_);
        broadcast_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / rate)),
            std::bind(&Amt1SortActionServer::onBroadcastTimer, this));

        action_server_ = rclcpp_action::create_server<Amt1Sort>(
            this, "/amt1_sort",
            std::bind(&Amt1SortActionServer::handle_goal, this,
                std::placeholders::_1, std::placeholders::_2),
            std::bind(&Amt1SortActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&Amt1SortActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "[AMT1] /amt1_sort no ar: observe_move=%s poses=[%s] anchor=%s slots=%s* "
            "nudge=%s (max %d, total <= %.2f m) tag_frames=%zu",
            observe_move_enabled_ ? "true" : "false", join(observe_poses_).c_str(),
            anchor_frame_.c_str(), slot_frame_prefix_.c_str(),
            nudge_enabled_ ? "true" : "false", max_nudges_, max_total_shift_m_,
            tag_frames_.size());
    }

private:
    // ------------------------------------------------------------ tipos
    /// Tag vista na mesa durante a observacao (melhor amostra por id).
    struct TagSample
    {
        int id{0};
        std::string frame;
        Eigen::Vector3d p_manip{0.0, 0.0, 0.0};   ///< ik_reference_frame
        double yaw_manip{0.0};
        geometry_msgs::msg::Transform tf_anchor;  ///< anchor_frame -> tag
        double axis_dist{1e9};                    ///< distancia ao eixo optico (m)
        rclcpp::Time stamp;
        std::string pose_name;
    };

    /// Slot registrado: frame filho de anchor_frame, difundido a 10 Hz.
    struct Slot
    {
        int index{0};
        std::string frame;
        int initial_tag{0};
        double x_manip{0.0};   ///< x no frame do braco na observacao (+ nudges)
        double y_manip{0.0};
        geometry_msgs::msg::Transform tf_anchor;  ///< anchor -> slot (z do tampo, so yaw)
    };

    enum class ChildOutcome
    {
        kOk,
        kSkippedUnreachable,
        kSkippedNotSeen,
        kSkippedOther,
        kRejected,
        kAborted,
        kCanceled,
        kTimeout,
        kServerMissing,
    };

    struct ChildResult
    {
        ChildOutcome outcome{ChildOutcome::kAborted};
        std::string fail_reason;
        std::string message;
        double suggested_shift_m{0.0};
    };

    enum class ObserveOutcome { kOk, kCanceled, kArmFailed, kLockBusy };

    /// Estado de UM goal (zerado em execute()).
    struct GoalState
    {
        std::shared_ptr<GoalHandleAmt1Sort> handle;
        std::string ws;
        std::string table_pose;
        std::vector<int> expected;
        bool left_to_right{true};
        bool observe_only{false};
        int buffer{0};

        std::map<int, TagSample> samples;
        amt1::SlotAssignment assignment;
        std::vector<int> table;      ///< tag por slot (0 = livre)
        std::vector<int> onboard;    ///< tags a bordo (ordem de coleta)
        std::set<int> touched;       ///< tags que o robo ja pegou (o slot deixa de ser ancora original)
        bool unknown{false};         ///< braco em estado desconhecido (limpa ao voltar a transport_pose)
        /// pick/place abortou ou estourou o prazo: o cubo pode estar na garra
        /// => nunca chamar recoverOnboard (o place abriria a garra em pegar_obj).
        bool child_aborted{false};
        std::string child_fail_reason;  ///< fail_reason do filho que abortou/estourou
        int picks{0};
        int places{0};
        int nudges{0};
        double total_shift_m{0.0};
        int ops_done{0};
        int ops_total{0};
        bool nudge_available{false};
        std::string fail_reason;
        std::string message;
    };

    // ------------------------------------------------------------ membros
    rclcpp_action::Server<Amt1Sort>::SharedPtr action_server_;
    rclcpp_action::Client<PickTag>::SharedPtr pick_client_;
    rclcpp_action::Client<PlaceTag>::SharedPtr place_client_;
    rclcpp_action::Client<NudgeBase>::SharedPtr nudge_client_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr broadcast_timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr base_shift_total_subscription_;
    std::atomic<double> base_shift_total_{0.0};

    std::unique_ptr<manip_task_execution::ManipulatorExecutionLock> execution_lock_;
    std::string container_state_file_;
    std::unique_ptr<manip_task_execution::ContainerStateStore> container_state_store_;
    std::vector<std::string> container_names_;

    bool observe_move_enabled_{true};
    std::vector<std::string> observe_poses_;
    std::vector<std::string> reobserve_poses_;
    double observe_dwell_s_{1.5};
    bool observe_stop_early_{false};
    std::string transport_pose_;
    std::vector<std::string> tag_frames_;
    std::string anchor_frame_;
    std::string ik_reference_frame_;
    std::string camera_optical_frame_;
    double max_tag_age_sec_{1.0};
    double observe_min_radial_m_{0.10};
    double observe_max_dz_m_{0.15};
    double slot_tie_eps_m_{0.01};
    double slot_min_spacing_m_{0.04};
    double cube_height_m_{0.042};
    std::string slot_frame_prefix_;
    double broadcast_rate_hz_{10.0};
    int min_seen_tags_{2};
    bool allow_partial_{true};

    bool nudge_enabled_{true};
    int max_nudges_{6};
    double min_shift_m_{0.10};
    double max_shift_m_{0.25};
    double max_total_shift_m_{0.35};
    double nudge_timeout_s_{20.0};
    bool reregister_after_nudge_{true};
    double reregister_dwell_s_{1.0};
    int reregister_min_anchors_{2};
    double reregister_max_spread_m_{0.02};
    int retry_not_seen_{1};
    double op_timeout_s_{300.0};
    double heartbeat_s_{5.0};
    double server_wait_s_{10.0};
    double lock_wait_s_{10.0};
    bool speech_enabled_{true};

    // Slots difundidos pelo timer (thread do executor) e editados pela
    // thread do goal.
    std::mutex slots_mutex_;
    std::vector<Slot> slots_;
    std::atomic_bool broadcasting_{false};

    std::atomic_bool goal_active_{false};
    std::atomic_bool cancel_requested_{false};
    std::mutex active_interfaces_mutex_;
    std::shared_ptr<MoveGroupInterface> active_arm_;
    std::mutex child_cancel_mutex_;
    std::function<void()> child_cancel_fn_;

    // Feedback/heartbeat.
    std::mutex feedback_mutex_;
    std::string current_stage_;
    std::chrono::steady_clock::time_point last_feedback_;

    // Estado do braco durante o goal (thread do goal).
    bool arm_state_known_{true};
    bool arm_at_transport_{false};
    bool lock_held_{false};

    // ------------------------------------------------------------ utilidades
    static std::string getDefaultContainerStatePath()
    {
        const char * home = std::getenv("HOME");
        if (home != nullptr && home[0] != '\0') {
            return std::string(home) + "/.config/caramelo/container_states.yaml";
        }
        return "container_states.yaml";
    }

    static std::string join(const std::vector<std::string> & items, const std::string & sep = ", ")
    {
        std::string out;
        for (std::size_t i = 0; i < items.size(); ++i) {
            if (i > 0) {out += sep;}
            out += items[i];
        }
        return out;
    }

    static std::string joinInts(const std::vector<int> & items, const std::string & sep = ", ")
    {
        std::string out;
        for (std::size_t i = 0; i < items.size(); ++i) {
            if (i > 0) {out += sep;}
            out += std::to_string(items[i]);
        }
        return out;
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

    static std::string toLower(std::string s)
    {
        for (char & c : s) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        return s;
    }

    /// "tag_12" -> 12; -1 se nao for tag_<numero>.
    static int parseTagId(const std::string & frame)
    {
        constexpr char prefix[] = "tag_";
        if (frame.rfind(prefix, 0) != 0 || frame.size() <= sizeof(prefix) - 1) {
            return -1;
        }
        const std::string digits = frame.substr(sizeof(prefix) - 1);
        for (const char c : digits) {
            if (!std::isdigit(static_cast<unsigned char>(c))) {
                return -1;
            }
        }
        return std::atoi(digits.c_str());
    }

    static std::string tagFrameForId(int id)
    {
        return "tag_" + std::to_string(id);
    }

    bool cancellationRequested() const
    {
        return cancel_requested_.load();
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

    void publishActive(bool active)
    {
        std_msgs::msg::Bool message;
        message.data = active;
        active_publisher_->publish(message);
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

    // ------------------------------------------------------------ feedback
    void publishStage(GoalState & gs, const std::string & stage)
    {
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            current_stage_ = stage;
            last_feedback_ = std::chrono::steady_clock::now();
        }
        auto feedback = std::make_shared<Amt1Sort::Feedback>();
        feedback->current_stage = stage;
        feedback->ops_done = gs.ops_done;
        feedback->ops_total = gs.ops_total;
        gs.handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "[AMT1] stage=%s (%d/%d)", stage.c_str(), gs.ops_done, gs.ops_total);
    }

    /// Republica o estagio atual se o ultimo feedback ja tem heartbeat_s.
    void heartbeat(GoalState & gs)
    {
        std::string stage;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_feedback_).count() <
                std::max(0.5, heartbeat_s_) * 0.8)
            {
                return;
            }
            last_feedback_ = now;
            stage = current_stage_;
        }
        auto feedback = std::make_shared<Amt1Sort::Feedback>();
        feedback->current_stage = stage;
        feedback->ops_done = gs.ops_done;
        feedback->ops_total = gs.ops_total;
        gs.handle->publish_feedback(feedback);
    }

    /// Feedback de um goal filho vira estagio "<op>:<estagio do filho>".
    void onChildStage(GoalState & gs, const std::string & label, const std::string & child_stage)
    {
        publishStage(gs, label + ":" + child_stage);
    }

    // ------------------------------------------------------------ MoveIt (copiado do place)
    void declarePlanningDefaults()
    {
        if (!this->has_parameter("ompl.planning_plugins")) {
            this->declare_parameter<std::vector<std::string>>(
                "ompl.planning_plugins",
                std::vector<std::string>{"ompl_interface/OMPLPlanner"});
        }
        if (!this->has_parameter("ompl.planning_plugin")) {
            this->declare_parameter<std::string>(
                "ompl.planning_plugin", "ompl_interface/OMPLPlanner");
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
                "robot_description_kinematics.arm.kinematics_solver", "pick_ik/PickIkPlugin");
        }
        if (!this->has_parameter("robot_description_kinematics.arm.kinematics_solver_timeout")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.kinematics_solver_timeout", 0.2);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.mode")) {
            this->declare_parameter<std::string>("robot_description_kinematics.arm.mode", "global");
        }
        if (!this->has_parameter("robot_description_kinematics.arm.position_scale")) {
            this->declare_parameter<double>("robot_description_kinematics.arm.position_scale", 1.0);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.rotation_scale")) {
            this->declare_parameter<double>("robot_description_kinematics.arm.rotation_scale", 0.03);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.position_threshold")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.position_threshold", 0.001);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.orientation_threshold")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.orientation_threshold", 0.08);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.cost_threshold")) {
            this->declare_parameter<double>("robot_description_kinematics.arm.cost_threshold", 0.001);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.minimal_displacement_weight")) {
            this->declare_parameter<double>(
                "robot_description_kinematics.arm.minimal_displacement_weight", 0.02);
        }
        if (!this->has_parameter("robot_description_kinematics.arm.gd_step_size")) {
            this->declare_parameter<double>("robot_description_kinematics.arm.gd_step_size", 0.0008);
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
            planning_plugins.empty() || planning_plugins.front().empty())
        {
            this->set_parameter(rclcpp::Parameter(
                "ompl.planning_plugins", std::vector<std::string>{"ompl_interface/OMPLPlanner"}));
        }
        std::string planning_plugin;
        if (!this->get_parameter("ompl.planning_plugin", planning_plugin) || planning_plugin.empty()) {
            this->set_parameter(rclcpp::Parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner"));
        }
        std::string kinematics_solver;
        if (!this->get_parameter("robot_description_kinematics.arm.kinematics_solver", kinematics_solver) ||
            kinematics_solver.empty())
        {
            this->set_parameter(rclcpp::Parameter(
                "robot_description_kinematics.arm.kinematics_solver", "pick_ik/PickIkPlugin"));
        }
        if (!this->get_parameter("arm.kinematics_solver", kinematics_solver) || kinematics_solver.empty()) {
            this->set_parameter(rclcpp::Parameter("arm.kinematics_solver", "pick_ik/PickIkPlugin"));
        }
    }

    // Sem prazo, o construtor do MoveGroupInterface espera o move_group PARA
    // SEMPRE; e sem move_group no ar ele aborta o processo (SIGABRT em thread
    // interna). Sonda antes, 15 s folgados.
    std::shared_ptr<MoveGroupInterface> makeInterfaceWithTimeout(const std::string & group)
    {
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
                "MoveGroupInterface('" << group << "') indisponivel em 15s: " << e.what());
            return nullptr;
        }
    }

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
        if (fut.wait_for(std::chrono::duration<double>(timeout_s)) == std::future_status::ready) {
            return fut.get();
        }
        RCLCPP_FATAL_STREAM(
            get_logger(),
            "DEADLINE de " << timeout_s << "s estourado em '" << label
                << "' (ver o log do move_group: controlador lento ou chamada pendurada).");
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
                return iface->plan(*plan_box) == moveit::core::MoveItErrorCode::SUCCESS;
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
        const double timeout_s = plannedTrajectoryDurationSec(plan) * 3.0 + 3.0 + 10.0;
        auto plan_box = std::make_shared<MoveGroupInterface::Plan>(plan);
        const bool ok = runWithDeadline(
            [iface, plan_box]() {
                return iface->execute(*plan_box) == moveit::core::MoveItErrorCode::SUCCESS;
            },
            timeout_s, label + " [execute]");
        if (!ok) {
            iface->stop();
        }
        return ok;
    }

    bool planAndExecute(const std::shared_ptr<MoveGroupInterface> & iface, const std::string & label)
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

    bool goToNamedPose(
        const std::shared_ptr<MoveGroupInterface> & arm, const std::string & pose,
        const std::string & label)
    {
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget(pose);
        return planAndExecute(arm, label);
    }

    void setActiveArm(const std::shared_ptr<MoveGroupInterface> & arm)
    {
        std::lock_guard<std::mutex> lock(active_interfaces_mutex_);
        active_arm_ = arm;
    }

    std::shared_ptr<MoveGroupInterface> activeArm()
    {
        std::lock_guard<std::mutex> lock(active_interfaces_mutex_);
        return active_arm_;
    }

    void stopActiveMotion()
    {
        auto arm = activeArm();
        if (arm) {
            arm->stop();
        }
    }

    // ------------------------------------------------------------ lock do braco
    /// Segura o flock so enquanto ESTE no move o braco (observacao/volta a
    /// transporte); solta antes de cada goal filho — pick/place o adquirem.
    bool acquireLock()
    {
        if (lock_held_) {
            return true;
        }
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(std::max(0.0, lock_wait_s_)));
        while (true) {
            if (execution_lock_->tryAcquire()) {
                lock_held_ = true;
                return true;
            }
            if (cancellationRequested() || std::chrono::steady_clock::now() >= deadline) {
                RCLCPP_ERROR(
                    get_logger(),
                    "[AMT1] lock do braco (%s) ocupado por %.0f s — outro pick/place em curso?",
                    execution_lock_->lockFile().c_str(), lock_wait_s_);
                return false;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
    }

    void releaseLock()
    {
        if (lock_held_) {
            execution_lock_->release();
            lock_held_ = false;
        }
    }

    /// MoveGroupInterface do braco (criado uma vez por goal, sob demanda).
    std::shared_ptr<MoveGroupInterface> ensureArm()
    {
        auto arm = activeArm();
        if (arm) {
            return arm;
        }
        arm = makeInterfaceWithTimeout("arm");
        if (!arm) {
            return nullptr;
        }
        arm->setPlanningTime(15.0);
        arm->setNumPlanningAttempts(20);
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(1.0);
        setActiveArm(arm);
        return arm;
    }

    // ------------------------------------------------------------ difusao dos slots
    void onBroadcastTimer()
    {
        if (!broadcasting_.load()) {
            return;
        }
        std::vector<geometry_msgs::msg::TransformStamped> msgs;
        {
            std::lock_guard<std::mutex> lock(slots_mutex_);
            if (slots_.empty()) {
                return;
            }
            const rclcpp::Time now = this->get_clock()->now();
            msgs.reserve(slots_.size());
            for (const Slot & slot : slots_) {
                geometry_msgs::msg::TransformStamped msg;
                msg.header.stamp = now;
                msg.header.frame_id = anchor_frame_;
                msg.child_frame_id = slot.frame;
                msg.transform = slot.tf_anchor;
                msgs.push_back(msg);
            }
        }
        tf_broadcaster_->sendTransform(msgs);
    }

    std::vector<Slot> slotsSnapshot()
    {
        std::lock_guard<std::mutex> lock(slots_mutex_);
        return slots_;
    }

    // ------------------------------------------------------------ observacao
    /// Le as TFs das tags por `dwell_s` (sem mover nada). Amostra por id: a
    /// mais perto do eixo optico (empate: a mais recente). Filtros: frescor
    /// (max_tag_age_sec), `not_before` (chegada do braco - 0,3 s), y radial
    /// minimo. Devolve false so em cancelamento.
    bool dwellObserve(
        GoalState & gs, double dwell_s, const std::optional<rclcpp::Time> & not_before,
        const std::string & pose_name)
    {
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(std::max(0.1, dwell_s)));
        std::set<std::string> anchor_warned;
        while (true) {
            const rclcpp::Time now = this->get_clock()->now();
            for (const std::string & frame : tag_frames_) {
                const int id = parseTagId(frame);
                if (id < 0) {
                    continue;
                }
                // _frameExists antes do canTransform: o tf2 avisa "Invalid
                // frame ID" para cada frame que nunca entrou no buffer.
                if (!tf_buffer_->_frameExists(frame) ||
                    !tf_buffer_->canTransform(ik_reference_frame_, frame, tf2::TimePointZero))
                {
                    continue;
                }
                geometry_msgs::msg::TransformStamped tf;
                try {
                    tf = tf_buffer_->lookupTransform(ik_reference_frame_, frame, tf2::TimePointZero);
                } catch (const tf2::TransformException &) {
                    continue;
                }
                const rclcpp::Time stamp(tf.header.stamp);
                const double age = (now - stamp).seconds();
                if (age > max_tag_age_sec_) {
                    continue;  // tag velha no buffer = fantasma
                }
                if (not_before && stamp < (*not_before - rclcpp::Duration::from_seconds(0.3))) {
                    continue;
                }
                const Eigen::Vector3d p(
                    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
                if (p.y() < observe_min_radial_m_) {
                    continue;  // encostado no eixo do braco / atras: nao e mesa
                }

                double axis_dist = 1e9;
                try {
                    const auto cam = tf_buffer_->lookupTransform(
                        camera_optical_frame_, frame, tf2::TimePointZero);
                    axis_dist = std::hypot(cam.transform.translation.x, cam.transform.translation.y);
                } catch (const tf2::TransformException &) {
                    // sem frame optico (bancada): fica a amostra mais recente
                }

                geometry_msgs::msg::TransformStamped anchor_tf;
                try {
                    anchor_tf = tf_buffer_->lookupTransform(anchor_frame_, frame, tf2::TimePointZero);
                } catch (const tf2::TransformException & ex) {
                    if (anchor_warned.insert(frame).second) {
                        RCLCPP_WARN(
                            get_logger(),
                            "[AMT1] %s vista mas sem TF %s -> %s (%s): ignorada.",
                            frame.c_str(), anchor_frame_.c_str(), frame.c_str(), ex.what());
                    }
                    continue;
                }

                auto it = gs.samples.find(id);
                if (it != gs.samples.end()) {
                    const TagSample & cur = it->second;
                    const bool better =
                        axis_dist < cur.axis_dist - 1e-6 ||
                        (std::abs(axis_dist - cur.axis_dist) <= 1e-6 && stamp > cur.stamp);
                    if (!better) {
                        continue;
                    }
                }
                TagSample s;
                s.id = id;
                s.frame = frame;
                s.p_manip = p;
                s.yaw_manip = manip_task_execution::projectedFrameYaw(
                    Eigen::Quaterniond(
                        tf.transform.rotation.w, tf.transform.rotation.x,
                        tf.transform.rotation.y, tf.transform.rotation.z).toRotationMatrix());
                s.tf_anchor = anchor_tf.transform;
                s.axis_dist = axis_dist;
                s.stamp = stamp;
                s.pose_name = pose_name;
                gs.samples[id] = s;
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

    bool allExpectedSeen(const GoalState & gs) const
    {
        if (gs.expected.empty()) {
            return false;
        }
        for (const int id : gs.expected) {
            if (gs.samples.count(id) == 0) {
                return false;
            }
        }
        return true;
    }

    /// Observa a mesa das poses dadas (move o braco se observe_move_enabled)
    /// e volta a transport_pose. Sem braco: uma leitura parada.
    ObserveOutcome observeTable(
        GoalState & gs, const std::vector<std::string> & poses, double dwell_s,
        const std::string & stage_prefix)
    {
        if (!observe_move_enabled_) {
            publishStage(gs, stage_prefix + "_static");
            if (!dwellObserve(gs, dwell_s, std::nullopt, "static")) {
                return ObserveOutcome::kCanceled;
            }
            return ObserveOutcome::kOk;
        }

        if (!acquireLock()) {
            return cancellationRequested() ? ObserveOutcome::kCanceled : ObserveOutcome::kLockBusy;
        }
        auto arm = ensureArm();
        if (!arm) {
            releaseLock();
            return ObserveOutcome::kArmFailed;
        }
        for (const std::string & pose : poses) {
            if (cancellationRequested()) {
                releaseLock();
                return ObserveOutcome::kCanceled;
            }
            publishStage(gs, stage_prefix + "_" + pose);
            arm_at_transport_ = false;
            if (!goToNamedPose(arm, pose, "observe go " + pose)) {
                arm_state_known_ = false;
                releaseLock();
                return cancellationRequested() ? ObserveOutcome::kCanceled : ObserveOutcome::kArmFailed;
            }
            const rclcpp::Time arrival = this->get_clock()->now();
            if (!dwellObserve(gs, dwell_s, arrival, pose)) {
                releaseLock();
                return ObserveOutcome::kCanceled;
            }
            if (observe_stop_early_ && allExpectedSeen(gs)) {
                RCLCPP_INFO(get_logger(), "[AMT1] todas as tags esperadas vistas em %s — parando cedo.", pose.c_str());
                break;
            }
        }
        publishStage(gs, stage_prefix + "_return_" + transport_pose_);
        if (!goToNamedPose(arm, transport_pose_, "observe return " + transport_pose_)) {
            arm_state_known_ = false;
            releaseLock();
            return cancellationRequested() ? ObserveOutcome::kCanceled : ObserveOutcome::kArmFailed;
        }
        arm_at_transport_ = true;
        arm_state_known_ = true;
        releaseLock();
        return ObserveOutcome::kOk;
    }

    /// Descarta amostras fora do nivel da mesa (|z - mediana| > observe_max_dz_m).
    void filterSamplesByHeight(GoalState & gs)
    {
        if (gs.samples.size() < 3) {
            return;
        }
        std::vector<double> zs;
        for (const auto & kv : gs.samples) {
            zs.push_back(kv.second.p_manip.z());
        }
        std::sort(zs.begin(), zs.end());
        const double median = zs[zs.size() / 2];
        for (auto it = gs.samples.begin(); it != gs.samples.end();) {
            const double dz = it->second.p_manip.z() - median;
            if (std::abs(dz) > observe_max_dz_m_) {
                RCLCPP_WARN(
                    get_logger(),
                    "[AMT1] %s a %.3f m do nivel da mesa (z=%.3f, mediana %.3f): ignorada.",
                    it->second.frame.c_str(), dz, it->second.p_manip.z(), median);
                it = gs.samples.erase(it);
            } else {
                ++it;
            }
        }
    }

    // ------------------------------------------------------------ slots
    static geometry_msgs::msg::Quaternion yawOnlyQuaternion(const geometry_msgs::msg::Quaternion & q_in)
    {
        const double yaw = manip_task_execution::projectedFrameYaw(
            Eigen::Quaterniond(q_in.w, q_in.x, q_in.y, q_in.z).toRotationMatrix());
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        geometry_msgs::msg::Quaternion out;
        out.x = q.x();
        out.y = q.y();
        out.z = q.z();
        out.w = q.w();
        return out;
    }

    /// Monta os slots a partir da atribuicao e liga a difusao.
    void buildSlots(GoalState & gs)
    {
        std::vector<Slot> slots;
        for (std::size_t k = 0; k < gs.assignment.slots.size(); ++k) {
            const amt1::TagPose & tp = gs.assignment.slots[k];
            const TagSample & s = gs.samples.at(tp.id);
            Slot slot;
            slot.index = static_cast<int>(k);
            slot.frame = slot_frame_prefix_ + std::to_string(k);
            slot.initial_tag = tp.id;
            slot.x_manip = tp.x;
            slot.y_manip = tp.y;
            slot.tf_anchor = s.tf_anchor;
            slot.tf_anchor.translation.z -= cube_height_m_;
            slot.tf_anchor.rotation = yawOnlyQuaternion(s.tf_anchor.rotation);
            slots.push_back(slot);
            RCLCPP_INFO(
                get_logger(),
                "[AMT1] slot %zu = %s <- tag_%d  manip(x=%.3f y=%.3f z=%.3f)  %s(x=%.3f y=%.3f z=%.3f) vista de %s (eixo %.3f m)",
                k, slot.frame.c_str(), tp.id, tp.x, tp.y, tp.z, anchor_frame_.c_str(),
                slot.tf_anchor.translation.x, slot.tf_anchor.translation.y,
                slot.tf_anchor.translation.z, s.pose_name.c_str(),
                s.axis_dist < 1e8 ? s.axis_dist : -1.0);
            if (k > 0) {
                const double dx = std::abs(tp.x - gs.assignment.slots[k - 1].x);
                if (dx < slot_min_spacing_m_) {
                    RCLCPP_WARN(
                        get_logger(),
                        "[AMT1] slots %zu e %zu a %.1f cm em x (< %.1f cm): cubos encostados, "
                        "a chegada de 3 cm pode empurrar o vizinho.",
                        k - 1, k, dx * 100.0, slot_min_spacing_m_ * 100.0);
                }
            }
        }
        {
            std::lock_guard<std::mutex> lock(slots_mutex_);
            slots_ = slots;
        }
        broadcasting_.store(true);
        onBroadcastTimer();  // primeira difusao sem esperar o timer
    }

    void stopBroadcasting()
    {
        broadcasting_.store(false);
    }

    std::string slotFrame(int k) const
    {
        return slot_frame_prefix_ + std::to_string(k);
    }

    // ------------------------------------------------------------ goals filhos
    template<typename ActionT>
    struct ChildRun
    {
        bool rejected{false};
        bool timed_out{false};
        bool server_missing{false};
        rclcpp_action::ResultCode code{rclcpp_action::ResultCode::UNKNOWN};
        typename ActionT::Result::SharedPtr result;
    };

    static std::string childStage(const PickTag::Feedback & fb) {return fb.current_stage;}
    static std::string childStage(const PlaceTag::Feedback & fb) {return fb.current_stage;}
    static std::string childStage(const NudgeBase::Feedback & fb)
    {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "travelled_%+.3f", static_cast<double>(fb.travelled));
        return buf;
    }

    void setChildCancel(std::function<void()> fn)
    {
        std::lock_guard<std::mutex> lock(child_cancel_mutex_);
        child_cancel_fn_ = std::move(fn);
    }

    void cancelChildGoal()
    {
        std::function<void()> fn;
        {
            std::lock_guard<std::mutex> lock(child_cancel_mutex_);
            fn = child_cancel_fn_;
        }
        if (fn) {
            fn();
        }
    }

    /// Envia um goal filho e espera o resultado com heartbeat, cancel
    /// propagado e prazo. O executor (thread principal) entrega as respostas.
    template<typename ActionT>
    ChildRun<ActionT> runChild(
        GoalState & gs,
        const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
        const typename ActionT::Goal & goal,
        const std::string & label,
        double timeout_s)
    {
        using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;
        ChildRun<ActionT> run;

        if (!client->action_server_is_ready() &&
            !client->wait_for_action_server(std::chrono::duration<double>(std::max(1.0, server_wait_s_))))
        {
            run.server_missing = true;
            return run;
        }

        typename rclcpp_action::Client<ActionT>::SendGoalOptions opts;
        // O feedback do filho chega na thread do executor; `alive` protege
        // contra um callback atrasado depois de esta funcao devolver.
        GoalState * gs_ptr = &gs;
        auto alive = std::make_shared<std::atomic_bool>(true);
        opts.feedback_callback =
            [this, gs_ptr, label, alive](
                typename GoalHandleT::SharedPtr,
                const std::shared_ptr<const typename ActionT::Feedback> fb) {
                if (alive->load()) {
                    onChildStage(*gs_ptr, label, childStage(*fb));
                }
            };
        struct AliveGuard
        {
            std::shared_ptr<std::atomic_bool> flag;
            ~AliveGuard() {flag->store(false);}
        } alive_guard{alive};
        auto goal_future = client->async_send_goal(goal, opts);

        const auto accept_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        while (goal_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
            heartbeat(gs);
            if (std::chrono::steady_clock::now() > accept_deadline) {
                RCLCPP_ERROR(get_logger(), "[AMT1] %s: servidor nao respondeu ao goal em 15 s.", label.c_str());
                run.timed_out = true;
                return run;
            }
        }
        typename GoalHandleT::SharedPtr handle = goal_future.get();
        if (!handle) {
            run.rejected = true;
            return run;
        }
        setChildCancel(
            [client, handle]() {
                try {
                    client->async_cancel_goal(handle);
                } catch (const std::exception &) {
                }
            });

        auto result_future = client->async_get_result(handle);
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(std::max(1.0, timeout_s)));
        bool cancel_sent = false;
        std::optional<std::chrono::steady_clock::time_point> grace_deadline;
        if (cancellationRequested()) {
            cancelChildGoal();
            cancel_sent = true;
        }
        while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
            heartbeat(gs);
            const auto now = std::chrono::steady_clock::now();
            if (!cancel_sent && cancellationRequested()) {
                cancelChildGoal();
                cancel_sent = true;
            }
            if (!cancel_sent && now > deadline) {
                RCLCPP_ERROR(
                    get_logger(), "[AMT1] %s: prazo de %.0f s estourado — cancelando o goal filho.",
                    label.c_str(), timeout_s);
                cancelChildGoal();
                cancel_sent = true;
                run.timed_out = true;
            }
            if (cancel_sent && !grace_deadline) {
                grace_deadline = now + std::chrono::seconds(20);
            }
            if (grace_deadline && now > *grace_deadline) {
                RCLCPP_ERROR(
                    get_logger(), "[AMT1] %s: servidor nao respondeu ao cancel em 20 s — desistindo.",
                    label.c_str());
                setChildCancel(nullptr);
                run.timed_out = true;
                return run;
            }
        }
        setChildCancel(nullptr);
        const auto wrapped = result_future.get();
        run.code = wrapped.code;
        run.result = wrapped.result;
        return run;
    }

    /// Desfechos comuns de um goal filho. Devolve false quando o filho
    /// SUCEDEU com result (o chamador le o desfecho real e atualiza o modelo
    /// da mesa — mesmo que um cancel nosso tenha chegado depois; a sequencia
    /// e interrompida pelo chamador). kCanceled SO quando o cancel veio do
    /// proprio /amt1_sort (our_cancel): um CANCELED do filho por prazo
    /// estourado (fomos nos que cancelamos) e kTimeout, e um CANCELED de
    /// terceiros e kAborted — nos dois casos o goal /amt1_sort NAO esta em
    /// CANCELING e responder canceled() mataria o no (RCLError).
    template<typename ActionT>
    static bool fillCommonOutcome(const ChildRun<ActionT> & run, bool our_cancel, ChildResult & out)
    {
        if (run.server_missing) {
            out.outcome = ChildOutcome::kServerMissing;
            out.message = "servidor indisponivel";
            return true;
        }
        if (run.rejected) {
            out.outcome = ChildOutcome::kRejected;
            out.message = "goal rejeitado";
            return true;
        }
        if (run.code == rclcpp_action::ResultCode::SUCCEEDED && run.result) {
            return false;
        }
        if (run.timed_out) {
            out.outcome = ChildOutcome::kTimeout;
            out.message = "prazo estourado (goal filho cancelado pelo /amt1_sort)";
            return true;
        }
        if (our_cancel) {
            out.outcome = ChildOutcome::kCanceled;
            out.message = "cancelado";
            return true;
        }
        if (run.code == rclcpp_action::ResultCode::CANCELED) {
            out.outcome = ChildOutcome::kAborted;
            out.message = "goal filho cancelado fora do /amt1_sort";
            return true;
        }
        out.outcome = ChildOutcome::kAborted;
        out.message = run.result ? "abortado" : "sem result";
        return true;
    }

    /// Completa um desfecho comum de pick/place com o fail_reason/message
    /// que o filho mandou junto (abort e cancel tambem trazem result).
    template<typename ResultT>
    static void attachChildReason(const std::shared_ptr<ResultT> & result, ChildResult & out)
    {
        if (!result) {
            return;
        }
        out.fail_reason = result->fail_reason;
        if (!result->message.empty()) {
            out.message = result->message;
        }
    }

    ChildResult doPick(GoalState & gs, int tag, const std::string & label)
    {
        PickTag::Goal goal;
        goal.tag_frame = tagFrameForId(tag);
        goal.table_pose = gs.table_pose;
        goal.final_attempt = false;
        RCLCPP_INFO(
            get_logger(), "[AMT1] %s: PickTag{tag_frame=%s table_pose=%s final_attempt=false}",
            label.c_str(), goal.tag_frame.c_str(), goal.table_pose.c_str());
        const auto run = runChild<PickTag>(gs, pick_client_, goal, label, op_timeout_s_);
        ChildResult out;
        if (fillCommonOutcome(run, cancellationRequested(), out)) {
            attachChildReason(run.result, out);
            return out;
        }
        const auto & r = *run.result;
        out.fail_reason = r.fail_reason;
        out.message = r.message;
        out.suggested_shift_m = static_cast<double>(r.suggested_base_shift_m);
        if (r.success && !r.skipped) {
            out.outcome = ChildOutcome::kOk;
        } else if (r.skipped && r.unreachable) {
            out.outcome = ChildOutcome::kSkippedUnreachable;
        } else if (r.skipped && r.fail_reason.find("tag_nao") != std::string::npos) {
            out.outcome = ChildOutcome::kSkippedNotSeen;
        } else if (r.skipped) {
            out.outcome = ChildOutcome::kSkippedOther;
        } else {
            out.outcome = ChildOutcome::kAborted;
        }
        return out;
    }

    ChildResult doPlace(GoalState & gs, int tag, int slot, const std::string & label)
    {
        PlaceTag::Goal goal;
        goal.tag_frame = tagFrameForId(tag);
        goal.table_pose = gs.table_pose;
        goal.ws = gs.ws;
        goal.stack_on = slotFrame(slot);
        goal.final_attempt = false;
        RCLCPP_INFO(
            get_logger(),
            "[AMT1] %s: PlaceTag{tag_frame=%s table_pose=%s ws=%s stack_on=%s final_attempt=false}",
            label.c_str(), goal.tag_frame.c_str(), goal.table_pose.c_str(), goal.ws.c_str(),
            goal.stack_on.c_str());
        const auto run = runChild<PlaceTag>(gs, place_client_, goal, label, op_timeout_s_);
        ChildResult out;
        if (fillCommonOutcome(run, cancellationRequested(), out)) {
            attachChildReason(run.result, out);
            return out;
        }
        const auto & r = *run.result;
        out.fail_reason = r.fail_reason;
        out.message = r.message;
        out.suggested_shift_m = static_cast<double>(r.suggested_base_shift_m);
        if (r.success && !r.skipped) {
            out.outcome = ChildOutcome::kOk;
        } else if (r.skipped && r.unreachable) {
            out.outcome = ChildOutcome::kSkippedUnreachable;
        } else if (r.skipped) {
            out.outcome = ChildOutcome::kSkippedOther;
        } else {
            out.outcome = ChildOutcome::kAborted;
        }
        return out;
    }

    /// Deslocamento lateral da base (+ = esquerda). Atualiza contadores e o
    /// x_manip dos slots (alvo fixo: x + dy).
    bool nudge(GoalState & gs, double dy, std::string * why)
    {
        char label[48];
        std::snprintf(label, sizeof(label), "nudge_%+.2f", dy);
        publishStage(gs, label);
        speak(dy > 0.0 ? "Vou me deslocar um pouco para a esquerda" : "Vou me deslocar um pouco para a direita");
        NudgeBase::Goal goal;
        goal.dy = static_cast<float>(dy);
        goal.timeout = static_cast<float>(std::max(0.0, nudge_timeout_s_));
        const auto run = runChild<NudgeBase>(
            gs, nudge_client_, goal, label, nudge_timeout_s_ + 10.0);
        ChildResult common;
        if (fillCommonOutcome(run, cancellationRequested(), common)) {
            if (why) {*why = common.message;}
            return false;
        }
        const auto & r = *run.result;
        const double travelled = static_cast<double>(r.travelled);
        // Mesmo falhando, a base pode ter andado: contabiliza o que a
        // odometria mediu para os slots (x_manip) e o total.
        if (std::abs(travelled) > 1e-4) {
            gs.total_shift_m += travelled;
            std::lock_guard<std::mutex> lock(slots_mutex_);
            for (Slot & slot : slots_) {
                slot.x_manip += travelled;
            }
        }
        if (!r.success) {
            if (why) {*why = r.reason.empty() ? "falha" : r.reason;}
            RCLCPP_ERROR(
                get_logger(), "[AMT1] nudge %+.2f m FALHOU: %s (andou %+.3f m).",
                dy, r.reason.c_str(), travelled);
            return false;
        }
        ++gs.nudges;
        RCLCPP_INFO(
            get_logger(),
            "[AMT1] nudge %+.2f m ok: andou %+.3f m (total %+.3f m, %d/%d; /manip/base_shift_total=%+.3f)",
            dy, travelled, gs.total_shift_m, gs.nudges, max_nudges_,
            base_shift_total_.load(std::memory_order_relaxed));
        return true;
    }

    double clampShift(double suggestion) const
    {
        if (!std::isfinite(suggestion) || std::abs(suggestion) < 1e-6) {
            return 0.0;
        }
        double lo = std::abs(min_shift_m_);
        double hi = std::abs(max_shift_m_);
        if (lo > hi) {
            std::swap(lo, hi);
        }
        return std::copysign(std::clamp(std::abs(suggestion), lo, hi), suggestion);
    }

    /// Re-observa de transport_pose e corrige os slots pelo delta medio
    /// (odom) das tags que continuam na mesa. Sem ancoras suficientes ou
    /// com espalhamento grande, mantem a odometria. false so em cancel.
    bool reregisterSlots(GoalState & gs)
    {
        publishStage(gs, "reregister");
        gs.samples.clear();
        const ObserveOutcome o = observeTable(gs, {transport_pose_}, reregister_dwell_s_, "reregister");
        if (o == ObserveOutcome::kCanceled) {
            return false;
        }
        if (o != ObserveOutcome::kOk) {
            RCLCPP_WARN(get_logger(), "[AMT1] re-registro: observacao falhou — mantendo a odometria.");
            return true;
        }
        // Ancoras ORIGINAIS primeiro: slots intactos (tag atual == tag
        // inicial e nunca pega). Um cubo que o proprio robo soltou pousa com
        // erro de ate ~3 cm (> reregister_max_spread_m) e estragaria o
        // espalhamento logo depois dos places. So sem ancoras originais
        // suficientes cai para todas as tags na mesa.
        std::vector<amt1::AnchorDelta> original;
        std::vector<amt1::AnchorDelta> all;
        const std::vector<Slot> slots = slotsSnapshot();
        for (const Slot & slot : slots) {
            const std::size_t k = static_cast<std::size_t>(slot.index);
            if (k >= gs.table.size() || gs.table[k] <= 0) {
                continue;
            }
            const auto it = gs.samples.find(gs.table[k]);
            if (it == gs.samples.end()) {
                continue;
            }
            amt1::AnchorDelta d;
            d.id = gs.table[k];
            d.dx = it->second.tf_anchor.translation.x - slot.tf_anchor.translation.x;
            d.dy = it->second.tf_anchor.translation.y - slot.tf_anchor.translation.y;
            const bool intact = gs.table[k] == slot.initial_tag && gs.touched.count(gs.table[k]) == 0;
            all.push_back(d);
            if (intact) {
                original.push_back(d);
            }
            RCLCPP_INFO(
                get_logger(), "[AMT1] ancora tag_%d @ slot %d: delta (%+.3f, %+.3f) m (%s)",
                d.id, slot.index, d.dx, d.dy, intact ? "original" : "pousado pelo robo");
        }
        const std::size_t min_anchors = static_cast<std::size_t>(std::max(1, reregister_min_anchors_));
        const std::vector<amt1::AnchorDelta> * anchors = &original;
        if (original.size() < min_anchors && all.size() > original.size()) {
            RCLCPP_WARN(
                get_logger(),
                "[AMT1] re-registro: so %zu ancora(s) original(is) (< %zu) — usando tambem %zu cubo(s) "
                "pousado(s) pelo robo (erro de pouso pode reprovar o espalhamento).",
                original.size(), min_anchors, all.size() - original.size());
            anchors = &all;
        }
        const amt1::RegistrationResult reg = amt1::registrationDelta(
            *anchors, min_anchors, reregister_max_spread_m_);
        if (!reg.ok) {
            RCLCPP_WARN(
                get_logger(),
                "[AMT1] re-registro NAO aplicado (%s: %zu ancora(s), %zu original(is), espalhamento %.3f m) — "
                "mantendo os slots pela odometria.",
                reg.reason.c_str(), reg.used, original.size(), reg.spread);
            return true;
        }
        {
            std::lock_guard<std::mutex> lock(slots_mutex_);
            for (Slot & slot : slots_) {
                slot.tf_anchor.translation.x += reg.dx;
                slot.tf_anchor.translation.y += reg.dy;
            }
        }
        RCLCPP_INFO(
            get_logger(),
            "[AMT1] re-registro aplicado: delta (%+.3f, %+.3f) m em %s com %zu ancora(s), "
            "espalhamento %.3f m. Ancoras originais: %zu%s",
            reg.dx, reg.dy, anchor_frame_.c_str(), reg.used, reg.spread, original.size(),
            anchors == &all && all.size() > original.size() ? " (usei tambem cubos pousados pelo robo)" : "");
        return true;
    }

    /// Executa uma op com a politica de alcance: unreachable => nudge +
    /// re-registro e repete; sugestao 0 => re-registra e repete 1x; pick nao
    /// visto => re-observa das poses laterais e repete (retry_not_seen).
    /// Devolve true no sucesso; senao preenche gs.fail_reason/message.
    bool runWithReachRetries(GoalState & gs, const amt1::SortOp & op, const std::string & label)
    {
        int reregister_only = 0;
        int not_seen = 0;
        int attempts = 0;
        const int max_attempts = max_nudges_ + retry_not_seen_ + 4;
        while (true) {
            if (cancellationRequested()) {
                gs.fail_reason = "cancelado";
                gs.message = "cancelado em " + label;
                return false;
            }
            if (++attempts > max_attempts) {
                gs.fail_reason = "fora_de_alcance";
                gs.message = label + ": tentativas esgotadas (" + std::to_string(attempts - 1) + ")";
                return false;
            }
            publishStage(gs, label);
            const bool is_pick = op.type == amt1::SortOpType::kPick;
            const ChildResult r = is_pick ? doPick(gs, op.tag, label) : doPlace(gs, op.tag, op.slot, label);
            const std::string kind = is_pick ? "pick" : "place";
            const std::string failed_reason = is_pick ? "pick_falhou" : "place_falhou";

            switch (r.outcome) {
                case ChildOutcome::kOk:
                    if (is_pick) {
                        if (static_cast<std::size_t>(op.slot) < gs.table.size()) {
                            if (gs.table[op.slot] != op.tag) {
                                RCLCPP_WARN(
                                    get_logger(), "[AMT1] modelo: slot %d tinha tag_%d, esperado tag_%d.",
                                    op.slot, gs.table[op.slot], op.tag);
                            }
                            gs.table[op.slot] = 0;
                        }
                        gs.onboard.push_back(op.tag);
                        gs.touched.insert(op.tag);
                        ++gs.picks;
                    } else {
                        const auto it = std::find(gs.onboard.begin(), gs.onboard.end(), op.tag);
                        if (it != gs.onboard.end()) {
                            gs.onboard.erase(it);
                        }
                        if (static_cast<std::size_t>(op.slot) < gs.table.size()) {
                            gs.table[op.slot] = op.tag;
                        }
                        ++gs.places;
                    }
                    // pick/place voltam o braco a pegar_obj ao terminar.
                    arm_at_transport_ = true;
                    arm_state_known_ = true;
                    return true;

                case ChildOutcome::kCanceled:
                    gs.fail_reason = "cancelado";
                    gs.message = label + " cancelado";
                    arm_state_known_ = false;
                    return false;

                case ChildOutcome::kRejected:
                    gs.fail_reason = failed_reason;
                    gs.message = label + ": goal rejeitado pelo servidor de " + kind + " (braco ocupado?)";
                    return false;

                case ChildOutcome::kServerMissing:
                    gs.fail_reason = failed_reason;
                    gs.message = label + ": servidor de " + kind + " indisponivel";
                    return false;

                case ChildOutcome::kAborted:
                case ChildOutcome::kTimeout:
                    gs.fail_reason = failed_reason;
                    gs.message = label + ": " + kind + " " +
                        (r.outcome == ChildOutcome::kTimeout ? "estourou o prazo" : "abortou") +
                        (r.fail_reason.empty() ? "" : " (" + r.fail_reason + ")") +
                        (r.message.empty() ? "" : ": " + r.message);
                    // Abort/timeout no meio do pick/place: o cubo pode estar
                    // na garra (falha_com_bloco_na_garra, empilhar_falhou,
                    // garra_nao_abriu_no_destino, cancel por prazo no
                    // transporte...). Nada de recoverOnboard depois disto.
                    gs.unknown = true;
                    gs.child_aborted = true;
                    gs.child_fail_reason = r.fail_reason.empty() ? r.message : r.fail_reason;
                    arm_state_known_ = false;
                    arm_at_transport_ = false;
                    return false;

                case ChildOutcome::kSkippedOther:
                    if (!is_pick && r.fail_reason == "tag_nao_esta_em_container") {
                        gs.fail_reason = "container_inconsistente";
                        gs.message = label + ": o place nao achou tag_" + std::to_string(op.tag) +
                            " em nenhum container de bordo";
                    } else {
                        gs.fail_reason = failed_reason;
                        gs.message = label + ": " + kind + " pulou (" + r.fail_reason + ")" +
                            (r.message.empty() ? "" : ": " + r.message);
                    }
                    // skipped = garra vazia e braco parado num lugar seguro,
                    // mas o recolhimento pode ter falhado: nao assume
                    // pegar_obj — finish() leva o braco la (barato se ja esta).
                    arm_at_transport_ = false;
                    arm_state_known_ = true;
                    return false;

                case ChildOutcome::kSkippedNotSeen:
                    arm_at_transport_ = false;
                    arm_state_known_ = true;
                    if (not_seen < retry_not_seen_) {
                        ++not_seen;
                        RCLCPP_WARN(
                            get_logger(),
                            "[AMT1] %s: pick nao viu tag_%d (%s) — re-observando das poses laterais (%d/%d).",
                            label.c_str(), op.tag, r.fail_reason.c_str(), not_seen, retry_not_seen_);
                        gs.samples.clear();
                        const ObserveOutcome o = observeTable(gs, reobserve_poses_, observe_dwell_s_, "reobserve");
                        if (o == ObserveOutcome::kCanceled) {
                            gs.fail_reason = "cancelado";
                            gs.message = label + " cancelado na re-observacao";
                            return false;
                        }
                        if (gs.samples.count(op.tag) == 0) {
                            RCLCPP_WARN(
                                get_logger(), "[AMT1] %s: tag_%d tambem nao apareceu na re-observacao.",
                                label.c_str(), op.tag);
                        }
                        continue;
                    }
                    gs.fail_reason = "tag_nao_encontrada";
                    gs.message = label + ": tag_" + std::to_string(op.tag) + " nao encontrada na mesa";
                    return false;

                case ChildOutcome::kSkippedUnreachable: {
                    arm_at_transport_ = true;
                    arm_state_known_ = true;
                    const double suggestion = r.suggested_shift_m;
                    const double dy = clampShift(suggestion);
                    const bool budget_ok =
                        dy != 0.0 && nudge_enabled_ && gs.nudge_available &&
                        gs.nudges < max_nudges_ &&
                        std::abs(gs.total_shift_m + dy) <= max_total_shift_m_ + 1e-6;
                    if (budget_ok) {
                        RCLCPP_WARN(
                            get_logger(),
                            "[AMT1] %s: alvo fora de alcance (sugestao %+.2f m) — nudge %+.2f m "
                            "(%d/%d, total %+.2f m).",
                            label.c_str(), suggestion, dy, gs.nudges + 1, max_nudges_, gs.total_shift_m);
                        std::string why;
                        if (!nudge(gs, dy, &why)) {
                            if (cancellationRequested()) {
                                gs.fail_reason = "cancelado";
                                gs.message = label + " cancelado no nudge";
                            } else {
                                gs.fail_reason = "nudge_falhou";
                                gs.message = label + ": nudge " + std::string(dy > 0 ? "esquerda" : "direita") +
                                    " falhou (" + why + ")";
                            }
                            return false;
                        }
                        if (reregister_after_nudge_ && !reregisterSlots(gs)) {
                            gs.fail_reason = "cancelado";
                            gs.message = label + " cancelado no re-registro";
                            return false;
                        }
                        continue;
                    }
                    if (std::abs(suggestion) < 1e-6 && reregister_only < 1) {
                        ++reregister_only;
                        RCLCPP_WARN(
                            get_logger(),
                            "[AMT1] %s: %s sem sugestao de deslocamento (%s) — re-registrando os slots e repetindo 1x.",
                            label.c_str(), kind.c_str(), r.fail_reason.c_str());
                        if (!reregisterSlots(gs)) {
                            gs.fail_reason = "cancelado";
                            gs.message = label + " cancelado no re-registro";
                            return false;
                        }
                        continue;
                    }
                    gs.fail_reason = "fora_de_alcance";
                    char buf[160];
                    std::snprintf(
                        buf, sizeof(buf),
                        ": fora de alcance (sugestao %+.2f m; nudges %d/%d, total %+.2f de %.2f m, nudge %s)",
                        suggestion, gs.nudges, max_nudges_, gs.total_shift_m, max_total_shift_m_,
                        (nudge_enabled_ && gs.nudge_available) ? "disponivel" : "indisponivel");
                    gs.message = label + buf + (r.fail_reason.empty() ? "" : " [" + r.fail_reason + "]");
                    return false;
                }
            }
        }
    }

    /// Falha no meio: solta os cubos a bordo nos slots livres (mais perto de
    /// x=0 primeiro) para a mesa ficar consistente. Contrato: SO apos falhas
    /// do tipo skipped/unreachable (braco em pegar_obj, garra vazia) — nunca
    /// apos abort/timeout de filho (child_aborted: o cubo pode estar na garra
    /// e o place abriria a garra em pegar_obj).
    void recoverOnboard(GoalState & gs)
    {
        if (gs.onboard.empty() || cancellationRequested() || gs.child_aborted) {
            return;
        }
        publishStage(gs, "recover_onboard");
        speak("Vou devolver os blocos que estao a bordo para a mesa");
        const std::vector<Slot> slots = slotsSnapshot();
        std::vector<int> onboard = gs.onboard;
        for (const int tag : onboard) {
            std::vector<const Slot *> free_slots;
            for (const Slot & slot : slots) {
                const std::size_t k = static_cast<std::size_t>(slot.index);
                if (k < gs.table.size() && gs.table[k] == 0) {
                    free_slots.push_back(&slot);
                }
            }
            std::sort(
                free_slots.begin(), free_slots.end(),
                [](const Slot * a, const Slot * b) {return std::abs(a->x_manip) < std::abs(b->x_manip);});
            bool placed = false;
            for (const Slot * slot : free_slots) {
                if (cancellationRequested()) {
                    return;
                }
                amt1::SortOp op;
                op.type = amt1::SortOpType::kPlace;
                op.tag = tag;
                op.slot = slot->index;
                const std::string label = "recover_" + amt1::describeOp(op);
                // Copia para nao sobrescrever a causa original; os flags de
                // estado sao zerados por tentativa e so o que ESTA tentativa
                // produzir volta para gs.
                GoalState probe = gs;
                probe.unknown = false;
                probe.child_aborted = false;
                probe.child_fail_reason.clear();
                probe.fail_reason.clear();
                probe.message.clear();
                if (runWithReachRetries(probe, op, label)) {
                    gs.table = probe.table;
                    gs.onboard = probe.onboard;
                    gs.touched = probe.touched;
                    gs.places = probe.places;
                    gs.nudges = probe.nudges;
                    gs.total_shift_m = probe.total_shift_m;
                    placed = true;
                    break;
                }
                gs.nudges = probe.nudges;
                gs.total_shift_m = probe.total_shift_m;
                if (probe.unknown) {
                    gs.unknown = true;
                }
                if (probe.child_aborted) {
                    gs.child_aborted = true;
                    gs.child_fail_reason = probe.child_fail_reason;
                }
                RCLCPP_WARN(
                    get_logger(), "[AMT1] recuperacao: nao consegui soltar tag_%d no slot %d (%s).",
                    tag, slot->index, probe.message.c_str());
                if (probe.fail_reason == "cancelado" || cancellationRequested() ||
                    probe.unknown || probe.child_aborted)
                {
                    return;
                }
            }
            if (!placed) {
                RCLCPP_ERROR(get_logger(), "[AMT1] recuperacao: tag_%d continua a bordo.", tag);
            }
        }
    }

    // ------------------------------------------------------------ capacidade
    /// Containers de bordo vazios segundo o yaml (arquivo ausente = todos).
    int countEmptyContainers(std::string * why)
    {
        try {
            if (!std::filesystem::exists(container_state_file_)) {
                if (why) {*why = "arquivo ausente, assumindo " + std::to_string(container_names_.size()) + " vazios";}
                return static_cast<int>(container_names_.size());
            }
            const YAML::Node root = YAML::LoadFile(container_state_file_);
            const YAML::Node containers = root["containers"];
            if (!containers || !containers.IsMap()) {
                if (why) {*why = "yaml sem 'containers', assumindo todos vazios";}
                return static_cast<int>(container_names_.size());
            }
            int empties = 0;
            std::string occupied_txt;
            for (const auto & it : containers) {
                const YAML::Node c = it.second;
                const bool occupied = c && c["occupied"] && c["occupied"].as<bool>(false);
                if (!occupied) {
                    ++empties;
                } else {
                    occupied_txt += it.first.as<std::string>() + "=" +
                        (c["tag_frame"] ? c["tag_frame"].as<std::string>("") : "") + " ";
                }
            }
            if (why) {*why = occupied_txt.empty() ? "todos vazios" : "ocupados: " + occupied_txt;}
            return empties;
        } catch (const std::exception & ex) {
            if (why) {*why = std::string("yaml ilegivel: ") + ex.what();}
            return 0;
        }
    }

    // ------------------------------------------------------------ action server
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const Amt1Sort::Goal> goal)
    {
        if (goal->ws.empty() || goal->table_pose.empty()) {
            RCLCPP_WARN(get_logger(), "[AMT1] goal rejeitado: ws e table_pose sao obrigatorios.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        const std::string dir = toLower(goal->direction);
        if (!dir.empty() && dir != "left_to_right" && dir != "right_to_left") {
            RCLCPP_WARN(
                get_logger(), "[AMT1] goal rejeitado: direction '%s' (esperado left_to_right|right_to_left).",
                goal->direction.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->max_onboard < 0 || goal->max_onboard > 3) {
            RCLCPP_WARN(get_logger(), "[AMT1] goal rejeitado: max_onboard %d fora de 0..3.", goal->max_onboard);
            return rclcpp_action::GoalResponse::REJECT;
        }
        bool expected_active = false;
        if (!goal_active_.compare_exchange_strong(expected_active, true)) {
            RCLCPP_WARN(get_logger(), "[AMT1] goal rejeitado: ja ha uma ordenacao em curso.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        cancel_requested_.store(false);
        RCLCPP_INFO(
            get_logger(),
            "[AMT1] goal aceito: ws=%s table_pose=%s expected=[%s] direction=%s max_onboard=%d observe_only=%s",
            goal->ws.c_str(), goal->table_pose.c_str(),
            joinInts(std::vector<int>(goal->expected_tags.begin(), goal->expected_tags.end())).c_str(),
            dir.empty() ? "left_to_right" : dir.c_str(), goal->max_onboard,
            goal->observe_only ? "true" : "false");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAmt1Sort>)
    {
        RCLCPP_WARN(get_logger(), "[AMT1] cancelamento pedido — propagando ao goal filho e parando o braco.");
        cancel_requested_.store(true);
        stopActiveMotion();
        cancelChildGoal();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleAmt1Sort> goal_handle)
    {
        std::thread{std::bind(&Amt1SortActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    /// Fim do goal: braco em transport_pose, difusao, result, resposta.
    /// A UNICA fonte para responder canceled() e handle->is_canceling():
    /// canceled() num goal EXECUTING lanca RCLError e, numa thread
    /// destacada, derruba o no (std::terminate). fail_reason "cancelado" com
    /// o goal fora de CANCELING vira succeed(success=false) se o braco esta
    /// conhecido na pose de transporte, senao abort().
    void finish(GoalState & gs)
    {
        auto result = std::make_shared<Amt1Sort::Result>();
        // handle_cancel marca cancel_requested_ ANTES de o servidor mover o
        // goal para CANCELING: da ate 1 s para a transicao chegar.
        for (int i = 0; i < 20 && cancellationRequested() && !gs.handle->is_canceling(); ++i) {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        const bool canceled = gs.handle->is_canceling();
        if (!canceled && (gs.fail_reason == "cancelado" || cancellationRequested())) {
            RCLCPP_WARN(
                get_logger(),
                "[AMT1] fail_reason cancelado mas o goal nao esta em CANCELING — respondendo como falha.");
        }

        // Braco: volta a transporte se saiu (melhor esforco, com lock).
        // Seguro mesmo com cubo na garra (child_aborted): so move o braco.
        if (observe_move_enabled_ && !canceled && (!arm_at_transport_ || !arm_state_known_)) {
            publishStage(gs, "returning_" + transport_pose_);
            if (acquireLock()) {
                auto arm = ensureArm();
                if (arm && goToNamedPose(arm, transport_pose_, "final return " + transport_pose_)) {
                    arm_at_transport_ = true;
                    arm_state_known_ = true;
                    gs.unknown = false;
                } else {
                    arm_state_known_ = false;
                    RCLCPP_ERROR(get_logger(), "[AMT1] nao consegui levar o braco a %s.", transport_pose_.c_str());
                }
                releaseLock();
            } else {
                arm_state_known_ = false;
            }
        }
        if (!observe_move_enabled_) {
            arm_state_known_ = true;  // sem braco: nada a desconhecer
            arm_at_transport_ = true;
        }

        // Ordem final reconstruida do modelo (0 = slot vazio).
        result->observed_order = std::vector<int32_t>(
            gs.assignment.observed_order.begin(), gs.assignment.observed_order.end());
        result->final_order = std::vector<int32_t>(gs.table.begin(), gs.table.end());
        result->missing_tags = std::vector<int32_t>(
            gs.assignment.missing_tags.begin(), gs.assignment.missing_tags.end());
        result->picks = gs.picks;
        result->places = gs.places;
        result->nudges = gs.nudges;
        result->total_shift_m = static_cast<float>(gs.total_shift_m);

        const bool sorted = !gs.table.empty() && gs.table == gs.assignment.target_order;
        result->success = gs.fail_reason.empty() && !canceled && gs.assignment.missing_tags.empty() &&
            (gs.observe_only || sorted || gs.table.empty());
        // partial = estado FISICO consistente: nada a bordo e braco conhecido.
        // Com cubos a bordo a mesa esta incompleta => partial=false.
        result->partial = !result->success && gs.onboard.empty() && arm_state_known_;
        result->fail_reason = canceled ? "cancelado" : gs.fail_reason;
        result->message = gs.message;
        if (!gs.observe_only && !canceled) {
            char tail[160];
            std::snprintf(
                tail, sizeof(tail), " [picks %d, places %d, nudges %d, total %+.2f m; final: %s]",
                gs.picks, gs.places, gs.nudges, gs.total_shift_m, joinInts(gs.table).c_str());
            result->message += tail;
        }
        if (!gs.onboard.empty()) {
            result->message += " | " + std::to_string(gs.onboard.size()) + " cubo(s) a bordo [" +
                joinInts(gs.onboard) + "] ficam nos containers (memoria yaml preservada)" +
                (gs.child_aborted ? "; cubo possivelmente na garra apos " + gs.child_fail_reason : "");
        }

        if (!gs.observe_only) {
            stopBroadcasting();
        }
        if (!canceled) {
            speak(result->success || (result->partial && gs.fail_reason.empty()) ?
                "Ordenacao concluida" : "Nao consegui terminar a ordenacao");
        }
        publishStage(gs, canceled ? "canceled" : (arm_state_known_ ? "done" : "aborted"));
        RCLCPP_INFO(
            get_logger(),
            "[AMT1] fim: success=%s partial=%s fail_reason='%s' observed=[%s] final=[%s] missing=[%s] "
            "picks=%d places=%d nudges=%d total=%+.2f m braco=%s — %s",
            result->success ? "true" : "false", result->partial ? "true" : "false",
            result->fail_reason.c_str(), joinInts(gs.assignment.observed_order).c_str(),
            joinInts(gs.table).c_str(), joinInts(gs.assignment.missing_tags).c_str(),
            gs.picks, gs.places, gs.nudges, gs.total_shift_m,
            arm_state_known_ ? "conhecido" : "DESCONHECIDO", result->message.c_str());

        setChildCancel(nullptr);
        setActiveArm(nullptr);
        releaseLock();
        publishActive(false);
        cancel_requested_.store(false);
        goal_active_.store(false);

        // Resposta: canceled() SO com o goal em CANCELING (ver acima). Uma
        // excecao aqui (transicao invalida) NAO pode escapar da thread do
        // goal: tenta abort() e, no pior caso, so loga.
        try {
            if (canceled) {
                gs.handle->canceled(result);
            } else if (arm_state_known_) {
                gs.handle->succeed(result);
            } else {
                gs.handle->abort(result);
            }
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(
                get_logger(), "[AMT1] falha ao responder o goal (%s): %s — tentando abort().",
                canceled ? "canceled" : (arm_state_known_ ? "succeed" : "abort"), ex.what());
            try {
                gs.handle->abort(result);
            } catch (const std::exception & ex2) {
                RCLCPP_FATAL(get_logger(), "[AMT1] abort() tambem falhou: %s", ex2.what());
            }
        }
    }

    void execute(const std::shared_ptr<GoalHandleAmt1Sort> goal_handle)
    {
        GoalState gs;
        gs.handle = goal_handle;
        const auto goal = goal_handle->get_goal();
        gs.ws = normalizedWs(goal->ws);
        gs.table_pose = goal->table_pose;
        gs.expected = std::vector<int>(goal->expected_tags.begin(), goal->expected_tags.end());
        gs.left_to_right = toLower(goal->direction) != "right_to_left";
        gs.observe_only = goal->observe_only;
        arm_state_known_ = true;
        arm_at_transport_ = true;
        lock_held_ = false;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            last_feedback_ = std::chrono::steady_clock::now() - std::chrono::hours(1);
        }
        publishActive(true);
        if (gs.observe_only) {
            stopBroadcasting();  // frames de um goal anterior nao valem mais
        }
        publishStage(gs, "starting");
        speak("Iniciando a ordenacao da mesa");

        if (cancellationRequested()) {
            gs.fail_reason = "cancelado";
            gs.message = "cancelado antes de comecar";
            finish(gs);
            return;
        }

        // 1. Capacidade: B = min(max_onboard, containers vazios).
        publishStage(gs, "checking_capacity");
        for (const int id : gs.expected) {
            std::string container;
            std::string err;
            if (container_state_store_->findContainerByTag(tagFrameForId(id), &container, &err)) {
                gs.fail_reason = "container_inconsistente";
                gs.message = "tag_" + std::to_string(id) + " esperada na mesa mas o yaml diz que esta no " +
                    container;
                RCLCPP_ERROR(get_logger(), "[AMT1] %s", gs.message.c_str());
                finish(gs);
                return;
            }
        }
        std::string cap_why;
        const int empties = countEmptyContainers(&cap_why);
        const int requested = goal->max_onboard <= 0 ? static_cast<int>(container_names_.size()) : goal->max_onboard;
        gs.buffer = std::min(requested, empties);
        RCLCPP_INFO(
            get_logger(), "[AMT1] buffer = min(max_onboard %d, vazios %d) = %d (%s)",
            requested, empties, gs.buffer, cap_why.c_str());
        if (gs.buffer < 2) {
            speak("Preciso de dois containers livres");
            if (!gs.observe_only) {
                gs.fail_reason = "buffer_insuficiente";
                gs.message = "buffer " + std::to_string(gs.buffer) + " < 2 (max_onboard " +
                    std::to_string(requested) + ", containers vazios " + std::to_string(empties) + ": " + cap_why + ")";
                finish(gs);
                return;
            }
            RCLCPP_WARN(get_logger(), "[AMT1] observe_only: seguindo com buffer 2 so para planejar.");
            gs.buffer = 2;
        }

        // Servidores filhos (so quando vai mover cubo).
        if (!gs.observe_only) {
            publishStage(gs, "waiting_servers");
            const auto wait = std::chrono::duration<double>(std::max(1.0, server_wait_s_));
            if (!pick_client_->wait_for_action_server(wait)) {
                gs.fail_reason = "pick_falhou";
                gs.message = "/pick_tag indisponivel em " + std::to_string(static_cast<int>(server_wait_s_)) + " s";
                finish(gs);
                return;
            }
            if (!place_client_->wait_for_action_server(wait)) {
                gs.fail_reason = "place_falhou";
                gs.message = "/place_tag indisponivel em " + std::to_string(static_cast<int>(server_wait_s_)) + " s";
                finish(gs);
                return;
            }
            gs.nudge_available = nudge_enabled_ && nudge_client_->wait_for_action_server(std::chrono::seconds(2));
            if (nudge_enabled_ && !gs.nudge_available) {
                RCLCPP_WARN(get_logger(), "[AMT1] nudge_base indisponivel (dock_align_node?) — sem ajuste lateral neste goal.");
            }
        }

        // 2. Observacao.
        gs.samples.clear();
        const ObserveOutcome obs = observeTable(gs, observe_poses_, observe_dwell_s_, "observing");
        if (obs == ObserveOutcome::kCanceled) {
            gs.fail_reason = "cancelado";
            gs.message = "cancelado na observacao";
            finish(gs);
            return;
        }
        if (obs != ObserveOutcome::kOk) {
            gs.fail_reason = "observacao_falhou";
            gs.message = obs == ObserveOutcome::kLockBusy ?
                "lock do braco ocupado: nao consegui observar a mesa" :
                "braco nao chegou nas poses de observacao (move_group?)";
            finish(gs);
            return;
        }
        filterSamplesByHeight(gs);

        std::vector<amt1::TagPose> seen;
        for (const auto & kv : gs.samples) {
            amt1::TagPose tp;
            tp.id = kv.first;
            tp.x = kv.second.p_manip.x();
            tp.y = kv.second.p_manip.y();
            tp.z = kv.second.p_manip.z();
            tp.yaw = kv.second.yaw_manip;
            seen.push_back(tp);
        }
        gs.assignment = amt1::assignSlots(seen, gs.expected, gs.left_to_right, slot_tie_eps_m_);
        RCLCPP_INFO(
            get_logger(),
            "[AMT1] vistas %zu tag(s): ordem [%s] alvo [%s] faltam [%s] ignoradas [%s]",
            gs.samples.size(), joinInts(gs.assignment.observed_order).c_str(),
            joinInts(gs.assignment.target_order).c_str(),
            joinInts(gs.assignment.missing_tags).c_str(), joinInts(gs.assignment.ignored_tags).c_str());

        if (!gs.assignment.missing_tags.empty()) {
            speak("Nao vi as tags " + joinInts(gs.assignment.missing_tags));
            RCLCPP_WARN(
                get_logger(), "[AMT1] tags esperadas nao vistas: [%s].",
                joinInts(gs.assignment.missing_tags).c_str());
        }
        if (static_cast<int>(gs.assignment.observed_order.size()) < min_seen_tags_) {
            gs.fail_reason = "poucas_tags_vistas";
            gs.message = "vi " + std::to_string(gs.assignment.observed_order.size()) + " tag(s) esperada(s) (< " +
                std::to_string(min_seen_tags_) + ")";
            finish(gs);
            return;
        }
        if (!gs.assignment.missing_tags.empty() && !allow_partial_) {
            gs.fail_reason = "tag_nao_encontrada";
            gs.message = "faltam as tags [" + joinInts(gs.assignment.missing_tags) + "] e allow_partial=false";
            finish(gs);
            return;
        }

        // 3. Slots + difusao dos frames.
        publishStage(gs, "registering_slots");
        buildSlots(gs);
        gs.table = gs.assignment.observed_order;
        if (!gs.assignment.target_order.empty()) {
            speak(
                "Ordem atual: " + joinInts(gs.assignment.observed_order) + ". Vou ordenar de " +
                std::to_string(gs.assignment.target_order.front()) + " a " +
                std::to_string(gs.assignment.target_order.back()));
        }

        // 4. Plano.
        publishStage(gs, "planning");
        const amt1::SortPlan plan = amt1::planSort(
            gs.assignment.observed_order, gs.assignment.target_order, gs.buffer);
        if (!plan.error.empty()) {
            gs.fail_reason = plan.error;
            gs.message = "planSort: " + plan.error;
            finish(gs);
            return;
        }
        const amt1::SimulationResult sim = amt1::simulatePlan(gs.assignment.observed_order, plan.ops, gs.buffer);
        if (!sim.ok) {
            gs.fail_reason = "buffer_insuficiente";
            gs.message = "plano reprovado na simulacao: " + sim.error;
            finish(gs);
            return;
        }
        std::string plan_txt;
        for (const amt1::SortOp & op : plan.ops) {
            plan_txt += amt1::describeOp(op) + " ";
        }
        RCLCPP_INFO(
            get_logger(), "[AMT1] plano: %zu op(s), %d pick(s), %d place(s), buffer %d: %s",
            plan.ops.size(), plan.picks, plan.places, gs.buffer, plan_txt.c_str());
        gs.ops_total = static_cast<int>(plan.ops.size());
        if (plan.ops.empty()) {
            speak("A mesa ja esta ordenada");
            gs.message = "mesa ja ordenada";
            finish(gs);
            return;
        }
        if (gs.observe_only) {
            gs.message = "observe_only: plano " + plan_txt;
            finish(gs);
            return;
        }

        // 5. Execucao.
        for (std::size_t i = 0; i < plan.ops.size(); ++i) {
            const amt1::SortOp & op = plan.ops[i];
            const std::string label =
                "op_" + std::to_string(i + 1) + "/" + std::to_string(plan.ops.size()) + "_" + amt1::describeOp(op);
            if (!runWithReachRetries(gs, op, label)) {
                RCLCPP_ERROR(
                    get_logger(), "[AMT1] %s falhou: %s — %s", label.c_str(), gs.fail_reason.c_str(),
                    gs.message.c_str());
                break;
            }
            ++gs.ops_done;
            publishStage(gs, label + ":done");
            // Cancel que chegou com o filho ja concluido: o modelo da mesa
            // foi atualizado acima; a sequencia para aqui.
            if (cancellationRequested()) {
                gs.fail_reason = "cancelado";
                gs.message = "cancelado apos " + label;
                RCLCPP_WARN(get_logger(), "[AMT1] %s", gs.message.c_str());
                break;
            }
        }

        // Falha no meio (nao cancel) com cubos a bordo.
        if (!gs.fail_reason.empty() && gs.fail_reason != "cancelado" && !cancellationRequested() &&
            !gs.onboard.empty())
        {
            if (gs.child_aborted) {
                // Abort/timeout do pick/place: o cubo pode estar na garra e
                // um place em pegar_obj o deixaria cair. Sem recoverOnboard:
                // finish() so leva o braco a transport_pose (seguro com cubo
                // na garra) e os cubos ficam nos containers (yaml preservado).
                RCLCPP_ERROR(
                    get_logger(),
                    "[AMT1] %s abortou/estourou o prazo (%s): o cubo pode estar na garra — NAO solto nada. "
                    "%zu cubo(s) a bordo [%s] ficam nos containers (memoria yaml preservada); "
                    "so devolvo o braco a %s.",
                    gs.fail_reason.c_str(), gs.child_fail_reason.c_str(), gs.onboard.size(),
                    joinInts(gs.onboard).c_str(), transport_pose_.c_str());
            } else {
                if (!arm_state_known_ && observe_move_enabled_) {
                    // Braco em estado desconhecido (sem abort de filho):
                    // tenta a pose de transporte antes de mais um place.
                    publishStage(gs, "recover_arm");
                    if (acquireLock()) {
                        auto arm = ensureArm();
                        if (arm && goToNamedPose(arm, transport_pose_, "recover arm " + transport_pose_)) {
                            arm_at_transport_ = true;
                            arm_state_known_ = true;
                            gs.unknown = false;
                        }
                        releaseLock();
                    }
                }
                if (arm_state_known_ || !observe_move_enabled_) {
                    recoverOnboard(gs);
                } else {
                    RCLCPP_ERROR(
                        get_logger(),
                        "[AMT1] braco em estado desconhecido: sem recuperacao; %zu cubo(s) a bordo [%s] "
                        "ficam nos containers (memoria yaml preservada).",
                        gs.onboard.size(), joinInts(gs.onboard).c_str());
                }
            }
        }
        if (gs.fail_reason.empty()) {
            gs.message = gs.assignment.missing_tags.empty() ?
                "mesa ordenada" :
                "ordenadas as tags vistas; faltaram [" + joinInts(gs.assignment.missing_tags) + "]";
        }
        finish(gs);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Amt1SortActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
