#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
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
#include <vector>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "manip_task_execution/container_state_store.hpp"
#include "manip_task_execution/manipulator_execution_lock.hpp"
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
        skip_missing_place_tag_ =
            this->declare_parameter<bool>("skip_missing_place_tag", true);
        // Auditoria 2026-08-07, item 2.4: verificacao de objeto preso por
        // esforco tambem no PLACE (garra fechando no vazio dentro do
        // container retornava sucesso fantasma). Limiares iniciais = os do
        // pick; CALIBRAR no robo real.
        verify_grasp_effort_ =
            this->declare_parameter<bool>("verify_grasp_effort", true);
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

        // 2026-08-12: entregar na prateleira exige um waypoint obrigatorio
        // (`pirocao`) antes e depois da pose de mesa — sem ele o braco bate
        // na estante. Mesmo vocabulario/parametro usados no no de pick.
        shelf_table_poses_ = this->declare_parameter<std::vector<std::string>>(
            "shelf_table_poses", std::vector<std::string>{"MesaSh"});

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
    std::string camera_info_topic_;
    double perception_warmup_timeout_{6.0};
    double container_place_z_offset_;
    std::unique_ptr<manip_task_execution::ManipulatorExecutionLock> execution_lock_;
    bool speech_enabled_{true};
    bool skip_missing_place_tag_{true};
    bool verify_grasp_effort_{true};
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
        clearActiveInterfaces();
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

    bool moveToTarget(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const geometry_msgs::msg::TransformStamped & tf,
        const std::string & eef_link,
        const std::string & label,
        bool use_orientation_constraint)
    {
        if (cancellationRequested()) {
            return false;
        }

        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink(eef_link);

        tf2::Quaternion tag_q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);


        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = tf.transform.translation.x;
        target_pose.position.y = tf.transform.translation.y;
        target_pose.position.z = tf.transform.translation.z;

        if (use_orientation_constraint) {
            tf2::Quaternion desired_q;
            desired_q.setRPY(0.0, M_PI, 1.57);
            desired_q.normalize();
            target_pose.orientation = tf2::toMsg(desired_q);
        } else {
            target_pose.orientation = arm->getCurrentPose(eef_link).pose.orientation;
        }

        MoveGroupInterface::Plan plan;

        arm->setGoalPositionTolerance(0.0003);
        arm->setGoalOrientationTolerance(use_orientation_constraint ? 0.05 : M_PI);
        arm->clearPoseTargets();
        arm->setPoseTarget(target_pose, eef_link);

        bool success = planWithDeadline(arm, plan, label + " setPoseTarget");

        if (!success) {
            RCLCPP_WARN_STREAM(
                get_logger(),
                "Plan failed with setPoseTarget for " << label
                    << ". Trying approximate IK.");

            // Guarda anti-travamento (auditoria 2026-08-07, item 1.4 —
            // espelho da guarda validada no pick): a IK aproximada precisa do
            // estado atual; sem ele o fluxo pendurava indefinidamente.
            if (!arm->getCurrentState(2.0)) {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "Sem estado atual do braco para IK aproximada em " << label
                        << " — abortando a tentativa.");
                return false;
            }

            arm->clearPoseTargets();
            arm->setApproximateJointValueTarget(target_pose, eef_link);
            success = planWithDeadline(arm, plan, label + " IK aproximada");
        }

        if (!success) {
            RCLCPP_ERROR_STREAM(get_logger(), "Planning failed: " << label);
            return false;
        }

        if (cancellationRequested()) {
            return false;
        }

        const bool exec_ok = executeWithDeadline(arm, plan, label);
        if (cancellationRequested()) {
            return false;
        }
        if (!exec_ok) {
            RCLCPP_ERROR_STREAM(get_logger(), "Execution failed: " << label);
            return false;
        }

        return true;
    }

    bool moveToPlaceTarget(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::string & table_pose,
        const std::shared_ptr<GoalHandlePlaceTag> & goal_handle)
    {
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
            speak("Indo para a pose de entrega " + spokenTargetName(table_pose));
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(table_pose);
            return planAndExecute(arm, "go " + table_pose);
        }

        publish_stage(goal_handle, "detecting_place_tag");
        speak("Procurando o alvo de entrega " + spokenTargetName(table_pose));
        waitForCameraStream("PLACE");

        const std::string place_reference_frame = "base_footprint";

        geometry_msgs::msg::TransformStamped target_tf;
        if (!waitForTagTransform(
                "manip_base_link",
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
            place_reference_frame.c_str(),
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
                place_reference_frame,
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
            place_reference_frame.c_str(),
            table_pose.c_str(),
            target_tf.transform.translation.x,
            target_tf.transform.translation.y,
            target_tf.transform.translation.z);

        target_tf.transform.translation.z += container_place_z_offset_;

        publish_stage(goal_handle, "place_final_approach");
        //speak("Fazendo a aproximacao final para entregar o bloco");
        return moveToTarget(arm, target_tf, "tcp", "place above " + table_pose, true);
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
            "Received place goal tag=%s table=%s",
            goal->tag_frame.c_str(),
            goal->table_pose.c_str());

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

        auto result =
            std::make_shared<PlaceTag::Result>();

        const auto finish_failure =
            [this, &goal_handle, &result, &execution_guard](const std::string & message)
            {
                result->success = false;
                result->fail_reason = last_place_failure_reason_;
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
            [this, &goal_handle, &result, &execution_guard](const std::string & message)
            {
                result->success = true;
                result->skipped = true;  // item 2.5: contrato explicito
                result->fail_reason = "tag_nao_esta_em_container";
                result->message = message;
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
            execution_guard.release();
            goal_handle->canceled(result);
            return;
        }

        // Item 2.9 (espelho do pick): sucesso FISICO manda — falha de I/O do
        // yaml vira WARN + flag na mensagem, nunca aborta entrega feita.
        result->success = success;
        if (success && state_write_success) {
            result->message = "Place completed";
        } else if (success && !state_write_success) {
            result->message =
                "Place completed (AVISO: falha ao gravar container yaml: " +
                state_write_error + ")";
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

    bool run_place_cycle(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string& container_pose,
        const std::string& table_pose,
        const std::shared_ptr<GoalHandlePlaceTag>& goal_handle)
    {
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
        if (verify_grasp_effort_) {
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
        if (verify_grasp_effort_) {
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
        }


        arm->setMaxAccelerationScalingFactor(1.0);
        
        publish_stage(goal_handle, "going_table");
        //speak("Levando o bloco para o destino");
        if (!moveToPlaceTarget(arm, table_pose, goal_handle)) {
            last_place_failure_reason_ = "falha_com_bloco_na_garra";
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
