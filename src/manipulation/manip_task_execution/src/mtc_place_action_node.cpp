#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <atomic>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <mutex>
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
    double container_place_z_offset_;
    std::unique_ptr<manip_task_execution::ManipulatorExecutionLock> execution_lock_;
    bool speech_enabled_{true};
    bool skip_missing_place_tag_{true};
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
            server_.releaseExecutionResources();
        }

    private:
        PlaceActionServer & server_;
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
        clearActiveInterfaces();
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

    bool planAndExecute(
        const std::shared_ptr<MoveGroupInterface> & iface,
        const std::string & label)
    {
        if (cancellationRequested()) {
            return false;
        }

        MoveGroupInterface::Plan plan;

        const bool success =
            (iface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR_STREAM(get_logger(), "Planning failed: " << label);
            return false;
        }

        if (cancellationRequested()) {
            return false;
        }

        const auto exec_result = iface->execute(plan);
        if (cancellationRequested()) {
            return false;
        }
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
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

        bool success =
            (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_WARN_STREAM(
                get_logger(),
                "Plan failed with setPoseTarget for " << label
                    << ". Trying approximate IK.");

            arm->clearPoseTargets();
            arm->setApproximateJointValueTarget(target_pose, eef_link);
            success =
                (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }

        if (!success) {
            RCLCPP_ERROR_STREAM(get_logger(), "Planning failed: " << label);
            return false;
        }

        if (cancellationRequested()) {
            return false;
        }

        const auto exec_result = arm->execute(plan);
        if (cancellationRequested()) {
            return false;
        }
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
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
            speak("Indo para a pose de entrega " + spokenTargetName(table_pose));
            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget(table_pose);
            return planAndExecute(arm, "go " + table_pose);
        }

        publish_stage(goal_handle, "detecting_place_tag");
        speak("Procurando o alvo de entrega " + spokenTargetName(table_pose));

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
        const auto goal = goal_handle->get_goal();

        auto result =
            std::make_shared<PlaceTag::Result>();

        const auto finish_failure =
            [this, &goal_handle, &result](const std::string & message)
            {
                result->success = false;
                releaseExecutionResources();
                if (cancellationRequested() || goal_handle->is_canceling()) {
                    result->message = "Place canceled: " + message;
                    goal_handle->canceled(result);
                } else {
                    result->message = message;
                    goal_handle->abort(result);
                }
            };
        const auto finish_skip =
            [this, &goal_handle, &result](const std::string & message)
            {
                result->success = true;
                result->message = message;
                publish_stage(goal_handle, "skipped_missing_container_tag");
                speak("Nao encontrei esse bloco no container. Vou seguir para a proxima tarefa");
                releaseExecutionResources();
                goal_handle->succeed(result);
            };

        if (cancellationRequested() || goal_handle->is_canceling()) {
            finish_failure("canceled before execution started");
            return;
        }

        speak(
            "Iniciando a rotina de entregar a " +
            spokenTargetName(goal->tag_frame));

        auto arm = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
        auto gripper = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");
        setActiveInterfaces(arm, gripper);

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

        if (cancellationRequested() || goal_handle->is_canceling()) {
            finish_failure("canceled during place cycle");
            return;
        }

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

        result->success = success && state_write_success;

        result->message =
            result->success ?
            "Place completed" :
            (!success ?
            "Place failed" :
            "Place completed but failed to update container state yaml: " + state_write_error);

        if (result->success)
        {
            publish_stage(goal_handle, "done");
            speak("Entrega concluida com sucesso");
            releaseExecutionResources();
            goal_handle->succeed(result);
        }
        else
        {
            speak("Nao consegui entregar o bloco");
            finish_failure(result->message);
        }
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

        publish_stage(goal_handle, "closing_gripper");
        //speak("Fechando a garra no bloco dentro do container");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_close");
        if (!planAndExecute(gripper, "close gripper")) {
            return false;
        }

        publish_stage(goal_handle, "returning_pre_container");
        //speak("Retirando o bloco do container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, "return pre_container")) {
            return false;
        }

        publish_stage(goal_handle, "going_pegar_obj");
        //speak("Indo para a pose de transporte");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "go pegar_obj")) {
            return false;
        }


        arm->setMaxAccelerationScalingFactor(1.0);
        
        publish_stage(goal_handle, "going_table");
        //speak("Levando o bloco para o destino");
        if (!moveToPlaceTarget(arm, table_pose, goal_handle)) {
            return false;
        }
        arm->setMaxAccelerationScalingFactor(1.0);
        

        publish_stage(goal_handle, "opening_gripper_final");
        //speak("Abrindo a garra para soltar o bloco no destino");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, "open gripper final")) {
            return false;
        }

        publish_stage(goal_handle, "returning_pegar_obj_final");
        //speak("Voltando para a pose segura depois da entrega");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "return pegar_obj")) {
            return false;
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
