#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <atomic>
#include <cmath>
#include <cstdlib>
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
        use_camera_alignment_retry_ =
            this->declare_parameter<bool>("use_camera_alignment_retry", true);

        effort_subscription_ =
            this->create_subscription<control_msgs::msg::DynamicJointState>(
                "/dynamic_joint_states",
                20,
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
    bool use_camera_alignment_retry_{true};
    bool skip_failed_pick_after_retries_{true};
    double active_goal_position_tolerance_{0.003};
    double active_goal_orientation_tolerance_{0.20};
    std::string container_state_file_;
    std::unique_ptr<manip_task_execution::ContainerStateStore> container_state_store_;
    std::unique_ptr<manip_task_execution::ManipulatorExecutionLock> execution_lock_;
    std::atomic_bool cancel_requested_{false};
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
            server_.releaseExecutionResources();
        }

    private:
        PickActionServer & server_;
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
        const auto set_ik_params =
            [this, &profile](const std::string & prefix)
            {
                this->set_parameter(
                    rclcpp::Parameter(prefix + ".kinematics_solver", profile.solver));
                this->set_parameter(
                    rclcpp::Parameter(
                        prefix + ".kinematics_solver_timeout",
                        profile.timeout));
                this->set_parameter(
                    rclcpp::Parameter(prefix + ".mode", profile.mode));
                this->set_parameter(
                    rclcpp::Parameter(prefix + ".solve_type", profile.solve_type));
                this->set_parameter(
                    rclcpp::Parameter(prefix + ".position_only_ik", profile.position_only_ik));
                this->set_parameter(
                    rclcpp::Parameter(
                        prefix + ".rotation_scale",
                        profile.rotation_scale));
                this->set_parameter(
                    rclcpp::Parameter(
                        prefix + ".position_threshold",
                        profile.position_threshold));
                this->set_parameter(
                    rclcpp::Parameter(
                        prefix + ".orientation_threshold",
                        profile.orientation_threshold));
                this->set_parameter(
                    rclcpp::Parameter(
                        prefix + ".minimal_displacement_weight",
                        profile.minimal_displacement_weight));
                this->set_parameter(
                    rclcpp::Parameter(prefix + ".gd_step_size", profile.gd_step_size));
            };

        set_ik_params("robot_description_kinematics.arm");
        set_ik_params("arm");

        RCLCPP_WARN(
            this->get_logger(),
            "Using %s IK profile: solver=%s mode=%s timeout=%.3f "
            "solve_type=%s position_only_ik=%s "
            "rotation_scale=%.3f position_threshold=%.3f orientation_threshold=%.3f "
            "goal_position_tolerance=%.3f goal_orientation_tolerance=%.3f "
            "minimal_displacement_weight=%.3f gd_step_size=%.4f",
            profile_name.c_str(),
            profile.solver.c_str(),
            profile.mode.c_str(),
            profile.timeout,
            profile.solve_type.c_str(),
            profile.position_only_ik ? "true" : "false",
            profile.rotation_scale,
            profile.position_threshold,
            profile.orientation_threshold,
            profile.goal_position_tolerance,
            profile.goal_orientation_tolerance,
            profile.minimal_displacement_weight,
            profile.gd_step_size);
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

    void configureArmInterface(const std::shared_ptr<MoveGroupInterface> & arm)
    {
        arm->setPoseReferenceFrame("base_footprint");
        arm->setPlanningTime(15.0);
        arm->setNumPlanningAttempts(20);
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(1.0);
    }

    std::shared_ptr<MoveGroupInterface> createArmInterface(bool fallback_profile)
    {
        applyIkProfile(
            fallback_profile ? fallback_ik_profile_ : primary_ik_profile_,
            fallback_profile ? "fallback" : "primary");
        auto arm = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
        configureArmInterface(arm);
        return arm;
    }

    std::shared_ptr<MoveGroupInterface> createArmInterfaceForAttempt(int attempt)
    {
        applyIkProfile(ikProfileForAttempt(attempt), ikProfileNameForAttempt(attempt));
        auto arm = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
        configureArmInterface(arm);
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
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Planning failed: " << label);
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
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Execution failed: " << label);
            return false;
        }

        return true;
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

        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink(eef_link);

        const double x = tf.transform.translation.x;
        const double y = tf.transform.translation.y;
        const double z = tf.transform.translation.z;

        tf2::Quaternion tag_q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(tag_q).getRPY(roll, pitch, yaw);

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        if (use_orientation_constraint) {
            tf2::Quaternion desired_q;
            if (enforce_pitch_pi) {
                desired_q.setRPY(0.0, M_PI, yaw);
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
                desired_q.setRPY(current_roll, 0.0, yaw);
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

        bool success =
            (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_WARN_STREAM(
                this->get_logger(),
                "Plan falhou com setPoseTarget em " << label
                                                                                         << ". Tentando IK aproximada.");

            arm->clearPoseTargets();
            arm->setApproximateJointValueTarget(target_pose, eef_link);

            success =
                (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
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

        const auto exec_result = arm->execute(plan);

        if (cancellationRequested()) {
            return false;
        }

        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Execution failed: " << label);
            return false;
        }

        return true;
    }

    mtc::Task createApproachTask()
    {
        mtc::Task task;

        task.stages()->setName("approach task");
        task.loadRobotModel(shared_from_this());

        const auto & arm_group_name = "arm";
        const auto & hand_group_name = "gripper";
        const auto & hand_frame = "tcp";

        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        auto current_state =
            std::make_unique<mtc::stages::CurrentState>("current");
        task.add(std::move(current_state));

        auto sampling_planner =
            std::make_shared<mtc::solvers::PipelinePlanner>(
            shared_from_this());

        auto move_to_pregrasp =
            std::make_unique<mtc::stages::MoveTo>(
            "go pegar_obj",
            sampling_planner);

        move_to_pregrasp->setGroup(arm_group_name);
        move_to_pregrasp->setGoal("pegar_obj");
        task.add(std::move(move_to_pregrasp));

        return task;
    }

    bool executeTask(
        mtc::Task & task,
        const std::string & task_name)
    {
        if (cancellationRequested()) {
            return false;
        }

        try {
            task.init();
        } catch (mtc::InitStageException & e) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Task init failed [" << task_name << "]: " << e);
            return false;
        } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Task init threw exception [" << task_name << "]: " << e.what());
            return false;
        }

        try {
            if (!task.plan(6)) {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Task planning failed [" << task_name << "]");
                return false;
            }

            if (cancellationRequested()) {
                return false;
            }

            task.introspection().publishSolution(*task.solutions().front());

            const auto result = task.execute(*task.solutions().front());
            if (cancellationRequested()) {
                return false;
            }
            if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Task execution failed [" << task_name << "]");
                return false;
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Task runtime exception [" << task_name << "]: " << e.what());
            return false;
        }

        return true;
    }

    bool transferGraspedObjectToContainer(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string & container_pose,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle)
    {
        publish_stage(goal_handle, "returning_to_pegar_obj");
        speak("Bloco seguro. Voltando para a pose de transporte");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, cycle_name + " return pegar_obj")) {
            return false;
        }

        publish_stage(goal_handle, "going_pre_container");
        //speak("Indo para a pre pose do container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        if (!planAndExecute(arm, cycle_name + " pre_container")) {
            return false;
        }

        publish_stage(goal_handle, "going_container");
        speak("Levando o bloco para o " + container_pose);
        arm->setStartStateToCurrentState();
        arm->setNamedTarget(container_pose);
        if (!planAndExecute(arm, cycle_name + " " + container_pose)) {
            return false;
        }

        publish_stage(goal_handle, "opening_gripper");
        //speak("Abrindo a garra para soltar o bloco");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_open");
        if (!planAndExecute(gripper, cycle_name + " open gripper")) {
            return false;
        }

        publish_stage(goal_handle, "going pre_container_final");
        //speak("Saindo do container");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pre_container");
        return planAndExecute(arm, cycle_name + " pre_container final");
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
        constexpr double kTagYNearZero = 0.4;
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

    bool runCameraAlignmentRetry(
        std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string & tag_frame,
        const std::string & container_pose,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle)
    {
        if (!use_camera_alignment_retry_) {
            return false;
        }

        publish_stage(goal_handle, "camera_alignment_retry");
        speak("Vou realinhar a camera e tentar novamente com a primeira I K");

        arm = createArmInterface(false);
        setActiveInterfaces(arm, gripper);

        bool failed_grasp_verification = false;
        return run_pick_cycle(
            arm,
            gripper,
            tag_frame,
            container_pose,
            "CAMERA_ALIGNMENT_RETRY",
            goal_handle,
            true,
            failed_grasp_verification);
    }

    bool run_pick_cycle(
        const std::shared_ptr<MoveGroupInterface> & arm,
        const std::shared_ptr<MoveGroupInterface> & gripper,
        const std::string & tag_frame,
        const std::string & container_pose,
        const std::string & cycle_name,
        const std::shared_ptr<GoalHandlePickTag> & goal_handle,
        bool enforce_pitch_pi,
        bool & failed_grasp_verification)
    {
        failed_grasp_verification = false;
        publish_stage(goal_handle, "detecting_tag");
        speak("Procurando a tag " + spokenTagName(tag_frame));

        geometry_msgs::msg::TransformStamped tag_tf;
        if (!waitForTagTransform(
                "base_footprint",
                tag_frame,
                tag_tf,
                std::chrono::milliseconds(900),
                std::chrono::milliseconds(100),
                cycle_name + " detect_tag")) {
            speak("Não encontrei a tag " + spokenTagName(tag_frame));
            return false;
        }

        speak("Identifiquei a tag " + spokenTagName(tag_frame));
        publish_stage(goal_handle, "pre_approach");
        if (!alignCameraToTagXY(
                arm,
                tag_frame,
                goal_handle,
                cycle_name + " pre_approach_xy")) {
            return false;
        }

        if (!sleepInterruptibly(std::chrono::milliseconds(120))) {
            return false;
        }

        if (!waitForTagTransform(
                "base_footprint",
                tag_frame,
                tag_tf,
                std::chrono::milliseconds(700),
                std::chrono::milliseconds(100),
                cycle_name + " final_approach")) {
            return false;
        }

        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(0.8);
        
        publish_stage(goal_handle, "final_approach");
        //speak("Fazendo a aproximacao final da tag");
        if (!moveToTarget(
                arm,
                tag_tf,
                "tcp",
                cycle_name + " tcp final",
                true,
                enforce_pitch_pi,
                "Encontrei uma solução de I K para a tag " + spokenTagName(tag_frame),
                "Não encontrei uma solução de I K para a tag " + spokenTagName(tag_frame))) {
            return false;
        }
        arm->setMaxVelocityScalingFactor(1.0);
        arm->setMaxAccelerationScalingFactor(1.0);

        std::optional<GripperEffortSample> effort_before_close;
        if (verify_grasp_effort_) {
            effort_before_close = waitForFreshGripperEffort(
                std::chrono::milliseconds(600));
            if (!effort_before_close) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Cannot verify grasp: gripper effort telemetry is unavailable");
                speak("Não consigo verificar o esforço da garra");
                return false;
            }
        }

        publish_stage(goal_handle, "closing_gripper");
        //speak("Fechando a garra no bloco");
        gripper->setStartStateToCurrentState();
        gripper->setNamedTarget("gripper_close");
        if (!planAndExecute(gripper, cycle_name + " close gripper")) {
            return false;
        }

        if (verify_grasp_effort_) {
            publish_stage(goal_handle, "verifying_grasp");
            speak("Verificando o bloco pela forca da garra");
            if (!verifyGraspByEffort(*effort_before_close, cycle_name)) {
                failed_grasp_verification = true;
                speak("A garra não detectou o bloco");
                RCLCPP_WARN(
                    this->get_logger(),
                    "[%s] opening gripper after failed grasp verification",
                    cycle_name.c_str());
                gripper->setStartStateToCurrentState();
                gripper->setNamedTarget("gripper_open");
                (void)planAndExecute(
                    gripper,
                    cycle_name + " reopen after failed grasp");
                return false;
            }
            speak("A garra detectou o bloco");
        }

        return transferGraspedObjectToContainer(
            arm,
            gripper,
            container_pose,
            cycle_name,
            goal_handle);
    }

    void execute(const std::shared_ptr<GoalHandlePickTag> goal_handle)
    {
        ExecutionGuard execution_guard(*this);
        publishPickActive(true);
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PickTag::Result>();

        const auto finish_failure =
            [this, &goal_handle, &result](const std::string & message)
            {
                result->success = false;
                releaseExecutionResources();
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
        auto gripper = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");
        setActiveInterfaces(arm, gripper);

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

        publish_stage(goal_handle, "going_pegar_obj");
        //speak("Indo para a pose inicial de pegar");
        arm->setStartStateToCurrentState();
        arm->setEndEffectorLink("tcp");
        arm->setNamedTarget("pegar_obj");
        if (!planAndExecute(arm, "pegar_obj initial")) {
            RCLCPP_WARN(
                this->get_logger(),
                "Initial pegar_obj move failed. Retrying from home to avoid residual state between sequential picks.");

            arm->setStartStateToCurrentState();
            arm->setNamedTarget("home");
            if (!planAndExecute(arm, "home recovery before pegar_obj")) {
                finish_failure("Pick failed while recovering to home before pegar_obj");
                return;
            }

            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget("pegar_obj");
            if (!planAndExecute(arm, "pegar_obj initial after home recovery")) {
                finish_failure("Pick failed while moving to pegar_obj after home recovery");
                return;
            }
        }

        publish_stage(goal_handle, "approach_task");
        //speak("Preparando a aproximacao planejada");
        mtc::Task approach_task;
        try {
            approach_task = createApproachTask();
        } catch (const std::exception & e) {
            result->success = false;
            result->message = std::string("Approach creation failed: ") + e.what();
            finish_failure(result->message);
            return;
        }

        if (!executeTask(approach_task, "approach")) {
            publish_stage(goal_handle, "approach_task_failed_fallback");
            //speak("A aproximacao planejada falhou. Vou continuar pelo caminho alternativo");
            RCLCPP_WARN(
                this->get_logger(),
                "Approach task failed. Continuing pick cycle with fallback path.");
        }

        if (!sleepInterruptibly(std::chrono::milliseconds(200))) {
            finish_failure("canceled before the pick cycle");
            return;
        }

        const int max_pick_attempts =
            grasp_retry_attempts_ < 0 ? 1 : grasp_retry_attempts_ + 1;
        bool cycle_success = false;
        bool failed_grasp_verification = false;

        for (int attempt = 1; attempt <= max_pick_attempts; ++attempt) {
            const std::string cycle_name =
                max_pick_attempts == 1 ?
                "ACTION_CYCLE" :
                "ACTION_CYCLE_ATTEMPT_" + std::to_string(attempt);

            arm = createArmInterfaceForAttempt(attempt);
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
                attempt < 3,
                failed_grasp_verification);

            if (cycle_success) {
                break;
            }

            if (cancellationRequested() || goal_handle->is_canceling()) {
                break;
            }

            if (!failed_grasp_verification || attempt >= max_pick_attempts) {
                break;
            }

            if (attempt < max_pick_attempts) {
                publish_stage(goal_handle, "preparing_next_pick_attempt");
                speak("Vou trocar o modelo de I K para a proxima tentativa");
            }

            //speak("Vou abrir a garra e tentar pegar novamente");
            publish_stage(goal_handle, "retrying_grasp");
            RCLCPP_WARN(
                this->get_logger(),
                "Grasp verification failed on attempt %d/%d. Retrying pick.",
                attempt,
                max_pick_attempts);

            arm->setStartStateToCurrentState();
            arm->setEndEffectorLink("tcp");
            arm->setNamedTarget("pegar_obj");
            if (!planAndExecute(arm, "return pegar_obj before grasp retry")) {
                finish_failure("Pick failed while preparing grasp retry");
                return;
            }

            if (!sleepInterruptibly(std::chrono::milliseconds(100))) {
                finish_failure("canceled before grasp retry");
                return;
            }
        }

        if (cancellationRequested() || goal_handle->is_canceling()) {
            finish_failure("canceled during pick cycle");
            return;
        }

        if (cancellationRequested() || goal_handle->is_canceling()) {
            finish_failure("canceled after pick retries");
            return;
        }

        const bool success = cycle_success;

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

        result->success = success && state_write_success;
        if (result->success) {
            result->message = "Pick completed";
        } else if (!cycle_success) {
            result->message = "Pick failed";
        } else if (!state_write_success) {
            result->message =
                "Pick completed but failed to update container state yaml: " + state_write_error;
        }

        if (result->success) {
            publish_stage(goal_handle, "done");
            speak("Coleta concluida com sucesso");
            releaseExecutionResources();
            goal_handle->succeed(result);
        } else if (!cycle_success && skip_failed_pick_after_retries_) {
            publish_stage(goal_handle, "skipped_after_pick_retries");
            result->success = true;
            result->message = "Pick skipped after all retry attempts failed";
            speak("Nao consegui pegar o bloco. Vou seguir para o proximo objetivo");
            releaseExecutionResources();
            goal_handle->succeed(result);
        } else {
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
