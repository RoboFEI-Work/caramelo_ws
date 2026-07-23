#include "manip_bt/go_to_named_pose_bt.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <chrono>
#include <cstdint>

namespace
{

void ensureRobotDescription(
  const rclcpp::Node::SharedPtr & node,
  const std::string & source_node,
  const std::string & parameter_name,
  const std::chrono::milliseconds timeout)
{
  if (node->has_parameter(parameter_name)) {
    return;
  }

  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, source_node);
  if (!client->wait_for_service(timeout)) {
    RCLCPP_WARN(
      node->get_logger(),
      "Parameter service from '%s' not available within %.2f s while fetching '%s'",
      source_node.c_str(),
      std::chrono::duration<double>(timeout).count(),
      parameter_name.c_str());
    return;
  }

  auto parameters = client->get_parameters({parameter_name});
  if (parameters.empty()) {
    return;
  }

  const auto & param = parameters.front();
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    return;
  }

  const auto robot_description = param.as_string();
  if (robot_description.empty()) {
    return;
  }

  node->declare_parameter<std::string>(parameter_name, robot_description);
}

}  // namespace

namespace manip_bt
{

GoToNamedPoseBT::GoToNamedPoseBT(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  const auto node_name =
    std::string("bt_go_to_named_pose_") +
    std::to_string(static_cast<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(this)));

  node_ = std::make_shared<rclcpp::Node>(node_name);

  const auto source_move_group_node =
    node_->declare_parameter<std::string>("move_group_node", "/move_group");
  const auto robot_description_parameter =
    node_->declare_parameter<std::string>("robot_description_parameter", "robot_description");
  const auto pose_reference_frame =
    node_->declare_parameter<std::string>("pose_reference_frame", "");
  const auto robot_description_wait_ms =
    node_->declare_parameter<int>("robot_description_wait_ms", 10000);
  const auto wait_ms = robot_description_wait_ms > 0 ? robot_description_wait_ms : 0;

  ensureRobotDescription(
    node_,
    source_move_group_node,
    robot_description_parameter,
    std::chrono::milliseconds(wait_ms));

  arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
  if (pose_reference_frame.empty()) {
    arm_->setPoseReferenceFrame(arm_->getPlanningFrame());
  } else {
    arm_->setPoseReferenceFrame(pose_reference_frame);
  }
  arm_->setPlanningTime(15.0);
  arm_->setNumPlanningAttempts(20);
  arm_->setMaxVelocityScalingFactor(1.0);
  arm_->setMaxAccelerationScalingFactor(1.0);
}

BT::PortsList GoToNamedPoseBT::providedPorts()
{
  return {
    BT::InputPort<std::string>("pose_name")
  };
}

BT::NodeStatus GoToNamedPoseBT::tick()
{
  std::string pose_name;
  if (!getInput("pose_name", pose_name)) {
    RCLCPP_ERROR(rclcpp::get_logger("GoToNamedPoseBT"), "Missing input port: pose_name");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("GoToNamedPoseBT"),
    "Moving arm to named pose: %s",
    pose_name.c_str());

  arm_->setStartStateToCurrentState();
  arm_->setNamedTarget(pose_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = arm_->plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GoToNamedPoseBT"),
      "Planning failed for named pose: %s",
      pose_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto exec_result = arm_->execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GoToNamedPoseBT"),
      "Execution failed for named pose: %s",
      pose_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("GoToNamedPoseBT"),
    "Reached named pose: %s",
    pose_name.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace manip_bt
