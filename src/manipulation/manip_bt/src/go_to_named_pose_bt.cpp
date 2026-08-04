#include "manip_bt/go_to_named_pose_bt.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>

namespace
{

// Cache compartilhado entre as instancias (prologo/home/epilogo): o
// robot_description e' estatico durante a missao — buscar 1x basta.
std::mutex g_description_mutex;
std::string g_cached_robot_description;

// Busca o robot_description do move_group SEM pendurar mudo: a versao antiga
// usava get_parameters SINCRONO sem timeout — se o move_group estivesse
// ocupado inicializando, ficava minutos em silencio total (e, com o stack
// desligado, o MoveGroupInterface travava PARA SEMPRE). Agora: espera em
// janelas com log de progresso, timeout total e retorno de erro.
bool ensureRobotDescription(
  const rclcpp::Node::SharedPtr & node,
  const std::string & source_node,
  const std::string & parameter_name,
  const std::chrono::milliseconds timeout)
{
  if (node->has_parameter(parameter_name)) {
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(g_description_mutex);
    if (!g_cached_robot_description.empty()) {
      node->declare_parameter<std::string>(parameter_name, g_cached_robot_description);
      return true;
    }
  }

  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + timeout;
  const auto elapsed_s = [&start]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    };

  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, source_node);
  while (!client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok() || std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Servico de parametros de '%s' nao apareceu em %.0f s.",
        source_node.c_str(), elapsed_s());
      return false;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Aguardando o servico de parametros de '%s'... (%.0f s)",
      source_node.c_str(), elapsed_s());
  }

  auto future = client->get_parameters({parameter_name});
  while (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(2)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (!rclcpp::ok() || std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        node->get_logger(),
        "'%s' nao respondeu '%s' em %.0f s.",
        source_node.c_str(), parameter_name.c_str(), elapsed_s());
      return false;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Aguardando '%s' servir '%s'... (%.0f s — o MoveIt pode levar um tempo "
      "para terminar de subir)",
      source_node.c_str(), parameter_name.c_str(), elapsed_s());
  }

  const auto parameters = future.get();
  if (parameters.empty() ||
    parameters.front().get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "'%s' nao tem o parametro string '%s'.",
      source_node.c_str(), parameter_name.c_str());
    return false;
  }

  const auto robot_description = parameters.front().as_string();
  if (robot_description.empty()) {
    RCLCPP_ERROR(
      node->get_logger(), "'%s' devolveu '%s' VAZIO.",
      source_node.c_str(), parameter_name.c_str());
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(g_description_mutex);
    g_cached_robot_description = robot_description;
  }
  node->declare_parameter<std::string>(parameter_name, robot_description);
  RCLCPP_INFO(
    node->get_logger(),
    "robot_description obtido de '%s' em %.0f s.",
    source_node.c_str(), elapsed_s());
  return true;
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
    node_->declare_parameter<int>("robot_description_wait_ms", 120000);
  const auto wait_ms = robot_description_wait_ms > 0 ? robot_description_wait_ms : 0;

  if (!ensureRobotDescription(
      node_,
      source_move_group_node,
      robot_description_parameter,
      std::chrono::milliseconds(wait_ms)))
  {
    // Sem descricao, o MoveGroupInterface abaixo bloquearia PARA SEMPRE.
    // Melhor falhar alto: o try/catch do executor imprime e sai limpo.
    throw std::runtime_error(
            "GoToNamedPose: nao obtive robot_description de '" + source_move_group_node +
            "' — o stack de manipulacao (move_group) esta' no ar? "
            "(ros2 launch caramelo_bringup robot_manipulation.launch.py ...)");
  }

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
    if (pose_name != "home") {
      RCLCPP_WARN(
        rclcpp::get_logger("GoToNamedPoseBT"),
        "Planning failed for named pose: %s. Retrying from safe home fallback.",
        pose_name.c_str());

      arm_->setStartStateToCurrentState();
      arm_->setNamedTarget("home");
      const auto home_plan_result = arm_->plan(plan);
      if (home_plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          rclcpp::get_logger("GoToNamedPoseBT"),
          "Planning failed for named pose: %s and fallback home also failed.",
          pose_name.c_str());
        return BT::NodeStatus::FAILURE;
      }

      const auto home_exec_result = arm_->execute(plan);
      if (home_exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          rclcpp::get_logger("GoToNamedPoseBT"),
          "Execution failed for fallback named pose: home");
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(
        rclcpp::get_logger("GoToNamedPoseBT"),
        "Reached named pose: home via fallback recovery from %s",
        pose_name.c_str());
      return BT::NodeStatus::SUCCESS;
    }

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
