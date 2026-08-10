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

  // Item 3.1 da auditoria: uma unica tentativa de plan/execute reprovava a
  // pose por causas transitorias (estado do braco velho, colisao fantasma no
  // primeiro plano). Agora sao ate 3 tentativas, com guarda de estado ANTES
  // de cada uma — sem ela o replan reusa o mesmo start state podre e falha
  // igual.
  const auto try_named_pose =
    [this](const std::string & name, int attempts) -> bool
    {
      for (int attempt = 1; attempt <= attempts; ++attempt) {
        if (!arm_->getCurrentState(2.0)) {
          RCLCPP_WARN(
            rclcpp::get_logger("GoToNamedPoseBT"),
            "Sem estado atual do braco ao ir para '%s' (tentativa %d/%d)",
            name.c_str(), attempt, attempts);
          continue;
        }

        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(
            rclcpp::get_logger("GoToNamedPoseBT"),
            "Planejamento falhou para '%s' (tentativa %d/%d)",
            name.c_str(), attempt, attempts);
          continue;
        }

        if (arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(
            rclcpp::get_logger("GoToNamedPoseBT"),
            "Execucao falhou para '%s' (tentativa %d/%d)",
            name.c_str(), attempt, attempts);
          continue;
        }

        return true;
      }
      return false;
    };

  if (try_named_pose(pose_name, 3)) {
    RCLCPP_INFO(
      rclcpp::get_logger("GoToNamedPoseBT"),
      "Reached named pose: %s",
      pose_name.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("GoToNamedPoseBT"),
    "Pose '%s' falhou nas 3 tentativas.",
    pose_name.c_str());

  // Recolher para home e SEGURANCA, nao sucesso: o codigo antigo retornava
  // SUCCESS aqui e a missao seguia acreditando que o braco estava na pose
  // pedida (item 3.1 — nunca fingir sucesso). Quem chama decide o que fazer
  // com a falha; o epilogo do task_planner segue para o FINISH mesmo assim.
  if (pose_name != "home") {
    if (try_named_pose("home", 2)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("GoToNamedPoseBT"),
        "Braco recolhido para home apos falhar em '%s' — reportando FAILURE "
        "porque ele NAO esta na pose pedida.",
        pose_name.c_str());
    } else {
      RCLCPP_FATAL(
        rclcpp::get_logger("GoToNamedPoseBT"),
        "Nem '%s' nem home foram alcancados — braco em pose imprevisivel, "
        "intervencao manual recomendada antes de seguir.",
        pose_name.c_str());
    }
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace manip_bt
