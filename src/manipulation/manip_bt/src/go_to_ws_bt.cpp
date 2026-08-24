#include "manip_bt/go_to_ws_bt.hpp"

#include <rclcpp_action/exceptions.hpp>

#include <cmath>
#include <cstdint>

using namespace std::chrono_literals;

namespace manip_bt
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("GoToWSBT");
}
}  // namespace

GoToWSBT::GoToWSBT(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  const auto node_name =
    std::string("bt_go_to_ws_client_") +
    std::to_string(static_cast<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(this)));

  node_ = std::make_shared<rclcpp::Node>(node_name);
  undock_.client = rclcpp_action::create_client<UndockRobot>(node_, "/undock_robot");
  dock_.client = rclcpp_action::create_client<DockRobot>(node_, "/dock_robot");
  // Nome RELATIVO de proposito — igual ao mission_runner/dock_align_node.
  align_.client = rclcpp_action::create_client<AlignToDock>(node_, "align_to_dock");
  nav_.client = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
  // Voz (2026-08-18): a falha de estacao agora pode virar SKIP na missao —
  // o operador precisa OUVIR que a mesa foi pulada, nao so ler o log.
  speech_pub_ = node_->create_publisher<std_msgs::msg::String>("/manip/speech", 10);
}

BT::PortsList GoToWSBT::providedPorts()
{
  return {
    BT::InputPort<std::string>("ws"),
    BT::InputPort<std::string>("mesa"),
    BT::InputPort<std::string>("dock_id"),
    BT::InputPort<bool>("use_docking"),
    BT::BidirectionalPort<bool>("docked"),
    BT::BidirectionalPort<std::string>("current_dock_id"),
    BT::BidirectionalPort<std::string>("current_dock_type"),
  };
}

double GoToWSBT::paramOr(const std::string & key, double fallback) const
{
  double value = fallback;
  if (config().blackboard && config().blackboard->get(key, value)) {
    return value;
  }
  return fallback;
}

void GoToWSBT::armDeadline(double seconds)
{
  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(seconds * 1000.0));
}

bool GoToWSBT::deadlineExpired() const
{
  return std::chrono::steady_clock::now() > deadline_;
}

template<typename ActionT>
GoToWSBT::Poll GoToWSBT::poll(
  Session<ActionT> & session,
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result_out)
{
  if (!session.waiting_result) {
    if (session.goal_future.valid() &&
      session.goal_future.wait_for(0s) == std::future_status::ready)
    {
      session.handle = session.goal_future.get();
      if (!session.handle) {
        return Poll::kRejected;
      }
      session.result_future = session.client->async_get_result(session.handle);
      session.waiting_result = true;
    }
    return Poll::kPending;
  }
  if (!session.result_future.valid() ||
    session.result_future.wait_for(0s) != std::future_status::ready)
  {
    return Poll::kPending;
  }
  result_out = session.result_future.get();
  session.active = false;
  return Poll::kFinished;
}

template<typename ActionT>
void GoToWSBT::cancel(Session<ActionT> & session)
{
  if (!session.active) {
    session.reset();
    return;
  }
  // 2026-08-21 (campo): o halt chegava 0,2 s depois do send_goal, antes de o
  // handle existir — o cancel virava no-op e o Nav2 seguiu navegando por 83 s
  // depois do Ctrl-C. Agora espera o handle (ate 1 s) e a RESPOSTA do cancel
  // (ate 0,5 s) — o processo encerra logo depois do halt e um cancel so
  // enfileirado se perderia.
  if (!session.handle && session.goal_future.valid()) {
    if (rclcpp::spin_until_future_complete(node_, session.goal_future, 1s) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      session.handle = session.goal_future.get();
    }
    if (!session.handle) {
      RCLCPP_ERROR(logger(),
        "cancel: goal sem handle (rejeitado ou server sem resposta em 1 s) — "
        "NAO foi possivel cancelar; confira se o robo parou.");
    }
  }
  if (session.handle) {
    try {
      auto cancel_future = session.client->async_cancel_goal(session.handle);
      if (rclcpp::spin_until_future_complete(node_, cancel_future, 500ms) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(logger(),
          "cancel: server nao confirmou o cancelamento em 0,5 s — confira se "
          "o robo parou.");
      }
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      // O resultado chegou entre o spin_some e o cancel (goal ja terminou no
      // server) — nada a cancelar.
    }
  }
  session.reset();
}

BT::NodeStatus GoToWSBT::onStart()
{
  undock_.reset();
  dock_.reset();
  align_.reset();
  nav_.reset();
  stage_ = Stage::kIdle;

  getInput("ws", ws_);
  getInput("mesa", mesa_);
  dock_id_.clear();
  getInput("dock_id", dock_id_);
  use_docking_ = true;
  getInput("use_docking", use_docking_);
  use_opennav_approach_ = false;
  config().blackboard->get("use_opennav_approach", use_opennav_approach_);

  if (dock_id_.empty()) {
    RCLCPP_ERROR(logger(), "GoToWS sem dock_id (ws='%s') — gerador deveria ter resolvido.",
      ws_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  mission_config_ = nullptr;
  config().blackboard->get("mission_config", mission_config_);
  if (!mission_config_) {
    RCLCPP_ERROR(logger(),
      "mission_config ausente no blackboard — passe -p map_folder:=<pasta do mapa> "
      "(ou use o run_mission).");
    return BT::NodeStatus::FAILURE;
  }

  if (mesa_.empty()) {
    RCLCPP_INFO(logger(), "=== Estacao: %s (%s) ===", dock_id_.c_str(), ws_.c_str());
  } else {
    RCLCPP_INFO(logger(), "=== Estacao: %s (%s, mesa %s) ===",
      dock_id_.c_str(), ws_.c_str(), mesa_.c_str());
  }

  bool docked = false;
  getInput("docked", docked);
  undock_type_.clear();
  getInput("current_dock_type", undock_type_);
  if (docked && undock_type_.empty()) {
    std::string current_dock_id;
    getInput("current_dock_id", current_dock_id);
    if (!current_dock_id.empty()) {
      undock_type_ = mission_config_->dockType(current_dock_id);
    }
  }

  if (docked) {
    if (!startUndock()) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  if (!startDispatch()) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

bool GoToWSBT::startUndock()
{
  const double wait_timeout = paramOr("server_wait_timeout", 10.0);
  if (!undock_.client->wait_for_action_server(
      std::chrono::duration<double>(wait_timeout)))
  {
    RCLCPP_ERROR(logger(), "Action server '/undock_robot' nao respondeu.");
    return false;
  }
  UndockRobot::Goal goal;
  goal.dock_type = undock_type_;
  goal.max_undocking_time = static_cast<float>(paramOr("max_undocking_time", 30.0));
  RCLCPP_INFO(logger(), "undock (dock_type='%s')...", goal.dock_type.c_str());
  undock_.goal_future = undock_.client->async_send_goal(goal);
  undock_.active = true;
  stage_ = Stage::kUndocking;
  armDeadline(paramOr("max_undocking_time", 30.0) + 15.0);
  return true;
}

bool GoToWSBT::startDispatch()
{
  const double wait_timeout = paramOr("server_wait_timeout", 10.0);
  if (use_docking_ && !use_opennav_approach_) {
    // Fluxo novo (2026-08-18): Nav2 ate a staging, align faz o approach.
    return startStagingNav();
  }
  if (use_docking_) {
    if (!dock_.client->wait_for_action_server(
        std::chrono::duration<double>(wait_timeout)))
    {
      RCLCPP_ERROR(logger(), "Action server '/dock_robot' nao respondeu.");
      return false;
    }
    DockRobot::Goal goal;
    goal.use_dock_id = true;
    goal.dock_id = dock_id_;
    goal.navigate_to_staging_pose = true;
    goal.max_staging_time = static_cast<float>(paramOr("max_staging_time", 120.0));
    RCLCPP_INFO(logger(), "[%s] dock (navega ate staging + approach)...", dock_id_.c_str());
    dock_.goal_future = dock_.client->async_send_goal(goal);
    dock_.active = true;
    stage_ = Stage::kDocking;
    armDeadline(paramOr("max_staging_time", 120.0) + 90.0);
    return true;
  }

  // Navegacao PURA (START/FINISH): mesma semantica do mission_runner.navigate_to.
  const auto info = mission_config_->dock(dock_id_);
  if (!info) {
    RCLCPP_ERROR(logger(), "[%s] pose nao encontrada no docking.yaml.", dock_id_.c_str());
    return false;
  }
  if (info->pose_is_placeholder) {
    RCLCPP_ERROR(logger(), "[%s] pose ainda e' placeholder [0,0,0] — grave a pose real.",
      dock_id_.c_str());
    return false;
  }
  if (!nav_.client->wait_for_action_server(
      std::chrono::duration<double>(wait_timeout)))
  {
    RCLCPP_ERROR(logger(), "Action server '/navigate_to_pose' nao respondeu.");
    return false;
  }
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = info->x;
  goal.pose.pose.position.y = info->y;
  goal.pose.pose.orientation.z = std::sin(info->yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(info->yaw / 2.0);
  RCLCPP_INFO(logger(), "[%s] navegacao pura ate (%.2f, %.2f)...",
    dock_id_.c_str(), info->x, info->y);
  nav_.goal_future = nav_.client->async_send_goal(goal);
  nav_.active = true;
  stage_ = Stage::kNavigating;
  armDeadline(paramOr("nav_timeout", 180.0));
  return true;
}

bool GoToWSBT::startStagingNav()
{
  // Fluxo novo (2026-08-18): navega ate a STAGING pose (mesma conta do
  // SimpleNonChargingDock) e deixa TODO o approach para o align_to_dock com
  // muro por LiDAR — o /dock_robot uniciclo que empurrava contra a mesa so'
  // roda no rollback (use_opennav_approach=true).
  const double wait_timeout = paramOr("server_wait_timeout", 10.0);
  const auto info = mission_config_->dock(dock_id_);
  if (!info) {
    RCLCPP_ERROR(logger(), "[%s] pose nao encontrada no docking.yaml.", dock_id_.c_str());
    return false;
  }
  if (info->pose_is_placeholder) {
    RCLCPP_ERROR(logger(), "[%s] pose ainda e' placeholder [0,0,0] — grave a pose real.",
      dock_id_.c_str());
    return false;
  }
  const auto staging = mission_config_->stagingPose(dock_id_);
  if (!staging) {
    RCLCPP_ERROR(logger(), "[%s] sem staging pose.", dock_id_.c_str());
    return false;
  }
  if (!staging->staging_from_yaml) {
    RCLCPP_WARN(logger(),
      "[%s] dock_plugins sem staging_x_offset para o type '%s' — usando "
      "fallback -0.45.", dock_id_.c_str(), staging->type.c_str());
  }
  if (!nav_.client->wait_for_action_server(
      std::chrono::duration<double>(wait_timeout)))
  {
    RCLCPP_ERROR(logger(), "Action server '/navigate_to_pose' nao respondeu.");
    return false;
  }
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = staging->x;
  goal.pose.pose.position.y = staging->y;
  goal.pose.pose.orientation.z = std::sin(staging->yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(staging->yaw / 2.0);
  RCLCPP_INFO(logger(), "[%s] navegando ate a staging (%.2f, %.2f)...",
    dock_id_.c_str(), staging->x, staging->y);
  nav_.goal_future = nav_.client->async_send_goal(goal);
  nav_.active = true;
  stage_ = Stage::kStagingNav;
  armDeadline(paramOr("max_staging_time", 120.0) + 30.0);
  return true;
}

bool GoToWSBT::startAlign()
{
  const double wait_timeout = paramOr("server_wait_timeout", 10.0);
  if (!align_.client->wait_for_action_server(
      std::chrono::duration<double>(wait_timeout)))
  {
    RCLCPP_ERROR(logger(), "Action server 'align_to_dock' nao respondeu.");
    return false;
  }
  AlignToDock::Goal goal;
  goal.dock_id = dock_id_;
  goal.map_name = mission_config_->mapName();
  goal.map_dir = mission_config_->mapDir();
  goal.xy_tolerance = 0.0;   // 0 => default do dock_align_node
  goal.yaw_tolerance = 0.0;  // idem
  bool use_lidar_refine = false;
  config().blackboard->get("use_lidar_refine", use_lidar_refine);
  goal.use_lidar_refine = use_lidar_refine;
  // 30 -> 60 (2026-08-18): o align agora cobre o approach inteiro + retries.
  goal.timeout = paramOr("align_timeout", 60.0);
  RCLCPP_INFO(logger(), "[%s] alinhamento fino holonomico...", dock_id_.c_str());
  align_.goal_future = align_.client->async_send_goal(goal);
  align_.active = true;
  stage_ = Stage::kAligning;
  armDeadline(paramOr("align_timeout", 60.0) + 15.0);
  return true;
}

BT::NodeStatus GoToWSBT::finishSuccess(bool docked)
{
  setOutput("docked", docked);
  setOutput("current_dock_id", dock_id_);
  setOutput("current_dock_type", mission_config_->dockType(dock_id_));
  if (docked) {
    RCLCPP_INFO(logger(), "[%s] pronto para manipulacao.", dock_id_.c_str());
  } else {
    RCLCPP_INFO(logger(), "[%s] chegou (navegacao pura, sem dock).", dock_id_.c_str());
  }
  stage_ = Stage::kIdle;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GoToWSBT::fail(const std::string & reason)
{
  RCLCPP_ERROR(logger(), "[%s] %s", dock_id_.c_str(), reason.c_str());
  if (speech_pub_) {
    std_msgs::msg::String msg;
    msg.data = "Não consegui chegar na estação " + dock_id_;
    speech_pub_->publish(msg);
  }
  cancel(undock_);
  cancel(dock_);
  cancel(align_);
  cancel(nav_);
  stage_ = Stage::kIdle;
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GoToWSBT::onRunning()
{
  rclcpp::spin_some(node_);

  if (deadlineExpired()) {
    return fail("timeout da etapa atual — cancelando goal em voo.");
  }

  switch (stage_) {
    case Stage::kUndocking: {
        rclcpp_action::ClientGoalHandle<UndockRobot>::WrappedResult result;
        const Poll status = poll(undock_, result);
        if (status == Poll::kPending) {
          return BT::NodeStatus::RUNNING;
        }
        if (status == Poll::kRejected) {
          return fail("goal de undock rejeitado.");
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !result.result || !result.result->success)
        {
          return fail("undock falhou.");
        }
        setOutput("docked", false);
        if (!startDispatch()) {
          return fail("falha ao iniciar a proxima etapa apos o undock.");
        }
        return BT::NodeStatus::RUNNING;
      }
    case Stage::kDocking: {
        rclcpp_action::ClientGoalHandle<DockRobot>::WrappedResult result;
        const Poll status = poll(dock_, result);
        if (status == Poll::kPending) {
          return BT::NodeStatus::RUNNING;
        }
        if (status == Poll::kRejected) {
          return fail("goal de dock rejeitado.");
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !result.result || !result.result->success)
        {
          return fail("dock falhou (staging ou approach).");
        }
        bool skip_align = false;
        config().blackboard->get("skip_align", skip_align);
        if (skip_align) {
          return finishSuccess(true);
        }
        if (!startAlign()) {
          return fail("falha ao iniciar o alinhamento fino.");
        }
        return BT::NodeStatus::RUNNING;
      }
    case Stage::kStagingNav: {
        rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult result;
        const Poll status = poll(nav_, result);
        if (status == Poll::kPending) {
          return BT::NodeStatus::RUNNING;
        }
        if (status == Poll::kRejected) {
          return fail("goal de navegacao ate a staging rejeitado.");
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          return fail("navegacao ate a staging falhou.");
        }
        bool skip_align = false;
        config().blackboard->get("skip_align", skip_align);
        if (skip_align) {
          // No fluxo novo o align E' o approach: pular deixaria o robo na
          // staging, longe da mesa. Ignora o skip e alinha mesmo assim.
          RCLCPP_ERROR(logger(),
            "[%s] skip_align=true nao se aplica ao fluxo sem opennav "
            "(o align faz o approach) — ignorando.", dock_id_.c_str());
        }
        if (!startAlign()) {
          return fail("falha ao iniciar o approach/alinhamento.");
        }
        return BT::NodeStatus::RUNNING;
      }
    case Stage::kAligning: {
        rclcpp_action::ClientGoalHandle<AlignToDock>::WrappedResult result;
        const Poll status = poll(align_, result);
        if (status == Poll::kPending) {
          return BT::NodeStatus::RUNNING;
        }
        if (status == Poll::kRejected) {
          return fail("goal de align rejeitado.");
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !result.result || !result.result->success)
        {
          return fail("alinhamento fino falhou.");
        }
        RCLCPP_INFO(logger(), "[%s] alinhado (erro x=%.3f y=%.3f yaw=%.3f)",
          dock_id_.c_str(), result.result->final_x_error,
          result.result->final_y_error, result.result->final_yaw_error);
        return finishSuccess(true);
      }
    case Stage::kNavigating: {
        rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult result;
        const Poll status = poll(nav_, result);
        if (status == Poll::kPending) {
          return BT::NodeStatus::RUNNING;
        }
        if (status == Poll::kRejected) {
          return fail("goal de navegacao rejeitado.");
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          return fail("navegacao pura falhou.");
        }
        return finishSuccess(false);
      }
    case Stage::kIdle:
    default:
      return fail("estado interno invalido.");
  }
}

void GoToWSBT::onHalted()
{
  cancel(undock_);
  cancel(dock_);
  cancel(align_);
  cancel(nav_);
  rclcpp::spin_some(node_);
  stage_ = Stage::kIdle;
  RCLCPP_WARN(logger(), "GoToWS interrompido — goals de navegacao cancelados.");
}

}  // namespace manip_bt
