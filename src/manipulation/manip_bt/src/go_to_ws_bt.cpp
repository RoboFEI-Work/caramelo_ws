#include "manip_bt/go_to_ws_bt.hpp"

#include <rclcpp_action/exceptions.hpp>

#include <cmath>
#include <cstdio>

#include "manip_bt/bt_node_name.hpp"

using namespace std::chrono_literals;

namespace manip_bt
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("go_to_ws_bt");
}

std::string fmt(const char * format, double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), format, value);
  return buf;
}
}  // namespace

GoToWSBT::GoToWSBT(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>(leafNodeName("bt_go_to_ws_", name));
  undock_.client = rclcpp_action::create_client<UndockRobot>(node_, "/undock_robot");
  dock_.client = rclcpp_action::create_client<DockRobot>(node_, "/dock_robot");
  // Nome RELATIVO de proposito: e assim que o dock_align_node anuncia a action.
  align_.client = rclcpp_action::create_client<AlignToDock>(node_, "align_to_dock");
  nav_.client = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
  // Voz: a falha de estacao pode virar SKIP na missao; o operador precisa
  // OUVIR que a mesa foi pulada, nao so ler o log.
  speech_pub_ = node_->create_publisher<std_msgs::msg::String>("/manip/speech", 10);
  // Odometria do EKF (mesma fonte do nudge_base): alimenta o detector de
  // parada. Idade pela hora de CHEGADA (relogio da Pi nao e' confiavel —
  // mesma regra do dock_align_node). SensorDataQoS casa com reliable ou
  // best-effort do publicador.
  odom_topic_ = stringParamOr("odom_topic", "/odom");
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      stop_detector_.feed(
        nowSeconds(), msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.angular.z);
    });
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

bool GoToWSBT::boolParamOr(const std::string & key, bool fallback) const
{
  bool value = fallback;
  if (config().blackboard && config().blackboard->get(key, value)) {
    return value;
  }
  return fallback;
}

int GoToWSBT::intParamOr(const std::string & key, int fallback) const
{
  int value = fallback;
  if (config().blackboard && config().blackboard->get(key, value)) {
    return value;
  }
  return fallback;
}

std::string GoToWSBT::stringParamOr(const std::string & key, const std::string & fallback) const
{
  std::string value = fallback;
  if (config().blackboard && config().blackboard->get(key, value)) {
    return value;
  }
  return fallback;
}

double GoToWSBT::nowSeconds() const
{
  return node_->get_clock()->now().nanoseconds() * 1e-9;
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

void GoToWSBT::speak(const std::string & text)
{
  if (!speech_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = text;
  speech_pub_->publish(msg);
}

// Fecha a etapa anterior (tempo gasto) e abre a nova — o resumo da estacao
// sai no fim (logStageSummary) com todos os tempos.
void GoToWSBT::markStage(const std::string & name)
{
  const double now = nowSeconds();
  if (!stage_name_.empty()) {
    stage_times_ += stage_name_ + " " + fmt("%.1f", now - stage_start_s_) + " s | ";
  }
  stage_name_ = name;
  stage_start_s_ = now;
}

void GoToWSBT::logStageSummary(const std::string & verdict)
{
  markStage("");
  RCLCPP_INFO(logger(),
    "[%s] resumo da estacao (%s): %stotal %.1f s; dockings %d; %s",
    dock_id_.c_str(), ws_.c_str(), stage_times_.c_str(),
    nowSeconds() - station_start_s_, dock_attempts_, verdict.c_str());
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
  // O halt pode chegar antes de o handle existir (o cancel viraria no-op e
  // o Nav2 seguiria navegando apos o Ctrl-C): espera o handle (ate 1 s) e a
  // RESPOSTA do cancel (ate 0,5 s), pois o processo encerra logo depois do
  // halt e um cancel so enfileirado se perderia.
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
  station_start_s_ = nowSeconds();
  stage_start_s_ = station_start_s_;
  stage_name_.clear();
  stage_times_.clear();
  dock_attempts_ = 0;
  last_align_ = AlignOutcome{};
  last_note_ = AlignNote{};

  getInput("ws", ws_);
  getInput("mesa", mesa_);
  dock_id_.clear();
  getInput("dock_id", dock_id_);
  use_docking_ = true;
  getInput("use_docking", use_docking_);
  use_opennav_approach_ = false;
  config().blackboard->get("use_opennav_approach", use_opennav_approach_);

  // Gate de pose e parada (2026-10-26): defaults justificados em
  // pose_gate.hpp; o bt_yaml_executor grava os parametros no blackboard.
  gate_enabled_ = boolParamOr("pose_gate_enabled", true);
  gate_limits_.max_abs_x = paramOr("pose_gate_max_x", 0.10);
  gate_limits_.max_abs_y = paramOr("pose_gate_max_y", 0.20);
  gate_limits_.max_abs_yaw = paramOr("pose_gate_max_yaw", 0.20);
  gate_strict_ = boolParamOr("pose_gate_strict", false);
  gate_retries_left_ = std::max(0, intParamOr("pose_gate_retries", 1));
  stop_check_enabled_ = boolParamOr("stop_check_enabled", true);
  stop_check_require_odom_ = boolParamOr("stop_check_require_odom", true);
  stop_check_timeout_s_ = paramOr("stop_check_timeout_s", 3.0);
  StopDetector::Limits lim;
  lim.max_lin = paramOr("stop_check_max_lin", 0.02);
  lim.max_ang = paramOr("stop_check_max_ang", 0.05);
  lim.window_s = paramOr("stop_check_window_s", 0.5);
  lim.max_age_s = paramOr("stop_check_odom_max_age_s", 0.5);
  stop_detector_ = StopDetector(lim);
  stop_detector_.reset(nowSeconds());

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
  if (use_docking_) {
    RCLCPP_INFO(logger(),
      "[%s] gate de pose %s: |x|<=%.2f m |y|<=%.2f m |yaw|<=%.2f rad (%.1f graus), "
      "re-dock %d x, strict=%s; parada %s: v<=%.3f m/s w<=%.3f rad/s por %.1f s "
      "(odom '%s', ate %.1f s)",
      dock_id_.c_str(), gate_enabled_ ? "ligado" : "DESLIGADO",
      gate_limits_.max_abs_x, gate_limits_.max_abs_y, gate_limits_.max_abs_yaw,
      gate_limits_.max_abs_yaw * 180.0 / M_PI, gate_retries_left_,
      gate_strict_ ? "true" : "false",
      stop_check_enabled_ ? "ligada" : "DESLIGADA", lim.max_lin, lim.max_ang,
      lim.window_s, odom_topic_.c_str(), stop_check_timeout_s_);
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
  // Sessao limpa ANTES de cada goal: no re-dock a mesma sessao e' reusada
  // na mesma execucao e o poll devolveria o result_future antigo (ja pronto).
  undock_.reset();
  undock_.goal_future = undock_.client->async_send_goal(goal);
  undock_.active = true;
  stage_ = Stage::kUndocking;
  markStage("undock");
  armDeadline(paramOr("max_undocking_time", 30.0) + 15.0);
  return true;
}

bool GoToWSBT::startDispatch()
{
  const double wait_timeout = paramOr("server_wait_timeout", 10.0);
  if (use_docking_ && !use_opennav_approach_) {
    // Fluxo padrao: Nav2 ate a staging, align faz o approach.
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
    dock_.reset();
    dock_.goal_future = dock_.client->async_send_goal(goal);
    dock_.active = true;
    stage_ = Stage::kDocking;
    markStage("dock_opennav");
    armDeadline(paramOr("max_staging_time", 120.0) + 90.0);
    return true;
  }

  // Navegacao PURA (START/FINISH): NavigateToPose ate a pose do dock, sem docking.
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
  nav_.reset();
  nav_.goal_future = nav_.client->async_send_goal(goal);
  nav_.active = true;
  stage_ = Stage::kNavigating;
  markStage("nav_pura");
  armDeadline(paramOr("nav_timeout", 180.0));
  return true;
}

bool GoToWSBT::startStagingNav()
{
  // Navega ate a STAGING pose (mesma conta do SimpleNonChargingDock) e deixa
  // TODO o approach para o align_to_dock com muro por LiDAR; o /dock_robot
  // uniciclo (que empurrava contra a mesa) so roda no rollback
  // (use_opennav_approach=true).
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
  RCLCPP_INFO(logger(), "[%s] navegando ate a staging (%.2f, %.2f, yaw %.1f graus)...",
    dock_id_.c_str(), staging->x, staging->y, staging->yaw * 180.0 / M_PI);
  nav_.reset();  // re-dock reusa a sessao: nunca herdar o result anterior
  nav_.goal_future = nav_.client->async_send_goal(goal);
  nav_.active = true;
  stage_ = Stage::kStagingNav;
  markStage("nav_staging");
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
  // Fallback 60 s so sem align_timeout no blackboard (o executor grava 90):
  // o align cobre o approach inteiro + retries.
  goal.timeout = paramOr("align_timeout", 60.0);
  ++dock_attempts_;
  RCLCPP_INFO(logger(), "[%s] alinhamento fino holonomico (docking %d)...",
    dock_id_.c_str(), dock_attempts_);
  align_.reset();  // idem: o 2o align nao pode ler o result do 1o
  align_.goal_future = align_.client->async_send_goal(goal);
  align_.active = true;
  stage_ = Stage::kAligning;
  markStage("align");
  armDeadline(paramOr("align_timeout", 60.0) + 15.0);
  return true;
}

BT::NodeStatus GoToWSBT::finishSuccess(bool docked)
{
  setOutput("docked", docked);
  setOutput("current_dock_id", dock_id_);
  setOutput("current_dock_type", mission_config_->dockType(dock_id_));
  if (docked) {
    logStageSummary(
      "pose x=" + fmt("%+.3f", last_align_.x_error) + " y=" + fmt("%+.3f", last_align_.y_error) +
      " yaw=" + fmt("%+.3f", last_align_.yaw_error) + " (" +
      yawSourceName(last_note_.yaw_source) + "); align='" + last_align_.message + "'");
    RCLCPP_INFO(logger(), "[%s] pronto para manipulacao.", dock_id_.c_str());
  } else {
    logStageSummary("chegou");
    RCLCPP_INFO(logger(), "[%s] chegou (navegacao pura, sem dock).", dock_id_.c_str());
  }
  stage_ = Stage::kIdle;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GoToWSBT::fail(const std::string & reason)
{
  RCLCPP_ERROR(logger(), "[%s] %s", dock_id_.c_str(), reason.c_str());
  logStageSummary("FALHA: " + reason);
  speak("!Nao cheguei na estacao " + dock_id_);
  cancel(undock_);
  cancel(dock_);
  cancel(align_);
  cancel(nav_);
  stage_ = Stage::kIdle;
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GoToWSBT::failAtDock(const std::string & reason)
{
  // O robo ESTA na mesa (align sucedeu): o proximo GoToWS precisa fazer o
  // undock antes de navegar, senao o Nav2 parte com o bico a 0,38 m da face
  // (zona inflada/letal => "start occupied", como na run de 22/10 14:14).
  setOutput("docked", true);
  setOutput("current_dock_id", dock_id_);
  setOutput("current_dock_type", mission_config_->dockType(dock_id_));
  return fail(reason);
}

BT::NodeStatus GoToWSBT::onAlignResult(const AlignToDock::Result & result)
{
  last_align_.success = result.success;
  last_align_.message = result.message;
  last_align_.x_error = result.final_x_error;
  last_align_.y_error = result.final_y_error;
  last_align_.yaw_error = result.final_yaw_error;
  last_note_ = classifyAlignMessage(result.message);

  RCLCPP_INFO(logger(),
    "[%s] alinhado (erro x=%.3f y=%.3f yaw=%.3f rad = %.1f graus, fonte do yaw: %s) — '%s'",
    dock_id_.c_str(), result.final_x_error, result.final_y_error, result.final_yaw_error,
    result.final_yaw_error * 180.0 / M_PI, yawSourceName(last_note_.yaw_source),
    result.message.c_str());

  if (!gate_enabled_) {
    RCLCPP_WARN(logger(), "[%s] gate de pose DESLIGADO (pose_gate_enabled=false) — "
      "manipulacao liberada sem validar x/y/yaw.", dock_id_.c_str());
    return startStopCheck();
  }

  const auto verdict = evaluatePoseGate(last_align_, gate_limits_);
  if (verdict.ok) {
    if (last_note_.yaw_out_of_tolerance || last_note_.yaw_unmeasured) {
      RCLCPP_WARN(logger(),
        "[%s] pose ACEITA COM RESSALVA: %s — dentro dos limites duros "
        "(|y|<=%.2f m, |yaw|<=%.2f rad) mas acima da tolerancia do docking.",
        dock_id_.c_str(), result.message.c_str(),
        gate_limits_.max_abs_y, gate_limits_.max_abs_yaw);
    } else {
      RCLCPP_INFO(logger(), "[%s] pose validada (x/y/yaw dentro dos limites).",
        dock_id_.c_str());
    }
    return startStopCheck();
  }

  // Fora dos limites duros.
  if (gate_retries_left_ > 0) {
    --gate_retries_left_;
    RCLCPP_WARN(logger(),
      "[%s] POSE FORA DOS LIMITES (%s): undock + nova navegacao ate a staging + "
      "align (re-dock %d restante(s) depois deste).",
      dock_id_.c_str(), verdict.reason.c_str(), gate_retries_left_);
    speak("Alinhamento ruim na estacao " + dock_id_ + ", vou encostar de novo");
    undock_type_ = mission_config_->dockType(dock_id_);
    setOutput("docked", true);
    setOutput("current_dock_id", dock_id_);
    setOutput("current_dock_type", undock_type_);
    if (!startUndock()) {
      return failAtDock("re-dock: nao consegui iniciar o undock.");
    }
    return BT::NodeStatus::RUNNING;
  }

  if (gate_strict_) {
    speak("!Estacao " + dock_id_ + " torta, nao vou manipular");
    return failAtDock(
      "POSE FORA DOS LIMITES apos " + std::to_string(dock_attempts_) +
      " docking(s) (" + verdict.reason + ") — pose_gate_strict=true: nao manipulo aqui.");
  }
  RCLCPP_ERROR(logger(),
    "[%s] ========== POSE FORA DOS LIMITES (%s) ==========\n"
    "[%s] apos %d docking(s); pose_gate_strict=false: SIGO para a manipulacao "
    "com a base torta/deslocada — conferir a pose do dock no mapa e o AMCL.",
    dock_id_.c_str(), verdict.reason.c_str(), dock_id_.c_str(), dock_attempts_);
  speak("Estacao " + dock_id_ + " torta, vou tentar assim mesmo");
  return startStopCheck();
}

BT::NodeStatus GoToWSBT::startStopCheck()
{
  if (!stop_check_enabled_) {
    return finishSuccess(true);
  }
  stop_detector_.reset(nowSeconds());
  stage_ = Stage::kStopCheck;
  markStage("parada");
  armDeadline(stop_check_timeout_s_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToWSBT::onStopCheckTick()
{
  const auto state = stop_detector_.evaluate(nowSeconds());
  if (state == StopDetector::State::kStopped) {
    RCLCPP_INFO(logger(),
      "[%s] base parada: %.1f s abaixo de v<=%.3f m/s, w<=%.3f rad/s "
      "(ultima amostra v=%.3f w=%.3f, %d amostras).",
      dock_id_.c_str(), stop_detector_.quietFor(), stop_detector_.limits().max_lin,
      stop_detector_.limits().max_ang, stop_detector_.lastLin(), stop_detector_.lastAng(),
      stop_detector_.samples());
    return finishSuccess(true);
  }
  if (!deadlineExpired()) {
    return BT::NodeStatus::RUNNING;
  }
  if (state == StopDetector::State::kNoOdom) {
    if (stop_check_require_odom_) {
      speak("!Sem odometria na estacao " + dock_id_);
      return failAtDock(
        "sem odometria fresca em '" + odom_topic_ + "' por " +
        fmt("%.1f", stop_check_timeout_s_) + " s (" +
        std::to_string(stop_detector_.samples()) +
        " amostras): nao consigo confirmar que a base parou "
        "(stop_check_require_odom=true).");
    }
    RCLCPP_WARN(logger(),
      "[%s] sem odometria fresca em '%s' (%d amostras em %.1f s): parada NAO "
      "verificada; sigo (stop_check_require_odom=false).",
      dock_id_.c_str(), odom_topic_.c_str(), stop_detector_.samples(), stop_check_timeout_s_);
    return finishSuccess(true);
  }
  // Ainda em movimento depois do align + timeout: outro publicador na
  // referencia da base (Nav2/teleop) ou ESC retendo comando — nao e' hora
  // de abaixar o braco.
  speak("!A base nao parou na estacao " + dock_id_);
  return failAtDock(
    "base AINDA EM MOVIMENTO " + fmt("%.1f", stop_check_timeout_s_) + " s apos o align "
    "(ultima amostra v=" + fmt("%.3f", stop_detector_.lastLin()) + " m/s, w=" +
    fmt("%.3f", stop_detector_.lastAng()) + " rad/s; parado ha " +
    fmt("%.2f", stop_detector_.quietFor()) + " s de " +
    fmt("%.2f", stop_detector_.limits().window_s) + " s exigidos) — quem mais publica "
    "em /mecanum_controller/reference ou /cmd_vel?");
}

BT::NodeStatus GoToWSBT::onRunning()
{
  rclcpp::spin_some(node_);

  if (stage_ == Stage::kStopCheck) {
    return onStopCheckTick();
  }

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
          // Sem align nao ha result para o gate: so' a parada e' verificada.
          last_align_ = AlignOutcome{};
          last_align_.success = true;
          last_align_.message = "skip_align";
          last_note_ = AlignNote{};
          return startStopCheck();
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
        return onAlignResult(*result.result);
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
    case Stage::kStopCheck:  // tratado acima
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
