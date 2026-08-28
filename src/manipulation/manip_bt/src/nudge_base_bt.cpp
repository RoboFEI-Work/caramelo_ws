#include "manip_bt/nudge_base_bt.hpp"

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
  return rclcpp::get_logger("NudgeBaseBT");
}
// Abaixo disso nao vale a pena pedir ao servidor (piso do ESC ~0,10 m e o
// ReachQueue ja aplica min_shift_m); tambem cobre o "0 = nada a fazer".
constexpr double kNoShiftEps = 1e-3;
// Folga alem do timeout do goal para o server responder/cancelar.
constexpr double kDeadlineSlackSec = 10.0;
}  // namespace

NudgeBaseBT::NudgeBaseBT(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  const auto node_name =
    std::string("bt_nudge_base_client_") +
    std::to_string(static_cast<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(this)));
  node_ = std::make_shared<rclcpp::Node>(node_name);
  // Nome RELATIVO de proposito — igual ao align_to_dock do dock_align_node.
  session_.client = rclcpp_action::create_client<NudgeBase>(node_, "nudge_base");
}

BT::PortsList NudgeBaseBT::providedPorts()
{
  return {
    BT::InputPort<double>("dy", "deslocamento lateral (m, + = esquerda)"),
    BT::InputPort<double>("timeout", 20.0, "timeout do goal (s)"),
    BT::InputPort<std::string>("ws", "", "estacao (so para o log)"),
  };
}

double NudgeBaseBT::paramOr(const std::string & key, double fallback) const
{
  double value = fallback;
  if (config().blackboard && config().blackboard->get(key, value)) {
    return value;
  }
  return fallback;
}

NudgeBaseBT::Poll NudgeBaseBT::poll(GoalHandle::WrappedResult & result_out)
{
  if (!session_.waiting_result) {
    if (session_.goal_future.valid() &&
      session_.goal_future.wait_for(0s) == std::future_status::ready)
    {
      session_.handle = session_.goal_future.get();
      if (!session_.handle) {
        return Poll::kRejected;
      }
      session_.result_future = session_.client->async_get_result(session_.handle);
      session_.waiting_result = true;
    }
    return Poll::kPending;
  }
  if (!session_.result_future.valid() ||
    session_.result_future.wait_for(0s) != std::future_status::ready)
  {
    return Poll::kPending;
  }
  result_out = session_.result_future.get();
  session_.active = false;
  return Poll::kFinished;
}

void NudgeBaseBT::cancel()
{
  if (!session_.active) {
    session_.reset();
    return;
  }
  // Mesma receita do GoToWSBT (2026-08-21): espera o handle (ate 1 s) e a
  // RESPOSTA do cancel (ate 0,5 s) — o processo pode encerrar logo depois do
  // halt e um cancel so enfileirado se perderia com a base andando.
  if (!session_.handle && session_.goal_future.valid()) {
    if (rclcpp::spin_until_future_complete(node_, session_.goal_future, 1s) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      session_.handle = session_.goal_future.get();
    }
    if (!session_.handle) {
      RCLCPP_ERROR(
        logger(), "%s cancel: goal sem handle (rejeitado ou server sem resposta em 1 s) — "
        "NAO foi possivel cancelar; confira se a base parou.", tag().c_str());
    }
  }
  if (session_.handle) {
    try {
      auto cancel_future = session_.client->async_cancel_goal(session_.handle);
      if (rclcpp::spin_until_future_complete(node_, cancel_future, 500ms) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          logger(), "%s cancel: server nao confirmou o cancelamento em 0,5 s — confira se "
          "a base parou.", tag().c_str());
      }
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      // O resultado chegou entre o spin_some e o cancel — nada a cancelar.
    }
  }
  session_.reset();
}

BT::NodeStatus NudgeBaseBT::fail(const std::string & reason)
{
  RCLCPP_ERROR(logger(), "%s %s", tag().c_str(), reason.c_str());
  cancel();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus NudgeBaseBT::onStart()
{
  session_.reset();
  ws_.clear();
  getInput("ws", ws_);
  dy_ = 0.0;
  if (!getInput("dy", dy_)) {
    RCLCPP_ERROR(logger(), "%s porta 'dy' ausente ou sem valor.", tag().c_str());
    return BT::NodeStatus::FAILURE;
  }
  timeout_ = 20.0;
  getInput("timeout", timeout_);
  if (!std::isfinite(dy_)) {
    RCLCPP_ERROR(logger(), "%s dy invalido (%f).", tag().c_str(), dy_);
    return BT::NodeStatus::FAILURE;
  }
  if (std::fabs(dy_) < kNoShiftEps) {
    RCLCPP_INFO(logger(), "%s dy=%+.3f m: nada a ajustar.", tag().c_str(), dy_);
    return BT::NodeStatus::SUCCESS;
  }

  const double wait_timeout = paramOr("server_wait_timeout", 10.0);
  if (!session_.client->wait_for_action_server(std::chrono::duration<double>(wait_timeout))) {
    RCLCPP_ERROR(
      logger(), "%s action server 'nudge_base' nao respondeu em %.1f s (dock_align_node "
      "no ar?).", tag().c_str(), wait_timeout);
    return BT::NodeStatus::FAILURE;
  }

  NudgeBase::Goal goal;
  goal.dy = static_cast<float>(dy_);
  goal.timeout = static_cast<float>(timeout_ > 0.0 ? timeout_ : 0.0);
  RCLCPP_INFO(
    logger(), "%s ajuste lateral %+.2f m (%s), timeout %.0f s...",
    tag().c_str(), dy_, dy_ > 0.0 ? "esquerda" : "direita", timeout_);
  session_.goal_future = session_.client->async_send_goal(goal);
  session_.active = true;
  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>((timeout_ + kDeadlineSlackSec) * 1000.0));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NudgeBaseBT::onRunning()
{
  rclcpp::spin_some(node_);

  if (std::chrono::steady_clock::now() > deadline_) {
    return fail("timeout esperando o nudge_base — cancelando goal em voo.");
  }

  GoalHandle::WrappedResult result;
  const Poll status = poll(result);
  if (status == Poll::kPending) {
    return BT::NodeStatus::RUNNING;
  }
  if (status == Poll::kRejected) {
    return fail("goal de nudge_base REJEITADO (ocupado? dy fora da faixa?).");
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
    !result.result->success)
  {
    const std::string reason = result.result ? result.result->reason : std::string("<sem result>");
    const double travelled = result.result ? static_cast<double>(result.result->travelled) : 0.0;
    RCLCPP_ERROR(
      logger(), "%s ajuste de %+.2f m FALHOU: %s (andou %+.3f m).",
      tag().c_str(), dy_, reason.c_str(), travelled);
    session_.reset();
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(
    logger(), "%s deslocou %+.3f m (pedido %+.2f m).",
    tag().c_str(), static_cast<double>(result.result->travelled), dy_);
  session_.reset();
  return BT::NodeStatus::SUCCESS;
}

void NudgeBaseBT::onHalted()
{
  cancel();
  rclcpp::spin_some(node_);
  RCLCPP_WARN(logger(), "%s interrompido — goal de nudge_base cancelado.", tag().c_str());
}

}  // namespace manip_bt
