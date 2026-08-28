#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "caramelo_msgs/action/nudge_base.hpp"

namespace manip_bt
{

// Ajuste LATERAL da base (fila de alcance, 2026-08-28).
//
// Cliente da action `nudge_base` (nome RELATIVO, como align_to_dock — servida
// pelo dock_align_node). Portas: dy (m, + = ESQUERDA em base_footprint),
// timeout (s) e ws (so para o log). |dy| < 1 mm = SUCCESS imediato sem goal.
// Este no NAO publica nada: o total acumulado (/manip/base_shift_total) e
// responsabilidade do servidor, que zera a cada align_to_dock.
// Padrao Session<> + poll/cancel copiado do GoToWSBT (halt cancela o goal e
// espera a confirmacao do server antes de voltar).
class NudgeBaseBT : public BT::StatefulActionNode
{
public:
  using NudgeBase = caramelo_msgs::action::NudgeBase;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NudgeBase>;

  NudgeBaseBT(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Poll { kPending, kRejected, kFinished };

  struct Session
  {
    rclcpp_action::Client<NudgeBase>::SharedPtr client;
    std::shared_future<GoalHandle::SharedPtr> goal_future;
    std::shared_future<GoalHandle::WrappedResult> result_future;
    GoalHandle::SharedPtr handle;
    bool waiting_result{false};
    bool active{false};

    void reset()
    {
      goal_future = {};
      result_future = {};
      handle = nullptr;
      waiting_result = false;
      active = false;
    }
  };

  Poll poll(GoalHandle::WrappedResult & result_out);
  void cancel();
  BT::NodeStatus fail(const std::string & reason);
  double paramOr(const std::string & key, double fallback) const;
  std::string tag() const {return "[NudgeBase " + ws_ + "]";}

  std::shared_ptr<rclcpp::Node> node_;
  Session session_;
  std::chrono::steady_clock::time_point deadline_;
  std::string ws_;
  double dy_{0.0};
  double timeout_{20.0};
};

}  // namespace manip_bt
