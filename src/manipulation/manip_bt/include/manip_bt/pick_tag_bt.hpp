#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "my_robot_msgs/action/pick_tag.hpp"

namespace manip_bt
{

class PickTagBT : public BT::StatefulActionNode
{
public:
  using PickTag = my_robot_msgs::action::PickTag;
  using GoalHandlePickTag = rclcpp_action::ClientGoalHandle<PickTag>;

  PickTagBT(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<PickTag>::SharedPtr action_client_;
  std::shared_future<GoalHandlePickTag::SharedPtr> goal_future_;
  std::shared_future<GoalHandlePickTag::WrappedResult> result_future_;
  GoalHandlePickTag::SharedPtr goal_handle_;
  bool goal_sent_;
  bool waiting_result_;
  // Watchdog de progresso (auditoria 2026-08-07, item 1.5): rearmado a cada
  // feedback de stage do server — acao saudavel publica stage continuamente
  // e nunca e cancelada; silencio prolongado = server pendurado -> FAILURE.
  std::chrono::steady_clock::time_point last_progress_;
  // 2026-08-28 (fila de alcance): valor enviado em goal.final_attempt, so
  // para o log do resultado. Porta ausente => true (XML antigo inalterado).
  bool final_attempt_{true};
  std::string tag_frame_;
};

}  // namespace manip_bt
