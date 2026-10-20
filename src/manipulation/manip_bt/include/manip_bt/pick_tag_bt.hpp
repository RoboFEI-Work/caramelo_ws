#pragma once
// Folha PickTag: cliente da action /pick_tag (mtc_pick_action_node). Manda o
// goal com tag_frame/table_pose/final_attempt, acompanha o feedback com um
// watchdog de progresso e publica nas portas de saida o veredito do server
// (unreachable, suggested_base_shift_m, skipped) para o ReachQueue decidir.

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
  // Watchdog de progresso: rearmado a cada feedback de stage do server. Acao
  // saudavel publica stage continuamente e nunca e cancelada; silencio
  // prolongado = server pendurado -> FAILURE.
  std::chrono::steady_clock::time_point last_progress_;
  // Valor enviado em goal.final_attempt (fila de alcance), so para o log do
  // resultado. Porta ausente => true (XML antigo inalterado).
  bool final_attempt_{true};
  std::string tag_frame_;
};

}  // namespace manip_bt
