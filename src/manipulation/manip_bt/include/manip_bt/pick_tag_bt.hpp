#pragma once

#include <behaviortree_cpp_v3/action_node.h>
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
};

}  // namespace manip_bt
