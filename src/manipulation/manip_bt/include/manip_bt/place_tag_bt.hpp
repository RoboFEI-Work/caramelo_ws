#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "my_robot_msgs/action/place_tag.hpp"

namespace manip_bt
{

class PlaceTagBT : public BT::StatefulActionNode
{
public:
  using PlaceTag = my_robot_msgs::action::PlaceTag;
  using GoalHandlePlaceTag = rclcpp_action::ClientGoalHandle<PlaceTag>;

  PlaceTagBT(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<PlaceTag>::SharedPtr action_client_;
  std::shared_future<GoalHandlePlaceTag::SharedPtr> goal_future_;
  std::shared_future<GoalHandlePlaceTag::WrappedResult> result_future_;
  GoalHandlePlaceTag::SharedPtr goal_handle_;
  bool goal_sent_;
  bool waiting_result_;
};

}  // namespace manip_bt
