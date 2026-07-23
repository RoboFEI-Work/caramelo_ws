#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace manip_bt
{

class GoToNamedPoseBT : public BT::SyncActionNode
{
public:
  GoToNamedPoseBT(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
};

}  // namespace manip_bt
