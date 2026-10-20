#pragma once
// Folha GoToNamedPose: move o braco para uma pose nomeada da SRDF (home,
// pegar_obj...) pelo MoveGroupInterface, de forma sincrona (SyncActionNode).
// Usada no prologo (startup_home), no `home` entre estacoes e no epilogo.
// O robot_description vem do topico latched ou do servico de parametros do
// move_group (parametros do no: move_group_node, robot_description_parameter,
// pose_reference_frame, robot_description_wait_ms).

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
