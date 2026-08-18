#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <memory>
#include <string>

#include <std_msgs/msg/string.hpp>

#include "caramelo_msgs/action/align_to_dock.hpp"
#include "manip_bt/mission_map_config.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/undock_robot.hpp"

namespace manip_bt
{

// Navegacao REAL ate uma estacao (substitui o antigo SimulatedGoto).
// Reproduz a sequencia validada do mission_runner.py do caramelo_navigation:
//   [se dockado] undock -> [use_docking] dock (staging+approach) -> align fino
//   [senao]      undock -> navegacao pura ate a pose do dock (START/FINISH)
// So' retorna SUCCESS com o robo alinhado/chegado — pronto p/ pick/place.
// Falha/rejeicao/timeout de qualquer etapa cancela o goal em voo e vira FAILURE.
class GoToWSBT : public BT::StatefulActionNode
{
public:
  using DockRobot = nav2_msgs::action::DockRobot;
  using UndockRobot = nav2_msgs::action::UndockRobot;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using AlignToDock = caramelo_msgs::action::AlignToDock;

  GoToWSBT(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // kStagingNav (2026-08-18): fluxo novo — NavigateToPose ate a STAGING e o
  // align faz TODO o approach com muro por LiDAR (o /dock_robot uniciclo que
  // empurrava contra a mesa vira rollback via use_opennav_approach).
  enum class Stage { kIdle, kUndocking, kDocking, kStagingNav, kAligning, kNavigating };
  enum class Poll { kPending, kRejected, kFinished };

  template<typename ActionT>
  struct Session
  {
    typename rclcpp_action::Client<ActionT>::SharedPtr client;
    std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr> goal_future;
    std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult>
    result_future;
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle;
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

  template<typename ActionT>
  Poll poll(
    Session<ActionT> & session,
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result_out);

  template<typename ActionT>
  void cancel(Session<ActionT> & session);

  bool startUndock();
  bool startDispatch();
  bool startStagingNav();
  bool startAlign();
  BT::NodeStatus finishSuccess(bool docked);
  BT::NodeStatus fail(const std::string & reason);
  void armDeadline(double seconds);
  bool deadlineExpired() const;
  double paramOr(const std::string & key, double fallback) const;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_pub_;
  Session<UndockRobot> undock_;
  Session<DockRobot> dock_;
  Session<AlignToDock> align_;
  Session<NavigateToPose> nav_;

  MissionMapConfigPtr mission_config_;
  Stage stage_{Stage::kIdle};
  std::chrono::steady_clock::time_point deadline_;

  std::string ws_;
  std::string mesa_;
  std::string dock_id_;
  bool use_docking_{true};
  bool use_opennav_approach_{false};
  std::string undock_type_;
};

}  // namespace manip_bt
