#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include "caramelo_msgs/action/align_to_dock.hpp"
#include "manip_bt/mission_map_config.hpp"
#include "manip_bt/pose_gate.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/undock_robot.hpp"

namespace manip_bt
{

// Folha GoToWS: navegacao REAL ate uma estacao (o mock para
// simulate_navigation vive no bt_yaml_executor). Sequencia:
//   [se dockado] undock -> [use_docking] dock (staging+approach) -> align fino
//                -> GATE DE POSE (x/y/yaw do result do align dentro dos
//                   limites duros; fora: re-docka uma vez, depois falha ou
//                   segue com aviso conforme pose_gate_strict)
//                -> PARADA (odometria abaixo dos limites por uma janela)
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
  // kStagingNav: NavigateToPose ate a STAGING e o align faz TODO o approach
  // com muro por LiDAR (o /dock_robot uniciclo vira rollback via
  // use_opennav_approach). kStopCheck: depois do align + gate, espera a base
  // ficar parada pela odometria antes de liberar a manipulacao.
  enum class Stage { kIdle, kUndocking, kDocking, kStagingNav, kAligning, kStopCheck, kNavigating };
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
  // Gate de pose + parada (2026-10-26). onAlignResult decide: sucesso ->
  // gate -> (re-dock | falha | aviso) -> kStopCheck ou SUCCESS.
  BT::NodeStatus onAlignResult(const AlignToDock::Result & result);
  BT::NodeStatus startStopCheck();
  BT::NodeStatus onStopCheckTick();
  // Falha COM o robo encostado na mesa: marca docked=true para o proximo
  // GoToWS fazer undock antes de navegar (o Nav2 nao parte colado na face).
  BT::NodeStatus failAtDock(const std::string & reason);
  BT::NodeStatus finishSuccess(bool docked);
  BT::NodeStatus fail(const std::string & reason);
  void speak(const std::string & text);
  void logStageSummary(const std::string & verdict);
  void markStage(const std::string & name);
  double nowSeconds() const;
  void armDeadline(double seconds);
  bool deadlineExpired() const;
  double paramOr(const std::string & key, double fallback) const;
  bool boolParamOr(const std::string & key, bool fallback) const;
  int intParamOr(const std::string & key, int fallback) const;
  std::string stringParamOr(const std::string & key, const std::string & fallback) const;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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

  // Gate de pose / parada (parametros lidos do blackboard em onStart).
  PoseGateLimits gate_limits_;
  bool gate_enabled_{true};
  bool gate_strict_{true};
  int gate_retries_left_{0};
  int dock_attempts_{0};
  AlignOutcome last_align_;
  AlignNote last_note_;
  bool stop_check_enabled_{true};
  bool stop_check_require_odom_{true};
  double stop_check_timeout_s_{3.0};
  StopDetector stop_detector_;
  std::string odom_topic_;

  // Tempos por etapa (log de diagnostico no fim da estacao).
  double station_start_s_{0.0};
  double stage_start_s_{0.0};
  std::string stage_name_;
  std::string stage_times_;
};

}  // namespace manip_bt
