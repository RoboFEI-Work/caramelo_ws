#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "my_robot_msgs/action/amt1_sort.hpp"

namespace manip_bt
{

// AMT1 — ORDENACAO das tags da mesa de precisao (nota de design, 2026-08-28).
//
// Folha cliente da action ABSOLUTA /amt1_sort (amt1_sort_action_node, pacote
// manip_task_execution). Toda a rotina (observar a mesa, planejar os ciclos,
// picks/places com stack_on nos slots, nudge da base) roda DENTRO do server:
// esta folha so manda o goal, acompanha o feedback e traduz o resultado.
//
// Portas: ws, table_pose, expected_tags (string "1,2,3" -> int32[]),
// direction (left_to_right | right_to_left), max_onboard (int, 0..3; 0 =
// todos os containers vazios), timeout (prazo total, s, default 1800) e
// stage_timeout (silencio maximo de feedback, s, default 300).
// server_wait_timeout vem do blackboard (mesmo padrao do PickTagBT).
//
// Prazos (achado 8, 2026-08-28): o prazo total so AVISA (WARN uma vez) —
// estourar no meio de um pick/place e cancelar deixaria cubo a bordo / braco
// no ar. O UNICO gatilho de cancel e o watchdog de feedback (stage_timeout):
// o server manda heartbeat <= 5 s, entao silencio = server pendurado.
//
// Status: SUCCESS sempre que o server termina com SUCCEEDED — inclusive com
// result.success=false (mesa consistente, so nao totalmente ordenada: WARN e
// a missao segue para o FINISH). FAILURE so em rejeitado, abortado,
// cancelado ou watchdog de feedback. onHalted cancela o goal e espera a
// confirmacao (receita da NudgeBaseBT), porque o server pode estar com o
// braco no ar.
class Amt1SortBT : public BT::StatefulActionNode
{
public:
  using Amt1Sort = my_robot_msgs::action::Amt1Sort;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Amt1Sort>;

  Amt1SortBT(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  // "1,2,3" / "1 2 3" / "[1, 2, 3]" -> {1,2,3}. Lanca std::runtime_error em
  // token invalido, id negativo ou id repetido. Vazio -> lista vazia.
  static std::vector<int32_t> parseExpectedTags(const std::string & text);

private:
  enum class Poll { kPending, kRejected, kFinished };

  struct Session
  {
    rclcpp_action::Client<Amt1Sort>::SharedPtr client;
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
  std::string tag() const {return "[Amt1Sort " + ws_ + "]";}

  std::shared_ptr<rclcpp::Node> node_;
  Session session_;
  std::chrono::steady_clock::time_point deadline_;
  std::chrono::steady_clock::time_point last_progress_;
  std::string ws_;
  std::string last_stage_;
  double timeout_{1800.0};
  double stage_timeout_{300.0};
  bool deadline_warned_{false};  // WARN do prazo total ja emitido neste goal
};

}  // namespace manip_bt
