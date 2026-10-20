// Folha PickTag: ver pick_tag_bt.hpp.
#include "manip_bt/pick_tag_bt.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>

#include "manip_bt/bt_node_name.hpp"

using namespace std::chrono_literals;

namespace manip_bt
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("pick_tag_bt");
}
}  // namespace

PickTagBT::PickTagBT(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  goal_sent_(false),
  waiting_result_(false)
{
  node_ = std::make_shared<rclcpp::Node>(leafNodeName("bt_pick_tag_", name));
  action_client_ = rclcpp_action::create_client<PickTag>(node_, "/pick_tag");
}

BT::PortsList PickTagBT::providedPorts()
{
  return {
    BT::InputPort<std::string>("tag_frame"),
    // Opcional: mesa da estacao ("Mesa15", "MesaSh"...). Vazio = mesa
    // comum. A prateleira tem sequencia propria no no de pick.
    BT::InputPort<std::string>("table_pose"),
    // Fila de alcance: tentativa FINAL na estacao? Porta AUSENTE => true
    // (XML antigo = comportamento de sempre). Dentro de uma <ReachQueue> vem
    // de {rq_<g>_final} e comeca em false: alvo visto sem IK faz o server
    // sair cedo com unreachable=true.
    BT::InputPort<bool>("final_attempt", true, "tentativa final (ausente = true)"),
    // Saidas do resultado: quem decide o que fazer e o ReachQueue; este no
    // continua devolvendo SUCCESS em todo success=true.
    BT::OutputPort<bool>("unreachable"),
    BT::OutputPort<double>("suggested_base_shift_m"),
    BT::OutputPort<bool>("skipped")
  };
}

BT::NodeStatus PickTagBT::onStart()
{
  std::string tag_frame;

  if (!getInput("tag_frame", tag_frame)) {
    RCLCPP_ERROR(logger(), "Porta de entrada ausente: tag_frame");
    return BT::NodeStatus::FAILURE;
  }
  tag_frame_ = tag_frame;

  std::string table_pose;
  getInput("table_pose", table_pose);   // opcional: ausente = mesa comum

  // Porta ausente (ou sem valor no blackboard) => true: comportamento antigo.
  final_attempt_ = true;
  if (!getInput("final_attempt", final_attempt_)) {
    final_attempt_ = true;
  }

  // Zera as saidas ANTES de mandar o goal: um ReachQueue le estas chaves
  // depois do SUCCESS e nao pode herdar o resultado da passada anterior.
  // setOutput devolve erro (nao lanca) quando a porta nao esta remapeada
  // no XML — caso do XML antigo, fora da fila — e e ignorado de proposito.
  setOutput("unreachable", false);
  setOutput("suggested_base_shift_m", 0.0);
  setOutput("skipped", false);

  RCLCPP_INFO(
    logger(),
    "Enviando goal de PICK: tag_frame=%s table_pose=%s final_attempt=%s",
    tag_frame.c_str(),
    table_pose.empty() ? "<mesa comum>" : table_pose.c_str(),
    final_attempt_ ? "true" : "false");

  // Respeita o server_wait_timeout do blackboard (mesmo padrao do GoToWSBT)
  // em vez de 10 s cravados: o pick/place respawna com delay de 3 s e uma
  // missao pode querer esperar mais.
  double wait_timeout = 10.0;
  if (config().blackboard) {
    config().blackboard->get("server_wait_timeout", wait_timeout);
  }
  if (!action_client_->wait_for_action_server(
      std::chrono::duration<double>(wait_timeout)))
  {
    RCLCPP_ERROR(
      logger(),
      "Action server /pick_tag indisponivel apos %.1fs", wait_timeout);
    return BT::NodeStatus::FAILURE;
  }

  PickTag::Goal goal_msg;
  goal_msg.tag_frame = tag_frame;
  goal_msg.table_pose = table_pose;
  goal_msg.final_attempt = final_attempt_;

  // Watchdog: rearma a cada feedback de stage. Acao saudavel publica stage
  // o tempo todo e nunca e cancelada.
  last_progress_ = std::chrono::steady_clock::now();
  rclcpp_action::Client<PickTag>::SendGoalOptions options;
  options.feedback_callback =
    [this](GoalHandlePickTag::SharedPtr,
      const std::shared_ptr<const PickTag::Feedback> &) {
      last_progress_ = std::chrono::steady_clock::now();
    };

  goal_future_ = action_client_->async_send_goal(goal_msg, options);
  goal_sent_ = true;
  waiting_result_ = false;
  goal_handle_.reset();
  result_future_ = {};

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PickTagBT::onRunning()
{
  rclcpp::spin_some(node_);

  if (!goal_sent_) {
    return BT::NodeStatus::FAILURE;
  }

  // 120s sem NENHUM feedback de stage = server pendurado (perda DDS ou
  // espera infinita interna): cancela e reporta FAILURE em vez de a
  // missao ficar muda para sempre. O ciclo completo de retry do pick
  // troca de stage muito antes disso.
  constexpr auto kStageWatchdog = std::chrono::seconds(120);
  if (std::chrono::steady_clock::now() - last_progress_ > kStageWatchdog) {
    RCLCPP_FATAL(
      logger(),
      "WATCHDOG: 120s sem feedback de stage do /pick_tag — server "
      "pendurado. Cancelando o goal e falhando o no.");
    if (goal_handle_) {
      action_client_->async_cancel_goal(goal_handle_);
    }
    goal_sent_ = false;
    waiting_result_ = false;
    return BT::NodeStatus::FAILURE;
  }

  if (!waiting_result_) {
    if (goal_future_.valid() && goal_future_.wait_for(0s) == std::future_status::ready) {
      goal_handle_ = goal_future_.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(logger(), "Goal de PICK rejeitado pelo server");
        goal_sent_ = false;
        return BT::NodeStatus::FAILURE;
      }

      result_future_ = action_client_->async_get_result(goal_handle_);
      waiting_result_ = true;
      last_progress_ = std::chrono::steady_clock::now();
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
  }

  if (!result_future_.valid() || result_future_.wait_for(0s) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  const auto wrapped_result = result_future_.get();
  goal_sent_ = false;
  waiting_result_ = false;

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(
      logger(),
      "Action de PICK terminou sem SUCCEEDED. message=%s",
      (wrapped_result.result ? wrapped_result.result->message.c_str() : "<resultado vazio>"));
    return BT::NodeStatus::FAILURE;
  }

  if (!wrapped_result.result || !wrapped_result.result->success) {
    RCLCPP_ERROR(
      logger(),
      "Action de PICK reportou falha: %s",
      wrapped_result.result ? wrapped_result.result->message.c_str() : "<resultado vazio>");
    return BT::NodeStatus::FAILURE;
  }

  // Fila de alcance: publica o veredito do server nas portas de saida ANTES
  // de devolver SUCCESS. O ReachQueue le {unreachable} e
  // {suggested_base_shift_m} pelo blackboard para decidir adiar/ajustar.
  const auto & res = *wrapped_result.result;
  setOutput("unreachable", res.unreachable);
  setOutput("suggested_base_shift_m", static_cast<double>(res.suggested_base_shift_m));
  setOutput("skipped", res.skipped);

  if (res.unreachable) {
    RCLCPP_WARN(
      logger(),
      "PICK FORA DE ALCANCE (%s): alvo x=%.2f y=%.2f em manip_base_link, "
      "sugestao lateral %+.2f m%s — %s",
      tag_frame_.c_str(),
      static_cast<double>(res.target_x),
      static_cast<double>(res.target_y),
      static_cast<double>(res.suggested_base_shift_m),
      final_attempt_ ? " (tentativa final: objeto pulado)" : " (a fila de alcance decide)",
      res.message.c_str());
  }
  if (res.skipped) {
    // skip e campo do contrato (sem matching de substring); a causa vem em
    // fail_reason. Vale tambem para o skip por alcance da passada final.
    RCLCPP_WARN(
      logger(),
      "PICK PULADO (causa: %s): %s — missao continua sem este objeto.",
      res.fail_reason.c_str(),
      res.message.c_str());
  } else if (!res.unreachable) {
    RCLCPP_INFO(logger(), "PICK concluido com sucesso");
  }
  return BT::NodeStatus::SUCCESS;
}

void PickTagBT::onHalted()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_sent_ = false;
  waiting_result_ = false;
  goal_handle_.reset();
  goal_future_ = {};
  result_future_ = {};
  RCLCPP_WARN(logger(), "No de PICK interrompido (halt)");
}

}  // namespace manip_bt
