#include "manip_bt/amt1_sort_bt.hpp"

#include <rclcpp_action/exceptions.hpp>

#include <cmath>
#include <cstdint>
#include <set>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;

namespace manip_bt
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("Amt1SortBT");
}

std::string joinIds(const std::vector<int32_t> & ids)
{
  std::string out;
  for (const int32_t id : ids) {
    out += (out.empty() ? "" : ",") + std::to_string(id);
  }
  return out.empty() ? "-" : out;
}
}  // namespace

Amt1SortBT::Amt1SortBT(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  const auto node_name =
    std::string("bt_amt1_sort_client_") +
    std::to_string(static_cast<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(this)));
  node_ = std::make_shared<rclcpp::Node>(node_name);
  // Nome ABSOLUTO, como /pick_tag e /place_tag.
  session_.client = rclcpp_action::create_client<Amt1Sort>(node_, "/amt1_sort");
}

BT::PortsList Amt1SortBT::providedPorts()
{
  return {
    BT::InputPort<std::string>("ws", "estacao (PP_1)"),
    BT::InputPort<std::string>("table_pose", "pose de mesa (Mesa10), repassada aos picks/places"),
    BT::InputPort<std::string>(
      "expected_tags", "", "ids esperados separados por virgula (vazio = tudo o que for visto)"),
    BT::InputPort<std::string>("direction", "left_to_right", "left_to_right | right_to_left"),
    BT::InputPort<int>(
      "max_onboard", 3, "buffer de containers de bordo (0..3; 0 = todos os vazios)"),
    BT::InputPort<double>(
      "timeout", 1800.0,
      "prazo total do goal (s) — estourar so AVISA; o cancel e do stage_timeout"),
    BT::InputPort<double>(
      "stage_timeout", 300.0, "silencio maximo de feedback (s) — unico gatilho de cancel"),
  };
}

std::vector<int32_t> Amt1SortBT::parseExpectedTags(const std::string & text)
{
  std::vector<int32_t> ids;
  std::set<int32_t> seen;
  std::string token;
  const auto flush = [&]() {
      if (token.empty()) {
        return;
      }
      std::size_t idx = 0;
      int value = 0;
      try {
        value = std::stoi(token, &idx, 10);
      } catch (const std::exception &) {
        throw std::runtime_error("expected_tags: token invalido '" + token + "'");
      }
      if (idx != token.size()) {
        throw std::runtime_error("expected_tags: token invalido '" + token + "'");
      }
      if (value < 0) {
        throw std::runtime_error("expected_tags: id negativo " + token);
      }
      if (!seen.insert(value).second) {
        throw std::runtime_error("expected_tags: id repetido " + token);
      }
      ids.push_back(static_cast<int32_t>(value));
      token.clear();
    };
  for (const char c : text) {
    if (c == ',' || c == ';' || c == ' ' || c == '\t' || c == '[' || c == ']') {
      flush();
    } else {
      token += c;
    }
  }
  flush();
  return ids;
}

double Amt1SortBT::paramOr(const std::string & key, double fallback) const
{
  double value = fallback;
  if (config().blackboard && config().blackboard->get(key, value)) {
    return value;
  }
  return fallback;
}

Amt1SortBT::Poll Amt1SortBT::poll(GoalHandle::WrappedResult & result_out)
{
  if (!session_.waiting_result) {
    if (session_.goal_future.valid() &&
      session_.goal_future.wait_for(0s) == std::future_status::ready)
    {
      session_.handle = session_.goal_future.get();
      if (!session_.handle) {
        return Poll::kRejected;
      }
      session_.result_future = session_.client->async_get_result(session_.handle);
      session_.waiting_result = true;
      last_progress_ = std::chrono::steady_clock::now();
    }
    return Poll::kPending;
  }
  if (!session_.result_future.valid() ||
    session_.result_future.wait_for(0s) != std::future_status::ready)
  {
    return Poll::kPending;
  }
  result_out = session_.result_future.get();
  session_.active = false;
  return Poll::kFinished;
}

void Amt1SortBT::cancel()
{
  if (!session_.active) {
    session_.reset();
    return;
  }
  // Receita da NudgeBaseBT: espera o handle (ate 1 s) e a RESPOSTA do cancel
  // (ate 0,5 s). O server pode estar com o braco no ar / cubo a bordo: um
  // cancel so enfileirado se perderia com o processo encerrando logo depois.
  if (!session_.handle && session_.goal_future.valid()) {
    if (rclcpp::spin_until_future_complete(node_, session_.goal_future, 1s) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      session_.handle = session_.goal_future.get();
    }
    if (!session_.handle) {
      RCLCPP_ERROR(
        logger(), "%s cancel: goal sem handle (rejeitado ou server sem resposta em 1 s) — "
        "NAO foi possivel cancelar; confira o braco.", tag().c_str());
    }
  }
  if (session_.handle) {
    try {
      auto cancel_future = session_.client->async_cancel_goal(session_.handle);
      if (rclcpp::spin_until_future_complete(node_, cancel_future, 500ms) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          logger(), "%s cancel: server nao confirmou o cancelamento em 0,5 s — confira o "
          "braco.", tag().c_str());
      }
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      // O resultado chegou entre o spin_some e o cancel — nada a cancelar.
    }
  }
  session_.reset();
}

BT::NodeStatus Amt1SortBT::fail(const std::string & reason)
{
  RCLCPP_ERROR(logger(), "%s %s", tag().c_str(), reason.c_str());
  cancel();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Amt1SortBT::onStart()
{
  session_.reset();
  last_stage_.clear();
  ws_.clear();
  getInput("ws", ws_);
  if (ws_.empty()) {
    RCLCPP_ERROR(logger(), "[Amt1Sort] porta 'ws' ausente ou vazia.");
    return BT::NodeStatus::FAILURE;
  }

  std::string table_pose;
  getInput("table_pose", table_pose);
  if (table_pose.empty()) {
    RCLCPP_ERROR(logger(), "%s porta 'table_pose' ausente ou vazia.", tag().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::string expected_text;
  getInput("expected_tags", expected_text);
  std::vector<int32_t> expected_tags;
  try {
    expected_tags = parseExpectedTags(expected_text);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger(), "%s %s", tag().c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  std::string direction = "left_to_right";
  getInput("direction", direction);
  if (direction.empty()) {
    direction = "left_to_right";
  }
  if (direction != "left_to_right" && direction != "right_to_left") {
    RCLCPP_ERROR(
      logger(), "%s direction='%s' invalida (left_to_right | right_to_left).",
      tag().c_str(), direction.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 0..3 como no Amt1Sort.action (0 = todos os containers vazios; o no
  // resolve B = min(max_onboard, vazios) e recusa B < 2).
  int max_onboard = 3;
  getInput("max_onboard", max_onboard);
  if (max_onboard < 0 || max_onboard > 3) {
    RCLCPP_ERROR(
      logger(), "%s max_onboard=%d fora de 0..3.", tag().c_str(), max_onboard);
    return BT::NodeStatus::FAILURE;
  }

  timeout_ = 1800.0;
  getInput("timeout", timeout_);
  stage_timeout_ = 300.0;
  getInput("stage_timeout", stage_timeout_);
  if (!std::isfinite(timeout_) || timeout_ <= 0.0 || !std::isfinite(stage_timeout_) ||
    stage_timeout_ <= 0.0)
  {
    RCLCPP_ERROR(
      logger(), "%s timeout=%.1f / stage_timeout=%.1f invalidos (> 0).",
      tag().c_str(), timeout_, stage_timeout_);
    return BT::NodeStatus::FAILURE;
  }

  const double wait_timeout = paramOr("server_wait_timeout", 10.0);
  if (!session_.client->wait_for_action_server(std::chrono::duration<double>(wait_timeout))) {
    RCLCPP_ERROR(
      logger(), "%s action server /amt1_sort nao respondeu em %.1f s (amt1_sort_action_node "
      "no ar?).", tag().c_str(), wait_timeout);
    return BT::NodeStatus::FAILURE;
  }

  Amt1Sort::Goal goal;
  goal.ws = ws_;
  goal.table_pose = table_pose;
  goal.expected_tags = expected_tags;
  goal.direction = direction;
  goal.max_onboard = max_onboard;
  goal.observe_only = false;

  RCLCPP_INFO(
    logger(), "%s ordenar %zu tag(s) [%s] na mesa %s, %s, max %d a bordo; prazo %.0f s "
    "(watchdog de feedback %.0f s)...",
    tag().c_str(), expected_tags.size(), joinIds(expected_tags).c_str(), table_pose.c_str(),
    direction.c_str(), max_onboard, timeout_, stage_timeout_);

  // Watchdog: rearmado a cada feedback (o server publica current_stage como
  // heartbeat, <= 5 s). Mudanca de estagio vai para o log. O prazo total
  // (deadline_) so gera um WARN — ver onRunning.
  const auto now = std::chrono::steady_clock::now();
  last_progress_ = now;
  deadline_ = now + std::chrono::milliseconds(static_cast<int64_t>(timeout_ * 1000.0));
  deadline_warned_ = false;

  rclcpp_action::Client<Amt1Sort>::SendGoalOptions options;
  options.feedback_callback =
    [this](GoalHandle::SharedPtr, const std::shared_ptr<const Amt1Sort::Feedback> feedback) {
      last_progress_ = std::chrono::steady_clock::now();
      if (feedback && feedback->current_stage != last_stage_) {
        last_stage_ = feedback->current_stage;
        RCLCPP_INFO(
          logger(), "%s estagio: %s (%d/%d ops)", tag().c_str(),
          last_stage_.c_str(), feedback->ops_done, feedback->ops_total);
      }
    };
  session_.goal_future = session_.client->async_send_goal(goal, options);
  session_.active = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Amt1SortBT::onRunning()
{
  rclcpp::spin_some(node_);

  // Prazo total (achado 8): estourar NAO cancela — cancelar no meio de um
  // pick/place deixaria cubo a bordo / braco no ar. Avisa UMA vez e segue
  // enquanto o server mandar feedback; o unico gatilho de cancel continua
  // sendo o watchdog de feedback (stage_timeout) logo abaixo.
  const auto now = std::chrono::steady_clock::now();
  const bool past_deadline = now > deadline_;
  if (past_deadline && !deadline_warned_) {
    deadline_warned_ = true;
    RCLCPP_WARN(
      logger(), "%s prazo total de %d s esgotado (ultimo estagio: %s) — NAO cancelando: o "
      "server ainda manda feedback; o goal so sera cancelado se o feedback parar por "
      "%d s (watchdog).",
      tag().c_str(), static_cast<int>(timeout_),
      last_stage_.empty() ? "<nenhum>" : last_stage_.c_str(),
      static_cast<int>(stage_timeout_));
  }
  if (now - last_progress_ > std::chrono::duration<double>(stage_timeout_)) {
    return fail(
      "WATCHDOG: " + std::to_string(static_cast<int>(stage_timeout_)) +
      " s sem feedback do /amt1_sort (ultimo estagio: " +
      (last_stage_.empty() ? std::string("<nenhum>") : last_stage_) +
      ") — server pendurado. Cancelando o goal." +
      (past_deadline ?
      " Prazo total de " + std::to_string(static_cast<int>(timeout_)) +
      " s tambem ja estava esgotado." : std::string("")));
  }

  GoalHandle::WrappedResult result;
  const Poll status = poll(result);
  if (status == Poll::kPending) {
    return BT::NodeStatus::RUNNING;
  }
  if (status == Poll::kRejected) {
    RCLCPP_ERROR(logger(), "%s goal /amt1_sort REJEITADO pelo server.", tag().c_str());
    session_.reset();
    return BT::NodeStatus::FAILURE;
  }

  const auto & res = result.result;
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    const char * code = result.code == rclcpp_action::ResultCode::CANCELED ? "CANCELADO" :
      result.code == rclcpp_action::ResultCode::ABORTED ? "ABORTADO" : "DESCONHECIDO";
    RCLCPP_ERROR(
      logger(), "%s goal /amt1_sort %s: %s (fail_reason=%s)", tag().c_str(), code,
      res ? res->message.c_str() : "<sem result>",
      res ? res->fail_reason.c_str() : "-");
    session_.reset();
    return BT::NodeStatus::FAILURE;
  }
  if (!res) {
    session_.reset();
    return fail("resultado vazio do /amt1_sort.");
  }

  const std::string summary =
    "ordem inicial [" + joinIds(res->observed_order) + "] -> final [" +
    joinIds(res->final_order) + "]; " + std::to_string(res->picks) + " pick(s), " +
    std::to_string(res->places) + " place(s), " + std::to_string(res->nudges) +
    " nudge(s), deslocamento total " +
    std::to_string(static_cast<double>(res->total_shift_m)).substr(0, 5) + " m";
  if (res->success) {
    RCLCPP_INFO(logger(), "%s ORDENACAO CONCLUIDA: %s", tag().c_str(), summary.c_str());
  } else {
    // Mesa consistente mas nao totalmente ordenada (ou nada feito): a missao
    // segue para o FINISH — quem decide e o operador lendo o log.
    RCLCPP_WARN(
      logger(), "%s ordenacao INCOMPLETA (%s, fail_reason=%s%s): %s%s%s",
      tag().c_str(), res->partial ? "parcial" : "nao feita", res->fail_reason.c_str(),
      res->missing_tags.empty() ? "" : (", faltaram " + joinIds(res->missing_tags)).c_str(),
      res->message.c_str(), res->message.empty() ? "" : " — ", summary.c_str());
  }
  session_.reset();
  return BT::NodeStatus::SUCCESS;
}

void Amt1SortBT::onHalted()
{
  cancel();
  rclcpp::spin_some(node_);
  RCLCPP_WARN(logger(), "%s interrompido — goal /amt1_sort cancelado.", tag().c_str());
}

}  // namespace manip_bt
