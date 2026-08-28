#include "manip_bt/reach_queue_bt.hpp"

#include <behaviortree_cpp_v3/exceptions.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace manip_bt
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("ReachQueueBT");
}
constexpr double kZeroShiftEps = 1e-6;
}  // namespace

ReachQueueBT::ReachQueueBT(const std::string & name, const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::PortsList ReachQueueBT::providedPorts()
{
  return {
    BT::InputPort<std::string>("ws", "", "estacao (so para o log)"),
    BT::InputPort<int>("max_nudges", 2, "maximo de ajustes laterais na estacao"),
    BT::InputPort<double>("max_shift_m", 0.25, "curso maximo por ajuste (m)"),
    BT::InputPort<double>("min_shift_m", 0.10, "passo minimo executavel (m)"),
    BT::InputPort<bool>("enabled", true, "false = passada unica com final_attempt=true"),
    BT::OutputPort<bool>("final_attempt", "lido pelas folhas: true na passada final"),
    BT::OutputPort<double>("shift_m", "dy do proximo NudgeBase (+ = esquerda)"),
  };
}

double ReachQueueBT::clampShift(double suggestion_m, double min_shift_m, double max_shift_m)
{
  if (!std::isfinite(suggestion_m) || std::fabs(suggestion_m) < kZeroShiftEps) {
    return 0.0;
  }
  double lo = std::fabs(min_shift_m);
  double hi = std::fabs(max_shift_m);
  if (lo > hi) {
    std::swap(lo, hi);
  }
  const double mag = std::clamp(std::fabs(suggestion_m), lo, hi);
  return std::copysign(mag, suggestion_m);
}

void ReachQueueBT::resetState()
{
  started_ = false;
  final_ = false;
  phase_ = Phase::kLeaves;
  nudges_ = 0;
  total_shift_m_ = 0.0;
  pending_shift_m_ = 0.0;
  queue_.clear();
  deferred_.clear();
}

void ReachQueueBT::startQueue()
{
  if (children_nodes_.empty()) {
    throw BT::RuntimeError("ReachQueue '", name(), "' sem filhos (esperado folhas + NudgeBase).");
  }
  const BT::TreeNode * last = children_nodes_.back();
  if (last->registrationName() != "NudgeBase") {
    throw BT::RuntimeError(
            "ReachQueue '", name(), "': o ULTIMO filho deve ser um NudgeBase (achei '",
            last->registrationName(), "').");
  }

  ws_.clear();
  getInput("ws", ws_);
  max_nudges_ = 2;
  getInput("max_nudges", max_nudges_);
  max_shift_m_ = 0.25;
  getInput("max_shift_m", max_shift_m_);
  min_shift_m_ = 0.10;
  getInput("min_shift_m", min_shift_m_);
  enabled_ = true;
  getInput("enabled", enabled_);
  if (max_nudges_ < 0) {
    max_nudges_ = 0;
  }

  resetState();
  started_ = true;
  // enabled=false: uma unica passada ja como "final" — as folhas recebem
  // final_attempt=true e se comportam como antes da fila; o NudgeBase nunca
  // e ticado.
  final_ = !enabled_;
  setOutput("final_attempt", final_);
  setOutput("shift_m", 0.0);
  for (std::size_t i = 0; i < nudgeIndex(); ++i) {
    queue_.push_back(i);
  }

  if (!enabled_) {
    RCLCPP_INFO(
      logger(), "%s desabilitada: passada unica com final_attempt=true (%zu acao(oes)).",
      tag().c_str(), queue_.size());
  } else {
    RCLCPP_INFO(
      logger(), "%s fila de alcance: %zu acao(oes), ate %d ajuste(s) lateral(is) de "
      "%.2f a %.2f m.",
      tag().c_str(), queue_.size(), max_nudges_, min_shift_m_, max_shift_m_);
  }
}

void ReachQueueBT::requeueDeferred()
{
  for (const auto & d : deferred_) {
    queue_.push_back(d.index);
  }
  deferred_.clear();
}

void ReachQueueBT::beginFinalPass()
{
  final_ = true;
  setOutput("final_attempt", true);
  RCLCPP_WARN(
    logger(), "%s PASSAGEM FINAL: %zu acao(oes) pendente(s)", tag().c_str(), deferred_.size());
  requeueDeferred();
  phase_ = Phase::kLeaves;
}

BT::NodeStatus ReachQueueBT::finishSuccess()
{
  RCLCPP_INFO(
    logger(), "%s concluida (ajustes=%d, total=%+.2f m)", tag().c_str(), nudges_,
    total_shift_m_);
  haltChildren();
  resetState();
  return BT::NodeStatus::SUCCESS;
}

void ReachQueueBT::readChildOutcome(
  const BT::TreeNode * child, bool & unreachable, double & suggestion_m) const
{
  unreachable = false;
  suggestion_m = 0.0;
  if (!child || !config().blackboard) {
    return;
  }
  // As folhas escrevem nas portas de saida unreachable / suggested_base_shift_m,
  // remapeadas para chaves do blackboard compartilhado ({action_<i>_...}).
  // Sem remapeamento (XML a mao) = tratamos como alcancavel.
  const auto & outputs = child->config().output_ports;
  const auto read_key = [&outputs](const std::string & port) -> std::string {
      const auto it = outputs.find(port);
      if (it == outputs.end()) {
        return "";
      }
      const auto remapped = BT::TreeNode::getRemappedKey(port, it->second);
      if (!remapped) {
        return "";
      }
      return std::string(remapped.value());
    };
  try {
    const std::string unreachable_key = read_key("unreachable");
    if (!unreachable_key.empty()) {
      config().blackboard->get(unreachable_key, unreachable);
    }
    const std::string shift_key = read_key("suggested_base_shift_m");
    if (!shift_key.empty()) {
      config().blackboard->get(shift_key, suggestion_m);
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      logger(), "%s nao consegui ler o resultado de %s (%s) — tratando como alcancavel.",
      tag().c_str(), child->name().c_str(), ex.what());
    unreachable = false;
    suggestion_m = 0.0;
  }
}

BT::NodeStatus ReachQueueBT::tick()
{
  if (!started_) {
    startQueue();
  }
  setStatus(BT::NodeStatus::RUNNING);

  while (true) {
    if (phase_ == Phase::kNudge) {
      BT::TreeNode * nudge = children_nodes_[nudgeIndex()];
      const BT::NodeStatus st = nudge->executeTick();
      if (st == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }
      // Rearma o NudgeBase para o proximo ajuste (SUCCESS/FAILURE -> IDLE).
      haltChild(nudgeIndex());
      if (st == BT::NodeStatus::SUCCESS) {
        ++nudges_;
        total_shift_m_ += pending_shift_m_;
        RCLCPP_INFO(
          logger(), "%s ajuste %d/%d concluido (total %+.2f m); refazendo %zu acao(oes) adiada(s).",
          tag().c_str(), nudges_, max_nudges_, total_shift_m_, deferred_.size());
        pending_shift_m_ = 0.0;
        requeueDeferred();
        phase_ = Phase::kLeaves;
      } else {
        RCLCPP_WARN(
          logger(), "%s ajuste lateral de %+.2f m FALHOU — segue para a passada final sem "
          "mover a base.", tag().c_str(), pending_shift_m_);
        pending_shift_m_ = 0.0;
        beginFinalPass();
      }
      // Devolve RUNNING de proposito (revisao 28/08): a proxima folha adiada
      // so e ticada — onStart = goal NOVO de pick/place — no tick SEGUINTE da
      // arvore. Se o nudge terminou por Ctrl-C (FAILURE do NudgeBase simulado
      // ou cancel do real), o laco principal do executor ve o g_interrupted e
      // halta a arvore ANTES de qualquer goal novo sair: sem goal orfao. Custo:
      // um tick (100 ms) entre o ajuste e a folha seguinte.
      return BT::NodeStatus::RUNNING;
    }

    // Phase::kLeaves
    if (queue_.empty()) {
      if (deferred_.empty() || final_) {
        return finishSuccess();
      }
      // Passada terminou com adiados: decide ajuste lateral ou passada final.
      const Deferred & first = deferred_.front();
      const double shift = clampShift(first.suggestion_m, min_shift_m_, max_shift_m_);
      const std::string & first_name = children_nodes_[first.index]->name();
      if (std::fabs(shift) >= kZeroShiftEps && nudges_ < max_nudges_) {
        pending_shift_m_ = shift;
        setOutput("shift_m", shift);
        RCLCPP_WARN(
          logger(), "%s AJUSTE LATERAL %d/%d: %+.2f m (%s)",
          tag().c_str(), nudges_ + 1, max_nudges_, shift, first_name.c_str());
        phase_ = Phase::kNudge;
        continue;
      }
      if (std::fabs(shift) < kZeroShiftEps) {
        RCLCPP_WARN(
          logger(), "%s sugestao 0 para %s: deslocar de lado nao resolve.",
          tag().c_str(), first_name.c_str());
      } else {
        RCLCPP_WARN(
          logger(), "%s orcamento de ajustes esgotado (%d/%d); %s ainda pede %+.2f m.",
          tag().c_str(), nudges_, max_nudges_, first_name.c_str(), shift);
      }
      beginFinalPass();
      continue;
    }

    const std::size_t idx = queue_.front();
    BT::TreeNode * child = children_nodes_[idx];
    const BT::NodeStatus st = child->executeTick();
    if (st == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    if (st != BT::NodeStatus::SUCCESS) {
      // FAILURE (ou IDLE, que um filho nunca deveria devolver): aborta como
      // uma Sequence — a missao trata igual a hoje.
      RCLCPP_ERROR(
        logger(), "%s %s falhou (%s) — abortando a estacao.",
        tag().c_str(), child->name().c_str(), BT::toStr(st).c_str());
      haltChildren();
      resetState();
      return BT::NodeStatus::FAILURE;
    }
    queue_.pop_front();

    bool unreachable = false;
    double suggestion_m = 0.0;
    readChildOutcome(child, unreachable, suggestion_m);
    if (!final_ && unreachable) {
      // Volta a IDLE: o proximo executeTick chama onStart de novo.
      haltChild(idx);
      deferred_.push_back({idx, suggestion_m});
      RCLCPP_WARN(
        logger(), "%s ADIADO %s: fora de alcance, sugestao %+.2f m",
        tag().c_str(), child->name().c_str(), suggestion_m);
    }
  }
}

void ReachQueueBT::halt()
{
  if (started_) {
    RCLCPP_WARN(logger(), "%s interrompida (halt).", tag().c_str());
  }
  resetState();
  BT::ControlNode::halt();
}

}  // namespace manip_bt
