#pragma once

#include <behaviortree_cpp_v3/control_node.h>

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace manip_bt
{

// FILA DE ALCANCE por estacao (nota de design, 2026-08-28).
//
// No de controle BT v3 que roda as folhas de pick/place de UMA estacao em
// ordem, adia as que voltarem "alvo visto mas fora de alcance"
// (unreachable=true no resultado), pede um ajuste LATERAL da base ao ultimo
// filho (NudgeBase) na direcao sugerida pela 1a acao adiada e refaz as
// adiadas — no maximo max_nudges vezes. Depois disso (ou se o ajuste falhar,
// ou se a sugestao for 0) roda a PASSADA FINAL com final_attempt=true, em que
// as folhas se comportam como sempre (escada completa + pular / fallback).
//
// Filhos: 0..n-2 = folhas (PickTag/PlaceTag) com as portas final_attempt,
// unreachable e suggested_base_shift_m; n-1 = NudgeBase (dy vem da saida
// shift_m deste no). A saida final_attempt e lida pelas folhas via
// blackboard ({rq_<g>_final}) — a mesma chave que elas recebem na entrada.
//
// Semantica de status: RUNNING enquanto qualquer filho roda; FAILURE do
// filho => haltChildren + FAILURE (a missao aborta como hoje); SUCCESS so
// quando a fila esvaziou (adiadas resolvidas ou pulada na passada final).
// Depois do resultado do NudgeBase (sucesso OU falha) o no devolve RUNNING e
// so tica a proxima folha no tick seguinte: um Ctrl-C que derrube o nudge e
// visto pelo executor antes de qualquer goal novo de pick/place (sem orfao).
//
// Frames: shift_m em base_footprint, + = ESQUERDA (o mesmo do
// suggested_base_shift_m dos servers e do dy do nudge_base).
class ReachQueueBT : public BT::ControlNode
{
public:
  ReachQueueBT(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  void halt() override;

  // Diagnostico (testes).
  int nudgeCount() const {return nudges_;}
  double totalShiftM() const {return total_shift_m_;}
  bool finalPass() const {return final_;}

  // sinal(sugestao) * clamp(|sugestao|, min, max); 0 se |sugestao| ~ 0.
  static double clampShift(double suggestion_m, double min_shift_m, double max_shift_m);

private:
  BT::NodeStatus tick() override;

  enum class Phase { kLeaves, kNudge };

  struct Deferred
  {
    std::size_t index;
    double suggestion_m;
  };

  void startQueue();
  void resetState();
  void requeueDeferred();
  void beginFinalPass();
  BT::NodeStatus finishSuccess();
  void readChildOutcome(const BT::TreeNode * child, bool & unreachable, double & suggestion_m) const;
  std::size_t nudgeIndex() const {return children_nodes_.size() - 1;}
  std::string tag() const {return "[ReachQueue " + ws_ + "]";}

  bool started_{false};
  bool enabled_{true};
  bool final_{false};
  Phase phase_{Phase::kLeaves};
  std::string ws_;
  int max_nudges_{2};
  double max_shift_m_{0.25};
  double min_shift_m_{0.10};
  int nudges_{0};
  double total_shift_m_{0.0};
  double pending_shift_m_{0.0};
  std::deque<std::size_t> queue_;
  std::vector<Deferred> deferred_;
};

}  // namespace manip_bt
