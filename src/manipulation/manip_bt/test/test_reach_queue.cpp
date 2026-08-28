// gtest do ReachQueueBT (fila de alcance, 2026-08-28) — sem ROS no ar.
//
// Folhas roteirizadas (ScriptedLeaf, com as MESMAS portas das folhas reais) e
// um NudgeBase de mentira (StubNudge, registrado como "NudgeBase") gravam a
// sequencia de eventos numa lista global; cada caso confere a ordem e o
// status final da arvore. Eventos: "<nome>" = tentativa alcancavel,
// "<nome>?" = folha voltou unreachable, sufixo "*" = a folha viu
// final_attempt=true; "nudge(+0.20)" = ajuste pedido ao NudgeBase.
#include <gtest/gtest.h>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "manip_bt/reach_queue_bt.hpp"

namespace
{

struct LeafStep
{
  BT::NodeStatus status{BT::NodeStatus::SUCCESS};
  bool unreachable{false};
  double suggestion_m{0.0};
};

std::vector<std::string> g_events;
std::map<std::string, std::deque<LeafStep>> g_leaf_scripts;
std::deque<BT::NodeStatus> g_nudge_script;
int g_leaf_started = 0;  // onStart de folha = goal novo de pick/place
int g_leaf_halted = 0;
int g_nudge_halted = 0;

void resetGlobals()
{
  g_events.clear();
  g_leaf_scripts.clear();
  g_nudge_script.clear();
  g_leaf_started = 0;
  g_leaf_halted = 0;
  g_nudge_halted = 0;
}

std::string joinEvents()
{
  std::ostringstream out;
  for (std::size_t i = 0; i < g_events.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << g_events[i];
  }
  return out.str();
}

std::string fmt2(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%+.2f", v);
  return buf;
}

// Folha roteirizada: onStart zera as saidas (como PickTag/PlaceTag) e devolve
// RUNNING; onRunning aplica o proximo passo do roteiro da folha (default:
// alcancavel) e grava o evento. Sem roteiro sobrando = alcancavel.
class ScriptedLeaf : public BT::StatefulActionNode
{
public:
  ScriptedLeaf(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("final_attempt", true, "ausente = true"),
      BT::OutputPort<bool>("unreachable"),
      BT::OutputPort<double>("suggested_base_shift_m"),
      BT::OutputPort<bool>("skipped"),
    };
  }

  BT::NodeStatus onStart() override
  {
    ++g_leaf_started;
    final_attempt_ = true;
    if (!getInput("final_attempt", final_attempt_)) {
      final_attempt_ = true;
    }
    setOutput("unreachable", false);
    setOutput("suggested_base_shift_m", 0.0);
    setOutput("skipped", false);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    LeafStep step;
    auto & script = g_leaf_scripts[name()];
    if (!script.empty()) {
      step = script.front();
      script.pop_front();
    }
    if (step.status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;  // fica pendurada (teste de halt)
    }
    std::string event = name();
    if (step.unreachable) {
      event += "?";
    }
    if (final_attempt_) {
      event += "*";
    }
    g_events.push_back(event);
    setOutput("unreachable", step.unreachable);
    setOutput("suggested_base_shift_m", step.suggestion_m);
    setOutput("skipped", step.unreachable);
    return step.status;
  }

  void onHalted() override
  {
    ++g_leaf_halted;
  }

private:
  bool final_attempt_{true};
};

class StubNudge : public BT::StatefulActionNode
{
public:
  StubNudge(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("dy"),
      BT::InputPort<double>("timeout", 20.0, ""),
      BT::InputPort<std::string>("ws", "", ""),
    };
  }

  BT::NodeStatus onStart() override
  {
    double dy = 0.0;
    getInput("dy", dy);
    g_events.push_back("nudge(" + fmt2(dy) + ")");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    BT::NodeStatus st = BT::NodeStatus::SUCCESS;
    if (!g_nudge_script.empty()) {
      st = g_nudge_script.front();
      g_nudge_script.pop_front();
    }
    return st;
  }

  void onHalted() override
  {
    ++g_nudge_halted;
  }
};

struct Fixture
{
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard{BT::Blackboard::create()};

  Fixture()
  {
    factory.registerNodeType<ScriptedLeaf>("Leaf");
    factory.registerNodeType<StubNudge>("NudgeBase");
    factory.registerNodeType<manip_bt::ReachQueueBT>("ReachQueue");
  }

  // Mesmo XML que o bt_yaml_executor gera (chaves pre-gravadas no blackboard).
  std::string buildXml(
    const std::vector<std::string> & leaves, const std::string & extra_attrs = "",
    bool with_nudge = true)
  {
    blackboard->set("rq_final", false);
    blackboard->set("rq_shift_m", 0.0);
    std::ostringstream xml;
    xml << "<root main_tree_to_execute=\"MainTree\">\n<BehaviorTree ID=\"MainTree\">\n";
    xml << "<ReachQueue name=\"rq\" ws=\"WS_1\" max_nudges=\"2\" max_shift_m=\"0.25\" "
      "min_shift_m=\"0.10\" final_attempt=\"{rq_final}\" shift_m=\"{rq_shift_m}\"" <<
      extra_attrs << ">\n";
    for (const auto & leaf : leaves) {
      blackboard->set(leaf + "_unreachable", false);
      blackboard->set(leaf + "_shift_m", 0.0);
      blackboard->set(leaf + "_skipped", false);
      xml << "<Leaf name=\"" << leaf << "\" final_attempt=\"{rq_final}\" unreachable=\"{" <<
        leaf << "_unreachable}\" suggested_base_shift_m=\"{" << leaf <<
        "_shift_m}\" skipped=\"{" << leaf << "_skipped}\"/>\n";
    }
    if (with_nudge) {
      xml << "<NudgeBase dy=\"{rq_shift_m}\" timeout=\"20\" ws=\"WS_1\"/>\n";
    }
    xml << "</ReachQueue>\n</BehaviorTree>\n</root>\n";
    return xml.str();
  }

  BT::Tree createTree(
    const std::vector<std::string> & leaves, const std::string & extra_attrs = "",
    bool with_nudge = true)
  {
    return factory.createTreeFromText(buildXml(leaves, extra_attrs, with_nudge), blackboard);
  }

  static BT::NodeStatus run(BT::Tree & tree, int max_ticks = 10000)
  {
    BT::NodeStatus st = BT::NodeStatus::IDLE;
    for (int i = 0; i < max_ticks; ++i) {
      st = tree.tickRoot();
      if (st != BT::NodeStatus::RUNNING) {
        return st;
      }
    }
    return st;
  }

  manip_bt::ReachQueueBT * queue(BT::Tree & tree)
  {
    return dynamic_cast<manip_bt::ReachQueueBT *>(tree.rootNode());
  }
};

void script(const std::string & leaf, std::initializer_list<LeafStep> steps)
{
  for (const auto & s : steps) {
    g_leaf_scripts[leaf].push_back(s);
  }
}

LeafStep unreachable(double suggestion_m)
{
  return LeafStep{BT::NodeStatus::SUCCESS, true, suggestion_m};
}

// Tica ate o StubNudge COMECAR (evento "nudge(...)" gravado no onStart dele)
// e devolve o status desse tick; para tambem se a arvore terminar antes.
BT::NodeStatus tickUntilNudgeStarted(BT::Tree & tree, int max_ticks = 100)
{
  BT::NodeStatus st = BT::NodeStatus::IDLE;
  for (int i = 0; i < max_ticks; ++i) {
    st = tree.tickRoot();
    const bool nudge_started = !g_events.empty() && g_events.back().rfind("nudge(", 0) == 0;
    if (nudge_started || st != BT::NodeStatus::RUNNING) {
      return st;
    }
  }
  return st;
}

}  // namespace

TEST(ReachQueue, ClampShift)
{
  using manip_bt::ReachQueueBT;
  EXPECT_DOUBLE_EQ(ReachQueueBT::clampShift(0.20, 0.10, 0.25), 0.20);
  EXPECT_DOUBLE_EQ(ReachQueueBT::clampShift(0.04, 0.10, 0.25), 0.10);
  EXPECT_DOUBLE_EQ(ReachQueueBT::clampShift(0.31, 0.10, 0.25), 0.25);
  EXPECT_DOUBLE_EQ(ReachQueueBT::clampShift(-0.31, 0.10, 0.25), -0.25);
  EXPECT_DOUBLE_EQ(ReachQueueBT::clampShift(-0.04, 0.10, 0.25), -0.10);
  EXPECT_DOUBLE_EQ(ReachQueueBT::clampShift(0.0, 0.10, 0.25), 0.0);
  EXPECT_DOUBLE_EQ(ReachQueueBT::clampShift(0.60, 0.10, 0.25), 0.25);
  // min/max trocados nao explodem.
  EXPECT_DOUBLE_EQ(ReachQueueBT::clampShift(0.20, 0.25, 0.10), 0.20);
}

TEST(ReachQueue, OrderPreservedWhenAllReachable)
{
  resetGlobals();
  Fixture f;
  auto tree = f.createTree({"p1", "p2", "p3"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1,p2,p3");
  EXPECT_FALSE(f.blackboard->get<bool>("rq_final"));
}

TEST(ReachQueue, SecondUnreachableIsDeferredThenNudged)
{
  resetGlobals();
  Fixture f;
  script("p2", {unreachable(0.20)});
  auto tree = f.createTree({"p1", "p2", "p3"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1,p2?,p3,nudge(+0.20),p2");
  EXPECT_FALSE(f.blackboard->get<bool>("rq_final"));
  EXPECT_DOUBLE_EQ(f.blackboard->get<double>("rq_shift_m"), 0.20);
}

TEST(ReachQueue, NeedsSixtyCmUsesTwoNudgesThenFinalPass)
{
  resetGlobals();
  Fixture f;
  script("p2", {unreachable(0.60), unreachable(0.35), unreachable(0.10), unreachable(0.10)});
  auto tree = f.createTree({"p1", "p2", "p3"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1,p2?,p3,nudge(+0.25),p2?,nudge(+0.25),p2?,p2?*");
  EXPECT_TRUE(f.blackboard->get<bool>("rq_final"));
}

TEST(ReachQueue, SuggestionZeroGoesStraightToFinalPass)
{
  resetGlobals();
  Fixture f;
  script("p2", {unreachable(0.0), unreachable(0.0)});
  auto tree = f.createTree({"p1", "p2", "p3"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1,p2?,p3,p2?*");
  EXPECT_TRUE(f.blackboard->get<bool>("rq_final"));
}

TEST(ReachQueue, NudgeFailureLeadsToFinalPassAndSuccess)
{
  resetGlobals();
  Fixture f;
  script("p2", {unreachable(0.20)});
  g_nudge_script.push_back(BT::NodeStatus::FAILURE);
  auto tree = f.createTree({"p1", "p2", "p3"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1,p2?,p3,nudge(+0.20),p2*");
  EXPECT_TRUE(f.blackboard->get<bool>("rq_final"));
}

TEST(ReachQueue, ChildFailureFailsTheQueue)
{
  resetGlobals();
  Fixture f;
  script("p2", {LeafStep{BT::NodeStatus::FAILURE, false, 0.0}});
  auto tree = f.createTree({"p1", "p2", "p3"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(joinEvents(), "p1,p2");
  // tickRoot() do v3.8 poe a raiz em IDLE quando ela termina — o que vale e
  // o status devolvido acima; aqui so conferimos que a fila zerou.
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
}

TEST(ReachQueue, HaltPropagatesToRunningChild)
{
  resetGlobals();
  Fixture f;
  script("p2", {LeafStep{BT::NodeStatus::RUNNING, false, 0.0},
      LeafStep{BT::NodeStatus::RUNNING, false, 0.0},
      LeafStep{BT::NodeStatus::RUNNING, false, 0.0}});
  auto tree = f.createTree({"p1", "p2", "p3"});
  EXPECT_EQ(Fixture::run(tree, 4), BT::NodeStatus::RUNNING);
  EXPECT_EQ(joinEvents(), "p1");
  tree.haltTree();
  EXPECT_EQ(g_leaf_halted, 1);
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
  // Depois do halt a fila recomeca do zero.
  g_leaf_scripts.clear();
  g_events.clear();
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1,p2,p3");
}

TEST(ReachQueue, OnlyPlacesDirectionFromFirstDeferred)
{
  resetGlobals();
  Fixture f;
  script("place_1", {unreachable(-0.15)});
  script("place_2", {unreachable(-0.20)});
  auto tree = f.createTree({"place_1", "place_2"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "place_1?,place_2?,nudge(-0.15),place_1,place_2");
}

TEST(ReachQueue, DisabledRunsSinglePassAsFinal)
{
  resetGlobals();
  Fixture f;
  script("p2", {unreachable(0.20)});
  auto tree = f.createTree({"p1", "p2", "p3"}, " enabled=\"false\"");
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1*,p2?*,p3*");
  EXPECT_TRUE(f.blackboard->get<bool>("rq_final"));
}

TEST(ReachQueue, SmallSuggestionClampedToMin)
{
  resetGlobals();
  Fixture f;
  script("p1", {unreachable(0.04)});
  auto tree = f.createTree({"p1", "p2"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1?,p2,nudge(+0.10),p1");
}

TEST(ReachQueue, LargeSuggestionClampedToMax)
{
  resetGlobals();
  Fixture f;
  script("p1", {unreachable(0.31)});
  auto tree = f.createTree({"p1", "p2"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1?,p2,nudge(+0.25),p1");
}

TEST(ReachQueue, TwoNudgesWithDifferentDirections)
{
  // O 1o adiado de cada passada decide a direcao.
  resetGlobals();
  Fixture f;
  script("p1", {unreachable(0.10)});
  script("p3", {unreachable(-0.20), unreachable(-0.20)});
  auto tree = f.createTree({"p1", "p2", "p3"});
  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1?,p2,p3?,nudge(+0.10),p1,p3?,nudge(-0.20),p3");
}

// Revisao 28/08 (achado alto): depois do resultado do NudgeBase a fila tem de
// devolver RUNNING e so ticar a proxima folha adiada no tick SEGUINTE. Se
// ticasse no mesmo tick, um Ctrl-C durante o nudge (FAILURE/cancel) dispararia
// um goal novo de pick/place antes de o executor ver a interrupcao (orfao).
TEST(ReachQueue, NudgeSuccessDoesNotStartNextLeafInTheSameTick)
{
  resetGlobals();
  Fixture f;
  script("p2", {unreachable(0.20)});
  auto tree = f.createTree({"p1", "p2", "p3"});
  ASSERT_EQ(tickUntilNudgeStarted(tree), BT::NodeStatus::RUNNING);
  EXPECT_EQ(joinEvents(), "p1,p2?,p3,nudge(+0.20)");
  const int starts_before = g_leaf_started;
  EXPECT_EQ(starts_before, 3);  // p1, p2, p3

  // Tick do RESULTADO do nudge: contabilizado, fila segue RUNNING e NENHUMA
  // folha recebeu onStart neste tick.
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(f.queue(tree)->nudgeCount(), 1);
  EXPECT_EQ(g_leaf_started, starts_before);
  EXPECT_EQ(joinEvents(), "p1,p2?,p3,nudge(+0.20)");

  // So o tick seguinte dispara o onStart da folha adiada.
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(g_leaf_started, starts_before + 1);

  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1,p2?,p3,nudge(+0.20),p2");
}

TEST(ReachQueue, NudgeFailureDoesNotStartFinalPassInTheSameTick)
{
  resetGlobals();
  Fixture f;
  script("p2", {unreachable(0.20)});
  g_nudge_script.push_back(BT::NodeStatus::FAILURE);
  auto tree = f.createTree({"p1", "p2", "p3"});
  ASSERT_EQ(tickUntilNudgeStarted(tree), BT::NodeStatus::RUNNING);
  const int starts_before = g_leaf_started;
  EXPECT_EQ(starts_before, 3);

  // Tick do RESULTADO (FAILURE): passada final armada, mas sem onStart ainda.
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(f.queue(tree)->finalPass());
  EXPECT_TRUE(f.blackboard->get<bool>("rq_final"));
  EXPECT_EQ(g_leaf_started, starts_before);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(g_leaf_started, starts_before + 1);

  EXPECT_EQ(Fixture::run(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(joinEvents(), "p1,p2?,p3,nudge(+0.20),p2*");
}

TEST(ReachQueue, HaltRightAfterNudgeResultFindsNoLeafInFlight)
{
  // Cenario do Ctrl-C: o NudgeBase simulado devolve FAILURE (g_interrupted),
  // o executor ve a interrupcao no laco principal e chama haltTree(). Nada
  // pode estar em voo — nenhuma folha (goal) nem o nudge.
  resetGlobals();
  Fixture f;
  script("p2", {unreachable(0.20)});
  g_nudge_script.push_back(BT::NodeStatus::FAILURE);
  auto tree = f.createTree({"p1", "p2", "p3"});
  ASSERT_EQ(tickUntilNudgeStarted(tree), BT::NodeStatus::RUNNING);
  const int starts_before = g_leaf_started;

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  tree.haltTree();
  EXPECT_EQ(g_leaf_started, starts_before);  // nenhum goal novo saiu
  EXPECT_EQ(g_leaf_halted, 0);               // nenhuma folha estava RUNNING
  EXPECT_EQ(g_nudge_halted, 0);              // nudge ja tinha terminado
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
}

TEST(ReachQueue, LastChildMustBeNudgeBase)
{
  resetGlobals();
  Fixture f;
  auto tree = f.createTree({"p1", "p2"}, "", false);
  EXPECT_THROW(tree.tickRoot(), BT::RuntimeError);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
