// Testes do planejador puro da rotina AMT1 (2026-08-28).
//
// O oraculo e simulatePlan: executa o plano numa mesa modelo e confere os
// invariantes (nunca mais de 2 a bordo, todo PLACE em slot livre, ninguem a
// bordo no fim). As 720 permutacoes de 6 tags passam por ele com buffer 2 e
// 3. O exemplo do operador (5,2,3,1,4,6) e conferido op a op.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <string>
#include <vector>

#include "manip_task_execution/amt1_sort_planner.hpp"

using manip_task_execution::amt1::AnchorDelta;
using manip_task_execution::amt1::assignSlots;
using manip_task_execution::amt1::describeOp;
using manip_task_execution::amt1::planSort;
using manip_task_execution::amt1::registrationDelta;
using manip_task_execution::amt1::simulatePlan;
using manip_task_execution::amt1::SimulationResult;
using manip_task_execution::amt1::SlotAssignment;
using manip_task_execution::amt1::SortOp;
using manip_task_execution::amt1::SortOpType;
using manip_task_execution::amt1::SortPlan;
using manip_task_execution::amt1::TagPose;

namespace
{

const std::vector<int> kTarget{1, 2, 3, 4, 5, 6};

SortOp pick(int tag, int slot)
{
  return SortOp{SortOpType::kPick, tag, slot};
}

SortOp place(int tag, int slot)
{
  return SortOp{SortOpType::kPlace, tag, slot};
}

bool sameOp(const SortOp & a, const SortOp & b)
{
  return a.type == b.type && a.tag == b.tag && a.slot == b.slot;
}

std::string opsText(const std::vector<SortOp> & ops)
{
  std::string s;
  for (const SortOp & op : ops) {
    s += describeOp(op) + " ";
  }
  return s;
}

void expectOps(const std::vector<SortOp> & got, const std::vector<SortOp> & want)
{
  ASSERT_EQ(got.size(), want.size()) << "got: " << opsText(got) << " want: " << opsText(want);
  for (std::size_t i = 0; i < got.size(); ++i) {
    EXPECT_TRUE(sameOp(got[i], want[i]))
      << "op " << i << ": got " << describeOp(got[i]) << " want " << describeOp(want[i]);
  }
}

int misplaced(const std::vector<int> & observed, const std::vector<int> & target)
{
  int n = 0;
  for (std::size_t k = 0; k < observed.size(); ++k) {
    if (observed[k] != target[k]) {
      ++n;
    }
  }
  return n;
}

/// Confere um plano contra o oraculo e os invariantes da secao 2 do plano.
void checkPlanAgainstOracle(const std::vector<int> & observed, int buffer)
{
  const SortPlan plan = planSort(observed, kTarget, buffer);
  ASSERT_TRUE(plan.error.empty()) << plan.error;
  const SimulationResult sim = simulatePlan(observed, plan.ops, buffer);
  ASSERT_TRUE(sim.ok) << sim.error << " | " << opsText(plan.ops);
  EXPECT_EQ(sim.final_order, kTarget) << opsText(plan.ops);
  EXPECT_LE(sim.max_onboard, 2);
  const int wrong = misplaced(observed, kTarget);
  EXPECT_EQ(sim.picks, wrong);
  EXPECT_EQ(sim.places, wrong);
  EXPECT_EQ(plan.picks, wrong);
  EXPECT_EQ(plan.places, wrong);
  EXPECT_EQ(plan.max_onboard, wrong == 0 ? 0 : 2);
  EXPECT_EQ(plan.final_order, kTarget);
  // Cubos certos nao sao tocados.
  for (std::size_t k = 0; k < observed.size(); ++k) {
    if (observed[k] == kTarget[k]) {
      for (const SortOp & op : plan.ops) {
        EXPECT_NE(op.tag, observed[k]) << "tocou o cubo certo " << observed[k];
      }
    }
  }
}

}  // namespace

// ---------------------------------------------------------------- planSort

TEST(Amt1PlanSort, AlreadySortedHasNoOps)
{
  const SortPlan plan = planSort(kTarget, kTarget, 2);
  EXPECT_TRUE(plan.error.empty());
  EXPECT_TRUE(plan.ops.empty());
  EXPECT_EQ(plan.max_onboard, 0);
  EXPECT_EQ(plan.picks, 0);
  EXPECT_EQ(plan.places, 0);
  EXPECT_EQ(plan.final_order, kTarget);
  // Mesa ja ordenada nao precisa de buffer.
  EXPECT_TRUE(planSort(kTarget, kTarget, 1).error.empty());
}

TEST(Amt1PlanSort, OperatorExampleExactSequence)
{
  const std::vector<int> observed{5, 2, 3, 1, 4, 6};
  const SortPlan plan = planSort(observed, kTarget, 2);
  ASSERT_TRUE(plan.error.empty()) << plan.error;
  // pega 5 (slot 0), pega 1 (slot 3), solta 1 em 0, pega 4 (slot 4),
  // solta 4 em 3, solta 5 em 4.
  expectOps(
    plan.ops,
    {pick(5, 0), pick(1, 3), place(1, 0), pick(4, 4), place(4, 3), place(5, 4)});
  EXPECT_EQ(plan.max_onboard, 2);
  EXPECT_EQ(plan.picks, 3);
  EXPECT_EQ(plan.places, 3);
  const SimulationResult sim = simulatePlan(observed, plan.ops, 2);
  ASSERT_TRUE(sim.ok) << sim.error;
  EXPECT_EQ(sim.final_order, kTarget);
  EXPECT_EQ(sim.max_onboard, 2);
}

TEST(Amt1PlanSort, TwoCycles)
{
  const std::vector<int> observed{2, 1, 4, 3, 5, 6};
  const SortPlan plan = planSort(observed, kTarget, 2);
  ASSERT_TRUE(plan.error.empty()) << plan.error;
  expectOps(
    plan.ops,
    {pick(2, 0), pick(1, 1), place(1, 0), place(2, 1),
      pick(4, 2), pick(3, 3), place(3, 2), place(4, 3)});
  const SimulationResult sim = simulatePlan(observed, plan.ops, 2);
  ASSERT_TRUE(sim.ok) << sim.error;
  EXPECT_EQ(sim.final_order, kTarget);
}

TEST(Amt1PlanSort, BufferOneIsInfeasible)
{
  const SortPlan plan = planSort({5, 2, 3, 1, 4, 6}, kTarget, 1);
  EXPECT_EQ(plan.error, "buffer_insuficiente");
  EXPECT_TRUE(plan.ops.empty());
  EXPECT_EQ(planSort({2, 1}, {1, 2}, 0).error, "buffer_insuficiente");
}

TEST(Amt1PlanSort, BufferThreeEqualsBufferTwo)
{
  const std::vector<int> observed{5, 2, 3, 1, 4, 6};
  const SortPlan two = planSort(observed, kTarget, 2);
  const SortPlan three = planSort(observed, kTarget, 3);
  ASSERT_TRUE(two.error.empty());
  ASSERT_TRUE(three.error.empty());
  expectOps(three.ops, two.ops);
  EXPECT_EQ(three.max_onboard, 2);
}

TEST(Amt1PlanSort, FullReverse)
{
  const std::vector<int> observed{6, 5, 4, 3, 2, 1};
  const SortPlan plan = planSort(observed, kTarget, 2);
  ASSERT_TRUE(plan.error.empty()) << plan.error;
  // Tres ciclos de 2: (0,5), (1,4), (2,3) — o primeiro comeca no slot 0.
  expectOps(
    plan.ops,
    {pick(6, 0), pick(1, 5), place(1, 0), place(6, 5),
      pick(5, 1), pick(2, 4), place(2, 1), place(5, 4),
      pick(4, 2), pick(3, 3), place(3, 2), place(4, 3)});
  const SimulationResult sim = simulatePlan(observed, plan.ops, 2);
  ASSERT_TRUE(sim.ok) << sim.error;
  EXPECT_EQ(sim.final_order, kTarget);
  EXPECT_EQ(sim.picks, 6);
  EXPECT_EQ(sim.places, 6);
}

TEST(Amt1PlanSort, SingleCycleOfSix)
{
  const std::vector<int> observed{2, 3, 4, 5, 6, 1};
  const SortPlan plan = planSort(observed, kTarget, 2);
  ASSERT_TRUE(plan.error.empty()) << plan.error;
  // dest = [1,2,3,4,5,0]: ciclo [0,1,2,3,4,5]. PICK 0; j=5..1: PICK j,
  // PLACE j -> dest[j]; PLACE 0 -> 1.
  expectOps(
    plan.ops,
    {pick(2, 0),
      pick(1, 5), place(1, 0),
      pick(6, 4), place(6, 5),
      pick(5, 3), place(5, 4),
      pick(4, 2), place(4, 3),
      pick(3, 1), place(3, 2),
      place(2, 1)});
  const SimulationResult sim = simulatePlan(observed, plan.ops, 2);
  ASSERT_TRUE(sim.ok) << sim.error;
  EXPECT_EQ(sim.final_order, kTarget);
  EXPECT_EQ(sim.max_onboard, 2);
  EXPECT_EQ(sim.picks, 6);
}

TEST(Amt1PlanSort, TargetMustBePermutation)
{
  EXPECT_EQ(planSort({1, 2, 3}, {1, 2}, 2).error, "alvo_invalido");
  EXPECT_EQ(planSort({1, 2, 3}, {1, 2, 4}, 2).error, "alvo_invalido");
  EXPECT_EQ(planSort({1, 2, 3}, {1, 1, 2}, 2).error, "alvo_invalido");
  EXPECT_EQ(planSort({1, 1, 2}, {1, 2, 2}, 2).error, "alvo_invalido");
  EXPECT_TRUE(planSort({}, {}, 2).error.empty());
  EXPECT_TRUE(planSort({}, {}, 2).ops.empty());
}

TEST(Amt1PlanSort, PartialTargetOfFourTags)
{
  // Faltam duas tags: ordena so as vistas (alvo = vistas em ordem).
  const std::vector<int> observed{6, 2, 5, 1};
  const std::vector<int> target{1, 2, 5, 6};
  const SortPlan plan = planSort(observed, target, 2);
  ASSERT_TRUE(plan.error.empty());
  const SimulationResult sim = simulatePlan(observed, plan.ops, 2);
  ASSERT_TRUE(sim.ok) << sim.error;
  EXPECT_EQ(sim.final_order, target);
  EXPECT_EQ(sim.picks, 2);
}

TEST(Amt1PlanSort, AllPermutationsBufferTwo)
{
  std::vector<int> observed = kTarget;
  int count = 0;
  do {
    checkPlanAgainstOracle(observed, 2);
    ++count;
  } while (std::next_permutation(observed.begin(), observed.end()));
  EXPECT_EQ(count, 720);
}

TEST(Amt1PlanSort, AllPermutationsBufferThree)
{
  std::vector<int> observed = kTarget;
  int count = 0;
  do {
    checkPlanAgainstOracle(observed, 3);
    ++count;
  } while (std::next_permutation(observed.begin(), observed.end()));
  EXPECT_EQ(count, 720);
}

TEST(Amt1PlanSort, EveryPlaceLandsOnFreeSlotAndPicksPrecedePlaces)
{
  // Invariante estrutural direto (sem oraculo): a sequencia de cada ciclo e
  // PICK, (PICK, PLACE)*, PLACE.
  std::vector<int> observed = kTarget;
  do {
    const SortPlan plan = planSort(observed, kTarget, 2);
    int onboard = 0;
    for (const SortOp & op : plan.ops) {
      onboard += op.type == SortOpType::kPick ? 1 : -1;
      ASSERT_GE(onboard, 0);
      ASSERT_LE(onboard, 2);
    }
    EXPECT_EQ(onboard, 0);
  } while (std::next_permutation(observed.begin(), observed.end()));
}

// ------------------------------------------------------------ simulatePlan

TEST(Amt1Simulate, RejectsInvalidPlans)
{
  const std::vector<int> observed{2, 1, 3};
  // place em slot ocupado
  EXPECT_FALSE(simulatePlan(observed, {pick(2, 0), place(2, 1)}, 2).ok);
  // pick de tag que nao esta no slot
  EXPECT_FALSE(simulatePlan(observed, {pick(1, 0)}, 2).ok);
  // buffer estourado
  EXPECT_FALSE(simulatePlan(observed, {pick(2, 0), pick(1, 1)}, 1).ok);
  // sobrou a bordo
  EXPECT_FALSE(simulatePlan(observed, {pick(2, 0)}, 2).ok);
  // slot fora da mesa
  EXPECT_FALSE(simulatePlan(observed, {pick(2, 7)}, 2).ok);
  // place de tag que nao esta a bordo
  EXPECT_FALSE(simulatePlan(observed, {pick(2, 0), place(1, 0)}, 2).ok);
  // plano vazio numa mesa desordenada e valido (so nao ordena)
  const SimulationResult sim = simulatePlan(observed, {}, 2);
  EXPECT_TRUE(sim.ok);
  EXPECT_EQ(sim.final_order, observed);
}

TEST(Amt1Simulate, ReportsStateOnFailure)
{
  const SimulationResult sim = simulatePlan({2, 1}, {pick(2, 0), pick(1, 1), pick(1, 1)}, 3);
  EXPECT_FALSE(sim.ok);
  EXPECT_EQ(sim.onboard, (std::vector<int>{2, 1}));
  EXPECT_EQ(sim.final_order, (std::vector<int>{0, 0}));
}

// ------------------------------------------------------------- assignSlots

namespace
{
TagPose tag(int id, double x, double y, double z = 0.10)
{
  TagPose t;
  t.id = id;
  t.x = x;
  t.y = y;
  t.z = z;
  return t;
}
}  // namespace

TEST(Amt1AssignSlots, LeftToRightByX)
{
  const std::vector<TagPose> seen{
    tag(5, -0.25, 0.30), tag(2, -0.15, 0.30), tag(3, -0.05, 0.30),
    tag(1, 0.05, 0.30), tag(4, 0.15, 0.30), tag(6, 0.25, 0.30)};
  const SlotAssignment a = assignSlots(seen, kTarget, true);
  EXPECT_EQ(a.observed_order, (std::vector<int>{5, 2, 3, 1, 4, 6}));
  EXPECT_EQ(a.target_order, kTarget);
  EXPECT_TRUE(a.missing_tags.empty());
  EXPECT_TRUE(a.ignored_tags.empty());
  ASSERT_EQ(a.slots.size(), 6u);
  EXPECT_EQ(a.slots[0].id, 5);
  EXPECT_DOUBLE_EQ(a.slots[0].x, -0.25);
}

TEST(Amt1AssignSlots, TieBreaksBySmallerY)
{
  // 7 e 8 a 5 mm em x (empate): menor y primeiro.
  const std::vector<TagPose> seen{
    tag(8, 0.102, 0.40), tag(7, 0.098, 0.30), tag(9, 0.30, 0.30), tag(6, -0.10, 0.30)};
  const SlotAssignment a = assignSlots(seen, {6, 7, 8, 9}, true, 0.01);
  EXPECT_EQ(a.observed_order, (std::vector<int>{6, 7, 8, 9}));
  // Fora do empate (dx = 5 cm) vale so o x.
  const std::vector<TagPose> seen2{tag(8, 0.15, 0.30), tag(7, 0.10, 0.40)};
  EXPECT_EQ(assignSlots(seen2, {7, 8}, true, 0.01).observed_order, (std::vector<int>{7, 8}));
  // Com eps maior, o mesmo par empata e o menor y (8) vem primeiro.
  EXPECT_EQ(assignSlots(seen2, {7, 8}, true, 0.10).observed_order, (std::vector<int>{8, 7}));
}

TEST(Amt1AssignSlots, RightToLeft)
{
  const std::vector<TagPose> seen{
    tag(5, -0.25, 0.30), tag(2, -0.15, 0.30), tag(3, -0.05, 0.30),
    tag(1, 0.05, 0.30), tag(4, 0.15, 0.30), tag(6, 0.25, 0.30)};
  const SlotAssignment a = assignSlots(seen, kTarget, false);
  EXPECT_EQ(a.observed_order, (std::vector<int>{6, 4, 1, 3, 2, 5}));
  EXPECT_EQ(a.target_order, kTarget);
  EXPECT_DOUBLE_EQ(a.slots[0].x, 0.25);
}

TEST(Amt1AssignSlots, PartialWhenTagsMissing)
{
  const std::vector<TagPose> seen{
    tag(6, -0.2, 0.3), tag(2, -0.1, 0.3), tag(5, 0.0, 0.3), tag(1, 0.1, 0.3)};
  const SlotAssignment a = assignSlots(seen, kTarget, true);
  EXPECT_EQ(a.observed_order, (std::vector<int>{6, 2, 5, 1}));
  EXPECT_EQ(a.target_order, (std::vector<int>{1, 2, 5, 6}));
  EXPECT_EQ(a.missing_tags, (std::vector<int>{3, 4}));
  EXPECT_TRUE(a.ignored_tags.empty());
  const SortPlan plan = planSort(a.observed_order, a.target_order, 2);
  EXPECT_TRUE(plan.error.empty());
  EXPECT_TRUE(simulatePlan(a.observed_order, plan.ops, 2).ok);
}

TEST(Amt1AssignSlots, IgnoresUnexpectedIdsAndDuplicates)
{
  const std::vector<TagPose> seen{
    tag(42, -0.3, 0.3), tag(2, -0.1, 0.3), tag(1, 0.1, 0.3), tag(2, 0.2, 0.3), tag(9, 0.4, 0.3)};
  const SlotAssignment a = assignSlots(seen, {1, 2, 3}, true);
  EXPECT_EQ(a.observed_order, (std::vector<int>{2, 1}));
  EXPECT_EQ(a.target_order, (std::vector<int>{1, 2}));
  EXPECT_EQ(a.missing_tags, (std::vector<int>{3}));
  EXPECT_EQ(a.ignored_tags, (std::vector<int>{9, 42}));
  ASSERT_EQ(a.slots.size(), 2u);
  EXPECT_DOUBLE_EQ(a.slots[0].x, -0.1);  // a primeira ocorrencia do id 2
}

TEST(Amt1AssignSlots, EmptyExpectedTakesEverything)
{
  const std::vector<TagPose> seen{tag(9, 0.2, 0.3), tag(4, -0.2, 0.3)};
  const SlotAssignment a = assignSlots(seen, {}, true);
  EXPECT_EQ(a.observed_order, (std::vector<int>{4, 9}));
  EXPECT_EQ(a.target_order, (std::vector<int>{4, 9}));
  EXPECT_TRUE(a.missing_tags.empty());
  EXPECT_TRUE(a.ignored_tags.empty());
  EXPECT_TRUE(assignSlots({}, kTarget, true).observed_order.empty());
  EXPECT_EQ(assignSlots({}, kTarget, true).missing_tags, kTarget);
}

// ------------------------------------------------------- registrationDelta

TEST(Amt1Registration, MeanOfConsistentAnchors)
{
  const std::vector<AnchorDelta> anchors{
    {1, 0.010, -0.020}, {2, 0.014, -0.016}, {3, 0.006, -0.024}};
  const auto r = registrationDelta(anchors, 2, 0.02);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_NEAR(r.dx, 0.010, 1e-9);
  EXPECT_NEAR(r.dy, -0.020, 1e-9);
  EXPECT_NEAR(r.spread, std::hypot(0.004, 0.004), 1e-9);
  EXPECT_EQ(r.used, 3u);
}

TEST(Amt1Registration, TooFewAnchors)
{
  const auto r = registrationDelta({{1, 0.01, 0.0}}, 2, 0.02);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "poucas_ancoras");
  EXPECT_FALSE(registrationDelta({}, 2, 0.02).ok);
  // min_anchors 1 aceita uma ancora so.
  EXPECT_TRUE(registrationDelta({{1, 0.01, 0.0}}, 1, 0.02).ok);
}

TEST(Amt1Registration, SpreadTooLargeKeepsOdometry)
{
  const std::vector<AnchorDelta> anchors{{1, 0.00, 0.00}, {2, 0.06, 0.00}};
  const auto r = registrationDelta(anchors, 2, 0.02);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason, "espalhamento");
  EXPECT_NEAR(r.spread, 0.03, 1e-9);
  // Exatamente no limite ainda vale.
  EXPECT_TRUE(registrationDelta({{1, 0.0, 0.0}, {2, 0.04, 0.0}}, 2, 0.02).ok);
}

TEST(Amt1DescribeOp, Text)
{
  EXPECT_EQ(describeOp(pick(5, 0)), "pick_5");
  EXPECT_EQ(describeOp(place(1, 0)), "place_1_slot_0");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
