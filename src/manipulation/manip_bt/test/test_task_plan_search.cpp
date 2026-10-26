// Testes do nucleo do task_planner (task_plan_search): modelo de custo,
// guloso historico e branch-and-bound por tempo, com transfers montados a
// mao (sem YAML, sem ROS). Como rodar:
//   colcon test --packages-select manip_bt --ctest-args -R test_task_plan_search
// Os tempos usam distancias e custos "redondos" para a conta a mao bater.

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "manip_bt/task_plan_search.hpp"

namespace tp = manip_bt::task_plan;

namespace
{

tp::TransferItem transfer(const int id, const std::string & from, const std::string & to)
{
  tp::TransferItem t;
  t.obj_id = id;
  t.tag_frame = "tag_" + std::to_string(id);
  t.from_ws = from;
  t.to_ws = to;
  t.needs_pick = true;
  t.needs_place = true;
  return t;
}

/// Modelo com distancias simetricas dadas como lista {a, b, d}.
tp::CostModel model(
  const std::vector<std::pair<std::pair<std::string, std::string>, double>> & edges,
  const double v_nav = 1.0, const double docking = 10.0, const double pick = 1.0,
  const double place = 1.0)
{
  tp::CostModel m;
  m.v_nav_mps = v_nav;
  m.docking_s = docking;
  m.pick_s = pick;
  m.place_s = place;
  for (const auto & [ab, d] : edges) {
    m.dist_m[ab.first][ab.second] = d;
  }
  return m;
}

std::vector<std::string> stations(const std::vector<tp::Visit> & visits)
{
  std::vector<std::string> out;
  for (const auto & v : visits) {
    out.push_back(v.ws);
  }
  return out;
}

/// Simula o plano acao por acao: no maximo 3 a bordo, place so depois do
/// pick, tudo entregue no fim. Devolve a ordem (kind, id) para checar
/// precedencias.
std::vector<std::pair<std::string, int>> simulate(
  const std::vector<tp::Visit> & visits, const std::vector<tp::TransferItem> & transfers)
{
  std::vector<std::pair<std::string, int>> order;
  std::vector<int> onboard;
  for (const auto & v : visits) {
    for (const int id : v.places) {
      const auto it = std::find(onboard.begin(), onboard.end(), id);
      EXPECT_NE(it, onboard.end()) << "place " << id << " sem pick antes";
      if (it != onboard.end()) {
        onboard.erase(it);
      }
      EXPECT_EQ(tp::findTransferById(transfers, id).to_ws, v.ws);
      order.emplace_back("place", id);
    }
    for (const int id : v.picks) {
      EXPECT_EQ(tp::findTransferById(transfers, id).from_ws, v.ws);
      onboard.push_back(id);
      EXPECT_LE(onboard.size(), static_cast<std::size_t>(tp::kMaxOnboard));
      order.emplace_back("pick", id);
    }
  }
  EXPECT_TRUE(onboard.empty()) << "sobrou a bordo";
  return order;
}

std::size_t indexOf(
  const std::vector<std::pair<std::string, int>> & order, const std::string & kind, const int id)
{
  const auto it = std::find(order.begin(), order.end(), std::make_pair(kind, id));
  EXPECT_NE(it, order.end()) << kind << " " << id << " ausente";
  return static_cast<std::size_t>(it - order.begin());
}

}  // namespace

// ---- modelo de custo -----------------------------------------------------------

TEST(CostModel, DistanceIsSymmetricAndMissingPairThrows)
{
  const tp::CostModel m = model({{{"START", "WS_1"}, 2.0}, {{"WS_1", "FINISH"}, 3.0}}, 0.5);
  EXPECT_DOUBLE_EQ(m.distance("START", "WS_1"), 2.0);
  EXPECT_DOUBLE_EQ(m.distance("WS_1", "START"), 2.0);   // so' um lado foi dado
  EXPECT_DOUBLE_EQ(m.distance("WS_1", "WS_1"), 0.0);
  EXPECT_DOUBLE_EQ(m.navTime("WS_1", "FINISH"), 6.0);
  EXPECT_TRUE(m.hasDistance("FINISH", "WS_1"));
  EXPECT_FALSE(m.hasDistance("START", "FINISH"));
  EXPECT_THROW(m.distance("START", "FINISH"), std::runtime_error);
}

TEST(CostModel, EstimatePlanTimeMatchesHandComputation)
{
  // START -2-> A (2 picks) -4-> B (2 places) -1-> FINISH; v 0.5 m/s,
  // docking 10 s, pick 5 s, place 6 s: nav 14 + docking 20 + manip 22 = 56.
  const tp::CostModel m = model(
    {{{"START", "A"}, 2.0}, {{"A", "B"}, 4.0}, {{"B", "FINISH"}, 1.0}}, 0.5, 10.0, 5.0, 6.0);
  std::vector<tp::Visit> visits{{"A", {}, {1, 2}}, {"B", {1, 2}, {}}};
  tp::PlanStats stats;
  EXPECT_DOUBLE_EQ(tp::estimatePlanTime(visits, m, &stats), 56.0);
  EXPECT_EQ(stats.visits, 2);
  EXPECT_EQ(stats.station_switches, 2);
  EXPECT_DOUBLE_EQ(stats.distance_m, 7.0);
  EXPECT_DOUBLE_EQ(stats.nav_s, 14.0);
  EXPECT_DOUBLE_EQ(stats.docking_s, 20.0);
  EXPECT_DOUBLE_EQ(stats.manipulation_s, 22.0);
  // Visita repetida na mesma estacao (sem goto) nao paga docking nem distancia.
  visits.push_back({"B", {}, {}});
  EXPECT_DOUBLE_EQ(tp::estimatePlanTime(visits, m), 56.0);
  // Plano vazio = so START -> FINISH.
  const tp::CostModel m2 = model({{{"START", "FINISH"}, 5.0}}, 0.5);
  EXPECT_DOUBLE_EQ(tp::estimatePlanTime({}, m2), 10.0);
}

// ---- guloso --------------------------------------------------------------------

TEST(Greedy, BmtGoesToSourceThenDestination)
{
  // BMT: 3 cubos da WS_5 para a WS_1 - 1 ida, 1 volta, ordem por id.
  const std::vector<tp::TransferItem> transfers{
    transfer(26, "WS_5", "WS_1"), transfer(13, "WS_5", "WS_1"), transfer(17, "WS_5", "WS_1")};
  std::mt19937 rng(1);
  const auto visits = tp::planGreedy(transfers, rng);
  ASSERT_EQ(visits.size(), 2u);
  EXPECT_EQ(visits[0].ws, "WS_5");
  EXPECT_EQ(visits[0].picks, (std::vector<int>{26, 13, 17}));   // ordem dos transfers
  EXPECT_TRUE(visits[0].places.empty());
  EXPECT_EQ(visits[1].ws, "WS_1");
  EXPECT_EQ(visits[1].places, (std::vector<int>{26, 13, 17}));

  // Emissao: goto, 3 picks, home, goto, 3 places (como sempre foi).
  const std::map<std::string, std::string> table{{"WS_1", "Mesa15"}, {"WS_5", "Mesa0"}};
  const YAML::Node actions = tp::emitActions(visits, transfers, table);
  std::vector<std::string> kinds;
  for (const auto & a : actions) {
    kinds.push_back(a["kind"].as<std::string>());
  }
  EXPECT_EQ(
    kinds, (std::vector<std::string>{"goto", "pick", "pick", "pick", "home", "goto", "place",
      "place", "place"}));
  EXPECT_EQ(actions[0]["mesa"].as<std::string>(), "Mesa0");
  EXPECT_EQ(actions[1]["table_pose"].as<std::string>(), "Mesa0");
  EXPECT_EQ(actions[6]["table_pose"].as<std::string>(), "Mesa15");
  EXPECT_EQ(actions[6]["tag_frame"].as<std::string>(), "tag_26");
}

TEST(Greedy, CapacityOfThreeSplitsIntoTwoTrips)
{
  const std::vector<tp::TransferItem> transfers{
    transfer(1, "A", "B"), transfer(2, "A", "B"), transfer(3, "A", "B"), transfer(4, "A", "B")};
  std::mt19937 rng(7);
  const auto visits = tp::planGreedy(transfers, rng);
  EXPECT_EQ(stations(visits), (std::vector<std::string>{"A", "B", "A", "B"}));
  EXPECT_EQ(visits[0].picks.size(), 3u);
  EXPECT_EQ(visits[2].picks, (std::vector<int>{4}));
  simulate(visits, transfers);
}

// ---- busca ---------------------------------------------------------------------

TEST(Optimized, PrefersShortRouteWhereGreedyTiesByCount)
{
  // 1: A -> C, 2: B -> C. START-A-B-C em linha (1 m cada); A-C e START-B
  // longe. Pelo numero de picks A e B empatam (o guloso sorteia); por tempo a
  // unica ordem boa e' A, B, C.
  const std::vector<tp::TransferItem> transfers{transfer(1, "A", "C"), transfer(2, "B", "C")};
  const tp::CostModel m = model(
    {{{"START", "A"}, 1.0}, {{"A", "B"}, 1.0}, {{"B", "C"}, 1.0}, {{"START", "B"}, 3.0},
      {{"A", "C"}, 5.0}, {{"START", "C"}, 9.0}, {{"A", "FINISH"}, 9.0}, {{"B", "FINISH"}, 9.0},
      {{"C", "FINISH"}, 1.0}, {{"START", "FINISH"}, 10.0}});
  // Plano inicial ruim (a ordem que o sorteio poderia dar): B, A, C.
  const std::vector<tp::Visit> bad{{"B", {}, {2}}, {"A", {}, {1}}, {"C", {1, 2}, {}}};
  const double bad_time = tp::estimatePlanTime(bad, m);
  EXPECT_DOUBLE_EQ(bad_time, (3.0 + 1.0 + 5.0 + 1.0) + 3 * 10.0 + 4.0);

  const tp::SearchResult r = tp::planOptimized(transfers, m, bad, 2.0);
  EXPECT_TRUE(r.optimal);
  EXPECT_TRUE(r.improved);
  EXPECT_EQ(stations(r.visits), (std::vector<std::string>{"A", "B", "C"}));
  EXPECT_DOUBLE_EQ(r.time_s, (1.0 + 1.0 + 1.0 + 1.0) + 3 * 10.0 + 4.0);
  EXPECT_LT(r.time_s, bad_time);
  simulate(r.visits, transfers);

  // Deterministico: mesma entrada, mesmo plano.
  const tp::SearchResult again = tp::planOptimized(transfers, m, bad, 2.0);
  EXPECT_EQ(stations(again.visits), stations(r.visits));
  EXPECT_EQ(again.nodes, r.nodes);
}

TEST(Optimized, FullCapacityNeedsTwoTripsAndTimeIsExact)
{
  // 4 cubos A -> B com capacidade 3: no minimo 4 visitas (A B A B).
  const std::vector<tp::TransferItem> transfers{
    transfer(1, "A", "B"), transfer(2, "A", "B"), transfer(3, "A", "B"), transfer(4, "A", "B")};
  const tp::CostModel m = model(
    {{{"START", "A"}, 1.0}, {{"A", "B"}, 2.0}, {{"B", "FINISH"}, 1.0}, {{"START", "B"}, 3.0},
      {{"A", "FINISH"}, 3.0}, {{"START", "FINISH"}, 4.0}});
  std::mt19937 rng(3);
  const auto greedy = tp::planGreedy(transfers, rng);
  const tp::SearchResult r = tp::planOptimized(transfers, m, greedy, 2.0);
  EXPECT_TRUE(r.optimal);
  EXPECT_EQ(stations(r.visits), (std::vector<std::string>{"A", "B", "A", "B"}));
  // nav 1+2+2+2+1 = 8; 4 dockings = 40; 4 picks + 4 places = 8 -> 56.
  EXPECT_DOUBLE_EQ(r.time_s, 56.0);
  EXPECT_FALSE(r.improved);   // o guloso ja era otimo aqui
  for (const auto & v : r.visits) {
    EXPECT_LE(v.picks.size(), 3u);
  }
  simulate(r.visits, transfers);
}

TEST(Optimized, PickAfterPlaceFreesSpaceAvoidsExtraTrip)
{
  // 1,2,3: A -> B; 4: B -> A. Chegando lotado em B, soltar os 3 libera lugar
  // para pegar o 4 na MESMA visita: A, B, A (3 visitas), nao A, B, A, B.
  const std::vector<tp::TransferItem> transfers{
    transfer(1, "A", "B"), transfer(2, "A", "B"), transfer(3, "A", "B"), transfer(4, "B", "A")};
  const tp::CostModel m = model(
    {{{"START", "A"}, 1.0}, {{"A", "B"}, 2.0}, {{"B", "FINISH"}, 5.0}, {{"START", "B"}, 3.0},
      {{"A", "FINISH"}, 1.0}, {{"START", "FINISH"}, 4.0}});
  // Plano inicial com uma visita a mais (valido, so' pior).
  const std::vector<tp::Visit> worse{
    {"A", {}, {1, 2}}, {"B", {1, 2}, {4}}, {"A", {4}, {3}}, {"B", {3}, {}}};
  const tp::SearchResult r = tp::planOptimized(transfers, m, worse, 2.0);
  EXPECT_TRUE(r.optimal);
  EXPECT_TRUE(r.improved);
  ASSERT_EQ(stations(r.visits), (std::vector<std::string>{"A", "B", "A"}));
  EXPECT_EQ(r.visits[0].picks, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(r.visits[1].places, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(r.visits[1].picks, (std::vector<int>{4}));
  EXPECT_EQ(r.visits[2].places, (std::vector<int>{4}));
  // nav 1+2+2+1 = 6; 3 dockings = 30; 8 manip -> 44.
  EXPECT_DOUBLE_EQ(r.time_s, 44.0);
  simulate(r.visits, transfers);
}

TEST(Optimized, WithThreeAboardGoesToDeliverBeforePickingMore)
{
  // 1,2,3: A -> C; 4: B -> C. Com 3 a bordo nao da para pegar o 4 em B:
  // A(3) C(3) B(1) C(1) = 4 visitas; pegar so' 2 em A custaria 5.
  const std::vector<tp::TransferItem> transfers{
    transfer(1, "A", "C"), transfer(2, "A", "C"), transfer(3, "A", "C"), transfer(4, "B", "C")};
  const tp::CostModel m = model(
    {{{"START", "A"}, 1.0}, {{"A", "B"}, 1.0}, {{"B", "C"}, 1.0}, {{"A", "C"}, 2.0},
      {{"START", "B"}, 2.0}, {{"START", "C"}, 3.0}, {{"C", "FINISH"}, 1.0},
      {{"A", "FINISH"}, 3.0}, {{"B", "FINISH"}, 2.0}, {{"START", "FINISH"}, 4.0}});
  std::mt19937 rng(11);
  const auto greedy = tp::planGreedy(transfers, rng);
  const tp::SearchResult r = tp::planOptimized(transfers, m, greedy, 2.0);
  EXPECT_TRUE(r.optimal);
  EXPECT_EQ(stations(r.visits), (std::vector<std::string>{"A", "C", "B", "C"}));
  EXPECT_EQ(r.visits[0].picks.size(), 3u);
  // nav 1+2+1+1+1 = 6; 4 dockings = 40; 8 manip -> 54.
  EXPECT_DOUBLE_EQ(r.time_s, 54.0);
  simulate(r.visits, transfers);
}

TEST(Optimized, TieBetweenSymmetricRoutesIsDeterministic)
{
  // 1: A -> C, 2: B -> C, tudo simetrico: A,B,C e B,A,C custam o mesmo. A
  // busca devolve sempre o mesmo plano (primeira estacao em ordem alfabetica)
  // com o tempo exato, e nao "melhora" um inicial que ja empata.
  const std::vector<tp::TransferItem> transfers{transfer(1, "A", "C"), transfer(2, "B", "C")};
  const tp::CostModel m = model(
    {{{"START", "A"}, 1.0}, {{"START", "B"}, 1.0}, {{"A", "B"}, 1.0}, {{"A", "C"}, 2.0},
      {{"B", "C"}, 2.0}, {{"C", "FINISH"}, 1.0}, {{"A", "FINISH"}, 9.0}, {{"B", "FINISH"}, 9.0},
      {{"START", "C"}, 9.0}, {{"START", "FINISH"}, 9.0}});
  const double expected = (1.0 + 1.0 + 2.0 + 1.0) + 3 * 10.0 + 4.0;
  const std::vector<tp::Visit> initial{{"B", {}, {2}}, {"A", {}, {1}}, {"C", {1, 2}, {}}};
  EXPECT_DOUBLE_EQ(tp::estimatePlanTime(initial, m), expected);

  const tp::SearchResult r1 = tp::planOptimized(transfers, m, initial, 2.0);
  const tp::SearchResult r2 = tp::planOptimized(transfers, m, initial, 2.0);
  EXPECT_TRUE(r1.optimal);
  EXPECT_DOUBLE_EQ(r1.time_s, expected);
  EXPECT_FALSE(r1.improved);
  EXPECT_EQ(stations(r1.visits), stations(r2.visits));
  EXPECT_EQ(r1.visits.size(), 3u);
  simulate(r1.visits, transfers);
}

TEST(Optimized, InitialStackOccupantIsPickedBeforePlaceOnTag)
{
  // Caso "ocupante_sai" do pytest: 14 ocupa a tag_200 da ST e vai para a
  // WS_1; 30 vem da WS_5 para a tag_200 (place so depois de pegar o 14).
  tp::TransferItem t14 = transfer(14, "ST", "WS_1");
  tp::TransferItem t30 = transfer(30, "WS_5", "ST");
  t30.stack_on_frame = "tag_200";
  t30.place_after_picked = {14};
  const std::vector<tp::TransferItem> transfers{t14, t30};
  const tp::CostModel m = model(
    {{{"START", "ST"}, 1.0}, {{"START", "WS_1"}, 1.0}, {{"START", "WS_5"}, 1.0},
      {{"ST", "WS_1"}, 1.0}, {{"ST", "WS_5"}, 1.0}, {{"WS_1", "WS_5"}, 1.0},
      {{"ST", "FINISH"}, 1.0}, {{"WS_1", "FINISH"}, 1.0}, {{"WS_5", "FINISH"}, 1.0},
      {{"START", "FINISH"}, 1.0}});
  std::mt19937 rng(5);
  const auto greedy = tp::planGreedy(transfers, rng);
  const tp::SearchResult r = tp::planOptimized(transfers, m, greedy, 2.0);
  EXPECT_TRUE(r.optimal);
  const auto order = simulate(r.visits, transfers);
  EXPECT_LT(indexOf(order, "pick", 14), indexOf(order, "place", 30));
  // 3 estacoes, cada uma visitada uma vez: nav 1+1+1+1, 3 dockings, 4 manip.
  EXPECT_DOUBLE_EQ(r.time_s, 4.0 + 30.0 + 4.0);
}

TEST(Optimized, ZeroTimeLimitKeepsInitialPlan)
{
  const std::vector<tp::TransferItem> transfers{transfer(1, "A", "B")};
  const tp::CostModel m = model(
    {{{"START", "A"}, 1.0}, {{"A", "B"}, 1.0}, {{"B", "FINISH"}, 1.0}, {{"START", "B"}, 1.0},
      {{"A", "FINISH"}, 1.0}, {{"START", "FINISH"}, 1.0}});
  const std::vector<tp::Visit> initial{{"A", {}, {1}}, {"B", {1}, {}}};
  const tp::SearchResult r = tp::planOptimized(transfers, m, initial, 0.0);
  EXPECT_FALSE(r.optimal);
  EXPECT_FALSE(r.improved);
  EXPECT_EQ(stations(r.visits), stations(initial));
  EXPECT_DOUBLE_EQ(r.time_s, 3.0 + 20.0 + 2.0);
}

TEST(Optimized, MissingDistanceIsAnError)
{
  const std::vector<tp::TransferItem> transfers{transfer(1, "A", "B")};
  const tp::CostModel m = model({{{"START", "A"}, 1.0}, {{"A", "B"}, 1.0}});   // sem FINISH
  const std::vector<tp::Visit> initial{{"A", {}, {1}}, {"B", {1}, {}}};
  EXPECT_THROW(tp::planOptimized(transfers, m, initial, 1.0), std::runtime_error);
}

TEST(Optimized, LargerMixedTaskFinishesQuicklyAndNeverBeatsLowerBoundOnVisits)
{
  // 8 objetos em 6 estacoes (cara de ATT2): a busca fecha bem dentro do prazo
  // e o plano respeita capacidade e precedencias; cada estacao com trabalho
  // aparece ao menos uma vez.
  const std::vector<tp::TransferItem> transfers{
    transfer(2, "WS_1", "WS_2"), transfer(8, "WS_1", "PP"), transfer(21, "WS_2", "PP"),
    transfer(3, "WS_4", "WS_5"), transfer(7, "WS_4", "WS_5"), transfer(9, "WS_5", "SH_1"),
    transfer(5, "WS_6", "WS_2"), transfer(20, "WS_6", "SH_1")};
  const std::vector<std::string> names{"START", "FINISH", "WS_1", "WS_2", "WS_4", "WS_5",
    "WS_6", "SH_1", "PP"};
  const std::vector<double> x{0.0, 8.0, 1.0, 2.0, 3.5, 5.0, 6.5, 6.0, 5.5};
  const std::vector<double> y{0.0, 0.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.5, 0.5};
  tp::CostModel m;
  m.v_nav_mps = 0.25;
  m.docking_s = 25.0;
  m.pick_s = 35.0;
  m.place_s = 35.0;
  for (std::size_t i = 0; i < names.size(); ++i) {
    for (std::size_t j = 0; j < names.size(); ++j) {
      const double dx = x[i] - x[j];
      const double dy = y[i] - y[j];
      m.dist_m[names[i]][names[j]] = std::sqrt(dx * dx + dy * dy);
    }
  }
  std::mt19937 rng(42);
  const auto greedy = tp::planGreedy(transfers, rng);
  const double greedy_time = tp::estimatePlanTime(greedy, m);
  const tp::SearchResult r = tp::planOptimized(transfers, m, greedy, 5.0);
  EXPECT_TRUE(r.optimal);
  EXPECT_LE(r.time_s, greedy_time + 1e-6);
  EXPECT_LT(r.elapsed_ms, 5000.0);
  simulate(r.visits, transfers);
  const auto visited = stations(r.visits);
  for (const auto & ws : tp::candidateStations(transfers)) {
    EXPECT_NE(std::find(visited.begin(), visited.end(), ws), visited.end()) << ws;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
