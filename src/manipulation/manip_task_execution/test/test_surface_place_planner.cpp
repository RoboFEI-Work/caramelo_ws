// Testes do planejador de superficie livre (sem ROS).
//
// Mesa sintetica em manip_base_link: frente em y = 0.16 (robo docado a
// 0,38 m do bico, braco a 0,217 m do base_footprint), 0,80 m de largura
// (x em [-0.40, 0.40]), 0,50 m de profundidade (y ate 0.66). Grade de 1 cm,
// x em [-0.60, 0.60], y em [0.05, 0.75].
#include <gtest/gtest.h>

#include <cmath>

#include "manip_task_execution/surface_place_planner.hpp"

using manip_task_execution::SurfaceCell;
using manip_task_execution::SurfaceGrid;
using manip_task_execution::SurfacePlaceRules;
using manip_task_execution::SurfacePlannerOptions;
using manip_task_execution::SurfaceExtraObstacle;
using manip_task_execution::SurfacePlan;

namespace
{

constexpr double kFrontY = 0.16;
constexpr double kBackY = 0.66;
constexpr double kLeftX = -0.40;
constexpr double kRightX = 0.40;

SurfaceGrid emptyGrid(int8_t fill = SurfaceCell::kUnknown)
{
  SurfaceGrid g;
  g.origin_x = -0.60;
  g.origin_y = 0.05;
  g.resolution = 0.01;
  g.width = 120;
  g.height = 70;
  g.cells.assign(static_cast<std::size_t>(g.width) * g.height, fill);
  return g;
}

/// Mesa inteira vista: tampo livre dentro do retangulo, fora-da-mesa no resto.
SurfaceGrid fullTable()
{
  SurfaceGrid g = emptyGrid(SurfaceCell::kOffTable);
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      const double x = g.cellX(i), y = g.cellY(j);
      if (x > kLeftX && x < kRightX && y > kFrontY && y < kBackY) {
        g.at(i, j) = SurfaceCell::kTableFree;
      }
    }
  }
  return g;
}

/// So a vista de pegar_obj: x em [-0.22, 0.22], y em [0.065, 0.32]; o resto
/// desconhecido. A frente aparece (y < 0.16 e' fora-da-mesa).
SurfaceGrid pegarObjView()
{
  SurfaceGrid g = emptyGrid(SurfaceCell::kUnknown);
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      const double x = g.cellX(i), y = g.cellY(j);
      if (x < -0.22 || x > 0.22 || y < 0.065 || y > 0.32) {continue;}
      g.at(i, j) = (y > kFrontY) ? SurfaceCell::kTableFree : SurfaceCell::kOffTable;
    }
  }
  return g;
}

void paintSquare(SurfaceGrid & g, double cx, double cy, double side, int8_t val = SurfaceCell::kObstacle)
{
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      if (std::abs(g.cellX(i) - cx) <= side / 2 && std::abs(g.cellY(j) - cy) <= side / 2) {
        g.at(i, j) = val;
      }
    }
  }
}

SurfacePlannerOptions defaultOptions()
{
  SurfacePlannerOptions o;
  o.preferred_x = 0.0;
  o.preferred_y = 0.30;
  return o;
}

/// Distancia minima de (x,y) ao retangulo da mesa (fora -> positivo).
double distToTableEdge(double x, double y)
{
  return std::min({x - kLeftX, kRightX - x, y - kFrontY, kBackY - y});
}

}  // namespace

TEST(SurfacePlacePlanner, EmptyTableFindsCenteredCandidateInsideZone)
{
  const SurfaceGrid g = fullTable();
  SurfacePlaceRules rules;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, rules, {}, defaultOptions());
  ASSERT_TRUE(plan.grid_valid);
  ASSERT_TRUE(plan.table_seen);
  ASSERT_TRUE(plan.front.found) << plan.diagnostics;
  EXPECT_FALSE(plan.front.from_fallback);
  EXPECT_NEAR(plan.front.y0, kFrontY, 0.011);
  EXPECT_NEAR(plan.front.slope, 0.0, 0.02);
  ASSERT_TRUE(plan.left.found) << plan.diagnostics;
  ASSERT_TRUE(plan.right.found) << plan.diagnostics;
  EXPECT_NEAR(plan.left.x, kLeftX, 0.011);
  EXPECT_NEAR(plan.right.x, kRightX, 0.011);
  ASSERT_FALSE(plan.candidates.empty()) << plan.diagnostics;
  for (const auto & c : plan.candidates) {
    // Zona: 2 cm da frente + meio cubo ... 20 cm - meio cubo.
    EXPECT_GE(c.depth_from_front_m, 0.02 + 0.0205 - 1e-6) << c.x << "," << c.y;
    EXPECT_LE(c.depth_from_front_m, 0.20 - 0.0205 + 1e-6) << c.x << "," << c.y;
    // Cubo (circunscrito) a >= 2 cm de qualquer borda.
    EXPECT_GE(distToTableEdge(c.x, c.y), 0.029 + 0.02 - 0.011) << c.x << "," << c.y;
  }
  // O melhor fica perto do ponto preferido em x (centro).
  EXPECT_NEAR(plan.candidates.front().x, 0.0, 0.03);
  EXPECT_GT(plan.candidates.front().clearance_obstacle_m, 0.05);
}

TEST(SurfacePlacePlanner, CandidatesAreSpaced)
{
  const SurfaceGrid g = fullTable();
  SurfacePlannerOptions o = defaultOptions();
  o.candidate_spacing_m = 0.03;
  o.max_candidates = 20;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  ASSERT_LE(plan.candidates.size(), 20u);
  for (std::size_t a = 0; a < plan.candidates.size(); ++a) {
    for (std::size_t b = a + 1; b < plan.candidates.size(); ++b) {
      const double d = std::hypot(plan.candidates[a].x - plan.candidates[b].x,
                                  plan.candidates[a].y - plan.candidates[b].y);
      EXPECT_GE(d, 0.03 - 1e-9);
    }
  }
}

TEST(SurfacePlacePlanner, ObstacleOnTableRespectsTwoCentimeters)
{
  SurfaceGrid g = fullTable();
  // Cubo de 41 mm no centro da zona (x=0, y=0.26).
  paintSquare(g, 0.0, 0.26, 0.041);
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, defaultOptions());
  ASSERT_FALSE(plan.candidates.empty()) << plan.diagnostics;
  for (const auto & c : plan.candidates) {
    // Distancia centro a centro >= meio cubo (obstaculo) + footprint + 2 cm.
    const double d = std::hypot(c.x - 0.0, c.y - 0.26);
    EXPECT_GE(d, 0.0205 + 0.029 + 0.02 - 0.012) << c.x << "," << c.y;
    EXPECT_GE(c.clearance_obstacle_m, 0.02 - 0.011);
  }
  // O centro deixou de ser candidato; o melhor migra para o lado.
  EXPECT_GT(std::abs(plan.candidates.front().x), 0.05);
}

TEST(SurfacePlacePlanner, ExtraObstaclesFromTfAreHonoured)
{
  const SurfaceGrid g = fullTable();
  std::vector<SurfaceExtraObstacle> extra;
  extra.push_back({0.0, 0.28, 0.029, "tag_5"});
  extra.push_back({0.15, 0.25, 0.10, "ct_azul"});
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, extra, defaultOptions());
  ASSERT_FALSE(plan.candidates.empty()) << plan.diagnostics;
  for (const auto & c : plan.candidates) {
    EXPECT_GE(std::hypot(c.x - 0.0, c.y - 0.28), 0.029 + 0.029 + 0.02 - 0.012);
    EXPECT_GE(std::hypot(c.x - 0.15, c.y - 0.25), 0.10 + 0.029 + 0.02 - 0.012);
  }
}

TEST(SurfacePlacePlanner, WallOnRightPushesCandidatesTenCentimetersAway)
{
  const SurfaceGrid g = fullTable();
  SurfacePlaceRules rules;
  rules.wall_right = true;
  SurfacePlannerOptions o = defaultOptions();
  o.preferred_x = 0.35;   // forca a preferencia para a direita
  o.weight_preferred_distance = 2.0;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, rules, {}, o);
  ASSERT_FALSE(plan.candidates.empty()) << plan.diagnostics;
  for (const auto & c : plan.candidates) {
    EXPECT_LE(c.x + 0.0205, kRightX - 0.10 + 0.011) << c.x;
  }
  // Sem parede o melhor chegaria a ~2 cm da borda direita.
  rules.wall_right = false;
  const SurfacePlan free_plan = manip_task_execution::planSurfacePlacement(g, rules, {}, o);
  EXPECT_GT(free_plan.candidates.front().x, plan.candidates.front().x + 0.05);
}

TEST(SurfacePlacePlanner, WallXFromLidarOverridesTableEdge)
{
  const SurfaceGrid g = fullTable();
  SurfacePlaceRules rules;
  rules.wall_left = true;
  rules.wall_left_x = -0.30;   // parede "dentro" da mesa (teste): x >= -0.30 + 0.10 + meio cubo
  SurfacePlannerOptions o = defaultOptions();
  o.preferred_x = -0.40;
  o.weight_preferred_distance = 2.0;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, rules, {}, o);
  ASSERT_FALSE(plan.candidates.empty()) << plan.diagnostics;
  for (const auto & c : plan.candidates) {
    EXPECT_GE(c.x - 0.0205, -0.30 + 0.10 - 0.011) << c.x;
  }
}

TEST(SurfacePlacePlanner, PartialViewUsesOnlyKnownCells)
{
  const SurfaceGrid g = pegarObjView();
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, defaultOptions());
  ASSERT_TRUE(plan.table_seen);
  ASSERT_TRUE(plan.front.found) << plan.diagnostics;
  EXPECT_FALSE(plan.front.from_fallback);
  EXPECT_NEAR(plan.front.y0, kFrontY, 0.011);
  EXPECT_FALSE(plan.left.found);    // laterais fora da vista
  EXPECT_FALSE(plan.right.found);
  ASSERT_FALSE(plan.candidates.empty()) << plan.diagnostics;
  for (const auto & c : plan.candidates) {
    // Cubo inteiro dentro da vista (x em [-0.22, 0.22], y <= 0.32).
    EXPECT_LE(std::abs(c.x) + 0.029, 0.22 + 0.011);
    EXPECT_LE(c.y + 0.029, 0.32 + 0.011);
    EXPECT_GE(c.depth_from_front_m, 0.0405 - 1e-6);
  }
}

TEST(SurfacePlacePlanner, FrontNotSeenFallsBackToGivenLine)
{
  SurfaceGrid g = emptyGrid(SurfaceCell::kUnknown);
  // So' tampo livre visto em y > 0.20 (frente fora da imagem).
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      const double x = g.cellX(i), y = g.cellY(j);
      if (std::abs(x) < 0.22 && y > 0.20 && y < 0.40) {g.at(i, j) = SurfaceCell::kTableFree;}
    }
  }
  SurfacePlannerOptions o = defaultOptions();
  SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  EXPECT_FALSE(plan.front.found) << plan.diagnostics;
  o.fallback_front_y = kFrontY;
  plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  ASSERT_TRUE(plan.front.found);
  EXPECT_TRUE(plan.front.from_fallback);
  ASSERT_FALSE(plan.candidates.empty()) << plan.diagnostics;
  for (const auto & c : plan.candidates) {
    EXPECT_LE(c.y - kFrontY, 0.20 - 0.0205 + 1e-6);
    EXPECT_GE(c.y - kFrontY, 0.0405 - 1e-6);
  }
}

TEST(SurfacePlacePlanner, FrontFitFarFromPriorIsRejected)
{
  // Mesa "comecando" em y = 0.30 na grade (faces de objetos alinhados
  // fingindo a frente); prior do docking diz 0.16.
  SurfaceGrid g = emptyGrid(SurfaceCell::kOffTable);
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      const double x = g.cellX(i), y = g.cellY(j);
      if (x > kLeftX && x < kRightX && y > 0.30 && y < kBackY) {g.at(i, j) = SurfaceCell::kTableFree;}
    }
  }
  SurfacePlannerOptions o = defaultOptions();
  o.fallback_front_y = kFrontY;
  o.front_prior_y = kFrontY;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  ASSERT_TRUE(plan.front.found);
  EXPECT_TRUE(plan.front.rejected_by_prior) << plan.diagnostics;
  EXPECT_TRUE(plan.front.from_fallback);
  EXPECT_NEAR(plan.front.y0, kFrontY, 1e-9);
  // Com a frente real em 0.16, a zona acaba em 0.36: nenhum candidato alem disso.
  for (const auto & c : plan.candidates) {
    EXPECT_LE(c.y, kFrontY + 0.20 - 0.0205 + 1e-6);
  }
  // Prior compativel: o ajuste vale.
  SurfacePlannerOptions o2 = defaultOptions();
  o2.front_prior_y = 0.31;
  const SurfacePlan plan2 = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o2);
  EXPECT_FALSE(plan2.front.rejected_by_prior);
  EXPECT_FALSE(plan2.front.from_fallback);
  EXPECT_NEAR(plan2.front.y0, 0.30, 0.011);
}

TEST(SurfacePlacePlanner, FullTableHasNoCandidates)
{
  SurfaceGrid g = fullTable();
  // Zona lotada: faixa de obstaculo cobrindo toda a profundidade da zona.
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      const double y = g.cellY(j);
      if (g.at(i, j) == SurfaceCell::kTableFree && y < kFrontY + 0.21) {
        g.at(i, j) = SurfaceCell::kObstacle;
      }
    }
  }
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, defaultOptions());
  EXPECT_TRUE(plan.table_seen);
  EXPECT_TRUE(plan.candidates.empty()) << plan.diagnostics;
}

TEST(SurfacePlacePlanner, InvalidGrid)
{
  SurfaceGrid g;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, defaultOptions());
  EXPECT_FALSE(plan.grid_valid);
  EXPECT_TRUE(plan.candidates.empty());
}

TEST(SurfacePlacePlanner, ParseWallSides)
{
  bool l = false, r = false;
  std::vector<std::string> unk;
  manip_task_execution::parseWallSides("left", &l, &r, &unk);
  EXPECT_TRUE(l); EXPECT_FALSE(r);
  manip_task_execution::parseWallSides(" Direita , ESQUERDA ", &l, &r, &unk);
  EXPECT_TRUE(l); EXPECT_TRUE(r);
  manip_task_execution::parseWallSides("", &l, &r, &unk);
  EXPECT_FALSE(l); EXPECT_FALSE(r);
  manip_task_execution::parseWallSides("right,xyz", &l, &r, &unk);
  EXPECT_FALSE(l); EXPECT_TRUE(r);
  ASSERT_EQ(unk.size(), 1u);
  EXPECT_EQ(unk.front(), "xyz");
}

TEST(SurfacePlacePlanner, PaintObstacleNeverOverwritesOffTable)
{
  SurfaceGrid g = fullTable();
  manip_task_execution::paintObstacle(g, kRightX, 0.30, 0.05);  // metade fora da mesa
  int i, j;
  ASSERT_TRUE(g.cellOf(kRightX - 0.02, 0.30, &i, &j));
  EXPECT_EQ(g.at(i, j), SurfaceCell::kObstacle);
  ASSERT_TRUE(g.cellOf(kRightX + 0.02, 0.30, &i, &j));
  EXPECT_EQ(g.at(i, j), SurfaceCell::kOffTable);
}

TEST(SideWalls, DetectsWallBesideRightEdgeOnly)
{
  std::vector<std::pair<double, double>> pts;
  // Parede a direita em x = 0.52, de y = 0.10 a 0.70.
  for (double y = 0.10; y <= 0.70; y += 0.02) {pts.emplace_back(0.52 + 0.004 * std::sin(y * 50), y);}
  // Ruido esparso a esquerda.
  pts.emplace_back(-0.55, 0.3);
  pts.emplace_back(-0.60, 0.4);
  manip_task_execution::SideWallParams p;
  const auto res = manip_task_execution::detectSideWalls(pts, kFrontY, kLeftX, kRightX, p);
  EXPECT_TRUE(res.right_found);
  EXPECT_NEAR(res.right_x, 0.52, 0.01);
  EXPECT_FALSE(res.left_found);
}

TEST(SideWalls, IgnoresPointsFarFromTable)
{
  std::vector<std::pair<double, double>> pts;
  for (double y = 0.10; y <= 0.70; y += 0.02) {pts.emplace_back(0.90, y);}  // longe demais
  const auto res = manip_task_execution::detectSideWalls(pts, kFrontY, kLeftX, kRightX, {});
  EXPECT_FALSE(res.right_found);
}
