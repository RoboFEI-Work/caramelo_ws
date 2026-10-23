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
using manip_task_execution::FrontPriorMode;

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

/// Mesa "comecando" em y = 0.30 na grade; o prior do docking diz 0.16.
SurfaceGrid tableFrontAt030()
{
  SurfaceGrid g = emptyGrid(SurfaceCell::kOffTable);
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      const double x = g.cellX(i), y = g.cellY(j);
      if (x > kLeftX && x < kRightX && y > 0.30 && y < kBackY) {g.at(i, j) = SurfaceCell::kTableFree;}
    }
  }
  return g;
}

TEST(SurfacePlacePlanner, FrontFitFarFromPriorIsRejectedInPriorGateMode)
{
  // Modo antigo (prior_gate): faces de objetos alinhados fingindo a frente
  // sao rejeitadas pelo prior e o fallback vale.
  const SurfaceGrid g = tableFrontAt030();
  SurfacePlannerOptions o = defaultOptions();
  o.front_prior_mode = FrontPriorMode::kPriorGate;
  o.fallback_front_y = kFrontY;
  o.front_prior_y = kFrontY;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  ASSERT_TRUE(plan.front.found);
  EXPECT_TRUE(plan.front.rejected_by_prior) << plan.diagnostics;
  EXPECT_TRUE(plan.front.from_fallback);
  EXPECT_NEAR(plan.front.y0, kFrontY, 1e-9);
  EXPECT_NEAR(plan.front.prior_deviation_m, 0.14, 0.011);
  // Com a frente real em 0.16, a zona acaba em 0.36: nenhum candidato alem disso.
  for (const auto & c : plan.candidates) {
    EXPECT_LE(c.y, kFrontY + 0.20 - 0.0205 + 1e-6);
  }
  // Prior compativel: o ajuste vale.
  SurfacePlannerOptions o2 = defaultOptions();
  o2.front_prior_mode = FrontPriorMode::kPriorGate;
  o2.front_prior_y = 0.31;
  const SurfacePlan plan2 = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o2);
  EXPECT_FALSE(plan2.front.rejected_by_prior);
  EXPECT_FALSE(plan2.front.from_fallback);
  EXPECT_NEAR(plan2.front.y0, 0.30, 0.011);
}

TEST(SurfacePlacePlanner, CameraFirstAcceptsFrontFarFromPrior)
{
  // Run de 21/10: robo docado girado, a camera viu a frente real 12-15 cm
  // alem do prior. camera_first (default): a frente ajustada vale e a zona
  // fica no lugar certo.
  const SurfaceGrid g = tableFrontAt030();
  SurfacePlannerOptions o = defaultOptions();
  EXPECT_EQ(o.front_prior_mode, FrontPriorMode::kCameraFirst);
  o.fallback_front_y = kFrontY;
  o.front_prior_y = kFrontY;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  ASSERT_TRUE(plan.front.found);
  EXPECT_FALSE(plan.front.from_fallback) << plan.diagnostics;
  EXPECT_FALSE(plan.front.rejected_by_prior);
  EXPECT_FALSE(plan.front.rejected_by_slope);
  EXPECT_NEAR(plan.front.y0, 0.30, 0.011);
  EXPECT_NEAR(plan.front.prior_deviation_m, 0.14, 0.011);
  EXPECT_NE(plan.diagnostics.find("camera_first: aceita"), std::string::npos) << plan.diagnostics;
  ASSERT_FALSE(plan.candidates.empty()) << plan.diagnostics;
  for (const auto & c : plan.candidates) {
    EXPECT_GE(c.y, 0.30 + 0.0405 - 0.011) << c.y;
    EXPECT_LE(c.y, 0.30 + 0.20 - 0.0205 + 0.011) << c.y;
  }
  // Sem prior nenhum: igual.
  o.front_prior_y.reset();
  const SurfacePlan plan2 = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  EXPECT_FALSE(plan2.front.from_fallback);
  EXPECT_NEAR(plan2.front.prior_deviation_m, 0.0, 1e-9);
}

TEST(SurfacePlacePlanner, CameraFirstRejectsSteepFrontAndFallsBack)
{
  // Frente inclinada 28 graus (y = 0.25 + tan(28) x): nao e' frente de mesa
  // -> rejeitada pela inclinacao, fallback vale. Com o limite em 30 ela passa.
  const double b = std::tan(28.0 * M_PI / 180.0);
  SurfaceGrid g = emptyGrid(SurfaceCell::kOffTable);
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      const double x = g.cellX(i), y = g.cellY(j);
      if (x > -0.30 && x < 0.30 && y > 0.25 + b * x && y < kBackY) {g.at(i, j) = SurfaceCell::kTableFree;}
    }
  }
  SurfacePlannerOptions o = defaultOptions();
  o.fallback_front_y = kFrontY;
  o.front_prior_y = kFrontY;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  ASSERT_TRUE(plan.front.found);
  EXPECT_TRUE(plan.front.rejected_by_slope) << plan.diagnostics;
  EXPECT_FALSE(plan.front.rejected_by_prior);
  EXPECT_TRUE(plan.front.from_fallback);
  EXPECT_NE(plan.diagnostics.find("inclinada"), std::string::npos) << plan.diagnostics;
  o.front_max_slope_deg = 30.0;
  const SurfacePlan plan2 = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  EXPECT_FALSE(plan2.front.from_fallback) << plan2.diagnostics;
  EXPECT_NEAR(plan2.front.slope, b, 0.06);
  EXPECT_NEAR(plan2.front.y0, 0.25, 0.02);
}

TEST(SurfacePlacePlanner, OffTableIslandsAreRemovedOnlyWhenEnclosedAndSmall)
{
  SurfaceGrid g = fullTable();
  // Ilhas pequenas cercadas por tampo: 3x3, 3x3, 5x5 (centros em centros de celula).
  paintSquare(g, -0.205, 0.405, 0.03, SurfaceCell::kOffTable);
  paintSquare(g, 0.105, 0.305, 0.03, SurfaceCell::kOffTable);
  paintSquare(g, 0.255, 0.505, 0.05, SurfaceCell::kOffTable);
  // Bloco cercado mas grande (13x13 = 169 >= 150): fica.
  paintSquare(g, -0.005, 0.555, 0.13, SurfaceCell::kOffTable);
  // Ilha encostada numa celula desconhecida: fica (pode ser o fora real).
  paintSquare(g, 0.305, 0.405, 0.03, SurfaceCell::kOffTable);
  paintSquare(g, 0.305, 0.425, 0.01, SurfaceCell::kUnknown);
  SurfaceGrid g2 = g;
  int cells = 0;
  EXPECT_EQ(manip_task_execution::removeOffTableIslands(g2, 150, &cells), 3);
  EXPECT_EQ(cells, 9 + 9 + 25);
  int i, j;
  ASSERT_TRUE(g2.cellOf(-0.205, 0.405, &i, &j));
  EXPECT_EQ(g2.at(i, j), SurfaceCell::kUnknown);
  ASSERT_TRUE(g2.cellOf(0.255, 0.525, &i, &j));
  EXPECT_EQ(g2.at(i, j), SurfaceCell::kUnknown);
  ASSERT_TRUE(g2.cellOf(-0.005, 0.555, &i, &j));
  EXPECT_EQ(g2.at(i, j), SurfaceCell::kOffTable);     // grande
  ASSERT_TRUE(g2.cellOf(0.305, 0.405, &i, &j));
  EXPECT_EQ(g2.at(i, j), SurfaceCell::kOffTable);     // toca desconhecido
  ASSERT_TRUE(g2.cellOf(0.0, 0.10, &i, &j));
  EXPECT_EQ(g2.at(i, j), SurfaceCell::kOffTable);     // fora real (toca a borda da grade)
  // Limite menor: a 5x5 (25) fica; desligado: nada muda.
  SurfaceGrid g3 = g;
  EXPECT_EQ(manip_task_execution::removeOffTableIslands(g3, 20, &cells), 2);
  EXPECT_EQ(cells, 18);
  SurfaceGrid g4 = g;
  EXPECT_EQ(manip_task_execution::removeOffTableIslands(g4, 0, &cells), 0);
  EXPECT_EQ(cells, 0);
  EXPECT_EQ(g4.cells, g.cells);
}

TEST(SurfacePlacePlanner, PlannerRemovesIslandsBeforeEdgesAndDistances)
{
  SurfaceGrid g = fullTable();
  paintSquare(g, -0.205, 0.405, 0.03, SurfaceCell::kOffTable);
  paintSquare(g, 0.105, 0.305, 0.03, SurfaceCell::kOffTable);
  paintSquare(g, 0.255, 0.505, 0.05, SurfaceCell::kOffTable);
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, defaultOptions());
  EXPECT_EQ(plan.n_islands_removed, 3);
  EXPECT_EQ(plan.n_island_cells_removed, 43);
  EXPECT_NE(plan.diagnostics.find("3 ilha(s) de fora removida(s) (43 cel)"), std::string::npos) << plan.diagnostics;
  // Bordas e frente intactas; as ilhas nao viram "borda".
  ASSERT_TRUE(plan.front.found) << plan.diagnostics;
  EXPECT_NEAR(plan.front.y0, kFrontY, 0.011);
  ASSERT_TRUE(plan.left.found) << plan.diagnostics;
  ASSERT_TRUE(plan.right.found) << plan.diagnostics;
  EXPECT_NEAR(plan.left.x, kLeftX, 0.011);
  EXPECT_NEAR(plan.right.x, kRightX, 0.011);
  EXPECT_EQ(plan.n_unknown, 43);
  ASSERT_FALSE(plan.candidates.empty()) << plan.diagnostics;
  // A ilha em (0.105, 0.305) ainda bloqueia o cubo (desconhecido sob ele), mas
  // nao os 5,4 cm de uma borda: ha candidato a menos de 5 cm dela.
  bool near = false;
  for (const auto & c : plan.candidates) {
    EXPECT_GE(std::hypot(c.x - 0.105, c.y - 0.305), 0.015 + 0.029 - 0.011);
    if (std::hypot(c.x - 0.105, c.y - 0.305) < 0.05) {near = true;}
  }
  EXPECT_TRUE(near) << plan.diagnostics;
  // Desligado: as ilhas ficam e empurram tudo (so' borda rejeita mais).
  SurfacePlannerOptions o = defaultOptions();
  o.island_max_cells = 0;
  const SurfacePlan plan0 = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  EXPECT_EQ(plan0.n_islands_removed, 0);
  EXPECT_GT(plan0.n_rejected_edge, plan.n_rejected_edge);
}

TEST(SurfacePlacePlanner, SideEdgeOnWrongSideOfTableIsRejected)
{
  // Run de 21/10: "borda direita x=-0.29, borda esquerda x=-0.22", as duas a
  // esquerda do robo, a "direita" mais a esquerda que a "esquerda" (faixa de
  // fora-da-mesa dentro do tampo). Vista parcial sem as laterais reais.
  auto make = [](double x0, double x1) {
      SurfaceGrid g = emptyGrid(SurfaceCell::kUnknown);
      for (int j = 0; j < g.height; ++j) {
        for (int i = 0; i < g.width; ++i) {
          const double x = g.cellX(i), y = g.cellY(j);
          if (x < -0.35 || x > 0.35 || y < 0.065 || y > 0.50) {continue;}
          g.at(i, j) = (y > kFrontY) ? SurfaceCell::kTableFree : SurfaceCell::kOffTable;
          // Faixa de "fora" ate o topo da vista (toca o desconhecido: nao e' ilha).
          if (x > x0 && x < x1 && y > 0.25) {g.at(i, j) = SurfaceCell::kOffTable;}
        }
      }
      return g;
    };
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(
    make(-0.29, -0.22), SurfacePlaceRules{}, {}, defaultOptions());
  EXPECT_EQ(plan.n_islands_removed, 0);
  EXPECT_NEAR(plan.table_x_median, 0.0, 0.03);
  EXPECT_FALSE(plan.right.found) << plan.diagnostics;
  EXPECT_TRUE(plan.right.rejected_wrong_side);
  EXPECT_NEAR(plan.right.x, -0.29, 0.011);
  EXPECT_NE(plan.diagnostics.find("borda direita nao vista (lado errado"), std::string::npos) << plan.diagnostics;
  EXPECT_TRUE(plan.left.found) << plan.diagnostics;      // esta a esquerda da mediana: aceita
  EXPECT_FALSE(plan.left.rejected_wrong_side);
  EXPECT_NEAR(plan.left.x, -0.22, 0.011);
  // Espelho: faixa a direita -> "esquerda" rejeitada, "direita" aceita.
  const SurfacePlan mirror = manip_task_execution::planSurfacePlacement(
    make(0.22, 0.29), SurfacePlaceRules{}, {}, defaultOptions());
  EXPECT_FALSE(mirror.left.found) << mirror.diagnostics;
  EXPECT_TRUE(mirror.left.rejected_wrong_side);
  EXPECT_TRUE(mirror.right.found) << mirror.diagnostics;
  EXPECT_NEAR(mirror.right.x, 0.22, 0.011);
  // Parede declarada a direita com a "borda" falsa: antes a referencia seria
  // x=-0.29 (zona inteira perdida); agora cai na extensao do tampo visto.
  SurfacePlaceRules rules;
  rules.wall_right = true;
  const SurfacePlan walled = manip_task_execution::planSurfacePlacement(
    make(-0.29, -0.22), rules, {}, defaultOptions());
  ASSERT_FALSE(walled.candidates.empty()) << walled.diagnostics;
  for (const auto & c : walled.candidates) {
    EXPECT_LE(c.x + 0.0205, 0.35 - 0.10 + 0.011) << c.x;
  }
  // Mesa inteira: as duas laterais reais continuam aceitas.
  const SurfacePlan full = manip_task_execution::planSurfacePlacement(fullTable(), SurfacePlaceRules{}, {}, defaultOptions());
  EXPECT_TRUE(full.left.found && full.right.found) << full.diagnostics;
}

TEST(SurfacePlacePlanner, RejectionCountersAddUpToFreeCells)
{
  SurfaceGrid g = fullTable();
  paintSquare(g, 0.0, 0.26, 0.041);
  // Um pedaco nao visto no fundo da zona.
  paintSquare(g, -0.20, 0.30, 0.05, SurfaceCell::kUnknown);
  SurfacePlaceRules rules;
  rules.wall_right = true;
  SurfacePlannerOptions o = defaultOptions();
  o.max_reach_radius_m = 0.45;
  const SurfacePlan plan = manip_task_execution::planSurfacePlacement(g, rules, {}, o);
  const int sum = plan.n_valid_cells + plan.n_rejected_edge + plan.n_rejected_object +
    plan.n_rejected_unknown + plan.n_rejected_zone + plan.n_rejected_wall + plan.n_rejected_reach;
  EXPECT_EQ(sum, plan.n_table_free) << plan.diagnostics;
  EXPECT_GT(plan.n_valid_cells, 0);
  EXPECT_GT(plan.n_rejected_edge, 0);
  EXPECT_GT(plan.n_rejected_object, 0);
  EXPECT_GT(plan.n_rejected_unknown, 0);
  EXPECT_GT(plan.n_rejected_zone, 0);
  EXPECT_GT(plan.n_rejected_wall, 0);
  EXPECT_GT(plan.n_rejected_reach, 0);
  EXPECT_NE(plan.diagnostics.find(plan.rejectionSummary()), std::string::npos) << plan.diagnostics;
  EXPECT_NE(plan.rejectionSummary().find("validas "), std::string::npos);
  EXPECT_NE(plan.rejectionSummary().find("(rejeitadas: borda "), std::string::npos);
  // Sem parede nem poda de alcance esses contadores zeram e a soma continua fechando.
  rules.wall_right = false;
  o.max_reach_radius_m = 0.0;
  const SurfacePlan plan2 = manip_task_execution::planSurfacePlacement(g, rules, {}, o);
  EXPECT_EQ(plan2.n_rejected_wall, 0);
  EXPECT_EQ(plan2.n_rejected_reach, 0);
  EXPECT_EQ(plan2.n_valid_cells + plan2.n_rejected_edge + plan2.n_rejected_object +
    plan2.n_rejected_unknown + plan2.n_rejected_zone, plan2.n_table_free);
  // Ordem borda > objeto: cubo encostado na frente conta como borda, nao objeto.
  SurfaceGrid g3 = fullTable();
  paintSquare(g3, 0.0, kFrontY + 0.0205, 0.041);
  const SurfacePlan plan3 = manip_task_execution::planSurfacePlacement(g3, SurfacePlaceRules{}, {}, defaultOptions());
  SurfaceGrid g4 = fullTable();
  paintSquare(g4, 0.0, 0.30, 0.041);
  const SurfacePlan plan4 = manip_task_execution::planSurfacePlacement(g4, SurfacePlaceRules{}, {}, defaultOptions());
  EXPECT_LT(plan3.n_rejected_object, plan4.n_rejected_object);
}

TEST(SurfacePlacePlanner, Run20261021IslandsAndFarFrontRegression)
{
  // Reproducao da WS1 (analise_run_2026-10-21, secao 2): vista parcial, frente real
  // em y = 0.28 (robo docado girado 19,7 graus: 12 cm alem do prior 0.163),
  // ~1.8 mil celulas de "fora" (o fora real + ilhas de profundidade lixo
  // espalhadas pelo tampo) e cubos com tag. Antes: 0 candidatos nas 3 vistas.
  SurfaceGrid g = emptyGrid(SurfaceCell::kUnknown);
  for (int j = 0; j < g.height; ++j) {
    for (int i = 0; i < g.width; ++i) {
      const double x = g.cellX(i), y = g.cellY(j);
      if (x < -0.45 || x > 0.45 || y < 0.15 || y > 0.60) {continue;}
      g.at(i, j) = (x > kLeftX && x < kRightX && y > 0.28) ? SurfaceCell::kTableFree : SurfaceCell::kOffTable;
    }
  }
  for (double cx = -0.36; cx <= 0.361; cx += 0.12) {
    for (const double cy : {0.34, 0.50}) {
      paintSquare(g, cx, cy, 0.04, SurfaceCell::kOffTable);   // ilhas 4x4
    }
  }
  paintSquare(g, 0.10, 0.45, 0.041);
  paintSquare(g, -0.15, 0.40, 0.041);
  int n_off = 0;
  for (const int8_t c : g.cells) {if (c == SurfaceCell::kOffTable) {++n_off;}}
  EXPECT_GT(n_off, 1600);
  EXPECT_LT(n_off, 2000);

  SurfacePlannerOptions o = defaultOptions();
  o.fallback_front_y = 0.163;
  o.front_prior_y = 0.163;
  o.front_prior_tolerance_m = 0.06;
  o.max_reach_radius_m = 0.45;
  // ANTES (prior_gate + sem remocao de ilhas): a frente ajustada cai nas
  // ilhas, e' rejeitada pelo prior, a zona vai para o lugar errado e as
  // ilhas rejeitam o resto por "borda" -> 0 candidatos.
  SurfacePlannerOptions before = o;
  before.front_prior_mode = FrontPriorMode::kPriorGate;
  before.island_max_cells = 0;
  const SurfacePlan pb = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, before);
  EXPECT_TRUE(pb.front.from_fallback) << pb.diagnostics;
  EXPECT_EQ(pb.n_valid_cells, 0) << pb.diagnostics;
  EXPECT_TRUE(pb.candidates.empty()) << pb.diagnostics;
  EXPECT_GT(pb.n_rejected_edge, 0);
  // DEPOIS (defaults): ilhas removidas, frente da camera aceita a 12 cm do
  // prior, candidatos dentro da zona REAL.
  const SurfacePlan pa = manip_task_execution::planSurfacePlacement(g, SurfacePlaceRules{}, {}, o);
  EXPECT_EQ(pa.n_islands_removed, 14) << pa.diagnostics;
  ASSERT_TRUE(pa.front.found);
  EXPECT_FALSE(pa.front.from_fallback) << pa.diagnostics;
  EXPECT_NEAR(pa.front.y0, 0.28, 0.011);
  EXPECT_GT(pa.n_valid_cells, 0) << pa.diagnostics;
  ASSERT_FALSE(pa.candidates.empty()) << pa.diagnostics;
  for (const auto & c : pa.candidates) {
    EXPECT_GE(c.y, 0.28 + 0.0405 - 0.011) << c.x << "," << c.y;
    EXPECT_LE(c.y, 0.28 + 0.20 - 0.0205 + 0.011) << c.x << "," << c.y;
    EXPECT_GE(std::hypot(c.x - 0.10, c.y - 0.45), 0.0205 + 0.029 + 0.02 - 0.012);
    EXPECT_GE(std::hypot(c.x + 0.15, c.y - 0.40), 0.0205 + 0.029 + 0.02 - 0.012);
  }
  EXPECT_EQ(pa.n_valid_cells + pa.n_rejected_edge + pa.n_rejected_object + pa.n_rejected_unknown +
    pa.n_rejected_zone + pa.n_rejected_wall + pa.n_rejected_reach, pa.n_table_free);
}

TEST(SurfacePlacePlanner, ParseFrontPriorMode)
{
  FrontPriorMode m = FrontPriorMode::kPriorGate;
  EXPECT_TRUE(manip_task_execution::parseFrontPriorMode("camera_first", &m));
  EXPECT_EQ(m, FrontPriorMode::kCameraFirst);
  EXPECT_TRUE(manip_task_execution::parseFrontPriorMode(" Prior-Gate ", &m));
  EXPECT_EQ(m, FrontPriorMode::kPriorGate);
  EXPECT_FALSE(manip_task_execution::parseFrontPriorMode("xyz", &m));
  EXPECT_EQ(m, FrontPriorMode::kPriorGate);
  EXPECT_STREQ(manip_task_execution::frontPriorModeName(FrontPriorMode::kCameraFirst), "camera_first");
  EXPECT_STREQ(manip_task_execution::frontPriorModeName(FrontPriorMode::kPriorGate), "prior_gate");
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
