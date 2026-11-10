// Testes da logica pura do escape (grade sintetica, sem ROS).
#include <cmath>
#include <functional>
#include <set>
#include <utility>

#include "caramelo_behaviors/escape_scoring.hpp"
#include "gtest/gtest.h"

using caramelo_behaviors::ChoiceParams;
using caramelo_behaviors::chooseEscape;
using caramelo_behaviors::convexHull;
using caramelo_behaviors::defaultDirections;
using caramelo_behaviors::EscapeDirection;
using caramelo_behaviors::PoseEval;
using caramelo_behaviors::RayScore;
using caramelo_behaviors::scoreRay;

namespace
{
constexpr double kRes = 0.02;
// Footprint do Caramelo (retangulo envolvente): x -0.285..+0.395, y +-0.195.
constexpr double kFront = 0.395, kBack = 0.285, kHalfW = 0.195;

// Avaliador sintetico: cost_fn(cx, cy) e' o custo da celula; rasteriza o
// retangulo orientado em passos de celula; keys0 = celulas ruins iniciais.
struct World
{
  std::function<double(double, double)> cost;
  std::set<std::pair<long, long>> keys0;

  PoseEval eval(double x, double y, double yaw, std::set<std::pair<long, long>> * collect = nullptr)
  {
    PoseEval e;
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (double bx = -kBack + kRes / 2; bx < kFront; bx += kRes) {
      for (double by = -kHalfW + kRes / 2; by < kHalfW; by += kRes) {
        const double wx = x + bx * c - by * s;
        const double wy = y + bx * s + by * c;
        const auto key = std::make_pair(std::lround(std::floor(wx / kRes)),
            std::lround(std::floor(wy / kRes)));
        const double cost = this->cost(wx, wy);
        if (cost >= 254.0) {
          if (keys0.count(key)) {++e.old_bad;} else {++e.new_bad;}
          if (collect) {collect->insert(key);}
        } else {
          e.max_cost = std::max(e.max_cost, cost);
          e.sum_cost += cost;
          ++e.n_cells;
        }
      }
    }
    return e;
  }
  int start(double x, double y, double yaw)
  {
    keys0.clear();
    PoseEval e = eval(x, y, yaw, &keys0);
    return e.new_bad;
  }
  std::vector<RayScore> rays(
    const std::vector<EscapeDirection> & dirs, double x, double y, double yaw, double dist)
  {
    std::vector<RayScore> out;
    for (const auto & d : dirs) {
      out.push_back(scoreRay([this](double px, double py, double pyaw) {return eval(px, py, pyaw);},
        x, y, yaw, d, dist, kRes));
    }
    return out;
  }
};

// Parede letal em x >= wall_x; custo cai com a distancia (253 encostado -> 0 a 0.4 m).
World wallWorld(double wall_x)
{
  World w;
  w.cost = [wall_x](double x, double) {
      if (x >= wall_x) {return 254.0;}
      return std::max(0.0, 253.0 * (1.0 - (wall_x - x) / 0.4));
    };
  return w;
}
}  // namespace

TEST(EscapeScoring, StuckWithFrontOnWallEscapesBackward)
{
  // Parede em x >= 0.30: a frente (0.395) ja esta 9.5 cm DENTRO do letal.
  World w = wallWorld(0.30);
  const int bad0 = w.start(0.0, 0.0, 0.0);
  ASSERT_GT(bad0, 0);
  const auto dirs = defaultDirections(true);
  const auto scores = w.rays(dirs, 0.0, 0.0, 0.0, 0.20);
  EXPECT_TRUE(scores[0].blocked);          // frente: entra mais na parede
  EXPECT_FALSE(scores.back().blocked);     // re: so' sai de cima
  EXPECT_EQ(scores.back().end_old_bad, 0); // 0.20 m de re > 9.5 cm de sobreposicao
  const auto c = chooseEscape(dirs, scores, bad0, 253.0, ChoiceParams{});
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(dirs[c->index].name, "re");
}

TEST(EscapeScoring, LateralDoesNotCountAsEscapeFromInfiniteWall)
{
  // Deslizar de lado ao longo da parede: entra em celulas letais NOVAS -> bloqueado.
  World w = wallWorld(0.30);
  w.start(0.0, 0.0, 0.0);
  const auto dirs = defaultDirections(true);
  const auto scores = w.rays(dirs, 0.0, 0.0, 0.0, 0.20);
  EXPECT_TRUE(scores[3].blocked);  // esquerda
  EXPECT_TRUE(scores[4].blocked);  // direita
}

TEST(EscapeScoring, WithoutBackwardStaysPut)
{
  World w = wallWorld(0.30);
  const int bad0 = w.start(0.0, 0.0, 0.0);
  const auto dirs = defaultDirections(false);
  const auto scores = w.rays(dirs, 0.0, 0.0, 0.0, 0.20);
  EXPECT_FALSE(chooseEscape(dirs, scores, bad0, 253.0, ChoiceParams{}).has_value());
}

TEST(EscapeScoring, HighCostButNotTouchingPrefersMovingAway)
{
  // Parede em x >= 0.50: frente a 10.5 cm dela, custo alto mas sem letal.
  World w = wallWorld(0.50);
  const int bad0 = w.start(0.0, 0.0, 0.0);
  EXPECT_EQ(bad0, 0);
  const PoseEval start = w.eval(0.0, 0.0, 0.0);
  EXPECT_GT(start.max_cost, 128.0);
  const auto dirs = defaultDirections(true);
  const auto scores = w.rays(dirs, 0.0, 0.0, 0.0, 0.20);
  const auto c = chooseEscape(dirs, scores, 0, start.max_cost, ChoiceParams{});
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(dirs[c->index].name, "re");
  EXPECT_LT(c->score.end_cost, start.max_cost - 10.0);
}

TEST(EscapeScoring, LowCostEverywherePrefersForward)
{
  World w;
  w.cost = [](double, double) {return 20.0;};
  const int bad0 = w.start(1.0, 2.0, 0.7);
  const auto dirs = defaultDirections(true);
  const auto scores = w.rays(dirs, 1.0, 2.0, 0.7, 0.20);
  const auto c = chooseEscape(dirs, scores, bad0, 20.0, ChoiceParams{});
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(dirs[c->index].name, "frente");
  EXPECT_NEAR(scores[c->index].free_len, 0.20, 1e-9);
}

TEST(EscapeScoring, UnknownBehindBlocksReverse)
{
  World w;
  w.cost = [](double x, double) {return x < -0.30 ? 255.0 : 10.0;};
  const int bad0 = w.start(0.0, 0.0, 0.0);
  EXPECT_EQ(bad0, 0);
  const auto dirs = defaultDirections(true);
  const auto scores = w.rays(dirs, 0.0, 0.0, 0.0, 0.20);
  EXPECT_TRUE(scores.back().blocked);  // re entra no desconhecido
  EXPECT_FALSE(scores[0].blocked);
}

TEST(EscapeScoring, RayRespectsYaw)
{
  // Robo virado 90 graus: "frente" do corpo e' +y do mundo; parede em y >= 0.50.
  World w;
  w.cost = [](double, double y) {return y >= 0.50 ? 254.0 : 5.0;};
  w.start(0.0, 0.0, M_PI / 2.0);
  const auto dirs = defaultDirections(true);
  const auto scores = w.rays(dirs, 0.0, 0.0, M_PI / 2.0, 0.20);
  EXPECT_TRUE(scores[0].blocked);       // frente (+y mundo) bate em 0.105 m
  EXPECT_LT(scores[0].free_len, 0.12);
  EXPECT_FALSE(scores.back().blocked);  // re (-y mundo)
}

TEST(EscapeScoring, TieBreaksByPreference)
{
  World w;
  w.cost = [](double, double) {return 0.0;};
  w.start(0.0, 0.0, 0.0);
  const auto dirs = defaultDirections(true);
  const auto scores = w.rays(dirs, 0.0, 0.0, 0.0, 0.20);
  ChoiceParams p;
  p.preference_weight = 0.0;
  const auto c = chooseEscape(dirs, scores, 0, 0.0, p);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(dirs[c->index].preference, 0);
}

TEST(EscapeScoring, ConvexHullCoversNotch)
{
  // Footprint real com recorte na frente: o casco vira o retangulo envolvente.
  std::vector<std::pair<double, double>> fp = {
    {0.395, 0.055}, {0.395, -0.055}, {0.285, -0.055}, {0.285, -0.195},
    {-0.285, -0.195}, {-0.285, 0.195}, {0.285, 0.195}, {0.285, 0.055}};
  const auto hull = convexHull(fp);
  EXPECT_EQ(hull.size(), 6u);
  double maxx = -1, minx = 1, maxy = -1, miny = 1;
  for (const auto & p : hull) {
    maxx = std::max(maxx, p.first); minx = std::min(minx, p.first);
    maxy = std::max(maxy, p.second); miny = std::min(miny, p.second);
  }
  EXPECT_NEAR(maxx, 0.395, 1e-9);
  EXPECT_NEAR(minx, -0.285, 1e-9);
  EXPECT_NEAR(maxy, 0.195, 1e-9);
  EXPECT_NEAR(miny, -0.195, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
