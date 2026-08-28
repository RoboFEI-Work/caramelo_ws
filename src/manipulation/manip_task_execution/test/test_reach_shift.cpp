// Testes da busca de deslocamento lateral verificada pela IK
// (fila de alcance, 2026-08-28).
//
// Os alvos abaixo foram CALIBRADOS rodando a IK propria (nao chutados). O
// alcance e' emergente do modelo + limites de junta + familia frontal;
// medido em 2026-08-28 com os defaults de ArmModel/IkOptions:
//   z = 0,04 (mesa): raio max ~0,370 (tilt 0) / ~0,440 (15 graus) / ~0,513 (30 graus)
//   z = 0,09 (mesa + lift 0,05): ~0,335 / ~0,410 / ~0,489
// praticamente isotropico em x. Os alvos ficam a >= ~1 cm de cada fronteira
// para nao oscilar com detalhes numericos. Se alguem mudar o modelo, os
// limites ou a escada de tilts, recalibrar com `custom_ik_check shift x y z`.
//
// Convencao: alvos em manip_base_link (+X = direita, +Y = frente); shift em
// base_footprint (+ = esquerda); x do alvo depois do shift = x + dy.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "manip_task_execution/custom_ik.hpp"
#include "manip_task_execution/reach_shift.hpp"

using manip_task_execution::findBaseShiftForReach;
using manip_task_execution::forwardKinematics;
using manip_task_execution::ReachShiftOptions;
using manip_task_execution::ReachShiftResult;
using manip_task_execution::solveIk;

namespace
{

constexpr double kTol = 1e-9;
constexpr double kTableZ = 0.04;

/// Escada completa de tilts num alvo, com as MESMAS opcoes da busca
/// (verificacao independente do que a biblioteca devolveu).
bool solvesDirect(const Eigen::Vector3d & target, const ReachShiftOptions & opts)
{
  for (const double tilt : opts.tilts_rad) {
    std::array<double, 5> q{};
    if (solveIk(target, tilt, opts.q5_fixed, q, opts.model, opts.ik)) {
      return true;
    }
  }
  return false;
}

bool isCandidate(double value, const std::vector<double> & candidates)
{
  return std::any_of(
    candidates.begin(), candidates.end(),
    [value](double c) {return std::abs(c - value) < kTol;});
}

bool isTilt(double value, const std::vector<double> & tilts)
{
  return std::any_of(
    tilts.begin(), tilts.end(),
    [value](double t) {return std::abs(t - value) < kTol;});
}

/// Nenhum candidato menor que |shift| resolve (com o mesmo desconto de
/// undershoot que a busca aplica) em nenhum dos alvos.
void expectNoSmallerCandidateSolves(
  const std::vector<Eigen::Vector3d> & targets, const ReachShiftResult & r,
  const ReachShiftOptions & opts)
{
  const double sign = r.shift_m > 0.0 ? 1.0 : -1.0;
  for (const double d : opts.candidates_m) {
    if (d >= std::abs(r.shift_m) - kTol) {
      continue;
    }
    const double dy_eff = sign * d - sign * opts.undershoot_margin_m;
    for (const auto & t : targets) {
      const Eigen::Vector3d shifted(t.x() + dy_eff, t.y(), t.z());
      EXPECT_FALSE(solvesDirect(shifted, opts))
        << "candidato menor " << d << " resolve em (" << shifted.transpose() << ")";
    }
  }
}

/// A solucao devolvida (q) leva o TCP ao alvo deslocado.
void expectSolutionReachesTarget(const ReachShiftResult & r, const ReachShiftOptions & opts)
{
  Eigen::Vector3d tcp;
  Eigen::Matrix3d rot;
  forwardKinematics(r.q, opts.model, tcp, rot);
  EXPECT_LT((tcp - r.shifted_target).norm(), 1e-3);
}

}  // namespace

TEST(ReachShift, EmptyTargetsReturnsNothing)
{
  const ReachShiftResult r = findBaseShiftForReach(std::vector<Eigen::Vector3d>{}, ReachShiftOptions{});
  EXPECT_FALSE(r.reachable_now);
  EXPECT_DOUBLE_EQ(r.shift_m, 0.0);
}

TEST(ReachShift, ReachableNow)
{
  const ReachShiftOptions opts;
  const Eigen::Vector3d target(0.0, 0.30, kTableZ);  // raio 0,30 < 0,370 (topo estrito)
  const ReachShiftResult r = findBaseShiftForReach(target, opts);

  EXPECT_TRUE(r.reachable_now);
  EXPECT_DOUBLE_EQ(r.shift_m, 0.0);
  EXPECT_NEAR(r.tilt_rad, 0.0, kTol);  // 1o degrau da escada ja resolve
  EXPECT_LT((r.shifted_target - target).norm(), kTol);
  expectSolutionReachesTarget(r, opts);
}

TEST(ReachShift, RightTargetNeedsRightShift)
{
  const ReachShiftOptions opts;
  // Raio 0,580: fora ate com 30 graus. Com -0,10 (efetivo -0,08) x=0,32,
  // raio 0,526 > 0,513: ainda fora. Com -0,15 (efetivo -0,13) x=0,27,
  // raio 0,499: resolve a 30 graus.
  const Eigen::Vector3d target(0.40, 0.42, kTableZ);
  const ReachShiftResult r = findBaseShiftForReach(target, opts);

  EXPECT_FALSE(r.reachable_now);
  EXPECT_LT(r.shift_m, 0.0);  // alvo a direita => base vai para a direita
  EXPECT_TRUE(isCandidate(std::abs(r.shift_m), opts.candidates_m));
  EXPECT_NEAR(r.shift_m, -0.15, kTol);
  EXPECT_TRUE(isTilt(r.tilt_rad, opts.tilts_rad));
  EXPECT_NEAR(r.shifted_target.x(), 0.40 - (0.15 - opts.undershoot_margin_m), kTol);
  EXPECT_NEAR(r.shifted_target.y(), target.y(), kTol);
  EXPECT_NEAR(r.shifted_target.z(), target.z(), kTol);

  std::array<double, 5> q{};
  EXPECT_TRUE(solveIk(r.shifted_target, r.tilt_rad, opts.q5_fixed, q, opts.model, opts.ik));
  expectSolutionReachesTarget(r, opts);
  expectNoSmallerCandidateSolves({target}, r, opts);
}

TEST(ReachShift, LeftTargetNeedsLeftShift)
{
  const ReachShiftOptions opts;
  const Eigen::Vector3d target(-0.40, 0.42, kTableZ);  // espelho do teste anterior
  const ReachShiftResult r = findBaseShiftForReach(target, opts);

  EXPECT_FALSE(r.reachable_now);
  EXPECT_GT(r.shift_m, 0.0);  // alvo a esquerda => base vai para a esquerda
  EXPECT_NEAR(r.shift_m, 0.15, kTol);
  EXPECT_NEAR(r.shifted_target.x(), -0.40 + (0.15 - opts.undershoot_margin_m), kTol);

  std::array<double, 5> q{};
  EXPECT_TRUE(solveIk(r.shifted_target, r.tilt_rad, opts.q5_fixed, q, opts.model, opts.ik));
  expectSolutionReachesTarget(r, opts);
  expectNoSmallerCandidateSolves({target}, r, opts);
}

TEST(ReachShift, StraightAheadTooFarGivesZero)
{
  const ReachShiftOptions opts;
  // |x| < 3 cm: de lado nao ajuda; raio 0,56 > 0,510 mesmo a 30 graus.
  const Eigen::Vector3d target(0.0, 0.56, kTableZ);
  const ReachShiftResult r = findBaseShiftForReach(target, opts);

  EXPECT_FALSE(r.reachable_now);
  EXPECT_DOUBLE_EQ(r.shift_m, 0.0);
  EXPECT_FALSE(solvesDirect(target, opts));
}

TEST(ReachShift, NoOvershootStopsAtFirstTarget)
{
  // 1o alvo a x=0,06 decide a direcao (direita) e limita o curso: com -0,10
  // (efetivo -0,08) fica em x=-0,02 (|x| < 0,06, permitido); -0,15 levaria
  // a x=-0,07 (|x| > 0,06): overshoot, a busca para. O 2o alvo so resolve
  // com -0,15 ou mais (x=0,29, raio 0,494) - sem overshoot nao pode ser
  // usado; com overshoot liberado, e' ele quem vence.
  const std::vector<Eigen::Vector3d> targets{
    Eigen::Vector3d(0.06, 0.53, kTableZ),   // raio 0,533 > 0,513: fora
    Eigen::Vector3d(0.42, 0.40, kTableZ)};  // raio 0,580: fora; x=0,34 (d=0,10) ainda fora

  ReachShiftOptions opts;
  const ReachShiftResult blocked = findBaseShiftForReach(targets, opts);
  EXPECT_FALSE(blocked.reachable_now);
  EXPECT_DOUBLE_EQ(blocked.shift_m, 0.0);
  // O unico candidato permitido (0,10) de fato nao resolve em nenhum alvo.
  EXPECT_FALSE(solvesDirect(Eigen::Vector3d(0.06 - 0.08, 0.53, kTableZ), opts));
  EXPECT_FALSE(solvesDirect(Eigen::Vector3d(0.42 - 0.08, 0.40, kTableZ), opts));

  opts.allow_overshoot = true;
  const ReachShiftResult allowed = findBaseShiftForReach(targets, opts);
  EXPECT_FALSE(allowed.reachable_now);
  EXPECT_NEAR(allowed.shift_m, -0.15, kTol);
  EXPECT_NEAR(allowed.shifted_target.x(), 0.42 - 0.13, kTol);
  EXPECT_NEAR(allowed.shifted_target.y(), 0.40, kTol);
  expectSolutionReachesTarget(allowed, opts);
}

TEST(ReachShift, LiftRequiredPushesToLargerShift)
{
  // Sem lift: -0,15 leva a (0,27, 0,42), raio 0,499 < 0,513: resolve.
  // Com lift 0,05 o mesmo ponto em z=0,09 tem raio 0,499 > 0,489: NAO
  // resolve; a busca precisa ir a -0,20 -> (0,22, 0,42), raio 0,474 < 0,489.
  const Eigen::Vector3d target(0.40, 0.42, kTableZ);

  ReachShiftOptions no_lift;
  const ReachShiftResult r0 = findBaseShiftForReach(target, no_lift);
  ASSERT_FALSE(r0.reachable_now);
  ASSERT_NEAR(r0.shift_m, -0.15, kTol);
  // Verificacao direta: a pose elevada do candidato sem lift nao resolve.
  const Eigen::Vector3d lifted0 = r0.shifted_target + Eigen::Vector3d(0.0, 0.0, 0.05);
  EXPECT_FALSE(solvesDirect(lifted0, no_lift));

  ReachShiftOptions with_lift;
  with_lift.lift_m = 0.05;
  const ReachShiftResult r1 = findBaseShiftForReach(target, with_lift);
  EXPECT_FALSE(r1.reachable_now);
  EXPECT_NEAR(r1.shift_m, -0.20, kTol);
  EXPECT_GT(std::abs(r1.shift_m), std::abs(r0.shift_m));
  EXPECT_NEAR(r1.shifted_target.x(), 0.40 - 0.18, kTol);

  std::array<double, 5> q{};
  EXPECT_TRUE(
    solveIk(r1.shifted_target, r1.tilt_rad, with_lift.q5_fixed, q, with_lift.model, with_lift.ik));
  const Eigen::Vector3d lifted1 = r1.shifted_target + Eigen::Vector3d(0.0, 0.0, with_lift.lift_m);
  EXPECT_TRUE(solveIk(lifted1, r1.tilt_rad, with_lift.q5_fixed, q, with_lift.model, with_lift.ik));
  expectSolutionReachesTarget(r1, with_lift);
}

TEST(ReachShift, LiftCanMakeMarginalTargetUnreachable)
{
  // Sem lift so o curso maximo (-0,25 -> x=0,27, raio 0,499) resolve. Com
  // lift 0,05 nem esse: nao ha candidato maior, devolve 0.
  const Eigen::Vector3d target(0.50, 0.42, kTableZ);

  ReachShiftOptions no_lift;
  const ReachShiftResult r0 = findBaseShiftForReach(target, no_lift);
  ASSERT_FALSE(r0.reachable_now);
  ASSERT_NEAR(r0.shift_m, -0.25, kTol);

  ReachShiftOptions with_lift;
  with_lift.lift_m = 0.05;
  const ReachShiftResult r1 = findBaseShiftForReach(target, with_lift);
  EXPECT_FALSE(r1.reachable_now);
  EXPECT_DOUBLE_EQ(r1.shift_m, 0.0);
}

TEST(ReachShift, MultiTargetAnyWins)
{
  const ReachShiftOptions opts;
  const Eigen::Vector3d never(0.30, 0.60, kTableZ);      // raio 0,67: nao resolve nem a -0,25
  const Eigen::Vector3d after_shift(0.36, 0.40, kTableZ);  // raio 0,538; com -0,10 -> x=0,28, raio 0,488

  const ReachShiftResult alone = findBaseShiftForReach(never, opts);
  EXPECT_FALSE(alone.reachable_now);
  EXPECT_DOUBLE_EQ(alone.shift_m, 0.0);

  const ReachShiftResult r =
    findBaseShiftForReach(std::vector<Eigen::Vector3d>{never, after_shift}, opts);
  EXPECT_FALSE(r.reachable_now);
  EXPECT_NEAR(r.shift_m, -0.10, kTol);  // direcao decidida pelo 1o alvo (x > 0)
  EXPECT_NEAR(r.shifted_target.x(), 0.36 - 0.08, kTol);
  EXPECT_NEAR(r.shifted_target.y(), 0.40, kTol);
  expectSolutionReachesTarget(r, opts);
}

TEST(ReachShift, UnsortedCandidatesStillPickSmallest)
{
  ReachShiftOptions opts;
  opts.candidates_m = {0.25, 0.10, 0.20, 0.15};
  const Eigen::Vector3d target(0.40, 0.42, kTableZ);  // mesmo alvo de RightTargetNeedsRightShift
  const ReachShiftResult r = findBaseShiftForReach(target, opts);

  EXPECT_FALSE(r.reachable_now);
  EXPECT_NEAR(r.shift_m, -0.15, kTol);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
