// Testes dos helpers puros do empilhamento (sem ROS, 2026-10-21):
// classificacao do stack_on, giro de 90 graus mais proximo, cota da pilha
// pela memoria e a decisao de verificacao apos soltar.
#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "manip_task_execution/place_precision.hpp"

using manip_task_execution::classifyStackTarget;
using manip_task_execution::decidePlacement;
using manip_task_execution::expectedPileTopZ;
using manip_task_execution::findStackMemory;
using manip_task_execution::nearestQuarterTurn;
using manip_task_execution::PileFix;
using manip_task_execution::pileFixFor;
using manip_task_execution::pileRegraspFloorZ;
using manip_task_execution::PlacementDecision;
using manip_task_execution::PlacementLimits;
using manip_task_execution::PlacementObservation;
using manip_task_execution::stackLevelFromMemory;
using manip_task_execution::findStackRoot;
using manip_task_execution::StackMemoryEntry;
using manip_task_execution::StackTargetKind;
using manip_task_execution::tagIdFromFrame;
using manip_task_execution::tiltDropZCompensation;

namespace
{

constexpr double kTol = 1e-9;
constexpr double kDeg = M_PI / 180.0;
const std::vector<std::int64_t> kRefIds{200};

double wrap90(double a)
{
  return std::remainder(a, M_PI / 2.0);
}

}  // namespace

// ---- classificacao -------------------------------------------------------

TEST(PlacePrecision, TagIdFromFrame)
{
  EXPECT_EQ(tagIdFromFrame("tag_7").value_or(-1), 7);
  EXPECT_EQ(tagIdFromFrame("tag_200").value_or(-1), 200);
  EXPECT_EQ(tagIdFromFrame("tag_0").value_or(-1), 0);
  EXPECT_FALSE(tagIdFromFrame("tag_").has_value());
  EXPECT_FALSE(tagIdFromFrame("tag_x").has_value());
  EXPECT_FALSE(tagIdFromFrame("tag_1a").has_value());
  EXPECT_FALSE(tagIdFromFrame("ct_vermelho").has_value());
  EXPECT_FALSE(tagIdFromFrame("").has_value());
  EXPECT_FALSE(tagIdFromFrame("tag_slot_3").has_value());
}

TEST(PlacePrecision, ClassifyStackTarget)
{
  EXPECT_EQ(classifyStackTarget("tag_200", kRefIds), StackTargetKind::kReference);
  EXPECT_EQ(classifyStackTarget("tag_4", kRefIds), StackTargetKind::kCube);
  // 2026-11-16: os ids dos slots da mesa de precisao (101..107) nao sao mais
  // especiais: a PP foi removida.
  EXPECT_EQ(classifyStackTarget("tag_104", kRefIds), StackTargetKind::kCube);
  // Frame que nao casa 'tag_<n>' = cubo.
  EXPECT_EQ(classifyStackTarget("ct_vermelho", kRefIds), StackTargetKind::kCube);
  EXPECT_EQ(classifyStackTarget("", kRefIds), StackTargetKind::kCube);
  // Lista vazia: tudo e' cubo.
  EXPECT_EQ(classifyStackTarget("tag_200", {}), StackTargetKind::kCube);
}

// ---- giro de 90 graus ----------------------------------------------------

TEST(PlacePrecision, NearestQuarterTurnAlignsModulo90)
{
  const double yaws_tag[] = {0.0, 0.3, -1.2, 2.9, -3.1, 1.0};
  const double yaws_cur[] = {0.0, 0.5, -2.0, 3.0, -0.1, 1.7, -3.0, 2.2};
  for (const double t : yaws_tag) {
    for (const double c : yaws_cur) {
      const double d = nearestQuarterTurn(t, c);
      EXPECT_LE(std::abs(d), M_PI / 4.0 + kTol) << t << " " << c;
      // c + d e' congruente a t modulo 90 graus.
      EXPECT_NEAR(wrap90((c + d) - t), 0.0, 1e-9) << t << " " << c;
    }
  }
}

TEST(PlacePrecision, NearestQuarterTurnPicksSmallestTurn)
{
  // Tag a 80 graus, punho a 0: o multiplo mais proximo e' 90 -> delta -10.
  EXPECT_NEAR(nearestQuarterTurn(80.0 * kDeg, 0.0), -10.0 * kDeg, 1e-12);
  // Tag a 10 graus, punho a 0: delta +10.
  EXPECT_NEAR(nearestQuarterTurn(10.0 * kDeg, 0.0), 10.0 * kDeg, 1e-12);
  // Tag a 95, punho a 0: 95 ~ 5 (mod 90) -> +5.
  EXPECT_NEAR(nearestQuarterTurn(95.0 * kDeg, 0.0), 5.0 * kDeg, 1e-12);
  // Punho ja alinhado (mesmo yaw + 180): zero.
  EXPECT_NEAR(nearestQuarterTurn(0.4, 0.4 + M_PI), 0.0, 1e-12);
  // Perto do empate: 44 graus -> +44 (nao -46), 46 graus -> -44 (nao +46).
  EXPECT_NEAR(nearestQuarterTurn(44.0 * kDeg, 0.0), 44.0 * kDeg, 1e-12);
  EXPECT_NEAR(nearestQuarterTurn(46.0 * kDeg, 0.0), -44.0 * kDeg, 1e-12);
  // Empate exato em 45: um dos dois lados, modulo 45.
  const double d45 = nearestQuarterTurn(45.0 * kDeg, 0.0);
  EXPECT_NEAR(std::abs(d45), 45.0 * kDeg, 1e-12);
  // Volta inteira na tag nao muda nada.
  EXPECT_NEAR(
    nearestQuarterTurn(0.7 + 2.0 * M_PI, 0.2), nearestQuarterTurn(0.7, 0.2), 1e-12);
}

// ---- soltura inclinada ---------------------------------------------------

TEST(PlacePrecision, TiltDropZCompensation)
{
  const double w = 0.042;
  const double h = 0.042;
  EXPECT_NEAR(tiltDropZCompensation(0.0, w, h), 0.0, kTol);
  // Quina mais baixa a 15 graus: 42*cos15 + 21*sin15 = 40,57 + 5,44 = 46,0 mm
  // abaixo do TCP -> 4,0 mm a mais que na vertical.
  EXPECT_NEAR(tiltDropZCompensation(15.0 * kDeg, w, h), 0.004004, 1e-5);
  // 30 graus: 42*0,866 + 21*0,5 = 46,87 mm -> 4,87 mm.
  EXPECT_NEAR(tiltDropZCompensation(30.0 * kDeg, w, h), 0.004873, 1e-5);
  // Pequenas inclinacoes: cresce com o angulo; simetrica no sinal; nunca
  // negativa (ate' 45 graus com cubo 42x42: 2,5 mm).
  EXPECT_LT(tiltDropZCompensation(10.0 * kDeg, w, h), tiltDropZCompensation(20.0 * kDeg, w, h));
  EXPECT_NEAR(
    tiltDropZCompensation(-15.0 * kDeg, w, h), tiltDropZCompensation(15.0 * kDeg, w, h), kTol);
  EXPECT_GE(tiltDropZCompensation(45.0 * kDeg, w, h), 0.0);
  EXPECT_GE(tiltDropZCompensation(80.0 * kDeg, w, h), 0.0);
  EXPECT_NEAR(tiltDropZCompensation(45.0 * kDeg, 0.0, 0.0), 0.0, kTol);
  // Cubo mais largo que alto: a largura pesa no seno.
  EXPECT_GT(tiltDropZCompensation(15.0 * kDeg, 0.08, h), tiltDropZCompensation(15.0 * kDeg, w, h));
}

// ---- correcao da pilha ---------------------------------------------------

TEST(PlacePrecision, PileFixFallenCubeRedropsAtOriginal)
{
  const double top = 0.120;   // topo esperado do cubo novo
  const double h = 0.042;
  const double tol = 0.012;
  // Piso de re-pega: um cubo abaixo do topo esperado, menos a tolerancia.
  EXPECT_NEAR(pileRegraspFloorZ(top, h, tol), 0.066, kTol);
  // Sobre a base, deslocado: corrigir xy (ate' a tolerancia em z).
  EXPECT_EQ(pileFixFor(top, top, h, tol), PileFix::kCorrectXy);
  EXPECT_EQ(pileFixFor(top + tol - 1e-6, top, h, tol), PileFix::kCorrectXy);
  EXPECT_EQ(pileFixFor(top - tol + 1e-6, top, h, tol), PileFix::kCorrectXy);
  EXPECT_EQ(pileFixFor(top + tol + 1e-6, top, h, tol), PileFix::kRedropOriginal);
  // Caiu um nivel (mesa ao lado de uma base de nivel 1): alvo original.
  EXPECT_EQ(pileFixFor(top - h, top, h, tol), PileFix::kRedropOriginal);
  EXPECT_EQ(pileFixFor(top - h - tol, top, h, tol), PileFix::kRedropOriginal);
  // Tombado sobre a base (acima ou abaixo da tolerancia): alvo original.
  EXPECT_EQ(pileFixFor(top + 0.020, top, h, tol), PileFix::kRedropOriginal);
  EXPECT_EQ(pileFixFor(top - 0.020, top, h, tol), PileFix::kRedropOriginal);
  // Abaixo do piso: nao re-pegar.
  EXPECT_EQ(pileFixFor(top - h - tol - 1e-6, top, h, tol), PileFix::kLeave);
  EXPECT_EQ(pileFixFor(top - 2.0 * h, top, h, tol), PileFix::kLeave);
}

// ---- pilha ---------------------------------------------------------------

TEST(PlacePrecision, ExpectedPileTopZLevels1To4)
{
  const double z_ref = 0.020;  // tag 200 no tampo
  const double h = 0.041;
  EXPECT_NEAR(expectedPileTopZ(z_ref, 0, h), 0.020, kTol);
  EXPECT_NEAR(expectedPileTopZ(z_ref, 1, h), 0.061, kTol);
  EXPECT_NEAR(expectedPileTopZ(z_ref, 2, h), 0.102, kTol);
  EXPECT_NEAR(expectedPileTopZ(z_ref, 3, h), 0.143, kTol);
  EXPECT_NEAR(expectedPileTopZ(z_ref, 4, h), 0.184, kTol);
  EXPECT_NEAR(expectedPileTopZ(z_ref, -2, h), 0.020, kTol);
}

TEST(PlacePrecision, StackLevelFromMemoryChain)
{
  // A sobre tag_200, B sobre A, C sobre B (3 cubos). D solto numa mesa
  // comum (slot Mesa15.1) nao conta como pilha.
  std::vector<StackMemoryEntry> mem{
    {"stack:tag_200", "tag_3", 0.0, 0.3, 0.061},
    {"stack:tag_3", "tag_5", 0.0, 0.3, 0.102},
    {"Mesa15.1", "tag_9", 0.1, 0.3, 0.061},
    {"stack:tag_5", "tag_7", 0.0, 0.3, 0.143},
  };
  EXPECT_EQ(stackLevelFromMemory(mem, "tag_200"), 0);
  EXPECT_EQ(stackLevelFromMemory(mem, "tag_3"), 1);
  EXPECT_EQ(stackLevelFromMemory(mem, "tag_5"), 2);
  EXPECT_EQ(stackLevelFromMemory(mem, "tag_7"), 3);
  EXPECT_EQ(stackLevelFromMemory(mem, "tag_9"), 0);
  EXPECT_EQ(stackLevelFromMemory(mem, "tag_42"), 0);
  EXPECT_EQ(stackLevelFromMemory({}, "tag_7"), 0);
  // Nivel do PROXIMO cubo = nivel da base + 1: 4o cubo sobre tag_7.
  EXPECT_EQ(stackLevelFromMemory(mem, "tag_7") + 1, 4);
  const auto top = findStackMemory(mem, "tag_7");
  ASSERT_TRUE(top.has_value());
  EXPECT_NEAR(top->z, 0.143, kTol);
  EXPECT_NEAR(expectedPileTopZ(top->z, 1, 0.041), 0.184, kTol);
}

TEST(PlacePrecision, StackLevelFromMemoryCycleAndLastWins)
{
  // Ciclo (dados corrompidos): nao trava.
  std::vector<StackMemoryEntry> cyc{
    {"stack:tag_2", "tag_1", 0, 0, 0.1},
    {"stack:tag_1", "tag_2", 0, 0, 0.1},
  };
  EXPECT_EQ(stackLevelFromMemory(cyc, "tag_1"), 2);
  // Dois registros da mesma tag: o ultimo vence.
  std::vector<StackMemoryEntry> dup{
    {"stack:tag_200", "tag_1", 0.0, 0.0, 0.061},
    {"stack:tag_200", "tag_1", 0.1, 0.2, 0.063},
  };
  const auto r = findStackMemory(dup, "tag_1");
  ASSERT_TRUE(r.has_value());
  EXPECT_NEAR(r->x, 0.1, kTol);
  EXPECT_NEAR(r->z, 0.063, kTol);
  EXPECT_FALSE(findStackMemory(dup, "tag_200").has_value());
}

// ---- decisao de verificacao (pilha) ---------------------------------------

TEST(PlacePrecision, DecidePlacementGenericBudgetAndProxy)
{
  PlacementLimits lim;
  lim.xy_tol_m = 0.008;
  lim.z_tol_m = 0.008;
  lim.fk_proxy_tol_m = 0.008;
  lim.max_attempts = 6;
  lim.retry_unseen = true;

  PlacementObservation ok;
  ok.seen = true;
  ok.err_xy_m = 0.005;
  ok.err_z_m = -0.004;
  lim.attempts_done = 1;
  EXPECT_EQ(decidePlacement(ok, lim), PlacementDecision::kAccept);

  PlacementObservation unseated = ok;
  unseated.err_z_m = 0.041;   // apoiado por cima
  EXPECT_EQ(decidePlacement(unseated, lim), PlacementDecision::kRetry);
  lim.attempts_done = 6;
  EXPECT_EQ(decidePlacement(unseated, lim), PlacementDecision::kGiveUp);
  lim.attempts_done = 5;
  EXPECT_EQ(decidePlacement(unseated, lim), PlacementDecision::kRetry);

  PlacementObservation off_xy = ok;
  off_xy.err_xy_m = 0.012;
  lim.attempts_done = 1;
  EXPECT_EQ(decidePlacement(off_xy, lim), PlacementDecision::kRetry);

  // Nao vista: proxy FK decide.
  PlacementObservation unseen;
  unseen.seen = false;
  unseen.fk_err_xy_m = 0.004;
  EXPECT_EQ(decidePlacement(unseen, lim), PlacementDecision::kAcceptUnverified);
  unseen.fk_err_xy_m = 0.02;
  EXPECT_EQ(decidePlacement(unseen, lim), PlacementDecision::kRetry);
  lim.attempts_done = 6;
  EXPECT_EQ(decidePlacement(unseen, lim), PlacementDecision::kGiveUp);
  // Sem estado (fk < 0) e sem orcamento: desiste; com orcamento: repete.
  unseen.fk_err_xy_m = -1.0;
  EXPECT_EQ(decidePlacement(unseen, lim), PlacementDecision::kGiveUp);
  lim.attempts_done = 1;
  EXPECT_EQ(decidePlacement(unseen, lim), PlacementDecision::kRetry);
  lim.retry_unseen = false;
  EXPECT_EQ(decidePlacement(unseen, lim), PlacementDecision::kGiveUp);
}

TEST(PlacePrecision, DecidePlacementPileOneCorrection)
{
  PlacementLimits lim;
  lim.xy_tol_m = 0.012;
  lim.z_tol_m = 0.012;
  lim.fk_proxy_tol_m = -1.0;   // pilha: nunca re-pega as cegas
  lim.max_attempts = 2;
  lim.retry_unseen = false;

  PlacementObservation obs;
  obs.seen = true;
  obs.err_xy_m = 0.02;
  obs.err_z_m = 0.0;
  lim.attempts_done = 1;
  EXPECT_EQ(decidePlacement(obs, lim), PlacementDecision::kRetry);
  lim.attempts_done = 2;
  EXPECT_EQ(decidePlacement(obs, lim), PlacementDecision::kGiveUp);

  obs.err_xy_m = 0.005;
  obs.err_z_m = 0.012;
  EXPECT_EQ(decidePlacement(obs, lim), PlacementDecision::kAccept);
  obs.err_z_m = -0.0121;
  lim.attempts_done = 1;
  EXPECT_EQ(decidePlacement(obs, lim), PlacementDecision::kRetry);

  PlacementObservation unseen;
  unseen.seen = false;
  unseen.fk_err_xy_m = 0.05;
  EXPECT_EQ(decidePlacement(unseen, lim), PlacementDecision::kAcceptUnverified);
}

TEST(PlacePrecision, FindStackRootSegueACadeiaAteOPrimeiroCubo)
{
  // A (tag_3) sobre tag_200, B (tag_7) sobre A, C (tag_9) sobre B.
  const std::vector<StackMemoryEntry> mem{
    {"stack:tag_200", "tag_3", 0.06, 0.36, 0.037},
    {"stack:tag_3", "tag_7", 0.07, 0.37, 0.079},
    {"stack:tag_7", "tag_9", 0.08, 0.38, 0.121},
  };
  for (const char * f : {"tag_3", "tag_7", "tag_9"}) {
    const auto r = findStackRoot(mem, f);
    ASSERT_TRUE(r.has_value()) << f;
    EXPECT_EQ(r->tag_frame, "tag_3") << f;
    EXPECT_DOUBLE_EQ(r->x, 0.06);
  }
  EXPECT_FALSE(findStackRoot(mem, "tag_200").has_value());
  EXPECT_FALSE(findStackRoot(mem, "tag_99").has_value());
}
