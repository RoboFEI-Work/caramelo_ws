// Testes dos helpers puros do place de precisao (sem ROS, 2026-10-21):
// classificacao do stack_on, giro de 90 graus mais proximo, espiral de
// busca, assentamento no slot, cota da pilha pela memoria e a decisao de
// verificacao apos soltar.
#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "manip_task_execution/place_precision.hpp"

using manip_task_execution::classifyStackTarget;
using manip_task_execution::decidePlacement;
using manip_task_execution::decideSlotAction;
using manip_task_execution::effectiveSeatTolerance;
using manip_task_execution::expectedPileTopZ;
using manip_task_execution::findStackMemory;
using manip_task_execution::isSeatedInSlot;
using manip_task_execution::nearestQuarterTurn;
using manip_task_execution::PileFix;
using manip_task_execution::pileFixFor;
using manip_task_execution::pileRegraspFloorZ;
using manip_task_execution::PlacementDecision;
using manip_task_execution::PlacementLimits;
using manip_task_execution::PlacementObservation;
using manip_task_execution::PlanarOffset;
using manip_task_execution::SeatState;
using manip_task_execution::SlotAction;
using manip_task_execution::SlotLimits;
using manip_task_execution::SlotObservation;
using manip_task_execution::slotRegraspTcpZ;
using manip_task_execution::slotRimCubeTagZ;
using manip_task_execution::slotSeatState;
using manip_task_execution::spiralOffsets;
using manip_task_execution::stackLevelFromMemory;
using manip_task_execution::StackMemoryEntry;
using manip_task_execution::StackTargetKind;
using manip_task_execution::tagIdFromFrame;
using manip_task_execution::tiltDropZCompensation;

namespace
{

constexpr double kTol = 1e-9;
constexpr double kDeg = M_PI / 180.0;
const std::vector<std::int64_t> kSlotIds{101, 102, 103, 104, 105, 106, 107};
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
  EXPECT_FALSE(tagIdFromFrame("tag_amt1_slot_3").has_value());
}

TEST(PlacePrecision, ClassifyStackTarget)
{
  for (int id = 101; id <= 107; ++id) {
    EXPECT_EQ(
      classifyStackTarget("tag_" + std::to_string(id), kSlotIds, kRefIds),
      StackTargetKind::kSlot) << id;
  }
  EXPECT_EQ(classifyStackTarget("tag_200", kSlotIds, kRefIds), StackTargetKind::kReference);
  EXPECT_EQ(classifyStackTarget("tag_4", kSlotIds, kRefIds), StackTargetKind::kCube);
  EXPECT_EQ(classifyStackTarget("tag_100", kSlotIds, kRefIds), StackTargetKind::kCube);
  EXPECT_EQ(classifyStackTarget("tag_108", kSlotIds, kRefIds), StackTargetKind::kCube);
  // Frame que nao casa 'tag_<n>' = comportamento atual (cubo).
  EXPECT_EQ(classifyStackTarget("tag_amt1_slot_2", kSlotIds, kRefIds), StackTargetKind::kCube);
  EXPECT_EQ(classifyStackTarget("ct_vermelho", kSlotIds, kRefIds), StackTargetKind::kCube);
  EXPECT_EQ(classifyStackTarget("", kSlotIds, kRefIds), StackTargetKind::kCube);
  // Listas vazias: tudo e' cubo.
  EXPECT_EQ(classifyStackTarget("tag_101", {}, {}), StackTargetKind::kCube);
  EXPECT_EQ(classifyStackTarget("tag_200", {}, {}), StackTargetKind::kCube);
  // Um id nas duas listas: slot vence (mais restritivo).
  EXPECT_EQ(classifyStackTarget("tag_200", {200}, {200}), StackTargetKind::kSlot);
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

// ---- espiral -------------------------------------------------------------

TEST(PlacePrecision, SpiralOffsetsOrderedByRadiusNoRepeat)
{
  const double s = 0.003;
  const auto sp = spiralOffsets(s, 0.009);
  ASSERT_GE(sp.size(), 6u);
  // Comeca em (0,0) e segue +X, +Y, -X, -Y, depois a diagonal (+s,+s).
  EXPECT_NEAR(sp[0].dx, 0.0, kTol); EXPECT_NEAR(sp[0].dy, 0.0, kTol);
  EXPECT_NEAR(sp[1].dx, s, kTol);   EXPECT_NEAR(sp[1].dy, 0.0, kTol);
  EXPECT_NEAR(sp[2].dx, 0.0, kTol); EXPECT_NEAR(sp[2].dy, s, kTol);
  EXPECT_NEAR(sp[3].dx, -s, kTol);  EXPECT_NEAR(sp[3].dy, 0.0, kTol);
  EXPECT_NEAR(sp[4].dx, 0.0, kTol); EXPECT_NEAR(sp[4].dy, -s, kTol);
  EXPECT_NEAR(sp[5].dx, s, kTol);   EXPECT_NEAR(sp[5].dy, s, kTol);
  // Raio nao decrescente e nunca acima do maximo.
  std::set<std::pair<long, long>> seen;
  double last_r = -1.0;
  for (const auto & p : sp) {
    EXPECT_GE(p.radius(), last_r - kTol);
    EXPECT_LE(p.radius(), 0.009 + 1e-9);
    last_r = p.radius();
    const auto key = std::make_pair(std::lround(p.dx / s), std::lround(p.dy / s));
    EXPECT_TRUE(seen.insert(key).second) << "repetido " << p.dx << " " << p.dy;
  }
  // Grade 7x7 cortada pelo circulo de 9 mm: 1 + 4 + 4 + 4 + 8 + 4 + 4 = 29.
  EXPECT_EQ(sp.size(), 29u);
}

TEST(PlacePrecision, SpiralOffsetsDegenerate)
{
  EXPECT_EQ(spiralOffsets(0.0, 0.01).size(), 1u);
  EXPECT_EQ(spiralOffsets(0.003, 0.0).size(), 1u);
  EXPECT_EQ(spiralOffsets(-1.0, 0.01).size(), 1u);
  // Raio menor que o passo: so a origem.
  EXPECT_EQ(spiralOffsets(0.005, 0.004).size(), 1u);
  // Raio = passo: origem + 4 vizinhos.
  EXPECT_EQ(spiralOffsets(0.005, 0.005).size(), 5u);
}

// ---- assentamento no slot ------------------------------------------------

TEST(PlacePrecision, SeatedInSlotWithTolerance)
{
  const double z_slot = 0.010;   // fundo da cavidade
  const double h = 0.042;
  const double d = 0.005;        // cavidade rasa da mesa de precisao (3-6 mm)
  const double tol = 0.0025;     // = cavidade/2
  const double seated = z_slot + h;  // 0.052
  // Tolerancia pedida maior que cavidade/2 e' cortada: assentado + 4 mm ja e' borda.
  EXPECT_NEAR(effectiveSeatTolerance(0.008, d), 0.0025, kTol);
  EXPECT_NEAR(effectiveSeatTolerance(0.001, d), 0.001, kTol);
  EXPECT_NEAR(effectiveSeatTolerance(-1.0, d), 0.0, kTol);
  EXPECT_FALSE(isSeatedInSlot(seated + 0.004, z_slot, h, d, 0.008));
  EXPECT_TRUE(isSeatedInSlot(seated + 0.002, z_slot, h, d, 0.008));
  EXPECT_TRUE(isSeatedInSlot(seated, z_slot, h, d, tol));
  EXPECT_TRUE(isSeatedInSlot(seated + tol - 1e-6, z_slot, h, d, tol));
  EXPECT_TRUE(isSeatedInSlot(seated + tol, z_slot, h, d, tol));
  EXPECT_FALSE(isSeatedInSlot(seated + tol + 1e-6, z_slot, h, d, tol));
  // Leitura um pouco abaixo do esperado continua 'encaixado'.
  EXPECT_TRUE(isSeatedInSlot(seated - tol, z_slot, h, d, tol));
  EXPECT_TRUE(isSeatedInSlot(seated - d, z_slot, h, d, tol));
  // Abaixo do fundo da cavidade + cubo: impossivel -> nao encaixado.
  EXPECT_FALSE(isSeatedInSlot(seated - d - 1e-6, z_slot, h, d, tol));
  // Apoiado por cima: ~cavidade mais alto.
  EXPECT_FALSE(isSeatedInSlot(seated + d, z_slot, h, d, tol));
  EXPECT_EQ(slotSeatState(seated + d, z_slot, h, d, tol), SeatState::kUnseated);
  // Metade da cavidade e exatamente a tolerancia (limite inclusivo = assentado);
  // um fio acima ja e borda.
  EXPECT_EQ(slotSeatState(seated + d / 2.0, z_slot, h, d, tol), SeatState::kSeated);
  EXPECT_EQ(slotSeatState(seated + d / 2.0 + 1e-6, z_slot, h, d, tol), SeatState::kUnseated);
  EXPECT_EQ(slotSeatState(seated, z_slot, h, d, tol), SeatState::kSeated);
  EXPECT_EQ(slotSeatState(seated - 0.06, z_slot, h, d, tol), SeatState::kInconsistent);
  // Tolerancia negativa vale 0.
  EXPECT_TRUE(isSeatedInSlot(seated, z_slot, h, d, -1.0));
  EXPECT_FALSE(isSeatedInSlot(seated + 1e-6, z_slot, h, d, -1.0));
}

TEST(PlacePrecision, SlotRimAndRegraspFloor)
{
  const double z_slot = 0.010;
  const double h = 0.042;
  const double d = 0.005;
  // Borda = fundo + cavidade + cubo: onde a tag de um cubo apoiado fica.
  EXPECT_NEAR(slotRimCubeTagZ(z_slot, h, d), 0.057, kTol);
  // Cavidade negativa vale 0.
  EXPECT_NEAR(slotRimCubeTagZ(z_slot, h, -1.0), 0.052, kTol);
  // Re-pega: o z visto, mas nunca abaixo da borda (dedos no tampo).
  const double rim = slotRimCubeTagZ(z_slot, h, d);
  EXPECT_NEAR(slotRegraspTcpZ(rim + 0.003, rim), rim + 0.003, kTol);
  EXPECT_NEAR(slotRegraspTcpZ(rim - 0.006, rim), rim, kTol);
  EXPECT_NEAR(slotRegraspTcpZ(rim, rim), rim, kTol);
}

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

// ---- decisao do slot -----------------------------------------------------

namespace
{

SlotLimits slotLimits(int attempts_done, int observations_done, double cavity = 0.005)
{
  SlotLimits lim;
  lim.z_slot_tag = 0.010;
  lim.cube_height_m = 0.042;
  lim.cavity_depth_m = cavity;
  lim.seat_tol_m = 0.008;   // cortada para cavidade/2 (2,5 mm com 5 mm)
  lim.attempts_done = attempts_done;
  lim.max_attempts = 6;
  lim.observations_done = observations_done;
  lim.max_observations = 2;
  return lim;
}

SlotObservation seenAt(double z, double xy = 0.0)
{
  SlotObservation o;
  o.seen = true;
  o.z_cube_tag = z;
  o.err_xy_m = xy;
  return o;
}

}  // namespace

TEST(PlacePrecision, DecideSlotActionSeatedIgnoresXy)
{
  const double seated = 0.052;
  // Assentado: aceita com qualquer xy (duas leituras de poses diferentes).
  EXPECT_EQ(decideSlotAction(seenAt(seated, 0.0), slotLimits(1, 1)), SlotAction::kAccept);
  EXPECT_EQ(decideSlotAction(seenAt(seated, 0.015), slotLimits(1, 1)), SlotAction::kAccept);
  EXPECT_EQ(decideSlotAction(seenAt(seated + 0.002, 0.03), slotLimits(1, 1)), SlotAction::kAccept);
  EXPECT_EQ(decideSlotAction(seenAt(seated - 0.004, 0.0), slotLimits(1, 1)), SlotAction::kAccept);
  // Mesmo sem orcamento nenhum: assentado e' assentado.
  EXPECT_EQ(decideSlotAction(seenAt(seated, 0.02), slotLimits(6, 2)), SlotAction::kAccept);
}

TEST(PlacePrecision, DecideSlotActionRimRegraspOnlyWithBudget)
{
  const double rim = slotRimCubeTagZ(0.010, 0.042, 0.005);  // 0.057
  // Borda na 1a leitura: confirma com a 2a antes de re-pegar (cavidade rasa).
  EXPECT_EQ(decideSlotAction(seenAt(rim), slotLimits(1, 1)), SlotAction::kReobserve);
  EXPECT_EQ(decideSlotAction(seenAt(rim), slotLimits(1, 2)), SlotAction::kRegrasp);
  EXPECT_EQ(decideSlotAction(seenAt(rim + 0.005), slotLimits(1, 2)), SlotAction::kRegrasp);
  // Leitura ate' tol (2,5 mm) abaixo da borda ainda conta como borda.
  EXPECT_EQ(decideSlotAction(seenAt(rim - 0.002), slotLimits(1, 2)), SlotAction::kRegrasp);
  // Ultima soltura: nao re-pega, deixa.
  EXPECT_EQ(decideSlotAction(seenAt(rim), slotLimits(6, 2)), SlotAction::kLeave);
  EXPECT_EQ(decideSlotAction(seenAt(rim), slotLimits(5, 2)), SlotAction::kRegrasp);
}

TEST(PlacePrecision, DecideSlotActionIntermediateNeverRegrasps)
{
  // Cavidade funda (41 mm) tem faixa intermediaria entre assentado + tol e
  // borda - tol (com 5 mm as duas faixas se encostam e nao ha meio-termo).
  const double seated = 0.052;
  const double rim = 0.093;
  const double deep = 0.041;
  for (const double z : {seated + 0.009, seated + 0.020, rim - 0.009, (seated + rim) / 2.0}) {
    EXPECT_EQ(decideSlotAction(seenAt(z), slotLimits(1, 1, deep)), SlotAction::kReobserve) << z;
    EXPECT_EQ(decideSlotAction(seenAt(z), slotLimits(1, 2, deep)), SlotAction::kLeave) << z;
    // Com orcamento de solturas sobrando continua sem re-pegar.
    EXPECT_NE(decideSlotAction(seenAt(z), slotLimits(1, 1, deep)), SlotAction::kRegrasp) << z;
    EXPECT_NE(decideSlotAction(seenAt(z), slotLimits(1, 2, deep)), SlotAction::kRegrasp) << z;
  }
  // Inconsistente (abaixo do fundo + cubo): idem.
  EXPECT_EQ(decideSlotAction(seenAt(seated - 0.05), slotLimits(1, 1)), SlotAction::kReobserve);
  EXPECT_EQ(decideSlotAction(seenAt(seated - 0.05), slotLimits(1, 2)), SlotAction::kLeave);
}

TEST(PlacePrecision, DecideSlotActionUnseenNeverAcceptsNorRegrasps)
{
  SlotObservation unseen;
  unseen.seen = false;
  EXPECT_EQ(decideSlotAction(unseen, slotLimits(1, 1)), SlotAction::kReobserve);
  EXPECT_EQ(decideSlotAction(unseen, slotLimits(1, 2)), SlotAction::kLeave);
  EXPECT_EQ(decideSlotAction(unseen, slotLimits(6, 2)), SlotAction::kLeave);
  // Nunca kAccept / kRegrasp sem ver, com qualquer orcamento.
  for (int a = 1; a <= 6; ++a) {
    for (int o = 1; o <= 2; ++o) {
      const SlotAction act = decideSlotAction(unseen, slotLimits(a, o));
      EXPECT_NE(act, SlotAction::kAccept);
      EXPECT_NE(act, SlotAction::kRegrasp);
    }
  }
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

// ---- decisao de verificacao (generica; a pilha usa, o slot nao) ---------

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
