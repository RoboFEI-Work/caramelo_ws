// gtest do gate de pose e do detector de parada do GoToWS (2026-10-26) —
// logica pura, sem ROS no ar. Os valores dos casos vem dos logs reais de
// ~/.ros/log das runs de 22 e 23/10 (docs/investigacao_incidente_2026-10.md).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "manip_bt/pose_gate.hpp"

namespace
{

using manip_bt::AlignOutcome;
using manip_bt::PoseGateLimits;
using manip_bt::StopDetector;
using manip_bt::YawSource;

AlignOutcome outcome(
  double x, double y, double yaw,
  const std::string & message = "alinhado (tentativa 1/1)")
{
  AlignOutcome r;
  r.success = true;
  r.message = message;
  r.x_error = x;
  r.y_error = y;
  r.yaw_error = yaw;
  return r;
}

}  // namespace

// ---------------------------------------------------------- classifyAlignMessage

TEST(AlignNote, MensagemNormalEhLidar)
{
  const auto note = manip_bt::classifyAlignMessage("alinhado (tentativa 1/1)");
  EXPECT_FALSE(note.yaw_out_of_tolerance);
  EXPECT_FALSE(note.yaw_unmeasured);
  EXPECT_EQ(note.yaw_source, YawSource::kLidar);
}

TEST(AlignNote, YawForaDaToleranciaVemDoLidar)
{
  // WS6, 23/10 19:08: 'yaw_fora_da_tolerancia (-12.7 graus); alinhado (tentativa 1/1)'
  const auto note = manip_bt::classifyAlignMessage(
    "yaw_fora_da_tolerancia (-12.7 graus); alinhado (tentativa 1/1)");
  EXPECT_TRUE(note.yaw_out_of_tolerance);
  EXPECT_FALSE(note.yaw_unmeasured);
  EXPECT_EQ(note.yaw_source, YawSource::kLidar);
}

TEST(AlignNote, YawNaoMedidoVemDoAmcl)
{
  // WS3, 23/10 18:51 e PP1, 22/10 15:42.
  const auto note = manip_bt::classifyAlignMessage("yaw_nao_medido; alinhado (tentativa 1/1)");
  EXPECT_FALSE(note.yaw_out_of_tolerance);
  EXPECT_TRUE(note.yaw_unmeasured);
  EXPECT_EQ(note.yaw_source, YawSource::kAmcl);
}

TEST(AlignNote, PrefixoSoNoInicio)
{
  // A nota so' vale como prefixo (contrato da action); no meio nao conta.
  const auto note = manip_bt::classifyAlignMessage("alinhado; yaw_nao_medido");
  EXPECT_FALSE(note.yaw_unmeasured);
  EXPECT_EQ(note.yaw_source, YawSource::kLidar);
}

// ------------------------------------------------------------- evaluatePoseGate

TEST(PoseGate, DockingsBonsDe23OutPassam)
{
  const PoseGateLimits lim;
  // WS5 18:26, WS1 18:29, WS4 18:48, WS6 18:49 (+8.7 graus, aceito com aviso)
  EXPECT_TRUE(manip_bt::evaluatePoseGate(outcome(-0.004, -0.000, -0.016), lim).ok);
  EXPECT_TRUE(manip_bt::evaluatePoseGate(outcome(0.000, 0.035, 0.077), lim).ok);
  EXPECT_TRUE(manip_bt::evaluatePoseGate(outcome(0.005, 0.052, 0.004), lim).ok);
  EXPECT_TRUE(manip_bt::evaluatePoseGate(
      outcome(-0.008, -0.034, 0.152,
      "yaw_fora_da_tolerancia (+8.7 graus); alinhado (tentativa 1/1)"), lim).ok);
}

TEST(PoseGate, Ws3De23OutReprovaPorYeYaw)
{
  // [WS3] alinhado (erro x=-0.006 y=0.304 yaw=0.319) — yaw_nao_medido
  const auto v = manip_bt::evaluatePoseGate(
    outcome(-0.006, 0.304, 0.319, "yaw_nao_medido; alinhado (tentativa 1/1)"), PoseGateLimits{});
  EXPECT_FALSE(v.ok);
  EXPECT_NE(v.reason.find("y +0.304 m"), std::string::npos) << v.reason;
  EXPECT_NE(v.reason.find("yaw +0.319 rad"), std::string::npos) << v.reason;
  EXPECT_EQ(v.reason.find("x "), std::string::npos) << v.reason;
}

TEST(PoseGate, Pp1De22OutReprovaPorYaw)
{
  // [PP1] alinhado (erro x=-0.011 y=0.084 yaw=0.271) — yaw_nao_medido (AMCL +15.5)
  const auto v = manip_bt::evaluatePoseGate(
    outcome(-0.011, 0.084, 0.271, "yaw_nao_medido; alinhado (tentativa 1/1)"), PoseGateLimits{});
  EXPECT_FALSE(v.ok);
  EXPECT_NE(v.reason.find("yaw"), std::string::npos);
  EXPECT_EQ(v.reason.find("y "), std::string::npos) << v.reason;
}

TEST(PoseGate, Ws6De23OutMenos12GrausReprova)
{
  // [WS6] alinhado (erro x=-0.002 y=0.028 yaw=-0.222) — yaw_fora_da_tolerancia (-12.7)
  const auto v = manip_bt::evaluatePoseGate(
    outcome(-0.002, 0.028, -0.222,
    "yaw_fora_da_tolerancia (-12.7 graus); alinhado (tentativa 1/1)"), PoseGateLimits{});
  EXPECT_FALSE(v.ok);
  EXPECT_NE(v.reason.find("yaw -0.222 rad (-12.7 graus)"), std::string::npos) << v.reason;
}

TEST(PoseGate, LimiteEhInclusivo)
{
  const PoseGateLimits lim;
  EXPECT_TRUE(manip_bt::evaluatePoseGate(outcome(0.0, 0.20, 0.20), lim).ok);
  EXPECT_TRUE(manip_bt::evaluatePoseGate(outcome(0.0, -0.20, -0.20), lim).ok);
  EXPECT_FALSE(manip_bt::evaluatePoseGate(outcome(0.0, 0.201, 0.0), lim).ok);
  EXPECT_FALSE(manip_bt::evaluatePoseGate(outcome(0.0, 0.0, -0.201), lim).ok);
  EXPECT_FALSE(manip_bt::evaluatePoseGate(outcome(0.101, 0.0, 0.0), lim).ok);
}

TEST(PoseGate, SemSucessoOuNaoFinitoReprova)
{
  const PoseGateLimits lim;
  auto r = outcome(0.0, 0.0, 0.0);
  r.success = false;
  EXPECT_FALSE(manip_bt::evaluatePoseGate(r, lim).ok);
  EXPECT_FALSE(manip_bt::evaluatePoseGate(
      outcome(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0), lim).ok);
  EXPECT_FALSE(manip_bt::evaluatePoseGate(
      outcome(0.0, 0.0, std::numeric_limits<double>::infinity()), lim).ok);
}

TEST(PoseGate, LimitesConfiguraveis)
{
  PoseGateLimits lim;
  lim.max_abs_y = 0.35;
  lim.max_abs_yaw = 0.35;
  EXPECT_TRUE(manip_bt::evaluatePoseGate(outcome(-0.006, 0.304, 0.319), lim).ok);
}

// ---------------------------------------------------------------- StopDetector

TEST(StopDetector, SemAmostraEhSemOdometria)
{
  StopDetector det;
  det.reset(10.0);
  EXPECT_EQ(det.evaluate(10.0), StopDetector::State::kNoOdom);
  EXPECT_EQ(det.evaluate(13.0), StopDetector::State::kNoOdom);
}

TEST(StopDetector, AmostraVelhaEhSemOdometria)
{
  StopDetector det;
  det.reset(10.0);
  det.feed(10.1, 0.0, 0.0, 0.0);
  EXPECT_EQ(det.evaluate(10.2), StopDetector::State::kMoving);   // janela ainda nao fechou
  EXPECT_EQ(det.evaluate(10.8), StopDetector::State::kNoOdom);   // 0.7 s sem amostra > 0.5
}

TEST(StopDetector, JanelaPrecisaSerCobertaPorAmostras)
{
  StopDetector det;
  det.reset(10.0);
  for (int i = 0; i <= 40; ++i) {        // 50 Hz, 0.8 s parado
    det.feed(10.0 + 0.02 * i, 0.0, 0.0, 0.0);
    const auto st = det.evaluate(10.0 + 0.02 * i);
    if (0.02 * i < 0.5) {
      EXPECT_EQ(st, StopDetector::State::kMoving) << "i=" << i;
    } else {
      EXPECT_EQ(st, StopDetector::State::kStopped) << "i=" << i;
    }
  }
  EXPECT_NEAR(det.quietFor(), 0.8, 1e-9);
}

TEST(StopDetector, MovimentoReiniciaAJanela)
{
  StopDetector det;
  det.reset(0.0);
  for (int i = 0; i < 30; ++i) {
    det.feed(0.02 * i, 0.0, 0.0, 0.0);
  }
  EXPECT_EQ(det.evaluate(0.58), StopDetector::State::kStopped);
  // Coast do ESC: 0.12 m/s lateral por 0.2 s
  for (int i = 30; i < 40; ++i) {
    det.feed(0.02 * i, 0.0, 0.12, 0.0);
    EXPECT_EQ(det.evaluate(0.02 * i), StopDetector::State::kMoving);
  }
  for (int i = 40; i < 64; ++i) {
    det.feed(0.02 * i, 0.0, 0.0, 0.0);
    EXPECT_EQ(det.evaluate(0.02 * i), StopDetector::State::kMoving) << "i=" << i;
  }
  det.feed(0.02 * 65, 0.0, 0.0, 0.0);  // 0.80 -> 1.30: 0.5 s parado
  EXPECT_EQ(det.evaluate(1.30), StopDetector::State::kStopped);
}

TEST(StopDetector, RuidoAbaixoDoLimiteContaComoParado)
{
  StopDetector det;
  det.reset(0.0);
  for (int i = 0; i <= 30; ++i) {
    det.feed(0.02 * i, 0.012, -0.010, 0.04);   // |v| 0.0156 < 0.02, |w| 0.04 < 0.05
  }
  EXPECT_EQ(det.evaluate(0.60), StopDetector::State::kStopped);
}

TEST(StopDetector, GiroResidualNoPisoDoEscEhMovimento)
{
  StopDetector det;
  det.reset(0.0);
  for (int i = 0; i <= 30; ++i) {
    det.feed(0.02 * i, 0.0, 0.0, 0.315);       // piso angular do ESC
  }
  EXPECT_EQ(det.evaluate(0.60), StopDetector::State::kMoving);
  EXPECT_NEAR(det.quietFor(), 0.0, 1e-12);
}

TEST(StopDetector, AmostrasAntesDoResetNaoContam)
{
  StopDetector det;
  det.reset(0.0);
  for (int i = 0; i <= 50; ++i) {
    det.feed(0.02 * i, 0.0, 0.0, 0.0);         // 1 s parado
  }
  EXPECT_EQ(det.evaluate(1.0), StopDetector::State::kStopped);
  det.reset(1.0);                              // novo align terminou agora
  det.feed(1.02, 0.0, 0.0, 0.0);
  EXPECT_EQ(det.evaluate(1.02), StopDetector::State::kMoving);
  for (int i = 2; i <= 26; ++i) {
    det.feed(1.0 + 0.02 * i, 0.0, 0.0, 0.0);
  }
  EXPECT_EQ(det.evaluate(1.52), StopDetector::State::kStopped);
}

TEST(StopDetector, LimitesParametrizaveis)
{
  StopDetector::Limits lim;
  lim.max_lin = 0.05;
  lim.max_ang = 0.10;
  lim.window_s = 1.0;
  lim.max_age_s = 0.2;
  StopDetector det(lim);
  det.reset(0.0);
  for (int i = 0; i <= 60; ++i) {
    det.feed(0.02 * i, 0.04, 0.0, 0.08);
  }
  EXPECT_EQ(det.evaluate(1.20), StopDetector::State::kStopped);
  EXPECT_EQ(det.evaluate(1.45), StopDetector::State::kNoOdom);  // 0.25 s > 0.2
}

TEST(StopDetector, NaoFinitoEhMovimento)
{
  StopDetector det;
  det.reset(0.0);
  det.feed(0.02, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  for (int i = 2; i <= 30; ++i) {
    det.feed(0.02 * i, 0.0, 0.0, 0.0);
  }
  // 0.04 -> 0.60 = 0.56 s parado depois do NaN
  EXPECT_EQ(det.evaluate(0.60), StopDetector::State::kStopped);
  det.feed(0.62, 0.0, 0.0, std::numeric_limits<double>::infinity());
  EXPECT_EQ(det.evaluate(0.62), StopDetector::State::kMoving);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
