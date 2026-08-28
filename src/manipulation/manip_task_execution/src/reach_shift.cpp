// Busca de deslocamento lateral da base verificada pela IK propria
// (fila de alcance, 2026-08-28). Contrato em reach_shift.hpp.
//
// Notas de design (2026-08-28):
// - A busca usa EXATAMENTE a mesma sobrecarga de solveIk (inclinacao
//   continua) e a mesma escada de tilts que o pick/place executam depois do
//   ajuste; o que resolve aqui resolve la (mesmo modelo, mesmas opcoes).
// - O 1o alvo decide a direcao: o sinal que reduz |x| dele. Um deslocamento
//   que afasta do 1o alvo (overshoot) e' recusado por padrao — a base nao
//   deve passar para o outro lado do alvo so porque um candidato mais longe
//   aconteceu de resolver.
// - A base para um pouco ANTES do pedido (piso do ESC, coast): o alvo e'
//   testado com o desconto `undershoot_margin_m`, mas o valor DEVOLVIDO e' o
//   candidato inteiro (o que se pede ao nudge_base).
// - Custo: cada solveIk custa ~0,1 s em -O2; pior caso = candidatos x tilts x
//   alvos x (1 + lift) solves. Com 4 candidatos, 3 tilts, 1 alvo e lift
//   sao ate 24 solves (~2-3 s), aceitavel para uma decisao por estacao.
#include "manip_task_execution/reach_shift.hpp"

#include <algorithm>

namespace manip_task_execution
{

namespace
{

/// |x| abaixo disso o alvo ja esta "na frente" do braco: de lado nao ajuda.
constexpr double kMinLateralOffsetM = 0.03;

/// Testa a escada completa de inclinacoes num alvo (e no alvo elevado quando
/// ha lift). Devolve a 1a inclinacao que resolve, na ordem de `tilts_rad`.
bool solvesAnyTilt(
  const Eigen::Vector3d & target, const ReachShiftOptions & opts,
  double & tilt_out, std::array<double, 5> & q_out)
{
  for (const double tilt : opts.tilts_rad) {
    std::array<double, 5> q{};
    if (!solveIk(target, tilt, opts.q5_fixed, q, opts.model, opts.ik)) {
      continue;
    }
    if (opts.lift_m != 0.0) {
      std::array<double, 5> q_lift{};
      const Eigen::Vector3d lifted = target + Eigen::Vector3d(0.0, 0.0, opts.lift_m);
      if (!solveIk(lifted, tilt, opts.q5_fixed, q_lift, opts.model, opts.ik)) {
        continue;
      }
    }
    tilt_out = tilt;
    q_out = q;
    return true;
  }
  return false;
}

}  // namespace

ReachShiftResult findBaseShiftForReach(
  const std::vector<Eigen::Vector3d> & targets_manip, const ReachShiftOptions & opts)
{
  ReachShiftResult result;
  if (targets_manip.empty()) {
    return result;
  }

  // Sanidade: algum alvo ja resolve sem mover a base? Entao shift 0 e a
  // solucao do proprio alvo vai como diagnostico.
  for (const auto & target : targets_manip) {
    double tilt = 0.0;
    std::array<double, 5> q{};
    if (solvesAnyTilt(target, opts, tilt, q)) {
      result.reachable_now = true;
      result.shift_m = 0.0;
      result.tilt_rad = tilt;
      result.shifted_target = target;
      result.q = q;
      return result;
    }
  }

  const double x0 = targets_manip.front().x();
  if (std::abs(x0) < kMinLateralOffsetM) {
    // Alvo alinhado com o braco: se nao alcanca, e' distancia/altura, nao lado.
    return result;
  }
  const double sign = x0 > 0.0 ? -1.0 : 1.0;

  // Candidatos em ordem crescente (o menor passo que resolve vence), sem
  // confiar que o chamador ja ordenou; passos nao positivos sao ignorados.
  std::vector<double> candidates = opts.candidates_m;
  std::sort(candidates.begin(), candidates.end());

  for (const double d : candidates) {
    if (d <= 0.0) {
      continue;
    }
    const double dy = sign * d;
    const double dy_eff = dy - sign * opts.undershoot_margin_m;
    if (!opts.allow_overshoot && std::abs(x0 + dy_eff) > std::abs(x0)) {
      // Passaria para o outro lado do 1o alvo mais do que ele esta hoje:
      // candidatos maiores so pioram, para aqui.
      break;
    }
    for (const auto & target : targets_manip) {
      const Eigen::Vector3d shifted(
        manipXAfterBaseShift(target.x(), dy_eff), target.y(), target.z());
      double tilt = 0.0;
      std::array<double, 5> q{};
      if (solvesAnyTilt(shifted, opts, tilt, q)) {
        result.shift_m = dy;
        result.tilt_rad = tilt;
        result.shifted_target = shifted;
        result.q = q;
        return result;
      }
    }
  }

  // Nada resolve de lado dentro do curso permitido.
  return result;
}

}  // namespace manip_task_execution
