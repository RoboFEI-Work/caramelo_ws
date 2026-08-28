// Busca de deslocamento LATERAL da base verificada pela IK propria
// (fila de alcance, 2026-08-28).
//
// Quando um alvo (tag ou ponto de soltura) e' VISTO mas nao tem solucao de
// IK da pose de estacionamento, o executor pode deslocar a base de lado.
// Esta funcao responde "quantos cm e para que lado" testando a mesma IK e a
// mesma escada de inclinacao que o pick/place vao usar depois do ajuste.
//
// Frames: alvos em manip_base_link (+X = direita do robo, +Y = frente).
// Base anda `dy` para a ESQUERDA (+y de base_footprint) => o x do alvo fixo
// vira x + dy. Logo a direcao que reduz |x| e' sign(dy) = -sign(x).
#pragma once

#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "manip_task_execution/custom_ik.hpp"

namespace manip_task_execution
{

struct ReachShiftOptions
{
  /// |dy| candidatos, crescentes, frame base_footprint (m). O piso do ESC
  /// nao executa passos < ~0,10 m; o curso maximo e' decisao do operador.
  std::vector<double> candidates_m{0.10, 0.15, 0.20, 0.25};
  /// Escada de inclinacao da ferramenta (rad a partir da vertical), a mesma
  /// que o pick/place usam (0, 15, 30 graus).
  std::vector<double> tilts_rad{0.0, 15.0 * M_PI / 180.0, 30.0 * M_PI / 180.0};
  double q5_fixed{0.0};
  /// Place: tambem exige IK em alvo + (0,0,lift_m) (pose de elevacao).
  double lift_m{0.0};
  /// A base para um pouco antes do pedido: testa o alvo com esse desconto.
  double undershoot_margin_m{0.02};
  /// false = rejeita candidatos que aumentariam |x| (afastam do alvo).
  bool allow_overshoot{false};
  ArmModel model{};
  IkOptions ik{};
};

struct ReachShiftResult
{
  bool reachable_now{false};        ///< IK resolve sem deslocar (sanidade)
  double shift_m{0.0};              ///< dy em base_footprint, + = ESQUERDA; 0 = nada resolve
  double tilt_rad{0.0};             ///< inclinacao que resolveu no alvo deslocado
  Eigen::Vector3d shifted_target{0.0, 0.0, 0.0};  ///< alvo (manip) apos o deslocamento
  std::array<double, 5> q{};        ///< solucao no alvo deslocado (diagnostico)
};

/// x (manip_base_link) de um alvo fixo no mundo depois de a base andar `dy`
/// para a esquerda.
inline double manipXAfterBaseShift(double x_manip, double dy) { return x_manip + dy; }

/// Alvos alternativos (ex.: candidatos de soltura no container, em ordem de
/// preferencia): a busca aceita um deslocamento se QUALQUER alvo resolver.
/// O 1o alvo decide a direcao.
ReachShiftResult findBaseShiftForReach(
  const std::vector<Eigen::Vector3d> & targets_manip, const ReachShiftOptions & opts);

inline ReachShiftResult findBaseShiftForReach(
  const Eigen::Vector3d & target_manip, const ReachShiftOptions & opts)
{
  return findBaseShiftForReach(std::vector<Eigen::Vector3d>{target_manip}, opts);
}

}  // namespace manip_task_execution
