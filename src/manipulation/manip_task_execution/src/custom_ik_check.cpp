// Conferencia do porte C++ da IK contra o solver Python do operador.
//
// Uso:
//   custom_ik_check                       -> varre uma grade de alvos e imprime
//                                            uma linha CSV por alvo resolvido
//   custom_ik_check <x> <y> <z> [modo] [free]
//                                         -> resolve um alvo unico
//                                            (modo: forward | shallow | middle | down
//                                             | tilt<graus>, ex. tilt20 = por cima
//                                             inclinado 20 graus a partir da vertical;
//                                             "free" = orientacao LIVRE: so a posicao
//                                             e exigida, o modo vira preferencia —
//                                             e' o que a shelf usa com shelf_j4_free)
//   custom_ik_check shift <x> <y> <z> [lift_m]
//                                         -> fila de alcance (2026-08-28): imprime
//                                            uma linha por (candidato de deslocamento
//                                            lateral, tilt) com ok/SEM_SOLUCAO e no fim
//                                            o deslocamento que findBaseShiftForReach
//                                            escolheria (mesma escada 0/15/30 do
//                                            pick/place; lift = tambem exige IK em
//                                            z+lift, como o place em pilha/container)
//
// O formato CSV e o mesmo consumido pelo script de paridade:
//   x,y,z,modo,q1,q2,q3,q4,q5,erro_pos_m,erro_dir_graus
//
// Compare com:
//   python3 ik_solver_foto.py <x> <y> <z> --tool-direction <modo>
// Diferenca acima de 1e-4 rad em qualquer junta = erro de transcricao.
//
// ATENCAO: as coordenadas aqui sao no frame da BASE DO BRACO
// (manip_base_link), igual ao script Python — nao em base_footprint.

#include "manip_task_execution/custom_ik.hpp"
#include "manip_task_execution/reach_shift.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using manip_task_execution::ArmModel;
using manip_task_execution::ToolDirection;

namespace
{

/// Modo de direcao: nome fixo (forward/shallow/middle/down) ou "tilt<graus>".
struct Mode
{
  std::string name;
  double tilt_from_vertical;
};

Mode parseMode(const std::string & name)
{
  if (name == "down") {
    return {name, manip_task_execution::tiltFromVertical(ToolDirection::kDown)};
  }
  if (name == "shallow") {
    return {name, manip_task_execution::tiltFromVertical(ToolDirection::kShallow)};
  }
  if (name == "middle") {
    return {name, manip_task_execution::tiltFromVertical(ToolDirection::kMiddle)};
  }
  if (name.rfind("tilt", 0) == 0 && name.size() > 4) {
    return {name, std::atof(name.c_str() + 4) * M_PI / 180.0};
  }
  return {"forward", manip_task_execution::tiltFromVertical(ToolDirection::kForward)};
}

Mode modeFor(ToolDirection d)
{
  switch (d) {
    case ToolDirection::kDown: return parseMode("down");
    case ToolDirection::kMiddle: return parseMode("middle");
    case ToolDirection::kShallow: return parseMode("shallow");
    default: return parseMode("forward");
  }
}

/// Resolve e imprime uma linha CSV. Devolve false se nao houve solucao.
bool reportOne(
  double x, double y, double z, const Mode & mode, double q5, bool free_orientation = false)
{
  const Eigen::Vector3d target(x, y, z);
  std::array<double, 5> q{};

  manip_task_execution::IkOptions options;
  ArmModel model;
  if (free_orientation) {
    // Mesmos limites que a shelf usa no fallback "J4 livre" (mtc_pick_action_node).
    options.orientation_weight = 0.0;
    options.max_orientation_error = 30.0 * M_PI / 180.0;
    model.j2_max = 1.2;
  }
  const std::string mode_name = free_orientation ? mode.name + "+free" : mode.name;
  if (!manip_task_execution::solveIk(
      target, mode.tilt_from_vertical, q5, q, model, options))
  {
    std::cout << std::fixed << std::setprecision(4)
              << x << "," << y << "," << z << "," << mode_name
              << ",SEM_SOLUCAO\n";
    return false;
  }

  Eigen::Vector3d achieved;
  Eigen::Matrix3d rotation;
  manip_task_execution::forwardKinematics(q, ArmModel{}, achieved, rotation);

  const double position_error = (target - achieved).norm();
  const Eigen::Vector3d axis_desired =
    manip_task_execution::desiredToolAxis(target, mode.tilt_from_vertical);
  const double dot = std::max(-1.0, std::min(1.0, rotation.col(2).dot(axis_desired)));
  const double direction_error_deg = std::acos(dot) * 180.0 / M_PI;

  std::cout << std::fixed << std::setprecision(6)
            << x << "," << y << "," << z << "," << mode_name << ","
            << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "," << q[4] << ","
            << position_error << "," << direction_error_deg << "\n";
  return true;
}

/// Modo "shift" (fila de alcance, 2026-08-28): tabela de depuracao de campo.
/// Uma linha por (candidato, tilt): candidato 0 = sem mover a base; depois
/// cada |dy| de ReachShiftOptions::candidates_m na direcao que aproxima o
/// alvo, ja com o desconto de undershoot que a busca aplica. Linhas
/// "afasta" sao candidatos que a busca (allow_overshoot=false) nem testa.
/// No fim, a escolha oficial de findBaseShiftForReach.
int runShiftMode(double x, double y, double z, double lift_m)
{
  using manip_task_execution::ReachShiftOptions;

  ReachShiftOptions opts;
  opts.lift_m = lift_m;
  const Eigen::Vector3d target(x, y, z);

  std::cout << "candidato_m,dy_efetivo_m,x_manip,y_manip,z_manip,tilt_graus,ik,ik_lift\n";
  const double sign = x > 0.0 ? -1.0 : 1.0;
  std::vector<double> steps{0.0};
  steps.insert(steps.end(), opts.candidates_m.begin(), opts.candidates_m.end());
  for (const double d : steps) {
    // d == 0 e' a linha "sem mover" (evita imprimir -0.000 com sign negativo).
    const double dy = d == 0.0 ? 0.0 : sign * d;
    const double dy_eff = d == 0.0 ? 0.0 : dy - sign * opts.undershoot_margin_m;
    const bool overshoot = std::abs(x + dy_eff) > std::abs(x) + 1e-12;
    const Eigen::Vector3d shifted(
      manip_task_execution::manipXAfterBaseShift(x, dy_eff), y, z);
    for (const double tilt : opts.tilts_rad) {
      std::array<double, 5> q{};
      const bool ok = manip_task_execution::solveIk(
        shifted, tilt, opts.q5_fixed, q, opts.model, opts.ik);
      std::string lift_state = "-";
      if (lift_m != 0.0) {
        std::array<double, 5> q_lift{};
        const Eigen::Vector3d lifted = shifted + Eigen::Vector3d(0.0, 0.0, lift_m);
        lift_state = manip_task_execution::solveIk(
          lifted, tilt, opts.q5_fixed, q_lift, opts.model, opts.ik) ? "ok" : "SEM_SOLUCAO";
      }
      std::cout << std::fixed << std::setprecision(3)
                << dy << "," << dy_eff << ","
                << shifted.x() << "," << shifted.y() << "," << shifted.z() << ","
                << std::setprecision(0) << tilt * 180.0 / M_PI << ","
                << (ok ? "ok" : "SEM_SOLUCAO") << "," << lift_state
                << (overshoot ? ",afasta" : "") << "\n";
    }
  }

  const auto result = manip_task_execution::findBaseShiftForReach(target, opts);
  std::cout << std::fixed << std::setprecision(3)
            << "escolha: alcancavel_agora=" << (result.reachable_now ? "sim" : "nao")
            << " shift_m=" << result.shift_m
            << " (base_footprint, + = esquerda; 0 = nada resolve)"
            << " tilt_graus=" << std::setprecision(0) << result.tilt_rad * 180.0 / M_PI
            << std::setprecision(3)
            << " alvo_deslocado=(" << result.shifted_target.x() << ","
            << result.shifted_target.y() << "," << result.shifted_target.z() << ")\n";
  return result.reachable_now || result.shift_m != 0.0 ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc >= 5 && std::string(argv[1]) == "shift") {
    const double lift_m = argc >= 6 ? std::atof(argv[5]) : 0.0;
    return runShiftMode(std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4]), lift_m);
  }

  std::cout << "x,y,z,modo,q1,q2,q3,q4,q5,erro_pos_m,erro_dir_graus\n";

  if (argc >= 4) {
    const double x = std::atof(argv[1]);
    const double y = std::atof(argv[2]);
    const double z = std::atof(argv[3]);
    if (argc >= 5 && std::string(argv[4]) == "free") {
      std::cerr << "uso: custom_ik_check x y z <modo> free  (o modo vem antes de 'free')\n";
      return 2;
    }
    const Mode mode = parseMode(argc >= 5 ? argv[4] : "forward");
    const bool free_orientation = argc >= 6 && std::string(argv[5]) == "free";
    return reportOne(x, y, z, mode, 0.0, free_orientation) ? 0 : 1;
  }

  // Grade de varredura: alcance util do braco a frente e para os lados.
  int solved = 0;
  int total = 0;
  for (double x = 0.20; x <= 0.45001; x += 0.05) {
    for (double y = -0.15; y <= 0.15001; y += 0.075) {
      for (double z = 0.05; z <= 0.45001; z += 0.10) {
        for (const auto dir :
          {ToolDirection::kForward, ToolDirection::kMiddle, ToolDirection::kDown})
        {
          ++total;
          if (reportOne(x, y, z, modeFor(dir), 0.0)) {
            ++solved;
          }
        }
      }
    }
  }

  std::cerr << "resolvidos " << solved << "/" << total << "\n";
  return 0;
}
