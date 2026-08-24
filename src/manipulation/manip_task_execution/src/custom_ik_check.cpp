// Conferencia do porte C++ da IK contra o solver Python do operador.
//
// Uso:
//   custom_ik_check                       -> varre uma grade de alvos e imprime
//                                            uma linha CSV por alvo resolvido
//   custom_ik_check <x> <y> <z> [modo]    -> resolve um alvo unico
//                                            (modo: forward | shallow | middle | down
//                                             | tilt<graus>, ex. tilt20 = por cima
//                                             inclinado 20 graus a partir da vertical)
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

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

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
bool reportOne(double x, double y, double z, const Mode & mode, double q5)
{
  const Eigen::Vector3d target(x, y, z);
  std::array<double, 5> q{};

  if (!manip_task_execution::solveIk(target, mode.tilt_from_vertical, q5, q)) {
    std::cout << std::fixed << std::setprecision(4)
              << x << "," << y << "," << z << "," << mode.name
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
            << x << "," << y << "," << z << "," << mode.name << ","
            << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "," << q[4] << ","
            << position_error << "," << direction_error_deg << "\n";
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::cout << "x,y,z,modo,q1,q2,q3,q4,q5,erro_pos_m,erro_dir_graus\n";

  if (argc >= 4) {
    const double x = std::atof(argv[1]);
    const double y = std::atof(argv[2]);
    const double z = std::atof(argv[3]);
    const Mode mode = parseMode(argc >= 5 ? argv[4] : "forward");
    return reportOne(x, y, z, mode, 0.0) ? 0 : 1;
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
