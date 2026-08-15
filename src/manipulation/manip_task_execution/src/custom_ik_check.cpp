// Conferencia do porte C++ da IK contra o solver Python do operador.
//
// Uso:
//   custom_ik_check                       -> varre uma grade de alvos e imprime
//                                            uma linha CSV por alvo resolvido
//   custom_ik_check <x> <y> <z> [modo]    -> resolve um alvo unico
//                                            (modo: forward | middle | down)
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

ToolDirection parseDirection(const std::string & name)
{
  if (name == "down") {
    return ToolDirection::kDown;
  }
  if (name == "middle") {
    return ToolDirection::kMiddle;
  }
  return ToolDirection::kForward;
}

const char * directionName(ToolDirection d)
{
  switch (d) {
    case ToolDirection::kDown: return "down";
    case ToolDirection::kMiddle: return "middle";
    default: return "forward";
  }
}

/// Resolve e imprime uma linha CSV. Devolve false se nao houve solucao.
bool reportOne(double x, double y, double z, ToolDirection dir, double q5)
{
  const Eigen::Vector3d target(x, y, z);
  std::array<double, 5> q{};

  if (!manip_task_execution::solveIk(target, dir, q5, q)) {
    std::cout << std::fixed << std::setprecision(4)
              << x << "," << y << "," << z << "," << directionName(dir)
              << ",SEM_SOLUCAO\n";
    return false;
  }

  Eigen::Vector3d achieved;
  Eigen::Matrix3d rotation;
  manip_task_execution::forwardKinematics(q, ArmModel{}, achieved, rotation);

  const double position_error = (target - achieved).norm();
  const Eigen::Vector3d axis_desired = manip_task_execution::desiredToolAxis(target, dir);
  const double dot = std::max(-1.0, std::min(1.0, rotation.col(2).dot(axis_desired)));
  const double direction_error_deg = std::acos(dot) * 180.0 / M_PI;

  std::cout << std::fixed << std::setprecision(6)
            << x << "," << y << "," << z << "," << directionName(dir) << ","
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
    const ToolDirection dir = parseDirection(argc >= 5 ? argv[4] : "forward");
    return reportOne(x, y, z, dir, 0.0) ? 0 : 1;
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
          if (reportOne(x, y, z, dir, 0.0)) {
            ++solved;
          }
        }
      }
    }
  }

  std::cerr << "resolvidos " << solved << "/" << total << "\n";
  return 0;
}
