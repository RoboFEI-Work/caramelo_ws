#pragma once

// IK numerica propria do braco do Caramelo (5 GDL), portada do solver Python
// do operador (`ik_solver_foto.py`, damped least-squares com numpy).
//
// Por que existe: a IK do MoveIt neste braco roda com rotation_scale 0.03, ou
// seja, praticamente position-only — um braco de 5 GDL nao satisfaz 6 GDL de
// pose. Pior: quando o setPoseTarget falha, o codigo caia em
// setApproximateJointValueTarget, que devolve a solucao alcancavel MAIS
// PROXIMA e ja levou o braco a fechar a garra no vazio. Este solver permite
// pedir explicitamente a DIRECAO da ferramenta (frente / 45 graus / baixo) e
// devolve as juntas, que sao entao comandadas por setJointValueTarget — o
// MoveIt segue fazendo colisao, time-parameterization e execucao.
//
// IMPORTANTE — FRAME: a entrada e o alvo do TCP no frame da BASE DO BRACO
// (`manip_base_link`), NAO em base_footprint. Entre os dois ha translacao
// (0.217, 0, 0.083) E rotacao de yaw -1.57 (manip_mount_rpy no
// robot.urdf.xacro). Use tf2 para converter antes de chamar.

#include <array>
#include <cstddef>

#include <Eigen/Dense>

namespace manip_task_execution
{

/// Direcao para a qual o eixo Z do TCP deve apontar.
enum class ToolDirection
{
  kForward,  ///< horizontal, radialmente para fora (entrar numa prateleira)
  kMiddle,   ///< 45 graus entre frente e baixo
  kDown,     ///< top-down (pegada de mesa)
};

/// Cadeia cinematica, conferida contra
/// caramelo_description/urdf/mech/manipulator_arm.xacro (2026-08-12):
/// joint1 na origem (rotZ), joint2 +0.09775 Z (rotY), joint3 +0.185 Z (rotY),
/// joint4 +0.205 Z (rotY), joint5 (-0.03, 0, 0.07815) (rotZ), tcp +0.17 Z.
struct ArmModel
{
  double z12{0.09775};
  double z23{0.185};
  double z34{0.205};
  double t45_x{-0.03};
  double t45_z{0.07815};
  double tcp_z{0.17};

  // Limites do URDF. j3 fica em +-2.6 por decisao do operador (2026-08-12):
  // houve uma medicao em 10/08 sugerindo batente do servo em ~2.17, mas o
  // operador testou depois com 2.6 sem problema. Se a fase 1 da shelf parar
  // curto, este e o primeiro lugar a olhar.
  double j1_min{-3.14}, j1_max{3.14};
  double j2_min{-1.8}, j2_max{1.8};
  double j3_min{-2.6}, j3_max{2.6};
  double j4_min{-2.2}, j4_max{2.2};
};

/// Parametros do laco de Newton amortecido (mesmos defaults do script Python).
struct IkOptions
{
  int max_iterations{250};
  double position_tolerance{1e-5};   ///< criterio de parada (m)
  double finite_diff_step{1e-6};
  double damping{1e-2};
  double orientation_weight{0.4};
  double accept_position_error{5e-4};  ///< erro maximo aceito na solucao (m)
};

/// Cinematica direta: devolve a posicao do TCP e a rotacao dele, no frame da
/// base do braco.
void forwardKinematics(
  const std::array<double, 5> & q,
  const ArmModel & model,
  Eigen::Vector3d & tcp_position,
  Eigen::Matrix3d & tcp_rotation);

/// Eixo Z desejado para o TCP, dado o alvo e o modo de direcao.
Eigen::Vector3d desiredToolAxis(const Eigen::Vector3d & target, ToolDirection direction);

/// Resolve a IK para o alvo do TCP (no frame da base do braco).
///
/// `q5_fixed` e imposto (o braco tem 5 GDL; o solver otimiza q1..q4 e usa q5
/// como dado). `q_out` recebe q1..q5 em radianos.
/// Retorna false se nenhuma solucao ficou dentro de `accept_position_error`.
bool solveIk(
  const Eigen::Vector3d & tcp_target_in_arm_base,
  ToolDirection direction,
  double q5_fixed,
  std::array<double, 5> & q_out,
  const ArmModel & model = ArmModel{},
  const IkOptions & options = IkOptions{});

}  // namespace manip_task_execution
