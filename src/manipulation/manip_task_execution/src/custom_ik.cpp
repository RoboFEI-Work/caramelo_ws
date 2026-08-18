#include "manip_task_execution/custom_ik.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace manip_task_execution
{

namespace
{

Eigen::Matrix3d rotZ(double theta)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
}

Eigen::Matrix3d rotY(double theta)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()));
}

double clampToLimit(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

void clampJoints(std::array<double, 5> & q, const ArmModel & m)
{
  q[0] = clampToLimit(q[0], m.j1_min, m.j1_max);
  q[1] = clampToLimit(q[1], m.j2_min, m.j2_max);
  q[2] = clampToLimit(q[2], m.j3_min, m.j3_max);
  q[3] = clampToLimit(q[3], m.j4_min, m.j4_max);
}

/// Residuo 6D: 3 de posicao + 3 de orientacao (produto vetorial entre o eixo
/// atual e o desejado, escalado pelo peso).
Eigen::Matrix<double, 6, 1> residual(
  const std::array<double, 5> & q,
  const Eigen::Vector3d & target,
  const Eigen::Vector3d & axis_desired,
  const ArmModel & model,
  double orientation_weight,
  Eigen::Vector3d & position_error_out)
{
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
  forwardKinematics(q, model, position, rotation);

  position_error_out = target - position;
  const Eigen::Vector3d axis_current = rotation.col(2);
  const Eigen::Vector3d orientation_error = axis_current.cross(axis_desired);

  Eigen::Matrix<double, 6, 1> res;
  res.head<3>() = position_error_out;
  res.tail<3>() = orientation_weight * orientation_error;
  return res;
}

/// Grade de sementes igual a do script Python: q1 fechado pelo azimute do
/// alvo, e q2/q3/q4 varridos em 7x7x7. Braco de 5 GDL tem multiplas solucoes;
/// varrer sementes e o que evita cair sempre no mesmo minimo local.
std::vector<std::array<double, 4>> seedsForTarget(double x, double y)
{
  const double q1 = std::atan2(y, x);
  std::vector<std::array<double, 4>> seeds;
  seeds.reserve(7 * 7 * 7);

  for (int i2 = 0; i2 < 7; ++i2) {
    const double s2 = -1.5 + (3.0 * i2) / 6.0;
    for (int i3 = 0; i3 < 7; ++i3) {
      const double s3 = -2.2 + (4.4 * i3) / 6.0;
      for (int i4 = 0; i4 < 7; ++i4) {
        const double s4 = -2.0 + (4.0 * i4) / 6.0;
        seeds.push_back({q1, s2, s3, s4});
      }
    }
  }
  return seeds;
}

/// Uma corrida do Newton amortecido a partir de uma semente.
bool solveFromSeed(
  const Eigen::Vector3d & target,
  const std::array<double, 4> & seed,
  double q5,
  const ArmModel & model,
  const Eigen::Vector3d & axis_desired,
  const IkOptions & opt,
  std::array<double, 5> & q_out)
{
  std::array<double, 5> q{seed[0], seed[1], seed[2], seed[3], q5};

  for (int iter = 0; iter < opt.max_iterations; ++iter) {
    Eigen::Vector3d position_error;
    const Eigen::Matrix<double, 6, 1> err =
      residual(q, target, axis_desired, model, opt.orientation_weight, position_error);

    if (position_error.norm() < opt.position_tolerance) {
      q_out = q;
      return true;
    }

    // Jacobiana numerica 6x4 (q5 e fixo, nao entra na otimizacao).
    Eigen::Matrix<double, 6, 4> J;
    for (int col = 0; col < 4; ++col) {
      std::array<double, 5> q_h = q;
      q_h[static_cast<std::size_t>(col)] += opt.finite_diff_step;
      Eigen::Vector3d ignored;
      const Eigen::Matrix<double, 6, 1> err_h =
        residual(q_h, target, axis_desired, model, opt.orientation_weight, ignored);
      J.col(col) = (err_h - err) / opt.finite_diff_step;
    }

    // Damped least-squares: dq = -J^T (J J^T + l^2 I)^-1 e
    const Eigen::Matrix<double, 6, 6> A =
      J * J.transpose() +
      (opt.damping * opt.damping) * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 4, 1> dq = -J.transpose() * A.ldlt().solve(err);

    for (int i = 0; i < 4; ++i) {
      q[static_cast<std::size_t>(i)] += dq(i);
    }
    clampJoints(q, model);
  }

  return false;
}

}  // namespace

void forwardKinematics(
  const std::array<double, 5> & q,
  const ArmModel & m,
  Eigen::Vector3d & tcp_position,
  Eigen::Matrix3d & tcp_rotation)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  T.rotate(rotZ(q[0]));
  T.translate(Eigen::Vector3d(0.0, 0.0, m.z12));
  T.rotate(rotY(q[1]));
  T.translate(Eigen::Vector3d(0.0, 0.0, m.z23));
  T.rotate(rotY(q[2]));
  T.translate(Eigen::Vector3d(0.0, 0.0, m.z34));
  T.rotate(rotY(q[3]));
  T.translate(Eigen::Vector3d(m.t45_x, 0.0, m.t45_z));
  T.rotate(rotZ(q[4]));
  T.translate(Eigen::Vector3d(0.0, 0.0, m.tcp_z));

  tcp_position = T.translation();
  tcp_rotation = T.rotation();
}

Eigen::Vector3d desiredToolAxis(const Eigen::Vector3d & target, ToolDirection direction)
{
  const Eigen::Vector3d down(0.0, 0.0, -1.0);
  if (direction == ToolDirection::kDown) {
    return down;
  }

  // Direcao radial no plano XY (para onde o braco "aponta" ao alcancar o alvo).
  const double r = std::hypot(target.x(), target.y());
  const Eigen::Vector3d radial =
    (r < 1e-9) ? Eigen::Vector3d(1.0, 0.0, 0.0)
               : Eigen::Vector3d(target.x() / r, target.y() / r, 0.0);

  if (direction == ToolDirection::kMiddle) {
    return (radial + down) / std::sqrt(2.0);
  }
  return radial;
}

bool solveIk(
  const Eigen::Vector3d & tcp_target_in_arm_base,
  ToolDirection direction,
  double q5_fixed,
  std::array<double, 5> & q_out,
  const ArmModel & model,
  const IkOptions & options)
{
  const Eigen::Vector3d axis_desired = desiredToolAxis(tcp_target_in_arm_base, direction);
  const double target_azimuth =
    std::atan2(tcp_target_in_arm_base.y(), tcp_target_in_arm_base.x());
  const double target_radius =
    std::hypot(tcp_target_in_arm_base.x(), tcp_target_in_arm_base.y());

  bool found = false;
  double best_orientation_error = 0.0;
  double best_joint_norm = 0.0;

  for (const auto & seed :
       seedsForTarget(tcp_target_in_arm_base.x(), tcp_target_in_arm_base.y()))
  {
    std::array<double, 5> q{};
    if (!solveFromSeed(
        tcp_target_in_arm_base, seed, q5_fixed, model, axis_desired, options, q))
    {
      continue;
    }

    Eigen::Vector3d position;
    Eigen::Matrix3d rotation;
    forwardKinematics(q, model, position, rotation);

    if ((tcp_target_in_arm_base - position).norm() > options.accept_position_error) {
      continue;
    }

    // Familia frontal apenas (ver IkOptions::require_forward). O azimute so e
    // bem definido com o alvo afastado do eixo da junta 1.
    if (options.require_forward && target_radius > 0.05) {
      double dq1 = q[0] - target_azimuth;
      while (dq1 > M_PI) {dq1 -= 2.0 * M_PI;}
      while (dq1 < -M_PI) {dq1 += 2.0 * M_PI;}
      if (std::abs(dq1) > M_PI / 2.0 || q[2] < 0.0) {
        continue;
      }
    }

    // Criterio de escolha igual ao do Python: primeiro quem aponta melhor na
    // direcao pedida, depois a solucao de menor norma articular.
    const double orientation_error = rotation.col(2).cross(axis_desired).norm();
    double joint_norm = 0.0;
    for (const double v : q) {
      joint_norm += v * v;
    }
    joint_norm = std::sqrt(joint_norm);

    if (!found ||
      orientation_error < best_orientation_error - 1e-9 ||
      (std::abs(orientation_error - best_orientation_error) <= 1e-9 &&
      joint_norm < best_joint_norm))
    {
      found = true;
      best_orientation_error = orientation_error;
      best_joint_norm = joint_norm;
      q_out = q;
    }
  }

  return found;
}

double computeWristForTagYaw(double tag_yaw_in_arm_base, double q1)
{
  return std::remainder(q1 - tag_yaw_in_arm_base, M_PI);
}

double projectedFrameYaw(const Eigen::Matrix3d & rotation)
{
  const Eigen::Vector3d x_axis = rotation.col(0);
  if (std::hypot(x_axis.x(), x_axis.y()) >= 0.2) {
    return std::atan2(x_axis.y(), x_axis.x());
  }
  const Eigen::Vector3d y_axis = rotation.col(1);
  return std::atan2(y_axis.y(), y_axis.x()) - M_PI / 2.0;
}

}  // namespace manip_task_execution
