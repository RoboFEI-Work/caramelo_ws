// Espera o braco ASSENTAR depois de uma execucao do MoveIt (2026-10-27).
//
// Por que existe: com o barramento a 100 Hz e os limites de 3,0 rad/s /
// 8,0 rad/s2 (caramelo_ws 047d2dc, 2026-10-26) o joint_trajectory_controller
// da Pi declara SUCCEEDED no fim do tempo da trajetoria (tolerancia de goal
// 0,30 rad), mas os servos ainda estao convergindo. O move_group espera so
// 3 amostras de joint_states "paradas" (variacao < allowed_start_tolerance
// por amostra = 30 ms a 100 Hz, ou seja, uma junta a 0,5 rad/s passa por
// parada) e devolve o execute. O no planeja a proxima trajetoria a partir
// desse estado e, ~80 ms depois, o move_group recusa com "Invalid
// Trajectory: start point deviates from current robot state more than
// 0.01" — o braco nao se mexe e o pick/place conclui que a trajetoria
// falhou (tour de 2026-10-27 14:05: 3 recusas num pick, bloco preso na garra
// e estacao abortada; place as 14:12 pelo mesmo motivo). Antes de 26/10
// (25 Hz, 1,2 rad/s) nao houve UMA recusa em 451 execucoes.
//
// O que faz: le as juntas do grupo pelo MoveGroupInterface (mesma fonte que
// o move_group usa para validar) e so devolve quando TODAS ficaram dentro
// de tol_rad por window_s seguidos. Nao exige chegar no alvo (o joint 4
// para no limite do servo e a garra para no bloco): so exige PARAR. Se o
// braco continua se mexendo apos timeout_s, avisa e segue — a validacao do
// move_group continua sendo a rede.
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace manip_task_execution
{

struct ArmSettleParams
{
  /// Tempo seguido com todas as juntas dentro de tol_rad. 0 = desliga a espera.
  double window_s{0.12};
  /// Deslocamento maximo de uma junta dentro da janela (XM540: 1 tick = 1,5 mrad).
  double tol_rad{0.004};
  /// Desiste (e segue) depois disso.
  double timeout_s{1.5};
  /// Intervalo entre leituras (cada leitura espera uma amostra NOVA de joint_states).
  double poll_s{0.02};
};

struct ArmSettleResult
{
  bool settled{true};
  double waited_s{0.0};
  /// Ultima junta vista em movimento (vazio = nenhuma).
  std::string moving_joint;
  double max_drift_rad{0.0};
};

/// Declara os parametros post_move_settle_* no no (mesmos nomes nos 3 nos).
inline ArmSettleParams declareArmSettleParams(rclcpp::Node & node)
{
  ArmSettleParams p;
  p.window_s = node.declare_parameter<double>("post_move_settle_window_s", p.window_s);
  p.tol_rad = node.declare_parameter<double>("post_move_settle_tol_rad", p.tol_rad);
  p.timeout_s = node.declare_parameter<double>("post_move_settle_timeout_s", p.timeout_s);
  return p;
}

/// Bloqueia ate as juntas do grupo pararem (ver cabecalho). cancel_requested
/// (opcional) interrompe a espera; a funcao nunca falha — so informa.
inline ArmSettleResult waitForGroupToSettle(
  moveit::planning_interface::MoveGroupInterface & iface,
  const ArmSettleParams & p,
  const std::function<bool()> & cancel_requested = nullptr)
{
  using clock = std::chrono::steady_clock;
  using fsec = std::chrono::duration<double>;

  ArmSettleResult r;
  if (p.window_s <= 0.0) {
    return r;
  }

  const auto t0 = clock::now();
  const auto deadline = t0 + std::chrono::duration_cast<clock::duration>(fsec(p.timeout_s));
  const auto window = std::chrono::duration_cast<clock::duration>(fsec(p.window_s));
  const auto poll = std::chrono::duration_cast<clock::duration>(fsec(std::max(0.005, p.poll_s)));
  const std::vector<std::string> names = iface.getJointNames();

  // getCurrentJointValues espera uma amostra de joint_states mais nova que
  // "agora" (ate 1 s): cada leitura e' fresca, nao o cache do monitor.
  std::vector<double> anchor = iface.getCurrentJointValues();
  auto anchor_time = clock::now();
  if (anchor.empty()) {
    r.settled = false;  // sem joint_states: nao da para afirmar nada
    r.waited_s = fsec(clock::now() - t0).count();
    return r;
  }

  r.settled = false;
  while (clock::now() < deadline) {
    if (cancel_requested && cancel_requested()) {
      break;
    }
    rclcpp::sleep_for(poll);
    const std::vector<double> cur = iface.getCurrentJointValues();
    if (cur.size() != anchor.size()) {
      break;  // perdeu o joint_states no meio: segue sem afirmar nada
    }
    double max_drift = 0.0;
    std::size_t worst = 0;
    for (std::size_t i = 0; i < cur.size(); ++i) {
      const double d = std::fabs(cur[i] - anchor[i]);
      if (d > max_drift) {
        max_drift = d;
        worst = i;
      }
    }
    if (max_drift > p.tol_rad) {
      // Ainda se mexe: a janela recomeca deste ponto.
      anchor = cur;
      anchor_time = clock::now();
      r.moving_joint = worst < names.size() ? names[worst] : std::to_string(worst);
      r.max_drift_rad = max_drift;
      continue;
    }
    if (clock::now() - anchor_time >= window) {
      r.settled = true;
      break;
    }
  }
  r.waited_s = fsec(clock::now() - t0).count();
  return r;
}

/// Espera + log padrao (INFO quando demorou, WARN quando nao assentou).
inline void settleAfterMove(
  moveit::planning_interface::MoveGroupInterface & iface,
  const ArmSettleParams & p,
  const rclcpp::Logger & logger,
  const std::string & label,
  const std::function<bool()> & cancel_requested = nullptr)
{
  const ArmSettleResult r = waitForGroupToSettle(iface, p, cancel_requested);
  if (!r.settled) {
    RCLCPP_WARN(
      logger,
      "[%s] braco AINDA se mexia apos %.2f s (ultima junta %s, %.1f mrad na janela) — "
      "seguindo; o move_group pode recusar a proxima trajetoria (start point deviates).",
      label.c_str(), r.waited_s, r.moving_joint.c_str(), r.max_drift_rad * 1e3);
  } else if (r.waited_s > p.window_s + 0.05) {
    RCLCPP_INFO(
      logger,
      "[%s] braco assentou em %.0f ms (ultima junta a parar: %s).",
      label.c_str(), r.waited_s * 1e3, r.moving_joint.c_str());
  }
}

}  // namespace manip_task_execution
