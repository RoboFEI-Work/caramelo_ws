// Helpers puros do empilhamento (2026-10-21). Ver o .hpp.
#include "manip_task_execution/place_precision.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace manip_task_execution
{

std::optional<int> tagIdFromFrame(const std::string & frame)
{
  static const std::string kPrefix = "tag_";
  if (frame.size() <= kPrefix.size() || frame.compare(0, kPrefix.size(), kPrefix) != 0) {
    return std::nullopt;
  }
  int value = 0;
  for (std::size_t i = kPrefix.size(); i < frame.size(); ++i) {
    const char c = frame[i];
    if (c < '0' || c > '9') {
      return std::nullopt;
    }
    if (value > 100000) {
      return std::nullopt;  // nunca um id de AprilTag
    }
    value = value * 10 + (c - '0');
  }
  return value;
}

StackTargetKind classifyStackTarget(
  const std::string & frame,
  const std::vector<std::int64_t> & reference_ids)
{
  const std::optional<int> id = tagIdFromFrame(frame);
  if (!id) {
    return StackTargetKind::kCube;
  }
  const std::int64_t id64 = *id;
  if (std::find(reference_ids.begin(), reference_ids.end(), id64) != reference_ids.end()) {
    return StackTargetKind::kReference;
  }
  return StackTargetKind::kCube;
}

double nearestQuarterTurn(double yaw_tag, double yaw_current)
{
  // remainder(x, pi/2) devolve x - n*pi/2 com n o inteiro mais proximo:
  // resultado em [-pi/4, pi/4] (empate exato vai para o n par).
  return std::remainder(yaw_tag - yaw_current, M_PI / 2.0);
}

double tiltDropZCompensation(double tilt_rad, double cube_width_m, double cube_height_m)
{
  const double t = std::abs(tilt_rad);
  if (t < 1e-9) {
    return 0.0;
  }
  // Quina mais baixa: h*cos(t) + (w/2)*sin(t) abaixo do TCP; na vertical e'
  // h. A diferenca e' o que a soltura precisa subir.
  const double w = std::max(0.0, cube_width_m);
  const double h = std::max(0.0, cube_height_m);
  return std::max(0.0, (w / 2.0) * std::sin(t) - h * (1.0 - std::cos(t)));
}

double pileRegraspFloorZ(double z_expected_top, double cube_height_m, double z_tol_m)
{
  return z_expected_top - std::max(0.0, cube_height_m) - std::max(0.0, z_tol_m);
}

PileFix pileFixFor(
  double z_cube_tag, double z_expected_top, double cube_height_m, double z_tol_m)
{
  if (z_cube_tag < pileRegraspFloorZ(z_expected_top, cube_height_m, z_tol_m)) {
    return PileFix::kLeave;
  }
  if (std::abs(z_cube_tag - z_expected_top) <= std::max(0.0, z_tol_m)) {
    return PileFix::kCorrectXy;
  }
  return PileFix::kRedropOriginal;
}

const char * pileFixName(PileFix f)
{
  switch (f) {
    case PileFix::kCorrectXy: return "corrigir_xy";
    case PileFix::kRedropOriginal: return "soltar_no_alvo_original";
    case PileFix::kLeave: return "nao_re-pegar";
  }
  return "?";
}

double expectedPileTopZ(double z_base_tag, int cubes_above, double cube_height_m)
{
  return z_base_tag + std::max(0, cubes_above) * cube_height_m;
}

std::optional<StackMemoryEntry> findStackMemory(
  const std::vector<StackMemoryEntry> & entries, const std::string & frame)
{
  static const std::string kPrefix = "stack:";
  std::optional<StackMemoryEntry> found;
  for (const auto & e : entries) {
    if (e.tag_frame == frame && e.slot.compare(0, kPrefix.size(), kPrefix) == 0) {
      found = e;  // o ultimo da lista vence (ordem de gravacao)
    }
  }
  return found;
}

int stackLevelFromMemory(
  const std::vector<StackMemoryEntry> & entries, const std::string & base_frame)
{
  static const std::string kPrefix = "stack:";
  int level = 0;
  std::set<std::string> visited;
  std::string frame = base_frame;
  while (!frame.empty() && visited.insert(frame).second) {
    const auto rec = findStackMemory(entries, frame);
    if (!rec) {
      break;
    }
    ++level;
    frame = rec->slot.substr(kPrefix.size());
  }
  return level;
}

std::optional<StackMemoryEntry> findStackRoot(
  const std::vector<StackMemoryEntry> & entries, const std::string & base_frame)
{
  static const std::string kPrefix = "stack:";
  std::optional<StackMemoryEntry> root;
  std::set<std::string> visited;
  std::string frame = base_frame;
  while (!frame.empty() && visited.insert(frame).second) {
    const auto rec = findStackMemory(entries, frame);
    if (!rec) {
      break;
    }
    root = rec;
    frame = rec->slot.substr(kPrefix.size());
  }
  return root;
}

PlacementDecision decidePlacement(
  const PlacementObservation & obs, const PlacementLimits & lim)
{
  const bool budget = lim.attempts_done < lim.max_attempts;
  if (obs.seen) {
    const bool ok = obs.err_xy_m <= lim.xy_tol_m && std::abs(obs.err_z_m) <= lim.z_tol_m;
    if (ok) {
      return PlacementDecision::kAccept;
    }
    return budget ? PlacementDecision::kRetry : PlacementDecision::kGiveUp;
  }
  // Tag nao vista.
  if (lim.fk_proxy_tol_m < 0.0) {
    return PlacementDecision::kAcceptUnverified;
  }
  if (obs.fk_err_xy_m >= 0.0 && obs.fk_err_xy_m <= lim.fk_proxy_tol_m) {
    return PlacementDecision::kAcceptUnverified;
  }
  if (lim.retry_unseen && budget) {
    return PlacementDecision::kRetry;
  }
  return PlacementDecision::kGiveUp;
}

const char * placementDecisionName(PlacementDecision d)
{
  switch (d) {
    case PlacementDecision::kAccept: return "aceito";
    case PlacementDecision::kAcceptUnverified: return "aceito_sem_verificar";
    case PlacementDecision::kRetry: return "repetir";
    case PlacementDecision::kGiveUp: return "desistir";
  }
  return "?";
}

}  // namespace manip_task_execution
