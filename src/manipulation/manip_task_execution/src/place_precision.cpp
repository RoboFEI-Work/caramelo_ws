// Helpers puros do place de precisao (2026-10-21). Ver o .hpp.
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
  const std::vector<std::int64_t> & slot_ids,
  const std::vector<std::int64_t> & reference_ids)
{
  const std::optional<int> id = tagIdFromFrame(frame);
  if (!id) {
    return StackTargetKind::kCube;
  }
  const std::int64_t id64 = *id;
  if (std::find(slot_ids.begin(), slot_ids.end(), id64) != slot_ids.end()) {
    return StackTargetKind::kSlot;
  }
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

double PlanarOffset::radius() const
{
  return std::hypot(dx, dy);
}

std::vector<PlanarOffset> spiralOffsets(double step_m, double max_radius_m)
{
  std::vector<PlanarOffset> out;
  out.push_back(PlanarOffset{0.0, 0.0});
  if (!(step_m > 0.0) || !(max_radius_m > 0.0)) {
    return out;
  }
  const int n = static_cast<int>(std::floor(max_radius_m / step_m + 1e-9));
  const double r_max = max_radius_m + 1e-9;
  for (int i = -n; i <= n; ++i) {
    for (int j = -n; j <= n; ++j) {
      if (i == 0 && j == 0) {
        continue;
      }
      PlanarOffset p{i * step_m, j * step_m};
      if (p.radius() <= r_max) {
        out.push_back(p);
      }
    }
  }
  const auto angle = [](const PlanarOffset & p) {
      double a = std::atan2(p.dy, p.dx);
      if (a < -1e-12) {
        a += 2.0 * M_PI;
      }
      return a;
    };
  std::stable_sort(
    out.begin() + 1, out.end(),
    [&](const PlanarOffset & a, const PlanarOffset & b) {
      const double ra = a.radius();
      const double rb = b.radius();
      if (std::abs(ra - rb) > 1e-9) {
        return ra < rb;
      }
      return angle(a) < angle(b);
    });
  return out;
}

double seatedCubeTagZ(double z_slot_tag, double cube_height_m)
{
  return z_slot_tag + cube_height_m;
}

double effectiveSeatTolerance(double tol_m, double cavity_depth_m)
{
  const double half_cavity = 0.5 * std::max(0.0, cavity_depth_m);
  return std::min(std::max(0.0, tol_m), half_cavity);
}

SeatState slotSeatState(
  double z_cube_tag, double z_slot_tag, double cube_height_m,
  double cavity_depth_m, double tol_m)
{
  const double expected = seatedCubeTagZ(z_slot_tag, cube_height_m);
  const double tol = effectiveSeatTolerance(tol_m, cavity_depth_m);
  if (z_cube_tag > expected + tol) {
    return SeatState::kUnseated;
  }
  if (z_cube_tag < expected - std::max(0.0, cavity_depth_m)) {
    return SeatState::kInconsistent;
  }
  return SeatState::kSeated;
}

bool isSeatedInSlot(
  double z_cube_tag, double z_slot_tag, double cube_height_m,
  double cavity_depth_m, double tol_m)
{
  return slotSeatState(z_cube_tag, z_slot_tag, cube_height_m, cavity_depth_m, tol_m) ==
         SeatState::kSeated;
}

double slotRimCubeTagZ(double z_slot_tag, double cube_height_m, double cavity_depth_m)
{
  return z_slot_tag + std::max(0.0, cavity_depth_m) + cube_height_m;
}

double slotRegraspTcpZ(double z_cube_seen, double z_rim)
{
  return std::max(z_cube_seen, z_rim);
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

SlotAction decideSlotAction(const SlotObservation & obs, const SlotLimits & lim)
{
  const bool can_drop_again = lim.attempts_done < lim.max_attempts;
  const bool can_look_again = lim.observations_done < lim.max_observations;
  if (!obs.seen) {
    return can_look_again ? SlotAction::kReobserve : SlotAction::kLeave;
  }
  const SeatState seat = slotSeatState(
    obs.z_cube_tag, lim.z_slot_tag, lim.cube_height_m, lim.cavity_depth_m, lim.seat_tol_m);
  if (seat == SeatState::kSeated) {
    return SlotAction::kAccept;
  }
  if (seat == SeatState::kUnseated) {
    const double z_rim = slotRimCubeTagZ(lim.z_slot_tag, lim.cube_height_m, lim.cavity_depth_m);
    const double tol = effectiveSeatTolerance(lim.seat_tol_m, lim.cavity_depth_m);
    if (obs.z_cube_tag >= z_rim - tol) {
      // Cavidade rasa: a borda fica so' ~cavidade acima do assentado e uma
      // leitura ruidosa pode cair nessa faixa. Re-pegar por engano custa um
      // ciclo; por isso a borda precisa ser CONFIRMADA numa segunda leitura.
      if (can_look_again) {
        return SlotAction::kReobserve;
      }
      return can_drop_again ? SlotAction::kRegrasp : SlotAction::kLeave;
    }
  }
  // Intermediario (meio dentro / tombado) ou abaixo do fundo: so' olhar.
  return can_look_again ? SlotAction::kReobserve : SlotAction::kLeave;
}

const char * slotActionName(SlotAction a)
{
  switch (a) {
    case SlotAction::kAccept: return "encaixado";
    case SlotAction::kRegrasp: return "re-pegar_na_borda";
    case SlotAction::kReobserve: return "olhar_de_novo";
    case SlotAction::kLeave: return "deixar_onde_esta";
  }
  return "?";
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
