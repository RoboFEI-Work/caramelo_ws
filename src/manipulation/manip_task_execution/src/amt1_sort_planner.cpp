// Planejador puro da rotina AMT1 (2026-08-28). Ver o cabecalho.
#include "manip_task_execution/amt1_sort_planner.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace manip_task_execution
{
namespace amt1
{

SlotAssignment assignSlots(
  const std::vector<TagPose> & seen,
  const std::vector<int> & expected,
  bool left_to_right,
  double tie_eps)
{
  SlotAssignment out;

  // Primeira ocorrencia de cada id vista.
  std::vector<TagPose> unique;
  std::set<int> seen_ids;
  for (const TagPose & t : seen) {
    if (seen_ids.insert(t.id).second) {
      unique.push_back(t);
    }
  }

  // Quem ganha slot: as esperadas (ou todas, sem lista).
  const std::set<int> expected_ids(expected.begin(), expected.end());
  std::vector<TagPose> slots;
  for (const TagPose & t : unique) {
    if (expected_ids.empty() || expected_ids.count(t.id) > 0) {
      slots.push_back(t);
    } else {
      out.ignored_tags.push_back(t.id);
    }
  }
  std::sort(out.ignored_tags.begin(), out.ignored_tags.end());

  for (const int id : expected_ids) {
    if (seen_ids.count(id) == 0) {
      out.missing_tags.push_back(id);
    }
  }

  // Ordena por x na direcao pedida. O empate (|dx| < tie_eps) nao e uma
  // ordem estrita, por isso a passada de bolhas depois do sort: pares
  // vizinhos quase no mesmo x ficam com o menor y primeiro.
  const double sign = left_to_right ? 1.0 : -1.0;
  std::stable_sort(
    slots.begin(), slots.end(),
    [sign](const TagPose & a, const TagPose & b) {return sign * a.x < sign * b.x;});
  bool swapped = true;
  while (swapped) {
    swapped = false;
    for (std::size_t i = 1; i < slots.size(); ++i) {
      TagPose & prev = slots[i - 1];
      TagPose & cur = slots[i];
      if (std::abs(prev.x - cur.x) < tie_eps && cur.y < prev.y) {
        std::swap(prev, cur);
        swapped = true;
      }
    }
  }

  out.slots = slots;
  for (const TagPose & t : slots) {
    out.observed_order.push_back(t.id);
  }
  out.target_order = out.observed_order;
  std::sort(out.target_order.begin(), out.target_order.end());
  return out;
}

SortPlan planSort(
  const std::vector<int> & observed,
  const std::vector<int> & target,
  int buffer)
{
  SortPlan plan;
  plan.final_order = target;

  // Alvo tem que ser permutacao de observed (mesmo multiconjunto, sem
  // repetidos).
  const std::size_t n = observed.size();
  if (target.size() != n) {
    plan.error = "alvo_invalido";
    return plan;
  }
  std::map<int, std::size_t> pos_target;
  for (std::size_t i = 0; i < n; ++i) {
    if (!pos_target.emplace(target[i], i).second) {
      plan.error = "alvo_invalido";
      return plan;
    }
  }
  std::set<int> observed_ids;
  for (const int id : observed) {
    if (!observed_ids.insert(id).second || pos_target.count(id) == 0) {
      plan.error = "alvo_invalido";
      return plan;
    }
  }

  // dest[k] = para onde vai o cubo que esta no slot k.
  std::vector<std::size_t> dest(n);
  bool sorted = true;
  for (std::size_t k = 0; k < n; ++k) {
    dest[k] = pos_target.at(observed[k]);
    if (dest[k] != k) {
      sorted = false;
    }
  }
  if (sorted) {
    return plan;  // ops vazio, max_onboard 0
  }
  if (buffer < 2) {
    plan.error = "buffer_insuficiente";
    return plan;
  }

  std::vector<bool> visited(n, false);
  for (std::size_t start = 0; start < n; ++start) {
    if (visited[start] || dest[start] == start) {
      visited[start] = true;
      continue;
    }
    // Ciclo [c0..c(m-1)] com c(j+1) = dest[cj]; c0 = slot mais a esquerda
    // do ciclo (o primeiro nao visitado na varredura crescente).
    std::vector<std::size_t> cycle;
    std::size_t k = start;
    while (!visited[k]) {
      visited[k] = true;
      cycle.push_back(k);
      k = dest[k];
    }
    const std::size_t m = cycle.size();
    // PICK c0 (guarda a pose).
    plan.ops.push_back({SortOpType::kPick, observed[cycle[0]], static_cast<int>(cycle[0])});
    // Para j = m-1..1: PICK cj, PLACE cj -> dest[cj] = c(j+1 mod m), que
    // acabou de ficar livre (c0 no primeiro passo, c(j+1) depois).
    for (std::size_t j = m - 1; j >= 1; --j) {
      const std::size_t cj = cycle[j];
      plan.ops.push_back({SortOpType::kPick, observed[cj], static_cast<int>(cj)});
      plan.ops.push_back({SortOpType::kPlace, observed[cj], static_cast<int>(dest[cj])});
    }
    // PLACE c0 -> dest[c0] = c1, o ultimo liberado.
    plan.ops.push_back({SortOpType::kPlace, observed[cycle[0]], static_cast<int>(dest[cycle[0]])});
  }

  plan.max_onboard = 2;
  for (const SortOp & op : plan.ops) {
    if (op.type == SortOpType::kPick) {
      ++plan.picks;
    } else {
      ++plan.places;
    }
  }
  return plan;
}

SimulationResult simulatePlan(
  const std::vector<int> & observed,
  const std::vector<SortOp> & ops,
  int buffer)
{
  SimulationResult res;
  std::vector<int> table = observed;  // 0 = vazio
  std::vector<int> onboard;
  const auto fail = [&res, &table, &onboard](const std::string & why) {
      res.ok = false;
      res.error = why;
      res.final_order = table;
      res.onboard = onboard;
      return res;
    };

  for (std::size_t i = 0; i < ops.size(); ++i) {
    const SortOp & op = ops[i];
    const std::string where = " (op " + std::to_string(i) + " " + describeOp(op) + ")";
    if (op.slot < 0 || static_cast<std::size_t>(op.slot) >= table.size()) {
      return fail("slot fora da mesa" + where);
    }
    if (op.tag == 0) {
      return fail("tag 0 nao e valida" + where);
    }
    if (op.type == SortOpType::kPick) {
      if (table[op.slot] != op.tag) {
        return fail("pick de tag que nao esta nesse slot" + where);
      }
      if (static_cast<int>(onboard.size()) >= buffer) {
        return fail("buffer estourado" + where);
      }
      table[op.slot] = 0;
      onboard.push_back(op.tag);
      ++res.picks;
      res.max_onboard = std::max(res.max_onboard, static_cast<int>(onboard.size()));
    } else {
      if (table[op.slot] != 0) {
        return fail("place em slot ocupado" + where);
      }
      const auto it = std::find(onboard.begin(), onboard.end(), op.tag);
      if (it == onboard.end()) {
        return fail("place de tag que nao esta a bordo" + where);
      }
      onboard.erase(it);
      table[op.slot] = op.tag;
      ++res.places;
    }
  }
  if (!onboard.empty()) {
    return fail("cubo(s) ainda a bordo no fim");
  }
  res.ok = true;
  res.final_order = table;
  return res;
}

RegistrationResult registrationDelta(
  const std::vector<AnchorDelta> & anchors,
  std::size_t min_anchors,
  double max_spread_m)
{
  RegistrationResult out;
  out.used = anchors.size();
  if (anchors.size() < std::max<std::size_t>(1, min_anchors)) {
    out.reason = "poucas_ancoras";
    return out;
  }
  double sx = 0.0;
  double sy = 0.0;
  for (const AnchorDelta & a : anchors) {
    sx += a.dx;
    sy += a.dy;
  }
  out.dx = sx / static_cast<double>(anchors.size());
  out.dy = sy / static_cast<double>(anchors.size());
  for (const AnchorDelta & a : anchors) {
    out.spread = std::max(out.spread, std::hypot(a.dx - out.dx, a.dy - out.dy));
  }
  if (out.spread > max_spread_m) {
    out.reason = "espalhamento";
    return out;
  }
  out.ok = true;
  return out;
}

std::string describeOp(const SortOp & op)
{
  if (op.type == SortOpType::kPick) {
    return "pick_" + std::to_string(op.tag);
  }
  return "place_" + std::to_string(op.tag) + "_slot_" + std::to_string(op.slot);
}

}  // namespace amt1
}  // namespace manip_task_execution
