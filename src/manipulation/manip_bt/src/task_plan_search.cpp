// task_plan_search: regras de pick/place, guloso historico, modelo de custo e
// branch-and-bound por tempo do task_planner. Ver o cabecalho.

#include "manip_bt/task_plan_search.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace manip_bt
{
namespace task_plan
{

namespace
{

/// Pilha inicial (2026): o objeto so pode ser pego depois dos que estao em
/// cima dele.
bool pickAfterSatisfied(const PlannerState & state, const TransferItem & t)
{
  for (const int above : t.pick_after) {
    if (state.picked.count(above) == 0) {
      return false;
    }
  }
  return true;
}

// ---- pilhas ----------------------------------------------------------------
// A base de uma pilha esta "na mesa" se ja foi solta ou se nunca precisou de
// acao (comeca e termina na mesma estacao, sem ser topo de outra pilha).
bool baseOnTable(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const int base_id)
{
  if (state.placed.count(base_id) > 0) {
    return true;
  }
  const auto it = std::find_if(
    transfers.begin(), transfers.end(),
    [base_id](const TransferItem & x) {return x.obj_id == base_id;});
  return it == transfers.end() || !it->needs_place;
}

bool baseCarried(const PlannerState & state, const int base_id)
{
  return std::find(state.inventory.begin(), state.inventory.end(), base_id) !=
         state.inventory.end();
}

/// Um topo so pode ser PEGO quando a base ja esta na mesa ou tambem esta no
/// inventario (senao o topo ficaria preso a bordo esperando a base - ver a
/// revisao adversarial de 24/08: isso travava o planejador num laco infinito).
bool topPickable(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t)
{
  return t.stack_on_obj < 0 || baseOnTable(transfers, state, t.stack_on_obj) ||
         baseCarried(state, t.stack_on_obj);
}

/// Pilha inicial (2026): o place sobre a tag de referencia/base so pode
/// acontecer depois que os ocupantes desse lugar forem pegos.
bool placeAfterPickedSatisfied(const PlannerState & state, const TransferItem & t)
{
  for (const int occupant : t.place_after_picked) {
    if (state.picked.count(occupant) == 0) {
      return false;
    }
  }
  return true;
}

/// Pode ser solto AGORA: ocupantes ja pegos e (se topo) base ja na mesa.
bool placeReady(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t)
{
  return placeAfterPickedSatisfied(state, t) &&
         (t.stack_on_obj < 0 || baseOnTable(transfers, state, t.stack_on_obj));
}

/// Vai poder ser solto ao CHEGAR na estacao: direto, ou topo cuja base
/// tambem a bordo sera solta antes (recursivo na cadeia de bases a bordo).
bool placeableOnArrival(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t)
{
  if (!placeAfterPickedSatisfied(state, t)) {
    return false;
  }
  if (t.stack_on_obj < 0 || baseOnTable(transfers, state, t.stack_on_obj)) {
    return true;
  }
  if (baseCarried(state, t.stack_on_obj)) {
    return placeableOnArrival(transfers, state, findTransferById(transfers, t.stack_on_obj));
  }
  return false;
}

/// Ocupantes da pilha inicial ainda nao pegos que bloqueiam o place de `t`
/// (ou da base que `t` espera, se ela tambem esta a bordo). `self_id` conta
/// como ja pego - e' o proprio objeto cuja pegada esta sendo decidida.
std::set<int> unpickedBlockers(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t,
  const int self_id)
{
  std::set<int> blockers;
  const TransferItem * cur = &t;
  // Cadeia de bases a bordo tem no maximo kMaxOnboard elos.
  for (int depth = 0; cur != nullptr && depth <= kMaxOnboard; ++depth) {
    for (const int occupant : cur->place_after_picked) {
      if (occupant != self_id && state.picked.count(occupant) == 0) {
        blockers.insert(occupant);
      }
    }
    if (cur->stack_on_obj >= 0 && !baseOnTable(transfers, state, cur->stack_on_obj) &&
      baseCarried(state, cur->stack_on_obj))
    {
      cur = &findTransferById(transfers, cur->stack_on_obj);
    } else {
      cur = nullptr;
    }
  }
  return blockers;
}

/// `t` ocupa o lugar onde algo que esta A BORDO precisa ser solto.
bool blocksCarriedPlace(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t)
{
  for (const int carried : state.inventory) {
    const auto & c = findTransferById(transfers, carried);
    if (std::find(c.place_after_picked.begin(), c.place_after_picked.end(), t.obj_id) !=
      c.place_after_picked.end())
    {
      return true;
    }
  }
  return false;
}

/// Pode PEGAR `t` em `ws` com `onboard` objetos ja a bordo. Alem das regras
/// de pilha (base disponivel, quem esta em cima sai antes), reserva lugar
/// a bordo para os ocupantes da pilha inicial que ainda precisam sair antes
/// do place de `t`: senao o robo lotaria com 3 objetos presos esperando um
/// pick que nao cabe.
bool pickAllowedNow(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t,
  const std::string & ws,
  const int onboard)
{
  if (!t.needs_pick || state.picked.count(t.obj_id) > 0 || t.from_ws != ws) {
    return false;
  }
  if (!pickAfterSatisfied(state, t)) {
    return false;
  }
  // Ocupante da pilha inicial que trava um place a bordo sai mesmo que a
  // base dele ainda nao esteja disponivel (a base dele tambem vai sair).
  if (!topPickable(transfers, state, t) && !blocksCarriedPlace(transfers, state, t)) {
    return false;
  }
  const int reserve = static_cast<int>(unpickedBlockers(transfers, state, t, t.obj_id).size());
  return onboard + 1 + reserve <= kMaxOnboard;
}

/// Picks possiveis em `ws` partindo de `onboard_after_places` objetos a bordo
/// (os que serao soltos ao chegar ja descontados). Aproximacao: nao simula
/// pick_after entre os contados - serve para escolher a estacao.
int countPendingPicksAtWs(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const std::string & ws,
  const int onboard_after_places)
{
  int count = 0;
  for (const auto & t : transfers) {
    if (pickAllowedNow(transfers, state, t, ws, onboard_after_places + count)) {
      ++count;
    }
  }
  return count;
}

int countCarriedPlacesAtWs(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const std::string & ws)
{
  int count = 0;
  for (const int obj_id : state.inventory) {
    const auto & t = findTransferById(transfers, obj_id);
    // Topo sem base disponivel ou place bloqueado por ocupante da pilha
    // inicial nao conta (senao a estacao seria escolhida sem ter o que fazer).
    if (t.needs_place && state.placed.count(t.obj_id) == 0 && t.to_ws == ws &&
      placeableOnArrival(transfers, state, t))
    {
      ++count;
    }
  }
  return count;
}

std::vector<int> shuffledBestCandidates(
  const std::vector<std::pair<std::string, std::pair<int, int>>> & scores,
  std::mt19937 & rng)
{
  std::vector<int> best_indices;
  std::pair<int, int> best_score{-1, -1};

  for (std::size_t i = 0; i < scores.size(); ++i) {
    if (scores[i].second > best_score) {
      best_score = scores[i].second;
      best_indices.clear();
      best_indices.push_back(static_cast<int>(i));
    } else if (scores[i].second == best_score) {
      best_indices.push_back(static_cast<int>(i));
    }
  }

  std::shuffle(best_indices.begin(), best_indices.end(), rng);
  return best_indices;
}

Visit visitStation(
  const std::vector<TransferItem> & transfers,
  PlannerState & state,
  const std::string & ws,
  const std::set<int> * only)
{
  Visit visit;
  visit.ws = ws;
  visit.places = placesAtWs(transfers, state, ws);
  visit.picks = picksAtWs(transfers, state, ws, only);
  return visit;
}

}  // namespace

// ---- regras compartilhadas -----------------------------------------------------

const TransferItem & findTransferById(const std::vector<TransferItem> & transfers, const int obj_id)
{
  const auto it = std::find_if(
    transfers.begin(),
    transfers.end(),
    [obj_id](const TransferItem & t) {return t.obj_id == obj_id;});
  if (it == transfers.end()) {
    throw std::runtime_error(
            "Internal planner error: missing transfer id " + std::to_string(obj_id));
  }
  return *it;
}

bool hasUnfinishedWork(const std::vector<TransferItem> & transfers, const PlannerState & state)
{
  for (const auto & t : transfers) {
    if (t.needs_pick && state.picked.count(t.obj_id) == 0) {
      return true;
    }
    if (t.needs_place && state.placed.count(t.obj_id) == 0) {
      return true;
    }
  }
  return false;
}

std::set<std::string> candidateStations(const std::vector<TransferItem> & transfers)
{
  std::set<std::string> candidate_ws;
  for (const auto & t : transfers) {
    if (t.needs_pick && !t.from_ws.empty()) {
      candidate_ws.insert(t.from_ws);
    }
    if (t.needs_place && !t.to_ws.empty()) {
      candidate_ws.insert(t.to_ws);
    }
  }
  return candidate_ws;
}

std::vector<int> placesAtWs(
  const std::vector<TransferItem> & transfers,
  PlannerState & state,
  const std::string & ws)
{
  std::vector<int> emitted;
  const auto emit_place = [&](const TransferItem & t) {
      emitted.push_back(t.obj_id);
      state.placed.insert(t.obj_id);
    };

  // Duas passadas: primeiro o que NAO e' topo de pilha (inclui as bases),
  // depois os topos cuja base ja esta na mesa. Um topo cuja base ainda nao
  // foi solta, ou qualquer place cujo lugar ainda esta ocupado por cubo da
  // pilha inicial (place_after_picked), fica no inventario: o laco do
  // planejador pega os ocupantes nesta mesma estacao e volta aqui.
  std::vector<int> remaining_inventory;
  std::vector<int> deferred;
  for (const int obj_id : state.inventory) {
    const auto & t = findTransferById(transfers, obj_id);
    const bool here = t.needs_place && state.placed.count(t.obj_id) == 0 && t.to_ws == ws;
    if (!here) {
      remaining_inventory.push_back(obj_id);
    } else if (t.stack_on_obj >= 0 || !placeAfterPickedSatisfied(state, t)) {
      deferred.push_back(obj_id);
    } else {
      emit_place(t);
    }
  }
  // Topos: repete ate nao haver progresso (pilha de 3+: o topo do topo so
  // sai depois do meio).
  bool progress = true;
  while (progress && !deferred.empty()) {
    progress = false;
    std::vector<int> still_deferred;
    for (const int obj_id : deferred) {
      const auto & t = findTransferById(transfers, obj_id);
      if (placeReady(transfers, state, t)) {
        emit_place(t);
        progress = true;
      } else {
        still_deferred.push_back(obj_id);
      }
    }
    deferred = still_deferred;
  }
  for (const int obj_id : deferred) {
    remaining_inventory.push_back(obj_id);
  }
  state.inventory = remaining_inventory;
  return emitted;
}

std::vector<int> picksAtWs(
  const std::vector<TransferItem> & transfers,
  PlannerState & state,
  const std::string & ws,
  const std::set<int> * only)
{
  std::vector<int> emitted;
  // Topo de pilha so e' pego com a base ja na mesa ou a bordo (topPickable);
  // como a base pode ter sido pega nesta mesma passada, repete ate nao haver
  // progresso. pickAllowedNow tambem reserva lugar a bordo para ocupantes da
  // pilha inicial que ainda precisam sair.
  bool progress = true;
  while (progress) {
    progress = false;
    for (const auto & t : transfers) {
      if (static_cast<int>(state.inventory.size()) >= kMaxOnboard) {
        return emitted;
      }
      if (only != nullptr && only->count(t.obj_id) == 0) {
        continue;
      }
      if (!pickAllowedNow(transfers, state, t, ws, static_cast<int>(state.inventory.size()))) {
        continue;
      }
      progress = true;
      emitted.push_back(t.obj_id);
      state.picked.insert(t.obj_id);
      if (t.needs_place) {
        state.inventory.push_back(t.obj_id);
      }
    }
  }
  return emitted;
}

std::string chooseNextWs(
  const std::vector<TransferItem> & transfers,
  const std::set<std::string> & candidate_ws,
  const PlannerState & state,
  std::mt19937 & rng)
{
  std::vector<std::pair<std::string, std::pair<int, int>>> scores;
  scores.reserve(candidate_ws.size());

  const int onboard = static_cast<int>(state.inventory.size());
  const bool empty_robot = state.inventory.empty();

  for (const auto & ws : candidate_ws) {
    const int places = countCarriedPlacesAtWs(transfers, state, ws);
    const int picks = countPendingPicksAtWs(transfers, state, ws, onboard - places);

    if (empty_robot) {
      if (picks > 0) {
        scores.push_back({ws, {picks, 0}});
      }
    } else {
      if (places > 0 || picks > 0) {
        scores.push_back({ws, {places, picks}});
      }
    }
  }

  if (scores.empty()) {
    return "";
  }

  const auto best_indices = shuffledBestCandidates(scores, rng);
  return scores[best_indices.front()].first;
}

std::vector<Visit> planGreedy(const std::vector<TransferItem> & transfers, std::mt19937 & rng)
{
  std::vector<Visit> visits;
  PlannerState state;
  const std::set<std::string> candidate_ws = candidateStations(transfers);

  while (hasUnfinishedWork(transfers, state)) {
    const std::string next_ws = chooseNextWs(transfers, candidate_ws, state, rng);
    if (next_ws.empty()) {
      throw std::runtime_error("Task planner got stuck: no reachable pick/place candidate");
    }

    const std::size_t picked_before = state.picked.size();
    const std::size_t placed_before = state.placed.size();
    visits.push_back(visitStation(transfers, state, next_ws, nullptr));
    // Guarda de progresso (revisao 24/08): sem pick nem place novo nesta
    // iteracao o laco giraria para sempre - falha explicita em vez disso.
    if (state.picked.size() == picked_before && state.placed.size() == placed_before) {
      throw std::runtime_error(
              "Task planner got stuck: sem progresso em " + next_ws +
              " (pilha cuja base nao pode ser alcancada, ou ocupante da pilha inicial "
              "que nao cabe a bordo? inventario: " +
              std::to_string(state.inventory.size()) + " objeto(s) a bordo)");
    }
  }
  return visits;
}

// ---- modelo de custo -----------------------------------------------------------

bool CostModel::hasDistance(const std::string & a, const std::string & b) const
{
  if (a == b) {
    return true;
  }
  const auto row = dist_m.find(a);
  if (row != dist_m.end() && row->second.count(b) > 0) {
    return true;
  }
  const auto col = dist_m.find(b);
  return col != dist_m.end() && col->second.count(a) > 0;
}

double CostModel::distance(const std::string & a, const std::string & b) const
{
  if (a == b) {
    return 0.0;
  }
  const auto row = dist_m.find(a);
  if (row != dist_m.end()) {
    const auto it = row->second.find(b);
    if (it != row->second.end()) {
      return it->second;
    }
  }
  const auto col = dist_m.find(b);
  if (col != dist_m.end()) {
    const auto it = col->second.find(a);
    if (it != col->second.end()) {
      return it->second;
    }
  }
  throw std::runtime_error("CostModel: distancia desconhecida entre " + a + " e " + b);
}

double CostModel::navTime(const std::string & a, const std::string & b) const
{
  if (v_nav_mps <= 0.0) {
    throw std::runtime_error("CostModel: v_nav_mps precisa ser > 0");
  }
  return distance(a, b) / v_nav_mps;
}

double estimatePlanTime(
  const std::vector<Visit> & visits, const CostModel & model, PlanStats * stats)
{
  PlanStats local;
  std::string prev = model.start_ws;
  double total = 0.0;
  for (const Visit & v : visits) {
    ++local.visits;
    if (v.ws != prev) {
      const double d = model.distance(prev, v.ws);
      local.distance_m += d;
      local.nav_s += d / model.v_nav_mps;
      local.docking_s += model.docking_s;
      ++local.station_switches;
      prev = v.ws;
    }
    local.picks += static_cast<int>(v.picks.size());
    local.places += static_cast<int>(v.places.size());
  }
  const double d_end = model.distance(prev, model.finish_ws);
  local.distance_m += d_end;
  local.nav_s += d_end / model.v_nav_mps;
  local.manipulation_s = local.picks * model.pick_s + local.places * model.place_s;
  total = local.nav_s + local.docking_s + local.manipulation_s;
  if (stats != nullptr) {
    *stats = local;
  }
  return total;
}

// ---- busca ---------------------------------------------------------------------

namespace
{

struct StateKey
{
  int ws;
  std::uint64_t picked;
  std::uint64_t placed;
  bool operator==(const StateKey & o) const
  {
    return ws == o.ws && picked == o.picked && placed == o.placed;
  }
};

struct StateKeyHash
{
  std::size_t operator()(const StateKey & k) const
  {
    std::size_t h = std::hash<std::uint64_t>()(k.picked);
    h ^= std::hash<std::uint64_t>()(k.placed) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= std::hash<int>()(k.ws) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};

constexpr double kEps = 1e-6;
constexpr int kStartIndex = -1;

class BranchAndBound
{
public:
  BranchAndBound(
    const std::vector<TransferItem> & transfers, const CostModel & model,
    const double bound_cost, const double time_limit_s)
  : transfers_(transfers), model_(model), bound_cost_(bound_cost),
    time_limit_s_(time_limit_s)
  {
    const std::set<std::string> cands = candidateStations(transfers);
    stations_.assign(cands.begin(), cands.end());
    for (std::size_t i = 0; i < transfers_.size(); ++i) {
      bit_by_id_[transfers_[i].obj_id] = i;
    }
    // Indices inteiros: estacoes 0..n-1, START = n, FINISH = n+1; matriz de
    // distancias pre-calculada (a heuristica roda milhares de vezes).
    const int n = static_cast<int>(stations_.size());
    start_idx_ = n;
    finish_idx_ = n + 1;
    std::vector<std::string> names = stations_;
    names.push_back(model_.start_ws);
    names.push_back(model_.finish_ws);
    d_.assign(names.size(), std::vector<double>(names.size(), 0.0));
    for (std::size_t i = 0; i < names.size(); ++i) {
      for (std::size_t j = 0; j < names.size(); ++j) {
        d_[i][j] = model_.distance(names[i], names[j]);
      }
    }
    std::map<std::string, int> index_of;
    for (int i = 0; i < n; ++i) {
      index_of[stations_[static_cast<std::size_t>(i)]] = i;
    }
    for (const auto & t : transfers_) {
      from_idx_.push_back(t.from_ws.empty() ? -1 : index_of.at(t.from_ws));
      to_idx_.push_back(t.to_ws.empty() ? -1 : index_of.at(t.to_ws));
    }
  }

  SearchResult run()
  {
    const auto t0 = std::chrono::steady_clock::now();
    deadline_ = t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(time_limit_s_));
    PlannerState state;
    dfs(state, kStartIndex, 0.0);
    SearchResult result;
    result.visits = best_plan_;
    result.time_s = best_cost_;
    result.optimal = !timed_out_;
    result.improved = have_plan_;
    result.nodes = nodes_;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    return result;
  }

private:
  struct Child
  {
    double f;
    double g;
    Visit visit;
    PlannerState state;
    int ws;
  };

  std::uint64_t maskOf(const std::set<int> & ids) const
  {
    std::uint64_t mask = 0;
    for (const int id : ids) {
      const auto it = bit_by_id_.find(id);
      if (it != bit_by_id_.end()) {
        mask |= std::uint64_t{1} << it->second;
      }
    }
    return mask;
  }

  bool checkDeadline()
  {
    if (timed_out_) {
      return true;
    }
    if ((nodes_ & 63) == 0 && std::chrono::steady_clock::now() > deadline_) {
      timed_out_ = true;
    }
    return timed_out_;
  }

  int matrixIndex(const int ws) const
  {
    return ws == kStartIndex ? start_idx_ : ws;
  }

  /// Arvore geradora minima (Prim) sobre {atual} U pendentes U {FINISH}: toda
  /// rota que ainda visita as pendentes e termina no FINISH e' um passeio que
  /// conecta esses nos, logo mede pelo menos a MST. `nodes` = indices da
  /// matriz, nodes[0] = atual.
  double mstLowerBound(const std::vector<int> & nodes) const
  {
    const std::size_t n = nodes.size();
    std::vector<bool> in_tree(n, false);
    std::vector<double> best(n, std::numeric_limits<double>::infinity());
    best[0] = 0.0;
    double total = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
      std::size_t pick = n;
      for (std::size_t i = 0; i < n; ++i) {
        if (!in_tree[i] && (pick == n || best[i] < best[pick])) {
          pick = i;
        }
      }
      in_tree[pick] = true;
      total += best[pick];
      const std::vector<double> & row = d_[static_cast<std::size_t>(nodes[pick])];
      for (std::size_t i = 0; i < n; ++i) {
        if (!in_tree[i]) {
          best[i] = std::min(best[i], row[static_cast<std::size_t>(nodes[i])]);
        }
      }
    }
    return total;
  }

  /// Filhos de (state, ws): para cada estacao X, soltar tudo o que da e pegar
  /// cada subconjunto permitido. X == ws (sem goto) so quando ha progresso
  /// (pick que liberou um place bloqueado, por exemplo).
  std::vector<Child> expand(const PlannerState & state, const int ws, const double g) const
  {
    std::vector<Child> children;
    for (std::size_t xi = 0; xi < stations_.size(); ++xi) {
      const int x = static_cast<int>(xi);
      const std::string & xname = stations_[xi];
      const double travel = (x == ws) ? 0.0 :
        d_[static_cast<std::size_t>(matrixIndex(ws))][xi] / model_.v_nav_mps + model_.docking_s;

      PlannerState after_places = state;
      const std::vector<int> places = placesAtWs(transfers_, after_places, xname);

      std::vector<int> cand;
      for (const auto & t : transfers_) {
        if (t.needs_pick && after_places.picked.count(t.obj_id) == 0 && t.from_ws == xname) {
          cand.push_back(t.obj_id);
        }
      }
      const int free_slots = kMaxOnboard - static_cast<int>(after_places.inventory.size());
      const std::size_t n_cand = std::min<std::size_t>(cand.size(), 16);
      const std::uint32_t n_masks = std::uint32_t{1} << n_cand;
      for (std::uint32_t mask = 0; mask < n_masks; ++mask) {
        const int wanted = __builtin_popcount(mask);
        if (wanted > free_slots) {
          continue;
        }
        std::set<int> only;
        for (std::size_t i = 0; i < n_cand; ++i) {
          if (mask & (std::uint32_t{1} << i)) {
            only.insert(cand[i]);
          }
        }
        PlannerState next = after_places;
        const std::vector<int> picks = picksAtWs(transfers_, next, xname, &only);
        if (static_cast<int>(picks.size()) != wanted) {
          continue;   // algum pedido nao e' permitido agora (pilha/reserva)
        }
        if (places.empty() && picks.empty()) {
          continue;
        }
        // Ficar na mesma estacao so' vale quando um pick liberou um place
        // (ocupante da pilha inicial): so' picks seria o mesmo que ter pego
        // mais na visita anterior (todos os subconjuntos ja sao enumerados).
        if (x == ws && places.empty()) {
          continue;
        }
        Child child;
        child.ws = x;
        child.g = g + travel + places.size() * model_.place_s + picks.size() * model_.pick_s;
        child.visit.ws = xname;
        child.visit.places = places;
        child.visit.picks = picks;
        child.state = std::move(next);
        child.f = child.g;   // + h, preenchido pelo chamador
        children.push_back(std::move(child));
      }
    }
    return children;
  }

  /// Limite inferior admissivel do que falta: picks/places pendentes, um
  /// docking por estacao com trabalho pendente (a atual tambem, se nada pode
  /// ser feito nela agora: vai sair e voltar) e a MST das distancias.
  double heuristic(const PlannerState & state, const int ws, const bool can_progress_here) const
  {
    std::vector<bool> pending(stations_.size(), false);
    int picks = 0;
    int places = 0;
    for (std::size_t i = 0; i < transfers_.size(); ++i) {
      const TransferItem & t = transfers_[i];
      if (t.needs_pick && state.picked.count(t.obj_id) == 0) {
        ++picks;
        if (from_idx_[i] >= 0) {
          pending[static_cast<std::size_t>(from_idx_[i])] = true;
        }
      }
      if (t.needs_place && state.placed.count(t.obj_id) == 0) {
        ++places;
        if (to_idx_[i] >= 0) {
          pending[static_cast<std::size_t>(to_idx_[i])] = true;
        }
      }
    }
    double h = picks * model_.pick_s + places * model_.place_s;
    std::vector<int> nodes;
    nodes.reserve(stations_.size() + 2);
    nodes.push_back(matrixIndex(ws));
    for (std::size_t i = 0; i < pending.size(); ++i) {
      if (!pending[i]) {
        continue;
      }
      if (static_cast<int>(i) != ws) {
        nodes.push_back(static_cast<int>(i));
        h += model_.docking_s;
      } else if (!can_progress_here) {
        h += model_.docking_s;
      }
    }
    nodes.push_back(finish_idx_);
    h += mstLowerBound(nodes) / model_.v_nav_mps;
    return h;
  }

  bool prunable(const double f) const
  {
    if (have_plan_) {
      return f >= best_cost_ - kEps;
    }
    return f > bound_cost_ + kEps;
  }

  void dfs(const PlannerState & state, const int ws, const double g)
  {
    ++nodes_;
    if (checkDeadline()) {
      return;
    }
    if (!hasUnfinishedWork(transfers_, state)) {
      const double total = g +
        d_[static_cast<std::size_t>(matrixIndex(ws))][static_cast<std::size_t>(finish_idx_)] /
        model_.v_nav_mps;
      if (!have_plan_ || total < best_cost_ - kEps) {
        have_plan_ = true;
        best_cost_ = total;
        best_plan_ = path_;
      }
      return;
    }

    // Limite fraco (assume que da para progredir aqui) antes de expandir.
    if (prunable(g + heuristic(state, ws, true))) {
      return;
    }
    // Tabela de transposicao: o mesmo estado ja foi alcancado mais barato
    // (nada a ganhar), ou ja foi fechado e deixou uma cota inferior do que
    // falta (limite na saida - g de entao, valida porque o limite so' cai).
    const StateKey key{ws, maskOf(state.picked), maskOf(state.placed)};
    const auto seen = memo_.find(key);
    if (seen != memo_.end()) {
      if (seen->second.best_g <= g + kEps) {
        return;
      }
      if (prunable(g + seen->second.remaining_lb)) {
        return;
      }
      seen->second.best_g = g;
    } else {
      memo_[key] = MemoEntry{g, 0.0};
    }

    std::vector<Child> children = expand(state, ws, g);
    bool can_progress_here = false;
    for (const auto & c : children) {
      if (c.ws == ws) {
        can_progress_here = true;
        break;
      }
    }
    // Limite forte: sem progresso possivel aqui, voltar custa um docking.
    if (!can_progress_here && prunable(g + heuristic(state, ws, false))) {
      return;
    }
    for (auto & c : children) {
      // Para o filho, "pode progredir onde esta" so se sabe expandindo:
      // assume que sim (limite mais fraco, continua admissivel).
      c.f = c.g + heuristic(c.state, c.ws, true);
    }
    std::stable_sort(
      children.begin(), children.end(),
      [](const Child & a, const Child & b) {return a.f < b.f;});
    for (auto & c : children) {
      if (timed_out_) {
        return;
      }
      if (prunable(c.f)) {
        continue;
      }
      path_.push_back(c.visit);
      dfs(c.state, c.ws, c.g);
      path_.pop_back();
    }
    if (!timed_out_) {
      // Subarvore fechada: toda conclusao a partir daqui custa >= limite atual
      // (as podadas custavam >= o limite de entao, que era >= o de agora).
      const double limit = have_plan_ ? best_cost_ : bound_cost_;
      auto & entry = memo_[key];
      entry.remaining_lb = std::max(entry.remaining_lb, limit - g);
    }
  }

  const std::vector<TransferItem> & transfers_;
  const CostModel & model_;
  const double bound_cost_;
  const double time_limit_s_;
  std::vector<std::string> stations_;
  std::unordered_map<int, std::size_t> bit_by_id_;
  std::vector<std::vector<double>> d_;   // distancias por indice (estacoes, START, FINISH)
  std::vector<int> from_idx_;            // por transfer: indice da origem (-1 = sem)
  std::vector<int> to_idx_;              // por transfer: indice do destino
  int start_idx_{0};
  int finish_idx_{0};
  struct MemoEntry
  {
    double best_g;         // menor custo com que o estado foi alcancado
    double remaining_lb;   // cota inferior do que falta (0 = ainda nao fechado)
  };
  std::unordered_map<StateKey, MemoEntry, StateKeyHash> memo_;
  std::vector<Visit> path_;
  std::vector<Visit> best_plan_;
  double best_cost_{std::numeric_limits<double>::infinity()};
  bool have_plan_{false};
  bool timed_out_{false};
  std::size_t nodes_{0};
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace

SearchResult planOptimized(
  const std::vector<TransferItem> & transfers, const CostModel & model,
  const std::vector<Visit> & initial, const double time_limit_s)
{
  SearchResult fallback;
  fallback.visits = initial;
  fallback.time_s = estimatePlanTime(initial, model);
  if (transfers.size() > 64) {
    // Mascaras de 64 bits: acima disso fica o plano inicial (nunca acontece
    // numa task real, que tem ate ~10 objetos).
    return fallback;
  }
  if (time_limit_s <= 0.0) {
    return fallback;
  }
  // Toda distancia usada pela busca precisa existir - falha cedo, com o par.
  std::set<std::string> names = candidateStations(transfers);
  names.insert(model.start_ws);
  names.insert(model.finish_ws);
  for (const auto & a : names) {
    for (const auto & b : names) {
      (void)model.distance(a, b);
    }
  }

  BranchAndBound search(transfers, model, fallback.time_s, time_limit_s);
  SearchResult result = search.run();
  if (!result.improved) {
    // Nada fechado dentro do prazo: fica o plano inicial.
    result.visits = initial;
    result.time_s = fallback.time_s;
    return result;
  }
  // Visitas consecutivas a mesma estacao ficam separadas de proposito: a
  // segunda existe porque um pick da primeira liberou um place, e a emissao
  // (places antes dos picks em cada visita) depende dessa ordem. O goto nao
  // se repete (appendGoto).
  result.improved = result.time_s < fallback.time_s - kEps;
  return result;
}

// ---- emissao -------------------------------------------------------------------

std::string resolveWsKey(
  const std::map<std::string, std::string> & table,
  const std::string & ws)
{
  if (table.count(ws) > 0) {
    return ws;
  }
  std::vector<std::string> candidates;
  for (const auto & [key, _] : table) {
    if (key.size() > ws.size() && key.compare(0, ws.size(), ws) == 0) {
      const char next = key[ws.size()];
      if (next == '_' || std::isdigit(static_cast<unsigned char>(next))) {
        candidates.push_back(key);
      }
    }
  }
  if (candidates.size() == 1) {
    return candidates.front();
  }
  if (candidates.empty()) {
    throw std::runtime_error(
            "Missing WS->Mesa mapping for workspace: " + ws + " - estacao sem entrada em "
            "ws_to_table_pose do ws_table_mapping.yaml do mapa (nem candidato " + ws +
            "_<n>): acrescente '" + ws + "_1: Mesa15' em ws_to_table_pose e '" + ws + "_1: " +
            ws + "1' em ws_to_dock_id (e grave a pose do dock " + ws + "1)");
  }
  std::string joined;
  for (const auto & c : candidates) {
    joined += (joined.empty() ? "" : ", ") + c;
  }
  throw std::runtime_error(
          "Estacao '" + ws + "' ambigua no ws_to_table_pose (candidatos: " + joined +
          ") - use o nome completo na task ou acrescente a chave '" + ws +
          "' no ws_table_mapping.yaml");
}

std::string tablePoseForWs(
  const std::map<std::string, std::string> & table,
  const std::string & ws)
{
  return table.at(resolveWsKey(table, ws));
}

void appendGoto(
  YAML::Node & action_seq,
  std::string & current_ws,
  const std::string & ws,
  const std::map<std::string, std::string> & ws_to_table_pose)
{
  const std::string table_pose = tablePoseForWs(ws_to_table_pose, ws);
  // Comparar/registrar SEMPRE pela estacao (ws), nunca pela mesa: WS_1 e
  // WS_2 podem ser a mesma Mesa e o goto seria pulado indevidamente.
  if (current_ws == ws) {
    return;
  }

  if (!current_ws.empty()) {
    YAML::Node home;
    home["kind"] = "home";
    home["pose_name"] = "home";
    action_seq.push_back(home);
  }

  YAML::Node nav;
  nav["kind"] = "goto";
  nav["ws"] = ws;
  nav["mesa"] = table_pose;
  action_seq.push_back(nav);
  current_ws = ws;
}

YAML::Node emitActions(
  const std::vector<Visit> & visits,
  const std::vector<TransferItem> & transfers,
  const std::map<std::string, std::string> & ws_to_table_pose)
{
  YAML::Node action_seq(YAML::NodeType::Sequence);
  std::string current_ws;
  for (const Visit & v : visits) {
    appendGoto(action_seq, current_ws, v.ws, ws_to_table_pose);
    for (const int obj_id : v.places) {
      const TransferItem & t = findTransferById(transfers, obj_id);
      YAML::Node place;
      place["kind"] = "place";
      place["tag_frame"] = t.tag_frame;
      place["ws"] = t.to_ws;
      place["table_pose"] = tablePoseForWs(ws_to_table_pose, t.to_ws);
      if (!t.stack_on_frame.empty()) {
        place["stack_on"] = t.stack_on_frame;
      }
      if (!t.container_color.empty()) {
        place["container_color"] = t.container_color;
      }
      if (!t.at_tag_frame.empty()) {
        place["at_tag"] = t.at_tag_frame;     // slot da mesa de precisao (2026)
      }
      if (!t.to_level.empty()) {
        place["shelf_level"] = t.to_level;    // informativo (2026)
      }
      action_seq.push_back(place);
    }
    for (const int obj_id : v.picks) {
      const TransferItem & t = findTransferById(transfers, obj_id);
      YAML::Node pick;
      pick["kind"] = "pick";
      pick["tag_frame"] = t.tag_frame;
      pick["ws"] = t.from_ws;
      // O pick tambem precisa saber o tipo de mesa: a prateleira (MesaSh)
      // exige uma sequencia de movimentos propria.
      pick["table_pose"] = tablePoseForWs(ws_to_table_pose, t.from_ws);
      if (!t.from_level.empty()) {
        pick["shelf_level"] = t.from_level;   // informativo (2026)
      }
      action_seq.push_back(pick);
    }
  }
  return action_seq;
}

}  // namespace task_plan
}  // namespace manip_bt
