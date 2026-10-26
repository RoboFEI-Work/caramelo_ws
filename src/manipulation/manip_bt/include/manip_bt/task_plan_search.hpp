#pragma once

// task_plan_search: nucleo do task_planner que decide a ORDEM das estacoes.
//
// O task_planner.cpp traduz o YAML da competicao em TransferItem (o que pegar
// onde e soltar onde, com as regras de pilha/container/slot). Este modulo
// recebe essa lista e devolve a sequencia de visitas (estacao + places +
// picks), de dois jeitos:
//   planGreedy     guloso historico (mais places/picks por visita, empates
//                  sorteados) - comportamento EXATO de antes, usado quando o
//                  mapa nao tem dock_distances.yaml;
//   planOptimized  branch-and-bound sobre o estado do mundo (estacao atual,
//                  pegos, soltos) minimizando o tempo estimado pelo CostModel
//                  (distancia/v_nav + docking por troca de estacao + pick +
//                  place), com o guloso como limite inicial e prazo de busca.
// As regras de precedencia (capacidade 3, pilha inicial, topo so com a base
// disponivel, reserva de lugar a bordo) sao as mesmas nos dois caminhos:
// placesAtWs/picksAtWs sao compartilhadas. Separado do main para o gtest
// (test/test_task_plan_search.cpp). Ver docs/planejador_por_tempo.md.

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace manip_bt
{
namespace task_plan
{

struct TransferItem
{
  int obj_id = -1;
  std::string tag_frame;
  std::string from_ws;
  std::string to_ws;
  // Cor do container da mesa onde soltar (RED|BLUE); vazia = mesa. Vem da
  // constraint "inside" (formato antigo) ou da cor do proprio
  // objeto (formato novo: se houver container dessa cor visivel, o place usa).
  std::string container_color;
  bool needs_pick = false;
  bool needs_place = false;
  // Empilhar: este objeto e' solto EM CIMA do objeto `stack_on_obj`
  // (frame `stack_on_frame`), que precisa estar na mesa de `to_ws` antes.
  // Formato 2026: a BASE da pilha e' solta sobre a tag de referencia da area
  // de empilhamento (stack_on_frame = tag_200, stack_on_obj = -1: a tag esta
  // sempre na mesa, nada a esperar).
  int stack_on_obj = -1;
  std::string stack_on_frame;
  // Formato 2026 - prateleira: nivel (BOTTOM|TOP) de origem/destino, so
  // informativo nas acoes (shelf_level); vazio fora da prateleira.
  std::string from_level;
  std::string to_level;
  // Formato 2026 - mesa de precisao como DESTINO: tag do slot onde soltar
  // (place com at_tag: tag_<slot>). -1 = nao e' slot.
  int to_slot_tag = -1;
  int from_slot_tag = -1;
  std::string at_tag_frame;
  // Formato 2026 - pilha INICIAL (start_state.<ST>.stack): ids que estao em
  // cima deste objeto e precisam ser pegos antes dele.
  std::vector<int> pick_after;
  // Formato 2026 - pilha INICIAL ocupando o lugar onde este objeto sera
  // solto (a tag de referencia, se e' base; o que esta em cima da base, se
  // e' topo): esses ids precisam ser pegos ANTES do place deste objeto.
  std::vector<int> place_after_picked;
};

constexpr int kMaxOnboard = 3;

struct PlannerState
{
  std::set<int> picked;
  std::set<int> placed;
  std::vector<int> inventory;
};

// Uma visita a estacao: o que e' solto e o que e' pego, na ordem de emissao.
struct Visit
{
  std::string ws;
  std::vector<int> places;
  std::vector<int> picks;
};

// ---- regras compartilhadas (guloso e busca) ---------------------------------
const TransferItem & findTransferById(const std::vector<TransferItem> & transfers, int obj_id);
bool hasUnfinishedWork(const std::vector<TransferItem> & transfers, const PlannerState & state);
// Estacoes com algum pick ou place pendente (from_ws/to_ws dos transfers).
std::set<std::string> candidateStations(const std::vector<TransferItem> & transfers);
// Solta em `ws` tudo o que esta a bordo e pode ser solto (bases antes dos
// topos; place bloqueado por ocupante da pilha inicial fica a bordo).
// Atualiza state; devolve os ids na ordem de emissao.
std::vector<int> placesAtWs(
  const std::vector<TransferItem> & transfers, PlannerState & state, const std::string & ws);
// Pega em `ws` tudo o que pickAllowedNow permite (capacidade, pilha inicial,
// reserva de lugar), na ordem dos transfers. `only` != nullptr restringe a
// esses ids (busca: subconjuntos). Atualiza state; devolve os ids pegos.
std::vector<int> picksAtWs(
  const std::vector<TransferItem> & transfers, PlannerState & state, const std::string & ws,
  const std::set<int> * only = nullptr);
// Guloso: proxima estacao = mais (places, picks) com robo carregado, mais
// picks com robo vazio; empate sorteado em `rng`. Vazia se nada a fazer.
std::string chooseNextWs(
  const std::vector<TransferItem> & transfers, const std::set<std::string> & candidate_ws,
  const PlannerState & state, std::mt19937 & rng);

// Plano guloso historico. Lanca std::runtime_error se travar (sem progresso).
std::vector<Visit> planGreedy(const std::vector<TransferItem> & transfers, std::mt19937 & rng);

// ---- modelo de custo ---------------------------------------------------------
// tempo = soma, por troca de estacao, de dist/v_nav_mps + docking_s (home +
// undock + aproximacao + docking/align) + pick_s por pick + place_s por place,
// mais o trecho final ate o FINISH. Defaults da run de 2026-10-21
// (docs/analise_run_2026-10-21.md). Chaves de dist_m sao os nomes de estacao
// DA TASK (WS_1, PP, ST...) mais start_ws/finish_ws; a matriz e' simetrica.
struct CostModel
{
  double v_nav_mps{0.25};
  double docking_s{25.0};
  double pick_s{35.0};
  double place_s{35.0};
  std::string start_ws{"START"};
  std::string finish_ws{"FINISH"};
  std::map<std::string, std::map<std::string, double>> dist_m;

  bool hasDistance(const std::string & a, const std::string & b) const;
  // Lanca std::runtime_error se o par nao estiver na matriz (a == b -> 0).
  double distance(const std::string & a, const std::string & b) const;
  double navTime(const std::string & a, const std::string & b) const;
};

struct PlanStats
{
  int visits{0};
  int station_switches{0};   // gotos emitidos (visitas consecutivas na mesma estacao nao contam)
  int picks{0};
  int places{0};
  double distance_m{0.0};
  double nav_s{0.0};
  double docking_s{0.0};
  double manipulation_s{0.0};
};

// Tempo estimado do plano (START -> visitas -> FINISH). Lanca se faltar distancia.
double estimatePlanTime(
  const std::vector<Visit> & visits, const CostModel & model, PlanStats * stats = nullptr);

// ---- busca -------------------------------------------------------------------
struct SearchResult
{
  std::vector<Visit> visits;
  double time_s{0.0};
  bool optimal{false};      // busca terminou (nao estourou o prazo)
  bool improved{false};     // melhor que o plano inicial (guloso)
  std::size_t nodes{0};     // nos expandidos
  double elapsed_ms{0.0};
};

// Branch-and-bound em profundidade: estado = (estacao atual, pegos, soltos);
// filho = ir a uma estacao, soltar tudo o que da (placesAtWs) e pegar um
// subconjunto permitido (picksAtWs com `only`). Limite inferior admissivel:
// picks/places pendentes + docking por estacao pendente + MST das distancias
// (estacao atual, pendentes, FINISH) / v_nav. `initial` (normalmente o
// guloso) da o limite superior e e' devolvido se nada melhor for achado ou se
// a busca estourar `time_limit_s` antes de fechar um plano.
SearchResult planOptimized(
  const std::vector<TransferItem> & transfers, const CostModel & model,
  const std::vector<Visit> & initial, double time_limit_s);

// ---- emissao -----------------------------------------------------------------
// Chave do ws_table_mapping para a estacao da task: a exata quando existe;
// senao o UNICO candidato "<ws>_<n>" / "<ws><n>" (formato 2026 nomeia PP e
// ST sem sufixo; os mapas tem PP_1 / ST_1). 0 ou >1 candidatos = erro
// dizendo o que acrescentar no ws_table_mapping.yaml.
std::string resolveWsKey(const std::map<std::string, std::string> & table, const std::string & ws);
std::string tablePoseForWs(
  const std::map<std::string, std::string> & table,
  const std::string & ws);
// goto <ws> (com `home` antes se ja houve estacao) - tambem usado pela AMT1.
void appendGoto(
  YAML::Node & action_seq, std::string & current_ws, const std::string & ws,
  const std::map<std::string, std::string> & ws_to_table_pose);
// Lista de acoes goto/home/pick/place do bt_yaml_executor a partir das visitas.
YAML::Node emitActions(
  const std::vector<Visit> & visits, const std::vector<TransferItem> & transfers,
  const std::map<std::string, std::string> & ws_to_table_pose);

}  // namespace task_plan
}  // namespace manip_bt
