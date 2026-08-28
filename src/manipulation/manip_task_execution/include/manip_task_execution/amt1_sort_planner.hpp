// Planejador PURO da rotina AMT1 (Advanced Manipulation Task 1, 2026-08-28):
// ordenar os cubos com AprilTag da mesa de precision placement em ordem
// crescente de id, da esquerda para a direita do robo, usando os containers
// de bordo como buffer.
//
// Sem ROS, sem IK: so aritmetica de permutacoes. O no amt1_sort_action_node
// observa a mesa, chama assignSlots (slots por x), planSort (ciclos) e
// executa os picks/places; simulatePlan e o oraculo dos testes (e uma
// conferencia barata antes de mover o braco); registrationDelta corrige os
// slots depois de a base andar de lado (nudge).
//
// Frames (convencao do braco): manip_base_link +X = DIREITA do robo, +Y =
// frente. "Esquerda para a direita" = x crescente.
//
// Algoritmo do operador (ordenacao por ciclos): ordem 5,2,3,1,4,6 ->
// pega 5 (guarda a pose A), pega 1 (libera D), solta 1 em A, pega 4
// (libera E), solta 4 em D, solta 5 em E. Nunca mais de 2 cubos a bordo;
// todo PLACE cai num slot ja livre; cubos certos nao sao tocados.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace manip_task_execution
{
namespace amt1
{

/// Tag vista na mesa: posicao no frame da base do braco (para ordenar).
struct TagPose
{
  int id{0};
  double x{0.0};    ///< manip_base_link, + = direita
  double y{0.0};    ///< manip_base_link, + = frente
  double z{0.0};
  double yaw{0.0};  ///< yaw do eixo X da tag projetado no plano XY (rad)
};

/// Resultado de assignSlots. `slots[k]` e a tag que ocupa o slot k no
/// inicio (slot 0 = o primeiro na direcao pedida); observed_order[k] =
/// slots[k].id; target_order = ids ordenados crescentes (mesmo tamanho).
struct SlotAssignment
{
  std::vector<TagPose> slots;
  std::vector<int> observed_order;
  std::vector<int> target_order;
  std::vector<int> missing_tags;   ///< esperadas e nao vistas
  std::vector<int> ignored_tags;   ///< vistas e nao esperadas (nao ganham slot)
};

/// Slots por x crescente (left_to_right) ou decrescente (right_to_left).
/// Empate |dx| < tie_eps: menor y primeiro. So as tags esperadas ganham
/// slot (expected vazio = todas as vistas). Ids repetidos em `seen`: fica a
/// primeira ocorrencia. target_order = ids vistos e esperados em ordem
/// crescente — ordena parcial quando falta tag.
SlotAssignment assignSlots(
  const std::vector<TagPose> & seen,
  const std::vector<int> & expected,
  bool left_to_right = true,
  double tie_eps = 0.01);

enum class SortOpType
{
  kPick,
  kPlace,
};

/// Uma operacao do plano. PICK: `slot` = slot de origem (onde a tag esta).
/// PLACE: `slot` = slot de destino (ja livre quando a op executa).
struct SortOp
{
  SortOpType type{SortOpType::kPick};
  int tag{0};
  int slot{0};
};

struct SortPlan
{
  std::vector<SortOp> ops;
  std::string error;          ///< "" = ok; buffer_insuficiente | alvo_invalido
  int max_onboard{0};         ///< pico de cubos a bordo (0 se ja ordenada, senao 2)
  int picks{0};
  int places{0};
  std::vector<int> final_order;  ///< = target quando ok
};

/// Plano por ciclos: dest[k] = posicao alvo de observed[k]; cada ciclo
/// comeca no seu slot mais a esquerda (menor indice) [c0..c(m-1)]:
/// PICK c0; para j = m-1..1: PICK cj, PLACE cj -> slot livre (dest[cj]);
/// PLACE c0 -> ultimo livre (dest[c0]). Precisa de buffer >= 2
/// (error "buffer_insuficiente"); `target` tem que ser permutacao de
/// `observed` (error "alvo_invalido"). Sem cubo fora do lugar => ops vazio.
SortPlan planSort(
  const std::vector<int> & observed,
  const std::vector<int> & target,
  int buffer);

struct SimulationResult
{
  bool ok{false};
  std::string error;
  std::vector<int> final_order;   ///< ids por slot ao fim (0 = slot vazio)
  std::vector<int> onboard;       ///< cubos que sobraram a bordo (vazio se ok)
  int max_onboard{0};
  int picks{0};
  int places{0};
};

/// Oraculo: executa `ops` numa mesa que comeca em `observed` (todos os slots
/// ocupados) e confere os invariantes: PICK so de slot ocupado pela tag
/// dita, PLACE so em slot livre com a tag a bordo, nunca mais de `buffer`
/// a bordo, e no fim ninguem a bordo. Nao exige que o resultado esteja
/// ordenado — quem chama compara final_order com o alvo.
SimulationResult simulatePlan(
  const std::vector<int> & observed,
  const std::vector<SortOp> & ops,
  int buffer);

/// Diferenca (frame odom, m) entre onde uma tag que continua na mesa foi
/// vista AGORA e onde o slot dela foi registrado: nova - registrada.
struct AnchorDelta
{
  int id{0};
  double dx{0.0};
  double dy{0.0};
};

struct RegistrationResult
{
  bool ok{false};
  double dx{0.0};        ///< delta medio a aplicar a todos os slots
  double dy{0.0};
  double spread{0.0};    ///< maior distancia de uma ancora ao delta medio
  std::size_t used{0};
  std::string reason;    ///< "" | poucas_ancoras | espalhamento
};

/// Delta medio das ancoras. ok so com >= min_anchors ancoras e espalhamento
/// (maior desvio ao delta medio) <= max_spread_m — acima disso alguma tag
/// foi vista errada e e mais seguro manter a odometria.
RegistrationResult registrationDelta(
  const std::vector<AnchorDelta> & anchors,
  std::size_t min_anchors = 2,
  double max_spread_m = 0.02);

/// Texto curto de uma op para log/feedback ("pick_5" / "place_1_slot_0").
std::string describeOp(const SortOp & op);

}  // namespace amt1
}  // namespace manip_task_execution
