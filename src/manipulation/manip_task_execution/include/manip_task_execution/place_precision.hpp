// Helpers PUROS do place de precisao (2026-10-21): encaixe do cubo num slot
// da mesa de precisao (tags 101..107 coladas no FUNDO de cada cavidade) e
// pilha sobre a tag de referencia da zona de empilhamento (tag 200 colada no
// tampo) ou sobre outro cubo. Tudo aqui e' geometria/decisao sem ROS, para
// o mtc_place_action_node reusar e o gtest cobrir (test_place_precision).
//
// FRAME: posicoes em manip_base_link (+X = direita do robo, +Y = frente,
// +Z = cima); yaw em rad no plano XY. Alturas: a tag de um cubo fica na
// face de CIMA dele; a tag de um slot fica no FUNDO da cavidade.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace manip_task_execution
{

/// O que PlaceTag.goal.stack_on aponta.
enum class StackTargetKind
{
  kCube,       ///< tag de um cubo ja na mesa (empilhar em cima, comportamento classico)
  kSlot,       ///< tag no fundo de uma cavidade da mesa de precisao (encaixar)
  kReference,  ///< tag colada no tampo da zona de empilhamento (1o cubo da pilha)
};

/// Numero n de um frame 'tag_<n>'; nullopt se o frame nao tem esse formato
/// (vazio, 'ct_vermelho', 'tag_x', 'tag_'...).
std::optional<int> tagIdFromFrame(const std::string & frame);

/// Classifica o frame pelo id: em slot_ids -> kSlot; em reference_ids ->
/// kReference; qualquer outro (inclusive frame que nao casa 'tag_<n>') ->
/// kCube, que e' o comportamento de sempre.
StackTargetKind classifyStackTarget(
  const std::string & frame,
  const std::vector<std::int64_t> & slot_ids,
  const std::vector<std::int64_t> & reference_ids);

/// Giro (rad) em [-pi/4, +pi/4] que leva `yaw_current` ao multiplo de 90
/// graus de `yaw_tag` mais proximo: yaw_current + delta == yaw_tag (mod
/// pi/2). Um cubo tem simetria de 90 graus, entao qualquer multiplo alinha
/// as faces - o escolhido e' o de MENOR giro do punho. Empate exato em
/// +-45 graus devolve um dos dois lados (|delta| <= pi/4 sempre).
double nearestQuarterTurn(double yaw_tag, double yaw_current);

/// Deslocamento no plano (m) de uma posicao da busca em espiral.
struct PlanarOffset
{
  double dx{0.0};
  double dy{0.0};
  double radius() const;
};

/// Posicoes da busca em espiral quadrada: a grade (i*step, j*step) com raio
/// <= max_radius, ordenada por raio crescente e, no mesmo raio, pelo angulo
/// a partir de +X no sentido anti-horario: (0,0), (+s,0), (0,+s), (-s,0),
/// (0,-s), (+s,+s), (-s,+s), ... Sem repeticao. step <= 0 ou max_radius < 0
/// devolve so (0,0).
std::vector<PlanarOffset> spiralOffsets(double step_m, double max_radius_m);

/// Onde o cubo ficou em relacao ao slot, pela altura da tag dele.
enum class SeatState
{
  kSeated,        ///< dentro da cavidade: topo ~ fundo + altura do cubo
  kUnseated,      ///< apoiado por cima (ou meio dentro): topo mais alto que isso
  kInconsistent,  ///< topo abaixo do fundo da cavidade + cubo: leitura ruim
};

/// z esperado da tag do cubo ENCAIXADO: fundo da cavidade + altura do cubo.
double seatedCubeTagZ(double z_slot_tag, double cube_height_m);

/// Compara a tag do cubo (depois de soltar) com a tag do slot (fundo):
///   seated       se z_cube_tag <= z_slot_tag + cube_h + tol
///                   e z_cube_tag >= z_slot_tag + cube_h - cavity_depth;
///   inconsistent se ficou abaixo disso (cubo nao pode estar abaixo do fundo);
///   unseated     se ficou acima (apoiado na borda: ~cavity_depth mais alto).
SeatState slotSeatState(
  double z_cube_tag, double z_slot_tag, double cube_height_m,
  double cavity_depth_m, double tol_m);

/// true so' quando slotSeatState == kSeated.
bool isSeatedInSlot(
  double z_cube_tag, double z_slot_tag, double cube_height_m,
  double cavity_depth_m, double tol_m);

/// z da tag de um cubo APOIADO POR CIMA da cavidade (na borda): fundo +
/// cavidade + altura do cubo. E' tambem a cota do TCP para re-pega-lo
/// (TCP no plano da tag): os dedos chegam ao tampo, nunca abaixo.
double slotRimCubeTagZ(double z_slot_tag, double cube_height_m, double cavity_depth_m);

/// z do TCP para re-pegar um cubo apoiado na borda: o que a camera viu,
/// mas NUNCA abaixo da borda (leitura baixa mandaria os dedos para dentro
/// do tampo). max(z_cube_seen, z_rim).
double slotRegraspTcpZ(double z_cube_seen, double z_rim);

/// Compensacao em z da soltura com a ferramenta inclinada `tilt_rad` a
/// partir da vertical: com o TCP no centro da face de cima, a quina mais
/// baixa de um cubo (largura w, altura h) fica h*cos(t) + (w/2)*sin(t)
/// abaixo do TCP, isto e' (w/2)*sin(t) - h*(1 - cos(t)) abaixo de onde
/// ficaria na vertical (4,0 mm a 15 graus e 4,9 mm a 30 para 42 mm). Somar
/// ao z do TCP mantem a folga nominal ate' o tampo/marca. 0 em t = 0;
/// nunca negativo.
double tiltDropZCompensation(double tilt_rad, double cube_width_m, double cube_height_m);

/// O que a camera viu do cubo depois de soltar no SLOT.
struct SlotObservation
{
  bool seen{false};        ///< tag do cubo lida (mediana) apos soltar
  double z_cube_tag{0.0};  ///< z da tag do cubo (so vale se seen)
  double err_xy_m{0.0};    ///< |xy_cubo - xy_slot| (so vale se seen; diagnostico)
};

/// Geometria do slot e orcamento (solturas e leituras) da decisao.
struct SlotLimits
{
  double z_slot_tag{0.0};
  double cube_height_m{0.042};
  double cavity_depth_m{0.041};
  double seat_tol_m{0.008};
  int attempts_done{1};       ///< solturas ja feitas (>= 1)
  int max_attempts{1};        ///< solturas permitidas no total
  int observations_done{1};   ///< leituras ja feitas desta soltura (>= 1)
  int max_observations{2};    ///< leituras permitidas por soltura
};

/// O que fazer com o cubo solto no slot.
enum class SlotAction
{
  kAccept,     ///< encaixado (z assentado); xy NAO decide
  kRegrasp,    ///< apoiado na BORDA (z >= borda - tol): re-pegar e soltar de novo
  kReobserve,  ///< nao visto / z intermediario / inconsistente: olhar de novo
  kLeave,      ///< sem como agir com seguranca ou sem orcamento: fica onde esta
};

/// Decisao do slot (pura). Regras:
///   nao visto            -> kReobserve se ha' leituras; senao kLeave
///                           (quem chama responde 'slot_nao_verificado':
///                           nunca sucesso sem ver, nunca re-pega as cegas);
///   assentado (kSeated)  -> kAccept, qualquer xy (duas leituras de poses
///                           diferentes do braco divergem 1-2 cm por FK; um
///                           cubo com z assentado nao pode estar fora da
///                           cavidade);
///   na borda             -> z >= fundo + cavidade + cubo - tol: kRegrasp se
///                           ha' solturas; senao kLeave;
///   intermediario        -> entre assentado e borda (meio dentro, tombado) ou
///   / inconsistente         abaixo do fundo: kReobserve se ha' leituras,
///                           senao kLeave. NUNCA re-pega: os dedos iriam
///                           abaixo do tampo.
SlotAction decideSlotAction(const SlotObservation & obs, const SlotLimits & lim);

const char * slotActionName(SlotAction a);

/// z do TOPO da pilha (tag do cubo mais alto) com `cubes_above` cubos
/// empilhados sobre uma superficie cuja tag/marca esta em z_base_tag:
/// z_base_tag + cubes_above * cube_h. (Base = tag 200 no tampo: cubes_above
/// = numero de cubos da pilha. Base = tag de um cubo: cubes_above = cubos
/// ACIMA dele.) cubes_above < 0 vale 0.
double expectedPileTopZ(double z_base_tag, int cubes_above, double cube_height_m);

/// Registro da memoria de slots relevante para a pilha (subconjunto do
/// TableSlotRecord do container_state_store, ja com x no frame atual).
struct StackMemoryEntry
{
  std::string slot;       ///< "stack:<frame base>" para cubos empilhados
  std::string tag_frame;  ///< tag do cubo que foi solto ali
  double x{0.0};
  double y{0.0};
  double z{0.0};          ///< z do TOPO do cubo (onde a tag dele esta)
};

/// Ultimo registro 'stack:' cujo tag_frame == frame (o cubo `frame` foi
/// solto por nos em cima de algo). nullopt se nao ha.
std::optional<StackMemoryEntry> findStackMemory(
  const std::vector<StackMemoryEntry> & entries, const std::string & frame);

/// Quantos cubos ha' na cadeia que termina em `base_frame`, seguindo os
/// registros 'stack:<x>' -> registro de x -> ... ate' um frame sem registro
/// (a tag de referencia no tampo, ou um cubo que nao fomos nos que pusemos).
/// Ex.: A sobre tag_200, B sobre A: level(B) = 2, level(A) = 1,
/// level(tag_200) = 0, level(cubo desconhecido) = 0. Ciclos param.
int stackLevelFromMemory(
  const std::vector<StackMemoryEntry> & entries, const std::string & base_frame);

/// Piso do TCP na re-pega de um cubo da PILHA: topo esperado - um cubo - tol
/// (o cubo pode ter caido para o nivel de baixo / a mesa ao lado de uma base
/// de nivel 1); nunca mais fundo que isso.
double pileRegraspFloorZ(double z_expected_top, double cube_height_m, double z_tol_m);

/// Como corrigir um cubo da pilha visto fora do lugar.
enum class PileFix
{
  kCorrectXy,       ///< sobre a base (|dz| <= tol), deslocado: soltar com a correcao xy
  kRedropOriginal,  ///< saiu da base (tombado/caiu um nivel): soltar de novo no alvo original
  kLeave,           ///< abaixo do piso de re-pega: nao descer ate' la
};

/// Decide pela altura vista: abaixo do piso -> kLeave; |dz| <= tol ->
/// kCorrectXy; resto -> kRedropOriginal (o xy de um cubo tombado/caido nao
/// e' erro sistematico, e' onde ele parou).
PileFix pileFixFor(
  double z_cube_tag, double z_expected_top, double cube_height_m, double z_tol_m);

const char * pileFixName(PileFix f);

/// O que a camera viu do cubo depois de soltar, ja comparado ao esperado.
struct PlacementObservation
{
  bool seen{false};        ///< a tag do cubo foi lida (mediana) apos soltar
  double err_xy_m{0.0};    ///< |xy_cubo - xy_esperado| (so vale se seen)
  double err_z_m{0.0};     ///< z_cubo - z_esperado, com sinal (so vale se seen)
  /// Erro XY do TCP por FK na soltura, proxy quando a tag nao foi vista;
  /// < 0 = sem estado / nao usar.
  double fk_err_xy_m{-1.0};
};

/// Tolerancias e orcamento de tentativas da verificacao.
struct PlacementLimits
{
  double xy_tol_m{0.008};
  double z_tol_m{0.008};
  /// Tag nao vista: erro FK <= isto aceita 'nao verificado'; < 0 = aceita
  /// nao verificado sempre (pilha: re-pegar as cegas e' pior que deixar).
  double fk_proxy_tol_m{-1.0};
  int attempts_done{1};   ///< solturas ja feitas (>= 1)
  int max_attempts{1};    ///< solturas permitidas no total
  /// Tag nao vista e proxy FK fora: tentar re-pegar no ponto da soltura?
  bool retry_unseen{false};
};

enum class PlacementDecision
{
  kAccept,            ///< vista e dentro das tolerancias
  kAcceptUnverified,  ///< nao vista; aceita com aviso (proxy FK ok ou sem proxy)
  kRetry,             ///< fora: re-pegar e soltar de novo (ha' orcamento)
  kGiveUp,            ///< fora e sem orcamento (ou nao vista sem como tentar)
};

/// Decisao da PILHA / tag de referencia (1 correcao). O slot usa
/// decideSlotAction. Regras na implementacao; casos no gtest.
PlacementDecision decidePlacement(
  const PlacementObservation & obs, const PlacementLimits & lim);

const char * placementDecisionName(PlacementDecision d);

}  // namespace manip_task_execution
