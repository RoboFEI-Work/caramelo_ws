// Helpers PUROS do empilhamento (2026-10-21): pilha sobre a tag de referencia
// da zona de empilhamento (tag 200 colada no tampo) ou sobre outro cubo. Tudo
// aqui e' geometria/decisao sem ROS, para o mtc_place_action_node reusar e o
// gtest cobrir (test_place_precision).
// 2026-11-16 (branch mudanca_pp, decisao do operador): tudo o que era da mesa
// de precisao (slot, cavidade, furo) foi removido daqui.
//
// FRAME: posicoes em manip_base_link (+X = direita do robo, +Y = frente,
// +Z = cima); yaw em rad no plano XY. Alturas: a tag de um cubo fica na
// face de CIMA dele.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace manip_task_execution
{

/// O que PlaceTag.goal.stack_on aponta.
enum class StackTargetKind
{
  kCube,       ///< tag de um cubo ja na mesa (empilhar em cima, comportamento classico)
  kReference,  ///< tag colada no tampo da zona de empilhamento (1o cubo da pilha)
};

/// Numero n de um frame 'tag_<n>'; nullopt se o frame nao tem esse formato
/// (vazio, 'ct_vermelho', 'tag_x', 'tag_'...).
std::optional<int> tagIdFromFrame(const std::string & frame);

/// Classifica o frame pelo id: em reference_ids -> kReference; qualquer outro
/// (inclusive frame que nao casa 'tag_<n>') -> kCube, que e' o comportamento
/// de sempre.
StackTargetKind classifyStackTarget(
  const std::string & frame,
  const std::vector<std::int64_t> & reference_ids);

/// Giro (rad) em [-pi/4, +pi/4] que leva `yaw_current` ao multiplo de 90
/// graus de `yaw_tag` mais proximo: yaw_current + delta == yaw_tag (mod
/// pi/2). Um cubo tem simetria de 90 graus, entao qualquer multiplo alinha
/// as faces - o escolhido e' o de MENOR giro do punho. Empate exato em
/// +-45 graus devolve um dos dois lados (|delta| <= pi/4 sempre).
double nearestQuarterTurn(double yaw_tag, double yaw_current);

/// Compensacao em z da soltura com a ferramenta inclinada `tilt_rad` a
/// partir da vertical: com o TCP no centro da face de cima, a quina mais
/// baixa de um cubo (largura w, altura h) fica h*cos(t) + (w/2)*sin(t)
/// abaixo do TCP, isto e' (w/2)*sin(t) - h*(1 - cos(t)) abaixo de onde
/// ficaria na vertical (4,0 mm a 15 graus e 4,9 mm a 30 para 42 mm). Somar
/// ao z do TCP mantem a folga nominal ate' o tampo/marca. 0 em t = 0;
/// nunca negativo.
double tiltDropZCompensation(double tilt_rad, double cube_width_m, double cube_height_m);

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

/// Registro do PRIMEIRO cubo da cadeia que termina em `base_frame` (o que
/// esta' direto sobre a tag de referencia / base desconhecida). nullopt se
/// `base_frame` nao tem registro. Ex.: A sobre tag_200, B sobre A:
/// root(B) = root(A) = registro de A. (2026-11-19: pilha no mesmo ponto.)
std::optional<StackMemoryEntry> findStackRoot(
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

/// Decisao da PILHA / tag de referencia (1 correcao). Regras na implementacao;
/// casos no gtest.
PlacementDecision decidePlacement(
  const PlacementObservation & obs, const PlacementLimits & lim);

const char * placementDecisionName(PlacementDecision d);

}  // namespace manip_task_execution
