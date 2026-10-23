// Planejador de SOLTURA EM SUPERFICIE LIVRE (2026-10-19).
//
// Entrada: um mapa top-down da mesa (grade de ocupacao em manip_base_link,
// produzido pelo workstation_surface_node a partir da RealSense do punho),
// obstaculos extras (tags/containers vistos por TF, memoria de slots) e as
// regras do rulebook (RoboCup@Work 4.4, Manipulation Zone):
//   - profundidade maxima da zona: 20 cm a partir da FRENTE da service area;
//   - 2 cm entre a borda da service area e a zona de manipulacao;
//   - 2 cm entre objetos (e entre objeto e container);
//   - 10 cm entre o comeco da zona e uma PAREDE.
// Saida: candidatos (x, y) no plano da mesa onde um cubo de 41 mm cabe
// respeitando tudo isso, ordenados por pontuacao (folga, centralidade).
// A IK e' verificada por quem chama (o no de place), fora desta biblioteca:
// aqui nao ha ROS nem MoveIt, so geometria — testavel por gtest.
//
// FRAME: manip_base_link (+X = direita do robo, +Y = frente). A FRENTE da
// mesa e' a borda de MENOR y (a mais proxima do robo docado).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace manip_task_execution
{

/// Codigos das celulas (os mesmos do workstation_surface_node.py e do
/// nav_msgs/OccupancyGrid publicado por ele).
struct SurfaceCell
{
  static constexpr int8_t kUnknown = -1;    ///< nunca vista (fora da imagem / garra)
  static constexpr int8_t kTableFree = 0;   ///< tampo branco livre
  static constexpr int8_t kOffTable = 70;   ///< fora da mesa (chao, parede, nada)
  static constexpr int8_t kObstacle = 100;  ///< objeto/container sobre a mesa
};

/// Grade regular no plano da mesa. Celula (i, j): centro em
/// (origin_x + (i+0.5)*res, origin_y + (j+0.5)*res); indice j*width + i
/// (layout do nav_msgs/OccupancyGrid).
struct SurfaceGrid
{
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.01};
  int width{0};
  int height{0};
  std::vector<int8_t> cells;

  bool valid() const
  {
    return width > 0 && height > 0 && resolution > 0.0 &&
           cells.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }
  int8_t at(int i, int j) const
  {
    return cells[static_cast<std::size_t>(j) * static_cast<std::size_t>(width) +
                 static_cast<std::size_t>(i)];
  }
  int8_t & at(int i, int j)
  {
    return cells[static_cast<std::size_t>(j) * static_cast<std::size_t>(width) +
                 static_cast<std::size_t>(i)];
  }
  double cellX(int i) const {return origin_x + (static_cast<double>(i) + 0.5) * resolution;}
  double cellY(int j) const {return origin_y + (static_cast<double>(j) + 0.5) * resolution;}
  /// Celula que contem (x, y); false se fora da grade.
  bool cellOf(double x, double y, int * i, int * j) const;
};

/// Regras do rulebook + geometria do objeto (todas em metros).
struct SurfacePlaceRules
{
  double cube_size_m{0.041};
  /// Raio do circulo que CIRCUNSCREVE o cubo (41 mm * sqrt(2) / 2 = 29 mm):
  /// o cubo cabe com qualquer yaw dentro dele — independente da linha dos
  /// dedos escolhida pelo punho.
  double footprint_radius_m{0.029};
  double edge_clearance_m{0.02};     ///< borda da service area -> zona
  double object_clearance_m{0.02};   ///< entre objetos (e containers)
  double wall_clearance_m{0.10};     ///< comeco da zona -> parede
  double zone_depth_m{0.20};         ///< frente da service area -> fim da zona
  /// Folga extra de seguranca somada a TODAS as distancias minimas (erro de
  /// ray-cast + erro de chegada do braco). 5 mm por default.
  double safety_margin_m{0.005};
  /// Celulas DESCONHECIDAS sob o cubo nao sao aceitas (o cubo inteiro tem que
  /// estar em tampo VISTO). false = ignora o desconhecido (so' para testes).
  bool unknown_blocks_footprint{true};
  bool wall_left{false};    ///< parede a ESQUERDA do robo (lado -X)
  bool wall_right{false};   ///< parede a DIREITA do robo (lado +X)
  /// Posicao X conhecida da parede (ex.: medida pelo LiDAR) — se dada, vale
  /// ELA (10 cm a partir da parede) em vez da borda da mesa.
  std::optional<double> wall_left_x;
  std::optional<double> wall_right_x;
};

/// Obstaculo extra (fora da grade): circulo no plano. Tags vistas por TF
/// (raio = footprint do cubo), containers (raio ~ meia diagonal), memoria.
struct SurfaceExtraObstacle
{
  double x{0.0};
  double y{0.0};
  double radius_m{0.029};
  std::string label;
};

/// Como o prior da frente (docking/LiDAR) entra na decisao (2026-10-21).
/// Na run da WS1 o robo docou girado 19,7 graus: a camera viu a frente real
/// a y = 0,28 e o prior (0,163 +- 0,06) a rejeitou: a zona foi imposta no
/// lugar errado e nenhum candidato sobrou.
enum class FrontPriorMode
{
  /// A frente ajustada pela camera vale se tem suporte (front_min_support,
  /// front_min_span_m) e inclinacao <= front_max_slope_deg, MESMO longe do
  /// prior; o prior so' entra como fallback (frente nao vista).
  kCameraFirst,
  /// Comportamento antigo: frente ajustada a mais de front_prior_tolerance_m
  /// do prior e' rejeitada e o fallback vale.
  kPriorGate
};

/// "camera_first" | "prior_gate" (maiusculas/minusculas e '-' aceitos).
/// false se desconhecido (`*mode` nao muda).
bool parseFrontPriorMode(const std::string & spec, FrontPriorMode * mode);
const char * frontPriorModeName(FrontPriorMode mode);

struct SurfacePlannerOptions
{
  /// Se a frente da mesa NAO for vista na grade, usa esta reta (y constante)
  /// como frente. nullopt = sem fallback: a profundidade da zona nao e'
  /// imposta e `front.found` sai false (quem chama decide se aceita).
  std::optional<double> fallback_front_y;
  /// Prior da frente (docking/LiDAR). Em kPriorGate, uma frente AJUSTADA pela
  /// grade que fique a mais de `front_prior_tolerance_m` dele e' rejeitada
  /// (faces de objetos alinhados fingem borda) e o fallback vale; em
  /// kCameraFirst so' e' registrado no diagnostico. nullopt = sem checagem.
  std::optional<double> front_prior_y;
  double front_prior_tolerance_m{0.06};
  FrontPriorMode front_prior_mode{FrontPriorMode::kCameraFirst};
  /// kCameraFirst: inclinacao maxima (graus) da frente ajustada para ela
  /// valer sem o prior (a frente de uma mesa e' quase perpendicular a +Y; na
  /// run de 21/10, docado a 19,7 graus, a frente vista tinha 11,6 graus).
  double front_max_slope_deg{25.0};
  /// Ilhas de FORA-DA-MESA cercadas por tampo (componente de kOffTable cujo
  /// contorno so' tem tampo/obstaculo: nao toca a borda da grade nem celula
  /// desconhecida) com menos que isto de celulas viram kUnknown ANTES das
  /// bordas e das distancias (profundidade ruim na run de 21/10: ilhas
  /// dentro do tampo faziam "borda direita" a esquerda da esquerda e
  /// rejeitavam tudo por d_off). 150 celulas = 1,5 dm2 a 1 cm. 0 = desligado.
  int island_max_cells{150};
  /// Borda lateral: a "direita" so' vale com x > mediana x do tampo + isto
  /// (e a "esquerda" com x < mediana - isto); caso contrario e' um objeto ou
  /// ilha do lado errado e a borda fica "nao vista (lado errado)".
  double side_median_margin_m{0.02};
  /// Ponto preferido (x, y): a pontuacao penaliza a distancia ate ele (a
  /// pose ensinada MesaX, confortavel para o braco). nullopt = so' folga.
  std::optional<double> preferred_x;
  std::optional<double> preferred_y;
  /// Pesos da pontuacao.
  double weight_obstacle_clearance{1.0};
  double weight_edge_clearance{0.5};
  double weight_preferred_distance{0.6};
  /// Folga acima disto nao melhora a nota (6 cm): evita fugir para o canto.
  double clearance_cap_m{0.06};
  /// Candidatos devolvidos: no maximo `max_candidates`, espacados de
  /// `candidate_spacing_m` (supressao de nao-maximos) para a IK testar
  /// pontos DIFERENTES.
  int max_candidates{40};
  double candidate_spacing_m{0.02};
  /// Raio maximo do alvo ao eixo do braco (poda grosseira antes da IK;
  /// 0 = sem poda).
  double max_reach_radius_m{0.0};
  /// Ajuste da reta da frente: pontos a mais que isto da reta sao descartados
  /// (2 passadas); suporte minimo em pontos e extensao minima em x.
  double front_fit_inlier_m{0.02};
  int front_min_support{8};
  double front_min_span_m{0.08};
  int side_min_support{6};
  double side_min_span_m{0.06};
};

struct SurfaceFrontEdge
{
  bool found{false};
  bool from_fallback{false};
  bool rejected_by_prior{false};   ///< ajuste da grade descartado (longe do prior; so' kPriorGate)
  bool rejected_by_slope{false};   ///< ajuste da grade descartado (inclinado demais; so' kCameraFirst)
  /// |y da frente ajustada - prior| (0 sem prior ou sem ajuste): em
  /// kCameraFirst e' so' informacao (o place loga).
  double prior_deviation_m{0.0};
  double y0{0.0};        ///< y da reta em x = 0
  double slope{0.0};     ///< dy/dx (frente quase perpendicular a +Y => ~0)
  int support{0};
  double x_min{0.0};
  double x_max{0.0};
  double yAt(double x) const {return y0 + slope * x;}
};

struct SurfaceSideEdge
{
  bool found{false};
  /// Borda vista do lado errado da mediana do tampo (objeto/ilha): found=false
  /// e `x` guarda o x visto, para o diagnostico.
  bool rejected_wrong_side{false};
  double x{0.0};
  int support{0};
};

struct SurfaceCandidate
{
  double x{0.0};
  double y{0.0};
  double clearance_obstacle_m{0.0};  ///< distancia ao obstaculo mais proximo menos o footprint
  double clearance_edge_m{0.0};      ///< idem para fora-da-mesa
  double depth_from_front_m{0.0};    ///< distancia da frente da service area
  double score{0.0};
};

struct SurfacePlan
{
  bool grid_valid{false};
  bool table_seen{false};          ///< ha celulas de tampo (livre ou obstaculo)
  SurfaceFrontEdge front;
  SurfaceSideEdge left;            ///< borda de menor x (esquerda do robo)
  SurfaceSideEdge right;           ///< borda de maior x (direita do robo)
  int n_table_free{0};
  int n_obstacle{0};
  int n_off_table{0};
  int n_unknown{0};
  /// Extensao do tampo LIVRE visto (sem obstaculos: o chao sem borda
  /// detectada chega como obstaculo e nao pode esticar a mesa).
  double table_x_min{0.0};
  double table_x_max{0.0};
  double table_y_min{0.0};
  double table_y_max{0.0};
  /// Mediana do x das celulas de tampo (livre ou obstaculo): referencia das
  /// bordas laterais (direita tem que ficar a direita dela e vice-versa).
  double table_x_median{0.0};
  /// Ilhas de fora-da-mesa removidas (viraram desconhecido) e suas celulas.
  int n_islands_removed{0};
  int n_island_cells_removed{0};
  /// Celulas livres que passaram em TODAS as regras (antes da supressao).
  int n_valid_cells{0};
  /// Celulas LIVRES rejeitadas por regra: cada celula conta na PRIMEIRA
  /// regra que a rejeitou, na ordem borda > objeto > desconhecido > zona >
  /// parede > alcance. n_table_free = n_valid_cells + soma destes.
  int n_rejected_edge{0};      ///< a menos de 2 cm (+ footprint) de fora-da-mesa
  int n_rejected_object{0};    ///< a menos de 2 cm (+ footprint) de objeto
  int n_rejected_unknown{0};   ///< cubo encostando em celula nao vista
  int n_rejected_zone{0};      ///< fora da profundidade 2..20 cm da frente
  int n_rejected_wall{0};      ///< a menos de 10 cm da parede declarada/vista
  int n_rejected_reach{0};     ///< alem de max_reach_radius_m
  std::vector<SurfaceCandidate> candidates;   ///< melhores primeiro
  std::string diagnostics;
  /// "validas N (rejeitadas: borda a, objeto b, desconhecido c, zona d,
  /// parede e, alcance f)": a mesma linha do diagnostico, para log/result.
  std::string rejectionSummary() const;
};

/// Planeja. Nunca lanca; `plan.diagnostics` explica o que faltou.
SurfacePlan planSurfacePlacement(
  const SurfaceGrid & grid,
  const SurfacePlaceRules & rules,
  const std::vector<SurfaceExtraObstacle> & extra,
  const SurfacePlannerOptions & options);

/// Pinta um circulo de obstaculo na grade (celulas de tampo livre OU
/// desconhecidas viram kObstacle; fora-da-mesa fica como esta).
void paintObstacle(SurfaceGrid & grid, double x, double y, double radius_m);

/// Ilhas de fora-da-mesa (ver SurfacePlannerOptions::island_max_cells):
/// componentes 4-conexos de kOffTable com menos de `max_cells` celulas cujos
/// vizinhos (8) sao todos tampo/obstaculo (nao tocam a borda da grade nem
/// kUnknown) viram kUnknown. Devolve quantas ilhas; `cells_removed` (se nao
/// nulo) recebe o total de celulas. max_cells <= 0 = nao faz nada.
int removeOffTableIslands(SurfaceGrid & grid, int max_cells, int * cells_removed = nullptr);

/// Converte um nav_msgs/OccupancyGrid "cru" (origem, resolucao, dados) —
/// mantido como funcao livre para o no nao depender do layout interno.
SurfaceGrid makeSurfaceGrid(
  double origin_x, double origin_y, double resolution, int width, int height,
  const std::vector<int8_t> & data);

/// "left,right" / "esquerda" / "DIREITA" -> flags. Tokens desconhecidos sao
/// ignorados (devolvidos em `unknown_tokens` se nao nulo).
void parseWallSides(
  const std::string & spec, bool * left, bool * right,
  std::vector<std::string> * unknown_tokens = nullptr);

// ---------------------------------------------------------------------------
// Paredes pelo LiDAR (opcional). Pontos do /scan ja em manip_base_link (x, y).
// Uma parede ao lado da mesa e' uma linha de pontos paralela a +Y, logo ao
// lado da borda lateral, estendendo-se da frente da mesa para tras (para
// y maiores). Devolve a posicao x da parede (mediana) de cada lado.
// ---------------------------------------------------------------------------
struct SideWallParams
{
  /// Faixa em y (ao longo da lateral da mesa) onde procurar a parede, a
  /// partir da frente da mesa `front_y`.
  double y_from_front_min_m{-0.10};
  double y_from_front_max_m{0.60};
  /// Faixa em x ALEM da borda lateral da mesa (|x| crescente): da borda ate
  /// borda + search_width.
  double search_width_m{0.30};
  /// Metade da largura nominal da mesa: usada como borda quando a camera nao
  /// viu a borda daquele lado.
  double half_table_width_m{0.40};
  int min_points{12};
  /// Dispersao maxima em x dos pontos (parede = reta paralela a y).
  double max_x_spread_m{0.06};
};

struct SideWallResult
{
  bool left_found{false};
  double left_x{0.0};
  int left_points{0};
  bool right_found{false};
  double right_x{0.0};
  int right_points{0};
};

SideWallResult detectSideWalls(
  const std::vector<std::pair<double, double>> & points_xy,
  double front_y,
  std::optional<double> left_edge_x,
  std::optional<double> right_edge_x,
  const SideWallParams & params);

}  // namespace manip_task_execution
