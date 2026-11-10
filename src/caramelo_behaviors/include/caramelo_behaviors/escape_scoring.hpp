// Logica PURA do escape costmap-aware (sem ROS): avaliar direcoes de fuga
// pelo custo das celulas sob o footprint ao longo de cada raio e escolher a
// melhor. Separada do behavior para ser testada em gtest com grades
// sinteticas.
//
// Convencao de custo (nav2_costmap_2d): 0 livre, 1..252 inflacao, 253
// inscrito, 254 letal, 255 desconhecido. Aqui >= 254 e' celula RUIM
// (desconhecido INCLUSO: sem sensor traseiro, o que nunca foi visto nao e'
// livre).
//
// Regra central (o caso de "preso"): o robo pode JA estar com parte do
// footprint sobre celulas ruins. Essas celulas iniciais (cells0) sao
// toleradas — o que bloqueia uma pose e' celula ruim NOVA sob o footprint.
// Assim deslizar PARA FORA do obstaculo e' permitido e entrar em outro nao.
#ifndef CARAMELO_BEHAVIORS__ESCAPE_SCORING_HPP_
#define CARAMELO_BEHAVIORS__ESCAPE_SCORING_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace caramelo_behaviors
{

constexpr double kBadCost = 254.0;

// Direcao candidata no frame do CORPO (dx, dy unitarios). preference: 0 =
// frente (vista pelo LiDAR), 1 = diagonal de frente, 2 = lateral, 3 =
// diagonal de re, 4 = re (cega). Empate de custo -> menor preference.
struct EscapeDirection
{
  double dx;
  double dy;
  int preference;
  std::string name;
};

inline std::vector<EscapeDirection> defaultDirections(bool allow_backward)
{
  const double d = std::sqrt(0.5);
  std::vector<EscapeDirection> out = {
    {1.0, 0.0, 0, "frente"},
    {d, d, 1, "diag_frente_esq"},
    {d, -d, 1, "diag_frente_dir"},
    {0.0, 1.0, 2, "esquerda"},
    {0.0, -1.0, 2, "direita"},
  };
  if (allow_backward) {
    out.push_back({-d, d, 3, "diag_re_esq"});
    out.push_back({-d, -d, 3, "diag_re_dir"});
    out.push_back({-1.0, 0.0, 4, "re"});
  }
  return out;
}

// Avaliacao do footprint numa pose: celulas ruins NOVAS (fora de cells0),
// celulas ruins ANTIGAS ainda sob o robo, e estatisticas das celulas boas.
struct PoseEval
{
  int new_bad{0};
  int old_bad{0};
  double max_cost{0.0};
  double sum_cost{0.0};
  int n_cells{0};
  double mean() const {return n_cells ? sum_cost / n_cells : 0.0;}
};

struct RayScore
{
  bool blocked{true};       // alguma pose do raio tem celula ruim NOVA
  double free_len{0.0};     // comprimento sem celula ruim nova (ou dist)
  double end_cost{255.0};   // max_cost das celulas boas na ultima pose livre
  double mean_cost{255.0};  // media (ao longo do raio) do custo medio das boas
  int end_old_bad{0};       // celulas ruins antigas ainda sob o robo no fim
};

// Avalia o raio que parte de (x0, y0, yaw) na direcao d (frame do corpo)
// ate dist, de step em step. eval_fn(x, y, yaw) -> PoseEval (frame do mundo).
template<typename EvalFn>
RayScore scoreRay(
  EvalFn && eval_fn, double x0, double y0, double yaw,
  const EscapeDirection & d, double dist, double step)
{
  RayScore r;
  if (!(dist > 0.0) || !(step > 0.0)) {
    return r;
  }
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double wx = d.dx * c - d.dy * s;
  const double wy = d.dx * s + d.dy * c;
  const int n = static_cast<int>(std::ceil(dist / step - 1e-9));
  double sum_mean = 0.0;
  int count = 0;
  double last_free = 0.0;
  for (int i = 1; i <= n; ++i) {
    const double along = std::min(dist, i * step);
    const PoseEval e = eval_fn(x0 + wx * along, y0 + wy * along, yaw);
    if (e.new_bad > 0) {
      r.blocked = true;
      r.free_len = last_free;
      r.mean_cost = count ? sum_mean / count : 255.0;
      return r;
    }
    last_free = along;
    sum_mean += e.mean();
    ++count;
    r.end_cost = e.max_cost;
    r.end_old_bad = e.old_bad;
  }
  r.blocked = false;
  r.free_len = last_free;
  r.mean_cost = count ? sum_mean / count : 0.0;
  return r;
}

struct ChoiceParams
{
  double preference_weight{10.0};  // custo extra por grau de preferencia
  double min_improvement{10.0};    // melhora minima de max_cost exigida quando em apuros
  double high_cost{128.0};         // max_cost atual acima disto = "em apuros"
  double old_bad_weight{40.0};     // custo por celula ruim antiga que CONTINUA sob o robo no fim
};

struct EscapeChoice
{
  std::size_t index;
  double objective;
  RayScore score;
};

// Escolhe a direcao: so' raios NAO bloqueados. Se o robo comecou sobre
// celulas ruins (bad0 > 0), o fim do curso precisa ter MENOS delas (saiu, ao
// menos em parte). Se nao ha ruins mas o custo e' alto, exige melhora de
// max_cost. Objetivo = end_cost + 0.25*media + old_bad_weight*old_bad_no_fim
// + preference_weight*preference (menor e' melhor).
inline std::optional<EscapeChoice> chooseEscape(
  const std::vector<EscapeDirection> & dirs, const std::vector<RayScore> & scores,
  int bad0, double current_max_cost, const ChoiceParams & p)
{
  std::optional<EscapeChoice> best;
  for (std::size_t i = 0; i < dirs.size() && i < scores.size(); ++i) {
    const RayScore & s = scores[i];
    if (s.blocked) {
      continue;
    }
    if (bad0 > 0) {
      if (s.end_old_bad >= bad0) {
        continue;  // nao saiu de cima de nada
      }
    } else if (current_max_cost >= p.high_cost &&
      s.end_cost > current_max_cost - p.min_improvement)
    {
      continue;
    }
    const double obj = s.end_cost + 0.25 * s.mean_cost +
      p.old_bad_weight * static_cast<double>(s.end_old_bad) +
      p.preference_weight * static_cast<double>(dirs[i].preference);
    const bool better = !best || obj < best->objective - 1e-9 ||
      (std::abs(obj - best->objective) <= 1e-9 &&
      dirs[i].preference < dirs[best->index].preference);
    if (better) {
      best = EscapeChoice{i, obj, s};
    }
  }
  return best;
}

// Casco convexo (monotone chain) de pontos 2D: o footprint do Caramelo tem
// um recorte (nao e' convexo) e o preenchimento de celulas do costmap so'
// vale para poligono convexo — usar o casco e' conservador (cobre o recorte).
inline std::vector<std::pair<double, double>> convexHull(
  std::vector<std::pair<double, double>> pts)
{
  if (pts.size() < 3) {
    return pts;
  }
  std::sort(pts.begin(), pts.end());
  auto cross = [](const std::pair<double, double> & o, const std::pair<double, double> & a,
      const std::pair<double, double> & b) {
      return (a.first - o.first) * (b.second - o.second) -
             (a.second - o.second) * (b.first - o.first);
    };
  std::vector<std::pair<double, double>> hull(2 * pts.size());
  std::size_t k = 0;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) {
      --k;
    }
    hull[k++] = pts[i];
  }
  for (std::size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i - 1]) <= 0) {
      --k;
    }
    hull[k++] = pts[i - 1];
  }
  hull.resize(k - 1);
  return hull;
}

}  // namespace caramelo_behaviors

#endif  // CARAMELO_BEHAVIORS__ESCAPE_SCORING_HPP_
