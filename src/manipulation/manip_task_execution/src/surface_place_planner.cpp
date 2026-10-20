// Implementacao do planejador de superficie livre; contrato em surface_place_planner.hpp.
#include "manip_task_execution/surface_place_planner.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

namespace manip_task_execution
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

/// Transformada de distancia EUCLIDIANA exata (Felzenszwalb & Huttenlocher,
/// 2012) em celulas: f = 0 nas celulas "fonte", +inf nas demais. Devolve a
/// distancia (em celulas) de cada celula a fonte mais proxima; +inf se nao
/// ha fonte. O(N) por passada, duas passadas (linhas e colunas).
void edt1d(const std::vector<double> & f, std::vector<double> & d, int n,
           std::vector<int> & v, std::vector<double> & z)
{
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    double s;
    while (true) {
      const int vk = v[k];
      const double fq = f[q] + static_cast<double>(q) * q;
      const double fv = f[vk] + static_cast<double>(vk) * vk;
      if (std::isinf(f[q])) {s = kInf; break;}
      if (std::isinf(f[vk])) {s = -kInf; break;}
      s = (fq - fv) / (2.0 * q - 2.0 * vk);
      if (s > z[k]) {break;}
      if (k == 0) {break;}
      --k;
    }
    if (std::isinf(f[q])) {
      continue;  // sem fonte aqui: nao entra como parabola
    }
    if (k == 0 && std::isinf(f[v[0]])) {
      v[0] = q;  // primeira fonte real
      z[0] = -kInf;
      z[1] = kInf;
      continue;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    const int vk = v[k];
    if (std::isinf(f[vk])) {
      d[q] = kInf;
    } else {
      const double dq = static_cast<double>(q) - vk;
      d[q] = dq * dq + f[vk];
    }
  }
}

/// dist[j*w+i] = distancia euclidiana (celulas) a celula mais proxima com
/// source[j*w+i] == true. +inf se nao ha fonte.
std::vector<double> distanceTransform(const std::vector<char> & source, int w, int h)
{
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  std::vector<double> g(n, kInf);
  bool any = false;
  for (std::size_t k = 0; k < n; ++k) {
    if (source[k]) {g[k] = 0.0; any = true;}
  }
  if (!any) {
    return g;
  }
  const int m = std::max(w, h);
  std::vector<double> f(static_cast<std::size_t>(m)), d(static_cast<std::size_t>(m));
  std::vector<int> v(static_cast<std::size_t>(m) + 1);
  std::vector<double> z(static_cast<std::size_t>(m) + 2);
  // colunas
  for (int i = 0; i < w; ++i) {
    for (int j = 0; j < h; ++j) {f[static_cast<std::size_t>(j)] = g[static_cast<std::size_t>(j) * w + i];}
    edt1d(f, d, h, v, z);
    for (int j = 0; j < h; ++j) {g[static_cast<std::size_t>(j) * w + i] = d[static_cast<std::size_t>(j)];}
  }
  // linhas
  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w; ++i) {f[static_cast<std::size_t>(i)] = g[static_cast<std::size_t>(j) * w + i];}
    edt1d(f, d, w, v, z);
    for (int i = 0; i < w; ++i) {g[static_cast<std::size_t>(j) * w + i] = d[static_cast<std::size_t>(i)];}
  }
  for (auto & val : g) {
    if (!std::isinf(val)) {val = std::sqrt(val);}
  }
  return g;
}

bool isTable(int8_t c)
{
  return c == SurfaceCell::kTableFree || c == SurfaceCell::kObstacle;
}

struct LineFit
{
  bool ok{false};
  double a{0.0};   // y = a + b x
  double b{0.0};
  int support{0};
  double x_min{0.0};
  double x_max{0.0};
};

/// Minimos quadrados y = a + b x com 2 passadas de rejeicao de outliers.
LineFit fitLine(std::vector<std::pair<double, double>> pts, double inlier_m, int min_support,
                double min_span)
{
  LineFit fit;
  for (int pass = 0; pass < 3; ++pass) {
    const int n = static_cast<int>(pts.size());
    if (n < min_support) {
      return fit;
    }
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const auto & p : pts) {sx += p.first; sy += p.second; sxx += p.first * p.first; sxy += p.first * p.second;}
    const double den = n * sxx - sx * sx;
    double b = 0.0;
    if (std::abs(den) > 1e-9) {
      b = (n * sxy - sx * sy) / den;
    }
    const double a = (sy - b * sx) / n;
    // Frente de mesa: quase perpendicular a +Y. Inclinacao absurda (> 30 graus)
    // = nao e' a frente (ou pontos de duas bordas misturados).
    if (std::abs(b) > std::tan(30.0 * M_PI / 180.0)) {
      b = 0.0;
    }
    std::vector<std::pair<double, double>> inl;
    for (const auto & p : pts) {
      if (std::abs(p.second - (a + b * p.first)) <= inlier_m) {inl.push_back(p);}
    }
    fit.a = a;
    fit.b = b;
    fit.support = static_cast<int>(inl.size());
    if (inl.size() == pts.size() || pass == 2) {
      pts = inl;
      break;
    }
    pts = inl;
  }
  if (static_cast<int>(pts.size()) < min_support) {
    return fit;
  }
  double xmin = kInf, xmax = -kInf;
  for (const auto & p : pts) {xmin = std::min(xmin, p.first); xmax = std::max(xmax, p.first);}
  fit.x_min = xmin;
  fit.x_max = xmax;
  fit.support = static_cast<int>(pts.size());
  fit.ok = (xmax - xmin) >= min_span;
  return fit;
}

double median(std::vector<double> v)
{
  if (v.empty()) {return 0.0;}
  std::sort(v.begin(), v.end());
  const std::size_t m = v.size() / 2;
  return v.size() % 2 ? v[m] : 0.5 * (v[m - 1] + v[m]);
}

}  // namespace

bool SurfaceGrid::cellOf(double x, double y, int * i, int * j) const
{
  if (resolution <= 0.0) {return false;}
  const double fi = std::floor((x - origin_x) / resolution);
  const double fj = std::floor((y - origin_y) / resolution);
  if (fi < 0 || fj < 0 || fi >= width || fj >= height) {return false;}
  *i = static_cast<int>(fi);
  *j = static_cast<int>(fj);
  return true;
}

SurfaceGrid makeSurfaceGrid(
  double origin_x, double origin_y, double resolution, int width, int height,
  const std::vector<int8_t> & data)
{
  SurfaceGrid g;
  g.origin_x = origin_x;
  g.origin_y = origin_y;
  g.resolution = resolution;
  g.width = width;
  g.height = height;
  g.cells = data;
  return g;
}

void paintObstacle(SurfaceGrid & grid, double x, double y, double radius_m)
{
  if (!grid.valid()) {return;}
  const double r2 = radius_m * radius_m;
  const int i0 = static_cast<int>(std::floor((x - radius_m - grid.origin_x) / grid.resolution));
  const int i1 = static_cast<int>(std::ceil((x + radius_m - grid.origin_x) / grid.resolution));
  const int j0 = static_cast<int>(std::floor((y - radius_m - grid.origin_y) / grid.resolution));
  const int j1 = static_cast<int>(std::ceil((y + radius_m - grid.origin_y) / grid.resolution));
  for (int j = std::max(0, j0); j <= std::min(grid.height - 1, j1); ++j) {
    for (int i = std::max(0, i0); i <= std::min(grid.width - 1, i1); ++i) {
      const double dx = grid.cellX(i) - x;
      const double dy = grid.cellY(j) - y;
      if (dx * dx + dy * dy <= r2) {
        int8_t & c = grid.at(i, j);
        if (c != SurfaceCell::kOffTable) {
          c = SurfaceCell::kObstacle;
        }
      }
    }
  }
}

void parseWallSides(
  const std::string & spec, bool * left, bool * right, std::vector<std::string> * unknown_tokens)
{
  *left = false;
  *right = false;
  std::string tok;
  auto flush = [&]() {
      std::string t;
      for (char c : tok) {
        if (!std::isspace(static_cast<unsigned char>(c))) {
          t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
      }
      tok.clear();
      if (t.empty()) {return;}
      if (t == "left" || t == "esquerda" || t == "l" || t == "e") {*left = true; return;}
      if (t == "right" || t == "direita" || t == "r" || t == "d") {*right = true; return;}
      if (t == "both" || t == "ambos" || t == "ambas") {*left = true; *right = true; return;}
      if (t == "none" || t == "nenhuma" || t == "nenhum") {return;}
      if (unknown_tokens) {unknown_tokens->push_back(t);}
    };
  for (char c : spec) {
    if (c == ',' || c == ';' || c == '|' || c == '+') {flush();} else {tok += c;}
  }
  flush();
}

SurfacePlan planSurfacePlacement(
  const SurfaceGrid & grid_in,
  const SurfacePlaceRules & rules,
  const std::vector<SurfaceExtraObstacle> & extra,
  const SurfacePlannerOptions & options)
{
  SurfacePlan plan;
  std::ostringstream diag;
  if (!grid_in.valid()) {
    plan.diagnostics = "grade invalida";
    return plan;
  }
  plan.grid_valid = true;

  SurfaceGrid grid = grid_in;
  for (const auto & ob : extra) {
    paintObstacle(grid, ob.x, ob.y, ob.radius_m);
  }

  const int w = grid.width;
  const int h = grid.height;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  std::vector<char> off(n, 0), obs(n, 0), unk(n, 0);
  plan.table_x_min = kInf; plan.table_x_max = -kInf;
  plan.table_y_min = kInf; plan.table_y_max = -kInf;
  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w; ++i) {
      const int8_t c = grid.at(i, j);
      const std::size_t k = static_cast<std::size_t>(j) * w + i;
      switch (c) {
        case SurfaceCell::kTableFree: ++plan.n_table_free; break;
        case SurfaceCell::kObstacle: ++plan.n_obstacle; obs[k] = 1; break;
        case SurfaceCell::kOffTable: ++plan.n_off_table; off[k] = 1; break;
        default: ++plan.n_unknown; unk[k] = 1; break;
      }
      // Extensao so' das celulas LIVRES: um obstaculo pode ser o chao visto
      // sem borda detectada (o no de superficie marca nao-branco sem reta
      // como objeto), e ele nao pode "esticar" a mesa.
      if (c == SurfaceCell::kTableFree) {
        plan.table_x_min = std::min(plan.table_x_min, grid.cellX(i));
        plan.table_x_max = std::max(plan.table_x_max, grid.cellX(i));
        plan.table_y_min = std::min(plan.table_y_min, grid.cellY(j));
        plan.table_y_max = std::max(plan.table_y_max, grid.cellY(j));
      }
    }
  }
  plan.table_seen = (plan.n_table_free + plan.n_obstacle) > 0;
  if (!plan.table_seen) {
    plan.diagnostics = "nenhuma celula de tampo na grade";
    return plan;
  }

  // ---- Frente da mesa: celulas de tampo cujo vizinho de baixo (y menor) e'
  // fora-da-mesa. Reta y = a + b x (quase horizontal).
  std::vector<std::pair<double, double>> front_pts;
  for (int j = 1; j < h; ++j) {
    for (int i = 0; i < w; ++i) {
      if (isTable(grid.at(i, j)) && grid.at(i, j - 1) == SurfaceCell::kOffTable) {
        // Posicao da borda = fronteira entre as duas celulas.
        front_pts.emplace_back(grid.cellX(i), grid.cellY(j) - 0.5 * grid.resolution);
      }
    }
  }
  {
    LineFit fit = fitLine(
      front_pts, options.front_fit_inlier_m, options.front_min_support,
      options.front_min_span_m);
    if (fit.ok && options.front_prior_y) {
      const double mid_x = 0.5 * (fit.x_min + fit.x_max);
      const double dev = std::abs((fit.a + fit.b * mid_x) - *options.front_prior_y);
      if (dev > options.front_prior_tolerance_m) {
        diag << "frente ajustada (y=" << (fit.a + fit.b * mid_x) << ") a " << dev
             << " m do prior " << *options.front_prior_y << " -> REJEITADA; ";
        plan.front.rejected_by_prior = true;
        fit.ok = false;
      }
    }
    if (fit.ok) {
      plan.front.found = true;
      plan.front.y0 = fit.a;
      plan.front.slope = fit.b;
      plan.front.support = fit.support;
      plan.front.x_min = fit.x_min;
      plan.front.x_max = fit.x_max;
      diag << "frente vista: y=" << fit.a << " + " << fit.b << "x (" << fit.support
           << " pts, x " << fit.x_min << ".." << fit.x_max << "); ";
    } else if (options.fallback_front_y) {
      plan.front.found = true;
      plan.front.from_fallback = true;
      plan.front.y0 = *options.fallback_front_y;
      plan.front.slope = 0.0;
      plan.front.support = 0;
      diag << "frente NAO vista (" << front_pts.size() << " pts de borda) -> fallback y="
           << *options.fallback_front_y << "; ";
    } else {
      diag << "frente NAO vista (" << front_pts.size()
           << " pts de borda) e sem fallback: profundidade da zona nao imposta; ";
    }
  }

  // ---- Bordas laterais: tampo cujo vizinho em +x (direita) / -x (esquerda)
  // e' fora-da-mesa. x = mediana.
  {
    std::vector<double> right_xs, left_xs, right_ys, left_ys;
    for (int j = 0; j < h; ++j) {
      for (int i = 1; i + 1 < w; ++i) {
        if (!isTable(grid.at(i, j))) {continue;}
        if (grid.at(i + 1, j) == SurfaceCell::kOffTable) {
          right_xs.push_back(grid.cellX(i) + 0.5 * grid.resolution);
          right_ys.push_back(grid.cellY(j));
        }
        if (grid.at(i - 1, j) == SurfaceCell::kOffTable) {
          left_xs.push_back(grid.cellX(i) - 0.5 * grid.resolution);
          left_ys.push_back(grid.cellY(j));
        }
      }
    }
    auto side = [&](std::vector<double> & xs, std::vector<double> & ys, SurfaceSideEdge & out,
                    const char * name) {
        if (static_cast<int>(xs.size()) < options.side_min_support) {
          diag << name << " nao vista; ";
          return;
        }
        const double xm = median(xs);
        // Rejeita pontos longe da mediana (objetos na borda, frente/fundo).
        std::vector<double> inl_x, inl_y;
        for (std::size_t k = 0; k < xs.size(); ++k) {
          if (std::abs(xs[k] - xm) <= 0.03) {inl_x.push_back(xs[k]); inl_y.push_back(ys[k]);}
        }
        if (static_cast<int>(inl_x.size()) < options.side_min_support) {
          diag << name << " nao vista (dispersa); ";
          return;
        }
        const double ymin = *std::min_element(inl_y.begin(), inl_y.end());
        const double ymax = *std::max_element(inl_y.begin(), inl_y.end());
        if (ymax - ymin < options.side_min_span_m) {
          diag << name << " nao vista (extensao " << (ymax - ymin) << " m); ";
          return;
        }
        out.found = true;
        out.x = median(inl_x);
        out.support = static_cast<int>(inl_x.size());
        diag << name << " x=" << out.x << " (" << out.support << " pts); ";
      };
    side(right_xs, right_ys, plan.right, "borda direita");
    side(left_xs, left_ys, plan.left, "borda esquerda");
  }

  // ---- Distancias (em metros) a fora-da-mesa, obstaculos e desconhecido.
  const std::vector<double> d_off = distanceTransform(off, w, h);
  const std::vector<double> d_obs = distanceTransform(obs, w, h);
  const std::vector<double> d_unk = distanceTransform(unk, w, h);
  const double res = grid.resolution;

  // Limites laterais por parede. Sem x da parede: a partir da borda da mesa
  // vista; sem borda vista: a partir da extensao vista do tampo (conservador:
  // o tampo real vai pelo menos ate ali).
  const double half = 0.5 * rules.cube_size_m;
  std::optional<double> x_max_allowed, x_min_allowed;
  if (rules.wall_right) {
    double ref;
    if (rules.wall_right_x) {ref = *rules.wall_right_x;} else if (plan.right.found) {ref = plan.right.x;} else {ref = plan.table_x_max + 0.5 * res;}
    x_max_allowed = ref - (rules.wall_clearance_m + half + rules.safety_margin_m);
    diag << "parede direita: x <= " << *x_max_allowed << "; ";
  }
  if (rules.wall_left) {
    double ref;
    if (rules.wall_left_x) {ref = *rules.wall_left_x;} else if (plan.left.found) {ref = plan.left.x;} else {ref = plan.table_x_min - 0.5 * res;}
    x_min_allowed = ref + (rules.wall_clearance_m + half + rules.safety_margin_m);
    diag << "parede esquerda: x >= " << *x_min_allowed << "; ";
  }

  const double min_edge = rules.footprint_radius_m + rules.edge_clearance_m + rules.safety_margin_m;
  const double min_obs = rules.footprint_radius_m + rules.object_clearance_m + rules.safety_margin_m;
  const double min_unk = rules.footprint_radius_m;
  // Zona em profundidade a partir da frente: centro do cubo entre
  // [2 cm + meio cubo, 20 cm - meio cubo] (+ seguranca).
  const double depth_min = rules.edge_clearance_m + half + rules.safety_margin_m;
  const double depth_max = rules.zone_depth_m - half - rules.safety_margin_m;
  const double front_cos = std::cos(std::atan(plan.front.slope));

  std::vector<SurfaceCandidate> all;
  all.reserve(n / 4);
  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w; ++i) {
      if (grid.at(i, j) != SurfaceCell::kTableFree) {continue;}
      const std::size_t k = static_cast<std::size_t>(j) * w + i;
      const double x = grid.cellX(i);
      const double y = grid.cellY(j);
      const double de = d_off[k] * res;
      const double dob = d_obs[k] * res;
      const double du = d_unk[k] * res;
      if (de < min_edge) {continue;}
      if (dob < min_obs) {continue;}
      if (rules.unknown_blocks_footprint && du < min_unk) {continue;}
      double depth = 0.0;
      if (plan.front.found) {
        depth = (y - plan.front.yAt(x)) * front_cos;
        if (depth < depth_min || depth > depth_max) {continue;}
      }
      if (x_max_allowed && x > *x_max_allowed) {continue;}
      if (x_min_allowed && x < *x_min_allowed) {continue;}
      if (options.max_reach_radius_m > 0.0 && std::hypot(x, y) > options.max_reach_radius_m) {continue;}
      SurfaceCandidate c;
      c.x = x;
      c.y = y;
      c.clearance_obstacle_m = (std::isinf(dob) ? options.clearance_cap_m + 1.0 : dob) - rules.footprint_radius_m;
      c.clearance_edge_m = (std::isinf(de) ? options.clearance_cap_m + 1.0 : de) - rules.footprint_radius_m;
      c.depth_from_front_m = depth;
      double score = 0.0;
      score += options.weight_obstacle_clearance * std::min(c.clearance_obstacle_m, options.clearance_cap_m);
      score += options.weight_edge_clearance * std::min(c.clearance_edge_m, options.clearance_cap_m);
      if (options.preferred_x && options.preferred_y) {
        score -= options.weight_preferred_distance *
          std::hypot(x - *options.preferred_x, y - *options.preferred_y);
      } else if (options.preferred_x) {
        score -= options.weight_preferred_distance * std::abs(x - *options.preferred_x);
      }
      c.score = score;
      all.push_back(c);
    }
  }
  plan.n_valid_cells = static_cast<int>(all.size());
  std::stable_sort(all.begin(), all.end(), [](const SurfaceCandidate & a, const SurfaceCandidate & b) {
      return a.score > b.score;
    });
  // Supressao de nao-maximos: candidatos espacados.
  const double sp2 = options.candidate_spacing_m * options.candidate_spacing_m;
  for (const auto & c : all) {
    if (static_cast<int>(plan.candidates.size()) >= options.max_candidates) {break;}
    bool close = false;
    for (const auto & kept : plan.candidates) {
      const double dx = kept.x - c.x, dy = kept.y - c.y;
      if (dx * dx + dy * dy < sp2) {close = true; break;}
    }
    if (!close) {plan.candidates.push_back(c);}
  }
  diag << "tampo livre " << plan.n_table_free << " cel, obstaculo " << plan.n_obstacle
       << ", fora " << plan.n_off_table << ", desconhecido " << plan.n_unknown
       << "; validas " << plan.n_valid_cells << " -> " << plan.candidates.size() << " candidatos";
  plan.diagnostics = diag.str();
  return plan;
}

SideWallResult detectSideWalls(
  const std::vector<std::pair<double, double>> & points_xy,
  double front_y,
  std::optional<double> left_edge_x,
  std::optional<double> right_edge_x,
  const SideWallParams & params)
{
  SideWallResult out;
  const double y_lo = front_y + params.y_from_front_min_m;
  const double y_hi = front_y + params.y_from_front_max_m;
  const double right_edge = right_edge_x ? *right_edge_x : params.half_table_width_m;
  const double left_edge = left_edge_x ? *left_edge_x : -params.half_table_width_m;
  std::vector<double> rx, lx;
  for (const auto & p : points_xy) {
    if (p.second < y_lo || p.second > y_hi) {continue;}
    if (p.first >= right_edge && p.first <= right_edge + params.search_width_m) {rx.push_back(p.first);}
    if (p.first <= left_edge && p.first >= left_edge - params.search_width_m) {lx.push_back(p.first);}
  }
  auto eval = [&](std::vector<double> & xs, bool & found, double & x_out, int & npts) {
      npts = static_cast<int>(xs.size());
      if (npts < params.min_points) {return;}
      const double m = median(xs);
      std::vector<double> inl;
      for (double v : xs) {if (std::abs(v - m) <= params.max_x_spread_m) {inl.push_back(v);}}
      if (static_cast<int>(inl.size()) < params.min_points) {return;}
      found = true;
      x_out = median(inl);
      npts = static_cast<int>(inl.size());
    };
  eval(rx, out.right_found, out.right_x, out.right_points);
  eval(lx, out.left_found, out.left_x, out.left_points);
  return out;
}

}  // namespace manip_task_execution
