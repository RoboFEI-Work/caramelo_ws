// Ver o .hpp. Padrao de ciclo copiado do DriveOnHeading do nav2 1.3.13
// (/opt/ros/jazzy/include/nav2_behaviors/plugins/drive_on_heading.hpp);
// a checagem usa o costmap cru (interior do footprint) em vez do
// CostmapTopicCollisionChecker, que so' olha as arestas e reprova qualquer
// pose enquanto o robo ainda esta parcialmente sobre o obstaculo.
#include "caramelo_behaviors/escape_free_space.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace caramelo_behaviors
{

EscapeFreeSpace::EscapeFreeSpace()
: TimedBehavior<ActionT>(),
  feedback_(std::make_shared<ActionT::Feedback>())
{
}

void EscapeFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"EscapeFreeSpace: nao conseguiu travar o no"};
  }
  const std::string p = behavior_name_ + ".";
  auto declare = [&](const std::string & name, auto & var) {
      nav2_util::declare_parameter_if_not_declared(node, p + name, rclcpp::ParameterValue(var));
      node->get_parameter(p + name, var);
    };
  declare("distance", distance_);
  declare("speed", speed_);
  declare("step", step_);
  declare("simulate_ahead_time", simulate_ahead_time_);
  declare("preference_weight", choice_.preference_weight);
  declare("min_improvement", choice_.min_improvement);
  declare("high_cost", choice_.high_cost);
  declare("old_bad_weight", choice_.old_bad_weight);
  declare("exit_cost", exit_cost_);
  declare("min_travel_for_exit", min_travel_for_exit_);
  declare("allow_backward", allow_backward_);
  declare("use_global_costmap", use_global_);
  declare("ignore_map_if_contradiction", ignore_map_if_contradiction_);
  declare("map_contradiction_min_cells", map_contradiction_min_cells_);

  // Topicos dos costmaps/footprint: os mesmos que o behavior_server usa.
  std::string local_topic = "local_costmap/costmap_raw";
  std::string global_topic = "global_costmap/costmap_raw";
  std::string footprint_topic = "local_costmap/published_footprint";
  node->get_parameter_or("local_costmap_topic", local_topic, local_topic);
  node->get_parameter_or("global_costmap_topic", global_topic, global_topic);
  node->get_parameter_or("local_footprint_topic", footprint_topic, footprint_topic);

  auto lnode = std::dynamic_pointer_cast<nav2_util::LifecycleNode>(node);
  if (!lnode) {
    throw std::runtime_error{"EscapeFreeSpace: o no pai nao e' nav2_util::LifecycleNode"};
  }
  nav2_util::LifecycleNode::WeakPtr weak = lnode;
  local_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(weak, local_topic);
  if (use_global_) {
    global_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(weak, global_topic);
  }
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    weak, footprint_topic, *tf_, robot_base_frame_, transform_tolerance_);

  RCLCPP_INFO(
    logger_, "[%s] configurado: dist %.2f m, vel %.2f m/s, passo %.3f, antevisao %.1f s, "
    "re %s, costmap global %s (%s)", behavior_name_.c_str(), distance_, speed_, step_,
    simulate_ahead_time_, allow_backward_ ? "sim" : "nao", use_global_ ? "sim" : "nao",
    global_topic.c_str());
}

void EscapeFreeSpace::onCleanup()
{
  local_sub_.reset();
  global_sub_.reset();
  footprint_sub_.reset();
  local_cm_.reset();
  global_cm_.reset();
}

bool EscapeFreeSpace::currentPose(geometry_msgs::msg::PoseStamped & pose, const std::string & frame)
{
  return nav2_util::getCurrentPose(pose, *tf_, frame, robot_base_frame_, transform_tolerance_);
}

long long EscapeFreeSpace::cellKey(const Costmap2D & cm, unsigned int mx, unsigned int my)
{
  // Chave em coordenadas de MUNDO quantizadas pela resolucao: a janela do
  // costmap local rola em multiplos de celula, entao a chave e' estavel.
  const double res = cm.getResolution();
  const long long kx = static_cast<long long>(std::floor((cm.getOriginX() + (mx + 0.5) * res) / res));
  const long long ky = static_cast<long long>(std::floor((cm.getOriginY() + (my + 0.5) * res) / res));
  return (kx << 32) ^ (ky & 0xffffffffLL);
}

void EscapeFreeSpace::odomToMap(
  double x, double y, double yaw, double & gx, double & gy, double & gyaw) const
{
  const double c = std::cos(dth_);
  const double s = std::sin(dth_);
  const double rx = x - odom_x0_;
  const double ry = y - odom_y0_;
  gx = map_x0_ + c * rx - s * ry;
  gy = map_y0_ + s * rx + c * ry;
  gyaw = yaw + dth_;
}

bool EscapeFreeSpace::evalOn(
  const std::shared_ptr<Costmap2D> & cm, double x, double y, double yaw,
  const CellKeys & keys0, PoseEval & e, CellKeys * collect)
{
  std::lock_guard<Costmap2D::mutex_t> lock(*cm->getMutex());
  std::vector<geometry_msgs::msg::Point> oriented;
  nav2_costmap_2d::transformFootprint(x, y, yaw, footprint_, oriented);
  std::vector<nav2_costmap_2d::MapLocation> poly;
  poly.reserve(oriented.size());
  for (const auto & pt : oriented) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!cm->worldToMap(pt.x, pt.y, mx, my)) {
      return false;
    }
    poly.push_back({mx, my});
  }
  std::vector<nav2_costmap_2d::MapLocation> cells;
  cm->convexFillCells(poly, cells);
  for (const auto & c : cells) {
    const unsigned char cost = cm->getCost(c.x, c.y);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      const long long key = cellKey(*cm, c.x, c.y);
      // Contar ANTES de coletar: na 1a passada keys0 e collect sao o mesmo
      // conjunto e tudo tem que contar como "novo" (= total inicial).
      if (keys0.count(key)) {
        ++e.old_bad;
      } else {
        ++e.new_bad;
      }
      if (collect) {
        collect->insert(key);
      }
    } else {
      e.max_cost = std::max(e.max_cost, static_cast<double>(cost));
      e.sum_cost += cost;
      ++e.n_cells;
    }
  }
  return true;
}

PoseEval EscapeFreeSpace::evalPose(double x, double y, double yaw)
{
  PoseEval e;
  if (!local_cm_ || !evalOn(local_cm_, x, y, yaw, keys0_local_, e, nullptr)) {
    e.new_bad += 1;  // fora da grade local = desconhecido
    return e;
  }
  if (have_map_ && global_cm_) {
    double gx, gy, gyaw;
    odomToMap(x, y, yaw, gx, gy, gyaw);
    // Fora do mapa global nao bloqueia: o local decide.
    evalOn(global_cm_, gx, gy, gyaw, keys0_global_, e, nullptr);
  }
  return e;
}

EscapeFreeSpace::ResultStatus EscapeFreeSpace::onRun(
  const std::shared_ptr<const ActionT::Goal> command)
{
  cmd_dist_ = command->target.x > 0.0 ? command->target.x : distance_;
  cmd_speed_ = command->speed > 0.0 ? command->speed : speed_;
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;

  // Dados: costmaps e footprint.
  try {
    local_cm_ = local_sub_->getCostmap();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "[%s] sem costmap local ainda: %s", behavior_name_.c_str(), e.what());
    return ResultStatus{Status::FAILED, ActionT::Result::UNKNOWN};
  }
  global_cm_.reset();
  if (use_global_ && global_sub_) {
    try {
      global_cm_ = global_sub_->getCostmap();
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "[%s] sem costmap global: checa so' o local", behavior_name_.c_str());
    }
  }
  std::vector<geometry_msgs::msg::Point> raw;
  std_msgs::msg::Header hdr;
  if (!footprint_sub_->getFootprintInRobotFrame(raw, hdr) || raw.size() < 3) {
    RCLCPP_ERROR(logger_, "[%s] footprint indisponivel", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, ActionT::Result::UNKNOWN};
  }
  std::vector<std::pair<double, double>> pts;
  for (const auto & p : raw) {
    pts.emplace_back(p.x, p.y);
  }
  footprint_.clear();
  for (const auto & p : convexHull(pts)) {
    geometry_msgs::msg::Point q;
    q.x = p.first;
    q.y = p.second;
    footprint_.push_back(q);
  }

  if (!currentPose(initial_pose_, local_frame_)) {
    RCLCPP_ERROR(logger_, "[%s] pose inicial indisponivel", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR};
  }
  const double x0 = initial_pose_.pose.position.x;
  const double y0 = initial_pose_.pose.position.y;
  const double yaw0 = tf2::getYaw(initial_pose_.pose.orientation);

  have_map_ = false;
  if (global_cm_) {
    geometry_msgs::msg::PoseStamped map_pose;
    if (currentPose(map_pose, global_frame_)) {
      have_map_ = true;
      odom_x0_ = x0;
      odom_y0_ = y0;
      map_x0_ = map_pose.pose.position.x;
      map_y0_ = map_pose.pose.position.y;
      dth_ = tf2::getYaw(map_pose.pose.orientation) - yaw0;
    } else {
      RCLCPP_WARN(logger_, "[%s] sem pose no %s: checa so' o costmap local",
        behavior_name_.c_str(), global_frame_.c_str());
    }
  }

  // Celulas ruins ja sob o robo (toleradas durante o escape).
  keys0_local_.clear();
  keys0_global_.clear();
  PoseEval start;
  if (!evalOn(local_cm_, x0, y0, yaw0, keys0_local_, start, &keys0_local_)) {
    RCLCPP_ERROR(logger_, "[%s] footprint fora do costmap local", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, ActionT::Result::UNKNOWN};
  }
  const int bad_local0 = start.new_bad + start.old_bad;
  if (have_map_) {
    double gx, gy, gyaw;
    odomToMap(x0, y0, yaw0, gx, gy, gyaw);
    // Avalia o global em separado para poder compara-lo com o local.
    PoseEval only_map;
    CellKeys keys_map;
    evalOn(global_cm_, gx, gy, gyaw, keys_map, only_map, &keys_map);
    const int bad_map0 = only_map.new_bad + only_map.old_bad;
    if (ignore_map_if_contradiction_ && bad_local0 == 0 &&
      bad_map0 >= map_contradiction_min_cells_)
    {
      // 2026-11-11: mapa diz "dentro de obstaculo", LiDAR diz "livre" => a
      // pose no mapa esta errada (AMCL deslocado). Sair pelo LiDAR e' o unico
      // jeito de o filtro receber movimento e se corrigir.
      RCLCPP_WARN(
        logger_, "[%s] mapa global diz %d celulas LETAIS sob o robo e o LiDAR (costmap "
        "local) diz 0: localizacao provavelmente deslocada; este escape ignora o costmap "
        "global e decide so' pelo LiDAR.", behavior_name_.c_str(), bad_map0);
      have_map_ = false;
    } else {
      keys0_global_ = keys_map;
      start.new_bad += only_map.new_bad;
      start.old_bad += only_map.old_bad;
      start.max_cost = std::max(start.max_cost, only_map.max_cost);
      start.sum_cost += only_map.sum_cost;
      start.n_cells += only_map.n_cells;
    }
  }
  bad0_ = start.new_bad + start.old_bad;  // total de celulas ruins sob o robo no inicio
  const double current_max = start.max_cost;

  const auto dirs = defaultDirections(allow_backward_);
  std::vector<RayScore> scores;
  scores.reserve(dirs.size());
  for (const auto & d : dirs) {
    scores.push_back(scoreRay(
      [this](double x, double y, double yaw) {return evalPose(x, y, yaw);},
      x0, y0, yaw0, d, cmd_dist_, step_));
  }
  const auto choice = chooseEscape(dirs, scores, bad0_, current_max, choice_);

  std::string resumo;
  for (std::size_t i = 0; i < dirs.size(); ++i) {
    resumo += dirs[i].name + (scores[i].blocked ?
      "=X@" + std::to_string(static_cast<int>(scores[i].free_len * 100)) + "cm " :
      "=" + std::to_string(static_cast<int>(scores[i].end_cost)) +
      "/" + std::to_string(scores[i].end_old_bad) + " ");
  }
  if (!choice) {
    RCLCPP_WARN(
      logger_, "[%s] NENHUMA direcao segura (%d celulas ruins sob o robo, custo max %.0f): "
      "nao vou me mover. %s", behavior_name_.c_str(), bad0_, current_max, resumo.c_str());
    return ResultStatus{Status::FAILED, ActionT::Result::COLLISION_AHEAD};
  }
  const auto & d = dirs[choice->index];
  body_dx_ = d.dx;
  body_dy_ = d.dy;
  dir_name_ = d.name;
  RCLCPP_INFO(
    logger_, "[%s] %d ruins sob o robo, custo max %.0f -> '%s' (%.2f m a %.2f m/s; no fim: "
    "custo %.0f, ruins antigas %d). %s", behavior_name_.c_str(), bad0_, current_max,
    d.name.c_str(), cmd_dist_, cmd_speed_, choice->score.end_cost, choice->score.end_old_bad,
    resumo.c_str());
  return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE};
}

EscapeFreeSpace::ResultStatus EscapeFreeSpace::onCycleUpdate()
{
  const rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "[%s] tempo esgotado", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, ActionT::Result::TIMEOUT};
  }

  geometry_msgs::msg::PoseStamped cur;
  if (!currentPose(cur, local_frame_)) {
    stopRobot();
    return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR};
  }
  const double x = cur.pose.position.x;
  const double y = cur.pose.position.y;
  const double yaw = tf2::getYaw(cur.pose.orientation);
  const double traveled = std::hypot(
    x - initial_pose_.pose.position.x, y - initial_pose_.pose.position.y);
  feedback_->distance_traveled = traveled;
  action_server_->publish_feedback(feedback_);

  if (traveled >= cmd_dist_) {
    stopRobot();
    RCLCPP_INFO(logger_, "[%s] andou %.2f m para '%s'", behavior_name_.c_str(),
      traveled, dir_name_.c_str());
    return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE};
  }

  // Costmaps frescos (o subscriber guarda o ultimo recebido).
  try {
    local_cm_ = local_sub_->getCostmap();
    if (use_global_ && global_sub_) {
      global_cm_ = global_sub_->getCostmap();
    }
  } catch (const std::exception & e) {
    // mantem o ultimo que tinha
  }

  // Antevisao de colisao ao longo da direcao escolhida (frame odom).
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double wx = body_dx_ * c - body_dy_ * s;
  const double wy = body_dx_ * s + body_dy_ * c;
  const double remaining = cmd_dist_ - traveled;
  const int max_cycles = std::max(1, static_cast<int>(cycle_frequency_ * simulate_ahead_time_));
  PoseEval here;
  for (int k = 0; k < max_cycles; ++k) {
    const double along = std::min(remaining, cmd_speed_ * (k / cycle_frequency_));
    const PoseEval e = evalPose(x + wx * along, y + wy * along, yaw);
    if (k == 0) {
      here = e;
    }
    if (e.new_bad > 0) {
      stopRobot();
      RCLCPP_WARN(logger_, "[%s] celula ruim nova a frente em '%s' apos %.2f m: parando",
        behavior_name_.c_str(), dir_name_.c_str(), traveled);
      return ResultStatus{Status::FAILED, ActionT::Result::COLLISION_AHEAD};
    }
    if (along >= remaining) {
      break;
    }
  }
  // Ja saiu da zona ruim: nao precisa completar o curso.
  if (traveled >= min_travel_for_exit_ && here.old_bad == 0 && here.max_cost < exit_cost_) {
    stopRobot();
    RCLCPP_INFO(logger_, "[%s] limpo (custo max %.0f) apos %.2f m em '%s': saiu",
      behavior_name_.c_str(), here.max_cost, traveled, dir_name_.c_str());
    return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE};
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = clock_->now();
  cmd_vel->header.frame_id = robot_base_frame_;
  cmd_vel->twist.linear.x = cmd_speed_ * body_dx_;
  cmd_vel->twist.linear.y = cmd_speed_ * body_dy_;
  cmd_vel->twist.angular.z = 0.0;
  vel_pub_->publish(std::move(cmd_vel));
  return ResultStatus{Status::RUNNING, ActionT::Result::NONE};
}

}  // namespace caramelo_behaviors

PLUGINLIB_EXPORT_CLASS(caramelo_behaviors::EscapeFreeSpace, nav2_core::Behavior)
