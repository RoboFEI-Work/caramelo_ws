#include "bridge/waypoint_manager.hpp"

#include <cmath>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::InteractiveMarkerFeedback;
using visualization_msgs::msg::Marker;

namespace
{
double yawFromQuat(double z, double w, double x = 0.0, double y = 0.0)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

WaypointManager::WaypointManager(rclcpp::Node::SharedPtr node, QObject * parent)
: QObject(parent), node_(node)
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "waypoints", node_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void WaypointManager::insertMarker(const std::string & name, double x, double y, double yaw)
{
  InteractiveMarker im;
  im.header.frame_id = "map";
  im.name = name;
  im.description = name;
  im.scale = 0.6;
  im.pose.position.x = x;
  im.pose.position.y = y;
  im.pose.orientation.z = std::sin(yaw / 2.0);
  im.pose.orientation.w = std::cos(yaw / 2.0);

  Marker esfera;
  esfera.type = Marker::SPHERE;
  esfera.scale.x = esfera.scale.y = esfera.scale.z = 0.18;
  esfera.color.r = 0.1f;
  esfera.color.g = 0.5f;
  esfera.color.b = 1.0f;
  esfera.color.a = 0.9f;

  Marker seta;
  seta.type = Marker::ARROW;
  seta.scale.x = 0.35;
  seta.scale.y = 0.05;
  seta.scale.z = 0.05;
  seta.color.r = 0.1f;
  seta.color.g = 0.5f;
  seta.color.b = 1.0f;
  seta.color.a = 0.9f;

  // Corpo DENTRO do controle MOVE_PLANE: arrasta clicando na propria esfera.
  InteractiveMarkerControl move;
  move.name = "mover_xy";
  move.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  move.orientation.w = 1.0;
  move.orientation.y = 1.0;
  move.always_visible = true;
  move.markers.push_back(esfera);
  move.markers.push_back(seta);
  im.controls.push_back(move);

  InteractiveMarkerControl gira;
  gira.name = "girar_yaw";
  gira.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  gira.orientation.w = 1.0;
  gira.orientation.y = 1.0;
  im.controls.push_back(gira);

  server_->insert(
    im, std::bind(&WaypointManager::onFeedback, this, std::placeholders::_1));
}

void WaypointManager::onFeedback(
  const InteractiveMarkerFeedback::ConstSharedPtr & fb)
{
  if (fb->event_type != InteractiveMarkerFeedback::POSE_UPDATE) {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = points_.find(fb->marker_name);
  if (it != points_.end()) {
    const auto & q = fb->pose.orientation;
    it->second = {fb->pose.position.x, fb->pose.position.y,
      yawFromQuat(q.z, q.w, q.x, q.y)};
  }
}

void WaypointManager::add(const QString & name)
{
  double x = 0.0, y = 0.0, yaw = 0.0;
  try {
    const auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    const auto & q = tf.transform.rotation;
    yaw = yawFromQuat(q.z, q.w, q.x, q.y);
  } catch (...) {
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    points_[name.toStdString()] = {x, y, yaw};
  }
  insertMarker(name.toStdString(), x, y, yaw);
  server_->applyChanges();
  emit waypointsChanged(names());
  emit status(QString("Waypoint %1 criado — arraste para ajustar").arg(name));
}

void WaypointManager::remove(const QString & name)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    points_.erase(name.toStdString());
  }
  server_->erase(name.toStdString());
  server_->applyChanges();
  emit waypointsChanged(names());
}

QStringList WaypointManager::names() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  QStringList out;
  for (const auto & kv : points_) {
    out << QString::fromStdString(kv.first);
  }
  return out;
}

std::vector<std::array<double, 3>> WaypointManager::orderedPoses() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  std::vector<std::array<double, 3>> out;
  out.reserve(points_.size());
  for (const auto & kv : points_) {
    out.push_back(kv.second);
  }
  return out;
}

bool WaypointManager::pose(const QString & name, double & x, double & y, double & yaw) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  const auto it = points_.find(name.toStdString());
  if (it == points_.end()) {
    return false;
  }
  x = it->second[0];
  y = it->second[1];
  yaw = it->second[2];
  return true;
}

void WaypointManager::load(const QString & map_dir)
{
  server_->clear();
  std::map<std::string, std::array<double, 3>> novos;
  const std::string path = (map_dir + "/waypoints.yaml").toStdString();
  try {
    YAML::Node root = YAML::LoadFile(path);
    for (const auto & kv : root["waypoints"]) {
      const auto name = kv.first.as<std::string>();
      novos[name] = {
        kv.second["x"].as<double>(0.0),
        kv.second["y"].as<double>(0.0),
        kv.second["yaw"].as<double>(0.0)};
    }
  } catch (...) {
    // Sem arquivo ainda: lista vazia (normal em mapa novo).
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    points_ = novos;
  }
  for (const auto & kv : novos) {
    insertMarker(kv.first, kv.second[0], kv.second[1], kv.second[2]);
  }
  server_->applyChanges();
  emit waypointsChanged(names());
  emit status(QString("%1 waypoint(s) carregado(s)").arg(novos.size()));
}

void WaypointManager::save(const QString & map_dir)
{
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "waypoints" << YAML::Value << YAML::BeginMap;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto & kv : points_) {
      out << YAML::Key << kv.first << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << kv.second[0];
      out << YAML::Key << "y" << YAML::Value << kv.second[1];
      out << YAML::Key << "yaw" << YAML::Value << kv.second[2];
      out << YAML::EndMap;
    }
  }
  out << YAML::EndMap << YAML::EndMap;

  std::ofstream file((map_dir + "/waypoints.yaml").toStdString());
  if (!file) {
    emit status("Falha ao salvar waypoints.yaml");
    return;
  }
  file << out.c_str() << "\n";
  emit status("waypoints.yaml salvo");
}
