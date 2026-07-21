#include "bridge/manual_localization.hpp"

#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::InteractiveMarkerFeedback;
using visualization_msgs::msg::Marker;

namespace
{
constexpr char kMarkerName[] = "robo_fantasma";

double yawFromQuat(double z, double w, double x = 0.0, double y = 0.0)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

ManualLocalization::ManualLocalization(rclcpp::Node::SharedPtr node, QObject * parent)
: QObject(parent), node_(node)
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "localizacao_manual", node_);
  preview_pub_ = node_->create_publisher<Marker>(
    "/localizacao_manual/scan_preview", rclcpp::QoS(1));
  initialpose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(1));
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&ManualLocalization::onScan, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

bool ManualLocalization::currentRobotPose(double & x, double & y, double & yaw)
{
  try {
    const auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    const auto & q = tf.transform.rotation;
    yaw = yawFromQuat(q.z, q.w, q.x, q.y);
    return true;
  } catch (...) {
    return false;
  }
}

void ManualLocalization::start()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!currentRobotPose(gx_, gy_, gyaw_)) {
    gx_ = gy_ = gyaw_ = 0.0;  // sem TF map->base: comeca na origem do mapa
  }

  InteractiveMarker im;
  im.header.frame_id = "map";
  im.name = kMarkerName;
  im.description = "Arraste e gire; depois Confirmar";
  im.scale = 1.0;
  im.pose.position.x = gx_;
  im.pose.position.y = gy_;
  im.pose.orientation.z = std::sin(gyaw_ / 2.0);
  im.pose.orientation.w = std::cos(gyaw_ / 2.0);

  // Visual: footprint do robo (caixa laranja translucida) + seta de frente.
  Marker corpo;
  corpo.type = Marker::CUBE;
  corpo.scale.x = 0.71;
  corpo.scale.y = 0.47;
  corpo.scale.z = 0.05;
  corpo.color.r = 1.0f;
  corpo.color.g = 0.55f;
  corpo.color.b = 0.0f;
  corpo.color.a = 0.55f;

  Marker seta;
  seta.type = Marker::ARROW;
  seta.scale.x = 0.45;
  seta.scale.y = 0.06;
  seta.scale.z = 0.06;
  seta.pose.position.z = 0.08;
  seta.color.r = 1.0f;
  seta.color.g = 0.3f;
  seta.color.b = 0.0f;
  seta.color.a = 0.95f;

  // O corpo fica DENTRO do controle MOVE_PLANE: assim arrasta-se clicando no
  // proprio fantasma (controle sem markers nao tem area clicavel — era o bug
  // de "nao consigo arrastar o robo").
  InteractiveMarkerControl move;
  move.name = "mover_xy";
  move.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  move.orientation.w = 1.0;
  move.orientation.y = 1.0;
  move.always_visible = true;
  move.markers.push_back(corpo);
  move.markers.push_back(seta);
  im.controls.push_back(move);

  // Girar em yaw (em torno de Z).
  InteractiveMarkerControl gira;
  gira.name = "girar_yaw";
  gira.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  gira.orientation.w = 1.0;
  gira.orientation.y = 1.0;
  im.controls.push_back(gira);

  server_->insert(
    im, std::bind(&ManualLocalization::onFeedback, this, std::placeholders::_1));
  server_->applyChanges();

  active_ = true;
  emit activeChanged(true);
  emit stateChanged("Arraste o fantasma ate o scan casar com o mapa; depois Confirmar.");
  emit ghostMoved(gx_, gy_, gyaw_);
}

void ManualLocalization::onFeedback(
  const InteractiveMarkerFeedback::ConstSharedPtr & fb)
{
  if (fb->event_type != InteractiveMarkerFeedback::POSE_UPDATE) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    gx_ = fb->pose.position.x;
    gy_ = fb->pose.position.y;
    const auto & q = fb->pose.orientation;
    gyaw_ = yawFromQuat(q.z, q.w, q.x, q.y);
  }
  publishPreview();
  emit ghostMoved(gx_, gy_, gyaw_);
}

void ManualLocalization::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_scan_ = msg;
    if (!active_) {
      return;
    }
  }
  publishPreview();
}

void ManualLocalization::publishPreview()
{
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  double gx, gy, gyaw;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!active_ || !last_scan_) {
      return;
    }
    scan = last_scan_;
    gx = gx_;
    gy = gy_;
    gyaw = gyaw_;
  }

  // Pose do laser relativa a base (TF estatica base_footprint -> laser).
  // ATENCAO: o LiDAR do Caramelo monta DE CABECA PARA BAIXO (RPY 180,0,180 no
  // URDF). Extrair "so o yaw" do quaternion (como era feito) vira uma rotacao
  // plana — mas a projecao 2D de um laser invertido e um ESPELHAMENTO, nao
  // uma rotacao: o preview saia espelhado. Usamos a base 3D completa da
  // rotacao projetada no plano XY (correta p/ qualquer montagem).
  // Fallback = a montagem real do Caramelo: (x,y)_base = (-x, +y)_laser.
  double lx = 0.0, ly = 0.0;
  double exx = -1.0, exy = 0.0;  // imagem do eixo X do laser no plano da base
  double eyx = 0.0, eyy = 1.0;   // imagem do eixo Y do laser no plano da base
  try {
    const auto tf = tf_buffer_->lookupTransform(
      "base_footprint", scan->header.frame_id, tf2::TimePointZero);
    lx = tf.transform.translation.x;
    ly = tf.transform.translation.y;
    const auto & q = tf.transform.rotation;
    // Colunas X e Y da matriz de rotacao do quaternion (projetadas em XY).
    exx = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    exy = 2.0 * (q.x * q.y + q.w * q.z);
    eyx = 2.0 * (q.x * q.y - q.w * q.z);
    eyy = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
  } catch (const std::exception & e) {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "Preview do fantasma sem TF base_footprint->%s (%s); usando a montagem "
      "conhecida do laser do Caramelo (invertido, RPY 180/0/180).",
      scan->header.frame_id.c_str(), e.what());
  }

  Marker m;
  m.header.frame_id = "map";
  m.header.stamp = node_->now();
  m.ns = "scan_fantasma";
  m.id = 0;
  m.type = Marker::POINTS;
  m.action = Marker::ADD;
  m.scale.x = 0.04;
  m.scale.y = 0.04;
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.3f;
  m.color.a = 0.9f;

  const double cg = std::cos(gyaw), sg = std::sin(gyaw);
  double angle = scan->angle_min;
  for (const float r : scan->ranges) {
    const double a = angle;
    angle += scan->angle_increment;
    if (!std::isfinite(r) || r <= scan->range_min || r >= scan->range_max) {
      continue;
    }
    // ponto no frame do laser -> base (rotacao 3D projetada) -> mapa
    const double pxl = r * std::cos(a);
    const double pyl = r * std::sin(a);
    const double pxb = lx + exx * pxl + eyx * pyl;
    const double pyb = ly + exy * pxl + eyy * pyl;
    geometry_msgs::msg::Point p;
    p.x = gx + cg * pxb - sg * pyb;
    p.y = gy + sg * pxb + cg * pyb;
    p.z = 0.05;
    m.points.push_back(p);
  }
  preview_pub_->publish(m);
}

void ManualLocalization::clearPreview()
{
  Marker m;
  m.header.frame_id = "map";
  m.header.stamp = node_->now();
  m.ns = "scan_fantasma";
  m.id = 0;
  m.action = Marker::DELETEALL;
  preview_pub_->publish(m);
}

void ManualLocalization::confirm()
{
  double gx, gy, gyaw;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!active_) {
      return;
    }
    gx = gx_;
    gy = gy_;
    gyaw = gyaw_;
    active_ = false;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();
  msg.pose.pose.position.x = gx;
  msg.pose.pose.position.y = gy;
  msg.pose.pose.orientation.z = std::sin(gyaw / 2.0);
  msg.pose.pose.orientation.w = std::cos(gyaw / 2.0);
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.0685;
  initialpose_pub_->publish(msg);

  server_->erase(kMarkerName);
  server_->applyChanges();
  clearPreview();
  emit activeChanged(false);
  emit stateChanged("Pose confirmada — AMCL relocalizando.");
}

void ManualLocalization::cancel()
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!active_) {
      return;
    }
    active_ = false;
  }
  server_->erase(kMarkerName);
  server_->applyChanges();
  clearPreview();
  emit activeChanged(false);
  emit stateChanged("Localizacao manual cancelada.");
}
