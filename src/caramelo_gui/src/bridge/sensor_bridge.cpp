#include "bridge/sensor_bridge.hpp"

#include <cmath>
#include <functional>

namespace
{
constexpr double kRad2Deg = 180.0 / M_PI;
}  // namespace

double SensorBridge::Frequencimetro::registrar(const rclcpp::Time & agora)
{
  const double t = agora.seconds();
  if (ultimo_ > 0.0 && t > ultimo_) {
    const double instantanea = 1.0 / (t - ultimo_);
    // Media movel: sem ela o numero pisca e o operador nao consegue ler.
    hz_ = (hz_ <= 0.0) ? instantanea : (hz_ * 0.8 + instantanea * 0.2);
  }
  ultimo_ = t;
  return hz_;
}

SensorBridge::SensorBridge(rclcpp::Node::SharedPtr node, QObject * parent)
: QObject(parent), node_(std::move(node))
{
  qRegisterMetaType<LeituraLidar>("LeituraLidar");
  qRegisterMetaType<LeituraImu>("LeituraImu");
  qRegisterMetaType<QuadroCamera>("QuadroCamera");
}

QStringList SensorBridge::topicosDeImagem() const
{
  QStringList achados;
  for (const auto & [nome, tipos] : node_->get_topic_names_and_types()) {
    for (const auto & tipo : tipos) {
      if (tipo == "sensor_msgs/msg/Image") {
        achados << QString::fromStdString(nome);
        break;
      }
    }
  }
  achados.sort();
  return achados;
}

bool SensorBridge::topicoExiste(const QString & topico) const
{
  const auto todos = node_->get_topic_names_and_types();
  return todos.count(topico.toStdString()) > 0;
}

void SensorBridge::assinarLidar(const QString & topico)
{
  hz_lidar_.zerar();
  sub_lidar_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    topico.toStdString(), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      LeituraLidar l;
      l.angulo_min = msg->angle_min;
      l.angulo_max = msg->angle_max;
      l.incremento = msg->angle_increment;
      l.alcance_min = msg->range_min;
      l.alcance_max = msg->range_max;
      l.distancias.reserve(static_cast<int>(msg->ranges.size()));
      for (const float r : msg->ranges) {
        l.distancias.push_back(r);
        if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
          ++l.validos;
        }
      }
      l.hz = hz_lidar_.registrar(node_->now());
      emit lidarRecebido(l);
    });
}

void SensorBridge::pararLidar()
{
  sub_lidar_.reset();
  hz_lidar_.zerar();
}

void SensorBridge::assinarImu(const QString & topico)
{
  hz_imu_.zerar();
  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    topico.toStdString(), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      LeituraImu i;
      // Quaternion -> roll/pitch/yaw sem puxar tf2 so' por isto.
      const auto & q = msg->orientation;
      const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
      const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
      i.roll = std::atan2(sinr, cosr) * kRad2Deg;
      double sinp = 2.0 * (q.w * q.y - q.z * q.x);
      sinp = std::max(-1.0, std::min(1.0, sinp));
      i.pitch = std::asin(sinp) * kRad2Deg;
      const double siny = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      i.yaw = std::atan2(siny, cosy) * kRad2Deg;

      i.giro_z = msg->angular_velocity.z;
      i.acel_x = msg->linear_acceleration.x;
      i.acel_y = msg->linear_acceleration.y;
      i.acel_z = msg->linear_acceleration.z;
      i.hz = hz_imu_.registrar(node_->now());
      emit imuRecebido(i);
    });
}

void SensorBridge::pararImu()
{
  sub_imu_.reset();
  hz_imu_.zerar();
}

void SensorBridge::assinarCamera(const QString & topico)
{
  hz_camera_.zerar();
  sub_camera_ = node_->create_subscription<sensor_msgs::msg::Image>(
    topico.toStdString(), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      QuadroCamera q;
      q.formato = QString::fromStdString(msg->encoding);
      q.hz = hz_camera_.registrar(node_->now());

      const int w = static_cast<int>(msg->width);
      const int h = static_cast<int>(msg->height);
      const int passo = static_cast<int>(msg->step);

      QImage img;
      if (msg->encoding == "rgb8") {
        img = QImage(msg->data.data(), w, h, passo, QImage::Format_RGB888);
      } else if (msg->encoding == "bgr8") {
        img = QImage(msg->data.data(), w, h, passo, QImage::Format_RGB888).rgbSwapped();
      } else if (msg->encoding == "mono8") {
        img = QImage(msg->data.data(), w, h, passo, QImage::Format_Grayscale8);
      } else if (msg->encoding == "rgba8") {
        img = QImage(msg->data.data(), w, h, passo, QImage::Format_RGBA8888);
      }
      // QImage sobre buffer emprestado nao possui os bytes: sem a copia, o dado
      // e' liberado com a mensagem antes de a thread da UI desenhar.
      q.imagem = img.isNull() ? QImage() : img.copy();
      emit cameraRecebida(q);
    });
}

void SensorBridge::pararCamera()
{
  sub_camera_.reset();
  hz_camera_.zerar();
}
