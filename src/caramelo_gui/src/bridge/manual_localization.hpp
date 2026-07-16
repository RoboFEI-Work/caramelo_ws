#pragma once

// Localizacao manual ASSISTIDA (requisito do dono):
//  1. start(): aparece um robo-fantasma (Interactive Marker) no mapa, arrastavel
//     em XY e girando em yaw.
//  2. A cada movimento/scan, o /scan ATUAL e' re-projetado na pose candidata do
//     fantasma e publicado como Marker -> o operador casa o scan contra as
//     paredes do mapa a olho, em tempo real (scan match visual).
//  3. confirm(): publica a pose em /initialpose (mesmo canal do 2D Pose
//     Estimate) -> AMCL relocaliza. cancel(): descarta.
//
// Roda no node do RosBridge (callbacks na thread ROS); sinais Qt levam o
// estado para a UI.

#include <memory>
#include <mutex>

#include <QObject>
#include <QString>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ManualLocalization : public QObject
{
  Q_OBJECT

public:
  explicit ManualLocalization(rclcpp::Node::SharedPtr node, QObject * parent = nullptr);

  void start();
  void confirm();
  void cancel();
  bool isActive() const {return active_;}

signals:
  void ghostMoved(double x, double y, double yaw);
  void stateChanged(const QString & message);

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & fb);
  void publishPreview();
  void clearPreview();
  bool currentRobotPose(double & x, double & y, double & yaw);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr preview_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mtx_;
  bool active_ = false;
  double gx_ = 0.0, gy_ = 0.0, gyaw_ = 0.0;          // pose candidata do fantasma
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;  // ultimo /scan recebido
};
