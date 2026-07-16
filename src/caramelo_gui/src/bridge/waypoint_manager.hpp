#pragma once

// Gerenciador de waypoints por mapa: cria/move (Interactive Marker no mapa),
// persiste em maps/<mapa>/waypoints.yaml e alimenta o "seguir todos"
// (FollowWaypoints). Rotas topologicas continuam OPCIONAIS (padrao do robo e'
// o planejamento livre do Nav2) — isto e' so uma lista ordenada de pontos.

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QObject>
#include <QString>
#include <QStringList>

#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class WaypointManager : public QObject
{
  Q_OBJECT

public:
  explicit WaypointManager(rclcpp::Node::SharedPtr node, QObject * parent = nullptr);

  void load(const QString & map_dir);
  void save(const QString & map_dir);
  void add(const QString & name);       // na pose atual do robo (TF) ou origem
  void remove(const QString & name);
  QStringList names() const;
  std::vector<std::array<double, 3>> orderedPoses() const;  // x, y, yaw
  bool pose(const QString & name, double & x, double & y, double & yaw) const;

signals:
  void waypointsChanged(const QStringList & names);
  void status(const QString & message);

private:
  void insertMarker(const std::string & name, double x, double y, double yaw);
  void onFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & fb);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mtx_;
  // Ordenado por nome (WP1, WP2, ...) — a ordem de "seguir todos".
  std::map<std::string, std::array<double, 3>> points_;
};
