#pragma once

// Ponte entre o grafo ROS 2 e a UI Qt. Roda um rclcpp::Node numa thread propria
// e converte mensagens em sinais Qt (entregues na thread da UI por conexao
// enfileirada). Widgets NUNCA falam ROS direto — so' via este RosBridge.

#include <atomic>
#include <memory>
#include <thread>

#include <QObject>
#include <QVector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "bridge/health_types.hpp"

class RosBridge : public QObject
{
  Q_OBJECT

public:
  explicit RosBridge(QObject * parent = nullptr);
  ~RosBridge() override;

  void start();
  void stop();

signals:
  void diagnosticsUpdated(const QVector<ComponentHealth> & health);

private:
  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
};
