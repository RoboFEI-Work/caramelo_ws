#include "bridge/ros_bridge.hpp"

#include <chrono>
#include <functional>

RosBridge::RosBridge(QObject * parent)
: QObject(parent)
{
  // Metatypes para os sinais atravessarem a fronteira thread ROS -> thread UI.
  qRegisterMetaType<ComponentHealth>("ComponentHealth");
  qRegisterMetaType<QVector<ComponentHealth>>("QVector<ComponentHealth>");

  node_ = std::make_shared<rclcpp::Node>("caramelo_gui_bridge");
  diag_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10),
    std::bind(&RosBridge::onDiagnostics, this, std::placeholders::_1));
}

RosBridge::~RosBridge()
{
  stop();
}

void RosBridge::start()
{
  if (running_.exchange(true)) {
    return;
  }
  spin_thread_ = std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    while (running_.load() && rclcpp::ok()) {
      exec.spin_some(std::chrono::milliseconds(50));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    exec.remove_node(node_);
  });
}

void RosBridge::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void RosBridge::onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  QVector<ComponentHealth> health;
  health.reserve(static_cast<int>(msg->status.size()));
  for (const auto & st : msg->status) {
    ComponentHealth c;
    c.name = QString::fromStdString(st.name);
    c.level = static_cast<int>(st.level);
    c.message = QString::fromStdString(st.message);
    for (const auto & kv : st.values) {
      c.values.insert(QString::fromStdString(kv.key), QString::fromStdString(kv.value));
    }
    health.push_back(c);
  }
  emit diagnosticsUpdated(health);
}
