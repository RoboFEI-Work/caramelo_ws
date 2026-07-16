#pragma once

// Ponte entre o grafo ROS 2 e a UI Qt. Roda um rclcpp::Node numa thread propria
// e converte mensagens em sinais Qt (entregues na thread da UI por conexao
// enfileirada). Widgets NUNCA falam ROS direto — so' via este RosBridge.

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <QObject>
#include <QString>
#include <QVector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "caramelo_msgs/action/align_to_dock.hpp"

#include "bridge/health_types.hpp"

class ManualLocalization;

class RosBridge : public QObject
{
  Q_OBJECT

public:
  explicit RosBridge(QObject * parent = nullptr);
  ~RosBridge() override;

  void start();
  void stop();

  // --- Navegacao ---
  // O goal chega pela ferramenta "2D Goal" do RViz (/goal_pose); o Bridge o
  // encaminha para a action navigate_to_pose. Cancelar/limpar sao chamados
  // pelos botoes do modulo de Navegacao.
  void cancelNav();
  void clearCostmaps();

  // --- Localizacao ---
  void publishInitialPose(double x, double y, double yaw);
  ManualLocalization * manualLocalization() {return manual_loc_;}

  // --- Docking ---
  void sendDock(const QString & dock_id);
  void sendUndock(const QString & dock_type);
  void sendAlign(const QString & dock_id, const QString & map_name, bool use_lidar_refine);
  void saveDockPose(const QString & dock_id);

signals:
  void diagnosticsUpdated(const QVector<ComponentHealth> & health);
  void navStatus(const QString & message);
  void navResult(bool success, const QString & message);
  void dockStatus(const QString & message);
  void dockResult(bool success, const QString & message);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using DockRobot = nav2_msgs::action::DockRobot;
  using UndockRobot = nav2_msgs::action::UndockRobot;
  using AlignToDock = caramelo_msgs::action::AlignToDock;

  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void sendNavGoal(const geometry_msgs::msg::PoseStamped & pose);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

  rclcpp_action::Client<DockRobot>::SharedPtr dock_client_;
  rclcpp_action::Client<UndockRobot>::SharedPtr undock_client_;
  rclcpp_action::Client<AlignToDock>::SharedPtr align_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr save_dock_pub_;

  ManualLocalization * manual_loc_ = nullptr;

  std::mutex nav_mtx_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_handle_;

  std::thread spin_thread_;
  std::atomic<bool> running_{false};
};
