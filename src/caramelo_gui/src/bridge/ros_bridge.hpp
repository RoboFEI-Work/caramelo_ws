#pragma once

// Ponte entre o grafo ROS 2 e a UI Qt. Roda um rclcpp::Node numa thread propria
// e converte mensagens em sinais Qt (entregues na thread da UI por conexao
// enfileirada). Widgets NUNCA falam ROS direto — so' via este RosBridge.

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <QObject>
#include <QString>
#include <QVector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "caramelo_msgs/action/align_to_dock.hpp"
#include "caramelo_msgs/action/save_service_area_pose.hpp"
#include "caramelo_msgs/srv/list_service_areas.hpp"
#include "caramelo_msgs/srv/validate_service_areas.hpp"

#include "bridge/health_types.hpp"

class ManualLocalization;
class WaypointManager;
class MissionBridge;
class RobotState;
class SensorBridge;

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
  void goTo(double x, double y, double yaw);   // meta direta (waypoints)
  void followWaypoints(const std::vector<std::array<double, 3>> & poses);

  // --- Localizacao ---
  void publishInitialPose(double x, double y, double yaw);
  ManualLocalization * manualLocalization() {return manual_loc_;}
  WaypointManager * waypoints() {return waypoints_;}

  // --- Missao (action /caramelo/run_mission) ---
  // Dominio proprio, em arquivo proprio: ver bridge/mission_bridge.hpp.
  MissionBridge * mission() {return mission_;}

  // --- Estado persistente do robo (mapa ativo) ---
  // Nao e' ROS, mas e' estado do robo, e a regra do projeto e' que os widgets
  // falem com uma fachada so'. Ver bridge/robot_state.hpp.
  RobotState * robotState() {return robot_state_;}

  // --- Sensores brutos (Modo Avancado) ---
  // Assinaturas sob demanda: ver bridge/sensor_bridge.hpp.
  SensorBridge * sensores() {return sensores_;}

  // Um no existe no grafo agora? Serve para a UI parar de oferecer algo que
  // ja' esta rodando -- ligar um segundo Nav2 poe dois controladores
  // disputando a mesma base.
  bool noAtivo(const QString & nome) const;

  // Melhor fixed frame disponivel: map (localizado) > odom > base_footprint.
  QString bestFixedFrame() const;

  // --- Mapas ---
  void loadMap(const QString & yaml_path);              // /map_server/load_map
  void saveMap(const QString & dir, const QString & name);  // /map_saver/save_map

  // --- Service Areas (API do service_area_manager_node) ---
  void listServiceAreas(const QString & map_name);
  void validateServiceAreas(const QString & map_name);
  void saveServiceAreaPose(
    const QString & map_name, const QString & area_id, const QString & area_type);

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
  void mapStatus(bool success, const QString & message);
  void serviceAreasListed(const QStringList & linhas);
  void serviceAreaStatus(bool success, const QString & message);
  void logLine(const QString & line);   // /rosout (logs dos nos), p/ painel oculto

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
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

  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;

  using SaveServiceAreaPose = caramelo_msgs::action::SaveServiceAreaPose;
  rclcpp::Client<caramelo_msgs::srv::ListServiceAreas>::SharedPtr sa_list_client_;
  rclcpp::Client<caramelo_msgs::srv::ValidateServiceAreas>::SharedPtr sa_validate_client_;
  rclcpp_action::Client<SaveServiceAreaPose>::SharedPtr sa_save_client_;

  ManualLocalization * manual_loc_ = nullptr;
  WaypointManager * waypoints_ = nullptr;
  MissionBridge * mission_ = nullptr;
  RobotState * robot_state_ = nullptr;
  SensorBridge * sensores_ = nullptr;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_client_;

  std::mutex nav_mtx_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_handle_;

  std::thread spin_thread_;
  std::atomic<bool> running_{false};
};
