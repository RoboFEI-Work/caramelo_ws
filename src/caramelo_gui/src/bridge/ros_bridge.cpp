#include "bridge/ros_bridge.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include "bridge/manual_localization.hpp"

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

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::QoS(1),
    std::bind(&RosBridge::onGoalPose, this, std::placeholders::_1));
  clear_global_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/global_costmap/clear_entirely_global_costmap");
  clear_local_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/local_costmap/clear_entirely_local_costmap");
  initialpose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(1));

  dock_client_ = rclcpp_action::create_client<DockRobot>(node_, "/dock_robot");
  undock_client_ = rclcpp_action::create_client<UndockRobot>(node_, "/undock_robot");
  align_client_ = rclcpp_action::create_client<AlignToDock>(node_, "align_to_dock");
  save_dock_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/save_dock_pose", rclcpp::QoS(1));

  load_map_client_ = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
  save_map_client_ = node_->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");

  manual_loc_ = new ManualLocalization(node_, this);
}

// ------------------------------------------------------------- Mapas
void RosBridge::loadMap(const QString & yaml_path)
{
  if (!load_map_client_->service_is_ready()) {
    emit mapStatus(false, "map_server indisponivel (suba a localizacao)");
    return;
  }
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  req->map_url = yaml_path.toStdString();
  load_map_client_->async_send_request(
    req, [this, yaml_path](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture fut) {
      const auto resp = fut.get();
      const bool ok = resp &&
        resp->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
      emit mapStatus(
        ok, ok ? QString("Mapa carregado: %1").arg(yaml_path)
        : "Falha ao carregar o mapa");
    });
  emit mapStatus(true, "Carregando mapa...");
}

void RosBridge::saveMap(const QString & dir, const QString & name)
{
  if (!save_map_client_->service_is_ready()) {
    emit mapStatus(false, "map_saver indisponivel (rodando o mapeamento?)");
    return;
  }
  auto req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  req->map_topic = "map";
  req->map_url = (dir + "/" + name + "/map").toStdString();
  req->image_format = "pgm";
  req->map_mode = "trinary";
  req->free_thresh = 0.196f;
  req->occupied_thresh = 0.65f;
  save_map_client_->async_send_request(
    req, [this](rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedFuture fut) {
      const auto resp = fut.get();
      const bool ok = resp && resp->result;
      emit mapStatus(ok, ok ? "Mapa salvo" : "Falha ao salvar o mapa");
    });
  emit mapStatus(true, "Salvando mapa...");
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

// ------------------------------------------------------------- Navegacao
void RosBridge::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  sendNavGoal(*msg);
}

void RosBridge::sendNavGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!nav_client_->action_server_is_ready()) {
    emit navResult(false, "Servidor navigate_to_pose indisponivel");
    return;
  }
  NavigateToPose::Goal goal;
  goal.pose = pose;
  if (goal.pose.header.frame_id.empty()) {
    goal.pose.header.frame_id = "map";
  }

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr handle) {
      {
        std::lock_guard<std::mutex> lk(nav_mtx_);
        nav_goal_handle_ = handle;
      }
      emit navStatus(handle ? "Meta aceita — navegando" : "Meta rejeitada");
    };
  opts.feedback_callback =
    [this](
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      emit navStatus(
        QString("Navegando — %1 m restantes")
        .arg(static_cast<double>(fb->distance_remaining), 0, 'f', 2));
    };
  opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
      const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
      QString msg;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: msg = "Chegou ao destino"; break;
        case rclcpp_action::ResultCode::ABORTED: msg = "Navegacao abortada"; break;
        case rclcpp_action::ResultCode::CANCELED: msg = "Navegacao cancelada"; break;
        default: msg = "Navegacao terminou"; break;
      }
      emit navResult(ok, msg);
    };
  nav_client_->async_send_goal(goal, opts);
  emit navStatus("Enviando meta...");
}

void RosBridge::cancelNav()
{
  std::lock_guard<std::mutex> lk(nav_mtx_);
  if (nav_goal_handle_) {
    nav_client_->async_cancel_goal(nav_goal_handle_);
    emit navStatus("Cancelando...");
  } else {
    emit navStatus("Nada para cancelar");
  }
}

void RosBridge::clearCostmaps()
{
  if (clear_global_->service_is_ready()) {
    clear_global_->async_send_request(
      std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());
  }
  if (clear_local_->service_is_ready()) {
    clear_local_->async_send_request(
      std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());
  }
  emit navStatus("Costmaps limpos");
}

// ------------------------------------------------------------- Localizacao
void RosBridge::publishInitialPose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
  msg.pose.covariance[0] = 0.25;    // x
  msg.pose.covariance[7] = 0.25;    // y
  msg.pose.covariance[35] = 0.0685; // yaw
  initialpose_pub_->publish(msg);
}

// ------------------------------------------------------------- Docking
void RosBridge::sendDock(const QString & dock_id)
{
  if (!dock_client_->action_server_is_ready()) {
    emit dockResult(false, "Servidor /dock_robot indisponivel");
    return;
  }
  DockRobot::Goal goal;
  goal.use_dock_id = true;
  goal.dock_id = dock_id.toStdString();
  goal.navigate_to_staging_pose = true;
  goal.max_staging_time = 120.0;

  rclcpp_action::Client<DockRobot>::SendGoalOptions opts;
  opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<DockRobot>::WrappedResult & r) {
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
        r.result->success;
      emit dockResult(ok, ok ? "Dock concluido" : "Dock falhou");
    };
  dock_client_->async_send_goal(goal, opts);
  emit dockStatus(QString("Dockando em %1...").arg(dock_id));
}

void RosBridge::sendUndock(const QString & dock_type)
{
  if (!undock_client_->action_server_is_ready()) {
    emit dockResult(false, "Servidor /undock_robot indisponivel");
    return;
  }
  UndockRobot::Goal goal;
  goal.dock_type = dock_type.toStdString();
  goal.max_undocking_time = 30.0;

  rclcpp_action::Client<UndockRobot>::SendGoalOptions opts;
  opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<UndockRobot>::WrappedResult & r) {
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
        r.result->success;
      emit dockResult(ok, ok ? "Undock concluido" : "Undock falhou");
    };
  undock_client_->async_send_goal(goal, opts);
  emit dockStatus("Undock...");
}

void RosBridge::sendAlign(const QString & dock_id, const QString & map_name, bool use_lidar_refine)
{
  if (!align_client_->action_server_is_ready()) {
    emit dockResult(false, "Servidor align_to_dock indisponivel");
    return;
  }
  AlignToDock::Goal goal;
  goal.dock_id = dock_id.toStdString();
  goal.map_name = map_name.toStdString();
  goal.use_lidar_refine = use_lidar_refine;
  goal.timeout = 30.0;

  rclcpp_action::Client<AlignToDock>::SendGoalOptions opts;
  opts.feedback_callback =
    [this](
    rclcpp_action::ClientGoalHandle<AlignToDock>::SharedPtr,
    const std::shared_ptr<const AlignToDock::Feedback> fb) {
      emit dockStatus(QString::fromStdString(fb->phase));
    };
  opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<AlignToDock>::WrappedResult & r) {
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
        r.result->success;
      emit dockResult(
        ok, ok ? "Alinhamento fino concluido" :
        QString::fromStdString(r.result ? r.result->message : "falhou"));
    };
  align_client_->async_send_goal(goal, opts);
  emit dockStatus(QString("Alinhando em %1...").arg(dock_id));
}

void RosBridge::saveDockPose(const QString & dock_id)
{
  std_msgs::msg::String msg;
  msg.data = dock_id.toStdString();
  save_dock_pub_->publish(msg);
  emit dockStatus(QString("Pose atual salva como dock %1").arg(dock_id));
}
