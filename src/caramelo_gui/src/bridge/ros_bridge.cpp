#include "bridge/ros_bridge.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include "bridge/manual_localization.hpp"
#include "bridge/mission_bridge.hpp"
#include "bridge/robot_state.hpp"
#include "bridge/sensor_bridge.hpp"
#include "bridge/waypoint_manager.hpp"

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
  // Topico PRIVADO da GUI: o bt_navigator do Nav2 Jazzy tambem assina
  // /goal_pose — com o topico compartilhado, um "Definir Goal" disparava DOIS
  // NavigateToPose (um preemptava o outro e o status da GUI mentia).
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/caramelo_gui/goal_pose", rclcpp::QoS(1),
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
  // O node do slam.launch.py chama-se "map_saver_server" -> servico
  // /map_saver_server/save_map (o antigo /map_saver/save_map nao existe).
  save_map_client_ = node_->create_client<nav2_msgs::srv::SaveMap>("/map_saver_server/save_map");

  sa_list_client_ = node_->create_client<caramelo_msgs::srv::ListServiceAreas>(
    "/caramelo/service_areas/list");
  sa_validate_client_ = node_->create_client<caramelo_msgs::srv::ValidateServiceAreas>(
    "/caramelo/service_areas/validate");
  sa_save_client_ = rclcpp_action::create_client<SaveServiceAreaPose>(
    node_, "/caramelo/service_areas/save_pose");

  manual_loc_ = new ManualLocalization(node_, this);
  waypoints_ = new WaypointManager(node_, this);
  mission_ = new MissionBridge(node_, this);
  robot_state_ = new RobotState(this);
  sensores_ = new SensorBridge(node_, this);
  follow_client_ = rclcpp_action::create_client<FollowWaypoints>(node_, "follow_waypoints");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Logs dos nos originais (/rosout) para o painel oculto da GUI. So WARN+
  // e os INFO de outros nos (o proprio caramelo_gui* fica de fora do painel).
  rosout_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::QoS(50),
    [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
      const QString nome = QString::fromStdString(msg->name);
      if (nome.startsWith("caramelo_gui")) {
        return;
      }
      static const QMap<int, QString> niveis = {
        {10, "DEBUG"}, {20, "INFO"}, {30, "WARN"}, {40, "ERROR"}, {50, "FATAL"}};
      if (msg->level < 20) {
        return;
      }
      emit logLine(
        QString("[%1] [%2] %3")
        .arg(niveis.value(msg->level, "?"), nome,
          QString::fromStdString(msg->msg)));
    });
}

bool RosBridge::noAtivo(const QString & nome) const
{
  const std::string alvo = nome.startsWith('/') ?
    nome.mid(1).toStdString() : nome.toStdString();
  for (const auto & n : node_->get_node_names()) {
    // get_node_names devolve com barra e, as vezes, com namespace.
    if (n == alvo || n == "/" + alvo || n.rfind("/" + alvo, 0) == 0) {
      return true;
    }
  }
  return false;
}

QString RosBridge::bestFixedFrame() const
{
  if (tf_buffer_ && tf_buffer_->_frameExists("map")) {
    return "map";
  }
  if (tf_buffer_ && tf_buffer_->_frameExists("odom")) {
    return "odom";
  }
  return "base_footprint";
}

void RosBridge::goTo(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  sendNavGoal(pose);
}

void RosBridge::followWaypoints(const std::vector<std::array<double, 3>> & poses)
{
  if (poses.empty()) {
    emit navResult(false, "Nenhum waypoint para seguir");
    return;
  }
  if (!follow_client_->action_server_is_ready()) {
    emit navResult(false, "Servidor follow_waypoints indisponivel");
    return;
  }
  FollowWaypoints::Goal goal;
  for (const auto & p : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = node_->now();
    ps.pose.position.x = p[0];
    ps.pose.position.y = p[1];
    ps.pose.orientation.z = std::sin(p[2] / 2.0);
    ps.pose.orientation.w = std::cos(p[2] / 2.0);
    goal.poses.push_back(ps);
  }
  rclcpp_action::Client<FollowWaypoints>::SendGoalOptions opts;
  opts.feedback_callback =
    [this](
    rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> fb) {
      emit navStatus(QString("Seguindo waypoints — atual: %1").arg(fb->current_waypoint + 1));
    };
  opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<FollowWaypoints>::WrappedResult & r) {
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
      const int perdidos = r.result ? static_cast<int>(r.result->missed_waypoints.size()) : 0;
      emit navResult(
        ok && perdidos == 0,
        perdidos ? QString("Percurso terminou com %1 waypoint(s) perdido(s)").arg(perdidos)
        : "Percurso de waypoints concluido");
    };
  follow_client_->async_send_goal(goal, opts);
  emit navStatus(QString("Seguindo %1 waypoints...").arg(poses.size()));
}

// ------------------------------------------------------------ Service Areas
void RosBridge::listServiceAreas(const QString & map_name)
{
  if (!sa_list_client_->service_is_ready()) {
    emit serviceAreaStatus(false, "service_area_manager indisponivel (suba a navegacao)");
    return;
  }
  auto req = std::make_shared<caramelo_msgs::srv::ListServiceAreas::Request>();
  req->map_name = map_name.toStdString();
  sa_list_client_->async_send_request(
    req, [this](rclcpp::Client<caramelo_msgs::srv::ListServiceAreas>::SharedFuture fut) {
      const auto resp = fut.get();
      if (!resp || !resp->success) {
        emit serviceAreaStatus(
          false, resp ? QString::fromStdString(resp->message) : "falha no list");
        return;
      }
      QStringList linhas;
      for (size_t i = 0; i < resp->area_ids.size(); ++i) {
        const double x = i < resp->x.size() ? resp->x[i] : 0.0;
        const double y = i < resp->y.size() ? resp->y[i] : 0.0;
        const double yaw = i < resp->yaw.size() ? resp->yaw[i] : 0.0;
        linhas << QString("%1  [%2]  x=%3  y=%4  yaw=%5")
          .arg(QString::fromStdString(resp->area_ids[i]))
          .arg(i < resp->area_types.size() ?
            QString::fromStdString(resp->area_types[i]) : "?")
          .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw, 0, 'f', 2);
      }
      emit serviceAreasListed(linhas);
      emit serviceAreaStatus(true, QString("%1 areas").arg(linhas.size()));
    });
}

void RosBridge::validateServiceAreas(const QString & map_name)
{
  if (!sa_validate_client_->service_is_ready()) {
    emit serviceAreaStatus(false, "service_area_manager indisponivel");
    return;
  }
  auto req = std::make_shared<caramelo_msgs::srv::ValidateServiceAreas::Request>();
  req->map_name = map_name.toStdString();
  req->sync_docking = true;
  sa_validate_client_->async_send_request(
    req, [this](rclcpp::Client<caramelo_msgs::srv::ValidateServiceAreas>::SharedFuture fut) {
      const auto resp = fut.get();
      if (!resp) {
        emit serviceAreaStatus(false, "falha na validacao");
        return;
      }
      QString msg = QString::fromStdString(resp->message);
      if (!resp->errors.empty()) {
        msg += QString(" | %1 erro(s)").arg(resp->errors.size());
      }
      if (!resp->warnings.empty()) {
        msg += QString(" | %1 aviso(s)").arg(resp->warnings.size());
      }
      emit serviceAreaStatus(resp->success, msg);
    });
}

void RosBridge::saveServiceAreaPose(
  const QString & map_name, const QString & area_id, const QString & area_type)
{
  if (!sa_save_client_->action_server_is_ready()) {
    emit serviceAreaStatus(false, "action save_pose indisponivel");
    return;
  }
  SaveServiceAreaPose::Goal goal;
  goal.map_name = map_name.toStdString();
  goal.name = area_id.toStdString();
  goal.area_type = area_type.toStdString();
  goal.yes = true;            // confirma sem prompt (a GUI ja e' a confirmacao)
  goal.sync_docking = true;   // mantem docking.yaml derivado

  rclcpp_action::Client<SaveServiceAreaPose>::SendGoalOptions opts;
  opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<SaveServiceAreaPose>::WrappedResult & r) {
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
        r.result->success;
      emit serviceAreaStatus(
        ok, ok ? QString("Pose salva (x=%1 y=%2)")
        .arg(r.result->x, 0, 'f', 2).arg(r.result->y, 0, 'f', 2)
        : QString::fromStdString(r.result ? r.result->message : "falhou"));
    };
  sa_save_client_->async_send_goal(goal, opts);
  emit serviceAreaStatus(true, QString("Salvando pose atual como %1...").arg(area_id));
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
  // /diagnostics recebe arrays de VARIOS publicadores (health_monitor com os
  // caramelo/*, controller_manager, EKF, lifecycle managers...). Emitir so a
  // mensagem recem-chegada fazia a StateMachine avaliar snapshots PARCIAIS:
  // quando chegava um array sem caramelo/rede_raspberry, levelOf() devolvia
  // STALE e a barra caia para OFFLINE mesmo com tudo verde. Mesclamos por
  // nome num cache e emitimos sempre o estado consolidado.
  static QMap<QString, ComponentHealth> latest;
  for (const auto & st : msg->status) {
    ComponentHealth c;
    c.name = QString::fromStdString(st.name);
    c.level = static_cast<int>(st.level);
    c.message = QString::fromStdString(st.message);
    for (const auto & kv : st.values) {
      c.values.insert(QString::fromStdString(kv.key), QString::fromStdString(kv.value));
    }
    latest.insert(c.name, c);
  }
  QVector<ComponentHealth> health;
  health.reserve(latest.size());
  for (const auto & c : latest) {
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
