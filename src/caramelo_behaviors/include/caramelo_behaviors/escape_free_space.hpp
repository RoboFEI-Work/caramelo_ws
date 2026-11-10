// EscapeFreeSpace: behavior do Nav2 (behavior_server) para sair de uma zona
// de custo alto / sem trajetoria valida ESCOLHENDO a direcao pelo costmap,
// em vez da re cega do BackUp. Robo omni: candidatas frente, diagonais,
// laterais e re. Rasteriza o INTERIOR do footprint no costmap LOCAL
// (memoria do LiDAR) e no GLOBAL (mesas/paredes do mapa) — sem sensor
// traseiro, e' a unica informacao sobre o que ha atras. Celulas ruins que ja
// estavam sob o robo no inicio sao toleradas (deslizar para fora e' o
// objetivo); celula ruim NOVA sob o footprint bloqueia. Se nenhuma direcao
// e' segura, nao se move e falha (COLLISION_AHEAD): a BT cai no Wait.
// Reusa a action nav2_msgs/action/DriveOnHeading (target.x = distancia,
// speed), entao a BT usa o no nativo <DriveOnHeading server_name=.../>.
#ifndef CARAMELO_BEHAVIORS__ESCAPE_FREE_SPACE_HPP_
#define CARAMELO_BEHAVIORS__ESCAPE_FREE_SPACE_HPP_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "caramelo_behaviors/escape_scoring.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"

namespace caramelo_behaviors
{

class EscapeFreeSpace : public nav2_behaviors::TimedBehavior<nav2_msgs::action::DriveOnHeading>
{
public:
  using ActionT = nav2_msgs::action::DriveOnHeading;
  using ResultStatus = nav2_behaviors::ResultStatus;
  using Status = nav2_behaviors::Status;
  using CostmapInfoType = nav2_core::CostmapInfoType;
  using Costmap2D = nav2_costmap_2d::Costmap2D;
  using CellKeys = std::unordered_set<long long>;

  EscapeFreeSpace();
  ~EscapeFreeSpace() override = default;

  ResultStatus onRun(const std::shared_ptr<const ActionT::Goal> command) override;
  ResultStatus onCycleUpdate() override;
  void onConfigure() override;
  void onCleanup() override;
  CostmapInfoType getResourceInfo() override {return CostmapInfoType::LOCAL;}

protected:
  bool currentPose(geometry_msgs::msg::PoseStamped & pose, const std::string & frame);
  // Avalia o footprint numa pose do frame LOCAL (odom): local + global.
  PoseEval evalPose(double x, double y, double yaw);
  // Avalia num costmap: celulas sob o footprint, separando ruins antigas
  // (em keys0) de novas. Devolve false se o footprint sai da grade.
  bool evalOn(
    const std::shared_ptr<Costmap2D> & cm, double x, double y, double yaw,
    const CellKeys & keys0, PoseEval & e, CellKeys * collect);
  static long long cellKey(const Costmap2D & cm, unsigned int mx, unsigned int my);
  void odomToMap(double x, double y, double yaw, double & gx, double & gy, double & gyaw) const;

  // parametros (prefixo <behavior_name>.)
  double distance_{0.20};
  double speed_{0.25};
  double step_{0.02};
  double simulate_ahead_time_{1.0};
  double exit_cost_{100.0};
  double min_travel_for_exit_{0.05};
  bool allow_backward_{true};
  bool use_global_{true};
  ChoiceParams choice_;

  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> local_sub_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> global_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::vector<geometry_msgs::msg::Point> footprint_;  // casco convexo, frame do robo
  std::shared_ptr<Costmap2D> local_cm_;
  std::shared_ptr<Costmap2D> global_cm_;

  // estado do goal corrente
  ActionT::Feedback::SharedPtr feedback_;
  geometry_msgs::msg::PoseStamped initial_pose_;
  double cmd_dist_{0.0};
  double cmd_speed_{0.0};
  double body_dx_{1.0};
  double body_dy_{0.0};
  std::string dir_name_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
  CellKeys keys0_local_;
  CellKeys keys0_global_;
  int bad0_{0};
  // transformacao 2D odom -> map congelada no onRun (para o costmap global)
  bool have_map_{false};
  double map_x0_{0.0}, map_y0_{0.0}, odom_x0_{0.0}, odom_y0_{0.0}, dth_{0.0};
};

}  // namespace caramelo_behaviors

#endif  // CARAMELO_BEHAVIORS__ESCAPE_FREE_SPACE_HPP_
