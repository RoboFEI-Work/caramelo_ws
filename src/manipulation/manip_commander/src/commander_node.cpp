// commander: ferramenta de BANCADA para mover o braco por topico (ros2 topic
// pub), sem missao. Nao faz parte do fluxo de competicao (nenhum no publica
// nestes topicos); sobe no manip_stack.launch.py com use_commander:=true.
//
// Topicos assinados:
//   open_gripper        (Bool)              true = abre, false = fecha a garra
//   go_to_joint_target  (Float64MultiArray) 5 juntas do braco, em rad
//   go_to_pose_target   (Float64MultiArray) [x, y, z, roll, pitch, yaw] em
//                                           base_link (angulos em rad)
//   go_to_named_target  (String)            pose nomeada da SRDF (home, ...)
// Planeja e executa via MoveGroupInterface ("arm" e "gripper") a 30% de
// velocidade/aceleracao.

#include <memory>
#include <string>
#include <vector>

#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using String = example_interfaces::msg::String;
using namespace std::placeholders;

namespace
{
constexpr std::size_t kArmJoints = 5;
constexpr std::size_t kPoseFields = 6;  // x, y, z, roll, pitch, yaw
constexpr double kVelocityScaling = 0.3;
constexpr double kAccelerationScaling = 0.3;
}  // namespace

class Commander
{
public:
  explicit Commander(std::shared_ptr<rclcpp::Node> node)
  : node_(std::move(node))
  {
    arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
    arm_->setMaxVelocityScalingFactor(kVelocityScaling);
    arm_->setMaxAccelerationScalingFactor(kAccelerationScaling);
    gripper_->setMaxVelocityScalingFactor(kVelocityScaling);
    gripper_->setMaxAccelerationScalingFactor(kAccelerationScaling);

    open_gripper_sub_ = node_->create_subscription<Bool>(
      "open_gripper", 10, std::bind(&Commander::openGripperCallback, this, _1));
    go_to_joint_target_sub_ = node_->create_subscription<FloatArray>(
      "go_to_joint_target", 10, std::bind(&Commander::goToJointTargetCallback, this, _1));
    go_to_pose_target_sub_ = node_->create_subscription<FloatArray>(
      "go_to_pose_target", 10, std::bind(&Commander::goToPoseTargetCallback, this, _1));
    go_to_named_target_sub_ = node_->create_subscription<String>(
      "go_to_named_target", 10, std::bind(&Commander::goToNamedTargetCallback, this, _1));
  }

  void goToNamedTarget(const std::string & name)
  {
    arm_->setStartStateToCurrentState();
    arm_->setNamedTarget(name);
    planAndExecute(arm_);
  }

  void goToJointTarget(const std::vector<double> & joints)
  {
    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(joints);
    planAndExecute(arm_);
  }

  // Ordem roll, pitch, yaw = a mesma do tf2::Quaternion::setRPY. O array do
  // topico segue essa ordem: [x, y, z, roll, pitch, yaw].
  void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm_->setStartStateToCurrentState();
    arm_->setPoseTarget(target_pose);
    planAndExecute(arm_);
  }

  void openGripper()
  {
    gripper_->setStartStateToCurrentState();
    gripper_->setNamedTarget("gripper_open");
    planAndExecute(gripper_);
  }

  void closeGripper()
  {
    gripper_->setStartStateToCurrentState();
    gripper_->setNamedTarget("gripper_close");
    planAndExecute(gripper_);
  }

private:
  void planAndExecute(const std::shared_ptr<MoveGroupInterface> & interface)
  {
    MoveGroupInterface::Plan plan;
    const bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      interface->execute(plan);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Planejamento falhou; nada executado.");
    }
  }

  void openGripperCallback(const Bool & msg)
  {
    if (msg.data) {
      openGripper();
    } else {
      closeGripper();
    }
  }

  void goToNamedTargetCallback(const String & msg)
  {
    goToNamedTarget(msg.data);
  }

  void goToJointTargetCallback(const FloatArray & msg)
  {
    const auto & joints = msg.data;
    if (joints.size() != kArmJoints) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Alvo de juntas com tamanho errado: %zu (esperado %zu)", joints.size(), kArmJoints);
      return;
    }
    goToJointTarget(joints);
  }

  void goToPoseTargetCallback(const FloatArray & msg)
  {
    const auto & data = msg.data;
    if (data.size() != kPoseFields) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Alvo de pose com tamanho errado: %zu (esperado %zu: x, y, z, roll, pitch, yaw)",
        data.size(), kPoseFields);
      return;
    }
    goToPoseTarget(data[0], data[1], data[2], data[3], data[4], data[5]);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_;
  std::shared_ptr<MoveGroupInterface> gripper_;

  rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
  rclcpp::Subscription<FloatArray>::SharedPtr go_to_joint_target_sub_;
  rclcpp::Subscription<FloatArray>::SharedPtr go_to_pose_target_sub_;
  rclcpp::Subscription<String>::SharedPtr go_to_named_target_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("commander");
  auto commander = std::make_shared<Commander>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
