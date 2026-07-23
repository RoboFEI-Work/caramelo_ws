#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("test_moveit");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);

  auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");
  gripper.setMaxVelocityScalingFactor(0.3);
  gripper.setMaxAccelerationScalingFactor(0.3);

  // Named goals are defined in the SRDF file, and can be accessed with the following method.

  //arm.setStartStateToCurrentState();
  //arm.setNamedTarget("pose_1");


  //moveit::planning_interface::MoveGroupInterface::Plan plan1;
  //bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  //if (success1)
  //{
  // arm.execute(plan1);
  //}



  //gripper.setStartStateToCurrentState();
  //gripper.setNamedTarget("gripper_close");

  //moveit::planning_interface::MoveGroupInterface::Plan plan3;
  //bool success3 = (gripper.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
  //if (success3)
  //{
  //  gripper.execute(plan3);
  //}

  //arm.setStartStateToCurrentState();
  //arm.setNamedTarget("home");


  //moveit::planning_interface::MoveGroupInterface::Plan plan2;
  //bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  //if (success2)
  //{
  //  arm.execute(plan2);
  //}

  //---------------------------------------------------------------------------------------

  //Joint Goal

  //std::vector<double> joints = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};

  //arm.setStartStateToCurrentState();
  //arm.setJointValueTarget(joints);

  //moveit::planning_interface::MoveGroupInterface::Plan plan4;
  //bool success4 = (arm.plan(plan4) == moveit::core::MoveItErrorCode::SUCCESS);

  //if (success4)  {
  //  arm.execute(plan4);
  //}

//---------------------------------------------------------------------------------------

  //pose goal

  tf2::Quaternion q;
  q.setRPY(0.0, 3.14, 0.0);
  q = q.normalize();

  geometry_msgs::msg::PoseStamped target_pose;

  target_pose.header.frame_id = "base_link";
  target_pose.pose.position.x = 0.4;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = -0.1;
  target_pose.pose.orientation.x = q.getX();
  target_pose.pose.orientation.y = q.getY();
  target_pose.pose.orientation.z = q.getZ();
  target_pose.pose.orientation.w = q.getW();

  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan5;
  bool success5 = (arm.plan(plan5) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success5) {
    arm.execute(plan5);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}