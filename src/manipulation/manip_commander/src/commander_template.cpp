#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/string.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using String = example_interfaces::msg::String;
using namespace std::placeholders;

class Commander
{

    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            node_ = node;
            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
            arm_->setMaxVelocityScalingFactor(0.3);
            arm_->setMaxAccelerationScalingFactor(0.3);
            gripper_->setMaxVelocityScalingFactor(0.3);
            gripper_->setMaxAccelerationScalingFactor(0.3);

            open_gripper_sub_ = node_->create_subscription<Bool>(
                "open_gripper", 10, std::bind(&Commander::OpenGripperCallback, this, _1));


            GoToJointTargetSub_ = node_->create_subscription<FloatArray>(
                "go_to_joint_target", 10, std::bind(&Commander::GoToJointTargetCallback, this, _1));

            GoToPoseTargetSub_ = node_->create_subscription<FloatArray>(
                "go_to_pose_target", 10, std::bind(&Commander::GoToPoseTargetCallback, this, _1));

            GoToNamedTargetSub_ = node_->create_subscription<String>(
                "go_to_named_target", 10, std::bind(&Commander::GoToNamedTargetCallback, this, _1));
        }

        void GoToNamedTarget(const std::string &name)
        {
            arm_->setStartStateToCurrentState();
            arm_->setNamedTarget(name);
            PlanAndExecute(arm_);
        }

        void GoToJointTarget(const std::vector<double> &joints)
        {
            arm_->setStartStateToCurrentState();
            arm_->setJointValueTarget(joints);
            PlanAndExecute(arm_);
        }

        void GoToPoseTarget(double x, double y, double z, double roll, double yaw, double pitch)
        {
            tf2::Quaternion q;
            q.setRPY(roll, yaw, pitch);
            q = q.normalize();

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
            PlanAndExecute(arm_);
        }

        void OpenGripper()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("gripper_open");
            PlanAndExecute(gripper_);
        }

        void CloseGripper()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("gripper_close");
            PlanAndExecute(gripper_);
        }



    private:

        void PlanAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success)
            {
                interface->execute(plan);
            }
        }

        void OpenGripperCallback(const Bool &msg)
        {
            if (msg.data)
            {
                OpenGripper();
            }
            else
            {
                CloseGripper();
            }
        }

        void GoToNamedTargetCallback(const String &msg)
        {
            GoToNamedTarget(msg.data);
        }

        void GoToJointTargetCallback(const FloatArray &msg)
        {
            auto joints = msg.data; // Assuming FloatArray has a member 'data' which is a vector of doubles

            if (joints.size() != 5) // Assuming a 5-DOF arm
            {
                RCLCPP_ERROR(node_->get_logger(), "Received joint target with incorrect size: %zu", joints.size());
                return;
            }
            GoToJointTarget(joints);
        }

        void GoToPoseTargetCallback(const FloatArray &msg)
        {
            auto data = msg.data; // Assuming FloatArray has a member 'data' which is a vector of doubles

            if (data.size() != 6) // Expecting x, y, z, roll, yaw, pitch
            {
                RCLCPP_ERROR(node_->get_logger(), "Received pose target with incorrect size: %zu", data.size());
                return;
            }
            GoToPoseTarget(data[0], data[1], data[2], data[3], data[4], data[5]); 
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;

        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
        rclcpp::Subscription<FloatArray>::SharedPtr GoToJointTargetSub_;
        rclcpp::Subscription<FloatArray>::SharedPtr GoToPoseTargetSub_;
        rclcpp::Subscription<String>::SharedPtr GoToNamedTargetSub_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("commander");
  auto commander = std::make_shared<Commander>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}