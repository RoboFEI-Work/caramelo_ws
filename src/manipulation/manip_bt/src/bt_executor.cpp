#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <string>
#include <thread>

#include "manip_bt/go_to_named_pose_bt.hpp"
#include "manip_bt/pick_tag_bt.hpp"
#include "manip_bt/place_tag_bt.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<manip_bt::GoToNamedPoseBT>("GoToNamedPose");
    factory.registerNodeType<manip_bt::PickTagBT>("PickTag");
    factory.registerNodeType<manip_bt::PlaceTagBT>("PlaceTag");

    const std::string tree_path =
        ament_index_cpp::get_package_share_directory("manip_bt") +
        "/behavior_tree_manip/test3.xml";

    auto tree = factory.createTreeFromFile(tree_path);

    rclcpp::Rate loop_rate(10.0);
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while (rclcpp::ok()) {
        status = tree.tickRoot();
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("manip_bt_executor"), "Behavior tree finished with SUCCESS");
            break;
        }
        if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(rclcpp::get_logger("manip_bt_executor"), "Behavior tree finished with FAILURE");
            break;
        }
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}