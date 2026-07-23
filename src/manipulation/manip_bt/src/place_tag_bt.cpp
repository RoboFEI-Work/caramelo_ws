#include "manip_bt/place_tag_bt.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>

using namespace std::chrono_literals;

namespace manip_bt
{

PlaceTagBT::PlaceTagBT(
	const std::string & name,
	const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
	goal_sent_(false),
	waiting_result_(false)
{
	const auto node_name =
		std::string("bt_place_tag_client_") +
		std::to_string(static_cast<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(this)));

	node_ = std::make_shared<rclcpp::Node>(node_name);
	action_client_ = rclcpp_action::create_client<PlaceTag>(node_, "/place_tag");
}

BT::PortsList PlaceTagBT::providedPorts()
{
	return {
		BT::InputPort<std::string>("tag_frame"),
		BT::InputPort<std::string>("table_pose")
	};
}

BT::NodeStatus PlaceTagBT::onStart()
{
	std::string tag_frame;
	std::string table_pose;

	if (!getInput("tag_frame", tag_frame)) {
		RCLCPP_ERROR(rclcpp::get_logger("PlaceTagBT"), "Missing input port: tag_frame");
		return BT::NodeStatus::FAILURE;
	}

	if (!getInput("table_pose", table_pose)) {
		RCLCPP_ERROR(rclcpp::get_logger("PlaceTagBT"), "Missing input port: table_pose");
		return BT::NodeStatus::FAILURE;
	}

	RCLCPP_INFO(
		rclcpp::get_logger("PlaceTagBT"),
		"Sending PLACE goal: tag_frame=%s table_pose=%s",
		tag_frame.c_str(),
		table_pose.c_str());

	if (!action_client_->wait_for_action_server(10s)) {
		RCLCPP_ERROR(rclcpp::get_logger("PlaceTagBT"), "Action server /place_tag not available");
		return BT::NodeStatus::FAILURE;
	}

	PlaceTag::Goal goal_msg;
	goal_msg.tag_frame = tag_frame;
	goal_msg.table_pose = table_pose;

	goal_future_ = action_client_->async_send_goal(goal_msg);
	goal_sent_ = true;
	waiting_result_ = false;
	goal_handle_.reset();
	result_future_ = {};

	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlaceTagBT::onRunning()
{
	rclcpp::spin_some(node_);

	if (!goal_sent_) {
		return BT::NodeStatus::FAILURE;
	}

	if (!waiting_result_) {
		if (goal_future_.valid() && goal_future_.wait_for(0s) == std::future_status::ready) {
			goal_handle_ = goal_future_.get();
			if (!goal_handle_) {
				RCLCPP_ERROR(rclcpp::get_logger("PlaceTagBT"), "PLACE goal was rejected by server");
				goal_sent_ = false;
				return BT::NodeStatus::FAILURE;
			}

			result_future_ = action_client_->async_get_result(goal_handle_);
			waiting_result_ = true;
			return BT::NodeStatus::RUNNING;
		}

		return BT::NodeStatus::RUNNING;
	}

	if (!result_future_.valid() || result_future_.wait_for(0s) != std::future_status::ready) {
		return BT::NodeStatus::RUNNING;
	}

	const auto wrapped_result = result_future_.get();
	goal_sent_ = false;
	waiting_result_ = false;

	if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_ERROR(rclcpp::get_logger("PlaceTagBT"), "PLACE action finished with non-success result code");
		return BT::NodeStatus::FAILURE;
	}

	if (!wrapped_result.result || !wrapped_result.result->success) {
		RCLCPP_ERROR(
			rclcpp::get_logger("PlaceTagBT"),
			"PLACE action reported failure: %s",
			wrapped_result.result ? wrapped_result.result->message.c_str() : "<empty result>");
		return BT::NodeStatus::FAILURE;
	}

	RCLCPP_INFO(rclcpp::get_logger("PlaceTagBT"), "PLACE action completed successfully");
	return BT::NodeStatus::SUCCESS;
}

void PlaceTagBT::onHalted()
{
	if (goal_handle_) {
		action_client_->async_cancel_goal(goal_handle_);
	}
	goal_sent_ = false;
	waiting_result_ = false;
	goal_handle_.reset();
	goal_future_ = {};
	result_future_ = {};
	RCLCPP_WARN(rclcpp::get_logger("PlaceTagBT"), "Place node halted");
}

}  // namespace manip_bt