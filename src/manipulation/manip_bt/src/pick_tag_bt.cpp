#include "manip_bt/pick_tag_bt.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>

using namespace std::chrono_literals;

namespace manip_bt
{

PickTagBT::PickTagBT(
	const std::string & name,
	const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
	goal_sent_(false),
	waiting_result_(false)
{
	const auto node_name =
		std::string("bt_pick_tag_client_") +
		std::to_string(static_cast<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(this)));

	node_ = std::make_shared<rclcpp::Node>(node_name);
	action_client_ = rclcpp_action::create_client<PickTag>(node_, "/pick_tag");
}

BT::PortsList PickTagBT::providedPorts()
{
	return {
		BT::InputPort<std::string>("tag_frame")
	};
}

BT::NodeStatus PickTagBT::onStart()
{
	std::string tag_frame;

	if (!getInput("tag_frame", tag_frame)) {
		RCLCPP_ERROR(rclcpp::get_logger("PickTagBT"), "Missing input port: tag_frame");
		return BT::NodeStatus::FAILURE;
	}

	RCLCPP_INFO(
		rclcpp::get_logger("PickTagBT"),
		"Sending PICK goal: tag_frame=%s",
		tag_frame.c_str());

	// Item 3.5: respeita o server_wait_timeout do blackboard (mesmo padrao do
	// GoToWSBT) em vez de 10s cravados — o pick/place respawna com delay de
	// 3s e uma missao pode querer esperar mais.
	double wait_timeout = 10.0;
	if (config().blackboard) {
		config().blackboard->get("server_wait_timeout", wait_timeout);
	}
	if (!action_client_->wait_for_action_server(
			std::chrono::duration<double>(wait_timeout)))
	{
		RCLCPP_ERROR(
			rclcpp::get_logger("PickTagBT"),
			"Action server /pick_tag not available after %.1fs", wait_timeout);
		return BT::NodeStatus::FAILURE;
	}

	PickTag::Goal goal_msg;
	goal_msg.tag_frame = tag_frame;

	// Watchdog (auditoria 2026-08-07, item 1.5): rearma a cada feedback de
	// stage. Acao saudavel publica stage o tempo todo — nunca e cancelada.
	last_progress_ = std::chrono::steady_clock::now();
	rclcpp_action::Client<PickTag>::SendGoalOptions options;
	options.feedback_callback =
		[this](GoalHandlePickTag::SharedPtr,
		const std::shared_ptr<const PickTag::Feedback> &) {
			last_progress_ = std::chrono::steady_clock::now();
		};

	goal_future_ = action_client_->async_send_goal(goal_msg, options);
	goal_sent_ = true;
	waiting_result_ = false;
	goal_handle_.reset();
	result_future_ = {};

	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PickTagBT::onRunning()
{
	rclcpp::spin_some(node_);

	if (!goal_sent_) {
		return BT::NodeStatus::FAILURE;
	}

	// 120s sem NENHUM feedback de stage = server pendurado (perda DDS ou
	// espera infinita interna): cancela e reporta FAILURE em vez de a
	// missao ficar muda para sempre. O ciclo completo de retry do pick
	// troca de stage muito antes disso.
	constexpr auto kStageWatchdog = std::chrono::seconds(120);
	if (std::chrono::steady_clock::now() - last_progress_ > kStageWatchdog) {
		RCLCPP_FATAL(
			rclcpp::get_logger("PickTagBT"),
			"WATCHDOG: 120s sem feedback de stage do /pick_tag — server "
			"pendurado. Cancelando o goal e falhando o no.");
		if (goal_handle_) {
			action_client_->async_cancel_goal(goal_handle_);
		}
		goal_sent_ = false;
		waiting_result_ = false;
		return BT::NodeStatus::FAILURE;
	}

	if (!waiting_result_) {
		if (goal_future_.valid() && goal_future_.wait_for(0s) == std::future_status::ready) {
			goal_handle_ = goal_future_.get();
			if (!goal_handle_) {
				RCLCPP_ERROR(rclcpp::get_logger("PickTagBT"), "PICK goal was rejected by server");
				goal_sent_ = false;
				return BT::NodeStatus::FAILURE;
			}

			result_future_ = action_client_->async_get_result(goal_handle_);
			waiting_result_ = true;
			last_progress_ = std::chrono::steady_clock::now();
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
		RCLCPP_ERROR(
			rclcpp::get_logger("PickTagBT"),
			"PICK action finished with non-success result code. message=%s",
			(wrapped_result.result ? wrapped_result.result->message.c_str() : "<empty result>"));
		return BT::NodeStatus::FAILURE;
	}

	if (!wrapped_result.result || !wrapped_result.result->success) {
		RCLCPP_ERROR(
			rclcpp::get_logger("PickTagBT"),
			"PICK action reported failure: %s",
			wrapped_result.result ? wrapped_result.result->message.c_str() : "<empty result>");
		return BT::NodeStatus::FAILURE;
	}

	// Item 2.5 (auditoria 2026-08-07): skip agora e campo do contrato — sem
	// matching de substring. A causa vem em fail_reason.
	if (wrapped_result.result->skipped) {
		RCLCPP_WARN(
			rclcpp::get_logger("PickTagBT"),
			"PICK PULADO (causa: %s): %s — missao continua sem este objeto.",
			wrapped_result.result->fail_reason.c_str(),
			wrapped_result.result->message.c_str());
	} else {
		RCLCPP_INFO(rclcpp::get_logger("PickTagBT"), "PICK action completed successfully");
	}
	return BT::NodeStatus::SUCCESS;
}

void PickTagBT::onHalted()
{
	if (goal_handle_) {
		action_client_->async_cancel_goal(goal_handle_);
	}
	goal_sent_ = false;
	waiting_result_ = false;
	goal_handle_.reset();
	goal_future_ = {};
	result_future_ = {};
	RCLCPP_WARN(rclcpp::get_logger("PickTagBT"), "Pick node halted");
}

}  // namespace manip_bt