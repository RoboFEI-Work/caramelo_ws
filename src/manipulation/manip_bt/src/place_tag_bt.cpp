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
		BT::InputPort<std::string>("table_pose"),
		// Opcional: estacao de destino ("WS_3", "PP_1"...). Vazio = legado.
		BT::InputPort<std::string>("ws"),
		// Opcional (2026-08-24): empilhar — frame da tag BASE ja na mesa.
		BT::InputPort<std::string>("stack_on"),
		// Opcional (2026-08-25): "RED"|"BLUE" = soltar dentro do container
		// dessa cor visto na mesa; vazio = mesa.
		BT::InputPort<std::string>("container_color"),
		// 2026-08-28 (fila de alcance): tentativa FINAL na estacao? Porta
		// AUSENTE => true (XML antigo = comportamento de sempre). Dentro de
		// uma <ReachQueue> vem de {rq_<g>_final} e comeca em false: alvo visto
		// sem IK faz o server devolver o cubo ao container de bordo e sair
		// cedo com unreachable=true (sem fallback na mesa).
		BT::InputPort<bool>("final_attempt", true, "tentativa final (ausente = true)"),
		// Saidas do resultado (2026-08-28) — quem decide o que fazer e o
		// ReachQueue; este no continua devolvendo SUCCESS em todo success=true.
		BT::OutputPort<bool>("unreachable"),
		BT::OutputPort<double>("suggested_base_shift_m"),
		BT::OutputPort<bool>("skipped")
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
	tag_frame_ = tag_frame;

	if (!getInput("table_pose", table_pose)) {
		RCLCPP_ERROR(rclcpp::get_logger("PlaceTagBT"), "Missing input port: table_pose");
		return BT::NodeStatus::FAILURE;
	}

	std::string ws;
	getInput("ws", ws);   // opcional: ausente = legado (sem escolha de slot)
	std::string stack_on;
	getInput("stack_on", stack_on);   // opcional: vazio = nao empilha
	std::string container_color;
	getInput("container_color", container_color);   // opcional: vazio = mesa

	// Porta ausente (ou sem valor no blackboard) => true: comportamento antigo.
	final_attempt_ = true;
	if (!getInput("final_attempt", final_attempt_)) {
		final_attempt_ = true;
	}

	// Zera as saidas ANTES de mandar o goal (ver pick_tag_bt.cpp): o erro de
	// setOutput sem remapeamento (XML antigo) e ignorado de proposito.
	setOutput("unreachable", false);
	setOutput("suggested_base_shift_m", 0.0);
	setOutput("skipped", false);

	RCLCPP_INFO(
		rclcpp::get_logger("PlaceTagBT"),
		"Sending PLACE goal: tag_frame=%s table_pose=%s ws=%s stack_on=%s container_color=%s "
		"final_attempt=%s",
		tag_frame.c_str(),
		table_pose.c_str(),
		ws.empty() ? "<sem ws>" : ws.c_str(),
		stack_on.empty() ? "<nao>" : stack_on.c_str(),
		container_color.empty() ? "<mesa>" : container_color.c_str(),
		final_attempt_ ? "true" : "false");

	// Item 3.5: mesmo tratamento do PickTagBT — timeout vem do blackboard.
	double wait_timeout = 10.0;
	if (config().blackboard) {
		config().blackboard->get("server_wait_timeout", wait_timeout);
	}
	if (!action_client_->wait_for_action_server(
			std::chrono::duration<double>(wait_timeout)))
	{
		RCLCPP_ERROR(
			rclcpp::get_logger("PlaceTagBT"),
			"Action server /place_tag not available after %.1fs", wait_timeout);
		return BT::NodeStatus::FAILURE;
	}

	PlaceTag::Goal goal_msg;
	goal_msg.tag_frame = tag_frame;
	goal_msg.table_pose = table_pose;
	goal_msg.ws = ws;
	goal_msg.stack_on = stack_on;
	goal_msg.container_color = container_color;
	goal_msg.final_attempt = final_attempt_;

	// Watchdog (auditoria 2026-08-07, item 1.5): rearma a cada feedback de
	// stage — acao saudavel publica stage continuamente, nunca e cancelada.
	last_progress_ = std::chrono::steady_clock::now();
	rclcpp_action::Client<PlaceTag>::SendGoalOptions options;
	options.feedback_callback =
		[this](GoalHandlePlaceTag::SharedPtr,
		const std::shared_ptr<const PlaceTag::Feedback> &) {
			last_progress_ = std::chrono::steady_clock::now();
		};

	goal_future_ = action_client_->async_send_goal(goal_msg, options);
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

	// 120s sem NENHUM feedback de stage = server pendurado: cancela e falha
	// em vez de a missao ficar muda para sempre.
	constexpr auto kStageWatchdog = std::chrono::seconds(120);
	if (std::chrono::steady_clock::now() - last_progress_ > kStageWatchdog) {
		RCLCPP_FATAL(
			rclcpp::get_logger("PlaceTagBT"),
			"WATCHDOG: 120s sem feedback de stage do /place_tag — server "
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
				RCLCPP_ERROR(rclcpp::get_logger("PlaceTagBT"), "PLACE goal was rejected by server");
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

	// 2026-08-28 (fila de alcance): veredito do server nas portas de saida
	// ANTES do SUCCESS — o ReachQueue le pelo blackboard.
	const auto & res = *wrapped_result.result;
	setOutput("unreachable", res.unreachable);
	setOutput("suggested_base_shift_m", static_cast<double>(res.suggested_base_shift_m));
	setOutput("skipped", res.skipped);

	if (res.unreachable) {
		RCLCPP_WARN(
			rclcpp::get_logger("PlaceTagBT"),
			"PLACE FORA DE ALCANCE (%s): alvo x=%.2f y=%.2f em manip_base_link, "
			"sugestao lateral %+.2f m%s — %s",
			tag_frame_.c_str(),
			static_cast<double>(res.target_x),
			static_cast<double>(res.target_y),
			static_cast<double>(res.suggested_base_shift_m),
			final_attempt_ ? " (tentativa final)" :
			" (cubo devolvido ao container de bordo; a fila de alcance decide)",
			res.message.c_str());
	}
	if (res.skipped) {
		// Item 2.5 (auditoria 2026-08-07): skip explicito no contrato. (Linha
		// estavel: vale tambem para o skip por alcance.)
		RCLCPP_WARN(
			rclcpp::get_logger("PlaceTagBT"),
			"PLACE PULADO (causa: %s): %s — missao continua.",
			res.fail_reason.c_str(),
			res.message.c_str());
	} else if (!res.unreachable) {
		RCLCPP_INFO(rclcpp::get_logger("PlaceTagBT"), "PLACE action completed successfully");
	}
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
