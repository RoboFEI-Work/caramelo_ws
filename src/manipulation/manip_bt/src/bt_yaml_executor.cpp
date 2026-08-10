#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <caramelo_msgs/msg/mission_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manip_bt/go_to_named_pose_bt.hpp"
#include "manip_bt/go_to_ws_bt.hpp"
#include "manip_bt/mission_map_config.hpp"
#include "manip_bt/pick_tag_bt.hpp"
#include "manip_bt/place_tag_bt.hpp"

namespace
{

// Ctrl-C com halt LIMPO: o handler default do rclcpp derrubaria o contexto e
// deixaria goals de nav/pick/place orfaos nos servers. Aqui so' levantamos a
// flag; o loop principal chama haltTree() (que cancela os goals) antes do
// shutdown.
std::atomic_bool g_interrupted{false};

void signalHandler(int)
{
  g_interrupted = true;
}

// Mock de navegacao (simulate_navigation:=true): MESMAS portas do GoToWS real,
// para o XML gerado ser identico entre mock e robo.
class SimulatedGoToWSBT : public BT::SyncActionNode
{
public:
  SimulatedGoToWSBT(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ws"),
      BT::InputPort<std::string>("mesa"),
      BT::InputPort<std::string>("dock_id"),
      BT::InputPort<bool>("use_docking"),
      BT::BidirectionalPort<bool>("docked"),
      BT::BidirectionalPort<std::string>("current_dock_id"),
      BT::BidirectionalPort<std::string>("current_dock_type"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string ws;
    std::string mesa;
    std::string dock_id;
    getInput("ws", ws);
    getInput("mesa", mesa);
    getInput("dock_id", dock_id);

    RCLCPP_INFO(
      rclcpp::get_logger("SimulatedGoToWS"),
      "[SIMULADO] Indo para %s (%s%s%s)...",
      dock_id.empty() ? ws.c_str() : dock_id.c_str(),
      ws.c_str(),
      mesa.empty() ? "" : ", mesa ",
      mesa.c_str());

    // Espera de 5 s em fatias de 100 ms para o Ctrl-C nao ficar preso.
    for (int i = 0; i < 50 && rclcpp::ok() && !g_interrupted; ++i) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok() || g_interrupted) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("docked", false);
    setOutput("current_dock_id", dock_id);
    setOutput("current_dock_type", std::string(""));
    return BT::NodeStatus::SUCCESS;
  }
};

std::string resolveActionsYamlPath(const std::string & input_path)
{
  namespace fs = std::filesystem;

  // Caminho literal (absoluto ou relativo ao CWD) vale como dado.
  if (fs::exists(input_path)) {
    return input_path;
  }

  const fs::path share_candidate =
    fs::path(ament_index_cpp::get_package_share_directory("manip_bt")) / "behavior_tree_manip" /
    input_path;
  if (fs::exists(share_candidate)) {
    return share_candidate.string();
  }

  throw std::runtime_error("Could not find actions yaml: " + input_path);
}

std::string escapeXmlAttr(const std::string & value)
{
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '\"':
        out += "&quot;";
        break;
      case '\'':
        out += "&apos;";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

std::string actionBlackboardKey(const std::size_t action_index, const std::string & field)
{
  return "action_" + std::to_string(action_index) + "_" + field;
}

std::string blackboardPort(const std::string & key)
{
  return "{" + key + "}";
}

std::string readRequiredString(
  const YAML::Node & action,
  const std::size_t action_index,
  const std::string & field)
{
  const std::string value = action[field].as<std::string>("");
  if (value.empty()) {
    throw std::runtime_error(
      "actions[" + std::to_string(action_index) + "] missing " + field);
  }
  return value;
}

void setStringOnBlackboard(
  const BT::Blackboard::Ptr & blackboard,
  const std::size_t action_index,
  const std::string & field,
  const std::string & value)
{
  blackboard->set(actionBlackboardKey(action_index, field), value);
}

// Um passo da arvore gerada, na ordem em que sera' executado. Inclui o prologo
// (braco em home) e o epilogo (home + FINISH), que o executor acrescenta
// sozinho e que nao existem no actions.yaml -- por isso a contagem daqui e' a
// unica que serve para uma barra de progresso.
struct MissionStep
{
  std::string kind;    // goto | pick | place | home
  std::string target;  // WS1 | tag_01 | Mesa15 | home
  std::string label;   // texto legivel (mesmo do --dry-run)
};

// Nome do no no XML gerado. E' por ele que o logger de status reencontra o
// passo correspondente quando a arvore muda de estado.
std::string missionStepNodeName(const std::size_t step_index)
{
  return "mission_step_" + std::to_string(step_index);
}

std::string missionStepNameAttr(const std::size_t step_index)
{
  return " name=\"" + missionStepNodeName(step_index) + "\"";
}

struct MissionBuildContext
{
  manip_bt::MissionMapConfigPtr map_config;
  bool simulate_navigation{false};
  std::string finish_dock_id;
  bool startup_home{true};
  std::string startup_pose_name{"home"};
  std::vector<std::string> plan_rows;  // tabela do --dry-run
  std::vector<MissionStep> steps;      // mesma ordem das plan_rows
};

// Resolve WS da tarefa -> dock_id e valida contra o docking.yaml (fail-fast:
// melhor abortar aqui do que descobrir com o robo andando).
std::string resolveDockIdForGoto(
  const std::string & ws,
  MissionBuildContext & ctx,
  const std::size_t action_index)
{
  std::string dock_id;
  if (ctx.map_config) {
    dock_id = ctx.map_config->dockIdForWs(ws);
  } else {
    dock_id = manip_bt::MissionMapConfig::normalizeDockId(ws);
  }

  if (ctx.map_config) {
    if (!ctx.map_config->hasDock(dock_id)) {
      throw std::runtime_error(
              "actions[" + std::to_string(action_index) + "] goto " + ws +
              " -> " + dock_id + ": estacao nao existe no docking.yaml do mapa.");
    }
    const auto info = ctx.map_config->dock(dock_id);
    if (info && info->pose_is_placeholder && !ctx.simulate_navigation) {
      throw std::runtime_error(
              "actions[" + std::to_string(action_index) + "] goto " + ws +
              " -> " + dock_id + ": pose ainda e' placeholder [0,0,0] no docking.yaml — "
              "grave a pose real antes da missao.");
    }
  }
  return dock_id;
}

void emitGoToWs(
  std::ostringstream & xml,
  const BT::Blackboard::Ptr & blackboard,
  const std::size_t action_index,
  const std::size_t step_index,
  const std::string & ws,
  const std::string & mesa,
  const std::string & dock_id,
  const bool use_docking)
{
  const std::string ws_key = actionBlackboardKey(action_index, "ws");
  const std::string mesa_key = actionBlackboardKey(action_index, "mesa");
  const std::string dock_key = actionBlackboardKey(action_index, "dock_id");
  const std::string use_dock_key = actionBlackboardKey(action_index, "use_docking");
  blackboard->set(ws_key, ws);
  blackboard->set(mesa_key, mesa);
  blackboard->set(dock_key, dock_id);
  blackboard->set(use_dock_key, use_docking);

  xml << "      <GoToWS" << missionStepNameAttr(step_index)
      << " ws=\"" << escapeXmlAttr(blackboardPort(ws_key))
      << "\" mesa=\"" << escapeXmlAttr(blackboardPort(mesa_key))
      << "\" dock_id=\"" << escapeXmlAttr(blackboardPort(dock_key))
      << "\" use_docking=\"" << escapeXmlAttr(blackboardPort(use_dock_key))
      << "\" docked=\"{docked}\""
      << " current_dock_id=\"{current_dock_id}\""
      << " current_dock_type=\"{current_dock_type}\"/>\n";
}

std::string buildTreeXmlFromActions(
  const YAML::Node & actions_root,
  const BT::Blackboard::Ptr & blackboard,
  MissionBuildContext & ctx)
{
  const YAML::Node actions = actions_root["actions"];
  if (!actions || !actions.IsSequence()) {
    throw std::runtime_error("Expected 'actions' as a sequence in input yaml");
  }

  std::ostringstream xml;
  // Formato v3 (o pacote linka behaviortree_cpp_v3); o antigo BTCPP_format="4"
  // era um rotulo mentiroso que o parser v3 apenas ignorava.
  xml << "<root main_tree_to_execute=\"MainTree\">\n";
  xml << "  <BehaviorTree ID=\"MainTree\">\n";
  xml << "    <Sequence>\n";

  // Prologo: braco em home ANTES de qualquer acao (pedido do operador —
  // o robo pode bootar com o braco em pose qualquer; GoToNamedPose planeja
  // do estado atual via MoveIt, com colisoes da SRDF). Chave "prologue_*"
  // nunca colide com action_<i>_* nem com o epilogo (N/N+1).
  if (ctx.startup_home) {
    blackboard->set("prologue_pose_name", ctx.startup_pose_name);
    xml << "      <GoToNamedPose" << missionStepNameAttr(ctx.steps.size())
        << " pose_name=\""
        << escapeXmlAttr(blackboardPort("prologue_pose_name")) << "\"/>\n";
    ctx.plan_rows.push_back("[ini] " + ctx.startup_pose_name + " (prologo automatico)");
    ctx.steps.push_back({"home", ctx.startup_pose_name, ctx.plan_rows.back()});
  }

  std::string current_station_dock;  // ultima estacao visitada (p/ warnings)

  for (std::size_t i = 0; i < actions.size(); ++i) {
    const YAML::Node action = actions[i];
    const std::string kind = action["kind"].as<std::string>("");
    if (kind == "home") {
      const std::string pose_name = action["pose_name"].as<std::string>("home");
      const std::string pose_name_key = actionBlackboardKey(i, "pose_name");
      setStringOnBlackboard(blackboard, i, "pose_name", pose_name);

      xml << "      <GoToNamedPose" << missionStepNameAttr(ctx.steps.size())
          << " pose_name=\"" << escapeXmlAttr(blackboardPort(pose_name_key))
          << "\"/>\n";
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] home  pose=" + pose_name);
      ctx.steps.push_back({"home", pose_name, ctx.plan_rows.back()});
      continue;
    }

    if (kind == "goto") {
      const std::string ws = action["ws"].as<std::string>("");
      const std::string mesa = action["mesa"].as<std::string>("");
      if (ws.empty()) {
        throw std::runtime_error(
                "actions[" + std::to_string(i) +
                "] goto sem 'ws' — necessario para resolver a estacao (dock_id).");
      }

      const std::string dock_id = resolveDockIdForGoto(ws, ctx, i);
      const bool use_docking = ctx.map_config ?
        ctx.map_config->useDocking(dock_id) :
        (dock_id != "START" && dock_id != "FINISH");

      emitGoToWs(xml, blackboard, i, ctx.steps.size(), ws, mesa, dock_id, use_docking);
      current_station_dock = dock_id;
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] goto  " + ws + " -> " + dock_id +
        (use_docking ? "  (dock+align)" : "  (navegacao pura)"));
      ctx.steps.push_back({"goto", dock_id, ctx.plan_rows.back()});
      continue;
    }

    if (kind == "pick" || kind == "place") {
      if (ctx.map_config && !current_station_dock.empty() &&
        !ctx.map_config->manipulationEnabled(current_station_dock))
      {
        RCLCPP_WARN(
          rclcpp::get_logger("bt_yaml_executor"),
          "actions[%zu] %s na estacao %s com manipulation_enabled=false no "
          "service_areas.yaml — conferir a arena.",
          i, kind.c_str(), current_station_dock.c_str());
      }
    }

    if (kind == "pick") {
      const std::string tag_frame = readRequiredString(action, i, "tag_frame");
      const std::string tag_frame_key = actionBlackboardKey(i, "tag_frame");
      setStringOnBlackboard(blackboard, i, "tag_frame", tag_frame);

      xml << "      <PickTag" << missionStepNameAttr(ctx.steps.size())
          << " tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\"/>\n";
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] pick  tag=" + tag_frame);
      ctx.steps.push_back({"pick", tag_frame, ctx.plan_rows.back()});
      continue;
    }

    if (kind == "place") {
      const std::string tag_frame = readRequiredString(action, i, "tag_frame");
      const std::string table_pose = readRequiredString(action, i, "table_pose");
      const std::string tag_frame_key = actionBlackboardKey(i, "tag_frame");
      const std::string table_pose_key = actionBlackboardKey(i, "table_pose");

      setStringOnBlackboard(blackboard, i, "tag_frame", tag_frame);
      setStringOnBlackboard(blackboard, i, "table_pose", table_pose);
      if (action["ws"]) {
        setStringOnBlackboard(blackboard, i, "ws", action["ws"].as<std::string>());
      }

      xml << "      <PlaceTag" << missionStepNameAttr(ctx.steps.size())
          << " tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\" table_pose=\"" << escapeXmlAttr(blackboardPort(table_pose_key)) << "\"/>\n";
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] place tag=" + tag_frame + " mesa=" + table_pose);
      ctx.steps.push_back({"place", tag_frame, ctx.plan_rows.back()});
      continue;
    }

    throw std::runtime_error("actions[" + std::to_string(i) + "] has unsupported kind: " + kind);
  }

  // Epilogo: braco em home e ida ao FINISH (o undock de saida da ultima
  // estacao acontece dentro do proprio GoToWS). O task_planner nao emite o
  // FINISH — e' responsabilidade do executor fechar a missao.
  if (!ctx.finish_dock_id.empty()) {
    const std::string finish_id =
      manip_bt::MissionMapConfig::normalizeDockId(ctx.finish_dock_id);
    if (ctx.map_config && !ctx.map_config->hasDock(finish_id)) {
      throw std::runtime_error(
              "finish_dock_id=" + finish_id + " nao existe no docking.yaml do mapa.");
    }
    if (ctx.map_config && !ctx.simulate_navigation) {
      const auto info = ctx.map_config->dock(finish_id);
      if (info && info->pose_is_placeholder) {
        throw std::runtime_error(
                "finish_dock_id=" + finish_id + " com pose placeholder [0,0,0].");
      }
    }
    const std::size_t home_index = actions.size();
    const std::size_t goto_index = actions.size() + 1;
    const std::string pose_name_key = actionBlackboardKey(home_index, "pose_name");
    blackboard->set(pose_name_key, std::string("home"));
    xml << "      <GoToNamedPose" << missionStepNameAttr(ctx.steps.size())
        << " pose_name=\"" << escapeXmlAttr(blackboardPort(pose_name_key))
        << "\"/>\n";
    ctx.steps.push_back({"home", "home", "[fim] home (epilogo automatico)"});
    // Item 3.9: o docking do FINISH era `false` cravado — se a arena definir
    // o FINISH como estacao com dock, o epilogo chegava perto e parava sem
    // encostar. Agora pergunta ao mapa, com o mesmo fallback do goto normal.
    const bool finish_use_docking = ctx.map_config ?
      ctx.map_config->useDocking(finish_id) :
      (finish_id != "START" && finish_id != "FINISH");
    emitGoToWs(
      xml, blackboard, goto_index, ctx.steps.size(), finish_id, "", finish_id,
      finish_use_docking);
    ctx.plan_rows.push_back(
      "[fim] home + goto " + finish_id + " (epilogo automatico)" +
      (finish_use_docking ? "  (dock+align)" : "  (navegacao pura)"));
    ctx.steps.push_back({"goto", finish_id, ctx.plan_rows.back()});
  }

  xml << "    </Sequence>\n";
  xml << "  </BehaviorTree>\n";
  xml << "</root>\n";

  return xml.str();
}

// ---------------------------------------------------------------------------
// Publicacao de MissionStatus.
//
// Ate' aqui uma missao so' podia ser acompanhada lendo o terminal: este executor
// escreve apenas em stdout/rosout. Nenhuma interface (GUI Qt ou painel web)
// conseguia saber em que passo a missao estava sem parsear terminal. O que vem
// abaixo e' ADITIVO: nao muda a arvore, os exit codes nem a saida de terminal --
// so' acrescenta um topico.
// ---------------------------------------------------------------------------
class MissionReporter
{
public:
  MissionReporter(
    rclcpp::Node::SharedPtr node,
    std::string mission_id,
    std::string task_id,
    std::vector<MissionStep> steps)
  : node_(std::move(node)),
    mission_id_(std::move(mission_id)),
    task_id_(std::move(task_id)),
    steps_(std::move(steps)),
    started_(std::chrono::steady_clock::now())
  {
    // transient_local com historico FUNDO, nao depth 1.
    //
    // O publicador nasce junto com a missao, entao todo assinante e' um late
    // joiner: mesmo uma UI aberta ha' horas so' descobre este topico depois que o
    // executor sobe. Medido no WSL (networkingMode=mirrored), o discovery levou
    // ~11 s -- com depth 1 a interface perderia o inicio de TODA missao e so'
    // pegaria o ultimo estado. Com depth 64 o late joiner recebe de uma vez toda
    // a historia ate' ali e consegue desenhar a missao inteira.
    rclcpp::QoS qos(64);
    qos.transient_local().reliable();
    pub_ = node_->create_publisher<caramelo_msgs::msg::MissionStatus>(
      "/caramelo/mission/status", qos);
  }

  // Estado de missao sem passo associado (planejamento, fim, abort).
  void publishState(const uint8_t state, const std::string & message)
  {
    auto msg = base();
    msg.action_index = -1;
    msg.state = state;
    msg.message = message;
    pub_->publish(msg);
  }

  // Idempotente. O GoToNamedPose e' SyncActionNode (bloqueia no MoveIt) e nunca
  // passa por RUNNING, entao o inicio dele e' anunciado pelo fim do passo
  // anterior; o guard evita anuncio duplicado quando o passo seguinte e'
  // assincrono e emite RUNNING por conta propria.
  void publishStepStarted(const std::size_t step_index)
  {
    if (step_index >= steps_.size() || started_steps_.count(step_index) != 0) {
      return;
    }
    started_steps_.insert(step_index);
    publishStep(step_index, "iniciado", steps_[step_index].label);
  }

  void publishStepFinished(const std::size_t step_index, const bool ok)
  {
    if (step_index >= steps_.size()) {
      return;
    }
    publishStep(
      step_index, ok ? "concluido" : "falhou",
      steps_[step_index].label + (ok ? " — concluido" : " — FALHOU"));
    if (ok) {
      publishStepStarted(step_index + 1);
    }
  }

private:
  caramelo_msgs::msg::MissionStatus base() const
  {
    caramelo_msgs::msg::MissionStatus msg;
    msg.header.stamp = node_->now();
    msg.mission_id = mission_id_;
    msg.task_id = task_id_;
    msg.action_index = -1;
    msg.action_total = static_cast<int32_t>(steps_.size());
    msg.elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
    return msg;
  }

  void publishStep(
    const std::size_t step_index, const std::string & stage, const std::string & message)
  {
    auto msg = base();
    msg.action_index = static_cast<int32_t>(step_index);
    msg.action_kind = steps_[step_index].kind;
    msg.action_target = steps_[step_index].target;
    msg.stage = stage;
    msg.state = caramelo_msgs::msg::MissionStatus::STATE_RUNNING;
    msg.message = message;
    pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<caramelo_msgs::msg::MissionStatus>::SharedPtr pub_;
  std::string mission_id_;
  std::string task_id_;
  std::vector<MissionStep> steps_;
  std::set<std::size_t> started_steps_;
  std::chrono::steady_clock::time_point started_;
};

// Traduz transicoes de estado da arvore em MissionStatus. So' reage aos nos que o
// gerador de XML batizou de mission_step_<k>; Sequence e decoradores nao
// interessam.
class MissionStatusLogger : public BT::StatusChangeLogger
{
public:
  MissionStatusLogger(BT::TreeNode * root_node, std::shared_ptr<MissionReporter> reporter)
  : BT::StatusChangeLogger(root_node), reporter_(std::move(reporter))
  {
  }

  void callback(
    BT::Duration /*timestamp*/,
    const BT::TreeNode & node,
    BT::NodeStatus /*prev_status*/,
    BT::NodeStatus status) override
  {
    const auto step_index = stepIndexFromName(node.name());
    if (!step_index.has_value()) {
      return;
    }
    switch (status) {
      case BT::NodeStatus::RUNNING:
        reporter_->publishStepStarted(*step_index);
        break;
      case BT::NodeStatus::SUCCESS:
        reporter_->publishStepFinished(*step_index, true);
        break;
      case BT::NodeStatus::FAILURE:
        reporter_->publishStepFinished(*step_index, false);
        break;
      default:
        break;
    }
  }

  void flush() override {}

private:
  static std::optional<std::size_t> stepIndexFromName(const std::string & name)
  {
    static const std::string prefix = "mission_step_";
    if (name.rfind(prefix, 0) != 0) {
      return std::nullopt;
    }
    try {
      return static_cast<std::size_t>(std::stoul(name.substr(prefix.size())));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  std::shared_ptr<MissionReporter> reporter_;
};

}  // namespace

int main(int argc, char ** argv)
{
  // SignalHandlerOptions::None: o Ctrl-C e' tratado por nos (halt limpo).
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  int exit_code = 0;
  try {
    const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    std::string input_yaml = "bmtt.yaml";
    bool dry_run = false;
    for (std::size_t i = 1; i < args.size(); ++i) {
      if (args[i] == "--dry-run") {
        dry_run = true;
      } else if (!args[i].empty() && args[i][0] != '-') {
        input_yaml = args[i];
      }
    }

    auto node = std::make_shared<rclcpp::Node>("mission_executor");
    const std::string map_folder_param =
      node->declare_parameter<std::string>("map_folder", "");
    const std::string map_name_param =
      node->declare_parameter<std::string>("map_name", "");
    const bool simulate_navigation =
      node->declare_parameter<bool>("simulate_navigation", false);
    const std::string finish_dock_id =
      node->declare_parameter<std::string>("finish_dock_id", "FINISH");
    const bool startup_home = node->declare_parameter<bool>("startup_home", true);
    const std::string startup_pose_name =
      node->declare_parameter<std::string>("startup_pose_name", "home");
    // So' rotula o MissionStatus; quem sabe a tarefa e' o run_mission/mission_server.
    const std::string task_id_param = node->declare_parameter<std::string>("task_id", "");

    auto blackboard = BT::Blackboard::create();
    blackboard->set("max_staging_time", node->declare_parameter<double>("max_staging_time", 120.0));
    blackboard->set("align_timeout", node->declare_parameter<double>("align_timeout", 30.0));
    blackboard->set(
      "max_undocking_time", node->declare_parameter<double>("max_undocking_time", 30.0));
    blackboard->set("nav_timeout", node->declare_parameter<double>("nav_timeout", 180.0));
    blackboard->set(
      "server_wait_timeout", node->declare_parameter<double>("server_wait_timeout", 10.0));
    // Default true (2026-08-03): sem refino a missao alinha só pelo AMCL e
    // herda o offset global (~cm) na hora critica. Desligar (debug):
    //   --ros-args -p use_lidar_refine:=false
    blackboard->set("use_lidar_refine", node->declare_parameter<bool>("use_lidar_refine", true));
    blackboard->set("skip_align", node->declare_parameter<bool>("skip_align", false));
    blackboard->set("docked", false);
    blackboard->set("current_dock_id", std::string(""));
    blackboard->set("current_dock_type", std::string(""));

    MissionBuildContext ctx;
    ctx.simulate_navigation = simulate_navigation;
    ctx.finish_dock_id = finish_dock_id;
    ctx.startup_home = startup_home;
    ctx.startup_pose_name = startup_pose_name;

    std::string map_folder = map_folder_param;
    if (map_folder.empty() && !map_name_param.empty()) {
      map_folder = manip_bt::MissionMapConfig::resolveMapFolder(map_name_param);
      if (map_folder.empty()) {
        throw std::runtime_error(
                "Nao encontrei a pasta do mapa '" + map_name_param +
                "' (tente -p map_folder:=/caminho/absoluto).");
      }
    }
    if (!map_folder.empty()) {
      ctx.map_config = std::make_shared<manip_bt::MissionMapConfig>(map_folder);
      RCLCPP_INFO(
        rclcpp::get_logger("bt_yaml_executor"),
        "Mapa da missao: %s", ctx.map_config->mapFolder().c_str());
    } else if (!simulate_navigation) {
      throw std::runtime_error(
              "Sem pasta de mapa: passe -p map_folder:=/caminho/absoluto (ou "
              "-p map_name:=<mapa>), ou use o run_mission. Para testar sem mapa, "
              "use -p simulate_navigation:=true.");
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("bt_yaml_executor"),
        "simulate_navigation sem mapa: dock_ids derivados por normalizacao, sem validacao.");
    }
    blackboard->set("mission_config", ctx.map_config);

    const std::string yaml_path = resolveActionsYamlPath(input_yaml);
    const YAML::Node actions_root = YAML::LoadFile(yaml_path);
    const std::string tree_xml = buildTreeXmlFromActions(actions_root, blackboard, ctx);

    // mission_id = nome do actions.yaml (o run_mission grava um por execucao, com
    // timestamp), para uma UI distinguir duas missoes seguidas.
    std::shared_ptr<MissionReporter> reporter;
    if (!dry_run) {
      std::string task_id = task_id_param;
      if (task_id.empty() && actions_root["task_id"]) {
        task_id = actions_root["task_id"].as<std::string>("");
      }
      reporter = std::make_shared<MissionReporter>(
        node, std::filesystem::path(yaml_path).stem().string(), task_id, ctx.steps);
      // A construcao da arvore conecta ao /move_group e pode levar minutos com o
      // MoveIt subindo. Sem este aviso a UI ficaria muda justamente no trecho em
      // que o operador mais se pergunta se travou.
      reporter->publishState(
        caramelo_msgs::msg::MissionStatus::STATE_PLANNING,
        "Montando a arvore da missao (conectando ao move_group)...");
    }

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<manip_bt::GoToNamedPoseBT>("GoToNamedPose");
    factory.registerNodeType<manip_bt::PickTagBT>("PickTag");
    factory.registerNodeType<manip_bt::PlaceTagBT>("PlaceTag");
    if (simulate_navigation) {
      factory.registerNodeType<SimulatedGoToWSBT>("GoToWS");
    } else {
      factory.registerNodeType<manip_bt::GoToWSBT>("GoToWS");
    }

    RCLCPP_INFO(
      rclcpp::get_logger("bt_yaml_executor"),
      "Instanciando os nos da arvore (conectando ao /move_group — o MoveIt "
      "pode levar um tempo para terminar de subir; acompanhe os logs)...");
    const auto build_start = std::chrono::steady_clock::now();
    auto tree = factory.createTreeFromText(tree_xml, blackboard);
    RCLCPP_INFO(
      rclcpp::get_logger("bt_yaml_executor"),
      "Arvore construida em %.1f s.",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - build_start).count());

    if (dry_run) {
      // Sem publicar nada: o --dry-run nao e' uma missao, e um MissionStatus aqui
      // faria uma UI acreditar que o robo esta' andando.
      std::cout << "=== Plano da missao (" << yaml_path << ") ===\n";
      for (const auto & row : ctx.plan_rows) {
        std::cout << "  " << row << "\n";
      }
      std::cout << "=== XML gerado ===\n" << tree_xml;
      rclcpp::shutdown();
      return 0;
    }

    MissionStatusLogger status_logger(tree.rootNode(), reporter);

    RCLCPP_INFO(
      rclcpp::get_logger("bt_yaml_executor"),
      "Running BT from actions yaml: %s (navegacao %s)",
      yaml_path.c_str(),
      simulate_navigation ? "SIMULADA" : "REAL");

    // O primeiro passo e' quase sempre o prologo (GoToNamedPose, sincrono), que
    // nunca emite RUNNING. Sem este anuncio a UI ficaria em branco durante todo o
    // primeiro movimento do braco.
    reporter->publishStepStarted(0);

    rclcpp::Rate loop_rate(10.0);
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while (rclcpp::ok() && !g_interrupted) {
      status = tree.tickRoot();
      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("bt_yaml_executor"), "Behavior tree finished with SUCCESS");
        reporter->publishState(
          caramelo_msgs::msg::MissionStatus::STATE_DONE, "Missao concluida com sucesso.");
        break;
      }
      if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(rclcpp::get_logger("bt_yaml_executor"), "Behavior tree finished with FAILURE");
        reporter->publishState(
          caramelo_msgs::msg::MissionStatus::STATE_FAILED, "Missao falhou (ver /rosout).");
        exit_code = 1;
        break;
      }
      loop_rate.sleep();
    }

    if (g_interrupted) {
      reporter->publishState(
        caramelo_msgs::msg::MissionStatus::STATE_ABORTED, "Missao abortada pelo operador.");
      // Nota: se o no atual for o GoToNamedPose (SyncActionNode com MoveIt
      // bloqueante), o halt so acontece quando o tick dele retornar — os nos
      // assincronos (GoToWS/PickTag/PlaceTag) cancelam imediatamente.
      RCLCPP_WARN(
        rclcpp::get_logger("bt_yaml_executor"),
        "Interrompido (Ctrl-C): haltando a arvore e cancelando goals em voo...");
      tree.haltTree();
      // Da um respiro para os cancels chegarem aos servers antes do shutdown.
      rclcpp::sleep_for(std::chrono::milliseconds(800));
      exit_code = 130;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("bt_yaml_executor"), "Execution failed: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
