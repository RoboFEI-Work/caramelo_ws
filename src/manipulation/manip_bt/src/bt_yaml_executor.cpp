#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
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

struct MissionBuildContext
{
  manip_bt::MissionMapConfigPtr map_config;
  bool simulate_navigation{false};
  std::string finish_dock_id;
  std::vector<std::string> plan_rows;  // tabela do --dry-run
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

  xml << "      <GoToWS ws=\"" << escapeXmlAttr(blackboardPort(ws_key))
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

  std::string current_station_dock;  // ultima estacao visitada (p/ warnings)

  for (std::size_t i = 0; i < actions.size(); ++i) {
    const YAML::Node action = actions[i];
    const std::string kind = action["kind"].as<std::string>("");
    if (kind == "home") {
      const std::string pose_name = action["pose_name"].as<std::string>("home");
      const std::string pose_name_key = actionBlackboardKey(i, "pose_name");
      setStringOnBlackboard(blackboard, i, "pose_name", pose_name);

      xml << "      <GoToNamedPose pose_name=\"" << escapeXmlAttr(blackboardPort(pose_name_key))
          << "\"/>\n";
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] home  pose=" + pose_name);
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

      emitGoToWs(xml, blackboard, i, ws, mesa, dock_id, use_docking);
      current_station_dock = dock_id;
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] goto  " + ws + " -> " + dock_id +
        (use_docking ? "  (dock+align)" : "  (navegacao pura)"));
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

      xml << "      <PickTag tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\"/>\n";
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] pick  tag=" + tag_frame);
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

      xml << "      <PlaceTag tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\" table_pose=\"" << escapeXmlAttr(blackboardPort(table_pose_key)) << "\"/>\n";
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] place tag=" + tag_frame + " mesa=" + table_pose);
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
    xml << "      <GoToNamedPose pose_name=\"" << escapeXmlAttr(blackboardPort(pose_name_key))
        << "\"/>\n";
    emitGoToWs(xml, blackboard, goto_index, finish_id, "", finish_id, false);
    ctx.plan_rows.push_back("[fim] home + goto " + finish_id + " (epilogo automatico)");
  }

  xml << "    </Sequence>\n";
  xml << "  </BehaviorTree>\n";
  xml << "</root>\n";

  return xml.str();
}

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

    auto blackboard = BT::Blackboard::create();
    blackboard->set("max_staging_time", node->declare_parameter<double>("max_staging_time", 120.0));
    blackboard->set("align_timeout", node->declare_parameter<double>("align_timeout", 30.0));
    blackboard->set(
      "max_undocking_time", node->declare_parameter<double>("max_undocking_time", 30.0));
    blackboard->set("nav_timeout", node->declare_parameter<double>("nav_timeout", 180.0));
    blackboard->set(
      "server_wait_timeout", node->declare_parameter<double>("server_wait_timeout", 10.0));
    blackboard->set("use_lidar_refine", node->declare_parameter<bool>("use_lidar_refine", false));
    blackboard->set("skip_align", node->declare_parameter<bool>("skip_align", false));
    blackboard->set("docked", false);
    blackboard->set("current_dock_id", std::string(""));
    blackboard->set("current_dock_type", std::string(""));

    MissionBuildContext ctx;
    ctx.simulate_navigation = simulate_navigation;
    ctx.finish_dock_id = finish_dock_id;

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

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<manip_bt::GoToNamedPoseBT>("GoToNamedPose");
    factory.registerNodeType<manip_bt::PickTagBT>("PickTag");
    factory.registerNodeType<manip_bt::PlaceTagBT>("PlaceTag");
    if (simulate_navigation) {
      factory.registerNodeType<SimulatedGoToWSBT>("GoToWS");
    } else {
      factory.registerNodeType<manip_bt::GoToWSBT>("GoToWS");
    }

    auto tree = factory.createTreeFromText(tree_xml, blackboard);

    if (dry_run) {
      std::cout << "=== Plano da missao (" << yaml_path << ") ===\n";
      for (const auto & row : ctx.plan_rows) {
        std::cout << "  " << row << "\n";
      }
      std::cout << "=== XML gerado ===\n" << tree_xml;
      rclcpp::shutdown();
      return 0;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("bt_yaml_executor"),
      "Running BT from actions yaml: %s (navegacao %s)",
      yaml_path.c_str(),
      simulate_navigation ? "SIMULADA" : "REAL");

    rclcpp::Rate loop_rate(10.0);
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while (rclcpp::ok() && !g_interrupted) {
      status = tree.tickRoot();
      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("bt_yaml_executor"), "Behavior tree finished with SUCCESS");
        break;
      }
      if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(rclcpp::get_logger("bt_yaml_executor"), "Behavior tree finished with FAILURE");
        exit_code = 1;
        break;
      }
      loop_rate.sleep();
    }

    if (g_interrupted) {
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
