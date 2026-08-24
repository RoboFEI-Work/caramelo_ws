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
  bool startup_home{true};
  std::string startup_pose_name{"home"};
  // 2026-08-18 (pedido do operador): GoToWS falhou depois de todos os retries
  // internos do align => PULA o bloco da estacao (picks/places dela) e a
  // missao segue — nao perder a task inteira por uma mesa inacessivel.
  bool skip_unreachable_station{true};
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

  // Prologo: braco em home ANTES de qualquer acao (pedido do operador —
  // o robo pode bootar com o braco em pose qualquer; GoToNamedPose planeja
  // do estado atual via MoveIt, com colisoes da SRDF). Chave "prologue_*"
  // nunca colide com action_<i>_* nem com o epilogo (N/N+1).
  if (ctx.startup_home) {
    blackboard->set("prologue_pose_name", ctx.startup_pose_name);
    xml << "      <GoToNamedPose pose_name=\""
        << escapeXmlAttr(blackboardPort("prologue_pose_name")) << "\"/>\n";
    ctx.plan_rows.push_back("[ini] " + ctx.startup_pose_name + " (prologo automatico)");
  }

  std::string current_station_dock;  // ultima estacao visitada (p/ warnings)
  std::string current_station_ws;    // 'ws' do ultimo goto (p/ o place legado sem 'ws')

  // Bloco de estacao com skip (2026-08-18, pedido do operador): cada goto de
  // estacao dockavel abre um <Fallback><Inverter><GoToWS/></Inverter>
  // <Sequence>picks/places</Sequence></Fallback>. GoToWS falhou (depois dos
  // retries internos do align) => Inverter vira SUCCESS => a Fallback fecha
  // SEM rodar o bloco => a missao segue para a proxima estacao. Pick/place
  // falhando mantem a semantica atual (so o goto aciona a Fallback).
  bool station_open = false;
  bool station_has_actions = false;
  const auto close_station_block = [&xml, &station_open, &station_has_actions]() {
      if (!station_open) {
        return;
      }
      if (!station_has_actions) {
        xml << "      <AlwaysSuccess/>\n";  // Sequence v3 nao pode ficar vazia
      }
      xml << "      </Sequence>\n";
      xml << "      </Fallback>\n";
      station_open = false;
      station_has_actions = false;
    };

  for (std::size_t i = 0; i < actions.size(); ++i) {
    const YAML::Node action = actions[i];
    const std::string kind = action["kind"].as<std::string>("");
    if (kind == "home") {
      const std::string pose_name = action["pose_name"].as<std::string>("home");
      const std::string pose_name_key = actionBlackboardKey(i, "pose_name");
      setStringOnBlackboard(blackboard, i, "pose_name", pose_name);

      xml << "      <GoToNamedPose pose_name=\"" << escapeXmlAttr(blackboardPort(pose_name_key))
          << "\"/>\n";
      station_has_actions = station_has_actions || station_open;
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

      close_station_block();
      const bool wrap = use_docking && ctx.skip_unreachable_station;
      if (wrap) {
        xml << "      <Fallback>\n";
        xml << "      <Inverter>\n";
      }
      emitGoToWs(xml, blackboard, i, ws, mesa, dock_id, use_docking);
      if (wrap) {
        xml << "      </Inverter>\n";
        xml << "      <Sequence>\n";
        station_open = true;
      }
      current_station_dock = dock_id;
      current_station_ws = ws;
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] goto  " + ws + " -> " + dock_id +
        (use_docking ? "  (dock+align)" : "  (navegacao pura)") +
        (wrap ? "  [falhou => pula a estacao]" : ""));
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

      // 2026-08-12: repassa a mesa tambem no pick — a prateleira (MesaSh) tem
      // sequencia propria de movimentos. Opcional de proposito: actions.yaml
      // antigo (sem table_pose no pick) continua valendo e cai no
      // comportamento de mesa comum.
      std::string table_pose;
      if (action["table_pose"]) {
        table_pose = action["table_pose"].as<std::string>();
      }
      const std::string table_pose_key = actionBlackboardKey(i, "table_pose");
      setStringOnBlackboard(blackboard, i, "table_pose", table_pose);

      xml << "      <PickTag tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\" table_pose=\"" << escapeXmlAttr(blackboardPort(table_pose_key)) << "\"/>\n";
      station_has_actions = station_has_actions || station_open;
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] pick  tag=" + tag_frame +
        (table_pose.empty() ? "" : " mesa=" + table_pose));
      continue;
    }

    if (kind == "place") {
      const std::string tag_frame = readRequiredString(action, i, "tag_frame");
      const std::string table_pose = readRequiredString(action, i, "table_pose");
      const std::string tag_frame_key = actionBlackboardKey(i, "tag_frame");
      const std::string table_pose_key = actionBlackboardKey(i, "table_pose");

      setStringOnBlackboard(blackboard, i, "tag_frame", tag_frame);
      setStringOnBlackboard(blackboard, i, "table_pose", table_pose);

      // 2026-08-21: o 'ws' agora viaja ate o no de place (escolha de slot na
      // mesa comum e guarda de PP/SH). Preferencia: 'ws' da propria acao
      // (task_planner sempre grava); yaml de baixo nivel sem 'ws' herda a
      // estacao do ultimo goto; sem nada = vazio (comportamento legado).
      std::string place_ws;
      if (action["ws"]) {
        place_ws = action["ws"].as<std::string>("");
      }
      if (place_ws.empty()) {
        place_ws = current_station_ws;
      }
      const std::string ws_key = actionBlackboardKey(i, "ws");
      setStringOnBlackboard(blackboard, i, "ws", place_ws);

      // 2026-08-24: EMPILHAR — `stack_on: tag_4` = soltar este objeto em cima
      // do objeto da tag_4 (ja na mesa da MESMA estacao). table_pose continua
      // obrigatorio (altura da mesa; fallback se a base nao for vista).
      // Validacao fail-fast: frame TF plausivel e diferente do proprio objeto.
      std::string stack_on;
      if (action["stack_on"]) {
        stack_on = action["stack_on"].as<std::string>("");
      }
      if (!stack_on.empty()) {
        const bool plausible =
          stack_on.rfind("tag_", 0) == 0 || stack_on.rfind("ct_", 0) == 0;
        if (!plausible || stack_on == tag_frame) {
          throw std::runtime_error(
            "action[" + std::to_string(i) + "]: stack_on='" + stack_on +
            "' invalido (esperado tag_N/ct_N diferente de tag_frame)");
        }
      }
      const std::string stack_on_key = actionBlackboardKey(i, "stack_on");
      setStringOnBlackboard(blackboard, i, "stack_on", stack_on);

      xml << "      <PlaceTag tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\" table_pose=\"" << escapeXmlAttr(blackboardPort(table_pose_key))
          << "\" ws=\"" << escapeXmlAttr(blackboardPort(ws_key))
          << "\" stack_on=\"" << escapeXmlAttr(blackboardPort(stack_on_key)) << "\"/>\n";
      station_has_actions = station_has_actions || station_open;
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] place tag=" + tag_frame + " mesa=" + table_pose +
        (place_ws.empty() ? "" : " ws=" + place_ws) +
        (stack_on.empty() ? "" : " EMPILHAR sobre " + stack_on));
      continue;
    }

    throw std::runtime_error("actions[" + std::to_string(i) + "] has unsupported kind: " + kind);
  }

  // Fecha o bloco da ultima estacao ANTES do epilogo: o home final e o goto
  // FINISH rodam SEMPRE, mesmo com a ultima estacao pulada.
  close_station_block();

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
    // Item 3.9: o docking do FINISH era `false` cravado — se a arena definir
    // o FINISH como estacao com dock, o epilogo chegava perto e parava sem
    // encostar. Agora pergunta ao mapa, com o mesmo fallback do goto normal.
    const bool finish_use_docking = ctx.map_config ?
      ctx.map_config->useDocking(finish_id) :
      (finish_id != "START" && finish_id != "FINISH");
    emitGoToWs(xml, blackboard, goto_index, finish_id, "", finish_id, finish_use_docking);
    ctx.plan_rows.push_back(
      "[fim] home + goto " + finish_id + " (epilogo automatico)" +
      (finish_use_docking ? "  (dock+align)" : "  (navegacao pura)"));
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
    const bool startup_home = node->declare_parameter<bool>("startup_home", true);
    const std::string startup_pose_name =
      node->declare_parameter<std::string>("startup_pose_name", "home");

    auto blackboard = BT::Blackboard::create();
    blackboard->set("max_staging_time", node->declare_parameter<double>("max_staging_time", 120.0));
    // 30 -> 60 (2026-08-18): o align cobre o approach inteiro + retries.
    // 60 -> 90 (2026-08-21, docking v4): 2 tentativas de ate 40 s (settle,
    // esquadro, centragem e pulsos) + recuo entre elas cabem em 90; o
    // GoToWS da +15 s e o recuo final usa esse tempo.
    blackboard->set("align_timeout", node->declare_parameter<double>("align_timeout", 90.0));
    // Fluxo novo do docking (2026-08-18): Nav2 ate a staging + align com muro
    // por LiDAR. true = ROLLBACK para o approach do opennav (/dock_robot).
    blackboard->set(
      "use_opennav_approach",
      node->declare_parameter<bool>("use_opennav_approach", false));
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
    // false = comportamento antigo (goto falhou => missao inteira falha).
    ctx.skip_unreachable_station =
      node->declare_parameter<bool>("skip_unreachable_station", true);

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
