// bt_yaml_executor: le a lista de acoes (goto/pick/place/home/amt1_sort)
// gerada pelo task_planner, monta a arvore de comportamento em XML e a executa
// com as folhas GoToWS, PickTag, PlaceTag, ReachQueue, NudgeBase, Amt1Sort e
// GoToNamedPose. Quem chama: caramelo_navigation/scripts/run_mission.py.
// Chaves desconhecidas de uma acao sao ignoradas; as do formato 2026
// (at_tag, shelf_level, target_order) sao lidas aqui — ver docs/guia_missoes.md §3.2.
//
// Uso: bt_yaml_executor <actions.yaml> [--dry-run] --ros-args -p map_folder:=...
//   --dry-run  so imprime o plano e o XML gerado (nao executa nada).
// Parametros principais (ros-args):
//   map_folder / map_name      pasta do mapa (docking.yaml, service_areas.yaml,
//                              ws_table_mapping.yaml); obrigatorio sem simulacao
//   simulate_navigation        GoToWS/NudgeBase viram mocks (manipulacao real)
//   finish_dock_id, startup_home, startup_pose_name
//   align_timeout, nav_timeout, max_staging_time, max_undocking_time,
//   use_lidar_refine, skip_align, use_opennav_approach, nudge_timeout
//   skip_unreachable_station   goto falhou => pula o bloco da estacao
//   pose_gate_enabled, pose_gate_max_x/y/yaw, pose_gate_retries,
//   pose_gate_strict           gate de pose apos o align (2026-10-26)
//   stop_check_enabled, stop_check_max_lin/max_ang/window_s/timeout_s,
//   stop_check_odom_max_age_s, stop_check_require_odom, odom_topic
//                              base parada antes da manipulacao
//   reach_queue_enabled, reach_queue_ws_prefixes, reach_queue_max_nudges,
//   reach_queue_max_shift_m, reach_queue_min_shift_m
//   amt1_timeout, amt1_stage_timeout, amt1_look_poses
//   retry_unfinished_at_end    (true) a arvore caiu ou deixou coisa por fazer
//                              => replaneja o que falta e segue; o epilogo
//                              (home + FINISH) roda sempre. false = arvore
//                              unica: falhou, acabou.
//   replan_max_rounds          (6) teto de rodadas de replanejamento
// Saida: 0 ok, 1 falha, 2 argumentos errados, 130 interrompido (Ctrl-C).

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "manip_bt/amt1_sort_bt.hpp"
#include "manip_bt/go_to_named_pose_bt.hpp"
#include "manip_bt/go_to_ws_bt.hpp"
#include "manip_bt/mission_map_config.hpp"
#include "manip_bt/nudge_base_bt.hpp"
#include "manip_bt/pick_tag_bt.hpp"
#include "manip_bt/place_tag_bt.hpp"
#include "manip_bt/reach_queue_bt.hpp"
#include "manip_bt/task_plan_search.hpp"

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
      rclcpp::get_logger("simulated_go_to_ws"),
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

// Mock do ajuste lateral (fila de alcance) para simulate_navigation:
// MESMAS portas do NudgeBase real. So loga e espera 1 s — a base NAO se move
// e nada e publicado (o /manip/base_shift_total e do dock_align_node), entao
// um pick/place "fora de alcance" continua fora de alcance ate a passada final.
class SimulatedNudgeBaseBT : public BT::SyncActionNode
{
public:
  SimulatedNudgeBaseBT(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("dy"),
      BT::InputPort<double>("timeout", 20.0, "timeout do goal (s)"),
      BT::InputPort<std::string>("ws", "", "estacao (so para o log)"),
    };
  }

  BT::NodeStatus tick() override
  {
    double dy = 0.0;
    getInput("dy", dy);
    std::string ws;
    getInput("ws", ws);

    RCLCPP_WARN(
      rclcpp::get_logger("simulated_nudge_base"),
      "[SIMULADO] ajuste lateral %+.2f m; base NAO se move (%s)", dy, ws.c_str());

    // Espera de 1 s em fatias de 100 ms para o Ctrl-C nao ficar preso.
    for (int i = 0; i < 10 && rclcpp::ok() && !g_interrupted; ++i) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok() || g_interrupted) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

// Caminho literal (absoluto ou relativo ao CWD); sem fallback por share.
std::string resolveActionsYamlPath(const std::string & input_path)
{
  if (std::filesystem::exists(input_path)) {
    return input_path;
  }
  throw std::runtime_error("actions.yaml nao encontrado: " + input_path);
}

void printUsage()
{
  std::cerr
    << "Uso: bt_yaml_executor <actions.yaml> [--dry-run] --ros-args -p map_folder:=/caminho\n"
    << "  actions.yaml  lista de acoes gerada pelo task_planner\n"
    << "  --dry-run     imprime o plano e o XML da arvore sem executar\n"
    << "  map_folder    pasta do mapa (ou -p map_name:=<mapa>); sem mapa so com\n"
    << "                -p simulate_navigation:=true" << std::endl;
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

// shelf_level (formato 2026: BOTTOM|TOP na SH_*). O pick/place da
// prateleira ainda nao usa o nivel: a chave e' aceita, mostrada no plano e
// IGNORADA, com um aviso (uma vez por execucao).
std::string readShelfLevel(const YAML::Node & action, const std::size_t action_index)
{
  if (!action["shelf_level"] || action["shelf_level"].IsNull()) {
    return "";
  }
  std::string level = action["shelf_level"].as<std::string>("");
  std::transform(
    level.begin(), level.end(), level.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  if (level.empty()) {
    return "";
  }
  static bool warned = false;
  if (!warned) {
    warned = true;
    RCLCPP_WARN(
      rclcpp::get_logger("bt_yaml_executor"),
      "actions[%zu] shelf_level=%s: nivel da prateleira ainda nao e' usado pelo "
      "pick/place (so informativo).", action_index, level.c_str());
  }
  return level;
}

struct MissionBuildContext
{
  manip_bt::MissionMapConfigPtr map_config;
  bool simulate_navigation{false};
  std::string finish_dock_id;
  bool startup_home{true};
  // 2026-10-27 (operador): braco em home antes de todo goto com docking —
  // a camera do docking so' ve a mesa nessa pose. O task_planner ja' emite
  // home antes de cada goto; isto cobre YAML de acoes escrito a mao. So'
  // insere se a acao anterior pode ter deixado o braco fora de home.
  bool home_before_dock{true};
  std::string startup_pose_name{"home"};
  // 2026-11-16 (AMT refeita): poses por onde o braco passa para a camera ver
  // a fileira INTEIRA da mesa de precisao (de pegar_obj cabem 5 dos 7 slots).
  // Em cada uma, menos a ultima, vai um <Amt1Sort observe_only> (o servidor so'
  // acumula a olhada); na ultima vai o <Amt1Sort> que ordena. Vazio = nao move
  // o braco (bancada sem MoveIt).
  std::string amt1_look_poses{"tag_esquerda,tag_direita,pegar_obj"};
  // 2026-11-17 ("disse que ordenou sem ter ordenado"): rodadas de olhar +
  // ordenar ate' o servidor responder success, que agora so' sai com a mesa
  // VISTA em ordem. Rodada 1 ordena, a 2 confere (e arruma o que faltou)...
  int amt1_rounds{4};
  // GoToWS falhou depois de todos os retries internos do align => PULA o
  // bloco da estacao (picks/places dela) e a missao segue, para nao perder
  // a task inteira por uma mesa inacessivel.
  bool skip_unreachable_station{true};
  // FILA DE ALCANCE por estacao (<ReachQueue>): picks/places de
  // uma mesa comum (prefixos abaixo) rodam numa fila que adia os "fora de
  // alcance", ajusta a base de lado (NudgeBase, ate max_nudges vezes) e
  // termina com a passada final. false = XML identico ao de antes.
  // Prefixos SEM '_' (2026): cobrem WS_1 e tambem PP (formato 2026 nomeia a
  // mesa de precisao sem sufixo).
  bool reach_queue_enabled{true};
  std::vector<std::string> reach_queue_ws_prefixes{"WS", "PP", "SH"};  // SH: alinhamento da base na shelf (2026-11-12)
  int reach_queue_max_nudges{2};
  double reach_queue_max_shift_m{0.25};
  double reach_queue_min_shift_m{0.10};
  // 2026-11-05: a missao roda em ate' 3 arvores (principal, re-tentativa do
  // que faltou e epilogo FINISH); cada montagem diz se leva o prologo (home
  // inicial) e o epilogo (home + goto FINISH).
  bool include_prologue{true};
  bool include_epilogue{true};
  std::vector<std::string> plan_rows;  // tabela do --dry-run
};

std::string formatMeters(const double value)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.2f", value);
  return buf;
}

bool wsHasReachQueuePrefix(const std::string & ws, const std::vector<std::string> & prefixes)
{
  for (const auto & prefix : prefixes) {
    if (!prefix.empty() && ws.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

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

  // name= vira o nome do no rclcpp da folha (bt_go_to_ws_goto_<i>).
  xml << "      <GoToWS name=\"goto_" << action_index << "\" ws=\""
      << escapeXmlAttr(blackboardPort(ws_key))
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
  xml << "      <AlwaysSuccess/>\n";  // arvore so' de epilogo/re-tentativa nunca fica vazia

  // Prologo: braco em home ANTES de qualquer acao (pedido do operador —
  // o robo pode bootar com o braco em pose qualquer; GoToNamedPose planeja
  // do estado atual via MoveIt, com colisoes da SRDF). Chave "prologue_*"
  // nunca colide com action_<i>_* nem com o epilogo (N/N+1).
  if (ctx.startup_home && ctx.include_prologue) {
    blackboard->set("prologue_pose_name", ctx.startup_pose_name);
    xml << "      <GoToNamedPose name=\"home_prologo\" pose_name=\""
        << escapeXmlAttr(blackboardPort("prologue_pose_name")) << "\"/>\n";
    ctx.plan_rows.push_back("[ini] " + ctx.startup_pose_name + " (prologo automatico)");
  }

  // Braco sabidamente em home (prologo/acao home) desde o ultimo pick/place?
  bool arm_at_home =
    ctx.include_prologue && ctx.startup_home && ctx.startup_pose_name == "home";
  std::string current_station_dock;  // ultima estacao visitada (p/ warnings)
  std::string current_station_ws;    // 'ws' do ultimo goto (p/ o place legado sem 'ws')
  std::string current_station_mesa;  // mesa do ultimo goto (fallback do amt1_sort)

  // Bloco de estacao com skip: cada goto de
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

  // Fila de alcance: abre no 1o pick/place depois do goto de uma
  // estacao elegivel (dock+align e prefixo WS/PP) e fecha — com o NudgeBase
  // como ultimo filho — em home, goto ou no fim das acoes. UMA fila por
  // estacao: um home no meio da estacao fecha a fila e os
  // picks/places seguintes da MESMA estacao saem FORA dela (XML antigo,
  // final_attempt=true) — o orcamento de max_nudges vale por estacao, nao
  // por fila.
  bool queue_open = false;
  bool station_queue_eligible = false;
  std::size_t station_goto_index = 0;
  std::string station_ws_key;
  std::string queue_final_key;
  std::string queue_shift_key;
  const auto close_reach_queue = [&]() {
      if (!queue_open) {
        return;
      }
      xml << "      <NudgeBase name=\"nudge_" << station_goto_index << "\" dy=\""
          << escapeXmlAttr(blackboardPort(queue_shift_key))
          << "\" timeout=\"{nudge_timeout}\""
          << " ws=\"" << escapeXmlAttr(blackboardPort(station_ws_key)) << "\"/>\n";
      xml << "      </ReachQueue>\n";
      queue_open = false;
      // A estacao ja gastou a fila dela: nada de reabrir com orcamento novo.
      station_queue_eligible = false;
    };
  const auto open_reach_queue_if_needed = [&]() {
      if (queue_open || !station_queue_eligible) {
        return;
      }
      const std::string queue_name = "rq_" + std::to_string(station_goto_index);
      queue_final_key = queue_name + "_final";
      queue_shift_key = queue_name + "_shift_m";
      // Tipos pre-gravados: o blackboard v3 trava o tipo de cada chave e as
      // folhas leem/escrevem bool e double nelas.
      blackboard->set(queue_final_key, false);
      blackboard->set(queue_shift_key, 0.0);
      xml << "      <ReachQueue name=\"" << queue_name
          << "\" ws=\"" << escapeXmlAttr(blackboardPort(station_ws_key))
          << "\" max_nudges=\"" << ctx.reach_queue_max_nudges
          << "\" max_shift_m=\"" << formatMeters(ctx.reach_queue_max_shift_m)
          << "\" min_shift_m=\"" << formatMeters(ctx.reach_queue_min_shift_m)
          << "\" final_attempt=\"" << escapeXmlAttr(blackboardPort(queue_final_key))
          << "\" shift_m=\"" << escapeXmlAttr(blackboardPort(queue_shift_key)) << "\">\n";
      queue_open = true;
      ctx.plan_rows.push_back(
        "[rq] fila de alcance " + current_station_ws + ": ate " +
        std::to_string(ctx.reach_queue_max_nudges) + " ajustes laterais de ate " +
        formatMeters(ctx.reach_queue_max_shift_m) + " m");
    };
  // Atributos extras das folhas DENTRO da fila (fora dela o XML fica igual).
  const auto reach_leaf_attrs = [&](const std::size_t action_index) -> std::string {
      if (!queue_open) {
        return "";
      }
      const std::string unreachable_key = actionBlackboardKey(action_index, "unreachable");
      const std::string shift_key = actionBlackboardKey(action_index, "shift_m");
      const std::string skipped_key = actionBlackboardKey(action_index, "skipped");
      blackboard->set(unreachable_key, false);
      blackboard->set(shift_key, 0.0);
      blackboard->set(skipped_key, false);
      return " final_attempt=\"" + escapeXmlAttr(blackboardPort(queue_final_key)) +
             "\" unreachable=\"" + escapeXmlAttr(blackboardPort(unreachable_key)) +
             "\" suggested_base_shift_m=\"" + escapeXmlAttr(blackboardPort(shift_key)) +
             "\" skipped=\"" + escapeXmlAttr(blackboardPort(skipped_key)) + "\"";
    };
  const auto reach_leaf_name = [&](const std::string & kind, const std::size_t action_index) {
      if (!queue_open) {
        return std::string();
      }
      return " name=\"" + kind + "_" + std::to_string(action_index) + "\"";
    };

  // Saida `completed` de todo pick/place (dentro ou fora da fila): a
  // re-tentativa do fim da missao le quem ficou por fazer.
  const auto completed_attr = [&](const std::size_t action_index) {
      const std::string key = actionBlackboardKey(action_index, "completed");
      blackboard->set(key, false);
      return " completed=\"" + escapeXmlAttr(blackboardPort(key)) + "\"";
    };

  for (std::size_t i = 0; i < actions.size(); ++i) {
    const YAML::Node action = actions[i];
    const std::string kind = action["kind"].as<std::string>("");
    if (kind == "home") {
      const std::string pose_name = action["pose_name"].as<std::string>("home");
      const std::string pose_name_key = actionBlackboardKey(i, "pose_name");
      setStringOnBlackboard(blackboard, i, "pose_name", pose_name);

      close_reach_queue();
      xml << "      <GoToNamedPose name=\"home_" << i << "\" pose_name=\""
          << escapeXmlAttr(blackboardPort(pose_name_key)) << "\"/>\n";
      arm_at_home = pose_name == "home";
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

      close_reach_queue();
      close_station_block();
      if (use_docking && ctx.home_before_dock && !arm_at_home) {
        // ForceSuccess: sem home o docking ainda funciona (so' LiDAR).
        const std::string pre_key = actionBlackboardKey(i, "pre_dock_pose");
        blackboard->set(pre_key, std::string("home"));
        xml << "      <ForceSuccess>\n";
        xml << "      <GoToNamedPose name=\"home_antes_" << i << "\" pose_name=\""
            << escapeXmlAttr(blackboardPort(pre_key)) << "\"/>\n";
        xml << "      </ForceSuccess>\n";
        ctx.plan_rows.push_back(
          "[" + std::to_string(i) + "] home  (automatico antes do docking: camera)");
        arm_at_home = true;
      }
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
      current_station_mesa = mesa;
      station_queue_eligible = ctx.reach_queue_enabled && use_docking &&
        wsHasReachQueuePrefix(ws, ctx.reach_queue_ws_prefixes);
      station_goto_index = i;
      station_ws_key = actionBlackboardKey(i, "ws");
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] goto  " + ws + " -> " + dock_id +
        (use_docking ? "  (dock+align)" : "  (navegacao pura)") +
        (wrap ? "  [falhou => pula a estacao]" : ""));
      continue;
    }

    if (kind == "pick" || kind == "place" || kind == "amt1_sort") {
      arm_at_home = false;
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

      // Repassa a mesa tambem no pick: a prateleira (MesaSh) tem
      // sequencia propria de movimentos. Opcional de proposito: actions.yaml
      // antigo (sem table_pose no pick) continua valendo e cai no
      // comportamento de mesa comum.
      std::string table_pose;
      if (action["table_pose"]) {
        table_pose = action["table_pose"].as<std::string>();
      }
      const std::string table_pose_key = actionBlackboardKey(i, "table_pose");
      setStringOnBlackboard(blackboard, i, "table_pose", table_pose);

      // shelf_level (formato 2026, BOTTOM|TOP): so informativo por enquanto —
      // o pick na prateleira ainda nao escolhe o nivel.
      const std::string shelf_level = readShelfLevel(action, i);

      open_reach_queue_if_needed();
      xml << "      <PickTag" << reach_leaf_name("pick", i)
          << " tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\" table_pose=\"" << escapeXmlAttr(blackboardPort(table_pose_key)) << "\""
          << reach_leaf_attrs(i) << completed_attr(i) << "/>\n";
      station_has_actions = station_has_actions || station_open;
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] pick  tag=" + tag_frame +
        (table_pose.empty() ? "" : " mesa=" + table_pose) +
        (shelf_level.empty() ? "" : " NIVEL " + shelf_level + " (informativo)"));
      continue;
    }

    if (kind == "place") {
      const std::string tag_frame = readRequiredString(action, i, "tag_frame");
      const std::string table_pose = readRequiredString(action, i, "table_pose");
      const std::string tag_frame_key = actionBlackboardKey(i, "tag_frame");
      const std::string table_pose_key = actionBlackboardKey(i, "table_pose");

      setStringOnBlackboard(blackboard, i, "tag_frame", tag_frame);
      setStringOnBlackboard(blackboard, i, "table_pose", table_pose);

      // O 'ws' viaja ate o no de place (escolha de slot na
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

      // EMPILHAR: `stack_on: tag_4` = soltar este objeto em cima
      // do objeto da tag_4 (ja na mesa da MESMA estacao). table_pose continua
      // obrigatorio (altura da mesa; fallback se a base nao for vista).
      // Validacao fail-fast: frame TF plausivel e diferente do proprio objeto.
      std::string stack_on;
      if (action["stack_on"]) {
        stack_on = action["stack_on"].as<std::string>("");
      }
      // SLOT DA MESA DE PRECISAO: `at_tag: tag_102` = soltar o cubo no furo
      // do slot dessa tag. Vai ao no de place pelo campo stack_on; la' o id
      // da tag (pp_slot_tag_ids) escolhe o caminho da PP (2026-11-16, refeito
      // do zero: chegada em paradas com a camera medindo o furo).
      std::string at_tag;
      if (action["at_tag"]) {
        at_tag = action["at_tag"].as<std::string>("");
      }
      if (!at_tag.empty()) {
        if (at_tag.rfind("tag_", 0) != 0 || at_tag == tag_frame) {
          throw std::runtime_error(
            "action[" + std::to_string(i) + "]: at_tag='" + at_tag +
            "' invalido (esperado tag_N diferente de tag_frame)");
        }
        if (!stack_on.empty()) {
          throw std::runtime_error(
            "action[" + std::to_string(i) + "]: at_tag e stack_on juntos nao fazem sentido "
            "(soltar no slot da mesa de precisao OU em cima de um bloco)");
        }
        stack_on = at_tag;
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

      // shelf_level (formato 2026): so informativo — ver o pick.
      const std::string shelf_level = readShelfLevel(action, i);

      // CONTAINER POR COR: `container_color: RED|BLUE` = soltar
      // dentro do container dessa cor visto pela camera na mesa de destino
      // (fallback: mesa). Fail-fast: cor conhecida e nunca junto com stack_on.
      std::string container_color;
      if (action["container_color"]) {
        container_color = action["container_color"].as<std::string>("");
        std::transform(
          container_color.begin(), container_color.end(), container_color.begin(),
          [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        while (!container_color.empty() && std::isspace(
            static_cast<unsigned char>(container_color.back())))
        {
          container_color.pop_back();
        }
        while (!container_color.empty() && std::isspace(
            static_cast<unsigned char>(container_color.front())))
        {
          container_color.erase(container_color.begin());
        }
      }
      if (!container_color.empty()) {
        if (container_color != "RED" && container_color != "BLUE") {
          throw std::runtime_error(
            "action[" + std::to_string(i) + "]: container_color='" + container_color +
            "' invalido (esperado RED|BLUE)");
        }
        if (!stack_on.empty()) {
          throw std::runtime_error(
            "action[" + std::to_string(i) + "]: container_color e " +
            (at_tag.empty() ? "stack_on" : "at_tag") + " juntos nao fazem sentido (soltar "
            "num container OU em cima de um bloco / no slot da mesa de precisao)");
        }
      }
      const std::string container_color_key = actionBlackboardKey(i, "container_color");
      setStringOnBlackboard(blackboard, i, "container_color", container_color);

      // SUPERFICIE LIVRE: paredes ao lado da mesa (rulebook 4.4:
      // 10 cm da parede). Preferencia: `wall_sides` da propria acao (lista ou
      // string "left,right"); senao o bloco ws_walls do ws_table_mapping.yaml
      // do mapa para a estacao do place; senao vazio (sem parede).
      std::string wall_sides;
      if (action["wall_sides"]) {
        if (action["wall_sides"].IsSequence()) {
          for (const auto & t : action["wall_sides"]) {
            const std::string tok = t.as<std::string>("");
            if (!tok.empty()) {
              wall_sides += (wall_sides.empty() ? "" : ",") + tok;
            }
          }
        } else {
          wall_sides = action["wall_sides"].as<std::string>("");
        }
      } else if (ctx.map_config && !place_ws.empty()) {
        wall_sides = ctx.map_config->wallSidesForWs(place_ws);
      }
      {
        std::string lowered = wall_sides;
        std::transform(lowered.begin(), lowered.end(), lowered.begin(),
          [](unsigned char c) {return static_cast<char>(std::tolower(c));});
        std::string tok;
        for (const char c : lowered + ",") {
          if (c == ',' || c == ';' || c == ' ') {
            if (!tok.empty() && tok != "left" && tok != "right" && tok != "esquerda" &&
              tok != "direita" && tok != "both" && tok != "none")
            {
              throw std::runtime_error(
                      "action[" + std::to_string(i) + "]: wall_sides='" + wall_sides +
                      "' invalido (esperado left|right|left,right)");
            }
            tok.clear();
          } else {
            tok += c;
          }
        }
      }
      const std::string wall_sides_key = actionBlackboardKey(i, "wall_sides");
      setStringOnBlackboard(blackboard, i, "wall_sides", wall_sides);

      open_reach_queue_if_needed();
      xml << "      <PlaceTag" << reach_leaf_name("place", i)
          << " tag_frame=\"" << escapeXmlAttr(blackboardPort(tag_frame_key))
          << "\" table_pose=\"" << escapeXmlAttr(blackboardPort(table_pose_key))
          << "\" ws=\"" << escapeXmlAttr(blackboardPort(ws_key))
          << "\" stack_on=\"" << escapeXmlAttr(blackboardPort(stack_on_key))
          << "\" container_color=\"" << escapeXmlAttr(blackboardPort(container_color_key))
          << "\" wall_sides=\"" << escapeXmlAttr(blackboardPort(wall_sides_key))
          << "\"" << reach_leaf_attrs(i) << completed_attr(i) << "/>\n";
      station_has_actions = station_has_actions || station_open;
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] place tag=" + tag_frame + " mesa=" + table_pose +
        (place_ws.empty() ? "" : " ws=" + place_ws) +
        (!at_tag.empty() ? " NO SLOT " + at_tag + " (via stack_on)" :
        stack_on.empty() ? "" : " EMPILHAR sobre " + stack_on) +
        (container_color.empty() ? "" : " NO CONTAINER " + container_color) +
        (wall_sides.empty() ? "" : " PAREDE " + wall_sides) +
        (shelf_level.empty() ? "" : " NIVEL " + shelf_level + " (informativo)"));
      continue;
    }

    // AMT1: ordenar as tags da mesa de precisao. UMA folha
    // <Amt1Sort/> cliente do /amt1_sort (a rotina inteira — observar,
    // planejar, picks/places nos slots e nudge — roda no server). Exige um
    // goto anterior: a estacao e a do ultimo goto; fecha a fila de alcance
    // ANTES (o ReachQueue exige NudgeBase como ultimo filho) e entra no bloco
    // da estacao (goto falhou => estacao pulada, missao segue ao FINISH).
    if (kind == "amt1_sort") {
      if (current_station_ws.empty()) {
        throw std::runtime_error(
                "actions[" + std::to_string(i) + "] amt1_sort sem goto anterior — a "
                "ordenacao precisa da estacao (PP_1) do ultimo goto.");
      }
      std::string sort_ws;
      if (action["ws"]) {
        sort_ws = action["ws"].as<std::string>("");
      }
      if (sort_ws.empty()) {
        sort_ws = current_station_ws;
      } else if (sort_ws != current_station_ws) {
        throw std::runtime_error(
                "actions[" + std::to_string(i) + "] amt1_sort ws=" + sort_ws +
                " mas o ultimo goto foi para " + current_station_ws + ".");
      }

      std::string table_pose;
      if (action["table_pose"]) {
        table_pose = action["table_pose"].as<std::string>("");
      }
      if (table_pose.empty()) {
        table_pose = current_station_mesa;
      }
      if (table_pose.empty()) {
        throw std::runtime_error(
                "actions[" + std::to_string(i) + "] amt1_sort sem table_pose (nem o goto "
                "anterior tem 'mesa').");
      }

      // expected_tags: lista de ints no yaml (task_planner) ou string "1,2,3".
      // Vira a string "1,2,3" na blackboard — a folha converte para int32[].
      // Qualquer erro de leitura sai com o indice da acao.
      std::vector<int> expected_tags;
      const YAML::Node tags_node = action["expected_tags"];
      try {
        if (tags_node && tags_node.IsSequence()) {
          for (const auto & t : tags_node) {
            try {
              expected_tags.push_back(t.as<int>());
            } catch (const YAML::Exception &) {
              throw std::runtime_error("'" + YAML::Dump(t) + "' nao e inteiro");
            }
          }
        } else if (tags_node && tags_node.IsScalar()) {
          for (const int id :
            manip_bt::Amt1SortBT::parseExpectedTags(tags_node.as<std::string>()))
          {
            expected_tags.push_back(id);
          }
        } else if (tags_node && !tags_node.IsNull()) {
          throw std::runtime_error("esperado lista de ints ou string \"1,2,3\"");
        }
      } catch (const std::exception & ex) {
        // parseExpectedTags ja prefixa "expected_tags: " — nao duplicar.
        std::string what = ex.what();
        const std::string prefix = "expected_tags: ";
        if (what.rfind(prefix, 0) == 0) {
          what = what.substr(prefix.size());
        }
        throw std::runtime_error(
                "actions[" + std::to_string(i) + "] amt1_sort expected_tags: " + what);
      }
      std::string expected_csv;
      for (std::size_t k = 0; k < expected_tags.size(); ++k) {
        if (expected_tags[k] < 0) {
          throw std::runtime_error(
                  "actions[" + std::to_string(i) + "] amt1_sort expected_tags com id negativo.");
        }
        if (std::find(expected_tags.begin(), expected_tags.begin() + k, expected_tags[k]) !=
          expected_tags.begin() + k)
        {
          throw std::runtime_error(
                  "actions[" + std::to_string(i) + "] amt1_sort expected_tags com id repetido " +
                  std::to_string(expected_tags[k]) + ".");
        }
        expected_csv += (k == 0 ? "" : ",") + std::to_string(expected_tags[k]);
      }
      if (expected_tags.empty()) {
        RCLCPP_WARN(
          rclcpp::get_logger("bt_yaml_executor"),
          "actions[%zu] amt1_sort sem expected_tags — o no ordena tudo o que vir na mesa.", i);
      }

      // target_order (formato 2026): alvo EXPLICITO esquerda -> direita, lista
      // de ints ou string "1,2,3". Vazio/ausente = ordem crescente de id
      // (comportamento de sempre). Tem que ser permutacao de expected_tags.
      std::vector<int> target_order;
      const YAML::Node order_node = action["target_order"];
      try {
        if (order_node && order_node.IsSequence()) {
          for (const auto & t : order_node) {
            try {
              target_order.push_back(t.as<int>());
            } catch (const YAML::Exception &) {
              throw std::runtime_error("'" + YAML::Dump(t) + "' nao e inteiro");
            }
          }
        } else if (order_node && order_node.IsScalar()) {
          for (const int id :
            manip_bt::Amt1SortBT::parseTargetOrder(order_node.as<std::string>()))
          {
            target_order.push_back(id);
          }
        } else if (order_node && !order_node.IsNull()) {
          throw std::runtime_error("esperado lista de ints ou string \"1,2,3\"");
        }
      } catch (const std::exception & ex) {
        std::string what = ex.what();
        const std::string prefix = "expected_tags: ";
        if (what.rfind(prefix, 0) == 0) {
          what = what.substr(prefix.size());
        }
        throw std::runtime_error(
                "actions[" + std::to_string(i) + "] amt1_sort target_order: " + what);
      }
      // 2026-11-26 (operador: "na AMT o slot vazio pode ser em qualquer
      // lugar"): -1 = slot vazio naquela posicao; pode repetir e nao conta
      // na permutacao.
      std::string target_csv;
      std::size_t target_ids = 0;
      for (std::size_t k = 0; k < target_order.size(); ++k) {
        if (target_order[k] == -1) {
          target_csv += (k == 0 ? "" : ",") + std::string("-1");
          continue;
        }
        ++target_ids;
        if (std::find(target_order.begin(), target_order.begin() + k, target_order[k]) !=
          target_order.begin() + k)
        {
          throw std::runtime_error(
                  "actions[" + std::to_string(i) + "] amt1_sort target_order com id repetido " +
                  std::to_string(target_order[k]) + ".");
        }
        if (!expected_tags.empty() &&
          std::find(expected_tags.begin(), expected_tags.end(), target_order[k]) ==
          expected_tags.end())
        {
          throw std::runtime_error(
                  "actions[" + std::to_string(i) + "] amt1_sort target_order tem o id " +
                  std::to_string(target_order[k]) + " que nao esta em expected_tags.");
        }
        target_csv += (k == 0 ? "" : ",") + std::to_string(target_order[k]);
      }
      if (!target_order.empty() && target_ids != expected_tags.size()) {
        throw std::runtime_error(
                "actions[" + std::to_string(i) + "] amt1_sort target_order (" +
                std::to_string(target_ids) + " ids) nao e permutacao de expected_tags (" +
                std::to_string(expected_tags.size()) + " ids).");
      }

      std::string direction = action["direction"].as<std::string>("left_to_right");
      if (direction.empty()) {
        direction = "left_to_right";
      }
      if (direction != "left_to_right" && direction != "right_to_left") {
        throw std::runtime_error(
                "actions[" + std::to_string(i) + "] amt1_sort direction='" + direction +
                "' invalida (left_to_right | right_to_left).");
      }
      // max_onboard: 0..3, como no Amt1Sort.action e no no (0 = todos os
      // containers vazios). Chave ausente => 3. Valor nao-inteiro ("tres") e
      // erro com o indice da acao (as<int>(3) engoliria e viraria 3 em
      // silencio).
      int max_onboard = 3;
      if (action["max_onboard"] && !action["max_onboard"].IsNull()) {
        try {
          max_onboard = action["max_onboard"].as<int>();
        } catch (const YAML::Exception &) {
          throw std::runtime_error(
                  "actions[" + std::to_string(i) + "] amt1_sort max_onboard=" +
                  YAML::Dump(action["max_onboard"]) + " nao e inteiro.");
        }
      }
      if (max_onboard < 0 || max_onboard > 3) {
        throw std::runtime_error(
                "actions[" + std::to_string(i) + "] amt1_sort max_onboard=" +
                std::to_string(max_onboard) + " fora de 0..3 (0 = todos os containers vazios).");
      }

      close_reach_queue();

      const std::string ws_key = actionBlackboardKey(i, "ws");
      const std::string table_pose_key = actionBlackboardKey(i, "table_pose");
      const std::string tags_key = actionBlackboardKey(i, "expected_tags");
      const std::string direction_key = actionBlackboardKey(i, "direction");
      const std::string max_onboard_key = actionBlackboardKey(i, "max_onboard");
      setStringOnBlackboard(blackboard, i, "ws", sort_ws);
      setStringOnBlackboard(blackboard, i, "table_pose", table_pose);
      setStringOnBlackboard(blackboard, i, "expected_tags", expected_csv);
      setStringOnBlackboard(blackboard, i, "direction", direction);
      blackboard->set(max_onboard_key, max_onboard);

      // target_order so entra no XML quando existe: sem ele o XML fica
      // byte a byte igual ao de antes (bancada run_amt1_scenario.sh).
      std::string target_attr;
      if (!target_csv.empty()) {
        const std::string target_key = actionBlackboardKey(i, "target_order");
        setStringOnBlackboard(blackboard, i, "target_order", target_csv);
        target_attr = " target_order=\"" + escapeXmlAttr(blackboardPort(target_key)) + "\"";
      }

      // 2026-11-16 (AMT refeita sobre a PP nova): o servidor /amt1_sort nao
      // move o braco — so' olha pela camera e manda pick/place. O braco passa
      // aqui pelas poses de olhar (amt1_look_poses).
      const auto amt_leaf = [&](const std::string & name, const bool observe) {
          xml << "      <Amt1Sort name=\"" << name
              << "\" ws=\"" << escapeXmlAttr(blackboardPort(ws_key))
              << "\" table_pose=\"" << escapeXmlAttr(blackboardPort(table_pose_key))
              << "\" expected_tags=\"" << escapeXmlAttr(blackboardPort(tags_key))
              << "\" direction=\"" << escapeXmlAttr(blackboardPort(direction_key))
              << "\" max_onboard=\"" << escapeXmlAttr(blackboardPort(max_onboard_key))
              << "\"" << target_attr << (observe ? " observe_only=\"true\"" : " confirm=\"true\"")
              << " timeout=\"{amt1_timeout}\" stage_timeout=\"{amt1_stage_timeout}\"/>\n";
        };
      std::vector<std::string> looks;
      {
        std::stringstream ss(ctx.amt1_look_poses);
        for (std::string pose; std::getline(ss, pose, ','); ) {
          if (!pose.empty()) {
            looks.push_back(pose);
          }
        }
      }
      // ForceSuccess: esgotadas as rodadas a missao segue para o FINISH, como antes.
      xml << "      <ForceSuccess>\n      <RetryUntilSuccessful num_attempts=\""
          << std::max(1, ctx.amt1_rounds) << "\">\n      <Sequence>\n";
      for (std::size_t k = 0; k < looks.size(); ++k) {
        xml << "      <GoToNamedPose name=\"amt1_olhar_" << i << "_" << k << "\" pose_name=\""
            << escapeXmlAttr(looks[k]) << "\"/>\n";
        if (k + 1 < looks.size()) {
          amt_leaf("amt1_olhada_" + std::to_string(i) + "_" + std::to_string(k), true);
        }
      }
      amt_leaf("amt1_" + std::to_string(i), false);
      xml << "      </Sequence>\n      </RetryUntilSuccessful>\n      </ForceSuccess>\n";
      station_has_actions = station_has_actions || station_open;
      ctx.plan_rows.push_back(
        "[" + std::to_string(i) + "] amt1  ordenar " + std::to_string(expected_tags.size()) +
        " tags na " + sort_ws + " (mesa=" + table_pose + ", " + direction +
        ", max_onboard=" + std::to_string(max_onboard) + ")" +
        (target_csv.empty() ? "" : " alvo explicito [" + target_csv + "]"));
      continue;
    }

    throw std::runtime_error("actions[" + std::to_string(i) + "] has unsupported kind: " + kind);
  }

  // Fecha a fila de alcance e o bloco da ultima estacao ANTES do epilogo: o
  // home final e o goto FINISH rodam SEMPRE, mesmo com a ultima estacao pulada.
  close_reach_queue();
  close_station_block();

  // Epilogo: braco em home e ida ao FINISH (o undock de saida da ultima
  // estacao acontece dentro do proprio GoToWS). O task_planner nao emite o
  // FINISH — e' responsabilidade do executor fechar a missao.
  if (!ctx.finish_dock_id.empty() && ctx.include_epilogue) {
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
    xml << "      <GoToNamedPose name=\"home_" << home_index << "\" pose_name=\""
        << escapeXmlAttr(blackboardPort(pose_name_key)) << "\"/>\n";
    // O docking do FINISH pergunta ao mapa (mesmo fallback do goto normal):
    // se a arena definir o FINISH como estacao com dock, o epilogo encosta.
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

// ---- re-tentativa no fim da missao (2026-11-05) ------------------------------
// Pedido do operador: "se ele nao conseguir chegar em uma ws, tentar de novo
// no final e fazer o que tem que ser feito". Depois da arvore principal, o
// que nao foi CONCLUIDO (saida `completed` das folhas): pick/place de
// estacao pulada (goto falhou), pick pulado, place pulado (objeto nao
// estava a bordo porque o pick dele nao aconteceu, base da pilha nao vista,
// fora de alcance) vira um plano novo pelo MESMO planejador
// (task_plan_search: capacidade 3, pilha, menos trocas de estacao), partindo
// do que ja esta feito e do que esta a bordo. As acoes emitidas sao COPIAS
// das originais (mesma mesa, slot, container, pilha). amt1_sort nao e'
// re-tentado.
// 2026-11-16 (replanejamento no meio da missao): `actions_root` e' sempre o
// plano MESTRE (o original) e `done` os indices dele ja concluidos; cada acao
// emitida leva `_orig` = indice na mestra. `hold_tags` = objetos fora desta
// rodada (o que acabou de falhar, ou de que ja desistimos): se estao a bordo
// entram como PASSAGEIROS (ocupam um pote, sem acao), e quem empilha em cima
// deles fica fora junto. `start_ws` = estacao onde o robo esta ("" = ultimo
// goto do plano, o caso do fim da missao).
std::optional<YAML::Node> buildRetryActions(
  const YAML::Node & actions_root, const std::set<std::size_t> & done,
  const std::set<std::string> & hold_tags, const std::string & start_ws,
  std::vector<std::string> & report)
{
  namespace tp = manip_bt::task_plan;
  const YAML::Node actions = actions_root["actions"];
  struct Obj
  {
    int id{-1};
    int pick_idx{-1};
    int place_idx{-1};
    bool pick_done{false};
    bool place_done{false};
    std::string from_ws;
    std::string to_ws;
    std::string stack_on;
  };
  std::map<std::string, Obj> objs;   // por tag_frame
  std::map<std::string, YAML::Node> goto_of_ws;
  std::string cur_ws;
  int next_synthetic = 100000;
  const auto completed = [&](const std::size_t i) {return done.count(i) > 0;};
  const auto obj_for = [&](const std::string & tag) -> Obj & {
      Obj & o = objs[tag];
      if (o.id < 0) {
        int id = -1;
        if (tag.rfind("tag_", 0) == 0) {
          try {
            id = std::stoi(tag.substr(4));
          } catch (const std::exception &) {
            id = -1;
          }
        }
        o.id = id >= 0 ? id : next_synthetic++;
      }
      return o;
    };
  for (std::size_t i = 0; i < actions.size(); ++i) {
    const YAML::Node a = actions[i];
    const std::string kind = a["kind"].as<std::string>("");
    if (kind == "goto") {
      cur_ws = a["ws"].as<std::string>("");
      if (goto_of_ws.count(cur_ws) == 0) {
        goto_of_ws[cur_ws] = YAML::Clone(a);
      }
    } else if (kind == "pick") {
      Obj & o = obj_for(a["tag_frame"].as<std::string>(""));
      o.pick_idx = static_cast<int>(i);
      o.pick_done = completed(i);
      o.from_ws = cur_ws;
    } else if (kind == "place") {
      Obj & o = obj_for(a["tag_frame"].as<std::string>(""));
      o.place_idx = static_cast<int>(i);
      o.place_done = completed(i);
      // Estacao do goto (e' por ela que a re-tentativa navega).
      o.to_ws = cur_ws;
      o.stack_on = a["stack_on"].as<std::string>("");
    }
  }

  // Fora desta rodada: os de hold_tags e quem empilha em cima deles (a base
  // nao vai estar na mesa).
  std::set<std::string> hold = hold_tags;
  for (bool grew = true; grew; ) {
    grew = false;
    for (const auto & [tag, o] : objs) {
      if (hold.count(tag) == 0 && !o.stack_on.empty() && hold.count(o.stack_on) > 0) {
        hold.insert(tag);
        grew = true;
      }
    }
  }

  std::vector<tp::TransferItem> transfers;
  tp::PlannerState state;
  std::map<int, const Obj *> by_id;
  std::string held;   // para o log
  for (const auto & [tag, o] : objs) {
    const bool needs_pick = o.pick_idx >= 0 && !o.pick_done;
    const bool needs_place = o.place_idx >= 0 && !o.place_done;
    if (!needs_pick && !needs_place) {
      continue;
    }
    if (needs_place && o.to_ws.empty()) {
      continue;
    }
    if (hold.count(tag) > 0) {
      held += (held.empty() ? "" : ", ") + tag;
      if (!needs_pick) {
        // Passageiro: a bordo, sem pick nem place nesta rodada. O planejador
        // so' o enxerga na capacidade (inventario).
        tp::TransferItem p;
        p.obj_id = o.id;
        p.tag_frame = tag;
        state.picked.insert(o.id);
        state.inventory.push_back(o.id);
        transfers.push_back(p);
      }
      continue;
    }
    tp::TransferItem t;
    t.obj_id = o.id;
    t.tag_frame = tag;
    // So' estacoes com trabalho pendente (a busca indexa candidateStations).
    t.from_ws = needs_pick ? o.from_ws : "";
    t.to_ws = needs_place ? o.to_ws : "";
    t.needs_pick = needs_pick;
    t.needs_place = needs_place;
    if (needs_place && !needs_pick) {
      // Pego (ou ja a bordo) e nao solto: esta no container de bordo.
      state.picked.insert(o.id);
      state.inventory.push_back(o.id);
    }
    transfers.push_back(t);
    by_id[o.id] = &o;
  }
  if (by_id.empty()) {
    report.push_back(
      held.empty() ? "[re-tentativa] nada pendente." :
      "[re-tentativa] so' resta o que ficou de fora: " + held);
    return std::nullopt;
  }
  // Pilha: topo pendente sobre base que tambem esta pendente.
  for (auto & t : transfers) {
    if (by_id.count(t.obj_id) == 0) {
      continue;   // passageiro
    }
    const Obj & o = *by_id.at(t.obj_id);
    if (!o.stack_on.empty() && objs.count(o.stack_on) > 0) {
      const int base_id = objs.at(o.stack_on).id;
      if (by_id.count(base_id) > 0) {
        t.stack_on_obj = base_id;
        t.stack_on_frame = o.stack_on;
      }
    }
  }
  if (static_cast<int>(state.inventory.size()) > tp::kMaxOnboard) {
    report.push_back("[re-tentativa] mais objetos a bordo que a capacidade - nao re-tento.");
    return std::nullopt;
  }

  std::vector<tp::Visit> visits;
  try {
    std::mt19937 rng(0);
    visits = tp::planGreedy(transfers, rng, state);
    // Distancia uniforme: minimiza trocas de estacao a partir de onde o robo esta.
    tp::CostModel model;
    model.start_ws = !start_ws.empty() ? start_ws : cur_ws.empty() ? "START" : cur_ws;
    std::set<std::string> names = tp::candidateStations(transfers);
    names.insert(model.start_ws);
    names.insert(model.finish_ws);
    for (const auto & x : names) {
      for (const auto & y : names) {
        if (x < y) {
          model.dist_m[x][y] = 3.0;
        }
      }
    }
    visits = tp::planOptimized(transfers, model, visits, 1.0, state).visits;
  } catch (const std::exception & ex) {
    report.push_back(std::string("[re-tentativa] sem plano para o que faltou: ") + ex.what());
    return std::nullopt;
  }

  YAML::Node out;
  YAML::Node seq(YAML::NodeType::Sequence);
  std::string summary;
  std::string last_ws;
  tp::PlannerState sim = state;
  for (const tp::Visit & v : visits) {
    if (v.ws != last_ws) {
      const auto g = goto_of_ws.find(v.ws);
      if (g == goto_of_ws.end()) {
        report.push_back("[re-tentativa] estacao " + v.ws + " sem goto no plano - ignorada.");
        continue;
      }
      seq.push_back(YAML::Clone(g->second));
      last_ws = v.ws;
      summary += (summary.empty() ? "" : " | ") + v.ws + ":";
    }
    const auto emit = [&](const int idx, const std::string & verb, const int id) {
        YAML::Node a = YAML::Clone(actions[static_cast<std::size_t>(idx)]);
        a["_orig"] = idx;   // harvestCompleted devolve o resultado ao plano mestre
        seq.push_back(a);
        summary += " " + verb + " " + tp::findTransferById(transfers, id).tag_frame;
      };
    // Pick primeiro na mesma visita (2026-11-19), a partir do estado real
    // (o que ja esta a bordo).
    for (const tp::VisitStep & step : tp::orderVisitSteps(transfers, sim, v)) {
      if (step.is_pick) {
        emit(by_id.at(step.obj_id)->pick_idx, "pick", step.obj_id);
      } else {
        emit(by_id.at(step.obj_id)->place_idx, "place", step.obj_id);
      }
    }
  }
  if (seq.size() == 0) {
    return std::nullopt;
  }
  out["actions"] = seq;
  std::string pending;
  for (const auto & t : transfers) {
    if (by_id.count(t.obj_id) == 0) {
      continue;   // passageiro
    }
    pending += (pending.empty() ? "" : ", ") + t.tag_frame +
      (t.needs_pick ? " (pick" : " (a bordo") + (t.needs_place ? " + place)" : ")");
  }
  report.push_back("[re-tentativa] pendente: " + pending);
  if (!held.empty()) {
    report.push_back("[re-tentativa] fora desta rodada: " + held);
  }
  report.push_back("[re-tentativa] plano: " + summary);
  return out;
}

// Soma a `done` (indices do plano mestre) os picks/places que a arvore que
// acabou de rodar concluiu; devolve quantos sao novos. Acoes copiadas pelo
// replanejamento levam `_orig`; sem ele a lista e' o proprio plano mestre.
std::size_t harvestCompleted(
  const YAML::Node & ran_root, const BT::Blackboard::Ptr & blackboard,
  std::set<std::size_t> & done)
{
  const YAML::Node actions = ran_root["actions"];
  std::size_t novos = 0;
  for (std::size_t j = 0; j < actions.size(); ++j) {
    const std::string kind = actions[j]["kind"].as<std::string>("");
    if (kind != "pick" && kind != "place") {
      continue;
    }
    bool completed = false;
    try {
      blackboard->get(actionBlackboardKey(j, "completed"), completed);
    } catch (const std::exception &) {
      completed = false;
    }
    if (completed && done.insert(actions[j]["_orig"].as<std::size_t>(j)).second) {
      ++novos;
    }
  }
  return novos;
}

// Estacao da primeira acao ainda pendente (pick ou place) do objeto `tag` no
// plano mestre; nullopt se ele nao tem mais nada a fazer.
std::optional<std::string> pendingWsOfTag(
  const YAML::Node & actions_root, const std::set<std::size_t> & done, const std::string & tag)
{
  const YAML::Node actions = actions_root["actions"];
  std::string cur_ws;
  for (std::size_t i = 0; i < actions.size(); ++i) {
    const std::string kind = actions[i]["kind"].as<std::string>("");
    if (kind == "goto") {
      cur_ws = actions[i]["ws"].as<std::string>("");
    } else if ((kind == "pick" || kind == "place") && done.count(i) == 0 &&
      actions[i]["tag_frame"].as<std::string>("") == tag)
    {
      return cur_ws;
    }
  }
  return std::nullopt;
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
    std::string input_yaml;
    bool dry_run = false;
    for (std::size_t i = 1; i < args.size(); ++i) {
      if (args[i] == "--dry-run") {
        dry_run = true;
      } else if (!args[i].empty() && args[i][0] != '-') {
        input_yaml = args[i];
      }
    }
    if (input_yaml.empty()) {
      printUsage();
      rclcpp::shutdown();
      return 2;
    }

    auto node = std::make_shared<rclcpp::Node>("bt_yaml_executor");
    const std::string map_folder_param =
      node->declare_parameter<std::string>("map_folder", "");
    const std::string map_name_param =
      node->declare_parameter<std::string>("map_name", "");
    const bool simulate_navigation =
      node->declare_parameter<bool>("simulate_navigation", false);
    const std::string finish_dock_id =
      node->declare_parameter<std::string>("finish_dock_id", "FINISH");
    const bool startup_home = node->declare_parameter<bool>("startup_home", true);
    const bool home_before_dock = node->declare_parameter<bool>("home_before_dock", true);
    const std::string startup_pose_name =
      node->declare_parameter<std::string>("startup_pose_name", "home");

    auto blackboard = BT::Blackboard::create();
    blackboard->set("max_staging_time", node->declare_parameter<double>("max_staging_time", 120.0));
    // 2026-11-11: perna curta (GoToWS). O XML default vem do share do
    // caramelo_navigation; vazio desliga.
    blackboard->set(
      "staging_short_dist", node->declare_parameter<double>("staging_short_dist", 0.30));
    // 2026-11-11: re-tentativa da staging (GoToWS) com nomotion update do AMCL.
    blackboard->set(
      "staging_nav_retries",
      static_cast<int>(node->declare_parameter<int64_t>("staging_nav_retries", 2)));
    blackboard->set(
      "staging_nav_retry_wait_s",
      node->declare_parameter<double>("staging_nav_retry_wait_s", 4.0));
    blackboard->set(
      "staging_nav_retry_nomotion_calls",
      static_cast<int>(node->declare_parameter<int64_t>("staging_nav_retry_nomotion_calls", 3)));
    {
      std::string curta_xml = node->declare_parameter<std::string>("staging_short_bt_xml", "");
      if (curta_xml.empty()) {
        try {
          curta_xml = ament_index_cpp::get_package_share_directory("caramelo_navigation") +
            "/behavior_tree/caramelo_lattice_dwb_curta.xml";
        } catch (const std::exception & e) {
          RCLCPP_WARN(node->get_logger(),
            "caramelo_navigation nao encontrado (%s): perna curta desligada.", e.what());
        }
      }
      blackboard->set("staging_short_bt_xml", curta_xml);
    }
    // 90 s: o align faz 2 tentativas de ate 40 s (settle, esquadro, centragem
    // e pulsos) + recuo entre elas; o GoToWS da +15 s e o recuo final usa
    // esse tempo (medido 2026-08-21, docking v4).
    blackboard->set("align_timeout", node->declare_parameter<double>("align_timeout", 90.0));
    // Docking = Nav2 ate a staging + align com muro por LiDAR.
    // true = ROLLBACK para o approach do opennav (/dock_robot).
    blackboard->set(
      "use_opennav_approach",
      node->declare_parameter<bool>("use_opennav_approach", false));
    blackboard->set(
      "max_undocking_time", node->declare_parameter<double>("max_undocking_time", 30.0));
    blackboard->set("nav_timeout", node->declare_parameter<double>("nav_timeout", 180.0));
    blackboard->set(
      "server_wait_timeout", node->declare_parameter<double>("server_wait_timeout", 10.0));
    // Default true: sem refino a missao alinha so pelo AMCL e herda o
    // offset global (~cm) na hora critica. Desligar (debug):
    //   --ros-args -p use_lidar_refine:=false
    blackboard->set("use_lidar_refine", node->declare_parameter<bool>("use_lidar_refine", true));
    blackboard->set("skip_align", node->declare_parameter<bool>("skip_align", false));
    // Fila de alcance: timeout do goal nudge_base (a base anda
    // <= 0,25 m; o servidor tem o proprio nudge_default_timeout quando 0).
    blackboard->set("nudge_timeout", node->declare_parameter<double>("nudge_timeout", 20.0));
    // 2026-11-12 (pedido do operador): o 1o GoToWS fala "START <task>" pouco
    // antes de a base andar. Nome vem do run_mission (-p task_name:=BMT).
    blackboard->set("task_name", node->declare_parameter<std::string>("task_name", ""));
    // Gate de pose do GoToWS (2026-10-26, runs de 22-23/10: o docking devolvia
    // sucesso com y +0,30 m / yaw +18 graus e a manipulacao comecava). Limites
    // DUROS justificados em include/manip_bt/pose_gate.hpp: |y| <= 0,20 m
    // (corpo dentro da largura da mesa), |yaw| <= 0,20 rad (= tolerancia de
    // yaw do Nav2; place por superficie erra <= 3 cm na zona). Fora deles:
    // re-docka (undock + staging + align) pose_gate_retries vezes e, ainda
    // fora, falha a estacao (strict) ou segue com aviso (strict=false, o
    // comportamento antigo).
    blackboard->set("pose_gate_enabled", node->declare_parameter<bool>("pose_gate_enabled", true));
    blackboard->set("pose_gate_max_x", node->declare_parameter<double>("pose_gate_max_x", 0.10));
    blackboard->set("pose_gate_max_y", node->declare_parameter<double>("pose_gate_max_y", 0.20));
    blackboard->set("pose_gate_max_yaw",
      node->declare_parameter<double>("pose_gate_max_yaw", 0.20));
    blackboard->set(
      "pose_gate_retries",
      static_cast<int>(node->declare_parameter<int64_t>("pose_gate_retries", 1)));
    // false ate' validar o gate no robo (2026-10-26): estacao torta vira aviso +
    // re-dock, nao falha. Depois de conferir os logs "pose validada/FORA DOS
    // LIMITES" em campo, mudar para true (ver docs/investigacao_incidente_2026-10.md).
    blackboard->set("pose_gate_strict", node->declare_parameter<bool>("pose_gate_strict", false));
    // Parada antes da manipulacao: odometria (/odom do EKF) abaixo de
    // stop_check_max_lin / max_ang por stop_check_window_s continuos (nao e'
    // atraso fixo: sem odometria fresca o estado e' 'sem_odometria'). Limites
    // bem abaixo do piso do ESC (0,1215 m/s / 0,315 rad/s); janela >= retencao
    // da referencia pelo ESC (~0,5 s).
    blackboard->set("stop_check_enabled",
      node->declare_parameter<bool>("stop_check_enabled", true));
    blackboard->set("stop_check_max_lin",
      node->declare_parameter<double>("stop_check_max_lin", 0.02));
    blackboard->set("stop_check_max_ang",
      node->declare_parameter<double>("stop_check_max_ang", 0.05));
    blackboard->set("stop_check_window_s",
      node->declare_parameter<double>("stop_check_window_s", 0.5));
    blackboard->set(
      "stop_check_timeout_s", node->declare_parameter<double>("stop_check_timeout_s", 3.0));
    blackboard->set(
      "stop_check_odom_max_age_s",
      node->declare_parameter<double>("stop_check_odom_max_age_s", 0.5));
    blackboard->set(
      "stop_check_require_odom", node->declare_parameter<bool>("stop_check_require_odom", true));
    blackboard->set("odom_topic", node->declare_parameter<std::string>("odom_topic", "/odom"));
    // AMT1: prazo total do goal /amt1_sort e silencio maximo de feedback
    // (o server manda heartbeat <= 5 s; um pick/place filho leva ate ~5 min
    // com retries). amt1_timeout = 1800 s porque 6 picks + 6 places levam
    // ~12-18 min. Estourar o prazo total NAO cancela o goal: a folha
    // Amt1SortBT so avisa (WARN, uma vez) e continua; o cancel de fato e
    // SEMPRE e SO do watchdog de feedback (amt1_stage_timeout) — cancelar
    // com cubo a bordo/braco no ar e pior que esperar o server terminar.
    blackboard->set("amt1_timeout", node->declare_parameter<double>("amt1_timeout", 1800.0));
    blackboard->set(
      "amt1_stage_timeout", node->declare_parameter<double>("amt1_stage_timeout", 300.0));
    blackboard->set("docked", false);
    blackboard->set("current_dock_id", std::string(""));
    blackboard->set("current_dock_type", std::string(""));

    MissionBuildContext ctx;
    ctx.simulate_navigation = simulate_navigation;
    ctx.finish_dock_id = finish_dock_id;
    ctx.startup_home = startup_home;
    ctx.home_before_dock = home_before_dock;
    ctx.startup_pose_name = startup_pose_name;
    ctx.amt1_look_poses = node->declare_parameter<std::string>(
      "amt1_look_poses", "tag_esquerda,tag_direita,pegar_obj");
    ctx.amt1_rounds = node->declare_parameter<int>("amt1_rounds", 4);
    // false = comportamento antigo (goto falhou => missao inteira falha).
    ctx.skip_unreachable_station =
      node->declare_parameter<bool>("skip_unreachable_station", true);
    // Fila de alcance. reach_queue_enabled:=false => XML
    // identico ao de antes (folhas sem final_attempt => true).
    ctx.reach_queue_enabled = node->declare_parameter<bool>("reach_queue_enabled", true);
    ctx.reach_queue_ws_prefixes = node->declare_parameter<std::vector<std::string>>(
      "reach_queue_ws_prefixes", std::vector<std::string>{"WS", "PP", "SH"});
    ctx.reach_queue_max_nudges = static_cast<int>(
      node->declare_parameter<int64_t>("reach_queue_max_nudges", 2));
    ctx.reach_queue_max_shift_m =
      node->declare_parameter<double>("reach_queue_max_shift_m", 0.25);
    ctx.reach_queue_min_shift_m =
      node->declare_parameter<double>("reach_queue_min_shift_m", 0.10);
    if (ctx.reach_queue_max_nudges < 0 || ctx.reach_queue_max_shift_m <= 0.0 ||
      ctx.reach_queue_min_shift_m < 0.0 ||
      ctx.reach_queue_min_shift_m > ctx.reach_queue_max_shift_m)
    {
      throw std::runtime_error(
              "reach_queue_*: esperado max_nudges >= 0 e 0 <= min_shift_m <= max_shift_m "
              "(max_shift_m > 0).");
    }

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
    // 2026-11-05: re-tentativa do que nao foi concluido, uma passada, ANTES
    // do FINISH (buildRetryActions). false = arvore unica de sempre.
    const bool retry_unfinished =
      node->declare_parameter<bool>("retry_unfinished_at_end", true);
    // 2026-11-16: teto de rodadas de replanejamento (arvore caiu ou deixou
    // coisa por fazer). So' vale com retry_unfinished_at_end=true.
    const int replan_max_rounds =
      static_cast<int>(node->declare_parameter<int64_t>("replan_max_rounds", 6));
    ctx.include_epilogue = !retry_unfinished;
    const std::string tree_xml = buildTreeXmlFromActions(actions_root, blackboard, ctx);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<manip_bt::GoToNamedPoseBT>("GoToNamedPose");
    factory.registerNodeType<manip_bt::PickTagBT>("PickTag");
    factory.registerNodeType<manip_bt::PlaceTagBT>("PlaceTag");
    factory.registerNodeType<manip_bt::ReachQueueBT>("ReachQueue");
    // Sempre real (tambem em simulate_navigation): a manipulacao nao e simulada.
    factory.registerNodeType<manip_bt::Amt1SortBT>("Amt1Sort");
    if (simulate_navigation) {
      factory.registerNodeType<SimulatedGoToWSBT>("GoToWS");
      factory.registerNodeType<SimulatedNudgeBaseBT>("NudgeBase");
    } else {
      factory.registerNodeType<manip_bt::GoToWSBT>("GoToWS");
      factory.registerNodeType<manip_bt::NudgeBaseBT>("NudgeBase");
    }

    // --dry-run imprime ANTES de instanciar (2026-10-27): montar a arvore
    // conecta cada GoToNamedPose ao /move_group, e sem o stack no ar o plano
    // nem aparecia. A montagem continua depois (pega erro de porta).
    if (dry_run) {
      std::cout << "=== Plano da missao (" << yaml_path << ") ===\n";
      for (const auto & row : ctx.plan_rows) {
        std::cout << "  " << row << "\n";
      }
      if (retry_unfinished) {
        std::cout << "  [fim] replanejamento do que falhar ou faltar (ate " << replan_max_rounds
                  << " rodadas), depois home + "
                  << "goto " << ctx.finish_dock_id << "\n";
      }
      std::cout << "=== XML gerado ===\n" << tree_xml << std::flush;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("bt_yaml_executor"),
      "Instanciando os nos da arvore (conectando ao /move_group — o MoveIt "
      "pode levar um tempo para terminar de subir; acompanhe os logs)...");
    const auto build_start = std::chrono::steady_clock::now();
    // Escrita pelas folhas PickTag/PlaceTag ao comecar (tipo travado aqui).
    blackboard->set("ultima_manip_tag", std::string());
    auto tree = factory.createTreeFromText(tree_xml, blackboard);
    RCLCPP_INFO(
      rclcpp::get_logger("bt_yaml_executor"),
      "Arvore construida em %.1f s.",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - build_start).count());

    if (dry_run) {
      std::cout << "=== Arvore montada (dry-run ok) ===\n";
      rclcpp::shutdown();
      return 0;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("bt_yaml_executor"),
      "Executando a arvore do actions.yaml: %s (navegacao %s)",
      yaml_path.c_str(),
      simulate_navigation ? "SIMULADA" : "REAL");

    // Tica uma arvore ate o fim. SUCCESS/FAILURE, ou IDLE se interrompida
    // (Ctrl-C: halt limpo, cancelando os goals em voo).
    const auto run_tree = [&](BT::Tree & t, const std::string & label) {
        rclcpp::Rate loop_rate(10.0);
        while (rclcpp::ok() && !g_interrupted) {
          const BT::NodeStatus st = t.tickRoot();
          if (st == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("bt_yaml_executor"), "%s terminou com SUCESSO",
              label.c_str());
            return st;
          }
          if (st == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(rclcpp::get_logger("bt_yaml_executor"), "%s terminou com FALHA",
              label.c_str());
            return st;
          }
          loop_rate.sleep();
        }
        // Nota: se o no atual for o GoToNamedPose (SyncActionNode com MoveIt
        // bloqueante), o halt so acontece quando o tick dele retornar — os nos
        // assincronos (GoToWS/PickTag/PlaceTag) cancelam imediatamente.
        RCLCPP_WARN(
          rclcpp::get_logger("bt_yaml_executor"),
          "Interrompido (Ctrl-C): haltando a arvore e cancelando goals em voo...");
        t.haltTree();
        // Da um respiro para os cancels chegarem aos servers antes do shutdown.
        rclcpp::sleep_for(std::chrono::milliseconds(800));
        return BT::NodeStatus::IDLE;
      };

    BT::NodeStatus status = run_tree(tree, "Arvore");
    if (retry_unfinished && status != BT::NodeStatus::IDLE) {
      // 2026-11-16 (operador: "ele nao pode falhar a missao, faca ele ajustar
      // o task planner em uma situacao dessa e tentar o place mais tarde"):
      // a arvore caiu ou deixou coisa por fazer => replaneja o que falta pelo
      // mesmo planejador, a partir do que ja foi feito e do que esta a bordo,
      // e segue. O objeto em que a arvore caiu fica FORA da rodada seguinte
      // (a bordo, ocupando um pote) e volta na outra; caiu 2 vezes no mesmo
      // objeto => desiste dele e segue com o resto. Para quando: nada
      // pendente; rodada inteira sem concluir nada novo; 2 quedas seguidas
      // sem objeto e sem progresso; replan_max_rounds.
      tree = BT::Tree();  // solta os nos da principal (mesmos nomes de no ROS)
      const auto log = rclcpp::get_logger("bt_yaml_executor");
      std::set<std::size_t> done;
      std::size_t novos = harvestCompleted(actions_root, blackboard, done);
      std::map<std::string, int> quedas;      // por objeto
      std::set<std::string> abandonados;
      int quedas_sem_objeto = 0;
      bool rodada_com_adiado = false;
      for (int round = 1; round <= replan_max_rounds && rclcpp::ok() && !g_interrupted; ++round) {
        std::set<std::string> hold = abandonados;
        std::string adiado;
        std::string start_ws;
        if (status == BT::NodeStatus::FAILURE) {
          std::string tag;
          try {
            blackboard->get("ultima_manip_tag", tag);
          } catch (const std::exception &) {
            tag.clear();
          }
          const std::optional<std::string> ws =
            tag.empty() ? std::nullopt : pendingWsOfTag(actions_root, done, tag);
          if (ws) {
            start_ws = *ws;
            quedas_sem_objeto = 0;
            if (++quedas[tag] >= 2) {
              abandonados.insert(tag);
              RCLCPP_ERROR(
                log, "[replano] %s falhou de novo em %s: desisto desse objeto e sigo com o resto.",
                tag.c_str(), ws->c_str());
            } else {
              adiado = tag;
              RCLCPP_WARN(
                log, "[replano] %s falhou em %s: fica para depois; replanejando o resto.",
                tag.c_str(), ws->c_str());
            }
            hold.insert(tag);
          } else if (novos == 0 && ++quedas_sem_objeto >= 2) {
            RCLCPP_ERROR(
              log, "[replano] a arvore caiu 2 vezes seguidas sem concluir nada: paro de replanejar.");
            break;
          }
        } else if (round > 1 && novos == 0 && !rodada_com_adiado) {
          break;  // rodada inteira sem nada novo: o que sobrou nao sai
        }

        std::vector<std::string> report;
        std::optional<YAML::Node> plan =
          buildRetryActions(actions_root, done, hold, start_ws, report);
        if (!plan && !adiado.empty()) {
          // So' sobrou o adiado: tenta agora mesmo (o goto refaz o docking).
          hold.erase(adiado);
          adiado.clear();
          report.clear();
          plan = buildRetryActions(actions_root, done, hold, start_ws, report);
        }
        if (!plan) {
          break;  // o resumo final (abaixo) diz o que ficou
        }
        for (const auto & row : report) {
          RCLCPP_INFO(log, "%s", row.c_str());
        }
        MissionBuildContext rctx = ctx;
        rctx.include_prologue = false;
        rctx.include_epilogue = false;
        rctx.plan_rows.clear();
        const std::string round_xml = buildTreeXmlFromActions(*plan, blackboard, rctx);
        for (const auto & row : rctx.plan_rows) {
          RCLCPP_INFO(log, "  [re-tentativa] %s", row.c_str());
        }
        blackboard->set("ultima_manip_tag", std::string());
        auto round_tree = factory.createTreeFromText(round_xml, blackboard);
        status = run_tree(
          round_tree, round == 1 ? "Re-tentativa" : "Re-tentativa " + std::to_string(round));
        if (status == BT::NodeStatus::IDLE) {
          break;
        }
        novos = harvestCompleted(*plan, blackboard, done);
        rodada_com_adiado = !adiado.empty();
      }
      if (status != BT::NodeStatus::IDLE) {
        // O que ficou para tras, no log (mesmo formato de antes).
        std::vector<std::string> left;
        (void)buildRetryActions(actions_root, done, {}, "", left);
        for (const auto & row : left) {
          if (row.find("plano:") == std::string::npos) {
            RCLCPP_INFO(log, "apos a re-tentativa: %s", row.c_str());
          }
        }
      }
    }
    // Epilogo SEMPRE que nao foi Ctrl-C (2026-11-16): antes uma re-tentativa
    // que falhava cancelava o home + FINISH.
    if (status != BT::NodeStatus::IDLE && retry_unfinished && !ctx.finish_dock_id.empty()) {
      MissionBuildContext ectx = ctx;
      ectx.include_prologue = false;
      ectx.include_epilogue = true;
      ectx.plan_rows.clear();
      YAML::Node empty;
      empty["actions"] = YAML::Node(YAML::NodeType::Sequence);
      auto end_tree = factory.createTreeFromText(
        buildTreeXmlFromActions(empty, blackboard, ectx), blackboard);
      status = run_tree(end_tree, "Epilogo (home + FINISH)");
    }
    if (status == BT::NodeStatus::FAILURE) {
      exit_code = 1;
    } else if (status == BT::NodeStatus::IDLE) {
      exit_code = 130;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("bt_yaml_executor"), "Execucao falhou: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
