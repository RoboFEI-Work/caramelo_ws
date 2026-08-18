#include "manip_bt/mission_map_config.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <regex>
#include <stdexcept>
#include <vector>

namespace fs = std::filesystem;

namespace manip_bt
{

namespace
{

const std::regex kDockIdPattern("^(START|FINISH|WS[0-9]+|SH[0-9]+|PP[0-9]+|RT[0-9]+)$");

// LoadFile com o NOME do arquivo no erro (o ParserException do yaml-cpp so
// diz linha/coluna e o operador nao saberia qual dos 3 YAMLs esta quebrado).
YAML::Node loadYamlOrThrow(const std::filesystem::path & path)
{
  try {
    return YAML::LoadFile(path.string());
  } catch (const std::exception & ex) {
    throw std::runtime_error("Erro lendo " + path.string() + ": " + ex.what());
  }
}

}  // namespace

MissionMapConfig::MissionMapConfig(const std::string & map_folder)
{
  const fs::path folder(map_folder);
  if (!fs::exists(folder)) {
    throw std::runtime_error("Pasta do mapa nao existe: " + map_folder);
  }
  const fs::path docking_path = folder / "docking.yaml";
  if (!fs::exists(docking_path)) {
    throw std::runtime_error("docking.yaml nao encontrado em: " + map_folder);
  }

  map_folder_ = fs::absolute(folder).string();
  map_name_ = folder.filename().string();
  map_dir_ = fs::absolute(folder).parent_path().string();

  const YAML::Node docking = loadYamlOrThrow(docking_path);
  const YAML::Node docks = docking["docks"];
  if (!docks || !docks.IsMap()) {
    throw std::runtime_error("docking.yaml sem a chave 'docks': " + docking_path.string());
  }

  // dock_plugins: staging offsets por type (2026-08-18) — o fluxo novo do
  // GoToWS navega ate a staging por conta propria.
  struct StagingOffsets
  {
    double x{-0.45};
    double yaw{0.0};
  };
  std::unordered_map<std::string, StagingOffsets> staging_by_type;
  const YAML::Node plugins = docking["dock_plugins"];
  if (plugins && plugins.IsMap()) {
    for (const auto & entry : plugins) {
      StagingOffsets off;
      off.x = entry.second["staging_x_offset"].as<double>(-0.45);
      off.yaw = entry.second["staging_yaw_offset"].as<double>(0.0);
      staging_by_type[entry.first.as<std::string>()] = off;
    }
  }

  for (const auto & entry : docks) {
    const std::string dock_id = entry.first.as<std::string>();
    const YAML::Node data = entry.second;
    DockInfo info;
    info.type = data["type"].as<std::string>("");
    const YAML::Node pose = data["pose"];
    if (pose && pose.IsSequence() && pose.size() >= 3) {
      info.x = pose[0].as<double>(0.0);
      info.y = pose[1].as<double>(0.0);
      info.yaw = pose[2].as<double>(0.0);
    }
    info.pose_is_placeholder =
      std::abs(info.x) < 1e-9 && std::abs(info.y) < 1e-9 && std::abs(info.yaw) < 1e-9;
    const auto staging_it = staging_by_type.find(info.type);
    if (staging_it != staging_by_type.end()) {
      info.staging_x_offset = staging_it->second.x;
      info.staging_yaw_offset = staging_it->second.yaw;
      info.staging_from_yaml = true;
    }
    docks_[dock_id] = info;
  }

  const fs::path service_path = folder / "service_areas.yaml";
  if (fs::exists(service_path)) {
    const YAML::Node service = loadYamlOrThrow(service_path);
    const YAML::Node areas = service["service_areas"];
    if (areas && areas.IsMap()) {
      for (const auto & entry : areas) {
        const std::string dock_id = entry.first.as<std::string>();
        const YAML::Node data = entry.second;
        if (data["docking"] && data["docking"]["use_docking"]) {
          use_docking_[dock_id] = data["docking"]["use_docking"].as<bool>(true);
        }
        if (data["manipulation_enabled"]) {
          manipulation_enabled_[dock_id] = data["manipulation_enabled"].as<bool>(true);
        }
      }
    }
  }

  const fs::path mapping_path = folder / "ws_table_mapping.yaml";
  if (fs::exists(mapping_path)) {
    const YAML::Node mapping = loadYamlOrThrow(mapping_path);
    const YAML::Node ws_map = mapping["ws_to_dock_id"];
    if (ws_map && ws_map.IsMap()) {
      for (const auto & entry : ws_map) {
        ws_to_dock_id_[entry.first.as<std::string>()] = entry.second.as<std::string>();
      }
    }
  }
}

bool MissionMapConfig::hasDock(const std::string & dock_id) const
{
  return docks_.count(dock_id) > 0;
}

std::optional<MissionMapConfig::DockInfo> MissionMapConfig::dock(const std::string & dock_id) const
{
  const auto it = docks_.find(dock_id);
  if (it == docks_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<MissionMapConfig::DockInfo> MissionMapConfig::stagingPose(
  const std::string & dock_id) const
{
  const auto it = docks_.find(dock_id);
  if (it == docks_.end()) {
    return std::nullopt;
  }
  DockInfo staging = it->second;
  // Mesma conta do getStagingPose do SimpleNonChargingDock: desloca ao longo
  // do yaw do dock (offset negativo = para tras) e soma o yaw offset.
  staging.x = it->second.x + it->second.staging_x_offset * std::cos(it->second.yaw);
  staging.y = it->second.y + it->second.staging_x_offset * std::sin(it->second.yaw);
  staging.yaw = it->second.yaw + it->second.staging_yaw_offset;
  return staging;
}

std::string MissionMapConfig::dockType(const std::string & dock_id) const
{
  const auto it = docks_.find(dock_id);
  if (it != docks_.end() && !it->second.type.empty()) {
    return it->second.type;
  }
  return defaultDockType(dock_id);
}

bool MissionMapConfig::useDocking(const std::string & dock_id) const
{
  const auto it = use_docking_.find(dock_id);
  if (it != use_docking_.end()) {
    return it->second;
  }
  return dock_id != "START" && dock_id != "FINISH";
}

bool MissionMapConfig::manipulationEnabled(const std::string & dock_id) const
{
  const auto it = manipulation_enabled_.find(dock_id);
  if (it != manipulation_enabled_.end()) {
    return it->second;
  }
  return true;
}

std::string MissionMapConfig::dockIdForWs(const std::string & task_ws) const
{
  const auto it = ws_to_dock_id_.find(task_ws);
  if (it != ws_to_dock_id_.end()) {
    return normalizeDockId(it->second);
  }
  return normalizeDockId(task_ws);
}

std::string MissionMapConfig::normalizeDockId(const std::string & dock_id)
{
  std::string out;
  out.reserve(dock_id.size());
  for (const char c : dock_id) {
    if (c == '_' || std::isspace(static_cast<unsigned char>(c))) {
      continue;
    }
    out += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  if (!std::regex_match(out, kDockIdPattern)) {
    throw std::runtime_error(
            "Estacao invalida '" + dock_id +
            "'. Use a nomenclatura do rulebook: WS1, WS2, SH1, PP1, RT1, START ou FINISH.");
  }
  return out;
}

std::string MissionMapConfig::defaultDockType(const std::string & dock_id)
{
  if (dock_id.rfind("SH", 0) == 0) {
    return "caramelo_shelf_front_dock";
  }
  if (dock_id.rfind("PP", 0) == 0 || dock_id.rfind("RT", 0) == 0) {
    return "caramelo_precision_front_dock";
  }
  return "caramelo_front_dock";
}

std::string MissionMapConfig::resolveMapFolder(const std::string & map_name)
{
  if (map_name.empty()) {
    return "";
  }
  std::vector<fs::path> roots;
  if (const char * env_dir = std::getenv("CARAMELO_MAPS_DIR")) {
    roots.emplace_back(env_dir);
  }
  try {
    const fs::path share(ament_index_cpp::get_package_share_directory("caramelo_mapping"));
    // Truque install->src (mesma ideia do resolve_map_folder Python): o YAML
    // editavel mora no src, o share instalado e' fallback.
    for (fs::path parent = share.parent_path(); !parent.empty();
      parent = parent.parent_path())
    {
      if (parent.filename() == "install") {
        roots.emplace_back(parent.parent_path() / "src" / "caramelo_mapping" / "maps");
        break;
      }
    }
    roots.emplace_back(share / "maps");
  } catch (const std::exception &) {
    // caramelo_mapping fora do ambiente: segue so' com CARAMELO_MAPS_DIR.
  }
  for (const auto & root : roots) {
    const fs::path candidate = root / map_name;
    if (fs::exists(candidate / "map.yaml") || fs::exists(candidate / "docking.yaml")) {
      return fs::absolute(candidate).string();
    }
  }
  return "";
}

}  // namespace manip_bt
