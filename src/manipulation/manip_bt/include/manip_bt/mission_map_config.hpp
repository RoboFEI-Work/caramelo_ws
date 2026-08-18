#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace manip_bt
{

// Configuracao da arena carregada 1x pelo executor de missao a partir da pasta
// do mapa (docking.yaml + service_areas.yaml + ws_table_mapping.yaml). E' a
// unica fonte de verdade da cadeia WS_da_tarefa -> dock_id -> pose/type.
class MissionMapConfig
{
public:
  struct DockInfo
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    std::string type;
    bool pose_is_placeholder{true};  // pose [0,0,0] = ainda nao gravada
    // Offsets de staging do dock_plugins do docking.yaml (2026-08-18) — o
    // fluxo novo navega ate a staging por NavigateToPose e o align faz o
    // approach; fallback -0.45 quando o plugin nao esta no YAML.
    double staging_x_offset{-0.45};
    double staging_yaw_offset{0.0};
    bool staging_from_yaml{false};
  };

  // Lanca std::runtime_error se map_folder nao existir ou docking.yaml faltar.
  explicit MissionMapConfig(const std::string & map_folder);

  const std::string & mapFolder() const {return map_folder_;}
  const std::string & mapName() const {return map_name_;}
  const std::string & mapDir() const {return map_dir_;}

  bool hasDock(const std::string & dock_id) const;
  std::optional<DockInfo> dock(const std::string & dock_id) const;
  std::string dockType(const std::string & dock_id) const;

  // Pose de STAGING do dock (2026-08-18): a pose do banco deslocada
  // staging_x_offset (negativo = para tras) ao longo do yaw — identica a
  // getStagingPose do SimpleNonChargingDock, entao rollback e fluxo novo
  // compartilham o mesmo ponto. nullopt se o dock nao existe.
  std::optional<DockInfo> stagingPose(const std::string & dock_id) const;

  // service_areas.yaml: docking.use_docking. Default quando ausente:
  // false p/ START/FINISH, true p/ o resto (mesma semantica do mission_runner).
  bool useDocking(const std::string & dock_id) const;
  // service_areas.yaml: manipulation_enabled (default true quando ausente —
  // usado so' para warning, nao bloqueia).
  bool manipulationEnabled(const std::string & dock_id) const;

  // WS da tarefa (WS_1) -> dock_id (WS1): ws_to_dock_id do ws_table_mapping.yaml
  // da pasta do mapa; sem entrada, deriva por normalizacao (WS_1 -> WS1).
  std::string dockIdForWs(const std::string & task_ws) const;

  // Espelho C++ do normalize_dock_id de caramelo_docking_common.py, com o
  // extra de remover '_' e espacos (WS_1 -> WS1). Lanca std::runtime_error
  // se o resultado nao casar com WS\d+|SH\d+|PP\d+|RT\d+|START|FINISH.
  static std::string normalizeDockId(const std::string & dock_id);
  // Espelho do default_dock_type (SH->shelf, PP/RT->precision, resto front).
  static std::string defaultDockType(const std::string & dock_id);

  // Fallback p/ uso standalone (sem run_mission): tenta, nesta ordem,
  // $CARAMELO_MAPS_DIR/<map_name>, share do caramelo_mapping/maps/<map_name>
  // e o truque install->src. Vazio se nao achar.
  static std::string resolveMapFolder(const std::string & map_name);

private:
  std::string map_folder_;
  std::string map_name_;
  std::string map_dir_;
  std::unordered_map<std::string, DockInfo> docks_;
  std::unordered_map<std::string, bool> use_docking_;
  std::unordered_map<std::string, bool> manipulation_enabled_;
  std::unordered_map<std::string, std::string> ws_to_dock_id_;
};

using MissionMapConfigPtr = std::shared_ptr<MissionMapConfig>;

}  // namespace manip_bt
