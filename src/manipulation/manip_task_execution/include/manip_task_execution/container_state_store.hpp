#pragma once

#include <string>
#include <vector>

namespace manip_task_execution
{

/// Slot de mesa ja usado pelo proprio robo numa estacao (memoria do
/// multi-place, 2026-08-21). Posicao do TCP no frame da base do braco.
struct TableSlotRecord
{
  std::string slot;        ///< pose nomeada usada ("Mesa15.1") ou "free_xy"
  std::string table_pose;  ///< nome-base da mesa ("Mesa15")
  std::string tag_frame;   ///< tag do bloco entregue
  std::string frame;       ///< frame de x/y/z (manip_base_link)
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double stamp_sec{0.0};   ///< epoch (segundos) da entrega
};

class ContainerStateStore
{
public:
  explicit ContainerStateStore(const std::string & file_path);

  /// Memoria de slots usados por estacao ('table_slots:' no mesmo yaml).
  /// Sobrevive ao respawn do no; o reset por missao do launch reescreve o
  /// arquivo so com 'containers:' e a apaga. `resetAllEmpty` NAO a apaga —
  /// por isso a leitura aceita um TTL.
  bool addTableSlotUsed(
    const std::string & ws,
    const TableSlotRecord & record,
    std::string * error_msg = nullptr);

  bool getTableSlotsUsed(
    const std::string & ws,
    double ttl_sec,
    double now_sec,
    std::vector<TableSlotRecord> * records,
    std::string * error_msg = nullptr) const;

  bool setOccupied(
    const std::string & container_name,
    const std::string & tag_frame,
    std::string * error_msg = nullptr);

  bool setEmpty(const std::string & container_name, std::string * error_msg = nullptr);

  bool resetAllEmpty(
    const std::vector<std::string> & default_container_names = {},
    std::string * error_msg = nullptr);

  bool findContainerByTag(
    const std::string & tag_frame,
    std::string * container_name,
    std::string * error_msg = nullptr) const;

  bool findFirstEmptyContainer(
    std::string * container_name,
    std::string * error_msg = nullptr) const;

private:
  bool updateContainer(
    const std::string & container_name,
    bool occupied,
    const std::string & tag_frame,
    std::string * error_msg);

  std::string file_path_;
};

}  // namespace manip_task_execution
