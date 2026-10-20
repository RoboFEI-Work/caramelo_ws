// Estado persistente dos containers de bordo e da memoria de slots de mesa.
// Yaml em ~/.config/caramelo/container_states.yaml (parametro
// container_state_file dos nos): 'containers:' (container1..3 ocupado/tag)
// e 'table_slots:' por WS. Escrita atomica (tmp + fsync + rename). Usado
// pelo pick, place e amt1; o manip_stack.launch.py reseta uma vez por
// rodada. Sem ROS: testado por test_container_state_store.
#pragma once

#include <cstddef>
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
  /// Sobrevive ao respawn do no. O reset por rodada (one-shot do launch) e
  /// `resetAllEmpty` apagam a chave 'table_slots'; o TTL da leitura so'
  /// protege contra registro orfao (no' morto sem reset).
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

  /// Apaga TODOS os registros de 'table_slots' (em qualquer WS) cujo
  /// tag_frame seja o dado; o resto do yaml ('containers:' e as outras
  /// tags) fica como esta e a escrita continua atomica (2026-10-19).
  /// Chamado quando o bloco muda de morada: o pick o tirou da mesa ou o
  /// place o soltou em outro lugar — sem isso o registro antigo virava
  /// obstaculo FANTASMA (e, somado a deteccao da camera, tag duplicada).
  /// Arquivo inexistente/vazio ou tag sem registro = sucesso com 0
  /// removidos (nada e reescrito). `removed` (opcional) recebe a contagem.
  bool removeTableSlotsForTag(
    const std::string & tag_frame,
    std::size_t * removed = nullptr,
    std::string * error_msg = nullptr);

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
