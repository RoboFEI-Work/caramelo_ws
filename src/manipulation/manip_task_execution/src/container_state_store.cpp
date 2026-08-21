#include "manip_task_execution/container_state_store.hpp"

#include <yaml-cpp/yaml.h>

#include <fcntl.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <system_error>
#include <vector>

namespace manip_task_execution
{

namespace
{

bool fsyncPath(const std::string & path, bool is_directory)
{
  const int fd = ::open(path.c_str(), is_directory ? (O_RDONLY | O_DIRECTORY) : O_RDONLY);
  if (fd < 0) {
    return false;
  }
  const bool ok = ::fsync(fd) == 0;
  ::close(fd);
  return ok;
}

bool writeYamlAtomically(
  const YAML::Node & root,
  const std::string & file_path,
  std::string * error_msg)
{
  YAML::Emitter out;
  out << root;

  const std::filesystem::path output_path(file_path);
  if (!output_path.parent_path().empty()) {
    std::filesystem::create_directories(output_path.parent_path());
  }

  const std::filesystem::path tmp_path = output_path.string() + ".tmp";
  {
    std::ofstream ofs(tmp_path, std::ios::trunc);
    if (!ofs.is_open()) {
      if (error_msg) {
        *error_msg = "failed to open temporary file for writing: " + tmp_path.string();
      }
      return false;
    }
    ofs << out.c_str() << '\n';
    ofs.flush();
    if (!ofs) {
      if (error_msg) {
        *error_msg = "failed to write temporary yaml: " + tmp_path.string();
      }
      return false;
    }
  }

  // Item 3.2: fsync ANTES do rename. Sem isso o rename podia ganhar do
  // flush do conteudo e uma queda de energia (robo desligado no tapa)
  // deixava um yaml de tamanho certo cheio de zeros — pior que o arquivo
  // velho, porque parece valido.
  if (!fsyncPath(tmp_path.string(), false)) {
    if (error_msg) {
      *error_msg = "failed to fsync temporary yaml: " + tmp_path.string();
    }
    return false;
  }

  // rename(2) e atomico e SOBRESCREVE o destino: o remove+rename anterior
  // abria uma janela em que o arquivo simplesmente nao existia — se o
  // processo morresse ali, o estado dos containers sumia.
  std::error_code ec;
  std::filesystem::rename(tmp_path, output_path, ec);
  if (ec) {
    if (error_msg) {
      *error_msg = "failed to replace yaml file: " + ec.message();
    }
    return false;
  }

  // Durabilidade do proprio rename: sem fsync no diretorio, a entrada nova
  // pode nao ter chegado ao disco.
  const auto parent = output_path.parent_path();
  if (!parent.empty()) {
    (void)fsyncPath(parent.string(), true);
  }

  return true;
}

}  // namespace

ContainerStateStore::ContainerStateStore(const std::string & file_path)
: file_path_(file_path)
{
}

bool ContainerStateStore::setOccupied(
  const std::string & container_name,
  const std::string & tag_frame,
  std::string * error_msg)
{
  return updateContainer(container_name, true, tag_frame, error_msg);
}

bool ContainerStateStore::setEmpty(const std::string & container_name, std::string * error_msg)
{
  return updateContainer(container_name, false, "", error_msg);
}

bool ContainerStateStore::resetAllEmpty(
  const std::vector<std::string> & default_container_names,
  std::string * error_msg)
{
  try {
    YAML::Node root;

    if (std::filesystem::exists(file_path_)) {
      root = YAML::LoadFile(file_path_);
    }

    if (!root || !root.IsMap()) {
      root = YAML::Node(YAML::NodeType::Map);
    }

    YAML::Node containers = root["containers"];
    if (!containers || !containers.IsMap()) {
      root["containers"] = YAML::Node(YAML::NodeType::Map);
      containers = root["containers"];
    }

    if (containers.size() == 0) {
      for (const auto & container_name : default_container_names) {
        containers[container_name] = YAML::Node(YAML::NodeType::Map);
      }
    }

    std::vector<std::string> container_names;
    for (const auto & it : containers) {
      container_names.push_back(it.first.as<std::string>());
    }

    for (const auto & name : container_names) {
      YAML::Node container = containers[name];
      container["occupied"] = false;
      container["tag_frame"] = "";
      container["status"] = "empty";
    }
    // Reset por rodada apaga tambem a memoria de slots de mesa (2026-08-21):
    // sem isso, uma bancada anterior viraria obstaculo virtual por ate um
    // TTL. Os dois caminhos de reset (este e o one-shot do launch, que
    // reescreve o arquivo) ficam equivalentes.
    if (root["table_slots"]) {
      root.remove("table_slots");
    }

    return writeYamlAtomically(root, file_path_, error_msg);
  } catch (const std::exception & ex) {
    if (error_msg) {
      *error_msg = ex.what();
    }
    return false;
  }
}

bool ContainerStateStore::findContainerByTag(
  const std::string & tag_frame,
  std::string * container_name,
  std::string * error_msg) const
{
  if (tag_frame.empty()) {
    if (error_msg) {
      *error_msg = "tag_frame is empty";
    }
    return false;
  }

  if (container_name == nullptr) {
    if (error_msg) {
      *error_msg = "container_name output pointer is null";
    }
    return false;
  }

  try {
    if (!std::filesystem::exists(file_path_)) {
      if (error_msg) {
        *error_msg = "container state file not found: " + file_path_;
      }
      return false;
    }

    YAML::Node root = YAML::LoadFile(file_path_);
    YAML::Node containers = root["containers"];
    if (!containers || !containers.IsMap()) {
      if (error_msg) {
        *error_msg = "missing 'containers' map in yaml";
      }
      return false;
    }

    for (const auto & it : containers) {
      const std::string name = it.first.as<std::string>();
      const YAML::Node container = it.second;
      if (!container || !container.IsMap()) {
        continue;
      }

      const bool occupied = container["occupied"].as<bool>(false);
      const std::string stored_tag = container["tag_frame"].as<std::string>("");

      if (occupied && stored_tag == tag_frame) {
        *container_name = name;
        return true;
      }
    }

    if (error_msg) {
      *error_msg = "tag not found in occupied containers: " + tag_frame;
    }
    return false;
  } catch (const std::exception & ex) {
    if (error_msg) {
      *error_msg = ex.what();
    }
    return false;
  }
}

bool ContainerStateStore::findFirstEmptyContainer(
  std::string * container_name,
  std::string * error_msg) const
{
  if (container_name == nullptr) {
    if (error_msg) {
      *error_msg = "container_name output pointer is null";
    }
    return false;
  }

  try {
    if (!std::filesystem::exists(file_path_)) {
      if (error_msg) {
        *error_msg = "container state file not found: " + file_path_;
      }
      return false;
    }

    YAML::Node root = YAML::LoadFile(file_path_);
    YAML::Node containers = root["containers"];
    if (!containers || !containers.IsMap()) {
      if (error_msg) {
        *error_msg = "missing 'containers' map in yaml";
      }
      return false;
    }

    for (const auto & it : containers) {
      const std::string name = it.first.as<std::string>();
      const YAML::Node container = it.second;
      if (!container || !container.IsMap()) {
        continue;
      }

      const bool occupied = container["occupied"].as<bool>(false);
      if (!occupied) {
        *container_name = name;
        return true;
      }
    }

    if (error_msg) {
      *error_msg = "no empty containers available";
    }
    return false;
  } catch (const std::exception & ex) {
    if (error_msg) {
      *error_msg = ex.what();
    }
    return false;
  }
}

bool ContainerStateStore::updateContainer(
  const std::string & container_name,
  bool occupied,
  const std::string & tag_frame,
  std::string * error_msg)
{
  if (container_name.empty()) {
    if (error_msg) {
      *error_msg = "container_name is empty";
    }
    return false;
  }

  try {
    YAML::Node root;

    if (std::filesystem::exists(file_path_)) {
      root = YAML::LoadFile(file_path_);
    }

    if (!root || !root.IsMap()) {
      root = YAML::Node(YAML::NodeType::Map);
    }

    YAML::Node containers = root["containers"];
    if (!containers || !containers.IsMap()) {
      root["containers"] = YAML::Node(YAML::NodeType::Map);
      containers = root["containers"];
    }

    YAML::Node container = containers[container_name];
    container["occupied"] = occupied;
    container["tag_frame"] = occupied ? tag_frame : "";
    container["status"] = occupied ? "filled" : "empty";

    return writeYamlAtomically(root, file_path_, error_msg);
  } catch (const std::exception & ex) {
    if (error_msg) {
      *error_msg = ex.what();
    }
    return false;
  }
}

bool ContainerStateStore::addTableSlotUsed(
  const std::string & ws,
  const TableSlotRecord & record,
  std::string * error_msg)
{
  if (ws.empty()) {
    if (error_msg) {
      *error_msg = "ws is empty";
    }
    return false;
  }

  try {
    YAML::Node root;
    if (std::filesystem::exists(file_path_)) {
      root = YAML::LoadFile(file_path_);
    }
    if (!root || !root.IsMap()) {
      root = YAML::Node(YAML::NodeType::Map);
    }

    YAML::Node slots = root["table_slots"];
    if (!slots || !slots.IsMap()) {
      root["table_slots"] = YAML::Node(YAML::NodeType::Map);
      slots = root["table_slots"];
    }
    YAML::Node list = slots[ws];
    if (!list || !list.IsSequence()) {
      slots[ws] = YAML::Node(YAML::NodeType::Sequence);
      list = slots[ws];
    }

    YAML::Node entry(YAML::NodeType::Map);
    entry.SetStyle(YAML::EmitterStyle::Flow);
    entry["slot"] = record.slot;
    entry["table_pose"] = record.table_pose;
    entry["tag_frame"] = record.tag_frame;
    entry["frame"] = record.frame;
    entry["x"] = record.x;
    entry["y"] = record.y;
    entry["z"] = record.z;
    entry["stamp"] = record.stamp_sec;
    list.push_back(entry);

    return writeYamlAtomically(root, file_path_, error_msg);
  } catch (const std::exception & ex) {
    if (error_msg) {
      *error_msg = ex.what();
    }
    return false;
  }
}

bool ContainerStateStore::getTableSlotsUsed(
  const std::string & ws,
  double ttl_sec,
  double now_sec,
  std::vector<TableSlotRecord> * records,
  std::string * error_msg) const
{
  if (!records) {
    if (error_msg) {
      *error_msg = "records is null";
    }
    return false;
  }
  records->clear();
  if (ws.empty() || !std::filesystem::exists(file_path_)) {
    return true;
  }

  try {
    const YAML::Node root = YAML::LoadFile(file_path_);
    if (!root || !root.IsMap()) {
      return true;
    }
    const YAML::Node slots = root["table_slots"];
    if (!slots || !slots.IsMap()) {
      return true;
    }
    const YAML::Node list = slots[ws];
    if (!list || !list.IsSequence()) {
      return true;
    }
    for (const auto & entry : list) {
      if (!entry.IsMap()) {
        continue;
      }
      TableSlotRecord rec;
      rec.slot = entry["slot"].as<std::string>("");
      rec.table_pose = entry["table_pose"].as<std::string>("");
      rec.tag_frame = entry["tag_frame"].as<std::string>("");
      rec.frame = entry["frame"].as<std::string>("");
      rec.x = entry["x"].as<double>(0.0);
      rec.y = entry["y"].as<double>(0.0);
      rec.z = entry["z"].as<double>(0.0);
      rec.stamp_sec = entry["stamp"].as<double>(0.0);
      if (ttl_sec > 0.0 && (now_sec - rec.stamp_sec) > ttl_sec) {
        continue;  // registro velho (bancada de outro dia)
      }
      records->push_back(rec);
    }
    return true;
  } catch (const std::exception & ex) {
    if (error_msg) {
      *error_msg = ex.what();
    }
    return false;
  }
}

}  // namespace manip_task_execution
