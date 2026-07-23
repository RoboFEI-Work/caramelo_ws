#include "manip_task_execution/container_state_store.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <system_error>
#include <vector>

namespace manip_task_execution
{

namespace
{

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
  }

  std::error_code ec;
  std::filesystem::rename(tmp_path, output_path, ec);
  if (ec) {
    std::filesystem::remove(output_path, ec);
    ec.clear();
    std::filesystem::rename(tmp_path, output_path, ec);
    if (ec) {
      if (error_msg) {
        *error_msg = "failed to replace yaml file: " + ec.message();
      }
      return false;
    }
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

}  // namespace manip_task_execution
