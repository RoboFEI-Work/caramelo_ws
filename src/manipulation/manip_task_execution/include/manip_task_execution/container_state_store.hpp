#pragma once

#include <string>
#include <vector>

namespace manip_task_execution
{

class ContainerStateStore
{
public:
  explicit ContainerStateStore(const std::string & file_path);

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
