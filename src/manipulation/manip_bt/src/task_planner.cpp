#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct ObjectInfo
{
  int id = -1;
  std::string type;
  std::string color;
};

struct ContainerInfo
{
  int id = -1;
  std::string color;
  std::string ws;
  std::string pose_name;
};

struct ContainerAssignment
{
  std::string container_pose;
  std::string container_ws;
};

struct TransferItem
{
  int obj_id = -1;
  std::string tag_frame;
  std::string from_ws;
  std::string to_ws;
  std::string destination_container_pose;
  bool needs_pick = false;
  bool needs_place = false;
};

struct PlannerState
{
  std::set<int> picked;
  std::set<int> placed;
  std::vector<int> inventory;
  std::string current_ws;
  std::mt19937 rng{std::random_device{}()};
};

std::string trim(const std::string & value)
{
  size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
    ++start;
  }

  size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }

  return value.substr(start, end - start);
}

std::string toUpper(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return value;
}

int parseObjectId(const YAML::Node & node)
{
  if (!node) {
    throw std::runtime_error("obj_id node is missing");
  }

  const std::string text = trim(node.as<std::string>());
  if (text.empty()) {
    throw std::runtime_error("obj_id is empty");
  }

  size_t idx = 0;
  const int value = std::stoi(text, &idx, 10);
  if (idx != text.size()) {
    throw std::runtime_error("obj_id contains invalid characters: " + text);
  }
  return value;
}

std::set<std::string> parseActiveAreas(const YAML::Node & root)
{
  std::set<std::string> active_areas;
  const YAML::Node areas = root["active_service_areas"];
  if (!areas || !areas.IsSequence()) {
    throw std::runtime_error("active_service_areas must be a sequence");
  }

  for (const auto & area : areas) {
    if (area.IsMap()) {
      const YAML::Node id_node = area["id"];
      if (!id_node) {
        throw std::runtime_error("active_service_areas map entries must contain 'id'");
      }
      active_areas.insert(trim(id_node.as<std::string>()));
    } else {
      active_areas.insert(trim(area.as<std::string>()));
    }
  }

  return active_areas;
}

std::map<int, ObjectInfo> parseObjects(const YAML::Node & root, std::set<int> & ignored_object_ids)
{
  const YAML::Node objects = root["objects"];
  if (!objects || !objects.IsSequence()) {
    throw std::runtime_error("objects must be a sequence");
  }

  std::map<int, ObjectInfo> by_id;
  for (const auto & obj : objects) {
    const int id = parseObjectId(obj["id"]);

    const std::string type_upper = toUpper(trim(obj["type"].as<std::string>("")));
    if (type_upper == "DECOY") {
      ignored_object_ids.insert(id);
      continue;
    }

    if (by_id.count(id) > 0) {
      throw std::runtime_error("Duplicate object id in objects: " + std::to_string(id));
    }

    ObjectInfo info;
    info.id = id;
    info.type = trim(obj["type"].as<std::string>(""));
    info.color = trim(obj["color"].as<std::string>(""));
    by_id[id] = info;
  }

  return by_id;
}

std::map<int, ContainerInfo> parseContainers(const YAML::Node & root)
{
  std::map<int, ContainerInfo> by_id;
  const YAML::Node containers = root["containers"];
  if (!containers) {
    return by_id;
  }
  if (!containers.IsSequence()) {
    throw std::runtime_error("containers must be a sequence when present");
  }

  for (const auto & container : containers) {
    const int id = parseObjectId(container["id"]);
    if (by_id.count(id) > 0) {
      throw std::runtime_error("Duplicate container id in containers: " + std::to_string(id));
    }

    ContainerInfo info;
    info.id = id;
    info.color = trim(container["color"].as<std::string>(""));
    info.ws = trim(container["at"].as<std::string>(""));
    if (info.ws.empty()) {
      info.ws = trim(container["at_ws"].as<std::string>(""));
    }
    if (info.ws.empty()) {
      throw std::runtime_error("containers id " + std::to_string(id) + " missing at/at_ws");
    }

    info.pose_name = "ct" + std::to_string(id);
    by_id[id] = info;
  }

  return by_id;
}

std::map<int, std::string> buildStateIndex(
  const YAML::Node & state_node,
  const std::set<std::string> & active_areas,
  const std::string & state_name)
{
  if (!state_node || !state_node.IsMap()) {
    throw std::runtime_error(state_name + " must be a map");
  }

  std::map<int, std::string> object_to_ws;
  for (const auto & it : state_node) {
    const std::string ws_name = trim(it.first.as<std::string>());
    if (active_areas.count(ws_name) == 0) {
      continue;
    }

    const YAML::Node obj_ids = it.second["obj_ids"];
    if (!obj_ids || !obj_ids.IsSequence()) {
      throw std::runtime_error(state_name + "." + ws_name + ".obj_ids must be a sequence");
    }

    for (const auto & id_node : obj_ids) {
      const int obj_id = parseObjectId(id_node);
      if (object_to_ws.count(obj_id) > 0) {
        throw std::runtime_error(
          "Object id " + std::to_string(obj_id) + " appears in multiple WS on " + state_name);
      }
      object_to_ws[obj_id] = ws_name;
    }
  }

  return object_to_ws;
}

std::map<int, ContainerAssignment> parseContainerAssignments(
  const YAML::Node & root,
  const std::map<int, ContainerInfo> & containers)
{
  std::map<int, ContainerAssignment> assignments;
  if (containers.empty()) {
    return assignments;
  }

  const YAML::Node finish_state = root["finish_state"];
  if (!finish_state || !finish_state.IsMap()) {
    return assignments;
  }

  const std::regex inside_regex("\\binside\\s+0*([0-9]+)\\b", std::regex_constants::icase);
  const std::regex object_regex("\\bO\\s*0*([0-9]+)\\b", std::regex_constants::icase);

  for (const auto & ws_it : finish_state) {
    const YAML::Node constraints = ws_it.second["constraints"];
    if (!constraints) {
      continue;
    }
    if (!constraints.IsSequence()) {
      throw std::runtime_error("finish_state constraints must be a sequence");
    }

    for (const auto & constraint_node : constraints) {
      const std::string constraint = trim(constraint_node.as<std::string>());
      std::smatch inside_match;
      if (!std::regex_search(constraint, inside_match, inside_regex)) {
        continue;
      }

      const int container_id = std::stoi(inside_match[1].str());
      const auto container_it = containers.find(container_id);
      if (container_it == containers.end()) {
        throw std::runtime_error(
                "Constraint references unknown container id: " + std::to_string(container_id));
      }

      const std::string objects_part = constraint.substr(0, inside_match.position());
      for (
        std::sregex_iterator it(objects_part.begin(), objects_part.end(), object_regex), end;
        it != end;
        ++it)
      {
        const int obj_id = std::stoi((*it)[1].str());
        if (assignments.count(obj_id) > 0) {
          throw std::runtime_error(
                  "Object id " + std::to_string(obj_id) +
                  " has more than one container constraint");
        }

        ContainerAssignment assignment;
        assignment.container_pose = container_it->second.pose_name;
        assignment.container_ws = container_it->second.ws;
        assignments[obj_id] = assignment;
      }
    }
  }

  return assignments;
}

std::map<int, std::string> parseApriltagIdToFrame(const std::string & apriltag_yaml_path)
{
  const YAML::Node tag_root = YAML::LoadFile(apriltag_yaml_path);
  const YAML::Node ids = tag_root["apriltag"]["ros__parameters"]["tag"]["ids"];
  const YAML::Node frames = tag_root["apriltag"]["ros__parameters"]["tag"]["frames"];

  if (!ids || !frames || !ids.IsSequence() || !frames.IsSequence()) {
    throw std::runtime_error("Invalid apriltag file: expected tag.ids and tag.frames sequences");
  }
  if (ids.size() != frames.size()) {
    throw std::runtime_error("apriltag ids and frames have different sizes");
  }

  std::map<int, std::string> id_to_frame;
  for (size_t i = 0; i < ids.size(); ++i) {
    const int id = ids[i].as<int>();
    const std::string frame = trim(frames[i].as<std::string>());
    id_to_frame[id] = frame;
  }

  return id_to_frame;
}

std::vector<TransferItem> buildTransfers(
  const std::map<int, ObjectInfo> & objects,
  const std::map<int, std::string> & start_index,
  const std::map<int, std::string> & finish_index,
  const std::map<int, std::string> & id_to_frame,
  const std::set<int> & ignored_object_ids,
  const std::map<int, ContainerAssignment> & container_assignments)
{
  std::set<int> all_ids;
  for (const auto & [obj_id, _] : objects) {
    all_ids.insert(obj_id);
  }
  for (const auto & [obj_id, _] : start_index) {
    if (ignored_object_ids.count(obj_id) == 0) {
      all_ids.insert(obj_id);
    }
  }
  for (const auto & [obj_id, _] : finish_index) {
    if (ignored_object_ids.count(obj_id) == 0) {
      all_ids.insert(obj_id);
    }
  }
  for (const auto & [obj_id, _] : container_assignments) {
    if (ignored_object_ids.count(obj_id) == 0) {
      all_ids.insert(obj_id);
    }
  }

  std::vector<TransferItem> transfers;
  transfers.reserve(all_ids.size());

  for (const int obj_id : all_ids) {
    TransferItem item;
    item.obj_id = obj_id;

    const auto from_it = start_index.find(obj_id);
    const auto to_it = finish_index.find(obj_id);
    const auto frame_it = id_to_frame.find(obj_id);

    if (from_it != start_index.end()) {
      item.from_ws = from_it->second;
    }

    if (to_it != finish_index.end()) {
      item.to_ws = to_it->second;
    }

    const auto assignment_it = container_assignments.find(obj_id);
    if (assignment_it != container_assignments.end()) {
      item.destination_container_pose = assignment_it->second.container_pose;
      if (item.to_ws.empty()) {
        item.to_ws = assignment_it->second.container_ws;
      } else if (item.to_ws != assignment_it->second.container_ws) {
        throw std::runtime_error(
                "Object id " + std::to_string(obj_id) +
                " finish WS does not match destination container WS");
      }
    }

    if (frame_it != id_to_frame.end()) {
      item.tag_frame = frame_it->second;
    }

    const bool has_start = !item.from_ws.empty();
    const bool has_finish = !item.to_ws.empty();
    const bool moved = has_start && has_finish && (item.from_ws != item.to_ws);
    const bool assigned_to_container = !item.destination_container_pose.empty();
    const bool removed = has_start && !has_finish;
    const bool added = !has_start && has_finish;

    item.needs_pick = moved || removed || assigned_to_container;
    item.needs_place = moved || added || assigned_to_container;

    if (!item.needs_pick && !item.needs_place) {
      continue;
    }

    // Without a resolved tag frame we cannot emit the minimal action schema.
    if (item.tag_frame.empty()) {
      continue;
    }

    transfers.push_back(item);
  }

  std::sort(
    transfers.begin(),
    transfers.end(),
    [](const TransferItem & a, const TransferItem & b) { return a.obj_id < b.obj_id; });

  return transfers;
}

const TransferItem & findTransferById(const std::vector<TransferItem> & transfers, const int obj_id)
{
  const auto it = std::find_if(
    transfers.begin(),
    transfers.end(),
    [obj_id](const TransferItem & t) { return t.obj_id == obj_id; });
  if (it == transfers.end()) {
    throw std::runtime_error("Internal planner error: missing transfer id " + std::to_string(obj_id));
  }
  return *it;
}

std::vector<int> shuffledBestCandidates(
  const std::vector<std::pair<std::string, std::pair<int, int>>> & scores,
  PlannerState & state)
{
  std::vector<int> best_indices;
  std::pair<int, int> best_score{-1, -1};

  for (std::size_t i = 0; i < scores.size(); ++i) {
    if (scores[i].second > best_score) {
      best_score = scores[i].second;
      best_indices.clear();
      best_indices.push_back(static_cast<int>(i));
    } else if (scores[i].second == best_score) {
      best_indices.push_back(static_cast<int>(i));
    }
  }

  std::shuffle(best_indices.begin(), best_indices.end(), state.rng);
  return best_indices;
}

int countPendingPicksAtWs(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const std::string & ws,
  const int free_slots)
{
  if (free_slots <= 0) {
    return 0;
  }

  int count = 0;
  for (const auto & t : transfers) {
    if (t.needs_pick && state.picked.count(t.obj_id) == 0 && t.from_ws == ws) {
      ++count;
    }
  }
  return std::min(count, free_slots);
}

int countCarriedPlacesAtWs(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const std::string & ws)
{
  int count = 0;
  for (const int obj_id : state.inventory) {
    const auto & t = findTransferById(transfers, obj_id);
    if (t.needs_place && state.placed.count(t.obj_id) == 0 && t.to_ws == ws) {
      ++count;
    }
  }
  return count;
}

bool hasUnfinishedWork(const std::vector<TransferItem> & transfers, const PlannerState & state)
{
  for (const auto & t : transfers) {
    if (t.needs_pick && state.picked.count(t.obj_id) == 0) {
      return true;
    }
    if (t.needs_place && state.placed.count(t.obj_id) == 0) {
      return true;
    }
  }
  return false;
}

std::string chooseNextWs(
  const std::vector<TransferItem> & transfers,
  const std::set<std::string> & candidate_ws,
  PlannerState & state)
{
  std::vector<std::pair<std::string, std::pair<int, int>>> scores;
  scores.reserve(candidate_ws.size());

  const int free_slots = 3 - static_cast<int>(state.inventory.size());
  const bool empty_robot = state.inventory.empty();

  for (const auto & ws : candidate_ws) {
    const int places = countCarriedPlacesAtWs(transfers, state, ws);
    const int picks = countPendingPicksAtWs(transfers, state, ws, free_slots + places);

    if (empty_robot) {
      if (picks > 0) {
        scores.push_back({ws, {picks, 0}});
      }
    } else {
      if (places > 0 || picks > 0) {
        scores.push_back({ws, {places, picks}});
      }
    }
  }

  if (scores.empty()) {
    return "";
  }

  const auto best_indices = shuffledBestCandidates(scores, state);
  return scores[best_indices.front()].first;
}

void appendGotoIfNeeded(
  YAML::Node & action_seq,
  PlannerState & state,
  const std::string & ws,
  const std::map<std::string, std::string> & ws_to_table_pose)
{
  const auto ws_it = ws_to_table_pose.find(ws);
  if (ws_it == ws_to_table_pose.end()) {
    throw std::runtime_error("Missing WS->Mesa mapping for navigation workspace: " + ws);
  }

  const std::string table_pose = ws_it->second;
  if (state.current_ws == table_pose) {
    return;
  }

  if (!state.current_ws.empty()) {
    YAML::Node home;
    home["kind"] = "home";
    home["pose_name"] = "home";
    action_seq.push_back(home);
  }

  YAML::Node nav;
  nav["kind"] = "goto";
  nav["ws"] = ws;
  nav["mesa"] = table_pose;
  action_seq.push_back(nav);
  state.current_ws = table_pose;
}

void appendPlacesAtWs(
  YAML::Node & action_seq,
  const std::vector<TransferItem> & transfers,
  PlannerState & state,
  const std::string & ws,
  const std::map<std::string, std::string> & ws_to_table_pose)
{
  std::vector<int> remaining_inventory;
  for (const int obj_id : state.inventory) {
    const auto & t = findTransferById(transfers, obj_id);
    if (t.needs_place && state.placed.count(t.obj_id) == 0 && t.to_ws == ws) {
      YAML::Node place;
      place["kind"] = "place";
      place["tag_frame"] = t.tag_frame;
      place["ws"] = t.to_ws;

      if (!t.destination_container_pose.empty()) {
        place["table_pose"] = t.destination_container_pose;
      } else {
        const auto ws_it = ws_to_table_pose.find(t.to_ws);
        if (ws_it == ws_to_table_pose.end()) {
          throw std::runtime_error("Missing WS->Mesa mapping for destination workspace: " + t.to_ws);
        }
        place["table_pose"] = ws_it->second;
      }

      action_seq.push_back(place);
      state.placed.insert(t.obj_id);
    } else {
      remaining_inventory.push_back(obj_id);
    }
  }
  state.inventory = remaining_inventory;
}

void appendPicksAtWs(
  YAML::Node & action_seq,
  const std::vector<TransferItem> & transfers,
  PlannerState & state,
  const std::string & ws)
{
  for (const auto & t : transfers) {
    if (state.inventory.size() >= 3) {
      return;
    }
    if (!t.needs_pick || state.picked.count(t.obj_id) > 0 || t.from_ws != ws) {
      continue;
    }

    YAML::Node pick;
    pick["kind"] = "pick";
    pick["tag_frame"] = t.tag_frame;
    pick["ws"] = t.from_ws;
    action_seq.push_back(pick);

    state.picked.insert(t.obj_id);
    if (t.needs_place) {
      state.inventory.push_back(t.obj_id);
    }
  }
}

YAML::Node buildOutput(
  const YAML::Node & competition_root,
  const std::vector<TransferItem> & transfers,
  const std::string & apriltag_yaml_path,
  const std::map<std::string, std::string> & ws_to_table_pose)
{
  (void)competition_root;
  (void)apriltag_yaml_path;

  YAML::Node output;
  YAML::Node action_seq(YAML::NodeType::Sequence);
  PlannerState state;
  std::set<std::string> candidate_ws;

  for (const auto & t : transfers) {
    if (t.needs_pick && !t.from_ws.empty()) {
      candidate_ws.insert(t.from_ws);
    }
    if (t.needs_place && !t.to_ws.empty()) {
      candidate_ws.insert(t.to_ws);
    }
  }

  while (hasUnfinishedWork(transfers, state)) {
    const std::string next_ws = chooseNextWs(transfers, candidate_ws, state);
    if (next_ws.empty()) {
      throw std::runtime_error("Task planner got stuck: no reachable pick/place candidate");
    }

    appendGotoIfNeeded(action_seq, state, next_ws, ws_to_table_pose);
    appendPlacesAtWs(action_seq, transfers, state, next_ws, ws_to_table_pose);
    appendPicksAtWs(action_seq, transfers, state, next_ws);
  }


  output["actions"] = action_seq;
  return output;
}

std::string resolveApriltagPath(int argc, char ** argv)
{
  if (argc >= 4) {
    return argv[3];
  }

  try {
    return ament_index_cpp::get_package_share_directory("apriltag_ros") + "/cfg/tags_36h11.yaml";
  } catch (const std::exception &) {
    throw std::runtime_error(
            "Could not resolve default apriltag config. Provide third argument: <apriltag_yaml>");
  }
}

std::string resolveWsTableMapPath(int argc, char ** argv)
{
  if (argc >= 5) {
    return argv[4];
  }

  try {
    return ament_index_cpp::get_package_share_directory("manip_bt") +
           "/behavior_tree_manip/ws_table_mapping.yaml";
  } catch (const std::exception &) {
    throw std::runtime_error(
            "Could not resolve default WS map config. Provide fourth argument: <ws_table_map_yaml>");
  }
}

std::map<std::string, std::string> parseWsToTablePose(const std::string & ws_map_yaml_path)
{
  const YAML::Node root = YAML::LoadFile(ws_map_yaml_path);
  const YAML::Node ws_map = root["ws_to_table_pose"];
  if (!ws_map || !ws_map.IsMap()) {
    throw std::runtime_error("Invalid WS map file: expected ws_to_table_pose map");
  }

  std::map<std::string, std::string> result;
  for (const auto & it : ws_map) {
    const std::string ws = trim(it.first.as<std::string>());
    const std::string table_pose = trim(it.second.as<std::string>());
    if (ws.empty() || table_pose.empty()) {
      continue;
    }
    result[ws] = table_pose;
  }

  if (result.empty()) {
    throw std::runtime_error("WS map file does not contain any valid ws_to_table_pose entries");
  }

  return result;
}

std::string resolvePathWithFallbacks(
  const std::string & input_path,
  const std::vector<std::string> & fallback_dirs)
{
  namespace fs = std::filesystem;

  if (fs::exists(input_path)) {
    return input_path;
  }

  for (const auto & dir : fallback_dirs) {
    fs::path candidate = fs::path(dir) / input_path;
    if (fs::exists(candidate)) {
      return candidate.string();
    }
  }

  std::ostringstream oss;
  oss << "bad file: " << input_path;
  throw std::runtime_error(oss.str());
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr
      << "Usage: task_planner <competition_yaml> <output_yaml> [apriltag_yaml] [ws_table_map_yaml]"
      << std::endl;
    return 2;
  }

  const std::string competition_yaml_arg = argv[1];
  const std::string output_yaml_path = argv[2];

  try {
    std::vector<std::string> competition_fallback_dirs;
    std::vector<std::string> apriltag_fallback_dirs;
    std::vector<std::string> ws_map_fallback_dirs;

    try {
      competition_fallback_dirs.push_back(
        ament_index_cpp::get_package_share_directory("manip_bt") + "/behavior_tree_manip");
    } catch (const std::exception &) {
      // Optional fallback only.
    }
    competition_fallback_dirs.push_back("src/manip_bt/behavior_tree_manip");
    ws_map_fallback_dirs = competition_fallback_dirs;

    try {
      apriltag_fallback_dirs.push_back(
        ament_index_cpp::get_package_share_directory("apriltag_ros") + "/cfg");
    } catch (const std::exception &) {
      // Optional fallback only.
    }
    apriltag_fallback_dirs.push_back("src/apriltag_ros/cfg");

    const std::string competition_yaml_path =
      resolvePathWithFallbacks(competition_yaml_arg, competition_fallback_dirs);

    const std::string apriltag_yaml_arg = resolveApriltagPath(argc, argv);
    const std::string apriltag_yaml_path =
      resolvePathWithFallbacks(apriltag_yaml_arg, apriltag_fallback_dirs);

    const std::string ws_map_yaml_arg = resolveWsTableMapPath(argc, argv);
    const std::string ws_map_yaml_path =
      resolvePathWithFallbacks(ws_map_yaml_arg, ws_map_fallback_dirs);
    const auto ws_to_table_pose = parseWsToTablePose(ws_map_yaml_path);

    const YAML::Node competition_root = YAML::LoadFile(competition_yaml_path);
    const auto active_areas = parseActiveAreas(competition_root);
    std::set<int> ignored_object_ids;
    const auto objects = parseObjects(competition_root, ignored_object_ids);
    const auto containers = parseContainers(competition_root);
    const auto container_assignments = parseContainerAssignments(competition_root, containers);
    const auto start_index = buildStateIndex(competition_root["start_state"], active_areas, "start_state");
    const auto finish_index = buildStateIndex(competition_root["finish_state"], active_areas, "finish_state");
    const auto id_to_frame = parseApriltagIdToFrame(apriltag_yaml_path);
    const auto transfers =
      buildTransfers(
        objects,
        start_index,
        finish_index,
        id_to_frame,
        ignored_object_ids,
        container_assignments);
    const auto output = buildOutput(competition_root, transfers, apriltag_yaml_path, ws_to_table_pose);

    YAML::Emitter out;
    out << output;

    std::ofstream fout(output_yaml_path);
    if (!fout.is_open()) {
      throw std::runtime_error("Could not open output file: " + output_yaml_path);
    }
    fout << out.c_str() << std::endl;

    std::cout << "Wrote translation to " << output_yaml_path << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "Translation failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
