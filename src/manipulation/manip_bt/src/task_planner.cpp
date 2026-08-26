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
  std::string color;   // canonica (RED, BLUE, ...) ou vazia
};

// Container da MESA (arena). 2026-08-25: nao tem mais tag — e' identificado
// pela COR (container_detector). `pose_name` (ct<id>) foi removido.
struct ContainerInfo
{
  int id = -1;
  std::string color;
  std::string ws;
};

struct ContainerAssignment
{
  int container_id = -1;
  std::string container_color;
  std::string container_ws;
};

struct TransferItem
{
  int obj_id = -1;
  std::string tag_frame;
  std::string from_ws;
  std::string to_ws;
  // 2026-08-25: cor do container da mesa onde soltar (RED|BLUE); vazia =
  // mesa. Vem da constraint "inside" (formato antigo) ou da cor do proprio
  // objeto (formato novo: se houver container dessa cor visivel, o place usa).
  std::string container_color;
  bool needs_pick = false;
  bool needs_place = false;
  // 2026-08-24 EMPILHAR: este objeto e' solto EM CIMA do objeto `stack_on_obj`
  // (frame `stack_on_frame`), que precisa estar na mesa de `to_ws` antes.
  int stack_on_obj = -1;
  std::string stack_on_frame;
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

// 2026-08-25 — cores. Aceita ingles/portugues, maiusculas/minusculas.
std::string canonicalColor(const std::string & raw)
{
  const std::string t = toUpper(trim(raw));
  static const std::map<std::string, std::string> kMap{
    {"RED", "RED"}, {"VERMELHO", "RED"}, {"VERMELHA", "RED"},
    {"BLUE", "BLUE"}, {"AZUL", "BLUE"},
    {"GREEN", "GREEN"}, {"VERDE", "GREEN"},
    {"YELLOW", "YELLOW"}, {"AMARELO", "YELLOW"}, {"AMARELA", "YELLOW"},
    {"BLACK", "BLACK"}, {"PRETO", "BLACK"}, {"PRETA", "BLACK"},
    {"WHITE", "WHITE"}, {"BRANCO", "WHITE"}, {"BRANCA", "WHITE"},
    {"ORANGE", "ORANGE"}, {"LARANJA", "ORANGE"},
  };
  const auto it = kMap.find(t);
  return it == kMap.end() ? "" : it->second;
}

/// Cor de um objeto: campo `color:` (formato antigo) ou um token de cor
/// dentro de `type:` (formato novo: "RED", "ATTC_RED", "ATTC-blue"...).
/// Vazia se nao houver cor conhecida (ex.: `type: ATTC` da versao equipe).
std::string extractColor(const YAML::Node & obj)
{
  const std::string color_field = trim(obj["color"].as<std::string>(""));
  if (!color_field.empty()) {
    const std::string c = canonicalColor(color_field);
    if (c.empty()) {
      std::cerr << "[task_planner] AVISO: color '" << color_field
                << "' desconhecida — usando como esta." << std::endl;
      return toUpper(color_field);
    }
    return c;
  }
  const std::string type = toUpper(trim(obj["type"].as<std::string>("")));
  std::string token;
  for (std::size_t i = 0; i <= type.size(); ++i) {
    const bool sep = i == type.size() || !std::isalnum(static_cast<unsigned char>(type[i]));
    if (!sep) {
      token += type[i];
      continue;
    }
    if (!token.empty()) {
      const std::string c = canonicalColor(token);
      if (!c.empty()) {
        return c;
      }
      token.clear();
    }
  }
  // Sem token de cor separado (ex.: "ATTCRED") = sem cor: inferir por
  // substring dava falso positivo ("REDUCED" -> RED).
  return "";
}

/// Cores que TEM container fisico na arena (detector: vermelho/azul).
bool colorHasContainer(const std::string & color)
{
  return color == "RED" || color == "BLUE";
}

/// 2026-08-25: YAML do @Work Commander traz schema_version/version. So
/// avisamos — chaves desconhecidas sempre foram ignoradas.
void checkSchemaVersion(const YAML::Node & root)
{
  const YAML::Node sv = root["schema_version"];
  if (sv) {
    const std::string v = trim(sv.as<std::string>(""));
    if (v != "1") {
      std::cerr << "[task_planner] AVISO: schema_version '" << v
                << "' desconhecida (esperado 1) — usando o parser v1." << std::endl;
    }
  }
  const YAML::Node ver = root["version"];
  if (ver) {
    std::cout << "[task_planner] task version: " << trim(ver.as<std::string>("")) << std::endl;
  }
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
    if (type_upper.find("DECOY") != std::string::npos) {   // "DECOY", "DECOY_RED"...
      ignored_object_ids.insert(id);
      continue;
    }

    if (by_id.count(id) > 0) {
      throw std::runtime_error("Duplicate object id in objects: " + std::to_string(id));
    }

    ObjectInfo info;
    info.id = id;
    info.type = trim(obj["type"].as<std::string>(""));
    info.color = extractColor(obj);
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
    info.color = canonicalColor(container["color"].as<std::string>(""));
    info.ws = trim(container["at"].as<std::string>(""));
    if (info.ws.empty()) {
      info.ws = trim(container["at_ws"].as<std::string>(""));
    }
    if (info.ws.empty()) {
      throw std::runtime_error("containers id " + std::to_string(id) + " missing at/at_ws");
    }

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
    const YAML::Node fs = root["finish_state"];
    if (fs && fs.IsMap()) {
      for (const auto & ws_it : fs) {
        const YAML::Node cons = ws_it.second["constraints"];
        if (cons && cons.IsSequence()) {
          for (const auto & c : cons) {
            if (toUpper(c.as<std::string>("")).find("INSIDE") != std::string::npos) {
              std::cerr << "[task_planner] AVISO: constraint '" << c.as<std::string>("")
                        << "' ignorada — nao ha bloco 'containers:' no yaml." << std::endl;
            }
          }
        }
      }
    }
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
      if (container_it->second.color.empty()) {
        throw std::runtime_error(
                "containers id " + std::to_string(container_id) +
                " sem 'color': a entrega em container agora e' por COR (deteccao visual), "
                "nao por tag");
      }
      if (!colorHasContainer(container_it->second.color)) {
        throw std::runtime_error(
                "containers id " + std::to_string(container_id) + " com cor '" +
                container_it->second.color + "': o detector so conhece RED|BLUE");
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
        assignment.container_id = container_id;
        assignment.container_color = container_it->second.color;
        assignment.container_ws = container_it->second.ws;
        assignments[obj_id] = assignment;
      }
    }
  }

  return assignments;
}

/// 2026-08-24 — EMPILHAMENTO no YAML da competicao. Em finish_state.<WS>:
///   stacks: [[04, 07], [1, 2, 3]]   # cada lista de BAIXO para CIMA
/// ou, em constraints, frases "O7 on top of O4" / "O7 em cima de O4" /
/// "O7 sobre O4" / "O7 stacked on O4". Devolve topo -> base. Valida que os
/// dois objetos terminam na MESMA estacao (a da declaracao), que nao ha
/// topo com duas bases e que nao ha ciclo.
std::map<int, int> parseStackAssignments(
  const YAML::Node & root,
  const std::map<int, std::string> & finish_index)
{
  std::map<int, int> top_to_base;
  const YAML::Node finish_state = root["finish_state"];
  if (!finish_state || !finish_state.IsMap()) {
    return top_to_base;
  }

  const std::regex on_top_regex(
    "\\bO\\s*0*([0-9]+)\\b\\s*(?:must\\s+be\\s+|deve\\s+(?:estar|ficar)\\s+)?"
    "(?:on\\s+top\\s+of|stacked\\s+on|em\\s+cima\\s+d[aoe]|sobre)\\s*\\bO?\\s*0*([0-9]+)\\b",
    std::regex_constants::icase);

  auto add = [&](const std::string & ws_name, const int top, const int base) {
      if (top == base) {
        throw std::runtime_error(
                "finish_state." + ws_name + ": objeto " + std::to_string(top) +
                " nao pode ser empilhado sobre ele mesmo");
      }
      for (const int id : {top, base}) {
        const auto it = finish_index.find(id);
        if (it == finish_index.end() || it->second != ws_name) {
          throw std::runtime_error(
                  "finish_state." + ws_name + ": pilha usa o objeto " + std::to_string(id) +
                  " que nao termina nessa estacao (obj_ids)");
        }
      }
      if (top_to_base.count(top) > 0 && top_to_base[top] != base) {
        throw std::runtime_error(
                "finish_state." + ws_name + ": objeto " + std::to_string(top) +
                " declarado em cima de duas bases");
      }
      top_to_base[top] = base;
    };

  for (const auto & ws_it : finish_state) {
    const std::string ws_name = trim(ws_it.first.as<std::string>());

    const YAML::Node stacks = ws_it.second["stacks"];
    if (stacks) {
      if (!stacks.IsSequence()) {
        throw std::runtime_error("finish_state." + ws_name + ".stacks must be a sequence");
      }
      for (const auto & stack : stacks) {
        if (!stack.IsSequence() || stack.size() < 2) {
          throw std::runtime_error(
                  "finish_state." + ws_name +
                  ".stacks: cada pilha e' uma lista de >= 2 ids, de baixo para cima");
        }
        for (std::size_t i = 1; i < stack.size(); ++i) {
          add(ws_name, parseObjectId(stack[i]), parseObjectId(stack[i - 1]));
        }
      }
    }

    const YAML::Node constraints = ws_it.second["constraints"];
    if (constraints && constraints.IsSequence()) {
      for (const auto & constraint_node : constraints) {
        const std::string constraint = trim(constraint_node.as<std::string>());
        // Todas as relacoes da frase ("O7 on top of O4, O1 on top of O7").
        for (
          std::sregex_iterator it(constraint.begin(), constraint.end(), on_top_regex), end;
          it != end;
          ++it)
        {
          add(ws_name, std::stoi((*it)[1].str()), std::stoi((*it)[2].str()));
        }
      }
    }
  }

  // Sem ciclos (4 sobre 7 e 7 sobre 4).
  for (const auto & [top, _] : top_to_base) {
    std::set<int> seen{top};
    int cur = top;
    while (top_to_base.count(cur) > 0) {
      cur = top_to_base[cur];
      if (!seen.insert(cur).second) {
        throw std::runtime_error(
                "finish_state: pilha com ciclo envolvendo o objeto " + std::to_string(top));
      }
    }
  }

  return top_to_base;
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
  const std::map<int, ContainerAssignment> & container_assignments,
  const std::map<int, int> & stack_assignments,
  const bool yaml_declares_containers)
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
  std::vector<int> ids_without_frame;

  // Participantes de pilha (topo ou base) nunca vao para container.
  std::set<int> stack_participants;
  for (const auto & [top, base] : stack_assignments) {
    stack_participants.insert(top);
    stack_participants.insert(base);
  }

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
    const bool assigned_to_container = assignment_it != container_assignments.end();
    if (assigned_to_container) {
      if (stack_participants.count(obj_id) > 0) {
        throw std::runtime_error(
                "Object id " + std::to_string(obj_id) +
                " esta numa pilha E numa constraint de container — contradicao");
      }
      item.container_color = assignment_it->second.container_color;
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

    const auto stack_it = stack_assignments.find(obj_id);
    const bool stacked_top = stack_it != stack_assignments.end();
    if (stacked_top) {
      item.stack_on_obj = stack_it->second;
      const auto base_frame_it = id_to_frame.find(item.stack_on_obj);
      if (base_frame_it == id_to_frame.end()) {
        throw std::runtime_error(
                "Objeto base da pilha " + std::to_string(item.stack_on_obj) +
                " nao tem frame de tag no apriltag yaml");
      }
      item.stack_on_frame = base_frame_it->second;
    }

    const bool has_start = !item.from_ws.empty();
    const bool has_finish = !item.to_ws.empty();
    const bool moved = has_start && has_finish && (item.from_ws != item.to_ws);
    const bool removed = has_start && !has_finish;
    const bool added = !has_start && has_finish;

    if (removed) {
      std::cerr << "[task_planner] AVISO: objeto " << obj_id << " esta no start_state ("
                << item.from_ws << ") mas nao no finish_state — sem destino, nao sera pego."
                << std::endl;
      continue;
    }
    if (added) {
      throw std::runtime_error(
              "Object id " + std::to_string(obj_id) + " esta no finish_state (" + item.to_ws +
              ") mas em nenhum start_state — sem origem para pegar");
    }

    // Topo de pilha sempre e' pego e solto (mesmo se comeca e termina na
    // mesma estacao: precisa subir em cima da base).
    item.needs_pick = moved || assigned_to_container || (stacked_top && has_start);
    item.needs_place = moved || assigned_to_container || stacked_top;

    if (!item.needs_pick && !item.needs_place) {
      continue;
    }

    // Formato novo (SEM bloco `containers:` no yaml): objeto de cor com
    // container fisico, solto numa mesa comum e fora de pilha -> o place
    // tenta o container dessa cor (se a camera vir um; senao, mesa). No
    // formato antigo (com `containers:`) so as constraints "inside" mandam
    // para container — quem nao esta numa constraint vai para a mesa.
    if (!yaml_declares_containers && item.needs_place && item.container_color.empty() &&
      stack_participants.count(obj_id) == 0 && item.to_ws.rfind("WS", 0) == 0)
    {
      const auto obj_it = objects.find(obj_id);
      if (obj_it != objects.end() && colorHasContainer(obj_it->second.color)) {
        item.container_color = obj_it->second.color;
      }
    } else if (assigned_to_container) {
      const auto obj_it = objects.find(obj_id);
      if (obj_it != objects.end() && !obj_it->second.color.empty() &&
        obj_it->second.color != item.container_color)
      {
        std::cerr << "[task_planner] AVISO: objeto " << obj_id << " e' " << obj_it->second.color
                  << " mas a constraint manda para o container " << item.container_color
                  << " — a constraint vence." << std::endl;
      }
    }

    // Sem frame de tag nao ha como pegar/soltar: erro explicito (antes o
    // objeto sumia do plano em silencio).
    if (item.tag_frame.empty()) {
      ids_without_frame.push_back(obj_id);
      continue;
    }

    transfers.push_back(item);
  }

  if (!ids_without_frame.empty()) {
    std::string ids;
    for (const int id : ids_without_frame) {
      ids += (ids.empty() ? "" : ", ") + std::to_string(id);
    }
    throw std::runtime_error(
            "Objetos sem frame de tag no apriltag yaml (tag.ids/frames/sizes): " + ids +
            " — acrescentar em tags_36h11.yaml");
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

// ---- pilhas (2026-08-24) --------------------------------------------------
// A base de uma pilha esta "na mesa" se ja foi solta ou se nunca precisou de
// acao (comeca e termina na mesma estacao, sem ser topo de outra pilha).
bool baseOnTable(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const int base_id)
{
  if (state.placed.count(base_id) > 0) {
    return true;
  }
  const auto it = std::find_if(
    transfers.begin(), transfers.end(),
    [base_id](const TransferItem & x) { return x.obj_id == base_id; });
  return it == transfers.end() || !it->needs_place;
}

bool baseCarried(const PlannerState & state, const int base_id)
{
  return std::find(state.inventory.begin(), state.inventory.end(), base_id) !=
         state.inventory.end();
}

/// Um topo so pode ser PEGO quando a base ja esta na mesa ou tambem esta no
/// inventario (senao o topo ficaria preso a bordo esperando a base — ver a
/// revisao adversarial de 24/08: isso travava o planejador num laco infinito).
bool topPickable(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t)
{
  return t.stack_on_obj < 0 || baseOnTable(transfers, state, t.stack_on_obj) ||
         baseCarried(state, t.stack_on_obj);
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
    if (t.needs_pick && state.picked.count(t.obj_id) == 0 && t.from_ws == ws &&
      topPickable(transfers, state, t))
    {
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
    if (t.needs_place && state.placed.count(t.obj_id) == 0 && t.to_ws == ws &&
      topPickable(transfers, state, t))   // topo sem base disponivel nao conta
    {
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
  // BUG 2026-08-07: current_ws guardava a MESA (table_pose), nao a estacao.
  // Com WS_1 e WS_2 ambas Mesa15, o goto WS_2 era pulado e o robo "colocava
  // na WS_2" sem sair da WS_1. Comparar/registrar SEMPRE pela estacao (ws).
  if (state.current_ws == ws) {
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
  state.current_ws = ws;
}

void appendPlacesAtWs(
  YAML::Node & action_seq,
  const std::vector<TransferItem> & transfers,
  PlannerState & state,
  const std::string & ws,
  const std::map<std::string, std::string> & ws_to_table_pose)
{
  const auto emit_place = [&](const TransferItem & t) {
      YAML::Node place;
      place["kind"] = "place";
      place["tag_frame"] = t.tag_frame;
      place["ws"] = t.to_ws;

      const auto ws_it = ws_to_table_pose.find(t.to_ws);
      if (ws_it == ws_to_table_pose.end()) {
        throw std::runtime_error("Missing WS->Mesa mapping for destination workspace: " + t.to_ws);
      }
      place["table_pose"] = ws_it->second;
      if (!t.stack_on_frame.empty()) {
        place["stack_on"] = t.stack_on_frame;
      }
      if (!t.container_color.empty()) {
        place["container_color"] = t.container_color;
      }

      action_seq.push_back(place);
      state.placed.insert(t.obj_id);
    };

  // Duas passadas: primeiro o que NAO e' topo de pilha (inclui as bases),
  // depois os topos cuja base ja esta na mesa. Um topo cuja base ainda nao
  // foi solta fica no inventario (o laco de buildOutput volta aqui).
  std::vector<int> remaining_inventory;
  std::vector<int> deferred_tops;
  for (const int obj_id : state.inventory) {
    const auto & t = findTransferById(transfers, obj_id);
    const bool here = t.needs_place && state.placed.count(t.obj_id) == 0 && t.to_ws == ws;
    if (!here) {
      remaining_inventory.push_back(obj_id);
    } else if (t.stack_on_obj >= 0) {
      deferred_tops.push_back(obj_id);
    } else {
      emit_place(t);
    }
  }
  // Topos: repete ate nao haver progresso (pilha de 3+: o topo do topo so
  // sai depois do meio).
  bool progress = true;
  while (progress && !deferred_tops.empty()) {
    progress = false;
    std::vector<int> still_deferred;
    for (const int obj_id : deferred_tops) {
      const auto & t = findTransferById(transfers, obj_id);
      if (baseOnTable(transfers, state, t.stack_on_obj)) {
        emit_place(t);
        progress = true;
      } else {
        still_deferred.push_back(obj_id);
      }
    }
    deferred_tops = still_deferred;
  }
  for (const int obj_id : deferred_tops) {
    remaining_inventory.push_back(obj_id);
  }
  state.inventory = remaining_inventory;
}

void appendPicksAtWs(
  YAML::Node & action_seq,
  const std::vector<TransferItem> & transfers,
  PlannerState & state,
  const std::string & ws,
  const std::map<std::string, std::string> & ws_to_table_pose)
{
  // Topo de pilha so e' pego com a base ja na mesa ou a bordo (topPickable);
  // como a base pode ter sido pega nesta mesma passada, repete ate nao haver
  // progresso.
  bool progress = true;
  while (progress) {
  progress = false;
  for (const auto & t : transfers) {
    if (state.inventory.size() >= 3) {
      return;
    }
    if (!t.needs_pick || state.picked.count(t.obj_id) > 0 || t.from_ws != ws) {
      continue;
    }
    if (!topPickable(transfers, state, t)) {
      continue;
    }
    progress = true;

    YAML::Node pick;
    pick["kind"] = "pick";
    pick["tag_frame"] = t.tag_frame;
    pick["ws"] = t.from_ws;
    // 2026-08-12: o pick tambem precisa saber o tipo de mesa — a prateleira
    // (MesaSh) exige uma sequencia de movimentos propria. Ate aqui so o place
    // recebia table_pose, e o pick era cego ao tipo de estacao.
    const auto ws_it = ws_to_table_pose.find(t.from_ws);
    if (ws_it == ws_to_table_pose.end()) {
      throw std::runtime_error("Missing WS->Mesa mapping for source workspace: " + t.from_ws);
    }
    pick["table_pose"] = ws_it->second;
    action_seq.push_back(pick);

    state.picked.insert(t.obj_id);
    if (t.needs_place) {
      state.inventory.push_back(t.obj_id);
    }
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

    const std::size_t picked_before = state.picked.size();
    const std::size_t placed_before = state.placed.size();
    appendGotoIfNeeded(action_seq, state, next_ws, ws_to_table_pose);
    appendPlacesAtWs(action_seq, transfers, state, next_ws, ws_to_table_pose);
    appendPicksAtWs(action_seq, transfers, state, next_ws, ws_to_table_pose);
    // Guarda de progresso (revisao 24/08): sem pick nem place novo nesta
    // iteracao o laco giraria para sempre — falha explicita em vez disso.
    if (state.picked.size() == picked_before && state.placed.size() == placed_before) {
      throw std::runtime_error(
              "Task planner got stuck: sem progresso em " + next_ws +
              " (pilha cuja base nao pode ser alcancada? inventario: " +
              std::to_string(state.inventory.size()) + " objeto(s) a bordo)");
    }
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
    checkSchemaVersion(competition_root);
    const auto active_areas = parseActiveAreas(competition_root);
    std::set<int> ignored_object_ids;
    const auto objects = parseObjects(competition_root, ignored_object_ids);
    const auto containers = parseContainers(competition_root);
    const auto container_assignments = parseContainerAssignments(competition_root, containers);
    const auto start_index = buildStateIndex(competition_root["start_state"], active_areas, "start_state");
    const auto finish_index = buildStateIndex(competition_root["finish_state"], active_areas, "finish_state");
    const auto id_to_frame = parseApriltagIdToFrame(apriltag_yaml_path);
    const auto stack_assignments = parseStackAssignments(competition_root, finish_index);
    const auto transfers =
      buildTransfers(
        objects,
        start_index,
        finish_index,
        id_to_frame,
        ignored_object_ids,
        container_assignments,
        stack_assignments,
        !containers.empty());
    const auto output = buildOutput(competition_root, transfers, apriltag_yaml_path, ws_to_table_pose);

    // Resumo para o operador (run_mission.py repassa o stdout antes de confirmar).
    {
      std::size_t n_container = 0;
      std::size_t n_stack = 0;
      std::string container_list;
      for (const auto & t : transfers) {
        if (!t.container_color.empty()) {
          ++n_container;
          container_list += (container_list.empty() ? "" : ", ") + t.tag_frame + "->" +
            t.container_color;
        }
        if (!t.stack_on_frame.empty()) {
          ++n_stack;
        }
      }
      std::cout << "[task_planner] " << transfers.size() << " objeto(s) a transportar; "
                << ignored_object_ids.size() << " decoy(s) ignorado(s); "
                << n_container << " place(s) em container por cor"
                << (container_list.empty() ? "" : " (" + container_list + ")") << "; "
                << n_stack << " empilhamento(s)." << std::endl;
    }

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
