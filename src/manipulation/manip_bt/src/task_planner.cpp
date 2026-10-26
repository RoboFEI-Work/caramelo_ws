// task_planner: traduz o YAML da competicao (@Work Commander: formato oficial
// 2026, e os dois formatos anteriores — antigo com constraints e "novo 25/08";
// BMT/BTT/ATT/AMT) em uma lista de acoes goto/pick/place/amt1_sort que o
// bt_yaml_executor consome.
//
// Uso: task_planner <competicao.yaml> <saida_actions.yaml> <tags_36h11.yaml>
//                   <ws_table_mapping.yaml>
// Quem chama: caramelo_navigation/scripts/run_mission.py (sempre com os 4
// caminhos absolutos), manip_bt/test/run_amt1_scenario.sh e o pytest
// manip_bt/test/test_task_planner_formato2026.py.
// Saida: 0 ok, 1 falha de traducao, 2 argumentos errados.
// Ordem das estacoes: se a pasta do ws_table_mapping.yaml tiver
// dock_distances.yaml, a ordem sai da busca por tempo (task_plan_search);
// senao, do guloso de sempre. Ver docs/planejador_por_tempo.md.

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "manip_bt/mission_map_config.hpp"
#include "manip_bt/task_plan_search.hpp"

namespace
{

struct ObjectInfo
{
  int id = -1;
  std::string type;
  std::string color;   // canonica (RED, BLUE, ...) ou vazia
};

// Container da MESA (arena): nao tem tag, e' identificado pela COR
// (container_detector). No formato 2026 vem sem id ({color, at}): o id e'
// interno e NEGATIVO (-1, -2, ...), so para distinguir as entradas.
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

// TransferItem, PlannerState, kMaxOnboard e as regras de pick/place moram em
// task_plan_search (compartilhadas com a busca por tempo e com o gtest).
using manip_bt::task_plan::TransferItem;
using manip_bt::task_plan::kMaxOnboard;
using manip_bt::task_plan::tablePoseForWs;


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

// Cores: aceita ingles/portugues, maiusculas/minusculas.
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

/// YAML do @Work Commander traz schema_version/version. So avisamos:
/// chaves desconhecidas sempre foram ignoradas.
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

std::string toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

// ---- AMT1 ------------------------------------------------------------------
// Advanced Manipulation Task 1: os cubos com tag da mesa de precisao (PP_*)
// comecam e terminam na MESMA estacao, so mudam de ORDEM (crescente de id, da
// esquerda para a direita; no formato 2026 o alvo por slot e' explicito —
// ver deriveAmtTargetOrder). O fluxo normal descartava esses objetos em
// silencio (0 acoes). A task e reconhecida SO pelo task_id ou pelo name;
// qualquer outra task segue o fluxo antigo, intocado (que, quando parecer
// AMT1, so AVISA — ver looksLikeAmt1 em main).
//
// Reconhecimento tolerante a escrita:
//   task_id: AMT ou AMT1 sem distinguir maiusculas, aceitando separadores
//            entre AMT e 1 (AMT-1, AMT_1, "AMT 1"); o caractere seguinte NAO
//            pode ser digito (AMT2 / AMT10 / AMT11 nao sao AMT1). A edicao
//            2026 do @Work Commander usa task_id AMT, sem o 1.
//   name:    contem "advanced manipulation task" OU "... test", com ou sem
//            o " 1", sem distinguir maiusculas e com espacos multiplos; o
//            que vier depois NAO pode ser outro numero (task 2 / task 10
//            nao casam).
bool isAmt1Task(const YAML::Node & root)
{
  static const std::regex kTaskId("^AMT(?:[\\s_-]*1)?(?![0-9])");
  static const std::regex kName(
    "\\badvanced\\s+manipulation\\s+(task|test)(?:\\s+1)?(?!\\s*[0-9])\\b");
  const std::string task_id = toUpper(trim(root["task_id"].as<std::string>("")));
  if (std::regex_search(task_id, kTaskId)) {
    return true;
  }
  const std::string name = toLower(trim(root["name"].as<std::string>("")));
  return std::regex_search(name, kName);
}

/// Erro unico (buildTransfers e AMT1) para ids sem frame no apriltag yaml.
void throwIdsWithoutFrame(const std::vector<int> & ids_without_frame)
{
  std::string ids;
  for (const int id : ids_without_frame) {
    ids += (ids.empty() ? "" : ", ") + std::to_string(id);
  }
  throw std::runtime_error(
          "Objetos sem frame de tag no apriltag yaml (tag.ids/frames/sizes): " + ids +
          " — acrescentar em tags_36h11.yaml");
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

  int next_internal_id = -1;
  for (const auto & container : containers) {
    // Formato 2026: {color, at} sem id -> id interno negativo.
    const int id = container["id"] && !container["id"].IsNull() ?
      parseObjectId(container["id"]) : next_internal_id--;
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
      throw std::runtime_error(
              "containers: entrada " + (id < 0 ? "#" + std::to_string(-id) : "id " +
              std::to_string(id)) + " sem at/at_ws");
    }
    if (id < 0 && info.color.empty()) {
      throw std::runtime_error(
              "containers: entrada #" + std::to_string(-id) + " (em " + info.ws +
              ") sem 'color' — sem id, o container so e' identificado pela cor");
    }

    by_id[id] = info;
  }

  return by_id;
}

/// Lista de ids (sequencia) de um sub-bloco; ausente ou null = vazia.
std::vector<int> parseIdList(const YAML::Node & node, const std::string & where)
{
  std::vector<int> ids;
  if (!node || node.IsNull()) {
    return ids;
  }
  if (!node.IsSequence()) {
    throw std::runtime_error(where + " must be a sequence of object ids");
  }
  for (const auto & id_node : node) {
    ids.push_back(parseObjectId(id_node));
  }
  return ids;
}

/// Ids de uma area do start/finish_state: obj_ids (como sempre) UNIDOS aos
/// sub-blocos do formato 2026 (table, containers.<COR>, levels.<NIVEL>,
/// slots.<tag> e stack). O contrato diz que a uniao e' igual a obj_ids; se
/// diferir, AVISA e usa a uniao. Formato antigo: so obj_ids.
std::vector<int> collectAreaObjectIds(
  const YAML::Node & area,
  const std::string & state_name,
  const std::string & ws_name)
{
  const std::string where = state_name + "." + ws_name;
  const YAML::Node obj_ids = area["obj_ids"];
  const bool has_obj_ids = obj_ids && obj_ids.IsSequence();

  std::vector<int> from_blocks;
  const auto append = [&from_blocks](const std::vector<int> & ids) {
      from_blocks.insert(from_blocks.end(), ids.begin(), ids.end());
    };
  append(parseIdList(area["table"], where + ".table"));
  append(parseIdList(area["stack"], where + ".stack"));
  for (const char * block : {"containers", "levels"}) {
    const YAML::Node node = area[block];
    if (node && node.IsMap()) {
      for (const auto & it : node) {
        append(parseIdList(it.second, where + "." + block + "." + it.first.as<std::string>("")));
      }
    }
  }
  const YAML::Node slots = area["slots"];
  if (slots && slots.IsMap()) {
    for (const auto & it : slots) {
      if (it.second && !it.second.IsNull()) {
        from_blocks.push_back(parseObjectId(it.second));
      }
    }
  }
  const bool has_blocks = area["table"] || area["stack"] || area["containers"] ||
    area["levels"] || (slots && slots.IsMap());

  if (!has_obj_ids && !has_blocks) {
    throw std::runtime_error(where + ".obj_ids must be a sequence");
  }

  std::vector<int> ids;
  if (has_obj_ids) {
    ids = parseIdList(obj_ids, where + ".obj_ids");
  }
  if (has_blocks && has_obj_ids) {
    std::set<int> a(ids.begin(), ids.end());
    std::set<int> b(from_blocks.begin(), from_blocks.end());
    if (a != b) {
      std::cerr << "[task_planner] AVISO: " << where
                << ": obj_ids difere da uniao de table/containers/levels/slots/stack — "
                << "usando a uniao." << std::endl;
    }
  }
  for (const int id : from_blocks) {
    if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
      ids.push_back(id);
    }
  }
  return ids;
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

    if (!it.second || !it.second.IsMap()) {
      throw std::runtime_error(state_name + "." + ws_name + " must be a map");
    }
    for (const int obj_id : collectAreaObjectIds(it.second, state_name, ws_name)) {
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

/// Empilhamento no YAML da competicao. Em finish_state.<WS>:
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

// ---- formato 2026 (@Work Commander) -----------------------------------------
// Arquivo entregue pela ferramenta oficial (edicao 2026): objects so com {id}
// (a cor dos ATTCs NAO e' informada); containers [{color, at}] sem id; cada
// area do start/finish_state tem obj_ids E sub-blocos:
//   table      soltos sobre a mesa, esquerda -> direita (a ordem nao gera acao)
//   containers {COR: [ids]} dentro do container daquela cor (place por cor)
//   levels     {BOTTOM|TOP: [ids]} prateleira (pick/place normais na SH_*,
//              nivel so informativo: shelf_level na acao)
//   slots      {tag: id|null} mesa de precisao; como DESTINO vira place com
//              at_tag: tag_<slot>; como ORIGEM e' pick normal pela tag do
//              objeto; `slots: null` no start da AMT = arranjo nao informado
//   stack      [base, ..., topo] area de empilhamento sobre a tag de
//              stacking_areas.<ST>.reference_tag: a base e' solta com
//              stack_on: tag_<ref>, cada seguinte com stack_on: tag_<anterior>
// Blocos de topo: precision_tables {PP: {slot_tags: [...]}} (ordem dos slots,
// esquerda -> direita) e stacking_areas {ST: {reference_tag: N}}. As
// estacoes PP e ST vem SEM sufixo: o ws_table_mapping e' consultado por
// prefixo (resolveWsKey) e o executor faz o mesmo para o dock.

struct Format2026Info
{
  bool detected = false;
  std::map<int, std::string> start_level;     // obj -> BOTTOM|TOP na origem
  std::map<int, std::string> finish_level;    // obj -> BOTTOM|TOP no destino
  std::map<int, int> start_slot_tag;          // obj -> tag do slot na origem
  std::map<int, int> finish_slot_tag;         // obj -> tag do slot no destino
  std::map<int, int> base_ref_tag;            // base da pilha -> tag de referencia
  std::map<int, int> top_to_base;             // topo -> objeto logo abaixo
  std::set<int> already_stacked;              // ja na posicao certa da pilha
  std::map<int, std::vector<int>> pick_after;  // obj -> ids acima dele no start
  std::map<std::string, std::vector<int>> start_stacks;  // ST -> [base..topo] no start
  std::map<int, ContainerAssignment> container_assignments;
  std::map<std::string, std::vector<int>> slot_tags_by_ws;  // precision_tables
};

/// Formato 2026: blocos precision_tables/stacking_areas, container sem id ou
/// qualquer area com sub-bloco table/levels/slots/stack/containers(mapa).
bool isFormat2026(const YAML::Node & root)
{
  if (root["precision_tables"] || root["stacking_areas"]) {
    return true;
  }
  const YAML::Node containers = root["containers"];
  if (containers && containers.IsSequence()) {
    for (const auto & c : containers) {
      if (c.IsMap() && (!c["id"] || c["id"].IsNull())) {
        return true;
      }
    }
  }
  for (const char * state_name : {"start_state", "finish_state"}) {
    const YAML::Node state = root[state_name];
    if (!state || !state.IsMap()) {
      continue;
    }
    for (const auto & it : state) {
      const YAML::Node area = it.second;
      if (!area || !area.IsMap()) {
        continue;
      }
      if (area["table"] || area["levels"] || area["slots"] || area["stack"] ||
        (area["containers"] && area["containers"].IsMap()))
      {
        return true;
      }
    }
  }
  return false;
}

void parseLevels(
  const YAML::Node & area,
  const std::string & where,
  std::map<int, std::string> & out)
{
  const YAML::Node levels = area["levels"];
  if (!levels || levels.IsNull()) {
    return;
  }
  if (!levels.IsMap()) {
    throw std::runtime_error(where + ".levels must be a map {BOTTOM: [...], TOP: [...]}");
  }
  for (const auto & it : levels) {
    const std::string level = toUpper(trim(it.first.as<std::string>("")));
    if (level != "BOTTOM" && level != "TOP") {
      std::cerr << "[task_planner] AVISO: " << where << ".levels." << level
                << ": nivel desconhecido (esperado BOTTOM|TOP) — repassado como esta."
                << std::endl;
    }
    for (const int id : parseIdList(it.second, where + ".levels." + level)) {
      out[id] = level;
    }
  }
}

void parseSlots(
  const YAML::Node & area,
  const std::string & where,
  std::map<int, int> & out)
{
  const YAML::Node slots = area["slots"];
  if (!slots || slots.IsNull()) {
    return;   // AMT: arranjo inicial nao informado
  }
  if (!slots.IsMap()) {
    throw std::runtime_error(where + ".slots must be a map {tag: id|null} (or null)");
  }
  for (const auto & it : slots) {
    if (!it.second || it.second.IsNull()) {
      continue;
    }
    const int tag = parseObjectId(it.first);
    const int id = parseObjectId(it.second);
    if (out.count(id) > 0) {
      throw std::runtime_error(
              where + ".slots: objeto " + std::to_string(id) + " em dois slots");
    }
    out[id] = tag;
  }
}

Format2026Info parseFormat2026(
  const YAML::Node & root,
  const std::map<int, ContainerInfo> & containers,
  const std::map<int, std::string> & finish_index)
{
  Format2026Info info;
  info.detected = true;

  const YAML::Node pts = root["precision_tables"];
  if (pts && pts.IsMap()) {
    for (const auto & it : pts) {
      const std::string ws = trim(it.first.as<std::string>(""));
      info.slot_tags_by_ws[ws] =
        parseIdList(it.second["slot_tags"], "precision_tables." + ws + ".slot_tags");
    }
  }

  std::map<std::string, int> ref_tag_by_ws;
  const YAML::Node sas = root["stacking_areas"];
  if (sas && sas.IsMap()) {
    for (const auto & it : sas) {
      const std::string ws = trim(it.first.as<std::string>(""));
      const YAML::Node ref = it.second["reference_tag"];
      if (!ref || ref.IsNull()) {
        throw std::runtime_error("stacking_areas." + ws + ".reference_tag ausente");
      }
      ref_tag_by_ws[ws] = parseObjectId(ref);
    }
  }

  const YAML::Node start = root["start_state"];
  if (start && start.IsMap()) {
    for (const auto & area_it : start) {
      const std::string ws = trim(area_it.first.as<std::string>(""));
      const YAML::Node area = area_it.second;
      if (!area || !area.IsMap()) {
        continue;
      }
      const std::string where = "start_state." + ws;
      parseLevels(area, where, info.start_level);
      parseSlots(area, where, info.start_slot_tag);
      const YAML::Node cons = area["containers"];
      if (cons && cons.IsMap()) {
        for (const auto & c : cons) {
          for (const int id : parseIdList(c.second, where + ".containers")) {
            std::cerr << "[task_planner] AVISO: objeto " << id << " comeca DENTRO do container "
                      << toUpper(trim(c.first.as<std::string>(""))) << " em " << ws
                      << " — o pick nao sabe pegar de dentro de container; sera um pick "
                      << "normal pela tag." << std::endl;
          }
        }
      }
      // Pilha inicial: quem esta em cima sai antes de quem esta embaixo.
      const std::vector<int> st = parseIdList(area["stack"], where + ".stack");
      info.start_stacks[ws] = st;
      for (std::size_t j = 0; j < st.size(); ++j) {
        for (std::size_t k = j + 1; k < st.size(); ++k) {
          info.pick_after[st[j]].push_back(st[k]);
        }
      }
    }
  }

  const YAML::Node finish = root["finish_state"];
  if (finish && finish.IsMap()) {
    for (const auto & area_it : finish) {
      const std::string ws = trim(area_it.first.as<std::string>(""));
      const YAML::Node area = area_it.second;
      if (!area || !area.IsMap()) {
        continue;
      }
      const std::string where = "finish_state." + ws;
      parseLevels(area, where, info.finish_level);
      parseSlots(area, where, info.finish_slot_tag);

      // Containers por cor: finish_state.<WS>.containers.<COR> = [ids].
      const YAML::Node cons = area["containers"];
      if (cons && cons.IsMap()) {
        for (const auto & c : cons) {
          const std::string color_raw = trim(c.first.as<std::string>(""));
          const std::string color = canonicalColor(color_raw);
          const std::vector<int> ids = parseIdList(c.second, where + ".containers." + color_raw);
          if (ids.empty()) {
            continue;
          }
          if (color.empty()) {
            throw std::runtime_error(where + ".containers: cor '" + color_raw + "' desconhecida");
          }
          if (!colorHasContainer(color)) {
            throw std::runtime_error(
                    where + ".containers." + color_raw + ": o detector so conhece RED|BLUE");
          }
          int container_id = -1;
          bool declared = false;
          for (const auto & [cid, cinfo] : containers) {
            if (cinfo.color == color && cinfo.ws == ws) {
              container_id = cid;
              declared = true;
            }
          }
          if (!declared) {
            std::cerr << "[task_planner] AVISO: " << where << ".containers." << color
                      << " usado sem {color: " << color << ", at: " << ws
                      << "} no bloco containers — seguindo pela cor." << std::endl;
          }
          for (const int id : ids) {
            if (info.container_assignments.count(id) > 0) {
              throw std::runtime_error(
                      "Object id " + std::to_string(id) + " aparece em dois containers no finish_state");
            }
            ContainerAssignment a;
            a.container_id = container_id;
            a.container_color = color;
            a.container_ws = ws;
            info.container_assignments[id] = a;
          }
        }
      }

      // Pilha: [base, ..., topo] sobre a tag de referencia da area.
      const std::vector<int> fs = parseIdList(area["stack"], where + ".stack");
      if (fs.empty()) {
        continue;
      }
      const auto ref_it = ref_tag_by_ws.find(ws);
      if (ref_it == ref_tag_by_ws.end()) {
        throw std::runtime_error(
                where + ".stack sem stacking_areas." + ws + ".reference_tag — a base da pilha "
                "precisa da tag de referencia");
      }
      std::set<int> uniq;
      for (const int id : fs) {
        if (!uniq.insert(id).second) {
          throw std::runtime_error(where + ".stack: objeto " + std::to_string(id) + " repetido");
        }
        const auto fi = finish_index.find(id);
        if (fi == finish_index.end() || fi->second != ws) {
          throw std::runtime_error(
                  where + ".stack usa o objeto " + std::to_string(id) +
                  " que nao termina nessa estacao (obj_ids)");
        }
      }
      info.base_ref_tag[fs[0]] = ref_it->second;
      for (std::size_t i = 1; i < fs.size(); ++i) {
        info.top_to_base[fs[i]] = fs[i - 1];
      }
      // Prefixo igual ao da pilha inicial = ja no lugar, nao mexer.
      const auto ss_it = info.start_stacks.find(ws);
      if (ss_it != info.start_stacks.end()) {
        for (std::size_t i = 0; i < fs.size() && i < ss_it->second.size(); ++i) {
          if (ss_it->second[i] != fs[i]) {
            break;
          }
          info.already_stacked.insert(fs[i]);
        }
      }
    }
  }

  return info;
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
  const bool yaml_declares_containers,
  const Format2026Info & f26)
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
  for (const auto & [base, _] : f26.base_ref_tag) {
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

    // Formato 2026: objeto ja na posicao certa da pilha (mesmo prefixo no
    // start e no finish) nao e' topo nem base a mover.
    const bool already_stacked = f26.already_stacked.count(obj_id) > 0;
    const auto stack_it = stack_assignments.find(obj_id);
    const bool stacked_top = stack_it != stack_assignments.end() && !already_stacked;
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

    // Formato 2026: base da pilha sobre a tag de referencia da area (ST).
    const auto ref_it = f26.base_ref_tag.find(obj_id);
    const bool stacked_on_ref = ref_it != f26.base_ref_tag.end() && !already_stacked;
    if (stacked_on_ref) {
      const auto ref_frame_it = id_to_frame.find(ref_it->second);
      if (ref_frame_it == id_to_frame.end()) {
        ids_without_frame.push_back(ref_it->second);
      } else {
        item.stack_on_frame = ref_frame_it->second;   // stack_on_obj fica -1
      }
    }
    // Pilha INICIAL (2026) ocupando o lugar do place: base sobre a tag de
    // referencia espera TODOS os cubos da pilha inicial sairem; topo sobre a
    // base B espera os que estao acima de B. Quem ja esta no lugar certo
    // (already_stacked) nao sai e nao bloqueia.
    if (stacked_on_ref || stacked_top) {
      const auto ss_it = f26.start_stacks.find(item.to_ws);
      if (ss_it != f26.start_stacks.end()) {
        const std::vector<int> & ss = ss_it->second;
        std::size_t first_above = 0;
        if (stacked_top) {
          const auto base_pos = std::find(ss.begin(), ss.end(), item.stack_on_obj);
          first_above = base_pos == ss.end() ?
            ss.size() : static_cast<std::size_t>(base_pos - ss.begin()) + 1;
        }
        for (std::size_t i = first_above; i < ss.size(); ++i) {
          if (ss[i] != obj_id && f26.already_stacked.count(ss[i]) == 0) {
            item.place_after_picked.push_back(ss[i]);
          }
        }
      }
    }
    // Formato 2026: prateleira (nivel informativo) e mesa de precisao (slot).
    if (const auto it = f26.start_level.find(obj_id); it != f26.start_level.end()) {
      item.from_level = it->second;
    }
    if (const auto it = f26.finish_level.find(obj_id); it != f26.finish_level.end()) {
      item.to_level = it->second;
    }
    if (const auto it = f26.start_slot_tag.find(obj_id); it != f26.start_slot_tag.end()) {
      item.from_slot_tag = it->second;
    }
    if (const auto it = f26.finish_slot_tag.find(obj_id); it != f26.finish_slot_tag.end()) {
      item.to_slot_tag = it->second;
      if (assigned_to_container) {
        throw std::runtime_error(
                "Object id " + std::to_string(obj_id) +
                " termina num slot da mesa de precisao E num container — contradicao");
      }
      if (stacked_top || stacked_on_ref) {
        throw std::runtime_error(
                "Object id " + std::to_string(obj_id) +
                " termina num slot da mesa de precisao E numa pilha — contradicao");
      }
      const auto slot_frame_it = id_to_frame.find(item.to_slot_tag);
      if (slot_frame_it == id_to_frame.end()) {
        ids_without_frame.push_back(item.to_slot_tag);
      } else {
        item.at_tag_frame = slot_frame_it->second;
      }
    }
    if (const auto it = f26.pick_after.find(obj_id); it != f26.pick_after.end()) {
      item.pick_after = it->second;
    }

    // Mesma estacao mas outro nivel da prateleira ou outro slot da mesa de
    // precisao: tambem e' pick + place.
    const bool level_changed = has_start && has_finish && !moved && !item.to_level.empty() &&
      item.from_level != item.to_level;
    const bool slot_changed = has_start && has_finish && !moved && item.to_slot_tag >= 0 &&
      item.from_slot_tag != item.to_slot_tag;

    // Topo de pilha sempre e' pego e solto (mesmo se comeca e termina na
    // mesma estacao: precisa subir em cima da base). A base sobre a tag de
    // referencia (2026) idem: precisa ser solta centrada na tag.
    const bool stacked = stacked_top || stacked_on_ref;
    item.needs_pick = moved || assigned_to_container || (stacked && has_start) ||
      level_changed || slot_changed;
    item.needs_place = moved || assigned_to_container || stacked || level_changed ||
      slot_changed;

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
    throwIdsWithoutFrame(ids_without_frame);
  }

  std::sort(
    transfers.begin(),
    transfers.end(),
    [](const TransferItem & a, const TransferItem & b) { return a.obj_id < b.obj_id; });

  // Pilha inicial (2026): quem esta em cima precisa ser pego — senao o de
  // baixo ficaria preso para sempre (o planejador travaria sem progresso).
  // Idem para quem ocupa a tag de referencia/base onde outro sera solto.
  const auto will_be_picked = [&transfers](const int id) {
      const auto it = std::find_if(
        transfers.begin(), transfers.end(),
        [id](const TransferItem & x) { return x.obj_id == id; });
      return it != transfers.end() && it->needs_pick;
    };
  for (TransferItem & t : transfers) {
    if (!t.needs_pick) {
      t.pick_after.clear();
    }
    for (const int above : t.pick_after) {
      if (!will_be_picked(above)) {
        throw std::runtime_error(
                "Objeto " + std::to_string(t.obj_id) + " esta embaixo do " +
                std::to_string(above) + " na pilha inicial de " + t.from_ws + ", mas o " +
                std::to_string(above) + " nao sera pego — impossivel pegar o de baixo");
      }
    }
    if (!t.needs_place) {
      t.place_after_picked.clear();
    }
    for (const int occupant : t.place_after_picked) {
      if (!will_be_picked(occupant)) {
        throw std::runtime_error(
                "Objeto " + std::to_string(t.obj_id) + " seria solto sobre " + t.stack_on_frame +
                " em " + t.to_ws + ", mas o " + std::to_string(occupant) +
                " da pilha inicial ocupa esse lugar e nao sera pego — impossivel soltar");
      }
    }
  }

  return transfers;
}

// ---- ordem das estacoes: guloso ou busca por tempo ---------------------------
// Com dock_distances.yaml na pasta do mapa (ao lado do ws_table_mapping.yaml,
// gerado por `ros2 run caramelo_navigation compute_dock_distances`), a ordem
// das visitas sai de um branch-and-bound que minimiza o tempo estimado
// (task_plan::planOptimized); sem o arquivo, o guloso de sempre
// (task_plan::planGreedy), byte a byte. Ver docs/planejador_por_tempo.md.
//
// Ajustes por variavel de ambiente (defaults da run de 2026-10-21):
//   CARAMELO_PLANNER_MODE          auto (default) | greedy (forca o guloso)
//   CARAMELO_PLANNER_V_NAV_MPS     velocidade media de navegacao (0.25)
//   CARAMELO_PLANNER_DOCKING_S     custo fixo por troca de estacao (25)
//   CARAMELO_PLANNER_PICK_S        custo por pick (35)
//   CARAMELO_PLANNER_PLACE_S       custo por place (35)
//   CARAMELO_PLANNER_TIME_LIMIT_S  prazo da busca (2.0); estourou = melhor achado
//   CARAMELO_PLANNER_SEED          semente do sorteio de empates do guloso

struct PlannerOptions
{
  std::string mode{"auto"};
  double time_limit_s{2.0};
  std::optional<unsigned int> seed;
  manip_bt::task_plan::CostModel model;
};

double envDouble(const char * name, const double fallback)
{
  const char * raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0') {
    return fallback;
  }
  try {
    const double value = std::stod(raw);
    if (!(value > 0.0)) {
      throw std::invalid_argument("<= 0");
    }
    return value;
  } catch (const std::exception &) {
    std::cerr << "[task_planner] AVISO: " << name << "='" << raw
              << "' invalido - usando " << fallback << "." << std::endl;
    return fallback;
  }
}

PlannerOptions plannerOptionsFromEnvironment()
{
  PlannerOptions opts;
  if (const char * mode = std::getenv("CARAMELO_PLANNER_MODE")) {
    const std::string m = toLower(trim(mode));
    if (m == "greedy" || m == "guloso") {
      opts.mode = "greedy";
    } else if (!m.empty() && m != "auto") {
      std::cerr << "[task_planner] AVISO: CARAMELO_PLANNER_MODE='" << mode
                << "' desconhecido (auto|greedy) - usando auto." << std::endl;
    }
  }
  opts.model.v_nav_mps = envDouble("CARAMELO_PLANNER_V_NAV_MPS", opts.model.v_nav_mps);
  opts.model.docking_s = envDouble("CARAMELO_PLANNER_DOCKING_S", opts.model.docking_s);
  opts.model.pick_s = envDouble("CARAMELO_PLANNER_PICK_S", opts.model.pick_s);
  opts.model.place_s = envDouble("CARAMELO_PLANNER_PLACE_S", opts.model.place_s);
  opts.time_limit_s = envDouble("CARAMELO_PLANNER_TIME_LIMIT_S", opts.time_limit_s);
  if (const char * seed = std::getenv("CARAMELO_PLANNER_SEED")) {
    try {
      opts.seed = static_cast<unsigned int>(std::stoul(seed));
    } catch (const std::exception &) {
      std::cerr << "[task_planner] AVISO: CARAMELO_PLANNER_SEED='" << seed
                << "' invalido - sorteio aleatorio." << std::endl;
    }
  }
  return opts;
}

/// Preenche model.dist_m (chaves = estacoes da task + START/FINISH) a partir
/// do dock_distances.yaml da pasta do ws_table_mapping.yaml. false = sem
/// matriz (ou mapa inconsistente): planejador guloso, como sempre.
bool loadDistancesFromMap(
  const std::string & ws_map_yaml_path,
  const std::vector<TransferItem> & transfers,
  manip_bt::task_plan::CostModel & model)
{
  const std::filesystem::path map_folder = std::filesystem::path(ws_map_yaml_path).parent_path();
  if (!std::filesystem::exists(map_folder / "dock_distances.yaml")) {
    std::cout << "[task_planner] sem dock_distances.yaml em " << map_folder.string()
              << ": ordem gulosa (gere com: ros2 run caramelo_navigation "
              << "compute_dock_distances --map-name " << map_folder.filename().string() << ")."
              << std::endl;
    return false;
  }
  std::unique_ptr<manip_bt::MissionMapConfig> cfg;
  try {
    cfg = std::make_unique<manip_bt::MissionMapConfig>(map_folder.string());
  } catch (const std::exception & ex) {
    std::cerr << "[task_planner] AVISO: nao li a pasta do mapa (" << ex.what()
              << ") - ordem gulosa." << std::endl;
    return false;
  }
  if (!cfg->hasDockDistances()) {
    std::cerr << "[task_planner] AVISO: dock_distances.yaml sem distancias - ordem gulosa."
              << std::endl;
    return false;
  }

  std::set<std::string> names = manip_bt::task_plan::candidateStations(transfers);
  names.insert(model.start_ws);
  names.insert(model.finish_ws);
  std::map<std::string, std::string> dock_of;
  for (const auto & ws : names) {
    try {
      dock_of[ws] = (ws == model.start_ws || ws == model.finish_ws) ?
        manip_bt::MissionMapConfig::normalizeDockId(ws) : cfg->dockIdForWs(ws);
    } catch (const std::exception & ex) {
      std::cerr << "[task_planner] AVISO: " << ex.what() << " - ordem gulosa." << std::endl;
      return false;
    }
  }

  std::set<std::string> unknown_docks;
  std::vector<std::pair<std::string, std::string>> unknown_pairs;
  double known_sum = 0.0;
  int known_count = 0;
  for (const auto & a : names) {
    for (const auto & b : names) {
      if (a >= b) {
        continue;
      }
      const auto d = cfg->dockDistance(dock_of[a], dock_of[b]);
      if (d.has_value()) {
        model.dist_m[a][b] = *d;
        known_sum += *d;
        ++known_count;
      } else {
        unknown_pairs.emplace_back(a, b);
      }
    }
  }
  if (!unknown_pairs.empty()) {
    if (known_count == 0) {
      std::cerr << "[task_planner] AVISO: nenhuma distancia conhecida entre as estacoes da task "
                << "- ordem gulosa." << std::endl;
      return false;
    }
    // Dock sem pose gravada (ex.: ST1 antes de calibrar): media das distancias
    // conhecidas, para a busca ainda minimizar o resto. Avisa para gravar.
    const double fallback = known_sum / known_count;
    for (const auto & [a, b] : unknown_pairs) {
      model.dist_m[a][b] = fallback;
      for (const auto & ws : {a, b}) {
        if (!cfg->hasDock(dock_of[ws]) || cfg->dock(dock_of[ws])->pose_is_placeholder) {
          unknown_docks.insert(ws + " (" + dock_of[ws] + ")");
        }
      }
    }
    std::string joined;
    for (const auto & d : unknown_docks) {
      joined += (joined.empty() ? "" : ", ") + d;
    }
    std::cerr << "[task_planner] AVISO: " << unknown_pairs.size() << " par(es) sem distancia"
              << (joined.empty() ? "" : " - dock sem pose gravada: " + joined)
              << " - usando a media das conhecidas (" << fallback
              << " m); grave a pose e regere o dock_distances.yaml." << std::endl;
  }
  std::cout << "[task_planner] distancias: " << cfg->dockDistancesPath() << " ("
            << cfg->dockDistancesMethod() << ")." << std::endl;
  return true;
}

std::string fmtSeconds(const double s)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(0);
  out << s << " s";
  return out.str();
}

std::string fmtStats(const manip_bt::task_plan::PlanStats & st)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(1);
  out << st.visits << " visita(s), " << st.station_switches << " troca(s), "
      << st.distance_m << " m";
  return out.str();
}

YAML::Node buildOutput(
  const std::vector<TransferItem> & transfers,
  const std::map<std::string, std::string> & ws_to_table_pose,
  const std::string & ws_map_yaml_path)
{
  namespace tp = manip_bt::task_plan;
  PlannerOptions opts = plannerOptionsFromEnvironment();
  std::mt19937 rng(opts.seed ? *opts.seed : std::random_device{}());

  // Guloso de sempre (e' o plano final sem matriz, e o limite inicial com ela).
  std::vector<tp::Visit> visits = tp::planGreedy(transfers, rng);
  YAML::Node estimate;
  estimate["mode"] = "greedy";

  const bool has_model = !transfers.empty() &&
    loadDistancesFromMap(ws_map_yaml_path, transfers, opts.model);
  if (has_model) {
    tp::PlanStats greedy_stats;
    const double greedy_time = tp::estimatePlanTime(visits, opts.model, &greedy_stats);
    std::ostringstream line;
    line << "[task_planner] modelo: v_nav " << opts.model.v_nav_mps << " m/s, docking "
         << opts.model.docking_s << " s, pick " << opts.model.pick_s << " s, place "
         << opts.model.place_s << " s. Tempo estimado guloso: " << fmtSeconds(greedy_time)
         << " (" << fmtStats(greedy_stats) << ")";
    estimate["greedy_time_s"] = greedy_time;
    if (opts.mode == "greedy") {
      line << " - CARAMELO_PLANNER_MODE=greedy: sem busca.";
    } else {
      const tp::SearchResult result =
        tp::planOptimized(transfers, opts.model, visits, opts.time_limit_s);
      tp::PlanStats best_stats;
      tp::estimatePlanTime(result.visits, opts.model, &best_stats);
      line.setf(std::ios::fixed);
      line.precision(0);
      line << "; otimizado: " << fmtSeconds(result.time_s) << " (" << fmtStats(best_stats)
           << ")" << (result.improved ? "" : " = guloso") << " ["
           << (result.optimal ? "busca completa" : "prazo estourado, melhor encontrado")
           << ", " << result.nodes << " nos, " << result.elapsed_ms << " ms]";
      visits = result.visits;
      estimate["mode"] = "optimized";
      estimate["optimal"] = result.optimal;
      estimate["search_nodes"] = static_cast<unsigned long long>(result.nodes);
    }
    std::cout << line.str() << std::endl;
    tp::PlanStats final_stats;
    estimate["time_s"] = tp::estimatePlanTime(visits, opts.model, &final_stats);
    estimate["visits"] = final_stats.visits;
    estimate["station_switches"] = final_stats.station_switches;
    estimate["distance_m"] = final_stats.distance_m;
  }

  YAML::Node output;
  output["actions"] = tp::emitActions(visits, transfers, ws_to_table_pose);
  if (has_model) {
    output["plan_estimate"] = estimate;   // so informativo (executor le apenas actions)
  }
  return output;
}

// ---- AMT1 ------------------------------------------------------------------
struct Amt1Station
{
  std::string ws;                 // "PP_1" (ou "PP" no formato 2026)
  std::vector<int> expected_tags; // ids em ordem crescente (sem decoy)
  // Formato 2026: alvo EXPLICITO, esquerda -> direita, derivado de
  // finish_state.<PP>.slots na ordem de precision_tables.<PP>.slot_tags.
  // Vazio = ordem crescente de id (comportamento antigo).
  std::vector<int> target_order;
};

bool isPpStation(const std::string & ws)
{
  return ws.rfind("PP", 0) == 0;
}

/// AMT1 disfarcada: no fluxo antigo, TODOS os objetos nao-decoy
/// do start_state comecam e terminam na MESMA estacao PP_* (e ha pelo menos
/// um). Com 0 transferencias isso e a cara da AMT1 com task_id/name fora do
/// padrao — main so avisa, nao muda o fluxo.
bool looksLikeAmt1(
  const std::map<int, std::string> & start_index,
  const std::map<int, std::string> & finish_index,
  const std::set<int> & ignored_object_ids)
{
  std::size_t counted = 0;
  for (const auto & [obj_id, from_ws] : start_index) {
    if (ignored_object_ids.count(obj_id) > 0) {
      continue;
    }
    const auto to_it = finish_index.find(obj_id);
    if (to_it == finish_index.end() || to_it->second != from_ws || !isPpStation(from_ws)) {
      return false;
    }
    ++counted;
  }
  return counted > 0;
}

/// Estacao da AMT1: a PP_* do start_state com objetos (empate: a que tiver
/// MAIS objetos). Se a PP com objetos existe mas NAO esta em
/// active_service_areas -> erro claro (nada de redirecionar em
/// silencio para outra PP_* ativa). So sem NENHUMA PP_* com objetos no
/// start_state cai no fallback: a 1a PP_* de active_service_areas (na ordem
/// do yaml) com expected_tags = todos os objetos nao-decoy do bloco
/// `objects:` (vazio = o no ordena tudo o que vir). A PP precisa estar em
/// active_service_areas — o indice de estado descarta estacoes fora dela.
Amt1Station resolveAmt1Station(
  const YAML::Node & root,
  const std::set<std::string> & active_areas,
  const std::map<int, ObjectInfo> & objects,
  const std::set<int> & ignored_object_ids,
  const std::map<int, std::string> & id_to_frame)
{
  Amt1Station station;
  std::vector<std::string> inactive_pp;

  const YAML::Node start_state = root["start_state"];
  if (start_state && start_state.IsMap()) {
    std::size_t best_count = 0;
    for (const auto & it : start_state) {
      const std::string ws_name = trim(it.first.as<std::string>());
      if (!isPpStation(ws_name)) {
        continue;
      }
      const YAML::Node obj_ids = it.second["obj_ids"];
      std::vector<int> ids;
      if (obj_ids && obj_ids.IsSequence()) {
        for (const auto & id_node : obj_ids) {
          const int id = parseObjectId(id_node);
          if (ignored_object_ids.count(id) == 0) {
            ids.push_back(id);
          }
        }
      }
      if (ids.empty()) {
        continue;
      }
      if (active_areas.count(ws_name) == 0) {
        inactive_pp.push_back(ws_name);
        continue;
      }
      if (ids.size() > best_count) {
        best_count = ids.size();
        station.ws = ws_name;
        station.expected_tags = ids;
      }
    }
  }

  if (station.ws.empty() && !inactive_pp.empty()) {
    // A PP com os cubos existe mas esta fora de
    // active_service_areas. Erro AQUI, antes do fallback — senao a rotina
    // rodaria em silencio numa PP_* ativa que nao tem os cubos.
    std::string hint;
    for (const auto & ws_name : inactive_pp) {
      hint += (hint.empty() ? "" : ", ") + ws_name;
    }
    throw std::runtime_error(
            "AMT1: start_state tem objetos em " + hint +
            ", mas ela nao esta ativa — a mesa de precisao com os cubos precisa estar em "
            "active_service_areas (nao vou redirecionar para outra PP_*)");
  }

  if (station.ws.empty()) {
    // Fallback (so sem NENHUMA PP_* com objetos no start_state): 1a PP_*
    // ativa, na ordem do yaml.
    const YAML::Node areas = root["active_service_areas"];
    if (areas && areas.IsSequence()) {
      for (const auto & area : areas) {
        const std::string ws_name = area.IsMap() ?
          trim(area["id"].as<std::string>("")) : trim(area.as<std::string>(""));
        if (isPpStation(ws_name) && active_areas.count(ws_name) > 0) {
          station.ws = ws_name;
          break;
        }
      }
    }
    if (station.ws.empty()) {
      throw std::runtime_error(
              "AMT1: nenhuma estacao PP_* em active_service_areas — a mesa de precisao "
              "precisa estar em active_service_areas");
    }
    for (const auto & [obj_id, _] : objects) {
      station.expected_tags.push_back(obj_id);
    }
    std::cerr << "[task_planner] AVISO: AMT1 sem objetos em start_state." << station.ws
              << " — usando os " << station.expected_tags.size()
              << " objeto(s) do bloco objects." << std::endl;
  }

  std::sort(station.expected_tags.begin(), station.expected_tags.end());
  station.expected_tags.erase(
    std::unique(station.expected_tags.begin(), station.expected_tags.end()),
    station.expected_tags.end());

  std::vector<int> ids_without_frame;
  for (const int id : station.expected_tags) {
    if (id_to_frame.count(id) == 0) {
      ids_without_frame.push_back(id);
    }
  }
  if (!ids_without_frame.empty()) {
    throwIdsWithoutFrame(ids_without_frame);
  }
  return station;
}

/// AMT (formato 2026): alvo explicito por slot. finish_state.<PP>.slots
/// {tag: id|null} lido na ordem de precision_tables.<PP>.slot_tags (esquerda
/// -> direita), nulls pulados -> target_order. Vazio quando o yaml nao traz
/// `slots` como mapa (formatos antigos: ordem crescente). Valida que o alvo
/// e' exatamente o conjunto de expected_tags.
std::vector<int> deriveAmtTargetOrder(const YAML::Node & root, const Amt1Station & station)
{
  std::vector<int> order;
  const YAML::Node finish = root["finish_state"];
  if (!finish || !finish.IsMap()) {
    return order;
  }
  const YAML::Node area = finish[station.ws];
  if (!area || !area.IsMap()) {
    return order;
  }
  const YAML::Node slots = area["slots"];
  if (!slots || !slots.IsMap()) {
    return order;
  }
  const std::string where = "finish_state." + station.ws + ".slots";

  std::map<int, int> id_by_tag;
  std::vector<int> yaml_tag_order;
  for (const auto & it : slots) {
    const int tag = parseObjectId(it.first);
    yaml_tag_order.push_back(tag);
    if (it.second && !it.second.IsNull()) {
      id_by_tag[tag] = parseObjectId(it.second);
    }
  }

  std::vector<int> slot_tags;
  const YAML::Node pts = root["precision_tables"];
  if (pts && pts.IsMap() && pts[station.ws] && pts[station.ws].IsMap()) {
    slot_tags = parseIdList(
      pts[station.ws]["slot_tags"], "precision_tables." + station.ws + ".slot_tags");
  }
  if (slot_tags.empty()) {
    std::cerr << "[task_planner] AVISO: precision_tables." << station.ws
              << ".slot_tags ausente — usando a ordem do bloco " << where << "." << std::endl;
    slot_tags = yaml_tag_order;
  }
  for (const auto & [tag, _] : id_by_tag) {
    if (std::find(slot_tags.begin(), slot_tags.end(), tag) == slot_tags.end()) {
      throw std::runtime_error(
              where + ": slot " + std::to_string(tag) + " nao esta em precision_tables." +
              station.ws + ".slot_tags");
    }
  }
  for (const int tag : slot_tags) {
    const auto it = id_by_tag.find(tag);
    if (it != id_by_tag.end()) {
      order.push_back(it->second);
    }
  }

  const std::set<int> expected(station.expected_tags.begin(), station.expected_tags.end());
  for (const int id : order) {
    if (expected.count(id) == 0) {
      throw std::runtime_error(
              where + ": objeto " + std::to_string(id) + " nao esta em start_state." +
              station.ws + ".obj_ids");
    }
  }
  for (const int id : expected) {
    if (std::find(order.begin(), order.end(), id) == order.end()) {
      throw std::runtime_error(
              where + " nao da slot ao objeto " + std::to_string(id) + " (esperado na mesa)");
    }
  }
  return order;
}

/// AMT1: goto PP_x (sem `home` antes — o prologo do executor cobre) +
/// UMA acao amt1_sort; a rotina inteira roda no amt1_sort_action_node.
YAML::Node buildAmt1Output(
  const Amt1Station & station,
  const std::map<std::string, std::string> & ws_to_table_pose)
{
  YAML::Node output;
  YAML::Node action_seq(YAML::NodeType::Sequence);
  std::string current_ws;
  manip_bt::task_plan::appendGoto(action_seq, current_ws, station.ws, ws_to_table_pose);

  YAML::Node sort;
  sort["kind"] = "amt1_sort";
  sort["ws"] = station.ws;
  sort["table_pose"] = tablePoseForWs(ws_to_table_pose, station.ws);
  YAML::Node tags(YAML::NodeType::Sequence);
  for (const int id : station.expected_tags) {
    tags.push_back(id);
  }
  sort["expected_tags"] = tags;
  sort["direction"] = "left_to_right";
  sort["max_onboard"] = kMaxOnboard;
  if (!station.target_order.empty()) {
    YAML::Node order(YAML::NodeType::Sequence);
    for (const int id : station.target_order) {
      order.push_back(id);
    }
    sort["target_order"] = order;   // alvo explicito (2026); vazio = crescente
  }
  action_seq.push_back(sort);

  output["actions"] = action_seq;
  return output;
}

void writeOutputYaml(const YAML::Node & output, const std::string & output_yaml_path)
{
  YAML::Emitter out;
  out << output;

  std::ofstream fout(output_yaml_path);
  if (!fout.is_open()) {
    throw std::runtime_error("Could not open output file: " + output_yaml_path);
  }
  fout << out.c_str() << std::endl;

  std::cout << "Wrote translation to " << output_yaml_path << std::endl;
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

// Caminho de entrada obrigatorio: sem fallback relativo nem por share (os
// chamadores passam caminhos absolutos).
std::string requireExistingFile(const std::string & path, const char * what)
{
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error(std::string("arquivo de ") + what + " nao existe: " + path);
  }
  return path;
}

void printUsage()
{
  std::cerr
    << "Uso: task_planner <competicao.yaml> <saida_actions.yaml> <tags_36h11.yaml>"
    << " <ws_table_mapping.yaml>\n"
    << "  competicao.yaml      YAML da task (missions/tasks geradas/*.yaml ou missions/task_*.yaml)\n"
    << "  saida_actions.yaml   lista de acoes gerada para o bt_yaml_executor\n"
    << "  tags_36h11.yaml      ids das AprilTags (manip_bringup/config/tags_36h11.yaml)\n"
    << "  ws_table_mapping.yaml  WS -> pose de mesa, da pasta do mapa\n"
    << "Os quatro argumentos sao obrigatorios." << std::endl;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5) {
    printUsage();
    return 2;
  }

  const std::string output_yaml_path = argv[2];

  try {
    const std::string competition_yaml_path = requireExistingFile(argv[1], "competicao");
    const std::string apriltag_yaml_path = requireExistingFile(argv[3], "tags");
    const std::string ws_map_yaml_path = requireExistingFile(argv[4], "ws_table_mapping");
    const auto ws_to_table_pose = parseWsToTablePose(ws_map_yaml_path);

    const YAML::Node competition_root = YAML::LoadFile(competition_yaml_path);
    checkSchemaVersion(competition_root);
    const auto active_areas = parseActiveAreas(competition_root);
    std::set<int> ignored_object_ids;
    const auto objects = parseObjects(competition_root, ignored_object_ids);
    const auto containers = parseContainers(competition_root);
    const auto start_index = buildStateIndex(competition_root["start_state"], active_areas, "start_state");
    const auto finish_index = buildStateIndex(competition_root["finish_state"], active_areas, "finish_state");
    const auto id_to_frame = parseApriltagIdToFrame(apriltag_yaml_path);

    // Formato 2026 do @Work Commander (principal): sub-blocos por area. Os
    // formatos anteriores (constraints / "novo 25/08") seguem o caminho de
    // sempre.
    const bool format_2026 = isFormat2026(competition_root);
    Format2026Info f26;
    if (format_2026) {
      std::cout << "[task_planner] formato 2026 do @Work Commander detectado." << std::endl;
      f26 = parseFormat2026(competition_root, containers, finish_index);
    }
    const auto container_assignments = format_2026 ?
      f26.container_assignments : parseContainerAssignments(competition_root, containers);

    // AMT1: ordenar as tags da mesa de precisao. Sai aqui: o
    // fluxo de transferencias abaixo descartaria os objetos (mesma estacao
    // no inicio e no fim) e geraria 0 acoes.
    if (isAmt1Task(competition_root)) {
      Amt1Station station = resolveAmt1Station(
        competition_root, active_areas, objects, ignored_object_ids, id_to_frame);
      station.target_order = deriveAmtTargetOrder(competition_root, station);
      const auto output = buildAmt1Output(station, ws_to_table_pose);
      std::string target_txt;
      for (const int id : station.target_order) {
        target_txt += (target_txt.empty() ? "" : ",") + std::to_string(id);
      }
      std::cout << "[task_planner] AMT1: ordenar " << station.expected_tags.size()
                << " tags na " << station.ws << " ("
                << tablePoseForWs(ws_to_table_pose, station.ws)
                << ", left_to_right, max 3 a bordo)"
                << (target_txt.empty() ? "" : " — alvo explicito por slot [" + target_txt + "]")
                << std::endl;
      writeOutputYaml(output, output_yaml_path);
      return 0;
    }

    std::map<int, int> stack_assignments = format_2026 ?
      f26.top_to_base : parseStackAssignments(competition_root, finish_index);
    const auto transfers =
      buildTransfers(
        objects,
        start_index,
        finish_index,
        id_to_frame,
        ignored_object_ids,
        container_assignments,
        stack_assignments,
        !containers.empty(),
        f26);
    const auto output = buildOutput(transfers, ws_to_table_pose, ws_map_yaml_path);

    // AMT1 disfarcada: 0 transferencias e todos os objetos
    // comecando e terminando na mesma PP_*. So avisa (o fluxo nao muda):
    // o operador confere o task_id/name do yaml antes de rodar.
    if (transfers.empty() && looksLikeAmt1(start_index, finish_index, ignored_object_ids)) {
      std::cerr << "[task_planner] AVISO: parece AMT1 - task_id/name nao bateram (task_id="
                << trim(competition_root["task_id"].as<std::string>("")) << ", name="
                << trim(competition_root["name"].as<std::string>(""))
                << "): todos os objetos comecam e terminam na mesma PP_* e nada sera "
                << "transportado. Para a rotina de ordenacao use task_id AMT1 ou name "
                << "'Advanced Manipulation Task 1'." << std::endl;
    }

    // Resumo para o operador (run_mission.py repassa o stdout antes de confirmar).
    {
      std::size_t n_container = 0;
      std::size_t n_stack = 0;
      std::size_t n_slot = 0;
      std::size_t n_shelf = 0;
      std::size_t n_wait_occupant = 0;
      std::string container_list;
      std::string slot_list;
      for (const auto & t : transfers) {
        if (!t.place_after_picked.empty()) {
          ++n_wait_occupant;
        }
        if (!t.container_color.empty()) {
          ++n_container;
          container_list += (container_list.empty() ? "" : ", ") + t.tag_frame + "->" +
            t.container_color;
        }
        if (!t.stack_on_frame.empty()) {
          ++n_stack;
        }
        if (!t.at_tag_frame.empty()) {
          ++n_slot;
          slot_list += (slot_list.empty() ? "" : ", ") + t.tag_frame + "->" + t.at_tag_frame;
        }
        if (!t.from_level.empty() || !t.to_level.empty()) {
          ++n_shelf;
        }
      }
      std::cout << "[task_planner] " << transfers.size() << " objeto(s) a transportar; "
                << ignored_object_ids.size() << " decoy(s) ignorado(s); "
                << n_container << " place(s) em container por cor"
                << (container_list.empty() ? "" : " (" + container_list + ")") << "; "
                << n_stack << " empilhamento(s)";
      if (format_2026) {
        std::cout << "; " << n_slot << " place(s) em slot da mesa de precisao"
                  << (slot_list.empty() ? "" : " (" + slot_list + ")") << "; "
                  << n_shelf << " objeto(s) na prateleira";
        if (n_wait_occupant > 0) {
          std::cout << "; " << n_wait_occupant
                    << " place(s) esperam a pilha inicial sair da tag/base";
        }
      }
      std::cout << "." << std::endl;
    }

    writeOutputYaml(output, output_yaml_path);
  } catch (const std::exception & e) {
    std::cerr << "Translation failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
