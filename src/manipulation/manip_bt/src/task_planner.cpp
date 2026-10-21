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

struct TransferItem
{
  int obj_id = -1;
  std::string tag_frame;
  std::string from_ws;
  std::string to_ws;
  // Cor do container da mesa onde soltar (RED|BLUE); vazia = mesa. Vem da
  // constraint "inside" (formato antigo) ou da cor do proprio
  // objeto (formato novo: se houver container dessa cor visivel, o place usa).
  std::string container_color;
  bool needs_pick = false;
  bool needs_place = false;
  // Empilhar: este objeto e' solto EM CIMA do objeto `stack_on_obj`
  // (frame `stack_on_frame`), que precisa estar na mesa de `to_ws` antes.
  // Formato 2026: a BASE da pilha e' solta sobre a tag de referencia da area
  // de empilhamento (stack_on_frame = tag_200, stack_on_obj = -1: a tag esta
  // sempre na mesa, nada a esperar).
  int stack_on_obj = -1;
  std::string stack_on_frame;
  // Formato 2026 — prateleira: nivel (BOTTOM|TOP) de origem/destino, so
  // informativo nas acoes (shelf_level); vazio fora da prateleira.
  std::string from_level;
  std::string to_level;
  // Formato 2026 — mesa de precisao como DESTINO: tag do slot onde soltar
  // (place com at_tag: tag_<slot>). -1 = nao e' slot.
  int to_slot_tag = -1;
  int from_slot_tag = -1;
  std::string at_tag_frame;
  // Formato 2026 — pilha INICIAL (start_state.<ST>.stack): ids que estao em
  // cima deste objeto e precisam ser pegos antes dele.
  std::vector<int> pick_after;
  // Formato 2026 — pilha INICIAL ocupando o lugar onde este objeto sera
  // solto (a tag de referencia, se e' base; o que esta em cima da base, se
  // e' topo): esses ids precisam ser pegos ANTES do place deste objeto.
  std::vector<int> place_after_picked;
};

constexpr int kMaxOnboard = 3;

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

/// Pilha inicial (2026): o objeto so pode ser pego depois dos que estao em
/// cima dele.
bool pickAfterSatisfied(const PlannerState & state, const TransferItem & t)
{
  for (const int above : t.pick_after) {
    if (state.picked.count(above) == 0) {
      return false;
    }
  }
  return true;
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

// ---- pilhas ----------------------------------------------------------------
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

/// Pilha inicial (2026): o place sobre a tag de referencia/base so pode
/// acontecer depois que os ocupantes desse lugar forem pegos.
bool placeAfterPickedSatisfied(const PlannerState & state, const TransferItem & t)
{
  for (const int occupant : t.place_after_picked) {
    if (state.picked.count(occupant) == 0) {
      return false;
    }
  }
  return true;
}

/// Pode ser solto AGORA: ocupantes ja pegos e (se topo) base ja na mesa.
bool placeReady(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t)
{
  return placeAfterPickedSatisfied(state, t) &&
         (t.stack_on_obj < 0 || baseOnTable(transfers, state, t.stack_on_obj));
}

/// Vai poder ser solto ao CHEGAR na estacao: direto, ou topo cuja base
/// tambem a bordo sera solta antes (recursivo na cadeia de bases a bordo).
bool placeableOnArrival(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t)
{
  if (!placeAfterPickedSatisfied(state, t)) {
    return false;
  }
  if (t.stack_on_obj < 0 || baseOnTable(transfers, state, t.stack_on_obj)) {
    return true;
  }
  if (baseCarried(state, t.stack_on_obj)) {
    return placeableOnArrival(transfers, state, findTransferById(transfers, t.stack_on_obj));
  }
  return false;
}

/// Ocupantes da pilha inicial ainda nao pegos que bloqueiam o place de `t`
/// (ou da base que `t` espera, se ela tambem esta a bordo). `self_id` conta
/// como ja pego — e' o proprio objeto cuja pegada esta sendo decidida.
std::set<int> unpickedBlockers(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t,
  const int self_id)
{
  std::set<int> blockers;
  const TransferItem * cur = &t;
  // Cadeia de bases a bordo tem no maximo kMaxOnboard elos.
  for (int depth = 0; cur != nullptr && depth <= kMaxOnboard; ++depth) {
    for (const int occupant : cur->place_after_picked) {
      if (occupant != self_id && state.picked.count(occupant) == 0) {
        blockers.insert(occupant);
      }
    }
    if (cur->stack_on_obj >= 0 && !baseOnTable(transfers, state, cur->stack_on_obj) &&
      baseCarried(state, cur->stack_on_obj))
    {
      cur = &findTransferById(transfers, cur->stack_on_obj);
    } else {
      cur = nullptr;
    }
  }
  return blockers;
}

/// `t` ocupa o lugar onde algo que esta A BORDO precisa ser solto.
bool blocksCarriedPlace(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t)
{
  for (const int carried : state.inventory) {
    const auto & c = findTransferById(transfers, carried);
    if (std::find(c.place_after_picked.begin(), c.place_after_picked.end(), t.obj_id) !=
      c.place_after_picked.end())
    {
      return true;
    }
  }
  return false;
}

/// Pode PEGAR `t` em `ws` com `onboard` objetos ja a bordo. Alem das regras
/// de pilha (base disponivel, quem esta em cima sai antes), reserva lugar
/// a bordo para os ocupantes da pilha inicial que ainda precisam sair antes
/// do place de `t`: senao o robo lotaria com 3 objetos presos esperando um
/// pick que nao cabe.
bool pickAllowedNow(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const TransferItem & t,
  const std::string & ws,
  const int onboard)
{
  if (!t.needs_pick || state.picked.count(t.obj_id) > 0 || t.from_ws != ws) {
    return false;
  }
  if (!pickAfterSatisfied(state, t)) {
    return false;
  }
  // Ocupante da pilha inicial que trava um place a bordo sai mesmo que a
  // base dele ainda nao esteja disponivel (a base dele tambem vai sair).
  if (!topPickable(transfers, state, t) && !blocksCarriedPlace(transfers, state, t)) {
    return false;
  }
  const int reserve = static_cast<int>(unpickedBlockers(transfers, state, t, t.obj_id).size());
  return onboard + 1 + reserve <= kMaxOnboard;
}

/// Picks possiveis em `ws` partindo de `onboard_after_places` objetos a bordo
/// (os que serao soltos ao chegar ja descontados). Aproximacao: nao simula
/// pick_after entre os contados — serve para escolher a estacao.
int countPendingPicksAtWs(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const std::string & ws,
  const int onboard_after_places)
{
  int count = 0;
  for (const auto & t : transfers) {
    if (pickAllowedNow(transfers, state, t, ws, onboard_after_places + count)) {
      ++count;
    }
  }
  return count;
}

int countCarriedPlacesAtWs(
  const std::vector<TransferItem> & transfers,
  const PlannerState & state,
  const std::string & ws)
{
  int count = 0;
  for (const int obj_id : state.inventory) {
    const auto & t = findTransferById(transfers, obj_id);
    // Topo sem base disponivel ou place bloqueado por ocupante da pilha
    // inicial nao conta (senao a estacao seria escolhida sem ter o que fazer).
    if (t.needs_place && state.placed.count(t.obj_id) == 0 && t.to_ws == ws &&
      placeableOnArrival(transfers, state, t))
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

  const int onboard = static_cast<int>(state.inventory.size());
  const bool empty_robot = state.inventory.empty();

  for (const auto & ws : candidate_ws) {
    const int places = countCarriedPlacesAtWs(transfers, state, ws);
    const int picks = countPendingPicksAtWs(transfers, state, ws, onboard - places);

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

/// Chave do ws_table_mapping para a estacao da task: a exata quando existe;
/// senao o UNICO candidato "<ws>_<n>" / "<ws><n>" (formato 2026 nomeia PP e
/// ST sem sufixo; os mapas tem PP_1 / ST_1). 0 ou >1 candidatos = erro
/// dizendo o que acrescentar no ws_table_mapping.yaml.
std::string resolveWsKey(
  const std::map<std::string, std::string> & table,
  const std::string & ws)
{
  if (table.count(ws) > 0) {
    return ws;
  }
  std::vector<std::string> candidates;
  for (const auto & [key, _] : table) {
    if (key.size() > ws.size() && key.compare(0, ws.size(), ws) == 0) {
      const char next = key[ws.size()];
      if (next == '_' || std::isdigit(static_cast<unsigned char>(next))) {
        candidates.push_back(key);
      }
    }
  }
  if (candidates.size() == 1) {
    return candidates.front();
  }
  if (candidates.empty()) {
    throw std::runtime_error(
            "Missing WS->Mesa mapping for workspace: " + ws + " — estacao sem entrada em "
            "ws_to_table_pose do ws_table_mapping.yaml do mapa (nem candidato " + ws +
            "_<n>): acrescente '" + ws + "_1: Mesa15' em ws_to_table_pose e '" + ws + "_1: " +
            ws + "1' em ws_to_dock_id (e grave a pose do dock " + ws + "1)");
  }
  std::string joined;
  for (const auto & c : candidates) {
    joined += (joined.empty() ? "" : ", ") + c;
  }
  throw std::runtime_error(
          "Estacao '" + ws + "' ambigua no ws_to_table_pose (candidatos: " + joined +
          ") — use o nome completo na task ou acrescente a chave '" + ws +
          "' no ws_table_mapping.yaml");
}

std::string tablePoseForWs(
  const std::map<std::string, std::string> & table,
  const std::string & ws)
{
  return table.at(resolveWsKey(table, ws));
}

void appendGotoIfNeeded(
  YAML::Node & action_seq,
  PlannerState & state,
  const std::string & ws,
  const std::map<std::string, std::string> & ws_to_table_pose)
{
  const std::string table_pose = tablePoseForWs(ws_to_table_pose, ws);
  // Comparar/registrar SEMPRE pela estacao (ws), nunca pela mesa: WS_1 e
  // WS_2 podem ser a mesma Mesa e o goto seria pulado indevidamente.
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
      place["table_pose"] = tablePoseForWs(ws_to_table_pose, t.to_ws);
      if (!t.stack_on_frame.empty()) {
        place["stack_on"] = t.stack_on_frame;
      }
      if (!t.container_color.empty()) {
        place["container_color"] = t.container_color;
      }
      if (!t.at_tag_frame.empty()) {
        place["at_tag"] = t.at_tag_frame;     // slot da mesa de precisao (2026)
      }
      if (!t.to_level.empty()) {
        place["shelf_level"] = t.to_level;    // informativo (2026)
      }

      action_seq.push_back(place);
      state.placed.insert(t.obj_id);
    };

  // Duas passadas: primeiro o que NAO e' topo de pilha (inclui as bases),
  // depois os topos cuja base ja esta na mesa. Um topo cuja base ainda nao
  // foi solta, ou qualquer place cujo lugar ainda esta ocupado por cubo da
  // pilha inicial (place_after_picked), fica no inventario: o laco de
  // buildOutput pega os ocupantes nesta mesma estacao e volta aqui.
  std::vector<int> remaining_inventory;
  std::vector<int> deferred;
  for (const int obj_id : state.inventory) {
    const auto & t = findTransferById(transfers, obj_id);
    const bool here = t.needs_place && state.placed.count(t.obj_id) == 0 && t.to_ws == ws;
    if (!here) {
      remaining_inventory.push_back(obj_id);
    } else if (t.stack_on_obj >= 0 || !placeAfterPickedSatisfied(state, t)) {
      deferred.push_back(obj_id);
    } else {
      emit_place(t);
    }
  }
  // Topos: repete ate nao haver progresso (pilha de 3+: o topo do topo so
  // sai depois do meio).
  bool progress = true;
  while (progress && !deferred.empty()) {
    progress = false;
    std::vector<int> still_deferred;
    for (const int obj_id : deferred) {
      const auto & t = findTransferById(transfers, obj_id);
      if (placeReady(transfers, state, t)) {
        emit_place(t);
        progress = true;
      } else {
        still_deferred.push_back(obj_id);
      }
    }
    deferred = still_deferred;
  }
  for (const int obj_id : deferred) {
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
  // progresso. pickAllowedNow tambem reserva lugar a bordo para ocupantes da
  // pilha inicial que ainda precisam sair.
  bool progress = true;
  while (progress) {
  progress = false;
  for (const auto & t : transfers) {
    if (static_cast<int>(state.inventory.size()) >= kMaxOnboard) {
      return;
    }
    if (!pickAllowedNow(transfers, state, t, ws, static_cast<int>(state.inventory.size()))) {
      continue;
    }
    progress = true;

    YAML::Node pick;
    pick["kind"] = "pick";
    pick["tag_frame"] = t.tag_frame;
    pick["ws"] = t.from_ws;
    // O pick tambem precisa saber o tipo de mesa: a prateleira (MesaSh)
    // exige uma sequencia de movimentos propria.
    pick["table_pose"] = tablePoseForWs(ws_to_table_pose, t.from_ws);
    if (!t.from_level.empty()) {
      pick["shelf_level"] = t.from_level;   // informativo (2026)
    }
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
              " (pilha cuja base nao pode ser alcancada, ou ocupante da pilha inicial "
              "que nao cabe a bordo? inventario: " +
              std::to_string(state.inventory.size()) + " objeto(s) a bordo)");
    }
  }


  output["actions"] = action_seq;
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
  PlannerState state;
  appendGotoIfNeeded(action_seq, state, station.ws, ws_to_table_pose);

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
    const auto output = buildOutput(competition_root, transfers, apriltag_yaml_path, ws_to_table_pose);

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
