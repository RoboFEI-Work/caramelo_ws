#pragma once
// Nome do no rclcpp de cada folha da arvore: prefixo da folha + nome da
// instancia no XML (ex.: "bt_pick_tag_pick_7"), em vez do ponteiro. Nomes
// repetidos (folhas sem atributo name, como GoToNamedPose) ganham sufixo
// _2, _3... para o `ros2 node list` ficar legivel e unico.

#include <mutex>
#include <string>
#include <unordered_map>

namespace manip_bt
{

inline std::string sanitizeRosName(const std::string & raw)
{
  std::string out;
  out.reserve(raw.size());
  for (const char c : raw) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
      (c >= '0' && c <= '9') || c == '_';
    out += ok ? c : '_';
  }
  return out.empty() ? std::string("node") : out;
}

inline std::string leafNodeName(const std::string & prefix, const std::string & leaf_name)
{
  static std::mutex mutex;
  static std::unordered_map<std::string, int> seen;
  const std::string base = prefix + sanitizeRosName(leaf_name);
  std::lock_guard<std::mutex> lock(mutex);
  const int n = ++seen[base];
  return n == 1 ? base : base + "_" + std::to_string(n);
}

}  // namespace manip_bt
