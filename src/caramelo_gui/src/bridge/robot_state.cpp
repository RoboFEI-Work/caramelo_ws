#include "bridge/robot_state.hpp"

#include <fstream>

#include <QDir>
#include <QFileInfo>

#include <yaml-cpp/yaml.h>

RobotState::RobotState(QObject * parent)
: QObject(parent)
{
  load();
}

QString RobotState::filePath()
{
  // Mesma pasta que a manipulacao ja' usa para o estado dos containers de
  // bordo (~/.config/caramelo/container_states.yaml).
  return QDir::homePath() + "/.config/caramelo/robot_state.yaml";
}

void RobotState::load()
{
  const QString caminho = filePath();
  if (!QFileInfo::exists(caminho)) {
    return;   // robo novo: sem mapa definido, e a UI tem que pedir um
  }
  try {
    const YAML::Node raiz = YAML::LoadFile(caminho.toStdString());
    if (raiz["map_name"]) {
      map_name_ = QString::fromStdString(raiz["map_name"].as<std::string>(""));
    }
  } catch (const std::exception &) {
    // Arquivo corrompido vale o mesmo que arquivo ausente: melhor pedir o mapa
    // ao operador do que subir num ambiente errado.
    map_name_.clear();
  }
}

bool RobotState::setMapName(const QString & name, QString * erro)
{
  const QString caminho = filePath();
  QDir().mkpath(QFileInfo(caminho).absolutePath());

  YAML::Node raiz;
  // Preserva chaves futuras que outro componente tenha gravado aqui.
  if (QFileInfo::exists(caminho)) {
    try {
      raiz = YAML::LoadFile(caminho.toStdString());
    } catch (const std::exception &) {
      raiz = YAML::Node();
    }
  }
  raiz["map_name"] = name.toStdString();

  std::ofstream out(caminho.toStdString());
  if (!out.is_open()) {
    if (erro) {
      *erro = "Nao consegui gravar " + caminho;
    }
    return false;
  }
  out << raiz;
  out << "\n";
  out.close();

  map_name_ = name;
  emit mapNameChanged(map_name_);
  return true;
}
