#include "core/state_machine.hpp"

namespace
{
constexpr int LEVEL_OK = 0;
constexpr int LEVEL_ERROR = 2;

int levelOf(const QVector<ComponentHealth> & health, const QString & name)
{
  for (const auto & c : health) {
    if (c.name == name) {
      return c.level;
    }
  }
  return 3;  // STALE: nao encontrado
}
}  // namespace

StateMachine::StateMachine(QObject * parent)
: QObject(parent)
{
}

QString StateMachine::label(State s)
{
  switch (s) {
    case State::OFFLINE: return "OFFLINE";
    case State::HARDWARE_READY: return "HARDWARE PRONTO";
    case State::MAPPING: return "MAPEANDO";
    case State::LOCALIZED: return "LOCALIZADO";
    case State::NAVIGATION_READY: return "NAVEGACAO PRONTA";
    case State::NAVIGATING: return "NAVEGANDO";
    case State::DOCKING: return "DOCKING";
    case State::ERROR: return "ERRO";
  }
  return "?";
}

void StateMachine::updateFromHealth(const QVector<ComponentHealth> & health)
{
  const int rede = levelOf(health, "caramelo/rede_raspberry");
  const int controllers = levelOf(health, "caramelo/controllers");
  const int tf = levelOf(health, "caramelo/tf");
  const int nav2 = levelOf(health, "caramelo/nav2");

  State next;
  if (rede == LEVEL_ERROR || rede == 3) {
    next = State::OFFLINE;                 // sem Raspberry, nada roda
  } else if (controllers == LEVEL_ERROR) {
    next = State::ERROR;                   // Pi presente mas controllers caidos
  } else if (nav2 == LEVEL_OK && tf == LEVEL_OK) {
    next = State::NAVIGATION_READY;        // Nav2 ativo + TF completo
  } else if (tf == LEVEL_OK) {
    next = State::LOCALIZED;               // TF map->base ok (localizado/mapeando)
  } else {
    next = State::HARDWARE_READY;          // Pi + controllers ok, sem localizacao
  }
  setState(next);
}

void StateMachine::setState(State s)
{
  if (s == state_) {
    return;
  }
  state_ = s;
  emit stateChanged(static_cast<int>(s), label(s));
}
