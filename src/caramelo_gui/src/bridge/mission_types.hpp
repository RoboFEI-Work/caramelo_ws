#pragma once

// Um instante de uma missao, ja' traduzido de caramelo_msgs/MissionStatus para
// tipos Qt. Segue o mesmo contrato do health_types.hpp: os widgets nunca veem
// tipos ROS, so' esta struct, transportada por sinal entre a thread ROS e a
// thread da UI (por isso precisa de metatype).

#include <QMetaType>
#include <QString>

struct MissionProgress
{
  // Espelha as constantes de caramelo_msgs/MissionStatus.
  enum class State
  {
    Idle = 0,
    Planning = 1,
    Preflight = 2,
    Running = 3,
    Done = 4,
    Failed = 5,
    Aborted = 6,
  };

  State state = State::Idle;
  QString missionId;
  QString taskId;
  // Passo atual dentro da arvore, contando prologo e epilogo (o executor os
  // acrescenta sozinho). -1 quando nenhum passo esta' em execucao.
  int index = -1;
  int total = 0;
  QString kind;     // goto | pick | place | home
  QString target;   // WS1 | tag_01 | Mesa15 | home
  QString stage;    // iniciado | concluido | falhou | ...
  QString message;
  double elapsed = 0.0;

  bool terminal() const
  {
    return state == State::Done || state == State::Failed || state == State::Aborted;
  }

  static QString label(State s)
  {
    switch (s) {
      case State::Idle: return "Parado";
      case State::Planning: return "Planejando";
      case State::Preflight: return "Pre-flight";
      case State::Running: return "Executando";
      case State::Done: return "Concluida";
      case State::Failed: return "Falhou";
      case State::Aborted: return "Abortada";
    }
    return "?";
  }
};

Q_DECLARE_METATYPE(MissionProgress)
