#pragma once

// Maquina de estados da GUI. Deriva o estado de sinais REAIS do grafo (saude dos
// componentes), evitando combinacoes invalidas. v1: derivacao grosseira a partir
// do /diagnostics; estados finos (MAPPING/DOCKING/...) entram com mais sinais.

#include <QObject>
#include <QString>
#include <QVector>

#include "bridge/health_types.hpp"

class StateMachine : public QObject
{
  Q_OBJECT

public:
  enum class State
  {
    OFFLINE,
    HARDWARE_READY,
    MAPPING,
    LOCALIZED,
    NAVIGATION_READY,
    NAVIGATING,
    DOCKING,
    ERROR
  };

  explicit StateMachine(QObject * parent = nullptr);

  State state() const {return state_;}
  static QString label(State s);

public slots:
  void updateFromHealth(const QVector<ComponentHealth> & health);

signals:
  void stateChanged(int state, const QString & label);

private:
  void setState(State s);
  State state_ = State::OFFLINE;
};
