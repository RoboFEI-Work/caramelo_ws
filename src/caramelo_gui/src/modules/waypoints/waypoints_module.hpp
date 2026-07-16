#pragma once

// Modulo Waypoints (generico e OPCIONAL — o padrao do robo continua sendo o
// planejamento livre do Nav2): criar na pose do robo, mover/girar pelo
// Interactive Marker no mapa, salvar por mapa, navegar ate um e seguir todos.

#include <QWidget>

class RosBridge;
class QLabel;
class QLineEdit;
class QListWidget;

class WaypointsModule : public QWidget
{
  Q_OBJECT

public:
  explicit WaypointsModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  QString mapDir() const;

  RosBridge * bridge_;
  QLineEdit * map_name_ = nullptr;
  QLineEdit * nome_ = nullptr;
  QListWidget * lista_ = nullptr;
  QLabel * status_ = nullptr;
};
