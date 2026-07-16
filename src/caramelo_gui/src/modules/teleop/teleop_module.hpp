#pragma once

// Modulo Teleop: liga/desliga o teleop (caramelo_utils/teleop.launch.py —
// joystick + teclado + twist_mux) como processo gerido pela GUI.

#include <QWidget>

class LaunchRunner;
class QLabel;
class QPushButton;

class TeleopModule : public QWidget
{
  Q_OBJECT

public:
  explicit TeleopModule(QWidget * parent = nullptr);

private:
  LaunchRunner * runner_ = nullptr;
  QPushButton * botao_ = nullptr;
  QLabel * status_ = nullptr;
};
