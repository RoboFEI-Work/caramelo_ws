#pragma once

// Modulo Navegacao: envia meta (via ferramenta "2D Goal" do RViz -> /goal_pose,
// encaminhada pelo RosBridge para navigate_to_pose), cancela e limpa costmaps.
// Mostra o status vindo dos feedbacks da action.

#include <QWidget>

class RosBridge;
class LaunchRunner;
class QLabel;
class QLineEdit;
class QCheckBox;
class QPushButton;

class NavegacaoModule : public QWidget
{
  Q_OBJECT

public:
  explicit NavegacaoModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  RosBridge * bridge_;
  LaunchRunner * runner_ = nullptr;
  QLineEdit * map_name_ = nullptr;
  QCheckBox * com_docking_ = nullptr;
  QPushButton * ligar_ = nullptr;
  QLabel * status_ = nullptr;
};
