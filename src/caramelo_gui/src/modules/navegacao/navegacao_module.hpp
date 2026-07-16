#pragma once

// Modulo Navegacao: envia meta (via ferramenta "2D Goal" do RViz -> /goal_pose,
// encaminhada pelo RosBridge para navigate_to_pose), cancela e limpa costmaps.
// Mostra o status vindo dos feedbacks da action.

#include <QWidget>

class RosBridge;
class QLabel;

class NavegacaoModule : public QWidget
{
  Q_OBJECT

public:
  explicit NavegacaoModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  RosBridge * bridge_;
  QLabel * status_ = nullptr;
};
