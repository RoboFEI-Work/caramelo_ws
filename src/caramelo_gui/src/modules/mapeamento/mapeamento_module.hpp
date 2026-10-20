#pragma once

// Modulo Mapeamento: inicia/para o SLAM (caramelo_mapping/slam.launch.py, sem
// RViz proprio — o mapa cresce no RViz embutido da GUI) e salva o mapa via
// /map_saver_server/save_map. RK-02: se o AMCL parecer ativo o launch so AVISA
// (nao aborta) e SLAM e AMCL passam a publicar map->odom; derrubar o Nav2 antes.

#include <QWidget>

class LaunchRunner;
class RosBridge;
class QLabel;
class QLineEdit;
class QPushButton;

class MapeamentoModule : public QWidget
{
  Q_OBJECT

public:
  explicit MapeamentoModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  RosBridge * bridge_;
  LaunchRunner * runner_ = nullptr;
  QLineEdit * nome_ = nullptr;
  QPushButton * botao_ = nullptr;
  QLabel * status_ = nullptr;
  QLabel * log_ = nullptr;
};
