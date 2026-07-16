#pragma once

// Modulo Mapeamento: inicia/para o SLAM (caramelo_mapping/slam.launch.py, sem
// RViz proprio — o mapa cresce no RViz embutido da GUI) e salva o mapa via
// /map_saver/save_map. A trava SLAMxAMCL (RK-02) do launch continua valendo:
// se a localizacao estiver ativa, o launch aborta e o erro aparece no log.

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
