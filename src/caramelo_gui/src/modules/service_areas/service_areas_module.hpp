#pragma once

// Modulo Service Areas: fluxo guiado do plano — posicionar o robo, "Salvar
// pose atual", escolher nome/tipo, aparece no mapa (MarkerArray ja exibido no
// RViz embutido). Reusa a API pronta do service_area_manager_node.

#include <QWidget>

class RosBridge;
class SeletorArena;
class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;

class ServiceAreasModule : public QWidget
{
  Q_OBJECT

public:
  explicit ServiceAreasModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  RosBridge * bridge_;
  SeletorArena * map_name_ = nullptr;
  QLineEdit * area_id_ = nullptr;
  QComboBox * tipo_ = nullptr;
  QListWidget * lista_ = nullptr;
  QLabel * status_ = nullptr;
};
