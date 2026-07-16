#pragma once

// Modulo Localizacao: relocalizacao manual assistida. O operador arrasta o
// robo-fantasma no mapa (pagina Robo) enquanto o scan re-projetado acompanha
// em tempo real; ao Confirmar, a pose vai para /initialpose (AMCL).

#include <QWidget>

class RosBridge;
class QLabel;

class LocalizacaoModule : public QWidget
{
  Q_OBJECT

public:
  explicit LocalizacaoModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  RosBridge * bridge_;
  QLabel * status_ = nullptr;
  QLabel * pose_ = nullptr;
};
