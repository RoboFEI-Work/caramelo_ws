#pragma once

// Modulo Service Areas: fluxo guiado do plano — posicionar o robo, "Salvar
// pose atual", escolher nome/tipo, aparece no mapa (MarkerArray ja exibido no
// RViz embutido). Reusa a API pronta do service_area_manager_node.
//
// O tipo vem do SeletorTipoArea, que e' a unica lista valida desses seis
// valores (a copia escrita a mao aqui saiu). O nome continua digitavel porque
// esta tela tambem CRIA area, mas passa pelo validador do regulamento antes de
// ir para a action: um nome torto fazia o save abortar calado.

#include <QWidget>

class RosBridge;
class SeletorArena;
class SeletorServiceArea;
class SeletorTipoArea;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;

class ServiceAreasModule : public QWidget
{
  Q_OBJECT

public:
  explicit ServiceAreasModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  // Desliga o que depende de arena e escreve o motivo na tela: botao cinza sem
  // frase le' como interface quebrada.
  void atualizarDisponibilidade();

  RosBridge * bridge_;
  SeletorArena * map_name_ = nullptr;
  SeletorServiceArea * existentes_ = nullptr;
  QLineEdit * area_id_ = nullptr;
  SeletorTipoArea * tipo_ = nullptr;
  QListWidget * lista_ = nullptr;
  QLabel * status_ = nullptr;
  QLabel * motivo_ = nullptr;

  QPushButton * botao_salvar_ = nullptr;
  QPushButton * botao_listar_ = nullptr;
  QPushButton * botao_validar_ = nullptr;
};
