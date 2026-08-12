#pragma once

// Ferramenta de Mapeamento: UMA ferramenta, um fluxo.
//
// Antes eram quatro modulos soltos na sidebar -- Mapeamento, Waypoints, Service
// Areas e Editor de Mapa -- como se fossem assuntos diferentes. Nao sao: tudo
// isso nasce dentro de um mapa e so' faz sentido depois que ele existe. Quem
// nunca mexeu com ROS nao tem como adivinhar essa ordem a partir de uma lista.
//
// Aqui o fluxo aparece como ele acontece de verdade:
//   1. Criar o mapa      — SLAM ligado, dirigir o robo, salvar com um nome
//   2. Marcar pontos     — levar o robo ao lugar e gravar a pose com um nome
//   3. Paredes virtuais  — pintar o que o LiDAR nao ve (fita de piso, degrau)
//   4. Conferir          — o mapa como ficou salvo, com avisos do que falta
//
// A unificacao de verdade esta no passo 2. Waypoint, dock e service area eram
// tres telas com tres formularios, mas sao a MESMA operacao: gravar a pose atual
// do robo com um nome e um tipo. Aqui viraram um seletor de tipo.

#include <QString>
#include <QWidget>

#include "widgets/map_preview.hpp"

class RosBridge;
class LaunchRunner;
class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QStackedWidget;

class FerramentaMapeamento : public QWidget
{
  Q_OBJECT

public:
  FerramentaMapeamento(RosBridge * bridge, QWidget * parent = nullptr);

signals:
  // A ferramenta pede o mapa ao vivo (RViz) nos passos em que ele e' o
  // instrumento de trabalho, e o preview do arquivo no passo de conferencia.
  void querMapaAoVivo(bool querer);

private:
  QWidget * construirPassoCriar();
  QWidget * construirPassoPontos();
  QWidget * construirPassoParedes();
  QWidget * construirPassoConferir();

  void irParaPasso(int passo);
  void gravarPonto();
  void recarregarConferencia();
  QString mapaAlvo() const;

  RosBridge * bridge_ = nullptr;
  LaunchRunner * slam_ = nullptr;
  LaunchRunner * teleop_ = nullptr;

  QListWidget * passos_ = nullptr;
  QStackedWidget * paginas_ = nullptr;

  // Passo 1
  QLineEdit * nome_do_mapa_ = nullptr;
  QPushButton * botao_slam_ = nullptr;
  QPushButton * botao_teleop_ = nullptr;
  QLabel * status_slam_ = nullptr;

  // Passo 2
  QComboBox * tipo_do_ponto_ = nullptr;
  QLineEdit * nome_do_ponto_ = nullptr;
  QLabel * ajuda_do_tipo_ = nullptr;
  QLabel * status_ponto_ = nullptr;
  QListWidget * pontos_ = nullptr;

  // Passo 4
  MapPreview * preview_ = nullptr;
  QLabel * conferencia_ = nullptr;
};
