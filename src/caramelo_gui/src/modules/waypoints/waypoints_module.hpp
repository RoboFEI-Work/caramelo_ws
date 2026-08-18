#pragma once

// Modulo Waypoints (generico e OPCIONAL — o padrao do robo continua sendo o
// planejamento livre do Nav2): criar na pose do robo, mover/girar pelo
// Interactive Marker no mapa, salvar por mapa, navegar ate um e seguir todos.
//
// O nome do waypoint e' livre de proposito (um waypoint e' "um lugar qualquer",
// nao uma estacao do regulamento), mas nao pode REPETIR: o WaypointManager
// guarda os pontos num mapa por nome, entao criar "WP1" com "WP1" ja' existente
// nao dava erro nenhum -- apagava por cima do ponto antigo, e o operador so'
// descobria quando o robo ia parar no lugar errado.

#include <QString>
#include <QWidget>

class RosBridge;
class SeletorArena;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;

class WaypointsModule : public QWidget
{
  Q_OBJECT

public:
  explicit WaypointsModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  QString mapDir() const;

  // Existe arena selecionada? Sem isso, mapDir() vira ".../maps/" e o
  // waypoints.yaml era gravado FORA de qualquer arena, sem erro na tela.
  bool temArena();

  // Proximo nome livre da familia WP<n>. Serve de sugestao: quem cria vinte
  // pontos seguidos nao deveria ter que lembrar em que numero parou.
  QString sugerirProximoNome() const;

  // Liga/desliga o que depende de arena e escreve o motivo na tela.
  void atualizarDisponibilidade();

  RosBridge * bridge_;
  SeletorArena * map_name_ = nullptr;
  QLineEdit * nome_ = nullptr;
  QListWidget * lista_ = nullptr;
  QLabel * status_ = nullptr;
  QLabel * motivo_ = nullptr;

  QPushButton * novo_ = nullptr;
  QPushButton * remover_ = nullptr;
  QPushButton * carregar_ = nullptr;
  QPushButton * salvar_ = nullptr;
  QPushButton * ir_ = nullptr;
  QPushButton * seguir_ = nullptr;

  // Sugestao que a tela colocou no campo. Se o operador digitou outra coisa,
  // nao sobrescrevemos o que ele escreveu.
  QString sugestao_;

  // De qual arena veio a lista que esta' na tela. Gravar a lista da arena A
  // dentro da arena B apagava os waypoints de B sem aviso nenhum.
  QString arena_carregada_;
};
