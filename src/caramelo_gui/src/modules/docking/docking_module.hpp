#pragma once

// Modulo Docking: reaproveita as actions /dock_robot, /undock_robot e
// align_to_dock (alinhamento fino holonomico), e permite regravar a pose atual
// como pose do dock via /save_dock_pose.
//
// O dock e o tipo de aproximacao saem de LISTAS lidas do docking.yaml da arena,
// nao de texto livre: um id digitado errado nao dava erro nenhum aqui -- a
// action ia procurar um dock inexistente e a falha aparecia longe do teclado.
// Criar dock novo nao e' desta tela; isso e' o passo 2 da Ferramenta de
// Mapeamento, onde o robo esta' posicionado no lugar.

#include <QWidget>

class RosBridge;
class SeletorArena;
class SeletorDock;
class SeletorTipoDock;
class QLabel;
class QCheckBox;
class QPushButton;

class DockingModule : public QWidget
{
  Q_OBJECT

public:
  explicit DockingModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  // Existe arena e existe estacao escolhida nela. E' o minimo para qualquer
  // acao, inclusive a de gravar a pose que ainda falta.
  bool temAlvo();

  // temAlvo() mais a pose ja' gravada: exigido por tudo que MOVE o robo, senao
  // a tela deixaria mandar o robo para a origem do mapa.
  bool podeAgir();

  // Desliga o que nao da' para fazer e ESCREVE o porque na tela. Botao cinza
  // sem frase le' como interface quebrada -- o operador fica clicando e
  // reiniciando a GUI achando que e' defeito.
  void atualizarDisponibilidade();

  RosBridge * bridge_;
  SeletorDock * dock_id_ = nullptr;
  SeletorArena * map_name_ = nullptr;
  SeletorTipoDock * tipo_dock_ = nullptr;
  QCheckBox * refine_ = nullptr;
  QLabel * status_ = nullptr;
  QLabel * motivo_ = nullptr;

  QPushButton * botao_dockar_ = nullptr;
  QPushButton * botao_alinhar_ = nullptr;
  QPushButton * botao_regravar_ = nullptr;
};
