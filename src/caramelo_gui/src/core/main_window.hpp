#pragma once

// Shell da GUI — aplicacao embarcada, nao ferramenta de desenvolvedor.
//
//   [ faixa de estado: voltar | contexto | arena | estado | logs ]
//   [            mapa (so' quando o contexto pede)  |  contexto  ]
//
// O mapa (RViz) deixou de ser fixo no centro de todas as telas. Ele existe uma
// unica vez e e' apenas MOSTRADO ou ESCONDIDO conforme o contexto declarar que
// precisa dele (ContextScreen::precisaDoMapa). Nao ha' reparent nem recriacao:
// o render do OGRE e' caro e ja' custou segfault neste projeto.
//
// A tela inicial e' um menu de acoes, nao um painel de diagnostico. O detalhe
// tecnico vive no contexto Modo Avancado.

#include <QMainWindow>

class RosBridge;
class StateMachine;
class RobotState;
class RVizFrame;
class HomeScreen;
class ContextScreen;

class InicioModule;
class CompeticaoModule;
class NavegacaoModule;
class DockingModule;
class LocalizacaoModule;
class MapasModule;
class MapeamentoModule;
class TeleopModule;
class ServiceAreasModule;
class WaypointsModule;
class EditorMapaModule;

class QLabel;
class QPushButton;
class QStackedWidget;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() override;

protected:
  void resizeEvent(QResizeEvent * e) override;

private:
  void ajustarLarguraDoPainel();
  QWidget * construirFaixa();
  QWidget * construirPainelDoMapa();   // camadas + ferramentas do RViz
  void construirContextos();
  void irPara(int indice);

  RosBridge * bridge_ = nullptr;
  StateMachine * state_ = nullptr;
  RobotState * robot_state_ = nullptr;
  RVizFrame * rviz_ = nullptr;

  HomeScreen * home_ = nullptr;
  QStackedWidget * telas_ = nullptr;

  InicioModule * inicio_ = nullptr;
  CompeticaoModule * competicao_ = nullptr;
  NavegacaoModule * navegacao_ = nullptr;
  DockingModule * docking_ = nullptr;
  LocalizacaoModule * localizacao_ = nullptr;
  MapasModule * mapas_ = nullptr;
  MapeamentoModule * mapeamento_ = nullptr;
  TeleopModule * teleop_ = nullptr;
  ServiceAreasModule * service_areas_ = nullptr;
  WaypointsModule * waypoints_ = nullptr;
  EditorMapaModule * editor_mapa_ = nullptr;

  QPushButton * voltar_ = nullptr;
  QPushButton * logs_btn_ = nullptr;
  QLabel * titulo_ = nullptr;
  QLabel * arena_label_ = nullptr;
  QLabel * state_label_ = nullptr;
};
