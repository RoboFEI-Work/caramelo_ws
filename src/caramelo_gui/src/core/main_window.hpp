#pragma once

// Janela principal (shell da GUI) — layout "mapa sempre visivel":
//   [ sidebar | MAPA (RViz, permanente) | painel de funcoes (troca) ]
// A sidebar troca SO o painel da direita; o mapa nunca sai da tela (pedido do
// dono: "como se a GUI fosse dividida em dois"). Logs ficam num painel
// inferior oculto (botao na barra de status).

#include <QMainWindow>

class RosBridge;
class StateMachine;
class InicioModule;
class NavegacaoModule;
class DockingModule;
class LocalizacaoModule;
class MapasModule;
class MapeamentoModule;
class TeleopModule;
class ServiceAreasModule;
class WaypointsModule;
class EditorMapaModule;
class RVizFrame;

class QListWidget;
class QStackedWidget;
class QLabel;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() override;

private:
  QWidget * buildSidebar();
  QWidget * buildRoboPanel();   // Camadas + Ferramentas + Localizacao rapida
  void addModule(const QString & nome, int panelIndex, bool enabled);

  RosBridge * bridge_ = nullptr;
  StateMachine * state_ = nullptr;
  RVizFrame * rviz_ = nullptr;

  InicioModule * inicio_ = nullptr;
  NavegacaoModule * navegacao_ = nullptr;
  DockingModule * docking_ = nullptr;
  LocalizacaoModule * localizacao_ = nullptr;
  MapasModule * mapas_ = nullptr;
  MapeamentoModule * mapeamento_ = nullptr;
  TeleopModule * teleop_ = nullptr;
  ServiceAreasModule * service_areas_ = nullptr;
  WaypointsModule * waypoints_ = nullptr;
  EditorMapaModule * editor_mapa_ = nullptr;

  QListWidget * sidebar_ = nullptr;
  QStackedWidget * painel_ = nullptr;   // painel de funcoes (direita)
  QLabel * state_label_ = nullptr;
};
