#pragma once

// Janela principal (shell da GUI). Sidebar de modulos (cartoes) + area central
// com paginas (QStackedWidget) + barra de estado com a maquina de estados.
// v1: modulos "Inicio" (cartoes de saude) e "Robo" (RViz embutido); os demais
// aparecem desabilitados para mostrar a estrutura.

#include <QMainWindow>

class RosBridge;
class StateMachine;
class InicioModule;
class NavegacaoModule;
class DockingModule;
class LocalizacaoModule;
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
  QWidget * buildRoboPage();
  void addModule(const QString & nome, int pageIndex, bool enabled);

  RosBridge * bridge_ = nullptr;
  StateMachine * state_ = nullptr;
  InicioModule * inicio_ = nullptr;
  NavegacaoModule * navegacao_ = nullptr;
  DockingModule * docking_ = nullptr;
  LocalizacaoModule * localizacao_ = nullptr;
  RVizFrame * rviz_ = nullptr;

  QListWidget * sidebar_ = nullptr;
  QStackedWidget * pages_ = nullptr;
  QLabel * state_label_ = nullptr;
};
