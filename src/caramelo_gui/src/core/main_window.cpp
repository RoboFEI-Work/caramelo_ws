#include "core/main_window.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QStackedWidget>
#include <QStatusBar>
#include <QWidget>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "bridge/ros_bridge.hpp"
#include "core/rviz_frame.hpp"
#include "core/state_machine.hpp"
#include "modules/inicio/inicio_module.hpp"

MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent)
{
  setWindowTitle("Caramelo — Interface de Operacao");
  resize(1280, 800);

  bridge_ = new RosBridge(this);
  state_ = new StateMachine(this);

  // Paginas (modulos).
  pages_ = new QStackedWidget();
  inicio_ = new InicioModule();

  QString rviz_config;
  try {
    rviz_config = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("caramelo_gui") +
      "/resources/rviz/gui_main.rviz");
  } catch (...) {
    rviz_config.clear();
  }
  rviz_ = new RVizFrame(rviz_config);

  pages_->addWidget(inicio_);   // indice 0
  pages_->addWidget(rviz_);     // indice 1

  // Layout central: sidebar + paginas.
  auto * central = new QWidget();
  auto * layout = new QHBoxLayout(central);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(buildSidebar());
  layout->addWidget(pages_, 1);
  setCentralWidget(central);

  // Barra de estado.
  state_label_ = new QLabel(StateMachine::label(StateMachine::State::OFFLINE));
  state_label_->setObjectName("estadoAtual");
  statusBar()->addPermanentWidget(state_label_);

  // Ligacoes ROS -> UI.
  connect(bridge_, &RosBridge::diagnosticsUpdated, inicio_, &InicioModule::onDiagnostics);
  connect(bridge_, &RosBridge::diagnosticsUpdated, state_, &StateMachine::updateFromHealth);
  connect(
    state_, &StateMachine::stateChanged, this,
    [this](int, const QString & label) {
      state_label_->setText("Estado: " + label);
    });

  bridge_->start();
}

MainWindow::~MainWindow()
{
  if (bridge_) {
    bridge_->stop();
  }
}

QWidget * MainWindow::buildSidebar()
{
  sidebar_ = new QListWidget();
  sidebar_->setObjectName("sidebar");
  sidebar_->setFixedWidth(220);

  addModule("Inicio", 0, true);
  addModule("Robo", 1, true);
  // Placeholders da estrutura (habilitam nas proximas versoes):
  addModule("Mapas", -1, false);
  addModule("Mapeamento", -1, false);
  addModule("Navegacao", -1, false);
  addModule("Service Areas", -1, false);
  addModule("Docking", -1, false);
  addModule("Waypoints", -1, false);
  addModule("Testes", -1, false);
  addModule("Diagnostico", -1, false);
  addModule("Simulacao", -1, false);

  connect(
    sidebar_, &QListWidget::currentItemChanged, this,
    [this](QListWidgetItem * current, QListWidgetItem *) {
      if (!current) {
        return;
      }
      const int page = current->data(Qt::UserRole).toInt();
      if (page >= 0) {
        pages_->setCurrentIndex(page);
      }
    });

  sidebar_->setCurrentRow(0);
  return sidebar_;
}

void MainWindow::addModule(const QString & nome, int pageIndex, bool enabled)
{
  auto * item = new QListWidgetItem(nome, sidebar_);
  item->setData(Qt::UserRole, pageIndex);
  item->setSizeHint(QSize(0, 48));
  if (!enabled) {
    item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
  }
}
