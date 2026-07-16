#include "core/main_window.hpp"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QStackedWidget>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QWidget>

#include "bridge/ros_bridge.hpp"
#include "core/rviz_frame.hpp"
#include "core/state_machine.hpp"
#include "modules/inicio/inicio_module.hpp"
#include "modules/navegacao/navegacao_module.hpp"
#include "modules/docking/docking_module.hpp"
#include "modules/localizacao/localizacao_module.hpp"
#include "modules/mapas/mapas_module.hpp"

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

  rviz_ = new RVizFrame();
  navegacao_ = new NavegacaoModule(bridge_);
  docking_ = new DockingModule(bridge_);
  localizacao_ = new LocalizacaoModule(bridge_);
  mapas_ = new MapasModule(bridge_);

  pages_->addWidget(inicio_);          // indice 0
  pages_->addWidget(buildRoboPage());  // indice 1
  pages_->addWidget(navegacao_);       // indice 2
  pages_->addWidget(docking_);         // indice 3
  pages_->addWidget(localizacao_);     // indice 4
  pages_->addWidget(mapas_);           // indice 5

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
  addModule("Navegacao", 2, true);
  addModule("Docking", 3, true);
  addModule("Localizacao", 4, true);
  addModule("Mapas", 5, true);
  // Placeholders da estrutura (habilitam nas proximas versoes):
  addModule("Mapeamento", -1, false);
  addModule("Service Areas", -1, false);
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

QWidget * MainWindow::buildRoboPage()
{
  auto * page = new QWidget();
  auto * layout = new QHBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(rviz_, 1);

  auto * side = new QWidget();
  side->setObjectName("painelLateral");
  side->setFixedWidth(240);
  auto * sideLayout = new QVBoxLayout(side);

  auto * layersTitle = new QLabel("Camadas");
  layersTitle->setObjectName("tituloModulo");
  sideLayout->addWidget(layersTitle);
  for (const QString & name : rviz_->layerNames()) {
    auto * cb = new QCheckBox(name);
    cb->setChecked(rviz_->isLayerEnabled(name));
    connect(
      cb, &QCheckBox::toggled, this,
      [this, name](bool on) {rviz_->setLayerEnabled(name, on);});
    sideLayout->addWidget(cb);
  }

  sideLayout->addSpacing(12);
  auto * toolsTitle = new QLabel("Ferramentas");
  toolsTitle->setObjectName("tituloModulo");
  sideLayout->addWidget(toolsTitle);
  auto addToolBtn = [this, sideLayout](const QString & text, const QString & key) {
      auto * b = new QPushButton(text);
      connect(b, &QPushButton::clicked, this, [this, key]() {rviz_->activateTool(key);});
      sideLayout->addWidget(b);
    };
  addToolBtn("Interagir", "interact");
  addToolBtn("Mover camera", "move");
  addToolBtn("Definir Goal", "goal");
  addToolBtn("Estimar Pose", "initial_pose");
  sideLayout->addStretch();

  layout->addWidget(side);
  return page;
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
