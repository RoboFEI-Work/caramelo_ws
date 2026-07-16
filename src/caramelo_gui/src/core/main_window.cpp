#include "core/main_window.hpp"

#include <QCheckBox>
#include <QGraphicsOpacityEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "bridge/manual_localization.hpp"
#include "bridge/ros_bridge.hpp"
#include "core/rviz_frame.hpp"
#include "core/state_machine.hpp"
#include "modules/inicio/inicio_module.hpp"
#include "modules/navegacao/navegacao_module.hpp"
#include "modules/docking/docking_module.hpp"
#include "modules/localizacao/localizacao_module.hpp"
#include "modules/mapas/mapas_module.hpp"
#include "modules/mapeamento/mapeamento_module.hpp"
#include "modules/teleop/teleop_module.hpp"
#include "modules/service_areas/service_areas_module.hpp"
#include "modules/waypoints/waypoints_module.hpp"
#include "modules/editor_mapa/editor_mapa_module.hpp"

MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent)
{
  setWindowTitle("Caramelo — Interface de Operacao");
  resize(1440, 860);

  bridge_ = new RosBridge(this);
  state_ = new StateMachine(this);

  // MAPA permanente no centro.
  rviz_ = new RVizFrame();

  // Painel de funcoes (direita) — a sidebar troca somente ele.
  painel_ = new QStackedWidget();
  painel_->setFixedWidth(420);

  inicio_ = new InicioModule();
  navegacao_ = new NavegacaoModule(bridge_);
  docking_ = new DockingModule(bridge_);
  localizacao_ = new LocalizacaoModule(bridge_);
  mapas_ = new MapasModule(bridge_);
  mapeamento_ = new MapeamentoModule(bridge_);
  teleop_ = new TeleopModule();
  service_areas_ = new ServiceAreasModule(bridge_);
  waypoints_ = new WaypointsModule(bridge_);
  editor_mapa_ = new EditorMapaModule(bridge_);

  // Modulos com muito conteudo ganham scroll no painel estreito.
  auto scrollable = [](QWidget * w) {
      auto * sc = new QScrollArea();
      sc->setWidgetResizable(true);
      sc->setFrameShape(QFrame::NoFrame);
      sc->setWidget(w);
      return sc;
    };

  painel_->addWidget(scrollable(inicio_));       // 0
  painel_->addWidget(buildRoboPanel());          // 1
  painel_->addWidget(scrollable(navegacao_));    // 2
  painel_->addWidget(scrollable(docking_));      // 3
  painel_->addWidget(scrollable(localizacao_));  // 4
  painel_->addWidget(scrollable(mapas_));        // 5
  painel_->addWidget(scrollable(mapeamento_));   // 6
  painel_->addWidget(scrollable(teleop_));       // 7
  painel_->addWidget(scrollable(service_areas_));// 8
  painel_->addWidget(scrollable(waypoints_));    // 9
  painel_->addWidget(editor_mapa_);              // 10 (tem canvas proprio)

  auto * central = new QWidget();
  auto * layout = new QHBoxLayout(central);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(buildSidebar());
  layout->addWidget(rviz_, 1);        // mapa SEMPRE visivel
  layout->addWidget(painel_);

  // Painel de logs (oculto por padrao — /rosout dos nos originais).
  auto * logs = new QPlainTextEdit();
  logs->setObjectName("painelLogs");
  logs->setReadOnly(true);
  logs->setMaximumBlockCount(500);
  logs->setFixedHeight(180);
  logs->hide();
  auto * centralComLogs = new QWidget();
  auto * vlayout = new QVBoxLayout(centralComLogs);
  vlayout->setContentsMargins(0, 0, 0, 0);
  vlayout->setSpacing(0);
  vlayout->addWidget(central, 1);
  vlayout->addWidget(logs);
  setCentralWidget(centralComLogs);
  connect(
    bridge_, &RosBridge::logLine, this,
    [logs](const QString & l) {logs->appendPlainText(l);});

  // Barra de estado: estado + bateria + logs.
  state_label_ = new QLabel(StateMachine::label(StateMachine::State::OFFLINE));
  state_label_->setObjectName("estadoAtual");
  statusBar()->addPermanentWidget(state_label_);

  auto * bateria = new QLabel("Bateria: —");
  bateria->setObjectName("bateriaLabel");
  statusBar()->addPermanentWidget(bateria);
  connect(
    bridge_, &RosBridge::diagnosticsUpdated, this,
    [bateria](const QVector<ComponentHealth> & health) {
      for (const auto & c : health) {
        if (c.name == "caramelo/bateria") {
          const QString pct = c.values.value("percentual", "");
          bateria->setText(pct.isEmpty() ? "Bateria: —" : "Bateria: " + pct + "%");
        }
      }
    });

  auto * logsBtn = new QPushButton("Logs");
  logsBtn->setCheckable(true);
  connect(logsBtn, &QPushButton::toggled, this, [logs](bool on) {logs->setVisible(on);});
  statusBar()->addPermanentWidget(logsBtn);

  // Fade animado ao trocar o painel de funcoes (sem GL ali — seguro).
  connect(
    painel_, &QStackedWidget::currentChanged, this, [this](int) {
      QWidget * w = painel_->currentWidget();
      auto * eff = new QGraphicsOpacityEffect(w);
      w->setGraphicsEffect(eff);
      auto * anim = new QPropertyAnimation(eff, "opacity");
      anim->setDuration(200);
      anim->setStartValue(0.0);
      anim->setEndValue(1.0);
      connect(anim, &QPropertyAnimation::finished, w, [w]() {w->setGraphicsEffect(nullptr);});
      anim->start(QAbstractAnimation::DeleteWhenStopped);
    });

  // Ligacoes ROS -> UI.
  connect(bridge_, &RosBridge::diagnosticsUpdated, inicio_, &InicioModule::onDiagnostics);
  connect(bridge_, &RosBridge::diagnosticsUpdated, state_, &StateMachine::updateFromHealth);
  connect(
    state_, &StateMachine::stateChanged, this,
    [this](int, const QString & label) {
      state_label_->setText("Estado: " + label);
    });

  // Fixed frame automatico: usa map quando existir (localizado/mapeando);
  // senao cai para odom — assim o ROBO SEMPRE APARECE, mesmo sem mapa
  // (a Raspberry/sim esta sempre publicando TF odom->base).
  auto * frameTimer = new QTimer(this);
  connect(
    frameTimer, &QTimer::timeout, this, [this]() {
      rviz_->setFixedFrame(bridge_->bestFixedFrame());
    });
  frameTimer->start(2000);

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
  sidebar_->setFixedWidth(200);

  addModule("Inicio", 0, true);
  addModule("Mapa & Camadas", 1, true);
  addModule("Navegacao", 2, true);
  addModule("Docking", 3, true);
  addModule("Localizacao", 4, true);
  addModule("Mapas", 5, true);
  addModule("Mapeamento", 6, true);
  addModule("Teleop", 7, true);
  addModule("Service Areas", 8, true);
  addModule("Waypoints", 9, true);
  addModule("Editor de Mapa", 10, true);
  addModule("Testes", -1, false);
  addModule("Simulacao", -1, false);

  connect(
    sidebar_, &QListWidget::currentItemChanged, this,
    [this](QListWidgetItem * current, QListWidgetItem *) {
      if (!current) {
        return;
      }
      const int panel = current->data(Qt::UserRole).toInt();
      if (panel >= 0) {
        painel_->setCurrentIndex(panel);
      }
    });

  sidebar_->setCurrentRow(0);
  return sidebar_;
}

QWidget * MainWindow::buildRoboPanel()
{
  auto * side = new QWidget();
  auto * sideLayout = new QVBoxLayout(side);

  auto * layersTitle = new QLabel("Camadas");
  layersTitle->setObjectName("tituloModulo");
  sideLayout->addWidget(layersTitle);
  auto * layersBox = new QVBoxLayout();
  sideLayout->addLayout(layersBox);
  connect(
    rviz_, &RVizFrame::pronto, this, [this, layersBox]() {
      for (const QString & name : rviz_->layerNames()) {
        auto * cb = new QCheckBox(name);
        cb->setChecked(rviz_->isLayerEnabled(name));
        connect(
          cb, &QCheckBox::toggled, this,
          [this, name](bool on) {rviz_->setLayerEnabled(name, on);});
        layersBox->addWidget(cb);
      }
    });

  sideLayout->addSpacing(12);
  auto * toolsTitle = new QLabel("Ferramentas");
  toolsTitle->setObjectName("tituloModulo");
  sideLayout->addWidget(toolsTitle);
  auto addToolBtn = [this, sideLayout](const QString & text, const QString & key) {
      auto * b = new QPushButton(text);
      connect(b, &QPushButton::clicked, this, [this, key]() {rviz_->activateTool(key);});
      sideLayout->addWidget(b);
    };
  addToolBtn("Interagir (arrastar itens)", "interact");
  addToolBtn("Mover camera", "move");
  addToolBtn("Definir Goal", "goal");

  // Localizacao rapida na tela do mapa: posicionar -> arrastar -> confirmar.
  sideLayout->addSpacing(12);
  auto * locTitle = new QLabel("Localizacao");
  locTitle->setObjectName("tituloModulo");
  sideLayout->addWidget(locTitle);
  auto * ml = bridge_->manualLocalization();
  auto * locIniciar = new QPushButton("Posicionar robo");
  connect(
    locIniciar, &QPushButton::clicked, this, [this, ml]() {
      ml->start();
      rviz_->activateTool("interact");
    });
  sideLayout->addWidget(locIniciar);
  auto * locConfirmar = new QPushButton("Confirmar pose");
  locConfirmar->setObjectName("acaoPrimaria");
  connect(
    locConfirmar, &QPushButton::clicked, this, [this, ml]() {
      ml->confirm();
      rviz_->activateTool("move");
    });
  sideLayout->addWidget(locConfirmar);
  auto * locCancelar = new QPushButton("Cancelar");
  connect(
    locCancelar, &QPushButton::clicked, this, [this, ml]() {
      ml->cancel();
      rviz_->activateTool("move");
    });
  sideLayout->addWidget(locCancelar);
  sideLayout->addStretch();

  auto * sc = new QScrollArea();
  sc->setWidgetResizable(true);
  sc->setFrameShape(QFrame::NoFrame);
  sc->setWidget(side);
  return sc;
}

void MainWindow::addModule(const QString & nome, int panelIndex, bool enabled)
{
  auto * item = new QListWidgetItem(nome, sidebar_);
  item->setData(Qt::UserRole, panelIndex);
  item->setSizeHint(QSize(0, 44));
  if (!enabled) {
    item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
  }
}
