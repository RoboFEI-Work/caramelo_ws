#include "core/rviz_frame.hpp"

#include <QApplication>
#include <QTimer>
#include <QVBoxLayout>

#include "rviz_common/display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_rendering/render_window.hpp"

using rviz_common::Display;

RVizFrame::RVizFrame(QWidget * parent)
: QWidget(parent)
{
  ros_node_ =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("caramelo_gui_rviz");

  render_panel_ = new rviz_common::RenderPanel(this);
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(render_panel_);
  // O resto da inicializacao acontece no primeiro showEvent (widget visivel),
  // senao o OGRE cria o contexto sem drawable valido e o render quebra.
}

RVizFrame::~RVizFrame() = default;

void RVizFrame::showEvent(QShowEvent * event)
{
  QWidget::showEvent(event);
  if (!initialized_) {
    initialized_ = true;
    // NAO inicializar dentro do showEvent: neste momento a janela ainda nao
    // esta exposta para o OpenGL (segfault no arranque). Adia para o loop de
    // eventos, com uma folga para a janela pintar de fato.
    QTimer::singleShot(300, this, [this]() {initializeRViz();});
  }
}

void RVizFrame::initializeRViz()
{
  // Sequencia de init do embedding (ref. mjeronimo/rviz_embed_test). Os
  // processEvents deixam o Qt materializar o widget antes do contexto OGRE.
  QApplication::processEvents();
  render_panel_->getRenderWindow()->initialize();

  auto raw_node = ros_node_->get_raw_node();
  manager_ = new rviz_common::VisualizationManager(
    render_panel_, ros_node_, this, raw_node->get_clock());
  render_panel_->initialize(manager_);
  QApplication::processEvents();
  manager_->initialize();

  manager_->setFixedFrame("map");
  createDisplays();
  setupTools();

  // Mapa 2D visto de CIMA (ortografico): e' a vista dos apps comerciais e o
  // que torna o arrasto de markers no plano natural (sem perspectiva 3D).
  auto * vm = manager_->getViewManager();
  if (vm) {
    vm->setCurrentViewControllerType("rviz_default_plugins/TopDownOrtho");
    if (auto * vc = vm->getCurrent()) {
      if (auto * scale = vc->subProp("Scale")) {
        scale->setValue(55.0);
      }
    }
  }

  manager_->startUpdate();
  emit pronto();
}

// --- WindowManagerInterface ---
QWidget * RVizFrame::getParentWindow()
{
  return this;
}

rviz_common::PanelDockWidget * RVizFrame::addPane(
  const QString &, QWidget *, Qt::DockWidgetArea, bool)
{
  // A GUI nao usa os docks internos do RViz; painel proprio cuida da UI.
  return nullptr;
}

void RVizFrame::setStatus(const QString & message)
{
  emit statusMessage(message);
}

// --- Displays / layers ---
void RVizFrame::createDisplays()
{
  auto add = [this](
    const QString & cls, const QString & name, bool enabled,
    const QString & topic_prop, const QString & topic) {
      Display * d = manager_->createDisplay(cls, name, enabled);
      if (d && !topic_prop.isEmpty()) {
        auto * prop = d->subProp(topic_prop);
        if (prop) {
          prop->setValue(topic);
        }
      }
      displays_.insert(name, d);
    };

  add("rviz_default_plugins/Grid", "Grade", true, "", "");
  // RobotModel SEMPRE ligado (a Raspberry/sim esta sempre publicando o
  // robot_description). Requisito: o ambiente da GUI deve resolver o mesmo
  // caramelo_description do robot_description publicado (sourcar o workspace
  // certo por ultimo), senao as malhas nao carregam.
  add("rviz_default_plugins/RobotModel", "Robo", true, "Description Topic", "/robot_description");
  add("rviz_default_plugins/TF", "TF", false, "", "");
  add("rviz_default_plugins/Map", "Mapa", true, "Topic", "/map");
  add("rviz_default_plugins/LaserScan", "Laser", true, "Topic", "/scan");
  add("rviz_default_plugins/Map", "Costmap Global", false, "Topic", "/global_costmap/costmap");
  add("rviz_default_plugins/Map", "Costmap Local", false, "Topic", "/local_costmap/costmap");
  add("rviz_default_plugins/Path", "Caminho Global", true, "Topic", "/plan");
  add("rviz_default_plugins/Path", "Caminho Local", true, "Topic", "/local_plan");
  add("rviz_default_plugins/MarkerArray", "Service Areas", true, "Topic",
    "/caramelo/service_areas/markers");

  // Localizacao manual assistida: fantasma arrastavel + scan re-projetado.
  add("rviz_default_plugins/InteractiveMarkers", "Fantasma", true,
    "Interactive Markers Namespace", "/localizacao_manual");
  add("rviz_default_plugins/Marker", "Scan Fantasma", true, "Topic",
    "/localizacao_manual/scan_preview");

  // Waypoints arrastaveis (modulo Waypoints).
  add("rviz_default_plugins/InteractiveMarkers", "Waypoints", true,
    "Interactive Markers Namespace", "/waypoints");
}

QStringList RVizFrame::layerNames() const
{
  return displays_.keys();
}

void RVizFrame::setLayerEnabled(const QString & name, bool enabled)
{
  auto it = displays_.find(name);
  if (it != displays_.end() && it.value()) {
    it.value()->setEnabled(enabled);
  }
}

bool RVizFrame::isLayerEnabled(const QString & name) const
{
  auto it = displays_.find(name);
  return it != displays_.end() && it.value() && it.value()->isEnabled();
}

// --- Tools ---
void RVizFrame::setupTools()
{
  auto * tm = manager_->getToolManager();
  tools_.insert("interact", tm->addTool("rviz_default_plugins/Interact"));
  tools_.insert("move", tm->addTool("rviz_default_plugins/MoveCamera"));
  // 2D Pose Estimate -> publica /initialpose ; 2D Goal -> topico PRIVADO da
  // GUI (/caramelo_gui/goal_pose): o bt_navigator tambem assina /goal_pose e
  // com o topico compartilhado cada goal disparava DUAS navegacoes.
  tools_.insert("initial_pose", tm->addTool("rviz_default_plugins/SetInitialPose"));
  auto * goal_tool = tm->addTool("rviz_default_plugins/SetGoal");
  if (goal_tool) {
    if (auto * topic_prop = goal_tool->getPropertyContainer()->subProp("Topic")) {
      topic_prop->setValue("/caramelo_gui/goal_pose");
    }
  }
  tools_.insert("goal", goal_tool);
  if (tools_.value("move")) {
    tm->setCurrentTool(tools_.value("move"));
  }
}

void RVizFrame::activateTool(const QString & key)
{
  auto it = tools_.find(key);
  if (it != tools_.end() && it.value()) {
    manager_->getToolManager()->setCurrentTool(it.value());
  }
}

void RVizFrame::setFixedFrame(const QString & frame)
{
  if (manager_) {
    manager_->setFixedFrame(frame);
  }
}
