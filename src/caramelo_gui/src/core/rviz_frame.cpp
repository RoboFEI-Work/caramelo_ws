#include "core/rviz_frame.hpp"

#include <QApplication>
#include <QVBoxLayout>

#include "rviz_common/display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/tool_manager.hpp"
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
    initializeRViz();
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
  // RobotModel comeca DESLIGADO: carregar as malhas STL pesadas derrubou o
  // OGRE embutido quando o pacote de malhas nao casava com o robot_description
  // publicado. Ligue pela camada "Robo" quando o ambiente tiver as malhas.
  add("rviz_default_plugins/RobotModel", "Robo", false, "Description Topic", "/robot_description");
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
  // 2D Pose Estimate -> publica /initialpose ; 2D Goal -> publica /goal_pose.
  tools_.insert("initial_pose", tm->addTool("rviz_default_plugins/SetInitialPose"));
  tools_.insert("goal", tm->addTool("rviz_default_plugins/SetGoal"));
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
