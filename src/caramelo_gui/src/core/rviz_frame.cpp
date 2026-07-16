#include "core/rviz_frame.hpp"

#include <QApplication>
#include <QMenuBar>
#include <QStatusBar>
#include <QVBoxLayout>

#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

RVizFrame::RVizFrame(const QString & config_path, QWidget * parent)
: QWidget(parent)
{
  // Node dedicado do RViz (separado do node do RosBridge). O RViz o "spina"
  // pelo proprio timer de update dentro do loop de eventos Qt.
  ros_node_ =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("caramelo_gui_rviz");

  frame_ = new rviz_common::VisualizationFrame(ros_node_);
  frame_->setApp(qApp);          // obrigatorio logo apos construir
  frame_->initialize(ros_node_);
  if (!config_path.isEmpty()) {
    frame_->loadDisplayConfig(config_path);
  }

  // Embutido: escondemos a barra de menu/status do proprio RViz para um visual limpo.
  if (frame_->menuBar()) {
    frame_->menuBar()->hide();
  }
  if (frame_->statusBar()) {
    frame_->statusBar()->hide();
  }

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(frame_);
}

RVizFrame::~RVizFrame() = default;
