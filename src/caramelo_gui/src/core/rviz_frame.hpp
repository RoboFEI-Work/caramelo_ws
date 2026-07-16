#pragma once

// RViz embutido pelo padrao RenderPanel + VisualizationManager (o "jeito certo"
// de embutir, ref. mjeronimo/rviz_embed_test). Diferente do VisualizationFrame,
// da acesso programatico para: criar displays, LIGAR/DESLIGAR layers
// (Display::setEnabled) e reusar as TOOLs ("2D Pose Estimate" -> /initialpose,
// "2D Goal" -> /goal_pose). A GUI controla o RViz por esta API.

#include <memory>

#include <QMap>
#include <QStringList>
#include <QWidget>

#include "rviz_common/window_manager_interface.hpp"

namespace rviz_common
{
class RenderPanel;
class VisualizationManager;
class Display;
class Tool;
namespace ros_integration
{
class RosNodeAbstraction;
}  // namespace ros_integration
}  // namespace rviz_common

class RVizFrame : public QWidget, public rviz_common::WindowManagerInterface
{
  Q_OBJECT

public:
  explicit RVizFrame(QWidget * parent = nullptr);
  ~RVizFrame() override;

  // --- WindowManagerInterface (minimo para o embedding) ---
  QWidget * getParentWindow() override;
  rviz_common::PanelDockWidget * addPane(
    const QString & name, QWidget * pane, Qt::DockWidgetArea area, bool floating) override;
  void setStatus(const QString & message) override;

  // --- API de controle para a GUI ---
  QStringList layerNames() const;
  void setLayerEnabled(const QString & name, bool enabled);
  bool isLayerEnabled(const QString & name) const;
  void activateTool(const QString & key);   // "interact" | "move" | "initial_pose" | "goal"
  void setFixedFrame(const QString & frame);

signals:
  void statusMessage(const QString & message);

private:
  void createDisplays();
  void setupTools();

  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> ros_node_;
  rviz_common::RenderPanel * render_panel_ = nullptr;
  rviz_common::VisualizationManager * manager_ = nullptr;
  QMap<QString, rviz_common::Display *> displays_;
  QMap<QString, rviz_common::Tool *> tools_;
};
