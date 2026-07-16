#pragma once

// Envolve o RViz embutido (rviz_common::VisualizationFrame) num QWidget, para
// entrar como painel de visualizacao dentro da GUI. Reaproveita os displays e
// os plugins do Nav2 via um arquivo .rviz.

#include <memory>

#include <QWidget>

namespace rviz_common
{
class VisualizationFrame;
namespace ros_integration
{
class RosNodeAbstraction;
}  // namespace ros_integration
}  // namespace rviz_common

class RVizFrame : public QWidget
{
  Q_OBJECT

public:
  explicit RVizFrame(const QString & config_path, QWidget * parent = nullptr);
  ~RVizFrame() override;

private:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> ros_node_;
  rviz_common::VisualizationFrame * frame_ = nullptr;
};
