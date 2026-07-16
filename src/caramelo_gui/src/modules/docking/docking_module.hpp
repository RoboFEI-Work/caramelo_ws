#pragma once

// Modulo Docking: reaproveita as actions /dock_robot, /undock_robot e
// align_to_dock (alinhamento fino holonomico), e permite salvar a pose atual
// como dock via /save_dock_pose. Criacao por Interactive Marker no mapa entra
// numa fatia futura.

#include <QWidget>

class RosBridge;
class QLabel;
class QLineEdit;
class QCheckBox;

class DockingModule : public QWidget
{
  Q_OBJECT

public:
  explicit DockingModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  RosBridge * bridge_;
  QLineEdit * dock_id_ = nullptr;
  QLineEdit * map_name_ = nullptr;
  QCheckBox * refine_ = nullptr;
  QLabel * status_ = nullptr;
};
