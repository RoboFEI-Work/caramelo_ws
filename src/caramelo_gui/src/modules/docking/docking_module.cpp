#include "modules/docking/docking_module.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"
#include "widgets/seletor_arena.hpp"

DockingModule::DockingModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Docking");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * form = new QFormLayout();
  dock_id_ = new QLineEdit("WS1");
  map_name_ = new SeletorArena(bridge);
  refine_ = new QCheckBox("Refinar alinhamento com LiDAR");
  // Default LIGADO (2026-08-03): politica unica com o bt_yaml_executor —
  // alinhar sem refino herda o offset do AMCL na hora critica.
  refine_->setChecked(true);
  form->addRow("Dock:", dock_id_);
  form->addRow("Mapa:", map_name_);
  form->addRow("", refine_);
  layout->addLayout(form);

  auto * dockBtn = new QPushButton("Dockar");
  connect(
    dockBtn, &QPushButton::clicked, this,
    [this]() {bridge_->sendDock(dock_id_->text());});
  layout->addWidget(dockBtn);

  auto * alignBtn = new QPushButton("Alinhamento fino (holonomico)");
  connect(
    alignBtn, &QPushButton::clicked, this,
    [this]() {
      bridge_->sendAlign(dock_id_->text(), map_name_->arena(), refine_->isChecked());
    });
  layout->addWidget(alignBtn);

  auto * undockBtn = new QPushButton("Undock");
  connect(
    undockBtn, &QPushButton::clicked, this,
    // Tipo explicito: com "" o opennav_docking so resolve se lembrar do ultimo
    // dock DA MESMA sessao — apos reiniciar a nav o undock falhava.
    [this]() {bridge_->sendUndock("caramelo_front_dock");});
  layout->addWidget(undockBtn);

  auto * saveBtn = new QPushButton("Salvar pose atual como este dock");
  connect(
    saveBtn, &QPushButton::clicked, this,
    [this]() {bridge_->saveDockPose(dock_id_->text());});
  layout->addWidget(saveBtn);

  status_ = new QLabel("Pronto.");
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);
  layout->addStretch();

  connect(
    bridge_, &RosBridge::dockStatus, this,
    [this](const QString & m) {status_->setText(m);});
  connect(
    bridge_, &RosBridge::dockResult, this,
    [this](bool ok, const QString & m) {
      status_->setText((ok ? "OK: " : "Falha: ") + m);
    });
}
