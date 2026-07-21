#include "modules/waypoints/waypoints_module.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"
#include "bridge/waypoint_manager.hpp"
#include "modules/mapas/mapas_module.hpp"

WaypointsModule::WaypointsModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Waypoints");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * form = new QFormLayout();
  map_name_ = new QLineEdit("arena2_520");
  nome_ = new QLineEdit("WP1");
  form->addRow("Mapa:", map_name_);
  form->addRow("Nome do waypoint:", nome_);
  layout->addLayout(form);

  auto * wm = bridge_->waypoints();

  auto * botoes = new QHBoxLayout();
  auto addBtn = [this, botoes](const QString & texto, auto slot) {
      auto * b = new QPushButton(texto);
      connect(b, &QPushButton::clicked, this, slot);
      botoes->addWidget(b);
      return b;
    };

  addBtn("Novo (pose do robo)", [this, wm]() {wm->add(nome_->text().trimmed());});
  addBtn(
    "Remover", [this, wm]() {
      if (auto * item = lista_->currentItem()) {
        wm->remove(item->text());
      }
    });
  addBtn("Carregar", [this, wm]() {wm->load(mapDir());});
  addBtn("Salvar", [this, wm]() {wm->save(mapDir());});
  layout->addLayout(botoes);

  lista_ = new QListWidget();
  layout->addWidget(lista_, 1);

  auto * acoes = new QHBoxLayout();
  auto * ir = new QPushButton("Ir ate o selecionado");
  ir->setObjectName("acaoPrimaria");
  connect(
    ir, &QPushButton::clicked, this, [this, wm]() {
      auto * item = lista_->currentItem();
      if (!item) {return;}
      double x, y, yaw;
      if (wm->pose(item->text(), x, y, yaw)) {
        bridge_->goTo(x, y, yaw);
      }
    });
  acoes->addWidget(ir);

  auto * seguir = new QPushButton("Seguir todos (em ordem)");
  connect(
    seguir, &QPushButton::clicked, this,
    [this, wm]() {bridge_->followWaypoints(wm->orderedPoses());});
  acoes->addWidget(seguir);
  layout->addLayout(acoes);

  status_ = new QLabel("Pronto. Arraste os marcadores na pagina Robo para ajustar.");
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  connect(
    wm, &WaypointManager::waypointsChanged, this,
    [this](const QStringList & nomes) {
      lista_->clear();
      lista_->addItems(nomes);
    });
  connect(
    wm, &WaypointManager::status, this,
    [this](const QString & m) {status_->setText(m);});
  connect(
    bridge_, &RosBridge::navStatus, this,
    [this](const QString & m) {status_->setText(m);});
}

QString WaypointsModule::mapDir() const
{
  return MapasModule::mapsDir() + "/" + map_name_->text().trimmed();
}
