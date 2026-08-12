#include "modules/service_areas/service_areas_module.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"
#include "widgets/seletor_arena.hpp"

ServiceAreasModule::ServiceAreasModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Service Areas");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * dica = new QLabel(
    "Posicione o robo parado em frente a estacao (a pose salva e' a pose FINAL "
    "do robo, nao o centro da mesa) e clique em Salvar pose atual.");
  dica->setObjectName("msgCartao");
  dica->setWordWrap(true);
  layout->addWidget(dica);

  auto * form = new QFormLayout();
  map_name_ = new SeletorArena(bridge);
  area_id_ = new QLineEdit("WS1");
  tipo_ = new QComboBox();
  // Tipos como strings de configuracao (R6) — nomenclatura do rulebook.
  tipo_->addItems(
    {"workstation", "shelf", "precision_placement", "rotating_table",
      "start", "finish"});
  form->addRow("Mapa:", map_name_);
  form->addRow("Nome (WS1, SH1...):", area_id_);
  form->addRow("Tipo:", tipo_);
  layout->addLayout(form);

  auto * salvar = new QPushButton("Salvar pose atual");
  salvar->setObjectName("acaoPrimaria");
  connect(
    salvar, &QPushButton::clicked, this, [this]() {
      bridge_->saveServiceAreaPose(
        map_name_->arena().trimmed(), area_id_->text().trimmed(),
        tipo_->currentText());
    });
  layout->addWidget(salvar);

  auto * botoes = new QHBoxLayout();
  auto * atualizar = new QPushButton("Listar areas");
  connect(
    atualizar, &QPushButton::clicked, this,
    [this]() {bridge_->listServiceAreas(map_name_->arena().trimmed());});
  botoes->addWidget(atualizar);

  auto * validar = new QPushButton("Validar (+sync docking)");
  connect(
    validar, &QPushButton::clicked, this,
    [this]() {bridge_->validateServiceAreas(map_name_->arena().trimmed());});
  botoes->addWidget(validar);
  layout->addLayout(botoes);

  lista_ = new QListWidget();
  layout->addWidget(lista_, 1);

  status_ = new QLabel("Pronto.");
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  connect(
    bridge_, &RosBridge::serviceAreasListed, this,
    [this](const QStringList & linhas) {
      lista_->clear();
      lista_->addItems(linhas);
    });
  connect(
    bridge_, &RosBridge::serviceAreaStatus, this,
    [this](bool ok, const QString & m) {
      status_->setText((ok ? "" : "Falha: ") + m);
    });
}
