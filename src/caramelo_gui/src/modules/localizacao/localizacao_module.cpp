#include "modules/localizacao/localizacao_module.hpp"

#include <cmath>

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "bridge/manual_localization.hpp"
#include "bridge/ros_bridge.hpp"

LocalizacaoModule::LocalizacaoModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Localizacao manual");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * dica = new QLabel(
    "1. Clique em Iniciar: um robo-fantasma laranja aparece no mapa (pagina Robo).\n"
    "2. Arraste e gire o fantasma; o scan verde acompanha em tempo real.\n"
    "3. Quando o scan casar com as paredes do mapa, clique em Confirmar.");
  dica->setObjectName("msgCartao");
  dica->setWordWrap(true);
  layout->addWidget(dica);

  auto * ml = bridge_->manualLocalization();

  auto * iniciar = new QPushButton("Iniciar localizacao manual");
  connect(iniciar, &QPushButton::clicked, this, [ml]() {ml->start();});
  layout->addWidget(iniciar);

  auto * confirmar = new QPushButton("Confirmar pose");
  confirmar->setObjectName("acaoPrimaria");
  connect(confirmar, &QPushButton::clicked, this, [ml]() {ml->confirm();});
  layout->addWidget(confirmar);

  auto * cancelar = new QPushButton("Cancelar");
  connect(cancelar, &QPushButton::clicked, this, [ml]() {ml->cancel();});
  layout->addWidget(cancelar);

  pose_ = new QLabel("Fantasma: —");
  pose_->setObjectName("msgCartao");
  layout->addWidget(pose_);

  status_ = new QLabel("Pronto.");
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);
  layout->addStretch();

  connect(
    ml, &ManualLocalization::stateChanged, this,
    [this](const QString & m) {status_->setText(m);});
  connect(
    ml, &ManualLocalization::ghostMoved, this,
    [this](double x, double y, double yaw) {
      pose_->setText(
        QString("Fantasma: x=%1 m  y=%2 m  yaw=%3°")
        .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2)
        .arg(yaw * 180.0 / M_PI, 0, 'f', 1));
    });
}
