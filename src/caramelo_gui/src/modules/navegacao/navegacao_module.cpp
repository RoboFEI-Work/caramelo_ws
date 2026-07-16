#include "modules/navegacao/navegacao_module.hpp"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"

NavegacaoModule::NavegacaoModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Navegacao");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * dica = new QLabel(
    "Use a ferramenta \"Definir Goal\" na pagina Robo e clique no mapa para "
    "enviar uma meta. O status aparece aqui.");
  dica->setObjectName("msgCartao");
  dica->setWordWrap(true);
  layout->addWidget(dica);

  status_ = new QLabel("Aguardando meta...");
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  auto * cancelar = new QPushButton("Cancelar navegacao");
  connect(cancelar, &QPushButton::clicked, this, [this]() {bridge_->cancelNav();});
  layout->addWidget(cancelar);

  auto * limpar = new QPushButton("Limpar costmaps");
  connect(limpar, &QPushButton::clicked, this, [this]() {bridge_->clearCostmaps();});
  layout->addWidget(limpar);

  layout->addStretch();

  connect(
    bridge_, &RosBridge::navStatus, this,
    [this](const QString & m) {status_->setText(m);});
  connect(
    bridge_, &RosBridge::navResult, this,
    [this](bool ok, const QString & m) {
      status_->setText((ok ? "OK: " : "Falha: ") + m);
    });
}
