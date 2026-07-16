#include "modules/teleop/teleop_module.hpp"

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "core/launch_runner.hpp"

TeleopModule::TeleopModule(QWidget * parent)
: QWidget(parent)
{
  runner_ = new LaunchRunner(this);

  auto * layout = new QVBoxLayout(this);
  auto * title = new QLabel("Teleop");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * dica = new QLabel(
    "Liga o teleop (joystick + teclado + twist_mux). Enquanto ativo, os comandos "
    "manuais tem prioridade sobre a navegacao no twist_mux.");
  dica->setObjectName("msgCartao");
  dica->setWordWrap(true);
  layout->addWidget(dica);

  botao_ = new QPushButton("Ligar teleop");
  botao_->setObjectName("acaoPrimaria");
  connect(
    botao_, &QPushButton::clicked, this, [this]() {
      if (runner_->isRunning()) {
        runner_->stop();
      } else {
        runner_->start("caramelo_utils", "teleop.launch.py");
      }
    });
  layout->addWidget(botao_);

  status_ = new QLabel("Desligado.");
  status_->setObjectName("estadoAtual");
  layout->addWidget(status_);
  layout->addStretch();

  connect(
    runner_, &LaunchRunner::started, this, [this]() {
      botao_->setText("Desligar teleop");
      status_->setText("Teleop ATIVO.");
    });
  connect(
    runner_, &LaunchRunner::stopped, this, [this](int) {
      botao_->setText("Ligar teleop");
      status_->setText("Desligado.");
    });
}
