#include "modules/mapeamento/mapeamento_module.hpp"

#include <QDir>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "core/launch_runner.hpp"
#include "bridge/ros_bridge.hpp"
#include "modules/mapas/mapas_module.hpp"

MapeamentoModule::MapeamentoModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  runner_ = new LaunchRunner(this);

  auto * layout = new QVBoxLayout(this);
  auto * title = new QLabel("Mapeamento (SLAM)");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * dica = new QLabel(
    "1. Informe o nome do mapa e clique em Iniciar SLAM (a localizacao nao pode "
    "estar ativa — RK-02).\n"
    "2. Dirija o robo (Teleop) vendo o mapa crescer na pagina Robo (camada Mapa).\n"
    "3. Clique em Salvar mapa ao terminar, depois Parar SLAM.");
  dica->setObjectName("msgCartao");
  dica->setWordWrap(true);
  layout->addWidget(dica);

  auto * form = new QFormLayout();
  nome_ = new QLineEdit("novo_mapa");
  form->addRow("Nome do mapa:", nome_);
  layout->addLayout(form);

  botao_ = new QPushButton("Iniciar SLAM");
  botao_->setObjectName("acaoPrimaria");
  connect(
    botao_, &QPushButton::clicked, this, [this]() {
      if (runner_->isRunning()) {
        runner_->stop();
      } else {
        runner_->start(
          "caramelo_mapping", "slam.launch.py",
          {QString("map_name:=%1").arg(nome_->text().trimmed()), "use_rviz:=false"});
      }
    });
  layout->addWidget(botao_);

  auto * salvar = new QPushButton("Salvar mapa");
  connect(
    salvar, &QPushButton::clicked, this, [this]() {
      const QString nome = nome_->text().trimmed();
      if (nome.isEmpty()) {return;}
      QDir().mkpath(MapasModule::mapsDir() + "/" + nome);
      bridge_->saveMap(MapasModule::mapsDir(), nome);
    });
  layout->addWidget(salvar);

  status_ = new QLabel("Parado.");
  status_->setObjectName("estadoAtual");
  layout->addWidget(status_);

  log_ = new QLabel();
  log_->setObjectName("detalheCartao");
  log_->setWordWrap(true);
  layout->addWidget(log_);
  layout->addStretch();

  connect(
    runner_, &LaunchRunner::started, this, [this]() {
      botao_->setText("Parar SLAM");
      status_->setText("SLAM ATIVO — dirija o robo para mapear.");
    });
  connect(
    runner_, &LaunchRunner::stopped, this, [this](int) {
      botao_->setText("Iniciar SLAM");
      status_->setText("Parado.");
    });
  connect(
    runner_, &LaunchRunner::outputLine, this,
    [this](const QString & l) {log_->setText(l);});
  connect(
    bridge_, &RosBridge::mapStatus, this,
    [this](bool ok, const QString & m) {
      status_->setText((ok ? "" : "Falha: ") + m);
    });
}
