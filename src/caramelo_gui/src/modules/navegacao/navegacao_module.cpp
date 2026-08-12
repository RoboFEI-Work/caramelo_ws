#include "modules/navegacao/navegacao_module.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"
#include "widgets/seletor_arena.hpp"
#include "core/launch_runner.hpp"

NavegacaoModule::NavegacaoModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  runner_ = new LaunchRunner(this);

  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Navegacao");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  // --- Ligar o stack de navegacao (Nav2 + AMCL) direto da GUI ---
  auto * form = new QFormLayout();
  map_name_ = new SeletorArena(bridge);
  com_docking_ = new QCheckBox("Com docking server");
  // Docking validado no robo real (2026-07-21) e default true no bringup:
  // a GUI nasce coerente com a operacao validada.
  com_docking_->setChecked(true);
  auto * sim_time = new QCheckBox("Relogio da simulacao (Gazebo)");
  form->addRow("Mapa:", map_name_);
  form->addRow("", com_docking_);
  form->addRow("", sim_time);
  layout->addLayout(form);

  ligar_ = new QPushButton("Ligar navegacao (Nav2)");
  ligar_->setObjectName("acaoPrimaria");

  // A navegacao normalmente ja' sobe com o robo (caramelo.launch.py). Este
  // modulo so' acompanhava o processo que ELE mesmo tivesse criado, entao dizia
  // "Navegacao desligada" com o Nav2 no ar -- e o operador concluia, com razao,
  // que a navegacao nao tinha subido. Pior: apertar o botao criaria um SEGUNDO
  // Nav2, e os dois disputariam o controle da base.
  auto * vigia = new QTimer(this);
  connect(
    vigia, &QTimer::timeout, this, [this]() {
      const bool nossa = runner_->isRunning();
      const bool no_ar = bridge_->noAtivo("bt_navigator");
      if (nossa) {
        return;   // quem ligou fomos nos; o LaunchRunner ja' cuida do rotulo
      }
      if (no_ar) {
        ligar_->setEnabled(false);
        ligar_->setText("Navegacao ja esta no ar");
        status_->setText(
          "A navegacao subiu junto com o robo. Nao ha nada a ligar aqui — "
          "ligar de novo criaria um segundo Nav2 disputando a base.");
      } else {
        ligar_->setEnabled(true);
        ligar_->setText("Ligar navegacao (Nav2)");
      }
    });
  vigia->start(2000);
  connect(
    ligar_, &QPushButton::clicked, this, [this, sim_time]() {
      if (runner_->isRunning()) {
        runner_->stop();
      } else {
        QStringList args;
        args << QString("map_name:=%1").arg(map_name_->arena().trimmed())
             << "use_rviz:=false"
             << QString("use_docking:=%1")
          .arg(com_docking_->isChecked() ? "true" : "false")
             << QString("use_sim_time:=%1")
          .arg(sim_time->isChecked() ? "true" : "false");
        runner_->start("caramelo_navigation", "bringup.launch.py", args);
      }
    });
  layout->addWidget(ligar_);
  connect(
    runner_, &LaunchRunner::started, this, [this]() {
      ligar_->setText("Desligar navegacao");
      status_->setText("Nav2 subindo... aguarde os cartoes do Inicio ficarem verdes.");
    });
  connect(
    runner_, &LaunchRunner::stopped, this, [this](int) {
      ligar_->setText("Ligar navegacao (Nav2)");
      status_->setText("Navegacao desligada.");
    });

  auto * dica = new QLabel(
    "Com a navegacao ligada: use \"Definir Goal\" (painel Mapa & Camadas) e "
    "clique no mapa para enviar uma meta. O status aparece aqui.");
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
