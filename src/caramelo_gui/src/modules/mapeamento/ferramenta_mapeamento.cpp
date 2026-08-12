#include "modules/mapeamento/ferramenta_mapeamento.hpp"

#include <iterator>

#include <QComboBox>
#include <QDir>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>

#include "bridge/robot_state.hpp"
#include "bridge/ros_bridge.hpp"
#include "bridge/waypoint_manager.hpp"
#include "core/launch_runner.hpp"
#include "modules/editor_mapa/editor_mapa_module.hpp"
#include "modules/mapas/mapas_module.hpp"

namespace
{

// Os tres tipos de ponto que um mapa guarda. Sao a mesma operacao — gravar a
// pose atual do robo com um nome —, mudam so' o arquivo de destino e o para que
// servem. O texto de ajuda existe porque "service area" nao significa nada para
// quem nunca leu o regulamento.
struct TipoDePonto
{
  const char * rotulo;
  const char * ajuda;
};

const TipoDePonto kTipos[] = {
  {"Waypoint",
    "Um lugar qualquer para onde o robo pode ser mandado. Fica em waypoints.yaml."},
  {"Dock (estacao)",
    "Uma estacao em que o robo encosta para trabalhar. Grava a pose de docking; "
    "sem ela a missao aborta ao chegar nesta estacao."},
  {"Service area (mesa)",
    "O ponto em que o robo PARA de frente para a mesa — nao e' a pose da mesa. "
    "E' daqui que o braco alcanca os objetos."},
};

const char * kTiposDeArea[] = {"workstation", "shelf", "precision_placement",
  "rotating_table", "start", "finish"};

}  // namespace

FerramentaMapeamento::FerramentaMapeamento(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  slam_ = new LaunchRunner(this);
  teleop_ = new LaunchRunner(this);

  auto * layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  // Coluna de passos: numerada, porque a ordem importa. Nao da para marcar um
  // dock antes de existir mapa, nem conferir o que nao foi salvo.
  passos_ = new QListWidget();
  passos_->setObjectName("abasContexto");
  passos_->setFixedWidth(210);
  for (const QString & p : {
      QString("1.  Criar o mapa"),
      QString("2.  Marcar pontos"),
      QString("3.  Paredes virtuais"),
      QString("4.  Conferir")})
  {
    auto * item = new QListWidgetItem(p, passos_);
    item->setSizeHint(QSize(0, 54));
  }
  layout->addWidget(passos_);

  paginas_ = new QStackedWidget();
  paginas_->addWidget(construirPassoCriar());
  paginas_->addWidget(construirPassoPontos());
  paginas_->addWidget(construirPassoParedes());
  paginas_->addWidget(construirPassoConferir());
  layout->addWidget(paginas_, 1);

  connect(
    passos_, &QListWidget::currentRowChanged, this,
    [this](int row) {if (row >= 0) {irParaPasso(row);}});
  passos_->setCurrentRow(0);
}

QString FerramentaMapeamento::mapaAlvo() const
{
  const QString digitado = nome_do_mapa_->text().trimmed();
  if (!digitado.isEmpty()) {
    return digitado;
  }
  return bridge_->robotState()->mapName();
}

// ---------------------------------------------------------------- passo 1
QWidget * FerramentaMapeamento::construirPassoCriar()
{
  auto * pagina = new QWidget();
  auto * layout = new QVBoxLayout(pagina);
  layout->setContentsMargins(20, 18, 20, 18);

  auto * titulo = new QLabel("1. Criar o mapa");
  titulo->setObjectName("tituloModulo");
  layout->addWidget(titulo);

  auto * explica = new QLabel(
    "Ligue o mapeamento e dirija o robo devagar pelo ambiente, passando por "
    "todos os cantos. O mapa vai se formando na tela ao lado. Quando estiver "
    "completo, de um nome e salve.");
  explica->setWordWrap(true);
  explica->setObjectName("msgCartao");
  layout->addWidget(explica);

  botao_slam_ = new QPushButton("Iniciar mapeamento");
  botao_slam_->setObjectName("acaoPrimaria");
  botao_slam_->setMinimumHeight(46);
  layout->addWidget(botao_slam_);

  botao_teleop_ = new QPushButton("Ligar controle manual");
  botao_teleop_->setMinimumHeight(40);
  layout->addWidget(botao_teleop_);

  auto * grupoSalvar = new QGroupBox("Salvar o mapa");
  auto * form = new QFormLayout(grupoSalvar);
  nome_do_mapa_ = new QLineEdit();
  nome_do_mapa_->setPlaceholderText("ex.: arena_regional");
  form->addRow("Nome:", nome_do_mapa_);
  auto * salvar = new QPushButton("Salvar mapa");
  form->addRow("", salvar);
  layout->addWidget(grupoSalvar);

  status_slam_ = new QLabel("Mapeamento desligado.");
  status_slam_->setObjectName("estadoAtual");
  status_slam_->setWordWrap(true);
  layout->addWidget(status_slam_);
  layout->addStretch();

  connect(
    botao_slam_, &QPushButton::clicked, this, [this]() {
      if (slam_->isRunning()) {
        slam_->stop();
        return;
      }
      // A trava SLAM x AMCL (RK-02) vive no proprio launch: se a localizacao
      // estiver ativa ele aborta. O aviso aparece no status.
      slam_->start("caramelo_mapping", "slam.launch.py", {"use_rviz:=false"});
    });
  connect(
    slam_, &LaunchRunner::started, this, [this]() {
      botao_slam_->setText("Parar mapeamento");
      status_slam_->setText("Mapeando. Dirija o robo devagar pelo ambiente.");
    });
  connect(
    slam_, &LaunchRunner::stopped, this, [this](int) {
      botao_slam_->setText("Iniciar mapeamento");
      status_slam_->setText("Mapeamento desligado.");
    });

  connect(
    botao_teleop_, &QPushButton::clicked, this, [this]() {
      if (teleop_->isRunning()) {
        teleop_->stop();
        botao_teleop_->setText("Ligar controle manual");
        return;
      }
      teleop_->start("caramelo_utils", "teleop.launch.py");
      botao_teleop_->setText("Desligar controle manual");
    });

  connect(
    salvar, &QPushButton::clicked, this, [this]() {
      const QString nome = nome_do_mapa_->text().trimmed();
      if (nome.isEmpty()) {
        status_slam_->setText("De um nome ao mapa antes de salvar.");
        return;
      }
      QDir().mkpath(MapasModule::mapsDir() + "/" + nome);
      bridge_->saveMap(MapasModule::mapsDir(), nome);
    });

  connect(
    bridge_, &RosBridge::mapStatus, this,
    [this](bool ok, const QString & m) {
      status_slam_->setText((ok ? "" : "Falha: ") + m);
    });

  return pagina;
}

// ---------------------------------------------------------------- passo 2
QWidget * FerramentaMapeamento::construirPassoPontos()
{
  auto * pagina = new QWidget();
  auto * layout = new QVBoxLayout(pagina);
  layout->setContentsMargins(20, 18, 20, 18);

  auto * titulo = new QLabel("2. Marcar pontos");
  titulo->setObjectName("tituloModulo");
  layout->addWidget(titulo);

  auto * explica = new QLabel(
    "Leve o robo ate' o lugar — dirigindo ou mandando ele navegar — e grave a "
    "pose com um nome. E' sempre a pose ATUAL do robo que fica gravada.");
  explica->setWordWrap(true);
  explica->setObjectName("msgCartao");
  layout->addWidget(explica);

  auto * form = new QFormLayout();
  tipo_do_ponto_ = new QComboBox();
  for (const auto & t : kTipos) {
    tipo_do_ponto_->addItem(t.rotulo);
  }
  form->addRow("Tipo:", tipo_do_ponto_);

  nome_do_ponto_ = new QLineEdit();
  nome_do_ponto_->setPlaceholderText("ex.: WS1");
  form->addRow("Nome:", nome_do_ponto_);
  layout->addLayout(form);

  ajuda_do_tipo_ = new QLabel(kTipos[0].ajuda);
  ajuda_do_tipo_->setWordWrap(true);
  ajuda_do_tipo_->setObjectName("motivoCartao");
  layout->addWidget(ajuda_do_tipo_);

  auto * gravar = new QPushButton("Gravar aqui (pose atual do robo)");
  gravar->setObjectName("acaoPrimaria");
  gravar->setMinimumHeight(46);
  layout->addWidget(gravar);

  pontos_ = new QListWidget();
  pontos_->setMinimumHeight(150);
  layout->addWidget(pontos_, 1);

  auto * remover = new QPushButton("Remover o selecionado");
  layout->addWidget(remover);

  status_ponto_ = new QLabel("Nenhum ponto gravado nesta sessao.");
  status_ponto_->setObjectName("estadoAtual");
  status_ponto_->setWordWrap(true);
  layout->addWidget(status_ponto_);

  connect(
    tipo_do_ponto_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int i) {
      if (i >= 0 && i < static_cast<int>(std::size(kTipos))) {
        ajuda_do_tipo_->setText(kTipos[i].ajuda);
      }
    });
  connect(gravar, &QPushButton::clicked, this, &FerramentaMapeamento::gravarPonto);

  connect(
    remover, &QPushButton::clicked, this, [this]() {
      auto * item = pontos_->currentItem();
      if (!item) {return;}
      // So' waypoints tem remocao local: dock e service area vivem em YAMLs
      // gerenciados por nos de fora, e apagar por baixo deles causaria
      // divergencia silenciosa entre o arquivo e o que o robo carregou.
      const QString texto = item->text();
      if (!texto.startsWith("Waypoint")) {
        status_ponto_->setText(
          "Por seguranca, remova docks e service areas editando o mapa "
          "(Conferir mostra o que esta salvo).");
        return;
      }
      bridge_->waypoints()->remove(texto.section("  ", 1));
      bridge_->waypoints()->save(MapasModule::mapsDir() + "/" + mapaAlvo());
    });

  connect(
    bridge_->waypoints(), &WaypointManager::status, this,
    [this](const QString & m) {status_ponto_->setText(m);});
  connect(
    bridge_, &RosBridge::dockStatus, this,
    [this](const QString & m) {status_ponto_->setText(m);});
  connect(
    bridge_, &RosBridge::serviceAreaStatus, this,
    [this](bool ok, const QString & m) {
      status_ponto_->setText((ok ? "" : "Falha: ") + m);
    });

  return pagina;
}

void FerramentaMapeamento::gravarPonto()
{
  const QString nome = nome_do_ponto_->text().trimmed();
  if (nome.isEmpty()) {
    status_ponto_->setText("De um nome ao ponto antes de gravar.");
    return;
  }
  const QString mapa = mapaAlvo();
  if (mapa.isEmpty()) {
    status_ponto_->setText(
      "Nenhuma arena ativa. Escolha uma em Mapas, ou salve o mapa no passo 1.");
    return;
  }

  const int tipo = tipo_do_ponto_->currentIndex();
  switch (tipo) {
    case 0: {   // waypoint
        auto * wm = bridge_->waypoints();
        wm->add(nome);
        wm->save(MapasModule::mapsDir() + "/" + mapa);
        pontos_->addItem("Waypoint  " + nome);
        break;
      }
    case 1: {   // dock
        bridge_->saveDockPose(nome);
        pontos_->addItem("Dock  " + nome);
        break;
      }
    default: {  // service area
        // O tipo da area decide o comportamento do robo na missao; perguntar
        // aqui evita um YAML editado a mao depois.
        bool ok = false;
        QStringList opcoes;
        for (const char * t : kTiposDeArea) {
          opcoes << t;
        }
        const QString tipoArea = QInputDialog::getItem(
          this, "Tipo da service area",
          "O que e' este ponto?", opcoes, 0, false, &ok);
        if (!ok) {return;}
        bridge_->saveServiceAreaPose(mapa, nome, tipoArea);
        pontos_->addItem("Service area  " + nome + "  (" + tipoArea + ")");
        break;
      }
  }
  nome_do_ponto_->clear();
}

// ---------------------------------------------------------------- passo 3
QWidget * FerramentaMapeamento::construirPassoParedes()
{
  auto * pagina = new QWidget();
  auto * layout = new QVBoxLayout(pagina);
  layout->setContentsMargins(20, 18, 20, 18);

  auto * titulo = new QLabel("3. Paredes virtuais");
  titulo->setObjectName("tituloModulo");
  layout->addWidget(titulo);

  auto * explica = new QLabel(
    "O LiDAR nao enxerga fita no chao, degrau baixo nem vidro. Onde o robo nao "
    "pode entrar mas nada aparece no mapa, pinte de PAREDE. Onde o mapa marcou "
    "obstaculo que nao existe (uma pessoa que passou durante o mapeamento), "
    "pinte de LIVRE.");
  explica->setWordWrap(true);
  explica->setObjectName("msgCartao");
  layout->addWidget(explica);

  // O editor de bitmap era um modulo solto na sidebar. E' exatamente esta a
  // ferramenta de parede virtual — vive aqui dentro, no passo em que se usa.
  layout->addWidget(new EditorMapaModule(bridge_), 1);
  return pagina;
}

// ---------------------------------------------------------------- passo 4
QWidget * FerramentaMapeamento::construirPassoConferir()
{
  auto * pagina = new QWidget();
  auto * layout = new QVBoxLayout(pagina);
  layout->setContentsMargins(12, 12, 12, 12);

  auto * titulo = new QLabel("4. Conferir");
  titulo->setObjectName("tituloModulo");
  layout->addWidget(titulo);

  preview_ = new MapPreview();
  layout->addWidget(preview_, 1);

  conferencia_ = new QLabel();
  conferencia_->setWordWrap(true);
  conferencia_->setObjectName("motivoCartao");
  layout->addWidget(conferencia_);

  connect(
    preview_, &MapPreview::carregado, this, [this](const MapaCarregado & m) {
      if (!m.valido) {
        conferencia_->setText(m.erro);
        return;
      }
      conferencia_->setText(
        m.avisos.isEmpty() ?
        "Arena completa: nada a apontar." :
        "• " + m.avisos.join("\n• "));
    });

  return pagina;
}

void FerramentaMapeamento::irParaPasso(int passo)
{
  paginas_->setCurrentIndex(passo);

  // Passos 1 e 2 usam o mapa AO VIVO (o robo se movendo). O passo 4 mostra o
  // arquivo salvo, que e' outra coisa: e' onde se descobre que um dock ficou
  // sem pose.
  emit querMapaAoVivo(passo == 0 || passo == 1);

  if (passo == 3) {
    recarregarConferencia();
  }
}

void FerramentaMapeamento::recarregarConferencia()
{
  const QString mapa = mapaAlvo();
  if (mapa.isEmpty()) {
    conferencia_->setText("Nenhuma arena ativa para conferir.");
    return;
  }
  preview_->carregar(MapasModule::mapsDir() + "/" + mapa);
}
