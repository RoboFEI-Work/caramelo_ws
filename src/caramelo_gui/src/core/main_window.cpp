#include "core/main_window.hpp"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "bridge/manual_localization.hpp"
#include "bridge/mission_bridge.hpp"
#include "bridge/robot_state.hpp"
#include "bridge/ros_bridge.hpp"
#include "core/context_screen.hpp"
#include "core/home_screen.hpp"
#include "core/rviz_frame.hpp"
#include "core/state_machine.hpp"
#include "modules/competicao/competicao_module.hpp"
#include "modules/docking/docking_module.hpp"
#include "modules/avancado/sensores_module.hpp"
#include "modules/inicio/inicio_module.hpp"
#include "modules/localizacao/localizacao_module.hpp"
#include "modules/mapas/mapas_module.hpp"
#include "modules/mapeamento/ferramenta_mapeamento.hpp"
#include "modules/navegacao/navegacao_module.hpp"
#include "modules/service_areas/service_areas_module.hpp"
#include "modules/teleop/teleop_module.hpp"
#include "modules/waypoints/waypoints_module.hpp"

MainWindow::MainWindow(QWidget * parent)
: QMainWindow(parent)
{
  setWindowTitle("Caramelo");
  resize(1440, 860);

  bridge_ = new RosBridge(this);
  state_ = new StateMachine(this);
  robot_state_ = bridge_->robotState();

  rviz_ = new RVizFrame();
  rviz_->hide();   // a tela inicial nao tem mapa

  telas_ = new QStackedWidget();

  auto * corpo = new QWidget();
  auto * corpoLayout = new QHBoxLayout(corpo);
  corpoLayout->setContentsMargins(0, 0, 0, 0);
  corpoLayout->setSpacing(0);
  corpoLayout->addWidget(rviz_, 1);
  corpoLayout->addWidget(telas_, 1);

  auto * logs = new QPlainTextEdit();
  logs->setObjectName("painelLogs");
  logs->setReadOnly(true);
  logs->setMaximumBlockCount(500);
  logs->setFixedHeight(180);
  logs->hide();
  connect(
    bridge_, &RosBridge::logLine, this,
    [logs](const QString & l) {logs->appendPlainText(l);});

  auto * central = new QWidget();
  auto * layout = new QVBoxLayout(central);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(construirFaixa());
  layout->addWidget(corpo, 1);
  layout->addWidget(logs);
  setCentralWidget(central);

  // O botao de logs mora na faixa, nao numa status bar: a faixa e' a unica
  // regiao permanente da tela, e uma barra de status a mais so' rouba altura.
  connect(logs_btn_, &QPushButton::toggled, this, [logs](bool on) {logs->setVisible(on);});

  construirContextos();

  // --- ligacoes ROS -> UI ---
  connect(bridge_, &RosBridge::diagnosticsUpdated, inicio_, &InicioModule::onDiagnostics);
  connect(bridge_, &RosBridge::diagnosticsUpdated, state_, &StateMachine::updateFromHealth);
  connect(bridge_, &RosBridge::diagnosticsUpdated, home_, &HomeScreen::onDiagnostics);
  connect(
    state_, &StateMachine::stateChanged, this,
    [this](int, const QString & label) {state_label_->setText(label);});

  auto aplicarArena = [this](const QString & nome) {
      arena_label_->setText(nome.isEmpty() ? "Arena: nenhuma" : "Arena: " + nome);
      home_->setMapaAtivo(nome);
    };
  aplicarArena(robot_state_->mapName());
  connect(robot_state_, &RobotState::mapNameChanged, this, aplicarArena);

  // Modo fantasma: esconde o robo e o scan reais enquanto o operador arrasta o
  // fantasma (ficavam sobrepostos e confundiam a localizacao manual).
  connect(
    bridge_->manualLocalization(), &ManualLocalization::activeChanged, this,
    [this](bool active) {
      rviz_->setLayerEnabled("Robo", !active);
      rviz_->setLayerEnabled("Laser", !active);
      rviz_->activateTool(active ? "interact" : "move");
    });

  // Guarda a pose confirmada pelo operador.
  connect(
    bridge_->manualLocalization(), &ManualLocalization::poseConfirmada, this,
    [this](double x, double y, double yaw) {
      robot_state_->setPoseInicial(x, y, yaw);
    });

  // Restaura a ultima posicao conhecida assim que o AMCL aparecer.
  //
  // Sem isto, "o robo subiu" e "o robo esta pronto" sao coisas diferentes: os
  // nos todos no ar, mas nenhuma meta funciona ate' alguem posicionar o
  // fantasma na mao. Publicar cedo demais tambem nao adianta -- o AMCL precisa
  // existir para receber. Por isso o relogio espera ele aparecer, tenta UMA vez
  // e avisa na tela, nunca em silencio: se o robo foi movido enquanto estava
  // desligado, o operador precisa saber que a pose veio da sessao anterior.
  if (robot_state_->temPoseInicial()) {
    auto * restaurar = new QTimer(this);
    restaurar->setInterval(1500);
    connect(
      restaurar, &QTimer::timeout, this, [this, restaurar]() {
        if (!bridge_->noAtivo("amcl")) {
          return;
        }
        restaurar->stop();
        bridge_->publishInitialPose(
          robot_state_->poseX(), robot_state_->poseY(), robot_state_->poseYaw());
        state_label_->setText("Pose restaurada");
        arena_label_->setToolTip(
          "A posicao veio da ultima sessao. Se o robo foi movido enquanto estava "
          "desligado, refaca a localizacao em Operacao.");
      });
    restaurar->start();
  }

  // Fixed frame automatico: usa map quando existir; senao cai para odom, para o
  // robo aparecer mesmo sem mapa.
  auto * frameTimer = new QTimer(this);
  connect(
    frameTimer, &QTimer::timeout, this, [this]() {
      rviz_->setFixedFrame(bridge_->bestFixedFrame());
      home_->setServidorMissao(bridge_->mission()->serverReady());
    });
  frameTimer->start(2000);

  bridge_->start();
  irPara(0);
}

MainWindow::~MainWindow()
{
  if (bridge_) {
    bridge_->stop();
  }
}

QWidget * MainWindow::construirFaixa()
{
  auto * faixa = new QWidget();
  faixa->setObjectName("faixaSuperior");
  faixa->setFixedHeight(58);
  auto * layout = new QHBoxLayout(faixa);
  layout->setContentsMargins(12, 6, 12, 6);

  voltar_ = new QPushButton("< Menu");
  voltar_->setObjectName("botaoVoltar");
  voltar_->setFixedWidth(110);
  connect(voltar_, &QPushButton::clicked, this, [this]() {irPara(0);});
  layout->addWidget(voltar_);

  titulo_ = new QLabel("Caramelo");
  titulo_->setObjectName("tituloFaixa");
  layout->addWidget(titulo_);
  layout->addStretch();

  // Arena e estado ficam SEMPRE visiveis: sao as duas perguntas que o operador
  // faz a cada dois minutos ("qual mapa?" e "da para andar?").
  arena_label_ = new QLabel("Arena: —");
  arena_label_->setObjectName("infoFaixa");
  layout->addWidget(arena_label_);

  state_label_ = new QLabel(StateMachine::label(StateMachine::State::OFFLINE));
  state_label_->setObjectName("estadoFaixa");
  layout->addWidget(state_label_);

  logs_btn_ = new QPushButton("Logs");
  logs_btn_->setCheckable(true);
  logs_btn_->setFixedWidth(80);
  layout->addWidget(logs_btn_);

  return faixa;
}

QWidget * MainWindow::construirPainelDoMapa()
{
  auto * side = new QWidget();
  auto * sideLayout = new QVBoxLayout(side);

  auto * toolsTitle = new QLabel("Ferramentas");
  toolsTitle->setObjectName("tituloModulo");
  sideLayout->addWidget(toolsTitle);

  // A instrucao existe porque um botao que apenas ARMA uma ferramenta nao
  // parece ter feito nada: o operador clicava em "Definir destino", nada
  // acontecia na tela, e a conclusao natural era que o botao estava quebrado.
  auto * instrucao = new QLabel(
    "Escolha uma ferramenta e depois use o mapa ao lado.");
  instrucao->setWordWrap(true);
  instrucao->setObjectName("msgCartao");

  auto addToolBtn =
    [this, sideLayout, instrucao](
    const QString & text, const QString & key, const QString & comoUsar) {
      auto * b = new QPushButton(text);
      connect(
        b, &QPushButton::clicked, this, [this, key, comoUsar, instrucao]() {
          rviz_->activateTool(key);
          instrucao->setText(comoUsar);
        });
      sideLayout->addWidget(b);
    };
  addToolBtn(
    "Definir destino no mapa", "goal",
    "Clique no mapa onde o robo deve chegar e ARRASTE para escolher a direcao "
    "em que ele deve parar. Solte para enviar.");
  addToolBtn("Mover camera", "move", "Arraste o mapa para deslocar a vista.");
  addToolBtn(
    "Interagir (arrastar itens)", "interact",
    "Arraste marcadores do mapa, como o robo-fantasma da localizacao.");
  sideLayout->addWidget(instrucao);

  // O que aconteceu com a meta aparece AQUI, ao lado do botao que a disparou.
  // Antes so' existia na secao Navegar: quem mandava a meta pelo mapa nao via
  // nem "meta aceita" nem "servidor indisponivel".
  auto * retorno = new QLabel("—");
  retorno->setWordWrap(true);
  retorno->setObjectName("estadoAtual");
  connect(bridge_, &RosBridge::navStatus, retorno, &QLabel::setText);
  connect(
    bridge_, &RosBridge::navResult, retorno,
    [retorno](bool ok, const QString & m) {retorno->setText((ok ? "" : "Falha: ") + m);});
  sideLayout->addWidget(retorno);

  sideLayout->addSpacing(12);
  auto * layersTitle = new QLabel("Camadas");
  layersTitle->setObjectName("tituloModulo");
  sideLayout->addWidget(layersTitle);
  auto * layersBox = new QVBoxLayout();
  sideLayout->addLayout(layersBox);
  connect(
    rviz_, &RVizFrame::pronto, this, [this, layersBox]() {
      for (const QString & name : rviz_->layerNames()) {
        auto * cb = new QCheckBox(name);
        cb->setChecked(rviz_->isLayerEnabled(name));
        connect(
          cb, &QCheckBox::toggled, this,
          [this, name](bool on) {rviz_->setLayerEnabled(name, on);});
        layersBox->addWidget(cb);
      }
    });

  sideLayout->addStretch();
  return side;
}

void MainWindow::construirContextos()
{
  home_ = new HomeScreen();
  connect(home_, &HomeScreen::abrirContexto, this, &MainWindow::irPara);
  // "Parar o robo" da barra inferior: cancela a navegacao em voo e aborta a
  // missao. Nao e' E-stop -- o robo nao tem um -- e por isso o rotulo nao
  // promete parada de emergencia.
  connect(
    home_, &HomeScreen::pararRobo, this, [this]() {
      bridge_->cancelNav();
      bridge_->mission()->abort();
    });
  telas_->addWidget(home_);   // 0

  inicio_ = new InicioModule();
  competicao_ = new CompeticaoModule(bridge_);
  navegacao_ = new NavegacaoModule(bridge_);
  docking_ = new DockingModule(bridge_);
  localizacao_ = new LocalizacaoModule(bridge_);
  mapas_ = new MapasModule(bridge_);
  teleop_ = new TeleopModule();
  service_areas_ = new ServiceAreasModule(bridge_);
  waypoints_ = new WaypointsModule(bridge_);
  // MapeamentoModule e EditorMapaModule sairam daqui: viraram passos da
  // FerramentaMapeamento, que os cria por dentro.

  // 1 — Operacao: tudo que se faz com o robo ja' mapeado e localizado.
  // Waypoints entram aqui, e nao no Mapeamento: CRIAR um waypoint e' mapeamento,
  // mas mandar o robo seguir a lista e' operacao.
  auto * operacao = new ContextScreen("Operacao", true);
  operacao->addSecao("Mapa", construirPainelDoMapa());
  operacao->addSecao("Navegar", navegacao_);
  operacao->addSecao("Waypoints", waypoints_);
  operacao->addSecao("Docking", docking_);
  operacao->addSecao("Localizacao", localizacao_);
  operacao->addSecao("Teleop", teleop_);
  telas_->addWidget(operacao);

  // 2 — Competicao: o mapa nao ajuda aqui; o que importa e' o passo da missao.
  auto * competicao = new ContextScreen("Competicao", false);
  competicao->addSecao("Missao", competicao_);
  telas_->addWidget(competicao);

  // 3 — Mapas: escolher a arena. O preview proprio (waypoints, docks, camadas)
  // entra aqui na proxima etapa; por isso ainda nao pede o RViz.
  auto * mapasCtx = new ContextScreen("Mapas", false);
  mapasCtx->addSecao("Arenas", mapas_, /*larguraTotal=*/true);
  telas_->addWidget(mapasCtx);

  // 4 — Mapeamento: UMA ferramenta com o fluxo inteiro (criar o mapa, marcar
  // pontos, paredes virtuais, conferir). Waypoints, service areas, docking e o
  // editor de bitmap deixaram de ser modulos soltos: sao passos dela.
  auto * mapeamentoCtx = new ContextScreen("Mapeamento", true);
  ferramenta_mapa_ = new FerramentaMapeamento(bridge_);
  mapeamentoCtx->addSecao("Mapeamento", ferramenta_mapa_, /*larguraTotal=*/true);
  telas_->addWidget(mapeamentoCtx);
  // O mapa ao vivo so' faz sentido nos passos em que o robo se move; no passo de
  // conferencia quem manda e' o preview do arquivo salvo.
  connect(
    ferramenta_mapa_, &FerramentaMapeamento::querMapaAoVivo, this,
    [this, mapeamentoCtx](bool querer) {
      mapeamentoCtx->setPrecisaDoMapa(querer);
      if (telas_->currentWidget() == mapeamentoCtx) {
        rviz_->setVisible(querer);
        ajustarLarguraDoPainel();
      }
    });

  // 5 — Modo Avancado: e' para ca' que foi o painel tecnico que ocupava a tela
  // inicial. Sensores ao vivo entram aqui na proxima etapa.
  auto * avancado = new ContextScreen("Modo Avancado", true);
  avancado->addSecao("Estado do robo", inicio_);
  sensores_ = new SensoresModule(bridge_);
  avancado->addSecao("Sensores", sensores_, /*larguraTotal=*/true);
  // Listar e validar service areas e' conferencia, nao criacao — a criacao vive
  // no Mapeamento. Aqui serve para responder "por que a missao recusou esta
  // arena?".
  avancado->addSecao("Service areas", service_areas_);
  telas_->addWidget(avancado);
}

void MainWindow::irPara(int indice)
{
  if (indice < 0 || indice >= telas_->count()) {
    return;
  }
  telas_->setCurrentIndex(indice);

  auto * ctx = qobject_cast<ContextScreen *>(telas_->currentWidget());
  const bool ehHome = (indice == 0);

  voltar_->setVisible(!ehHome);
  titulo_->setText(ctx ? ctx->titulo() : "Caramelo");

  // Mostrar/esconder em vez de criar/destruir: ver o comentario do cabecalho.
  rviz_->setVisible(ctx && ctx->precisaDoMapa());
  ajustarLarguraDoPainel();
}

void MainWindow::ajustarLarguraDoPainel()
{
  auto * ctx = qobject_cast<ContextScreen *>(telas_->currentWidget());
  if (!ctx || !ctx->precisaDoMapa()) {
    telas_->setMinimumWidth(0);
    telas_->setMaximumWidth(QWIDGETSIZE_MAX);
    return;
  }
  // Largura proporcional, com piso: um painel fixo de 460 px cortava os botoes
  // dos modulos em telas grandes e nao respirava em telas pequenas. O piso vem
  // do proprio contexto (abas + largura util da ferramenta).
  const int piso = ctx->larguraComMapa();
  const int proporcional = static_cast<int>(width() * 0.34);
  const int largura = std::min(std::max(piso, proporcional), 760);
  telas_->setFixedWidth(largura);
}

void MainWindow::resizeEvent(QResizeEvent * e)
{
  QMainWindow::resizeEvent(e);
  ajustarLarguraDoPainel();
}
