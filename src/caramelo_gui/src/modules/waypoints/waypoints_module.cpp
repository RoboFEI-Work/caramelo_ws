#include "modules/waypoints/waypoints_module.hpp"

#include <algorithm>

#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QRegularExpression>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"
#include "widgets/seletor_arena.hpp"
#include "bridge/waypoint_manager.hpp"
#include "modules/mapas/mapas_module.hpp"

WaypointsModule::WaypointsModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Waypoints");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * dica = new QLabel(
    "Um waypoint e' um lugar qualquer para onde o robo pode ser mandado. O nome "
    "e' livre, mas nao pode repetir: dois pontos com o mesmo nome viram um so'.");
  dica->setObjectName("msgCartao");
  dica->setWordWrap(true);
  layout->addWidget(dica);

  auto * form = new QFormLayout();
  map_name_ = new SeletorArena(bridge);
  nome_ = new QLineEdit();
  nome_->setPlaceholderText("ex.: WP1");
  form->addRow("Mapa:", map_name_);
  form->addRow("Nome do waypoint:", nome_);
  layout->addLayout(form);

  motivo_ = new QLabel();
  motivo_->setObjectName("motivoCartao");
  motivo_->setWordWrap(true);
  layout->addWidget(motivo_);

  auto * wm = bridge_->waypoints();

  auto * botoes = new QHBoxLayout();
  auto addBtn = [this, botoes](const QString & texto, auto slot) {
      auto * b = new QPushButton(texto);
      connect(b, &QPushButton::clicked, this, slot);
      botoes->addWidget(b);
      return b;
    };

  novo_ = addBtn(
    "Novo (pose do robo)", [this, wm]() {
      if (!temArena()) {return;}
      const QString nome = nome_->text().trimmed();
      if (nome.isEmpty()) {
        status_->setText(
          "De um nome ao waypoint antes de criar. Sugestao: " + sugerirProximoNome() + ".");
        return;
      }
      // Sintoma que esta checagem mata: o WaypointManager guarda os pontos num
      // mapa por nome. Repetir o nome nao dava erro -- substituia o ponto
      // antigo em silencio, e o robo passava a parar no lugar novo achando que
      // era o antigo.
      for (const QString & existente : wm->names()) {
        if (existente.compare(nome, Qt::CaseInsensitive) == 0) {
          status_->setText(
            "Ja' existe um waypoint chamado \"" + existente + "\" neste mapa. "
            "Escolha outro nome (sugestao: " + sugerirProximoNome() + ") ou "
            "remova o antigo primeiro.");
          return;
        }
      }
      wm->add(nome);
      // Depois de criar, o campo ja' fica pronto para o proximo: quem marca
      // vinte pontos seguidos nao deveria ter que lembrar onde parou.
      sugestao_ = sugerirProximoNome();
      nome_->setText(sugestao_);
    });

  remover_ = addBtn(
    "Remover", [this, wm]() {
      auto * item = lista_->currentItem();
      if (!item) {
        status_->setText("Escolha na lista qual waypoint remover.");
        return;
      }
      const QString alvo = item->text();
      const auto resp = QMessageBox::question(
        this, "Remover waypoint",
        "Remover o waypoint \"" + alvo + "\" da lista?\n\n"
        "Ele so' some do arquivo depois que voce clicar em Salvar.");
      if (resp != QMessageBox::Yes) {return;}
      wm->remove(alvo);
    });
  remover_->setObjectName("acaoDestrutiva");

  carregar_ = addBtn(
    "Carregar", [this, wm]() {
      if (!temArena()) {return;}
      wm->load(mapDir());
      arena_carregada_ = map_name_->arena();
    });

  salvar_ = addBtn(
    "Salvar", [this, wm]() {
      if (!temArena()) {return;}
      const QString arena = map_name_->arena();
      // Gravar sobrescreve o waypoints.yaml inteiro. Se a lista na tela veio de
      // OUTRA arena (o operador trocou o mapa sem carregar), salvar apagaria os
      // waypoints da arena escolhida e poria os da anterior no lugar -- sem
      // nenhum aviso.
      if (!arena_carregada_.isEmpty() && arena_carregada_ != arena) {
        const auto resp = QMessageBox::question(
          this, "Salvar em outra arena",
          "A lista na tela foi carregada da arena \"" + arena_carregada_ +
          "\", e voce esta' gravando na arena \"" + arena + "\".\n\n"
          "Isso substitui os waypoints de \"" + arena + "\" pelos que estao na "
          "tela. Continuar?");
        if (resp != QMessageBox::Yes) {return;}
      }
      wm->save(mapDir());
      arena_carregada_ = arena;
    });
  layout->addLayout(botoes);

  lista_ = new QListWidget();
  layout->addWidget(lista_, 1);

  auto * acoes = new QHBoxLayout();
  ir_ = new QPushButton("Ir ate o selecionado");
  ir_->setObjectName("acaoPrimaria");
  connect(
    ir_, &QPushButton::clicked, this, [this, wm]() {
      auto * item = lista_->currentItem();
      if (!item) {
        status_->setText("Escolha na lista para onde o robo deve ir.");
        return;
      }
      double x, y, yaw;
      if (wm->pose(item->text(), x, y, yaw)) {
        bridge_->goTo(x, y, yaw);
      }
    });
  acoes->addWidget(ir_);

  seguir_ = new QPushButton("Seguir todos (em ordem)");
  connect(
    seguir_, &QPushButton::clicked, this, [this, wm]() {
      if (lista_->count() == 0) {
        status_->setText("A lista esta' vazia: nao ha' por onde o robo passar.");
        return;
      }
      bridge_->followWaypoints(wm->orderedPoses());
    });
  acoes->addWidget(seguir_);
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
      // So' mexe no campo se ele estiver vazio ou ainda com a sugestao que a
      // propria tela colocou: o que o operador digitou e' dele.
      const QString atual = nome_->text().trimmed();
      if (atual.isEmpty() || atual == sugestao_) {
        sugestao_ = sugerirProximoNome();
        nome_->setText(sugestao_);
      }
    });
  connect(
    wm, &WaypointManager::status, this,
    [this](const QString & m) {status_->setText(m);});
  connect(
    bridge_, &RosBridge::navStatus, this,
    [this](const QString & m) {status_->setText(m);});

  connect(
    map_name_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int) {atualizarDisponibilidade();});

  sugestao_ = sugerirProximoNome();
  nome_->setText(sugestao_);
  atualizarDisponibilidade();
}

bool WaypointsModule::temArena()
{
  if (map_name_->vazio() || map_name_->arena().trimmed().isEmpty()) {
    status_->setText(
      "Este robo ainda nao tem nenhum mapa salvo. Crie um em Mapeamento antes "
      "de marcar waypoints.");
    return false;
  }
  return true;
}

QString WaypointsModule::sugerirProximoNome() const
{
  // Olha so' a familia WP<n>: nomes fora dela sao escolha do operador e nao
  // entram na conta do proximo numero.
  static const QRegularExpression padrao("^WP([0-9]+)$", QRegularExpression::CaseInsensitiveOption);
  int maior = 0;
  for (const QString & nome : bridge_->waypoints()->names()) {
    const auto m = padrao.match(nome.trimmed());
    if (m.hasMatch()) {
      maior = std::max(maior, m.captured(1).toInt());
    }
  }
  return QString("WP%1").arg(maior + 1);
}

void WaypointsModule::atualizarDisponibilidade()
{
  const bool temMapa = !map_name_->vazio() && !map_name_->arena().trimmed().isEmpty();
  novo_->setEnabled(temMapa);
  carregar_->setEnabled(temMapa);
  salvar_->setEnabled(temMapa);

  motivo_->setText(
    temMapa ?
    QString() :
    "Criar, carregar e salvar estao desligados porque este robo ainda nao tem "
    "nenhum mapa salvo. Crie um em Mapeamento — waypoints sao guardados dentro "
    "da pasta da arena.");
}

QString WaypointsModule::mapDir() const
{
  return MapasModule::mapsDir() + "/" + map_name_->arena().trimmed();
}
