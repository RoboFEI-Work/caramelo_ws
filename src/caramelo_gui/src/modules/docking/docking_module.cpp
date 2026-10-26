#include "modules/docking/docking_module.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"
#include "widgets/seletor_arena.hpp"
#include "widgets/seletor_dock.hpp"
#include "widgets/seletor_tipo_dock.hpp"

DockingModule::DockingModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Docking");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * dica = new QLabel(
    "Encostar o robo numa estacao ja' marcada no mapa. Se a estacao que voce "
    "procura nao esta' na lista, ela ainda nao foi marcada: use a Ferramenta de "
    "Mapeamento, passo 2.");
  dica->setObjectName("msgCartao");
  dica->setWordWrap(true);
  layout->addWidget(dica);

  auto * form = new QFormLayout();
  map_name_ = new SeletorArena(bridge);
  dock_id_ = new SeletorDock(bridge);
  tipo_dock_ = new SeletorTipoDock(bridge);
  refine_ = new QCheckBox("Refinar alinhamento com LiDAR");
  // Default LIGADO (2026-08-03): politica unica com o bt_yaml_executor —
  // alinhar sem refino herda o offset do AMCL na hora critica.
  refine_->setChecked(true);
  form->addRow("Mapa:", map_name_);
  form->addRow("Estacao:", dock_id_);
  form->addRow("Tipo de aproximacao:", tipo_dock_);
  form->addRow("", refine_);
  layout->addLayout(form);

  motivo_ = new QLabel();
  motivo_->setObjectName("motivoCartao");
  motivo_->setWordWrap(true);
  layout->addWidget(motivo_);

  botao_dockar_ = new QPushButton("Dockar");
  connect(
    botao_dockar_, &QPushButton::clicked, this,
    [this]() {
      if (!podeAgir()) {return;}
      bridge_->sendDock(dock_id_->dock());
    });
  layout->addWidget(botao_dockar_);

  botao_alinhar_ = new QPushButton("Alinhamento fino (holonomico)");
  connect(
    botao_alinhar_, &QPushButton::clicked, this,
    [this]() {
      if (!podeAgir()) {return;}
      bridge_->sendAlign(dock_id_->dock(), map_name_->arena(), refine_->isChecked());
    });
  layout->addWidget(botao_alinhar_);

  auto * undockBtn = new QPushButton("Sair da estacao (undock)");
  connect(
    undockBtn, &QPushButton::clicked, this,
    // Tipo explicito: com "" o opennav_docking so resolve se lembrar do ultimo
    // dock DA MESMA sessao — apos reiniciar a nav o undock falhava. O tipo vem
    // da lista da arena; antes era "caramelo_front_dock" cravado aqui, o que
    // dava re' errada ao sair de prateleira.
    [this]() {bridge_->sendUndock(tipo_dock_->tipoDock());});
  layout->addWidget(undockBtn);

  botao_regravar_ = new QPushButton("Regravar a pose desta estacao (pose atual do robo)");
  connect(
    botao_regravar_, &QPushButton::clicked, this,
    // Regravar NAO passa por podeAgir(): uma estacao sem pose gravada e'
    // exatamente o caso que este botao conserta. Exigir pose aqui trancaria o
    // operador do lado de fora do proprio conserto.
    [this]() {
      if (!temAlvo()) {return;}
      // Sobrescrever uma pose que ja' existe e' destrutivo: a estacao passa a
      // apontar para onde o robo esta' AGORA. Se ele nao estiver encostado no
      // lugar certo, a missao seguinte para no lugar errado -- e a pose boa ja'
      // foi embora. Por isso a pergunta so' aparece quando ha' o que perder.
      if (!dock_id_->semPose()) {
        const auto resp = QMessageBox::question(
          this, "Regravar a pose",
          "A estacao \"" + dock_id_->dock() + "\" ja' tem uma posicao gravada.\n\n"
          "Regravar substitui essa posicao pela pose ATUAL do robo. Confira se "
          "ele esta' parado, encostado na estacao, antes de continuar.\n\n"
          "Uma copia de seguranca do arquivo e' criada automaticamente.");
        if (resp != QMessageBox::Yes) {return;}
      }
      bridge_->saveDockPose(dock_id_->dock());
      // O pedido e' assincrono: quem grava o docking.yaml e' um no de fora.
      // Reler a lista na mesma linha pegava o arquivo ANTES da escrita, e a
      // estacao continuava aparecendo como "(sem pose)" mesmo depois de gravada
      // -- o operador gravava tres, quatro vezes achando que nao tinha pegado.
      QTimer::singleShot(
        2000, this, [this]() {
          dock_id_->recarregar();
          atualizarDisponibilidade();
        });
    });
  layout->addWidget(botao_regravar_);

  status_ = new QLabel("Pronto.");
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);
  layout->addStretch();

  // A lista de docks e a de tipos vivem dentro do mapa escolhido nesta tela, e
  // nao na arena que o robo carregou. Sem este repasse o operador trocava o
  // mapa e continuava vendo os docks do mapa anterior.
  auto seguirArena = [this]() {
      const QString arena = map_name_->arena();
      if (arena.isEmpty()) {
        atualizarDisponibilidade();
        return;
      }
      dock_id_->setArena(arena);
      tipo_dock_->setArena(arena);
      // setArena recarrega o combo com os sinais bloqueados (para nao disparar
      // um currentIndexChanged por item inserido), entao a disponibilidade nao
      // se atualiza sozinha aqui.
      atualizarDisponibilidade();
    };
  connect(
    map_name_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [seguirArena](int) {seguirArena();});
  seguirArena();

  // Trocar de estacao muda o que da' para fazer: uma estacao sem pose gravada
  // so' aceita "regravar".
  connect(
    dock_id_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int) {atualizarDisponibilidade();});

  connect(
    bridge_, &RosBridge::dockStatus, this,
    [this](const QString & m) {status_->setText(m);});
  connect(
    bridge_, &RosBridge::dockResult, this,
    [this](bool ok, const QString & m) {
      status_->setText((ok ? "OK: " : "Falha: ") + m);
    });
}

void DockingModule::atualizarDisponibilidade()
{
  const bool temMapa = !map_name_->vazio() && !map_name_->arena().isEmpty();
  const bool temEstacao = temMapa && !dock_id_->vazio() && !dock_id_->dock().isEmpty();
  const bool temPose = temEstacao && !dock_id_->semPose();

  botao_dockar_->setEnabled(temPose);
  botao_alinhar_->setEnabled(temPose);
  botao_regravar_->setEnabled(temEstacao);

  if (!temMapa) {
    motivo_->setText(
      "Este robo ainda nao tem nenhum mapa salvo, entao nao ha' estacao para "
      "encostar. Crie o mapa em Mapeamento.");
  } else if (!temEstacao) {
    motivo_->setText(
      "Este mapa nao tem nenhuma estacao marcada. Leve o robo ate' a estacao e "
      "grave o ponto na Ferramenta de Mapeamento, passo 2.");
  } else if (!temPose) {
    motivo_->setText(
      "A estacao \"" + dock_id_->dock() + "\" esta' na lista mas nunca teve a "
      "posicao gravada, por isso mandar o robo encostar esta' desligado. "
      "Leve o robo ate' ela e use \"Regravar a pose desta estacao\".");
  } else {
    motivo_->setText(QString());
  }
}

bool DockingModule::temAlvo()
{
  if (map_name_->vazio()) {
    status_->setText(
      "Este robo ainda nao tem nenhum mapa salvo. Crie um em Ferramenta de "
      "Mapeamento antes de usar as estacoes.");
    return false;
  }
  if (dock_id_->vazio() || dock_id_->dock().isEmpty()) {
    status_->setText(
      "Este mapa nao tem nenhuma estacao marcada. Leve o robo ate' a estacao e "
      "grave o ponto na Ferramenta de Mapeamento, passo 2.");
    return false;
  }
  return true;
}

bool DockingModule::podeAgir()
{
  if (!temAlvo()) {
    return false;
  }
  if (dock_id_->semPose()) {
    // Sintoma que isto evita: a estacao existia no arquivo com pose [0,0,0], o
    // robo aceitava a ordem e saia rumo a origem do mapa, atravessando a arena.
    status_->setText(
      "A estacao \"" + dock_id_->dock() + "\" esta' na lista mas nunca teve a "
      "posicao gravada. Leve o robo ate' ela e grave o ponto antes de mandar o "
      "robo encostar.");
    return false;
  }
  return true;
}
