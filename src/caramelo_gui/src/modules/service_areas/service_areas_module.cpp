#include "modules/service_areas/service_areas_module.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"
#include "modules/mapas/mapas_module.hpp"
#include "widgets/map_preview.hpp"
#include "widgets/seletor_arena.hpp"
#include "widgets/seletor_service_area.hpp"
#include "widgets/seletor_tipo_area.hpp"
#include "widgets/validador_id_ponto.hpp"

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
  existentes_ = new SeletorServiceArea(bridge);
  area_id_ = new QLineEdit();
  area_id_->setPlaceholderText("ex.: WS1");
  tipo_ = new SeletorTipoArea();
  form->addRow("Mapa:", map_name_);
  form->addRow("Ja' gravadas:", existentes_);
  form->addRow("Nome (WS1, SH1...):", area_id_);
  form->addRow("Tipo:", tipo_);
  layout->addLayout(form);

  motivo_ = new QLabel();
  motivo_->setObjectName("motivoCartao");
  motivo_->setWordWrap(true);
  layout->addWidget(motivo_);

  botao_salvar_ = new QPushButton("Salvar pose atual");
  botao_salvar_->setObjectName("acaoPrimaria");
  connect(
    botao_salvar_, &QPushButton::clicked, this, [this]() {
      const QString arena = map_name_->arena().trimmed();
      if (map_name_->vazio() || arena.isEmpty()) {
        status_->setText(
          "Este robo ainda nao tem nenhum mapa salvo. Crie um em Ferramenta de "
          "Mapeamento antes de marcar estacoes.");
        return;
      }
      // Sintoma que esta validacao mata: "ws 1" ou "mesa" era aceito pela tela,
      // a action de salvar abortava sem texto e a estacao simplesmente nao
      // existia na hora da prova. Caixa errada ("ws1") nao e' recusada, e'
      // corrigida -- e o campo passa a mostrar o nome que foi mesmo gravado.
      if (!idDePontoAceitavel(area_id_->text())) {
        status_->setText(erroDeIdDePonto(area_id_->text()));
        return;
      }
      const QString id = idDePontoNormalizado(area_id_->text());
      area_id_->setText(id);

      // Regravar por cima de uma area que ja' tem pose e' destrutivo: a estacao
      // passa a apontar para onde o robo esta' AGORA. Se ele nao estiver parado
      // no lugar certo, a proxima missao para no lugar errado -- e a pose boa
      // ja' foi embora. So' perguntamos quando ha' mesmo o que perder.
      const auto existentes = lerAreasDaArena(MapasModule::mapsDir() + "/" + arena);
      const auto atual = existentes.constFind(id);
      if (atual != existentes.constEnd() && !atual.value().pose.placeholder) {
        const auto resp = QMessageBox::question(
          this, "Substituir a posicao",
          "A estacao \"" + id + "\" ja' esta' gravada nesta arena, em "
          + QString::number(atual.value().pose.x, 'f', 2) + " m, "
          + QString::number(atual.value().pose.y, 'f', 2) + " m.\n\n"
          "Salvar substitui essa posicao pela pose ATUAL do robo. Confira se ele "
          "esta' parado, de frente para a estacao, antes de continuar.\n\n"
          "Uma copia de seguranca do arquivo e' criada automaticamente.");
        if (resp != QMessageBox::Yes) {return;}
      }

      bridge_->saveServiceAreaPose(arena, id, tipo_->tipo());
    });
  layout->addWidget(botao_salvar_);

  auto * botoes = new QHBoxLayout();
  botao_listar_ = new QPushButton("Listar areas");
  connect(
    botao_listar_, &QPushButton::clicked, this,
    [this]() {bridge_->listServiceAreas(map_name_->arena().trimmed());});
  botoes->addWidget(botao_listar_);

  botao_validar_ = new QPushButton("Validar (+sync docking)");
  connect(
    botao_validar_, &QPushButton::clicked, this,
    [this]() {bridge_->validateServiceAreas(map_name_->arena().trimmed());});
  botoes->addWidget(botao_validar_);
  layout->addLayout(botoes);

  lista_ = new QListWidget();
  layout->addWidget(lista_, 1);

  status_ = new QLabel("Pronto.");
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  // A lista de areas existentes e' do mapa escolhido AQUI, nao da arena que o
  // robo carregou: sem este repasse o operador trocava de mapa e continuava
  // vendo (e regravando por cima de) as areas do mapa anterior.
  auto seguirArena = [this]() {
      const QString arena = map_name_->arena();
      if (!arena.isEmpty()) {
        existentes_->setArena(arena);
      }
      atualizarDisponibilidade();
    };
  connect(
    map_name_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [seguirArena](int) {seguirArena();});
  seguirArena();

  // Escolher uma area existente preenche nome e tipo. E' o caminho de regravar
  // a pose de uma estacao que ja' existe sem redigitar o nome (e sem trocar o
  // tipo dela por acidente, que e' o que acontecia com o combo fixo em
  // "workstation").
  connect(
    existentes_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int) {
      if (existentes_->vazio()) {return;}
      const QString id = existentes_->area();
      if (id.isEmpty()) {return;}
      area_id_->setText(id);
      tipo_->setTipo(existentes_->tipo());
    });

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
      // Gravou area nova: a lista de existentes tem que enxergar isso, senao a
      // tela contradiz o arquivo que ela mesma acabou de escrever.
      if (ok) {existentes_->recarregar();}
    });
}

void ServiceAreasModule::atualizarDisponibilidade()
{
  const bool temMapa = !map_name_->vazio() && !map_name_->arena().trimmed().isEmpty();
  botao_salvar_->setEnabled(temMapa);
  botao_listar_->setEnabled(temMapa);
  botao_validar_->setEnabled(temMapa);

  motivo_->setText(
    temMapa ?
    QString() :
    "Salvar, listar e validar estao desligados porque este robo ainda nao tem "
    "nenhum mapa salvo. As estacoes moram dentro da pasta de uma arena: crie o "
    "mapa em Mapeamento primeiro.");
}
