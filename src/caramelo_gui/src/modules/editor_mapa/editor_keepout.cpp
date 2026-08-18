#include "modules/editor_mapa/editor_keepout.hpp"

#include <QButtonGroup>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QVBoxLayout>

#include "modules/editor_mapa/editor_mapa_module.hpp"
#include "modules/mapas/mapas_module.hpp"
#include "modules/mapeamento/scripts_do_mapa.hpp"

namespace
{

// Os dois unicos valores que uma mascara aceita. Ver o cabecalho do .hpp.
const int kBloqueado = 0;
const int kLiberado = 255;

// Botao de opcao no estilo do resto da ferramenta (duas escolhas lado a lado,
// as duas sempre visiveis).
QPushButton * segmento(const QString & texto)
{
  auto * b = new QPushButton(texto);
  b->setObjectName("segmento");
  b->setCheckable(true);
  return b;
}

}  // namespace

EditorKeepout::EditorKeepout(ScriptsDoMapa * scripts, QWidget * parent)
: QWidget(parent), scripts_(scripts)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  auto * explica = new QLabel(
    "Pinte de VERMELHO por onde o robo nao pode passar, mesmo que o mapa nao "
    "mostre nada ali: fita no chao, degrau baixo, vidro, area de outra equipe. "
    "O mapa embaixo e' so' referencia — ele nao muda.");
  explica->setObjectName("msgCartao");
  explica->setWordWrap(true);
  layout->addWidget(explica);

  // Caixa que aparece so' quando a arena ainda nao tem mascara. Um aviso sem
  // botao obriga o operador a sair da tela para resolver -- e ele nao tem como
  // saber que existe um comando para isso.
  caixa_criar_ = new QFrame();
  caixa_criar_->setObjectName("cartaoStatus");
  auto * criarLayout = new QHBoxLayout(caixa_criar_);
  auto * textoCriar = new QLabel(
    "Esta arena ainda nao tem paredes virtuais.");
  textoCriar->setWordWrap(true);
  criarLayout->addWidget(textoCriar, 1);
  botao_criar_ = new QPushButton("Criar agora");
  botao_criar_->setObjectName("acaoPrimaria");
  criarLayout->addWidget(botao_criar_);
  layout->addWidget(caixa_criar_);

  // --- barra de ferramentas ---
  barra_ = new QWidget();
  auto * barra = new QHBoxLayout(barra_);
  barra->setContentsMargins(0, 0, 0, 0);

  auto * caixaPintar = new QFrame();
  caixaPintar->setObjectName("segmentado");
  auto * pintarLayout = new QHBoxLayout(caixaPintar);
  pintarLayout->setContentsMargins(0, 0, 0, 0);
  pintarLayout->setSpacing(0);
  auto * bloquear = segmento("Bloquear");
  auto * liberar = segmento("Liberar");
  bloquear->setChecked(true);
  auto * grupoPintar = new QButtonGroup(this);
  grupoPintar->addButton(bloquear);
  grupoPintar->addButton(liberar);
  grupoPintar->setExclusive(true);
  pintarLayout->addWidget(bloquear);
  pintarLayout->addWidget(liberar);
  barra->addWidget(caixaPintar);

  auto * caixaForma = new QFrame();
  caixaForma->setObjectName("segmentado");
  auto * formaLayout = new QHBoxLayout(caixaForma);
  formaLayout->setContentsMargins(0, 0, 0, 0);
  formaLayout->setSpacing(0);
  auto * pincel = segmento("Pincel");
  auto * retangulo = segmento("Retangulo");
  auto * linha = segmento("Linha");
  pincel->setChecked(true);
  auto * grupoForma = new QButtonGroup(this);
  grupoForma->addButton(pincel);
  grupoForma->addButton(retangulo);
  grupoForma->addButton(linha);
  grupoForma->setExclusive(true);
  formaLayout->addWidget(pincel);
  formaLayout->addWidget(retangulo);
  formaLayout->addWidget(linha);
  barra->addWidget(caixaForma);

  tamanho_ = new QSlider(Qt::Horizontal);
  tamanho_->setRange(1, 30);
  tamanho_->setValue(4);
  tamanho_->setFixedWidth(120);
  barra->addWidget(new QLabel("Espessura:"));
  barra->addWidget(tamanho_);

  auto * zoomIn = new QPushButton("Zoom +");
  auto * zoomOut = new QPushButton("Zoom -");
  auto * verTudo = new QPushButton("Ver tudo");
  auto * desfazer = new QPushButton("Desfazer");
  barra->addWidget(zoomIn);
  barra->addWidget(zoomOut);
  barra->addWidget(verTudo);
  barra->addWidget(desfazer);
  barra->addStretch();
  layout->addWidget(barra_);

  // --- area de desenho ---
  canvas_ = new MapCanvas();
  canvas_->setModoMascara(true);
  canvas_->setBrushValue(kBloqueado);
  canvas_->setBrushSize(tamanho_->value());
  scroll_ = new QScrollArea();
  scroll_->setWidget(canvas_);
  layout->addWidget(scroll_, 1);

  auto * rodape = new QHBoxLayout();
  botao_salvar_ = new QPushButton("Salvar as paredes virtuais");
  botao_salvar_->setObjectName("acaoPrimaria");
  rodape->addWidget(botao_salvar_);
  rodape->addStretch();
  layout->addLayout(rodape);

  status_ = new QLabel();
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  connect(
    bloquear, &QPushButton::clicked, this,
    [this]() {canvas_->setBrushValue(kBloqueado);});
  connect(
    liberar, &QPushButton::clicked, this,
    [this]() {canvas_->setBrushValue(kLiberado);});
  connect(
    pincel, &QPushButton::clicked, this,
    [this]() {canvas_->setFerramenta(MapCanvas::Ferramenta::Pincel);});
  connect(
    retangulo, &QPushButton::clicked, this,
    [this]() {canvas_->setFerramenta(MapCanvas::Ferramenta::Retangulo);});
  connect(
    linha, &QPushButton::clicked, this,
    [this]() {canvas_->setFerramenta(MapCanvas::Ferramenta::Linha);});
  connect(
    tamanho_, &QSlider::valueChanged, this,
    [this](int v) {canvas_->setBrushSize(v);});
  connect(
    zoomIn, &QPushButton::clicked, this,
    [this]() {canvas_->setZoom(canvas_->zoom() * 1.5);});
  connect(
    zoomOut, &QPushButton::clicked, this,
    [this]() {canvas_->setZoom(canvas_->zoom() / 1.5);});
  connect(
    verTudo, &QPushButton::clicked, this,
    [this]() {canvas_->ajustarPara(scroll_->viewport()->size());});
  connect(desfazer, &QPushButton::clicked, this, [this]() {canvas_->undo();});
  connect(botao_salvar_, &QPushButton::clicked, this, [this]() {salvar();});

  connect(
    botao_criar_, &QPushButton::clicked, this, [this]() {
      if (!scripts_ || arena_.isEmpty()) {
        mostrar("Escolha uma arena antes.");
        return;
      }
      botao_criar_->setEnabled(false);
      scripts_->criarParedesVirtuais(arena_);
    });

  if (scripts_) {
    connect(
      scripts_, &ScriptsDoMapa::terminou, this,
      [this](const QString & tarefa, bool ok, const QString & mensagem) {
        // So' a tarefa desta tela: recarregar por causa de um save de ponto
        // jogaria fora o desenho que o operador ainda nao salvou.
        if (tarefa != ScriptsDoMapa::rotuloParedesVirtuais()) {
          return;
        }
        botao_criar_->setEnabled(true);
        mostrar(mensagem);
        if (ok) {
          recarregar();
        }
      });
  }

  atualizarDisponibilidade(false);
}

QString EditorKeepout::pastaDoMapa() const
{
  return MapasModule::mapsDir() + "/" + arena_;
}

void EditorKeepout::setArena(const QString & arena)
{
  arena_ = arena.trimmed();
  recarregar();
}

void EditorKeepout::mostrar(const QString & frase)
{
  status_->setText(frase);
  emit status(frase);
}

void EditorKeepout::atualizarDisponibilidade(bool temMascara)
{
  caixa_criar_->setVisible(!temMascara);
  barra_->setEnabled(temMascara);
  scroll_->setEnabled(temMascara);
  botao_salvar_->setEnabled(temMascara);
}

void EditorKeepout::recarregar()
{
  if (arena_.isEmpty()) {
    atualizarDisponibilidade(false);
    mostrar("Escolha uma arena para editar as paredes virtuais.");
    return;
  }

  const QString mascara = pastaDoMapa() + "/keepout_mask.pgm";
  if (!QFileInfo::exists(mascara)) {
    atualizarDisponibilidade(false);
    mostrar("Esta arena ainda nao tem paredes virtuais. Clique em Criar agora.");
    return;
  }

  if (!canvas_->loadPgm(mascara)) {
    atualizarDisponibilidade(false);
    mostrar("O arquivo das paredes virtuais existe mas nao pude abrir. Crie de novo.");
    return;
  }

  // O mapa por baixo e' referencia, nao e' editado: e' o que diz ao operador
  // onde ficam as paredes de verdade.
  QImage mapa(pastaDoMapa() + "/map.pgm");
  canvas_->setImagemDeFundo(mapa);
  canvas_->ajustarPara(scroll_->viewport()->size());
  atualizarDisponibilidade(true);
  mostrar(
    mapa.isNull() ?
    "Paredes virtuais abertas (nao achei o desenho do mapa para mostrar por baixo)." :
    "Paredes virtuais abertas. O vermelho e' o que esta bloqueado.");
}

bool EditorKeepout::temAlteracoesNaoSalvas() const
{
  return canvas_->isModified();
}

bool EditorKeepout::salvar()
{
  if (arena_.isEmpty() || !canvas_->temImagem()) {
    mostrar("Nao ha' paredes virtuais abertas para salvar.");
    return false;
  }
  const QString mascara = pastaDoMapa() + "/keepout_mask.pgm";
  const QString backup = mascara + ".bak_" +
    QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
  QFile::copy(mascara, backup);
  if (!canvas_->savePgm(mascara)) {
    mostrar("Nao consegui salvar as paredes virtuais (a pasta esta protegida?).");
    return false;
  }
  canvas_->clearModified();
  mostrar(
    "Paredes virtuais salvas. Elas passam a valer na proxima vez que a "
    "navegacao for iniciada.");
  return true;
}
