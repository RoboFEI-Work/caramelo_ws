#include "core/home_screen.hpp"

#include <functional>

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QVBoxLayout>

#include "core/icones.hpp"

namespace
{

// Cartao clicavel inteiro. Alvo de toque grande vale mais que area de clique
// pequena num touchscreen, ainda mais com o robo em movimento.
class CartaoClicavel : public QFrame
{
public:
  explicit CartaoClicavel(std::function<void()> aoClicar)
  : aoClicar_(std::move(aoClicar))
  {
    setCursor(Qt::PointingHandCursor);
  }

protected:
  void mouseReleaseEvent(QMouseEvent * e) override
  {
    if (e->button() == Qt::LeftButton && rect().contains(e->pos()) && aoClicar_) {
      aoClicar_();
    }
    QFrame::mouseReleaseEvent(e);
  }

private:
  std::function<void()> aoClicar_;
};

QString corDoEstado(int estado)
{
  switch (estado) {
    case 0: return "#27ae60";
    case 1: return "#f2994a";
    case 2: return "#eb5757";
    default: return "#7f93b0";
  }
}

QString textoDoEstado(int estado)
{
  switch (estado) {
    case 0: return "PRONTO";
    case 1: return "ATENCAO";
    case 2: return "BLOQUEADO";
    default: return "SEM DADOS";
  }
}

}  // namespace

HomeScreen::HomeScreen(QWidget * parent)
: QWidget(parent)
{
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(28, 22, 28, 18);
  outer->setSpacing(18);

  auto * meio = new QHBoxLayout();
  meio->setSpacing(22);
  meio->addWidget(construirColunaDoRobo(), 0);
  meio->addWidget(construirGrade(), 1);
  outer->addLayout(meio, 1);

  outer->addWidget(construirBarraInferior(), 0);

  reavaliar();
}

QWidget * HomeScreen::construirColunaDoRobo()
{
  auto * col = new QFrame();
  col->setObjectName("colunaRobo");
  col->setFixedWidth(260);

  auto * layout = new QVBoxLayout(col);
  layout->setContentsMargins(20, 22, 20, 22);
  layout->setSpacing(10);

  auto * nome = new QLabel("Caramelo");
  nome->setObjectName("nomeRobo");
  layout->addWidget(nome);

  auto * equipe = new QLabel("RoboFEI@Work");
  equipe->setObjectName("equipeRobo");
  layout->addWidget(equipe);

  layout->addSpacing(10);

  estado_robo_ = new QLabel("Aguardando o robo");
  estado_robo_->setObjectName("chipEstadoRobo");
  estado_robo_->setWordWrap(true);
  layout->addWidget(estado_robo_);

  arena_ = new QLabel("Arena: —");
  arena_->setObjectName("arenaRobo");
  arena_->setWordWrap(true);
  layout->addWidget(arena_);

  layout->addStretch();
  return col;
}

QWidget * HomeScreen::construirGrade()
{
  auto * cont = new QWidget();
  grade_ = new QGridLayout(cont);
  grade_->setSpacing(18);

  struct Def
  {
    const char * chave;
    const char * titulo;
    const char * sub;
    icones::Tipo icone;
    int contexto;
  };
  const Def defs[] = {
    {"operacao", "Operacao", "Navegar, dockar e teleoperar",
      icones::Tipo::Operacao, Operacao},
    {"competicao", "Competicao", "Rodar uma prova de ponta a ponta",
      icones::Tipo::Competicao, Competicao},
    {"mapas", "Mapas", "Escolher a arena em que o robo esta",
      icones::Tipo::Mapas, Mapas},
    {"mapeamento", "Mapeamento", "Criar mapa, waypoints e docks",
      icones::Tipo::Mapeamento, Mapeamento},
    {"avancado", "Modo Avancado", "Sensores, sistema e diagnostico",
      icones::Tipo::Avancado, Avancado},
  };

  int i = 0;
  for (const auto & d : defs) {
    auto * card = criarCartao(
      d.chave, d.titulo, d.sub, static_cast<int>(d.icone), d.contexto);
    grade_->addWidget(card, i / 3, i % 3);
    ++i;
  }
  // Uma coluna vazia no fim da segunda linha ficaria colada; o stretch mantem os
  // cartoes com a mesma largura em qualquer resolucao.
  for (int c = 0; c < 3; ++c) {
    grade_->setColumnStretch(c, 1);
  }
  grade_->setRowStretch(2, 1);
  return cont;
}

QFrame * HomeScreen::criarCartao(
  const QString & chave, const QString & titulo, const QString & subtitulo,
  int tipoIcone, int contexto)
{
  auto * frame = new CartaoClicavel([this, contexto]() {emit abrirContexto(contexto);});
  frame->setObjectName("cartaoHome");
  frame->setMinimumSize(230, 200);

  auto * layout = new QVBoxLayout(frame);
  layout->setContentsMargins(18, 18, 18, 14);
  layout->setSpacing(8);

  auto * icone = new QLabel();
  icone->setPixmap(icones::desenhar(static_cast<icones::Tipo>(tipoIcone), 72));
  icone->setFixedSize(72, 72);
  layout->addWidget(icone, 0, Qt::AlignLeft);

  auto * t = new QLabel(titulo);
  t->setObjectName("tituloCartaoHome");
  layout->addWidget(t);

  auto * s = new QLabel(subtitulo);
  s->setObjectName("subCartaoHome");
  s->setWordWrap(true);
  layout->addWidget(s);

  layout->addStretch();

  Cartao cartao;
  cartao.frame = frame;
  cartao.chip = new QLabel("SEM DADOS");
  cartao.chip->setObjectName("chipEstado");
  layout->addWidget(cartao.chip, 0, Qt::AlignLeft);

  cartao.motivo = new QLabel();
  cartao.motivo->setObjectName("motivoCartao");
  cartao.motivo->setWordWrap(true);
  layout->addWidget(cartao.motivo);

  cartoes_.insert(chave, cartao);
  return frame;
}

QWidget * HomeScreen::construirBarraInferior()
{
  auto * barra = new QFrame();
  barra->setObjectName("barraInferior");
  auto * layout = new QHBoxLayout(barra);
  layout->setContentsMargins(18, 10, 18, 10);
  layout->setSpacing(28);

  auto botao = [](icones::Tipo tipo, const QString & texto) {
      auto * b = new QPushButton(texto);
      b->setObjectName("acaoRapida");
      b->setIcon(QIcon(icones::desenhar(tipo, 40)));
      b->setIconSize(QSize(28, 28));
      b->setMinimumHeight(52);
      return b;
    };

  auto * parar = botao(icones::Tipo::Parar, "Parar o robo");
  parar->setObjectName("acaoParar");
  connect(parar, &QPushButton::clicked, this, [this]() {emit pararRobo();});
  layout->addWidget(parar);

  auto * base = botao(icones::Tipo::Base, "Levar para a base");
  // Desabilitado COM MOTIVO. Um botao cinza sem explicacao faz o operador achar
  // que a interface esta quebrada; com o motivo, ele sabe o que fazer.
  base->setEnabled(false);
  base_btn_ = base;
  layout->addWidget(base);
  base_motivo_ = new QLabel("Sem base definida — crie um dock START em Mapeamento.");
  base_motivo_->setObjectName("motivoBarra");
  base_motivo_->setWordWrap(true);
  layout->addWidget(base_motivo_, 1);

  auto * diag = botao(icones::Tipo::Config, "Diagnostico");
  connect(diag, &QPushButton::clicked, this, [this]() {emit abrirContexto(Avancado);});
  layout->addWidget(diag);

  return barra;
}

void HomeScreen::aplicar(const QString & chave, Estado estado, const QString & motivo)
{
  auto it = cartoes_.find(chave);
  if (it == cartoes_.end()) {
    return;
  }
  const int e = static_cast<int>(estado);
  it->chip->setText(textoDoEstado(e));
  it->chip->setStyleSheet(
    QString("background:%1; color:#06121f; padding:3px 12px; border-radius:9px;")
    .arg(corDoEstado(e)));
  // Estado sozinho nao ajuda ninguem: "BLOQUEADO" sem motivo so' informa que
  // algo deu errado. O motivo e' a parte util.
  it->motivo->setText(motivo);
  it->motivo->setVisible(!motivo.isEmpty());
}

void HomeScreen::onDiagnostics(const QVector<ComponentHealth> & health)
{
  for (const auto & c : health) {
    saude_.insert(c.name, c);
  }
  reavaliar();
}

void HomeScreen::setMapaAtivo(const QString & nome)
{
  mapa_ativo_ = nome;
  arena_->setText(nome.isEmpty() ? "Arena: nenhuma escolhida" : "Arena: " + nome);
  reavaliar();
}

void HomeScreen::setServidorMissao(bool disponivel)
{
  servidor_missao_ = disponivel;
  reavaliar();
}

void HomeScreen::reavaliar()
{
  // Traduz o /diagnostics para a pergunta que o operador faz. Nenhum texto aqui
  // cita topico, action ou nome de no.
  auto nivel = [this](const QString & nome) {
      auto it = saude_.find(nome);
      return it == saude_.end() ? 3 : it->level;
    };
  const bool semDados = saude_.isEmpty();

  if (semDados) {
    estado_robo_->setText("Aguardando o robo responder");
  } else if (nivel("caramelo/rede_raspberry") >= 2) {
    estado_robo_->setText("Sem conexao com o robo");
  } else if (nivel("caramelo/tf") >= 2) {
    estado_robo_->setText("Ligado, mas sem saber onde esta");
  } else {
    estado_robo_->setText("Pronto para operar");
  }

  // --- Operacao ---
  if (semDados) {
    aplicar("operacao", Estado::Desconhecido, "Aguardando o robo responder.");
  } else if (nivel("caramelo/rede_raspberry") >= 2) {
    aplicar("operacao", Estado::Bloqueado, "Sem conexao com o robo.");
  } else if (nivel("caramelo/lidar") >= 2 || nivel("caramelo/odometria") >= 2) {
    aplicar("operacao", Estado::Bloqueado, "Sensores de navegacao sem dados.");
  } else if (nivel("caramelo/tf") >= 2) {
    aplicar("operacao", Estado::Degradado,
      "O robo ainda nao sabe onde esta. Defina a posicao inicial em Operacao.");
  } else if (nivel("caramelo/nav2") >= 1) {
    aplicar("operacao", Estado::Degradado, "Navegacao ainda subindo.");
  } else {
    aplicar("operacao", Estado::Pronto, QString());
  }

  // --- Competicao ---
  if (!servidor_missao_) {
    aplicar("competicao", Estado::Bloqueado,
      "Sistema de missoes fora do ar. Reinicie o robo.");
  } else if (mapa_ativo_.isEmpty()) {
    aplicar("competicao", Estado::Bloqueado, "Escolha a arena antes.");
  } else if (nivel("caramelo/tf") >= 2) {
    aplicar("competicao", Estado::Degradado, "Defina a posicao inicial do robo.");
  } else {
    aplicar("competicao", Estado::Pronto, QString());
  }

  // --- Mapas e Mapeamento ---
  aplicar(
    "mapas", mapa_ativo_.isEmpty() ? Estado::Degradado : Estado::Pronto,
    mapa_ativo_.isEmpty() ? "Nenhuma arena escolhida." : QString());
  if (semDados) {
    aplicar("mapeamento", Estado::Desconhecido, "Aguardando o robo responder.");
  } else if (nivel("caramelo/lidar") >= 2) {
    aplicar("mapeamento", Estado::Bloqueado, "LiDAR sem dados: nao da para mapear.");
  } else {
    aplicar("mapeamento", Estado::Pronto, QString());
  }

  // Diagnostico existe justamente para quando algo esta errado.
  aplicar("avancado", Estado::Pronto, QString());
}
