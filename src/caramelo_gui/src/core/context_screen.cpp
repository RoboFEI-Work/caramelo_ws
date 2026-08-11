#include "core/context_screen.hpp"

#include <QHBoxLayout>
#include <QListWidget>
#include <QScrollArea>
#include <QStackedWidget>
#include <QVBoxLayout>

namespace
{
// Largura util minima de uma ferramenta. Abaixo disso os formularios dos
// modulos (rotulo + campo + par de botoes lado a lado) comecam a ser cortados.
constexpr int kLarguraMinimaConteudo = 430;

// Em tela cheia, esticar um formulario por 1400 px deixa campos gigantes e
// ilegiveis. O conteudo fica numa coluna centralizada com largura maxima.
constexpr int kLarguraMaximaConteudo = 900;
}  // namespace

ContextScreen::ContextScreen(const QString & titulo, bool precisaDoMapa, QWidget * parent)
: QWidget(parent), titulo_(titulo), precisa_do_mapa_(precisaDoMapa)
{
  auto * layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  abas_ = new QListWidget();
  abas_->setObjectName("abasContexto");
  abas_->setFixedWidth(kLarguraAbas);
  abas_->hide();   // aparece so' a partir da segunda secao
  layout->addWidget(abas_);

  pilha_ = new QStackedWidget();
  layout->addWidget(pilha_, 1);

  connect(
    abas_, &QListWidget::currentRowChanged, this, [this](int row) {
      if (row >= 0) {
        pilha_->setCurrentIndex(row);
      }
    });
}

int ContextScreen::larguraComMapa() const
{
  // O painel divide a tela com o mapa. Precisa caber a lista de abas MAIS a
  // largura util da ferramenta -- foi somar errado aqui que cortou os botoes
  // dos modulos na primeira versao.
  const int abas = (pilha_ && pilha_->count() > 1) ? kLarguraAbas : 0;
  return abas + kLarguraMinimaConteudo + 24;
}

void ContextScreen::addSecao(const QString & nome, QWidget * conteudo)
{
  conteudo->setMinimumWidth(kLarguraMinimaConteudo);

  // Coluna centralizada: em tela cheia o formulario nao estica pela largura
  // toda; com o mapa ao lado, a largura maxima nem chega a valer.
  auto * caixa = new QWidget();
  auto * caixaLayout = new QHBoxLayout(caixa);
  caixaLayout->setContentsMargins(0, 0, 0, 0);
  caixaLayout->addStretch();
  conteudo->setMaximumWidth(kLarguraMaximaConteudo);
  caixaLayout->addWidget(conteudo, 1);
  caixaLayout->addStretch();

  // Toda secao rola: no touchscreen a altura util e' pequena e um formulario
  // cortado sem barra de rolagem simplesmente some. A barra horizontal fica
  // como rede de seguranca -- nada pode ficar inalcancavel.
  auto * scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setWidget(caixa);
  pilha_->addWidget(scroll);

  auto * item = new QListWidgetItem(nome, abas_);
  item->setSizeHint(QSize(0, 48));

  if (pilha_->count() == 1) {
    abas_->setCurrentRow(0);
  }
  abas_->setVisible(pilha_->count() > 1);
}
