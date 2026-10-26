#include "modules/mapeamento/trilho_de_etapas.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStyle>
#include <QVBoxLayout>
#include <QVariant>

namespace
{

// O tema desenha o estado a partir desta propriedade; o widget nao conhece
// cor nenhuma. Trocar a propriedade exige repolir, senao o Qt reaproveita o
// estilo antigo e a etapa continua parecendo travada depois de liberada.
void aplicarEstado(QWidget * alvo, const QString & estado)
{
  if (!alvo || alvo->property("estado").toString() == estado) {
    return;
  }
  alvo->setProperty("estado", estado);
  alvo->style()->unpolish(alvo);
  alvo->style()->polish(alvo);
}

}  // namespace

TrilhoDeEtapas::TrilhoDeEtapas(QWidget * parent)
: QFrame(parent)
{
  setObjectName("trilhoEtapas");
  layout_ = new QVBoxLayout(this);
  layout_->setContentsMargins(0, 14, 0, 14);
  layout_->setSpacing(2);
  layout_->addStretch();
}

void TrilhoDeEtapas::adicionarEtapa(const QString & nome, const QString & motivo)
{
  const int indice = etapas_.size();

  Etapa etapa;
  etapa.botao = new QPushButton();
  etapa.botao->setObjectName("etapaTrilho");
  etapa.botao->setCursor(Qt::PointingHandCursor);

  auto * linha = new QHBoxLayout(etapa.botao);
  linha->setContentsMargins(14, 10, 14, 10);
  linha->setSpacing(10);

  etapa.numero = new QLabel(QString::number(indice + 1));
  etapa.numero->setObjectName("numeroEtapa");
  linha->addWidget(etapa.numero);

  auto * texto = new QLabel(nome);
  texto->setWordWrap(true);
  linha->addWidget(texto, 1);

  etapa.motivo = new QLabel(motivo);
  etapa.motivo->setObjectName("motivoEtapa");
  etapa.motivo->setWordWrap(true);

  // O stretch final e' sempre o ultimo item: as etapas entram antes dele.
  layout_->insertWidget(layout_->count() - 1, etapa.botao);
  layout_->insertWidget(layout_->count() - 1, etapa.motivo);

  connect(
    etapa.botao, &QPushButton::clicked, this,
    [this, indice]() {emit etapaEscolhida(indice);});

  etapas_.append(etapa);
  aplicarEstados();
}

void TrilhoDeEtapas::setEtapaAtual(int indice)
{
  if (indice < 0 || indice >= etapas_.size()) {
    return;
  }
  atual_ = indice;
  aplicarEstados();
}

void TrilhoDeEtapas::setLiberadaAte(int indice)
{
  liberada_ = qBound(0, indice, etapas_.size() - 1);
  aplicarEstados();
}

void TrilhoDeEtapas::aplicarEstados()
{
  for (int i = 0; i < etapas_.size(); ++i) {
    const bool travada = (i > liberada_);
    const QString estado = travada ? "travada" : (i == atual_ ? "atual" : "feita");
    aplicarEstado(etapas_[i].botao, estado);
    aplicarEstado(etapas_[i].numero, estado);
    etapas_[i].botao->setEnabled(!travada);
    // A frase do motivo so' existe enquanto a etapa esta travada; depois ela
    // vira ruido no meio da coluna.
    etapas_[i].motivo->setVisible(travada);
  }
}
