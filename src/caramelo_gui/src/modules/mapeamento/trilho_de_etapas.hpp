#pragma once

// Trilho de etapas: a coluna da esquerda da ferramenta de mapeamento.
//
// Por que trilho e nao abas: preparar um robo num ambiente novo tem ordem
// OBRIGATORIA -- mapear, limpar o mapa, marcar os pontos, fechar as passagens
// proibidas, conferir. Abas sao todas iguais entre si e nao contam nada sobre
// ordem: quem nunca usou o robo clicava na terceira antes da primeira e nao
// entendia por que nada funcionava (marcar um dock antes de existir mapa grava
// pose no vazio).
//
// Aqui cada etapa tem tres estados -- feita, atual, travada -- e a travada diz
// em texto POR QUE esta travada. Cinza sem motivo le'-se como "quebrado".

#include <QFrame>
#include <QString>
#include <QVector>

class QLabel;
class QPushButton;
class QVBoxLayout;

class TrilhoDeEtapas : public QFrame
{
  Q_OBJECT

public:
  explicit TrilhoDeEtapas(QWidget * parent = nullptr);

  // `motivo` e' a frase que aparece embaixo da etapa enquanto ela esta travada.
  void adicionarEtapa(const QString & nome, const QString & motivo);

  void setEtapaAtual(int indice);
  void setLiberadaAte(int indice);
  int liberadaAte() const {return liberada_;}
  int etapaAtual() const {return atual_;}

signals:
  void etapaEscolhida(int indice);

private:
  struct Etapa
  {
    QPushButton * botao = nullptr;
    QLabel * numero = nullptr;
    QLabel * motivo = nullptr;
  };

  void aplicarEstados();

  QVBoxLayout * layout_ = nullptr;
  QVector<Etapa> etapas_;
  int atual_ = 0;
  int liberada_ = 0;
};
