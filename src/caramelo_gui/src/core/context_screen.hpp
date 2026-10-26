#pragma once

// Tela de contexto: uma area de trabalho do robo (Operacao, Mapeamento,
// Diagnostico...). Agrupa as ferramentas daquele contexto em secoes e declara
// se precisa do mapa na tela.
//
// Por que declara: o mapa (RViz) era fixo no centro em TODAS as telas, inclusive
// nas que nao tem nada a ver com ele. Isso rouba metade do espaco util num
// touchscreen e faz a aplicacao parecer uma ferramenta de desenvolvedor, nao um
// produto embarcado. Agora cada contexto diz se quer o mapa, e a MainWindow
// apenas mostra ou esconde o unico RVizFrame que existe -- sem destruir e
// recriar o render do OGRE, que e' caro e ja' deu segfault neste projeto.

#include <QString>
#include <QWidget>

class QListWidget;
class QStackedWidget;

class ContextScreen : public QWidget
{
  Q_OBJECT

public:
  ContextScreen(const QString & titulo, bool precisaDoMapa, QWidget * parent = nullptr);

  // Acrescenta uma ferramenta ao contexto. Com uma so' secao, a lista lateral
  // nem aparece -- nada de menu de um item.
  //
  // larguraTotal: a secao ocupa toda a area, sem coluna centralizada nem
  // rolagem. E' o que uma tela de mapa precisa -- limitar a largura de um
  // preview so' desperdicaria tela.
  void addSecao(const QString & nome, QWidget * conteudo, bool larguraTotal = false);

  QString titulo() const {return titulo_;}
  bool precisaDoMapa() const {return precisa_do_mapa_;}

  // Alguns contextos alternam: a ferramenta de mapeamento usa o mapa ao vivo
  // enquanto se dirige o robo e o preview do arquivo na conferencia.
  void setPrecisaDoMapa(bool precisa) {precisa_do_mapa_ = precisa;}

  // Largura minima do painel quando o mapa divide a tela com este contexto.
  // Inclui a lista de abas: esquecer isso foi o que cortou os botoes dos
  // modulos na primeira versao.
  int larguraComMapa() const;

  static constexpr int kLarguraAbas = 150;

private:
  QString titulo_;
  bool precisa_do_mapa_;
  QListWidget * abas_ = nullptr;
  QStackedWidget * pilha_ = nullptr;
};
