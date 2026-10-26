#pragma once

// Seletor de dock: a lista dos docks que EXISTEM no docking.yaml da arena.
//
// Por que existe: o dock era digitado a mao ("WS1") em telas de docking e de
// missao. Errar uma letra nao dava erro nenhum na tela -- a action saia
// procurando um dock inexistente e a missao morria no meio, longe do teclado.
// Pior: um dock que existe no arquivo mas ainda esta em [0,0,0] parece pronto
// numa lista de nomes, e so' se revela na hora em que o robo tenta encostar.
//
// Aqui os dois casos aparecem antes: a lista so' tem docks reais, e os que
// ainda nao tem pose gravada vem marcados com "(sem pose)".
//
// A lista sai do MESMO parser que o MapPreview usa para desenhar
// (lerDocksDaArena), para tela e desenho nunca discordarem.

#include <QComboBox>
#include <QString>

class RosBridge;

class SeletorDock : public QComboBox
{
  Q_OBJECT

public:
  explicit SeletorDock(RosBridge * bridge, QWidget * parent = nullptr);

  // Arena de onde ler os docks. Enquanto ninguem chamar, o seletor acompanha a
  // arena ativa do robo; depois da primeira chamada ele obedece so' a tela.
  void setArena(const QString & arena);
  QString arena() const {return arena_;}

  // Id do dock escolhido (ex.: "WS1"), sem o aviso "(sem pose)". Vazio quando a
  // arena nao tem dock nenhum -- quem usa precisa testar antes de mandar acao.
  QString dock() const;

  // true quando o dock escolhido ainda esta em [0,0,0]: a tela pode avisar em
  // vez de deixar o operador mandar o robo para a origem do mapa.
  bool semPose() const;

  bool vazio() const {return vazio_;}

  void recarregar();

private:
  RosBridge * bridge_ = nullptr;
  QString arena_;
  bool segue_robo_ = true;
  bool vazio_ = true;
};
