#pragma once

// Seletor de service area: as areas que EXISTEM no service_areas.yaml da arena.
//
// Por que existe: o mesmo problema do dock digitado a mao. "WS3" numa arena que
// so' tem WS1 e WS2 vira uma missao que aborta no meio da prova; uma area em
// [0,0,0] parece pronta numa lista de nomes e leva o robo para a origem do
// mapa. Aqui a lista so' tem area real e quem esta zerada vem com "(sem pose)".
//
// Mesma API do SeletorDock (setArena + getter), de proposito: as telas alternam
// entre dock e service area e nao deveriam aprender dois widgets diferentes.
// A leitura sai do mesmo parser que o MapPreview usa (lerAreasDaArena).

#include <QComboBox>
#include <QString>

class RosBridge;

class SeletorServiceArea : public QComboBox
{
  Q_OBJECT

public:
  explicit SeletorServiceArea(RosBridge * bridge, QWidget * parent = nullptr);

  // Arena de onde ler as areas. Enquanto ninguem chamar, o seletor acompanha a
  // arena ativa do robo; depois da primeira chamada ele obedece so' a tela.
  void setArena(const QString & arena);
  QString arena() const {return arena_;}

  // Id da area escolhida (ex.: "WS1"), sem o aviso "(sem pose)".
  QString area() const;

  // Tipo tecnico da area escolhida (workstation, shelf, ...), como esta no YAML.
  QString tipo() const;

  // true quando a area escolhida ainda esta em [0,0,0].
  bool semPose() const;

  bool vazio() const {return vazio_;}

  void recarregar();

private:
  RosBridge * bridge_ = nullptr;
  QString arena_;
  bool segue_robo_ = true;
  bool vazio_ = true;
};
