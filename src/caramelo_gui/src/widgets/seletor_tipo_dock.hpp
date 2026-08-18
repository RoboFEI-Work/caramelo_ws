#pragma once

// Seletor do tipo de dock -- as chaves de dock_plugins: do docking.yaml da arena.
//
// Por que existe: o tipo decide COMO o robo encosta (a que distancia ele para,
// com que tolerancia, se e' prateleira ou encaixe de precisao). Ele era um nome
// de plugin cravado no codigo ou digitado a mao; um nome fora do docking.yaml
// faz a action de docking recusar o dock -- e a tela nao mostrava nada.
//
// Aqui a lista vem do proprio arquivo da arena, entao so' existe opcao que o
// robo consegue executar. Os nomes tecnicos ganham rotulo em portugues, porque
// "caramelo_precision_front_dock" nao ensina nada a quem opera.

#include <QComboBox>
#include <QString>

class RosBridge;

class SeletorTipoDock : public QComboBox
{
  Q_OBJECT

public:
  explicit SeletorTipoDock(RosBridge * bridge, QWidget * parent = nullptr);

  // Arena de onde ler dock_plugins. Enquanto ninguem chamar, acompanha a arena
  // ativa do robo.
  void setArena(const QString & arena);
  QString arena() const {return arena_;}

  // Nome tecnico do plugin escolhido (ex.: "caramelo_front_dock") -- e' o que
  // vai para o docking.yaml e para a action.
  QString tipoDock() const;

  // Seleciona pelo nome tecnico; ignora nome que a arena nao tem.
  void setTipoDock(const QString & tipo);

  void recarregar();

private:
  RosBridge * bridge_ = nullptr;
  QString arena_;
  bool segue_robo_ = true;
};
