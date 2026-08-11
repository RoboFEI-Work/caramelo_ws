#pragma once

// Estado persistente do robo — o que ele precisa lembrar entre desligamentos.
//
// Por que existe: o mapa da arena era escolhido a cada launch, por argumento de
// linha de comando. Em prova isso significa alguem digitar o nome certo do mapa
// num terminal, sob pressao, toda vez que o robo liga. Aqui o mapa fica gravado
// no robo: ele sobe sozinho no ambiente em que estava, e trocar de mapa e' uma
// escolha na tela, nao um argumento de terminal.
//
// Arquivo: ~/.config/caramelo/robot_state.yaml
//   map_name: warehouse
//
// O MESMO arquivo e' lido pelo launch de inicializacao
// (caramelo_bringup/launch/caramelo.launch.py) para subir o map_server e o AMCL
// ja' no mapa certo. Qualquer mudanca de formato tem que acontecer nos dois
// lados ao mesmo tempo.

#include <QObject>
#include <QString>

class RobotState : public QObject
{
  Q_OBJECT

public:
  explicit RobotState(QObject * parent = nullptr);

  // Mapa ativo. Vazio quando o robo nunca teve mapa definido — nesse caso a UI
  // precisa pedir um, e nao inventar um default silencioso.
  QString mapName() const {return map_name_;}

  // Grava e avisa. Devolve false (com motivo) se nao conseguiu escrever: sem
  // isso o operador acharia que trocou de mapa e o robo voltaria no antigo no
  // proximo boot.
  bool setMapName(const QString & name, QString * erro = nullptr);

  static QString filePath();

signals:
  void mapNameChanged(const QString & name);

private:
  void load();

  QString map_name_;
};
