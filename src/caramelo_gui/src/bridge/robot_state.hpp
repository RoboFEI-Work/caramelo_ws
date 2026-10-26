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

  // Ultima posicao conhecida do robo no mapa, gravada quando o operador
  // confirma a localizacao. Sem isso o robo liga sem saber onde esta: o AMCL
  // sobe, mas nenhuma meta funciona ate' alguem posicionar o fantasma na mao.
  // E' o que separa "os nos subiram" de "o robo esta pronto para uso".
  bool temPoseInicial() const {return tem_pose_;}
  double poseX() const {return pose_x_;}
  double poseY() const {return pose_y_;}
  double poseYaw() const {return pose_yaw_;}
  bool setPoseInicial(double x, double y, double yaw, QString * erro = nullptr);

  static QString filePath();

signals:
  void mapNameChanged(const QString & name);

private:
  void load();

  QString map_name_;
  bool tem_pose_ = false;
  double pose_x_ = 0.0;
  double pose_y_ = 0.0;
  double pose_yaw_ = 0.0;
};
