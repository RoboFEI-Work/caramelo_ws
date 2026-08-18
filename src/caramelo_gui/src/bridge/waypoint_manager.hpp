#pragma once

// Gerenciador de waypoints por mapa: cria/move (Interactive Marker no mapa),
// persiste em maps/<mapa>/waypoints.yaml e alimenta o "seguir todos"
// (FollowWaypoints). Rotas topologicas continuam OPCIONAIS (padrao do robo e'
// o planejamento livre do Nav2) — isto e' so uma lista ordenada de pontos.

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QObject>
#include <QString>
#include <QStringList>

#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class WaypointManager : public QObject
{
  Q_OBJECT

public:
  explicit WaypointManager(rclcpp::Node::SharedPtr node, QObject * parent = nullptr);

  void load(const QString & map_dir);
  // Re-le' o arquivo da ultima pasta carregada. Existe porque outra tela pode
  // ter escrito no waypoints.yaml da mesma arena: sem reler, a lista em memoria
  // continua a antiga e o proximo save() APAGA o que a outra tela gravou.
  void recarregar();
  // Pasta da arena que a lista em memoria esta representando agora ("" antes do
  // primeiro load). Quem escreve no waypoints.yaml por fora precisa saber se
  // mexeu justamente na arena que esta carregada.
  QString mapDir() const {return map_dir_;}
  void save(const QString & map_dir);
  void add(const QString & name);       // na pose atual do robo (TF) ou origem
  // Mesma coisa, mas com a pose vinda de um clique no mapa. Sem esta versao a
  // tela que marca ponto clicando tinha de escrever o waypoints.yaml por fora
  // do manager, e as duas verdades divergiam.
  void add(const QString & name, double x, double y, double yaw);
  void remove(const QString & name);
  QStringList names() const;
  std::vector<std::array<double, 3>> orderedPoses() const;  // x, y, yaw
  bool pose(const QString & name, double & x, double & y, double & yaw) const;

signals:
  void waypointsChanged(const QStringList & names);
  void status(const QString & message);

private:
  void insertMarker(const std::string & name, double x, double y, double yaw);
  void onFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & fb);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Ultima pasta passada a load(): e' o que recarregar() e o save automatico
  // do add() precisam saber.
  QString map_dir_;
  // De qual pasta a lista em memoria veio, de fato. Vazio = nunca foi lida.
  // save() recusa gravar numa pasta diferente desta: sem isso, os pontos da
  // arena A eram despejados dentro do waypoints.yaml da arena B.
  QString carregado_de_;
  // O arquivo existia e NAO deu para interpretar (erro de sintaxe do YAML).
  // Diferente de "arquivo ainda nao existe": num caso a lista vazia e' a
  // verdade, no outro e' ignorancia -- e gravar por cima apaga o que havia.
  bool leitura_falhou_ = false;

  mutable std::mutex mtx_;
  // Ordenado por nome (WP1, WP2, ...) — a ordem de "seguir todos".
  std::map<std::string, std::array<double, 3>> points_;
};
