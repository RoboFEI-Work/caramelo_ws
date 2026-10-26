#pragma once

// Fachada da missao para a UI: cliente da action /caramelo/run_mission e
// assinante de /caramelo/mission/status.
//
// Vive separado do ros_bridge.cpp de proposito. O ros_bridge ja' tem 500 linhas
// e concentra navegacao, docking, mapas e service areas; missao e' um dominio
// novo inteiro. Compartilha o mesmo rclcpp::Node por injecao, exatamente como o
// ManualLocalization e o WaypointManager ja' fazem.

#include <memory>
#include <mutex>

#include <QObject>
#include <QString>
#include <QStringList>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "caramelo_msgs/action/run_mission.hpp"
#include "caramelo_msgs/msg/mission_status.hpp"

#include "bridge/mission_types.hpp"

class MissionBridge : public QObject
{
  Q_OBJECT

public:
  // Tudo que o operador escolhe na tela antes de comecar.
  struct Options
  {
    QString taskYaml;
    QString mapName;
    QString mapDir;
    QString finishDockId = "FINISH";
    bool simulateNav = false;
    bool useLidarRefine = true;
    bool skipStartupHome = false;
    double preflightTimeout = 120.0;
  };

  MissionBridge(rclcpp::Node::SharedPtr node, QObject * parent = nullptr);

  // Gera o plano sem mover o robo (dry-run). Responde por planReady().
  void requestPlan(const Options & opts);

  // Executa. actionsYaml vazio => o servidor planeja sozinho; preenchido =>
  // reusa o plano ja' mostrado ao operador, sem replanejar.
  void run(const Options & opts, const QString & actionsYaml);

  // Cancela o goal em voo: o servidor manda SIGINT no executor, que faz o halt
  // limpo da arvore. Nunca mata o processo.
  void abort();

  bool busy() const {return busy_;}

  // O servidor de missao esta' no ar?
  bool serverReady() const;

  // Pre-flight visual: quais action servers a missao exige e quais ja' estao no
  // grafo. Consulta os servicos <action>/_action/send_goal, o que evita
  // depender dos tipos de action da manipulacao so' para conferir presenca.
  struct ServerCheck
  {
    QString action;
    QString rotulo;
    bool presente = false;
  };
  QVector<ServerCheck> checkServers(bool simulateNav) const;

signals:
  void progress(const MissionProgress & p);
  void planReady(bool ok, const QStringList & rows, const QString & actionsYaml,
    const QString & message);
  void finished(bool success, const QString & message);
  void busyChanged(bool busy);

private:
  using RunMission = caramelo_msgs::action::RunMission;
  using GoalHandle = rclcpp_action::ClientGoalHandle<RunMission>;

  RunMission::Goal buildGoal(const Options & opts, bool dryRun,
    const QString & actionsYaml) const;
  void send(const RunMission::Goal & goal, bool dryRun);
  void setBusy(bool busy);
  void onStatus(const caramelo_msgs::msg::MissionStatus::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<RunMission>::SharedPtr client_;
  rclcpp::Subscription<caramelo_msgs::msg::MissionStatus>::SharedPtr status_sub_;

  std::mutex goal_mtx_;
  GoalHandle::SharedPtr goal_handle_;
  bool busy_ = false;
};
