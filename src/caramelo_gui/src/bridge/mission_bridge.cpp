#include "bridge/mission_bridge.hpp"

#include <functional>

namespace
{

// Action servers exigidos por uma missao, na ordem em que o operador espera
// ve-los subir. Os quatro ultimos so' fazem sentido com navegacao real.
struct ServerSpec
{
  const char * action;
  const char * rotulo;
  bool navegacao;
};

const ServerSpec kServers[] = {
  {"/move_action", "MoveIt (move_group)", false},
  {"/pick_tag", "Pegar objeto", false},
  {"/place_tag", "Depositar objeto", false},
  {"/navigate_to_pose", "Navegacao (Nav2)", true},
  {"/dock_robot", "Docking", true},
  {"/undock_robot", "Undocking", true},
  {"/align_to_dock", "Alinhamento fino", true},
};

MissionProgress::State toState(uint8_t s)
{
  if (s > static_cast<uint8_t>(MissionProgress::State::Aborted)) {
    return MissionProgress::State::Idle;
  }
  return static_cast<MissionProgress::State>(s);
}

}  // namespace

MissionBridge::MissionBridge(rclcpp::Node::SharedPtr node, QObject * parent)
: QObject(parent), node_(std::move(node))
{
  qRegisterMetaType<MissionProgress>("MissionProgress");

  client_ = rclcpp_action::create_client<RunMission>(node_, "/caramelo/run_mission");

  // MESMA QoS do bt_yaml_executor: transient_local com historico fundo. O
  // publicador nasce junto com a missao, entao a GUI e' sempre late joiner --
  // com depth 1 (ou durabilidade volatile) ela perderia o inicio de toda missao
  // e so' veria o ultimo passo.
  rclcpp::QoS qos(64);
  qos.transient_local().reliable();
  status_sub_ = node_->create_subscription<caramelo_msgs::msg::MissionStatus>(
    "/caramelo/mission/status", qos,
    std::bind(&MissionBridge::onStatus, this, std::placeholders::_1));
}

bool MissionBridge::serverReady() const
{
  return client_ && client_->action_server_is_ready();
}

QVector<MissionBridge::ServerCheck> MissionBridge::checkServers(bool simulateNav) const
{
  // Um action server aparece no grafo como um conjunto de servicos; conferir
  // <action>/_action/send_goal e' suficiente e nao obriga a GUI a depender dos
  // tipos de action da manipulacao (my_robot_msgs, moveit_msgs) so' para saber
  // se o servidor existe.
  const auto servicos = node_->get_service_names_and_types();

  QVector<ServerCheck> saida;
  for (const auto & spec : kServers) {
    if (spec.navegacao && simulateNav) {
      continue;
    }
    ServerCheck c;
    c.action = QString::fromUtf8(spec.action);
    c.rotulo = QString::fromUtf8(spec.rotulo);
    const std::string alvo = std::string(spec.action) + "/_action/send_goal";
    c.presente = servicos.count(alvo) > 0;
    saida.push_back(c);
  }
  return saida;
}

caramelo_msgs::action::RunMission::Goal MissionBridge::buildGoal(
  const Options & opts, bool dryRun, const QString & actionsYaml) const
{
  RunMission::Goal goal;
  goal.task_yaml = opts.taskYaml.toStdString();
  goal.map_name = opts.mapName.toStdString();
  goal.map_dir = opts.mapDir.toStdString();
  goal.dry_run = dryRun;
  goal.simulate_nav = opts.simulateNav;
  goal.use_lidar_refine = opts.useLidarRefine;
  goal.finish_dock_id = opts.finishDockId.toStdString();
  goal.skip_startup_home = opts.skipStartupHome;
  goal.actions_yaml = actionsYaml.toStdString();
  goal.preflight_timeout = opts.preflightTimeout;
  return goal;
}

void MissionBridge::requestPlan(const Options & opts)
{
  send(buildGoal(opts, true, QString()), true);
}

void MissionBridge::run(const Options & opts, const QString & actionsYaml)
{
  send(buildGoal(opts, false, actionsYaml), false);
}

void MissionBridge::send(const RunMission::Goal & goal, bool dryRun)
{
  if (busy_) {
    emit finished(false, "Ja existe uma missao em andamento.");
    return;
  }
  if (!serverReady()) {
    const QString msg =
      "Servidor de missao indisponivel (/caramelo/run_mission). O stack esta no ar?";
    if (dryRun) {
      emit planReady(false, {}, QString(), msg);
    } else {
      emit finished(false, msg);
    }
    return;
  }

  setBusy(true);

  rclcpp_action::Client<RunMission>::SendGoalOptions options;

  options.goal_response_callback =
    [this, dryRun](GoalHandle::SharedPtr handle) {
      if (!handle) {
        setBusy(false);
        const QString msg = "Goal recusado pelo servidor de missao.";
        if (dryRun) {
          emit planReady(false, {}, QString(), msg);
        } else {
          emit finished(false, msg);
        }
        return;
      }
      std::lock_guard<std::mutex> lk(goal_mtx_);
      goal_handle_ = handle;
    };

  // O feedback da action carrega o mesmo MissionStatus que ja' chega pelo
  // topico; deixamos o topico ser a fonte unica para nao publicar o passo duas
  // vezes na tela.
  options.result_callback =
    [this, dryRun](const GoalHandle::WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lk(goal_mtx_);
        goal_handle_.reset();
      }
      setBusy(false);

      const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->success;
      const QString mensagem = result.result ?
        QString::fromStdString(result.result->message) :
        QString("Sem resposta do servidor.");

      if (dryRun) {
        QStringList linhas;
        if (result.result) {
          for (const auto & row : result.result->plan_rows) {
            linhas << QString::fromStdString(row);
          }
        }
        const QString yaml = result.result ?
          QString::fromStdString(result.result->actions_yaml) : QString();
        emit planReady(ok, linhas, yaml, mensagem);
        return;
      }

      if (result.code == rclcpp_action::ResultCode::CANCELED) {
        emit finished(false, "Missao abortada pelo operador.");
        return;
      }
      emit finished(ok, mensagem);
    };

  client_->async_send_goal(goal, options);
}

void MissionBridge::abort()
{
  GoalHandle::SharedPtr handle;
  {
    std::lock_guard<std::mutex> lk(goal_mtx_);
    handle = goal_handle_;
  }
  if (!handle) {
    emit finished(false, "Nenhuma missao em andamento para abortar.");
    return;
  }
  client_->async_cancel_goal(handle);
}

void MissionBridge::setBusy(bool busy)
{
  if (busy_ == busy) {
    return;
  }
  busy_ = busy;
  emit busyChanged(busy_);
}

void MissionBridge::onStatus(const caramelo_msgs::msg::MissionStatus::SharedPtr msg)
{
  MissionProgress p;
  p.state = toState(msg->state);
  p.missionId = QString::fromStdString(msg->mission_id);
  p.taskId = QString::fromStdString(msg->task_id);
  p.index = msg->action_index;
  p.total = msg->action_total;
  p.kind = QString::fromStdString(msg->action_kind);
  p.target = QString::fromStdString(msg->action_target);
  p.stage = QString::fromStdString(msg->stage);
  p.message = QString::fromStdString(msg->message);
  p.elapsed = msg->elapsed;
  emit progress(p);
}
