#include "modules/competicao/competicao_module.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QListWidget>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>

#include <yaml-cpp/yaml.h>

#include "bridge/mission_bridge.hpp"
#include "bridge/ros_bridge.hpp"
#include "modules/mapas/mapas_module.hpp"
#include "widgets/seletor_arena.hpp"

namespace
{

// Subpastas de <ws>/missions varridas, nesta ordem. "tasks geradas" guarda os
// arquivos OFICIAIS do @Work Commander (formato 2026, o principal); a raiz
// guarda as tasks escritas a mao (formatos anteriores). missions/exemplos/ sao
// YAMLs de ACOES (nivel baixo), nao de tarefa, e ficam de fora.
const char * kSubpastasDeTasks[] = {"tasks geradas", ""};

// Le o task_id de um YAML de competicao. Serve tambem de filtro: os arquivos
// vizinhos (README, YAML de acoes) nao tem task_id e nao sao tarefas.
QString lerTaskId(const QString & caminho)
{
  try {
    const YAML::Node raiz = YAML::LoadFile(caminho.toStdString());
    if (raiz.IsMap() && raiz["task_id"]) {
      return QString::fromStdString(raiz["task_id"].as<std::string>(""));
    }
  } catch (const std::exception &) {
    // YAML invalido nao e' tarefa; segue o baile.
  }
  return QString();
}

// "BMT  (tasks geradas/BMT_seed974164_equipe.yaml)": o task_id na frente, o
// arquivo atras -- duas tasks com o mesmo id (teste a mao e oficial) precisam
// ser distinguiveis na lista.
QString rotuloDaTarefa(const QString & caminho, const QString & raizMissions)
{
  const QString id = lerTaskId(caminho);
  const QString relativo = QDir(raizMissions).relativeFilePath(caminho);
  return id.isEmpty() ? relativo : (id + "  (" + relativo + ")");
}

}  // namespace

CompeticaoModule::CompeticaoModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge), mission_(bridge->mission())
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Competicao");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  // --- 1. Tarefa e mapa ---
  auto * grupoTarefa = new QGroupBox("1. Tarefa");
  auto * form = new QFormLayout(grupoTarefa);
  tarefa_ = new QComboBox();
  // Lista FIXA das arenas que existem, do mesmo seletor das outras telas.
  // Sintoma que isto conserta: a lista era montada aqui e mostrava QUALQUER
  // subpasta de maps/ (inclusive .lixeira e pastas sem map.yaml), e ainda
  // selecionava "arena3_520" cravado no codigo -- com outra arena carregada no
  // robo, a missao subia com o mapa errado e ninguem via a troca.
  mapa_ = new SeletorArena(bridge);
  form->addRow("Task:", tarefa_);
  form->addRow("Mapa:", mapa_);

  origem_tarefas_ = new QLabel();
  origem_tarefas_->setObjectName("msgCartao");
  origem_tarefas_->setWordWrap(true);
  form->addRow("", origem_tarefas_);

  motivo_ = new QLabel();
  motivo_->setObjectName("motivoCartao");
  motivo_->setWordWrap(true);
  form->addRow("", motivo_);
  layout->addWidget(grupoTarefa);

  auto * grupoOpcoes = new QGroupBox("Opcoes");
  auto * opcoes = new QVBoxLayout(grupoOpcoes);
  simular_nav_ = new QCheckBox("Simular navegacao (sem Nav2)");
  refino_lidar_ = new QCheckBox("Refinar docking com LiDAR");
  ir_ao_finish_ = new QCheckBox("Ir ao FINISH ao terminar");
  home_no_inicio_ = new QCheckBox("Levar o braco para home antes de comecar");
  // Defaults iguais aos do run_mission/bt_yaml_executor, para a GUI e o
  // terminal nao divergirem (use_lidar_refine e startup_home sao true la').
  refino_lidar_->setChecked(true);
  ir_ao_finish_->setChecked(true);
  home_no_inicio_->setChecked(true);
  opcoes->addWidget(simular_nav_);
  opcoes->addWidget(refino_lidar_);
  opcoes->addWidget(ir_ao_finish_);
  opcoes->addWidget(home_no_inicio_);
  layout->addWidget(grupoOpcoes);

  // --- 2. Plano ---
  auto * grupoPlano = new QGroupBox("2. Plano");
  auto * planoLayout = new QVBoxLayout(grupoPlano);
  ver_plano_ = new QPushButton("Ver plano (nao move o robo)");
  planoLayout->addWidget(ver_plano_);
  titulo_lista_ = new QLabel("Nenhum plano gerado ainda.");
  titulo_lista_->setWordWrap(true);
  planoLayout->addWidget(titulo_lista_);
  lista_ = new QListWidget();
  lista_->setMinimumHeight(150);
  planoLayout->addWidget(lista_);
  layout->addWidget(grupoPlano);

  // --- 3. Pre-flight ---
  auto * grupoPre = new QGroupBox("3. Pre-flight");
  auto * preLayout = new QVBoxLayout(grupoPre);
  verificar_ = new QPushButton("Verificar servidores");
  preLayout->addWidget(verificar_);
  servidores_ = new QListWidget();
  servidores_->setMinimumHeight(110);
  preLayout->addWidget(servidores_);
  layout->addWidget(grupoPre);

  // --- 4. Execucao ---
  auto * grupoExec = new QGroupBox("4. Execucao");
  auto * execLayout = new QVBoxLayout(grupoExec);
  executar_ = new QPushButton("EXECUTAR MISSAO");
  executar_->setObjectName("acaoPrimaria");
  executar_->setMinimumHeight(48);
  execLayout->addWidget(executar_);
  // Abortar fica SEMPRE visivel e sempre habilitado enquanto houver missao:
  // procurar o botao de parada com o robo andando e' o pior momento possivel.
  abortar_ = new QPushButton("ABORTAR");
  abortar_->setMinimumHeight(40);
  abortar_->setEnabled(false);
  execLayout->addWidget(abortar_);

  barra_ = new QProgressBar();
  barra_->setRange(0, 1);
  barra_->setValue(0);
  barra_->setFormat("%v / %m passos");
  execLayout->addWidget(barra_);

  estado_ = new QLabel("Estado: parado");
  estado_->setObjectName("estadoAtual");
  estado_->setWordWrap(true);
  execLayout->addWidget(estado_);

  detalhe_ = new QLabel("—");
  detalhe_->setWordWrap(true);
  execLayout->addWidget(detalhe_);

  // O que o robo esta' FALANDO (manip_audio/speech_node -> /manip/speech_status).
  // Quem opera de longe do alto-falante le' aqui o mesmo que o robo narra.
  voz_ = new QLabel("Voz: —");
  voz_->setObjectName("msgCartao");
  voz_->setWordWrap(true);
  execLayout->addWidget(voz_);
  layout->addWidget(grupoExec);

  layout->addStretch();

  recarregarTarefas();
  verificarServidores();
  atualizarDisponibilidade();

  // --- ligacoes ---
  connect(
    ver_plano_, &QPushButton::clicked, this, [this]() {
      if (tarefa_->count() == 0) {
        titulo_lista_->setText(
          "Nenhuma task encontrada em " + raizDoWorkspace() + "/missions.");
        return;
      }
      if (mapa_->vazio() || mapa_->arena().isEmpty()) {
        titulo_lista_->setText(
          "Escolha o mapa da prova. Este robo ainda nao tem nenhum mapa salvo: "
          "crie um em Mapeamento antes de planejar a missao.");
        return;
      }
      lista_->clear();
      actions_yaml_.clear();
      titulo_lista_->setText("Gerando plano (task_planner)...");
      MissionBridge::Options o;
      o.taskYaml = tarefa_->currentData().toString();
      o.mapName = mapa_->arena();
      o.simulateNav = simular_nav_->isChecked();
      o.useLidarRefine = refino_lidar_->isChecked();
      o.finishDockId = ir_ao_finish_->isChecked() ? "FINISH" : "";
      o.skipStartupHome = !home_no_inicio_->isChecked();
      mission_->requestPlan(o);
    });

  connect(verificar_, &QPushButton::clicked, this, &CompeticaoModule::verificarServidores);

  connect(
    executar_, &QPushButton::clicked, this, [this]() {
      if (tarefa_->count() == 0) {
        estado_->setText("Estado: sem task selecionada.");
        return;
      }
      if (mapa_->vazio() || mapa_->arena().isEmpty()) {
        estado_->setText("Estado: sem mapa escolhido — a missao nao pode comecar.");
        return;
      }
      MissionBridge::Options o;
      o.taskYaml = tarefa_->currentData().toString();
      o.mapName = mapa_->arena();
      o.simulateNav = simular_nav_->isChecked();
      o.useLidarRefine = refino_lidar_->isChecked();
      o.finishDockId = ir_ao_finish_->isChecked() ? "FINISH" : "";
      o.skipStartupHome = !home_no_inicio_->isChecked();
      // Reusa o plano ja' mostrado ao operador: se replanejassemos aqui, o que
      // roda poderia nao ser o que ele acabou de aprovar na tela.
      mission_->run(o, actions_yaml_);
      estado_->setText("Estado: iniciando...");
    });

  connect(abortar_, &QPushButton::clicked, this, [this]() {mission_->abort();});

  connect(
    mission_, &MissionBridge::planReady, this,
    [this](bool ok, const QStringList & rows, const QString & yaml, const QString & msg) {
      lista_->clear();
      if (!ok) {
        titulo_lista_->setText("Falha ao gerar o plano: " + msg.trimmed());
        return;
      }
      actions_yaml_ = yaml;
      for (const QString & r : rows) {
        lista_->addItem(r.trimmed());
      }
      titulo_lista_->setText(
        QString("Plano com %1 acoes (%2). O executor ainda acrescenta o home inicial "
        "e a ida ao FINISH.").arg(rows.size()).arg(QFileInfo(yaml).fileName()));
    });

  connect(mission_, &MissionBridge::progress, this, &CompeticaoModule::aoProgresso);
  connect(mission_, &MissionBridge::busyChanged, this, &CompeticaoModule::aoMudarOcupado);
  connect(
    mission_, &MissionBridge::finished, this, [this](bool ok, const QString & msg) {
      estado_->setText(QString("Estado: %1").arg(ok ? "concluida" : "encerrada"));
      detalhe_->setText(msg.trimmed());
      barra_->setRange(0, 1);
      barra_->setValue(ok ? 1 : 0);
    });
  connect(
    mission_, &MissionBridge::speechStatus, this,
    [this](const QString & s) {voz_->setText("Voz: " + s);});
}

QString CompeticaoModule::raizDoWorkspace()
{
  // <ws>/src/caramelo_mapping/maps -> <ws>
  QDir dir(MapasModule::mapsDir());
  bool ok = true;
  for (int i = 0; i < 3 && ok; ++i) {
    ok = dir.cdUp();
  }
  if (ok && QDir(dir.filePath("missions")).exists()) {
    return dir.absolutePath();
  }
  return QDir::homePath() + "/caramelo_ws";
}

QStringList CompeticaoModule::descobrirTarefas()
{
  const QString raiz = raizDoWorkspace() + "/missions";
  QStringList achados;
  for (const char * sub : kSubpastasDeTasks) {
    QDir dir(QString(sub).isEmpty() ? raiz : raiz + "/" + sub);
    if (!dir.exists()) {
      continue;
    }
    for (const QFileInfo & fi : dir.entryInfoList({"*.yaml"}, QDir::Files, QDir::Name)) {
      if (!lerTaskId(fi.absoluteFilePath()).isEmpty()) {
        achados << fi.absoluteFilePath();
      }
    }
  }
  return achados;
}

void CompeticaoModule::recarregarTarefas()
{
  tarefa_->clear();
  const QString raiz = raizDoWorkspace() + "/missions";
  for (const QString & caminho : descobrirTarefas()) {
    tarefa_->addItem(rotuloDaTarefa(caminho, raiz), caminho);
  }
  origem_tarefas_->setText(
    QString("Tasks lidas de %1 (e tasks geradas/ — formato 2026 do @Work Commander). "
    "Na prova, copie o YAML recebido para essa pasta.").arg(raiz));
  if (tarefa_->count() == 0) {
    titulo_lista_->setText(
      "Nenhuma task encontrada. Esperado: YAML com task_id em " + raiz +
      " ou em " + raiz + "/tasks geradas.");
  }
}

void CompeticaoModule::atualizarDisponibilidade()
{
  const bool temTarefa = tarefa_->count() > 0;
  const bool temMapa = !mapa_->vazio() && !mapa_->arena().isEmpty();
  const bool ocupado = abortar_->isEnabled();

  ver_plano_->setEnabled(temTarefa && temMapa && !ocupado);
  executar_->setEnabled(temTarefa && temMapa && !ocupado);

  QStringList faltas;
  if (!temTarefa) {
    // O caminho tecnico de onde as tarefas sao lidas continua na mensagem do
    // painel do plano; aqui embaixo do formulario quem le' quer saber o que
    // fazer, nao em que pasta procurar.
    faltas << "nenhuma tarefa de prova foi encontrada neste robo";
  }
  if (!temMapa) {
    faltas << "este robo ainda nao tem nenhum mapa salvo — crie um em Mapeamento";
  }
  motivo_->setText(
    faltas.isEmpty() ?
    QString() :
    "Planejar e executar estao desligados porque " + faltas.join("; e ") + ".");
}

void CompeticaoModule::verificarServidores()
{
  servidores_->clear();

  const bool servidorMissao = mission_->serverReady();
  auto * itemMissao = new QListWidgetItem(
    QString("%1  Servidor de missao (/caramelo/run_mission)")
    .arg(servidorMissao ? "OK  " : "FALTA"));
  if (!servidorMissao) {
    // Nunca so' "indisponivel": o operador precisa saber o que fazer.
    itemMissao->setToolTip(
      "Suba o mission_server: ros2 launch caramelo_gui caramelo_gui.launch.py "
      "(use_mission_server:=true, default) ou ros2 launch caramelo_navigation "
      "mission_server.launch.py");
  }
  servidores_->addItem(itemMissao);

  int faltando = 0;
  for (const auto & c : mission_->checkServers(simular_nav_->isChecked())) {
    servidores_->addItem(
      QString("%1  %2 (%3)").arg(c.presente ? "OK  " : "FALTA", c.rotulo, c.action));
    if (!c.presente) {
      ++faltando;
    }
  }

  if (!servidorMissao || faltando > 0) {
    detalhe_->setText(
      QString("Pre-flight: %1 item(ns) faltando. Suba o stack com "
      "'ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=<mapa>'"
      "%2.")
      .arg(faltando + (servidorMissao ? 0 : 1))
      .arg(servidorMissao ? "" : " e o mission_server (launch da GUI)"));
  } else {
    detalhe_->setText("Pre-flight: tudo no ar (AMT1 exige ainda o /amt1_sort).");
  }
}

void CompeticaoModule::prepararChecklist(int total)
{
  // As linhas do PLANO e os PASSOS da arvore nao se alinham: o executor
  // acrescenta o home inicial e o par home+FINISH no fim, que nao existem no
  // actions.yaml. Por isso, quando a execucao comeca, a lista e' reconstruida a
  // partir do que o proprio executor reporta -- nunca indexada pelo plano.
  if (total <= 0 || total_passos_ == total) {
    return;
  }
  total_passos_ = total;
  lista_->clear();
  for (int i = 0; i < total; ++i) {
    lista_->addItem(QString("%1. —").arg(i + 1));
  }
  barra_->setRange(0, total);
  barra_->setValue(0);
  titulo_lista_->setText("Execucao em andamento (passos reportados pelo executor).");
}

void CompeticaoModule::aoProgresso(const MissionProgress & p)
{
  estado_->setText(
    QString("Estado: %1   •   %2 s")
    .arg(MissionProgress::label(p.state))
    .arg(p.elapsed, 0, 'f', 0));

  if (!p.message.isEmpty()) {
    detalhe_->setText(p.message);
  }

  if (p.state == MissionProgress::State::Running) {
    if (p.index >= 0 && p.total > 0) {
      prepararChecklist(p.total);
      if (p.index < lista_->count()) {
        const QString marca = p.stage == "concluido" ? "OK" :
          (p.stage == "falhou" ? "FALHOU" : ">>");
        lista_->item(p.index)->setText(
          QString("%1. [%2] %3 %4").arg(p.index + 1).arg(marca, p.kind, p.target));
        lista_->setCurrentRow(p.index);
        lista_->scrollToItem(lista_->item(p.index));
      }
      barra_->setValue(p.stage == "concluido" ? p.index + 1 : p.index);
    } else if (ultimo_estado_ != MissionProgress::State::Running) {
      // Sem indice de passo (executor atual): barra em "ocupado", e o plano
      // continua listado como referencia do que vai acontecer.
      barra_->setRange(0, 0);
      titulo_lista_->setText(
        "Executando. O executor nao reporta o passo atual; acompanhe pelo "
        "detalhe abaixo e pela voz do robo.");
    }
  }

  if (p.terminal()) {
    if (barra_->maximum() == 0) {
      barra_->setRange(0, 1);
    }
    barra_->setValue(p.state == MissionProgress::State::Done ? barra_->maximum() : 0);
    total_passos_ = 0;
  }
  ultimo_estado_ = p.state;
}

void CompeticaoModule::aoMudarOcupado(bool ocupado)
{
  abortar_->setEnabled(ocupado);
  tarefa_->setEnabled(!ocupado);
  mapa_->setEnabled(!ocupado);
  // atualizarDisponibilidade le' o estado de abortar_ para saber se a missao
  // esta' em voo, entao ele precisa ser ajustado ANTES desta chamada.
  atualizarDisponibilidade();
}
