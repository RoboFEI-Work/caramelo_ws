#pragma once

// Modulo Competicao: rodar uma task de prova de ponta a ponta SEM abrir
// terminal. E' o mesmo caminho do `ros2 run caramelo_navigation run_mission
// --map-name <mapa> --task missions/<task>.yaml`, servido pela action
// /caramelo/run_mission (caramelo_navigation/mission_server, que importa o
// run_mission.py: nada e' reimplementado aqui).
//
// A tela segue a ordem em que o operador pensa em prova:
//   1. Tarefa e mapa (tasks lidas de missions/ e missions/tasks geradas/,
//      formato 2026 do @Work Commander e os anteriores — so' YAML com task_id)
//   2. Plano (dry-run: so' o task_planner; nada se move)
//   3. Pre-flight (quais servidores faltam, e nao apenas "pronto/nao pronto")
//   4. Executar, com o que o executor esta' dizendo e ABORTAR sempre visivel
//
// O executor atual (bt_yaml_executor) nao publica progresso por passo: durante
// a execucao a barra fica em "ocupado" e o detalhe mostra a ultima linha do
// log do executor (repassada pelo mission_server). A voz do robo
// (/manip/speech_status) aparece embaixo, para quem esta' longe do alto-falante.

#include <QString>
#include <QWidget>

#include "bridge/mission_types.hpp"

class RosBridge;
class MissionBridge;
class SeletorArena;
class QCheckBox;
class QComboBox;
class QLabel;
class QListWidget;
class QProgressBar;
class QPushButton;

class CompeticaoModule : public QWidget
{
  Q_OBJECT

public:
  explicit CompeticaoModule(RosBridge * bridge, QWidget * parent = nullptr);

  // Raiz do workspace (<ws>/missions e' a fonte unica das tasks). Deduzida da
  // arvore fonte dos mapas (<ws>/src/caramelo_mapping/maps); sem ela, ~/caramelo_ws.
  static QString raizDoWorkspace();

  // YAMLs de tarefa (os que tem task_id) de <ws>/missions e <ws>/missions/tasks geradas.
  static QStringList descobrirTarefas();

private:
  void recarregarTarefas();
  void verificarServidores();

  // Liga/desliga o que depende de ter tarefa E mapa, e escreve na tela o
  // motivo quando algo fica desligado. Botao cinza sem explicacao le' como
  // "quebrado" -- o operador reinicia a interface achando que e' defeito.
  void atualizarDisponibilidade();
  void aoProgresso(const MissionProgress & p);
  void aoMudarOcupado(bool ocupado);
  void prepararChecklist(int total);
  MissionProgress::State ultimo_estado_ = MissionProgress::State::Idle;

  RosBridge * bridge_ = nullptr;
  MissionBridge * mission_ = nullptr;

  QComboBox * tarefa_ = nullptr;
  SeletorArena * mapa_ = nullptr;
  QLabel * origem_tarefas_ = nullptr;
  QLabel * motivo_ = nullptr;
  QCheckBox * simular_nav_ = nullptr;
  QCheckBox * refino_lidar_ = nullptr;
  QCheckBox * ir_ao_finish_ = nullptr;
  QCheckBox * home_no_inicio_ = nullptr;

  QPushButton * ver_plano_ = nullptr;
  QPushButton * verificar_ = nullptr;
  QPushButton * executar_ = nullptr;
  QPushButton * abortar_ = nullptr;

  QLabel * titulo_lista_ = nullptr;
  QListWidget * lista_ = nullptr;
  QListWidget * servidores_ = nullptr;
  QProgressBar * barra_ = nullptr;
  QLabel * estado_ = nullptr;
  QLabel * detalhe_ = nullptr;
  QLabel * voz_ = nullptr;

  QString actions_yaml_;   // plano ja gerado, reusado na execucao
  int total_passos_ = 0;
};
