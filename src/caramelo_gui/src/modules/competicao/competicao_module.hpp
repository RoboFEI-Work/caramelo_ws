#pragma once

// Modulo Competicao: rodar uma task de prova de ponta a ponta SEM abrir
// terminal. Ate' aqui a missao so' existia via run_mission.py, e o que mais
// consome tempo em prova era justamente subir e rodar uma task.
//
// A tela segue a ordem em que o operador pensa em prova:
//   1. Tarefa e mapa
//   2. Plano (dry-run: mostra o que vai acontecer antes de qualquer movimento)
//   3. Pre-flight (quais servidores faltam, e nao apenas "pronto/nao pronto")
//   4. Executar, com progresso passo a passo e ABORTAR sempre visivel

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

  // Coleta os YAMLs de tarefa (os que tem task_id) do share do manip_bt e da
  // pasta missions/ do workspace.
  static QStringList descobrirTarefas();

  RosBridge * bridge_ = nullptr;
  MissionBridge * mission_ = nullptr;

  QComboBox * tarefa_ = nullptr;
  SeletorArena * mapa_ = nullptr;
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

  QString actions_yaml_;   // plano ja gerado, reusado na execucao
  int total_passos_ = 0;
};
