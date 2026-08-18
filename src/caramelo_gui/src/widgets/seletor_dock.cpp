#include "widgets/seletor_dock.hpp"

#include <QMap>

#include "bridge/robot_state.hpp"
#include "bridge/ros_bridge.hpp"
#include "modules/mapas/mapas_module.hpp"
#include "widgets/combo_aviso.hpp"
#include "widgets/map_preview.hpp"

namespace
{
// Marca o dock que ainda esta em [0,0,0]. Mandar o robo para um desses e' manda-
// -lo para a origem do mapa, quase sempre no meio da arena.
const char * kSufixoSemPose = "  (sem pose)";
const char * kNenhum = "esta arena nao tem docks";
const char * kSemArena = "escolha uma arena primeiro";
}  // namespace

SeletorDock::SeletorDock(RosBridge * bridge, QWidget * parent)
: QComboBox(parent), bridge_(bridge)
{
  setEditable(false);   // lista fixa: e' o ponto do widget

  if (bridge_ && bridge_->robotState()) {
    arena_ = bridge_->robotState()->mapName();
    connect(
      bridge_->robotState(), &RobotState::mapNameChanged, this,
      [this](const QString & nome) {
        // Enquanto a tela nao escolheu arena, seguir o robo. Depois disso a
        // escolha da tela manda: trocar a arena do robo debaixo de um formulario
        // aberto trocaria o dock sem o operador perceber.
        if (segue_robo_) {
          arena_ = nome;
          recarregar();
        }
      });
  }

  recarregar();
}

void SeletorDock::setArena(const QString & arena)
{
  segue_robo_ = false;
  if (arena_ == arena) {
    return;
  }
  arena_ = arena;
  recarregar();
}

QString SeletorDock::dock() const
{
  // Id puro no itemData: o texto pode carregar "(sem pose)" ou ser um aviso.
  return itemData(currentIndex()).toString();
}

bool SeletorDock::semPose() const
{
  return currentText().endsWith(kSufixoSemPose);
}

void SeletorDock::recarregar()
{
  const QString escolhidoAntes = dock();

  blockSignals(true);
  clear();

  QMap<QString, PoseMapa> docks;
  QString erro;
  if (!arena_.isEmpty()) {
    docks = lerDocksDaArena(MapasModule::mapsDir() + "/" + arena_, &erro);
  }

  int semPose = 0;
  for (auto it = docks.constBegin(); it != docks.constEnd(); ++it) {
    const QString texto = it.value().placeholder ?
      (it.key() + kSufixoSemPose) : it.key();
    addItem(texto, it.key());
    if (it.value().placeholder) {
      ++semPose;
    }
  }

  vazio_ = docks.isEmpty();
  if (vazio_) {
    adicionarAvisoDeListaVazia(this, arena_.isEmpty() ? kSemArena : kNenhum);
    setToolTip(
      erro.isEmpty() ?
      "Nenhuma estacao de docking gravada nesta arena. Grave as estacoes em Mapeamento." :
      erro);
  } else if (semPose > 0) {
    setToolTip(
      "As estacoes marcadas com \"(sem pose)\" ainda nao tiveram a posicao "
      "gravada: leve o robo ate' cada uma e grave a pose em Mapeamento.");
  } else {
    setToolTip(QString());
  }

  // Mantem a escolha anterior quando ela sobrevive a troca de arena.
  for (int i = 0; i < count(); ++i) {
    if (!escolhidoAntes.isEmpty() && itemData(i).toString() == escolhidoAntes) {
      setCurrentIndex(i);
      break;
    }
  }
  blockSignals(false);
}
