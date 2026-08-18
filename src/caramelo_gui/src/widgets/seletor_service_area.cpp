#include "widgets/seletor_service_area.hpp"

#include <QMap>

#include "bridge/robot_state.hpp"
#include "bridge/ros_bridge.hpp"
#include "modules/mapas/mapas_module.hpp"
#include "widgets/combo_aviso.hpp"
#include "widgets/map_preview.hpp"
#include "widgets/seletor_tipo_area.hpp"

namespace
{
const char * kSufixoSemPose = "  (sem pose)";
const char * kNenhuma = "esta arena nao tem areas de servico";
const char * kSemArena = "escolha uma arena primeiro";
// Guarda o tipo tecnico junto do item, para a tela nao ter que reabrir o YAML
// so' para saber se a area escolhida e' prateleira ou bancada.
const int kPapelTipo = Qt::UserRole + 1;
}  // namespace

SeletorServiceArea::SeletorServiceArea(RosBridge * bridge, QWidget * parent)
: QComboBox(parent), bridge_(bridge)
{
  setEditable(false);   // lista fixa: e' o ponto do widget

  if (bridge_ && bridge_->robotState()) {
    arena_ = bridge_->robotState()->mapName();
    connect(
      bridge_->robotState(), &RobotState::mapNameChanged, this,
      [this](const QString & nome) {
        if (segue_robo_) {
          arena_ = nome;
          recarregar();
        }
      });
  }

  recarregar();
}

void SeletorServiceArea::setArena(const QString & arena)
{
  segue_robo_ = false;
  if (arena_ == arena) {
    return;
  }
  arena_ = arena;
  recarregar();
}

QString SeletorServiceArea::area() const
{
  // Id puro no itemData: o texto pode carregar "(sem pose)" ou ser um aviso.
  return itemData(currentIndex()).toString();
}

QString SeletorServiceArea::tipo() const
{
  return itemData(currentIndex(), kPapelTipo).toString();
}

bool SeletorServiceArea::semPose() const
{
  return currentText().endsWith(kSufixoSemPose);
}

void SeletorServiceArea::recarregar()
{
  const QString escolhidaAntes = area();

  blockSignals(true);
  clear();

  QMap<QString, AreaDeServico> areas;
  QString erro;
  if (!arena_.isEmpty()) {
    areas = lerAreasDaArena(MapasModule::mapsDir() + "/" + arena_, &erro);
  }

  int semPose = 0;
  for (auto it = areas.constBegin(); it != areas.constEnd(); ++it) {
    const bool zerada = it.value().pose.placeholder;
    addItem(zerada ? (it.key() + kSufixoSemPose) : it.key(), it.key());
    const int indice = count() - 1;
    setItemData(indice, it.value().tipo, kPapelTipo);
    // Dica por item em portugues: "WS1" nao diz nada a quem nunca leu o
    // regulamento; "Bancada de trabalho" diz.
    setItemData(indice, rotuloDeTipoDeArea(it.value().tipo), Qt::ToolTipRole);
    if (zerada) {
      ++semPose;
    }
  }

  vazio_ = areas.isEmpty();
  if (vazio_) {
    adicionarAvisoDeListaVazia(this, arena_.isEmpty() ? kSemArena : kNenhuma);
    setToolTip(
      erro.isEmpty() ?
      "Nenhuma area de servico gravada nesta arena. Grave as areas em Mapeamento." :
      erro);
  } else if (semPose > 0) {
    setToolTip(
      "As areas marcadas com \"(sem pose)\" ainda nao tiveram a posicao gravada: "
      "pare o robo de frente para a estacao e grave a pose em Mapeamento.");
  } else {
    setToolTip(QString());
  }

  for (int i = 0; i < count(); ++i) {
    if (!escolhidaAntes.isEmpty() && itemData(i).toString() == escolhidaAntes) {
      setCurrentIndex(i);
      break;
    }
  }
  blockSignals(false);
}
