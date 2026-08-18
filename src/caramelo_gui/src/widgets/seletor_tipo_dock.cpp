#include "widgets/seletor_tipo_dock.hpp"

#include <QStringList>

#include "bridge/robot_state.hpp"
#include "bridge/ros_bridge.hpp"
#include "modules/mapas/mapas_module.hpp"
#include "widgets/map_preview.hpp"

namespace
{

struct RotuloDeDock
{
  const char * plugin;
  const char * rotulo;
  const char * ajuda;
};

// Rotulos dos plugins que o robo usa hoje. Plugin fora desta lista continua
// aparecendo pelo nome tecnico: esconder uma opcao que existe no arquivo seria
// pior que mostrar um nome feio.
const RotuloDeDock kRotulos[] = {
  {"caramelo_front_dock", "Aproximacao normal (bancada)",
    "Encosta de frente na estacao, parando a 25 cm. E' o padrao das bancadas."},
  {"caramelo_shelf_front_dock", "Aproximacao de prateleira",
    "Para mais longe (90 cm) porque a prateleira e' alta e o braco precisa de espaco."},
  {"caramelo_precision_front_dock", "Aproximacao de precisao",
    "Chega com tolerancia menor, para as mesas de encaixe."},
};

QString rotuloDe(const QString & plugin)
{
  for (const RotuloDeDock & r : kRotulos) {
    if (plugin == QString::fromLatin1(r.plugin)) {
      return QString("%1  (%2)").arg(QString::fromLatin1(r.rotulo), plugin);
    }
  }
  return plugin;
}

QString ajudaDe(const QString & plugin)
{
  for (const RotuloDeDock & r : kRotulos) {
    if (plugin == QString::fromLatin1(r.plugin)) {
      return QString::fromLatin1(r.ajuda);
    }
  }
  return QString();
}

}  // namespace

SeletorTipoDock::SeletorTipoDock(RosBridge * bridge, QWidget * parent)
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

void SeletorTipoDock::setArena(const QString & arena)
{
  segue_robo_ = false;
  if (arena_ == arena) {
    return;
  }
  arena_ = arena;
  recarregar();
}

QString SeletorTipoDock::tipoDock() const
{
  return itemData(currentIndex()).toString();
}

void SeletorTipoDock::setTipoDock(const QString & tipo)
{
  for (int i = 0; i < count(); ++i) {
    if (itemData(i).toString() == tipo) {
      setCurrentIndex(i);
      return;
    }
  }
}

void SeletorTipoDock::recarregar()
{
  const QString escolhidoAntes = tipoDock();

  blockSignals(true);
  clear();

  // lerTiposDeDockDaArena ja' devolve os plugins padrao quando a arena nao tem
  // docking.yaml: esta lista nunca fica vazia, entao nao ha' item de aviso.
  const QStringList tipos = lerTiposDeDockDaArena(MapasModule::mapsDir() + "/" + arena_);
  for (const QString & t : tipos) {
    addItem(rotuloDe(t), t);
    setItemData(count() - 1, ajudaDe(t), Qt::ToolTipRole);
  }

  for (int i = 0; i < count(); ++i) {
    if (!escolhidoAntes.isEmpty() && itemData(i).toString() == escolhidoAntes) {
      setCurrentIndex(i);
      break;
    }
  }
  blockSignals(false);
}
