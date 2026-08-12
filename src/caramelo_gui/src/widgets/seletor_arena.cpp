#include "widgets/seletor_arena.hpp"

#include <QDir>
#include <QFile>

#include "bridge/robot_state.hpp"
#include "bridge/ros_bridge.hpp"
#include "modules/mapas/mapas_module.hpp"

namespace
{
const char * kSufixoAtiva = "   (ativa)";
}  // namespace

SeletorArena::SeletorArena(RosBridge * bridge, QWidget * parent)
: QComboBox(parent), bridge_(bridge)
{
  setEditable(false);   // lista fixa: e' o ponto do widget
  recarregar();

  // Se a arena do robo mudar em Mapas, todas as telas passam a apontar para ela.
  if (bridge_ && bridge_->robotState()) {
    connect(
      bridge_->robotState(), &RobotState::mapNameChanged, this,
      [this](const QString &) {recarregar();});
  }
}

QString SeletorArena::arena() const
{
  QString nome = currentText();
  if (nome.endsWith(kSufixoAtiva)) {
    nome.chop(static_cast<int>(qstrlen(kSufixoAtiva)));
  }
  return nome;
}

void SeletorArena::recarregar()
{
  const QString selecionadaAntes = arena();
  const QString ativa = (bridge_ && bridge_->robotState()) ?
    bridge_->robotState()->mapName() : QString();

  blockSignals(true);
  clear();

  QDir dir(MapasModule::mapsDir());
  QStringList arenas;
  for (const QString & nome : dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name)) {
    // Pasta so' e' arena se tiver map.yaml; a lixeira e resquicios ficam de fora.
    if (!nome.startsWith('.') && QFile::exists(dir.filePath(nome + "/map.yaml"))) {
      arenas << nome;
    }
  }

  for (const QString & nome : arenas) {
    addItem(nome == ativa ? nome + kSufixoAtiva : nome, nome);
  }

  // Preferencia: manter o que ja estava escolhido; senao, a arena do robo.
  const QString alvo = arenas.contains(selecionadaAntes) ? selecionadaAntes : ativa;
  for (int i = 0; i < count(); ++i) {
    if (itemData(i).toString() == alvo) {
      setCurrentIndex(i);
      break;
    }
  }
  blockSignals(false);
}
