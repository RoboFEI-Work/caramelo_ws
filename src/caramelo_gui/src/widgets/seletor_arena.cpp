#include "widgets/seletor_arena.hpp"

#include <QDir>
#include <QFile>

#include "bridge/robot_state.hpp"
#include "bridge/ros_bridge.hpp"
#include "modules/mapas/mapas_module.hpp"
#include "widgets/combo_aviso.hpp"

namespace
{
const char * kSufixoAtiva = "   (ativa)";
// Lista vazia era um combo em branco: o operador escolhia "nada", arena()
// devolvia "" e a tela seguia montando ".../maps/" -- gravando fora de qualquer
// arena, sem erro na tela. Agora a lista diz o que aconteceu.
const char * kNenhuma = "nenhuma arena encontrada";
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
  // O nome puro vive no itemData; o texto pode carregar o sufixo "(ativa)" ou
  // ser o aviso de lista vazia, e nenhum dos dois e' nome de pasta.
  return itemData(currentIndex()).toString();
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

  vazio_ = arenas.isEmpty();
  if (vazio_) {
    adicionarAvisoDeListaVazia(this, kNenhuma);
    setToolTip("Nenhum mapa salvo no robo. Crie um em Mapeamento.");
  } else {
    setToolTip(QString());
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
