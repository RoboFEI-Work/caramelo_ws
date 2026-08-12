#pragma once

// Tela Mapas: escolher a arena em que o robo esta, VENDO o que esta salvo nela.
//
// Antes era uma lista com miniatura do map.pgm. A miniatura mostra as paredes e
// esconde tudo que decide se a missao roda: docks sem pose gravada, service
// areas zeradas, ausencia de paredes virtuais, onde fica o ponto de inicio.
// Agora a arena selecionada aparece no MapPreview com essas camadas por cima, e
// os problemas viram avisos em portugues.
//
// "Usar esta arena" carrega o mapa no robo E grava a escolha em
// ~/.config/caramelo/robot_state.yaml, para ele religar no mesmo lugar.

#include <QString>
#include <QWidget>

#include "widgets/map_preview.hpp"

class RosBridge;
class QLabel;
class QListWidget;

class MapasModule : public QWidget
{
  Q_OBJECT

public:
  explicit MapasModule(RosBridge * bridge, QWidget * parent = nullptr);

  // Pasta dos mapas (arvore fonte) — compartilhada com Mapeamento.
  static QString mapsDir();

private:
  QWidget * construirListaEAcoes();
  QWidget * construirDetalhes();
  void refresh();
  QString selectedMap() const;
  void mostrarResumo(const MapaCarregado & mapa);

  RosBridge * bridge_;
  QListWidget * lista_ = nullptr;
  MapPreview * preview_ = nullptr;
  QLabel * status_ = nullptr;
  QLabel * resumo_ = nullptr;
  QLabel * avisos_ = nullptr;
  QLabel * arena_ativa_ = nullptr;
};
