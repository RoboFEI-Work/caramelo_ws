#pragma once

// Modulo Mapas: lista os mapas de caramelo_mapping/maps com miniatura (Qt le
// PGM nativamente), usa um mapa em runtime (/map_server/load_map), renomeia,
// duplica, exclui (com confirmacao -> vai para a lixeira, nunca apaga direto)
// e salva o mapa atual do SLAM (/map_saver/save_map).

#include <QString>
#include <QWidget>

class RosBridge;
class QLabel;
class QListWidget;

class MapasModule : public QWidget
{
  Q_OBJECT

public:
  explicit MapasModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  void refresh();
  QString selectedMap() const;
  static QString mapsDir();

  RosBridge * bridge_;
  QListWidget * lista_ = nullptr;
  QLabel * status_ = nullptr;
};
