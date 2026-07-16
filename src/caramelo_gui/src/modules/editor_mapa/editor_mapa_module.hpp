#pragma once

// Editor de Mapa (raster): limpeza de ruido do map.pgm no estilo dos
// comissionadores comerciais (MiR/OTTO) — pincel com 3 modos (Parede=preto,
// Livre=branco, Desconhecido=cinza 205), tamanho ajustavel, zoom, desfazer,
// salvar com BACKUP automatico, ajuste da ORIGEM (map.yaml) e recarregar no
// map_server. Edita SO o mapa base; keepout continua em mascara separada.

#include <QImage>
#include <QVector>
#include <QWidget>

class RosBridge;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QScrollArea;
class QSlider;

// Tela de pintura do mapa (QImage em escala de cinza + pincel).
class MapCanvas : public QWidget
{
  Q_OBJECT

public:
  explicit MapCanvas(QWidget * parent = nullptr);

  bool loadPgm(const QString & path);
  bool savePgm(const QString & path) const;
  void setBrushValue(int value) {brush_value_ = value;}
  void setBrushSize(int size) {brush_size_ = size;}
  void setZoom(double zoom);
  double zoom() const {return zoom_;}
  void undo();
  bool isModified() const {return modified_;}
  void clearModified() {modified_ = false;}

signals:
  void edited();

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  QSize sizeHint() const override;

private:
  void paintAt(const QPoint & widget_pos);

  QImage img_;
  double zoom_ = 3.0;
  int brush_value_ = 254;   // Livre
  int brush_size_ = 3;
  bool modified_ = false;
  QVector<QImage> undo_stack_;
};

class EditorMapaModule : public QWidget
{
  Q_OBJECT

public:
  explicit EditorMapaModule(RosBridge * bridge, QWidget * parent = nullptr);

private:
  QString mapDir() const;
  void loadOrigin();
  void applyOrigin();

  RosBridge * bridge_;
  MapCanvas * canvas_ = nullptr;
  QScrollArea * scroll_ = nullptr;
  QLineEdit * map_name_ = nullptr;
  QComboBox * ferramenta_ = nullptr;
  QSlider * tamanho_ = nullptr;
  QDoubleSpinBox * origem_x_ = nullptr;
  QDoubleSpinBox * origem_y_ = nullptr;
  QLabel * status_ = nullptr;
};
