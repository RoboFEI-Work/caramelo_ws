#pragma once

// Editor de Mapa (raster): limpeza de ruido do map.pgm no estilo dos
// comissionadores comerciais (MiR/OTTO) — pincel com 3 modos (Parede=preto,
// Livre=branco, Desconhecido=cinza 205), tamanho ajustavel, zoom, desfazer,
// salvar com BACKUP automatico, ajuste da ORIGEM (map.yaml + keepout_mask.yaml)
// e recarregar no map_server. Edita SO o mapa base; parede virtual e' outra
// coisa e tem editor proprio (editor_keepout.hpp).
//
// O MapCanvas daqui e' compartilhado com o editor de paredes virtuais, por isso
// ele sabe fazer mais do que este modulo usa: desenhar uma imagem de referencia
// por baixo, pintar a imagem editada como mascara vermelha, e ter ferramenta de
// retangulo e de linha.

#include <QImage>
#include <QPoint>
#include <QRect>
#include <QSize>
#include <QString>
#include <QVector>
#include <QWidget>

class RosBridge;
class SeletorArena;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QScrollArea;
class QSlider;

// Tela de pintura do mapa (QImage em escala de cinza + ferramentas).
class MapCanvas : public QWidget
{
  Q_OBJECT

public:
  // Pincel e' o traco livre. Retangulo e linha existem porque quase tudo que se
  // marca num mapa e' RETO: fita de piso, borda de rampa, corredor fechado.
  // Fazer uma reta a mao com pincel livre e' sofrimento e sai torto.
  enum class Ferramenta
  {
    Pincel,
    Retangulo,
    Linha
  };

  explicit MapCanvas(QWidget * parent = nullptr);

  bool loadPgm(const QString & path);
  bool savePgm(const QString & path) const;

  void setBrushValue(int value) {brush_value_ = value;}
  void setBrushSize(int size) {brush_size_ = size;}
  void setFerramenta(Ferramenta ferramenta) {ferramenta_ = ferramenta;}
  Ferramenta ferramenta() const {return ferramenta_;}

  // Imagem desenhada POR BAIXO, so' como referencia (nunca e' editada nem
  // salva). E' o map.pgm sob a mascara de paredes virtuais: pintar uma parede
  // sem ver onde ficam as paredes reais e' adivinhacao.
  void setImagemDeFundo(const QImage & fundo);

  // Modo mascara: em vez de desenhar a imagem editada em cinza, pinta VERMELHO
  // translucido onde ela esta bloqueada (preto). Uma mascara desenhada em cinza
  // por cima do mapa esconderia o mapa inteiro.
  void setModoMascara(bool mascara);

  void setZoom(double zoom);
  double zoom() const {return zoom_;}
  void ajustarPara(const QSize & area);   // zoom que faz a imagem inteira caber

  void undo();
  bool isModified() const {return modified_;}
  void clearModified() {modified_ = false;}
  bool temImagem() const {return !img_.isNull();}
  QSize tamanhoDaImagem() const {return img_.size();}

signals:
  void edited();

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  QSize sizeHint() const override;

private:
  QPoint paraImagem(const QPointF & pos) const;
  void guardarParaDesfazer();
  void pintarEm(const QPoint & pixel);
  void pintarLinha(const QPoint & de, const QPoint & ate);
  void pintarRetangulo(const QRect & area);
  void marcarEditado();
  void reconstruirOverlay() const;

  QImage img_;
  QImage fundo_;
  mutable QImage overlay_;        // cache do vermelho translucido (modo mascara)
  mutable bool overlay_valido_ = false;
  bool modo_mascara_ = false;

  double zoom_ = 3.0;
  int brush_value_ = 254;   // Livre
  int brush_size_ = 3;
  Ferramenta ferramenta_ = Ferramenta::Pincel;
  bool modified_ = false;

  bool arrastando_ = false;
  QPoint inicio_;
  QPoint ultimo_;
  QPoint atual_;

  QVector<QImage> undo_stack_;
};

class EditorMapaModule : public QWidget
{
  Q_OBJECT

public:
  explicit EditorMapaModule(RosBridge * bridge, QWidget * parent = nullptr);

  // Prende o editor a UMA arena e esconde o seletor. E' o que a ferramenta de
  // mapeamento precisa: la' a arena ja' foi decidida no passo 1, e oferecer um
  // seletor de mapa no meio do fluxo e' convite a editar o mapa errado.
  void setArena(const QString & arena);

  void carregar();
  bool temAlteracoesNaoSalvas() const;
  bool salvar();

signals:
  void status(const QString & frase);

private:
  QString mapDir() const;
  void loadOrigin();
  void applyOrigin();
  bool escreverOrigem(const QString & yaml, double x, double y, QString * erro);

  RosBridge * bridge_;
  MapCanvas * canvas_ = nullptr;
  QScrollArea * scroll_ = nullptr;
  QWidget * linha_arena_ = nullptr;
  SeletorArena * map_name_ = nullptr;
  QString arena_fixa_;
  QComboBox * ferramenta_ = nullptr;
  QComboBox * forma_ = nullptr;
  QSlider * tamanho_ = nullptr;
  QDoubleSpinBox * origem_x_ = nullptr;
  QDoubleSpinBox * origem_y_ = nullptr;
  QLabel * status_ = nullptr;
};
