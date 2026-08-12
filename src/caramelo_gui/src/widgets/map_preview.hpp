#pragma once

// Preview de uma arena: mostra o mapa COMO ELE FOI SALVO.
//
// Por que existe: ate' aqui escolher uma arena era ver uma miniatura do
// map.pgm. Isso mostra as paredes e esconde tudo que decide se a missao vai
// funcionar -- se os docks tem pose real ou ainda estao zerados, se as service
// areas apontam para onde deveriam, se existe mascara de keepout, onde fica o
// ponto de inicio. Esses erros so' apareciam com o robo andando, na arena.
//
// Aqui eles aparecem antes: cada camada e' desenhada sobre o mapa e as poses
// placeholder [0,0,0] sao pintadas em vermelho e listadas como aviso.
//
// O widget e' independente de ROS de proposito: le' os arquivos da pasta do
// mapa. Isso permite conferir uma arena com o robo desligado, e permite reusar
// o mesmo desenho na ferramenta de mapeamento.

#include <QHash>
#include <QImage>
#include <QMap>
#include <QPointF>
#include <QString>
#include <QStringList>
#include <QWidget>

struct PoseMapa
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  // Pose ainda nao gravada: o docking.yaml nasce com [0,0,0] e o
  // bt_yaml_executor recusa a missao quando encontra uma assim.
  bool placeholder = false;
};

struct AreaDeServico
{
  PoseMapa pose;
  QString tipo;                  // workstation | shelf | start | finish | ...
  bool manipulacao = true;
};

struct MapaCarregado
{
  bool valido = false;
  QString erro;
  QString nome;
  QString pasta;

  QImage ocupacao;
  QImage keepout;                // vazia quando a arena nao tem mascara
  double resolucao = 0.05;
  double origem_x = 0.0;
  double origem_y = 0.0;

  QMap<QString, PoseMapa> waypoints;
  QMap<QString, PoseMapa> docks;
  QMap<QString, AreaDeServico> areas;

  QStringList avisos;            // problemas prontos para exibir ao operador
};

class MapPreview : public QWidget
{
  Q_OBJECT

public:
  explicit MapPreview(QWidget * parent = nullptr);

  // Carrega a pasta de uma arena (caramelo_mapping/maps/<nome>).
  void carregar(const QString & pastaDoMapa);
  const MapaCarregado & mapa() const {return mapa_;}

  static QStringList camadasDisponiveis();
  void setCamada(const QString & camada, bool ligada);
  bool camadaLigada(const QString & camada) const;

  void enquadrar();   // volta ao zoom que mostra a arena inteira

signals:
  void carregado(const MapaCarregado & mapa);

protected:
  void paintEvent(QPaintEvent * e) override;
  void resizeEvent(QResizeEvent * e) override;
  void wheelEvent(QWheelEvent * e) override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent * e) override;
  void mouseReleaseEvent(QMouseEvent * e) override;

private:
  QPointF paraTela(double x, double y) const;   // metros (map) -> pixels (widget)
  void desenharGrade(QPainter & p) const;
  void desenharKeepout(QPainter & p) const;
  void desenharDocks(QPainter & p) const;
  void desenharAreas(QPainter & p) const;
  void desenharWaypoints(QPainter & p) const;
  void desenharMarcador(
    QPainter & p, const PoseMapa & pose, const QColor & cor,
    const QString & rotulo, bool comSeta) const;

  MapaCarregado mapa_;
  QHash<QString, bool> camadas_;

  double escala_ = 1.0;          // pixels de tela por pixel de imagem
  QPointF deslocamento_;         // canto superior esquerdo da imagem, em tela
  bool arrastando_ = false;
  QPointF ultimo_arrasto_;
};
