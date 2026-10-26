#pragma once

// Editor de PAREDES VIRTUAIS (keepout_mask.pgm).
//
// Por que existe: o passo "paredes virtuais" da ferramenta de mapeamento abria
// o editor do map.pgm. Isso e' o contrario do que o rotulo promete: pintar de
// preto no map.pgm inventa uma parede FISICA no mapa, o AMCL passa a procurar
// essa parede com o LiDAR e a localizacao piora. Parede virtual mora noutro
// arquivo, o keepout_mask.pgm, que so' o planejador le'.
//
// Diferencas de proposito para o editor do mapa:
//   - a mascara so' tem dois valores: PRETO (0) = o robo nao entra, BRANCO
//     (255) = liberado. O cinza 205 ("desconhecido") do map.pgm nao existe
//     aqui -- numa mascara ele nao quer dizer nada;
//   - o map.pgm aparece POR BAIXO, como referencia, e a mascara por cima em
//     vermelho: pintar sem ver as paredes reais e' adivinhacao;
//   - quando a arena nao tem mascara, a tela oferece criar (init_keepout_mask),
//     em vez de so' avisar que falta.

#include <QString>
#include <QWidget>

class MapCanvas;
class ScriptsDoMapa;
class QLabel;
class QPushButton;
class QScrollArea;
class QSlider;
class QWidget;

class EditorKeepout : public QWidget
{
  Q_OBJECT

public:
  explicit EditorKeepout(ScriptsDoMapa * scripts, QWidget * parent = nullptr);

  void setArena(const QString & arena);
  void recarregar();

  bool temAlteracoesNaoSalvas() const;
  bool salvar();

signals:
  void status(const QString & frase);

private:
  QString pastaDoMapa() const;
  void mostrar(const QString & frase);
  void atualizarDisponibilidade(bool temMascara);

  ScriptsDoMapa * scripts_ = nullptr;
  QString arena_;

  MapCanvas * canvas_ = nullptr;
  QScrollArea * scroll_ = nullptr;
  QWidget * barra_ = nullptr;
  QWidget * caixa_criar_ = nullptr;
  QPushButton * botao_criar_ = nullptr;
  QPushButton * botao_salvar_ = nullptr;
  QSlider * tamanho_ = nullptr;
  QLabel * status_ = nullptr;
};
