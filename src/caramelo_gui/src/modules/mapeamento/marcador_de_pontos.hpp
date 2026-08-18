#pragma once

// Marcar os pontos da arena: docking das bancadas, prateleiras, mesas de
// precisao, mesa giratoria, START e FINISH.
//
// As DUAS formas de marcar um ponto vivem aqui, lado a lado, porque as duas
// sao necessarias em momentos diferentes:
//
//   NO MAPA  — clicar onde o robo deve parar e arrastar para dizer em que
//              direcao ele fica de frente. Serve para montar a arena inteira
//              sentado, antes da prova, e para CORRIGIR um ponto torto sem
//              precisar levar o robo ate' la' de novo.
//   COM O ROBO — dirigir o robo ate' o lugar, encostar na mesa como ele vai
//              encostar na hora, e gravar onde ele esta. E' a forma exata: a
//              pose sai da mecanica do robo, nao do olho de quem clica.
//
// As duas terminam no MESMO script Python (ver scripts_do_mapa.hpp), com o
// mapa passado explicitamente. O caminho antigo -- publicar num topico e
// mostrar "pose salva" sem esperar resposta -- mentia duas vezes: dizia
// "salvo" mesmo sem ninguem escutando, e gravava na arena que o NO tinha sido
// lancado, nao na que o operador escolheu na tela.

#include <QString>
#include <QStringList>
#include <QWidget>

#include "modules/mapeamento/scripts_do_mapa.hpp"
#include "widgets/map_preview.hpp"

class RosBridge;
class LaunchRunner;
class SeletorTipoArea;
class SeletorTipoDock;
class QComboBox;
class QFrame;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QVBoxLayout;

class MarcadorDePontos : public QWidget
{
  Q_OBJECT

public:
  MarcadorDePontos(
    RosBridge * bridge, ScriptsDoMapa * scripts, LaunchRunner * teleop,
    QWidget * parent = nullptr);

  void setArena(const QString & arena);

  // Re-le' os arquivos da arena: a lista da tela passa a ser o que esta
  // GRAVADO, e nao o que foi digitado nesta sessao.
  void recarregar();

  // O que ainda falta para esta arena servir numa prova (frases prontas).
  QStringList pendencias() const;

  // A ferramenta de mapeamento pendura aqui os botoes de navegacao do passo.
  void adicionarNoPainel(QWidget * widget);

  // Reaplica o pedido de mapa ao vivo (o passo acabou de ficar visivel).
  void reaplicarContexto();

signals:
  // O modo "com o robo" precisa ver o robo se mexendo; o modo "no mapa" precisa
  // da largura toda para o proprio mapa.
  void querMapaAoVivo(bool querer);
  void status(const QString & frase);

private:
  enum class Forma
  {
    NoMapa,
    ComORobo
  };
  enum class Especie
  {
    Estacao,
    Dock,
    Waypoint
  };

  QWidget * construirMapa();
  QWidget * construirPainel();

  void aplicarForma(Forma forma);
  void aplicarEspecie();
  Especie especie() const;

  void gravar();
  void gravarEstacao(const QString & id, const PoseEscolhida & pose);
  void gravarDock(const QString & id, const PoseEscolhida & pose);
  void gravarWaypoint(const QString & id, const PoseEscolhida & pose);
  bool escreverWaypointNoArquivo(
    const QString & id, double x, double y, double yaw, QString * erro);

  void aoEscolherPose(double x, double y, double yaw);
  void aoArrastarMarcador(
    const QString & tipo, const QString & id, double x, double y, double yaw);
  void selecionarMarcador(const QString & tipo, const QString & id);

  QString pastaDaArena() const;
  void mostrar(const QString & frase);
  void atualizarBotaoGravar();

  RosBridge * bridge_ = nullptr;
  ScriptsDoMapa * scripts_ = nullptr;
  LaunchRunner * teleop_ = nullptr;
  QString arena_;

  Forma forma_ = Forma::NoMapa;
  bool tem_pose_ = false;
  PoseEscolhida pose_;

  QFrame * moldura_ = nullptr;
  MapPreview * preview_ = nullptr;
  QLabel * dica_ = nullptr;

  QVBoxLayout * painel_ = nullptr;
  QPushButton * seg_mapa_ = nullptr;
  QPushButton * seg_robo_ = nullptr;
  QLabel * explica_forma_ = nullptr;
  QComboBox * especie_ = nullptr;
  SeletorTipoArea * tipo_area_ = nullptr;
  SeletorTipoDock * tipo_dock_ = nullptr;
  QWidget * linha_tipo_area_ = nullptr;
  QWidget * linha_tipo_dock_ = nullptr;
  QWidget * linha_altura_ = nullptr;
  QComboBox * altura_ = nullptr;
  QLineEdit * nome_ = nullptr;
  QLabel * ajuda_nome_ = nullptr;
  QPushButton * botao_teleop_ = nullptr;
  QPushButton * gravar_ = nullptr;
  QLabel * status_ = nullptr;
  QListWidget * lista_ = nullptr;
  QPushButton * remover_ = nullptr;
};
