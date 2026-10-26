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
//
// Alem de desenhar, o widget agora ACEITA edicao (Modo::PosicionarPose):
// escolher uma pose com o gesto do "2D Goal" e pegar um ponto ja' salvo para
// mover/girar. Antes disso, corrigir um dock torto exigia levar o robo fisico
// ate' o lugar certo e regravar a pose -- na arena, durante a prova. Quem
// consome esses gestos sao as telas (Mapeamento, Service Areas, Waypoints); o
// widget so' avisa por sinal, nunca grava arquivo.

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

// --- Leitura dos arquivos de uma arena, sem widget e sem ROS ---
//
// Os seletores de dock, de service area e de tipo de dock precisam EXATAMENTE
// da mesma lista que o preview desenha. Ter um segundo parser era o caminho
// curto para a tela oferecer um dock que o executor da missao nao encontra --
// e o operador so' descobrir isso com a missao abortando. Por isso o parser
// mora aqui, num lugar so'.
//
// `erro`, quando fornecido, recebe uma frase pronta para o operador (vazia
// quando deu tudo certo).
QMap<QString, PoseMapa> lerDocksDaArena(
  const QString & pastaDoMapa, QString * erro = nullptr);
QMap<QString, AreaDeServico> lerAreasDaArena(
  const QString & pastaDoMapa, QString * erro = nullptr);
QMap<QString, PoseMapa> lerWaypointsDaArena(
  const QString & pastaDoMapa, QString * erro = nullptr);

// Chaves de dock_plugins: do docking.yaml da arena (o "tipo" que cada dock usa
// para se aproximar). Quando a arena nao tem o arquivo, devolve os plugins
// padrao do robo, para a tela nunca ficar sem opcao.
QStringList lerTiposDeDockDaArena(const QString & pastaDoMapa);

// Um marcador ja' desenhado no mapa, identificado pelo hit-test.
struct MarcadorNoMapa
{
  bool valido = false;
  QString tipo;        // "dock" | "area" | "waypoint"
  QString id;
  PoseMapa pose;
};

class MapPreview : public QWidget
{
  Q_OBJECT

public:
  // Navegar    = comportamento historico: arrastar com o botao esquerdo faz pan.
  // PosicionarPose = edicao: o botao esquerdo escolhe pose (gesto do "2D Goal")
  //                  ou pega um ponto ja' salvo. O pan continua disponivel no
  //                  botao do meio, senao o operador ficaria preso no zoom.
  enum class Modo
  {
    Navegar,
    PosicionarPose
  };

  explicit MapPreview(QWidget * parent = nullptr);

  // Carrega a pasta de uma arena (caramelo_mapping/maps/<nome>).
  void carregar(const QString & pastaDoMapa);
  const MapaCarregado & mapa() const {return mapa_;}

  static QStringList camadasDisponiveis();

  // Nome da camada que so' existe no modo de edicao. Telas de consulta filtram
  // este item da lista de checkboxes: a string estava sendo redigitada fora
  // daqui, que e' exatamente como duas listas comecam a divergir.
  static QString camadaDeEdicao();

  void setCamada(const QString & camada, bool ligada);
  bool camadaLigada(const QString & camada) const;

  void enquadrar();   // volta ao zoom que mostra a arena inteira

  void setModo(Modo modo);
  Modo modo() const {return modo_;}

  // Pixels (widget) -> metros (map). Inversa de paraTela.
  QPointF paraMapa(const QPointF & tela) const;

  // Marca um ponto como "sendo editado": ele ganha um halo por cima de tudo,
  // para o operador nao perder de vista qual dos vinte marcadores esta mexendo.
  // Id vazio limpa o destaque.
  void destacar(const QString & tipo, const QString & id);
  void limparEdicao();   // apaga halo e a pose provisoria desenhada

  // Qual marcador esta sob este ponto da tela (invalido se nenhum).
  MarcadorNoMapa marcadorEm(const QPointF & tela) const;

signals:
  void carregado(const MapaCarregado & mapa);

  // Gesto do "2D Goal" concluido em area livre do mapa.
  void poseEscolhida(double x, double y, double yaw);

  // O operador tocou num ponto ja' salvo (sem arrastar).
  void marcadorClicado(const QString & tipo, const QString & id);

  // O operador moveu (arrastando o circulo) ou girou (arrastando a ponta da
  // seta) um ponto ja' salvo. A tela decide se grava.
  void marcadorArrastado(
    const QString & tipo, const QString & id, double x, double y, double yaw);

protected:
  void paintEvent(QPaintEvent * e) override;
  void resizeEvent(QResizeEvent * e) override;
  void wheelEvent(QWheelEvent * e) override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent * e) override;
  void mouseReleaseEvent(QMouseEvent * e) override;

private:
  // Fase do gesto em andamento no modo PosicionarPose.
  enum class Gesto
  {
    Nenhum,
    NovaPose,        // clicou no vazio: posicao no press, yaw no arrasto
    MoverMarcador,   // pegou o circulo de um ponto existente
    GirarMarcador    // pegou a ponta da seta de um ponto existente
  };

  QPointF paraTela(double x, double y) const;   // metros (map) -> pixels (widget)
  void desenharGrade(QPainter & p) const;
  void desenharKeepout(QPainter & p) const;
  void desenharDocks(QPainter & p) const;
  void desenharAreas(QPainter & p) const;
  void desenharWaypoints(QPainter & p) const;
  void desenharMarcador(
    QPainter & p, const PoseMapa & pose, const QColor & cor,
    const QString & rotulo, bool comSeta) const;
  void desenharEdicao(QPainter & p) const;
  void desenharSeta(
    QPainter & p, const QPointF & centro, double yaw, double comprimento,
    const QColor & cor) const;

  MarcadorNoMapa buscarMarcador(const QPointF & tela, bool * naSeta) const;
  bool poseDoMarcador(const QString & tipo, const QString & id, PoseMapa * pose) const;
  void atualizarCursor(const QPointF & tela);
  // Angulo do arrasto ja' com o Y da tela invertido (a tela cresce para baixo,
  // o mapa cresce para cima) -- errar isso grava todo yaw espelhado.
  static double yawDoArrasto(const QPointF & origem, const QPointF & destino);

  MapaCarregado mapa_;
  QHash<QString, bool> camadas_;

  double escala_ = 1.0;          // pixels de tela por pixel de imagem
  QPointF deslocamento_;         // canto superior esquerdo da imagem, em tela
  bool arrastando_ = false;
  QPointF ultimo_arrasto_;

  Modo modo_ = Modo::Navegar;
  Gesto gesto_ = Gesto::Nenhum;
  QPointF gesto_inicio_;         // pixel do press
  bool gesto_moveu_ = false;     // passou do limiar de tremor da mao
  PoseMapa gesto_pose_;          // pose provisoria, em metros
  bool tem_pose_provisoria_ = false;
  QString alvo_tipo_;            // marcador sob edicao no gesto atual
  QString alvo_id_;

  QString destaque_tipo_;
  QString destaque_id_;
};
