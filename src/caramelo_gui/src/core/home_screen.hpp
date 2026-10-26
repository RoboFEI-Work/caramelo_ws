#pragma once

// Menu principal: a primeira coisa que alguem ve ao ligar o robo.
//
// Layout de robo de servico (referencia: PUDU Kettybot Pro): coluna do robo a
// esquerda, grade de cartoes grandes com icone a direita, barra de acoes
// rapidas embaixo. Substitui a grade de 8 cartoes de diagnostico que estava
// aqui -- aquilo respondia "como esta cada componente ROS", que e' pergunta de
// desenvolvedor. Quem opera pergunta outra coisa: "o que eu posso fazer agora, e
// o que esta me impedindo".
//
// Por isso cada cartao e' uma ACAO, com estado e MOTIVO em portugues -- nunca
// nome de topico, action ou no. O detalhe tecnico nao sumiu: mudou de lugar,
// para o Modo Avancado.

#include <QMap>
#include <QVector>
#include <QWidget>

#include "bridge/health_types.hpp"

class QFrame;
class QGridLayout;
class QLabel;
class QPushButton;

class HomeScreen : public QWidget
{
  Q_OBJECT

public:
  explicit HomeScreen(QWidget * parent = nullptr);

  // Indice do contexto que cada cartao abre. Combina com a ordem em que a
  // MainWindow empilha as telas.
  enum Contexto
  {
    Operacao = 1,
    Competicao = 2,
    Mapas = 3,
    Mapeamento = 4,
    Avancado = 5,
  };

public slots:
  void onDiagnostics(const QVector<ComponentHealth> & health);
  void setMapaAtivo(const QString & nome);
  void setServidorMissao(bool disponivel);

signals:
  void abrirContexto(int contexto);
  void pararRobo();

private:
  enum class Estado {Pronto, Degradado, Bloqueado, Desconhecido};

  struct Cartao
  {
    QFrame * frame = nullptr;
    QLabel * chip = nullptr;
    QLabel * motivo = nullptr;
  };

  QWidget * construirColunaDoRobo();
  QWidget * construirGrade();
  QWidget * construirBarraInferior();
  QFrame * criarCartao(
    const QString & chave, const QString & titulo, const QString & subtitulo,
    int tipoIcone, int contexto);
  void aplicar(const QString & chave, Estado estado, const QString & motivo);
  void reavaliar();

  QMap<QString, Cartao> cartoes_;
  QMap<QString, ComponentHealth> saude_;
  QGridLayout * grade_ = nullptr;
  QLabel * arena_ = nullptr;
  QLabel * estado_robo_ = nullptr;
  QPushButton * base_btn_ = nullptr;
  QLabel * base_motivo_ = nullptr;
  QString mapa_ativo_;
  bool servidor_missao_ = false;
};
