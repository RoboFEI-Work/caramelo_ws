#pragma once

// Chama os scripts Python que MANDAM nos arquivos de uma arena.
//
// Por que nao gravar o YAML em C++: docking.yaml e service_areas.yaml tem
// regras que nao cabem num "escreve tres numeros" -- backup antes de tocar,
// normalizacao do id pelo rulebook, tipo de dock deduzido do prefixo, e o
// sync do docking.yaml a partir das service areas. Essas regras ja' existem,
// testadas, em caramelo_navigation/scripts. Uma segunda implementacao aqui
// seria uma segunda verdade, e quem descobriria a divergencia seria o operador
// no meio da prova.
//
// Entao a GUI faz o que um humano faria no terminal: chama o script, com o
// mapa EXPLICITO, e transforma o codigo de saida numa frase de tela. Os
// scripts ja' escrevem os erros em portugues no stderr justamente para isso.
//
// Uma tarefa de cada vez, em fila: dois saves simultaneos mexeriam no mesmo
// service_areas.yaml (e no docking.yaml derivado dele) e o segundo backup
// guardaria o arquivo ja' meio escrito pelo primeiro.

#include <QObject>
#include <QQueue>
#include <QString>
#include <QStringList>

class QProcess;

// Pose escolhida no mapa (metros e radianos, frame do mapa). Quando
// `informada` e' falso, o script pergunta ao robo onde ele esta -- que e' a
// outra forma de marcar um ponto.
struct PoseEscolhida
{
  bool informada = false;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

class ScriptsDoMapa : public QObject
{
  Q_OBJECT

public:
  explicit ScriptsDoMapa(QObject * parent = nullptr);

  // Cria service_areas.yaml + docking.yaml (so' START e FINISH) e a mascara de
  // paredes virtuais. Chamado assim que o mapa e' salvo: sem isso a arena nasce
  // sem nenhum desses arquivos e todo passo seguinte vira uma lista de avisos
  // que a tela nao oferece como resolver.
  void prepararArena(const QString & arena);

  // So' a mascara de keepout (o passo de paredes virtuais chama quando o
  // arquivo nao existe).
  void criarParedesVirtuais(const QString & arena);

  // Rotulo dessa tarefa. Quem escuta `terminou` precisa saber se a tarefa que
  // acabou e' a dele; comparar com um texto redigitado na outra ponta e' como
  // duas listas comecam a divergir.
  static QString rotuloParedesVirtuais();

  // Estacao (bancada, prateleira, mesa de precisao, giratoria, START, FINISH):
  // vai para service_areas.yaml, que e' a fonte da verdade, e o proprio script
  // sincroniza o docking.yaml a partir dela.
  void gravarEstacao(
    const QString & arena, const QString & id, const QString & tipo,
    double alturaDaMesa, const PoseEscolhida & pose);

  // Dock que NAO e' service area. Vai direto para docking.yaml.
  void gravarDock(
    const QString & arena, const QString & id, const QString & tipoDeDock,
    const PoseEscolhida & pose);

  bool ocupado() const {return rodando_;}

signals:
  void comecou(const QString & tarefa);
  void terminou(const QString & tarefa, bool ok, const QString & mensagem);

private:
  struct Tarefa
  {
    QString rotulo;      // o que aparece na tela enquanto roda
    QString sucesso;     // frase de sucesso, ja' pronta
    QString programa;    // nome do executavel dentro do pacote
    QStringList args;
  };

  void enfileirar(const Tarefa & tarefa);
  void proxima();
  void encerrar(bool ok, const QString & mensagem);

  QProcess * processo_ = nullptr;
  QQueue<Tarefa> fila_;
  Tarefa atual_;
  QString saida_;
  bool rodando_ = false;
  bool ja_reportou_ = false;   // errorOccurred e finished podem vir os dois
};
