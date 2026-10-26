#pragma once

// Tela Mapas: escolher a arena em que o robo esta, VER o que esta salvo nela e
// CORRIGIR o que estiver errado.
//
// Antes era uma lista com miniatura do map.pgm. A miniatura mostra as paredes e
// esconde tudo que decide se a missao roda: docks sem pose gravada, service
// areas zeradas, ausencia de paredes virtuais, onde fica o ponto de inicio.
// Agora a arena selecionada aparece no MapPreview com essas camadas por cima, e
// os problemas viram avisos em portugues.
//
// A parte nova e' a EDICAO de um mapa ja' salvo. Ate' aqui, consertar um mapa
// significava abrir um terminal: apagar um dock a mao no docking.yaml, ou levar
// o robo fisicamente ate' o lugar certo para regravar uma pose torta -- na
// arena, no dia da prova. Aqui da' para remover um ponto, criar um ponto
// clicando no mapa e arrastar um ponto existente para o lugar certo, sem tirar
// o robo do lugar.
//
// Nada e' apagado sem pergunta e sem copia de seguranca: todo arquivo alterado
// e' copiado antes para <pasta da arena>/.backup/.
//
// "Usar esta arena" carrega o mapa no robo E grava a escolha em
// ~/.config/caramelo/robot_state.yaml, para ele religar no mesmo lugar.

#include <QString>
#include <QStringList>
#include <QWidget>

#include "widgets/map_preview.hpp"

class RosBridge;
class EditorKeepout;
class EditorMapaModule;
class ScriptsDoMapa;
class SeletorTipoArea;
class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QStackedWidget;

class MapasModule : public QWidget
{
  Q_OBJECT

public:
  explicit MapasModule(RosBridge * bridge, QWidget * parent = nullptr);

  // Pasta dos mapas (arvore fonte) — compartilhada com Mapeamento.
  static QString mapsDir();

private:
  QWidget * construirListaEAcoes();
  QWidget * construirCentro();
  QWidget * construirPainelDeAcoes();

  void refresh();
  QString selectedMap() const;
  QString pastaDaArena() const;    // vazio quando nenhuma arena esta selecionada
  void mostrarResumo(const MapaCarregado & mapa);
  void listarConteudo(const MapaCarregado & mapa);
  void recarregarPreview();

  // Desliga o que nao da' para fazer e escreve o motivo na tela. Botao cinza
  // sem frase le' como interface quebrada.
  void atualizarDisponibilidade();

  // O centro tem tres paginas: ver a arena, pintar o mapa e pintar as paredes
  // virtuais. Uma funcao so' porque sair de qualquer uma das duas edicoes tem a
  // mesma pergunta a fazer (ha' coisa nao salva?).
  void mostrarPagina(int pagina);

  // --- edicao de pontos ---
  void comecarPosicionamento(bool ligar);

  // Modo "pegar no mapa": liberar o arrasto dos pontos JA' salvos. Nao pode ser
  // o comportamento padrao da tela -- com ele ligado o botao esquerdo deixa de
  // mover a vista, e quem so' queria olhar a arena sairia arrastando os pontos
  // sem querer.
  void corrigirNoMapa(bool ligar);
  void aoEscolherPose(double x, double y, double yaw);
  void aoArrastarMarcador(
    const QString & tipo, const QString & id, double x, double y, double yaw);
  void removerPontoSelecionado();

  // Manda gravar a pose de um ponto pelos scripts do caramelo_navigation (eles
  // aceitam --pose e por isso nao precisam do robo no lugar).
  void gravarPose(
    const QString & tipo, const QString & id, const QString & tipoDeArea,
    double x, double y, double yaw);

  // tipo tecnico ("dock" | "area" | "waypoint") do que esta selecionado na
  // lista de conteudo; vazio quando nada esta selecionado.
  QString tipoSelecionado() const;
  QString idSelecionado() const;

  RosBridge * bridge_;

  QListWidget * lista_ = nullptr;
  MapPreview * preview_ = nullptr;
  EditorMapaModule * editor_ = nullptr;
  EditorKeepout * paredes_ = nullptr;
  QStackedWidget * centro_ = nullptr;
  QLabel * dica_do_mapa_ = nullptr;

  QPushButton * botao_limpar_ruido_ = nullptr;
  QPushButton * botao_paredes_ = nullptr;
  QPushButton * botao_remover_ponto_ = nullptr;
  QPushButton * botao_corrigir_ = nullptr;
  QPushButton * botao_posicionar_ = nullptr;

  QListWidget * conteudo_ = nullptr;
  QComboBox * tipo_novo_ = nullptr;
  QLineEdit * nome_novo_ = nullptr;
  SeletorTipoArea * tipo_de_area_ = nullptr;
  QLabel * rotulo_tipo_de_area_ = nullptr;

  QLabel * status_ = nullptr;
  QLabel * resumo_ = nullptr;
  QLabel * avisos_ = nullptr;
  QLabel * motivo_ = nullptr;
  QLabel * arena_ativa_ = nullptr;

  // Quem grava dock e service area sao os scripts do caramelo_navigation, pela
  // MESMA fachada que a ferramenta de mapeamento usa. Reimplementar a escrita
  // do YAML aqui criaria uma segunda verdade sobre as regras do rulebook, e a
  // divergencia so' apareceria com o robo em prova.
  ScriptsDoMapa * scripts_ = nullptr;
};
