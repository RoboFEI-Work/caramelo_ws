#include "modules/mapas/mapas_module.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPixmap>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QStackedWidget>
#include <QTextStream>
#include <QVBoxLayout>

#include <yaml-cpp/yaml.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "bridge/robot_state.hpp"
#include "bridge/ros_bridge.hpp"
#include "bridge/waypoint_manager.hpp"
#include "modules/editor_mapa/editor_keepout.hpp"
#include "modules/editor_mapa/editor_mapa_module.hpp"
#include "modules/mapeamento/scripts_do_mapa.hpp"
#include "widgets/seletor_tipo_area.hpp"
#include "widgets/validador_id_ponto.hpp"

namespace
{

// Tipos tecnicos que atravessam os sinais do MapPreview e o itemData da lista.
const char * kTipoDock = "dock";
const char * kTipoArea = "area";
const char * kTipoWaypoint = "waypoint";

// Paginas do centro da tela.
const int kPaginaMapa = 0;
const int kPaginaPintura = 1;
const int kPaginaParedes = 2;

// Altura de mesa que o proprio script usa quando ninguem diz outra. So' vale
// para area NOVA: mexer numa area que ja' existe reaproveita a altura gravada.
const double kAlturaPadraoDaMesa = 0.10;

// Copia recursiva simples de diretorio (para Duplicar).
bool copyDir(const QString & src, const QString & dst)
{
  QDir source(src);
  if (!source.exists()) {
    return false;
  }
  QDir().mkpath(dst);
  for (const auto & info :
    source.entryInfoList(QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot))
  {
    const QString target = dst + "/" + info.fileName();
    if (info.isDir()) {
      if (!copyDir(info.absoluteFilePath(), target)) {
        return false;
      }
    } else if (!QFile::copy(info.absoluteFilePath(), target)) {
      return false;
    }
  }
  return true;
}

// Nome de arena que nao quebra caminho de launch. Sintoma que isto evita: uma
// arena renomeada para "arena 520" era aceita, e depois o
// "map_name:=arena 520" chegava cortado no launch -- o Nav2 subia procurando um
// mapa chamado "arena", nao achava, e a falha aparecia longe daqui.
bool nomeDeArenaValido(const QString & nome)
{
  static const QRegularExpression padrao("^[a-z0-9_]+$");
  return padrao.match(nome).hasMatch();
}

// Copia o arquivo para <pasta da arena>/.backup/<nome>_<data>.<ext> antes de
// qualquer alteracao. Backup com data no nome, e nao ".bak": um ".bak" unico e'
// sobrescrito na segunda edicao, e a copia boa some justamente quando alguem
// errou duas vezes seguidas.
bool fazerBackup(const QString & arquivo, QString * erro)
{
  if (!QFile::exists(arquivo)) {
    return true;
  }
  const QFileInfo info(arquivo);
  const QString pasta = info.absolutePath() + "/.backup";
  if (!QDir().mkpath(pasta)) {
    if (erro) {
      *erro = "Nao consegui criar a pasta de copias de seguranca em " + pasta + ".";
    }
    return false;
  }
  const QString destino = pasta + "/" + info.completeBaseName() + "_" +
    QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + "." + info.suffix();
  if (!QFile::copy(arquivo, destino)) {
    if (erro) {
      *erro = "Nao consegui gravar a copia de seguranca em " + destino + ".";
    }
    return false;
  }
  return true;
}

int indentacaoDe(const QString & linha)
{
  int i = 0;
  while (i < linha.size() && linha.at(i) == QLatin1Char(' ')) {
    ++i;
  }
  return i;
}

// Apaga UMA chave (e o bloco dela) de dentro de uma secao de primeiro nivel do
// YAML, mexendo no texto e nao no documento.
//
// Por que no texto: reemitir o YAML com o yaml-cpp reescreve o arquivo inteiro
// e leva junto comentarios, ordem e formatacao que os scripts em Python e as
// pessoas usam para se orientar. Apagar so' as linhas do bloco deixa todo o
// resto do arquivo byte a byte como estava.
bool removerChaveDoYaml(
  const QString & arquivo, const QString & secao, const QString & chave,
  bool * removeu, QString * erro)
{
  if (removeu) {
    *removeu = false;
  }
  QFile entrada(arquivo);
  if (!entrada.open(QIODevice::ReadOnly | QIODevice::Text)) {
    if (erro) {
      *erro = "Nao consegui abrir " + QFileInfo(arquivo).fileName() + " para leitura.";
    }
    return false;
  }
  const QStringList linhas = QString::fromUtf8(entrada.readAll()).split(QLatin1Char('\n'));
  entrada.close();

  QStringList saida;
  bool naSecao = false;
  bool apagando = false;
  int indentDaChave = 0;

  for (const QString & linha : linhas) {
    const QString conteudo = linha.trimmed();
    const int indent = indentacaoDe(linha);

    if (apagando) {
      // O bloco da chave sao as linhas mais indentadas que ela. A primeira
      // linha que volta ao nivel da chave (ou acima) ja' e' outro ponto.
      if (conteudo.isEmpty() || indent > indentDaChave) {
        continue;
      }
      apagando = false;
    }

    if (indent == 0 && !conteudo.isEmpty() && !conteudo.startsWith(QLatin1Char('#'))) {
      naSecao = conteudo == (secao + ":");
    } else if (naSecao && indent > 0 &&
      (conteudo == (chave + ":") || conteudo.startsWith(chave + ": ")))
    {
      apagando = true;
      indentDaChave = indent;
      if (removeu) {
        *removeu = true;
      }
      continue;
    }
    saida << linha;
  }

  // Tirar o ultimo filho deixaria "docks:" sozinho, sem valor -- que em YAML e'
  // NULO, e nao "mapa vazio". O yaml.safe_load do lado Python devolve None ali,
  // e quem faz "for id in doc['docks']" quebra com erro de tipo num arquivo que
  // parece certo a olho nu. Um "{}" explicito mantem o arquivo utilizavel.
  for (int i = 0; i < saida.size(); ++i) {
    if (indentacaoDe(saida.at(i)) != 0 || saida.at(i).trimmed() != (secao + ":")) {
      continue;
    }
    bool temFilho = false;
    for (int j = i + 1; j < saida.size(); ++j) {
      if (saida.at(j).trimmed().isEmpty()) {
        continue;
      }
      temFilho = indentacaoDe(saida.at(j)) > 0;
      break;
    }
    if (!temFilho) {
      saida[i] = secao + ": {}";
    }
    break;
  }

  QFile destino(arquivo);
  if (!destino.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
    if (erro) {
      *erro = "Nao consegui gravar " + QFileInfo(arquivo).fileName() + ".";
    }
    return false;
  }
  // Quando a chave apagada era o ultimo bloco do arquivo, a linha vazia final
  // vai junto com ela e o YAML termina sem quebra de linha. Nao chega a ser
  // invalido, mas atrapalha diff, "cat" e append -- e o arquivo e' lido por
  // gente tambem.
  QString texto = saida.join(QLatin1Char('\n'));
  if (!texto.endsWith(QLatin1Char('\n'))) {
    texto += QLatin1Char('\n');
  }
  destino.write(texto.toUtf8());
  destino.close();
  return true;
}

// Reescreve o waypoints.yaml inteiro a partir da lista dada. E' o unico dos
// tres arquivos que a GUI grava sozinha: dock e service area tem script proprio
// (que ainda sincroniza docking.yaml), waypoint nao tem.
bool gravarWaypoints(
  const QString & pastaDaArena, const QMap<QString, PoseMapa> & pontos, QString * erro)
{
  const QString caminho = pastaDaArena + "/waypoints.yaml";
  QFile arquivo(caminho);
  if (!arquivo.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
    if (erro) {
      *erro = "Nao consegui gravar o arquivo de waypoints em " + caminho + ".";
    }
    return false;
  }
  QTextStream saida(&arquivo);
  saida << "waypoints:\n";
  for (auto it = pontos.constBegin(); it != pontos.constEnd(); ++it) {
    saida << "  " << it.key() << ":\n";
    saida << "    x: " << QString::number(it.value().x, 'f', 6) << "\n";
    saida << "    y: " << QString::number(it.value().y, 'f', 6) << "\n";
    saida << "    yaw: " << QString::number(it.value().yaw, 'f', 6) << "\n";
  }
  arquivo.close();
  return true;
}

// Altura da mesa ja' gravada para esta area, em metros.
//
// Por que ler o arquivo: a tela sempre manda --height ao script, entao ela
// precisa mandar a altura CERTA. Sem isso, arrastar um ponto tres centimetros
// para o lado rebaixaria uma prateleira de 0.15 para a altura padrao -- e o
// braco passaria a procurar os objetos na altura errada, sem ninguem ter
// pedido isso. (O script tambem passou a preservar a altura quando --height
// nao vem; aqui e' cinto e suspensorio, e vale para a area nova tambem.)
double alturaDaMesaGravada(const QString & pastaDaArena, const QString & id)
{
  const QString caminho = pastaDaArena + "/service_areas.yaml";
  if (!QFileInfo::exists(caminho)) {
    return kAlturaPadraoDaMesa;
  }
  try {
    const YAML::Node raiz = YAML::LoadFile(caminho.toStdString());
    const YAML::Node area = raiz["service_areas"][id.toStdString()];
    if (area && area["table"] && area["table"]["height"]) {
      return area["table"]["height"].as<double>(kAlturaPadraoDaMesa);
    }
  } catch (const std::exception &) {
    // Arquivo ilegivel: o script cuida do erro de verdade; aqui so' escolhemos
    // a altura padrao em vez de derrubar a tela.
  }
  return kAlturaPadraoDaMesa;
}

QString rotuloDoTipo(const QString & tipo)
{
  if (tipo == kTipoDock) {
    return "Dock (estacao)";
  }
  if (tipo == kTipoArea) {
    return "Service area (mesa)";
  }
  return "Waypoint";
}

}  // namespace

QString MapasModule::mapsDir()
{
  // 1) Override explicito.
  const QString env = QProcessEnvironment::systemEnvironment().value("CARAMELO_MAPS_DIR");
  if (!env.isEmpty() && QDir(env).exists()) {
    return env;
  }
  // 2) Arvore FONTE do workspace (a verdade — igual aos scripts de docking):
  //    <ws>/install/caramelo_mapping/share/caramelo_mapping -> <ws>/src/...
  try {
    QString share = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("caramelo_mapping"));
    const int idx = share.indexOf("/install/");
    if (idx > 0) {
      const QString src = share.left(idx) + "/src/caramelo_mapping/maps";
      if (QDir(src).exists()) {
        return src;
      }
    }
    if (QDir(share + "/maps").exists()) {
      return share + "/maps";
    }
  } catch (...) {
  }
  return QDir::homePath() + "/caramelo_ws/src/caramelo_mapping/maps";
}

MapasModule::MapasModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  // Criado antes das colunas porque o centro (o editor de paredes virtuais) e o
  // painel de acoes dependem dele.
  scripts_ = new ScriptsDoMapa(this);

  // Tres colunas: arenas | o mapa | o que fazer com ele. E' a mesma leitura da
  // esquerda para a direita em toda a interface: escolher, ver, agir.
  auto * layout = new QHBoxLayout(this);
  layout->setContentsMargins(12, 12, 12, 12);
  layout->setSpacing(14);

  layout->addWidget(construirListaEAcoes(), 0);
  layout->addWidget(construirCentro(), 1);
  layout->addWidget(construirPainelDeAcoes(), 0);

  connect(
    lista_, &QListWidget::currentItemChanged, this,
    [this](QListWidgetItem * atual, QListWidgetItem *) {
      // Trocar de arena com um posicionamento em andamento gravaria o ponto na
      // arena errada -- exatamente o erro silencioso que esta tela existe para
      // acabar.
      if (botao_posicionar_->isChecked()) {
        botao_posicionar_->setChecked(false);
      }
      if (botao_corrigir_->isChecked()) {
        botao_corrigir_->setChecked(false);
      }
      if (atual) {
        preview_->carregar(mapsDir() + "/" + atual->text());
        // Com um editor aberto, trocar de arena tem que levar o editor junto --
        // senao o operador troca de arena na lista e continua pintando a
        // anterior.
        if (centro_->currentIndex() == kPaginaPintura) {
          editor_->setArena(atual->text());
          editor_->carregar();
        } else if (centro_->currentIndex() == kPaginaParedes) {
          paredes_->setArena(atual->text());
          paredes_->recarregar();
        }
      }
      atualizarDisponibilidade();
    });
  connect(
    editor_, &EditorMapaModule::status, this,
    [this](const QString & frase) {status_->setText(frase);});
  connect(preview_, &MapPreview::carregado, this, &MapasModule::mostrarResumo);
  connect(preview_, &MapPreview::carregado, this, &MapasModule::listarConteudo);

  connect(preview_, &MapPreview::poseEscolhida, this, &MapasModule::aoEscolherPose);
  connect(preview_, &MapPreview::marcadorArrastado, this, &MapasModule::aoArrastarMarcador);
  connect(
    preview_, &MapPreview::marcadorClicado, this,
    [this](const QString & tipo, const QString & id) {
      // Tocar num ponto do mapa seleciona a linha dele na lista: sao as duas
      // metades da mesma coisa, e ter que reencontrar o nome numa lista de
      // vinte itens depois de achar o ponto no mapa e' trabalho a toa.
      for (int i = 0; i < conteudo_->count(); ++i) {
        auto * item = conteudo_->item(i);
        if (item->data(Qt::UserRole).toString() == tipo &&
          item->data(Qt::UserRole + 1).toString() == id)
        {
          conteudo_->setCurrentItem(item);
          return;
        }
      }
    });

  connect(
    bridge_, &RosBridge::mapStatus, this,
    [this](bool ok, const QString & m) {
      status_->setText((ok ? "" : "Falha: ") + m);
    });
  connect(
    bridge_->robotState(), &RobotState::mapNameChanged, this,
    [this](const QString & nome) {
      arena_ativa_->setText("Arena ativa no robo: " + nome);
    });

  connect(
    scripts_, &ScriptsDoMapa::comecou, this,
    [this](const QString & tarefa) {
      status_->setText(tarefa + "...");
      atualizarDisponibilidade();
    });
  connect(
    scripts_, &ScriptsDoMapa::terminou, this,
    [this](const QString &, bool ok, const QString & mensagem) {
      status_->setText(ok ? mensagem : "Nao consegui gravar. " + mensagem);
      // Recarrega mesmo em caso de falha: se o script gravou pela metade, o
      // operador precisa ver o que ficou no arquivo, e nao o que a tela achava
      // que ia ficar.
      recarregarPreview();
      atualizarDisponibilidade();
    });

  refresh();
}

// ------------------------------------------------------------- coluna 1
QWidget * MapasModule::construirListaEAcoes()
{
  auto * col = new QWidget();
  auto * layout = new QVBoxLayout(col);
  layout->setContentsMargins(0, 0, 8, 0);

  auto * title = new QLabel("Arenas");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  arena_ativa_ = new QLabel("Arena ativa no robo: —");
  arena_ativa_->setObjectName("msgCartao");
  arena_ativa_->setWordWrap(true);
  layout->addWidget(arena_ativa_);

  lista_ = new QListWidget();
  lista_->setMinimumHeight(150);
  layout->addWidget(lista_, 1);

  auto addBtn = [this, layout](const QString & texto, auto slot) {
      auto * b = new QPushButton(texto);
      connect(b, &QPushButton::clicked, this, slot);
      layout->addWidget(b);
      return b;
    };

  addBtn(
    "Usar esta arena", [this]() {
      const QString nome = selectedMap();
      if (nome.isEmpty()) {return;}
      // Carrega agora E grava a escolha no robo. Sem gravar, o mapa voltaria ao
      // anterior no proximo boot e alguem teria que lembrar de passar o nome
      // certo num terminal — exatamente o que esta interface elimina.
      bridge_->loadMap(mapsDir() + "/" + nome + "/map.yaml");
      QString erro;
      if (!bridge_->robotState()->setMapName(nome, &erro)) {
        QMessageBox::warning(
          this, "Arena",
          "O mapa foi carregado agora, mas nao consegui gravar a escolha no robo:\n" +
          erro + "\n\nAo reiniciar, ele voltara a arena anterior.");
      }
    })->setObjectName("acaoPrimaria");

  addBtn(
    "Renomear", [this]() {
      const QString nome = selectedMap();
      if (nome.isEmpty()) {return;}
      bool ok = false;
      const QString novo = QInputDialog::getText(
        this, "Renomear arena",
        "Novo nome (so' letras minusculas, numeros e _ ):",
        QLineEdit::Normal, nome, &ok).trimmed();
      if (!ok || novo.isEmpty() || novo == nome) {return;}
      // Sintoma que estas duas checagens matam: "arena 520" era aceito e depois
      // chegava cortado no map_name:= do launch (o Nav2 subia procurando
      // "arena"); e renomear para um nome que ja' existe fazia o QDir::rename
      // devolver false com uma caixa dizendo so' "nao foi possivel".
      if (!nomeDeArenaValido(novo)) {
        QMessageBox::warning(
          this, "Renomear",
          "O nome \"" + novo + "\" nao serve. Use so' letras minusculas, numeros "
          "e o sinal _ , sem espacos nem acentos — o nome da arena vai inteiro "
          "para a linha que liga a navegacao, e um espaco corta essa linha ao "
          "meio.\n\nExemplos: arena3_520, sala_520, arena_regional.");
        return;
      }
      if (QDir(mapsDir() + "/" + novo).exists()) {
        QMessageBox::warning(
          this, "Renomear",
          "Ja' existe uma arena chamada \"" + novo + "\". Escolha outro nome.");
        return;
      }
      if (!QDir(mapsDir()).rename(nome, novo)) {
        QMessageBox::warning(
          this, "Renomear",
          "Nao consegui renomear a pasta da arena. Verifique se algum programa "
          "esta' usando os arquivos dela.");
        return;
      }
      // A escolha gravada no robo aponta para a PASTA. Renomear sem atualizar
      // deixaria o robo religando num mapa que nao existe mais.
      if (bridge_->robotState()->mapName() == nome) {
        bridge_->robotState()->setMapName(novo);
      }
      refresh();
    });

  addBtn(
    "Duplicar", [this]() {
      const QString nome = selectedMap();
      if (nome.isEmpty()) {return;}
      const QString copia = nome + "_copia";
      if (QDir(mapsDir() + "/" + copia).exists()) {
        QMessageBox::warning(
          this, "Duplicar",
          "Ja' existe uma arena chamada \"" + copia + "\". Renomeie ou apague a "
          "copia anterior antes de duplicar de novo.");
        return;
      }
      if (!copyDir(mapsDir() + "/" + nome, mapsDir() + "/" + copia)) {
        QMessageBox::warning(this, "Duplicar", "Nao foi possivel duplicar.");
      }
      refresh();
    });

  addBtn(
    "Excluir", [this]() {
      const QString nome = selectedMap();
      if (nome.isEmpty()) {return;}
      const auto resp = QMessageBox::question(
        this, "Excluir arena",
        QString("Mover a arena '%1' para a lixeira?\n\nTudo que esta' dentro dela "
        "vai junto: o mapa, os docks, as service areas e os waypoints.").arg(nome));
      if (resp != QMessageBox::Yes) {return;}
      // Nunca apaga direto: move para maps/.lixeira/<nome>_<data>.
      const QString lixeira = mapsDir() + "/.lixeira";
      QDir().mkpath(lixeira);
      const QString destino = lixeira + "/" + nome + "_" +
        QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
      if (!QDir(mapsDir()).rename(nome, destino)) {
        QMessageBox::warning(this, "Excluir", "Nao foi possivel mover para a lixeira.");
      }
      refresh();
    })->setObjectName("acaoDestrutiva");

  addBtn("Atualizar lista", [this]() {refresh();});

  status_ = new QLabel();
  status_->setObjectName("msgCartao");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  // Num touchscreen baixo, "Excluir" e "Atualizar lista" ficavam abaixo da
  // borda da tela e nao havia barra nenhuma dizendo que existia mais coisa.
  auto * scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setWidget(col);
  scroll->setFixedWidth(250);
  return scroll;
}

// ------------------------------------------------------------- coluna 2
QWidget * MapasModule::construirCentro()
{
  auto * col = new QWidget();
  auto * layout = new QVBoxLayout(col);
  layout->setContentsMargins(0, 0, 0, 0);

  // O centro alterna entre VER a arena, PINTAR o mapa e PINTAR as paredes
  // virtuais. Sao os tres motivos de alguem abrir esta tela, e trocar de tela
  // para editar obrigaria a reencontrar o mapa selecionado.
  //
  // Pintar o mapa e pintar parede virtual sao coisas diferentes de proposito:
  // preto no map.pgm inventa uma parede FISICA, que o LiDAR vai procurar e nao
  // achar (a localizacao piora); parede virtual vive na mascara, que so' o
  // planejador le'.
  centro_ = new QStackedWidget();
  preview_ = new MapPreview();
  editor_ = new EditorMapaModule(bridge_);
  paredes_ = new EditorKeepout(scripts_);
  centro_->addWidget(preview_);      // kPaginaMapa
  centro_->addWidget(editor_);       // kPaginaPintura
  centro_->addWidget(paredes_);      // kPaginaParedes
  layout->addWidget(centro_, 1);

  dica_do_mapa_ = new QLabel(
    "Arraste para mover a vista e use a roda do mouse para aproximar. Para "
    "mexer num ponto, ligue \"Corrigir a posicao\".");
  dica_do_mapa_->setObjectName("dicaGesto");
  dica_do_mapa_->setWordWrap(true);
  layout->addWidget(dica_do_mapa_);

  return col;
}

// ------------------------------------------------------------- coluna 3
QWidget * MapasModule::construirPainelDeAcoes()
{
  auto * painel = new QWidget();
  auto * layout = new QVBoxLayout(painel);
  layout->setContentsMargins(0, 0, 8, 0);

  // --- o que existe dentro da arena ---
  auto * grupoConteudo = new QGroupBox("O que esta salvo nesta arena");
  auto * conteudoLayout = new QVBoxLayout(grupoConteudo);
  conteudo_ = new QListWidget();
  conteudo_->setMinimumHeight(170);
  conteudoLayout->addWidget(conteudo_);

  botao_corrigir_ = new QPushButton("Corrigir a posicao (arrastar no mapa)");
  botao_corrigir_->setCheckable(true);
  connect(botao_corrigir_, &QPushButton::toggled, this, &MapasModule::corrigirNoMapa);
  conteudoLayout->addWidget(botao_corrigir_);

  botao_remover_ponto_ = new QPushButton("Remover o ponto selecionado");
  botao_remover_ponto_->setObjectName("acaoDestrutiva");
  connect(
    botao_remover_ponto_, &QPushButton::clicked, this,
    &MapasModule::removerPontoSelecionado);
  conteudoLayout->addWidget(botao_remover_ponto_);
  layout->addWidget(grupoConteudo);

  // O motivo fica logo abaixo dos botoes que ele explica. No pe' do painel ele
  // sairia da area visivel justamente quando o operador esta' olhando para o
  // botao cinza, perguntando por que nao da' para clicar.
  motivo_ = new QLabel();
  motivo_->setObjectName("motivoCartao");
  motivo_->setWordWrap(true);
  layout->addWidget(motivo_);

  connect(
    conteudo_, &QListWidget::currentItemChanged, this,
    [this](QListWidgetItem * atual, QListWidgetItem *) {
      if (atual) {
        preview_->destacar(
          atual->data(Qt::UserRole).toString(), atual->data(Qt::UserRole + 1).toString());
      } else {
        preview_->limparEdicao();
      }
      atualizarDisponibilidade();
    });

  // --- criar um ponto clicando no mapa ---
  auto * grupoNovo = new QGroupBox("Adicionar um ponto");
  auto * novoLayout = new QVBoxLayout(grupoNovo);

  auto * explicaNovo = new QLabel(
    "Marca um ponto SEM levar o robo ate' la': escolha o tipo, de o nome e "
    "clique no mapa onde ele fica.");
  explicaNovo->setObjectName("msgCartao");
  explicaNovo->setWordWrap(true);
  novoLayout->addWidget(explicaNovo);

  auto * formNovo = new QFormLayout();
  tipo_novo_ = new QComboBox();
  tipo_novo_->addItem("Dock (estacao)", kTipoDock);
  tipo_novo_->addItem("Service area (mesa)", kTipoArea);
  tipo_novo_->addItem("Waypoint (lugar qualquer)", kTipoWaypoint);
  formNovo->addRow("Tipo:", tipo_novo_);

  nome_novo_ = new QLineEdit();
  nome_novo_->setPlaceholderText("ex.: WS1");
  formNovo->addRow("Nome:", nome_novo_);

  tipo_de_area_ = new SeletorTipoArea();
  rotulo_tipo_de_area_ = new QLabel("Tipo da mesa:");
  formNovo->addRow(rotulo_tipo_de_area_, tipo_de_area_);
  novoLayout->addLayout(formNovo);

  botao_posicionar_ = new QPushButton("Escolher a posicao no mapa");
  botao_posicionar_->setObjectName("acaoPrimaria");
  botao_posicionar_->setCheckable(true);
  connect(
    botao_posicionar_, &QPushButton::toggled, this, &MapasModule::comecarPosicionamento);
  novoLayout->addWidget(botao_posicionar_);
  layout->addWidget(grupoNovo);

  connect(
    tipo_novo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int) {
      // O tipo da mesa so' existe para service area; mostrar sempre daria a
      // entender que um waypoint tambem tem tipo de mesa.
      const bool ehArea = tipo_novo_->currentData().toString() == kTipoArea;
      tipo_de_area_->setVisible(ehArea);
      rotulo_tipo_de_area_->setVisible(ehArea);
      // Trocar o tipo no meio de um posicionamento gravaria um dock com o nome
      // que era de uma mesa (ou o contrario), sem nada na tela dizendo isso.
      if (botao_posicionar_->isChecked()) {
        botao_posicionar_->setChecked(false);
      }
    });
  // O formulario ja' nasce com "Dock", entao a linha do tipo de mesa comeca
  // escondida -- deixar visivel sugeriria que um dock tambem tem tipo de mesa.
  const bool comecaComoArea = tipo_novo_->currentData().toString() == kTipoArea;
  tipo_de_area_->setVisible(comecaComoArea);
  rotulo_tipo_de_area_->setVisible(comecaComoArea);

  // --- consertar o desenho do mapa ---
  auto * grupoMapa = new QGroupBox("Corrigir o mapa");
  auto * mapaLayout = new QVBoxLayout(grupoMapa);

  botao_limpar_ruido_ = new QPushButton("Limpar ruido (pintar o mapa)");
  connect(
    botao_limpar_ruido_, &QPushButton::clicked, this, [this]() {
      mostrarPagina(
        centro_->currentIndex() == kPaginaPintura ? kPaginaMapa : kPaginaPintura);
    });
  mapaLayout->addWidget(botao_limpar_ruido_);

  auto * explicaLimpar = new QLabel(
    "Apaga do mapa a pessoa que passou na frente do LiDAR durante o mapeamento "
    "e fecha o buraco que ficou num canto mal varrido.");
  explicaLimpar->setObjectName("msgCartao");
  explicaLimpar->setWordWrap(true);
  mapaLayout->addWidget(explicaLimpar);

  botao_paredes_ = new QPushButton("Paredes virtuais (onde o robo nao entra)");
  connect(
    botao_paredes_, &QPushButton::clicked, this, [this]() {
      mostrarPagina(
        centro_->currentIndex() == kPaginaParedes ? kPaginaMapa : kPaginaParedes);
    });
  mapaLayout->addWidget(botao_paredes_);

  auto * explicaParedes = new QLabel(
    "Marca lugares proibidos que o LiDAR nao enxerga: fita no chao, degrau "
    "baixo, vidro. Nao mexe no mapa — o robo continua se localizando pelas "
    "paredes de verdade.");
  explicaParedes->setObjectName("msgCartao");
  explicaParedes->setWordWrap(true);
  mapaLayout->addWidget(explicaParedes);
  layout->addWidget(grupoMapa);

  connect(
    paredes_, &EditorKeepout::status, this,
    [this](const QString & frase) {status_->setText(frase);});

  // --- camadas do desenho ---
  auto * grupoCamadas = new QGroupBox("Camadas");
  auto * camadasLayout = new QVBoxLayout(grupoCamadas);
  for (const QString & camada : MapPreview::camadasDisponiveis()) {
    // "Ponto sendo editado" e' controlado pela propria tela (aparece quando ha'
    // um ponto selecionado). Oferecer o interruptor daria ao operador um
    // controle que so' atrapalha o que ele acabou de pedir.
    if (camada == MapPreview::camadaDeEdicao()) {
      continue;
    }
    auto * cb = new QCheckBox(camada);
    cb->setChecked(true);
    connect(
      cb, &QCheckBox::toggled, this,
      [this, camada](bool on) {preview_->setCamada(camada, on);});
    camadasLayout->addWidget(cb);
  }
  auto * enquadrar = new QPushButton("Enquadrar arena");
  connect(enquadrar, &QPushButton::clicked, this, [this]() {preview_->enquadrar();});
  camadasLayout->addWidget(enquadrar);
  layout->addWidget(grupoCamadas);

  // --- resumo e avisos ---
  auto * grupoResumo = new QGroupBox("Resumo");
  auto * resumoLayout = new QVBoxLayout(grupoResumo);
  resumo_ = new QLabel("Selecione uma arena.");
  resumo_->setWordWrap(true);
  resumoLayout->addWidget(resumo_);
  layout->addWidget(grupoResumo);

  auto * grupoAvisos = new QGroupBox("Atencao");
  auto * avisosLayout = new QVBoxLayout(grupoAvisos);
  avisos_ = new QLabel();
  avisos_->setObjectName("motivoCartao");
  avisos_->setWordWrap(true);
  avisosLayout->addWidget(avisos_);
  layout->addWidget(grupoAvisos);

  layout->addStretch();

  // O painel inteiro rola: sao seis grupos, e num touchscreen baixo os de baixo
  // (resumo e avisos) simplesmente sumiam da tela, sem barra nenhuma indicando
  // que havia mais coisa.
  auto * scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setWidget(painel);
  scroll->setFixedWidth(320);
  return scroll;
}

// ------------------------------------------------------------- conteudo
void MapasModule::listarConteudo(const MapaCarregado & mapa)
{
  conteudo_->clear();
  if (!mapa.valido) {
    atualizarDisponibilidade();
    return;
  }

  // Amarelo do tema para "falta alguma coisa aqui" (o mesmo do motivoCartao).
  const QColor corDeAviso("#ffd28f");

  auto acrescentar = [this, &corDeAviso](
    const QString & tipo, const QString & id, const QString & detalhe, bool semPose) {
      QString texto = rotuloDoTipo(tipo) + "   " + id;
      if (!detalhe.isEmpty()) {
        texto += "   —   " + detalhe;
      }
      if (semPose) {
        texto += "   (sem posicao gravada)";
      }
      auto * item = new QListWidgetItem(texto, conteudo_);
      item->setData(Qt::UserRole, tipo);
      item->setData(Qt::UserRole + 1, id);
      if (semPose) {
        item->setForeground(corDeAviso);
        item->setToolTip(
          "Este ponto existe no arquivo mas esta' na origem do mapa. Mandar o "
          "robo para ele o faria atravessar a arena ate' o canto errado.");
      }
    };

  for (auto it = mapa.areas.constBegin(); it != mapa.areas.constEnd(); ++it) {
    acrescentar(
      kTipoArea, it.key(), rotuloDeTipoDeArea(it.value().tipo), it.value().pose.placeholder);
  }
  for (auto it = mapa.docks.constBegin(); it != mapa.docks.constEnd(); ++it) {
    acrescentar(kTipoDock, it.key(), QString(), it.value().placeholder);
  }
  for (auto it = mapa.waypoints.constBegin(); it != mapa.waypoints.constEnd(); ++it) {
    acrescentar(kTipoWaypoint, it.key(), QString(), it.value().placeholder);
  }

  if (conteudo_->count() == 0) {
    auto * vazio = new QListWidgetItem(
      "Esta arena ainda nao tem nenhum ponto marcado.", conteudo_);
    vazio->setFlags(Qt::NoItemFlags);
  }
  atualizarDisponibilidade();
}

void MapasModule::mostrarResumo(const MapaCarregado & mapa)
{
  if (!mapa.valido) {
    resumo_->setText(mapa.erro.isEmpty() ? "Selecione uma arena." : mapa.erro);
    avisos_->setText(QString());
    return;
  }

  int docksComPose = 0;
  for (const auto & d : mapa.docks) {
    if (!d.placeholder) {
      ++docksComPose;
    }
  }
  const double largura = mapa.ocupacao.width() * mapa.resolucao;
  const double altura = mapa.ocupacao.height() * mapa.resolucao;

  resumo_->setText(
    QString(
      "Tamanho: %1 x %2 m  (%3 cm/pixel)\n"
      "Docks: %4 (%5 com posicao gravada)\n"
      "Service areas: %6\n"
      "Waypoints: %7\n"
      "Paredes virtuais: %8")
    .arg(largura, 0, 'f', 1).arg(altura, 0, 'f', 1)
    .arg(mapa.resolucao * 100.0, 0, 'f', 1)
    .arg(mapa.docks.size()).arg(docksComPose)
    .arg(mapa.areas.size())
    .arg(mapa.waypoints.size())
    .arg(mapa.keepout.isNull() ? "nao" : "sim"));

  avisos_->setText(
    mapa.avisos.isEmpty() ? "Nada a apontar nesta arena." : "• " + mapa.avisos.join("\n• "));
}

// ------------------------------------------------------------- edicao
void MapasModule::comecarPosicionamento(bool ligar)
{
  if (!ligar) {
    if (!botao_corrigir_->isChecked()) {
      preview_->setModo(MapPreview::Modo::Navegar);
      preview_->limparEdicao();
      dica_do_mapa_->setText(
        "Arraste para mover a vista e use a roda do mouse para aproximar. Para "
        "mexer num ponto, ligue \"Corrigir a posicao\".");
    }
    botao_posicionar_->setText("Escolher a posicao no mapa");
    return;
  }

  const QString pasta = pastaDaArena();
  if (pasta.isEmpty()) {
    status_->setText("Escolha primeiro em qual arena o ponto vai ficar.");
    botao_posicionar_->setChecked(false);
    return;
  }
  if (botao_corrigir_->isChecked()) {
    // Os dois modos usam o mesmo clique com significados diferentes; ficar nos
    // dois ao mesmo tempo nao existe.
    botao_corrigir_->setChecked(false);
  }

  // Validar o nome ANTES de pedir o clique: descobrir que o nome nao serve
  // depois de acertar a posicao no mapa obriga a refazer o gesto inteiro.
  const QString tipo = tipo_novo_->currentData().toString();
  const QString nome = nome_novo_->text().trimmed();
  if (tipo == kTipoWaypoint) {
    if (nome.isEmpty()) {
      status_->setText("De um nome ao waypoint antes de escolher a posicao.");
      botao_posicionar_->setChecked(false);
      return;
    }
  } else if (!idDePontoAceitavel(nome)) {
    status_->setText(erroDeIdDePonto(nome));
    botao_posicionar_->setChecked(false);
    return;
  }

  // Pintar e escolher pose sao modos do mesmo centro; o preview precisa estar
  // na frente para o clique chegar nele.
  if (centro_->currentIndex() != kPaginaMapa) {
    mostrarPagina(kPaginaMapa);
    if (centro_->currentIndex() != kPaginaMapa) {
      // O operador cancelou o "salvar antes de sair": nao vamos arrastar ele
      // para fora da edicao contra a vontade.
      botao_posicionar_->setChecked(false);
      return;
    }
  }

  preview_->setModo(MapPreview::Modo::PosicionarPose);
  botao_posicionar_->setText("Cancelar (nao marcar ponto)");
  dica_do_mapa_->setText(
    "Clique no mapa onde o ponto deve ficar e, sem soltar, arraste para o lado "
    "que o robo deve olhar. Ao soltar, o ponto e' gravado. Para mover a vista "
    "neste modo, arraste com o botao do meio do mouse.");
  status_->setText("Clique no mapa para marcar \"" + nome + "\".");
}

void MapasModule::corrigirNoMapa(bool ligar)
{
  if (!ligar) {
    preview_->setModo(MapPreview::Modo::Navegar);
    botao_corrigir_->setText("Corrigir a posicao (arrastar no mapa)");
    dica_do_mapa_->setText(
      "Arraste para mover a vista e use a roda do mouse para aproximar. Para "
      "mexer num ponto, ligue \"Corrigir a posicao\".");
    // O halo do ponto selecionado continua valendo: sair do modo de arrasto nao
    // e' o mesmo que deixar de olhar aquele ponto.
    const QString id = idSelecionado();
    if (!id.isEmpty()) {
      preview_->destacar(tipoSelecionado(), id);
    }
    return;
  }

  if (pastaDaArena().isEmpty()) {
    status_->setText("Escolha primeiro qual arena voce quer corrigir.");
    botao_corrigir_->setChecked(false);
    return;
  }
  if (botao_posicionar_->isChecked()) {
    botao_posicionar_->setChecked(false);
  }
  if (centro_->currentIndex() != kPaginaMapa) {
    mostrarPagina(kPaginaMapa);
    if (centro_->currentIndex() != kPaginaMapa) {
      botao_corrigir_->setChecked(false);
      return;
    }
  }

  preview_->setModo(MapPreview::Modo::PosicionarPose);
  botao_corrigir_->setText("Parar de corrigir");
  dica_do_mapa_->setText(
    "Arraste o CIRCULO de um ponto para mudar o lugar dele, ou a PONTA DA SETA "
    "para mudar o lado que o robo olha. Ao soltar, a tela pergunta antes de "
    "gravar. Neste modo a vista se move com o botao do meio do mouse.");
  const QString id = idSelecionado();
  if (!id.isEmpty()) {
    preview_->destacar(tipoSelecionado(), id);
  }
}

void MapasModule::aoEscolherPose(double x, double y, double yaw)
{
  if (!botao_posicionar_->isChecked()) {
    return;
  }
  const QString tipo = tipo_novo_->currentData().toString();
  const QString nome = tipo == kTipoWaypoint ?
    nome_novo_->text().trimmed() : idDePontoNormalizado(nome_novo_->text());

  const auto & mapa = preview_->mapa();
  const bool jaExiste =
    (tipo == kTipoDock && mapa.docks.contains(nome)) ||
    (tipo == kTipoArea && mapa.areas.contains(nome)) ||
    (tipo == kTipoWaypoint && mapa.waypoints.contains(nome));
  if (jaExiste) {
    const auto resp = QMessageBox::question(
      this, "Ponto ja' existe",
      "\"" + nome + "\" ja' existe nesta arena como " + rotuloDoTipo(tipo).toLower() +
      ".\n\nGravar agora substitui a posicao antiga pela que voce acabou de "
      "marcar no mapa. Uma copia de seguranca do arquivo e' criada "
      "automaticamente antes da troca.");
    if (resp != QMessageBox::Yes) {
      botao_posicionar_->setChecked(false);
      return;
    }
  }

  gravarPose(tipo, nome, tipo_de_area_->tipo(), x, y, yaw);
  botao_posicionar_->setChecked(false);
  nome_novo_->clear();
}

void MapasModule::aoArrastarMarcador(
  const QString & tipo, const QString & id, double x, double y, double yaw)
{
  // Arrastar um ponto e' a correcao mais pedida: um dock 20 cm torto fazia o
  // robo bater na mesa, e o unico conserto era levar o robo ate' la' e regravar
  // a pose -- na arena, durante a prova. Mesmo assim, um arrasto sem querer nao
  // pode reescrever o arquivo calado.
  const auto resp = QMessageBox::question(
    this, "Mover o ponto",
    "Gravar a nova posicao de \"" + id + "\" (" + rotuloDoTipo(tipo).toLower() + ")?\n\n"
    "Nova posicao: " + QString::number(x, 'f', 2) + " m, " +
    QString::number(y, 'f', 2) + " m.\n\n"
    "Uma copia de seguranca do arquivo e' criada automaticamente antes da troca.");
  if (resp != QMessageBox::Yes) {
    // Recarrega para o marcador voltar visualmente ao lugar de origem: deixar o
    // desenho na posicao nova depois de recusar seria mentir sobre o arquivo.
    recarregarPreview();
    return;
  }

  QString tipoDeArea = tipo_de_area_->tipo();
  if (tipo == kTipoArea) {
    const auto it = preview_->mapa().areas.constFind(id);
    if (it != preview_->mapa().areas.constEnd() && !it.value().tipo.isEmpty()) {
      // O tipo da area vem do arquivo, nao do combo da tela: mover um ponto nao
      // pode trocar uma prateleira em bancada por causa do que estava escolhido
      // no formulario de criacao.
      tipoDeArea = it.value().tipo;
    }
  }
  gravarPose(tipo, id, tipoDeArea, x, y, yaw);
}

void MapasModule::gravarPose(
  const QString & tipo, const QString & id, const QString & tipoDeArea,
  double x, double y, double yaw)
{
  const QString pasta = pastaDaArena();
  const QString arena = selectedMap();
  if (pasta.isEmpty() || arena.isEmpty()) {
    status_->setText("Escolha primeiro em qual arena o ponto vai ficar.");
    return;
  }

  if (tipo == kTipoWaypoint) {
    QString erro;
    if (!fazerBackup(pasta + "/waypoints.yaml", &erro)) {
      status_->setText(erro);
      return;
    }
    QMap<QString, PoseMapa> pontos = lerWaypointsDaArena(pasta);
    PoseMapa nova;
    nova.x = x;
    nova.y = y;
    nova.yaw = yaw;
    pontos.insert(id, nova);
    if (!gravarWaypoints(pasta, pontos, &erro)) {
      status_->setText(erro);
      return;
    }
    // A tela Waypoints trabalha com a lista em MEMORIA. Se ela estiver na mesma
    // arena e nao reler o arquivo, o proximo Salvar de la' regrava o
    // waypoints.yaml sem este ponto -- ou seja, apaga o que acabou de ser
    // gravado aqui, calado.
    auto * gerente = bridge_->waypoints();
    const bool mesmaArena = gerente && !gerente->mapDir().isEmpty() &&
      QFileInfo(gerente->mapDir()).absoluteFilePath() ==
      QFileInfo(pasta).absoluteFilePath();
    if (mesmaArena) {
      gerente->recarregar();
    }
    status_->setText(
      mesmaArena ?
      "Waypoint \"" + id + "\" gravado." :
      "Waypoint \"" + id + "\" gravado. Na tela Waypoints, escolha esta arena "
      "para ve'-lo na lista.");
    recarregarPreview();
    return;
  }

  PoseEscolhida pose;
  pose.informada = true;   // veio do clique no mapa, nao do TF do robo
  pose.x = x;
  pose.y = y;
  pose.yaw = yaw;

  if (tipo == kTipoDock) {
    // Tipo de dock vazio: quem decide e' o script, pelo prefixo do id (WS, SH,
    // PP, RT). Escolher aqui seria uma segunda regra para a mesma coisa.
    scripts_->gravarDock(arena, id, QString(), pose);
  } else {
    scripts_->gravarEstacao(arena, id, tipoDeArea, alturaDaMesaGravada(pasta, id), pose);
  }
  atualizarDisponibilidade();
}

void MapasModule::removerPontoSelecionado()
{
  const QString tipo = tipoSelecionado();
  const QString id = idSelecionado();
  const QString pasta = pastaDaArena();
  if (tipo.isEmpty() || id.isEmpty() || pasta.isEmpty()) {
    status_->setText("Escolha na lista qual ponto remover.");
    return;
  }

  // Um mesmo nome pode viver em DOIS arquivos: WS1 e' uma service area (onde o
  // robo para) e tambem um dock (como ele encosta). Apagar so' um dos dois
  // deixa a arena inconsistente -- e pior: o service_areas.yaml e' a fonte da
  // verdade do sync, entao um dock apagado sozinho VOLTA no proximo save de
  // area. Por isso a remocao trata os dois juntos e diz isso na pergunta.
  const bool ehArea = tipo == kTipoArea;
  const bool ehDock = tipo == kTipoDock;
  const bool temArea = preview_->mapa().areas.contains(id);
  const bool temDock = preview_->mapa().docks.contains(id);

  QStringList oQueSai;
  if (ehArea || ehDock) {
    if (temArea) {
      oQueSai << "a service area \"" + id + "\" (a pose em que o robo para)";
    }
    if (temDock) {
      oQueSai << "o dock \"" + id + "\" (como o robo encosta nela)";
    }
  } else {
    oQueSai << "o waypoint \"" + id + "\"";
  }

  QString pergunta = "Remover " + oQueSai.join(" e ") + " da arena?";
  if ((ehArea || ehDock) && temArea && temDock) {
    pergunta += "\n\nOs dois saem juntos de proposito: apagar so' um deles faz o "
      "outro voltar sozinho na proxima vez que a arena for salva.";
  }
  pergunta += "\n\nUma copia de seguranca de cada arquivo alterado fica na pasta "
    ".backup, dentro da pasta da arena.";

  if (QMessageBox::question(this, "Remover ponto", pergunta) != QMessageBox::Yes) {
    return;
  }

  QStringList feitos;
  QStringList falhas;

  auto tirarDoArquivo = [&](
    const QString & arquivo, const QString & secao, const QString & rotulo) {
      if (!QFile::exists(arquivo)) {
        return;
      }
      QString erro;
      if (!fazerBackup(arquivo, &erro)) {
        falhas << erro;
        return;
      }
      bool removeu = false;
      if (!removerChaveDoYaml(arquivo, secao, id, &removeu, &erro)) {
        falhas << erro;
        return;
      }
      if (removeu) {
        feitos << rotulo;
      }
    };

  if (ehArea || ehDock) {
    if (temArea) {
      tirarDoArquivo(pasta + "/service_areas.yaml", "service_areas", "service area");
    }
    if (temDock) {
      tirarDoArquivo(pasta + "/docking.yaml", "docks", "dock");
    }
  } else {
    QString erro;
    if (!fazerBackup(pasta + "/waypoints.yaml", &erro)) {
      falhas << erro;
    } else {
      QMap<QString, PoseMapa> pontos = lerWaypointsDaArena(pasta);
      pontos.remove(id);
      if (gravarWaypoints(pasta, pontos, &erro)) {
        feitos << "waypoint";
        // Mesmo motivo da gravacao: a tela Waypoints guarda a lista em memoria
        // e o Salvar dela ressuscitaria o ponto apagado aqui.
        auto * gerente = bridge_->waypoints();
        if (gerente && !gerente->mapDir().isEmpty() &&
          QFileInfo(gerente->mapDir()).absoluteFilePath() ==
          QFileInfo(pasta).absoluteFilePath())
        {
          gerente->recarregar();
        }
      } else {
        falhas << erro;
      }
    }
  }

  if (!falhas.isEmpty()) {
    status_->setText("Nao consegui remover \"" + id + "\". " + falhas.first());
  } else if (feitos.isEmpty()) {
    status_->setText("Nao encontrei \"" + id + "\" nos arquivos da arena.");
  } else {
    status_->setText("\"" + id + "\" removido (" + feitos.join(" e ") + ").");
  }
  recarregarPreview();
}

// ------------------------------------------------------------- apoio
void MapasModule::mostrarPagina(int pagina)
{
  const int atual = centro_->currentIndex();
  if (atual == pagina) {
    return;
  }

  if (pagina != kPaginaMapa) {
    // Nao da' para pintar e mexer nos pontos ao mesmo tempo: sao significados
    // diferentes para o mesmo clique, e o mapa nem esta' mais na frente.
    if (botao_posicionar_->isChecked()) {
      botao_posicionar_->setChecked(false);
    }
    if (botao_corrigir_->isChecked()) {
      botao_corrigir_->setChecked(false);
    }
  }

  // Sair de uma edicao sem salvar joga fora tudo que foi pintado, e o mapa ao
  // lado volta a mostrar o desenho antigo -- pareceria que a edicao "nao
  // pegou". Perguntar e' obrigatorio aqui.
  const bool saindoDaPintura = atual == kPaginaPintura && editor_->temAlteracoesNaoSalvas();
  const bool saindoDasParedes = atual == kPaginaParedes && paredes_->temAlteracoesNaoSalvas();
  if (saindoDaPintura || saindoDasParedes) {
    const auto resp = QMessageBox::question(
      this, "Sair sem salvar",
      QString("Voce pintou %1 e ainda nao salvou.\n\nSalvar agora?")
      .arg(saindoDaPintura ? "o mapa" : "as paredes virtuais"),
      QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    if (resp == QMessageBox::Cancel) {
      return;
    }
    if (resp == QMessageBox::Save) {
      const bool salvou = saindoDaPintura ? editor_->salvar() : paredes_->salvar();
      if (!salvou) {
        return;   // o proprio editor ja' explicou o que deu errado no status
      }
    }
  }

  centro_->setCurrentIndex(pagina);
  botao_limpar_ruido_->setText(
    pagina == kPaginaPintura ? "Voltar para a visao da arena" : "Limpar ruido (pintar o mapa)");
  botao_paredes_->setText(
    pagina == kPaginaParedes ?
    "Voltar para a visao da arena" : "Paredes virtuais (onde o robo nao entra)");

  const QString arena = selectedMap();
  switch (pagina) {
    case kPaginaPintura:
      // Os editores seguem a arena escolhida AQUI. Sem isto o de baixo abriria
      // com o proprio seletor dele, e o operador pintaria o mapa de outra arena
      // achando que estava consertando a que acabou de olhar.
      editor_->setArena(arena);
      editor_->carregar();
      dica_do_mapa_->setText(
        "Pinte de PAREDE (preto) o que o robo deve enxergar como obstaculo, de "
        "LIVRE (branco) o que apareceu no mapa mas nao existe, e de "
        "DESCONHECIDO (cinza) o que ninguem mapeou. Salvar cria uma copia de "
        "seguranca automatica.");
      break;
    case kPaginaParedes:
      paredes_->setArena(arena);
      paredes_->recarregar();
      dica_do_mapa_->setText(
        "O vermelho e' onde o robo NAO pode entrar. O mapa aparece por baixo so' "
        "como referencia: nada do que voce pinta aqui muda o mapa nem atrapalha "
        "o robo a se localizar.");
      break;
    default:
      dica_do_mapa_->setText(
        botao_corrigir_->isChecked() ?
        "Arraste o CIRCULO de um ponto para mudar o lugar dele, ou a PONTA DA "
        "SETA para mudar o lado que o robo olha." :
        "Arraste para mover a vista e use a roda do mouse para aproximar. Para "
        "mexer num ponto, ligue \"Corrigir a posicao\".");
      // Ao voltar, recarrega o preview: os editores podem ter mudado o bitmap
      // ou a mascara, e mostrar o desenho antigo faria o operador achar que a
      // edicao nao pegou.
      recarregarPreview();
      break;
  }
}

void MapasModule::recarregarPreview()
{
  const QString pasta = pastaDaArena();
  if (!pasta.isEmpty()) {
    preview_->carregar(pasta);
  }
}

QString MapasModule::tipoSelecionado() const
{
  auto * item = conteudo_->currentItem();
  return item ? item->data(Qt::UserRole).toString() : QString();
}

QString MapasModule::idSelecionado() const
{
  auto * item = conteudo_->currentItem();
  return item ? item->data(Qt::UserRole + 1).toString() : QString();
}

void MapasModule::atualizarDisponibilidade()
{
  const bool temArena = !pastaDaArena().isEmpty();
  const bool gravando = scripts_ && scripts_->ocupado();
  const bool temPonto = !idSelecionado().isEmpty();

  botao_remover_ponto_->setEnabled(temArena && temPonto && !gravando);
  botao_corrigir_->setEnabled(temArena && !gravando);
  botao_posicionar_->setEnabled(temArena && !gravando);
  botao_limpar_ruido_->setEnabled(temArena);
  botao_paredes_->setEnabled(temArena);

  QString motivo;
  if (!temArena) {
    motivo = "Escolha uma arena na lista a esquerda. Sem arena selecionada nao "
      "ha' o que ver nem o que corrigir; se a lista estiver vazia, o robo ainda "
      "nao tem nenhum mapa salvo — crie um em Mapeamento.";
  } else if (gravando) {
    motivo = "Aguarde: o ponto ainda esta' sendo gravado no arquivo da arena.";
  } else if (!temPonto) {
    motivo = "Remover esta' desligado porque nenhum ponto esta' selecionado na "
      "lista acima.";
  }
  motivo_->setText(motivo);
}

QString MapasModule::pastaDaArena() const
{
  const QString nome = selectedMap();
  return nome.isEmpty() ? QString() : mapsDir() + "/" + nome;
}

QString MapasModule::selectedMap() const
{
  auto * item = lista_->currentItem();
  return item ? item->text() : QString();
}

void MapasModule::refresh()
{
  const QString selecionada = selectedMap();
  lista_->clear();
  QDir dir(mapsDir());
  for (const QString & nome :
    dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name))
  {
    if (nome.startsWith(".")) {
      continue;
    }
    if (!QFile::exists(dir.filePath(nome + "/map.yaml"))) {
      continue;
    }
    auto * item = new QListWidgetItem(nome);
    QPixmap pm(dir.filePath(nome + "/map.pgm"));
    if (!pm.isNull()) {
      item->setIcon(QIcon(pm.scaled(48, 48, Qt::KeepAspectRatio)));
    }
    lista_->addItem(item);
  }

  arena_ativa_->setText(
    "Arena ativa no robo: " +
    (bridge_->robotState()->mapName().isEmpty() ?
    QString("nenhuma") : bridge_->robotState()->mapName()));

  // Abre na arena que o robo esta usando; e' quase sempre a que interessa.
  const QString alvo = selecionada.isEmpty() ? bridge_->robotState()->mapName() : selecionada;
  const auto encontrados = lista_->findItems(alvo, Qt::MatchExactly);
  if (!encontrados.isEmpty()) {
    lista_->setCurrentItem(encontrados.first());
  } else if (lista_->count() > 0) {
    lista_->setCurrentRow(0);
  }
  atualizarDisponibilidade();
}
