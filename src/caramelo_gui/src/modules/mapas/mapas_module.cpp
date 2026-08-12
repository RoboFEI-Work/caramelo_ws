#include "modules/mapas/mapas_module.hpp"

#include <QDateTime>
#include <QDir>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPixmap>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "bridge/robot_state.hpp"
#include "bridge/ros_bridge.hpp"

namespace
{
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
  // Tres colunas: arenas | o mapa como ele foi salvo | o que ha' dentro dele.
  auto * layout = new QHBoxLayout(this);
  layout->setContentsMargins(12, 12, 12, 12);
  layout->setSpacing(14);

  layout->addWidget(construirListaEAcoes(), 0);

  preview_ = new MapPreview();
  layout->addWidget(preview_, 1);

  layout->addWidget(construirDetalhes(), 0);

  connect(
    lista_, &QListWidget::currentItemChanged, this,
    [this](QListWidgetItem * atual, QListWidgetItem *) {
      if (atual) {
        preview_->carregar(mapsDir() + "/" + atual->text());
      }
    });
  connect(preview_, &MapPreview::carregado, this, &MapasModule::mostrarResumo);

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

  refresh();
}

QWidget * MapasModule::construirListaEAcoes()
{
  auto * col = new QWidget();
  col->setFixedWidth(250);
  auto * layout = new QVBoxLayout(col);
  layout->setContentsMargins(0, 0, 0, 0);

  auto * title = new QLabel("Arenas");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  arena_ativa_ = new QLabel("Arena ativa no robo: —");
  arena_ativa_->setObjectName("msgCartao");
  arena_ativa_->setWordWrap(true);
  layout->addWidget(arena_ativa_);

  lista_ = new QListWidget();
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
        this, "Renomear arena", "Novo nome:", QLineEdit::Normal, nome, &ok).trimmed();
      if (!ok || novo.isEmpty() || novo == nome) {return;}
      if (!QDir(mapsDir()).rename(nome, novo)) {
        QMessageBox::warning(this, "Renomear", "Nao foi possivel renomear.");
      }
      refresh();
    });

  addBtn(
    "Duplicar", [this]() {
      const QString nome = selectedMap();
      if (nome.isEmpty()) {return;}
      const QString copia = nome + "_copia";
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
        QString("Mover a arena '%1' para a lixeira?").arg(nome));
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
    });

  addBtn("Atualizar lista", [this]() {refresh();});

  status_ = new QLabel();
  status_->setObjectName("msgCartao");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  return col;
}

QWidget * MapasModule::construirDetalhes()
{
  auto * col = new QWidget();
  col->setFixedWidth(300);
  auto * layout = new QVBoxLayout(col);
  layout->setContentsMargins(0, 0, 0, 0);

  auto * grupoCamadas = new QGroupBox("Camadas");
  auto * camadasLayout = new QVBoxLayout(grupoCamadas);
  for (const QString & camada : MapPreview::camadasDisponiveis()) {
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

  auto * grupoResumo = new QGroupBox("O que esta salvo");
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
  return col;
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
}
