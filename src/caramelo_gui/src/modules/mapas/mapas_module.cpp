#include "modules/mapas/mapas_module.hpp"

#include <QDateTime>
#include <QDir>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPixmap>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QVBoxLayout>

#include "ament_index_cpp/get_package_share_directory.hpp"

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
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Mapas");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  lista_ = new QListWidget();
  lista_->setIconSize(QSize(96, 96));
  layout->addWidget(lista_, 1);

  auto * botoes = new QHBoxLayout();
  auto addBtn = [this, botoes](const QString & texto, auto slot) {
      auto * b = new QPushButton(texto);
      connect(b, &QPushButton::clicked, this, slot);
      botoes->addWidget(b);
      return b;
    };

  addBtn(
    "Usar", [this]() {
      const QString nome = selectedMap();
      if (nome.isEmpty()) {return;}
      bridge_->loadMap(mapsDir() + "/" + nome + "/map.yaml");
    })->setObjectName("acaoPrimaria");

  addBtn(
    "Renomear", [this]() {
      const QString nome = selectedMap();
      if (nome.isEmpty()) {return;}
      bool ok = false;
      const QString novo = QInputDialog::getText(
        this, "Renomear mapa", "Novo nome:", QLineEdit::Normal, nome, &ok).trimmed();
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
        this, "Excluir mapa",
        QString("Mover o mapa '%1' para a lixeira?").arg(nome));
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

  addBtn(
    "Salvar mapa atual...", [this]() {
      bool ok = false;
      const QString nome = QInputDialog::getText(
        this, "Salvar mapa do SLAM", "Nome do novo mapa:",
        QLineEdit::Normal, "novo_mapa", &ok).trimmed();
      if (!ok || nome.isEmpty()) {return;}
      QDir().mkpath(mapsDir() + "/" + nome);
      bridge_->saveMap(mapsDir(), nome);
      refresh();
    });

  addBtn("Atualizar", [this]() {refresh();});
  layout->addLayout(botoes);

  status_ = new QLabel(QString("Pasta: %1").arg(mapsDir()));
  status_->setObjectName("msgCartao");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  connect(
    bridge_, &RosBridge::mapStatus, this,
    [this](bool ok, const QString & m) {
      status_->setText((ok ? "" : "Falha: ") + m);
    });

  refresh();
}

QString MapasModule::selectedMap() const
{
  auto * item = lista_->currentItem();
  return item ? item->text() : QString();
}

void MapasModule::refresh()
{
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
    // Miniatura direto do map.pgm (Qt le PGM nativamente).
    QPixmap pm(dir.filePath(nome + "/map.pgm"));
    if (!pm.isNull()) {
      item->setIcon(QIcon(pm.scaled(96, 96, Qt::KeepAspectRatio)));
    }
    lista_->addItem(item);
  }
}
