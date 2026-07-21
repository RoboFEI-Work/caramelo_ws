#include "modules/editor_mapa/editor_mapa_module.hpp"

#include <fstream>

#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QTextStream>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"
#include "modules/mapas/mapas_module.hpp"

// ============================== MapCanvas ==================================

MapCanvas::MapCanvas(QWidget * parent)
: QWidget(parent)
{
  setAttribute(Qt::WA_StaticContents);
}

bool MapCanvas::loadPgm(const QString & path)
{
  QImage img(path);
  if (img.isNull()) {
    return false;
  }
  img_ = img.convertToFormat(QImage::Format_Grayscale8);
  undo_stack_.clear();
  modified_ = false;
  resize(sizeHint());
  update();
  return true;
}

bool MapCanvas::savePgm(const QString & path) const
{
  // Qt nao ESCREVE PGM; gravamos P5 na mao (mesma geometria/valores do nav2).
  std::ofstream out(path.toStdString(), std::ios::binary);
  if (!out || img_.isNull()) {
    return false;
  }
  out << "P5\n" << img_.width() << " " << img_.height() << "\n255\n";
  for (int y = 0; y < img_.height(); ++y) {
    out.write(reinterpret_cast<const char *>(img_.constScanLine(y)), img_.width());
  }
  return static_cast<bool>(out);
}

void MapCanvas::setZoom(double zoom)
{
  zoom_ = qBound(0.5, zoom, 16.0);
  resize(sizeHint());
  update();
}

void MapCanvas::undo()
{
  if (undo_stack_.isEmpty()) {
    return;
  }
  img_ = undo_stack_.takeLast();
  update();
}

QSize MapCanvas::sizeHint() const
{
  return img_.isNull() ? QSize(400, 300) : img_.size() * zoom_;
}

void MapCanvas::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.scale(zoom_, zoom_);
  painter.drawImage(0, 0, img_);
}

void MapCanvas::mousePressEvent(QMouseEvent * event)
{
  if (img_.isNull() || !(event->buttons() & Qt::LeftButton)) {
    return;
  }
  // Um snapshot por traco (pressionar = inicio de traco).
  undo_stack_.push_back(img_);
  if (undo_stack_.size() > 15) {
    undo_stack_.removeFirst();
  }
  paintAt(event->pos());
}

void MapCanvas::mouseMoveEvent(QMouseEvent * event)
{
  if (!img_.isNull() && (event->buttons() & Qt::LeftButton)) {
    paintAt(event->pos());
  }
}

void MapCanvas::paintAt(const QPoint & widget_pos)
{
  const int cx = static_cast<int>(widget_pos.x() / zoom_);
  const int cy = static_cast<int>(widget_pos.y() / zoom_);
  const int r = brush_size_;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy > r * r) {
        continue;
      }
      const int x = cx + dx;
      const int y = cy + dy;
      if (x >= 0 && x < img_.width() && y >= 0 && y < img_.height()) {
        img_.scanLine(y)[x] = static_cast<uchar>(brush_value_);
      }
    }
  }
  modified_ = true;
  update();
  emit edited();
}

// =========================== EditorMapaModule ==============================

EditorMapaModule::EditorMapaModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Editor de Mapa");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  // Linha 1: mapa + carregar/salvar.
  auto * topo = new QHBoxLayout();
  map_name_ = new QLineEdit("arena2_520");
  topo->addWidget(new QLabel("Mapa:"));
  topo->addWidget(map_name_);

  auto * carregar = new QPushButton("Carregar");
  connect(
    carregar, &QPushButton::clicked, this, [this]() {
      const QString pgm = mapDir() + "/map.pgm";
      status_->setText(
        canvas_->loadPgm(pgm) ? QString("Carregado: %1").arg(pgm)
        : QString("Falha ao carregar %1").arg(pgm));
      loadOrigin();
    });
  topo->addWidget(carregar);

  auto * salvar = new QPushButton("Salvar (com backup)");
  salvar->setObjectName("acaoPrimaria");
  connect(
    salvar, &QPushButton::clicked, this, [this]() {
      const QString pgm = mapDir() + "/map.pgm";
      // Backup antes de sobrescrever (regra do plano).
      const QString backup = pgm + ".bak_" +
        QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
      QFile::copy(pgm, backup);
      if (canvas_->savePgm(pgm)) {
        canvas_->clearModified();
        status_->setText(QString("Salvo. Backup: %1").arg(backup));
      } else {
        status_->setText("Falha ao salvar.");
      }
    });
  topo->addWidget(salvar);

  auto * recarregar = new QPushButton("Recarregar no map_server");
  connect(
    recarregar, &QPushButton::clicked, this,
    [this]() {bridge_->loadMap(mapDir() + "/map.yaml");});
  topo->addWidget(recarregar);
  layout->addLayout(topo);

  // Linha 2: ferramenta + tamanho + zoom + desfazer.
  auto * barra = new QHBoxLayout();
  ferramenta_ = new QComboBox();
  ferramenta_->addItem("Borracha (livre)", 254);
  ferramenta_->addItem("Parede (ocupado)", 0);
  ferramenta_->addItem("Desconhecido", 205);
  connect(
    ferramenta_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int) {canvas_->setBrushValue(ferramenta_->currentData().toInt());});
  barra->addWidget(new QLabel("Pincel:"));
  barra->addWidget(ferramenta_);

  tamanho_ = new QSlider(Qt::Horizontal);
  tamanho_->setRange(1, 25);
  tamanho_->setValue(3);
  tamanho_->setFixedWidth(140);
  connect(
    tamanho_, &QSlider::valueChanged, this,
    [this](int v) {canvas_->setBrushSize(v);});
  barra->addWidget(new QLabel("Tamanho:"));
  barra->addWidget(tamanho_);

  auto * zoomIn = new QPushButton("Zoom +");
  connect(
    zoomIn, &QPushButton::clicked, this,
    [this]() {canvas_->setZoom(canvas_->zoom() * 1.5);});
  barra->addWidget(zoomIn);
  auto * zoomOut = new QPushButton("Zoom -");
  connect(
    zoomOut, &QPushButton::clicked, this,
    [this]() {canvas_->setZoom(canvas_->zoom() / 1.5);});
  barra->addWidget(zoomOut);

  auto * desfazer = new QPushButton("Desfazer");
  connect(desfazer, &QPushButton::clicked, this, [this]() {canvas_->undo();});
  barra->addWidget(desfazer);
  barra->addStretch();
  layout->addLayout(barra);

  // Canvas com scroll (pan pelas barras).
  canvas_ = new MapCanvas();
  scroll_ = new QScrollArea();
  scroll_->setWidget(canvas_);
  layout->addWidget(scroll_, 1);

  // Linha 3: origem do mapa (map.yaml).
  auto * origem = new QHBoxLayout();
  origem->addWidget(new QLabel("Origem do mapa (m):"));
  origem_x_ = new QDoubleSpinBox();
  origem_x_->setRange(-1000, 1000);
  origem_x_->setDecimals(3);
  origem_y_ = new QDoubleSpinBox();
  origem_y_->setRange(-1000, 1000);
  origem_y_->setDecimals(3);
  origem->addWidget(new QLabel("x:"));
  origem->addWidget(origem_x_);
  origem->addWidget(new QLabel("y:"));
  origem->addWidget(origem_y_);
  auto * aplicarOrigem = new QPushButton("Aplicar origem");
  connect(aplicarOrigem, &QPushButton::clicked, this, [this]() {applyOrigin();});
  origem->addWidget(aplicarOrigem);
  origem->addStretch();
  layout->addLayout(origem);

  status_ = new QLabel("Carregue um mapa para editar.");
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);
}

QString EditorMapaModule::mapDir() const
{
  return MapasModule::mapsDir() + "/" + map_name_->text().trimmed();
}

void EditorMapaModule::loadOrigin()
{
  QFile file(mapDir() + "/map.yaml");
  if (!file.open(QFile::ReadOnly | QFile::Text)) {
    return;
  }
  const QString text = QString::fromUtf8(file.readAll());
  for (const QString & line : text.split('\n')) {
    const QString t = line.trimmed();
    if (t.startsWith("origin:")) {
      const QString inner = t.section('[', 1).section(']', 0, 0);
      const QStringList parts = inner.split(',');
      if (parts.size() >= 2) {
        origem_x_->setValue(parts[0].trimmed().toDouble());
        origem_y_->setValue(parts[1].trimmed().toDouble());
      }
    }
  }
}

void EditorMapaModule::applyOrigin()
{
  const QString path = mapDir() + "/map.yaml";
  QFile file(path);
  if (!file.open(QFile::ReadOnly | QFile::Text)) {
    status_->setText("map.yaml nao encontrado.");
    return;
  }
  QStringList lines = QString::fromUtf8(file.readAll()).split('\n');
  file.close();

  for (QString & line : lines) {
    if (line.trimmed().startsWith("origin:")) {
      // Preserva o yaw atual (3o elemento), se existir.
      double yaw = 0.0;
      const QString inner = line.section('[', 1).section(']', 0, 0);
      const QStringList parts = inner.split(',');
      if (parts.size() >= 3) {
        yaw = parts[2].trimmed().toDouble();
      }
      line = QString("origin: [%1, %2, %3]")
        .arg(origem_x_->value(), 0, 'f', 3)
        .arg(origem_y_->value(), 0, 'f', 3)
        .arg(yaw);
    }
  }

  QFile::copy(path, path + ".bak");
  QFile out(path);
  if (!out.open(QFile::WriteOnly | QFile::Text)) {
    status_->setText("Falha ao escrever map.yaml.");
    return;
  }
  QTextStream ts(&out);
  ts << lines.join('\n');
  status_->setText("Origem aplicada (backup map.yaml.bak). Recarregue no map_server.");
}
