#include "modules/editor_mapa/editor_mapa_module.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QTextStream>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"
#include "widgets/seletor_arena.hpp"
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
  arrastando_ = false;
  overlay_valido_ = false;
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

void MapCanvas::setImagemDeFundo(const QImage & fundo)
{
  fundo_ = fundo.isNull() ? QImage() : fundo.convertToFormat(QImage::Format_Grayscale8);
  update();
}

void MapCanvas::setModoMascara(bool mascara)
{
  modo_mascara_ = mascara;
  overlay_valido_ = false;
  update();
}

void MapCanvas::setZoom(double zoom)
{
  zoom_ = qBound(0.5, zoom, 16.0);
  resize(sizeHint());
  update();
}

void MapCanvas::ajustarPara(const QSize & area)
{
  if (img_.isNull() || area.width() <= 0 || area.height() <= 0) {
    return;
  }
  const double sx = static_cast<double>(area.width()) / img_.width();
  const double sy = static_cast<double>(area.height()) / img_.height();
  setZoom(std::min(sx, sy));
}

void MapCanvas::undo()
{
  if (undo_stack_.isEmpty()) {
    return;
  }
  img_ = undo_stack_.takeLast();
  overlay_valido_ = false;
  update();
}

QSize MapCanvas::sizeHint() const
{
  return img_.isNull() ? QSize(400, 300) : img_.size() * zoom_;
}

QPoint MapCanvas::paraImagem(const QPointF & pos) const
{
  return QPoint(
    static_cast<int>(std::floor(pos.x() / zoom_)),
    static_cast<int>(std::floor(pos.y() / zoom_)));
}

void MapCanvas::reconstruirOverlay() const
{
  if (overlay_valido_ || img_.isNull()) {
    return;
  }
  overlay_ = QImage(img_.size(), QImage::Format_ARGB32_Premultiplied);
  overlay_.fill(Qt::transparent);
  for (int y = 0; y < img_.height(); ++y) {
    const uchar * linha = img_.constScanLine(y);
    auto * saida = reinterpret_cast<QRgb *>(overlay_.scanLine(y));
    for (int x = 0; x < img_.width(); ++x) {
      // Mascara de keepout: preto = bloqueado. So' o bloqueado e' pintado, para
      // o mapa continuar visivel por baixo.
      if (linha[x] < 128) {
        saida[x] = qPremultiply(qRgba(235, 87, 87, 130));
      }
    }
  }
  overlay_valido_ = true;
}

void MapCanvas::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  if (img_.isNull()) {
    return;
  }
  painter.scale(zoom_, zoom_);

  if (modo_mascara_) {
    if (!fundo_.isNull()) {
      painter.drawImage(0, 0, fundo_);
    } else {
      painter.fillRect(QRect(QPoint(0, 0), img_.size()), QColor("#e9edf2"));
    }
    reconstruirOverlay();
    painter.drawImage(0, 0, overlay_);
  } else {
    painter.drawImage(0, 0, img_);
  }

  // Previa do que vai ser pintado ao soltar. Sem ela, retangulo e linha viram
  // aposta: o operador so' descobre onde acertou depois de ja' ter pintado.
  if (arrastando_ && ferramenta_ != Ferramenta::Pincel) {
    QPen caneta(QColor("#35c3f0"));
    caneta.setCosmetic(true);   // 1 px de TELA, independente do zoom
    caneta.setWidth(1);
    painter.setPen(caneta);
    painter.setBrush(Qt::NoBrush);
    if (ferramenta_ == Ferramenta::Retangulo) {
      painter.drawRect(QRect(inicio_, atual_).normalized());
    } else {
      painter.drawLine(inicio_, atual_);
    }
  }
}

void MapCanvas::guardarParaDesfazer()
{
  undo_stack_.push_back(img_);
  if (undo_stack_.size() > 15) {
    undo_stack_.removeFirst();
  }
}

void MapCanvas::mousePressEvent(QMouseEvent * event)
{
  if (img_.isNull() || !(event->buttons() & Qt::LeftButton)) {
    return;
  }
  // Um snapshot por traco (pressionar = inicio de traco).
  guardarParaDesfazer();
  arrastando_ = true;
  inicio_ = ultimo_ = atual_ = paraImagem(event->pos());
  if (ferramenta_ == Ferramenta::Pincel) {
    pintarEm(inicio_);
    marcarEditado();
  } else {
    update();
  }
}

void MapCanvas::mouseMoveEvent(QMouseEvent * event)
{
  if (img_.isNull() || !arrastando_) {
    return;
  }
  atual_ = paraImagem(event->pos());
  if (ferramenta_ == Ferramenta::Pincel) {
    // Interpolar entre os pontos do movimento: o mouse manda um evento a cada
    // poucos milissegundos, e num traco rapido dois eventos vizinhos ficam a
    // dezenas de pixels um do outro. Sintoma antigo: a parede virtual saia
    // pontilhada, com buracos por onde o planejador passava o robo.
    pintarLinha(ultimo_, atual_);
    ultimo_ = atual_;
    marcarEditado();
  } else {
    update();
  }
}

void MapCanvas::mouseReleaseEvent(QMouseEvent * event)
{
  if (img_.isNull() || !arrastando_ || event->button() != Qt::LeftButton) {
    return;
  }
  arrastando_ = false;
  atual_ = paraImagem(event->pos());
  if (ferramenta_ == Ferramenta::Retangulo) {
    pintarRetangulo(QRect(inicio_, atual_).normalized());
    marcarEditado();
  } else if (ferramenta_ == Ferramenta::Linha) {
    pintarLinha(inicio_, atual_);
    marcarEditado();
  }
  update();
}

void MapCanvas::pintarEm(const QPoint & pixel)
{
  const int r = brush_size_;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy > r * r) {
        continue;
      }
      const int x = pixel.x() + dx;
      const int y = pixel.y() + dy;
      if (x >= 0 && x < img_.width() && y >= 0 && y < img_.height()) {
        img_.scanLine(y)[x] = static_cast<uchar>(brush_value_);
      }
    }
  }
}

void MapCanvas::pintarLinha(const QPoint & de, const QPoint & ate)
{
  const int dx = ate.x() - de.x();
  const int dy = ate.y() - de.y();
  const int passos = std::max(std::abs(dx), std::abs(dy));
  if (passos == 0) {
    pintarEm(de);
    return;
  }
  for (int i = 0; i <= passos; ++i) {
    const double t = static_cast<double>(i) / passos;
    pintarEm(
      QPoint(
        de.x() + static_cast<int>(std::lround(dx * t)),
        de.y() + static_cast<int>(std::lround(dy * t))));
  }
}

void MapCanvas::pintarRetangulo(const QRect & area)
{
  const QRect corte = area.intersected(QRect(QPoint(0, 0), img_.size()));
  for (int y = corte.top(); y <= corte.bottom(); ++y) {
    uchar * linha = img_.scanLine(y);
    for (int x = corte.left(); x <= corte.right(); ++x) {
      linha[x] = static_cast<uchar>(brush_value_);
    }
  }
}

void MapCanvas::marcarEditado()
{
  modified_ = true;
  overlay_valido_ = false;
  update();
  emit edited();
}

// =========================== EditorMapaModule ==============================

EditorMapaModule::EditorMapaModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge)
{
  auto * layout = new QVBoxLayout(this);

  auto * title = new QLabel("Limpar o mapa");
  title->setObjectName("tituloModulo");
  layout->addWidget(title);

  auto * explica = new QLabel(
    "Apague do mapa o que nao existe de verdade: gente que passou durante o "
    "mapeamento, sombras do LiDAR, riscos soltos. Pinte de PAREDE o que e' "
    "parede e ficou falhado.");
  explica->setObjectName("msgCartao");
  explica->setWordWrap(true);
  layout->addWidget(explica);

  // Linha 1: mapa + carregar/salvar.
  linha_arena_ = new QWidget();
  auto * topo = new QHBoxLayout(linha_arena_);
  topo->setContentsMargins(0, 0, 0, 0);
  map_name_ = new SeletorArena(bridge);
  topo->addWidget(new QLabel("Mapa:"));
  topo->addWidget(map_name_, 1);

  auto * botao_carregar = new QPushButton("Carregar");
  connect(botao_carregar, &QPushButton::clicked, this, [this]() {carregar();});
  topo->addWidget(botao_carregar);
  layout->addWidget(linha_arena_);

  // Linha 2: ferramenta + forma + tamanho + zoom + desfazer.
  auto * barra = new QHBoxLayout();
  ferramenta_ = new QComboBox();
  ferramenta_->addItem("Apagar (livre)", 254);
  ferramenta_->addItem("Parede", 0);
  ferramenta_->addItem("Desconhecido", 205);
  connect(
    ferramenta_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int) {canvas_->setBrushValue(ferramenta_->currentData().toInt());});
  barra->addWidget(new QLabel("Pintar de:"));
  barra->addWidget(ferramenta_);

  forma_ = new QComboBox();
  forma_->addItem("Pincel", static_cast<int>(MapCanvas::Ferramenta::Pincel));
  forma_->addItem("Retangulo", static_cast<int>(MapCanvas::Ferramenta::Retangulo));
  forma_->addItem("Linha", static_cast<int>(MapCanvas::Ferramenta::Linha));
  connect(
    forma_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int) {
      canvas_->setFerramenta(
        static_cast<MapCanvas::Ferramenta>(forma_->currentData().toInt()));
    });
  barra->addWidget(new QLabel("Forma:"));
  barra->addWidget(forma_);

  tamanho_ = new QSlider(Qt::Horizontal);
  tamanho_->setRange(1, 25);
  tamanho_->setValue(3);
  tamanho_->setFixedWidth(120);
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
  auto * encaixar = new QPushButton("Ver tudo");
  connect(
    encaixar, &QPushButton::clicked, this,
    [this]() {canvas_->ajustarPara(scroll_->viewport()->size());});
  barra->addWidget(encaixar);

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

  // Linha 3: salvar / recarregar / origem do mapa.
  auto * acoes = new QHBoxLayout();
  auto * botao_salvar = new QPushButton("Salvar o mapa limpo");
  botao_salvar->setObjectName("acaoPrimaria");
  connect(botao_salvar, &QPushButton::clicked, this, [this]() {salvar();});
  acoes->addWidget(botao_salvar);

  auto * recarregar = new QPushButton("Usar agora no robo");
  connect(
    recarregar, &QPushButton::clicked, this,
    [this]() {bridge_->loadMap(mapDir() + "/map.yaml");});
  acoes->addWidget(recarregar);
  acoes->addStretch();
  layout->addLayout(acoes);

  auto * origem = new QHBoxLayout();
  origem->addWidget(new QLabel("Onde fica o canto do mapa (m):"));
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
  auto * aplicarOrigem = new QPushButton("Aplicar");
  connect(aplicarOrigem, &QPushButton::clicked, this, [this]() {applyOrigin();});
  origem->addWidget(aplicarOrigem);
  origem->addStretch();
  layout->addLayout(origem);

  status_ = new QLabel("Carregue um mapa para editar.");
  status_->setObjectName("estadoAtual");
  status_->setWordWrap(true);
  layout->addWidget(status_);
}

void EditorMapaModule::setArena(const QString & arena)
{
  arena_fixa_ = arena.trimmed();
  linha_arena_->setVisible(arena_fixa_.isEmpty());
  carregar();
}

QString EditorMapaModule::mapDir() const
{
  const QString arena = arena_fixa_.isEmpty() ? map_name_->arena().trimmed() : arena_fixa_;
  return MapasModule::mapsDir() + "/" + arena;
}

void EditorMapaModule::carregar()
{
  const QString pgm = mapDir() + "/map.pgm";
  const bool ok = canvas_->loadPgm(pgm);
  status_->setText(
    ok ? QString("Mapa aberto. Pinte para corrigir e salve quando terminar.")
    : QString("Nao consegui abrir o desenho deste mapa (%1).").arg(pgm));
  emit status(status_->text());
  if (ok) {
    canvas_->ajustarPara(scroll_->viewport()->size());
  }
  loadOrigin();
}

bool EditorMapaModule::temAlteracoesNaoSalvas() const
{
  return canvas_->isModified();
}

bool EditorMapaModule::salvar()
{
  const QString pgm = mapDir() + "/map.pgm";
  if (!canvas_->temImagem()) {
    status_->setText("Nao ha' mapa aberto para salvar.");
    emit status(status_->text());
    return false;
  }
  // Backup antes de sobrescrever: um mapa apagado por engano custa uma tarde
  // inteira de mapeamento.
  const QString backup = pgm + ".bak_" +
    QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
  QFile::copy(pgm, backup);
  if (canvas_->savePgm(pgm)) {
    canvas_->clearModified();
    status_->setText("Mapa salvo. A versao anterior ficou guardada ao lado.");
    emit status(status_->text());
    return true;
  }
  status_->setText("Nao consegui salvar o mapa (a pasta esta protegida?).");
  emit status(status_->text());
  return false;
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

bool EditorMapaModule::escreverOrigem(const QString & yaml, double x, double y, QString * erro)
{
  QFile file(yaml);
  if (!file.open(QFile::ReadOnly | QFile::Text)) {
    if (erro) {
      *erro = QFileInfo(yaml).fileName() + " nao encontrado";
    }
    return false;
  }
  QStringList lines = QString::fromUtf8(file.readAll()).split('\n');
  file.close();

  bool achou = false;
  for (QString & line : lines) {
    if (!line.trimmed().startsWith("origin:")) {
      continue;
    }
    // Preserva o yaw atual (3o elemento), se existir.
    double yaw = 0.0;
    const QString inner = line.section('[', 1).section(']', 0, 0);
    const QStringList parts = inner.split(',');
    if (parts.size() >= 3) {
      yaw = parts[2].trimmed().toDouble();
    }
    line = QString("origin: [%1, %2, %3]")
      .arg(x, 0, 'f', 3)
      .arg(y, 0, 'f', 3)
      .arg(yaw);
    achou = true;
  }
  if (!achou) {
    if (erro) {
      *erro = QFileInfo(yaml).fileName() + " nao tem a linha da origem";
    }
    return false;
  }

  QFile::copy(yaml, yaml + ".bak");
  QFile out(yaml);
  if (!out.open(QFile::WriteOnly | QFile::Truncate | QFile::Text)) {
    if (erro) {
      *erro = "nao consegui escrever " + QFileInfo(yaml).fileName();
    }
    return false;
  }
  QTextStream ts(&out);
  ts << lines.join('\n');
  return true;
}

void EditorMapaModule::applyOrigin()
{
  const double x = origem_x_->value();
  const double y = origem_y_->value();

  QString erro;
  if (!escreverOrigem(mapDir() + "/map.yaml", x, y, &erro)) {
    status_->setText("Nao consegui mudar o canto do mapa: " + erro + ".");
    emit status(status_->text());
    return;
  }

  // A mascara de paredes virtuais tem origem PROPRIA, e as duas precisam
  // combinar. Bug corrigido: antes so' o map.yaml era alterado, e o resultado
  // era uma arena em que todas as paredes virtuais apareciam deslocadas do
  // lugar onde tinham sido pintadas -- o robo desviava de nada e entrava onde
  // nao podia. Como o desenho e' o mesmo, a origem tem que ser a mesma.
  const QString mascara = mapDir() + "/keepout_mask.yaml";
  QString aviso;
  if (QFileInfo::exists(mascara) && !escreverOrigem(mascara, x, y, &aviso)) {
    status_->setText(
      "O mapa foi movido, mas as paredes virtuais NAO acompanharam (" + aviso +
      "). Confira o passo de paredes virtuais antes de usar esta arena.");
    emit status(status_->text());
    return;
  }

  status_->setText(
    "Canto do mapa alterado (mapa e paredes virtuais juntos). "
    "A versao anterior ficou guardada ao lado.");
  emit status(status_->text());
}
