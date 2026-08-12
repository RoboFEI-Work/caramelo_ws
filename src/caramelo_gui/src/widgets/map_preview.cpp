#include "widgets/map_preview.hpp"

#include <cmath>

#include <QDir>
#include <QFileInfo>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <yaml-cpp/yaml.h>

namespace
{

const char * kGrade = "Grade (1 m)";
const char * kKeepout = "Paredes virtuais";
const char * kDocks = "Docks";
const char * kAreas = "Service areas";
const char * kWaypoints = "Waypoints";
const char * kRotulos = "Nomes";

// Uma pose que ninguem gravou ainda. O docking.yaml e o service_areas.yaml
// nascem assim, e o executor da missao recusa esses valores em modo real -- por
// isso vale destacar no preview em vez de deixar passar.
bool ehPlaceholder(double x, double y, double yaw)
{
  return std::abs(x) < 1e-6 && std::abs(y) < 1e-6 && std::abs(yaw) < 1e-6;
}

PoseMapa lerPoseLista(const YAML::Node & node)
{
  PoseMapa p;
  if (node && node.IsSequence() && node.size() >= 3) {
    p.x = node[0].as<double>(0.0);
    p.y = node[1].as<double>(0.0);
    p.yaw = node[2].as<double>(0.0);
  }
  p.placeholder = ehPlaceholder(p.x, p.y, p.yaw);
  return p;
}

PoseMapa lerPoseMapa(const YAML::Node & node)
{
  PoseMapa p;
  if (node && node.IsMap()) {
    p.x = node["x"].as<double>(0.0);
    p.y = node["y"].as<double>(0.0);
    p.yaw = node["yaw"].as<double>(0.0);
  }
  p.placeholder = ehPlaceholder(p.x, p.y, p.yaw);
  return p;
}

}  // namespace

MapPreview::MapPreview(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(360, 300);
  setMouseTracking(false);
  for (const QString & c : camadasDisponiveis()) {
    camadas_.insert(c, true);
  }
}

QStringList MapPreview::camadasDisponiveis()
{
  return {kGrade, kKeepout, kDocks, kAreas, kWaypoints, kRotulos};
}

void MapPreview::setCamada(const QString & camada, bool ligada)
{
  camadas_.insert(camada, ligada);
  update();
}

bool MapPreview::camadaLigada(const QString & camada) const
{
  return camadas_.value(camada, true);
}

void MapPreview::carregar(const QString & pastaDoMapa)
{
  mapa_ = MapaCarregado();
  mapa_.pasta = pastaDoMapa;
  mapa_.nome = QFileInfo(pastaDoMapa).fileName();

  const QString mapYaml = pastaDoMapa + "/map.yaml";
  if (!QFileInfo::exists(mapYaml)) {
    mapa_.erro = "Esta arena nao tem map.yaml.";
    emit carregado(mapa_);
    update();
    return;
  }

  try {
    const YAML::Node raiz = YAML::LoadFile(mapYaml.toStdString());
    mapa_.resolucao = raiz["resolution"].as<double>(0.05);
    if (raiz["origin"] && raiz["origin"].size() >= 2) {
      mapa_.origem_x = raiz["origin"][0].as<double>(0.0);
      mapa_.origem_y = raiz["origin"][1].as<double>(0.0);
    }
    const QString imagem = QString::fromStdString(raiz["image"].as<std::string>("map.pgm"));
    const QString caminhoImagem = QDir::isAbsolutePath(imagem) ?
      imagem : (pastaDoMapa + "/" + imagem);
    if (!mapa_.ocupacao.load(caminhoImagem)) {
      mapa_.erro = "Nao consegui abrir a imagem do mapa: " + imagem;
      emit carregado(mapa_);
      update();
      return;
    }
  } catch (const std::exception & ex) {
    mapa_.erro = QString("map.yaml invalido: ") + ex.what();
    emit carregado(mapa_);
    update();
    return;
  }

  // --- keepout (opcional) ---
  const QString keepoutPgm = pastaDoMapa + "/keepout_mask.pgm";
  if (QFileInfo::exists(keepoutPgm)) {
    mapa_.keepout.load(keepoutPgm);
  }

  // --- docks ---
  const QString dockingYaml = pastaDoMapa + "/docking.yaml";
  int docksPlaceholder = 0;
  if (QFileInfo::exists(dockingYaml)) {
    try {
      const YAML::Node raiz = YAML::LoadFile(dockingYaml.toStdString());
      for (const auto & kv : raiz["docks"]) {
        const QString id = QString::fromStdString(kv.first.as<std::string>());
        const PoseMapa pose = lerPoseLista(kv.second["pose"]);
        mapa_.docks.insert(id, pose);
        if (pose.placeholder) {
          ++docksPlaceholder;
        }
      }
    } catch (const std::exception & ex) {
      mapa_.avisos << QString("docking.yaml ilegivel (%1).").arg(ex.what());
    }
  } else {
    mapa_.avisos << "Esta arena nao tem docking.yaml — nao da para dockar nem rodar missao.";
  }

  // --- service areas ---
  const QString areasYaml = pastaDoMapa + "/service_areas.yaml";
  int areasPlaceholder = 0;
  if (QFileInfo::exists(areasYaml)) {
    try {
      const YAML::Node raiz = YAML::LoadFile(areasYaml.toStdString());
      for (const auto & kv : raiz["service_areas"]) {
        AreaDeServico a;
        a.pose = lerPoseMapa(kv.second["final_robot_pose"]);
        a.tipo = QString::fromStdString(kv.second["type"].as<std::string>(""));
        a.manipulacao = kv.second["manipulation_enabled"].as<bool>(true);
        mapa_.areas.insert(QString::fromStdString(kv.first.as<std::string>()), a);
        if (a.pose.placeholder) {
          ++areasPlaceholder;
        }
      }
    } catch (const std::exception & ex) {
      mapa_.avisos << QString("service_areas.yaml ilegivel (%1).").arg(ex.what());
    }
  }

  // --- waypoints ---
  const QString wpYaml = pastaDoMapa + "/waypoints.yaml";
  if (QFileInfo::exists(wpYaml)) {
    try {
      const YAML::Node raiz = YAML::LoadFile(wpYaml.toStdString());
      for (const auto & kv : raiz["waypoints"]) {
        mapa_.waypoints.insert(
          QString::fromStdString(kv.first.as<std::string>()), lerPoseMapa(kv.second));
      }
    } catch (const std::exception & ex) {
      mapa_.avisos << QString("waypoints.yaml ilegivel (%1).").arg(ex.what());
    }
  }

  // Avisos em linguagem de operador: dizem o que fazer, nao so' o que ha' de
  // errado.
  if (docksPlaceholder > 0) {
    mapa_.avisos << QString(
      "%1 dock(s) ainda sem posicao gravada (marcados em vermelho). "
      "Leve o robo ate' cada um e grave a pose em Mapeamento.").arg(docksPlaceholder);
  }
  if (areasPlaceholder > 0) {
    mapa_.avisos << QString(
      "%1 service area(s) sem posicao gravada.").arg(areasPlaceholder);
  }
  if (mapa_.keepout.isNull()) {
    mapa_.avisos << "Sem paredes virtuais nesta arena (fita de piso e' invisivel ao LiDAR).";
  }
  if (mapa_.waypoints.isEmpty()) {
    mapa_.avisos << "Nenhum waypoint salvo.";
  }
  if (!QFileInfo::exists(pastaDoMapa + "/ws_table_mapping.yaml")) {
    mapa_.avisos << "Sem ws_table_mapping.yaml — missoes com manipulacao nao rodam aqui.";
  }

  mapa_.valido = true;
  enquadrar();
  emit carregado(mapa_);
  update();
}

void MapPreview::enquadrar()
{
  if (mapa_.ocupacao.isNull()) {
    return;
  }
  const double sx = static_cast<double>(width() - 24) / mapa_.ocupacao.width();
  const double sy = static_cast<double>(height() - 24) / mapa_.ocupacao.height();
  escala_ = std::max(0.02, std::min(sx, sy));
  deslocamento_ = QPointF(
    (width() - mapa_.ocupacao.width() * escala_) / 2.0,
    (height() - mapa_.ocupacao.height() * escala_) / 2.0);
  update();
}

QPointF MapPreview::paraTela(double x, double y) const
{
  // map.yaml: origin e' a pose do pixel INFERIOR esquerdo da imagem, e a imagem
  // cresce para baixo. Errar este sinal espelha todos os marcadores.
  const double px = (x - mapa_.origem_x) / mapa_.resolucao;
  const double py = mapa_.ocupacao.height() - (y - mapa_.origem_y) / mapa_.resolucao;
  return deslocamento_ + QPointF(px * escala_, py * escala_);
}

void MapPreview::resizeEvent(QResizeEvent * e)
{
  QWidget::resizeEvent(e);
  enquadrar();
}

void MapPreview::wheelEvent(QWheelEvent * e)
{
  if (mapa_.ocupacao.isNull()) {
    return;
  }
  const double fator = e->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
  const QPointF cursor = e->position();
  // Zoom ancorado no cursor: sem isso o mapa foge da tela ao ampliar.
  deslocamento_ = cursor - (cursor - deslocamento_) * fator;
  escala_ = std::max(0.02, std::min(escala_ * fator, 40.0));
  update();
}

void MapPreview::mousePressEvent(QMouseEvent * e)
{
  if (e->button() == Qt::LeftButton) {
    arrastando_ = true;
    ultimo_arrasto_ = e->localPos();
    setCursor(Qt::ClosedHandCursor);
  }
}

void MapPreview::mouseMoveEvent(QMouseEvent * e)
{
  if (arrastando_) {
    deslocamento_ += e->localPos() - ultimo_arrasto_;
    ultimo_arrasto_ = e->localPos();
    update();
  }
}

void MapPreview::mouseReleaseEvent(QMouseEvent * e)
{
  if (e->button() == Qt::LeftButton) {
    arrastando_ = false;
    unsetCursor();
  }
}

void MapPreview::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.fillRect(rect(), QColor("#0b1626"));

  if (!mapa_.valido) {
    p.setPen(QColor("#7d9cc4"));
    p.drawText(
      rect(), Qt::AlignCenter,
      mapa_.erro.isEmpty() ? "Selecione uma arena" : mapa_.erro);
    return;
  }

  const QRectF destino(
    deslocamento_,
    QSizeF(mapa_.ocupacao.width() * escala_, mapa_.ocupacao.height() * escala_));
  p.setRenderHint(QPainter::SmoothPixmapTransform, escala_ < 4.0);
  p.drawImage(destino, mapa_.ocupacao);

  if (camadaLigada(kKeepout)) {
    desenharKeepout(p);
  }
  if (camadaLigada(kGrade)) {
    desenharGrade(p);
  }
  if (camadaLigada(kAreas)) {
    desenharAreas(p);
  }
  if (camadaLigada(kDocks)) {
    desenharDocks(p);
  }
  if (camadaLigada(kWaypoints)) {
    desenharWaypoints(p);
  }
}

void MapPreview::desenharKeepout(QPainter & p) const
{
  if (mapa_.keepout.isNull()) {
    return;
  }
  // Pinta de vermelho translucido so' onde o operador marcou (a mascara nasce
  // toda branca). Converter uma vez por paint e' aceitavel no tamanho de arena
  // que usamos.
  QImage overlay(mapa_.keepout.size(), QImage::Format_ARGB32_Premultiplied);
  overlay.fill(Qt::transparent);
  const QImage cinza = mapa_.keepout.convertToFormat(QImage::Format_Grayscale8);
  for (int y = 0; y < cinza.height(); ++y) {
    const uchar * linha = cinza.constScanLine(y);
    auto * saida = reinterpret_cast<QRgb *>(overlay.scanLine(y));
    for (int x = 0; x < cinza.width(); ++x) {
      if (linha[x] < 128) {
        saida[x] = qPremultiply(qRgba(235, 87, 87, 110));
      }
    }
  }
  const QRectF destino(
    deslocamento_,
    QSizeF(mapa_.ocupacao.width() * escala_, mapa_.ocupacao.height() * escala_));
  p.drawImage(destino, overlay);
}

void MapPreview::desenharGrade(QPainter & p) const
{
  const double passo = 1.0 / mapa_.resolucao * escala_;   // 1 metro em pixels
  if (passo < 12.0) {
    return;   // grade mais densa que isso vira ruido
  }
  p.setPen(QPen(QColor(255, 255, 255, 38), 1));
  const QRectF area(
    deslocamento_,
    QSizeF(mapa_.ocupacao.width() * escala_, mapa_.ocupacao.height() * escala_));
  for (double x = area.left(); x <= area.right(); x += passo) {
    p.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
  }
  for (double y = area.top(); y <= area.bottom(); y += passo) {
    p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
  }
}

void MapPreview::desenharMarcador(
  QPainter & p, const PoseMapa & pose, const QColor & cor,
  const QString & rotulo, bool comSeta) const
{
  // Pose nao gravada em vermelho: e' a diferenca entre uma arena pronta e uma
  // que vai abortar a missao no primeiro goto.
  const QColor usada = pose.placeholder ? QColor("#eb5757") : cor;
  const QPointF centro = paraTela(pose.x, pose.y);
  const double raio = 7.0;

  p.setPen(QPen(QColor("#06121f"), 2));
  p.setBrush(usada);
  p.drawEllipse(centro, raio, raio);

  if (comSeta && !pose.placeholder) {
    const double comprimento = 22.0;
    // O eixo Y da tela aponta para baixo: o seno entra negativo.
    const QPointF ponta = centro + QPointF(
      std::cos(pose.yaw) * comprimento, -std::sin(pose.yaw) * comprimento);
    p.setPen(QPen(usada, 3));
    p.drawLine(centro, ponta);
  }

  if (camadaLigada(kRotulos) && !rotulo.isEmpty()) {
    const QString texto = pose.placeholder ? (rotulo + " (sem pose)") : rotulo;
    QFont f = p.font();
    f.setPointSizeF(9.0);
    f.setBold(true);
    p.setFont(f);
    const QRectF caixa(centro.x() + 10, centro.y() - 22, 190, 16);
    p.setPen(QPen(QColor(6, 18, 31, 200)));
    p.setBrush(QColor(6, 18, 31, 170));
    const QRectF fundo = p.boundingRect(caixa, Qt::AlignLeft | Qt::AlignVCenter, texto)
      .adjusted(-4, -2, 4, 2);
    p.drawRoundedRect(fundo, 4, 4);
    p.setPen(usada.lighter(140));
    p.drawText(caixa, Qt::AlignLeft | Qt::AlignVCenter, texto);
  }
}

void MapPreview::desenharDocks(QPainter & p) const
{
  for (auto it = mapa_.docks.constBegin(); it != mapa_.docks.constEnd(); ++it) {
    // START e FINISH sao o comeco e o fim da prova: cor propria para o operador
    // achar o ponto de inicializacao de relance.
    const bool inicio = it.key().compare("START", Qt::CaseInsensitive) == 0;
    const bool fim = it.key().compare("FINISH", Qt::CaseInsensitive) == 0;
    QColor cor = QColor("#f2994a");
    if (inicio) {cor = QColor("#27ae60");}
    if (fim) {cor = QColor("#9b51e0");}
    const QString rotulo = inicio ? (it.key() + " (inicio)") : it.key();
    desenharMarcador(p, it.value(), cor, rotulo, true);
  }
}

void MapPreview::desenharAreas(QPainter & p) const
{
  for (auto it = mapa_.areas.constBegin(); it != mapa_.areas.constEnd(); ++it) {
    const QPointF centro = paraTela(it.value().pose.x, it.value().pose.y);
    const QColor cor = it.value().pose.placeholder ?
      QColor("#eb5757") : QColor("#56ccf2");
    p.setPen(QPen(cor, 2));
    p.setBrush(QColor(86, 204, 242, 45));
    p.drawRect(QRectF(centro.x() - 11, centro.y() - 11, 22, 22));
  }
}

void MapPreview::desenharWaypoints(QPainter & p) const
{
  for (auto it = mapa_.waypoints.constBegin(); it != mapa_.waypoints.constEnd(); ++it) {
    desenharMarcador(p, it.value(), QColor("#35c3f0"), it.key(), true);
  }
}
