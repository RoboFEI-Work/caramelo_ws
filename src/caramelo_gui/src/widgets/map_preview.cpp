#include "widgets/map_preview.hpp"

#include <cmath>

#include <QDir>
#include <QFileInfo>
#include <QList>
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
const char * kEdicao = "Ponto sendo editado";

// Tipos de marcador usados nos sinais. Sao strings e nao enum porque atravessam
// o limite do widget e caem em telas que gravam arquivos diferentes.
const char * kTipoDock = "dock";
const char * kTipoArea = "area";
const char * kTipoWaypoint = "waypoint";

// Raio de tolerancia do toque, em pixels. Dedo em tela de robo nao acerta 7 px.
const double kToleranciaCorpo = 12.0;
const double kToleranciaSeta = 10.0;
const double kComprimentoSeta = 22.0;
// Abaixo disso o arrasto e' tremor de mao, nao intencao de mover.
const double kLimiarArrasto = 3.0;

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

double distancia(const QPointF & a, const QPointF & b)
{
  const double dx = a.x() - b.x();
  const double dy = a.y() - b.y();
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

QMap<QString, PoseMapa> lerDocksDaArena(const QString & pastaDoMapa, QString * erro)
{
  QMap<QString, PoseMapa> docks;
  if (erro) {
    erro->clear();
  }
  const QString caminho = pastaDoMapa + "/docking.yaml";
  if (!QFileInfo::exists(caminho)) {
    if (erro) {
      *erro = "Esta arena nao tem docking.yaml — nao da para dockar nem rodar missao.";
    }
    return docks;
  }
  try {
    const YAML::Node raiz = YAML::LoadFile(caminho.toStdString());
    for (const auto & kv : raiz["docks"]) {
      docks.insert(
        QString::fromStdString(kv.first.as<std::string>()),
        lerPoseLista(kv.second["pose"]));
    }
  } catch (const std::exception & ex) {
    if (erro) {
      *erro = QString("docking.yaml ilegivel (%1).").arg(ex.what());
    }
  }
  return docks;
}

QMap<QString, AreaDeServico> lerAreasDaArena(const QString & pastaDoMapa, QString * erro)
{
  QMap<QString, AreaDeServico> areas;
  if (erro) {
    erro->clear();
  }
  const QString caminho = pastaDoMapa + "/service_areas.yaml";
  if (!QFileInfo::exists(caminho)) {
    return areas;   // arena sem service areas e' situacao normal (ainda em mapeamento)
  }
  try {
    const YAML::Node raiz = YAML::LoadFile(caminho.toStdString());
    for (const auto & kv : raiz["service_areas"]) {
      AreaDeServico a;
      a.pose = lerPoseMapa(kv.second["final_robot_pose"]);
      a.tipo = QString::fromStdString(kv.second["type"].as<std::string>(""));
      a.manipulacao = kv.second["manipulation_enabled"].as<bool>(true);
      areas.insert(QString::fromStdString(kv.first.as<std::string>()), a);
    }
  } catch (const std::exception & ex) {
    if (erro) {
      *erro = QString("service_areas.yaml ilegivel (%1).").arg(ex.what());
    }
  }
  return areas;
}

QMap<QString, PoseMapa> lerWaypointsDaArena(const QString & pastaDoMapa, QString * erro)
{
  QMap<QString, PoseMapa> waypoints;
  if (erro) {
    erro->clear();
  }
  const QString caminho = pastaDoMapa + "/waypoints.yaml";
  if (!QFileInfo::exists(caminho)) {
    return waypoints;
  }
  try {
    const YAML::Node raiz = YAML::LoadFile(caminho.toStdString());
    for (const auto & kv : raiz["waypoints"]) {
      waypoints.insert(
        QString::fromStdString(kv.first.as<std::string>()), lerPoseMapa(kv.second));
    }
  } catch (const std::exception & ex) {
    if (erro) {
      *erro = QString("waypoints.yaml ilegivel (%1).").arg(ex.what());
    }
  }
  return waypoints;
}

QStringList lerTiposDeDockDaArena(const QString & pastaDoMapa)
{
  // Fallback: os plugins que o robo sempre tem. Uma arena recem-criada pode
  // ainda nao ter docking.yaml, e deixar a tela sem opcao nenhuma seria pior
  // que oferecer os tres padroes.
  const QStringList padrao = {
    "caramelo_front_dock", "caramelo_shelf_front_dock", "caramelo_precision_front_dock"};

  const QString caminho = pastaDoMapa + "/docking.yaml";
  if (!QFileInfo::exists(caminho)) {
    return padrao;
  }
  QStringList tipos;
  try {
    const YAML::Node raiz = YAML::LoadFile(caminho.toStdString());
    for (const auto & kv : raiz["dock_plugins"]) {
      tipos << QString::fromStdString(kv.first.as<std::string>());
    }
  } catch (const std::exception &) {
    return padrao;
  }
  return tipos.isEmpty() ? padrao : tipos;
}

MapPreview::MapPreview(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(360, 300);
  // Tracking ligado: no modo de edicao o cursor precisa mudar ao passar por
  // cima de um ponto, senao nao ha' como o operador descobrir que da' para
  // pegar o marcador.
  setMouseTracking(true);
  for (const QString & c : camadasDisponiveis()) {
    camadas_.insert(c, true);
  }
}

QStringList MapPreview::camadasDisponiveis()
{
  return {kGrade, kKeepout, kDocks, kAreas, kWaypoints, kRotulos, kEdicao};
}

QString MapPreview::camadaDeEdicao()
{
  return kEdicao;
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
  limparEdicao();

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

  // --- docks / service areas / waypoints ---
  // O parser vive nas funcoes livres acima porque os seletores de tela precisam
  // da mesma lista; aqui so' sobra a contagem do que esta zerado.
  QString erroDocks;
  mapa_.docks = lerDocksDaArena(pastaDoMapa, &erroDocks);
  if (!erroDocks.isEmpty()) {
    mapa_.avisos << erroDocks;
  }
  int docksPlaceholder = 0;
  for (auto it = mapa_.docks.constBegin(); it != mapa_.docks.constEnd(); ++it) {
    if (it.value().placeholder) {
      ++docksPlaceholder;
    }
  }

  QString erroAreas;
  mapa_.areas = lerAreasDaArena(pastaDoMapa, &erroAreas);
  if (!erroAreas.isEmpty()) {
    mapa_.avisos << erroAreas;
  }
  int areasPlaceholder = 0;
  for (auto it = mapa_.areas.constBegin(); it != mapa_.areas.constEnd(); ++it) {
    if (it.value().pose.placeholder) {
      ++areasPlaceholder;
    }
  }

  QString erroWaypoints;
  mapa_.waypoints = lerWaypointsDaArena(pastaDoMapa, &erroWaypoints);
  if (!erroWaypoints.isEmpty()) {
    mapa_.avisos << erroWaypoints;
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

QPointF MapPreview::paraMapa(const QPointF & tela) const
{
  // Inversa exata de paraTela: mesma inversao de Y, mesma origem no pixel
  // inferior esquerdo. Sem ela nao existe clique-no-mapa, so' desenho.
  if (mapa_.ocupacao.isNull() || escala_ <= 0.0) {
    return QPointF();
  }
  const double u = (tela.x() - deslocamento_.x()) / escala_;
  const double v = (tela.y() - deslocamento_.y()) / escala_;
  return QPointF(
    u * mapa_.resolucao + mapa_.origem_x,
    (mapa_.ocupacao.height() - v) * mapa_.resolucao + mapa_.origem_y);
}

double MapPreview::yawDoArrasto(const QPointF & origem, const QPointF & destino)
{
  // O eixo Y da tela aponta para baixo e o do mapa para cima: sem o sinal
  // trocado aqui, arrastar para cima gravava o robo virado para baixo.
  return std::atan2(-(destino.y() - origem.y()), destino.x() - origem.x());
}

void MapPreview::setModo(Modo modo)
{
  if (modo_ == modo) {
    return;
  }
  modo_ = modo;
  gesto_ = Gesto::Nenhum;
  arrastando_ = false;
  if (modo_ == Modo::Navegar) {
    tem_pose_provisoria_ = false;
    unsetCursor();
  } else {
    setCursor(Qt::CrossCursor);
  }
  update();
}

void MapPreview::destacar(const QString & tipo, const QString & id)
{
  destaque_tipo_ = tipo;
  destaque_id_ = id;
  update();
}

void MapPreview::limparEdicao()
{
  destaque_tipo_.clear();
  destaque_id_.clear();
  alvo_tipo_.clear();
  alvo_id_.clear();
  gesto_ = Gesto::Nenhum;
  tem_pose_provisoria_ = false;
  update();
}

bool MapPreview::poseDoMarcador(
  const QString & tipo, const QString & id, PoseMapa * pose) const
{
  if (id.isEmpty()) {
    return false;
  }
  if (tipo == kTipoDock && mapa_.docks.contains(id)) {
    *pose = mapa_.docks.value(id);
    return true;
  }
  if (tipo == kTipoArea && mapa_.areas.contains(id)) {
    *pose = mapa_.areas.value(id).pose;
    return true;
  }
  if (tipo == kTipoWaypoint && mapa_.waypoints.contains(id)) {
    *pose = mapa_.waypoints.value(id);
    return true;
  }
  return false;
}

MarcadorNoMapa MapPreview::marcadorEm(const QPointF & tela) const
{
  bool naSeta = false;
  return buscarMarcador(tela, &naSeta);
}

MarcadorNoMapa MapPreview::buscarMarcador(const QPointF & tela, bool * naSeta) const
{
  MarcadorNoMapa achado;
  if (naSeta) {
    *naSeta = false;
  }
  if (!mapa_.valido) {
    return achado;
  }

  // Candidatos: so' o que esta visivel. Pegar um ponto de uma camada desligada
  // pareceria ao operador que o widget mexeu sozinho em algo que ele nao ve.
  // Na MESMA ordem em que sao pintados (area, dock, waypoint): assim o ultimo
  // da lista e' o que aparece por cima, e o desempate do toque segue o que o
  // operador esta vendo.
  QList<MarcadorNoMapa> candidatos;
  if (camadaLigada(kAreas)) {
    for (auto it = mapa_.areas.constBegin(); it != mapa_.areas.constEnd(); ++it) {
      candidatos << MarcadorNoMapa{true, kTipoArea, it.key(), it.value().pose};
    }
  }
  if (camadaLigada(kDocks)) {
    for (auto it = mapa_.docks.constBegin(); it != mapa_.docks.constEnd(); ++it) {
      candidatos << MarcadorNoMapa{true, kTipoDock, it.key(), it.value()};
    }
  }
  if (camadaLigada(kWaypoints)) {
    for (auto it = mapa_.waypoints.constBegin(); it != mapa_.waypoints.constEnd(); ++it) {
      candidatos << MarcadorNoMapa{true, kTipoWaypoint, it.key(), it.value()};
    }
  }

  // 1) corpo do marcador (mover). Vence a seta: o circulo e' o alvo obvio.
  // Empate: ganha o ultimo da lista, que e' o desenhado por cima. Acontece com
  // varios pontos ainda em [0,0,0], todos empilhados na origem do mapa -- o
  // operador tira um de la' por vez.
  double melhor = kToleranciaCorpo;
  for (const MarcadorNoMapa & c : candidatos) {
    const double d = distancia(tela, paraTela(c.pose.x, c.pose.y));
    if (d <= melhor) {
      melhor = d;
      achado = c;
    }
  }
  if (achado.valido) {
    return achado;
  }

  // 2) ponta da seta (girar) -- so' das setas que estao desenhadas.
  melhor = kToleranciaSeta;
  for (const MarcadorNoMapa & c : candidatos) {
    const bool destacado = (c.tipo == destaque_tipo_ && c.id == destaque_id_);
    const bool temSeta = destacado ||
      (!c.pose.placeholder && c.tipo != QString(kTipoArea));
    if (!temSeta) {
      continue;
    }
    const QPointF centro = paraTela(c.pose.x, c.pose.y);
    const QPointF ponta = centro + QPointF(
      std::cos(c.pose.yaw) * kComprimentoSeta, -std::sin(c.pose.yaw) * kComprimentoSeta);
    const double d = distancia(tela, ponta);
    if (d <= melhor) {
      melhor = d;
      achado = c;
      if (naSeta) {
        *naSeta = true;
      }
    }
  }
  return achado;
}

void MapPreview::atualizarCursor(const QPointF & tela)
{
  if (modo_ != Modo::PosicionarPose) {
    return;
  }
  bool naSeta = false;
  const MarcadorNoMapa sob = buscarMarcador(tela, &naSeta);
  if (!sob.valido) {
    setCursor(Qt::CrossCursor);          // clicar aqui cria uma pose nova
  } else if (naSeta) {
    setCursor(Qt::PointingHandCursor);   // arrastar a ponta gira o ponto
  } else {
    setCursor(Qt::SizeAllCursor);        // arrastar o circulo move o ponto
  }
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
  if (modo_ == Modo::Navegar) {
    if (e->button() == Qt::LeftButton) {
      arrastando_ = true;
      ultimo_arrasto_ = e->localPos();
      setCursor(Qt::ClosedHandCursor);
    }
    return;
  }

  // Modo edicao: o botao do meio continua sendo o pan, senao o operador fica
  // preso no enquadramento em que estava ao entrar na edicao.
  if (e->button() == Qt::MiddleButton) {
    arrastando_ = true;
    ultimo_arrasto_ = e->localPos();
    setCursor(Qt::ClosedHandCursor);
    return;
  }
  if (e->button() != Qt::LeftButton || !mapa_.valido) {
    return;
  }

  gesto_inicio_ = e->localPos();
  gesto_moveu_ = false;

  bool naSeta = false;
  const MarcadorNoMapa sob = buscarMarcador(gesto_inicio_, &naSeta);
  if (sob.valido) {
    alvo_tipo_ = sob.tipo;
    alvo_id_ = sob.id;
    gesto_pose_ = sob.pose;
    gesto_ = naSeta ? Gesto::GirarMarcador : Gesto::MoverMarcador;
    destaque_tipo_ = sob.tipo;
    destaque_id_ = sob.id;
    tem_pose_provisoria_ = true;
    emit marcadorClicado(sob.tipo, sob.id);
  } else {
    // Gesto do "2D Goal": o press ja' define a posicao; o yaw so' existe depois
    // de arrastar, por isso comeca em zero.
    alvo_tipo_.clear();
    alvo_id_.clear();
    const QPointF metros = paraMapa(gesto_inicio_);
    gesto_pose_ = PoseMapa();
    gesto_pose_.x = metros.x();
    gesto_pose_.y = metros.y();
    gesto_ = Gesto::NovaPose;
    tem_pose_provisoria_ = true;
  }
  update();
}

void MapPreview::mouseMoveEvent(QMouseEvent * e)
{
  if (arrastando_) {
    deslocamento_ += e->localPos() - ultimo_arrasto_;
    ultimo_arrasto_ = e->localPos();
    update();
    return;
  }

  if (gesto_ == Gesto::Nenhum) {
    atualizarCursor(e->localPos());
    return;
  }

  if (distancia(e->localPos(), gesto_inicio_) > kLimiarArrasto) {
    gesto_moveu_ = true;
  }

  switch (gesto_) {
    case Gesto::NovaPose:
      if (gesto_moveu_) {
        gesto_pose_.yaw = yawDoArrasto(gesto_inicio_, e->localPos());
      }
      break;
    case Gesto::MoverMarcador: {
        const QPointF metros = paraMapa(e->localPos());
        gesto_pose_.x = metros.x();
        gesto_pose_.y = metros.y();
        break;
      }
    case Gesto::GirarMarcador:
      gesto_pose_.yaw = yawDoArrasto(paraTela(gesto_pose_.x, gesto_pose_.y), e->localPos());
      break;
    case Gesto::Nenhum:
      break;
  }
  update();
}

void MapPreview::mouseReleaseEvent(QMouseEvent * e)
{
  if (modo_ == Modo::Navegar) {
    if (e->button() == Qt::LeftButton) {
      arrastando_ = false;
      unsetCursor();
    }
    return;
  }

  if (e->button() == Qt::MiddleButton) {
    arrastando_ = false;
    atualizarCursor(e->localPos());
    return;
  }
  if (e->button() != Qt::LeftButton || gesto_ == Gesto::Nenhum) {
    return;
  }

  const Gesto encerrado = gesto_;
  gesto_ = Gesto::Nenhum;

  if (encerrado == Gesto::NovaPose) {
    emit poseEscolhida(gesto_pose_.x, gesto_pose_.y, gesto_pose_.yaw);
  } else if (gesto_moveu_) {
    // Clique curto num ponto ja' salvo e' selecao (marcadorClicado, no press);
    // so' vira alteracao quando o operador de fato arrastou.
    emit marcadorArrastado(
      alvo_tipo_, alvo_id_, gesto_pose_.x, gesto_pose_.y, gesto_pose_.yaw);
  }
  atualizarCursor(e->localPos());
  update();
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
  // Sempre por cima: o ponto em edicao nao pode ficar escondido atras de um
  // vizinho justamente na hora em que o operador esta mexendo nele.
  if (camadaLigada(kEdicao)) {
    desenharEdicao(p);
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
    const double comprimento = kComprimentoSeta;
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

void MapPreview::desenharSeta(
  QPainter & p, const QPointF & centro, double yaw, double comprimento,
  const QColor & cor) const
{
  // Mesma convencao de desenharMarcador: seno negativo por causa do Y da tela.
  const QPointF direcao(std::cos(yaw), -std::sin(yaw));
  const QPointF ponta = centro + direcao * comprimento;
  p.setPen(QPen(cor, 3));
  p.setBrush(Qt::NoBrush);
  p.drawLine(centro, ponta);

  const QPointF lado(-direcao.y(), direcao.x());
  QPainterPath cabeca;
  cabeca.moveTo(ponta);
  cabeca.lineTo(ponta - direcao * 9.0 + lado * 5.0);
  cabeca.lineTo(ponta - direcao * 9.0 - lado * 5.0);
  cabeca.closeSubpath();
  p.setPen(Qt::NoPen);
  p.setBrush(cor);
  p.drawPath(cabeca);
}

void MapPreview::desenharEdicao(QPainter & p) const
{
  const QColor cor("#f2c94c");

  // Halo do ponto destacado: sobrevive ao fim do gesto, para o operador nao
  // perder de vista qual ponto a tela esta editando.
  PoseMapa destacada;
  if (poseDoMarcador(destaque_tipo_, destaque_id_, &destacada)) {
    const QPointF centro = paraTela(destacada.x, destacada.y);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(QColor(242, 201, 76, 120), 2, Qt::DashLine));
    p.drawEllipse(centro, 16.0, 16.0);
    if (!tem_pose_provisoria_) {
      // A seta do ponto destacado e' a alca de giro: se ela nao aparecesse, o
      // operador estaria arrastando um ponto invisivel da tela.
      desenharSeta(p, centro, destacada.yaw, kComprimentoSeta, QColor(242, 201, 76, 200));
    }
  }

  if (!tem_pose_provisoria_) {
    return;
  }

  const QPointF centro = paraTela(gesto_pose_.x, gesto_pose_.y);
  p.setPen(QPen(QColor("#06121f"), 2));
  p.setBrush(cor);
  p.drawEllipse(centro, 8.0, 8.0);
  desenharSeta(p, centro, gesto_pose_.yaw, kComprimentoSeta + 6.0, cor);

  // Numeros do que esta sendo escolhido: o operador confere o valor antes de
  // soltar, em vez de salvar e conferir depois no YAML.
  const QString texto = QString("x %1 m   y %2 m   giro %3 graus")
    .arg(gesto_pose_.x, 0, 'f', 2)
    .arg(gesto_pose_.y, 0, 'f', 2)
    .arg(gesto_pose_.yaw * 180.0 / M_PI, 0, 'f', 0);
  QFont f = p.font();
  f.setPointSizeF(9.0);
  f.setBold(true);
  p.setFont(f);
  const QRectF caixa(centro.x() + 12, centro.y() + 12, 240, 16);
  const QRectF fundo = p.boundingRect(caixa, Qt::AlignLeft | Qt::AlignVCenter, texto)
    .adjusted(-4, -2, 4, 2);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(6, 18, 31, 200));
  p.drawRoundedRect(fundo, 4, 4);
  p.setPen(cor.lighter(130));
  p.drawText(caixa, Qt::AlignLeft | Qt::AlignVCenter, texto);
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
