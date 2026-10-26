#include "modules/avancado/sensores_module.hpp"

#include <cmath>

#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>

#include "bridge/ros_bridge.hpp"

namespace
{
constexpr double kRad2Deg = 180.0 / M_PI;

// O LiDAR esta montado de cabeca para baixo e virado: no URDF
// (base_tophat.xacro) rpy = (pi, 0, pi), que da a matriz diag(-1, +1, -1).
// No plano XY isso nao e rotacao, e ESPELHO -- x troca de sinal, y nao:
//
//     angulo_no_robo = pi - angulo_no_laser
//
// Este desenho somava o angulo do laser como se fosse angulo do robo. Como
// espelho nao e rotacao, o erro nao era um giro que ninguem notaria: o que
// esta na FRENTE aparecia ATRAS, e o setor cego era pintado do lado errado.
// Com o /scan recortado em [1.5708, 4.7124] rad no laser (scan_normalizer),
// no robo isso vira [-pi/2, +pi/2]: o Caramelo ENXERGA A FRENTE, e quem fica
// sem leitura e a TRASEIRA.
inline double anguloNoRobo(double angulo_no_laser)
{
  return M_PI - angulo_no_laser;
}

// Topicos do robo real. Na simulacao a IMU sai em /imu; no robo, em
// /imu/data_raw. O modulo tenta os dois.
const char * kTopicoLidar = "/scan";
const char * kTopicoImuSim = "/imu";
const char * kTopicoImuReal = "/imu/data_raw";
}  // namespace

// ============================================================ VisorLidar
VisorLidar::VisorLidar(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(280);
}

void VisorLidar::atualizar(const LeituraLidar & leitura)
{
  leitura_ = leitura;
  tem_dado_ = true;
  update();
}

void VisorLidar::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.fillRect(rect(), QColor("#081120"));

  const QPointF centro(width() / 2.0, height() / 2.0);
  const double raio = std::min(width(), height()) / 2.0 - 22.0;
  if (raio <= 10.0) {
    return;
  }

  if (!tem_dado_ || leitura_.distancias.isEmpty()) {
    p.setPen(QColor("#7d9cc4"));
    p.drawText(rect(), Qt::AlignCenter, "Sem dados do LiDAR");
    return;
  }

  const double alcance = std::max(1.0f, leitura_.alcance_max);

  // Aneis de distancia, com rotulo em metros: um desenho sem escala nao permite
  // julgar se o obstaculo esta a 30 cm ou a 3 m.
  p.setPen(QPen(QColor(255, 255, 255, 40), 1));
  for (int m = 1; m <= static_cast<int>(alcance); ++m) {
    const double r = raio * (m / alcance);
    p.drawEllipse(centro, r, r);
  }

  // Setor CEGO em destaque. O Caramelo recorta o LiDAR para os 180 graus da
  // FRENTE; desenhar um circulo completo daria a impressao de cobertura total,
  // que e' exatamente o engano que provoca colisao ao dar re'.
  const double abertura = leitura_.angulo_max - leitura_.angulo_min;
  if (abertura < 2.0 * M_PI - 0.05) {
    QPainterPath cego;
    cego.moveTo(centro);
    // Qt mede angulos em 1/16 de grau, no sentido anti-horario; o setor cego e'
    // o complemento do coberto.
    // O espelho INVERTE os extremos: o angulo_max do laser e' o MENOR
    // angulo no robo. E' esta troca que poe o setor cego atras.
    const double inicioCego = anguloNoRobo(leitura_.angulo_min) * kRad2Deg;
    const double extensaoCego = (2.0 * M_PI - abertura) * kRad2Deg;
    cego.arcTo(
      QRectF(centro.x() - raio, centro.y() - raio, raio * 2, raio * 2),
      inicioCego, extensaoCego);
    cego.closeSubpath();
    p.fillPath(cego, QColor(235, 87, 87, 45));

    p.setPen(QColor("#ff9d9d"));
    // O rotulo fica embaixo, junto do setor: em cima ele apontava para o lado
    // coberto e dizia o contrario do que o desenho mostra.
    p.drawText(
      QRectF(centro.x() - 90, centro.y() + raio + 4, 180, 18),
      Qt::AlignCenter, "traseira sem visao");
  }

  // Pontos do scan.
  p.setPen(Qt::NoPen);
  p.setBrush(QColor("#35c3f0"));
  for (int i = 0; i < leitura_.distancias.size(); ++i) {
    const float d = leitura_.distancias[i];
    if (!std::isfinite(d) || d < leitura_.alcance_min || d > leitura_.alcance_max) {
      continue;
    }
    const double ang = anguloNoRobo(leitura_.angulo_min + i * leitura_.incremento);
    const double r = raio * (d / alcance);
    // Y da tela cresce para baixo: o seno entra negativo.
    p.drawEllipse(
      QPointF(centro.x() + std::cos(ang) * r, centro.y() - std::sin(ang) * r), 1.6, 1.6);
  }

  // Robo no centro, apontando para +X (frente).
  p.setBrush(QColor("#f2994a"));
  QPainterPath robo;
  robo.moveTo(centro + QPointF(11, 0));
  robo.lineTo(centro + QPointF(-7, -7));
  robo.lineTo(centro + QPointF(-7, 7));
  robo.closeSubpath();
  p.fillPath(robo, QColor("#f2994a"));
}

// ============================================================ SensoresModule
SensoresModule::SensoresModule(RosBridge * bridge, QWidget * parent)
: QWidget(parent), bridge_(bridge), sensores_(bridge->sensores())
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(16, 14, 16, 14);
  layout->setSpacing(14);

  auto * titulo = new QLabel("Sensores ao vivo");
  titulo->setObjectName("tituloModulo");
  layout->addWidget(titulo);

  auto * explica = new QLabel(
    "Os cartoes de estado dizem se a mensagem chega. Aqui da para ver se o dado "
    "presta: um sensor pode publicar na frequencia certa e enxergar so' ruido.");
  explica->setWordWrap(true);
  explica->setObjectName("msgCartao");
  layout->addWidget(explica);

  auto * linha = new QHBoxLayout();
  linha->setSpacing(14);
  linha->addWidget(construirLidar(), 1);
  linha->addWidget(construirImu(), 0);
  layout->addLayout(linha, 1);

  layout->addWidget(construirCamera(), 1);

  connect(
    sensores_, &SensorBridge::lidarRecebido, this,
    [this](const LeituraLidar & l) {
      visor_lidar_->atualizar(l);
      const double abertura = (l.angulo_max - l.angulo_min) * kRad2Deg;
      resumo_lidar_->setText(
        QString("%1 Hz   •   %2 de %3 leituras validas   •   campo de visao %4 graus"
        "   •   alcance ate' %5 m")
        .arg(l.hz, 0, 'f', 1)
        .arg(l.validos).arg(l.distancias.size())
        .arg(abertura, 0, 'f', 0)
        .arg(l.alcance_max, 0, 'f', 1));
    });

  connect(
    sensores_, &SensorBridge::imuRecebido, this,
    [this](const LeituraImu & i) {
      resumo_imu_->setText(QString("%1 Hz").arg(i.hz, 0, 'f', 1));
      valores_imu_->setText(
        QString(
          "Inclinacao lateral: %1 graus\n"
          "Inclinacao frontal: %2 graus\n"
          "Direcao (yaw): %3 graus\n\n"
          "Giro em torno do eixo vertical: %4 rad/s\n"
          "Aceleracao: %5 / %6 / %7 m/s2")
        .arg(i.roll, 0, 'f', 1).arg(i.pitch, 0, 'f', 1).arg(i.yaw, 0, 'f', 1)
        .arg(i.giro_z, 0, 'f', 2)
        .arg(i.acel_x, 0, 'f', 2).arg(i.acel_y, 0, 'f', 2).arg(i.acel_z, 0, 'f', 2));
    });

  connect(
    sensores_, &SensorBridge::cameraRecebida, this,
    [this](const QuadroCamera & q) {
      if (q.imagem.isNull()) {
        resumo_camera_->setText(
          "Chegou imagem em formato que esta tela nao decodifica: " + q.formato);
        return;
      }
      visor_camera_->setPixmap(
        QPixmap::fromImage(q.imagem).scaled(
          visor_camera_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
      resumo_camera_->setText(
        QString("%1 Hz   •   %2 x %3   •   %4")
        .arg(q.hz, 0, 'f', 1).arg(q.imagem.width()).arg(q.imagem.height()).arg(q.formato));
    });
}

QWidget * SensoresModule::construirLidar()
{
  auto * grupo = new QGroupBox("LiDAR");
  auto * layout = new QVBoxLayout(grupo);
  visor_lidar_ = new VisorLidar();
  layout->addWidget(visor_lidar_, 1);
  resumo_lidar_ = new QLabel("Sem dados.");
  resumo_lidar_->setWordWrap(true);
  resumo_lidar_->setObjectName("msgCartao");
  layout->addWidget(resumo_lidar_);
  return grupo;
}

QWidget * SensoresModule::construirImu()
{
  auto * grupo = new QGroupBox("IMU");
  grupo->setFixedWidth(300);
  auto * layout = new QVBoxLayout(grupo);
  resumo_imu_ = new QLabel("Sem dados.");
  resumo_imu_->setObjectName("estadoAtual");
  layout->addWidget(resumo_imu_);
  valores_imu_ = new QLabel("—");
  valores_imu_->setWordWrap(true);
  layout->addWidget(valores_imu_);
  layout->addStretch();
  return grupo;
}

QWidget * SensoresModule::construirCamera()
{
  auto * grupo = new QGroupBox("Camera");
  auto * layout = new QVBoxLayout(grupo);

  auto * linhaTopico = new QHBoxLayout();
  linhaTopico->addWidget(new QLabel("Fonte:"));
  topico_camera_ = new QComboBox();
  topico_camera_->setEditable(true);
  linhaTopico->addWidget(topico_camera_, 1);
  layout->addLayout(linhaTopico);

  visor_camera_ = new QLabel();
  visor_camera_->setMinimumHeight(220);
  visor_camera_->setAlignment(Qt::AlignCenter);
  visor_camera_->setStyleSheet("background:#050d18; border-radius:10px;");
  visor_camera_->setText("Sem imagem");
  layout->addWidget(visor_camera_, 1);

  resumo_camera_ = new QLabel();
  resumo_camera_->setWordWrap(true);
  resumo_camera_->setObjectName("msgCartao");
  layout->addWidget(resumo_camera_);

  connect(
    topico_camera_, &QComboBox::currentTextChanged, this,
    [this](const QString &) {reassinarCamera();});

  return grupo;
}

void SensoresModule::reassinarCamera()
{
  sensores_->pararCamera();
  const QString topico = topico_camera_->currentText().trimmed();
  if (topico.isEmpty()) {
    return;
  }
  visor_camera_->setText("Aguardando imagem...");
  sensores_->assinarCamera(topico);
}

void SensoresModule::showEvent(QShowEvent * e)
{
  QWidget::showEvent(e);

  sensores_->assinarLidar(kTopicoLidar);

  // A IMU muda de nome entre o robo e a simulacao. Assinar as duas nao funciona
  // (a segunda substituiria a primeira): pergunta-se ao grafo qual existe.
  const QString topicoImu = sensores_->topicoExiste(kTopicoImuReal) ?
    kTopicoImuReal : kTopicoImuSim;
  sensores_->assinarImu(topicoImu);
  resumo_imu_->setText("Aguardando " + topicoImu);

  const QString atual = topico_camera_->currentText();
  topico_camera_->blockSignals(true);
  topico_camera_->clear();
  const QStringList imagens = sensores_->topicosDeImagem();
  topico_camera_->addItems(imagens);
  if (!atual.isEmpty() && imagens.contains(atual)) {
    topico_camera_->setCurrentText(atual);
  }
  topico_camera_->blockSignals(false);

  if (imagens.isEmpty()) {
    visor_camera_->setText("Nenhuma camera publicando");
    resumo_camera_->setText(
      "Na simulacao isso e' esperado: o modelo tem a RealSense so' como "
      "geometria, sem sensor de camera do Gazebo. No robo real, a camera "
      "aparece quando o stack de manipulacao sobe.");
  } else {
    reassinarCamera();
  }
}

void SensoresModule::hideEvent(QHideEvent * e)
{
  QWidget::hideEvent(e);
  // Sair da tela cancela tudo: a camera a 30 Hz nao pode continuar consumindo
  // CPU e banda enquanto ninguem olha.
  sensores_->pararLidar();
  sensores_->pararImu();
  sensores_->pararCamera();
}
