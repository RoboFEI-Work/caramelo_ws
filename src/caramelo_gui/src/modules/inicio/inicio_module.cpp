#include "modules/inicio/inicio_module.hpp"

#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <QVBoxLayout>

namespace
{
// Nome amigavel exibido no cartao a partir do nome tecnico do DiagnosticStatus.
QString friendlyName(const QString & name)
{
  static const QMap<QString, QString> nomes = {
    {"caramelo/rede_raspberry", "Raspberry / Rede"},
    {"caramelo/controllers", "Controllers"},
    {"caramelo/lidar", "LiDAR"},
    {"caramelo/imu", "IMU"},
    {"caramelo/odometria", "Odometria"},
    {"caramelo/tf", "TF"},
    {"caramelo/nav2", "Nav2"},
    {"caramelo/bateria", "Bateria"},
  };
  return nomes.value(name, name);
}

// Cor do "farol" por nivel (0=OK,1=WARN,2=ERROR,3=STALE).
QString dotColor(int level)
{
  switch (level) {
    case 0: return "#2e7d32";   // verde
    case 1: return "#f9a825";   // ambar
    case 2: return "#c62828";   // vermelho
    default: return "#757575";  // cinza (sem dado)
  }
}
}  // namespace

InicioModule::InicioModule(QWidget * parent)
: QWidget(parent)
{
  auto * outer = new QVBoxLayout(this);

  auto * header = new QHBoxLayout();
  auto * title = new QLabel("Estado do robo");
  title->setObjectName("tituloModulo");
  header->addWidget(title);
  header->addStretch();
  engineer_ = new QCheckBox("Modo Engenharia");
  connect(engineer_, &QCheckBox::toggled, this, [this]() {applyEngineerMode();});
  header->addWidget(engineer_);
  outer->addLayout(header);

  auto * scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto * content = new QWidget();
  grid_ = new QGridLayout(content);
  grid_->setSpacing(16);
  scroll->setWidget(content);
  outer->addWidget(scroll);
}

InicioModule::Card & InicioModule::ensureCard(const QString & name)
{
  auto it = cards_.find(name);
  if (it != cards_.end()) {
    return it.value();
  }

  Card card;
  card.frame = new QFrame();
  card.frame->setObjectName("cartaoStatus");
  card.frame->setMinimumSize(240, 120);

  auto * layout = new QVBoxLayout(card.frame);
  auto * topo = new QHBoxLayout();
  card.dot = new QLabel();
  card.dot->setFixedSize(16, 16);
  card.title = new QLabel(friendlyName(name));
  card.title->setObjectName("tituloCartao");
  topo->addWidget(card.dot);
  topo->addWidget(card.title);
  topo->addStretch();
  layout->addLayout(topo);

  card.msg = new QLabel("...");
  card.msg->setObjectName("msgCartao");
  card.msg->setWordWrap(true);
  layout->addWidget(card.msg);

  card.details = new QLabel();
  card.details->setObjectName("detalheCartao");
  card.details->setWordWrap(true);
  card.details->setVisible(false);
  layout->addWidget(card.details);
  layout->addStretch();

  const int row = next_pos_ / 3;
  const int col = next_pos_ % 3;
  ++next_pos_;
  grid_->addWidget(card.frame, row, col);

  cards_.insert(name, card);
  return cards_[name];
}

void InicioModule::onDiagnostics(const QVector<ComponentHealth> & health)
{
  for (const auto & c : health) {
    Card & card = ensureCard(c.name);
    card.dot->setStyleSheet(
      QString("background:%1; border-radius:8px;").arg(dotColor(c.level)));
    card.msg->setText(c.message.isEmpty() ? "sem mensagem" : c.message);

    QString det;
    for (auto it = c.values.constBegin(); it != c.values.constEnd(); ++it) {
      if (it.value().isEmpty()) {
        continue;
      }
      det += QString("%1: %2\n").arg(it.key(), it.value());
    }
    card.details->setText(det.trimmed());
  }
  applyEngineerMode();
}

void InicioModule::applyEngineerMode()
{
  const bool eng = engineer_->isChecked();
  for (auto & card : cards_) {
    card.details->setVisible(eng && !card.details->text().isEmpty());
  }
}
