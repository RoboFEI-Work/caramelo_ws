#include "core/icones.hpp"

#include <QColor>
#include <QPainter>
#include <QPainterPath>

namespace icones
{
namespace
{

QColor corDe(Tipo tipo)
{
  switch (tipo) {
    case Tipo::Operacao: return QColor("#2f80ed");
    case Tipo::Competicao: return QColor("#27ae60");
    case Tipo::Mapas: return QColor("#f2994a");
    case Tipo::Mapeamento: return QColor("#9b51e0");
    case Tipo::Avancado: return QColor("#00b3a4");
    case Tipo::Parar: return QColor("#eb5757");
    case Tipo::Base: return QColor("#56ccf2");
    case Tipo::Config: return QColor("#7f93b0");
  }
  return QColor("#35c3f0");
}

// Seta de navegacao (aviaozinho de papel).
void glifoOperacao(QPainter & p, qreal s)
{
  QPainterPath path;
  path.moveTo(0.50 * s, 0.16 * s);
  path.lineTo(0.82 * s, 0.84 * s);
  path.lineTo(0.50 * s, 0.66 * s);
  path.lineTo(0.18 * s, 0.84 * s);
  path.closeSubpath();
  p.fillPath(path, Qt::white);
}

// Bandeira quadriculada.
void glifoCompeticao(QPainter & p, qreal s)
{
  p.fillRect(QRectF(0.20 * s, 0.14 * s, 0.055 * s, 0.72 * s), Qt::white);
  const qreal x0 = 0.28 * s, y0 = 0.18 * s, c = 0.115 * s;
  for (int l = 0; l < 3; ++l) {
    for (int col = 0; col < 4; ++col) {
      if ((l + col) % 2 == 0) {
        p.fillRect(QRectF(x0 + col * c, y0 + l * c, c, c), Qt::white);
      }
    }
  }
}

// Mapa dobrado com um pino.
void glifoMapas(QPainter & p, qreal s)
{
  QPainterPath mapa;
  mapa.moveTo(0.14 * s, 0.30 * s);
  mapa.lineTo(0.38 * s, 0.20 * s);
  mapa.lineTo(0.62 * s, 0.30 * s);
  mapa.lineTo(0.86 * s, 0.20 * s);
  mapa.lineTo(0.86 * s, 0.76 * s);
  mapa.lineTo(0.62 * s, 0.86 * s);
  mapa.lineTo(0.38 * s, 0.76 * s);
  mapa.lineTo(0.14 * s, 0.86 * s);
  mapa.closeSubpath();
  p.fillPath(mapa, QColor(255, 255, 255, 235));

  QPen dobra(QColor(0, 0, 0, 70), 0.035 * s);
  p.setPen(dobra);
  p.drawLine(QPointF(0.38 * s, 0.20 * s), QPointF(0.38 * s, 0.76 * s));
  p.drawLine(QPointF(0.62 * s, 0.30 * s), QPointF(0.62 * s, 0.86 * s));
}

// Lapis inclinado (criar/editar).
void glifoMapeamento(QPainter & p, qreal s)
{
  p.save();
  p.translate(0.50 * s, 0.50 * s);
  p.rotate(-45.0);
  const qreal larg = 0.20 * s;
  p.fillRect(QRectF(-larg / 2, -0.34 * s, larg, 0.50 * s), Qt::white);
  QPainterPath ponta;
  ponta.moveTo(-larg / 2, 0.16 * s);
  ponta.lineTo(larg / 2, 0.16 * s);
  ponta.lineTo(0.0, 0.36 * s);
  ponta.closeSubpath();
  p.fillPath(ponta, Qt::white);
  p.restore();
}

// Engrenagem.
void glifoEngrenagem(QPainter & p, qreal s, qreal escala)
{
  p.save();
  p.translate(0.5 * s, 0.5 * s);
  const qreal raio = 0.26 * s * escala;
  const qreal dente = 0.10 * s * escala;
  QPainterPath eng;
  eng.addEllipse(QPointF(0, 0), raio, raio);
  for (int i = 0; i < 8; ++i) {
    QPainterPath d;
    d.addRoundedRect(
      QRectF(-dente / 2, -(raio + dente * 0.9), dente, dente * 1.6),
      dente * 0.25, dente * 0.25);
    QTransform t;
    t.rotate(i * 45.0);
    eng.addPath(t.map(d));
  }
  QPainterPath furo;
  furo.addEllipse(QPointF(0, 0), raio * 0.42, raio * 0.42);
  p.fillPath(eng.subtracted(furo), Qt::white);
  p.restore();
}

// Quadrado de parada.
void glifoParar(QPainter & p, qreal s)
{
  QPainterPath q;
  q.addRoundedRect(QRectF(0.28 * s, 0.28 * s, 0.44 * s, 0.44 * s), 0.06 * s, 0.06 * s);
  p.fillPath(q, Qt::white);
}

// Seta de retorno (meia-volta).
void glifoBase(QPainter & p, qreal s)
{
  QPen caneta(Qt::white, 0.10 * s, Qt::SolidLine, Qt::RoundCap);
  p.setPen(caneta);
  p.setBrush(Qt::NoBrush);
  const QRectF arco(0.22 * s, 0.26 * s, 0.56 * s, 0.44 * s);
  p.drawArc(arco, 20 * 16, 290 * 16);
  QPainterPath seta;
  seta.moveTo(0.30 * s, 0.30 * s);
  seta.lineTo(0.30 * s, 0.56 * s);
  seta.lineTo(0.54 * s, 0.42 * s);
  seta.closeSubpath();
  p.fillPath(seta, Qt::white);
}

}  // namespace

QPixmap desenhar(Tipo tipo, int lado)
{
  QPixmap pm(lado, lado);
  pm.fill(Qt::transparent);

  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing, true);

  const qreal s = lado;
  QPainterPath fundo;
  fundo.addRoundedRect(QRectF(0, 0, s, s), s * 0.24, s * 0.24);
  QColor cor = corDe(tipo);
  QLinearGradient grad(0, 0, 0, s);
  grad.setColorAt(0.0, cor.lighter(118));
  grad.setColorAt(1.0, cor);
  p.fillPath(fundo, grad);

  switch (tipo) {
    case Tipo::Operacao: glifoOperacao(p, s); break;
    case Tipo::Competicao: glifoCompeticao(p, s); break;
    case Tipo::Mapas: glifoMapas(p, s); break;
    case Tipo::Mapeamento: glifoMapeamento(p, s); break;
    case Tipo::Avancado: glifoEngrenagem(p, s, 1.0); break;
    case Tipo::Config: glifoEngrenagem(p, s, 0.9); break;
    case Tipo::Parar: glifoParar(p, s); break;
    case Tipo::Base: glifoBase(p, s); break;
  }
  return pm;
}

}  // namespace icones
