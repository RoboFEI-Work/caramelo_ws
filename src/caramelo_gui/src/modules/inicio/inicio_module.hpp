#pragma once

// Modulo Inicio: "bringup visual". Um cartao por subsistema (rede/Raspberry,
// controllers, LiDAR, IMU, odometria, TF, Nav2), com cor por estado. Modo
// Operador mostra so' a mensagem; modo Engenharia expande causa/acao/frequencia.
// Alimentado pelo /diagnostics (via RosBridge).

#include <QFrame>
#include <QMap>
#include <QVector>
#include <QWidget>

#include "bridge/health_types.hpp"

class QGridLayout;
class QCheckBox;
class QLabel;

class InicioModule : public QWidget
{
  Q_OBJECT

public:
  explicit InicioModule(QWidget * parent = nullptr);

public slots:
  void onDiagnostics(const QVector<ComponentHealth> & health);

private:
  struct Card
  {
    QFrame * frame = nullptr;
    QLabel * dot = nullptr;
    QLabel * title = nullptr;
    QLabel * msg = nullptr;
    QLabel * details = nullptr;
  };

  Card & ensureCard(const QString & name);
  void applyEngineerMode();

  QGridLayout * grid_ = nullptr;
  QCheckBox * engineer_ = nullptr;
  QMap<QString, Card> cards_;
  int next_pos_ = 0;
};
