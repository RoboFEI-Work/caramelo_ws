#pragma once

// Tipo simples de saude de um componente, transportado do RosBridge (thread ROS)
// para os widgets (thread da UI) via sinais/slots Qt. Precisa de metatype para
// atravessar threads (conexao enfileirada).

#include <QMap>
#include <QMetaType>
#include <QString>
#include <QVector>

struct ComponentHealth
{
  QString name;         // ex.: "caramelo/lidar"
  int level = 3;        // 0=OK, 1=WARN, 2=ERROR, 3=STALE (mesmo esquema do DiagnosticStatus)
  QString message;      // mensagem curta (modo operador)
  QMap<QString, QString> values;  // pares (causa, acao, frequencia, ...) para o modo engenharia
};

Q_DECLARE_METATYPE(ComponentHealth)
Q_DECLARE_METATYPE(QVector<ComponentHealth>)
