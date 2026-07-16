#pragma once

// Gerencia um "ros2 launch ..." como processo filho da GUI (ligar/desligar
// teleop, mapeamento, simulacao). Encerramento educado: SIGINT no grupo do
// launch (para os nos filhos cairem juntos), depois SIGTERM/kill se demorar.

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>

class LaunchRunner : public QObject
{
  Q_OBJECT

public:
  explicit LaunchRunner(QObject * parent = nullptr);
  ~LaunchRunner() override;

  void start(const QString & pacote, const QString & launch, const QStringList & args = {});
  void stop();
  bool isRunning() const;

signals:
  void started();
  void stopped(int exit_code);
  void outputLine(const QString & line);

private:
  QProcess * process_ = nullptr;
};
