#include "core/launch_runner.hpp"

#include <csignal>

LaunchRunner::LaunchRunner(QObject * parent)
: QObject(parent)
{
  process_ = new QProcess(this);
  process_->setProcessChannelMode(QProcess::MergedChannels);

  connect(
    process_, &QProcess::readyReadStandardOutput, this, [this]() {
      while (process_->canReadLine()) {
        emit outputLine(QString::fromUtf8(process_->readLine()).trimmed());
      }
    });
  connect(process_, &QProcess::started, this, [this]() {emit started();});
  connect(
    process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
    [this](int code, QProcess::ExitStatus) {emit stopped(code);});
}

LaunchRunner::~LaunchRunner()
{
  stop();
  if (process_->state() != QProcess::NotRunning) {
    process_->waitForFinished(5000);
    process_->kill();
  }
}

void LaunchRunner::start(
  const QString & pacote, const QString & launch, const QStringList & args)
{
  if (isRunning()) {
    return;
  }
  QStringList full{"launch", pacote, launch};
  full += args;
  process_->start("ros2", full);
}

void LaunchRunner::stop()
{
  if (!isRunning()) {
    return;
  }
  // SIGINT derruba o ros2 launch e os nos filhos de forma limpa (como Ctrl+C).
  const qint64 pid = process_->processId();
  if (pid > 0) {
    ::kill(static_cast<pid_t>(pid), SIGINT);
  }
  if (!process_->waitForFinished(8000)) {
    process_->terminate();
    if (!process_->waitForFinished(4000)) {
      process_->kill();
    }
  }
}

bool LaunchRunner::isRunning() const
{
  return process_->state() == QProcess::Running;
}
