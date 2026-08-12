#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "core/main_window.hpp"

// Carrega o tema PUDU (QSS) de share/caramelo_gui/resources/tema.qss.
static void aplicarTema(QApplication & app)
{
  QString path;
  try {
    path = QString::fromStdString(
      ament_index_cpp::get_package_share_directory("caramelo_gui") +
      "/resources/tema.qss");
  } catch (...) {
    return;
  }
  QFile file(path);
  if (file.open(QFile::ReadOnly | QFile::Text)) {
    QTextStream in(&file);
    app.setStyleSheet(in.readAll());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);
  app.setApplicationName("Caramelo GUI");
  aplicarTema(app);

  MainWindow window;
  window.show();

  // SIGINT/SIGTERM (Ctrl+C ou ros2 launch encerrando) derrubam o contexto ROS;
  // o Qt nao sabe disso sozinho e a janela ficaria viva ate o SIGKILL.
  //
  // Dois caminhos, de proposito. O on_shutdown reage na hora -- o rclcpp chama
  // esses callbacks numa thread propria, entao dar um quit enfileirado no Qt e'
  // seguro. O timer fica de rede: se o app estiver ocupado (RViz sob carga, por
  // exemplo) e perder o callback, ele ainda sai. Antes so' havia o timer de
  // 200 ms, e com a maquina saturada a GUI passava dos 5 s de tolerancia do
  // ros2 launch e era escalada para SIGTERM.
  rclcpp::on_shutdown(
    [&app]() {
      QMetaObject::invokeMethod(&app, "quit", Qt::QueuedConnection);
    });

  QTimer ros_watchdog;
  QObject::connect(
    &ros_watchdog, &QTimer::timeout, &app, [&app]() {
      if (!rclcpp::ok()) {
        app.quit();
      }
    });
  ros_watchdog.start(100);

  const int ret = app.exec();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return ret;
}
