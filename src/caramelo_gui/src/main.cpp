#include <QApplication>
#include <QFile>
#include <QTextStream>

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

  const int ret = app.exec();

  rclcpp::shutdown();
  return ret;
}
