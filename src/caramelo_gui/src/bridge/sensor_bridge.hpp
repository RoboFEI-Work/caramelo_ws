#pragma once

// Assinaturas dos sensores brutos, para o Modo Avancado.
//
// Regra que governa este arquivo: NADA e' assinado enquanto ninguem esta
// olhando. Uma camera a 30 Hz decodificada e desenhada o tempo todo custa CPU
// que o robo precisa para navegar, e num Wi-Fi de competicao custa banda que a
// navegacao precisa mais. Os visores assinam ao aparecer na tela e cancelam ao
// sair -- por isso os metodos vem em pares assinar/parar.
//
// Os dados atravessam para a thread da UI ja' convertidos em tipos Qt: os
// widgets nunca veem mensagem ROS, mesma regra do resto da GUI.

#include <memory>

#include <QImage>
#include <QMetaType>
#include <QObject>
#include <QString>
#include <QVector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

struct LeituraLidar
{
  float angulo_min = 0.0f;
  float angulo_max = 0.0f;
  float incremento = 0.0f;
  float alcance_min = 0.0f;
  float alcance_max = 0.0f;
  QVector<float> distancias;
  int validos = 0;
  double hz = 0.0;
};

struct LeituraImu
{
  // Em graus: quem opera o robo le' inclinacao em graus, nao em radianos.
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double giro_z = 0.0;      // rad/s, o eixo que importa numa base terrestre
  double acel_x = 0.0;
  double acel_y = 0.0;
  double acel_z = 0.0;
  double hz = 0.0;
};

struct QuadroCamera
{
  QImage imagem;
  double hz = 0.0;
  QString formato;
};

Q_DECLARE_METATYPE(LeituraLidar)
Q_DECLARE_METATYPE(LeituraImu)
Q_DECLARE_METATYPE(QuadroCamera)

class SensorBridge : public QObject
{
  Q_OBJECT

public:
  SensorBridge(rclcpp::Node::SharedPtr node, QObject * parent = nullptr);

  void assinarLidar(const QString & topico);
  void pararLidar();

  void assinarImu(const QString & topico);
  void pararImu();

  void assinarCamera(const QString & topico);
  void pararCamera();

  // Topicos de imagem visiveis no grafo agora. Serve para a tela oferecer o que
  // existe, em vez de exigir que alguem digite o nome certo de cor.
  QStringList topicosDeImagem() const;

  // Alguns sensores mudam de nome entre o robo e a simulacao (a IMU sai em
  // /imu no Gazebo e em /imu/data_raw no robo). Perguntar ao grafo evita
  // assinar um topico que nunca vai publicar.
  bool topicoExiste(const QString & topico) const;

signals:
  void lidarRecebido(const LeituraLidar & leitura);
  void imuRecebido(const LeituraImu & leitura);
  void cameraRecebida(const QuadroCamera & quadro);

private:
  // Frequencia por media movel simples. E' o numero que responde "o sensor esta
  // vivo?" melhor do que qualquer valor instantaneo.
  class Frequencimetro
  {
public:
    double registrar(const rclcpp::Time & agora);
    void zerar() {ultimo_ = 0.0; hz_ = 0.0;}

private:
    double ultimo_ = 0.0;
    double hz_ = 0.0;
  };

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;

  Frequencimetro hz_lidar_;
  Frequencimetro hz_imu_;
  Frequencimetro hz_camera_;
};
