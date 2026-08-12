#pragma once

// Modo Avancado > Sensores: ver o que o robo ve, para julgar se ele esta
// saudavel ANTES de mandar ele andar.
//
// Os cartoes de /diagnostics dizem "LiDAR OK, 10 Hz". Isso responde se a
// mensagem chega, nao se o dado presta: um LiDAR pode publicar na frequencia
// certa e enxergar so' ruido, e uma IMU pode responder com a orientacao travada.
// Aqui aparece o dado bruto.
//
// O visor do LiDAR desenha o CAMPO DE VISAO REAL, nao um circulo completo. O
// Caramelo recorta o LiDAR para os 180 graus traseiros -- a frente e' cega no
// costmap. Uma tela que desenhasse um circulo inteiro esconderia justamente o
// que mais causa colisao.
//
// Nada e' assinado enquanto esta tela nao esta aberta: camera a 30 Hz custa CPU
// que a navegacao precisa.

#include <QWidget>

#include "bridge/sensor_bridge.hpp"

class RosBridge;
class QComboBox;
class QLabel;

// Desenho polar do LiDAR, com o setor coberto e o setor cego marcados.
class VisorLidar : public QWidget
{
  Q_OBJECT

public:
  explicit VisorLidar(QWidget * parent = nullptr);
  void atualizar(const LeituraLidar & leitura);

protected:
  void paintEvent(QPaintEvent * e) override;

private:
  LeituraLidar leitura_;
  bool tem_dado_ = false;
};

class SensoresModule : public QWidget
{
  Q_OBJECT

public:
  explicit SensoresModule(RosBridge * bridge, QWidget * parent = nullptr);

protected:
  // Assina ao aparecer, cancela ao sair. Ver o cabecalho.
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

private:
  QWidget * construirLidar();
  QWidget * construirImu();
  QWidget * construirCamera();
  void reassinarCamera();

  RosBridge * bridge_ = nullptr;
  SensorBridge * sensores_ = nullptr;

  VisorLidar * visor_lidar_ = nullptr;
  QLabel * resumo_lidar_ = nullptr;

  QLabel * resumo_imu_ = nullptr;
  QLabel * valores_imu_ = nullptr;

  QComboBox * topico_camera_ = nullptr;
  QLabel * visor_camera_ = nullptr;
  QLabel * resumo_camera_ = nullptr;
};
