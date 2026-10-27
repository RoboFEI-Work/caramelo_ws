#pragma once

// Gate de pose do GoToWS (2026-10-26, investigacao das runs de 22-23/10):
// logica PURA (sem ROS) que decide, a partir do result do align_to_dock e
// da odometria, se a manipulacao pode comecar. Testada em
// test/test_pose_gate.cpp sem nenhum no' no ar.
//
// Motivo: o GoToWS so' olhava `success` do AlignToDock. Nos logs de 22-23/10
// o docking devolveu sucesso com `final_y_error` +0,304 m e
// `final_yaw_error` +0,319 rad (WS3, 23/10 18:51), 'yaw_nao_medido' com
// +0,271 rad pelo AMCL (PP1, 22/10 15:42) e 'yaw_fora_da_tolerancia
// (-12,7 graus)' (WS6, 23/10 19:08) — e a manipulacao comecou em todos.
//
// Limites DUROS (acima deles a geometria nao fecha; abaixo o docking ja'
// avisa e a manipulacao comprovadamente trabalha):
//   |y|   <= 0,20 m : meia-largura do chassi 0,195 + 0,20 = 0,395 <= meia-
//                     largura da mesa 0,40 — o corpo do robo ainda fica
//                     dentro da largura da mesa; alem disso metade do
//                     alcance lateral do braco (+-0,45 m medido 23/10) cai
//                     fora do tampo e a fila de alcance (<= 0,35 m) nao cobre.
//   |yaw| <= 0,20 rad: = yaw_goal_tolerance do Nav2 (11,5 graus) — a staging
//                     nunca chega pior que isso por si so'; e' tambem o maior
//                     yaw em que o place por superficie (frente da mesa lida
//                     como UMA distancia y pelo LiDAR) erra <= 3 cm na zona
//                     de 2-20 cm nos candidatos tipicos (|x| <= 0,15 m:
//                     0,15 * tan 11,5 = 3,1 cm).
//   |x|   <= 0,10 m : o proprio align ja' falha acima de ~0,085 m (janela
//                     frontal do muro); aqui so' fecha a porta.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace manip_bt
{

// Fonte do `final_yaw_error` do AlignToDock (contrato da action): yaw da
// FACE pelo LiDAR, ou erro pelo AMCL quando a face nao foi medida no fim
// ('yaw_nao_medido' — nada girou, so' informativo).
enum class YawSource { kLidar, kAmcl };

struct AlignNote
{
  bool yaw_out_of_tolerance{false};  // message comeca com 'yaw_fora_da_tolerancia'
  bool yaw_unmeasured{false};        // message comeca com 'yaw_nao_medido'
  YawSource yaw_source{YawSource::kLidar};
};

inline bool startsWith(const std::string & text, const std::string & prefix)
{
  return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

// Interpreta a `message` do result (prefixos documentados em
// caramelo_msgs/action/AlignToDock.action).
inline AlignNote classifyAlignMessage(const std::string & message)
{
  AlignNote note;
  note.yaw_out_of_tolerance = startsWith(message, "yaw_fora_da_tolerancia");
  note.yaw_unmeasured = startsWith(message, "yaw_nao_medido");
  note.yaw_source = note.yaw_unmeasured ? YawSource::kAmcl : YawSource::kLidar;
  return note;
}

inline const char * yawSourceName(YawSource source)
{
  return source == YawSource::kLidar ? "lidar" : "amcl";
}

struct AlignOutcome
{
  bool success{false};
  std::string message;
  double x_error{0.0};    // m, ao longo da normal da face (front - stop)
  double y_error{0.0};    // m, lateral pelo AMCL (+ = alvo a esquerda do robo)
  double yaw_error{0.0};  // rad (fonte: ver AlignNote)
};

struct PoseGateLimits
{
  double max_abs_x{0.10};
  double max_abs_y{0.20};
  double max_abs_yaw{0.20};
};

struct PoseGateVerdict
{
  bool ok{false};
  std::string reason;  // vazio quando ok; senao o que estourou, com valores
};

inline std::string formatMeters(double value)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%+.3f m", value);
  return buf;
}

inline std::string formatRad(double value)
{
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%+.3f rad (%+.1f graus)", value, value * 180.0 / M_PI);
  return buf;
}

// Decide se a pose final permite manipular. Erro nao finito (NaN/inf) e'
// reprovado: nao se manipula sem saber onde a base esta.
inline PoseGateVerdict evaluatePoseGate(const AlignOutcome & r, const PoseGateLimits & lim)
{
  PoseGateVerdict v;
  if (!r.success) {
    v.reason = "align sem sucesso";
    return v;
  }
  if (!std::isfinite(r.x_error) || !std::isfinite(r.y_error) || !std::isfinite(r.yaw_error)) {
    v.reason = "erro nao finito no result do align";
    return v;
  }
  std::string why;
  if (std::fabs(r.x_error) > lim.max_abs_x) {
    why += "x " + formatMeters(r.x_error) + " > " + formatMeters(lim.max_abs_x) + "; ";
  }
  if (std::fabs(r.y_error) > lim.max_abs_y) {
    why += "y " + formatMeters(r.y_error) + " > " + formatMeters(lim.max_abs_y) + "; ";
  }
  if (std::fabs(r.yaw_error) > lim.max_abs_yaw) {
    why += "yaw " + formatRad(r.yaw_error) + " > " + formatRad(lim.max_abs_yaw) + "; ";
  }
  if (!why.empty()) {
    why.erase(why.size() - 2);  // "; " final
    v.reason = why;
    return v;
  }
  v.ok = true;
  return v;
}

// Detector de PARADA por odometria: a base esta parada quando TODAS as
// amostras de /odom desde `quiet_since` (>= o reset) ficaram abaixo dos
// limites por pelo menos `window_s` E a ultima amostra e' fresca (<=
// max_age_s). Nao e' um atraso fixo: sem odometria fresca o estado e'
// kNoOdom; com qualquer amostra acima do limite a janela recomeca.
//
// Limites fisicos: o piso do ESC e' 0,1215 m/s / 0,315 rad/s de corpo (a
// roda nao gira abaixo de 2,43 rad/s) — qualquer movimento real aparece
// bem acima de 0,02 m/s / 0,05 rad/s; a janela de 0,5 s cobre a retencao da
// referencia pelo ESC (~0,5 s medido em campo, docs/docking_alinhamento_holonomico.md).
//
// Yaw (opcional, 2026-10-26, F14/C2): o detector tambem roda ANTES do align,
// logo apos o Nav2 dizer "Reached the goal", quando a base ainda gira
// 0,2-0,5 rad/s por ~1 s. Para o log dizer quanto o rumo mudou enquanto se
// esperava, `feed` aceita o yaw da odometria e `yawDrift()` devolve a
// diferenca (embrulhada) entre a primeira e a ultima amostra desde o reset.
class StopDetector
{
public:
  struct Limits
  {
    double max_lin{0.02};    // m/s, modulo de (vx, vy)
    double max_ang{0.05};    // rad/s, |wz|
    double window_s{0.5};    // tempo continuo abaixo dos limites
    double max_age_s{0.5};   // idade maxima da ultima amostra
  };

  enum class State { kNoOdom, kMoving, kStopped };

  StopDetector() = default;
  explicit StopDetector(const Limits & limits)
  : lim_(limits) {}

  // Abre uma janela nova: amostras anteriores a `t_now` nao contam como
  // "parado" (o align acabou de devolver o result; o ESC ainda pode estar
  // retendo a ultima referencia).
  void reset(double t_now)
  {
    t_reset_ = t_now;
    has_sample_ = false;
    t_last_ = 0.0;
    last_lin_ = last_ang_ = 0.0;
    quiet_since_ = std::nan("");
    samples_ = 0;
    yaw_first_ = yaw_last_ = std::nan("");
  }

  // `yaw` (rad, frame da odometria) e' opcional: NaN = nao acompanhar.
  void feed(double t, double vx, double vy, double wz, double yaw = std::nan(""))
  {
    const double lin = std::hypot(vx, vy);
    const double ang = std::fabs(wz);
    has_sample_ = true;
    t_last_ = t;
    last_lin_ = lin;
    last_ang_ = ang;
    ++samples_;
    if (std::isfinite(yaw)) {
      if (!std::isfinite(yaw_first_)) {
        yaw_first_ = yaw;
      }
      yaw_last_ = yaw;
    }
    const bool quiet = std::isfinite(lin) && std::isfinite(ang) &&
      lin <= lim_.max_lin && ang <= lim_.max_ang;
    if (!quiet) {
      quiet_since_ = std::nan("");
      return;
    }
    if (!std::isfinite(quiet_since_)) {
      quiet_since_ = std::max(t, t_reset_);
    }
  }

  State evaluate(double t_now) const
  {
    if (!has_sample_ || t_now - t_last_ > lim_.max_age_s) {
      return State::kNoOdom;
    }
    if (!std::isfinite(quiet_since_)) {
      return State::kMoving;
    }
    return (t_last_ - quiet_since_ >= lim_.window_s) ? State::kStopped : State::kMoving;
  }

  double lastLin() const {return last_lin_;}
  double lastAng() const {return last_ang_;}
  double lastSampleTime() const {return t_last_;}
  int samples() const {return samples_;}
  // Quanto o yaw da odometria mudou desde o reset (ultima - primeira amostra
  // com yaw), em rad embrulhado em (-pi, pi]; 0 sem yaw alimentado.
  double yawDrift() const
  {
    if (!std::isfinite(yaw_first_) || !std::isfinite(yaw_last_)) {
      return 0.0;
    }
    double d = yaw_last_ - yaw_first_;
    while (d > M_PI) {d -= 2.0 * M_PI;}
    while (d <= -M_PI) {d += 2.0 * M_PI;}
    return d;
  }
  // Tempo continuo parado ate a ultima amostra (0 se em movimento/sem amostra).
  double quietFor() const
  {
    if (!has_sample_ || !std::isfinite(quiet_since_)) {
      return 0.0;
    }
    return t_last_ - quiet_since_;
  }
  const Limits & limits() const {return lim_;}

private:
  Limits lim_;
  double t_reset_{0.0};
  bool has_sample_{false};
  double t_last_{0.0};
  double last_lin_{0.0};
  double last_ang_{0.0};
  double quiet_since_{std::nan("")};
  int samples_{0};
  double yaw_first_{std::nan("")};
  double yaw_last_{std::nan("")};
};

inline const char * stopStateName(StopDetector::State state)
{
  switch (state) {
    case StopDetector::State::kStopped: return "parado";
    case StopDetector::State::kMoving: return "em_movimento";
    default: return "sem_odometria";
  }
}

}  // namespace manip_bt
