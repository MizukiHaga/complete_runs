#ifndef GUARD_HCR_H_
#define GUARD_HCR_H_
#include <pbio/port.h>
#include <stdint.h>


class HCR {
public:
  void configure_hrc(pbio_port_id_t left_motor_port,
                       pbio_port_id_t right_motor_port,
                       pbio_port_id_t color_sensor_port);
  void turn_90(const char* direction, int32_t inner_arc_deg);
  void SlowDown(float left_speed, float right_speed, float duration_sec, float target_speed);
  void SpeedUp(float left_speed, float right_speed, float duration_sec, float target_speed, bool is_straight);
  /* LR は app.h の ETROBO_COURSE_L で切り替え（Nansho1/2/3 と同様）。 */
  void run_hrc();
  void ShortCutRun();
  void LineTrace();
  void AllLineTrace();

private:
  void catch_black_line();
  void catch_blue_line();
  pbio_port_id_t left_motor_port_;
  pbio_port_id_t right_motor_port_;
  pbio_port_id_t color_sensor_port_;
  bool configured_ = false;
  double TARGET_ANGLE = 80.0; /* 黒検知時の終了判定に使う角度 */
};

#endif  // GUARD_HCR_H_
