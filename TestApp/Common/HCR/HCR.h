#ifndef GUARD_HCR_H_
#define GUARD_HCR_H_
#include <pbio/port.h>


class HCR {
public:  void configure_hrc(pbio_port_id_t left_motor_port,
                 pbio_port_id_t right_motor_port,
                 pbio_port_id_t color_sensor_port);
  void turn_90(const char* direction, float inner_radius_mm);
  void SlowDown(float left_speed, float right_speed, float duration_sec, float target_speed);
  void run_hrc(void);

private:
  void catch_black_line();
  void catch_blue_line();
  pbio_port_id_t left_motor_port_;
  pbio_port_id_t right_motor_port_;
  pbio_port_id_t color_sensor_port_;
  bool configured_ = false;
  double TARGET_ANGLE = 80.0; /* 黒検知時の終了判定に使う角度 */
  // static float angle[3];
};

#endif  // GUARD_HCR_H_