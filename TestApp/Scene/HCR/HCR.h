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
  void LineTraceRotation(float left_rotation, float right_rotation, float left_speed, float right_speed, float p, float i, float d, float scale);
  void LineTraceRotationCheck(float duration_sec);
  void AllLineTrace();

private:
  void catch_black_line();
  void catch_blue_line();
  pbio_port_id_t left_motor_port_;
  pbio_port_id_t right_motor_port_;
  pbio_port_id_t color_sensor_port_;
  bool configured_ = false;
  double TARGET_ANGLE = 80.0; /* 黒検知時の終了判定に使う角度 */
  static constexpr uint32_t ignore_time = 500U;
  uint32_t ms = 0U;
  // R:L
  float hcr_90 = ETROBO_COURSE_L ? 90.0f : -90.0f;
  float hcr_175 = ETROBO_COURSE_L ? 175.0f : -175.0f;
  float hcr_p_s = ETROBO_COURSE_L ? 0.06f : 0.06f;
  float hcr_p_t = ETROBO_COURSE_L ? 0.07f : 0.07f;

  float hcr_i = 0.0006f;
  float hcr_d = ETROBO_COURSE_L ? 0.52f : 0.52f;
  float hcr_scale_s = ETROBO_COURSE_L ? 10.0f : 10.0f;
  float hcr_scale_t = ETROBO_COURSE_L ? 18.0f : 18.0f;
  float targetHeading = ETROBO_COURSE_L ? 145.0f : -145.0f;
};

#endif  // GUARD_HCR_H_
