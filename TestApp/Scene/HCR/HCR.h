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
  void StraightRotation(int power, float left_rotations, float right_rotations,float correction = 0.0f);
  void LineTraceRotation(float left_rotation, float right_rotation, float left_speed, float right_speed, float p, float i, float d, float scale);
  void LineTraceRotationCheck(float duration_sec);
  void AllLineTrace();
  void TestLineTrace();
  float Test_p = ETROBO_COURSE_L ? 0.05f : 0.07f;
  float Test_i = 0.0006f;
  float Test_d = ETROBO_COURSE_L ? 0.52f : 0.52f;
  float Test_scale = ETROBO_COURSE_L ? 19.0f : 15.0f;

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
  // 共通パラメータ

  float hcr_p = ETROBO_COURSE_L ? 0.07f : 0.07f;
  float hcr_i = 0.0006f;
  float hcr_d = ETROBO_COURSE_L ? 0.52f : 0.52f;
  float hcr_scale = ETROBO_COURSE_L ? 20.0f : 20.0f;
  bool separate_pattern = true;
  float motor_speed = 100.0f;
  float hcr_90 = ETROBO_COURSE_L ? 90.0f : -90.0f;
  float hcr_175 = ETROBO_COURSE_L ? 175.0f : -175.0f;

  float hcr_p_s = ETROBO_COURSE_L ? 0.06f : 0.06f;
  float hcr_p_t = ETROBO_COURSE_L ? 0.07f : 0.07f;
  float hcr_scale_s = ETROBO_COURSE_L ? 10.0f : 10.0f;
  float hcr_scale_t = ETROBO_COURSE_L ? 18.0f : 18.0f;
  float targetHeading = ETROBO_COURSE_L ? 145.0f : -145.0f;

// 最初の直線のパラメータ
  float straight_rotation = 2.8f;
// 2個目の直線のパラメータ
  float second_straight_rotation = 2.0f;
// 3個目の直線のパラメータ
  float third_straight_rotation = 2.0f;
// 4個目の直線のパラメータ
  float fourth_straight_rotation = 2.0f;

//1個目と2個目のカーブのパラメータ
  float first_curve_rotation_r = ETROBO_COURSE_L ? 2.0f : 1.5f;
  float first_curve_rotation_l = ETROBO_COURSE_L ? 1.5f : 2.0f;
  // 2個目のカーブのパラメータ
  float second_curve_rotation_r = ETROBO_COURSE_L ? 2.1f : 1.1f;
  float second_curve_rotation_l = ETROBO_COURSE_L ? 1.1f : 2.1f;

  float curve_1and2_p = ETROBO_COURSE_L ? 0.09f : 0.09f;
  float curve_1and2_i = 0.0006f;
  float curve_1and2_d = ETROBO_COURSE_L ? 0.2f : 0.2f;
  float curve_1and2_scale = ETROBO_COURSE_L ? 12.0f : 12.0f;
  // 3個目のカーブのパラメータ
  float third_curve_rotation = 2.0f;
  float third_curve_rotation_r = ETROBO_COURSE_L ? 1.0f : 1.5f;
  float third_curve_rotation_l = ETROBO_COURSE_L ? 1.5f : 1.0f;
  float third_curve_p = ETROBO_COURSE_L ? 0.095f : 0.095f;
  float third_curve_scale = ETROBO_COURSE_L ? 15.0f : 15.0f;
  // 4個目のカーブのパラメータ
  float triple_curve_rotation_r = ETROBO_COURSE_L ? 16.0f : 16.0f;
  float triple_curve_rotation_l = ETROBO_COURSE_L ? 15.0f : 15.0f;
  float triple_curve_p = ETROBO_COURSE_L ? 0.06f : 0.06f;
  float triple_curve_scale = ETROBO_COURSE_L ? 18.0f : 18.0f;

  float detect_speed = 50.0f;

// 決め打ち用: 開始ライン追従
  float hcr_rotation =  ETROBO_COURSE_L ? 3.2f : 3.2f;
  bool point_1 = false;
  bool point_2 = false;
  bool area_1 = false;
  bool area_2 = false;
  bool area_3 = false;
  // 90度区間
  float heading_90 = ETROBO_COURSE_L ? -88.0f : 88.0f;
  float heading_90_straight_r = ETROBO_COURSE_L ? 0.7f : 0.4f;
  float heading_90_straight_l = ETROBO_COURSE_L ? 0.4f : 0.7f;
  float after_90_straight = ETROBO_COURSE_L ? 1.2f : 1.2f;

  // 165度区間
  float heading_165 = ETROBO_COURSE_L ? -168.0f : 168.0f;
  float after_165_straight = ETROBO_COURSE_L ? 1.0f : 1.0f;

  // 50度区間
  float heading_50 = ETROBO_COURSE_L ? -50.0f : 50.0f;
  float heading_50_rotation = 0.3f;
  float heading_50_straight = ETROBO_COURSE_L ? 1.0f : 1.0f;
  float before_triple_curve_straight = ETROBO_COURSE_L ? 0.5f : 0.5f;

  // トリプルカーブ区間
  float triple_curve_heading = ETROBO_COURSE_L ? -28.0f : 28.0f;
  float triple_curve_heading_rotation_r = ETROBO_COURSE_L ? 0.1f : 0.5f;
  float triple_curve_heading_rotation_l = ETROBO_COURSE_L ? 0.5f : 0.1f;
  float after_triple_curve_heading_rotation_r = ETROBO_COURSE_L ? 0.1f : 0.1f;
  float after_triple_curve_heading_rotation_l = ETROBO_COURSE_L ? 0.1f : 0.1f;
  float triple_curve_rotation = ETROBO_COURSE_L ? 6.2f : 6.2f;

  // 終端ライン追従
  float after_triple_curve_heading = ETROBO_COURSE_L ? -20.0f : 20.0f;
  float finish_line_trace_rotation = 1.5f;
  float finish_line_trace_speed = 100.0f;
  float finish_line_trace_p = 0.055f;
  float finish_line_trace_d = 0.3f;
  float finish_line_trace_scale = 18.0f;
  float finish_skyblue_speed = 50.0f;
  float finish_skyblue_p = 0.052f;
  float finish_skyblue_i = 0.0006f;
  float finish_skyblue_d = 0.3f;
  float finish_skyblue_scale = 22.0f;
};

#endif  // GUARD_HCR_H_
