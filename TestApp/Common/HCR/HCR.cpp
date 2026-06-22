#include "app.h"
#include "ColorHSV.h"
#include "LineTracer.h"
#include "../Straight/Straight.h"
#include "HCR.h"
#include "kernel_cfg.h"

#include <spike/hub/speaker.h>

extern "C" {
#include <spike/pup/motor.h>
#include <spike/hub/imu.h>
}

#include <math.h>
#include <string.h>

extern Straight gMotors;
extern ColorHSV gColorHSV;
extern LineTracer gLineTracer;

void HCR::configure_hrc(pbio_port_id_t left_motor_port,
                    pbio_port_id_t right_motor_port,
                    pbio_port_id_t color_sensor_port)
{
  left_motor_port_ = left_motor_port;
  right_motor_port_ = right_motor_port;
  color_sensor_port_ = color_sensor_port;
  configured_ = true;

  gMotors.configure(left_motor_port, right_motor_port);
  gColorHSV.configure(color_sensor_port);
  gLineTracer.configure(left_motor_port, right_motor_port, color_sensor_port);
}

void HCR::turn_90(const char* direction, float inner_radius_mm)
{
  if (!configured_) {
    return;
  }

  const bool turn_right = strcmp(direction, "RIGHT") == 0;
  const bool turn_left = strcmp(direction, "LEFT") == 0;
  if (!turn_right && !turn_left) {
    return;
  }

  pup_motor_t *left_motor = pup_motor_get_device(left_motor_port_);
  pup_motor_t *right_motor = pup_motor_get_device(right_motor_port_);
  if (left_motor == nullptr || right_motor == nullptr) {
    return;
  }

  if (inner_radius_mm <= 0.0f) {
    inner_radius_mm = 140.0f;
  }

  /* 内側の円弧を 90 度ぶん進むように、内輪の距離を基準に左右の速度比を決める。 */
  constexpr float kPi = 3.14159265358979323846f;
  constexpr float kWheelDiameterMm = 56.0f;
  constexpr float kTrackWidthMm = 120.0f;
  /* 速度を少し上げて応答性を改善 */
  constexpr int kInnerPower = 60;

  const float outerRadiusMm = inner_radius_mm + kTrackWidthMm;
  const float wheelCircumferenceMm = kPi * kWheelDiameterMm;
  const float innerArcMm = 0.5f * kPi * inner_radius_mm;
  const int targetDegrees = (int)(innerArcMm * 360.0f / wheelCircumferenceMm + 0.5f);

  int outerPower = (int)(kInnerPower * outerRadiusMm / inner_radius_mm + 0.5f);
  if (outerPower > 100) {
    outerPower = 100;
  }

  const int left_power = turn_right ? outerPower : kInnerPower;
  const int right_power = turn_right ? kInnerPower : outerPower;
  const pup_motor_t *inner_motor = turn_right ? right_motor : left_motor;

  gMotors.run(left_power, right_power);
  gColorHSV.resetBlackEdge();

  /* IMU の heading を直接参照して停止判定する。 */
  hub_imu_reset_heading();
  dly_tsk(2 * 1000); /* リセット後の短い安定時間（短縮） */

  constexpr int kSampleMs = 3; /* 高レスポンスのため短めにサンプリング */
  const float target_deg = TARGET_ANGLE; /* 黒検知と組み合わせる角度の下限 */
  const float slow_deg = target_deg - 25.0f; /* この角度から曲がり幅を小さくする */
  const float min_turn_ratio = 0.3f; /* 終盤の曲がり幅の下限 */

  /* 80度以上かつ黒線を検知したら停止する */
  while (true) {
    float heading = hub_imu_get_heading();
    /* 左回転では IMU が負の角度を返すため、絶対値で扱う */
    float abs_heading = turn_right ? heading : -heading;

    float turn_ratio = 1.0f;
    if (abs_heading >= slow_deg) {
      float t = (abs_heading - slow_deg) / (target_deg - slow_deg);
      if (t < 0.0f) {
        t = 0.0f;
      }
      if (t > 1.0f) {
        t = 1.0f;
      }
      // t = t * t;
      turn_ratio = 1.0f - t * (1.0f - min_turn_ratio);
    }

    const float center_power = 0.5f * (left_power + right_power);
    const float power_delta = 0.5f * (left_power - right_power) * turn_ratio;

    int cur_left = (int)(center_power + power_delta + 0.5f);
    int cur_right = (int)(center_power - power_delta + 0.5f);

    gMotors.run(cur_left, cur_right);

    if (abs_heading >= target_deg && gColorHSV.isBlack(true)) {
      gMotors.stop();
      dly_tsk(500 * 1000);
      // hub_speaker_set_volume(50);
      // hub_speaker_play_tone(1500, 300);
      break;
    }

    dly_tsk(kSampleMs * 1000);
  }

}


void HCR::catch_black_line()
{
  if (!configured_) {
    return;
  }
  gMotors.run(75, 70);
  while (true) {
    if (gColorHSV.isBlack(true)) {
      gMotors.stop();
      dly_tsk(500 * 1000);
      gLineTracer.setBaseSpeed(70.0f, 70.0f);
      gLineTracer.setPid(0.12f,0.0004f,0.8f, 100.0f);
      gLineTracer.setScale(4.5f);
      sta_cyc(LINE_TRACER_TASK_CYC);
      dly_tsk(2000 * 1000);
      stp_cyc(LINE_TRACER_TASK_CYC);
      break;
    }
    dly_tsk(5 * 1000);
  }
}

void HCR::catch_blue_line()
{
  gLineTracer.setBaseSpeed(80.0f, 80.0f);
  gLineTracer.setPid(0.04f,0.0004f,0.3f, 100.0f);
  gLineTracer.setScale(2.5f);
  sta_cyc(LINE_TRACER_TASK_CYC);
  while (true) {
    if (gColorHSV.is(COLOR_DETECT_BLUE, true)) {
      stp_cyc(LINE_TRACER_TASK_CYC);
      gMotors.stop();
      break;
    }
    dly_tsk(5 * 1000);
  }
}

void HCR::run_hrc(void)
{
  gLineTracer.setBaseSpeed(70.0f, 70.0f);
  gLineTracer.setPid(0.12f,0.0004f,0.8f, 100.0f);
  gLineTracer.setScale(5.5f);
  // sta_cyc(LINE_TRACER_TASK_CYC);
  // dly_tsk(9600 * 1000);
  // dly_tsk(1000 * 1000);
  // stp_cyc(LINE_TRACER_TASK_CYC);
  // gMotors.run(73, 70);
  // dly_tsk(2500 * 1000);
  gMotors.run(60, 60);
  dly_tsk(1000 * 1000);
  gMotors.stop();
  dly_tsk(5000 * 1000);

  // catch_black_line();
  // catch_blue_line();
}