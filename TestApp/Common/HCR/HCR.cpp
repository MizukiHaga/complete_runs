#include "app.h"
#include "ColorHSV.h"
#include "../Color/ColorRGB/ColorRGB.h"
#include "LineTracer.h"
#include "../Straight/Straight.h"
#include "../Gyro/Gyro.h"
#include "HCR.h"
#include "ColorLAB.h"
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
extern ColorRGB gColorRGB;
extern ColorLAB gColorLAB;
extern LineTracer gLineTracer;
extern Gyro gGyro;

void HCR::configure_hrc(pbio_port_id_t left_motor_port,
                    pbio_port_id_t right_motor_port,
                    pbio_port_id_t color_sensor_port)
{
  left_motor_port_ = left_motor_port;
  right_motor_port_ = right_motor_port;
  color_sensor_port_ = color_sensor_port;
  configured_ = true;

  // gMotors.configure(left_motor_port, right_motor_port);
  // gColorHSV.configure(color_sensor_port);
  // gLineTracer.configure(left_motor_port, right_motor_port, color_sensor_port);
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

    if (abs_heading >= target_deg && gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
      gMotors.stop();
      dly_tsk(500 * 1000);
      // hub_speaker_set_volume(50);
      // hub_speaker_play_tone(1500, 300);
      break;
    }

    dly_tsk(kSampleMs * 1000);
  }

}



void HCR::catch_blue_line()
{
  gLineTracer.setBaseSpeed(80.0f, 80.0f);
  gLineTracer.setPid(0.07f,0.0006f,0.5f, 100.0f);
  gLineTracer.setScale(10.0f);
  sta_cyc(LINE_TRACER_TASK_CYC);
  dly_tsk(800 * 1000);
  stp_cyc(LINE_TRACER_TASK_CYC);

  gLineTracer.setBaseSpeed(80.0f, 80.0f);
  gLineTracer.setPid(0.05f,0.0004f,0.3f, 100.0f);
  gLineTracer.setScale(4.5f);
  // gLineTracer.setBaseSpeed(85.0f, 85.0f);
  // gLineTracer.setPid(0.04f,0.0004f,0.3f, 100.0f);
  // gLineTracer.setScale(1.0f);
  sta_cyc(LINE_TRACER_TASK_CYC);
  dly_tsk(800 * 1000);
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setBaseSpeed(40.0f, 40.0f);
  gLineTracer.setPid(0.045f,0.0004f,0.4f, 100.0f);
  gLineTracer.setScale(5.0f);
  while (true) {
    if (gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
      gLineTracer.setBaseSpeed(30.0f, 30.0f);
      gLineTracer.setPid(0.06f,0.0004f,0.4f, 100.0f);
      gLineTracer.setScale(5.0f);
      while (true) {
        if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
          gMotors.stop();
          dly_tsk(5000 * 1000);
          break;
        }
        gLineTracer.step();
        dly_tsk(1 * 1000);
      }
    }
    gLineTracer.step();
    dly_tsk(5 * 1000);
  }
}
void HCR::catch_black_line()
{
  while (true) {
    if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
      gMotors.stop();
      dly_tsk(500 * 1000);
      break;
    }
    gMotors.run(67, 70);
    dly_tsk(5 * 1000);
  }
}


void HCR::run_hrc(void)
{
  gLineTracer.setBaseSpeed(80.0f, 80.0f);
  gLineTracer.setPid(0.07f,0.0006f,0.5f, 100.0f);
  gLineTracer.setScale(10.0f);
  // 
  // gLineTracer.setBaseSpeed(85.0f, 85.0f);
  // gLineTracer.setPid(0.068f,0.0006f,0.6f, 100.0f);
  // gLineTracer.setScale(15.0f);
  sta_cyc(LINE_TRACER_TASK_CYC);
  dly_tsk(4300 * 1000);
  stp_cyc(LINE_TRACER_TASK_CYC);
  gGyro.reset();
  sta_cyc(LINE_TRACER_TASK_CYC);
  dly_tsk(3500 * 1000);
  stp_cyc(LINE_TRACER_TASK_CYC);
  gMotors.setTargetHeading(-152.0f);
  SpeedUp(80.0f, 80.0f, 0.5f, 100.0f, true);
  gMotors.setTargetHeading(-152.0f);
  gMotors.run(100.0f, 100.0f);
  dly_tsk(1000 * 1000);
  catch_black_line();
  catch_blue_line();
}

void HCR::SlowDown(float left_speed, float right_speed, float duration_sec, float target_speed)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  

  if (duration_sec <= 0.0f) {
    sta_cyc(LINE_TRACER_TASK_CYC);
    return;
  }

  const uint32_t total_us = (uint32_t)(duration_sec * 1000000.0f + 0.5f);
  const uint32_t step_us = 10000U;

  uint32_t elapsed_us = 0U;
  while (elapsed_us < total_us) {
    uint32_t slice_us = step_us;
    if (elapsed_us + slice_us > total_us) {
      slice_us = total_us - elapsed_us;
    }

    elapsed_us += slice_us;

    const float ratio = (float)elapsed_us / (float)total_us;
    const float cur_left = left_speed + (target_speed - left_speed) * ratio;
    const float cur_right = right_speed + (target_speed - right_speed) * ratio;

    gLineTracer.setBaseSpeed(cur_left, cur_right);
    sta_cyc(LINE_TRACER_TASK_CYC);
    dly_tsk((RELTIM)slice_us);
  }
  gLineTracer.setBaseSpeed(target_speed, target_speed);
  sta_cyc(LINE_TRACER_TASK_CYC);
  return;
}

void HCR::SpeedUp(float left_speed, float right_speed, float duration_sec, float target_speed, bool is_straight)
{
  stp_cyc(LINE_TRACER_TASK_CYC);

  if (duration_sec <= 0.0f) {
    sta_cyc(LINE_TRACER_TASK_CYC);
    return;
  }

  const uint32_t total_us = (uint32_t)(duration_sec * 1000000.0f + 0.5f);
  const uint32_t step_us = 10000U;

  uint32_t elapsed_us = 0U;
  while (elapsed_us < total_us) {
    uint32_t slice_us = step_us;
    if (elapsed_us + slice_us > total_us) {
      slice_us = total_us - elapsed_us;
    }

    elapsed_us += slice_us;

    const float ratio = (float)elapsed_us / (float)total_us;
    const float cur_left = left_speed + (target_speed - left_speed) * ratio;
    const float cur_right = right_speed + (target_speed - right_speed) * ratio;
    if (is_straight) {
      gMotors.run(cur_left, cur_right);
    }else {
      gLineTracer.setBaseSpeed(cur_left, cur_right);
      sta_cyc(LINE_TRACER_TASK_CYC);
    }
    dly_tsk((RELTIM)slice_us);
  }
  if (is_straight) {
    gMotors.run(target_speed, target_speed);
  } else {
    gLineTracer.setBaseSpeed(target_speed, target_speed);
    sta_cyc(LINE_TRACER_TASK_CYC);
  }
  return;
}