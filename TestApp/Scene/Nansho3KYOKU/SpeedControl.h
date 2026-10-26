#ifndef NANSHO3_KYOKU_SPEED_CONTROL_H
#define NANSHO3_KYOKU_SPEED_CONTROL_H

#include "Settings.h"
#include <math.h>

namespace kyoku {

inline float clampSpeedValue(float value, float minimum, float maximum)
{
  return fmaxf(minimum, fminf(value, maximum));
}

/* 片輪の目標速度と実測速度 [mm/s] からパワーを計算する。
 * 前進のみ。目標0なら停止、正の出力は MinRunPower〜MaxPower。
 * 速度超過で出力が不要なら0にして減速を許す。 */
inline int wheelPower(float target_speed, float measured_speed,
                      float &integral, float elapsed_sec)
{
  using namespace kyoku_settings;

  if (target_speed <= 0.0f) {
    integral = 0.0f;
    return 0;
  }

  const float speed_error = target_speed - measured_speed;
  const float next_integral = clampSpeedValue(
      integral + speed_error * elapsed_sec,
      -SpeedIntegralMax, SpeedIntegralMax);

  const float friction_ratio = clampSpeedValue(
      target_speed / StaticFrictionBlendSpeed, 0.0f, 1.0f);
  const float feed_forward = StaticFrictionPower * friction_ratio +
                             SpeedFeedForward * target_speed;
  const float output = feed_forward + SpeedKp * speed_error +
                       SpeedKi * next_integral;

  /* 出力上限・下限で積分を溜めすぎない。
   * 下限未満でも加速方向の積分は許し、必要なら35以上へ増やせるようにする。 */
  const bool within_range = output >= MinRunPower && output <= MaxPower;
  const bool reducing_high_output = output > MaxPower && speed_error < 0.0f;
  const bool increasing_low_output = output < MinRunPower && speed_error > 0.0f;
  if (within_range || reducing_high_output || increasing_low_output) {
    integral = next_integral;
  }

  if (output <= 0.0f) {
    return 0;
  }
  return static_cast<int>(lroundf(clampSpeedValue(
      output, static_cast<float>(MinRunPower), static_cast<float>(MaxPower))));
}

} // namespace kyoku

#endif // NANSHO3_KYOKU_SPEED_CONTROL_H
