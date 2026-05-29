/*
 * Gyro モジュール 実装
 *  Hub 内蔵 IMU のヨー角 (heading) を扱う薄いラッパー。
 *  ハードウェア依存の API (hub_imu_*) はすべてこのファイル内に閉じ込めている。
 */

#include "Gyro.h"

#include <math.h>

extern "C" {
#include <kernel.h>
#include <spike/hub/imu.h>
}

/* === 内部定数 === */
/* is_ready ポーリングの間隔 [ms]。短すぎるとビジーループ気味、
 * 長すぎると初期化完了から実走行までのレイテンシが目立つので 10ms に。 */
static constexpr uint32_t kReadyPollMs = 10U;

void Gyro::init()
{
  hub_imu_init();
  while (!hub_imu_is_ready()) {
    dly_tsk(kReadyPollMs * 1000U);
  }
}

bool Gyro::isReady() const
{
  return hub_imu_is_ready();
}

void Gyro::reset()
{
  hub_imu_reset_heading();
}

float Gyro::heading() const
{
  return hub_imu_get_heading();
}

float Gyro::diffTo(float target_heading) const
{
  return wrapDeg180(target_heading - heading());
}

bool Gyro::reached(float target_heading, float tol_deg) const
{
  return fabsf(diffTo(target_heading)) <= tol_deg;
}

/* static */
float Gyro::wrapDeg180(float diff)
{
  while (diff > 180.0f)  diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}
