/*
 * Gyro モジュール 実装
 *  ハードウェア依存の API (hub_imu_* / pbio_imu_*) はすべてこのファイル内に閉じる。
 *
 *  3D: 加速度込みの融合方位（加速で揺れやすい）
 *  1D: 単軸ジャイロ積分（直進の加減速でも方位が汚染されにくい）
 */

#include "Gyro.h"

#include <math.h>

extern "C" {
#include <kernel.h>
#include <spike/hub/imu.h>
#include <pbio/imu.h>
}

static constexpr uint32_t kReadyPollMs = 10U;

void Gyro::init()
{
  hub_imu_init();
  hub_imu_set_tilt(TiltDeg);
  while (!hub_imu_is_ready()) {
    dly_tsk(kReadyPollMs * 1000U);
  }
  rate_heading_   = false;
  heading_offset_ = 0.0f;
}

bool Gyro::isReady() const
{
  return hub_imu_is_ready();
}

void Gyro::reset(float offset_deg)
{
  hub_imu_reset_heading();
  heading_offset_ = offset_deg;
}

void Gyro::setRateHeading(bool enable)
{
  if (enable == rate_heading_) {
    return;
  }
  /* 切替前後で heading() を連続にする */
  const float current = heading();
  rate_heading_       = enable;
  heading_offset_     = current - rawHeading();
}

float Gyro::rawHeading() const
{
  if (rate_heading_) {
    /* 単軸ジャイロ積分。base orientation（tilt）適用済みの -Z */
    return pbio_imu_get_heading(PBIO_IMU_HEADING_TYPE_1D);
  }
  return hub_imu_get_heading();
}

float Gyro::heading() const
{
  return rawHeading() + heading_offset_;
}

float Gyro::diffTo(float target_heading) const
{
  return wrapDeg180(target_heading - heading());
}

bool Gyro::reached(float target_heading, float tol_deg) const
{
  return fabsf(diffTo(target_heading)) <= tol_deg;
}

float Gyro::wrapDeg180(float diff)
{
  /* +180 と -180 を同じ最短側に寄せ、±180 境界での取りこぼしを減らす */
  while (diff >= 180.0f) {
    diff -= 360.0f;
  }
  while (diff < -180.0f) {
    diff += 360.0f;
  }
  return diff;
}
