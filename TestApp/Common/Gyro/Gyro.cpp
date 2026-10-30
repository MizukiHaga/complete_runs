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

void Gyro::update()
{
  if (!hub_imu_is_ready()) {
    return;
  }
  updateKalman();
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
  float angv[3] = {0.0f, 0.0f, 0.0f};
  hub_imu_get_angular_velocity(angv);
  /* pybricks: heading は CW 正、heading_rate = -angv.z */
  return -angv[2];
}

void Gyro::updateKalman()
{
  SYSTIM now = 0U;
  get_tim(&now);

  if (!kf_ready_) {
    kf_angle_    = hub_imu_get_heading();
    kf_variance_ = kf_r_;
    kf_last_tim_ = (uint64_t)now;
    kf_ready_    = true;
    return;
  }

  float dt_s =
      (float)((int64_t)now - (int64_t)kf_last_tim_) * kSystimToSec;
  if (dt_s <= 0.0f) {
    return;
  }
  if (dt_s > kMaxKalmanDtSec) {
    dt_s = kMaxKalmanDtSec;
  }

  const float yaw_rate = readYawRateDegPerSec();

  /* 予測 */
  kf_angle_ += yaw_rate * dt_s;
  kf_variance_ += kf_q_ * dt_s;

  /* 更新 */
  const float measurement = hub_imu_get_heading();
  const float innovation  = wrapDeg180(measurement - kf_angle_);
  const float gain        = kf_variance_ / (kf_variance_ + kf_r_);
  kf_angle_ += gain * innovation;
  kf_variance_ *= (1.0f - gain);

  kf_last_tim_ = (uint64_t)now;
}

float Gyro::heading() const
{
  return kf_angle_ + heading_offset_;
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
