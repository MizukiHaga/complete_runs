/*
 * Gyro モジュール 実装
 *  Hub 内蔵 IMU のヨー角 (heading) を扱う薄いラッパー。
 *  ハードウェア依存の API (hub_imu_*) はすべてこのファイル内に閉じ込めている。
 *
 *  1 次元カルマンフィルタ:
 *    予測: θ += ω_yaw * dt        (ω_yaw = -angv.z, CW 正)
 *    観測: hub_imu_get_heading()
 */

#include "Gyro.h"

#include <math.h>

extern "C" {
#include <kernel.h>
#include <spike/hub/imu.h>
}

static constexpr uint32_t kReadyPollMs = 10U;

void Gyro::init()
{
  hub_imu_init();
  while (!hub_imu_is_ready()) {
    dly_tsk(kReadyPollMs * 1000U);
  }
  kf_ready_ = false;
}

bool Gyro::isReady() const
{
  return hub_imu_is_ready();
}

void Gyro::setKalmanNoise(float process_noise_deg2_per_s,
                          float measurement_noise_deg2)
{
  if (process_noise_deg2_per_s > 0.0f) {
    kf_q_ = process_noise_deg2_per_s;
  }
  if (measurement_noise_deg2 > 0.0f) {
    kf_r_ = measurement_noise_deg2;
  }
}

void Gyro::reset()
{
  hub_imu_reset_heading();
  kf_angle_    = 0.0f;
  kf_variance_ = kf_r_;
  SYSTIM now     = 0U;
  get_tim(&now);
  kf_last_tim_ = (uint64_t)now;
  kf_ready_    = true;
}

float Gyro::readYawRateDegPerSec() const
{
  float angv[3] = {0.0f, 0.0f, 0.0f};
  hub_imu_get_angular_velocity(angv);
  /* pybricks: heading は CW 正、heading_rate = -angv.z */
  return -angv[2];
}

void Gyro::updateKalman() const
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
  updateKalman();
  return kf_angle_;
}

float Gyro::rawHeading() const
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
