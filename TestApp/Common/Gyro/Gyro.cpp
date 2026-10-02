/*
 * Gyro モジュール 実装
 *  Hub 内蔵 IMU のヨー角 (heading) を扱う薄いラッパー。
 *  ハードウェア依存の API (hub_imu_*) はすべてこのファイル内に閉じ込めている。
 *
 *  1 次元カルマンフィルタ:
 *    予測: θ += EncYawScale * (r/b) * (ΔL − ΔR)   系統込みエンコーダ差、CW 正
 *    観測: hub_imu_get_heading()
 */

#include "Gyro.h"
#include "Straight.h"

#include <math.h>

extern "C" {
#include <kernel.h>
#include <spike/hub/imu.h>
}

extern Straight gMotors;

static constexpr uint32_t kReadyPollMs = 10U;

void Gyro::init()
{
  hub_imu_init();
  hub_imu_set_tilt(TiltDeg);
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

void Gyro::snapshotEncoders() const
{
  kf_enc_left_  = gMotors.leftEncoderCount();
  kf_enc_right_ = gMotors.rightEncoderCount();
}

void Gyro::reset(float offset_deg)
{
  hub_imu_reset_heading();
  heading_offset_ = offset_deg;
  kf_angle_    = 0.0f;
  kf_variance_ = kf_r_;
  snapshotEncoders();
  SYSTIM now     = 0U;
  get_tim(&now);
  kf_last_tim_ = (uint64_t)now;
  kf_ready_    = true;
}

float Gyro::encoderYawDeltaDeg() const
{
  const int32_t left  = gMotors.leftEncoderCount();
  const int32_t right = gMotors.rightEncoderCount();
  const int32_t dL    = left  - kf_enc_left_;
  const int32_t dR    = right - kf_enc_right_;
  kf_enc_left_  = left;
  kf_enc_right_ = right;
  /* 前進で左右とも増加。CW ピボットは L+ / R− → ΔL−ΔR > 0
   * kEncYawPerMotorDeg に 180/175 の系統スケールを含む */
  return kEncYawPerMotorDeg * static_cast<float>(dL - dR);
}

void Gyro::updateKalman() const
{
  SYSTIM now = 0U;
  get_tim(&now);

  if (!kf_ready_) {
    kf_angle_    = 0.0f;
    kf_variance_ = kf_r_;
    snapshotEncoders();
    kf_last_tim_ = (uint64_t)now;
    kf_ready_    = true;
    return;
  }

  float dt_s =
      (float)((int64_t)now - (int64_t)kf_last_tim_) * kSystimToSec;
  if (dt_s < 0.0f) {
    dt_s = 0.0f;
  }
  if (dt_s > kMaxKalmanDtSec) {
    dt_s = kMaxKalmanDtSec;
  }

  const float enc_yaw = encoderYawDeltaDeg();

  /* 予測（エンコーダ差。発進加速度がヨーに漏れない） */
  kf_angle_ += enc_yaw;
  kf_variance_ += kf_q_ * dt_s;

  /* 更新（ハブ方位） */
  const float measurement = hub_imu_get_heading();
  const float innovation  = wrapDeg180(measurement - kf_angle_);
  const float r           = kf_r_;
  if (r > 0.0f) {
    const float gain = kf_variance_ / (kf_variance_ + r);
    kf_angle_ += gain * innovation;
    kf_variance_ *= (1.0f - gain);
  }

  kf_last_tim_ = (uint64_t)now;
}

float Gyro::heading() const
{
  updateKalman();
  return kf_angle_ + heading_offset_;
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
