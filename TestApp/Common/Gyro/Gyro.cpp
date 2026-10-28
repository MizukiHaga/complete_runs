/*
 * Gyro モジュール 実装
 *  ハードウェア依存の API (hub_imu_* / pbio_imu_*) はすべてこのファイル内に閉じる。
 *
 *  有効時: 校正済み1D差分角と車輪角から連続KF推定。
 *  無効時: 従来の3D融合方位／1D積分角を選択。
 */

#include "Gyro.h"
#include "HeadingFusionSettings.h"
#include "Straight.h"

#include <math.h>

extern "C" {
#include <kernel.h>
#include <spike/hub/imu.h>
#include <pbio/imu.h>
#include <pbdrv/imu.h>
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
  rate_heading_   = false;
  heading_offset_ = 0.0f;
  fusion_enabled_ = HeadingFusionSettings::Enabled &&
                    fusion_.configure(HeadingFusionSettings::config());
  if (fusion_enabled_) {
    SYSTIM now = 0;
    get_tim(&now);
    fusion_time_ = now;
    fusion_raw_ = pbio_imu_get_heading(PBIO_IMU_HEADING_TYPE_1D);
    fusion_raw_valid_ = isfinite(fusion_raw_);
    fusion_encoder_valid_ = gMotors.readEncoders(fusion_left_, fusion_right_);
    fusion_.reset(fusion_raw_valid_ ? fusion_raw_ : 0.0f, false);
  }
}

bool Gyro::isReady() const
{
  return hub_imu_is_ready();
}

void Gyro::reset(float offset_deg)
{
  SYSTIM now = 0;
  get_tim(&now);
  loc_cpu();
  ++fusion_generation_;
  hub_imu_reset_heading();
  heading_offset_ = offset_deg;
  if (fusion_enabled_) {
    fusion_.reset(0.0f, true);
    fusion_raw_ = pbio_imu_get_heading(PBIO_IMU_HEADING_TYPE_1D);
    fusion_raw_valid_ = isfinite(fusion_raw_);
    fusion_encoder_valid_ = gMotors.readEncoders(fusion_left_, fusion_right_);
    fusion_time_ = now;
  }
  unl_cpu();
}

void Gyro::setRateHeading(bool enable)
{
  if (fusion_enabled_) {
    rate_heading_ = enable; // 呼出互換。推定器の観測源は常に同じ1D角で連続更新する。
    return;
  }
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
  if (rate_heading_ || fusion_enabled_) {
    /* 単軸ジャイロ積分。base orientation（tilt）適用済みの -Z */
    return pbio_imu_get_heading(PBIO_IMU_HEADING_TYPE_1D);
  }
  return hub_imu_get_heading();
}

float Gyro::heading() const
{
  return (fusion_enabled_ ? fusion_.state().angle : rawHeading()) + heading_offset_;
}

HeadingFusion::State Gyro::fusionState() const
{
  loc_cpu();
  const HeadingFusion::State state = fusion_.state();
  unl_cpu();
  return state;
}

void Gyro::beginTurnFusion()
{
  if (!fusion_enabled_) return;
  loc_cpu();
  fusion_.beginTurn();
  unl_cpu();
}

void Gyro::endTurnFusion()
{
  if (!fusion_enabled_) return;
  loc_cpu();
  fusion_.endTurn();
  unl_cpu();
}

void Gyro::invalidateFusionEncoders()
{
  loc_cpu();
  fusion_encoder_valid_ = false;
  fusion_.invalidateStationary();
  ++fusion_generation_;
  unl_cpu();
}

void Gyro::updateFusion()
{
  if (!fusion_enabled_) return;
  const uint32_t generation = fusion_generation_;
  SYSTIM now;
  if (get_tim(&now) != E_OK) return;
  const float raw = pbio_imu_get_heading(PBIO_IMU_HEADING_TYPE_1D);
  int32_t left = 0, right = 0;
  const bool enc_valid = gMotors.readEncoders(left, right);
  const bool raw_valid = isfinite(raw);
  /* hub_imu_is_stationaryは全モータcoastも要求するため、brake中は常にfalse。
   * センサ自身の静止判定と、別途stopCommanded/車輪範囲/角速度を組み合わせる。 */
  pbdrv_imu_dev_t *imu_dev = nullptr;
  pbdrv_imu_config_t *imu_config = nullptr;
  const bool stationary = pbdrv_imu_get_imu(&imu_dev, &imu_config) == PBIO_SUCCESS &&
      imu_dev != nullptr && pbdrv_imu_is_stationary(imu_dev);
  /* センサ取得中に高優先度タスクがresetした場合は、その古い観測を破棄する。
   * KF更新とresetは同じCPUロックで保護し、途中状態を混ぜない。 */
  loc_cpu();
  if (generation != fusion_generation_) {
    unl_cpu();
    return;
  }
  const float dt = static_cast<float>(now - fusion_time_) / 1000000.0f;
  const bool time_valid = dt > 0 && dt <= HeadingFusionSettings::MaxPeriodMs / 1000.0f;
  if (time_valid && raw_valid && fusion_raw_valid_) {
    const float left_delta = static_cast<float>(static_cast<int64_t>(left) - fusion_left_);
    const float right_delta = static_cast<float>(static_cast<int64_t>(right) - fusion_right_);
    const float limit = HeadingFusionSettings::MaxWheelRateDegPerSec * dt + 2.0f;
    const bool wheel_valid = enc_valid && fusion_encoder_valid_ &&
        fabsf(left_delta) <= limit && fabsf(right_delta) <= limit;
    fusion_.step(wrapDeg180(raw - fusion_raw_), left_delta, right_delta, dt,
                  wheel_valid, stationary, gMotors.stopCommanded());
  } else {
    fusion_.invalidateStationary();
  }
  fusion_raw_ = raw;
  fusion_raw_valid_ = raw_valid;
  fusion_left_ = left;
  fusion_right_ = right;
  fusion_encoder_valid_ = enc_valid;
  fusion_time_ = now;
  unl_cpu();
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
