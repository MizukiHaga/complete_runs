#ifndef TESTAPP_TURN_DRIFT_H_
#define TESTAPP_TURN_DRIFT_H_

#include <math.h>
#include <stdint.h>

/* CW正。両輪逆回転のその場旋回専用。 */
namespace TurnDriftConfig {
constexpr bool Enabled = true;
constexpr float EncoderPer90Deg = 180.0f;
constexpr float EncoderWeight = 0.17f;
constexpr uint32_t SettleMs = 80U;
constexpr float MaxWheelMismatch = 0.35f;
// エンコーダ量子化の影響が大きい走り出しだけジャイロを使う。
constexpr float MinSampleEncoderDeg = 20.0f;
constexpr float WheelDifferenceToleranceDeg = 4.0f;
constexpr float ReverseWheelToleranceDeg = 4.0f;
constexpr float ReverseGyroToleranceDeg = 2.0f;
constexpr uint32_t MotionFaultConfirmMs = 20U;
}

class TurnDrift {
public:
  // 既存ログの状態番号を維持。
  enum Status { Accepted, WarmingUp, Invalid, Delayed, WheelMismatch,
                WrongDirection, TooSlow, RateMismatch, AngleMismatch, StatusCount };

  // 旋回開始からの累積角を融合。戻り値はジャイロ角へ足す差分。
  // 周期ごとに計算し直し、前周期の補正量を積算しない。
  float estimate(float gyro_angle, float l, float r, float direction,
                 bool encoder_valid = true, uint32_t sample_ms = 0U,
                 bool direction_valid = true)
  {
    if (applied_) return total_;
    if (faulted_) return 0.0f;
    gain_ = total_ = angle_error_ = residual_ = 0.0f;
    if (!encoder_valid || !isfinite(gyro_angle) || !isfinite(l) || !isfinite(r) ||
        l < 0.0f || r < 0.0f || (direction != -1.0f && direction != 1.0f))
      return reject(Invalid);
    const bool wrong_direction = !direction_valid ||
        direction * gyro_angle < -TurnDriftConfig::ReverseGyroToleranceDeg;
    // 大きな逆回転は暖機中でも確認する。小さな起動時の揺れは許容する。
    if (wrong_direction) return motionFault(WrongDirection, sample_ms);
    if (0.5f * (l + r) < TurnDriftConfig::MinSampleEncoderDeg) {
      pending_ = false;
      status_ = WarmingUp;
      ++counts_[WarmingUp];
      return 0.0f;
    }
    const float difference = fabsf(l-r);
    if (difference > TurnDriftConfig::WheelDifferenceToleranceDeg &&
        difference / fmaxf(l,r) > TurnDriftConfig::MaxWheelMismatch)
      return motionFault(WheelMismatch, sample_ms);
    pending_ = false;
    const float enc_angle = direction * 0.5f * (l+r) *
                            90.0f / TurnDriftConfig::EncoderPer90Deg;
    angle_error_ = enc_angle - gyro_angle;
    gain_ = TurnDriftConfig::EncoderWeight;
    total_ = gain_ * angle_error_;
    residual_ = angle_error_ - total_;
    status_ = Accepted;
    ++counts_[Accepted];
    return total_;
  }

  // 静定後に一度だけ適用する差分。制御時と同じ式を使う。
  float update(float gyro_angle, float l, float r, float elapsed, float direction,
               bool encoder_valid = true, bool direction_valid = true)
  {
    if (applied_) return 0.0f;
    if (!isfinite(elapsed) || elapsed <= 0.0f) return reject(Invalid);
    const float correction = estimate(gyro_angle, l, r, direction, encoder_valid,
                                      static_cast<uint32_t>(elapsed * 1000.0f),
                                      direction_valid);
    applied_ = true;
    return correction;
  }
  float gain() const { return gain_; }
  float rate() const { return 0.0f; }
  float total() const { return total_; }
  float angleError() const { return angle_error_; }
  float residual() const { return residual_; }
  Status status() const { return status_; }
  uint32_t count(Status status) const { return counts_[status]; }
private:
  float motionFault(Status status, uint32_t sample_ms) {
    if (!pending_ || pending_status_ != status) {
      pending_ = true;
      pending_status_ = status;
      pending_since_ms_ = sample_ms;
    }
    if (sample_ms - pending_since_ms_ >= TurnDriftConfig::MotionFaultConfirmMs)
      return reject(status);
    // 確認中は一時的にジャイロだけを使い、正常復帰を許す。
    status_ = WarmingUp;
    ++counts_[WarmingUp];
    return 0.0f;
  }
  bool pending_ = false;
  Status pending_status_ = WarmingUp;
  uint32_t pending_since_ms_ = 0U;
  float reject(Status status) {
    status_ = status;
    ++counts_[status];
    faulted_ = true; // 同じ旋回中はジャイロへ切り替えたままにする。
    gain_ = total_ = 0.0f;
    return 0.0f;
  }
  bool applied_ = false;
  bool faulted_ = false;
  float gain_ = 0.0f;
  float total_ = 0.0f;
  float angle_error_ = 0.0f;
  float residual_ = 0.0f;
  Status status_ = WarmingUp;
  uint32_t counts_[StatusCount] = {};
};
static_assert(TurnDriftConfig::EncoderPer90Deg > 0.0f, "encoder scale must be positive");
static_assert(TurnDriftConfig::EncoderWeight >= 0.0f &&
              TurnDriftConfig::EncoderWeight <= 1.0f, "weight must be in [0,1]");
static_assert(TurnDriftConfig::MaxWheelMismatch > 0.0f, "wheel threshold must be positive");
#endif
