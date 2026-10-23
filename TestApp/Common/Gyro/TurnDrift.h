#ifndef TESTAPP_TURN_DRIFT_H_
#define TESTAPP_TURN_DRIFT_H_

#include <math.h>
#include <stdint.h>

/* CW 正。両輪逆回転のその場旋回専用。 */
namespace TurnDriftConfig {
constexpr bool Enabled = true;
constexpr float EncoderPer90Deg = 180.0f;
constexpr float SampleEncoderDeg = 180.0f; // 左右平均が初めてこの回転量に達した値を採用
constexpr uint32_t SettleMs = 80U;     // coast 後も到達サンプルを監視し、待機後に補正する
// 補正後の目標方位へ低速で寄せる。微調整は再学習せず、一方向の1回だけ。
constexpr float AlignToleranceDeg = 1.0f;
constexpr float AlignMaxErrorDeg = 10.0f;
constexpr int AlignPower = 35;
constexpr uint32_t AlignMaxMs = 1000U;
constexpr uint32_t AlignSettleMs = 80U;
constexpr float LeftGyroStdDevDeg = 2.6f;     // 左90°旋回のジャイロ角の標準偏差（仮値）
constexpr float LeftEncoderStdDevDeg = 5.77f;  // 左90°旋回のエンコーダ角の標準偏差（仮値）
constexpr float RightGyroStdDevDeg = 2.6f;    // 右90°旋回のジャイロ角の標準偏差（仮値）
constexpr float RightEncoderStdDevDeg = 5.88f; // 右90°旋回のエンコーダ角の標準偏差（仮値）
constexpr float MinEncoderRate = 5.0f;    // 開始からの平均旋回速度 [deg/s]
constexpr float MaxRateError = 20.0f;     // 開始からの平均角速度差の上限 [deg/s]
constexpr float MaxWheelMismatch = 0.35f; // 累積 |L-R|/max(L,R)
constexpr float MaxCorrectionDeg = 5.0f; // 1旋回あたりの補正角上限
constexpr float MaxAngleErrorDeg = 15.0f; // 累積角度差の異常判定 [deg]
}

class TurnDrift {
public:
  enum Status { Accepted, WarmingUp, Invalid, Delayed, WheelMismatch,
                WrongDirection, TooSlow, RateMismatch, AngleMismatch, StatusCount };

  /* 停止後に一度呼ぶ。旋回開始から採用時点までの角度を分散に基づいて融合する。
   * 旋回間で共分散を蓄積するカルマンフィルタではなく、旋回単位の重み付け。
   * 同一旋回を重複して適用しない。 */
  float update(float gyro_angle, float l, float r, float elapsed, float direction)
  {
    using namespace TurnDriftConfig;
    if (applied_) return 0.0f;
    gain_ = 0.0f;
    if (!isfinite(gyro_angle) || !isfinite(l) || !isfinite(r) ||
        !isfinite(elapsed) || elapsed <= 0.0f || l < 0.0f || r < 0.0f ||
        (direction != -1.0f && direction != 1.0f)) {
      return reject(Invalid);
    }
    const float enc_angle = direction * 0.5f * (l + r) * 90.0f / EncoderPer90Deg;
    angle_error_ = enc_angle - gyro_angle;
    residual_ = angle_error_ - total_;
    if (l <= 0.0f || r <= 0.0f || fabsf(l-r) / fmaxf(l,r) > MaxWheelMismatch)
      return reject(WheelMismatch);
    if (direction * gyro_angle <= 0.0f) return reject(WrongDirection);
    if (fabsf(enc_angle) / elapsed < MinEncoderRate) return reject(TooSlow);
    if (fabsf(angle_error_) > MaxAngleErrorDeg) return reject(AngleMismatch);
    if (fabsf(angle_error_) / elapsed > MaxRateError) return reject(RateMismatch);

    const float scale = fmaxf(1.0f, fabsf(enc_angle) / 90.0f);
    const float mismatch = fabsf(l-r) / fmaxf(l,r);
    const float wheel_penalty = mismatch / MaxWheelMismatch;
    const float gyro_stddev = (direction < 0.0f) ? LeftGyroStdDevDeg : RightGyroStdDevDeg;
    const float encoder_stddev = (direction < 0.0f) ? LeftEncoderStdDevDeg : RightEncoderStdDevDeg;
    const float gyro_variance = gyro_stddev * gyro_stddev * scale * scale;
    const float encoder_variance = encoder_stddev * encoder_stddev *
        scale * scale * (1.0f + 9.0f * wheel_penalty * wheel_penalty);
    gain_ = gyro_variance / (gyro_variance + encoder_variance);
    const float next = fmaxf(-MaxCorrectionDeg,
                            fminf(MaxCorrectionDeg, gain_ * angle_error_));
    const float increment = next - total_;
    total_ = next;
    residual_ = angle_error_ - total_;
    rate_ = 0.0f; // 停止後の一括補正
    applied_ = true;
    status_ = Accepted;
    ++counts_[Accepted];
    return increment;
  }
  float gain() const { return gain_; }
  float rate() const { return rate_; }
  float total() const { return total_; }
  float angleError() const { return angle_error_; }
  float residual() const { return residual_; }
  Status status() const { return status_; }
  uint32_t count(Status status) const { return counts_[status]; }
private:
  float reject(Status status) {
    status_ = status;
    ++counts_[status];
    rate_ = 0.0f;
    return 0.0f; // 適用済み角度は維持
  }
  bool applied_ = false;
  float gain_ = 0.0f;
  float rate_ = 0.0f;
  float total_ = 0.0f;
  float angle_error_ = 0.0f;
  float residual_ = 0.0f;
  Status status_ = WarmingUp;
  uint32_t counts_[StatusCount] = {};
};

static_assert(TurnDriftConfig::EncoderPer90Deg > 0.0f, "encoder scale must be positive");
static_assert(TurnDriftConfig::LeftGyroStdDevDeg > 0.0f, "left gyro uncertainty must be positive");
static_assert(TurnDriftConfig::LeftEncoderStdDevDeg > 0.0f, "left encoder uncertainty must be positive");
static_assert(TurnDriftConfig::RightGyroStdDevDeg > 0.0f, "right gyro uncertainty must be positive");
static_assert(TurnDriftConfig::RightEncoderStdDevDeg > 0.0f, "right encoder uncertainty must be positive");
static_assert(TurnDriftConfig::MaxWheelMismatch > 0.0f, "wheel threshold must be positive");
#endif
