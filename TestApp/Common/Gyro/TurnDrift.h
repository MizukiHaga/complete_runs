#ifndef TESTAPP_TURN_DRIFT_H_
#define TESTAPP_TURN_DRIFT_H_

#include <math.h>
#include <stdint.h>

/* CW正。両輪逆回転のその場旋回専用。 */
namespace TurnDriftConfig {
constexpr bool Enabled = true;
constexpr float EncoderPer90Deg = 180.0f;
constexpr float EncoderWeight = 0.128f;
constexpr uint32_t SettleMs = 80U;

}

class TurnDrift {
public:
  // 既存ログの状態番号を維持。
  enum Status { Accepted, WarmingUp, Invalid, Delayed, WheelMismatch,
                WrongDirection, TooSlow, RateMismatch, AngleMismatch, EncoderUnderRead, StatusCount };

  // 旋回開始からの累積角を融合。l/rは前進を正とする符号付き車輪回転量。
  // 戻り値はジャイロ角へ足す差分。
  // 周期ごとに計算し直し、前周期の補正量を積算しない。
  float estimate(float gyro_angle, float l, float r, float direction,
                 bool encoder_valid = true, uint32_t sample_ms = 0U,
                 bool direction_valid = true)
  {
    if (applied_) return total_;
    // 旧APIとの互換用。時間・方向判定では融合を止めない。
    (void)sample_ms;
    (void)direction_valid;
    gain_ = total_ = angle_error_ = residual_ = 0.0f;
    gyro_angle_ = gyro_angle;
    left_encoder_ = l;
    right_encoder_ = r;
    encoder_angle_ = 0.0f;
    if (!encoder_valid || !isfinite(gyro_angle) || !isfinite(l) || !isfinite(r) ||
        (direction != -1.0f && direction != 1.0f))
      return reject(Invalid);
    // 前進成分(l+r)ではなく左右差(l-r)から旋回角を求める。CW正。
    const float enc_angle = (0.5f * l - 0.5f * r) *
                            90.0f / TurnDriftConfig::EncoderPer90Deg;
    if (!isfinite(enc_angle)) return reject(Invalid);
    encoder_angle_ = enc_angle;
    angle_error_ = enc_angle - gyro_angle;
    if (!isfinite(angle_error_)) return reject(Invalid);
    // 小さい側も同じ重みで融合し、片側採用によるオフセットの偏りを防ぐ。
    // underは診断用の回数だけ残し、補正の採否には使わない。
    if (fabsf(enc_angle) < fabsf(gyro_angle)) ++counts_[EncoderUnderRead];
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
    (void)elapsed; // 固定比率融合では経過時間を判定に使わない。
    const float correction = estimate(gyro_angle, l, r, direction, encoder_valid,
                                      0U, direction_valid);
    applied_ = true;
    return correction;
  }
  float gain() const { return gain_; }
  float rate() const { return 0.0f; }
  float total() const { return total_; }
  float angleError() const { return angle_error_; }
  float residual() const { return residual_; }
  float gyroAngle() const { return gyro_angle_; }
  float encoderAngle() const { return encoder_angle_; }
  float leftEncoder() const { return left_encoder_; }
  float rightEncoder() const { return right_encoder_; }
  Status status() const { return status_; }
  uint32_t count(Status status) const { return counts_[status]; }
private:
  float reject(Status status) {
    status_ = status;
    ++counts_[status];
    gain_ = total_ = angle_error_ = residual_ = 0.0f;
    return 0.0f;
  }
  bool applied_ = false;
  float gain_ = 0.0f;
  float total_ = 0.0f;
  float angle_error_ = 0.0f;
  float residual_ = 0.0f;
  float gyro_angle_ = 0.0f;
  float encoder_angle_ = 0.0f;
  float left_encoder_ = 0.0f;
  float right_encoder_ = 0.0f;
  Status status_ = WarmingUp;
  uint32_t counts_[StatusCount] = {};
};
static_assert(TurnDriftConfig::EncoderPer90Deg > 0.0f, "encoder scale must be positive");
static_assert(TurnDriftConfig::EncoderWeight >= 0.0f &&
              TurnDriftConfig::EncoderWeight <= 1.0f, "weight must be in [0,1]");
#endif
