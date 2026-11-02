#ifndef TESTAPP_TURN_DRIFT_H_
#define TESTAPP_TURN_DRIFT_H_

#include <math.h>
#include <stdint.h>

/* CW正、車輪回転は前進正。符号付き左右差から旋回角を求める。 */
namespace TurnDriftConfig {
constexpr bool Enabled = true;
constexpr float EncoderPer90Deg = 180.0f;
constexpr float EncoderWeightLeft = 0.15f;  // 左旋回(CCW)
constexpr float EncoderWeightRight = 0.145f; // 右旋回(CW)
constexpr uint32_t SettleMs = 80U;

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
    (void)sample_ms;
    (void)direction_valid;
    gain_ = total_ = angle_error_ = residual_ = 0.0f;
    if (!encoder_valid || !isfinite(gyro_angle) || !isfinite(l) || !isfinite(r) ||
        (direction != -1.0f && direction != 1.0f))
      return reject(Invalid);
    const float enc_angle = 0.5f * (l-r) *
                            90.0f / TurnDriftConfig::EncoderPer90Deg;
    angle_error_ = enc_angle - gyro_angle;
    // 旋回指令の向きで重みを選ぶ。車輪角の符号は実測値を維持する。
    gain_ = direction < 0.0f ? TurnDriftConfig::EncoderWeightLeft
                             : TurnDriftConfig::EncoderWeightRight;
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
    (void)elapsed;
    const float correction = estimate(gyro_angle, l, r, direction, encoder_valid,
                                      0U,
                                      direction_valid);
    applied_ = status_ == Accepted;
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
  float reject(Status status) {
    status_ = status;
    ++counts_[status];
    gain_ = total_ = 0.0f;
    return 0.0f;
  }
  bool applied_ = false;
  float gain_ = 0.0f;
  float total_ = 0.0f;
  float angle_error_ = 0.0f;
  float residual_ = 0.0f;
  Status status_ = WarmingUp;
  uint32_t counts_[StatusCount] = {};
};
static_assert(TurnDriftConfig::EncoderPer90Deg > 0.0f, "encoder scale must be positive");
static_assert(TurnDriftConfig::EncoderWeightLeft >= 0.0f &&
              TurnDriftConfig::EncoderWeightLeft <= 1.0f &&
              TurnDriftConfig::EncoderWeightRight >= 0.0f &&
              TurnDriftConfig::EncoderWeightRight <= 1.0f,
              "left/right weights must be in [0,1]");
#endif
