/*
 * Straight モジュール 実装
 */

#include "app.h"
#include "Gyro.h"
#include "Straight.h"

#include <math.h>
#include <stdio.h>

extern Straight gMotors;
extern Gyro     gGyro;

float Straight::clampFloat(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

int Straight::clampInt(int value, int min_value, int max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

int Straight::clampMotorPower(int power)
{
  if (power > 100) {
    return 100;
  }
  if (power < -100) {
    return -100;
  }
  return power;
}

bool Straight::samePowerRatio(int left, int right, int prev_left, int prev_right)
{
  return (int64_t)left * (int64_t)prev_right ==
         (int64_t)right * (int64_t)prev_left;
}

void Straight::resetCorrection()
{
  integral_              = 0.0f;
  target_heading_        = 0.0f;
  last_cmd_left_         = 0;
  last_cmd_right_        = 0;
  cmd_initialized_       = false;
  manual_target_heading_ = false;
}

void Straight::setTargetHeading(float heading_deg)
{
  target_heading_        = heading_deg;
  manual_target_heading_ = true;
  integral_              = 0.0f;
}

void Straight::setPowerRaw(int left_power, int right_power)
{
  pup_motor_set_power(left_, clampMotorPower(left_power));
  pup_motor_set_power(right_, clampMotorPower(right_power));

  last_cmd_left_   = left_power;
  last_cmd_right_  = right_power;
  cmd_initialized_ = true;
}

/* ヨー角 PI で左右パワーを補正して set_power する。
 *
 * 目標角 = setTargetHeading() で指定、または run() 開始時の gGyro.heading()
 * error  = wrap(現在角 − 目標角) [deg]
 *          (>0 なら左に曲がった → 右旋回方向へ補正)
 * trim   = Kp * error + Ki * integral
 * left   = pl − trim/2,  right = pr + trim/2
 *
 * 1:1 直進コマンドのみ補正。弧 (左右パワー差)・ピボット・片輪は open-loop。 */
void Straight::applyWithCorrection(int left_power, int right_power)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }

  if (left_power == 0 && right_power == 0) {
    pup_motor_set_power(left_, 0);
    pup_motor_set_power(right_, 0);
    resetCorrection();
    return;
  }

  const bool same_sign = (left_power > 0) == (right_power > 0);
  const int  apl       = (left_power  >= 0) ? left_power  : -left_power;
  const int  apr       = (right_power >= 0) ? right_power : -right_power;
  const bool straight_cmd =
      same_sign && left_power != 0 && right_power != 0 && apl == apr;

  if (!correction_enabled_ || !straight_cmd || !gGyro.isReady()) {
    integral_ = 0.0f;
    setPowerRaw(left_power, right_power);
    return;
  }

  const bool cmd_changed =
      !cmd_initialized_ ||
      left_power != last_cmd_left_ ||
      right_power != last_cmd_right_;

  if (cmd_changed) {
    const bool keep_heading =
        cmd_initialized_ &&
        samePowerRatio(left_power, right_power,
                       last_cmd_left_, last_cmd_right_);

    if (!keep_heading) {
      if (!manual_target_heading_) {
        target_heading_ = gGyro.heading();
      }
      integral_ = 0.0f;
      setPowerRaw(left_power, right_power);
      return;
    }
  }

  const float error =
      Gyro::wrapDeg180(gGyro.heading() - target_heading_);

  integral_ += error;
  integral_ = clampFloat(integral_, -i_max_, i_max_);

  const float trim = kp_ * error + ki_ * integral_;
  int         half = (int)roundf(trim / 2.0f);
  half             = clampInt(half, -MaxHeadingTrim, MaxHeadingTrim);

  const int sgnL = (left_power  >= 0) ? 1 : -1;
  const int sgnR = (right_power >= 0) ? 1 : -1;

  const int out_left  = clampMotorPower(left_power  - sgnL * half);
  const int out_right = clampMotorPower(right_power + sgnR * half);

  if (correction_debug_) {
    printf("Heading tgt=%d cur=%d err=%d int=%d trim=%d -> L=%d R=%d\n",
           (int)target_heading_, (int)gGyro.heading(), (int)error,
           (int)integral_, (int)trim, out_left, out_right);
  }

  pup_motor_set_power(left_, out_left);
  pup_motor_set_power(right_, out_right);

  last_cmd_left_  = left_power;
  last_cmd_right_ = right_power;
}

void Straight::tickCorrection()
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  if (!correction_enabled_) {
    return;
  }
  if (!cmd_initialized_) {
    return;
  }
  if (last_cmd_left_ == 0 && last_cmd_right_ == 0) {
    return;
  }
  applyWithCorrection(last_cmd_left_, last_cmd_right_);
}

void Straight::configure(pbio_port_id_t left_motor_port,
                         pbio_port_id_t right_motor_port)
{
  left_  = pup_motor_get_device(left_motor_port);
  right_ = pup_motor_get_device(right_motor_port);

  pup_motor_setup(left_, PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(right_, PUP_DIRECTION_CLOCKWISE, true);

  resetCorrection();
}

void Straight::run(int left_power, int right_power)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  applyWithCorrection(left_power, right_power);
}

void Straight::stop()
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  pup_motor_set_power(left_, 0);
  pup_motor_set_power(right_, 0);
  resetCorrection();
}

void Straight::runFor(int left_power, int right_power, uint32_t duration_ms)
{
  if (duration_ms == 0U) {
    stop();
    return;
  }

  if (duration_ms > MaxDurationMs) {
    duration_ms = MaxDurationMs;
  }

  uint32_t decel_ms = DecelMs;
  if (decel_ms > duration_ms) {
    decel_ms = duration_ms;
  }

  const uint32_t cruise_ms = duration_ms - decel_ms;

  if (cruise_ms > 0U) {
    run(left_power, right_power);
    dly_tsk((RELTIM)(cruise_ms * 1000U));
  }

  for (uint32_t t = 0U; t < decel_ms;) {
    uint32_t slice = DecelStepMs;
    if (t + slice > decel_ms) {
      slice = decel_ms - t;
    }
    const uint32_t numer = decel_ms - t;
    const int left =
        (int)((int64_t)left_power * (int64_t)numer / (int64_t)decel_ms);
    const int right =
        (int)((int64_t)right_power * (int64_t)numer / (int64_t)decel_ms);
    run(left, right);
    dly_tsk((RELTIM)(slice * 1000U));
    t += slice;
  }

  stop();
}
