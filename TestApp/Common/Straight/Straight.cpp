/*
 * Straight モジュール 実装
 */

#include "app.h"
#include "Gyro.h"
#include "Straight.h"
#include "kernel_cfg.h"

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

bool Straight::isSameStraightPower(int left_power, int right_power)
{
  const int apl = (left_power >= 0) ? left_power : -left_power;
  const int apr = (right_power >= 0) ? right_power : -right_power;
  const bool same_sign = (left_power > 0) == (right_power > 0);
  return same_sign && left_power != 0 && right_power != 0 && apl == apr;
}

int32_t Straight::absEncoderDeg(int32_t count)
{
  return (count < 0) ? -count : count;
}

bool Straight::wheelEncoderReached(int32_t count, int power, int32_t target_deg)
{
  const int32_t target = absEncoderDeg(target_deg);
  if (target <= 0) {
    return true;
  }
  if (power == 0) {
    return true;
  }
  if (power > 0 && count >= target) {
    return true;
  }
  if (power < 0 && count <= -target) {
    return true;
  }
  /* モータ向き設定により符号が逆のときは絶対値で判定 */
  return absEncoderDeg(count) >= target;
}

void Straight::syncStoppedState()
{
  correction_enabled_    = false;
  last_cmd_left_         = 0;
  last_cmd_right_        = 0;
  cmd_initialized_       = false;
  manual_target_heading_ = false;
  integral_              = 0.0f;
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

void Straight::setBalancePid(float kp, float ki, float i_max)
{
  kp_       = kp;
  ki_       = ki;
  i_max_    = i_max;
  integral_ = 0.0f;
}

void Straight::setHeadingCorrection(bool enabled)
{
  heading_correction_ = enabled;
  if (!enabled) {
    integral_ = 0.0f;
  }
}

void Straight::setPowerRaw(int left_power, int right_power)
{
  /* stop() との競合: 停止中に周期補正が古い指令を書き戻さない */
  if (!correction_enabled_) {
    pup_motor_set_power(left_, 0);
    pup_motor_set_power(right_, 0);
    return;
  }

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
 *          本機は右 = +。error>0（右にずれた）→ 左へ戻す
 *          ＝ 左輪を弱く / 右輪を強く (L −, R +)
 * trim   = Kp * error + Ki * integral
 * left   = pl − trim配分,  right = pr + trim配分
 *        （後退も同じ式。負パワーなら L−で絶対値増＝後退強、R+で絶対値減）
 *
 * 1:1 直進コマンドのみ補正。弧 (左右パワー差)・ピボット・片輪は open-loop。 */
void Straight::applyWithCorrection(int left_power, int right_power)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }

  if (!correction_enabled_) {
    pup_motor_set_power(left_, 0);
    pup_motor_set_power(right_, 0);
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

  /* isReady は「静止校正済み」判定であり、heading 取得可否ではない。
   * ここで弾くと PI が常に無効になり、Kp を変えても挙動が変わらない。 */
  if (!correction_enabled_ || !heading_correction_ || !straight_cmd) {
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
      /* ピボット等 → 直進へ切替時は raw のみ。1:1 直進なら PI へ続行。 */
      if (!straight_cmd) {
        setPowerRaw(left_power, right_power);
        return;
      }
    }
  }

  const float error =
      Gyro::wrapDeg180(gGyro.heading() - target_heading_);

  integral_ += error;
  integral_ = clampFloat(integral_, -i_max_, i_max_);

  const float trim = kp_ * error + ki_ * integral_;
  /* 整数化で消えないよう、|trim| がごく小さくても少なくとも ±1 を出す */
  int diff = (int)roundf(trim);
  if (diff == 0 && fabsf(trim) > 0.0f) {
    diff = (trim > 0.0f) ? 1 : -1;
  }
  diff = clampInt(diff, -MaxHeadingTrim, MaxHeadingTrim);
  const int dL = diff / 2;
  const int dR = diff - dL;  /* |diff|==1 のとき片側に ±1 が残る */

  /* error>0（右ずれ）→ 左旋回: 前進は L弱/R強、後退は L強/R弱（同じ式で負パワーが効く） */
  const int out_left  = clampMotorPower(left_power - dL);
  const int out_right = clampMotorPower(right_power + dR);

  if (correction_debug_) {
    if (++correction_debug_div_ >= 10U) {
      correction_debug_div_ = 0;
      printf("Heading tgt=%d cur=%d err=%d trim=%d diff=%d kp=%d -> L=%d R=%d\n",
             (int)target_heading_, (int)gGyro.heading(), (int)error,
             (int)trim, diff, (int)(kp_ * 100.0f), out_left, out_right);
    }
  }

  /* stop() が計算中に入った場合、古い補正結果を捨てる */
  if (!correction_enabled_) {
    pup_motor_set_power(left_, 0);
    pup_motor_set_power(right_, 0);
    return;
  }

  pup_motor_set_power(left_, out_left);
  pup_motor_set_power(right_, out_right);

  last_cmd_left_  = left_power;
  last_cmd_right_ = right_power;
  cmd_initialized_ = true;
}

void Straight::tickCorrection()
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  if (!correction_enabled_ || !heading_correction_) {
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

void Straight::resetEncoderCount()
{
  if (left_ != nullptr) {
    pup_motor_brake(left_);
  }
  if (right_ != nullptr) {
    pup_motor_brake(right_);
  }
  if (left_ != nullptr) {
    pup_motor_reset_count(left_);
  }
  if (right_ != nullptr) {
    pup_motor_reset_count(right_);
  }
}

int32_t Straight::straightEncoderDeg(int left_power, int right_power) const
{
  if (left_ == nullptr || right_ == nullptr) {
    return 0;
  }
  const int32_t lc = pup_motor_get_count(left_);
  const int32_t rc = pup_motor_get_count(right_);

  if (isSameStraightPower(left_power, right_power)) {
    return (absEncoderDeg(lc) + absEncoderDeg(rc)) / 2;
  }

  return (absEncoderDeg(lc) + absEncoderDeg(rc)) / 2;
}

bool Straight::straightEncoderReached(int left_power, int right_power,
                                      int32_t target_deg) const
{
  if (left_ == nullptr || right_ == nullptr) {
    return false;
  }
  if (target_deg <= 0) {
    return true;
  }
  const int32_t lc = pup_motor_get_count(left_);
  const int32_t rc = pup_motor_get_count(right_);

  if (isSameStraightPower(left_power, right_power)) {
    return (absEncoderDeg(lc) + absEncoderDeg(rc)) / 2 >= absEncoderDeg(target_deg);
  }

  return wheelEncoderReached(lc, left_power, target_deg) &&
         wheelEncoderReached(rc, right_power, target_deg);
}

void Straight::beginBalancedEncoderRun()
{
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  correction_enabled_ = true;
  if (left_ != nullptr) {
    pup_motor_brake(left_);
  }
  if (right_ != nullptr) {
    pup_motor_brake(right_);
  }
  if (left_ != nullptr) {
    pup_motor_reset_count(left_);
  }
  if (right_ != nullptr) {
    pup_motor_reset_count(right_);
  }
}

void Straight::endBalancedEncoderRun()
{
  if (left_ != nullptr) {
    pup_motor_brake(left_);
  }
  if (right_ != nullptr) {
    pup_motor_brake(right_);
  }
  syncStoppedState();
}

void Straight::runBalancedEncoderFor(int left_power, int right_power,
                                     int32_t target_deg, uint32_t step_ms)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  if (target_deg <= 0) {
    return;
  }
  if (step_ms == 0U) {
    step_ms = 1U;
  }

  const int l_pwr = clampMotorPower(left_power);
  const int r_pwr = clampMotorPower(right_power);

  beginBalancedEncoderRun();
  applyWithCorrection(l_pwr, r_pwr);

  while (!straightEncoderReached(l_pwr, r_pwr, target_deg)) {
    applyWithCorrection(l_pwr, r_pwr);
    dly_tsk((RELTIM)(step_ms * 1000U));
  }

  endBalancedEncoderRun();
}

void Straight::configure(pbio_port_id_t left_motor_port,
                         pbio_port_id_t right_motor_port)
{
  left_  = pup_motor_get_device(left_motor_port);
  right_ = pup_motor_get_device(right_motor_port);

  pup_motor_setup(left_, PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(right_, PUP_DIRECTION_CLOCKWISE, true);

  correction_enabled_ = true;
  resetCorrection();
}

void Straight::run(int left_power, int right_power)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  correction_enabled_ = true;
  applyWithCorrection(left_power, right_power);
}

void Straight::stop()
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  /* 先に周期補正を止め、計算途中の apply が指令を復活させない */
  correction_enabled_ = false;
  last_cmd_left_         = 0;
  last_cmd_right_        = 0;
  cmd_initialized_       = false;
  manual_target_heading_ = false;
  integral_              = 0.0f;
  target_heading_        = 0.0f;
  pup_motor_set_power(left_, 0);
  pup_motor_set_power(right_, 0);
}

bool Straight::encoderReached(int left_power, int right_power, int32_t target_deg)
{
  if (left_ == nullptr || right_ == nullptr) {
    return false;
  }
  const int32_t lc = pup_motor_get_count(left_);
  const int32_t rc = pup_motor_get_count(right_);

  if (isSameStraightPower(left_power, right_power)) {
    const int32_t l_abs = absEncoderDeg(lc);
    const int32_t r_abs = absEncoderDeg(rc);
    /* 1:1 直進: 左右の回転量平均で距離判定（エンコーダのみ） */
    return (l_abs + r_abs) / 2 >= absEncoderDeg(target_deg);
  }

  return wheelEncoderReached(lc, left_power, target_deg) &&
         wheelEncoderReached(rc, right_power, target_deg);
}

void Straight::runFor(int left_power, int right_power, int32_t target_deg)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  if (target_deg <= 0 || (left_power == 0 && right_power == 0)) {
    stop();
    return;
  }

  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);

  const bool prev_heading_correction = heading_correction_;
  heading_correction_ = false;

  const int l_pwr = clampMotorPower(left_power);
  const int r_pwr = clampMotorPower(right_power);

  pup_motor_brake(left_);
  pup_motor_brake(right_);
  pup_motor_reset_count(left_);
  pup_motor_reset_count(right_);
  pup_motor_set_power(left_, l_pwr);
  pup_motor_set_power(right_, r_pwr);

  uint32_t safety_ms = 0U;
  while (safety_ms < EncoderRunMaxMs) {
    if (encoderReached(l_pwr, r_pwr, target_deg)) {
      break;
    }
    dly_tsk(EncoderStepMs * 1000U);
    safety_ms += EncoderStepMs;
  }

  pup_motor_brake(left_);
  pup_motor_brake(right_);
  syncStoppedState();
  heading_correction_ = prev_heading_correction;
}

void Straight::runForMs(int left_power, int right_power, uint32_t duration_ms)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
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

void Straight::turnToHeading(float target_heading, float tolerance_deg,
                             int left_power, int right_power, uint32_t max_ms)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }

  /* 開始時点の最短角 delta を固定し、その方向へ |delta| 回る。
   * 毎周期 wrap(target - heading) すると、アンラップ済み heading と
   * ±180 目標が 360° ずれて差 0 になり、モータが動かない。 */
  float prev = gGyro.heading();
  const float delta    = Gyro::wrapDeg180(target_heading - prev);
  const float turn_dir = (delta > 0.0f) ? 1.0f : -1.0f;
  const float need     = fabsf(delta);
  float       traveled = 0.0f;

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < max_ms) {
    const float h = gGyro.heading();
    traveled += turn_dir * Gyro::wrapDeg180(h - prev);
    prev = h;

    if ((need - traveled) <= tolerance_deg) {
      break;
    }

    /* ピボットは 1:1 直進ではないので run() 内で補正はかからない */
    if (turn_dir > 0.0f) {
      run(left_power, -right_power);
    } else {
      run(-left_power, right_power);
    }

    dly_tsk(TurnStepMs * 1000U);
    elapsed_ms += TurnStepMs;
  }

  stop();
  dly_tsk(TurnStepMs * 1000U);
}
