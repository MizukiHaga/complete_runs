/*
 * Straight モジュール 実装
 */

#include "app.h"
#include "Gyro.h"
#include "Straight.h"
#include "kernel_cfg.h"

#include <math.h>
#include <stdio.h>

/* pup_motor_get_count() は pbio のサーボ更新ループが動いているときしか値を返さない。
 * set_power (dcmotor duty) を呼ぶとサーボが親から切り離されて更新ループが止まり、
 * 以降 get_count は常に 0 になる。走行中に距離を測るため回転センサを直接読む。 */
extern "C" {
#include <pbio/tacho.h>
}

extern Straight gMotors;
extern Gyro     gGyro;

/* 回転センサの角度 [deg]。取得できなければ 0。 */
static int32_t tachoDeg(pup_motor_t *motor)
{
  if (motor == nullptr) {
    return 0;
  }
  pbio_angle_t angle;
  if (pbio_tacho_get_angle(motor->tacho, &angle) != PBIO_SUCCESS) {
    return 0;
  }
  return (angle.rotations * 360) + (angle.millidegrees / 1000);
}

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
  correction_enabled_ = false;
  last_cmd_left_      = 0;
  last_cmd_right_     = 0;
  cmd_initialized_    = false;
  integral_           = 0.0f;
  /* 目標角は残す。捨てると次の run() が現在角を目標に掴み、
   * 走り出しのずれ角のまま直進する */
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

// 直進補正のpid設定
void Straight::setBalancePid(float kp, float ki, float i_max, float deadband_deg)
{
  kp_            = kp;
  ki_            = ki;
  i_max_         = i_max;
  deadband_deg_  = (deadband_deg < 0.0f) ? 0.0f : deadband_deg;
  integral_      = 0.0f;
}

// 直進補正のオン・オフ
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
 * 目標角 = setTargetHeading()（未指定時は 0°。現在角は掴み直さない）
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
    /* 指令だけ落とす。目標角は resetCorrection しない（掴み直し防止） */
    correction_enabled_ = false;
    last_cmd_left_      = 0;
    last_cmd_right_     = 0;
    cmd_initialized_    = false;
    integral_           = 0.0f;
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
      /* 現在角を目標に掴み直さない。
       * 走り出しのヨーずれを目標にすると、ずれた向きのまま直進する。
       * 目標は setTargetHeading() か、既定の target_heading_(0°) を使う。 */
      integral_ = 0.0f;
      /* ピボット等 → 直進へ切替時は raw のみ。1:1 直進なら PI へ続行。 */
      if (!straight_cmd) {
        setPowerRaw(left_power, right_power);
        return;
      }
    }
  }

  float error = Gyro::wrapDeg180(gGyro.heading() - target_heading_);

  /* 不感帯: ジャイロの微小揺れで trim を出し続けない。
   * 帯内では I を減衰させ、補正がマスをまたいで長く残らないようにする。 */
  if (deadband_deg_ > 0.0f && fabsf(error) <= deadband_deg_) {
    error = 0.0f;
    integral_ *= 0.90f;
    if (fabsf(integral_) < 0.5f) {
      integral_ = 0.0f;
    }
  } else {
    integral_ += error;
    integral_ = clampFloat(integral_, -i_max_, i_max_);
  }

  const float trim = kp_ * error + ki_ * integral_;
  int diff = (int)roundf(trim);
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
  /* reset_count が効いても効かなくても、ここを 0 点として扱えるようにする。 */
  enc_zero_left_  = tachoDeg(left_);
  enc_zero_right_ = tachoDeg(right_);
}

int32_t Straight::leftEncoderCount() const
{
  return tachoDeg(left_);
}

int32_t Straight::rightEncoderCount() const
{
  return tachoDeg(right_);
}

int32_t Straight::traveledDegFrom(int32_t left_base, int32_t right_base) const
{
  const int32_t left_diff  = tachoDeg(left_)  - left_base;
  const int32_t right_diff = tachoDeg(right_) - right_base;
  return (absEncoderDeg(left_diff) + absEncoderDeg(right_diff)) / 2;
}

int32_t Straight::straightEncoderDeg(int left_power, int right_power) const
{
  if (left_ == nullptr || right_ == nullptr) {
    return 0;
  }
  (void)left_power;
  (void)right_power;
  return traveledDegFrom(enc_zero_left_, enc_zero_right_);
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
  const int32_t lc = tachoDeg(left_)  - enc_zero_left_;
  const int32_t rc = tachoDeg(right_) - enc_zero_right_;

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
  enc_zero_left_  = tachoDeg(left_);
  enc_zero_right_ = tachoDeg(right_);
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
  /* begin で止めた周期補正を戻す。戻さないと、以降の runForMs のように
   * 指令を 1 回置いて待つ直進でヨー補正が更新されなくなる。 */
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);
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

void Straight::run(int left_power, int right_power, int32_t encoder_deg)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  correction_enabled_ = true;
  applyWithCorrection(left_power, right_power);

  if (encoder_deg <= 0) {
    return;
  }

  const int32_t left_base  = leftEncoderCount();
  const int32_t right_base = rightEncoderCount();
  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < MaxDurationMs) {
    if (traveledDegFrom(left_base, right_base) >= encoder_deg) {
      break;
    }
    tickCorrection();
    dly_tsk(EncoderStepMs * 1000U);
    elapsed_ms += EncoderStepMs;
  }
  stop();
}

void Straight::stop()
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  /* 先に周期補正を止め、計算途中の apply が指令を復活させない */
  correction_enabled_ = false;
  last_cmd_left_      = 0;
  last_cmd_right_     = 0;
  cmd_initialized_    = false;
  integral_           = 0.0f;
  /* 目標角は残す（次 run で現在角を目標に掴み直さない） */
  pup_motor_set_power(left_, 0);
  pup_motor_set_power(right_, 0);
}

bool Straight::encoderReached(int left_power, int right_power, int32_t target_deg)
{
  if (left_ == nullptr || right_ == nullptr) {
    return false;
  }
  const int32_t lc = tachoDeg(left_)  - enc_zero_left_;
  const int32_t rc = tachoDeg(right_) - enc_zero_right_;

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

  resetEncoderCount();
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

void Straight::runForMs(int left_power, int right_power, uint32_t duration_ms,
                        uint32_t decel_ms, uint32_t step_ms, bool coast_end)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  if (duration_ms == 0U) {
    if (coast_end) {
      coast();
    } else {
      stop();
    }
    return;
  }

  if (duration_ms > MaxDurationMs) {
    duration_ms = MaxDurationMs;
  }

  if (decel_ms > duration_ms) {
    decel_ms = duration_ms;
  }

  if (step_ms == 0U) {
    step_ms = DecelStepMs;
  }

  const uint32_t cruise_ms = duration_ms - decel_ms;

  if (cruise_ms > 0U) {
    run(left_power, right_power);
    dly_tsk((RELTIM)(cruise_ms * 1000U));
  }

  for (uint32_t t = 0U; t < decel_ms;) {
    uint32_t slice = step_ms;
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

  if (coast_end) {
    coast();
  } else {
    stop();
  }
}

void Straight::decelToStop(uint32_t decel_ms, uint32_t step_ms, bool coast_end)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  const int lp = last_cmd_left_;
  const int rp = last_cmd_right_;
  if (!correction_enabled_ || !cmd_initialized_ || decel_ms == 0U ||
      (lp == 0 && rp == 0) || !isSameStraightPower(lp, rp)) {
    if (coast_end) {
      coast();
    } else {
      stop();
    }
    return;
  }
  runForMs(lp, rp, decel_ms, decel_ms, step_ms, coast_end);
}

void Straight::coast()
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  correction_enabled_ = false;
  last_cmd_left_      = 0;
  last_cmd_right_     = 0;
  cmd_initialized_    = false;
  integral_           = 0.0f;
  /* 目標角は残す。coast のたびに捨てると、次の 1:1 run が
   * 走り出し直後の現在角を目標にしてしまい、ずれた向きのまま直進する */
  pup_motor_stop(left_);
  pup_motor_stop(right_);
}

void Straight::turnToHeading(float target_heading, float tolerance_deg,
                             int left_power, int right_power, uint32_t max_ms,
                             float slow_approach_deg, int slow_power,
                             float stop_early_deg, bool coast_end,
                             int prefer_sign)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }

  /* 開始時の目標差分を固定し、経過角で打ち切る。
   * prefer_sign=0 なら最短側。±1 ならその向きに強制し、必要なら遠回りする。
   * 残りが slow_approach_deg 以内なら低速（行き過ぎ防止）。
   * その場旋回は両輪逆符号（L+/R− または L−/R+）。
   * 左右のエンコーダ回転量が揃うようパワーを補正する。 */
  float prev = gGyro.heading();
  const float delta = Gyro::wrapDeg180(target_heading - prev);
  if (fabsf(delta) <= tolerance_deg) {
    if (coast_end) {
      coast();
    } else {
      stop();
    }
    dly_tsk(TurnStepMs * 1000U);
    return;
  }

  float turn_dir;
  float need;
  if (prefer_sign > 0 || prefer_sign < 0) {
    const float forced = (prefer_sign > 0) ? 1.0f : -1.0f;
    if (forced * delta > 0.0f) {
      /* 最短が計画どおり */
      turn_dir = forced;
      need     = fabsf(delta);
    } else if (fabsf(delta) < 45.0f) {
      /* 目標をわずかに通り過ぎているだけ。逆向きに大旋回せず最短で合わせる */
      turn_dir = (delta > 0.0f) ? 1.0f : -1.0f;
      need     = fabsf(delta);
    } else {
      /* 最短が計画と逆で角も大きい → 計画方向へ遠回り */
      turn_dir = forced;
      need     = 360.0f - fabsf(delta);
    }
  } else {
    turn_dir = (delta > 0.0f) ? 1.0f : -1.0f;
    need     = fabsf(delta);
  }
  float       traveled = 0.0f;
  float       early    = stop_early_deg;
  if (early < 0.0f) {
    early = 0.0f;
  }
  if (early > need) {
    early = 0.0f;
  }

  const int32_t enc_l0 = tachoDeg(left_);
  const int32_t enc_r0 = tachoDeg(right_);
  const bool    prev_heading_correction = heading_correction_;
  /* 旋回中は直進ヨー補正を掛けない（ピボット指令を崩さない） */
  heading_correction_ = false;

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < max_ms) {
    const float h = gGyro.heading();
    traveled += turn_dir * Gyro::wrapDeg180(h - prev);
    prev = h;

    const float rem = need - traveled;
    if (rem <= early) {
      break;
    }

    int base = left_power;
    if (right_power < base) {
      base = right_power;
    }
    if (base < 0) {
      base = -base;
    }
    if (slow_approach_deg > 0.0f && rem <= slow_approach_deg) {
      base = slow_power;
      if (base < 0) {
        base = -base;
      }
    }
    if (base < TurnEncBalanceMinPower) {
      base = TurnEncBalanceMinPower;
    }

    /* |ΔL| と |ΔR| を揃える。進みすぎた側のパワーを落とす */
    const int32_t l_abs =
        absEncoderDeg(tachoDeg(left_) - enc_l0);
    const int32_t r_abs =
        absEncoderDeg(tachoDeg(right_) - enc_r0);
    const int32_t enc_diff = l_abs - r_abs;

    int l_mag = base;
    int r_mag = base;
    if (enc_diff > TurnEncBalanceDeadbandDeg) {
      const int corr = clampInt(
          static_cast<int>(TurnEncBalanceKp *
                           static_cast<float>(enc_diff - TurnEncBalanceDeadbandDeg)),
          0, TurnEncBalanceMaxCorr);
      l_mag = clampInt(base - corr, TurnEncBalanceMinPower, base);
    } else if (enc_diff < -TurnEncBalanceDeadbandDeg) {
      const int corr = clampInt(
          static_cast<int>(TurnEncBalanceKp *
                           static_cast<float>(-enc_diff - TurnEncBalanceDeadbandDeg)),
          0, TurnEncBalanceMaxCorr);
      r_mag = clampInt(base - corr, TurnEncBalanceMinPower, base);
    }

    if (turn_dir > 0.0f) {
      run(l_mag, -r_mag);
    } else {
      run(-l_mag, r_mag);
    }

    dly_tsk(TurnStepMs * 1000U);
    elapsed_ms += TurnStepMs;
  }

  heading_correction_ = prev_heading_correction;

  if (coast_end) {
    coast();
  } else {
    stop();
  }
  dly_tsk(TurnStepMs * 1000U);
}
