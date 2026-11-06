/*
 * Straight モジュール 実装
 */

#include "app.h"
#include "Gyro.h"
#include "TurnDrift.h"
#include "Straight.h"
#include "kernel_cfg.h"

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

/* pup_motor_get_count() は pbio のサーボ更新ループが動いているときしか値を返さない。
 * set_power (dcmotor duty) を呼ぶとサーボが親から切り離されて更新ループが止まり、
 * 以降 get_count は常に 0 になる。走行中に距離を測るため回転センサを直接読む。 */
extern "C" {
#include <pbio/tacho.h>
#include <spike/hub/battery.h>
#include <serial/newlib.h>
#include <serial/serial.h>
#include <syssvc/serial.h>
}

extern Straight gMotors;
extern Gyro     gGyro;
extern FILE     *gBluetoothOut;

static constexpr uint32_t KimeSettleMs = 100U;
static constexpr size_t HeadingLogBufferSize = 16384U;
static char heading_log_buffer[HeadingLogBufferSize];
static size_t heading_log_length = 0U;
static uint32_t heading_log_dropped = 0U;

static void appendHeadingLog(const char *format, ...)
{
  if (heading_log_length >= HeadingLogBufferSize - 1U) {
    ++heading_log_dropped;
    return;
  }

  va_list args;
  va_start(args, format);
  const int written = vsnprintf(
      heading_log_buffer + heading_log_length,
      HeadingLogBufferSize - heading_log_length,
      format, args);
  va_end(args);
  if (written < 0) {
    return;
  }

  const size_t available = HeadingLogBufferSize - heading_log_length - 1U;
  if (static_cast<size_t>(written) > available) {
    heading_log_length += available;
    heading_log_buffer[heading_log_length] = '\0';
    ++heading_log_dropped;
    return;
  }
  heading_log_length += static_cast<size_t>(written);
}

/* 回転センサの角度 [deg]。取得できなければ 0。 */
static int32_t tachoDeg(pup_motor_t *motor, bool *valid = nullptr)
{
  if (valid != nullptr) *valid = false;
  if (motor == nullptr) {
    return 0;
  }
  pbio_angle_t angle;
  if (pbio_tacho_get_angle(motor->tacho, &angle) != PBIO_SUCCESS) {
    return 0;
  }
  if (valid != nullptr) *valid = true;
  return (angle.rotations * 360) + (angle.millidegrees / 1000);
}

void Straight::setDebug(bool enabled)
{
  correction_debug_ = enabled;
  sync_debug_ = enabled;
  heading_debug_div_ = 0;
  if (enabled && gBluetoothOut == nullptr) {
    serial_opn_por(SIO_BLUETOOTH_PORTID);
    gBluetoothOut = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);
  }
}

void Straight::setHeadingDebug(bool enabled)
{
  heading_debug_enabled_ = enabled;
  heading_debug_div_ = 0;
  if (enabled && gBluetoothOut == nullptr) {
    serial_opn_por(SIO_BLUETOOTH_PORTID);
    gBluetoothOut = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);
  }
}

void Straight::setSyncDebug(bool enabled)
{
  sync_debug_ = enabled;
  sync_debug_div_ = 0;
  if (enabled && gBluetoothOut == nullptr) {
    serial_opn_por(SIO_BLUETOOTH_PORTID);
    gBluetoothOut = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);
  }
}

void Straight::shutdownDebugOutput()
{
  /* 出力を止めてから FILE を閉じ、以後のデバッグ出力が解放済み
   * Bluetooth FILE を参照しないようにする。 */
  heading_debug_enabled_ = false;
  correction_debug_ = false;
  sync_debug_ = false;
  heading_debug_div_ = 0;
  correction_debug_div_ = 0;
  sync_debug_div_ = 0;
  flushHeadingDebugLog();

  FILE *output = gBluetoothOut;
  gBluetoothOut = nullptr;
  if (output != nullptr) {
    fflush(output);
    fclose(output);
  }
  serial_cls_por(SIO_BLUETOOTH_PORTID);
}

void Straight::setHeadingDebugPhase(const char *phase)
{
  heading_debug_phase_ = (phase != nullptr) ? phase : "unknown";
}

void Straight::debugHeading()
{
  if (!heading_debug_enabled_) {
    return;
  }

  if (++heading_debug_div_ < 100U) {
    return;
  }
  heading_debug_div_ = 0;
  appendHeadingLog("Heading,Phase=%s,H=%.2f,T=%.2f,E=%.2f\n",
                   heading_debug_phase_, gGyro.heading(), target_heading_,
                   Gyro::wrapDeg180(gGyro.heading() - target_heading_));
}

void Straight::flushHeadingDebugLog()
{
  if (heading_log_length == 0U && heading_log_dropped == 0U) {
    return;
  }

  FILE *output = gBluetoothOut != nullptr ? gBluetoothOut : stdout;
  if (heading_log_length > 0U) {
    fwrite(heading_log_buffer, 1U, heading_log_length, output);
  }
  if (heading_log_dropped > 0U) {
    fprintf(output, "Heading,log-dropped=%lu\n",
            (unsigned long)heading_log_dropped);
  }
  fflush(output);
  heading_log_length = 0U;
  heading_log_dropped = 0U;
  heading_log_buffer[0] = '\0';
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

float Straight::readHeading() const
{
  return heading_reader_ ? heading_reader_() : gGyro.heading();
}

float Straight::readRawHeading() const
{
  return raw_heading_reader_ ? raw_heading_reader_() : gGyro.rawHeading();
}

void Straight::turnWheelMags(int base, int32_t enc_diff, int *l_mag, int *r_mag)
{
  int l = base;
  int r = base;
  if (enc_diff > TurnEncBalanceDeadbandDeg) {
    const int corr = clampInt(
        static_cast<int>(TurnEncBalanceKp *
                         static_cast<float>(enc_diff - TurnEncBalanceDeadbandDeg)),
        0, TurnEncBalanceMaxCorr);
    l = clampInt(base - corr, TurnEncBalanceMinPower, base);
  } else if (enc_diff < -TurnEncBalanceDeadbandDeg) {
    const int corr = clampInt(
        static_cast<int>(TurnEncBalanceKp *
                         static_cast<float>(-enc_diff - TurnEncBalanceDeadbandDeg)),
        0, TurnEncBalanceMaxCorr);
    r = clampInt(base - corr, TurnEncBalanceMinPower, base);
  }
  /* 同調P補正後、左輪のパワー絶対値へ加算する。モータ上限は100。 */
  l = clampInt(l + TurnLeftPowerTrim, TurnEncBalanceMinPower, 100);
  *l_mag = l;
  *r_mag = r;
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
void Straight::setBalancePid(float kp, float ki, float i_max,
                             float deadband_deg, float kd)
{
  kp_            = kp;
  ki_            = ki;
  kd_            = kd;
  i_max_         = i_max;
  deadband_deg_  = (deadband_deg < 0.0f) ? 0.0f : deadband_deg;
  integral_      = 0.0f;
  heading_correction_scale_ = 1.0f;
}

void Straight::setHeadingTrimLimit(int limit)
{
  heading_trim_limit_ = clampInt(limit, 0, 100);
}

void Straight::setHeadingRunPowerScale(float scale)
{
  heading_run_power_scale_ = clampFloat(scale, 0.2f, 1.0f);
}

void Straight::setHeadingCorrectionScale(float scale)
{
  heading_correction_scale_ = clampFloat(scale, 0.0f, 10.0f);
}

// 直進補正のオン・オフ
void Straight::setHeadingCorrection(bool enabled)
{
  heading_correction_ = enabled;
  if (!enabled) {
    integral_ = 0.0f;
  }
}

void Straight::setEncoderStraightBalance(bool enabled)
{
  if (enabled == encoder_straight_) {
    return;
  }
  encoder_straight_ = enabled;
  integral_         = 0.0f;
  if (enabled && left_ != nullptr && right_ != nullptr) {
    enc_bal_left_  = tachoDeg(left_);
    enc_bal_right_ = tachoDeg(right_);
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

  const int out_l = clampMotorPower(left_power);
  const int out_r = clampMotorPower(right_power);
  pup_motor_set_power(left_, out_l);
  pup_motor_set_power(right_, out_r);
  if (out_l != 0 || out_r != 0) {
    last_out_left_  = out_l;
    last_out_right_ = out_r;
  }

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
  if (!correction_enabled_ || !heading_correction_ ||
      (!straight_cmd && !allow_heading_correction_for_ratio_)) {
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
      if (encoder_straight_) {
        enc_bal_left_  = tachoDeg(left_);
        enc_bal_right_ = tachoDeg(right_);
      }
      /* ピボット等 → 直進へ切替時は raw のみ。1:1 直進なら PI へ続行。 */
      if (!straight_cmd) {
        setPowerRaw(left_power, right_power);
        return;
      }
    }
  }

  float error = 0.0f;
  int   travel_sign = 1;
  if (encoder_straight_) {
    /* 進行方向の左右進み差。左が多いと正 → 左を弱める */
    travel_sign = (left_power >= 0) ? 1 : -1;
    const int32_t tL =
        travel_sign * (tachoDeg(left_) - enc_bal_left_);
    const int32_t tR =
        travel_sign * (tachoDeg(right_) - enc_bal_right_);
    error = static_cast<float>(tL - tR);
  } else {
    error = Gyro::wrapDeg180(readHeading() - target_heading_);
  }

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
  /* ヨー: error>0（右ずれ）→ 左旋回。前進は L弱/R強、後退は同じ式で負パワーが効く。
   * エンコーダ: 左が進みすぎ → 進行方向の左マグを落とす（後退は符号を反転）。 */
  const int signed_diff = encoder_straight_ ? travel_sign * diff : diff;
  // 奇数の補正も、符号反転で左右の出力が入れ替わるように配分する。
  const int dL = signed_diff > 0 ? (signed_diff + 1) / 2 : signed_diff / 2;
  const int dR = signed_diff - dL;
  int out_left = left_power - dL;
  int out_right = right_power + dR;
  // 片輪だけを飽和させると補正差が失われる。両輪を同量ずらして差を保つ。
  const int high = out_left > out_right ? out_left : out_right;
  if (high > 100) {
    out_left -= high - 100;
    out_right -= high - 100;
  }
  const int low = out_left < out_right ? out_left : out_right;
  if (low < -100) {
    out_left += -100 - low;
    out_right += -100 - low;
  }

  if (correction_debug_) {
    if (++correction_debug_div_ >= 10U) {
      correction_debug_div_ = 0;
      printf("Heading tgt=%d cur=%d err=%d trim=%d diff=%d kp=%d -> L=%d R=%d\n",
             (int)target_heading_, (int)readHeading(), (int)error,
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
  if (out_left != 0 || out_right != 0) {
    last_out_left_  = out_left;
    last_out_right_ = out_right;
  }

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

void Straight::resetEncoderCountNoBrake()
{
  if (left_ != nullptr) {
    pup_motor_reset_count(left_);
  }
  if (right_ != nullptr) {
    pup_motor_reset_count(right_);
  }
  enc_zero_left_  = tachoDeg(left_);
  enc_zero_right_ = tachoDeg(right_);
}

void Straight::resetRotationCount()
{
  resetEncoderCountNoBrake();
}

int32_t Straight::leftEncoderCount() const
{
  return tachoDeg(left_);
}

int32_t Straight::rightEncoderCount() const
{
  return tachoDeg(right_);
}

float Straight::leftRotationCount() const
{
  return (float)leftEncoderCount() / 360.0f;
}

float Straight::rightRotationCount() const
{
  return (float)rightEncoderCount() / 360.0f;
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

void Straight::run(int left_power, int right_power, int32_t encoder_deg,
                   bool stop_end)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  correction_enabled_ = true;

  /* 走り出しの移動量も含めるため、指令の前に基準を取る。 */
  const int32_t left_base  = leftEncoderCount();
  const int32_t right_base = rightEncoderCount();

  applyWithCorrection(left_power, right_power);

  if (encoder_deg <= 0) {
    return;
  }

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < MaxDurationMs) {
    if (traveledDegFrom(left_base, right_base) >= encoder_deg) {
      break;
    }
    tickCorrection();
    dly_tsk(EncoderStepMs * 1000U);
    elapsed_ms += EncoderStepMs;
  }

  if (stop_end) {
    stop();
  }
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
  flushHeadingDebugLog();
}

void Straight::brake()
{
  syncStoppedState();
  if (left_ != nullptr) pup_motor_brake(left_);
  if (right_ != nullptr) pup_motor_brake(right_);
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

void Straight::runForEach(int left_power, int right_power,
                          int32_t left_target_deg, int32_t right_target_deg)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  if ((left_target_deg <= 0 && right_target_deg <= 0) ||
      (left_power == 0 && right_power == 0)) {
    stop();
    return;
  }

  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);

  const bool prev_heading_correction = heading_correction_;
  heading_correction_ = false;

  const int l_pwr = clampMotorPower(left_power);
  const int r_pwr = clampMotorPower(right_power);

  resetEncoderCount();
  pup_motor_set_power(left_, (left_target_deg > 0) ? l_pwr : 0);
  pup_motor_set_power(right_, (right_target_deg > 0) ? r_pwr : 0);

  uint32_t safety_ms = 0U;
  while (safety_ms < EncoderRunMaxMs) {
    const bool left_done = wheelEncoderReached(
        tachoDeg(left_) - enc_zero_left_, l_pwr, left_target_deg);
    const bool right_done = wheelEncoderReached(
        tachoDeg(right_) - enc_zero_right_, r_pwr, right_target_deg);
    if (left_done && right_done) {
      break;
    }
    pup_motor_set_power(left_, left_done ? 0 : l_pwr);
    pup_motor_set_power(right_, right_done ? 0 : r_pwr);
    dly_tsk(EncoderStepMs * 1000U);
    safety_ms += EncoderStepMs;
  }

  pup_motor_brake(left_);
  pup_motor_brake(right_);
  syncStoppedState();
  heading_correction_ = prev_heading_correction;
}

void Straight::runForEachSynchronized(int left_power, int right_power,
                                      int32_t left_target_deg,
                                      int32_t right_target_deg)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  if ((left_target_deg <= 0 && right_target_deg <= 0) ||
      (left_power == 0 && right_power == 0)) {
    stop();
    return;
  }

  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);

  const bool prev_heading_correction = heading_correction_;
  heading_correction_ = false;

  const int l_pwr = clampMotorPower(left_power);
  const int r_pwr = clampMotorPower(right_power);
  resetEncoderCount();
  pup_motor_set_power(left_, (left_target_deg > 0) ? l_pwr : 0);
  pup_motor_set_power(right_, (right_target_deg > 0) ? r_pwr : 0);

  uint32_t safety_ms = 0U;
  while (safety_ms < EncoderRunMaxMs) {
    debugHeading();
    const int32_t left_count =
        absEncoderDeg(tachoDeg(left_) - enc_zero_left_);
    const int32_t right_count =
        absEncoderDeg(tachoDeg(right_) - enc_zero_right_);
    const bool left_done = left_target_deg <= 0 ||
                           left_count >= left_target_deg;
    const bool right_done = right_target_deg <= 0 ||
                            right_count >= right_target_deg;
    if (left_done && right_done) {
      break;
    }

    const float left_progress =
        (float)left_count / (float)left_target_deg;
    const float right_progress =
        (float)right_count / (float)right_target_deg;
    int left_command = left_done ? 0 : l_pwr;
    int right_command = right_done ? 0 : r_pwr;
    if (!left_done && !right_done &&
        left_progress > right_progress + 0.01f) {
      left_command = (l_pwr > 0) ? l_pwr - 1 : l_pwr + 1;
    } else if (!left_done && !right_done &&
               right_progress > left_progress + 0.01f) {
      right_command = (r_pwr > 0) ? r_pwr - 1 : r_pwr + 1;
    }
    pup_motor_set_power(left_, left_command);
    pup_motor_set_power(right_, right_command);
    dly_tsk(EncoderStepMs * 1000U);
    safety_ms += EncoderStepMs;
  }

  pup_motor_brake(left_);
  pup_motor_brake(right_);
  syncStoppedState();
  heading_correction_ = prev_heading_correction;
}

void Straight::runForRotations(int left_power, int right_power,
                               float left_rotations, float right_rotations)
{
  const float left_corrected_deg =
      fabsf(left_rotations) * 360.0f - 10.0f;
  const int32_t left_target_deg =
      (int32_t)ceilf((left_corrected_deg > 0.0f) ? left_corrected_deg : 0.0f);
  const int32_t right_target_deg =
      (int32_t)ceilf(fabsf(right_rotations) * 360.0f);
  runForEach(left_power, right_power, left_target_deg, right_target_deg);
}

void Straight::runForRotationsSameTime(int left_power, int right_power,
                                       float left_rotations,
                                       float right_rotations)
{
  const float left_corrected_deg = fabsf(left_rotations) * 360.0f;
  const int32_t left_target_deg =
      (int32_t)ceilf((left_corrected_deg > 0.0f) ? left_corrected_deg : 0.0f);
  const int32_t right_target_deg =
      (int32_t)ceilf(fabsf(right_rotations) * 360.0f);
  runForEachSynchronizedFractional((float)left_power, (float)right_power,
                                    left_target_deg, right_target_deg);
}

void Straight::runForRotationsSameTime(float left_power, float right_power,
                                       float left_rotations,
                                       float right_rotations)
{
  const int32_t left_target_deg =
      (int32_t)ceilf(fabsf(left_rotations) * 360.0f);
  const int32_t right_target_deg =
      (int32_t)ceilf(fabsf(right_rotations) * 360.0f);
  runForEachSynchronizedFractional(left_power, right_power,
                                    left_target_deg, right_target_deg);
}

bool Straight::runForRotationsSameTimeWithHeading(float left_power,
                                                  float right_power,
                                                  float left_rotations,
                                                  float right_rotations,
                                                  bool stop_at_end,
                                                  float tolerance_deg,
                                                  float settle_min_power,
                                                  bool emit_debug)
{
  const int32_t left_target_deg =
      (int32_t)ceilf(fabsf(left_rotations) * 360.0f);
  const int32_t right_target_deg =
      (int32_t)ceilf(fabsf(right_rotations) * 360.0f);
  if (left_target_deg <= 0 && right_target_deg <= 0) {
    stop();
    return true;
  }
  if (left_ == nullptr || right_ == nullptr) {
    return false;
  }

  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  const float heading_tolerance =
      (tolerance_deg >= 0.0f) ? tolerance_deg : 0.0f;
  const float left_base_power = clampFloat(left_power, -100.0f, 100.0f);
  const float right_base_power = clampFloat(right_power, -100.0f, 100.0f);
  float sync_integral = 0.0f;
  float previous_heading_error = 0.0f;
  float filtered_heading_rate = 0.0f;
  bool have_previous_heading_error = false;
  float left_fraction = 0.0f;
  float right_fraction = 0.0f;

  const auto nextPower = [](float desired_power, float& fraction) {
    int command = (int)desired_power;
    fraction += desired_power - (float)command;
    if (fraction >= 1.0f) {
      ++command;
      fraction -= 1.0f;
    } else if (fraction <= -1.0f) {
      --command;
      fraction += 1.0f;
    }
    if (command > 0 && command < TurnEncBalanceMinPower) {
      command = TurnEncBalanceMinPower;
    } else if (command < 0 && command > -TurnEncBalanceMinPower) {
      command = -TurnEncBalanceMinPower;
    }
    return clampMotorPower(command);
  };

  if (stop_at_end) {
    resetEncoderCount();
  } else {
    resetEncoderCountNoBrake();
  }
  uint32_t safety_ms = 0U;
  uint32_t detailed_debug_div = 0U;
  if (emit_debug && (sync_debug_ || heading_debug_enabled_)) {
    appendHeadingLog("Heading,start,T=%.2f,H=%.2f,E=%.2f\n",
                     target_heading_, gGyro.heading(),
                     Gyro::wrapDeg180(gGyro.heading() - target_heading_));
  }
  while (safety_ms < EncoderRunMaxMs) {
    const int32_t left_count =
        absEncoderDeg(tachoDeg(left_) - enc_zero_left_);
    const int32_t right_count =
        absEncoderDeg(tachoDeg(right_) - enc_zero_right_);
    const bool left_done = left_target_deg <= 0 || left_count >= left_target_deg;
    const bool right_done = right_target_deg <= 0 || right_count >= right_target_deg;
    const float heading_error =
      Gyro::wrapDeg180(gGyro.heading() - target_heading_);
    if (have_previous_heading_error) {
      const float heading_rate =
          (heading_error - previous_heading_error) /
          ((float)EncoderStepMs / 1000.0f);
      filtered_heading_rate =
          filtered_heading_rate * 0.8f + heading_rate * 0.2f;
    } else {
      have_previous_heading_error = true;
    }
    previous_heading_error = heading_error;
    const bool distance_done = left_done && right_done;
    if (distance_done && fabsf(heading_error) <= heading_tolerance) {
      break;
    }

    const bool pivot_profile =
        (left_target_deg == 0) != (right_target_deg == 0);
    float left_desired =
        (distance_done || left_done) ? 0.0f : left_base_power;
    float right_desired =
        (distance_done || right_done) ? 0.0f : right_base_power;
    float pivot_progress = 0.0f;
    if (!distance_done && pivot_profile) {
      const int32_t pivot_count =
          (left_target_deg > 0) ? left_count : right_count;
      const int32_t pivot_target =
          (left_target_deg > 0) ? left_target_deg : right_target_deg;
      pivot_progress =
          clampFloat((float)pivot_count / (float)pivot_target, 0.0f, 1.0f);
      const float start_power =
          fmaxf(fabsf(left_base_power), fabsf(right_base_power));
      const float middle_power = fmaxf(start_power - 5.0f, 0.0f);
      float profile_power;
      if (pivot_progress <= 0.25f) {
        const float phase = pivot_progress / 0.25f;
        profile_power = start_power +
                        (middle_power - start_power) * phase;
      } else if (pivot_progress <= 0.75f) {
        profile_power = middle_power;
      } else {
        const float phase = (pivot_progress - 0.75f) / 0.25f;
        profile_power = middle_power * (1.0f - phase);
      }
      if (left_desired != 0.0f) {
        left_desired = copysignf(profile_power, left_base_power);
      }
      if (right_desired != 0.0f) {
        right_desired = copysignf(profile_power, right_base_power);
      }
    }
    if (!distance_done && heading_run_power_scale_ < 1.0f) {
      const float error_ratio = clampFloat(fabsf(heading_error) / 15.0f,
                                           0.0f, 1.0f);
      const float power_scale =
          heading_run_power_scale_ +
          (1.0f - heading_run_power_scale_) * error_ratio;
      left_desired *= power_scale;
      right_desired *= power_scale;
    }
    float sync_correction = 0.0f;
    if (!left_done && !right_done) {
      const float left_progress = (float)left_count / (float)left_target_deg;
      const float right_progress = (float)right_count / (float)right_target_deg;
      const float progress_error = right_progress - left_progress;
      sync_integral = clampFloat(sync_integral + progress_error, -20.0f, 20.0f);
      sync_correction = clampFloat(progress_error * 500.0f +
                                       sync_integral * 1.5f,
                                   -40.0f, 40.0f);
      left_desired += (left_base_power >= 0.0f)
                          ? sync_correction : -sync_correction;
      right_desired -= (right_base_power >= 0.0f)
                           ? sync_correction : -sync_correction;
    }

    /* 距離到達後はピボット収束へ切り替えるため、走行中の積分値を
     * 残さない。積分値が残ると収束中に補正方向が遅れて反転する。 */
    if (distance_done) {
      integral_ = 0.0f;
    } else {
      /* 積分値は秒単位で蓄積する。 */
      integral_ = clampFloat(
          integral_ + heading_error * ((float)EncoderStepMs / 1000.0f),
          -i_max_, i_max_);
    }
    const float heading_correction = clampFloat(
        (kp_ * heading_error + ki_ * integral_ -
         kd_ * filtered_heading_rate) * heading_correction_scale_,
        (float)-heading_trim_limit_, (float)heading_trim_limit_);
    int heading_diff = clampInt((int)roundf(heading_correction),
                                -heading_trim_limit_, heading_trim_limit_);
    /* 小さな誤差が丸めで補正ゼロにならないよう、許容誤差超では
     * 最低1段の補正を残す。 */
    if (heading_diff == 0 && fabsf(heading_error) > heading_tolerance) {
      heading_diff = (heading_error > 0.0f) ? 1 : -1;
    }
    if (distance_done) {
      /* 距離到達後は走行パワーを残さず、角度だけピボットで収束させる。 */
        const int settle_power_limit = clampInt(
            (int)ceilf(fmaxf(fabsf(left_base_power),
                             fabsf(right_base_power))) + 5,
            1, 100);
        const int settle_power = clampInt(
            (int)roundf(fmaxf((float)abs(heading_diff), settle_min_power)),
            1, settle_power_limit);
        if (heading_error > 0.0f) {
        left_desired = -(float)settle_power;
        right_desired = (float)settle_power;
      } else {
        left_desired = (float)settle_power;
        right_desired = -(float)settle_power;
      }
    } else if (!pivot_profile) {
      left_desired -= (float)(heading_diff / 2);
      right_desired += (float)(heading_diff - heading_diff / 2);

      /* 片側が100%に飽和して補正差が消えないよう、余剰分を反対側へ回す。 */
      if (right_desired > 100.0f) {
        left_desired -= right_desired - 100.0f;
        right_desired = 100.0f;
      }
      if (left_desired < -100.0f) {
        right_desired += left_desired + 100.0f;
        left_desired = -100.0f;
      }
      if (left_desired > 100.0f) {
        right_desired += left_desired - 100.0f;
        left_desired = 100.0f;
      }
      if (right_desired < -100.0f) {
        left_desired -= right_desired + 100.0f;
        right_desired = -100.0f;
      }
    }

    const int left_command = nextPower(left_desired, left_fraction);
    const int right_command = nextPower(right_desired, right_fraction);
    pup_motor_set_power(left_, left_command);
    pup_motor_set_power(right_, right_command);
    if (emit_debug && (sync_debug_ || heading_debug_enabled_) &&
        ++detailed_debug_div >= 20U) {
      detailed_debug_div = 0U;
      appendHeadingLog("Heading,step,T=%.2f,H=%.2f,E=%.2f,Hc=%.2f,"
                       "Lcmd=%d,Rcmd=%d\n",
                       target_heading_, gGyro.heading(), heading_error,
                       heading_correction, left_command, right_command);
    }
    dly_tsk(EncoderStepMs * 1000U);
    safety_ms += EncoderStepMs;
  }

  const int32_t final_left_count =
      absEncoderDeg(tachoDeg(left_) - enc_zero_left_);
  const int32_t final_right_count =
      absEncoderDeg(tachoDeg(right_) - enc_zero_right_);
  const float final_heading = gGyro.heading();
  const float final_error =
      Gyro::wrapDeg180(final_heading - target_heading_);
  const bool completed =
      final_left_count >= left_target_deg &&
      final_right_count >= right_target_deg &&
      fabsf(final_error) <= heading_tolerance;
  if (emit_debug && (sync_debug_ || heading_debug_enabled_)) {
    appendHeadingLog("Heading,end,T=%.2f,H=%.2f,E=%.2f\n",
                     target_heading_, final_heading, final_error);
  }
  flushHeadingDebugLog();
  if (stop_at_end) {
    pup_motor_brake(left_);
    pup_motor_brake(right_);
    syncStoppedState();
  }
  return completed;
}

void Straight::runForEachSynchronizedFractional(float left_power,
                                                float right_power,
                                                int32_t left_target_deg,
                                                int32_t right_target_deg)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  if ((left_target_deg <= 0 && right_target_deg <= 0) ||
      (left_power == 0.0f && right_power == 0.0f)) {
    stop();
    return;
  }

  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  const float left_base_power = clampFloat(left_power, -100.0f, 100.0f);
  const float right_base_power = clampFloat(right_power, -100.0f, 100.0f);
  float left_fraction = 0.0f;
  float right_fraction = 0.0f;
  float sync_integral = 0.0f;

  const auto nextPower = [](float desired_power, float& fraction) {
    int command = (int)desired_power;
    fraction += desired_power - (float)command;
    if (fraction >= 1.0f) {
      ++command;
      fraction -= 1.0f;
    } else if (fraction <= -1.0f) {
      --command;
      fraction += 1.0f;
    }
    if (command > 0 && command < TurnEncBalanceMinPower) {
      command = TurnEncBalanceMinPower;
    } else if (command < 0 && command > -TurnEncBalanceMinPower) {
      command = -TurnEncBalanceMinPower;
    }
    return clampMotorPower(command);
  };

  resetEncoderCount();
  int left_command = nextPower(left_base_power, left_fraction);
  int right_command = nextPower(right_base_power, right_fraction);
  pup_motor_set_power(left_, left_command);
  pup_motor_set_power(right_, right_command);

  uint32_t safety_ms = 0U;
  while (safety_ms < EncoderRunMaxMs) {
    const int32_t left_count =
        absEncoderDeg(tachoDeg(left_) - enc_zero_left_);
    const int32_t right_count =
        absEncoderDeg(tachoDeg(right_) - enc_zero_right_);
    const bool left_done = left_target_deg <= 0 || left_count >= left_target_deg;
    const bool right_done = right_target_deg <= 0 || right_count >= right_target_deg;
    if (left_done && right_done) {
      break;
    }

    float left_desired = left_done ? 0.0f : left_base_power;
    float right_desired = right_done ? 0.0f : right_base_power;
    float sync_correction = 0.0f;
    if (!left_done && !right_done) {
      const float left_progress =
          (float)left_count / (float)left_target_deg;
      const float right_progress =
          (float)right_count / (float)right_target_deg;
      const float progress_error = right_progress - left_progress;
        sync_integral = clampFloat(sync_integral + progress_error, -20.0f, 20.0f);
        sync_correction = clampFloat(progress_error * 500.0f +
                         sync_integral * 1.5f,
                       -40.0f, 40.0f);
      left_desired += (left_base_power >= 0.0f)
                          ? sync_correction : -sync_correction;
      right_desired -= (right_base_power >= 0.0f)
                           ? sync_correction : -sync_correction;
    }

    left_command = nextPower(left_desired, left_fraction);
    right_command = nextPower(right_desired, right_fraction);
    pup_motor_set_power(left_, left_command);
    pup_motor_set_power(right_, right_command);
    if (sync_debug_ && ++sync_debug_div_ >= 10U) {
      sync_debug_div_ = 0U;
      appendHeadingLog(
          "Sync,Lcmd=%d,Rcmd=%d,Ldeg=%ld/%ld,Rdeg=%ld/%ld,"
          "Lp=%.3f,Rp=%.3f,C=%.2f\n",
          left_command, right_command,
          (long)left_count, (long)left_target_deg,
          (long)right_count, (long)right_target_deg,
          (float)left_count / (float)left_target_deg,
          (float)right_count / (float)right_target_deg,
          sync_correction);
    }
    dly_tsk(EncoderStepMs * 1000U);
    safety_ms += EncoderStepMs;
  }

  pup_motor_brake(left_);
  pup_motor_brake(right_);
  syncStoppedState();
  flushHeadingDebugLog();
}

bool Straight::runForRotationsUntilHeading(float left_power, float right_power,
                                            float left_rotations,
                                            float right_rotations,
                                            float target_heading,
                                            float tolerance_deg,
                                            float min_left_rotations,
                                            float min_right_rotations,
                                            bool pivot_on_failure,
                                            int pivot_min_power,
                                            uint32_t pivot_max_ms)
{
  if (left_ == nullptr || right_ == nullptr) {
    return false;
  }

  const int32_t left_target_deg =
      (int32_t)ceilf(fabsf(left_rotations) * 360.0f);
  const int32_t right_target_deg =
      (int32_t)ceilf(fabsf(right_rotations) * 360.0f);
    const int32_t left_min_deg =
      (int32_t)ceilf(fabsf(min_left_rotations) * 360.0f);
    const int32_t right_min_deg =
      (int32_t)ceilf(fabsf(min_right_rotations) * 360.0f);
  if ((left_target_deg <= 0 && right_target_deg <= 0) ||
      (left_power == 0.0f && right_power == 0.0f) ||
      (left_target_deg > 0 && left_min_deg > left_target_deg) ||
      (right_target_deg > 0 && right_min_deg > right_target_deg)) {
    stop();
    return false;
  }

  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  setTargetHeading(target_heading);
  correction_enabled_ = true;
  resetEncoderCount();

  const float left_base_power = clampFloat(left_power, -100.0f, 100.0f);
  const float right_base_power = clampFloat(right_power, -100.0f, 100.0f);
  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < EncoderRunMaxMs) {
    debugHeading();
    const int32_t left_count =
        absEncoderDeg(tachoDeg(left_) - enc_zero_left_);
    const int32_t right_count =
        absEncoderDeg(tachoDeg(right_) - enc_zero_right_);
    const float heading_error =
        Gyro::wrapDeg180(gGyro.heading() - target_heading);
    const bool heading_reached = fabsf(heading_error) <= tolerance_deg;
    const bool minimum_rotation_reached =
      (left_min_deg <= 0 || left_count >= left_min_deg) &&
      (right_min_deg <= 0 || right_count >= right_min_deg);
    const bool rotation_limit_reached =
        (left_target_deg > 0 && left_count >= left_target_deg) ||
        (right_target_deg > 0 && right_count >= right_target_deg);
    if (heading_reached && minimum_rotation_reached) {
      break;
    }
    if (rotation_limit_reached) {
      pup_motor_brake(left_);
      pup_motor_brake(right_);
      syncStoppedState();
      if (pivot_on_failure) {
        turnToHeading(target_heading, tolerance_deg,
                      70, 70, pivot_max_ms, 30.0f,
                      pivot_min_power);
        return fabsf(Gyro::wrapDeg180(gGyro.heading() - target_heading)) <=
               tolerance_deg;
      }
      return false;
    }

    integral_ = clampFloat(integral_ + heading_error, -i_max_, i_max_);
    const float correction = clampFloat(
        kp_ * heading_error + ki_ * integral_,
        (float)-heading_trim_limit_, (float)heading_trim_limit_);
    const int heading_diff = clampInt((int)roundf(correction),
                                      -heading_trim_limit_, heading_trim_limit_);
    const int left_command = clampMotorPower(
        (int)roundf(left_base_power - (float)(heading_diff / 2)));
    const int right_command = clampMotorPower(
        (int)roundf(right_base_power +
                    (float)(heading_diff - heading_diff / 2)));
    pup_motor_set_power(left_, left_command);
    pup_motor_set_power(right_, right_command);
    dly_tsk(EncoderStepMs * 1000U);
    elapsed_ms += EncoderStepMs;
  }

  pup_motor_brake(left_);
  pup_motor_brake(right_);
  syncStoppedState();
  if (pivot_on_failure &&
      fabsf(Gyro::wrapDeg180(gGyro.heading() - target_heading)) >
          tolerance_deg) {
    turnToHeading(target_heading, tolerance_deg,
                  70, 70, pivot_max_ms, 30.0f,
                  pivot_min_power);
  }
  return fabsf(Gyro::wrapDeg180(gGyro.heading() - target_heading)) <=
         tolerance_deg;
}

bool Straight::runForRotationsUntilAngle(
    float left_power, float right_power,
    float left_rotations, float right_rotations,
    float angle_deg, float tolerance_deg,
    bool pivot_on_failure, int pivot_min_power, uint32_t pivot_max_ms,
    bool allow_negative_power, bool emit_debug)
{
  const float target_heading = Gyro::wrapDeg180(angle_deg);
  const float initial_left_power =
      allow_negative_power ? left_power : fabsf(left_power);
  const float initial_right_power =
      allow_negative_power ? right_power : fabsf(right_power);
  if (emit_debug && (sync_debug_ || heading_debug_enabled_)) {
    appendHeadingLog("kime,start,T=%.2f,H=%.2f,E=%.2f\n",
                     target_heading, gGyro.heading(),
                     Gyro::wrapDeg180(gGyro.heading() - target_heading));
  }
  /* 指定角度を走行開始時から目標にし、距離走行中もヨー角を補正する。 */
  setTargetHeading(target_heading);
  runForRotationsSameTimeWithHeading(
      initial_left_power, initial_right_power,
      left_rotations, right_rotations, true, tolerance_deg,
      (float)pivot_min_power, false);

  /* 停止直後の慣性による行き過ぎを確認してから補正の要否を判定する。 */
  dly_tsk(KimeSettleMs * 1000U);
  float final_error =
      Gyro::wrapDeg180(gGyro.heading() - target_heading);
  if (fabsf(final_error) <= tolerance_deg) {
    if (emit_debug && (sync_debug_ || heading_debug_enabled_)) {
      appendHeadingLog("kime,end,T=%.2f,H=%.2f,E=%.2f\n",
                       target_heading, gGyro.heading(), final_error);
    }
    flushHeadingDebugLog();
    return true;
  }

  if (pivot_on_failure) {
    const int correction_power = clampInt(pivot_min_power, 1, 100);
    turnToHeading(target_heading, tolerance_deg,
                  correction_power, correction_power, pivot_max_ms,
                  correction_power, correction_power);
    final_error = Gyro::wrapDeg180(gGyro.heading() - target_heading);
  }

  const bool completed = fabsf(final_error) <= tolerance_deg;
  if (emit_debug && (sync_debug_ || heading_debug_enabled_)) {
    appendHeadingLog("kime,end,T=%.2f,H=%.2f,E=%.2f\n",
                     target_heading, gGyro.heading(), final_error);
  }
  flushHeadingDebugLog();
  return completed;
}

bool Straight::runForRotationsUntilAngleNansho2Kimeuti(
    float left_power, float right_power,
    float left_rotations, float right_rotations,
    float angle_deg, float tolerance_deg,
    bool pivot_on_failure, int pivot_min_power, uint32_t pivot_max_ms,
    bool allow_negative_power)
{
  const bool completed = runForRotationsUntilAngle(
      left_power, right_power, left_rotations, right_rotations,
      angle_deg, tolerance_deg, pivot_on_failure, pivot_min_power,
      pivot_max_ms, allow_negative_power, false);
  if (completed) {
    const float target_heading = Gyro::wrapDeg180(angle_deg);
    const float current_heading = gGyro.heading();
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "KimeComplete,T=%.2f,H=%.2f,E=%.2f\n",
            target_heading, current_heading,
            Gyro::wrapDeg180(current_heading - target_heading));
  }
  return completed;
}

void Straight::runBalancedRotationsSameTime(int left_power, int right_power,
                                            float left_rotations,
                                            float right_rotations)
{
  const float left_corrected_deg = fabsf(left_rotations) * 360.0f;
  const int32_t left_target_deg =
      (int32_t)ceilf((left_corrected_deg > 0.0f) ? left_corrected_deg : 0.0f);
  const int32_t right_target_deg =
      (int32_t)ceilf(fabsf(right_rotations) * 360.0f);
  const int32_t max_target_deg =
      (left_target_deg > right_target_deg) ? left_target_deg : right_target_deg;

  if (max_target_deg <= 0) {
    stop();
    return;
  }

  int synchronized_left_power =
      (int)((int64_t)left_power * left_target_deg / max_target_deg);
  int synchronized_right_power =
      (int)((int64_t)right_power * right_target_deg / max_target_deg);
  if (left_target_deg > 0 && synchronized_left_power == 0 && left_power != 0) {
    synchronized_left_power = (left_power > 0) ? 1 : -1;
  }
  if (right_target_deg > 0 && synchronized_right_power == 0 && right_power != 0) {
    synchronized_right_power = (right_power > 0) ? 1 : -1;
  }

  beginBalancedEncoderRun();
  allow_heading_correction_for_ratio_ = true;
  const int left_command = clampMotorPower(synchronized_left_power);
  const int right_command = clampMotorPower(synchronized_right_power);

  uint32_t safety_ms = 0U;
  while (safety_ms < EncoderRunMaxMs) {
    const int32_t left_count =
        absEncoderDeg(tachoDeg(left_) - enc_zero_left_);
    const int32_t right_count =
        absEncoderDeg(tachoDeg(right_) - enc_zero_right_);
    const bool left_done = left_target_deg <= 0 || left_count >= left_target_deg;
    const bool right_done = right_target_deg <= 0 || right_count >= right_target_deg;
    if (left_done && right_done) {
      break;
    }
    const uint32_t ramp_ms = (safety_ms < 500U) ? safety_ms : 500U;
    const int ramp_percent = 10 + (int)(90U * ramp_ms / 500U);
    const int ramped_left =
      (left_command * ramp_percent) / 100;
    const int ramped_right =
      (right_command * ramp_percent) / 100;
    applyWithCorrection(ramped_left, ramped_right);
    dly_tsk(EncoderStepMs * 1000U);
    safety_ms += EncoderStepMs;
  }

  allow_heading_correction_for_ratio_ = false;
  endBalancedEncoderRun();
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

void Straight::captureTurnStopSnapshot(uint32_t elapsed_ms, float gyro_acc,
                                       int32_t left_abs, int32_t right_abs)
{
  last_turn_elapsed_ms_    = elapsed_ms;
  last_turn_gyro_acc_deg_  = gyro_acc;
  last_turn_enc_left_abs_  = left_abs;
  last_turn_enc_right_abs_ = right_abs;
  last_turn_heading_deg_   = readHeading();
}

void Straight::captureEncoder180(uint32_t ms, int32_t left, int32_t right,
                                 float gyro_delta)
{
  if (!turn_encoder180_.reached &&
      0.5f * (fabsf(static_cast<float>(left)) + fabsf(static_cast<float>(right))) >= 180.0f) {
    turn_encoder180_.reached = true;
    turn_encoder180_.elapsed_ms = ms;
    turn_encoder180_.left = left;
    turn_encoder180_.right = right;
    turn_encoder180_.gyro_delta = gyro_delta;
  }
}

void Straight::turnToHeading(float target_heading, float tolerance_deg,
                             int left_power, int right_power, uint32_t max_ms,
                             float slow_approach_deg, int slow_power,
                             float stop_early_deg, bool coast_end,
                             int prefer_sign, bool encoder_drift_correction)
{
  turn_drift_ = TurnDrift();
  turn_encoder180_ = TurnEncoder180Snapshot();
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }

  /* 絶対目標まで、開始基準の累積旋回角で回す。
   * prefer_sign=0 なら最短側。±1 ならその向きに強制し、必要なら遠回りする。
   * 残りが slow_approach_deg 以内なら低速（行き過ぎ防止）。
   * その場旋回は両輪逆符号（L+/R− または L−/R+）。
   * 左右のエンコーダ回転量が揃うようパワーを補正する。 */
  const float start = readHeading();
  const float delta = Gyro::wrapDeg180(target_heading - start);
  if (fabsf(delta) <= tolerance_deg) {
    captureTurnStopSnapshot(0U, 0.0f, 0, 0);
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
  const float early = stop_early_deg;

  bool left_valid = false;
  bool right_valid = false;
  const int32_t enc_l0 = tachoDeg(left_, &left_valid);
  const int32_t enc_r0 = tachoDeg(right_, &right_valid);
  const bool encoder_base_valid = left_valid && right_valid;
  bool encoder_valid = encoder_base_valid;
  const bool    prev_heading_correction = heading_correction_;
  /* 旋回中は直進ヨー補正を掛けない（ピボット指令を崩さない） */
  heading_correction_ = false;

  SYSTIM turn_start = 0;
  (void)get_tim(&turn_start); // TOPPERS システム時刻は us
  float previous_raw = readRawHeading();
  float gyro_traveled = 0.0f;
  const bool fusion_enabled = encoder_drift_correction && !heading_reader_ && TurnDriftConfig::Enabled;
  float enc_left = 0.0f;
  float enc_right = 0.0f;
  uint32_t sample_ms = 0U;
  const auto read_turn_encoders = [&]() {
    bool lv = false;
    bool rv = false;
    const int32_t l = tachoDeg(left_, &lv) - enc_l0;
    const int32_t r = tachoDeg(right_, &rv) - enc_r0;
    // 欠測した周期だけ除外し、次の正常取得で補正へ復帰する。
    encoder_valid = encoder_base_valid && lv && rv;
    SYSTIM sample_time = 0;
    (void)get_tim(&sample_time);
    sample_ms = static_cast<uint32_t>((sample_time - turn_start) / 1000U);
    enc_left = static_cast<float>(l);
    enc_right = static_cast<float>(r);
  };
  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < max_ms) {
    const float raw = readRawHeading();
    if (!isfinite(raw) || !isfinite(previous_raw)) break;
    // 周期差だけ wrap し、累積角は wrap しない（180°を超えても保持）。
    gyro_traveled += Gyro::wrapDeg180(raw - previous_raw);
    previous_raw = raw;
    read_turn_encoders();
    if (encoder_valid) {
      captureEncoder180(sample_ms, static_cast<int32_t>(enc_left),
                        static_cast<int32_t>(enc_right), gyro_traveled);
    }
    const float correction = fusion_enabled
        ? turn_drift_.estimate(gyro_traveled, enc_left, enc_right, turn_dir,
                               encoder_valid, sample_ms)
        : 0.0f;
    const float fused_angle = gyro_traveled + correction;
    // 方向付きの累積角で残りを求め、180°越え・目標通過も連続に扱う。
    const float rem = need - turn_dir * fused_angle;
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

    /* |ΔL| と |ΔR| を揃える。進みすぎた側のパワーを落とす。左は常に trim */
    const int32_t l_abs =
        absEncoderDeg(tachoDeg(left_) - enc_l0);
    const int32_t r_abs =
        absEncoderDeg(tachoDeg(right_) - enc_r0);
    int l_mag = 0;
    int r_mag = 0;
    turnWheelMags(base, l_abs - r_abs, &l_mag, &r_mag);

    if (turn_dir > 0.0f) {
      run(l_mag, -r_mag);
    } else {
      run(-l_mag, r_mag);
    }

    dly_tsk(TurnStepMs * 1000U);
    elapsed_ms += TurnStepMs;
  }

  captureTurnStopSnapshot(
      elapsed_ms, gyro_traveled,
      absEncoderDeg(tachoDeg(left_) - enc_l0),
      absEncoderDeg(tachoDeg(right_) - enc_r0));
  heading_correction_ = prev_heading_correction;

  if (coast_end) {
    coast();
  } else {
    stop();
  }
  if (fusion_enabled) {
    // 惰性を含む最終角を同じ式で融合し、方位へ一度だけ反映する。
    for (uint32_t waited = 0; waited < TurnDriftConfig::SettleMs; ++waited) {
      dly_tsk(1000U);
      const float raw = readRawHeading();
      if (!isfinite(raw) || !isfinite(previous_raw)) {
        encoder_valid = false;
        break;
      }
      gyro_traveled += Gyro::wrapDeg180(raw - previous_raw);
      previous_raw = raw;
      read_turn_encoders();
      turn_drift_.estimate(gyro_traveled, enc_left, enc_right, turn_dir,
                               encoder_valid, sample_ms);
    }
    SYSTIM now = 0;
    (void)get_tim(&now);
    const float correction = turn_drift_.update(
        gyro_traveled, enc_left, enc_right,
        static_cast<float>(now - turn_start) * 1.0e-6f, turn_dir,
        encoder_valid);
    // 異なる取得時刻の方位差で上書きせず、計算した補正量だけを反映する。
    // 重み0では方位を変更しない（静定待機は同じ条件で維持）。
    if (isfinite(correction) && correction != 0.0f) {
      gGyro.addHeadingCorrection(correction);
    }
  }
  dly_tsk(TurnStepMs * 1000U);
}

void Straight::turnByEncoder(int32_t target_deg, bool left, int left_power,
                             int right_power, uint32_t max_ms,
                             int32_t slow_approach_deg, int slow_power,
                             int32_t stop_early_deg, bool coast_end)
{
  turn_drift_ = TurnDrift();
  turn_encoder180_ = TurnEncoder180Snapshot();
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }

  const int32_t need = absEncoderDeg(target_deg);
  if (need <= 0) {
    captureTurnStopSnapshot(0U, 0.0f, 0, 0);
    if (coast_end) {
      coast();
    } else {
      stop();
    }
    return;
  }

  const float   turn_dir = left ? -1.0f : 1.0f;
  const int32_t early    = stop_early_deg;
  const int32_t enc_l0   = tachoDeg(left_);
  const int32_t enc_r0   = tachoDeg(right_);
  const bool    prev_heading_correction = heading_correction_;
  heading_correction_ = false;

  SYSTIM encoder_turn_start = 0;
  (void)get_tim(&encoder_turn_start);
  float previous_raw = readRawHeading();
  float gyro_traveled = 0.0f;
  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < max_ms) {
    const float raw = readRawHeading();
    gyro_traveled += Gyro::wrapDeg180(raw - previous_raw);
    previous_raw = raw;
    bool lv = false, rv = false;
    const int32_t l = tachoDeg(left_, &lv) - enc_l0;
    const int32_t r = tachoDeg(right_, &rv) - enc_r0;
    const int32_t l_abs = absEncoderDeg(l);
    const int32_t r_abs = absEncoderDeg(r);
    SYSTIM now = 0;
    (void)get_tim(&now);
    if (lv && rv) captureEncoder180(static_cast<uint32_t>((now - encoder_turn_start) / 1000U), l, r, gyro_traveled);
    const int32_t rem   = need - ((l_abs + r_abs) / 2);
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
    if (slow_approach_deg > 0 && rem <= slow_approach_deg) {
      base = slow_power;
      if (base < 0) {
        base = -base;
      }
    }
    if (base < TurnEncBalanceMinPower) {
      base = TurnEncBalanceMinPower;
    }

    const int32_t enc_diff = l_abs - r_abs;
    int l_mag = 0;
    int r_mag = 0;
    turnWheelMags(base, enc_diff, &l_mag, &r_mag);

    if (turn_dir > 0.0f) {
      run(l_mag, -r_mag);
    } else {
      run(-l_mag, r_mag);
    }

    dly_tsk(TurnStepMs * 1000U);
    elapsed_ms += TurnStepMs;
  }

  captureTurnStopSnapshot(
      elapsed_ms, gyro_traveled,
      absEncoderDeg(tachoDeg(left_) - enc_l0),
      absEncoderDeg(tachoDeg(right_) - enc_r0));
  heading_correction_ = prev_heading_correction;
  if (coast_end) {
    coast();
  } else {
    stop();
  }
  dly_tsk(TurnStepMs * 1000U);
}

void Straight::curveToHeading(float target_heading, int outer_power,
                              int inner_power, uint32_t max_ms,
                              float exit_accel_deg, int prefer_sign,
                              bool stop_end)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }

  correction_enabled_ = true;
  const bool prev_heading_correction = heading_correction_;
  heading_correction_ = false;

  float current = readHeading();
  const float delta = Gyro::wrapDeg180(target_heading - current);

  float turn_dir;
  if (prefer_sign > 0) {
    turn_dir = 1.0f;
  } else if (prefer_sign < 0) {
    turn_dir = -1.0f;
  } else {
    turn_dir = (delta >= 0.0f) ? 1.0f : -1.0f;
  }

  float need;
  if (turn_dir * delta >= 0.0f) {
    need = fabsf(delta);
  } else {
    need = 360.0f - fabsf(delta);
  }

  float traveled = 0.0f;
  float previous = current;
  uint32_t elapsed_ms = 0U;

  while (elapsed_ms < max_ms) {
    current = readHeading();
    traveled += turn_dir * Gyro::wrapDeg180(current - previous);
    previous = current;
    const float remaining = need - traveled;
    if (remaining <= 2.0f) {
      break;
    }

    int outer = outer_power;
    int inner = inner_power;
    if (exit_accel_deg > 0.0f && remaining <= exit_accel_deg) {
      float ratio = (exit_accel_deg - remaining) / exit_accel_deg;
      ratio = clampFloat(ratio, 0.0f, 1.0f);
      outer = outer_power +
              static_cast<int>(roundf((100 - outer_power) * ratio));
      inner = inner_power +
              static_cast<int>(roundf((100 - inner_power) * ratio));
    }

    outer = clampMotorPower(outer);
    inner = clampMotorPower(inner);
    if (turn_dir > 0.0f) {
      applyWithCorrection(outer, inner);
    } else {
      applyWithCorrection(inner, outer);
    }

    dly_tsk(TurnStepMs * 1000U);
    elapsed_ms += TurnStepMs;
  }

  heading_correction_ = prev_heading_correction;
  if (stop_end) {
    stop();
  }
}
