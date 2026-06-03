/*
 * Straight モジュール 実装
 *  左右モータをまとめて操作する直進系の共通モジュール。
 *  PUP モータ API はこのファイル内だけで使い、外からは Straight クラスを介して利用する。
 */

#include "app.h"
#include "Straight.h"

#include <math.h>
#include <stdio.h>

/* グローバルな Straight インスタンス。実体は app.cpp で定義されている。 */
extern Straight gMotors;

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

/* モータパワーを安全な範囲 [-100, 100] に丸める。
 * 計算誤差や外部からの不正値で範囲外が来てもクラッシュしないようにするため。 */
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

void Straight::resetCorrection()
{
  integral_        = 0.0f;
  last_cmd_left_   = 0;
  last_cmd_right_  = 0;
  cmd_initialized_ = false;

  if (left_ != nullptr && right_ != nullptr) {
    prev_cL_ = pup_motor_get_count(left_);
    prev_cR_ = pup_motor_get_count(right_);
  } else {
    prev_cL_ = 0;
    prev_cR_ = 0;
  }
}

void Straight::setPowerRaw(int left_power, int right_power)
{
  pup_motor_set_power(left_, clampMotorPower(left_power));
  pup_motor_set_power(right_, clampMotorPower(right_power));

  prev_cL_         = pup_motor_get_count(left_);
  prev_cR_         = pup_motor_get_count(right_);
  last_cmd_left_   = left_power;
  last_cmd_right_  = right_power;
  cmd_initialized_ = true;
}

/* エンコーダ PID で左右パワーを補正して set_power する。
 *
 * error = |dcL| * |pr| - |dcR| * |pl|
 * trim  = Kp * error + Ki * integral
 * left  = pl - trim/2
 * right = pr + trim/2
 */
void Straight::applyWithCorrection(int left_power, int right_power)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }

  /* (0, 0): 停止 + 状態リセット */
  if (left_power == 0 && right_power == 0) {
    pup_motor_set_power(left_, 0);
    pup_motor_set_power(right_, 0);
    resetCorrection();
    return;
  }

  /* 補正無効 / 一輪駆動: open-loop */
  if (!correction_enabled_ || left_power == 0 || right_power == 0) {
    integral_ = 0.0f;
    setPowerRaw(left_power, right_power);
    return;
  }

  const bool cmd_changed =
      !cmd_initialized_ ||
      left_power != last_cmd_left_ ||
      right_power != last_cmd_right_;

  /* コマンド変化: 積分リセット + 生パワー (1 周期目は補正しない) */
  if (cmd_changed) {
    integral_ = 0.0f;
    setPowerRaw(left_power, right_power);
    return;
  }

  /* 同一コマンド継続: エンコーダ差分から PID 補正 */
  const int32_t cL   = pup_motor_get_count(left_);
  const int32_t cR   = pup_motor_get_count(right_);
  const int32_t dcL  = cL - prev_cL_;
  const int32_t dcR  = cR - prev_cR_;
  const int     apl  = (left_power  >= 0) ? left_power  : -left_power;
  const int     apr  = (right_power >= 0) ? right_power : -right_power;
  const int32_t adcL = (dcL >= 0) ? dcL : -dcL;
  const int32_t adcR = (dcR >= 0) ? dcR : -dcR;

  /* 左右モータのエンコーダ差分から PID 補正 */
  const float error = (float)adcL * (float)apr - (float)adcR * (float)apl;

  integral_ += error;
  integral_ = clampFloat(integral_, -i_max_, i_max_);

  const float trim = kp_ * error + ki_ * integral_;
  const int   half = (int)(trim / 2.0f);

  /* 補正はパワーの符号に合わせて「速い車輪の絶対値を減らす」方向に効かせる。
   * こうしないと後退 (負パワー) のとき符号が逆になり、ズレを増幅して曲がる。 */
  const int sgnL = (left_power  >= 0) ? 1 : -1;
  const int sgnR = (right_power >= 0) ? 1 : -1;

  const int out_left  = clampMotorPower(left_power  - sgnL * half);
  const int out_right = clampMotorPower(right_power + sgnR * half);

  if (correction_debug_) {
    printf("Balance dcL=%d dcR=%d err=%d int=%d trim=%d -> L=%d R=%d\n",
           (int)dcL, (int)dcR, (int)error, (int)integral_, (int)trim,
           out_left, out_right);
  }

  pup_motor_set_power(left_, out_left);
  pup_motor_set_power(right_, out_right);

  prev_cL_        = cL;
  prev_cR_        = cR;
  last_cmd_left_  = left_power;
  last_cmd_right_ = right_power;
}

/* バックグラウンド補正タスクから BalanceStepMs ごとに呼ぶ。
 * 最後にセットされたコマンド (last_cmd_left_, last_cmd_right_) を
 * 再度 applyWithCorrection に通して、エンコーダ PID 補正を継続適用する。
 *
 * 何もしない条件:
 *  - configure 未実行 (left_ / right_ が null)
 *  - 補正が無効 (correction_enabled_ == false)
 *  - まだ一度も run() が呼ばれていない (cmd_initialized_ == false)
 *  - 停止指令中 (last_cmd_* がどちらも 0)
 */
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

/* 初期化: app.cpp 側で 1 回だけ呼ぶ。
 *  - 左右モータデバイスを取得
 *  - 正回転方向を設定 (機体の向きに合わせる)
 *  - 第3引数 true でエンコーダをリセット */
void Straight::configure(pbio_port_id_t left_motor_port,
                         pbio_port_id_t right_motor_port)
{
  left_  = pup_motor_get_device(left_motor_port);
  right_ = pup_motor_get_device(right_motor_port);

  pup_motor_setup(left_, PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(right_, PUP_DIRECTION_CLOCKWISE, true);

  resetCorrection();
}

/* 左右モータに指定パワーをセットする。すぐ戻る (ブロッキングしない)。
 * エンコーダ PID 補正が有効なら applyWithCorrection 経由。 */
void Straight::run(int left_power, int right_power)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  applyWithCorrection(left_power, right_power);
}

/* 左右モータのパワーを 0 にする (惰性停止)。
 * 急停止させたい場合は pup_motor_brake / pup_motor_hold を使う。 */
void Straight::stop()
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  pup_motor_set_power(left_, 0);
  pup_motor_set_power(right_, 0);
  resetCorrection();
}

/* 指定パワーで duration_ms だけ直進し、最後は線形減速して止まる (ブロッキング)。
 *
 * 内部の時間配分:
 *   全体 duration_ms = 定速 cruise_ms + 減速 decel_ms
 *
 *   [   定速 cruise_ms   ][  減速 decel_ms  ]
 *   ────────────────────  ＼＿＿＿＿＿＿
 *
 * 補正は StraightBalanceTask が BalanceStepMs ごとに tickCorrection()
 * を回して継続適用するため、ここでは BalanceStepMs での再呼び出しは不要。
 * 定速区間は一度コマンドを置いて寝るだけ、減速区間は DecelStepMs ごとに
 * コマンドを更新する。
 */
void Straight::runFor(int left_power, int right_power, uint32_t duration_ms)
{
  /* 時間 0 のときは停止だけして帰る */
  if (duration_ms == 0U) {
    stop();
    return;
  }

  /* dly_tsk の上限を超えないようにクランプ */
  if (duration_ms > MaxDurationMs) {
    duration_ms = MaxDurationMs;
  }

  /* 減速にかける時間。指定全体より長い場合は全部を減速に充てる */
  uint32_t decel_ms = DecelMs;
  if (decel_ms > duration_ms) {
    decel_ms = duration_ms;
  }

  /* 定速で走る時間 = 全体 - 減速 */
  const uint32_t cruise_ms = duration_ms - decel_ms;

  /* --- 定速区間: コマンドを 1 回置いて寝るだけ。補正は周期タスクが回す --- */
  if (cruise_ms > 0U) {
    run(left_power, right_power);
    dly_tsk((RELTIM)(cruise_ms * 1000U));
  }

  /* --- 線形減速区間 (DecelStepMs 刻みでコマンド更新) --- */
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
