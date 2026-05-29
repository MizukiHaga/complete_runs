/*
 * Straight モジュール 実装
 *  左右モータをまとめて操作する直進系の共通モジュール。
 *  PUP モータ API はこのファイル内だけで使い、外からは Straight クラスを介して利用する。
 */

#include "app.h"
#include "Straight.h"

/* グローバルな Straight インスタンス。実体は app.cpp で定義されている。 */
extern Straight gMotors;

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

/* 初期化: app.cpp 側で 1 回だけ呼ぶ。
 *  - 左右モータデバイスを取得
 *  - 正回転方向を設定 (機体の向きに合わせる)
 *  - 第3引数 true でエンコーダをリセット */
void Straight::configure(pbio_port_id_t left_motor_port,
                         pbio_port_id_t right_motor_port)
{
  left_ = pup_motor_get_device(left_motor_port);
  right_ = pup_motor_get_device(right_motor_port);

  pup_motor_setup(left_, PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(right_, PUP_DIRECTION_CLOCKWISE, true);
}

/* 左右モータに指定パワーをセットする。すぐ戻る (ブロッキングしない)。
 * - 正の値: 前進方向
 * - 負の値: 後退方向
 * - 0     : 惰性停止 (set_power(0) はブレーキではない) */
void Straight::run(int left_power, int right_power)
{
  if (left_ == nullptr || right_ == nullptr) {
    return;
  }
  pup_motor_set_power(left_, clampMotorPower(left_power));
  pup_motor_set_power(right_, clampMotorPower(right_power));
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
}

/* 指定パワーで duration_ms だけ直進し、最後は線形減速して止まる (ブロッキング)。
 *
 * 内部の時間配分:
 *   全体 duration_ms = 定速 cruise_ms + 減速 decel_ms
 *
 *   [   定速 cruise_ms   ][  減速 decel_ms  ]
 *   ────────────────────  ＼＿＿＿＿＿＿
 *
 * 例: duration_ms=2000, DecelMs=800 なら
 *   1200ms 定速 → 800ms かけて 0 まで線形に下げる。
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

  /* --- 定速区間 ---
   * dly_tsk は μs 単位なので 1000 倍してから渡す */
  if (cruise_ms > 0U) {
    run(left_power, right_power);
    dly_tsk((RELTIM)(cruise_ms * 1000U));
  }

  /* --- 線形減速区間 ---
   * DecelStepMs ごとにパワーを更新して滑らかに 0 まで落とす。
   * 残り時間 numer に比例したパワーを出すことで、グラフは右下がりの直線になる。 */
  for (uint32_t t = 0U; t < decel_ms;) {
    uint32_t slice = DecelStepMs;
    if (t + slice > decel_ms) {
      /* 最後のスライスは端数に合わせて短くする */
      slice = decel_ms - t;
    }
    const uint32_t numer = decel_ms - t;
    /* int の桁あふれを避けるため int64_t にキャストしてから掛け算 */
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
