/*
 * Straight モジュール 実装
 *  左右モータをまとめて操作する直進・蛇行の共通モジュール。
 *  PUP モータ API はこのファイル内だけで使い、外からは Straight クラスを介して利用する。
 */

#include "app.h"
#include "Straight.h"

/* straight_task (テスト用タスク) からも参照する、グローバルな Straight インスタンス。
 * 実体は app.cpp で定義されている。 */
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
  pup_motor_set_power(left_, clampMotorPower(left_power));
  pup_motor_set_power(right_, clampMotorPower(right_power));
}

/* 左右モータのパワーを 0 にする (惰性停止)。
 * 急停止させたい場合は pup_motor_brake / pup_motor_hold を使う。 */
void Straight::stop()
{
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

/* 蛇行走行: 左右のパワー差を phase_ms ごとに反転させてゆれながら進む。
 *
 *   左寄り: 左 = base+steer, 右 = base-steer
 *   右寄り: 左 = base-steer, 右 = base+steer
 *
 * 終了条件 (いずれかを満たすと stop() して return):
 *   - poll() が true を返した (外部からの停止要求)
 *   - run_forever でない && elapsed_ms が total_ms に到達した (時間切れ)
 */
void Straight::runSerpentine(int left_base, int right_base, int steer,
                             uint32_t phase_ms, uint32_t total_ms,
                             PollFn poll)
{
  /* phase_ms == 0 は周期が定義できないので何もせず停止 */
  if (phase_ms == 0U) {
    stop();
    return;
  }

  /* total_ms == SerpentineForever のときは時間制限なしモード */
  const bool run_forever = (total_ms == SerpentineForever);
  if (!run_forever && total_ms > MaxDurationMs) {
    total_ms = MaxDurationMs;
  }

  uint32_t elapsed_ms = 0U;        // 全体の経過時間
  uint32_t phase_elapsed_ms = 0U;  // 今のフェーズ (左寄り/右寄り) の経過時間
  bool steer_left = true;          // true: 左寄りステア, false: 右寄りステア

  while (1) {
    /* --- 終了判定 (ループ先頭で必ずチェック) --- */
    if (poll != nullptr && poll()) {
      break;  // 外部からの停止要求
    }
    if (!run_forever && elapsed_ms >= total_ms) {
      break;  // 時間切れ
    }

    /* --- パワー計算: 現在のステア方向に応じて左右の差を作る --- */
    int left_power;
    int right_power;
    if (steer_left) {
      left_power = left_base + steer;
      right_power = right_base - steer;
    } else {
      left_power = left_base - steer;
      right_power = right_base + steer;
    }

    run(left_power, right_power);

    /* --- ステップ時間だけ待つ。total_ms をはみ出さないよう端数調整 --- */
    uint32_t step_ms = SerpentineStepMs;
    if (!run_forever && elapsed_ms + step_ms > total_ms) {
      step_ms = total_ms - elapsed_ms;
    }
    dly_tsk((RELTIM)(step_ms * 1000U));
    elapsed_ms += step_ms;
    phase_elapsed_ms += step_ms;

    /* --- フェーズが終わったらステア方向を反転 --- */
    if (phase_elapsed_ms >= phase_ms) {
      phase_elapsed_ms = 0U;
      steer_left = !steer_left;
    }
  }

  stop();
}

/* テスト用タスク。app.cfg に CRE_TSK で登録されており、
 * STRAIGHT_TASK を act_tsk すると以下の動きをする:
 *   1) 1 秒前進
 *   2) その場で 0.5 秒右ピボット旋回
 *   3) もう 1 秒前進
 * 各動作は runFor によりブロッキングで実行される。 */
extern "C" void straight_task(intptr_t unused)
{
  (void)unused;

  /* 1 秒前進 (左右パワーはデフォルト値) */
  gMotors.runFor(Straight::DefaultLeftPower, Straight::DefaultRightPower, 1000U);
  /* その場で 0.5 秒右ピボット旋回 (左前進・右逆回転) */
  gMotors.runFor(50, -50, 500U);
  /* もう 1 秒前進 */
  gMotors.runFor(Straight::DefaultLeftPower, Straight::DefaultRightPower, 1000U);

  ext_tsk();
}
