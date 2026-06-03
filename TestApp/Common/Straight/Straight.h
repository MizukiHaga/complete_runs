/*
 * Straight モジュール ヘッダ
 *  左右モータをまとめて操作する直進系の共通モジュール。
 *  pup_motor_* の C API は Straight クラスの中だけで使い、
 *  外からは run/stop/runFor などのメソッドで操作する。
 *
 *  使い方:
 *    Straight gMotors;
 *    gMotors.configure(PBIO_PORT_ID_B, PBIO_PORT_ID_A);   // 左, 右
 *    gMotors.run(70, 65);              // 走る
 *    gMotors.stop();                   // 止める
 *    gMotors.runFor(70, 65, 2000);     // 2 秒走って減速停止
 */

#ifndef TESTAPP_STRAIGHT_H_
#define TESTAPP_STRAIGHT_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

/* PUP の C API は C リンクで提供されているので extern "C" で囲む */
extern "C" {
#include <spike/pup/motor.h>
}

class Straight {
public:
  /* runFor の最後で線形減速にかける時間 [ms] */
  static constexpr uint32_t DecelMs = 800U;
  /* 減速中のパワー更新間隔 [ms] (小さいほど滑らかだが負荷が増える) */
  static constexpr uint32_t DecelStepMs = 40U;
  /* dly_tsk の上限 TMAX_RELTIM (4,000,000 ms) を超えないようにするためのクランプ */
  static constexpr uint32_t MaxDurationMs = 4000000U;

  /* === 左右エンコーダ PID 補正 (既定値) ==================== */
  /* エンコーダ差分 error に対する比例ゲイン */
  static constexpr float    DefaultBalanceKp   = 0.10f;
  /* 積分ゲイン (定常ズレを消す) */
  static constexpr float    DefaultBalanceKi   = 0.02f;
  /* 積分項の上限 (ワインドアップ防止) */
  static constexpr float    BalanceIntegralMax = 200.0f;
  /* run / runFor 内の補正更新周期 [ms] */
  static constexpr uint32_t BalanceStepMs      = 10U;

  /* === API ================================================ */
  /* 初期化: 左右モータデバイスを取得し、回転方向を設定する。
   * 第3引数 true でエンコーダをリセット。 */
  void configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port);

  /* 左右モータに指定パワーをセットする。すぐ戻る。
   * 正: 前進方向 / 負: 後退方向 / 0: 惰性停止 (ブレーキではない) */
  void run(int left_power, int right_power);

  /* 左右モータのパワーを 0 にする (惰性停止)。 */
  void stop();

  /* 指定パワーで duration_ms だけ走り、最後は線形減速で 0 まで落として停止する (ブロッキング)。
   * duration_ms == 0 のときは停止のみ。
   * MaxDurationMs を超える指定は MaxDurationMs にクランプ。 */
  void runFor(int left_power, int right_power, uint32_t duration_ms);

  /* === ピボット旋回ヘルパ ====================================
   * その場で左右逆符号同パワーを与えて旋回する (中心を変えない旋回)。
   * power は常に正の値で渡すこと (内部で符号を組む)。 */

  /* CW 方向 (機体上から見て時計回り、ヨー角が増える側) にピボット旋回。 */
  void pivotCW(int power)  { run( power, -power); }
  /* CCW 方向 (反時計回り、ヨー角が減る側) にピボット旋回。 */
  void pivotCCW(int power) { run(-power,  power); }
  /* 目標との角度差 diff_deg に応じて短い側にピボット旋回する。
   *   diff_deg > 0 -> pivotCW
   *   diff_deg < 0 -> pivotCCW
   *   diff_deg = 0 -> stop
   * power は常に正の値で渡すこと。 */
  void pivotTowardDiff(int power, float diff_deg) {
    if      (diff_deg > 0.0f) pivotCW(power);
    else if (diff_deg < 0.0f) pivotCCW(power);
    else                      stop();
  }

  /* バックグラウンド周期タスクから BalanceStepMs ごとに呼ぶ。
   * 最後に run() でセットされたコマンドをエンコーダ PID 補正付きで再適用する。
   * 未 configure / 未コマンド / (0, 0) のときは何もしない。 */
  void tickCorrection();

private:
  /* モータパワーを [-100, 100] に丸める。範囲外の値が来てもクラッシュしないため。 */
  static int clampMotorPower(int power);
  static float clampFloat(float value, float min_value, float max_value);

  /* 補正なしで set_power し、前回エンコーダ値を更新する。 */
  void setPowerRaw(int left_power, int right_power);

  /* エンコーダ PID で左右パワーを補正して set_power する。 */
  void applyWithCorrection(int left_power, int right_power);

  /* 積分・前回エンコーダ・前回コマンドをリセットする。 */
  void resetCorrection();

  /* 取得した左右モータデバイス。configure() で初期化、以降の関数で使い回す。 */
  pup_motor_t *left_ = nullptr;
  pup_motor_t *right_ = nullptr;

  /* --- 補正内部状態 --- */
  int32_t prev_cL_         = 0;
  int32_t prev_cR_         = 0;
  float   integral_        = 0.0f;
  int     last_cmd_left_   = 0;
  int     last_cmd_right_  = 0;
  bool    cmd_initialized_ = false;
  bool    correction_enabled_ = true;
  bool    correction_debug_   = false;
  float   kp_              = DefaultBalanceKp;
  float   ki_              = DefaultBalanceKi;
  float   i_max_           = BalanceIntegralMax;
};

#endif  // TESTAPP_STRAIGHT_H_
