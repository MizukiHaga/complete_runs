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
  /* === 既定値 / 定数 ====================================== */
  /* 直進時のデフォルト左右パワー。機械差を吸収するため左右で別値にできる。 */
  static constexpr int DefaultLeftPower = 70;
  static constexpr int DefaultRightPower = 65;
  /* runFor で直進する既定時間 [ms] */
  static constexpr uint32_t DefaultTimeMs = 2000U;
  /* runFor の最後で線形減速にかける時間 [ms] */
  static constexpr uint32_t DecelMs = 800U;
  /* 減速中のパワー更新間隔 [ms] (小さいほど滑らかだが負荷が増える) */
  static constexpr uint32_t DecelStepMs = 40U;
  /* dly_tsk の上限 TMAX_RELTIM (4,000,000 ms) を超えないようにするためのクランプ */
  static constexpr uint32_t MaxDurationMs = 4000000U;

  /* === API ================================================ */
  /* 初期化: 左右モータデバイスを取得し、回転方向を設定する。
   * 第3引数 true でエンコーダをリセット。 */
  void configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port);

  /* 左右両方の PUP モータが取れているか。 */
  bool isConfigured() const { return left_ != nullptr && right_ != nullptr; }

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

private:
  /* モータパワーを [-100, 100] に丸める。範囲外の値が来てもクラッシュしないため。 */
  static int clampMotorPower(int power);

  /* 取得した左右モータデバイス。configure() で初期化、以降の関数で使い回す。 */
  pup_motor_t *left_ = nullptr;
  pup_motor_t *right_ = nullptr;
};

/* === 後方互換マクロ ======================================
 * 旧 C スタイルのコードから参照されている名前を、クラスの定数に橋渡しする。
 * 新規コードでは Straight::DefaultLeftPower のように直接書く方がよい。 */
#define STRAIGHT_LEFT_POWER        Straight::DefaultLeftPower
#define STRAIGHT_RIGHT_POWER       Straight::DefaultRightPower
#define STRAIGHT_TIME              Straight::DefaultTimeMs
#define STRAIGHT_DECEL_MS          Straight::DecelMs
#define STRAIGHT_DECEL_STEP_MS     Straight::DecelStepMs
#define STRAIGHT_MAX_DURATION_MS   Straight::MaxDurationMs

#endif  // TESTAPP_STRAIGHT_H_
