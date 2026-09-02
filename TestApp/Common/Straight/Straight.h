/*
 * Straight モジュール ヘッダ
 *  左右モータをまとめて操作する直進系の共通モジュール。
 *  pup_motor_* の C API は Straight クラスの中だけで使い、
 *  外からは run/stop/runFor などのメソッドで操作する。
 *
 *  直進補正:
 *    左右パワーが同じ (1:1) のとき、目標ヨー角を維持するよう
 *    gGyro の角度ずれを PI 補正する。目標角は run() 開始時の向きか、
 *    setTargetHeading() で指定した角度。弧走行・ピボットでは補正しない。
 *
 *  使い方:
 *    Straight gMotors;
 *    gGyro.reset();
 *    gMotors.configure(PBIO_PORT_ID_B, PBIO_PORT_ID_A);
 *    gMotors.setTargetHeading(0.0f);   // 任意: 目標角を明示指定
 *    gMotors.setBalancePid(0.35f, 0.01f);
 *    gMotors.run(50, 50);
 *    gMotors.stop();
 *    gMotors.runFor(50, 50, 720);  // 左右モータ各 720° 回転で停止
 */

#ifndef TESTAPP_STRAIGHT_H_
#define TESTAPP_STRAIGHT_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include <spike/pup/motor.h>
}

class Straight {
public:
  static constexpr uint32_t DecelMs = 800U;
  static constexpr uint32_t DecelStepMs = 40U;
  static constexpr uint32_t MaxDurationMs = 4000000U;

  /* === ヨー角 PI 直進補正 (既定値) ==========================
   * error = 現在角 − 目標角 [deg] (±180 に折り畳み)
   * 右 = + なので error>0 は右ずれ → 左旋回 (L−trim / R+trim)
   * 後退時も同じ式（負パワーで L−＝後退強、R+＝後退弱） */
  static constexpr float    DefaultBalanceKp   = 0.35f;
  static constexpr float    DefaultBalanceKi   = 0.01f;
  static constexpr float    BalanceIntegralMax = 50.0f;
  /* 1 周期あたりの左右パワー差 |L-R| の上限。
   * ここが小さいと Kp を上げても trim が頭打ちになり挙動が変わらない。 */
  static constexpr int      MaxHeadingTrim     = 40;
  static constexpr uint32_t BalanceStepMs      = 10U;

  void configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port);

  /* 直進補正の目標ヨー角 [deg] を指定する (integral をリセット)。
   * 未指定時は 1:1 直進開始時の gGyro.heading() が目標になる。
   * stop() / resetCorrection() で自動目標に戻る。 */
  void setTargetHeading(float heading_deg);

  /* ヨー角 PI ゲインを実行時に変更する。設定時に積分項をリセット。
   * i_max: 積分項クランプ上限 [deg] (省略時 BalanceIntegralMax)。 */
  void setBalancePid(float kp, float ki, float i_max = BalanceIntegralMax);

  /* シリアルに tgt/cur/err/trim/L/R を出す（調整用）。 */
  void setCorrectionDebug(bool enabled) { correction_debug_ = enabled; }

  /* ヨー角 PI 直進補正の有効/無効。false なら 1:1 直進も open-loop。
   * stop() とは別（stop 中の出力抑止は correction_enabled_ 側）。 */
  void setHeadingCorrection(bool enabled);

  static constexpr uint32_t EncoderStepMs      = 5U;
  /* エンコーダ未到達時の安全打ち切り [ms]（距離制御には使わない） */
  static constexpr uint32_t EncoderRunMaxMs    = 5000U;

  void run(int left_power, int right_power);
  void stop();
  /* 左右モータのエンコーダ [deg] で直進／ピボットを終了する。 */
  void runFor(int left_power, int right_power, int32_t target_deg);
  /* 指定時間 [ms] 走行して減速停止（後退など時間制御向け）。 */
  void runForMs(int left_power, int right_power, uint32_t duration_ms);

  /* 目標ヨー角までその場旋回する（直進補正は使わない）。
   * diff>0 → CW (L+, R−) / diff<0 → CCW (L−, R+)
   * tolerance_deg 以内で終了。max_ms で安全打ち切り。 */
  void turnToHeading(float target_heading,
                     float tolerance_deg = 4.0f,
                     int left_power = 40,
                     int right_power = 40,
                     uint32_t max_ms = 3000U);

  void pivotCW(int power)  { run( power, -power); }
  void pivotCCW(int power) { run(-power,  power); }
  void pivotTowardDiff(int power, float diff_deg) {
    if      (diff_deg > 0.0f) pivotCW(power);
    else if (diff_deg < 0.0f) pivotCCW(power);
    else                      stop();
  }

  void tickCorrection();

  void resetEncoderCount();
  int32_t straightEncoderDeg(int left_power, int right_power) const;
  bool straightEncoderReached(int left_power, int right_power,
                              int32_t target_deg) const;
  /* ヨー補正つきエンコーダ直進の開始（brake→reset、周期補正停止）。 */
  void beginBalancedEncoderRun();
  /* ヨー補正つきエンコーダ直進の終了（brake→停止状態同期）。 */
  void endBalancedEncoderRun();
  /* ヨー補正つきでエンコーダ到達まで走行する。 */
  void runBalancedEncoderFor(int left_power, int right_power,
                             int32_t target_deg, uint32_t step_ms);

private:
  static constexpr uint32_t TurnStepMs = 10U;

  static int clampMotorPower(int power);
  static int clampInt(int value, int min_value, int max_value);
  static float clampFloat(float value, float min_value, float max_value);
  static bool samePowerRatio(int left, int right, int prev_left, int prev_right);
  static bool isSameStraightPower(int left_power, int right_power);
  static int32_t absEncoderDeg(int32_t count);
  static bool wheelEncoderReached(int32_t count, int power, int32_t target_deg);

  void setPowerRaw(int left_power, int right_power);
  void applyWithCorrection(int left_power, int right_power);
  void resetCorrection();
  void syncStoppedState();
  bool encoderReached(int left_power, int right_power, int32_t target_deg);

  pup_motor_t *left_ = nullptr;
  pup_motor_t *right_ = nullptr;

  float   target_heading_   = 0.0f;
  float   integral_         = 0.0f;
  int     last_cmd_left_    = 0;
  int     last_cmd_right_   = 0;
  bool    cmd_initialized_  = false;
  bool    correction_enabled_ = true;
  bool    heading_correction_ = true;
  bool    correction_debug_   = false;
  bool    manual_target_heading_ = false;
  uint8_t correction_debug_div_ = 0;
  float   kp_               = DefaultBalanceKp;
  float   ki_               = DefaultBalanceKi;
  float   i_max_            = BalanceIntegralMax;
};

#endif  // TESTAPP_STRAIGHT_H_
