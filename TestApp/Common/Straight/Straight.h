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
 *    gMotors.run(50, 50);
 *    gMotors.stop();
 *    gMotors.runFor(50, 50, 2000);
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
   * trim  = Kp * error + Ki * integral → 左右に ±trim/2 を分配 */
  static constexpr float    DefaultBalanceKp   = 1.8f;
  static constexpr float    DefaultBalanceKi   = 0.01f;
  static constexpr float    BalanceIntegralMax = 50.0f;
  /* 1 周期あたりの左右パワー差分上限 (|half| の上限) */
  static constexpr int      MaxHeadingTrim     = 12;
  static constexpr uint32_t BalanceStepMs      = 10U;

  void configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port);

  /* 直進補正の目標ヨー角 [deg] を指定する (integral をリセット)。
   * 未指定時は 1:1 直進開始時の gGyro.heading() が目標になる。
   * stop() / resetCorrection() で自動目標に戻る。 */
  void setTargetHeading(float heading_deg);

  void run(int left_power, int right_power);
  void stop();
  void runFor(int left_power, int right_power, uint32_t duration_ms);

  void pivotCW(int power)  { run( power, -power); }
  void pivotCCW(int power) { run(-power,  power); }
  void pivotTowardDiff(int power, float diff_deg) {
    if      (diff_deg > 0.0f) pivotCW(power);
    else if (diff_deg < 0.0f) pivotCCW(power);
    else                      stop();
  }

  void tickCorrection();

private:
  static int clampMotorPower(int power);
  static int clampInt(int value, int min_value, int max_value);
  static float clampFloat(float value, float min_value, float max_value);
  static bool samePowerRatio(int left, int right, int prev_left, int prev_right);

  void setPowerRaw(int left_power, int right_power);
  void applyWithCorrection(int left_power, int right_power);
  void resetCorrection();

  pup_motor_t *left_ = nullptr;
  pup_motor_t *right_ = nullptr;

  float   target_heading_   = 0.0f;
  float   integral_         = 0.0f;
  int     last_cmd_left_    = 0;
  int     last_cmd_right_   = 0;
  bool    cmd_initialized_  = false;
  bool    correction_enabled_ = true;
  bool    correction_debug_   = false;
  bool    manual_target_heading_ = false;
  float   kp_               = DefaultBalanceKp;
  float   ki_               = DefaultBalanceKi;
  float   i_max_            = BalanceIntegralMax;
};

#endif  // TESTAPP_STRAIGHT_H_
