#ifndef TESTAPP_TEST_H_
#define TESTAPP_TEST_H_

#include <kernel.h>
#include <pbio/port.h>
#include <stdint.h>

#include "app.h"

class TestScene {
public:
  /* ETROBO_SCENE==6: スカイブルーLT → Nansho3相当直進 → Nansho1〜3
   * ETROBO_SCENE==7: Nansho3相当直進のみ（S字加速→クルーズ→S字減速→coast。方位PI）
   * ETROBO_SCENE==8: エンコーダ基準直進のみ（距離・S字は 7 と同じ。左右進み差で補正）
   * ETROBO_SCENE==14: その場旋回ループ（IMU 90刻み or エンコーダ。TEST_TURN_LEFT）
   * ETROBO_SCENE==15: その場旋回1回（同上）
   * ETROBO_SCENE==16: その場旋回を左右ランダム（14 と同じ止め方・ログ） */
  void run();

  /* 直進中: 開始基準からの左右エンコーダ絶対値と平均 [deg] */
  void logStraightEncoders(int32_t left_base, int32_t right_base,
                           const char *phase);

private:
  void runLineTraceUntilSkyblueThenStraight();
  void runStraightTest();
  /* シーン8: エンコーダ左右差で直進補正 */
  void runEncoderStraightTest();
  /* S字加速→クルーズ→S字減速→coast。
   * encoder_balance なら方位PIではなく左右エンコーダ差。 */
  void driveStraight(float target_heading, int32_t total_deg, int power,
                     bool encoder_balance = false);
  void cruiseUntilDeg(int32_t left_base, int32_t right_base,
                      int32_t target_deg);
  void rampStraightPower(int32_t left_base, int32_t right_base, int32_t dist_deg,
                         int from_power, int to_power);
  void accelToCruise(int32_t left_base, int32_t right_base, int32_t accel_deg,
                     int cruise_power, int start_power);
  void decelToStopByEncoder(int32_t left_base, int32_t right_base,
                            int32_t total_deg, int cruise_power,
                            int32_t decel_deg);

  void runNansho3SpinLoop();
  void runRandomSpinLoop();
  void waitLogConnected();
  void runNansho3OnceTurn();
  void turnNansho3SpinOnce(float target, bool left);
  void turnNansho3SpinByEncoder(bool left);
  static float spinGridTarget(float grid_deg, float heading_add);
  static void accumulateSpinPer(bool left, int *turn_count, float *heading_add);
  void logAfterTurn(int32_t left_abs_deg, int32_t right_abs_deg,
                    float heading_rel_deg, bool left);

  static constexpr uint32_t StepMs = 1U;
  static constexpr bool EnableLog = true;
  static constexpr uint32_t StraightEncLogIntervalMs = 200U;
  static constexpr uint32_t GyroResetSettleMs = 80U;

  /* === スカイブルー到達前の LT（HCR catch_blue_line 相当） === */
  static constexpr float TracePower         = 80.0f;
  static constexpr float TraceKp            = 0.07f;
  static constexpr float TraceKi            = 0.0006f;
  static constexpr float TraceKd            = 0.5f;
  static constexpr float TraceOutMax        = 100.0f;
  static constexpr float TraceScale         = 16.0f;
  static constexpr bool  TraceEdgeFollow = (ETROBO_COURSE_L == 0);

  /* === Nansho3 CoreStraight 相当（S字加速→クルーズ→S字減速） === */
  static constexpr int      StraightPower           = 75;
  static constexpr float    StraightKp              = 1.5f;
  static constexpr float    StraightKi              = 0.06f;
  static constexpr float    StraightIMax            = 50.0f;
  static constexpr float    StraightDeadbandDeg     = 0.0f;
  static constexpr float    StraightTargetDeg       = 0.0f;
  static constexpr int32_t  StraightTotalDeg        = 1500;
  static constexpr int32_t  StraightAccelDeg        = 150;
  static constexpr int      StraightAccelStartPower = 33;
  static constexpr int32_t  StraightDecelDeg        = 150;
  static constexpr int      StraightDecelMinPower   = 33;
  static constexpr uint32_t StraightDecelStallMs    = 250U;
  static constexpr uint32_t StraightDecelMaxMs      = 2500U;

  /* === Nansho3 その場旋回（ETROBO_SCENE==14/15/16） === */
  static constexpr bool     Nansho3SpinStopOnEncoder    = false;
  /* その場 90° に相当する片輪エンコーダ [deg]。実測 183〜186 の中央 */
  static constexpr float    Nansho3SpinEncoderPer90Deg = 184.5f;
  static constexpr int32_t  Nansho3SpinEncoderDeg       = 172;
  static constexpr int32_t  Nansho3SpinEncoderSlowDeg   = 60;
  static constexpr int32_t  Nansho3SpinEncoderEarlyDeg  = 0;
  static constexpr int      Nansho3SpinPower            = 45;
  static constexpr float    Nansho3SpinSlowApproachDeg  = 30.0f;
  static constexpr int      Nansho3SpinSlowPower        = 33;
  static constexpr uint32_t Nansho3SpinMaxMs            = 8000U;
  static constexpr uint32_t Nansho3SpinBeforeMs         = 200U;
  static constexpr float    Nansho3SpinEarlyStopDeg     = 3.0f;
  static constexpr float    Nansho3SpinLeftEarlyExtraDeg  = 0.0f;
  static constexpr float    Nansho3SpinRightEarlyExtraDeg = -1.5f;
  static constexpr uint32_t Nansho3SpinAfterMs          = 0U;
  static constexpr uint32_t Nansho3SpinLogSettleMs      = 500U;
  static constexpr float    Nansho3SpinPerLeftAddDeg  =  0.8f;
  static constexpr float    Nansho3SpinPerRightAddDeg = -0.85f;
  static constexpr int      Nansho3SpinTurnsPerAdd      = 1;
};

#ifdef __cplusplus
extern "C" {
#endif

void TestTask(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_TEST_H_
