#ifndef TESTAPP_TEST_H_
#define TESTAPP_TEST_H_

#include <kernel.h>
#include <pbio/port.h>
#include <stdint.h>

#include "app.h"
#include "TurnDrift.h"
#include "../Nansho3/Nansho3.h"

/* === テストケース切り替え（ETROBO_SCENE=6 で Test に入ってから、ここで選ぶ） ===
 * TEST_CASE:
 *   6: スカイブルーLT → Nansho3相当直進 → Nansho1〜3
 *   7: Nansho3相当直進（S字加速→クルーズ→S字減速→coast。方位PI）
 *   8: エンコーダ基準直進（距離・S字は 7 と同じ。左右進み差で補正）
 *  14: その場旋回ループ（IMU 90刻み or エンコーダ。TEST_TURN_LEFT）
 *  15: その場旋回1回（同上）
 *  16: その場旋回を左右ランダム（14 と同じ止め方・ログ）
 *  17: 直進と90°旋回で四角走行（TEST_TURN_LEFT で左右）
 *  19: Nansho3 initial（白検知LT→停止・reset→バック→旋回→直進停止）
 *  20: 通常直進を TEST_STRAIGHT_CELLS マス分（座標更新なし） */
#ifndef TEST_CASE
#define TEST_CASE 17
#endif

#ifndef TEST_TURN_LEFT
#define TEST_TURN_LEFT 0 /* 1=左(CCW -90°) 0=右(CW +90°) */
#endif

#ifndef TEST_STRAIGHT_CELLS
#define TEST_STRAIGHT_CELLS 2
#endif
#if TEST_STRAIGHT_CELLS < 1 || TEST_STRAIGHT_CELLS > 3
#error "TEST_STRAIGHT_CELLS must be 1, 2 or 3"
#endif

class TestScene {
public:
  void run();

  /* 直進中: 開始基準からの左右エンコーダ絶対値と平均 [deg] */
  void logStraightEncoders(int32_t left_base, int32_t right_base,
                           const char *phase);

private:
  void runLineTraceUntilSkyblueThenStraight();
  void runStraightTest();
  /* TEST_CASE 8: エンコーダ左右差で直進補正 */
  void runEncoderStraightTest();
  /* S字加速→クルーズ→S字減速→coast。
   * encoder_balance なら方位PIではなく左右エンコーダ差。 */
  void driveStraight(float target_heading, int32_t total_deg, int power,
                     bool encoder_balance = false, bool initial_settings = false);
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
  void runSquareTest();
  void waitLogConnected();
  void runNansho3OnceTurn();
  void turnNansho3SpinOnce(float target, bool left);
  void turnNansho3SpinByEncoder(bool left);
  float spinGridTarget(float grid_deg) const;
  void addFixedTurnCorrection(bool left);
  float heading_add_ = 0.0f;
  int left_turn_count_ = 0;
  int right_turn_count_ = 0;
  void logAfterTurn(int32_t left_abs_deg, int32_t right_abs_deg,
                    float heading_rel_deg, bool left);
  /* 停止指令直前スナップショット。値は coast 直前。 */
  void logStopInstant();

  static constexpr uint32_t StepMs = (TEST_CASE == 17) ? Nansho3Scene::CoreStraightStepMs : 1U;
  static constexpr bool EnableLog = false;
  static constexpr uint32_t StraightEncLogIntervalMs = 200U;
  static constexpr uint32_t GyroResetSettleMs = 110U;
  /* Test17: 一辺の距離のみ独自設定。走行条件はNansho3の定数を参照。 */
  static constexpr int32_t SquareSideDeg = 1500;

  /* === スカイブルー到達前の LT（HCR catch_blue_line 相当） === */
  static constexpr float TracePower         = 80.0f;
  static constexpr float TraceKp            = 0.07f;
  static constexpr float TraceKi            = 0.0006f;
  static constexpr float TraceKd            = 0.5f;
  static constexpr float TraceOutMax        = 100.0f;
  static constexpr float TraceScale         = 16.0f;
  static constexpr bool  TraceEdgeFollow = (ETROBO_COURSE_L == 0);

  /* === Nansho3 CoreStraight 相当（S字加速→クルーズ→S字減速） === */
  static constexpr int      StraightPower           = (TEST_CASE == 17) ? Nansho3Scene::CoreRunStraightPower : 100;
  static constexpr float    StraightKp              = (TEST_CASE == 17) ? Nansho3Scene::StraightBalanceKp : 1.5f;
  static constexpr float    StraightKi              = (TEST_CASE == 17) ? Nansho3Scene::StraightBalanceKi : 0.06f;
  static constexpr float    StraightIMax            = (TEST_CASE == 17) ? Nansho3Scene::StraightBalanceIMax : 40.0f;
  static constexpr float    StraightDeadbandDeg     = (TEST_CASE == 17) ? Nansho3Scene::StraightBalanceDeadbandDeg : 0.0f;
  static constexpr float    StraightTargetDeg       = 0.0f;
  static constexpr int32_t  StraightTotalDeg        = 475;
  static constexpr int32_t  StraightAccelDeg        = (TEST_CASE == 17) ? Nansho3Scene::CoreStraightAccelDeg : 150;
  static constexpr int      StraightAccelStartPower = (TEST_CASE == 17) ? Nansho3Scene::CoreStraightAccelStartPower : 40;
  static constexpr int32_t  StraightDecelDeg        = (TEST_CASE == 17) ? Nansho3Scene::CoreStraightDecelDeg : 250;
  static constexpr int      StraightDecelMinPower   = (TEST_CASE == 17) ? Nansho3Scene::CoreStraightDecelMinPower : 40;
  static constexpr uint32_t StraightDecelStallMs    = (TEST_CASE == 17) ? Nansho3Scene::CoreStraightDecelStallMs : 250U;
  static constexpr uint32_t StraightDecelMaxMs      = (TEST_CASE == 17) ? Nansho3Scene::CoreStraightDecelMaxMs : 2500U;

  /* Nansho3 initialRun 相当の直進設定。 */
  static constexpr int      InitialStraightPower = 50;
  static constexpr int32_t  InitialStraightDeg = 610;
  static constexpr int32_t  InitialStraightDecelDeg = 100;
  static constexpr float    InitialStraightBalanceIMax = 18.0f;
  static constexpr float    InitialStraightBalanceDeadbandDeg = 0.8f;

  /* === Nansho3 その場旋回（TEST_CASE 14/15/16） === */
  static constexpr bool     Nansho3SpinStopOnEncoder    = false;
  // エンコーダのみで停止する専用テストを除き、Nansho3と補正方式を共有。
  static constexpr bool Nansho3UseTurnDrift =
      NANSHO3_USE_TURN_DRIFT && !Nansho3SpinStopOnEncoder;
  /* その場 90° に相当する片輪エンコーダ [deg]。実測 183〜186 の中央 */
  static constexpr float    Nansho3SpinEncoderPer90Deg = TurnDriftConfig::EncoderPer90Deg;
  static constexpr int32_t  Nansho3SpinEncoderDeg       = 172;
  static constexpr int32_t  Nansho3SpinEncoderSlowDeg   = 90;
  static constexpr int32_t  Nansho3SpinEncoderEarlyDeg  = 0;
  static constexpr int      Nansho3SpinPower            = (TEST_CASE == 17) ? Nansho3Scene::CoreTurnPower : 80;
  static constexpr float    Nansho3SpinSlowApproachDeg  = (TEST_CASE == 17) ? Nansho3Scene::CoreTurnSlowApproachDeg : 30.0f;
  static constexpr int      Nansho3SpinSlowPower        = (TEST_CASE == 17) ? Nansho3Scene::CoreTurnSlowPower : 40;
  static constexpr uint32_t Nansho3SpinMaxMs            = (TEST_CASE == 17) ? Nansho3Scene::CoreTurnMaxMs : 8000U;
  static constexpr uint32_t Nansho3SpinBeforeMs         = (TEST_CASE == 17) ? Nansho3Scene::CoreTurnBeforeMs : 200U;
  static constexpr float    Nansho3SpinEarlyStopDeg     = (TEST_CASE == 17) ? Nansho3Scene::CoreTurnEarlyStopDeg : 3.0f;
  static constexpr float    Nansho3SpinLeftEarlyExtraDeg  = (TEST_CASE == 17) ? Nansho3Scene::CoreTurnLeftEarlyExtraDeg : -1.0f;
  static constexpr float    Nansho3SpinRightEarlyExtraDeg = (TEST_CASE == 17) ? Nansho3Scene::CoreTurnRightEarlyExtraDeg : -1.0f;
  static constexpr uint32_t Nansho3SpinAfterMs          = (TEST_CASE == 17) ? Nansho3Scene::CoreTurnAfterMs : 0U;
  static constexpr float Nansho3SpinToleranceDeg =
      (TEST_CASE == 17) ? Nansho3Scene::CoreTurnToleranceDeg : 0.0f;
  /* TurnDrift有効時は固定値補正を加算しない。 */
};

#ifdef __cplusplus
extern "C" {
#endif

void TestTask(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_TEST_H_
