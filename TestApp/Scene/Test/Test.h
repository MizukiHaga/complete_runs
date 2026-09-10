#ifndef TESTAPP_TEST_H_
#define TESTAPP_TEST_H_

#include <kernel.h>
#include <pbio/port.h>
#include <stdint.h>

#include "app.h"

/* Nansho3 のカルマン Q/R をそのまま使うため（シーン11・12） */
#include "Nansho3.h"

class TestScene {
public:
  /* ETROBO_SCENE==6: スカイブルーLT → 時間制御直進 → Nansho1〜3
   * ETROBO_SCENE==7: 時間制御直進のみ（0°・ヨー補正・減速なし）
   * ETROBO_SCENE==8: 直進＋90°旋回を繰り返す四角走行（無限ループ）
   * ETROBO_SCENE==9: 前後往復（カルマン効果確認・台形減速）
   * ETROBO_SCENE==10: 直進のみ（0°・ヨー補正・台形減速・カルマンログ）
   * ETROBO_SCENE==11: Nansho3 直進と同じパワー・PI（0°・周期補正・距離で終了・減速なし）
   * ETROBO_SCENE==12: 直進のみ（Nansho3 と同じ距離枠・クルーズ後に減速して coast。旋回なし） */
  void run();

private:
  void runLineTraceUntilSkyblueThenStraight();
  void runStraightOnly();
  void runStraightOnlyTest();
  void runNansho3StraightTest();
  void runNansho3DecelStraightTest();
  void runSquareLoop();
  void runKalmanPingPongLoop();
  void logGyroKalman(const char *label);
  void runBalancedStraightWithDecelAndLog(int left_power, int right_power,
                                          float target_heading, float kp,
                                          float ki, uint32_t duration_ms,
                                          uint32_t decel_ms, const char *label,
                                          uint32_t log_interval_ms);
  void turnToHeading(float target_heading, float tolerance_deg);
  void turnToHeading(float target_heading, float tolerance_deg,
                     int left_power, int right_power);

  static constexpr uint32_t StepMs = 1U;

  /* === スカイブルー到達前の LT（HCR catch_blue_line 相当） === */
  static constexpr float TracePower         = 80.0f;
  static constexpr float TraceKp            = 0.07f;
  static constexpr float TraceKi            = 0.0006f;
  static constexpr float TraceKd            = 0.5f;
  static constexpr float TraceOutMax        = 100.0f;
  static constexpr float TraceScale         = 16.0f;
  /* L=右エッジ, R=左エッジ */
  static constexpr bool  TraceEdgeFollow = (ETROBO_COURSE_L == 0);

  /* === スカイブルー検知後の直進（0°補正、時間で終了） === */
  static constexpr int      AfterSkybluePower         = 35;
  static constexpr float    AfterSkyblueBalanceKp     = 0.35f;
  static constexpr float    AfterSkyblueBalanceKi     = 0.01f;
  static constexpr uint32_t AfterSkyblueStraightMs    = 800U;

  /* === 直進のみ（ETROBO_SCENE==7） === */
  static constexpr int      StraightOnlyPower         = 50;
  static constexpr float    StraightOnlyBalanceKp     = 5.0f;
  static constexpr float    StraightOnlyBalanceKi     = 0.3f;
  static constexpr uint32_t StraightOnlyMs            = 600U;

  /* === 直進テスト（ETROBO_SCENE==10） === */
  static constexpr int      StraightTestPower         = 75;
  static constexpr float    StraightTestBalanceKp     = 1.5f;
  static constexpr float    StraightTestBalanceKi     = 0.01f;
  static constexpr uint32_t StraightTestMs            = 640U;
  static constexpr uint32_t StraightTestDecelMs       = 400U;
  static constexpr uint32_t StraightTestLogIntervalMs = 200U;
  static constexpr float    StraightTestProcessNoise  = 8.0f;
  static constexpr float    StraightTestMeasNoise     = 25.0f;

  /* === 四角走行（ETROBO_SCENE==8） === */
  static constexpr int      SquareStraightPower       = 50;
  static constexpr float    SquareBalanceKp           = 1.0f;
  static constexpr float    SquareBalanceKi           = 0.02f;
  static constexpr uint32_t SquareStraightMs          = 500U; /* 1辺の直進時間 [ms] */
  static constexpr float    SquareRelativeTurnDeg     = 90.0f; /* 相対角 +90° (CW) */
  /* 旋回（Nansho1::turnToHeading と同じパラメータ） */
  static constexpr int      SquareTurnPower           = 40;
  static constexpr float    SquareTurnToleranceDeg    = 2.0f;
  static constexpr float    SquareTurnSlowApproachDeg = 30.0f;
  static constexpr int      SquareTurnSlowPower       = 33;
  static constexpr uint32_t SquareTurnMaxMs           = 8000U;

  /* === 前後往復・カルマン確認（ETROBO_SCENE==9） === */
  static constexpr int      KalmanPingPongPower       = 50;
  static constexpr float    KalmanBalanceKp           = 1.1f;
  static constexpr float    KalmanBalanceKi           = 0.001f;
  static constexpr uint32_t KalmanForwardMs           = 800U;
  static constexpr uint32_t KalmanBackMs              = 800U;
  static constexpr uint32_t KalmanDecelMs             = 400U;
  static constexpr uint32_t KalmanDecelStepMs         = 40U;
  static constexpr uint32_t KalmanLogIntervalMs       = 200U;
  /* カルマン Q/R（Gyro::setKalmanNoise）。実機で raw と kf の差を見て調整 */
  static constexpr float    KalmanTestProcessNoise    = 8.0f;
  static constexpr float    KalmanTestMeasNoise       = 25.0f;

  /* === Nansho3 直進コピー（ETROBO_SCENE==11）
   * 目標ヨーは常に 0° を明示指定（走り出しで現在角を掴み直さない） === */
  static constexpr int      Nansho3StraightPower    = 75;
  static constexpr float    Nansho3StraightKp       = 3.0f;
  static constexpr float    Nansho3StraightKi       = 0.07f;
  static constexpr float    Nansho3StraightIMax     = 50.0f;
  static constexpr float    Nansho3StraightTargetDeg = 0.0f;
  /* 走行距離 [deg]。左右エンコーダの絶対値平均で判定する。
   * 旧設定のパワー75・750ms 相当 */
  static constexpr int32_t  Nansho3StraightDeg      = 490;
  /* カルマン Q/R は Nansho3 本番と同じ。ここで別値にすると
   * 追い込んだ結果が Nansho3 に移らない */
  static constexpr float    Nansho3StraightProcessNoise =
      Nansho3Scene::KalmanProcessNoise;
  static constexpr float    Nansho3StraightMeasNoise    =
      Nansho3Scene::KalmanMeasNoise;

  /* === Nansho3 直進の減速コピー（ETROBO_SCENE==12）
   * 直進のみ。旋回なし。枠 CoreStraightDeg、クルーズ後 decelToStop+coast === */
  static constexpr int      Nansho3DecelPower         = 75;
  static constexpr float    Nansho3DecelKp            = 3.0f;
  static constexpr float    Nansho3DecelKi            = 0.07f;
  static constexpr float    Nansho3DecelIMax          = 18.0f;
  static constexpr float    Nansho3DecelDeadbandDeg   = 0.8f;
  /* Nansho3 の CoreStraightDeg / CoreStraightDecelDeg と同じ値にする */
  static constexpr int32_t  Nansho3DecelTotalDeg      = 370;
  static constexpr int32_t  Nansho3DecelDeg           = 150;
  static constexpr uint32_t Nansho3DecelMs            = 400U;
  static constexpr uint32_t Nansho3DecelStepMs        = 100U;
  /* カルマン Q/R は Nansho3 本番と同じ */
  static constexpr float    Nansho3DecelProcessNoise  =
      Nansho3Scene::KalmanProcessNoise;
  static constexpr float    Nansho3DecelMeasNoise     =
      Nansho3Scene::KalmanMeasNoise;
};

#ifdef __cplusplus
extern "C" {
#endif

void TestTask(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_TEST_H_
