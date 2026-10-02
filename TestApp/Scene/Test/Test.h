#ifndef TESTAPP_TEST_H_
#define TESTAPP_TEST_H_

#include <kernel.h>
#include <pbio/port.h>
#include <stdint.h>

#include "app.h"

class TestScene {
public:
  /* ETROBO_SCENE==6: スカイブルーLT → 時間制御直進 → Nansho1〜3
   * ETROBO_SCENE==7: 時間制御直進のみ（0°・ヨー補正・減速なし）
   * ETROBO_SCENE==8: 直進＋90°旋回を繰り返す四角走行（無限ループ）
   * ETROBO_SCENE==9: 前後往復（カルマン効果確認・台形減速）
   * ETROBO_SCENE==10: 直進のみ（0°・ヨー補正・台形減速・カルマンログ）
   * ETROBO_SCENE==11: Nansho3 直進と同じパワー・PI（0°・周期補正・距離で終了・減速なし）
   * ETROBO_SCENE==12: 直進のみ（Nansho3 と同じ距離枠・クルーズ後に減速して coast。旋回なし）
   * ETROBO_SCENE==14: Nansho3 と同じその場旋回を繰り返し、1回ごとに停止（TEST_TURN_LEFT で左右）
   * ETROBO_SCENE==15: Nansho3 と同じその場旋回を1回だけ（TEST_TURN_LEFT で左右） */
  void run();

  /* カルマン Q/R を KalmanGain* 定数で Gyro に載せる。
   * どちらも 0 以下は無視される（Gyro 側の既定値のまま）。 */
  void setKalmanGain();

private:
  void runLineTraceUntilSkyblueThenStraight();
  void runStraightOnly();
  void runStraightOnlyTest();
  void runNansho3StraightTest();
  void runNansho3DecelStraightTest();
  void runNansho3SpinLoop();
  void runNansho3OnceTurn();
  void turnNansho3SpinOnce(float target, bool left);
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

  /* カルマン Q [deg^2/s] / R [deg^2]（setKalmanGain）。予測はエンコーダ差、観測はハブ方位。
   * Q↑R↓=ハブ追従、Q↓R↑=エンコーダ信頼 */
  static constexpr float KalmanGainProcessNoise = 80.0f;
  static constexpr float KalmanGainMeasNoise    = 200.0f;

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
  static constexpr float    StraightOnlyBalanceKp     = 0.0f;
  static constexpr float    StraightOnlyBalanceKi     = 0.0f;
  static constexpr uint32_t StraightOnlyMs            = 1500U;

  /* === 直進テスト（ETROBO_SCENE==10） === */
  static constexpr int      StraightTestPower         = 75;
  static constexpr float    StraightTestBalanceKp     = 1.5f;
  static constexpr float    StraightTestBalanceKi     = 0.03f;
  static constexpr uint32_t StraightTestMs            = 3000U;
  static constexpr uint32_t StraightTestDecelMs       = 400U;
  static constexpr uint32_t StraightTestLogIntervalMs = 200U;

  /* === 四角走行（ETROBO_SCENE==8） === */
  static constexpr int      SquareStraightPower       = 50;
  static constexpr float    SquareBalanceKp           = 1.0f;
  static constexpr float    SquareBalanceKi           = 0.02f;
  static constexpr uint32_t SquareStraightMs          = 500U; /* 1辺の直進時間 [ms] */
  /* 1辺ごとの相対旋回角 [deg]。CW 正 */
  static constexpr float    SquareRelativeTurnDeg     = 90.0f;
  /* 旋回（Nansho1::turnToHeading と同じパラメータ） */
  static constexpr int      SquareTurnPower           = 40;
  /* 残りがこれ以下なら着いたとみなす [deg] */
  static constexpr float    SquareTurnToleranceDeg    = 2.0f;
  /* 残りがこれ以下なら低速に落とす [deg] */
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

  /* === Nansho3 直進コピー（ETROBO_SCENE==11）
   * 目標ヨーは常に 0° を明示指定（走り出しで現在角を掴み直さない） === */
  static constexpr int      Nansho3StraightPower    = 75;
  static constexpr float    Nansho3StraightKp       = 3.0f;
  static constexpr float    Nansho3StraightKi       = 0.07f;
  static constexpr float    Nansho3StraightIMax     = 50.0f;
  /* 直進 PI の目標ヨー [deg]。CW 正。常に 0° 固定 */
  static constexpr float    Nansho3StraightTargetDeg = 0.0f;
  /* 走行距離 [deg]（左右エンコーダ絶対値平均）。ヨー角ではない。
   * 旧設定のパワー75・750ms 相当 */
  static constexpr int32_t  Nansho3StraightDeg      = 490;

  /* === Nansho3 直進の減速コピー（ETROBO_SCENE==12）
   * 直進のみ。旋回なし。枠 CoreStraightDeg、クルーズ後 decelToStop+coast === */
  static constexpr int      Nansho3DecelPower         = 75;
  static constexpr float    Nansho3DecelKp            = 3.0f;
  static constexpr float    Nansho3DecelKi            = 0.07f;
  static constexpr float    Nansho3DecelIMax          = 18.0f;
  /* |ヨー誤差| がこれ以下なら直進 PI を掛けない [deg] */
  static constexpr float    Nansho3DecelDeadbandDeg   = 0.8f;
  /* 1マス枠のエンコーダ距離 [deg]（Nansho3 CoreStraightDeg 相当）。ヨーではない */
  static constexpr int32_t  Nansho3DecelTotalDeg      = 370;
  /* 上のうち減速中に進むと見込む距離 [deg]（CoreStraightDecelDeg 相当） */
  static constexpr int32_t  Nansho3DecelDeg           = 150;
  static constexpr uint32_t Nansho3DecelMs            = 400U;
  static constexpr uint32_t Nansho3DecelStepMs        = 100U;

  /* === Nansho3 その場旋回（ETROBO_SCENE==14/15）
   * 目標は 0 / 90 / ±180 / -90 の絶対角。14 は繰り返す。15 は1回。
   * 左右は app.h の TEST_TURN_LEFT。 === */
  static constexpr int      Nansho3SpinPower            = 45;
  static constexpr float    Nansho3SpinSlowApproachDeg  = 30.0f;
  static constexpr int      Nansho3SpinSlowPower        = 40;
  static constexpr uint32_t Nansho3SpinMaxMs            = 8000U;
  static constexpr uint32_t Nansho3SpinBeforeMs         = 200U;
  /* 経過角打ち切りをこの角手前で止める [deg]（慣性＋h 遅れの行き過ぎ） */
  static constexpr float    Nansho3SpinEarlyStopDeg     = 3.0f;
  /* 左/右だけさらに手前（負なら目標より奥まで回す）[deg]。
   * 実際の停止は EarlyStop + この Extra */
  static constexpr float    Nansho3SpinLeftEarlyExtraDeg  = 3.0f;
  static constexpr float    Nansho3SpinRightEarlyExtraDeg = 1.3f;
  static constexpr uint32_t Nansho3SpinAfterMs          = 0U;
  /* 左旋回 N 回ごとに目標へ載せる量 [deg]。積まず、今回の分だけ */
  static constexpr float    Nansho3SpinPerLeftAddDeg  =  0.0f;
  /* 右旋回 N 回ごとに目標へ載せる量 [deg] */
  static constexpr float    Nansho3SpinPerRightAddDeg = 0.0f;
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
