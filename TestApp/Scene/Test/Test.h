#ifndef TESTAPP_TEST_H_
#define TESTAPP_TEST_H_

#include <kernel.h>
#include <pbio/port.h>
#include <stdint.h>

#include "app.h"

class TestScene {
public:
  /* ライントレース走行。
   * 終了条件:
   *   スカイブルー検知 → 0°直進を時間で終了。
   * ETROBO_SCENE==6 では完了後に Nansho1 → Nansho2 → Nansho3 へ続く。 */
  void run();

private:
  void runLineTraceUntilSkyblueThenStraight();

  static constexpr uint32_t StepMs = 1U;

  /* === スカイブルー到達前の LT（HCR catch_blue_line 相当） === */
  static constexpr float TracePower         = 80.0f;
  static constexpr float TraceKp            = 0.07f;
  static constexpr float TraceKi            = 0.0006f;
  static constexpr float TraceKd            =0.5f;
  static constexpr float TraceOutMax        = 100.0f;
  static constexpr float TraceScale         = 16.0f;
  /* L=右エッジ, R=左エッジ */
  static constexpr bool  TraceEdgeFollow = (ETROBO_COURSE_L == 0);

  /* === スカイブルー検知後の直進（0°補正、時間で終了） === */
  static constexpr int      AfterSkybluePower     = 35;
  static constexpr float    AfterSkyblueBalanceKp = 0.35f;
  static constexpr float    AfterSkyblueBalanceKi = 0.01f;
  static constexpr uint32_t AfterSkyblueStraightMs = 800U;
};

#ifdef __cplusplus
extern "C" {
#endif

void TestTask(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_TEST_H_
