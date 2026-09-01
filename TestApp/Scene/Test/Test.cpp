#include "Test.h"

#include "app.h"
#include "kernel_cfg.h"

#include "ColorLAB.h"
#include "Common/LineTracer/LineTracer.h"
#include "Common/Straight/Straight.h"
#include "Gyro.h"

extern ColorLAB   gColorLAB;
extern Gyro       gGyro;
extern Straight   gMotors;
extern LineTracer gLineTracer;
extern TestScene  gTest;

/* スカイブルー検知 → 0°直進を時間で終了 */
void TestScene::runLineTraceUntilSkyblueThenStraight()
{
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(TracePower, TracePower);
  gLineTracer.setPid(TraceKp, TraceKi, TraceKd, TraceOutMax);
  gLineTracer.setScale(TraceScale);
  gLineTracer.setEdgeFollow(TraceEdgeFollow);

  while (1) {
    if (gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
      gLineTracer.setEnabled(false);
      gMotors.stop();
      gGyro.reset();
      gMotors.setBalancePid(AfterSkyblueBalanceKp, AfterSkyblueBalanceKi);
      gMotors.setTargetHeading(0.0f);
      gMotors.run(AfterSkybluePower, AfterSkybluePower);

      uint32_t after_skyblue_ms = 0U;
      while (after_skyblue_ms < AfterSkyblueStraightMs) {
        gMotors.tickCorrection();
        dly_tsk(StepMs * 1000U);
        after_skyblue_ms += StepMs;
      }
      gMotors.stop();
      dly_tsk(StepMs * 1000U);
      return;
    }
    gLineTracer.step();
    dly_tsk(5 * 1000U);
  }
}

void TestScene::run()
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

  runLineTraceUntilSkyblueThenStraight();
}

extern "C" void TestTask(intptr_t exinf)
{
  (void)exinf;
  gTest.run();

#if ETROBO_SCENE == 6
  gMotors.stop();
  gLineTracer.setEnabled(false);
  dly_tsk(50 * 1000U);
  act_tsk(NANSHO1_TASK);
#endif

  ext_tsk();
}
