#include "Test.h"

#include "app.h"
#include "kernel_cfg.h"

#include <math.h>
#include <stdio.h>

#include "ColorLAB.h"
#include "Common/LineTracer/LineTracer.h"
#include "Common/Straight/Straight.h"
#include "Gyro.h"

extern ColorLAB   gColorLAB;
extern Gyro       gGyro;
extern Straight   gMotors;
extern LineTracer gLineTracer;
extern TestScene  gTest;

static void runBalancedStraightFor(int left_power, int right_power,
                                 float target_heading, float kp, float ki,
                                 uint32_t duration_ms)
{
  static constexpr uint32_t kStepMs = 1U;

  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(kp, ki);
  gMotors.setTargetHeading(target_heading);
  gMotors.run(left_power, right_power);

  for (uint32_t elapsed_ms = 0U; elapsed_ms < duration_ms;
       elapsed_ms += kStepMs) {
    gMotors.tickCorrection();
    dly_tsk(kStepMs * 1000U);
  }

  gMotors.stop();
}

void TestScene::logGyroKalman(const char *label)
{
  const float raw  = gGyro.rawHeading();
  const float kf   = gGyro.heading();
  const float diff = Gyro::wrapDeg180(raw - kf);
  printf("%s raw=%d kf=%d diff=%d\n", label, (int)raw, (int)kf, (int)diff);
}

void TestScene::runBalancedStraightWithDecelAndLog(int left_power, int right_power,
                                                   float target_heading, float kp,
                                                   float ki, uint32_t duration_ms,
                                                   uint32_t decel_ms,
                                                   const char *label,
                                                   uint32_t log_interval_ms)
{
  if (duration_ms == 0U) {
    gMotors.stop();
    return;
  }
  if (decel_ms > duration_ms) {
    decel_ms = duration_ms;
  }
  const uint32_t cruise_ms = duration_ms - decel_ms;

  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(kp, ki);
  gMotors.setTargetHeading(target_heading);
  gMotors.run(left_power, right_power);

  logGyroKalman(label);

  for (uint32_t elapsed_ms = 0U; elapsed_ms < cruise_ms; elapsed_ms += StepMs) {
    gMotors.tickCorrection();
    if (log_interval_ms > 0U && elapsed_ms > 0U &&
        (elapsed_ms % log_interval_ms) == 0U) {
      logGyroKalman(label);
    }
    dly_tsk(StepMs * 1000U);
  }

  for (uint32_t t = 0U; t < decel_ms; t += KalmanDecelStepMs) {
    uint32_t slice = KalmanDecelStepMs;
    if (t + slice > decel_ms) {
      slice = decel_ms - t;
    }
    const uint32_t numer = decel_ms - t;
    const int left =
        (int)((int64_t)left_power * (int64_t)numer / (int64_t)decel_ms);
    const int right =
        (int)((int64_t)right_power * (int64_t)numer / (int64_t)decel_ms);
    if (left == 0 && right == 0) {
      break;
    }
    gMotors.run(left, right);
    gMotors.tickCorrection();
    dly_tsk(slice * 1000U);
  }

  gMotors.stop();
  logGyroKalman("stop");
}

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
      dly_tsk(StepMs * 1000U);
      gGyro.reset();
      runBalancedStraightFor(AfterSkybluePower, AfterSkybluePower, 0.0f,
                             AfterSkyblueBalanceKp, AfterSkyblueBalanceKi,
                             AfterSkyblueStraightMs);
      dly_tsk(StepMs * 1000U);
      return;
    }
    gLineTracer.step();
    dly_tsk(5 * 1000U);
  }
}

/* 0°・ヨー補正つき直進のみ（距離調整用・減速なし） */
void TestScene::runStraightOnly()
{
  gGyro.reset();
  runBalancedStraightFor(StraightOnlyPower, StraightOnlyPower, 0.0f,
                         StraightOnlyBalanceKp, StraightOnlyBalanceKi,
                         StraightOnlyMs);
  dly_tsk(StepMs * 1000U);
}

/* 0°維持の直進のみ（台形減速停止・カルマン raw/kf ログ） */
void TestScene::runStraightOnlyTest()
{
  gGyro.reset();
  gGyro.setKalmanNoise(StraightTestProcessNoise, StraightTestMeasNoise);
  gMotors.setBalancePid(StraightTestBalanceKp, StraightTestBalanceKi);

  printf("Straight test Q=%d R=%d ms=%u decel=%u\n",
         (int)StraightTestProcessNoise, (int)StraightTestMeasNoise,
         (unsigned)StraightTestMs, (unsigned)StraightTestDecelMs);

  runBalancedStraightWithDecelAndLog(
      StraightTestPower, StraightTestPower, 0.0f,
      StraightTestBalanceKp, StraightTestBalanceKi,
      StraightTestMs, StraightTestDecelMs, "fwd",
      StraightTestLogIntervalMs);
  dly_tsk(StepMs * 1000U);
}

/* Nansho3 直進コピー。目標 0° を先に確定してから周期補正を開始する */
void TestScene::runNansho3StraightTest()
{
  gGyro.reset();
  gGyro.setKalmanNoise(Nansho3StraightProcessNoise, Nansho3StraightMeasNoise);
  /* 周期補正は目標確定＋run のあと。先に回すと掴み直し競合の余地がある */
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);

  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(Nansho3StraightKp, Nansho3StraightKi,
                        Nansho3StraightIMax);
  gMotors.setTargetHeading(Nansho3StraightTargetDeg);
  const int32_t enc_left_base  = gMotors.leftEncoderCount();
  const int32_t enc_right_base = gMotors.rightEncoderCount();
  gMotors.run(Nansho3StraightPower, Nansho3StraightPower);
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);

  while (gMotors.traveledDegFrom(enc_left_base, enc_right_base) <
         Nansho3StraightDeg) {
    dly_tsk(StepMs * 1000U);
  }
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

/* 直進のみ。Nansho3 と同じ減速。目標 0° を先に確定してから周期補正を開始 */
void TestScene::runNansho3DecelStraightTest()
{
  gGyro.reset();
  gGyro.setKalmanNoise(Nansho3DecelProcessNoise, Nansho3DecelMeasNoise);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);

  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(Nansho3DecelKp, Nansho3DecelKi, Nansho3DecelIMax,
                        Nansho3DecelDeadbandDeg);
  gMotors.setTargetHeading(0.0f);
  const int32_t enc_left_base  = gMotors.leftEncoderCount();
  const int32_t enc_right_base = gMotors.rightEncoderCount();
  gMotors.run(Nansho3DecelPower, Nansho3DecelPower);
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);

  int32_t cruise = Nansho3DecelTotalDeg - Nansho3DecelDeg;
  if (cruise < 0) {
    cruise = 0;
  }
  while (gMotors.traveledDegFrom(enc_left_base, enc_right_base) < cruise) {
    dly_tsk(StepMs * 1000U);
  }
  gMotors.decelToStop(Nansho3DecelMs, Nansho3DecelStepMs, /*coast_end=*/true);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  dly_tsk(StepMs * 1000U);
}

/* Nansho3 と同じ両輪ピボット。左右で許容・早め停止を切り替える */
void TestScene::turnNansho3SpinOnce(float target, bool left)
{
  const float tol   = left ? Nansho3SpinLeftToleranceDeg
                           : Nansho3SpinRightToleranceDeg;
  const float extra = left ? Nansho3SpinLeftEarlyExtraDeg
                           : Nansho3SpinRightEarlyExtraDeg;
  const int   prefer = left ? -1 : 1;
  gMotors.turnToHeading(target, tol, Nansho3SpinPower, Nansho3SpinPower,
                        Nansho3SpinMaxMs, Nansho3SpinSlowApproachDeg,
                        Nansho3SpinSlowPower,
                        Nansho3SpinEarlyStopDeg + extra, /*coast_end=*/false,
                        prefer);
  gMotors.stop();
}

/* Nansho3 と同じ両輪ピボット。90° ごとに停止し、旋回方向の補正を積む */
void TestScene::runNansho3SpinLoop()
{
  gGyro.reset();
  gGyro.setKalmanNoise(Nansho3SpinProcessNoise, Nansho3SpinMeasNoise);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

  static constexpr float kFacingDeg[4] = {0.0f, 90.0f, -180.0f, -90.0f};
  const bool left = (TEST_TURN_LEFT != 0);
  int   facing_i    = 0;
  float heading_add = 0.0f;
  int   turn_count  = 0;
  const float per_add =
      left ? Nansho3SpinPerLeftAddDeg : Nansho3SpinPerRightAddDeg;

  while (1) {
    dly_tsk(Nansho3SpinBeforeMs * 1000U);
    facing_i = left ? (facing_i + 3) % 4 : (facing_i + 1) % 4;
    const float target = Gyro::wrapDeg180(
        kFacingDeg[facing_i] + Nansho3SpinHeadingOffsetDeg + heading_add);
    turnNansho3SpinOnce(target, left);

    ++turn_count;
    if (turn_count >= Nansho3SpinTurnsPerAdd) {
      turn_count  = 0;
      heading_add = Gyro::wrapDeg180(heading_add + per_add);
    }

    if (Nansho3SpinAfterMs > 0U) {
      dly_tsk(Nansho3SpinAfterMs * 1000U);
    }
  }
}

/* Nansho3 と同じ両輪ピボットを1回だけ。TEST_TURN_LEFT で左右 */
void TestScene::runNansho3OnceTurn()
{
  gGyro.reset();
  gGyro.setKalmanNoise(Nansho3SpinProcessNoise, Nansho3SpinMeasNoise);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

  const bool  left    = (TEST_TURN_LEFT != 0);
  const float logical = left ? -90.0f : 90.0f;
  const float target  = Gyro::wrapDeg180(
      logical + Nansho3SpinHeadingOffsetDeg);

  dly_tsk(Nansho3SpinBeforeMs * 1000U);
  turnNansho3SpinOnce(target, left);
}

/* Nansho1 と同じピボット旋回（経過角で終了、ジャイロ reset はしない） */
void TestScene::turnToHeading(float target_heading, float tolerance_deg)
{
  turnToHeading(target_heading, tolerance_deg, SquareTurnPower, SquareTurnPower);
}

void TestScene::turnToHeading(float target_heading, float tolerance_deg,
                              int left_power, int right_power)
{
  float prev = gGyro.heading();
  const float delta = Gyro::wrapDeg180(target_heading - prev);
  if (fabsf(delta) <= tolerance_deg) {
    gMotors.stop();
    dly_tsk(StepMs * 1000U);
    return;
  }

  const float turn_dir = (delta > 0.0f) ? 1.0f : -1.0f;
  const float need     = fabsf(delta);
  float       traveled = 0.0f;

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < SquareTurnMaxMs) {
    const float h = gGyro.heading();
    traveled += turn_dir * Gyro::wrapDeg180(h - prev);
    prev = h;

    const float rem = need - traveled;
    if (rem <= tolerance_deg) {
      break;
    }

    int lp = left_power;
    int rp = right_power;
    if (rem <= SquareTurnSlowApproachDeg) {
      lp = SquareTurnSlowPower;
      rp = SquareTurnSlowPower;
    }

    if (turn_dir > 0.0f) {
      gMotors.run(lp, -rp);
    } else {
      gMotors.run(-lp, rp);
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

/* 直進 → 相対 +90° CW 旋回を繰り返して四角を描き続ける */
void TestScene::runSquareLoop()
{
  gGyro.reset();
  gMotors.setBalancePid(SquareBalanceKp, SquareBalanceKi);

  float target_heading = 0.0f;
  while (1) {
    runBalancedStraightFor(SquareStraightPower, SquareStraightPower,
                           target_heading, SquareBalanceKp, SquareBalanceKi,
                           SquareStraightMs);
    dly_tsk(StepMs * 1000U);

    /* 現在角 +90° を旋回目標にする（累積目標角との差が小さいと一瞬で終了するため） */
    const float turn_target =
        gGyro.heading() + SquareRelativeTurnDeg;
    turnToHeading(turn_target, SquareTurnToleranceDeg);
    target_heading += SquareRelativeTurnDeg;
    dly_tsk(StepMs * 1000U);
  }
}

/* 0°維持の前後往復（カルマン raw/kf をシリアル出力） */
void TestScene::runKalmanPingPongLoop()
{
  gGyro.reset();
  gGyro.setKalmanNoise(KalmanTestProcessNoise, KalmanTestMeasNoise);
  gMotors.setBalancePid(KalmanBalanceKp, KalmanBalanceKi);

  printf("Kalman test Q=%d R=%d\n", (int)KalmanTestProcessNoise,
         (int)KalmanTestMeasNoise);

  const float target_heading = 0.0f;
  while (1) {
    runBalancedStraightWithDecelAndLog(
        KalmanPingPongPower, KalmanPingPongPower, target_heading,
        KalmanBalanceKp, KalmanBalanceKi, KalmanForwardMs, KalmanDecelMs,
        "fwd", KalmanLogIntervalMs);
    dly_tsk(StepMs * 1000U);

    runBalancedStraightWithDecelAndLog(
        -KalmanPingPongPower, -KalmanPingPongPower, target_heading,
        KalmanBalanceKp, KalmanBalanceKi, KalmanBackMs, KalmanDecelMs, "back",
        KalmanLogIntervalMs);
    dly_tsk(StepMs * 1000U);
  }
}

void TestScene::run()
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

#if ETROBO_SCENE == 7
  runStraightOnly();
#elif ETROBO_SCENE == 8
  runSquareLoop();
#elif ETROBO_SCENE == 9
  runKalmanPingPongLoop();
#elif ETROBO_SCENE == 10
  runStraightOnlyTest();
#elif ETROBO_SCENE == 11
  runNansho3StraightTest();
#elif ETROBO_SCENE == 12
  runNansho3DecelStraightTest();
#elif ETROBO_SCENE == 14
  runNansho3SpinLoop();
#elif ETROBO_SCENE == 15
  runNansho3OnceTurn();
#else
  runLineTraceUntilSkyblueThenStraight();
#endif
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
