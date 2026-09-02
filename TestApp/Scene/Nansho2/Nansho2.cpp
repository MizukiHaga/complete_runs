#include "Nansho2.h"

#include "app.h"
#include "kernel_cfg.h"

#include <stdio.h>

#include "Arm.h"
#include "ColorLAB.h"
#include "Common/LineTracer/LineTracer.h"
#include "Common/Straight/Straight.h"
#include "Gyro.h"

extern Arm        gArm;
extern ColorLAB   gColorLAB;
extern Gyro       gGyro;
extern Straight   gMotors;
extern LineTracer gLineTracer;
extern Nansho2Scene gNansho2;

void Nansho2Scene::runInitialLineTrace()
{
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(TracePower, TracePower);
  gLineTracer.setPid(InitialTraceKp, InitialTraceKi,
                     InitialTraceKd, InitialTraceOutMax);
  gLineTracer.setScale(InitialTraceScale);
  gLineTracer.setEdgeFollow(InitialTraceEdgeFollow);

  bool skyblue_seen = false;
  while (1) {
    gLineTracer.step();

    if (!skyblue_seen) {
      if (gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
        skyblue_seen = true;
        gGyro.reset();
      }
    } else if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
      beepOnBlackDetected();
      break;
    }
  }
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::runShortBackAfterGray()
{
  gLineTracer.setEnabled(false);
  gMotors.setTargetHeading(0.0f);
  gMotors.runForMs(-BackAfterGrayPower, -BackAfterGrayPower, BackAfterGrayMs);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::runShortBackBeforeLowerArm()
{
  gLineTracer.setEnabled(false);
  gMotors.setTargetHeading(0.0f);
  gMotors.runForMs(-BackBeforeLowerPower, -BackBeforeLowerPower,
                   BackBeforeLowerMs);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::raiseArm()
{
  gArm.raise(ArmRaisePower, ArmRaiseDeg, ArmRaiseMaxMs);
}

void Nansho2Scene::lowerArm()
{
  gArm.lower(ArmLowerPower, ArmLowerMaxMs, ArmLowerOvershootDeg);
}

void Nansho2Scene::beepOnRouteBranchDetected()
{
}

void Nansho2Scene::beepOnBlackDetected()
{
}

void Nansho2Scene::beepOnBlueDetected()
{
}


bool Nansho2Scene::scanRouteBranchOnce()
{
  if (route_branch_ != RouteBranch::None) {
    return true;
  }

  if (gColorLAB.is(COLOR_DETECT_PETRED, true)) {
    route_branch_ = RouteBranch::Red;
    return true;
  }
  if (gColorLAB.is(COLOR_DETECT_PETYELLOW, true)) {
    route_branch_ = RouteBranch::Yellow;
    return true;
  }
  if (gColorLAB.is(COLOR_DETECT_PETBLUE, true)) {
    route_branch_ = RouteBranch::Blue;
    return true;
  }
  return false;
}

void Nansho2Scene::driveForwardAndDetectRouteBranch()
{
  route_branch_ = RouteBranch::None;
  gLineTracer.setEnabled(false);
  gMotors.setBalancePid(ColorForwardBalanceKp, ColorForwardBalanceKi,
                        ColorForwardBalanceIntegralMax);
  gMotors.setTargetHeading(0.0f);
  gMotors.run(ColorForwardLeftPower, ColorForwardRightPower);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < ColorForwardMaxMs) {
    if (scanRouteBranchOnce()) {
      gMotors.stop();
      dly_tsk(StepMs * 1000U);
      gMotors.stop();
      beepOnRouteBranchDetected();
      runShortBackBeforeLowerArm();
      lowerArm();
      return;
    }

    gMotors.tickCorrection();
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
}

void Nansho2Scene::runAfterColorLineTraceUntilHeading()
{
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(AfterColorTraceLeftPower, AfterColorTraceRightPower);
  gLineTracer.setPid(AfterColorTraceKp, AfterColorTraceKi,
                     AfterColorTraceKd, AfterColorTraceOutMax);
  gLineTracer.setScale(AfterColorTraceScale);
  gLineTracer.setEdgeFollow(AfterColorEdgeFollow);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    gLineTracer.step();
    if (gGyro.reached(AfterColorTraceStopHeadingDeg,
                      AfterColorTraceStopToleranceDeg)) {
      break;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gLineTracer.setEnabled(false);
  gMotors.stop();
}

void Nansho2Scene::runAfterBlackLeftTurnAndBranch()
{
  turnToHeading(AfterBlackLeftTurnDeg, AfterBlackLeftTurnToleranceDeg,
                AfterBlackLeftTurnLeftPower, AfterBlackLeftTurnRightPower);
  dly_tsk(StepMs * 1000U);

  /* 旋回直後は低速で線に乗せる */
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(PostBlackSlowTracePower, PostBlackSlowTracePower);
  gLineTracer.setPid(PostBlackSlowTraceKp, PostBlackSlowTraceKi,
                     PostBlackSlowTraceKd, PostBlackSlowTraceOutMax);
  gLineTracer.setScale(PostBlackSlowTraceScale);
  gLineTracer.setEdgeFollow(PostBlackTraceEdgeFollow);
  for (uint32_t elapsed_ms = 0U; elapsed_ms < PostBlackSlowTraceMs;
       elapsed_ms += StepMs) {
    gLineTracer.step();
    dly_tsk(StepMs * 1000U);
  }

  runLineTraceUntilColor(COLOR_DETECT_SKYBLUE);
  gGyro.reset();
  runAfterBlueByRouteBranch();
}

void Nansho2Scene::runLineTraceUntilHeadingThenStraight()
{
  runAfterColorLineTraceUntilHeading();
  dly_tsk(StepMs * 1000U);

  runStraightAfterTurn(AfterColorTraceStopHeadingDeg,
                       AfterColorStraightLeftPower,
                       AfterColorStraightRightPower,
                       AfterColorStraightMs);
  dly_tsk(StepMs * 1000U);

  turnToHeading(AfterPivotTurnDeg, AfterPivotTurnToleranceDeg,
                AfterPivotTurnLeftPower, AfterPivotTurnRightPower);
  dly_tsk(StepMs * 1000U);

  driveStraightUntilBlackLine(AfterStraightHeadingDeg);
  runAfterBlackLeftTurnAndBranch();
}

void Nansho2Scene::driveStraightUntilBlackLine(float target_heading)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(AfterStraightBalanceKp, AfterStraightBalanceKi,
                        AfterStraightBalanceIntegralMax);
  gMotors.setTargetHeading(target_heading);

  const int cruise_l = AfterStraightToBlackLeftPower;
  const int cruise_r = AfterStraightToBlackRightPower;
  const uint32_t cruise_ms = AfterStraightToBlackCruiseMs;
  const uint32_t decel_ms  = AfterStraightToBlackDecelMs;
  const uint32_t total_ms  = cruise_ms + decel_ms;

  gMotors.run(cruise_l, cruise_r);

  /* 指定時間クルーズ → 台形減速（ヨー補正は維持） */
  for (uint32_t elapsed_ms = 0U; elapsed_ms < total_ms; elapsed_ms += StepMs) {
    int left  = cruise_l;
    int right = cruise_r;

    if (elapsed_ms >= cruise_ms) {
      const uint32_t traveled_decel = elapsed_ms - cruise_ms;
      if (traveled_decel >= decel_ms) {
        break;
      }

      const uint32_t numer = decel_ms - traveled_decel;
      left = (int)((int64_t)cruise_l * (int64_t)numer / (int64_t)decel_ms);
      right =
          (int)((int64_t)cruise_r * (int64_t)numer / (int64_t)decel_ms);
      if (left == 0 && right == 0) {
        break;
      }
    }

    gMotors.run(left, right);
    gMotors.tickCorrection();
    dly_tsk(StepMs * 1000U);
  }

  gMotors.stop();
}

void Nansho2Scene::setupPostBlackLineTrace()
{
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(PostBlackTraceLeftPower, PostBlackTraceRightPower);
  gLineTracer.setPid(PostBlackTraceKp, PostBlackTraceKi,
                     PostBlackTraceKd, PostBlackTraceOutMax);
  gLineTracer.setScale(PostBlackTraceScale);
  gLineTracer.setEdgeFollow(PostBlackTraceEdgeFollow);
}

void Nansho2Scene::runLineTraceUntilColor(color_detect_t color)
{
  setupPostBlackLineTrace();

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    gLineTracer.step();

    const bool hit = gColorLAB.is(color, true);
    if (hit) {
      if (color == COLOR_DETECT_BLACKLINE) {
        beepOnBlackDetected();
      } else if (color == COLOR_DETECT_PETBLUE ||
                 color == COLOR_DETECT_SKYBLUE ||
                 color == COLOR_DETECT_BLUE) {
        beepOnBlueDetected();
      }
      break;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
  gLineTracer.setEnabled(false);
  gMotors.stop();
}

void Nansho2Scene::turnToHeading(float target_heading, float tolerance_deg,
                                 int left_power, int right_power)
{
  gLineTracer.setEnabled(false);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    const float diff = gGyro.diffTo(target_heading);
    if (gGyro.reached(target_heading, tolerance_deg)) {
      break;
    }

    const float abs_diff = fabsf(diff);
    int         lp       = left_power;
    int         rp       = right_power;
    if (abs_diff <= TurnSlowApproachDeg) {
      lp = TurnSlowLeftPower;
      rp = TurnSlowRightPower;
    }

    if (diff > 0.0f) {
      gMotors.run(lp, -rp);
    } else if (diff < 0.0f) {
      gMotors.run(-lp, rp);
    } else {
      gMotors.stop();
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::runStraightAfterTurn(float heading, int left_power,
                                        int right_power, uint32_t ms)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
  gMotors.setHeadingCorrection(true);
  gMotors.setTargetHeading(heading);
  gMotors.run(left_power, right_power);

  for (uint32_t elapsed_ms = 0U; elapsed_ms < ms; elapsed_ms += StepMs) {
    gMotors.tickCorrection();
    dly_tsk(StepMs * 1000U);
  }

  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::driveStraightUntilReflectionBlackLine(float target_heading,
                                                         uint32_t ignore_ms)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.setTargetHeading(target_heading);
  gMotors.run(AfterFinalBackBlueApproachLeftPower,
              AfterFinalBackBlueApproachRightPower);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    gMotors.tickCorrection();

    if (elapsed_ms >= ignore_ms &&
        (gColorLAB.reflection() <= AfterFinalBackReflectionLineMax ||
         gColorLAB.is(COLOR_DETECT_BLUE, true))) {
      if (gColorLAB.reflection() <= AfterFinalBackReflectionLineMax) {
        beepOnBlackDetected();
      } else {
        beepOnBlueDetected();
      }
      break;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
}

void Nansho2Scene::runAfterFinalBackTurnToBlue()
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  const float turn_target = AfterFinalBackTurnDeg;
  turnToHeading(turn_target, AfterFinalBackTurnToleranceDeg,
                AfterFinalBackTurnLeftPower, AfterFinalBackTurnRightPower);
  dly_tsk(StepMs * 1000U);

  driveStraightUntilReflectionBlackLine(turn_target, AfterFinalBackBlueIgnoreMs);
  runAfterFinalStopTurnRightAndSlowTraceToWhite();
}

void Nansho2Scene::runAfterFinalStopTurnRightAndSlowTraceToWhite()
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  turnToHeading(AfterFinalTurnToZeroDeg, AfterFinalRightTurnToleranceDeg,
                AfterFinalRightTurnLeftPower, AfterFinalRightTurnRightPower);
  dly_tsk(StepMs * 1000U);

  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(FinalSlowTracePower, FinalSlowTracePower);
  gLineTracer.setPid(FinalSlowTraceKp, FinalSlowTraceKi, FinalSlowTraceKd,
                     FinalSlowTraceOutMax);
  gLineTracer.setScale(FinalSlowTraceScale);
  gLineTracer.setEdgeFollow(FinalSlowTraceEdgeFollow);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    gLineTracer.step();

    if (elapsed_ms >= FinalSlowTraceWhiteIgnoreMs &&
        gColorLAB.is(COLOR_DETECT_WHITE, true)) {
      break;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  gLineTracer.setEnabled(false);
}

void Nansho2Scene::runYellowRouteAfterBlue()
{
  const float turn_target = runShortForwardTurnRight();
  runStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                       RouteFinalApproachRightPower, RouteFinalApproachMs);
  /* 検知後旋回→直進の完了向きを 0° にし、後退後旋回を絶対角で扱う */
  gGyro.reset();
  runRouteFinalBack(0.0f, YellowFinalBackPower, YellowFinalBackMs);
}

float Nansho2Scene::runShortForwardTurnRight()
{
  gLineTracer.setEnabled(false);
  gMotors.runForMs(BlueAfterSecondBlueForwardPower, BlueAfterSecondBlueForwardPower,
                   BlueAfterSecondBlueForwardMs);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  const float turn_target =
      Gyro::wrapDeg180(gGyro.heading() + BlueTurnRightOffsetDeg);
  turnToHeading(turn_target, BlueTurnToleranceDeg,
                BlueTurnLeftPower, BlueTurnRightPower);
  dly_tsk(StepMs * 1000U);

  gMotors.setTargetHeading(turn_target);
  return turn_target;
}

float Nansho2Scene::runBlackBlueForwardTurnRight(int black_blue_pairs)
{
  for (int i = 0; i < black_blue_pairs; ++i) {
    runLineTraceUntilColor(COLOR_DETECT_BLACKLINE);
    runLineTraceUntilColor(COLOR_DETECT_SKYBLUE);
  }

  return runShortForwardTurnRight();
}

void Nansho2Scene::runRouteFinalBack(float heading, int power, uint32_t ms)
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
  gMotors.setTargetHeading(heading);
  gMotors.runForMs(-power, -power, ms);
  gMotors.stop();
  runAfterFinalBackTurnToBlue();
}

void Nansho2Scene::runBlueRouteAfterBlue()
{
  const float turn_target = runBlackBlueForwardTurnRight();
  runStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                       RouteFinalApproachRightPower, RouteFinalApproachMs);
  gGyro.reset();
  runRouteFinalBack(0.0f, BlueFinalBackPower, BlueFinalBackMs);
}

void Nansho2Scene::runRedRouteAfterBlue()
{
  const float turn_target = runBlackBlueForwardTurnRight(2);
  runStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                       RouteFinalApproachRightPower, RouteFinalApproachMs);
  gGyro.reset();
  runRouteFinalBack(0.0f, RedFinalBackPower, RedFinalBackMs);
}

void Nansho2Scene::runAfterBlueByRouteBranch()
{
  switch (route_branch_) {
  case RouteBranch::Yellow:
    runYellowRouteAfterBlue();
    break;
  case RouteBranch::Blue:
    runBlueRouteAfterBlue();
    break;
  case RouteBranch::Red:
    runRedRouteAfterBlue();
    break;
  default:
    break;
  }
}

void Nansho2Scene::run()
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

  runInitialLineTrace();
  //runShortBackAfterGray();
  raiseArm();
  driveForwardAndDetectRouteBranch();

  if (route_branch_ != RouteBranch::None) {
    runLineTraceUntilHeadingThenStraight();
  }
}

extern "C" void Nansho2Task(intptr_t exinf)
{
  (void)exinf;
  gNansho2.run();

#if ETROBO_SCENE == 0 || ETROBO_SCENE == 6
  gMotors.stop();
  gLineTracer.setEnabled(false);
  dly_tsk(50 * 1000U);
  act_tsk(NANSHO3_TASK);
#endif

  ext_tsk();
}
