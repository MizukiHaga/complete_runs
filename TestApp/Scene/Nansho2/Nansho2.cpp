#include "Nansho2.h"

#include <stdio.h>

#include <spike/hub/speaker.h>

#include "Arm.h"
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
  gMotors.runFor(-BackAfterGrayPower, -BackAfterGrayPower, BackAfterGrayMs);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::runShortBackBeforeLowerArm()
{
  gLineTracer.setEnabled(false);
  gMotors.setTargetHeading(0.0f);
  gMotors.runFor(-BackBeforeLowerPower, -BackBeforeLowerPower,
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
  gArm.lower(ArmLowerPower, ArmLowerMaxMs);
}

void Nansho2Scene::beepOnRouteBranchDetected()
{
  hub_speaker_set_volume(ColorDetectBeepVolume);
  hub_speaker_play_tone(ColorDetectBeepHz,
                        static_cast<int32_t>(ColorDetectBeepMs));
}


bool Nansho2Scene::scanRouteBranchOnce()
{
  if (route_branch_ != RouteBranch::None) {
    return true;
  }

  if (gColorLAB.is(COLOR_DETECT_RED, true)) {
    route_branch_ = RouteBranch::Red;
    return true;
  }
  if (gColorLAB.is(COLOR_DETECT_YELLOW, true)) {
    route_branch_ = RouteBranch::Yellow;
    return true;
  }
  if (gColorLAB.is(COLOR_DETECT_BLUE, true)) {
    route_branch_ = RouteBranch::Blue;
    return true;
  }
  return false;
}

void Nansho2Scene::driveForwardAndDetectRouteBranch()
{
  route_branch_ = RouteBranch::None;
  gLineTracer.setEnabled(false);
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

    /* 周期補正が止まらないよう、直進指令を維持 */
    gMotors.run(ColorForwardLeftPower, ColorForwardRightPower);
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

  runLineTraceUntilColor(COLOR_DETECT_BLUE);
  gGyro.reset();
  runAfterBlueByRouteBranch();
}

void Nansho2Scene::runLineTraceUntilHeadingThenStraight()
{
  runAfterColorLineTraceUntilHeading();
  dly_tsk(StepMs * 1000U);

  runTimedStraightAfterTurn(AfterColorTraceStopHeadingDeg,
                            AfterColorStraightLeftPower,
                            AfterColorStraightRightPower,
                            AfterColorStraightMs);
  dly_tsk(StepMs * 1000U);

  turnToHeading(AfterPivotTurnDeg, AfterPivotTurnToleranceDeg,
                AfterPivotTurnLeftPower, AfterPivotTurnRightPower);
  dly_tsk(StepMs * 1000U);

  driveStraightUntilBlackLine(AfterStraightHeadingDeg, AfterStraightToBlackIgnoreMs);
  runAfterBlackLeftTurnAndBranch();
}

void Nansho2Scene::driveStraightUntilBlackLine(float target_heading,
                                               uint32_t black_ignore_ms)
{
  gLineTracer.setEnabled(false);
  gMotors.setBalancePid(AfterStraightBalanceKp, AfterStraightBalanceKi,
                        AfterStraightBalanceIntegralMax);
  gMotors.setTargetHeading(target_heading);
  gMotors.run(AfterStraightToBlackLeftPower, AfterStraightToBlackRightPower);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    gMotors.tickCorrection();

    if (elapsed_ms >= black_ignore_ms &&
        gColorLAB.reflection() <= AfterStraightToBlackReflectionLineMax) {
      gMotors.stop();
      break;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
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

    if (gColorLAB.is(color, true)) {
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

void Nansho2Scene::runTimedStraightAfterTurn(float heading, int left_power,
                                             int right_power, uint32_t ms)
{
  gLineTracer.setEnabled(false);
  gMotors.setTargetHeading(heading);
  gMotors.runFor(left_power, right_power, ms);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::driveStraightUntilReflectionBlackLine(float target_heading,
                                                         uint32_t ignore_ms)
{
  gLineTracer.setEnabled(false);
  gMotors.setTargetHeading(target_heading);
  gMotors.run(AfterFinalBackBlueApproachLeftPower,
              AfterFinalBackBlueApproachRightPower);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    if (elapsed_ms >= ignore_ms &&
        (gColorLAB.reflection() <= AfterFinalBackReflectionLineMax ||
         gColorLAB.is(COLOR_DETECT_BLUE, true))) {
      gMotors.stop();
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

  const float turn_target =
      Gyro::wrapDeg180(gGyro.heading() + AfterFinalBackTurnOffsetDeg);
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
  runTimedStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                            RouteFinalApproachRightPower, YellowFinalApproachMs);
  runRouteFinalBack(turn_target, YellowFinalBackPower, YellowFinalBackMs);
}

float Nansho2Scene::runShortForwardTurnRight()
{
  gLineTracer.setEnabled(false);
  gMotors.runFor(BlueAfterSecondBlueForwardPower, BlueAfterSecondBlueForwardPower,
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
  gMotors.runFor(-power, -power, ms);
  gMotors.stop();
  runAfterFinalBackTurnToBlue();
}

void Nansho2Scene::runBlueRouteAfterBlue()
{
  const float turn_target = runBlackBlueForwardTurnRight();
  runTimedStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                            RouteFinalApproachRightPower, BlueFinalApproachMs);
  runRouteFinalBack(turn_target, BlueFinalBackPower, BlueFinalBackMs);
}

void Nansho2Scene::runRedRouteAfterBlue()
{
  const float turn_target = runBlackBlueForwardTurnRight(2);
  runTimedStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                            RouteFinalApproachRightPower, RedFinalApproachMs);
  runRouteFinalBack(turn_target, RedFinalBackPower, RedFinalBackMs);
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
  ext_tsk();
}
