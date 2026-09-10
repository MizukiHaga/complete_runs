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
  gMotors.setTargetHeading(ColorForwardTargetHeadingDeg);
  gMotors.run(ColorForwardLeftPower, ColorForwardRightPower);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < ColorForwardMaxMs) {
    if (scanRouteBranchOnce()/*色読んで何色読んだか保存*/) {
      gMotors.stop();
      dly_tsk(StepMs * 1000U);
      gMotors.stop();
      beepOnRouteBranchDetected();/*色読む（空関数）*/
      runShortBackBeforeLowerArm();/*後退*/
      lowerArm();/*アーム下げる*/
      return;
    }

    gMotors.tickCorrection();/*修正*/
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
  /* 90°直進で黒停止済み → 旋回（R:左のみ+90° / L:右のみ-90°） */
  const float turn_target = gGyro.heading() + AfterBlackTurnOffsetDeg;
  turnToHeading(turn_target, AfterBlackLeftTurnToleranceDeg,
                AfterBlackLeftTurnLeftPower, AfterBlackLeftTurnRightPower,
                AfterBlackTurnPreferSign);
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
  // 青検知判定まで一定時間トレース
  gGyro.reset();
  runAfterBlueByRouteBranch();
}

void Nansho2Scene::runLineTraceUntilHeadingThenStraight()
{
  runAfterColorLineTraceUntilHeading();
  // AfterColorTraceStopHeadingDeg までライントレース、到達後停止
  dly_tsk(StepMs * 1000U);

  runStraightAfterTurn(AfterColorStraightHeadingDeg,
                       AfterColorStraightLeftPower,
                       AfterColorStraightRightPower,
                       AfterColorStraightDeg);
                      //  AfterColorStraightHeadingDeg を維持して直進
  dly_tsk(StepMs * 1000U);

  turnToHeading(AfterPivotTurnDeg, AfterPivotTurnToleranceDeg,
                AfterPivotTurnLeftPower, AfterPivotTurnRightPower);
                // 90度へ旋回
  dly_tsk(StepMs * 1000U);

  driveStraightUntilBlackLine(AfterStraightHeadingDeg);
  // 90度直進: クルーズ→37まで減速→黒検知まで低速（時間停止なし）
  runAfterBlackLeftTurnAndBranch();
  // 直進黒停止後 → 旋回（R:左のみ+90 / L:右のみ-90）→ 低速LT → SKYBLUE
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
  const int slow_power = AfterStraightSlowPower;
  const uint32_t decel_ms  = AfterStraightToBlackDecelMs;

  const int32_t enc_left_base  = gMotors.leftEncoderCount();
  const int32_t enc_right_base = gMotors.rightEncoderCount();

  gMotors.run(cruise_l, cruise_r);

  /* クルーズはエンコーダ距離で終える。そのあと低速まで減速し、
   * 黒検知まで低速を維持する（時間では止めない）。 */
  bool     decel_started = false;
  uint32_t decel_ms_done = 0U;
  for (uint32_t elapsed_ms = 0U; elapsed_ms < AfterStraightBlackMaxMs;
       elapsed_ms += StepMs) {
    int left  = cruise_l;
    int right = cruise_r;

    if (!decel_started &&
        gMotors.traveledDegFrom(enc_left_base, enc_right_base) >=
            AfterStraightToBlackCruiseDeg) {
      decel_started = true;
    }

    if (decel_started) {
      if (decel_ms_done < decel_ms) {
        const int delta_l = cruise_l - slow_power;
        const int delta_r = cruise_r - slow_power;
        left = cruise_l -
               (int)((int64_t)delta_l * (int64_t)decel_ms_done /
                     (int64_t)decel_ms);
        right = cruise_r -
                (int)((int64_t)delta_r * (int64_t)decel_ms_done /
                      (int64_t)decel_ms);
      } else {
        left  = slow_power;
        right = slow_power;
      }
      decel_ms_done += StepMs;

      if (gColorLAB.reflection() <= AfterStraightBlackReflectionMax ||
          gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
        beepOnBlackDetected();
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
  // pid,scale,speed,edgeのセッター

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
                                 int left_power, int right_power,
                                 int prefer_turn_sign)
{
  gLineTracer.setEnabled(false);

  /* prefer_turn_sign: 0=最短, +1=正方向優先, -1=負方向優先（開始時に方向固定） */
  float turn_dir = 0.0f;
  {
    float d0 = gGyro.diffTo(target_heading);
    if (prefer_turn_sign > 0 && d0 < 0.0f) {
      d0 += 360.0f;
    } else if (prefer_turn_sign < 0 && d0 > 0.0f) {
      d0 -= 360.0f;
    }
    if (d0 > 0.0f) {
      turn_dir = 1.0f;
    } else if (d0 < 0.0f) {
      turn_dir = -1.0f;
    }
  }

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    const float diff = gGyro.diffTo(target_heading);
    if (gGyro.reached(target_heading, tolerance_deg)) {
      break;
    }

    const float abs_diff = fabsf(diff);
    int lp = left_power;
    int rp = right_power;
    if (abs_diff <= TurnSlowApproachDeg) {
      lp = TurnSlowLeftPower;
      rp = TurnSlowRightPower;
    }

    float cmd_dir = turn_dir;
    if (prefer_turn_sign == 0) {
      if (diff > 0.0f) {
        cmd_dir = 1.0f;
      } else if (diff < 0.0f) {
        cmd_dir = -1.0f;
      } else {
        cmd_dir = 0.0f;
      }
    }

    if (cmd_dir > 0.0f) {
      gMotors.run(lp, -rp);
    } else if (cmd_dir < 0.0f) {
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
                                        int right_power, int32_t target_deg)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
  gMotors.setHeadingCorrection(true);
  gMotors.setTargetHeading(heading);

  const int32_t enc_left_base  = gMotors.leftEncoderCount();
  const int32_t enc_right_base = gMotors.rightEncoderCount();
  gMotors.run(left_power, right_power);

  while (gMotors.traveledDegFrom(enc_left_base, enc_right_base) < target_deg) {
    gMotors.tickCorrection();
    dly_tsk(StepMs * 1000U);
  }

  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::runAfterFinalStopTurnRightAndSlowTraceToWhite()
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  /* パワーは定数の符号のまま出す。turnToHeading のように向きから
   * 符号を付け直すことはしない。 */
  uint32_t turn_ms = 0U;
  while (turn_ms < LtMaxMs) {
    if (gGyro.reached(AfterFinalTurnTargetDeg, AfterFinalTurnToleranceDeg)) {
      break;
    }
    gMotors.run(AfterFinalTurnLeftPower, AfterFinalTurnRightPower);
    dly_tsk(StepMs * 1000U);
    turn_ms += StepMs;
  }
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  gLineTracer.setEnabled(true);
  if (route_branch_ == RouteBranch::Red) {
    /* 赤: 従来の低速 LT */
    gLineTracer.setBaseSpeed(FinalRedSlowTracePower, FinalRedSlowTracePower);
    gLineTracer.setPid(FinalRedSlowTraceKp, FinalRedSlowTraceKi,
                       FinalRedSlowTraceKd, FinalRedSlowTraceOutMax);
    gLineTracer.setScale(FinalRedSlowTraceScale);
  } else {
    gLineTracer.setBaseSpeed(FinalSlowTracePower, FinalSlowTracePower);
    gLineTracer.setPid(FinalSlowTraceKp, FinalSlowTraceKi, FinalSlowTraceKd,
                       FinalSlowTraceOutMax);
    gLineTracer.setScale(FinalSlowTraceScale);
  }
  gLineTracer.setEdgeFollow(FinalSlowTraceEdgeFollow);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    gLineTracer.step();

    if (elapsed_ms >= FinalSlowTraceWhiteIgnoreMs &&
        gColorLAB.reflection() >= FinalSlowTraceWhiteReflectionMin) {
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
  // 200ms補正前進からの90or-90への旋回
  const float turn_target = runShortForwardTurnRight();
  // 補正しながらの前進
  runStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                       RouteFinalApproachRightPower, RouteFinalApproachDeg);
  /* 検知後旋回→直進の完了向きを 0° にし、後退後旋回を絶対角で扱う */
  gGyro.reset();

  // 800msかけての後退
  runRouteFinalBack(0.0f, YellowFinalBackPower, YellowFinalBackIgnoreMs);
}

float Nansho2Scene::runShortForwardTurnRight()
{
  // ライントレースオフ
  gLineTracer.setEnabled(false);
  /* 直前の直進 (driveStraightUntilBlackLine) が残した目標角のまま
   * ヨー補正つきで走ると、青検知後の reset で現在角が 0 になっている分だけ
   * 最大 trim の弧を描いて線を外す。いまの向きを保つ目標に置き換える。 */
  gMotors.setTargetHeading(gGyro.heading());
  // 一定距離だけ前進
  gMotors.runBalancedEncoderFor(BlueAfterSecondBlueForwardPower,
                                BlueAfterSecondBlueForwardPower,
                                BlueAfterSecondBlueForwardDeg, StepMs);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  // 現在角度+旋回角度＝目標角
  const float turn_target =
      Gyro::wrapDeg180(gGyro.heading() + BlueTurnRightOffsetDeg);
  //目標角への旋回 
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

/* ボトル設置後: 旋回も前進も挟まず、後退したままラインを検知するまで下がる。
 * ignore_ms の間は判定しない（設置位置のラインを拾い直さないため）。 */
void Nansho2Scene::runRouteFinalBack(float heading, int power,
                                     uint32_t ignore_ms)
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  gMotors.setHeadingCorrection(true);
  gMotors.setTargetHeading(heading);
  gMotors.run(-power, -power);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    gMotors.tickCorrection();

    if (elapsed_ms >= ignore_ms) {
      if (gColorLAB.reflection() <= AfterFinalBackReflectionLineMax) {
        beepOnBlackDetected();
        break;
      }
      if (gColorLAB.is(COLOR_DETECT_BLUE, true)) {
        beepOnBlueDetected();
        break;
      }
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  dly_tsk(StepMs * 1000U);
  runAfterFinalStopTurnRightAndSlowTraceToWhite();
}

void Nansho2Scene::runBlueRouteAfterBlue()
{
  const float turn_target = runBlackBlueForwardTurnRight();
  runStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                       RouteFinalApproachRightPower, RouteFinalApproachDeg);
  gGyro.reset();
  runRouteFinalBack(0.0f, BlueFinalBackPower, BlueFinalBackIgnoreMs);
}

void Nansho2Scene::runRedRouteAfterBlue()
{
  const float turn_target = runBlackBlueForwardTurnRight(2);
  runStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                       RouteFinalApproachRightPower, RouteFinalApproachDeg);
  gGyro.reset();
  runRouteFinalBack(0.0f, RedFinalBackPower, RedFinalBackIgnoreMs);
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
  gGyro.reset();

  // runInitialLineTrace();/*青からの黒検知までライトレ*/
  // runShortBackAfterGray();
  raiseArm();/*腕上げ*/
  driveForwardAndDetectRouteBranch();
// ↑直進、腕上げ、色読み、後退、腕下げ、
  if (route_branch_ != RouteBranch::None) {/*何かしら色読んだら*/
    runLineTraceUntilHeadingThenStraight();
  }
}

extern "C" void Nansho2Task(intptr_t exinf)
{
  (void)exinf;
  gNansho2.run();

#if ETROBO_SCENE == 0 || ETROBO_SCENE == 5 || ETROBO_SCENE == 6
  gMotors.stop();
  gLineTracer.setEnabled(false);
  dly_tsk(50 * 1000U);
  act_tsk(NANSHO3_TASK);
#endif

  ext_tsk();
}
