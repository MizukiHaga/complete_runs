#include "Nansho2.h"

#include "app.h"
#include "kernel_cfg.h"

#include <stdio.h>

#include "Arm.h"
#include "ColorLAB.h"
#include "Common/LineTracer/LineTracer.h"
#include "Common/Straight/Straight.h"
#include "Gyro.h"
#include "HCR.h"

extern "C" {
#include <spike/pup/motor.h>
#include <spike/hub/imu.h>
#include <spike/hub/display.h>
#include <spike/hub/speaker.h>
}

extern Arm        gArm;
extern ColorLAB   gColorLAB;
extern Gyro       gGyro;
extern Straight   gMotors;
extern LineTracer gLineTracer;
extern HCR        gHCR;
extern Nansho2Scene gNansho2;
extern FILE       *gBluetoothOut;

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
    gMotors.debugHeading();

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
  gMotors.runForRotationsSameTime(-motor_speed_slow, -motor_speed_slow,
                                  BackBeforeLowerArmRotations,
                                  BackBeforeLowerArmRotations);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::raiseArm()
{
  dly_tsk(ArmRaiseBeforeMs * 1000U);
  if (!gArm.isConfigured()) {
    printf("Nansho2: arm is not configured\n");
    hub_speaker_play_tone(400, 500);
    return;
  }
  printf("Nansho2: raising arm\n");
  gArm.raise(ArmRaisePower, ArmRaiseDeg, ArmRaiseMaxMs);
}

void Nansho2Scene::lowerArm()
{
  if (!gArm.isConfigured()) {
    printf("Nansho2: arm is not configured; lowering skipped\n");
    return;
  }
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

void Nansho2Scene::logBlueLineDetected(const char *source)
{
  ++blue_line_count_;
  fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
          "Nansho2: blue-line-detected,count=%u,source=%s\n",
          (unsigned)blue_line_count_,
          source != nullptr ? source : "unknown");
}


bool Nansho2Scene::scanRouteBranchOnce()
{
  if (route_branch_ != RouteBranch::None) {
    return true;
  }

  if (gColorLAB.is(COLOR_DETECT_PETRED, true)) {
    route_branch_ = RouteBranch::Red;
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "Nansho2: color=PETRED,blue_line_count=%u\n",
            (unsigned)blue_line_count_);
    return true;
  }
  if (gColorLAB.is(COLOR_DETECT_PETYELLOW, true)) {
    route_branch_ = RouteBranch::Yellow;
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "Nansho2: color=PETYELLOW,blue_line_count=%u\n",
            (unsigned)blue_line_count_);
    return true;
  }
  if (gColorLAB.is(COLOR_DETECT_PETBLUE, true)) {
    route_branch_ = RouteBranch::Blue;
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "Nansho2: color=PETBLUE,blue_line_count=%u\n",
            (unsigned)blue_line_count_);
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
      runShortBackBeforeLowerArm();
      lowerArm();/*アーム下げる*/
      return;
    }

    if ((elapsed_ms % 100U) == 0U) {
      float lab_l;
      float lab_a;
      float lab_b;
      gColorLAB.readLab(true, &lab_l, &lab_a, &lab_b);
            fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
              "Nansho2: color=UNKNOWN reflection=%ld L=%.1f a=%.1f b=%.1f\n",
              (long)gColorLAB.reflection(), lab_l, lab_a, lab_b);
    }

    gMotors.tickCorrection();/*修正*/
    gMotors.debugHeading();
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
    gMotors.debugHeading();
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

  runLowSpeedTraceAndBranchToNansho3();
}

void Nansho2Scene::runLowSpeedTraceAndBranchToNansho3()
{
  const uint32_t blue_line_count_before = blue_line_count_;
  fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
          "Nansho2: low-speed-trace-start,blue_line_count=%u\n",
          (unsigned)blue_line_count_before);

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
    gMotors.debugHeading();
    dly_tsk(StepMs * 1000U);
  }

  runLineTraceUntilColor(COLOR_DETECT_SKYBLUE, "low-speed-trace", true);
  const uint32_t blue_line_count_in_trace =
      blue_line_count_ - blue_line_count_before;
  fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
          "Nansho2: low-speed-trace-blue-lines,count=%u,total=%u\n",
          (unsigned)blue_line_count_in_trace,
          (unsigned)blue_line_count_);
  // 青検知判定まで一定時間トレース
  gGyro.reset();
  runAfterBlueByRouteBranch();

#if ETROBO_SCENE == 0 || ETROBO_SCENE == 5 || ETROBO_SCENE == 6
  handed_off_to_nansho3_ = true;
  act_tsk(NANSHO3_TASK);
#endif
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
  runLowSpeedTraceAndBranchToNansho3();
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
      if (decel_ms > 0U && decel_ms_done < decel_ms) {
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
    gMotors.debugHeading();
    dly_tsk(StepMs * 1000U);
  }

  gMotors.stop();
}

bool Nansho2Scene::runStraightAtHeadingUntilBlackLine(
    float target_heading, int left_power, int right_power)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(HeadingRunKp, HeadingRunKi,
                        HeadingRunIntegralMax, 0.0f, HeadingRunKd);
  gMotors.setHeadingTrimLimit(HeadingRunTrimLimit);
  gMotors.setHeadingRunPowerScale(HeadingRunPowerScale);
  gMotors.setTargetHeading(target_heading);
  gMotors.run(left_power, right_power);

  bool line_detected = false;
  for (uint32_t elapsed_ms = 0U; elapsed_ms < HeadingBlackLineMaxMs;
       elapsed_ms += StepMs) {
    const bool black_detected =
        gColorLAB.is(COLOR_DETECT_BLACKLINE, true);
    const bool blue_detected =
        gColorLAB.is(COLOR_DETECT_BLUE, true) ||
        gColorLAB.is(COLOR_DETECT_SKYBLUE, true);
    if (black_detected || blue_detected) {
      line_detected = true;
      if (black_detected) {
        beepOnBlackDetected();
      } else {
        beepOnBlueDetected();
      }
      break;
    }

    gMotors.tickCorrection();
    gMotors.debugHeading();
    dly_tsk(StepMs * 1000U);
  }

  gMotors.stop();
  if (!line_detected) {
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "Nansho2: stop reason=heading-line-timeout,"
            "max_ms=%u,H=%.2f,T=%.2f,E=%.2f\n",
            (unsigned)HeadingBlackLineMaxMs,
            gGyro.heading(), target_heading,
            Gyro::wrapDeg180(gGyro.heading() - target_heading));
  }
  return line_detected;
}

bool Nansho2Scene::runStraightUntilBlackLineThenHeading(
    float before_heading, float after_heading,
    int before_left_power, int before_right_power,
    float after_left_power, float after_right_power,
    float after_left_rotations, float after_right_rotations,
    float tolerance_deg)
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  gMotors.setHeadingCorrection(true);
  gMotors.setTargetHeading(before_heading);
  gMotors.run(before_left_power, before_right_power);

  bool black_detected = false;
  bool detected_blackline = false;
  bool detected_blue = false;
  bool detected_skyblue = false;
  bool detected_low_reflection = false;
  for (uint32_t elapsed_ms = 0U; elapsed_ms < AfterStraightBlackMaxMs;
       elapsed_ms += StepMs) {
    gMotors.tickCorrection();
    gMotors.debugHeading();
    detected_blackline = gColorLAB.is(COLOR_DETECT_BLACKLINE, true);
    detected_blue = gColorLAB.is(COLOR_DETECT_BLUE, true);
    detected_skyblue = gColorLAB.is(COLOR_DETECT_SKYBLUE, true);
    detected_low_reflection =
        gColorLAB.reflection() <= AfterStraightBlackReflectionMax;
    if (detected_blackline || detected_blue || detected_skyblue ||
        detected_low_reflection) {
      black_detected = true;
      break;
    }
    dly_tsk(StepMs * 1000U);
  }

  gMotors.stop();
  if (!black_detected) {
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "Nansho2: stop reason=black-line-detection-failed,"
            "max_ms=%u,reflection=%ld,H=%.2f,T=%.2f,E=%.2f\n",
            (unsigned)AfterStraightBlackMaxMs,
            (long)gColorLAB.reflection(),
            gGyro.heading(), before_heading,
            Gyro::wrapDeg180(gGyro.heading() - before_heading));
    return false;
  }

  fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
          "Nansho2: black-line-detected,black=%s,blue=%s,skyblue=%s,"
          "low_reflection=%s,reflection=%ld,H=%.2f,T=%.2f,E=%.2f\n",
          detected_blackline ? "true" : "false",
          detected_blue ? "true" : "false",
          detected_skyblue ? "true" : "false",
          detected_low_reflection ? "true" : "false",
          (long)gColorLAB.reflection(),
          gGyro.heading(), before_heading,
          Gyro::wrapDeg180(gGyro.heading() - before_heading));

  gMotors.setHeadingDebugPhase("Straight::turnByAngle(after_line)");
  gMotors.turnByAngle(after_heading, tolerance_deg,
                      70, 70, 5000U, 30.0f, 30);
  const float turn_heading = gGyro.heading();
  const float turn_error =
      Gyro::wrapDeg180(turn_heading - after_heading);
  if (fabsf(turn_error) > tolerance_deg) {
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "Nansho2: stop reason=turn-heading-not-reached,"
            "H=%.2f,T=%.2f,E=%.2f,tolerance=%.2f\n",
            turn_heading, after_heading, turn_error, tolerance_deg);
    return false;
  }
  fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
          "Nansho2: turn-complete,H=%.2f,T=%.2f,E=%.2f\n",
          turn_heading, after_heading, turn_error);

  gMotors.setTargetHeading(after_heading);
  gMotors.setHeadingDebugPhase(
      "Straight::runForRotationsSameTimeWithHeading(after_line)");
  const bool distance_completed = gMotors.runForRotationsSameTimeWithHeading(
      after_left_power, after_right_power,
      after_left_rotations, after_right_rotations,
      true, tolerance_deg);
  const float final_heading = gGyro.heading();
  const float final_error =
      Gyro::wrapDeg180(final_heading - after_heading);
  if (!distance_completed) {
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "Nansho2: stop reason=after-line-rotations-not-completed,"
            "left_rotations=%.3f,right_rotations=%.3f,"
            "H=%.2f,T=%.2f,E=%.2f,tolerance=%.2f\n",
            after_left_rotations, after_right_rotations,
            final_heading, after_heading, final_error, tolerance_deg);
    return false;
  }
  if (fabsf(final_error) > tolerance_deg) {
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "Nansho2: stop reason=after-line-heading-not-reached,"
            "H=%.2f,T=%.2f,E=%.2f,tolerance=%.2f\n",
            final_heading, after_heading, final_error, tolerance_deg);
    return false;
  }
  fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
          "Nansho2: after-line-run-complete,H=%.2f,T=%.2f,E=%.2f\n",
          final_heading, after_heading, final_error);
  return true;
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

void Nansho2Scene::setupPostBlackAfterTurnTrace()
{
  gLineTracer.setBaseSpeed(PostBlackTraceLeftPower,
                           PostBlackTraceRightPower);
  gLineTracer.setPid(PostBlackAfterTurnTraceKp,
                     PostBlackAfterTurnTraceKi,
                     PostBlackAfterTurnTraceKd,
                     PostBlackAfterTurnTraceOutMax);
  gLineTracer.setScale(PostBlackAfterTurnTraceScale);
  gLineTracer.setEdgeFollow(PostBlackTraceEdgeFollow);
}

void Nansho2Scene::runLineTraceUntilColor(color_detect_t color,
                                          const char *blue_log_source,
                                          bool after_turn)
{
  if (after_turn) {
    setupPostBlackAfterTurnTrace();
  } else {
    setupPostBlackLineTrace();
  }
  // pid,scale,speed,edgeのセッター

  uint32_t elapsed_ms = 0U;
  uint8_t blue_hits = 0U;
  const bool blue_target =
      color == COLOR_DETECT_SKYBLUE || color == COLOR_DETECT_BLUE;
  while (elapsed_ms < LtMaxMs) {
    gLineTracer.step();
    gMotors.debugHeading();

    const bool color_hit = gColorLAB.is(color, true);
    const bool blue_hit = blue_target &&
        (gColorLAB.is(COLOR_DETECT_SKYBLUE, true) ||
         gColorLAB.is(COLOR_DETECT_BLUE, true));
    if (blue_hit) {
      if (blue_hits < 3U) {
        ++blue_hits;
      }
    } else {
      blue_hits = 0U;
    }
    const bool hit = color_hit || blue_hits >= 2U;
    if (hit) {
      if (color == COLOR_DETECT_BLACKLINE) {
        beepOnBlackDetected();
      } else if (color == COLOR_DETECT_PETBLUE ||
                 color == COLOR_DETECT_SKYBLUE ||
                 color == COLOR_DETECT_BLUE) {
        logBlueLineDetected(blue_log_source);
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
  runStraightAfterTurn(heading, left_power, right_power, target_deg,
                       RouteFinalApproachBalanceKp, RouteFinalApproachBalanceKi,
                       RouteFinalApproachBalanceIntegralMax);
}

void Nansho2Scene::runStraightAfterTurn(float heading, int left_power,
                                        int right_power, int32_t target_deg,
                                        float kp, float ki, float i_max)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(kp, ki, i_max);
  gMotors.setTargetHeading(heading);

  const int32_t enc_left_base  = gMotors.leftEncoderCount();
  const int32_t enc_right_base = gMotors.rightEncoderCount();
  gMotors.run(left_power, right_power);

  uint32_t elapsed_ms = 0U;
  while (gMotors.traveledDegFrom(enc_left_base, enc_right_base) < target_deg &&
         elapsed_ms < StraightRunMaxMs) {
    gMotors.tickCorrection();
    gMotors.debugHeading();
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  if (elapsed_ms >= StraightRunMaxMs) {
    fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
            "Nansho2: stop reason=straight-distance-timeout,"
            "max_ms=%u,target_deg=%ld\n",
            (unsigned)StraightRunMaxMs, (long)target_deg);
  }
  dly_tsk(StepMs * 1000U);
}

void Nansho2Scene::runAfterFinalStopTurnRightAndSlowTraceToWhite(bool do_turn)
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  if (do_turn) {
    if (route_branch_ == RouteBranch::Red) {
      /* ラインをつかんだ向きのまま、ほんの少し乗せてから旋回する */
      runStraightAfterTurn(RedAfterBackTurnDeg, RedAfterLineForwardPower,
                           RedAfterLineForwardPower, RedAfterLineForwardDeg);
    }
    /* パワーは定数の符号のまま出す。turnToHeading のように向きから
     * 符号を付け直すことはしない。 */
    const float turn_target =
        (route_branch_ == RouteBranch::Red) ? RedAfterFinalTurnDeg
                                            : AfterFinalTurnTargetDeg;
    uint32_t turn_ms = 0U;
    while (turn_ms < LtMaxMs) {
      if (gGyro.reached(turn_target, AfterFinalTurnToleranceDeg)) {
        break;
      }
      gMotors.run(AfterFinalTurnLeftPower, AfterFinalTurnRightPower);
      dly_tsk(StepMs * 1000U);
      turn_ms += StepMs;
    }
    gMotors.stop();
  }

  gLineTracer.setEnabled(true);
  if (route_branch_ == RouteBranch::Red) {
    gLineTracer.setBaseSpeed(FinalRedSlowTracePower, FinalRedSlowTracePower);
    gLineTracer.setPid(FinalRedSlowTraceKp, FinalRedSlowTraceKi,
                       FinalRedSlowTraceKd, FinalRedSlowTraceOutMax);
    gLineTracer.setScale(FinalRedSlowTraceScale);
  } else if (route_branch_ == RouteBranch::Yellow) {
    gLineTracer.setBaseSpeed(FinalYellowStartTracePower,
                             FinalYellowStartTracePower);
    gLineTracer.setPid(FinalSlowTraceKp, FinalSlowTraceKi, FinalSlowTraceKd,
                       FinalSlowTraceOutMax);
    gLineTracer.setScale(FinalSlowTraceScale);
  } else {
    gLineTracer.setBaseSpeed(FinalSlowTracePower, FinalSlowTracePower);
    gLineTracer.setPid(FinalSlowTraceKp, FinalSlowTraceKi, FinalSlowTraceKd,
                       FinalSlowTraceOutMax);
    gLineTracer.setScale(FinalSlowTraceScale);
  }
  gLineTracer.setEdgeFollow(FinalSlowTraceEdgeFollow);

  uint32_t elapsed_ms = 0U;
  bool     yellow_slowed = false;
  while (elapsed_ms < LtMaxMs) {
    if (route_branch_ == RouteBranch::Yellow && !yellow_slowed &&
        elapsed_ms >= FinalYellowStartTraceMs) {
      gLineTracer.setBaseSpeed(FinalSlowTracePower, FinalSlowTracePower);
      yellow_slowed = true;
    }

    gLineTracer.step();
    gMotors.debugHeading();

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
                                     uint32_t ignore_ms, bool then_final)
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(RouteFinalBackBalanceKp, RouteFinalBackBalanceKi,
                        RouteFinalBackBalanceIntegralMax);
  gMotors.setTargetHeading(heading);
  gMotors.run(-power, -power);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < LtMaxMs) {
    gMotors.tickCorrection();
    gMotors.debugHeading();

    if (elapsed_ms >= ignore_ms) {
      if (gColorLAB.reflection() <= AfterFinalBackReflectionLineMax) {
        beepOnBlackDetected();
        break;
      }
      if (gColorLAB.is(COLOR_DETECT_BLUE, true)) {
        logBlueLineDetected("route-final-back");
        beepOnBlueDetected();
        break;
      }
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  dly_tsk(StepMs * 1000U);
  if (then_final) {
    runAfterFinalStopTurnRightAndSlowTraceToWhite();
  }
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
  // ボトルを置く位置まで前進
  runStraightAfterTurn(turn_target, RouteFinalApproachLeftPower,
                       RouteFinalApproachRightPower, RouteFinalApproachDeg);
  gGyro.reset();
  dly_tsk(StepMs * 1000U);
  runStraightAfterTurn(0.0f, -RedFinalBackPower, -RedFinalBackPower,
                       RedFirstBackDeg, RouteFinalBackBalanceKp,
                       RouteFinalBackBalanceKi,
                       RouteFinalBackBalanceIntegralMax);
  turnToHeading(RedAfterBackTurnDeg, RedAfterBackTurnToleranceDeg,
                RedAfterBackTurnLeftPower, RedAfterBackTurnRightPower);
  /* ラインをつかんだあと: 少し直進 → 絶対角旋回 → 低速LT → 白 */
  runRouteFinalBack(RedAfterBackTurnDeg, RedFinalBackPower, RedFinalBackIgnoreMs);
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
  blue_line_count_ = 0U;
  /* PowerShell のBluetoothシリアルで、Nansho2走行中の角度を観測する。 */
  gGyro.reset();
  // runInitialLineTrace();/*青からの黒検知までライトレ*/
  // runShortBackAfterGray();
  raiseArm();/*腕上げ*/
  driveForwardAndDetectRouteBranch();
// ↑直進、腕上げ、色読み、後退、腕下げ、
  if (route_branch_ != RouteBranch::None) {/*何かしら色読んだら*/
    if (!kimeuti){
      runLineTraceUntilHeadingThenStraight();
    }else{
      gMotors.setHeadingDebugPhase("HCR::LineTraceRotation");
      gHCR.LineTraceRotation(1.77f,1.77f,80.0f,80.0f,0.07f,0.0006f,0.5f,10.0f);
      gMotors.setBalancePid(HeadingRunKp, HeadingRunKi,
                            HeadingRunIntegralMax, 0.0f, HeadingRunKd);
      gMotors.setHeadingTrimLimit(HeadingRunTrimLimit);
      gMotors.setHeadingRunPowerScale(HeadingRunPowerScale);
      gMotors.setHeadingDebugPhase("Straight::runForRotationsUntilAngle");
      gMotors.runForRotationsUntilAngleNansho2Kimeuti(
        turn_speed_l, turn_speed_r,  // 左右パワー
        turn_90_l, turn_90_r,    // 左右最大回転数
        heading_90,    // gGyro.reset()基準の目標角
        1.0f,          // 許容誤差
        true,          // 未達ならピボット補正
        40.0f,            // 最低補正旋回パワー
        5000U,          // 補正最大時間
        true
      );
      gMotors.setTargetHeading(heading_90);
      gMotors.setBalancePid(Heading90RunKp, Heading90RunKi,
                            Heading90RunIntegralMax, 0.0f,
                            Heading90RunKd);
      gMotors.setHeadingRunPowerScale(Heading90RunPowerScale);
      gMotors.setHeadingCorrectionScale(Heading90RunCorrectionScale);
      gMotors.setHeadingDebugPhase(
          "Straight::runForRotationsSameTimeWithHeading(initial)");
      gMotors.runForRotationsSameTimeWithHeading(
          motor_speed, motor_speed,
          HeadingRunInitialRotations, HeadingRunInitialRotations,
          true, 0.3f);
      gMotors.runForRotationsSameTimeWithHeading(
          motor_speed, motor_speed,
          heading_100_rotation - HeadingRunInitialRotations,
          heading_100_rotation - HeadingRunInitialRotations,
          true, 0.3f);
      stp_cyc(LINE_TRACER_TASK_CYC);
      stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
      gMotors.shutdownDebugOutput();
      runStraightAtHeadingUntilBlackLine(heading_90, (int)motor_speed_slow,
                                          (int)motor_speed_slow);
      // 90度直進: 指定角度を維持し、黒線検知で停止
      gMotors.runForRotationsUntilAngleNansho2Kimeuti(
        turn_speed_l, turn_speed_r,  // 左右パワー
        turn_90_l, turn_90_r,    // 左右最大回転数
        180.0f,    // gGyro.reset()基準の目標角
        1.0f,          // 許容誤差
        true,          // 未達ならピボット補正
        40.0f,            // 最低補正旋回パワー
        5000U,          // 補正最大時間
        true
      );
      runLowSpeedTraceAndBranchToNansho3();
      // 180度旋回後 → 低速LT → SKYBLUE → 色別ルート → 難所3
    }
  }
}

extern "C" void Nansho2Task(intptr_t exinf)
{
  (void)exinf;
  gNansho2.run();

#if ETROBO_SCENE == 0 || ETROBO_SCENE == 5 || ETROBO_SCENE == 6
  if (!gNansho2.handedOffToNansho3()) {
    gMotors.stop();
    gLineTracer.setEnabled(false);
    dly_tsk(50 * 1000U);
  }
#endif

  ext_tsk();
}
