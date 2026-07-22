#include "Nansho1.h"

#include "app.h"
#include "kernel_cfg.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "Common/Straight/Straight.h"
#include "Common/LineTracer/LineTracer.h"

extern Sonic gSonic;
extern ColorRGB gColorRGB;
extern ColorLAB gColorLAB;
extern Gyro gGyro;
extern Straight   gMotors;
extern LineTracer gLineTracer;
extern Nansho1Scene gNansho1;
/* ---------------------------------------------------------------------
 * 旋回中スキャン: target_heading までピボット旋回しつつ
 *   - 超音波距離 <= DetectMaxMm               → SonicDetected
 *   - 目標角に到達 (|rem| <= tolerance_deg)   → Completed
 *   - TurnMaxMs 経過                            → Completed (安全停止)
 *
 *  注: 黒線判定はここでは行わない。
 *  その場ピボット旋回中はカラーセンサが弧を描いて元の黒線を
 *  横切ってしまい誤終了するため、黒は前進フェーズでのみ見る。
 *  wigwag 中はジャイロリセットしない。
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult
Nansho1Scene::rotateToHeadingWithScan(float target_heading, float *out_heading,
                                      ScanSide scan_side, float tolerance_deg)
{
  float prev = gGyro.heading();
  const float delta = Gyro::wrapDeg180(target_heading - prev);
  if (fabsf(delta) <= tolerance_deg) {
    gMotors.stop();
    return ScanResult::Completed;
  }

  const float turn_dir = (delta > 0.0f) ? 1.0f : -1.0f;
  const float need     = fabsf(delta);
  float       traveled = 0.0f;

  uint32_t elapsed_ms = 0U;
  bool     left_sonic_pending = false;
  uint32_t left_delay_ms = 0U;

  while (elapsed_ms < TurnMaxMs) {
    const float h = gGyro.heading();
    traveled += turn_dir * Gyro::wrapDeg180(h - prev);
    prev = h;

    const float rem = need - traveled;

    /* (1) 超音波距離検知 (旋回中も常時監視) */
    if (!left_sonic_pending && gSonic.isCloserThan(DetectMaxMm)) {
      if (scan_side == ScanSide::Left) {
        left_sonic_pending = true;
        if (out_heading != nullptr) {
          *out_heading = h;
        }
      } else {
        gMotors.stop();
        if (out_heading != nullptr) {
          *out_heading = h;
        }
        return ScanResult::SonicDetected;
      }
    }

    /* (2) 左走査で検知後: n ms の間は旋回継続 → 経過後に SonicDetected で確定 */
    if (left_sonic_pending) {
      gMotors.pivotTowardDiff(ScanTurnPower, turn_dir * rem);

      dly_tsk(StepMs * 1000U);
      elapsed_ms += StepMs;
      left_delay_ms += StepMs;
      if (left_delay_ms >= LeftScanDetectContinueMs) {
        gMotors.stop();
        return ScanResult::SonicDetected;
      }
      continue;
    }

    /* (3) 目標角度に到達 (または行き過ぎ) → 正面戻りでも 180° 回り続けない */
    if (rem <= tolerance_deg) {
      gMotors.stop();
      return ScanResult::Completed;
    }

    /* (4) 開始時に決めた方向だけへピボット旋回 */
    gMotors.pivotTowardDiff(ScanTurnPower, turn_dir * rem);

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  if (left_sonic_pending) {
    return ScanResult::SonicDetected;
  }
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * 直進中スキャン: 指定時間 power で前進しつつ
 *   - 超音波距離 <= DetectMaxMm → SonicDetected
 *   - 黒線           → BlackDetected
 *   - 時間切れ         → Completed
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult
Nansho1Scene::driveForwardWithScan(uint32_t duration_ms, int power, float *out_heading)
{
  uint32_t elapsed_ms = 0U;

  /* runFor ではなく定速 run + 周期補正。
   * LineTracer 周期タスクは Straight 区間では setEnabled(false) 必須。 */
  gMotors.setTargetHeading(gGyro.heading());
  gMotors.run(power, power);

  while (elapsed_ms < duration_ms) {
    const float h = gGyro.heading();
    gMotors.tickCorrection();

    if (gSonic.isCloserThan(DetectMaxMm)) {
      gMotors.stop();
      if (out_heading != nullptr) {
        *out_heading = h;
      }
      return ScanResult::SonicDetected;
    }

    if (gColorLAB.is(COLOR_DETECT_BLACK, true)) {
      gMotors.stop();
      return ScanResult::BlackDetected;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * 旋回走査 (片側振り・規定回数で打ち切り)
 *   各サイクル (ジャイロリセットなし):
 *     a) 正面から amp_deg だけ側方を向く
 *     b) 振り始めの正面へ向き直す
 *     c) WigWagForwardMs だけ前進
 *
 *   返り値 (ScanResult):
 *     SonicDetected  = 超音波検知 (out_heading に検知時ヨー角)
 *     BlackDetected  = 前進中に黒線を踏んだ
 *     Completed      = max_cycles サイクルを検知なしで完了
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult
Nansho1Scene::wigwagScan(float amp_deg, int max_cycles, float *out_heading,
                         ScanSide scan_side)
{
  for (int cycle = 1; cycle <= max_cycles; ++cycle) {
    /* このサイクルの正面。側方を見たあと、ここへ向き直す。 */
    const float front = gGyro.heading();

    /* a) 側方を向く */
    switch (rotateToHeadingWithScan(front + amp_deg, out_heading, scan_side,
                                    WigWagOutToleranceDeg)) {
      case ScanResult::SonicDetected:  return ScanResult::SonicDetected;
      case ScanResult::BlackDetected:  return ScanResult::BlackDetected;
      case ScanResult::Completed:      break;
    }

    /* b) 反対に旋回して正面へ向き直す */
    switch (rotateToHeadingWithScan(front, out_heading, scan_side,
                                    WigWagReturnToleranceDeg)) {
      case ScanResult::SonicDetected:  return ScanResult::SonicDetected;
      case ScanResult::BlackDetected:  return ScanResult::BlackDetected;
      case ScanResult::Completed:      break;
    }

    /* c) 少し前進 */
    switch (driveForwardWithScan(WigWagForwardMs, WigWagForwardPower, out_heading)) {
      case ScanResult::SonicDetected:  return ScanResult::SonicDetected;
      case ScanResult::BlackDetected:  return ScanResult::BlackDetected;
      case ScanResult::Completed:      break;
    }
  }
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * ③〜⑤ wig走査フェーズ (LRコースで左右・検知後分岐を app.h から反転)。
 * --------------------------------------------------------------------- */
bool Nansho1Scene::runWigwagScanPhases()
{
  const float first_amp = ETROBO_COURSE_L ? WigWagAmpDeg : -WigWagAmpDeg;
  const float second_amp = ETROBO_COURSE_L ? -WigWagAmpDeg : WigWagAmpDeg;
  const ScanSide first_side = ETROBO_COURSE_L ? ScanSide::Right : ScanSide::Left;
  const ScanSide second_side = ETROBO_COURSE_L ? ScanSide::Left : ScanSide::Right;

  if (wigwagScan(first_amp, ScanCycles, nullptr, first_side) ==
      ScanResult::SonicDetected) {
    /* 検知後: 90°を向いて停止 → 0°に向き直す → 黒まで直進 */
    faceDetectHeadingThenReturnToZero();
    driveForwardUntilBlackLine();
    runAfterFinalBlackLine();
    return true;
  }

  readOppositeSide();

  if (wigwagScan(second_amp, ScanCycles, nullptr, second_side) ==
      ScanResult::SonicDetected) {
    faceDetectHeadingThenReturnToZero();
    driveForwardUntilBlackLine();
    runAfterFinalBlackLine();
    return true;
  }

  return false;
}

/* ---------------------------------------------------------------------
 * 円の反対を読み取る動き:
 *   1. まっすぐ指定時間直進
 *   2. 90° 旋回 → ジャイロリセット → 指定時間直進
 *   3. 90° 旋回 → ジャイロリセット → 灰色 (グレーライン) を踏むまで直進
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult Nansho1Scene::readOppositeSide()
{
  gMotors.runFor(OppositeForwardPower, OppositeForwardPower,
                 OppositeForwardFirstMs);
  dly_tsk(StepMs * 1000U);

  turnToHeading(OppositeTurnDeg, OppositeTurnToleranceDeg,
                OppositeTurnPower, OppositeTurnPower, true);
  dly_tsk(StepMs * 1000U);
  gMotors.setTargetHeading(0.0f);
  gMotors.runFor(OppositeForwardPower, OppositeForwardPower,
                 OppositeForwardSecondMs);
  dly_tsk(StepMs * 1000U);

  turnToHeading(OppositeTurnDeg, OppositeTurnToleranceDeg,
                OppositeTurnPower, OppositeTurnPower, true);
  dly_tsk(StepMs * 1000U);
  driveUntilGrayLine(0U, OppositeForwardPower, OppositeForwardPower);

  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * 黒線を読む動き:
 *   1. 初期化したゼロ度を基準とした 0° を向く
 *      (ここでは gGyro.reset() しない。直前に初期化された 0° 基準のまま)
 *   2. 黒線を踏むまで前進して停止する。
 * --------------------------------------------------------------------- */
void Nansho1Scene::driveUntilBlackLine(float target_heading)
{
  turnToHeading(target_heading);
  dly_tsk(StepMs * 1000U);

  /* turnToHeading 末尾で reset 済み → 現在向きが 0° */
  gMotors.setTargetHeading(0.0f);
  while (1) {
    if (gColorLAB.is(COLOR_DETECT_BLACK, true)) {
      gMotors.stop();
      break;
    }
    gMotors.run(LeftPower, RightPower);
    dly_tsk(StepMs * 1000U);
  }
}

/* ---------------------------------------------------------------------
 * 灰色ラインを踏むまで直進して停止する。
 * 開始直後 gray_ignore_ms の間は灰色を見ない (0 なら最初から判定)。
 * --------------------------------------------------------------------- */
void Nansho1Scene::driveUntilGrayLine(uint32_t gray_ignore_ms,
                                      int left_power, int right_power)
{
  uint32_t elapsed_ms = 0U;

  while (1) {
    if (elapsed_ms >= gray_ignore_ms &&
        gColorRGB.is(COLOR_DETECT_GRAYLINE)) {
      gMotors.stop();
      break;
    }
    gMotors.run(left_power, right_power);
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
}

/* ---------------------------------------------------------------------
 * 現在の向きのまま黒線エッジを踏むまで前進して停止する。
 * --------------------------------------------------------------------- */
void Nansho1Scene::driveForwardUntilBlackLine()
{
  /* turnToHeading 末尾で reset 済みなら現在向きが 0° */
  gMotors.setTargetHeading(0.0f);
  while (1) {
    if (gColorLAB.is(COLOR_DETECT_BLACK, true)) {
      gMotors.stop();
      break;
    }
    gMotors.run(LeftPower, RightPower);
    dly_tsk(StepMs * 1000U);
  }
}

/* ---------------------------------------------------------------------
 * 検知後の終了動作 (走査方向で分岐):
 *   共通: 灰色直進後 → 待機 → バック
 *   Right: 180° → 黒線まで直進
 *   Left:  heading < -56° → (0°+offset) 向き黒線
 *          heading >= -56° → (-90°+offset) → 直進 → (90°+offset) → 黒線まで直進
 * --------------------------------------------------------------------- */
void Nansho1Scene::finishAfterDetect(ScanSide side, float heading_offset_deg)
{
  dly_tsk(DetectFinishBackDelayMs * 1000U);
  gMotors.runFor(-DetectFinishBackPower, -DetectFinishBackPower,
                 DetectFinishBackMs);
  dly_tsk(StepMs * 1000U);

  if (side == ScanSide::Right) {
    turnToHeading(DetectAfterSonicTurnDeg);
    driveForwardUntilBlackLine();
    runAfterFinalBlackLine();
    return;
  }

  const float h = gGyro.heading();

  if ((float)DetectFinishAngleSign * h <
      (float)DetectFinishAngleSign * DetectFinishClassicMaxDeg) {
    driveUntilBlackLine(0.0f + heading_offset_deg);
    runAfterFinalBlackLine();
    return;
  }

  /* 左走査・広い検知角ルート */
  turnToHeading(DetectWideTurnFirstDeg + heading_offset_deg);
  gMotors.runFor(LeftPower, RightPower, DetectMidForwardMs);
  dly_tsk(StepMs * 1000U);

  /* 黒線向きの最終旋回は offset を加えない（+180° 時に 180° 逆になるため） */
  turnToHeading(DetectWideTurnSecondDeg);
  driveForwardUntilBlackLine();
  runAfterFinalBlackLine();
}
/* ---------------------------------------------------------------------
 * 目標ヨー角まで短い側にピボット旋回 (検知判定なし)
 * --------------------------------------------------------------------- */
void Nansho1Scene::turnToHeading(float target_heading, float tolerance_deg)
{
  turnToHeading(target_heading, tolerance_deg, TurnPower, TurnPower, true);
}

void Nansho1Scene::turnToHeading(float target_heading, float tolerance_deg,
                                 int left_power, int right_power,
                                 bool reset_after)
{
  /* 開始時点の最短経路だけを、経過角で打ち切る。
   * heading の ±180 折り返しでも local_target 差が壊れないようにする。 */
  float prev = gGyro.heading();
  const float delta = Gyro::wrapDeg180(target_heading - prev);
  if (fabsf(delta) <= tolerance_deg) {
    gMotors.stop();
    dly_tsk(StepMs * 1000U);
    if (reset_after) {
      gGyro.reset();
    }
    return;
  }

  const float turn_dir = (delta > 0.0f) ? 1.0f : -1.0f;
  const float need     = fabsf(delta);
  float       traveled = 0.0f;

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < TurnMaxMs) {
    const float h = gGyro.heading();
    traveled += turn_dir * Gyro::wrapDeg180(h - prev);
    prev = h;

    const float rem = need - traveled;
    /* 目標到達、または行き過ぎたら即停止（90°超え防止） */
    if (rem <= tolerance_deg) {
      break;
    }

    int lp = left_power;
    int rp = right_power;
    if (rem <= TurnSlowApproachDeg) {
      lp = TurnSlowPower;
      rp = TurnSlowPower;
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
  if (reset_after) {
    gGyro.reset();
  }
}

/* ---------------------------------------------------------------------
 * 超音波検知後: DetectAfterSonicTurnDeg を向いて停止 → 元の正面へ向き直す。
 * 各旋回の完了ごとに reset する。
 * --------------------------------------------------------------------- */
void Nansho1Scene::faceDetectHeadingThenReturnToZero()
{
  turnToHeading(DetectAfterSonicTurnDeg);
  dly_tsk(StepMs * 1000U);
  /* reset 後の 0° が旧 DetectAfterSonic 向き → 符号反転で元の正面へ */
  turnToHeading(-DetectAfterSonicTurnDeg);
}

/* ---------------------------------------------------------------------
 * 開始時: ライントレース (スカイブルー → 黒) → 90° 旋回。
 * --------------------------------------------------------------------- */
void Nansho1Scene::runInitialForwardTrace()
{
  gGyro.reset();

  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(InitialTracePower, InitialTracePower);
  gLineTracer.setPid(0.30f, 0.01f, 0.8f, 100.0f);
  gLineTracer.setEdgeFollow(ETROBO_COURSE_L != 0);

  bool skyblue_seen = false;
  while (1) {
    gLineTracer.step();

    if (!skyblue_seen) {
      if (gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
        skyblue_seen = true;
      }
    } else if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
      break;
    }

    dly_tsk(StepMs * 1000U);
  }
  gMotors.stop();
  gLineTracer.setEnabled(false);

  turnToHeading(InitialBlackTurnDeg);
}

/* ---------------------------------------------------------------------
 * 最終黒線読み後の旋回 (FinalBlackTurnLeft で左右指定)。
 * --------------------------------------------------------------------- */
void Nansho1Scene::turnAfterFinalBlackLine()
{
  const float target_heading =
      gGyro.heading() +
      (float)FinalBlueTraceTurnSign * FinalBlueTraceTurnDeg;
  uint32_t elapsed_ms = 0U;

  while (elapsed_ms < FinalBlueTraceTurnMaxMs) {
    if (gGyro.reached(target_heading, FinalBlueTraceTurnToleranceDeg)) {
      break;
    }

    gMotors.run(FinalBlueTraceTurnLeftPower, FinalBlueTraceTurnRightPower);
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gGyro.reset();
  gMotors.stop();
}

/* ---------------------------------------------------------------------
 * 最終黒線読み後: 指定方向に 90° 旋回 → 青線までライントレース。
 * --------------------------------------------------------------------- */
void Nansho1Scene::runAfterFinalBlackLine()
{
  turnAfterFinalBlackLine();
  dly_tsk(StepMs * 1000U);

  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(FinalBlueTracePower, FinalBlueTracePower);
  gLineTracer.setPid(0.30f, 0.0005f, 0.45f, 100.0f);
  gLineTracer.setEdgeFollow(ETROBO_COURSE_L == 0);

  while (1) {
    gLineTracer.step();

    if (gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
      break;
    }

    dly_tsk(StepMs * 1000U);
  }
  gMotors.stop();
}

/* ---------------------------------------------------------------------
 * シーン本体
 * --------------------------------------------------------------------- */
void Nansho1Scene::run()
{
  /* ⓪ 開始時: ライントレース → 90° 旋回。 */
  runInitialForwardTrace();

  /* 開始時: ジャイロを初期化する。 */
  gGyro.reset();

  
  /* ① 旋回後: 低速 LT → 本線 LT で円の終端まで移動 (固定時間・減速あり)。 */
  {
    gLineTracer.setEnabled(true);
    gLineTracer.setPid(0.6f, 0.0005f, 0.8f, 100.0f);
    gLineTracer.setEdgeFollow(false);

    /* 旋回直後は低速で線に乗せる。 */
    gLineTracer.setBaseSpeed((float)BackSlowTracePower, (float)BackSlowTracePower);
    for (uint32_t elapsed_ms = 0U; elapsed_ms < BackSlowTraceMs;
         elapsed_ms += StepMs) {
      gLineTracer.step();
      dly_tsk(StepMs * 1000U);
    }

    uint32_t elapsed_ms = 0U;
    while (elapsed_ms < BackToEndMs) {
      int trace_power = BackTracePower;
      if (elapsed_ms < BackTraceDecelMs) {
        trace_power = BackTracePower -
                      (int)((float)elapsed_ms / (float)BackTraceDecelMs *
                            (float)(BackTracePower - BackTraceEndPower));
      } else {
        trace_power = BackTraceEndPower;
      }

      gLineTracer.setBaseSpeed((float)trace_power, (float)trace_power);
      gLineTracer.step();
      dly_tsk(StepMs * 1000U);
      elapsed_ms += StepMs;
    }
    gGyro.reset();
    gMotors.stop();
  }
  gLineTracer.setEnabled(false);
  dly_tsk(StepMs * 1000U);

  /* ② 円の方向へ旋回 (turnToHeading 末尾で reset 済み)。 */
  turnToHeading(PreWigwagTurnDeg);
  dly_tsk(StepMs * 1000U);
  /* 指定時間だけ前進してから走査を始める。 */
  gMotors.setBalancePid(PreWigwagBalanceKp, PreWigwagBalanceKi,
                        PreWigwagBalanceIntegralMax);
  gMotors.setTargetHeading(0.0f);
  gMotors.runFor(LeftPower, RightPower, ForwardBeforeScanMs);
  dly_tsk(StepMs * 1000U);

  if (runWigwagScanPhases()) {
    return;
  }

  /* ⑥ 両方の wig走査で検知なし → 黒線を読む動きへ。 */
  driveUntilBlackLine();
  runAfterFinalBlackLine();
}

extern "C" void Nansho1Task(intptr_t exinf)
{
  (void)exinf;
  gNansho1.run();

#if ETROBO_SCENE == 0
  gMotors.stop();
  gLineTracer.setEnabled(false);
  dly_tsk(50 * 1000U);
  act_tsk(NANSHO2_TASK);
#endif

  ext_tsk();
}
