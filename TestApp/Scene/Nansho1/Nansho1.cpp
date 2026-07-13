#include "Nansho1.h"

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
 *   - 超音波で DetectMaxMm 以下を検知           → SonicDetected
 *   - 目標角に到達 (|diff| <= ScanTurnToleranceDeg) → Completed
 *   - TurnMaxMs 経過                            → Completed (安全停止)
 *
 *  注: 黒線判定はここでは行わない。
 *  その場ピボット旋回中はカラーセンサが弧を描いて元の黒線を
 *  横切ってしまい誤終了するため、黒は前進フェーズでのみ見る。
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult
Nansho1Scene::rotateToHeadingWithScan(float target_heading, float *out_heading)
{
  uint32_t elapsed_ms = 0U;
  uint32_t log_acc_ms = 0U;

  while (elapsed_ms < TurnMaxMs) {
    const int32_t dist = gSonic.readDistanceMm();
    const float   h    = gGyro.heading();

    /* (1) 超音波検知 (旋回中も常時監視) */
    if (dist > 0 && dist <= DetectMaxMm) {
      gMotors.stop();
      if (out_heading != nullptr) {
        *out_heading = h;
      }
      return ScanResult::SonicDetected;
    }

    /* (2) 目標角度に到達? */
    const float diff = gGyro.diffTo(target_heading);
    if (gGyro.reached(target_heading, ScanTurnToleranceDeg)) {
      gMotors.stop();
      return ScanResult::Completed;
    }

    /* (3) 短い方向にピボット旋回
     *   diff > 0 → ヨー角を増やしたい → CW
     *   diff < 0 → ヨー角を減らしたい → CCW */
    gMotors.pivotTowardDiff(ScanTurnPower, diff);

    log_acc_ms += StepMs;
    if (log_acc_ms >= 500U) { 
      log_acc_ms = 0U;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  /* 安全タイムアウト */
  gMotors.stop();
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * 直進中スキャン: 指定時間 power で前進しつつ
 *   - 超音波で検知 → SonicDetected
 *   - 黒線       → BlackDetected
 *   - 時間切れ     → Completed
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult
Nansho1Scene::driveForwardWithScan(uint32_t duration_ms, int power, float *out_heading)
{
  uint32_t elapsed_ms = 0U;
  uint32_t log_acc_ms = 0U;

  while (elapsed_ms < duration_ms) {
    const int32_t dist = gSonic.readDistanceMm();
    const float   h    = gGyro.heading();

    if (dist > 0 && dist <= DetectMaxMm) {
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

    gMotors.run(power, power);

    log_acc_ms += StepMs;
    if (log_acc_ms >= 500U) {
      log_acc_ms = 0U;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * 旋回走査 (片側振り・規定回数で打ち切り)
 *   各サイクル:
 *     a) amp_deg まで旋回 (検知監視)   ※右走査は +90、左走査は -90
 *     b) 0° に戻る旋回         (検知監視)
 *     c) WigWagForwardMs だけ前進 (検知監視)
 *     d) 次サイクルに備えてヨー角リセット
 *
 *   返り値 (ScanResult):
 *     SonicDetected  = 超音波検知 (out_heading に検知時ヨー角)
 *     BlackDetected  = 前進中に黒線を踏んだ
 *     Completed      = max_cycles サイクルを検知なしで完了
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult
Nansho1Scene::wigwagScan(float amp_deg, int max_cycles, float *out_heading)
{
  for (int cycle = 1; cycle <= max_cycles; ++cycle) {
    /* a) amp_deg まで旋回 */
    switch (rotateToHeadingWithScan(amp_deg, out_heading)) {
      case ScanResult::SonicDetected:  return ScanResult::SonicDetected;
      case ScanResult::BlackDetected:  return ScanResult::BlackDetected;
      case ScanResult::Completed:      break;
    }

    /* b) 0° に戻る旋回 */
    switch (rotateToHeadingWithScan(0.0f, out_heading)) {
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

    /* d) 次サイクルに備えてヨー角リセット。
     *    直進中の左右モータ差で多少ヨーが動いても、
     *    次の amp_deg を「いまの向き」基準で振れるようにするため。 */
    gGyro.reset();
  }
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * 円の反対を読み取る動き:
 *   1. まっすぐ指定時間直進
 *   2. 右旋回 → 指定時間直進
 *   3. 右旋回 → 灰色 (グレーライン) を踏むまで直進
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult Nansho1Scene::readOppositeSide()
{
  gMotors.runFor(OppositeForwardPower, OppositeForwardPower,
                 OppositeForwardFirstMs);
  dly_tsk(StepMs * 1000U);

  turnToHeading(OppositeTurnRightDeg, OppositeTurnToleranceDeg,
                OppositeTurnPower, OppositeTurnPower);
  dly_tsk(StepMs * 1000U);
  gGyro.reset();
  gMotors.runFor(OppositeForwardPower, OppositeForwardPower,
                 OppositeForwardSecondMs);
  dly_tsk(StepMs * 1000U);

  turnToHeading(OppositeTurnRightDeg, OppositeTurnToleranceDeg,
                OppositeTurnPower, OppositeTurnPower);
  dly_tsk(StepMs * 1000U);
  gGyro.reset();
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
    turnToHeading(DetectFinishRightTurnDeg);
    driveForwardUntilBlackLine();
    runAfterFinalBlackLine();
    return;
  }

  const float h = gGyro.heading();

  if (h < DetectFinishClassicMaxDeg) {
    driveUntilBlackLine(0.0f + heading_offset_deg);
    runAfterFinalBlackLine();
    return;
  }

  /* 左走査・heading >= -56° */
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
  turnToHeading(target_heading, tolerance_deg, TurnPower, TurnPower);
}

void Nansho1Scene::turnToHeading(float target_heading, float tolerance_deg,
                                 int left_power, int right_power)
{
  uint32_t elapsed_ms = 0U;

  while (elapsed_ms < TurnMaxMs) {
    if (gGyro.reached(target_heading, tolerance_deg)) {
      break;
    }
    const float diff = gGyro.diffTo(target_heading);
    if (diff > 0.0f) {
      gMotors.run(left_power, -right_power);
    } else if (diff < 0.0f) {
      gMotors.run(-left_power, right_power);
    } else {
      gMotors.stop();
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
  gGyro.reset();
  gMotors.stop();
}

/* ---------------------------------------------------------------------
 * 開始時: ライントレース (スカイブルー → 黒) → 90° 旋回。
 * --------------------------------------------------------------------- */
void Nansho1Scene::runInitialForwardTrace()
{
  gGyro.reset();

  gLineTracer.setBaseSpeed(InitialTracePower, InitialTracePower);
  gLineTracer.setPid(0.30f, 0.01f, 0.8f, 100.0f);
  gLineTracer.setEdgeFollow(true);

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

  turnToHeading(InitialBlackTurnDeg);
}

/* ---------------------------------------------------------------------
 * 最終黒線読み後: 左 90° 旋回 → 青線までライントレース。
 * --------------------------------------------------------------------- */
void Nansho1Scene::runAfterFinalBlackLine()
{
  turnToHeading(gGyro.heading() - FinalBlueTraceTurnDeg, TurnToleranceDeg,
                FinalBlueTraceTurnLeftPower, FinalBlueTraceTurnRightPower);
  dly_tsk(StepMs * 1000U);

  gLineTracer.setBaseSpeed(FinalBlueTracePower, FinalBlueTracePower);
  gLineTracer.setPid(0.30f, 0.0005f, 0.45f, 100.0f);
  gLineTracer.setEdgeFollow(false);

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

  
  /* ① ライントレースで円の終端まで移動 (固定時間・減速あり)。 */
  {
    gLineTracer.setPid(0.6f, 0.0005f, 0.8f, 100.0f);
    gLineTracer.setEdgeFollow(false);

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
    gMotors.stop();
  }
  dly_tsk(StepMs * 1000U);

  gGyro.reset();
  /* ② 円の方向へ旋回 (右 90°)。 */
  turnToHeading(90.0f);
  dly_tsk(StepMs * 1000U);

  /* 旋回後の向きを 0° 基準にする。
   * これで wigwag の「右 90°」が、円に正対した向きを基準に振れる。 */
  gGyro.reset();

  /* 指定時間だけ前進してから走査を始める。 */
  gMotors.runFor(LeftPower, RightPower, ForwardBeforeScanMs);
  dly_tsk(StepMs * 1000U);

  /* ③ 右 90° 走査 (0°→+90°→0° を ScanCycles サイクル)。 */
  if (wigwagScan(+WigWagAmpDeg, ScanCycles, nullptr)
      == ScanResult::SonicDetected) {
    driveUntilGrayLine();
    finishAfterDetect(ScanSide::Right);
    return;
  }

  /* ④ 規定回数を検知なしで終了 → 円の反対を読み取る動き。 */
  readOppositeSide();

  /* ⑤ 左 90° 走査 (0°→-90°→0° を ScanCycles サイクル)。 */
  if (wigwagScan(-WigWagAmpDeg, ScanCycles, nullptr)
      == ScanResult::SonicDetected) {
    driveUntilGrayLine();
    finishAfterDetect(ScanSide::Left);
    return;
  }

  /* ⑥ 左走査も検知なしで終了 → 黒線を読む動きへ。 */
  driveUntilBlackLine();
  runAfterFinalBlackLine();
}

extern "C" void Nansho1Task(intptr_t exinf)
{
  (void)exinf;
  gNansho1.run();
  ext_tsk();
}
