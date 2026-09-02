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
        if (out_heading != nullptr) {
          *out_heading = gGyro.heading();
        }
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
    if (out_heading != nullptr) {
      *out_heading = gGyro.heading();
    }
    return ScanResult::SonicDetected;
  }
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * 円向き後直進 / wigwag 直進の共通開始・終了。
 * ピボット直後に STRAIGHT_BALANCE が古い指令を書き戻すのを防ぐため、
 * 直進中は周期補正タスクを止め、ここからの tickCorrection だけにする。
 * --------------------------------------------------------------------- */
void Nansho1Scene::beginBalancedStraight(int left_power, int right_power,
                                         float target_heading)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  /* ピボット等の残留指令・補正状態を捨てる */
  gMotors.stop();
  dly_tsk(StepMs * 1000U);
  if (UseHeadingCorrection) {
    gMotors.setBalancePid(PreWigwagBalanceKp, PreWigwagBalanceKi,
                          PreWigwagBalanceIntegralMax);
    gMotors.setTargetHeading(target_heading);
  }
  gMotors.run(left_power, right_power);
}

void Nansho1Scene::endBalancedStraight()
{
  gMotors.stop();
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);
}

/* ---------------------------------------------------------------------
 * 直進中スキャン: 指定時間 power で前進しつつ
 *   - 超音波距離 <= DetectMaxMm → SonicDetected
 *   - 黒線           → BlackDetected
 *   - 時間切れ         → Completed
 * ヨー保持は beginBalancedStraight と同じ。
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult
Nansho1Scene::driveForwardWithScan(uint32_t duration_ms, int power,
                                   float *out_heading, float target_heading)
{
  uint32_t elapsed_ms = 0U;

  beginBalancedStraight(power, power, target_heading);

  while (elapsed_ms < duration_ms) {
    const float h = gGyro.heading();
    if (UseHeadingCorrection) {
      gMotors.tickCorrection();
    }

    if (gSonic.isCloserThan(DetectMaxMm)) {
      endBalancedStraight();
      if (out_heading != nullptr) {
        *out_heading = h;
      }
      return ScanResult::SonicDetected;
    }

    if (gColorLAB.is(COLOR_DETECT_BLACK, true)) {
      endBalancedStraight();
      return ScanResult::BlackDetected;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  endBalancedStraight();
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * 旋回走査 (片側振り・規定回数で打ち切り)
 *   各サイクル:
 *     a) 正面から amp_deg だけ側方を向く
 *     b) 正面へ向き直す
 *     c) WigWagForwardMs だけ前進
 *
 *   右走査: 正面・直進目標は円向き reset 後 0°
 *   左走査: 正面・直進目標は反対読み reset 後 0°
 *
 *   返り値 (ScanResult):
 *     SonicDetected  = 超音波検知 (out_heading に検知時ヨー角)
 *     BlackDetected  = 前進中に黒線を踏んだ
 *     Completed      = max_cycles サイクルを検知なしで完了
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult
Nansho1Scene::wigwagScan(float amp_deg, int max_cycles, float *out_heading,
                         ScanSide scan_side, float front_heading)
{
  (void)scan_side;
  const float front = front_heading;

  for (int cycle = 1; cycle <= max_cycles; ++cycle) {

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

    /* c) 少し前進（正面向きを維持） */
    dly_tsk(StepMs * 1000U);
    switch (driveForwardWithScan(WigWagForwardMs, WigWagForwardPower,
                                 out_heading, front)) {
      case ScanResult::SonicDetected:  return ScanResult::SonicDetected;
      case ScanResult::BlackDetected:  return ScanResult::BlackDetected;
      case ScanResult::Completed:      break;
    }
  }
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * ③〜⑤ wig走査フェーズ。
 *   最初の走査は LR 共通で Right（黒線向きは分岐なし 180°）。
 *   振り角の符号だけ app.h の ETROBO_COURSE_L で反転する。
 * --------------------------------------------------------------------- */
bool Nansho1Scene::runWigwagScanPhases()
{
  const float first_amp = ETROBO_COURSE_L ? WigWagAmpDeg : -WigWagAmpDeg;
  const float second_amp = ETROBO_COURSE_L ? -WigWagAmpDeg : WigWagAmpDeg;
  const ScanSide first_side = ScanSide::Right;
  const ScanSide second_side = ScanSide::Left;

  float detect_heading = gGyro.heading();
  if (wigwagScan(first_amp, ScanCycles, &detect_heading, first_side) ==
      ScanResult::SonicDetected) {
    /* 検知した向きへ（wigwag 中はジャイロ reset しない） */
    turnToHeading(detect_heading, TurnToleranceDeg, TurnPower, TurnPower);
    gMotors.setTargetHeading(detect_heading);
    driveUntilGrayLine(DetectGrayIgnoreMs);
    finishAfterDetect(first_side);
    return true;
  }

  readOppositeSide();

  /* 反対読み後に reset したため、2 回目走査の正面は 0° */
  const float second_front = 0.0f;
  detect_heading = second_front;
  if (wigwagScan(second_amp, ScanCycles, &detect_heading, second_side,
                 second_front) == ScanResult::SonicDetected) {
    turnToHeading(detect_heading, TurnToleranceDeg, TurnPower, TurnPower);
    gMotors.setTargetHeading(detect_heading);
    driveUntilGrayLine(DetectGrayIgnoreMs);
    /* 反対側 reset 後: 黒線方向は 0° */
    finishAfterDetect(second_side, second_front);
    return true;
  }

  return false;
}

/* ---------------------------------------------------------------------
 * 円の反対を読み取る動き (各セグメント開始で reset、旋回は 0° 基準):
 *   1. reset → 指定時間直進
 *   2. ±90° 旋回 → reset
 *   3. 指定時間直進
 *   4. ±90° 旋回 → reset → 灰色まで直進 → reset
 * --------------------------------------------------------------------- */
Nansho1Scene::ScanResult Nansho1Scene::readOppositeSide()
{
  gGyro.reset();
  gMotors.setTargetHeading(0.0f);
  gMotors.runForMs(OppositeForwardPower, OppositeForwardPower,
                   OppositeForwardFirstMs);
  dly_tsk(StepMs * 1000U);

  turnToHeading(OppositeFirstTurnDeg, OppositeTurnToleranceDeg,
                OppositeTurnPower, OppositeTurnPower);
  dly_tsk(StepMs * 1000U);
  gGyro.reset();

  gMotors.setTargetHeading(0.0f);
  gMotors.runForMs(OppositeForwardPower, OppositeForwardPower,
                   OppositeForwardSecondMs);
  dly_tsk(StepMs * 1000U);

  turnToHeading(OppositeSecondTurnDeg, OppositeTurnToleranceDeg,
                OppositeTurnPower, OppositeTurnPower);
  dly_tsk(StepMs * 1000U);
  gGyro.reset();

  gMotors.setTargetHeading(0.0f);
  driveUntilGrayLine(0U, OppositeForwardPower, OppositeForwardPower,
                     ScanMaxMs);
  gGyro.reset();

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
  gGyro.reset();

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
                                      int left_power, int right_power,
                                      uint32_t max_ms)
{
  uint32_t elapsed_ms = 0U;

  while (1) {
    if (UseHeadingCorrection) {
      gMotors.tickCorrection();
    }

    if (elapsed_ms >= gray_ignore_ms &&
        gColorRGB.is(COLOR_DETECT_GRAYLINE)) {
      gMotors.stop();
      break;
    }
    if (max_ms > 0U && elapsed_ms >= max_ms) {
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
 *   Right: 円側 0° 基準 180° → 黒線まで直進
 *   Left (反対側 reset 後):
 *     浅い角 → 0° → 黒線まで直進
 *     広い角 → ±90° → 直進 → 0° → 黒線まで直進
 *   heading_offset_deg: 2 回目走査の正面 (反対読み reset 後 0°)。
 * --------------------------------------------------------------------- */
void Nansho1Scene::finishAfterDetect(ScanSide side, float heading_offset_deg)
{
  dly_tsk(DetectFinishBackDelayMs * 1000U);
  gMotors.runForMs(-DetectFinishBackPower, -DetectFinishBackPower,
                   DetectFinishBackMs);
  dly_tsk(StepMs * 1000U);

  if (side == ScanSide::Right) {
    /* バック後も reset せず、円向き 0° 基準で 180°（黒線方向）を向く */
    turnToHeading(DetectFinishRightTurnDeg);
    dly_tsk(StepMs * 1000U);
    gGyro.reset();
    driveForwardUntilBlackLine();
    runAfterFinalBlackLine();
    return;
  }

  /* 左走査: 反対読み reset 後 0° 基準の相対角で分岐 */
  const float h_rel =
      Gyro::wrapDeg180(gGyro.heading() - heading_offset_deg);

  /* 広い: |相対角| が大きい側 → 反対側 ±90° ルート
   * 浅い: |相対角| が小さい側 → 反対側 0° (= 黒線)
   * L: h_rel < -40 が広い / h_rel >= -40 が浅い */
  if ((float)DetectFinishAngleSign * h_rel <
      (float)DetectFinishAngleSign * DetectFinishClassicMaxDeg) {
    /* 広い検知角: 反対側 ±90° → 直進 → 反対側 0° → 黒線まで直進 */
    turnToHeading(DetectWideTurnFirstDeg);
    dly_tsk(StepMs * 1000U);
    gMotors.runForMs(LeftPower, RightPower, DetectMidForwardMs);
    dly_tsk(StepMs * 1000U);

    turnToHeading(DetectWideTurnSecondDeg);
    dly_tsk(StepMs * 1000U);
    gGyro.reset();
    driveForwardUntilBlackLine();
    runAfterFinalBlackLine();
    return;
  }

  turnToHeading(DetectFinishLeftTurnDeg);
  dly_tsk(StepMs * 1000U);
  gGyro.reset();
  driveForwardUntilBlackLine();
  runAfterFinalBlackLine();
}
/* ---------------------------------------------------------------------
 * 目標ヨー角まで短い側にピボット旋回 (検知判定なし)。
 * ジャイロ reset は行わない。
 * --------------------------------------------------------------------- */
void Nansho1Scene::turnToHeading(float target_heading, float tolerance_deg)
{
  turnToHeading(target_heading, tolerance_deg, TurnPower, TurnPower);
}

void Nansho1Scene::turnToHeading(float target_heading, float tolerance_deg,
                                 int left_power, int right_power)
{
  /* 開始時点の最短経路だけを、経過角で打ち切る。
   * heading の ±180 折り返しでも local_target 差が壊れないようにする。 */
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
}

/* ---------------------------------------------------------------------
 * 開始時: ジャイロ初期化 → InitialBlackTurnDeg 旋回。
 * （開始 LT は省略。試合配置で向きを合わせてから起動する想定）
 * --------------------------------------------------------------------- */
void Nansho1Scene::runInitialForwardTrace()
{
  gGyro.reset();
  gLineTracer.setEnabled(false);
  turnToHeading(InitialBlackTurnDeg);
  dly_tsk(StepMs * 1000U);
  gGyro.reset();
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
 * 最終黒線読み後: 指定方向に 90° 旋回 → 低速LT → 青線までライントレース。
 * --------------------------------------------------------------------- */
void Nansho1Scene::runAfterFinalBlackLine()
{
  turnAfterFinalBlackLine();
  dly_tsk(StepMs * 1000U);

  gLineTracer.setEnabled(true);
  gLineTracer.setEdgeFollow(ETROBO_COURSE_L == 0);

  /* 旋回直後は低速で線に乗せる */
  gLineTracer.setBaseSpeed(FinalSlowTracePower, FinalSlowTracePower);
  gLineTracer.setPid(FinalSlowTraceKp, FinalSlowTraceKi, FinalSlowTraceKd,
                     FinalSlowTraceOutMax);
  gLineTracer.setScale(FinalSlowTraceScale);
  for (uint32_t elapsed_ms = 0U; elapsed_ms < FinalSlowTraceMs;
       elapsed_ms += StepMs) {
    gLineTracer.step();
    dly_tsk(StepMs * 1000U);
  }

  /* 本線 LT で青まで */
  gLineTracer.setBaseSpeed(FinalBlueTracePower, FinalBlueTracePower);
  gLineTracer.setPid(FinalBlueTraceKp, FinalBlueTraceKi,
                     FinalBlueTraceKd, FinalBlueTraceOutMax);
  gLineTracer.setScale(FinalBlueTraceScale);

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
  /* HCR から引き継いだ周期 LT が残っていても止める */
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();
  gMotors.setHeadingCorrection(UseHeadingCorrection);

  /* ⓪ 開始時: InitialBlackTurnDeg 旋回。 */
  runInitialForwardTrace();

  /* 開始時: ジャイロを初期化する。 */
  gGyro.reset();

  
  /* ① 旋回後: 低速 LT → 本線 LT で円の終端まで移動 (固定時間・減速あり)。 */
  {
    gLineTracer.setEnabled(true);
    gLineTracer.setPid(BackTraceKp, BackTraceKi, BackTraceKd, BackTraceOutMax);
    gLineTracer.setScale(BackTraceScale);
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

  /* ② 円の方向へ旋回。reset は旋回後に明示。 */
  turnToHeading(PreWigwagTurnDeg);
  dly_tsk(StepMs * 1000U);
  gGyro.reset();
  /* 指定時間だけ前進してから走査を始める（wigwag 直進と同じ共通処理）。 */
  beginBalancedStraight(LeftPower, RightPower, 0.0f);
  for (uint32_t elapsed_ms = 0U; elapsed_ms < ForwardBeforeScanMs;
       elapsed_ms += StepMs) {
    if (UseHeadingCorrection) {
      gMotors.tickCorrection();
    }
    dly_tsk(StepMs * 1000U);
  }
  endBalancedStraight();
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
  /* Nansho2 以降は既定どおりヨー角 PI 補正を使う */
  gMotors.setHeadingCorrection(true);

#if ETROBO_SCENE == 0 || ETROBO_SCENE == 5 || ETROBO_SCENE == 6
  gMotors.stop();
  gLineTracer.setEnabled(false);
  dly_tsk(50 * 1000U);
  act_tsk(NANSHO2_TASK);
#endif

  ext_tsk();
}
