#include "Nansho1.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "Common/Straight/Straight.h"

extern Sonic gSonic;
extern ColorRGB gColorRGB;
extern ColorHSV gColorHSV;
extern Gyro gGyro;
extern Straight gMotors;
extern Nansho1Scene gNansho1;

/* ---------------------------------------------------------------------
 * 旋回中スキャン: target_heading までピボット旋回しつつ
 *   - 超音波で DetectMaxMm 以下を検知           → SonicDetected
 *   - 目標角に到達 (|diff| <= TurnToleranceDeg) → Completed
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
      printf("WigWag rotate HIT(sonic) dist=%d heading=%d\n",
             (int)dist, (int)(h * 10.0f));
      return ScanResult::SonicDetected;
    }

    /* (2) 目標角度に到達? */
    const float diff = gGyro.diffTo(target_heading);
    if (gGyro.reached(target_heading, TurnToleranceDeg)) {
      gMotors.stop();
      return ScanResult::Completed;
    }

    /* (3) 短い方向にピボット旋回
     *   diff > 0 → ヨー角を増やしたい → CW
     *   diff < 0 → ヨー角を減らしたい → CCW */
    gMotors.pivotTowardDiff(TurnPower, diff);

    log_acc_ms += StepMs;
    if (log_acc_ms >= 500U) {
      printf("WigWag rotate target=%d heading=%d dist=%d\n",
             (int)(target_heading * 10.0f), (int)(h * 10.0f), (int)dist);
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
 *   - 黒線エッジ   → BlackDetected
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
      printf("WigWag forward HIT(sonic) dist=%d heading=%d\n",
             (int)dist, (int)(h * 10.0f));
      return ScanResult::SonicDetected;
    }

    /* 黒線エッジ判定は Color クラスに移譲。
     * 「いったん非黒を踏んでから再び黒を踏んだ瞬間」だけ true。 */
    if (gColorHSV.blackEdgeHit(true)) {
      gMotors.stop();
      printf("WigWag forward HIT(black) heading=%d\n", (int)(h * 10.0f));
      return ScanResult::BlackDetected;
    }

    gMotors.run(power, power);

    log_acc_ms += StepMs;
    if (log_acc_ms >= 500U) {
      printf("WigWag forward dist=%d heading=%d elapsed=%u\n",
             (int)dist, (int)(h * 10.0f), (unsigned)elapsed_ms);
      log_acc_ms = 0U;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  return ScanResult::Completed;
}

/* ---------------------------------------------------------------------
 * ジグザグ確認スキャン (回数制限なし、黒線エッジ or 超音波検知で終了)
 *   各サイクル:
 *     a) +WigWagAmpDeg まで旋回 (検知監視)
 *     b) -WigWagAmpDeg まで旋回 (検知監視)
 *     c)  0° に戻る旋回         (検知監視)
 *     d) WigWagForwardMs だけ前進 (検知監視)
 *     e) 次サイクルに備えてヨー角リセット
 *
 *   返り値:
 *     true  = 超音波検知 (out_heading に検知時ヨー角)
 *     false = 黒線エッジ検知 (= 黒線を再び踏んだ)
 *   関数の入口で gColorHSV.resetBlackEdge() を呼ぶので、呼び側での初期化は不要。
 * --------------------------------------------------------------------- */
bool Nansho1Scene::wigwagScan(float *out_heading)
{
  gColorHSV.resetBlackEdge();  /* 開始は黒上にいる前提なのでクリア */

  int cycle = 0;
  while (true) {
    ++cycle;

    /* a) +WigWagAmpDeg まで旋回 */
    printf("WigWag cycle %d: -> +%d deg\n", cycle, (int)WigWagAmpDeg);
    switch (rotateToHeadingWithScan(+WigWagAmpDeg, out_heading)) {
      case ScanResult::SonicDetected:  return true;
      case ScanResult::BlackDetected:  return false;
      case ScanResult::Completed:      break;
    }

    /* b) -WigWagAmpDeg まで旋回 */
    printf("WigWag cycle %d: -> -%d deg\n", cycle, (int)WigWagAmpDeg);
    switch (rotateToHeadingWithScan(-WigWagAmpDeg, out_heading)) {
      case ScanResult::SonicDetected:  return true;
      case ScanResult::BlackDetected:  return false;
      case ScanResult::Completed:      break;
    }

    /* c) 0° に戻る旋回 */
    printf("WigWag cycle %d: -> 0 deg\n", cycle);
    switch (rotateToHeadingWithScan(0.0f, out_heading)) {
      case ScanResult::SonicDetected:  return true;
      case ScanResult::BlackDetected:  return false;
      case ScanResult::Completed:      break;
    }

    /* d) 少し前進 */
    printf("WigWag cycle %d: forward %u ms\n",
           cycle, (unsigned)WigWagForwardMs);
    switch (driveForwardWithScan(WigWagForwardMs, WigWagForwardPower, out_heading)) {
      case ScanResult::SonicDetected:  return true;
      case ScanResult::BlackDetected:  return false;
      case ScanResult::Completed:      break;
    }

    /* e) 次サイクルに備えてヨー角リセット。
     *    直進中の左右モータ差で多少ヨーが動いても、
     *    次の ±WigWagAmpDeg を「いまの向き」基準で振れるようにするため。 */
    gGyro.reset();
  }
}

/* ---------------------------------------------------------------------
 * その場 360° スキャン (即時停止式)
 *   ピボット旋回 (左右逆符号同パワー) で回りながら超音波を読み続け、
 *   DetectMaxMm 以下を検知した瞬間にモータを止めて、その時のヨー角を
 *   out_heading に入れて true を返す。
 *   1 周回り切っても検知できなければ false。
 * --------------------------------------------------------------------- */
bool Nansho1Scene::scan360(float *out_heading)
{
  float    prev_heading = gGyro.heading();
  float    total_rot    = 0.0f;
  uint32_t elapsed_ms   = 0U;
  uint32_t log_acc_ms   = 0U;

  /* 累積角の絶対値で判定 (機体差で IMU の符号が逆でも 1 周で抜けるように)。 */
  while (fabsf(total_rot) < 360.0f && elapsed_ms < ScanMaxMs) {
    gMotors.pivotCW(ScanTurnPower);

    const int32_t dist = gSonic.readDistanceMm();
    const float   h    = gGyro.heading();

    if (dist > 0 && dist <= DetectMaxMm) {
      gMotors.stop();
      if (out_heading != nullptr) {
        *out_heading = h;
      }
      printf("Scan360 HIT dist=%d heading=%d\n",
             (int)dist, (int)(h * 10.0f));
      return true;
    }

    /* CW 回転前提なのでヨー進捗は正の側に積む (符号付き diff の絶対値を加算)。 */
    total_rot   += Gyro::wrapDeg180(h - prev_heading);
    prev_heading = h;

    log_acc_ms += StepMs;
    if (log_acc_ms >= 500U) {
      printf("Scan360 dist=%d heading=%d total_rot=%d\n",
             (int)dist, (int)(h * 10.0f), (int)(total_rot * 10.0f));
      log_acc_ms = 0U;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  if (out_heading != nullptr) {
    *out_heading = 0.0f;
  }
  printf("Scan360 done. no detection.\n");
  return false;
}

/* ---------------------------------------------------------------------
 * 目標ヨー角まで短い側にピボット旋回 (検知判定なし)
 * --------------------------------------------------------------------- */
void Nansho1Scene::turnToHeading(float target_heading)
{
  uint32_t elapsed_ms = 0U;

  while (elapsed_ms < TurnMaxMs) {
    if (gGyro.reached(target_heading, TurnToleranceDeg)) {
      break;
    }
    gMotors.pivotTowardDiff(TurnPower, gGyro.diffTo(target_heading));

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
}

/* ---------------------------------------------------------------------
 * シーン共通の終了動作: 検知方向を向いて一定時間前進する。
 * --------------------------------------------------------------------- */
void Nansho1Scene::approachHeading(float heading)
{
  turnToHeading(heading);
  gMotors.runFor(ApproachPower, ApproachPower, ApproachTimeMs);
}

/* ---------------------------------------------------------------------
 * シーン本体
 * --------------------------------------------------------------------- */
void Nansho1Scene::run()
{

  gGyro.reset();
  dly_tsk(StepMs * 1000U);
  gMotors.runFor((-1*LeftPower), (-1*RightPower), 4000U);
  dly_tsk(StepMs * 1000U);
  /* -90°へ旋回するだけ。ここで前進すると手前の線を通り越して
   * 円の向こう側の線を踏んでしまうため、前進は次のライン検出ループに任せる。 */
  turnToHeading(-90.0f);
  dly_tsk(StepMs * 1000U);


  /* 1. 黒線を踏むまで前進 */
  while (1) {
    if (gColorRGB.isGrayLine()) {
      gMotors.stop();
      break;
    }
    gMotors.run(LeftPower, RightPower);
    dly_tsk(StepMs * 1000U);
  }

  /* 2. ヨー角リセット */
  gGyro.reset();

  /* 3. ジグザグスキャン: 超音波検知 か 黒線エッジ検知 のいずれかで終了 */
  float detected_heading = 0.0f;
  const bool detected_by_sonic = wigwagScan(&detected_heading);

  if (detected_by_sonic) {
    /* 4a. 超音波で検知 → 検知方向へ向き直って前進 */
    printf("Detected during wigwag (sonic), heading=%d (deci-deg)\n",
           (int)(detected_heading * 10.0f));
    approachHeading(detected_heading);
    return;
  }

  /* 4b. 黒線エッジで終了 (= ジグザグで物体が見つからず別の黒線まで来た)
   *     その場で 360° スキャン → 検知方向へ向き直って前進 */
  printf("WigWag terminated by black line. Start 360 scan.\n");
  gMotors.stop();
  dly_tsk(300 * 1000);
  gGyro.reset();

  float target_heading = 0.0f;
  const bool found = scan360(&target_heading);
  if (!found) {
    printf("Scan360: nothing detected. Idle.\n");
    return;
  }

  printf("Scan360: detected at heading=%d (deci-deg)\n",
         (int)(target_heading * 10.0f));
  approachHeading(target_heading);
}

extern "C" void Nansho1Task(intptr_t exinf)
{
  (void)exinf;
  gNansho1.run();
  ext_tsk();
}
