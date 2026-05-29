#include "Nansho1.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

extern "C" {
#include <spike/hub/imu.h>
#include <spike/pup/ultrasonicsensor.h>
}

extern Sonic gSonic;
extern Color gColor;
extern Straight gMotors;

/* === ヘルパ ============================================================ *
 * 2 つの heading の差を [-180, 180] に折り畳む。
 * 連続値 (アンラップ済み) でも ±180 でラップする実装でも、
 * 1 ステップでの角度進捗を表すのに使える。
 * ====================================================================== */
static float wrapDeg180(float diff)
{
  while (diff > 180.0f)  diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

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
    const float   h    = hub_imu_get_heading();

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
    const float diff = wrapDeg180(target_heading - h);
    if (fabsf(diff) <= TurnToleranceDeg) {
      gMotors.stop();
      return ScanResult::Completed;
    }

    /* (3) 短い方向にピボット旋回
     *   diff > 0 → ヨー角を増やしたい → CW 旋回 (左 +, 右 -)
     *   diff < 0 → ヨー角を減らしたい → CCW 旋回 (左 -, 右 +) */
    if (diff > 0.0f) {
      gMotors.run(TurnPower, -TurnPower);
    } else {
      gMotors.run(-TurnPower, TurnPower);
    }

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
    const int32_t dist      = gSonic.readDistanceMm();
    const float   h         = hub_imu_get_heading();
    const bool    now_black = gColor.isBlack(true);

    if (dist > 0 && dist <= DetectMaxMm) {
      gMotors.stop();
      if (out_heading != nullptr) {
        *out_heading = h;
      }
      printf("WigWag forward HIT(sonic) dist=%d heading=%d\n",
             (int)dist, (int)(h * 10.0f));
      return ScanResult::SonicDetected;
    }

    if (wigwag_off_black_ && now_black) {
      gMotors.stop();
      printf("WigWag forward HIT(black) heading=%d\n", (int)(h * 10.0f));
      return ScanResult::BlackDetected;
    }
    if (!now_black) {
      wigwag_off_black_ = true;
    }

    gMotors.run(power, power);

    log_acc_ms += StepMs;
    if (log_acc_ms >= 500U) {
      printf("WigWag forward dist=%d heading=%d elapsed=%u black=%d offb=%d\n",
             (int)dist, (int)(h * 10.0f), (unsigned)elapsed_ms,
             now_black ? 1 : 0, wigwag_off_black_ ? 1 : 0);
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
 *   呼ぶ前に wigwag_off_black_ を false に初期化すること。
 * --------------------------------------------------------------------- */
bool Nansho1Scene::wigwagScan(float *out_heading)
{
  wigwag_off_black_ = false;  /* 開始は黒上にいる前提なのでクリア */

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
    hub_imu_reset_heading();
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
  float    prev_heading = hub_imu_get_heading();
  float    total_rot    = 0.0f;
  uint32_t elapsed_ms   = 0U;
  uint32_t log_acc_ms   = 0U;

  while (fabsf(total_rot) < 360.0f && elapsed_ms < ScanMaxMs) {
    gMotors.run(ScanTurnPower, -ScanTurnPower);

    const int32_t dist = gSonic.readDistanceMm();
    const float   h    = hub_imu_get_heading();

    if (dist > 0 && dist <= DetectMaxMm) {
      gMotors.stop();
      if (out_heading != nullptr) {
        *out_heading = h;
      }
      printf("Scan360 HIT dist=%d heading=%d\n",
             (int)dist, (int)(h * 10.0f));
      return true;
    }

    total_rot   += wrapDeg180(h - prev_heading);
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
    const float h    = hub_imu_get_heading();
    const float diff = wrapDeg180(target_heading - h);

    if (fabsf(diff) <= TurnToleranceDeg) {
      break;
    }

    if (diff > 0.0f) {
      gMotors.run(TurnPower, -TurnPower);
    } else {
      gMotors.run(-TurnPower, TurnPower);
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
}

/* ---------------------------------------------------------------------
 * シーン本体
 * --------------------------------------------------------------------- */
void Nansho1Scene::run()
{
  /* 0. センサ死活確認 (デバッグ用) */
  printf("Sonic configured=%d\n", gSonic.isConfigured() ? 1 : 0);
  for (int i = 0; i < 5; ++i) {
    const int32_t d = gSonic.readDistanceMm();
    printf("Sonic warmup dist=%d (mm)\n", (int)d);
    dly_tsk(100 * 1000);
  }

  /* 1. 黒線を踏むまで前進 */
  while (1) {
    if (gColor.isBlack(true)) {
      gMotors.stop();
      break;
    }
    gMotors.run(LeftPower, RightPower);
    dly_tsk(StepMs * 1000U);
  }

  /* 2. ヨー角リセット */
  hub_imu_reset_heading();

  /* 3. ジグザグスキャン: 超音波検知 か 黒線エッジ検知 のいずれかで終了 */
  float detected_heading = 0.0f;
  const bool detected_by_sonic = wigwagScan(&detected_heading);

  if (detected_by_sonic) {
    /* 4a. 超音波で検知 → 検知方向へ向き直って前進 */
    printf("Detected during wigwag (sonic), heading=%d (deci-deg)\n",
           (int)(detected_heading * 10.0f));
    turnToHeading(detected_heading);
    gMotors.runFor(ApproachPower, ApproachPower, ApproachTimeMs);
    return;
  }

  /* 4b. 黒線エッジで終了 (= ジグザグで物体が見つからず別の黒線まで来た)
   *     その場で 360° スキャン → 検知方向へ向き直って前進 */
  printf("WigWag terminated by black line. Start 360 scan.\n");
  gMotors.stop();
  dly_tsk(300 * 1000);
  hub_imu_reset_heading();

  float target_heading = 0.0f;
  const bool found = scan360(&target_heading);
  if (!found) {
    printf("Scan360: nothing detected. Idle.\n");
    return;
  }

  printf("Scan360: detected at heading=%d (deci-deg)\n",
         (int)(target_heading * 10.0f));
  turnToHeading(target_heading);
  gMotors.runFor(ApproachPower, ApproachPower, ApproachTimeMs);
}

extern Nansho1Scene gNansho1;

extern "C" void Nansho1Task(intptr_t exinf)
{
  (void)exinf;
  gNansho1.run();
  ext_tsk();
}
