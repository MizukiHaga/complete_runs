#include "Test.h"

#include "app.h"
#include "kernel_cfg.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "ColorLAB.h"
#include "Common/LineTracer/LineTracer.h"
#include "Common/Straight/Straight.h"
#include "Gyro.h"
#include "HeadingFusionLog.h"
#include "HeadingFusionSettings.h"
#include "Scene/Nansho3/Nansho3.h"

#include <serial/newlib.h>
#include <serial/serial.h>
#include <syssvc/serial.h>
#include <spike/hub/bluetooth.h>
#include <pbio/error.h>

extern ColorLAB   gColorLAB;
extern Gyro       gGyro;
extern Straight   gMotors;
extern LineTracer gLineTracer;
extern TestScene  gTest;
extern Nansho3Scene gNansho3;

/* LogTest と同じ BLE CSV。Viewcmd は SIO_BLUETOOTH を読む */
static FILE *s_bt_out = nullptr;

static void openBtLog()
{
  if (s_bt_out != nullptr) {
    return;
  }
  serial_opn_por(SIO_BLUETOOTH_PORTID);
  s_bt_out = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);
}

static bool btLogWritable()
{
  if (s_bt_out == nullptr) {
    return false;
  }
  bool connected = false;
  return hub_bluetooth_is_connected(&connected) == PBIO_SUCCESS && connected;
}

static int32_t absEncoderDelta(int32_t now, int32_t base)
{
  const int32_t d = now - base;
  return (d < 0) ? -d : d;
}

/* S字: s∈[0,1] → smoothstep（両端で傾き 0） */
static float smoothstep01(float s)
{
  if (s <= 0.0f) {
    return 0.0f;
  }
  if (s >= 1.0f) {
    return 1.0f;
  }
  return s * s * (3.0f - 2.0f * s);
}

static int lerpPowerS(int from_power, int to_power, float s01)
{
  const float u = smoothstep01(s01);
  const float p =
      static_cast<float>(from_power) +
      static_cast<float>(to_power - from_power) * u;
  return static_cast<int>((p >= 0.0f) ? (p + 0.5f) : (p - 0.5f));
}

static void emitTurnEncodersOnce(int32_t left_abs_deg, int32_t right_abs_deg,
                                 float heading_deg, bool left,
                                 float enc_ang_deg, float ang_err_deg)
{
  const char *dir = left ? "L" : "R";
  if (btLogWritable()) {
    fprintf(s_bt_out, "MOT_L,%d,MOT_R,%d,MOT_Lp,%d,MOT_Rp,%d\n",
            (int)left_abs_deg, (int)right_abs_deg, 0, 0);
    fprintf(s_bt_out, "IMU_h,%.2f\n", (double)heading_deg);
    fprintf(s_bt_out, "ENC_ang,%.2f,ANG_err,%.2f\n",
            (double)enc_ang_deg, (double)ang_err_deg);
    fprintf(s_bt_out, "DIR,%s\n", dir);
    fflush(s_bt_out);
  }
  printf("MOT_L,%d,MOT_R,%d,MOT_Lp,%d,MOT_Rp,%d\n",
         (int)left_abs_deg, (int)right_abs_deg, 0, 0);
  printf("IMU_h,%.2f\n", (double)heading_deg);
  printf("ENC_ang,%.2f,ANG_err,%.2f\n",
         (double)enc_ang_deg, (double)ang_err_deg);
  printf("DIR,%s\n", dir);
}

void TestScene::logStraightEncoders(int32_t left_base, int32_t right_base,
                                    const char *phase)
{
  if (!EnableLog) {
    return;
  }
  openBtLog();
  const int32_t l =
      absEncoderDelta(gMotors.leftEncoderCount(), left_base);
  const int32_t r =
      absEncoderDelta(gMotors.rightEncoderCount(), right_base);
  const int32_t avg = (l + r) / 2;
  const char *ph = (phase != nullptr) ? phase : "run";
  if (btLogWritable()) {
    fprintf(s_bt_out, "MOT_L,%d,MOT_R,%d,ENC_avg,%d,PHASE,%s\n",
            (int)l, (int)r, (int)avg, ph);
    fflush(s_bt_out);
  }
  printf("MOT_L,%d,MOT_R,%d,ENC_avg,%d,PHASE,%s\n",
         (int)l, (int)r, (int)avg, ph);
  const float raw = gGyro.rawHeading();
  const float heading = gGyro.heading();
  const float target = gMotors.targetHeading();
  const HeadingFusion::State state = gGyro.fusionState();
  const auto write = [&](FILE *output) {
    if (output == nullptr) return;
    fprintf(output,
        "SQUARE_DRIVE,PHASE,%s,TGT,%.3f,IMU_raw,%.3f,KF_h,%.3f,ERR,%.3f,"
        "LAST_L,%d,LAST_R,%d\n",
        ph, (double)target, (double)raw, (double)heading,
        (double)Gyro::wrapDeg180(heading - target),
        gMotors.lastAppliedLeftPower(), gMotors.lastAppliedRightPower());
    writeHeadingFusionLog(output, state, heading);
    fflush(output);
  };
  write(stdout);
  if (btLogWritable()) write(s_bt_out);

}

void TestScene::waitLogConnected()
{
  if (!EnableLog) {
    return;
  }
  openBtLog();
  while (1) {
    bool connected = false;
    if (hub_bluetooth_is_connected(&connected) == PBIO_SUCCESS && connected) {
      break;
    }
    dly_tsk(50 * 1000U);
  }
  if (btLogWritable()) {
    fprintf(s_bt_out, "LOG_READY\n");
    fflush(s_bt_out);
  }
  printf("LOG_READY\n");
}

void TestScene::logStopInstant()
{
  if (!EnableLog) {
    return;
  }
  openBtLog();
  writeTurnEncoderLog(stdout, gMotors.turnEncoderSnapshot());
  if (btLogWritable()) {
    writeTurnEncoderLog(s_bt_out, gMotors.turnEncoderSnapshot());
    fflush(s_bt_out);
  }
  const int32_t left_abs = gMotors.lastTurnEncLeftAbsDeg();
  const int32_t right_abs = gMotors.lastTurnEncRightAbsDeg();
  const float enc_avg =
      0.5f * static_cast<float>(left_abs + right_abs);
  const float per90 = Nansho3SpinEncoderPer90Deg;
  const float enc_ang =
      (per90 > 0.0f) ? (enc_avg * 90.0f / per90) : 0.0f;
  const char *format =
      "STOP_ms,%lu,IMU_acc,%.2f,ENC_ang,%.2f,IMU_corrected,%.2f\n";
  printf(format, (unsigned long)gMotors.lastTurnElapsedMs(),
         (double)gMotors.lastTurnGyroAccDeg(), (double)enc_ang,
         (double)gMotors.lastTurnHeadingDeg());
  if (btLogWritable()) {
    fprintf(s_bt_out, format, (unsigned long)gMotors.lastTurnElapsedMs(),
            (double)gMotors.lastTurnGyroAccDeg(), (double)enc_ang,
            (double)gMotors.lastTurnHeadingDeg());
    fflush(s_bt_out);
  }
}

void TestScene::logAfterTurn(int32_t left_abs_deg, int32_t right_abs_deg,
                             float heading_rel_deg, bool left)
{
  if (!EnableLog) {
    return;
  }
  openBtLog();
  logStopInstant();
  /* 左右平均を 90°=Nansho3SpinEncoderPer90Deg で角度にし、90° との差を出す。
   * ANG_err > 0 はエンコーダ上 90° より多く回った。 */
  const float enc_avg =
      0.5f * static_cast<float>(left_abs_deg + right_abs_deg);
  const float per90 = Nansho3SpinEncoderPer90Deg;
  const float enc_ang =
      (per90 > 0.0f) ? (enc_avg * 90.0f / per90) : 0.0f;
  const float ang_err = enc_ang - 90.0f;
  emitTurnEncodersOnce(left_abs_deg, right_abs_deg, heading_rel_deg, left,
                       enc_ang, ang_err);
  // IMU_h は補正前の相対角を維持。補正量を別列にして効果を検証する。
  if (gGyro.fusionEnabled()) {
    const HeadingFusion::State state = gGyro.fusionState();
    const float heading = gGyro.heading();
    writeHeadingFusionLog(stdout, state, heading);
    if (btLogWritable()) {
      writeHeadingFusionLog(s_bt_out, state, heading);
      fflush(s_bt_out);
    }
    return;
  }
  const float correction = gMotors.turnDriftCorrectionDeg();
  const float rate = gMotors.turnDriftRate();
  const TurnDrift &diag = gMotors.turnDriftDiagnostics();
  // 旋回終了時点の累積角度差と、補正後に残った差。
  const char *format = "DRIFT_gain,%.3f,DRIFT_err,%.3f,DRIFT_rem,%.3f,DRIFT_status,%d,DRIFT_ok,%lu,DRIFT_warm,%lu,DRIFT_invalid,%lu,DRIFT_delay,%lu,DRIFT_wheel,%lu,DRIFT_dir,%lu,DRIFT_slow,%lu,DRIFT_rateReject,%lu,DRIFT_angleReject,%lu\n";
  printf(format, (double)diag.gain(), (double)diag.angleError(), (double)diag.residual(), (int)diag.status(),
         (unsigned long)diag.count(TurnDrift::Accepted), (unsigned long)diag.count(TurnDrift::WarmingUp),
         (unsigned long)diag.count(TurnDrift::Invalid), (unsigned long)diag.count(TurnDrift::Delayed),
         (unsigned long)diag.count(TurnDrift::WheelMismatch), (unsigned long)diag.count(TurnDrift::WrongDirection),
         (unsigned long)diag.count(TurnDrift::TooSlow), (unsigned long)diag.count(TurnDrift::RateMismatch),
         (unsigned long)diag.count(TurnDrift::AngleMismatch));
  if (btLogWritable()) {
    fprintf(s_bt_out, format, (double)diag.gain(), (double)diag.angleError(), (double)diag.residual(), (int)diag.status(),
            (unsigned long)diag.count(TurnDrift::Accepted), (unsigned long)diag.count(TurnDrift::WarmingUp),
            (unsigned long)diag.count(TurnDrift::Invalid), (unsigned long)diag.count(TurnDrift::Delayed),
            (unsigned long)diag.count(TurnDrift::WheelMismatch), (unsigned long)diag.count(TurnDrift::WrongDirection),
            (unsigned long)diag.count(TurnDrift::TooSlow), (unsigned long)diag.count(TurnDrift::RateMismatch),
            (unsigned long)diag.count(TurnDrift::AngleMismatch));
  }
  printf("DRIFT_corr,%.3f,DRIFT_rate,%.3f,IMU_corrected,%.3f\n",
         (double)correction, (double)rate, (double)gGyro.heading());
  if (btLogWritable()) {
    fprintf(s_bt_out, "DRIFT_corr,%.3f,DRIFT_rate,%.3f,IMU_corrected,%.3f\n",
            (double)correction, (double)rate, (double)gGyro.heading());
    fflush(s_bt_out);
  }
}

void TestScene::cruiseUntilDeg(int32_t left_base, int32_t right_base,
                               int32_t target_deg)
{
  uint32_t elapsed_ms = 0U;
  while (gMotors.traveledDegFrom(left_base, right_base) < target_deg) {
    if (StraightEncLogIntervalMs > 0U &&
        (elapsed_ms % StraightEncLogIntervalMs) == 0U) {
      logStraightEncoders(left_base, right_base, "cruise");
    }
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
}

void TestScene::rampStraightPower(int32_t left_base, int32_t right_base,
                                  int32_t dist_deg, int from_power,
                                  int to_power)
{
  if (from_power == to_power || dist_deg <= 0) {
    gMotors.run(to_power, to_power);
    return;
  }
  uint32_t elapsed_ms = 0U;
  while (gMotors.traveledDegFrom(left_base, right_base) < dist_deg) {
    const int32_t d = gMotors.traveledDegFrom(left_base, right_base);
    const float s =
        static_cast<float>(d) / static_cast<float>(dist_deg);
    const int p = lerpPowerS(from_power, to_power, s);
    gMotors.run(p, p);
    if (StraightEncLogIntervalMs > 0U &&
        (elapsed_ms % StraightEncLogIntervalMs) == 0U) {
      logStraightEncoders(left_base, right_base, "accel");
    }
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
  gMotors.run(to_power, to_power);
}

void TestScene::accelToCruise(int32_t left_base, int32_t right_base,
                              int32_t accel_deg, int cruise_power,
                              int start_power)
{
  /* 直進中は 1D レート積分方位（setRateHeading）。加速でも PI をかけられる。 */
  const int start = (cruise_power >= 0) ? start_power : -start_power;
  const int mag_c = (cruise_power >= 0) ? cruise_power : -cruise_power;
  const int mag_s = (start_power >= 0) ? start_power : -start_power;
  if (mag_c <= mag_s) {
    gMotors.run(cruise_power, cruise_power);
  } else {
    rampStraightPower(left_base, right_base, accel_deg, start, cruise_power);
  }
}

void TestScene::decelToStopByEncoder(int32_t left_base, int32_t right_base,
                                     int32_t total_deg, int cruise_power,
                                     int32_t decel_deg)
{
  /* S字減速: 残り距離比を smoothstep してパワーを落とす。
   * 最低パワー未満にしない。停滞/上限で抜けて coast。 */
  if (decel_deg < 1) {
    decel_deg = 1;
  }
  const int sign = (cruise_power >= 0) ? 1 : -1;
  const int mag0 = (cruise_power >= 0) ? cruise_power : -cruise_power;
  const int min_mag =
      (mag0 < StraightDecelMinPower) ? mag0 : StraightDecelMinPower;

  int32_t  last_traveled = gMotors.traveledDegFrom(left_base, right_base);
  uint32_t stall_ms      = 0U;
  uint32_t elapsed_ms    = 0U;

  while (gMotors.traveledDegFrom(left_base, right_base) < total_deg) {
    if (elapsed_ms >= StraightDecelMaxMs) {
      break;
    }
    const int32_t traveled =
        gMotors.traveledDegFrom(left_base, right_base);
    if (traveled <= last_traveled) {
      stall_ms += StepMs;
      if (stall_ms >= StraightDecelStallMs) {
        break;
      }
    } else {
      last_traveled = traveled;
      stall_ms      = 0U;
    }

    const int32_t rem = total_deg - traveled;
    const float s =
        static_cast<float>(rem) / static_cast<float>(decel_deg);
    int mag = lerpPowerS(min_mag, mag0, s);
    if (mag > mag0) {
      mag = mag0;
    }
    if (mag < min_mag) {
      mag = min_mag;
    }
    gMotors.run(sign * mag, sign * mag);
    if (StraightEncLogIntervalMs > 0U &&
        (elapsed_ms % StraightEncLogIntervalMs) == 0U) {
      logStraightEncoders(left_base, right_base, "decel");
    }
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
  gMotors.coast();
}

/* S字加速→クルーズ→S字減速→coast。
 * encoder_balance なら左右エンコーダ差、そうでなければ 1D 方位 PI。 */
void TestScene::driveStraight(float target_heading, int32_t total_deg, int power,
                              bool encoder_balance, bool initial_settings)
{
  if (total_deg <= 0) {
    gMotors.coast();
    return;
  }

  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gMotors.setEncoderStraightBalance(encoder_balance);
  if (!encoder_balance) {
    gGyro.setRateHeading(true);
  }
  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(StraightKp, StraightKi,
                        initial_settings ? InitialStraightBalanceIMax : StraightIMax,
                        initial_settings ? InitialStraightBalanceDeadbandDeg : StraightDeadbandDeg);
  gMotors.setTargetHeading(target_heading);
  const int32_t enc_left_base  = gMotors.leftEncoderCount();
  const int32_t enc_right_base = gMotors.rightEncoderCount();

  const int32_t decel_deg = initial_settings ? InitialStraightDecelDeg : StraightDecelDeg;
  int32_t cruise = total_deg - decel_deg;
  if (cruise < 0) {
    cruise = 0;
  }
  int32_t accel_end = StraightAccelDeg;
  if (accel_end > cruise) {
    accel_end = cruise;
  }

  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);
  logStraightEncoders(enc_left_base, enc_right_base, "start");
  accelToCruise(enc_left_base, enc_right_base, accel_end, power,
                StraightAccelStartPower);
  cruiseUntilDeg(enc_left_base, enc_right_base, cruise);
  decelToStopByEncoder(enc_left_base, enc_right_base, total_deg, power,
                       decel_deg);
  logStraightEncoders(enc_left_base, enc_right_base, "end");
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  if (!encoder_balance) {
    gGyro.setRateHeading(false);
  }
  gMotors.setEncoderStraightBalance(false);
  dly_tsk(StepMs * 1000U);
}

void TestScene::runStraightTest()
{
  gGyro.reset();
  dly_tsk(GyroResetSettleMs * 1000U);
  driveStraight(StraightTargetDeg, StraightTotalDeg, StraightPower, false);
}

void TestScene::runEncoderStraightTest()
{
  gGyro.reset();
  dly_tsk(GyroResetSettleMs * 1000U);
  driveStraight(StraightTargetDeg, StraightTotalDeg, StraightPower, true);
}

/* スカイブルー検知 → Nansho3 相当直進 */
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
      dly_tsk(GyroResetSettleMs * 1000U);
      driveStraight(StraightTargetDeg, StraightTotalDeg, StraightPower);
      return;
    }
    gLineTracer.step();
    dly_tsk(5 * 1000U);
  }
}

/* Nansho3 と同じ両輪ピボット（終端 coast） */
void TestScene::turnNansho3SpinOnce(float target, bool left)
{
  const int   prefer = left ? -1 : 1;
  gMotors.turnToHeading(target,
                        /*tolerance_deg=*/0.0f, Nansho3SpinPower,
                        Nansho3SpinPower, Nansho3SpinMaxMs,
                        Nansho3SpinSlowApproachDeg, Nansho3SpinSlowPower,
                        /*stop_early_deg=*/0.0f, /*coast_end=*/true,
                        prefer, /*encoder_drift_correction=*/true);
}

void TestScene::turnNansho3SpinByEncoder(bool left)
{
  gMotors.turnByEncoder(Nansho3SpinEncoderDeg, left, Nansho3SpinPower,
                        Nansho3SpinPower, Nansho3SpinMaxMs,
                        Nansho3SpinEncoderSlowDeg, Nansho3SpinSlowPower,
                        Nansho3SpinEncoderEarlyDeg, /*coast_end=*/true);
}

float TestScene::spinGridTarget(float grid_deg)
{
  return Gyro::wrapDeg180(grid_deg);
}

/* Nansho3 と同じ両輪ピボット。90° 刻みの絶対角へ回す */
void TestScene::runNansho3SpinLoop()
{
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();
  waitLogConnected();
  gGyro.reset();

  static constexpr float kFacingDeg[4] = {0.0f, 90.0f, -180.0f, -90.0f};
  const bool left = (TEST_TURN_LEFT != 0);
  int   facing_i    = 0;

  while (1) {
    dly_tsk(Nansho3SpinBeforeMs * 1000U);
    facing_i = left ? (facing_i + 3) % 4 : (facing_i + 1) % 4;
    const int32_t l0 = gMotors.leftEncoderCount();
    const int32_t r0 = gMotors.rightEncoderCount();
    const float   h0 = gGyro.rawHeading();
    if (Nansho3SpinStopOnEncoder) {
      turnNansho3SpinByEncoder(left);
    } else {
      turnNansho3SpinOnce(spinGridTarget(kFacingDeg[facing_i]), left);
    }
    logAfterTurn(absEncoderDelta(gMotors.leftEncoderCount(), l0),
                 absEncoderDelta(gMotors.rightEncoderCount(), r0),
                 Gyro::wrapDeg180(gGyro.rawHeading() - h0), left);

    if (Nansho3SpinAfterMs > 0U) {
      dly_tsk(Nansho3SpinAfterMs * 1000U);
    }
  }
}

/* Nansho3 と同じ両輪ピボットを1回だけ。TEST_TURN_LEFT で左右 */
void TestScene::runNansho3OnceTurn()
{
  gGyro.reset();
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

  const bool  left    = (TEST_TURN_LEFT != 0);
  const float logical = left ? -90.0f : 90.0f;
  const float target  = spinGridTarget(Gyro::wrapDeg180(logical));

  dly_tsk(Nansho3SpinBeforeMs * 1000U);
  const int32_t l0 = gMotors.leftEncoderCount();
  const int32_t r0 = gMotors.rightEncoderCount();
  const float   h0 = gGyro.rawHeading();
  if (Nansho3SpinStopOnEncoder) {
    turnNansho3SpinByEncoder(left);
  } else {
    turnNansho3SpinOnce(target, left);
  }
  logAfterTurn(absEncoderDelta(gMotors.leftEncoderCount(), l0),
               absEncoderDelta(gMotors.rightEncoderCount(), r0),
               Gyro::wrapDeg180(gGyro.rawHeading() - h0), left);
}

/* その場旋回を左右ランダムに繰り返す。止め方・待ち・ログは 14 と同じ */
void TestScene::runRandomSpinLoop()
{
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();
  waitLogConnected();

  SYSTIM now = 0U;
  (void)get_tim(&now);
  srand((unsigned int)now);

  gGyro.reset();

  static constexpr float kFacingDeg[4] = {0.0f, 90.0f, -180.0f, -90.0f};
  int   facing_i    = 0;

  while (1) {
    dly_tsk(Nansho3SpinBeforeMs * 1000U);
    const bool left = ((rand() & 1) != 0);
    facing_i = left ? (facing_i + 3) % 4 : (facing_i + 1) % 4;
    const int32_t l0 = gMotors.leftEncoderCount();
    const int32_t r0 = gMotors.rightEncoderCount();
    const float   h0 = gGyro.rawHeading();
    if (Nansho3SpinStopOnEncoder) {
      turnNansho3SpinByEncoder(left);
    } else {
      turnNansho3SpinOnce(spinGridTarget(kFacingDeg[facing_i]), left);
    }
    logAfterTurn(absEncoderDelta(gMotors.leftEncoderCount(), l0),
                 absEncoderDelta(gMotors.rightEncoderCount(), r0),
                 Gyro::wrapDeg180(gGyro.rawHeading() - h0), left);

    if (Nansho3SpinAfterMs > 0U) {
      dly_tsk(Nansho3SpinAfterMs * 1000U);
    }
  }
}

/* 同じ方向へ90°旋回して四角を周回。基準方位は開始時だけリセットする。 */
void TestScene::runSquareTest()
{
  waitLogConnected();
  const auto write_config = [&](FILE *output) {
    if (output == nullptr) return;
    fprintf(output,
        "SQUARE_CONFIG,MOVING_WHEEL,%d,TURN_GAIN,%.3f,BIAS_STATIC_ONLY,1,EARLY,0,"
        "TURN_POWER,%d,SLOW_DEG,%.1f,SLOW_POWER,%d,AFTER_MS,%lu\n",
        HeadingFusionSettings::MovingWheelObservation ? 1 : 0,
        (double)HeadingFusionSettings::TurnCorrectionGain, Nansho3SpinPower, (double)Nansho3SpinSlowApproachDeg,
        Nansho3SpinSlowPower, (unsigned long)Nansho3SpinAfterMs);
    fflush(output);
  };
  if (EnableLog) {
    write_config(stdout);
    if (btLogWritable()) write_config(s_bt_out);
  }
  gGyro.setRateHeading(false);
  gGyro.reset();
  dly_tsk(GyroResetSettleMs * 1000U);

  const bool left = (TEST_TURN_LEFT != 0);
  float target = 0.0f;
  uint32_t lap = 1U;
  while (true) {
    for (int side = 0; side < 4; ++side) {
      if (EnableLog) {
        const char *format = "SQUARE_lap,%lu,SQUARE_side,%d,DIR,%s,TARGET,%.2f\n";
        printf(format, (unsigned long)lap, side + 1, left ? "L" : "R", (double)target);
        if (btLogWritable()) {
          fprintf(s_bt_out, format, (unsigned long)lap, side + 1, left ? "L" : "R", (double)target);
          fflush(s_bt_out);
        }
      }
      driveStraight(target, SquareSideDeg, StraightPower);
      dly_tsk(Nansho3SpinBeforeMs * 1000U);
      target = spinGridTarget(target + (left ? -90.0f : 90.0f));
      const int32_t l0 = gMotors.leftEncoderCount();
      const int32_t r0 = gMotors.rightEncoderCount();
      const float h0 = gGyro.rawHeading();
      if (Nansho3SpinStopOnEncoder) {
        turnNansho3SpinByEncoder(left);
      } else {
        turnNansho3SpinOnce(target, left);
      }
      logAfterTurn(absEncoderDelta(gMotors.leftEncoderCount(), l0),
                   absEncoderDelta(gMotors.rightEncoderCount(), r0),
                   Gyro::wrapDeg180(gGyro.rawHeading() - h0), left);
      // coast後の惰性が落ち着いてから次辺を開始する。
      dly_tsk(Nansho3SpinAfterMs * 1000U);

    }
    if (lap < UINT32_MAX) ++lap;
  }
}

/* 静止試験と、実角を外部計測するための1回だけの車輪旋回。 */
void TestScene::logHeadingCalibration(const char *phase, uint32_t elapsed_ms,
                                      float raw_start, float heading_start,
                                      int32_t left_start, int32_t right_start)
{
  int32_t left = 0, right = 0;
  const bool encoder_valid = gMotors.readEncoders(left, right);
  const float l = static_cast<float>(left - left_start);
  const float r = static_cast<float>(right - right_start);
  const float raw = gGyro.rawHeading();
  const float heading = gGyro.heading();
  const HeadingFusion::State state = gGyro.fusionState();
  const HeadingFusionConfig config = HeadingFusionSettings::config();
  const float wheel = (config.leftDiameter * l - config.rightDiameter * r) /
                       (2.0f * config.track);
  const auto write = [&](FILE *output) {
    if (output == nullptr) return;
    fprintf(output,
        "CAL_phase,%s,CAL_ms,%lu,ENC_ok,%d,L,%.0f,R,%.0f,AVG,%.1f,"
        "IMU_raw,%.4f,IMU_delta,%.4f,KF_delta,%.4f,WHEEL_delta,%.4f\n",
        phase, (unsigned long)elapsed_ms, encoder_valid ? 1 : 0,
        (double)l, (double)r, (double)(0.5f * (fabsf(l) + fabsf(r))),
        (double)raw, (double)Gyro::wrapDeg180(raw - raw_start),
        (double)(heading - heading_start), (double)wheel);
    writeHeadingFusionLog(output, state, heading);
    fflush(output);
  };
  write(stdout);
  if (btLogWritable()) write(s_bt_out);
}

void TestScene::runHeadingCalibration(bool turn)
{
  gMotors.brake();
  waitLogConnected();
  // BLE接続や設置時の揺れが収まってから原点だけ取り直す。biasは保持。
  dly_tsk(CalibrationSettleMs * 1000U);
  gGyro.reset();
  int32_t left = 0, right = 0;
  if (!gMotors.readEncoders(left, right) || !gGyro.fusionEnabled()) {
    printf("CAL_ERROR,encoder_or_fusion_unavailable\n");
    if (btLogWritable()) {
      fprintf(s_bt_out, "CAL_ERROR,encoder_or_fusion_unavailable\n");
      fflush(s_bt_out);
    }
    return;
  }
  float raw = gGyro.rawHeading();
  float heading = gGyro.heading();
  SYSTIM start = 0, now = 0;
  get_tim(&start);
  while (true) {
    get_tim(&now);
    const uint32_t elapsed = static_cast<uint32_t>((now - start) / 1000U);
    logHeadingCalibration("still", elapsed, raw, heading, left, right);
    if (elapsed >= CalibrationStillMs) break;
    dly_tsk(CalibrationLogMs * 1000U);
  }
  if (!turn) return;

  // 静止学習後のbiasを維持し、旋回前の実測基準を保存する。
  if (!gMotors.readEncoders(left, right)) return;
  raw = gGyro.rawHeading();
  heading = gGyro.heading();
  get_tim(&start);
  logHeadingCalibration("turn_start", 0U, raw, heading, left, right);
  gMotors.turnByEncoder(CalibrationEncoderDeg, TEST_TURN_LEFT != 0,
      CalibrationPower, CalibrationPower, Nansho3SpinMaxMs,
      CalibrationEncoderDeg, CalibrationPower, 0, /*coast_end=*/true);
  gMotors.brake();
  get_tim(&now);
  logHeadingCalibration("turn_brake", static_cast<uint32_t>((now - start) / 1000U),
                        raw, heading, left, right);
  dly_tsk(CalibrationSettleMs * 1000U);
  get_tim(&now);
  logHeadingCalibration("turn_settled", static_cast<uint32_t>((now - start) / 1000U),
                        raw, heading, left, right);
  // ここで終了して保持。次の左右試験は基準線へ置き直し、再起動する。
}

void TestScene::run()
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

#if TEST_CASE == 7
  runStraightTest();
#elif TEST_CASE == 8
  runEncoderStraightTest();
#elif TEST_CASE == 14
  runNansho3SpinLoop();
#elif TEST_CASE == 15
  runNansho3OnceTurn();
#elif TEST_CASE == 16
  runRandomSpinLoop();
#elif TEST_CASE == 17
  runSquareTest();
#elif TEST_CASE == 19
  gNansho3.runInitialTest();
#elif TEST_CASE == 20
  gNansho3.runStraightCellsTest(TEST_STRAIGHT_CELLS);
#elif TEST_CASE == 21
  runHeadingCalibration(false);
#elif TEST_CASE == 22
  runHeadingCalibration(true);
#else
  /* TEST_CASE 6: LT → 直進 → Nansho1〜3 */
  runLineTraceUntilSkyblueThenStraight();
#endif
}

extern "C" void TestTask(intptr_t exinf)
{
  (void)exinf;
  gTest.run();

#if TEST_CASE == 6
  gMotors.stop();
  gLineTracer.setEnabled(false);
  dly_tsk(50 * 1000U);
  act_tsk(NANSHO1_TASK);
#endif

  ext_tsk();
}
