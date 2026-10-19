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
  if (s_bt_out != nullptr) {
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
  if (s_bt_out != nullptr) {
    fprintf(s_bt_out, "MOT_L,%d,MOT_R,%d,ENC_avg,%d,PHASE,%s\n",
            (int)l, (int)r, (int)avg, ph);
    fflush(s_bt_out);
  }
  printf("MOT_L,%d,MOT_R,%d,ENC_avg,%d,PHASE,%s\n",
         (int)l, (int)r, (int)avg, ph);
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
  if (s_bt_out != nullptr) {
    fprintf(s_bt_out, "LOG_READY\n");
    fflush(s_bt_out);
  }
  printf("LOG_READY\n");
}

void TestScene::logAfterTurn(int32_t left_abs_deg, int32_t right_abs_deg,
                             float heading_rel_deg, bool left)
{
  if (!EnableLog) {
    return;
  }
  openBtLog();
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
                              bool encoder_balance)
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
  gMotors.setBalancePid(StraightKp, StraightKi, StraightIMax,
                        StraightDeadbandDeg);
  gMotors.setTargetHeading(target_heading);
  const int32_t enc_left_base  = gMotors.leftEncoderCount();
  const int32_t enc_right_base = gMotors.rightEncoderCount();

  int32_t cruise = total_deg - StraightDecelDeg;
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
                       StraightDecelDeg);
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
  const float extra = left ? Nansho3SpinLeftEarlyExtraDeg
                           : Nansho3SpinRightEarlyExtraDeg;
  const int   prefer = left ? -1 : 1;
  gMotors.turnToHeading(target, /*tolerance_deg=*/0.0f, Nansho3SpinPower,
                        Nansho3SpinPower, Nansho3SpinMaxMs,
                        Nansho3SpinSlowApproachDeg, Nansho3SpinSlowPower,
                        Nansho3SpinEarlyStopDeg + extra, /*coast_end=*/true,
                        prefer);
}

void TestScene::turnNansho3SpinByEncoder(bool left)
{
  gMotors.turnByEncoder(Nansho3SpinEncoderDeg, left, Nansho3SpinPower,
                        Nansho3SpinPower, Nansho3SpinMaxMs,
                        Nansho3SpinEncoderSlowDeg, Nansho3SpinSlowPower,
                        Nansho3SpinEncoderEarlyDeg, /*coast_end=*/true);
}

float TestScene::spinGridTarget(float grid_deg, float heading_add)
{
  return Gyro::wrapDeg180(grid_deg + heading_add);
}

void TestScene::accumulateSpinPer(bool left, int *turn_count, float *heading_add)
{
  if (turn_count == nullptr || heading_add == nullptr ||
      Nansho3SpinTurnsPerAdd <= 0) {
    return;
  }
  ++(*turn_count);
  if (*turn_count < Nansho3SpinTurnsPerAdd) {
    return;
  }
  *turn_count = 0;
  const float per = left ? Nansho3SpinPerLeftAddDeg : Nansho3SpinPerRightAddDeg;
  *heading_add = Gyro::wrapDeg180(*heading_add + per);
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
  int   turn_count  = 0;
  float heading_add = 0.0f;

  while (1) {
    dly_tsk(Nansho3SpinBeforeMs * 1000U);
    facing_i = left ? (facing_i + 3) % 4 : (facing_i + 1) % 4;
    const int32_t l0 = gMotors.leftEncoderCount();
    const int32_t r0 = gMotors.rightEncoderCount();
    const float   h0 = gGyro.heading();
    if (Nansho3SpinStopOnEncoder) {
      turnNansho3SpinByEncoder(left);
    } else {
      const float target =
          spinGridTarget(kFacingDeg[facing_i], heading_add);
      turnNansho3SpinOnce(target, left);
    }
    if (Nansho3SpinLogSettleMs > 0U) {
      dly_tsk(Nansho3SpinLogSettleMs * 1000U);
    }
    logAfterTurn(absEncoderDelta(gMotors.leftEncoderCount(), l0),
                 absEncoderDelta(gMotors.rightEncoderCount(), r0),
                 Gyro::wrapDeg180(gGyro.heading() - h0), left);
    if (!Nansho3SpinStopOnEncoder) {
      accumulateSpinPer(left, &turn_count, &heading_add);
    }

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
  const float target  = spinGridTarget(Gyro::wrapDeg180(logical), 0.0f);

  dly_tsk(Nansho3SpinBeforeMs * 1000U);
  const int32_t l0 = gMotors.leftEncoderCount();
  const int32_t r0 = gMotors.rightEncoderCount();
  const float   h0 = gGyro.heading();
  if (Nansho3SpinStopOnEncoder) {
    turnNansho3SpinByEncoder(left);
  } else {
    turnNansho3SpinOnce(target, left);
  }
  if (Nansho3SpinLogSettleMs > 0U) {
    dly_tsk(Nansho3SpinLogSettleMs * 1000U);
  }
  logAfterTurn(absEncoderDelta(gMotors.leftEncoderCount(), l0),
               absEncoderDelta(gMotors.rightEncoderCount(), r0),
               Gyro::wrapDeg180(gGyro.heading() - h0), left);
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
  int   turn_count  = 0;
  float heading_add = 0.0f;

  while (1) {
    dly_tsk(Nansho3SpinBeforeMs * 1000U);
    const bool left = ((rand() & 1) != 0);
    facing_i = left ? (facing_i + 3) % 4 : (facing_i + 1) % 4;
    const int32_t l0 = gMotors.leftEncoderCount();
    const int32_t r0 = gMotors.rightEncoderCount();
    const float   h0 = gGyro.heading();
    if (Nansho3SpinStopOnEncoder) {
      turnNansho3SpinByEncoder(left);
    } else {
      const float target =
          spinGridTarget(kFacingDeg[facing_i], heading_add);
      turnNansho3SpinOnce(target, left);
    }
    if (Nansho3SpinLogSettleMs > 0U) {
      dly_tsk(Nansho3SpinLogSettleMs * 1000U);
    }
    logAfterTurn(absEncoderDelta(gMotors.leftEncoderCount(), l0),
                 absEncoderDelta(gMotors.rightEncoderCount(), r0),
                 Gyro::wrapDeg180(gGyro.heading() - h0), left);
    if (!Nansho3SpinStopOnEncoder) {
      accumulateSpinPer(left, &turn_count, &heading_add);
    }

    if (Nansho3SpinAfterMs > 0U) {
      dly_tsk(Nansho3SpinAfterMs * 1000U);
    }
  }
}

void TestScene::run()
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

#if ETROBO_SCENE == 7
  runStraightTest();
#elif ETROBO_SCENE == 8
  runEncoderStraightTest();
#elif ETROBO_SCENE == 14
  runNansho3SpinLoop();
#elif ETROBO_SCENE == 15
  runNansho3OnceTurn();
#elif ETROBO_SCENE == 16
  runRandomSpinLoop();
#else
  /* SCENE 6: LT → 直進 → Nansho1〜3 */
  runLineTraceUntilSkyblueThenStraight();
#endif
}

extern "C" void TestTask(intptr_t exinf)
{
  (void)exinf;
  gTest.run();

#if ETROBO_SCENE == 6
  gMotors.stop();
  gLineTracer.setEnabled(false);
  dly_tsk(50 * 1000U);
  act_tsk(NANSHO1_TASK);
#endif

  ext_tsk();
}
