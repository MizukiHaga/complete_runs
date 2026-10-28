#include "Nansho3.h"

#include "kernel_cfg.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <serial/newlib.h>
#include <serial/serial.h>
#include <syssvc/serial.h>
#include <spike/hub/bluetooth.h>
#include <pbio/error.h>

#include "ColorLAB.h"
#include "Gyro.h"
#include "HeadingFusionLog.h"
#include "LineTracer.h"
#include "Straight.h"
#include "TurnDrift.h"

extern Gyro         gGyro;
extern Straight     gMotors;
extern LineTracer   gLineTracer;
extern ColorLAB     gColorLAB;
extern Nansho3Scene gNansho3;

static FILE *s_turn_log = nullptr;

static void openTurnLog()
{
  if (s_turn_log != nullptr) {
    return;
  }
  serial_opn_por(SIO_BLUETOOTH_PORTID);
  s_turn_log = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);
}

/* 未接続の fprintf/fflush は制御タスクを止める。接続中だけ BLE に書く。 */
static bool turnLogWritable()
{
  if (s_turn_log == nullptr) {
    return false;
  }
  bool connected = false;
  return hub_bluetooth_is_connected(&connected) == PBIO_SUCCESS && connected;
}

void Nansho3Scene::waitLogConnected()
{
  if (!EnableTurnLog && !EnableStraightLog) {
    return;
  }
  openTurnLog();
  while (1) {
    bool connected = false;
    if (hub_bluetooth_is_connected(&connected) == PBIO_SUCCESS && connected) {
      break;
    }
    dly_tsk(50 * 1000U);
  }
  if (turnLogWritable()) {
    fprintf(s_turn_log, "LOG_READY\n");
    fflush(s_turn_log);
  }
  printf("LOG_READY\n");
}

static int32_t absEncoderDelta(int32_t now, int32_t base)
{
  const int32_t d = now - base;
  return (d < 0) ? -d : d;
}

static void emitTurnEncodersOnce(int32_t left_abs_deg, int32_t right_abs_deg,
                                 float heading_deg, bool left,
                                 float enc_ang_deg, float ang_err_deg)
{
  const char *dir = left ? "L" : "R";
  if (turnLogWritable()) {
    fprintf(s_turn_log, "MOT_L,%d,MOT_R,%d,MOT_Lp,%d,MOT_Rp,%d\n",
            (int)left_abs_deg, (int)right_abs_deg, 0, 0);
    fprintf(s_turn_log, "IMU_h,%.2f\n", (double)heading_deg);
    fprintf(s_turn_log, "ENC_ang,%.2f,ANG_err,%.2f\n",
            (double)enc_ang_deg, (double)ang_err_deg);
    fprintf(s_turn_log, "DIR,%s\n", dir);
    fflush(s_turn_log);
  }
  printf("MOT_L,%d,MOT_R,%d,MOT_Lp,%d,MOT_Rp,%d\n",
         (int)left_abs_deg, (int)right_abs_deg, 0, 0);
  printf("IMU_h,%.2f\n", (double)heading_deg);
  printf("ENC_ang,%.2f,ANG_err,%.2f\n",
         (double)enc_ang_deg, (double)ang_err_deg);
  printf("DIR,%s\n", dir);
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

#if ETROBO_COURSE_L
/* Rコース（L/R で左右が鏡）。facing_i: 0=0°(+x), 1=90°(+y), 2=180°(-x), 3=-90°(-y) */
static constexpr int kDx[4] = {1, 0, -1, 0};
static constexpr int kDy[4] = {0, 1, 0, -1};
#else
/* Lコース。facing_i: 0=0°(+x), 1=90°(-y), 2=180°(-x), 3=-90°(+y) */
static constexpr int kDx[4] = {1, 0, -1, 0};
static constexpr int kDy[4] = {0, -1, 0, 1};
#endif

float Nansho3Scene::snapTo90(float heading_deg)
{
  const float wrapped = Gyro::wrapDeg180(heading_deg);
  return Gyro::wrapDeg180(roundf(wrapped / 90.0f) * 90.0f);
}

int Nansho3Scene::facingIndex(float heading_deg)
{
  const float f = snapTo90(heading_deg);
  if (fabsf(f) < 1.0f) {
    return 0;
  }
  if (fabsf(f - 90.0f) < 1.0f) {
    return 1;
  }
  if (fabsf(f + 90.0f) < 1.0f) {
    return 3;
  }
  return 2;
}

float Nansho3Scene::facingDegFromIndex(int facing_i)
{
  static constexpr float kDeg[4] = {0.0f, 90.0f, -180.0f, -90.0f};
  if (facing_i < 0 || facing_i >= FacingCount) {
    return 0.0f;
  }
  return kDeg[facing_i];
}

bool Nansho3Scene::inGrid(int x, int y)
{
  return x >= 0 && x < GridSize && y >= 0 && y < GridSize;
}

bool Nansho3Scene::isYAdjacent(GateSeg g)
{
  return g.a.x == g.b.x && abs(g.a.y - g.b.y) == 1 &&
         inGrid(g.a.x, g.a.y) && inGrid(g.b.x, g.b.y);
}

bool Nansho3Scene::isXAdjacent(GateSeg g)
{
  return g.a.y == g.b.y && abs(g.a.x - g.b.x) == 1 &&
         inGrid(g.a.x, g.a.y) && inGrid(g.b.x, g.b.y);
}

bool Nansho3Scene::isOrthoAdjacent(QrPoint p, QrPoint q)
{
  return (abs(p.x - q.x) + abs(p.y - q.y)) == 1;
}

QrPoint Nansho3Scene::nearerGateEnd(GateSeg gate, int sx, int sy,
                                    int sfacing_i,
                                    const RouteBlocks &blocked) const
{
  int face_a = sfacing_i;
  int face_b = sfacing_i;
  const int ca =
      bfsCost(sx, sy, sfacing_i, gate.a.x, gate.a.y, blocked, &face_a);
  const int cb =
      bfsCost(sx, sy, sfacing_i, gate.b.x, gate.b.y, blocked, &face_b);
  if (ca < 0 && cb < 0) {
    return gate.a;
  }
  if (ca < 0) {
    return gate.b;
  }
  if (cb < 0) {
    return gate.a;
  }
  return (ca <= cb) ? gate.a : gate.b;
}

void Nansho3Scene::clearBlocked(RouteBlocks &blocked)
{
  memset(blocked.cells, 0, sizeof(blocked.cells));
  blocked.edge_count = 0;
}

void Nansho3Scene::blockPoint(RouteBlocks &blocked, QrPoint p,
                              QrPoint except_goal)
{
  if (!inGrid(p.x, p.y)) {
    return;
  }
  if (p.x == except_goal.x && p.y == except_goal.y) {
    return;
  }
  blocked.cells[p.x][p.y] = true;
}

void Nansho3Scene::blockGates(RouteBlocks &blocked, const GateSeg *gates, int count)
{
  for (int i = 0; gates != nullptr && i < count && blocked.edge_count < 3; ++i) {
    blocked.edges[blocked.edge_count++] = gates[i];
  }
}

bool Nansho3Scene::blocksStep(const RouteBlocks &blocked, int x, int y, int nx, int ny)
{
  if (blocked.cells[nx][ny]) {
    return true;
  }
  for (int i = 0; i < blocked.edge_count; ++i) {
    const GateSeg &g = blocked.edges[i];
    if ((x == g.a.x && y == g.a.y && nx == g.b.x && ny == g.b.y) ||
        (x == g.b.x && y == g.b.y && nx == g.a.x && ny == g.a.y)) {
      return true;
    }
  }
  return false;
}

void Nansho3Scene::beepError() const
{
}

float Nansho3Scene::absoluteCardinalDeg(float logical_facing_deg) const
{
  /* 90° 刻み + 直前旋回の相対角平均補正。EarlyStop は載せない。 */
  return Gyro::wrapDeg180(snapTo90(logical_facing_deg + cardinal_offset_) +
                          heading_add_);
}

float Nansho3Scene::straightTargetDeg(float logical_facing_deg) const
{
  return absoluteCardinalDeg(logical_facing_deg);
}

void Nansho3Scene::setNextTarget(QrPoint p)
{
  next_target_     = p;
  has_next_target_ = true;
}

void Nansho3Scene::onArrivedCell()
{
  if (!has_next_target_) {
    return;
  }
  if (x_ == next_target_.x && y_ == next_target_.y) {
    has_next_target_ = false;
  }
}

/* 走り出しの 0 点から target_deg 進むまで待つ。モータは止めない。 */
void Nansho3Scene::cruiseUntilDeg(int32_t left_base, int32_t right_base,
                                  int32_t target_deg)
{
  uint32_t elapsed_ms = 0U;
  while (gMotors.traveledDegFrom(left_base, right_base) < target_deg) {
    if (EnableStraightLog && StraightLogIntervalMs > 0U &&
        (elapsed_ms % StraightLogIntervalMs) == 0U) {
      logDriveControl("cruise");
    }
    dly_tsk(CoreStraightStepMs * 1000U);
    elapsed_ms += CoreStraightStepMs;
  }
}

void Nansho3Scene::rampStraightPower(int32_t left_base, int32_t right_base,
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
    if (EnableStraightLog && StraightLogIntervalMs > 0U &&
        (elapsed_ms % StraightLogIntervalMs) == 0U) {
      logDriveControl("accel");
    }
    dly_tsk(CoreStraightStepMs * 1000U);
    elapsed_ms += CoreStraightStepMs;
  }
  gMotors.run(to_power, to_power);
}

void Nansho3Scene::accelToCruise(int32_t left_base, int32_t right_base,
                                 int32_t accel_deg, int cruise_power)
{
  /* 直進中は 1D レート積分方位。加速でも方位 PI をかけたまま。 */
  gGyro.setRateHeading(true);

  const int start = (cruise_power >= 0) ? CoreStraightAccelStartPower
                                        : -CoreStraightAccelStartPower;
  const int mag_c = (cruise_power >= 0) ? cruise_power : -cruise_power;
  if (mag_c <= CoreStraightAccelStartPower) {
    gMotors.run(cruise_power, cruise_power);
  } else {
    rampStraightPower(left_base, right_base, accel_deg, start, cruise_power);
  }
}

void Nansho3Scene::decelToStopByEncoder(int32_t left_base, int32_t right_base,
                                        int32_t total_deg, int cruise_power,
                                        int32_t decel_deg)
{
  /* total_degまでの残り距離でS字減速し、最後は短絡ブレーキで止める。
   * 最低パワー未満にしない（進まず永久待ちになるのを防ぐ）。
   * 停滞・時間上限でもブレーキを掛け、停止待ち後に後続へ進む。 */
  if (decel_deg < 1) {
    decel_deg = 1;
  }
  const int sign = (cruise_power >= 0) ? 1 : -1;
  const int mag0 = (cruise_power >= 0) ? cruise_power : -cruise_power;
  const int min_mag =
      (mag0 < CoreStraightDecelMinPower) ? mag0 : CoreStraightDecelMinPower;

  int32_t  last_traveled = gMotors.traveledDegFrom(left_base, right_base);
  uint32_t stall_ms      = 0U;
  uint32_t elapsed_ms    = 0U;

  while (gMotors.traveledDegFrom(left_base, right_base) < total_deg) {
    if (elapsed_ms >= CoreStraightDecelMaxMs) {
      break;
    }
    const int32_t traveled =
        gMotors.traveledDegFrom(left_base, right_base);
    if (traveled <= last_traveled) {
      stall_ms += CoreStraightStepMs;
      if (stall_ms >= CoreStraightDecelStallMs) {
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
    if (EnableStraightLog && StraightLogIntervalMs > 0U &&
        (elapsed_ms % StraightLogIntervalMs) == 0U) {
      logDriveControl("decel");
    }
    dly_tsk(CoreStraightStepMs * 1000U);
    elapsed_ms += CoreStraightStepMs;
  }
  gMotors.brake();
  dly_tsk(CoreStraightBrakeSettleMs * 1000U);
}

void Nansho3Scene::driveOneCell(bool stop_after)
{
  driveCellDistance(stop_after);

  const int fi = facingIndex(facing_deg_);
  const int nx = x_ + kDx[fi];
  const int ny = y_ + kDy[fi];
  if (inGrid(nx, ny)) {
    x_ = nx;
    y_ = ny;
    grid_[x_][y_] = 1;
  }
  onArrivedCell();
}

void Nansho3Scene::driveCellDistance(bool stop_after)
{
  gLineTracer.setEnabled(false);

  const int fi_ms = facingIndex(facing_deg_);
  const int32_t extra =
      (fi_ms == 1 || fi_ms == 3) ? CoreStraightYExtraDeg : CoreStraightXExtraDeg;
  const int pwr = CoreRunStraightPower;

  /* このマスの走り出しを 0 点にする。連続 Fwd の途中でも止めずに測れる。 */
  const int32_t enc_left_base  = gMotors.leftEncoderCount();
  const int32_t enc_right_base = gMotors.rightEncoderCount();
  const int32_t total_stop = CoreStraightDeg + extra;
  const int32_t total_go   = CoreStraightContinueDeg + extra;
  int32_t cruise =
      stop_after ? (total_stop - CoreStraightDecelDeg) : total_go;
  if (cruise < 0) {
    cruise = 0;
  }

  if (!straight_running_) {
    gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
    int32_t accel_end = CoreStraightAccelDeg;
    if (accel_end > cruise) {
      accel_end = cruise;
    }
    accelToCruise(enc_left_base, enc_right_base, accel_end, pwr);
    straight_running_ = true;
  } else {
    gGyro.setRateHeading(true);
  }

  if (stop_after) {
    cruiseUntilDeg(enc_left_base, enc_right_base, cruise);
    decelToStopByEncoder(enc_left_base, enc_right_base, total_stop, pwr,
                         CoreStraightDecelDeg);
    straight_running_ = false;
  } else {
    cruiseUntilDeg(enc_left_base, enc_right_base, total_go);
  }

}

void Nansho3Scene::runStraightCellsTest(int cells)
{
  if (cells < 1 || cells > 3) {
    gMotors.stop();
    return;
  }
  waitLogConnected();
  gLineTracer.setEnabled(false);
  gGyro.setRateHeading(false);
  gGyro.reset();
  dly_tsk(100U * 1000U);
  facing_deg_ = 0.0f;
  cardinal_offset_ = 0.0f;
  heading_add_ = 0.0f;
  straight_running_ = false;
  gMotors.setEncoderStraightBalance(false);
  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);
  for (int cell = 0; cell < cells; ++cell) {
    driveCellDistance(cell + 1 == cells);
  }
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gGyro.setRateHeading(false);
  // 通常直進と同じ coast 終了。即時ブレーキによる停止距離の差を避ける。
}

void Nansho3Scene::logDriveControl(const char *phase, bool use_motor_target,
                                   float target_deg)
{
  openTurnLog();
  const char *ph = (phase != nullptr) ? phase : "run";
  const float raw = gGyro.rawHeading();
  const float corrected = gGyro.heading();
  const float tgt =
      use_motor_target ? gMotors.targetHeading() : target_deg;
  const int lp = gMotors.lastAppliedLeftPower();
  const int rp = gMotors.lastAppliedRightPower();
  const char *format =
      "IMU_raw,%.2f,IMU_h,%.2f,TGT,%.2f,MOT_Lp,%d,MOT_Rp,%d,PHASE,%s\n";
  printf(format, (double)raw, (double)corrected, (double)tgt, lp, rp, ph);
  if (turnLogWritable()) {
    fprintf(s_turn_log, format, (double)raw, (double)corrected, (double)tgt, lp,
            rp, ph);
    fflush(s_turn_log);
  }
  if (gGyro.fusionEnabled() && use_motor_target) {
    const HeadingFusion::State state = gGyro.fusionState();
    writeHeadingFusionLog(stdout, state, corrected);
    if (turnLogWritable()) {
      writeHeadingFusionLog(s_turn_log, state, corrected);
      fflush(s_turn_log);
    }
  }
}

void Nansho3Scene::logStopInstant()
{
  if (!EnableTurnLog) {
    return;
  }
  openTurnLog();
  writeTurnEncoderLog(stdout, gMotors.turnEncoderSnapshot());
  if (turnLogWritable()) {
    writeTurnEncoderLog(s_turn_log, gMotors.turnEncoderSnapshot());
    fflush(s_turn_log);
  }
  const int32_t left_abs = gMotors.lastTurnEncLeftAbsDeg();
  const int32_t right_abs = gMotors.lastTurnEncRightAbsDeg();
  const float enc_avg =
      0.5f * static_cast<float>(left_abs + right_abs);
  const float per90 = TurnDriftConfig::EncoderPer90Deg;
  const float enc_ang =
      (per90 > 0.0f) ? (enc_avg * 90.0f / per90) : 0.0f;
  const char *format =
      "STOP_ms,%lu,IMU_acc,%.2f,ENC_ang,%.2f,IMU_corrected,%.2f\n";
  printf(format, (unsigned long)gMotors.lastTurnElapsedMs(),
         (double)gMotors.lastTurnGyroAccDeg(), (double)enc_ang,
         (double)gMotors.lastTurnHeadingDeg());
  if (turnLogWritable()) {
    fprintf(s_turn_log, format, (unsigned long)gMotors.lastTurnElapsedMs(),
            (double)gMotors.lastTurnGyroAccDeg(), (double)enc_ang,
            (double)gMotors.lastTurnHeadingDeg());
    fflush(s_turn_log);
  }
}

void Nansho3Scene::logAfterTurn(int32_t left_abs_deg, int32_t right_abs_deg,
                                float heading_rel_deg, bool left,
                                float turn_target_deg)
{
  if (!EnableTurnLog) {
    return;
  }
  openTurnLog();
  const float enc_avg =
      0.5f * static_cast<float>(left_abs_deg + right_abs_deg);
  const float per90 = TurnDriftConfig::EncoderPer90Deg;
  const float enc_ang =
      (per90 > 0.0f) ? (enc_avg * 90.0f / per90) : 0.0f;
  const float ang_err = enc_ang - 90.0f;
  emitTurnEncodersOnce(left_abs_deg, right_abs_deg, heading_rel_deg, left,
                       enc_ang, ang_err);
  logDriveControl("turn", /*use_motor_target=*/false, turn_target_deg);
  if (gGyro.fusionEnabled()) {
    const HeadingFusion::State state = gGyro.fusionState();
    const float heading = gGyro.heading();
    writeHeadingFusionLog(stdout, state, heading);
    if (turnLogWritable()) {
      writeHeadingFusionLog(s_turn_log, state, heading);
      fflush(s_turn_log);
    }
    return;
  }
  const float correction = gMotors.turnDriftCorrectionDeg();
  const float rate = gMotors.turnDriftRate();
  const TurnDrift &diag = gMotors.turnDriftDiagnostics();
  const char *format = "DRIFT_gain,%.3f,DRIFT_err,%.3f,DRIFT_rem,%.3f,DRIFT_status,%d,DRIFT_ok,%lu,DRIFT_warm,%lu,DRIFT_invalid,%lu,DRIFT_delay,%lu,DRIFT_wheel,%lu,DRIFT_dir,%lu,DRIFT_slow,%lu,DRIFT_rateReject,%lu,DRIFT_angleReject,%lu\n";
  printf(format, (double)diag.gain(), (double)diag.angleError(), (double)diag.residual(), (int)diag.status(),
         (unsigned long)diag.count(TurnDrift::Accepted), (unsigned long)diag.count(TurnDrift::WarmingUp),
         (unsigned long)diag.count(TurnDrift::Invalid), (unsigned long)diag.count(TurnDrift::Delayed),
         (unsigned long)diag.count(TurnDrift::WheelMismatch), (unsigned long)diag.count(TurnDrift::WrongDirection),
         (unsigned long)diag.count(TurnDrift::TooSlow), (unsigned long)diag.count(TurnDrift::RateMismatch),
         (unsigned long)diag.count(TurnDrift::AngleMismatch));
  if (turnLogWritable()) {
    fprintf(s_turn_log, format, (double)diag.gain(), (double)diag.angleError(), (double)diag.residual(), (int)diag.status(),
            (unsigned long)diag.count(TurnDrift::Accepted), (unsigned long)diag.count(TurnDrift::WarmingUp),
            (unsigned long)diag.count(TurnDrift::Invalid), (unsigned long)diag.count(TurnDrift::Delayed),
            (unsigned long)diag.count(TurnDrift::WheelMismatch), (unsigned long)diag.count(TurnDrift::WrongDirection),
            (unsigned long)diag.count(TurnDrift::TooSlow), (unsigned long)diag.count(TurnDrift::RateMismatch),
            (unsigned long)diag.count(TurnDrift::AngleMismatch));
  }
  printf("DRIFT_corr,%.3f,DRIFT_rate,%.3f,IMU_corrected,%.3f\n",
         (double)correction, (double)rate, (double)gGyro.heading());
  if (turnLogWritable()) {
    fprintf(s_turn_log, "DRIFT_corr,%.3f,DRIFT_rate,%.3f,IMU_corrected,%.3f\n",
            (double)correction, (double)rate, (double)gGyro.heading());
    fflush(s_turn_log);
  }
}

void Nansho3Scene::turnToAbsolute(float target_heading, int prefer_sign,
                                  bool log_turn)
{
  gGyro.setRateHeading(false);
  const bool do_log = EnableTurnLog && log_turn;
  const int32_t l0 = do_log ? gMotors.leftEncoderCount() : 0;
  const int32_t r0 = do_log ? gMotors.rightEncoderCount() : 0;
  const float   h0 = do_log ? gGyro.rawHeading() : 0.0f;

  gMotors.turnToHeading(target_heading, /*tolerance_deg=*/0.0f, CoreTurnPower,
                        CoreTurnPower, CoreTurnMaxMs, CoreTurnSlowApproachDeg,
                        CoreTurnSlowPower, /*stop_early_deg=*/0.0f,
                        /*coast_end=*/true,
                        prefer_sign, /*encoder_drift_correction=*/true);

  if (do_log) {
    logStopInstant();
    const float heading_rel = Gyro::wrapDeg180(gGyro.rawHeading() - h0);
    const bool left = gMotors.lastTurnGyroAccDeg() < 0.0f;
    logAfterTurn(absEncoderDelta(gMotors.leftEncoderCount(), l0),
                 absEncoderDelta(gMotors.rightEncoderCount(), r0),
                 heading_rel, left, target_heading);
  }

  if (do_log && !gGyro.fusionEnabled()) {
    const unsigned long skipped = static_cast<unsigned long>(
        gMotors.turnDriftDiagnostics().count(TurnDrift::EncoderUnderRead));
    const TurnDrift &diag = gMotors.turnDriftDiagnostics();
    const char *format =
        "DRIFT_Lsigned,%.2f,DRIFT_Rsigned,%.2f,DRIFT_enc_signed,%.3f,"
        "DRIFT_gyro_signed,%.3f,DRIFT_under,%lu\n";
    printf(format, (double)diag.leftEncoder(), (double)diag.rightEncoder(),
           (double)diag.encoderAngle(), (double)diag.gyroAngle(), skipped);
    if (turnLogWritable()) {
      fprintf(s_turn_log, format, (double)diag.leftEncoder(), (double)diag.rightEncoder(),
              (double)diag.encoderAngle(), (double)diag.gyroAngle(), skipped);
      fflush(s_turn_log);
    }
  }
}

void Nansho3Scene::turnExitOneWheel(float target_heading)
{
  gLineTracer.setEnabled(false);
  gGyro.setRateHeading(false);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < ExitTurnMaxMs) {
    if (gGyro.reached(target_heading, ExitTurnToleranceDeg)) {
      break;
    }
    gMotors.run(ExitTurnLeftPower, ExitTurnRightPower);
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
  gMotors.stop();
}

void Nansho3Scene::onGatePassed()
{
  ++gates_since_gyro_reset_;
}

void Nansho3Scene::turnCoreToAbsolute(float target, int prefer_sign)
{
  gLineTracer.setEnabled(false);
  /* 減速は直前の Fwd のマス時間内。終端は coast（stop だと尾が上がる） */
  gMotors.coast();
  straight_running_ = false;
  gGyro.setRateHeading(false);
  dly_tsk(CoreTurnBeforeMs * 1000U);

  turnToAbsolute(target, prefer_sign);
  heading_add_ = 0.0f; // 動的補正は Gyro 側に反映済み
  gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
  dly_tsk(CoreTurnAfterMs * 1000U);
}

void Nansho3Scene::turnByAction(MoveAction action)
{
  const float offset =
      (action == MoveAction::L90) ? -90.0f : 90.0f;
  /* 通常は最短側。±180°で同距離の場合だけBFSの旋回方向を使う。 */
  const int prefer_sign = (action == MoveAction::L90) ? -1 : 1;
  facing_deg_ =
      facingDegFromIndex(facingIndex(facing_deg_ + offset));
  /* 旋回目標は 90° 刻みの絶対角。EarlyStop 分は直進目標に載せない */
  const float target = Gyro::wrapDeg180(
      snapTo90(facing_deg_ + cardinal_offset_));
  turnCoreToAbsolute(target, prefer_sign);
}

int Nansho3Scene::bfsPlan(int sx, int sy, int sfacing_i, int gx, int gy,
                          const RouteBlocks &blocked, MoveAction *out_actions,
                          int max_actions, int *out_facing_i, int *out_cost) const
{
  if (!inGrid(sx, sy) || !inGrid(gx, gy)) {
    return -1;
  }
  if (sfacing_i < 0 || sfacing_i >= FacingCount) {
    return -1;
  }
  /* ゴール自身が禁止だと到達不能 */
  if (blocked.cells[gx][gy]) {
    return -1;
  }

  if (sx == gx && sy == gy) {
    if (out_facing_i != nullptr) {
      *out_facing_i = sfacing_i;
    }
    if (out_cost != nullptr) {
      *out_cost = 0;
    }
    return 0;
  }

  int  dist[GridSize][GridSize][FacingCount];
  bool used[GridSize][GridSize][FacingCount];
  for (int x = 0; x < GridSize; ++x) {
    for (int y = 0; y < GridSize; ++y) {
      for (int f = 0; f < FacingCount; ++f) {
        dist[x][y][f] = BfsInfCost;
        used[x][y][f] = false;
      }
    }
  }

  int8_t  parent_x[StateCount];
  int8_t  parent_y[StateCount];
  int8_t  parent_f[StateCount];
  uint8_t parent_action[StateCount];

  for (int i = 0; i < StateCount; ++i) {
    parent_x[i] = -1;
    parent_y[i] = -1;
    parent_f[i] = -1;
    parent_action[i] = 0;
  }

  dist[sx][sy][sfacing_i] = 0;

  int  goal_f = -1;
  bool found  = false;

  for (int step = 0; step < StateCount; ++step) {
    int bx = -1;
    int by = -1;
    int bf = -1;
    int bd = BfsInfCost;
    for (int x = 0; x < GridSize; ++x) {
      for (int y = 0; y < GridSize; ++y) {
        for (int f = 0; f < FacingCount; ++f) {
          if (!used[x][y][f] && dist[x][y][f] < bd) {
            bd = dist[x][y][f];
            bx = x;
            by = y;
            bf = f;
          }
        }
      }
    }
    if (bx < 0) {
      break;
    }
    used[bx][by][bf] = true;
    if (bx == gx && by == gy) {
      goal_f = bf;
      found  = true;
      break;
    }

    const int cx = bx;
    const int cy = by;
    const int cf = bf;
    const MoveAction cands[3] = {MoveAction::Fwd, MoveAction::L90,
                                 MoveAction::R90};
    for (int ci = 0; ci < 3; ++ci) {
      int nx = cx;
      int ny = cy;
      int nf = cf;
      int add = BfsTurnCost;
      const MoveAction act = cands[ci];

      if (act == MoveAction::Fwd) {
        add = BfsFwdCost;
        nx = cx + kDx[cf];
        ny = cy + kDy[cf];
        if (!inGrid(nx, ny)) {
          continue;
        }
        /* 禁止マスと順番外のゲート辺を避ける */
        if (blocksStep(blocked, cx, cy, nx, ny)) {
          continue;
        }
      } else if (act == MoveAction::L90) {
        nf = (cf + 3) % FacingCount;
      } else {
        nf = (cf + 1) % FacingCount;
      }

      const int nd = dist[cx][cy][cf] + add;
      if (nd >= dist[nx][ny][nf]) {
        continue;
      }
      dist[nx][ny][nf] = nd;
      const int ni = nx * (GridSize * FacingCount) + ny * FacingCount + nf;
      parent_x[ni]      = static_cast<int8_t>(cx);
      parent_y[ni]      = static_cast<int8_t>(cy);
      parent_f[ni]      = static_cast<int8_t>(cf);
      parent_action[ni] = static_cast<uint8_t>(act);
    }
  }

  if (!found) {
    return -1;
  }

  if (out_facing_i != nullptr) {
    *out_facing_i = goal_f;
  }
  if (out_cost != nullptr) {
    *out_cost = dist[gx][gy][goal_f];
  }

  if (out_actions == nullptr || max_actions <= 0) {
    int len = 0;
    int cx  = gx;
    int cy  = gy;
    int cf  = goal_f;
    while (!(cx == sx && cy == sy && cf == sfacing_i)) {
      const int ni = cx * (GridSize * FacingCount) + cy * FacingCount + cf;
      const int px = parent_x[ni];
      const int py = parent_y[ni];
      const int pf = parent_f[ni];
      if (px < 0) {
        return -1;
      }
      ++len;
      cx = px;
      cy = py;
      cf = pf;
    }
    return len;
  }

  MoveAction rev[MaxPathLen];
  int        len = 0;
  int        cx  = gx;
  int        cy  = gy;
  int        cf  = goal_f;
  while (!(cx == sx && cy == sy && cf == sfacing_i)) {
    const int ni = cx * (GridSize * FacingCount) + cy * FacingCount + cf;
    const int px = parent_x[ni];
    const int py = parent_y[ni];
    const int pf = parent_f[ni];
    if (px < 0 || len >= MaxPathLen) {
      return -1;
    }
    rev[len++] = static_cast<MoveAction>(parent_action[ni]);
    cx = px;
    cy = py;
    cf = pf;
  }

  if (len > max_actions) {
    return -1;
  }
  for (int i = 0; i < len; ++i) {
    out_actions[i] = rev[len - 1 - i];
  }
  return len;
}

int Nansho3Scene::bfsCost(int sx, int sy, int sfacing_i, int gx, int gy,
                          const RouteBlocks &blocked, int *out_facing_i) const
{
  int cost = -1;
  if (bfsPlan(sx, sy, sfacing_i, gx, gy, blocked, nullptr, 0, out_facing_i,
              &cost) < 0) {
    return -1;
  }
  return cost;
}

bool Nansho3Scene::planStartsWithFwd(int sx, int sy, int sfacing_i, QrPoint goal,
                                     const RouteBlocks &blocked) const
{
  MoveAction acts[MaxPathLen];
  int        end_f = sfacing_i;
  const int  n =
      bfsPlan(sx, sy, sfacing_i, goal.x, goal.y, blocked, acts, MaxPathLen,
              &end_f);
  return n > 0 && acts[0] == MoveAction::Fwd;
}

bool Nansho3Scene::passGateStartsWithFwd(
    int sx, int sy, int sfacing_i, GateSeg gate, const GateSeg *past_gates,
    int past_count, const GateSeg *future_gates, int future_count,
    const GateSeg *tgt_continue_gate, const GateSeg *tgt_continue_futures,
    int tgt_continue_future_count) const
{
  /* 予測と実走に同じ通過順制約を使う。端点への進入は可、順番外の辺だけ通過禁止。 */

  RouteBlocks block_none;
  clearBlocked(block_none);
  blockGates(block_none, past_gates, past_count);
  blockGates(block_none, future_gates, future_count);

  RouteBlocks block_to_a = block_none;
  blockPoint(block_to_a, gate.b, gate.a);

  int face_at_a = sfacing_i;
  const int cost_to_a =
      bfsCost(sx, sy, sfacing_i, gate.a.x, gate.a.y, block_to_a, &face_at_a);
  int face_ab_at_b = face_at_a;
  const int cost_a_to_b =
      (cost_to_a < 0)
          ? -1
          : bfsCost(gate.a.x, gate.a.y, face_at_a, gate.b.x, gate.b.y,
                    block_none, &face_ab_at_b);

  RouteBlocks block_to_b = block_none;
  blockPoint(block_to_b, gate.a, gate.b);

  int face_at_b = sfacing_i;
  const int cost_to_b =
      bfsCost(sx, sy, sfacing_i, gate.b.x, gate.b.y, block_to_b, &face_at_b);
  int face_ba_at_a = face_at_b;
  const int cost_b_to_a =
      (cost_to_b < 0)
          ? -1
          : bfsCost(gate.b.x, gate.b.y, face_at_b, gate.a.x, gate.a.y,
                    block_none, &face_ba_at_a);

  /* passGateShortest と同じく、出口→次ゲートも足して a→b / b→a を選ぶ */
  int extra_ab = 0;
  int extra_ba = 0;
  const GateSeg *next_gate = nullptr;
  if (future_count > 0) {
    next_gate = &future_gates[0];
  } else if (tgt_continue_gate != nullptr) {
    next_gate = tgt_continue_gate;
    (void)tgt_continue_futures;
    (void)tgt_continue_future_count;
  }
  if (next_gate != nullptr) {
    if (cost_a_to_b >= 0) {
      const QrPoint next_ab =
          nearerGateEnd(*next_gate, gate.b.x, gate.b.y, face_ab_at_b,
                        block_none);
      int face_tmp = face_ab_at_b;
      const int c = bfsCost(gate.b.x, gate.b.y, face_ab_at_b, next_ab.x,
                            next_ab.y, block_none, &face_tmp);
      if (c >= 0) {
        extra_ab = c;
      }
    }
    if (cost_b_to_a >= 0) {
      const QrPoint next_ba =
          nearerGateEnd(*next_gate, gate.a.x, gate.a.y, face_ba_at_a,
                        block_none);
      int face_tmp = face_ba_at_a;
      const int c = bfsCost(gate.a.x, gate.a.y, face_ba_at_a, next_ba.x,
                            next_ba.y, block_none, &face_tmp);
      if (c >= 0) {
        extra_ba = c;
      }
    }
  }

  const bool ab_ok = (cost_to_a >= 0 && cost_a_to_b >= 0);
  const bool ba_ok = (cost_to_b >= 0 && cost_b_to_a >= 0);
  if (!ab_ok && !ba_ok) {
    return false;
  }

  QrPoint first = gate.a;
  bool    use_ab = ab_ok;
  if (ab_ok && ba_ok) {
    /* 次色へ出やすい向きを優先。近い入口から入ると出口が次色と逆になり、
     * L では赤のあと青と反対へ出て回り道になる。 */
    if (next_gate != nullptr && extra_ab != extra_ba) {
      use_ab = (extra_ab < extra_ba);
    } else {
      use_ab = (cost_to_a + cost_a_to_b) <= (cost_to_b + cost_b_to_a);
    }
    const QrPoint here = {sx, sy};
    /* 既に端上なら、その端から他端へ通過（逆向き→戻りは二重通過になる） */
    if (here.x == gate.a.x && here.y == gate.a.y) {
      use_ab = true;
    } else if (here.x == gate.b.x && here.y == gate.b.y) {
      use_ab = false;
    }
  } else if (!ab_ok) {
    use_ab = false;
  }
  if (!use_ab) {
    first = gate.b;
  }

  const QrPoint other = use_ab ? gate.b : gate.a;
  if (sx == first.x && sy == first.y) {
    /* 入口にいるなら、最初の動きは他端への通過 */
    return planStartsWithFwd(sx, sy, sfacing_i, other, block_none);
  }

  RouteBlocks block_first = block_none;
  blockPoint(block_first, other, first);
  return planStartsWithFwd(sx, sy, sfacing_i, first, block_first);
}

bool Nansho3Scene::planStartsWithFwdTowardGate(
    int sx, int sy, int sfacing_i, GateSeg gate, const GateSeg *block_gates,
    int block_count) const
{
  RouteBlocks block_a;
  clearBlocked(block_a);
  blockGates(block_a, block_gates, block_count);
  blockPoint(block_a, gate.b, gate.a);
  if (planStartsWithFwd(sx, sy, sfacing_i, gate.a, block_a)) {
    return true;
  }

  RouteBlocks block_b;
  clearBlocked(block_b);
  blockGates(block_b, block_gates, block_count);
  blockPoint(block_b, gate.a, gate.b);
  return planStartsWithFwd(sx, sy, sfacing_i, gate.b, block_b);
}

bool Nansho3Scene::goTo(QrPoint goal, const RouteBlocks &blocked,
                        bool stop_at_end)
{
  setNextTarget(goal);

  if (x_ == goal.x && y_ == goal.y) {
    if (stop_at_end && straight_running_) {
      gMotors.coast();
      straight_running_ = false;
    }
    onArrivedCell();
    return true;
  }

  MoveAction actions[MaxPathLen];
  int        end_facing = facingIndex(facing_deg_);
  const int  n =
      bfsPlan(x_, y_, facingIndex(facing_deg_), goal.x, goal.y, blocked,
              actions, MaxPathLen, &end_facing);
  if (n < 0) {
    gMotors.coast();
    straight_running_ = false;
    beepError();
    return false;
  }

  /* 前ゲートから走りっぱなしでも、次が旋回なら先に止める */
  if (straight_running_ && actions[0] != MoveAction::Fwd) {
    gMotors.coast();
    straight_running_ = false;
  }

  for (int i = 0; i < n; ++i) {
    if (actions[i] == MoveAction::Fwd) {
      const bool next_is_fwd =
          (i + 1 < n) && (actions[i + 1] == MoveAction::Fwd);
      const bool is_last = (i + 1 >= n);
      /* 連続 Fwd は止めない（ゲートの goTo またぎも stop_at_end=false でつなぐ）。
       * 次が旋回なら止める。1マスで止まるのはそのときと stop_at_end のときだけ */
      const bool stop_after =
          next_is_fwd ? false : (is_last ? stop_at_end : true);
      driveOneCell(stop_after);
    } else {
      turnByAction(actions[i]);
    }
  }

  if (has_next_target_ && x_ == goal.x && y_ == goal.y) {
    onArrivedCell();
  }
  facing_deg_ = facingDegFromIndex(facingIndex(facing_deg_));
  (void)end_facing;
  return x_ == goal.x && y_ == goal.y;
}

bool Nansho3Scene::passGateShortest(GateSeg gate, const GateSeg *past_gates,
                                    int past_count,
                                    const GateSeg *future_gates,
                                    int future_count,
                                    const GateSeg *continue_gate,
                                    const GateSeg *continue_futures,
                                    int continue_future_count)
{
  const int fi = facingIndex(facing_deg_);

  /* 過去・未来のゲート辺を通過禁止し、現在の色だけ通過可能にする。
   * 未通過色のマスへの進入は禁止しない。 */
  RouteBlocks block_none;
  clearBlocked(block_none);
  blockGates(block_none, past_gates, past_count);
  blockGates(block_none, future_gates, future_count);

  /* 候補 A: a → b 。接近時だけ他端 b を禁止 */
  RouteBlocks block_to_a = block_none;
  blockPoint(block_to_a, gate.b, gate.a);

  int face_at_a = fi;
  const int cost_to_a =
      bfsCost(x_, y_, fi, gate.a.x, gate.a.y, block_to_a, &face_at_a);
  int face_ab_at_b = face_at_a;
  const int cost_a_to_b =
      (cost_to_a < 0)
          ? -1
          : bfsCost(gate.a.x, gate.a.y, face_at_a, gate.b.x, gate.b.y,
                    block_none, &face_ab_at_b);

  /* 候補 B: b → a */
  RouteBlocks block_to_b = block_none;
  blockPoint(block_to_b, gate.a, gate.b);

  int face_at_b = fi;
  const int cost_to_b =
      bfsCost(x_, y_, fi, gate.b.x, gate.b.y, block_to_b, &face_at_b);
  int face_ba_at_a = face_at_b;
  const int cost_b_to_a =
      (cost_to_b < 0)
          ? -1
          : bfsCost(gate.b.x, gate.b.y, face_at_b, gate.a.x, gate.a.y,
                    block_none, &face_ba_at_a);

  /* 出口から次ゲート入口までの最短も足して a→b / b→a を選ぶ。
   * 周回の黄は future が空なので、次周の赤 (continue_gate) を次ゲートとして使う。 */
  int extra_ab = 0;
  int extra_ba = 0;
  const GateSeg *next_gate = nullptr;
  if (future_count > 0) {
    next_gate = &future_gates[0];
  } else if (continue_gate != nullptr) {
    next_gate = continue_gate;
    (void)continue_futures;
    (void)continue_future_count;
  }
  if (next_gate != nullptr) {
    if (cost_a_to_b >= 0) {
      const QrPoint next_ab =
          nearerGateEnd(*next_gate, gate.b.x, gate.b.y, face_ab_at_b,
                        block_none);
      int face_tmp = face_ab_at_b;
      const int c = bfsCost(gate.b.x, gate.b.y, face_ab_at_b, next_ab.x,
                            next_ab.y, block_none, &face_tmp);
      if (c >= 0) {
        extra_ab = c;
      }
    }
    if (cost_b_to_a >= 0) {
      const QrPoint next_ba =
          nearerGateEnd(*next_gate, gate.a.x, gate.a.y, face_ba_at_a,
                        block_none);
      int face_tmp = face_ba_at_a;
      const int c = bfsCost(gate.a.x, gate.a.y, face_ba_at_a, next_ba.x,
                            next_ba.y, block_none, &face_tmp);
      if (c >= 0) {
        extra_ba = c;
      }
    }
  }

  const bool ab_ok = (cost_to_a >= 0 && cost_a_to_b >= 0);
  const bool ba_ok = (cost_to_b >= 0 && cost_b_to_a >= 0);
  if (!ab_ok && !ba_ok) {
    gMotors.coast();
    straight_running_ = false;
    beepError();
    return false;
  }

  QrPoint first  = gate.a;
  QrPoint second = gate.b;
  bool    use_ab = ab_ok;
  if (ab_ok && ba_ok) {
    /* 次色へ出やすい向きを優先。近い入口から入ると出口が次色と逆になり、
     * L では赤のあと青と反対へ出て回り道になる。 */
    if (next_gate != nullptr && extra_ab != extra_ba) {
      use_ab = (extra_ab < extra_ba);
    } else {
      use_ab = (cost_to_a + cost_a_to_b) <= (cost_to_b + cost_b_to_a);
    }
    {
      const QrPoint here = {x_, y_};
      /* 既に端上なら、その端から他端へ通過（開始(2,2)=赤a で b→a→戻りを防ぐ） */
      if (here.x == gate.a.x && here.y == gate.a.y) {
        use_ab = true;
      } else if (here.x == gate.b.x && here.y == gate.b.y) {
        use_ab = false;
      }
    }
  } else if (!ab_ok) {
    use_ab = false;
  }

  if (!use_ab) {
    first  = gate.b;
    second = gate.a;
  }

  /* 実走にも候補評価と同じゲート辺の禁止を適用する。 */
  RouteBlocks block_first = block_none;
  blockPoint(block_first, second, first);

  RouteBlocks block_second = block_none;

  int face_at_first = facingIndex(facing_deg_);
  const int n_to_first =
      bfsPlan(x_, y_, facingIndex(facing_deg_), first.x, first.y, block_first,
              nullptr, 0, &face_at_first);
  const bool continue_to_second =
      (n_to_first >= 0) &&
      planStartsWithFwd(first.x, first.y, face_at_first, second, block_second);
  if (!goTo(first, block_first, /*stop_at_end=*/!continue_to_second)) {
    return false;
  }

  int face_at_second = facingIndex(facing_deg_);
  const int n_to_second =
      bfsPlan(x_, y_, facingIndex(facing_deg_), second.x, second.y,
              block_second, nullptr, 0, &face_at_second);

  bool continue_after_second = false;
  if (n_to_second >= 0) {
    if (future_count > 0) {
      GateSeg past_next[4];
      int     past_n = 0;
      for (int i = 0; i < past_count && past_n < 4; ++i) {
        past_next[past_n++] = past_gates[i];
      }
      if (past_n < 4) {
        past_next[past_n++] = gate;
      }
      continue_after_second = passGateStartsWithFwd(
          second.x, second.y, face_at_second, future_gates[0], past_next,
          past_n, (future_count > 1) ? &future_gates[1] : nullptr,
          (future_count > 1) ? future_count - 1 : 0, continue_gate,
          continue_futures, continue_future_count);
    } else if (continue_gate != nullptr) {
      /* 周回末の黄のみ: 次周の赤が Fwd で始まるときだけ止めない。
       * 赤・青でここを見ると、次が黄なのに次周赤へつなぐ誤判定になる */
      continue_after_second = passGateStartsWithFwd(
          second.x, second.y, face_at_second, *continue_gate, nullptr, 0,
          continue_futures, continue_future_count);
    }
  }
  return goTo(second, block_second, /*stop_at_end=*/!continue_after_second);
}

bool Nansho3Scene::runRallyLap(GateSeg red, GateSeg blue, GateSeg yellow,
                               bool continue_to_next_lap)
{
  if (!isYAdjacent(red) || !isXAdjacent(blue) || !isYAdjacent(yellow)) {
    gMotors.coast();
    straight_running_ = false;
    beepError();
    return false;
  }

  const GateSeg after_red[2] = {blue, yellow};
  if (!passGateShortest(red, nullptr, 0, after_red, 2,
                   continue_to_next_lap ? &red : nullptr,
                   continue_to_next_lap ? after_red : nullptr,
                   continue_to_next_lap ? 2 : 0)) {
    return false;
  }
  onGatePassed();

  const GateSeg before_blue[1] = {red};
  const GateSeg after_blue[1]  = {yellow};
  if (!passGateShortest(blue, before_blue, 1, after_blue, 1,
                   continue_to_next_lap ? &red : nullptr,
                   continue_to_next_lap ? after_red : nullptr,
                   continue_to_next_lap ? 2 : 0)) {
    return false;
  }
  onGatePassed();

  const GateSeg before_yellow[2] = {red, blue};
  if (continue_to_next_lap) {
    /* 次周の赤通過と同じ禁止マスで、黄終点→赤が Fwd なら止めない */
    const GateSeg next_red_futures[2] = {blue, yellow};
    if (!passGateShortest(yellow, before_yellow, 2, nullptr, 0, &red,
                     next_red_futures, 2)) {
      return false;
    }
  } else {
    if (!passGateShortest(yellow, before_yellow, 2, nullptr, 0)) {
      return false;
    }
  }
  onGatePassed();
  return true;
}

void Nansho3Scene::prepareNextLap(GateSeg red, GateSeg blue, GateSeg yellow)
{
  (void)red;
  (void)blue;
  (void)yellow;
}

/* ---------------------------------------------------------------------
 * 指定周回後:
 *   1. 論理 +y（L=-90 / R=+90）へ向き直し、ラインを検知するまで直進
 *   2. ライン上で旋回（左右パワーは ExitTurn* で指定）→ LT
 *   3. 青を検知したら指定エンコーダ距離だけ直進して停止
 * --------------------------------------------------------------------- */
void Nansho3Scene::runExitStraightToBlackLine()
{
  gLineTracer.setEnabled(false);
  if (straight_running_) {
    gMotors.decelToStop(CoreStraightDecelMs, CoreStraightDecelStepMs,
                        /*coast_end=*/true);
    straight_running_ = false;
  } else {
    gMotors.coast();
  }

  /* x<=0 だと +y 直進がコース外寄りなので、同じ y のまま x=1 へ戻してから脱出 */
  if (x_ <= ExitMinXBeforeLeave) {
    RouteBlocks blocked;
    clearBlocked(blocked);
    const QrPoint ret = {ExitReturnX, y_};
    if (!goTo(ret, blocked, /*stop_at_end=*/true)) {
      return;
    }
  }

  dly_tsk(CoreTurnBeforeMs * 1000U);

  /* 1) +y へ向き → ラインまで直進 */
  facing_deg_       = ExitYLogicalHeadingDeg;
  const float y_abs = absoluteCardinalDeg(ExitYLogicalHeadingDeg);
  turnToAbsolute(y_abs);
  dly_tsk(CoreTurnAfterMs * 1000U);

  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(y_abs);
  {
    const int32_t enc_left_base  = gMotors.leftEncoderCount();
    const int32_t enc_right_base = gMotors.rightEncoderCount();
    accelToCruise(enc_left_base, enc_right_base, CoreStraightAccelDeg,
                  ExitYStraightPower);
  }

  /* y>=5 → 0ms、それ未満は (5-y)*PerYMs */
  uint32_t black_ignore_ms = 0U;
  if (y_ < ExitYBlackIgnoreBaseY) {
    black_ignore_ms = static_cast<uint32_t>(ExitYBlackIgnoreBaseY - y_) *
                      ExitYBlackIgnorePerYMs;
  }

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < ExitYStraightMaxMs) {
    if (elapsed_ms >= black_ignore_ms &&
        (gColorLAB.reflection() <= ExitBlackReflectionMax ||
         gColorLAB.is(COLOR_DETECT_BLACKLINE, true) ||
         gColorLAB.is(COLOR_DETECT_BLACK, true))) {
      break;
    }
    gMotors.tickCorrection();
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  /* 2) ライン上で旋回（左右パワーは ExitTurnLeft/RightPower）→ LT（青まで） */
  gLineTracer.setEnabled(false);
  dly_tsk(CoreTurnBeforeMs * 1000U);
  {
    const float right_target =
        Gyro::wrapDeg180(gGyro.heading() + ExitFirstRightTurnDeg);
    turnExitOneWheel(right_target);
  }
  dly_tsk(CoreTurnAfterMs * 1000U);

  gLineTracer.setEnabled(true);
  gLineTracer.setEdgeFollow(ExitTraceEdgeFollow);
  gLineTracer.setPid(ExitTraceKp, ExitTraceKi, ExitTraceKd, ExitTraceOutMax);
  gLineTracer.setScale(ExitTraceScale);
  gLineTracer.setBaseSpeed(ExitSlowTracePower, ExitSlowTracePower);

  elapsed_ms = 0U;
  while (elapsed_ms < ExitBlueTraceMaxMs) {
    gLineTracer.step();
    if (elapsed_ms >= ExitBlueIgnoreMs &&
        (gColorLAB.is(COLOR_DETECT_BLUE, true) ||
         gColorLAB.is(COLOR_DETECT_SKYBLUE, true))) {
      break;
    }
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  /* 3) 青検知後: 止まらず LT を切り、現在向きのまま指定距離直進して減速停止 */
  gLineTracer.setEnabled(false);
  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(gGyro.heading());
  {
    const int32_t enc_left_base  = gMotors.leftEncoderCount();
    const int32_t enc_right_base = gMotors.rightEncoderCount();
    int32_t cruise =
        ExitAfterBlueStraightDeg - ExitAfterBlueStraightDecelDeg;
    if (cruise < 0) {
      cruise = 0;
    }
    int32_t accel_end = CoreStraightAccelDeg;
    if (accel_end > cruise) {
      accel_end = cruise;
    }
    gGyro.setRateHeading(true);
    rampStraightPower(enc_left_base, enc_right_base, accel_end,
                      static_cast<int>(ExitSlowTracePower),
                      ExitAfterBlueStraightPower);
    cruiseUntilDeg(enc_left_base, enc_right_base, cruise);
    decelToStopByEncoder(enc_left_base, enc_right_base,
                         ExitAfterBlueStraightDeg, ExitAfterBlueStraightPower,
                         ExitAfterBlueStraightDecelDeg);
  }
  dly_tsk(StepMs * 1000U);
}

void Nansho3Scene::runExitAfterRally()
{
  gLineTracer.setEnabled(false);
  if (straight_running_) {
    gMotors.decelToStop(CoreStraightDecelMs, CoreStraightDecelStepMs,
                        /*coast_end=*/true);
    straight_running_ = false;
  } else {
    gMotors.coast();
  }
  dly_tsk(CoreTurnBeforeMs * 1000U);

  /* 1) Y軸方向（論理 +y）へ旋回 → 黒線まで直進 */
  facing_deg_       = ExitYLogicalHeadingDeg;
  const float y_abs = absoluteCardinalDeg(ExitYLogicalHeadingDeg);
  turnToAbsolute(y_abs);
  dly_tsk(CoreTurnAfterMs * 1000U);

  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(y_abs);
  {
    const int32_t enc_left_base  = gMotors.leftEncoderCount();
    const int32_t enc_right_base = gMotors.rightEncoderCount();
    accelToCruise(enc_left_base, enc_right_base, CoreStraightAccelDeg,
                  ExitYStraightPower);
  }

  /* y>=5 → 0ms、それ未満は (5-y)*PerYMs */
  uint32_t black_ignore_ms = 0U;
  if (y_ < ExitYBlackIgnoreBaseY) {
    black_ignore_ms = static_cast<uint32_t>(ExitYBlackIgnoreBaseY - y_) *
                      ExitYBlackIgnorePerYMs;
  }

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < ExitYStraightMaxMs) {
    if (elapsed_ms >= black_ignore_ms &&
        (gColorLAB.reflection() <= ExitBlackReflectionMax ||
         gColorLAB.is(COLOR_DETECT_BLACKLINE, true) ||
         gColorLAB.is(COLOR_DETECT_BLACK, true))) {
      break;
    }
    gMotors.tickCorrection();
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  /* 1' ) 黒線つかみ後: 右へ相対 +90° 旋回 */
  gLineTracer.setEnabled(false);
  dly_tsk(CoreTurnBeforeMs * 1000U);
  {
    const float right_target =
        Gyro::wrapDeg180(gGyro.heading() + ExitFirstRightTurnDeg);
    turnToAbsolute(right_target);
  }
  dly_tsk(CoreTurnAfterMs * 1000U);

  /* 2) 低速 LT → ジャイロ reset */
  gLineTracer.setEnabled(true);
  gLineTracer.setEdgeFollow(ExitTraceEdgeFollow);
  gLineTracer.setPid(ExitTraceKp, ExitTraceKi, ExitTraceKd, ExitTraceOutMax);
  gLineTracer.setScale(ExitTraceScale);
  gLineTracer.setBaseSpeed(ExitSlowTracePower, ExitSlowTracePower);
  for (elapsed_ms = 0U; elapsed_ms < ExitSlowTraceMs; elapsed_ms += StepMs) {
    gLineTracer.step();
    dly_tsk(StepMs * 1000U);
  }

  gMotors.stop();
  gLineTracer.setEnabled(false);
  gGyro.reset();
  dly_tsk(StepMs * 1000U);
  /* reset 後の現在向きを 0° とする。以降は絶対角のみ */
  cardinal_offset_ = 0.0f;
  facing_deg_ = 0.0f;

  /* 3) LT → ±17° 到達 */
  gLineTracer.setEnabled(true);
  gLineTracer.setEdgeFollow(ExitTraceEdgeFollow);
  gLineTracer.setPid(ExitAfter17TraceKp, ExitAfter17TraceKi,
                     ExitAfter17TraceKd, ExitAfter17TraceOutMax);
  gLineTracer.setScale(ExitAfter17TraceScale);
  gLineTracer.setBaseSpeed(ExitAfter17TracePower, ExitAfter17TracePower);

  elapsed_ms = 0U;
  while (elapsed_ms < ExitAfter17TraceMaxMs) {
    gLineTracer.step();
    if (gGyro.reached(ExitLtStopHeadingDeg, ExitLtStopToleranceDeg)) {
      break;
    }
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  gLineTracer.setEnabled(false);
  dly_tsk(StepMs * 1000U);

  /* 4) 到達角を目標に直進 → 黒線 */
  const float h17 = ExitLtStopHeadingDeg;
  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(h17);
  {
    const int32_t enc_left_base  = gMotors.leftEncoderCount();
    const int32_t enc_right_base = gMotors.rightEncoderCount();
    accelToCruise(enc_left_base, enc_right_base, CoreStraightAccelDeg,
                  ExitAfter17StraightPower);
  }

  elapsed_ms = 0U;
  while (elapsed_ms < ExitAfter17StraightMaxMs) {
    if (elapsed_ms >= ExitAfter17BlackIgnoreMs &&
        (gColorLAB.reflection() <= ExitBlackReflectionMax ||
         gColorLAB.is(COLOR_DETECT_BLACKLINE, true) ||
         gColorLAB.is(COLOR_DETECT_BLACK, true))) {
      break;
    }
    gMotors.tickCorrection();
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  /* 4' ) 黒線つかみ後: 絶対 +90° へ旋回 */
  gLineTracer.setEnabled(false);
  dly_tsk(CoreTurnBeforeMs * 1000U);
  turnToAbsolute(ExitSecondTurnAbsDeg);
  dly_tsk(CoreTurnAfterMs * 1000U);

  /* 5) SKYBLUE まで LT */
  gLineTracer.setEnabled(true);
  gLineTracer.setEdgeFollow(ExitTraceEdgeFollow);
  gLineTracer.setPid(ExitTraceKp, ExitTraceKi, ExitTraceKd, ExitTraceOutMax);
  gLineTracer.setScale(ExitTraceScale);
  gLineTracer.setBaseSpeed(ExitFinalTracePower, ExitFinalTracePower);

  elapsed_ms = 0U;
  while (elapsed_ms < ExitFinalTraceMaxMs) {
    gLineTracer.step();
    if (elapsed_ms >= ExitFinalSkyblueIgnoreMs &&
        gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
      break;
    }
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  gLineTracer.setEnabled(false);
}

void Nansho3Scene::testInitialTrace()
{
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(InitialSlowTracePower, InitialSlowTracePower);
  gLineTracer.setPid(InitialSlowTraceKp, InitialSlowTraceKi,
                     InitialSlowTraceKd, InitialSlowTraceOutMax);
  gLineTracer.setScale(InitialSlowTraceScale);
  gLineTracer.setEdgeFollow(InitialSlowTraceEdgeFollow);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < InitialSlowTraceMaxMs) {
    gLineTracer.step();

    if (elapsed_ms >= InitialSlowTraceWhiteIgnoreMs &&
        gColorLAB.reflection() >= InitialSlowTraceWhiteReflectionMin) {
      break;
    }

    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  gLineTracer.setEnabled(false);

  dly_tsk(InitialSlowTraceStopSettleMs * 1000U);

  /* 停止して揺れが落ち着いた向きを 0° にする */
  gGyro.reset();
  dly_tsk(StepMs * 1000U);
}

void Nansho3Scene::runInitialTest()
{
  waitLogConnected();
  gGyro.setRateHeading(false);
  testInitialTrace();
  facing_deg_ = 0.0f;
  cardinal_offset_ = InitialTurnOffsetDeg;
  heading_add_ = 0.0f;
  straight_running_ = false;
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);
  initialRun(false);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gGyro.setRateHeading(false);
  gMotors.stop();
}

void Nansho3Scene::initialRun(bool continue_after)
{
  gLineTracer.setEnabled(false);

  /* 1) バック（reset 後の絶対 0°）。一定パワーで距離まで */
  gGyro.setRateHeading(true);
  gMotors.setBalancePid(InitialBackBalanceKp, InitialBackBalanceKi,
                        InitialBackBalanceIMax);
  gMotors.setTargetHeading(0.0f);
  {
    const int32_t enc_left_base  = gMotors.leftEncoderCount();
    const int32_t enc_right_base = gMotors.rightEncoderCount();
    gMotors.run(-InitialBackPower, -InitialBackPower);
    cruiseUntilDeg(enc_left_base, enc_right_base, InitialBackDeg);
    gMotors.stop();
  }

  /* 2) 初回も CoreTurn と同じ準備・補正・左右Extra・ログ・終了待ちで旋回。
   * 論理方位は +x のまま、初期座標系の絶対角へ向ける。 */
  const float turn_target = Gyro::wrapDeg180(InitialTurnOffsetDeg);
  const int init_prefer = (InitialTurnOffsetDeg >= 0.0f) ? 1 : -1;
  turnCoreToAbsolute(turn_target, init_prefer);

  /* 3) 直進 */
  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        InitialStraightBalanceIMax,
                        InitialStraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
  {
    const int32_t enc_left_base  = gMotors.leftEncoderCount();
    const int32_t enc_right_base = gMotors.rightEncoderCount();
    int32_t accel_end = CoreStraightAccelDeg;
    int32_t cruise    = continue_after
                         ? InitialStraightDeg
                         : (InitialStraightDeg - InitialStraightDecelDeg);
    if (cruise < 0) {
      cruise = 0;
    }
    if (accel_end > cruise) {
      accel_end = cruise;
    }
    accelToCruise(enc_left_base, enc_right_base, accel_end, InitialStraightPower);
    if (continue_after) {
      cruiseUntilDeg(enc_left_base, enc_right_base, InitialStraightDeg);
      straight_running_ = true;
    } else {
      cruiseUntilDeg(enc_left_base, enc_right_base, cruise);
      decelToStopByEncoder(enc_left_base, enc_right_base, InitialStraightDeg,
                           InitialStraightPower, InitialStraightDecelDeg);
      straight_running_ = false;
      dly_tsk(StepMs * 1000U);
    }
  }
}

void Nansho3Scene::run()
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  waitLogConnected();
  /* Nansho2 が stp_cyc している。直進中の tickCorrection に必要 */
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);

  /* 単体(4): 低速LTで白まで＋ジャイロreset
   * 通し(0/5/6): Nansho2 終了位置から。LT は省略し向きを 0° にしてバックへ */
#if ETROBO_SCENE == 4
  testInitialTrace();
#else
  gGyro.reset();
  dly_tsk(StepMs * 1000U);
#endif

  /* 初期 QR・仮ゲート配置（当日公開パターンで差し替え）
   * ETROBO_COURSE_L: 0=L / 1=R。開始は LR とも (1,1)。赤=Y連続, 青=X連続, 黄=Y連続。
   * 同じ軸は1つ: Y連続 {{x},{y1,y2}} / X連続 {{x1,x2},{y}}。座標は 0..5。 */
  x_ = LapStartX;
  y_ = LapStartY;
#if ETROBO_COURSE_L
  /* Rコース（開始(1,1)が赤a。通過後に他端へ戻さない） */
  const GateSeg red    = {{3}, {3, 4}}; /* Y連続 */
  const GateSeg blue   = {{1, 2}, {2}}; /* X連続 */
  const GateSeg yellow = {{4}, {4, 5}}; /* Y連続 */
#else
  /* Lコース */
  const GateSeg red    = {{3}, {2, 3}}; /* Y連続 */
  const GateSeg blue   = {{0, 1}, {4}}; /* X連続 */
  const GateSeg yellow = {{4}, {0, 1}}; /* Y連続 */
#endif

  facing_deg_             = 0.0f;
  cardinal_offset_        = InitialTurnOffsetDeg;
  heading_add_            = 0.0f;
  straight_running_       = false;
  has_next_target_        = false;
  gates_since_gyro_reset_ = 0;
  memset(grid_, 0, sizeof(grid_));
  grid_[x_][y_] = 1;

  /* 次が Fwd なら initialRun の直進から止まらずつなぐ */
  const GateSeg after_red[2] = {blue, yellow};
  const bool continue_after_initial = passGateStartsWithFwd(
      x_, y_, facingIndex(facing_deg_), red, nullptr, 0, after_red, 2);
  initialRun(continue_after_initial);

  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  if (straight_running_) {
    gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
    gMotors.run(CoreRunStraightPower, CoreRunStraightPower);
  }

  for (int lap = 0; lap < RallyLapCount; ++lap) {
    if (lap > 0) {
      prepareNextLap(red, blue, yellow);
    }
    if (!runRallyLap(red, blue, yellow,
                     /*continue_to_next_lap=*/lap + 1 < RallyLapCount)) {
      return; // 経路不成立を通過済み扱いにして次色・退出へ進めない
    }
  }

  /* 指定周回終了後: ライン→右旋回→LT→青→エンコーダ直進 */
  runExitStraightToBlackLine();
  gLineTracer.setEnabled(false);
}

extern "C" void Nansho3Task(intptr_t exinf)
{
  (void)exinf;
  gNansho3.run();
  ext_tsk();
}
