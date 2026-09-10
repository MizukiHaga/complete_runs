#include "Nansho3.h"

#include "kernel_cfg.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ColorLAB.h"
#include "Gyro.h"
#include "LineTracer.h"
#include "Straight.h"

extern Gyro         gGyro;
extern Straight     gMotors;
extern LineTracer   gLineTracer;
extern ColorLAB     gColorLAB;
extern Nansho3Scene gNansho3;

/* facing_i: 0=0°(+x), 1=90°(-y), 2=180°(-x), 3=-90°(+y) */
static constexpr int kDx[4] = {1, 0, -1, 0};
static constexpr int kDy[4] = {0, -1, 0, 1};

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

void Nansho3Scene::clearBlocked(bool blocked[GridSize][GridSize])
{
  memset(blocked, 0, sizeof(bool) * GridSize * GridSize);
}

void Nansho3Scene::blockPoint(bool blocked[GridSize][GridSize], QrPoint p,
                              QrPoint except_goal)
{
  if (!inGrid(p.x, p.y)) {
    return;
  }
  if (p.x == except_goal.x && p.y == except_goal.y) {
    return;
  }
  blocked[p.x][p.y] = true;
}

void Nansho3Scene::blockGate(bool blocked[GridSize][GridSize], GateSeg g,
                             QrPoint except_goal)
{
  blockPoint(blocked, g.a, except_goal);
  blockPoint(blocked, g.b, except_goal);
}

void Nansho3Scene::blockFutures(bool blocked[GridSize][GridSize],
                                const GateSeg *future_gates, int future_count,
                                QrPoint except_goal)
{
  for (int i = 0; i < future_count; ++i) {
    blockGate(blocked, future_gates[i], except_goal);
  }
}

void Nansho3Scene::beepError() const
{
}

float Nansho3Scene::absoluteCardinalDeg(float logical_facing_deg) const
{
  /* 90° 刻み + HeadingOffsetDeg。旋回も直進も同じ量を載せる */
  return Gyro::wrapDeg180(snapTo90(logical_facing_deg + cardinal_offset_) +
                          HeadingOffsetDeg);
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
  while (gMotors.traveledDegFrom(left_base, right_base) < target_deg) {
    dly_tsk(CoreStraightStepMs * 1000U);
  }
}

void Nansho3Scene::driveOneCell(bool stop_after)
{
  gLineTracer.setEnabled(false);

  const int fi_ms = facingIndex(facing_deg_);
  const int32_t extra =
      (fi_ms == 1 || fi_ms == 3) ? CoreStraightYExtraDeg : CoreStraightXExtraDeg;
  const int pwr = CoreRunStraightPower;

  /* このマスの走り出しを 0 点にする。連続 Fwd の途中でも止めずに測れる。 */
  const int32_t enc_left_base  = gMotors.leftEncoderCount();
  const int32_t enc_right_base = gMotors.rightEncoderCount();

  if (!straight_running_) {
    gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
    gMotors.run(pwr, pwr);
    straight_running_ = true;
  }

  if (stop_after) {
    /* 1マス移動・連続の最後: 減速で進む分を残してクルーズしてから減速 */
    int32_t cruise = CoreStraightDeg + extra - CoreStraightDecelDeg;
    if (cruise < 0) {
      cruise = 0;
    }
    cruiseUntilDeg(enc_left_base, enc_right_base, cruise);
    gMotors.decelToStop(CoreStraightDecelMs, CoreStraightDecelStepMs,
                        /*coast_end=*/true);
    straight_running_ = false;
  } else {
    cruiseUntilDeg(enc_left_base, enc_right_base,
                   CoreStraightContinueDeg + extra);
  }

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

void Nansho3Scene::turnToAbsolute(float target_heading, float extra_early_deg)
{
  /* 旋回中だけハブ方位に追従。終わったら直進用 Q/R に戻す */
  gGyro.setKalmanNoise(KalmanTurnProcessNoise, KalmanTurnMeasNoise);

  float early = CoreTurnEarlyStopDeg + extra_early_deg;
  if (early < 0.0f) {
    early = 0.0f;
  }
  gMotors.turnToHeading(target_heading, CoreTurnToleranceDeg, CoreTurnPower,
                        CoreTurnPower, CoreTurnMaxMs, CoreTurnSlowApproachDeg,
                        CoreTurnSlowPower, early, /*coast_end=*/true);

  gGyro.setKalmanNoise(KalmanProcessNoise, KalmanMeasNoise);
}

/* 周回中のジャイロ引き直し。現在はどこからも呼んでいない
 * （1 周ごとの reset は入れない方針）。戻すときは prepareNextLap から呼ぶ。 */
void Nansho3Scene::resetGyroForLap()
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

  /* 今向いている向きを 0° にする */
  gGyro.reset();
  dly_tsk(StepMs * 1000U);

  /* 論理 0=+x / -90=+y / ±180=-x / +90=-y の絶対角を、この reset に合わせる */
  cardinal_offset_ = Gyro::wrapDeg180(-snapTo90(facing_deg_));
  gates_since_gyro_reset_ = 0;
  gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
}

void Nansho3Scene::onGatePassed()
{
  ++gates_since_gyro_reset_;
}

void Nansho3Scene::turnByAction(MoveAction action)
{
  gLineTracer.setEnabled(false);
  /* 減速は直前の Fwd のマス時間内。終端は coast（stop だと尾が上がる） */
  gMotors.coast();
  straight_running_ = false;
  dly_tsk(CoreTurnBeforeMs * 1000U);

  const float offset =
      (action == MoveAction::L90) ? -90.0f : 90.0f;
  facing_deg_ =
      facingDegFromIndex(facingIndex(facing_deg_ + offset));
  const float target = absoluteCardinalDeg(facing_deg_);
  const float extra_early =
      (action == MoveAction::L90) ? CoreTurnLeftEarlyExtraDeg : 0.0f;
  turnToAbsolute(target, extra_early);
  gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
  dly_tsk(CoreTurnAfterMs * 1000U);
}

int Nansho3Scene::bfsPlan(int sx, int sy, int sfacing_i, int gx, int gy,
                          const bool blocked[GridSize][GridSize], MoveAction *out_actions,
                          int max_actions, int *out_facing_i) const
{
  if (!inGrid(sx, sy) || !inGrid(gx, gy)) {
    return -1;
  }
  if (sfacing_i < 0 || sfacing_i >= FacingCount) {
    return -1;
  }
  /* ゴール自身が禁止だと到達不能 */
  if (blocked[gx][gy]) {
    return -1;
  }

  if (sx == gx && sy == gy) {
    if (out_facing_i != nullptr) {
      *out_facing_i = sfacing_i;
    }
    return 0;
  }

  bool visited[GridSize][GridSize][FacingCount];
  memset(visited, 0, sizeof(visited));

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

  int qx[StateCount];
  int qy[StateCount];
  int qf[StateCount];
  int qh = 0;
  int qt = 0;

  qx[qt] = sx;
  qy[qt] = sy;
  qf[qt] = sfacing_i;
  ++qt;
  visited[sx][sy][sfacing_i] = true;

  int  goal_f = -1;
  bool found  = false;

  while (qh < qt) {
    const int cx = qx[qh];
    const int cy = qy[qh];
    const int cf = qf[qh];
    ++qh;

    if (cx == gx && cy == gy) {
      goal_f = cf;
      found  = true;
      break;
    }

    const MoveAction cands[3] = {MoveAction::Fwd, MoveAction::L90,
                                 MoveAction::R90};
    for (int ci = 0; ci < 3; ++ci) {
      int nx = cx;
      int ny = cy;
      int nf = cf;
      const MoveAction act = cands[ci];

      if (act == MoveAction::Fwd) {
        nx = cx + kDx[cf];
        ny = cy + kDy[cf];
        if (!inGrid(nx, ny)) {
          continue;
        }
        /* 禁止マスへ直進しない（ゴールは呼び出し前に非禁止を保証） */
        if (blocked[nx][ny]) {
          continue;
        }
      } else if (act == MoveAction::L90) {
        nf = (cf + 3) % FacingCount;
      } else {
        nf = (cf + 1) % FacingCount;
      }

      if (visited[nx][ny][nf]) {
        continue;
      }
      visited[nx][ny][nf] = true;
      const int ni = nx * (GridSize * FacingCount) + ny * FacingCount + nf;
      parent_x[ni]      = static_cast<int8_t>(cx);
      parent_y[ni]      = static_cast<int8_t>(cy);
      parent_f[ni]      = static_cast<int8_t>(cf);
      parent_action[ni] = static_cast<uint8_t>(act);

      qx[qt] = nx;
      qy[qt] = ny;
      qf[qt] = nf;
      ++qt;
    }
  }

  if (!found) {
    return -1;
  }

  if (out_facing_i != nullptr) {
    *out_facing_i = goal_f;
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
                          const bool blocked[GridSize][GridSize], int *out_facing_i) const
{
  return bfsPlan(sx, sy, sfacing_i, gx, gy, blocked, nullptr, 0,
                 out_facing_i);
}

bool Nansho3Scene::planStartsWithFwd(int sx, int sy, int sfacing_i, QrPoint goal,
                                     const bool blocked[GridSize][GridSize]) const
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
    int past_count, const GateSeg *future_gates, int future_count) const
{
  bool block_to_a[GridSize][GridSize];
  clearBlocked(block_to_a);
  blockPoint(block_to_a, gate.b, gate.a);
  blockFutures(block_to_a, past_gates, past_count, gate.a);
  blockFutures(block_to_a, future_gates, future_count, gate.a);

  bool block_a_to_b[GridSize][GridSize];
  clearBlocked(block_a_to_b);
  blockFutures(block_a_to_b, past_gates, past_count, gate.b);
  blockFutures(block_a_to_b, future_gates, future_count, gate.b);

  int face_at_a = sfacing_i;
  const int cost_to_a =
      bfsCost(sx, sy, sfacing_i, gate.a.x, gate.a.y, block_to_a, &face_at_a);
  const int cost_a_to_b =
      (cost_to_a < 0)
          ? -1
          : bfsCost(gate.a.x, gate.a.y, face_at_a, gate.b.x, gate.b.y,
                    block_a_to_b, nullptr);

  bool block_to_b[GridSize][GridSize];
  clearBlocked(block_to_b);
  blockPoint(block_to_b, gate.a, gate.b);
  blockFutures(block_to_b, past_gates, past_count, gate.b);
  blockFutures(block_to_b, future_gates, future_count, gate.b);

  bool block_b_to_a[GridSize][GridSize];
  clearBlocked(block_b_to_a);
  blockFutures(block_b_to_a, past_gates, past_count, gate.a);
  blockFutures(block_b_to_a, future_gates, future_count, gate.a);

  int face_at_b = sfacing_i;
  const int cost_to_b =
      bfsCost(sx, sy, sfacing_i, gate.b.x, gate.b.y, block_to_b, &face_at_b);
  const int cost_b_to_a =
      (cost_to_b < 0)
          ? -1
          : bfsCost(gate.b.x, gate.b.y, face_at_b, gate.a.x, gate.a.y,
                    block_b_to_a, nullptr);

  const bool ab_ok = (cost_to_a >= 0 && cost_a_to_b >= 0);
  const bool ba_ok = (cost_to_b >= 0 && cost_b_to_a >= 0);
  if (!ab_ok && !ba_ok) {
    return false;
  }

  QrPoint first = gate.a;
  bool    use_ab = ab_ok;
  if (ab_ok && ba_ok) {
    use_ab = (cost_to_a + cost_a_to_b) <= (cost_to_b + cost_b_to_a);
  } else if (!ab_ok) {
    use_ab = false;
  }
  if (!use_ab) {
    first = gate.b;
  }

  bool block_first[GridSize][GridSize];
  clearBlocked(block_first);
  blockPoint(block_first, use_ab ? gate.b : gate.a, first);
  blockFutures(block_first, past_gates, past_count, first);
  blockFutures(block_first, future_gates, future_count, first);
  return planStartsWithFwd(sx, sy, sfacing_i, first, block_first);
}

void Nansho3Scene::goTo(QrPoint goal, const bool blocked[GridSize][GridSize],
                        bool stop_at_end)
{
  setNextTarget(goal);

  if (x_ == goal.x && y_ == goal.y) {
    if (stop_at_end && straight_running_) {
      gMotors.coast();
      straight_running_ = false;
    }
    onArrivedCell();
    return;
  }

  MoveAction actions[MaxPathLen];
  int        end_facing = facingIndex(facing_deg_);
  const int  n =
      bfsPlan(x_, y_, facingIndex(facing_deg_), goal.x, goal.y, blocked,
              actions, MaxPathLen, &end_facing);
  if (n < 0) {
    beepError();
    return;
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
}

void Nansho3Scene::passGateShortest(GateSeg gate, const GateSeg *past_gates,
                                    int past_count,
                                    const GateSeg *future_gates,
                                    int future_count)
{
  const int fi = facingIndex(facing_deg_);

  /* 候補 A: a → b 。接近時は b + 通過済 + 後続を禁止 */
  bool block_to_a[GridSize][GridSize];
  clearBlocked(block_to_a);
  blockPoint(block_to_a, gate.b, gate.a);
  blockFutures(block_to_a, past_gates, past_count, gate.a);
  blockFutures(block_to_a, future_gates, future_count, gate.a);

  bool block_a_to_b[GridSize][GridSize];
  clearBlocked(block_a_to_b);
  blockFutures(block_a_to_b, past_gates, past_count, gate.b);
  blockFutures(block_a_to_b, future_gates, future_count, gate.b);

  int face_at_a = fi;
  const int cost_to_a =
      bfsCost(x_, y_, fi, gate.a.x, gate.a.y, block_to_a, &face_at_a);
  const int cost_a_to_b =
      (cost_to_a < 0)
          ? -1
          : bfsCost(gate.a.x, gate.a.y, face_at_a, gate.b.x, gate.b.y,
                    block_a_to_b, nullptr);

  /* 候補 B: b → a */
  bool block_to_b[GridSize][GridSize];
  clearBlocked(block_to_b);
  blockPoint(block_to_b, gate.a, gate.b);
  blockFutures(block_to_b, past_gates, past_count, gate.b);
  blockFutures(block_to_b, future_gates, future_count, gate.b);

  bool block_b_to_a[GridSize][GridSize];
  clearBlocked(block_b_to_a);
  blockFutures(block_b_to_a, past_gates, past_count, gate.a);
  blockFutures(block_b_to_a, future_gates, future_count, gate.a);

  int face_at_b = fi;
  const int cost_to_b =
      bfsCost(x_, y_, fi, gate.b.x, gate.b.y, block_to_b, &face_at_b);
  const int cost_b_to_a =
      (cost_to_b < 0)
          ? -1
          : bfsCost(gate.b.x, gate.b.y, face_at_b, gate.a.x, gate.a.y,
                    block_b_to_a, nullptr);

  const bool ab_ok = (cost_to_a >= 0 && cost_a_to_b >= 0);
  const bool ba_ok = (cost_to_b >= 0 && cost_b_to_a >= 0);
  if (!ab_ok && !ba_ok) {
    beepError();
    return;
  }

  QrPoint first  = gate.a;
  QrPoint second = gate.b;
  bool    use_ab = ab_ok;
  if (ab_ok && ba_ok) {
    const int total_ab = cost_to_a + cost_a_to_b;
    const int total_ba = cost_to_b + cost_b_to_a;
    use_ab = (total_ab <= total_ba);
  } else if (!ab_ok) {
    use_ab = false;
  }

  if (!use_ab) {
    first  = gate.b;
    second = gate.a;
  }

  /* 実走: first へ → second へ。次が Fwd ならつなぐ。次が旋回なら
   * 連続の最後を CoreStraightDeg で止める（2マス→旋回で距離が変わる）。 */
  bool block_first[GridSize][GridSize];
  clearBlocked(block_first);
  blockPoint(block_first, second, first);
  blockFutures(block_first, past_gates, past_count, first);
  blockFutures(block_first, future_gates, future_count, first);

  bool block_second[GridSize][GridSize];
  clearBlocked(block_second);
  blockFutures(block_second, past_gates, past_count, second);
  blockFutures(block_second, future_gates, future_count, second);

  int face_at_first = facingIndex(facing_deg_);
  const int n_to_first =
      bfsPlan(x_, y_, facingIndex(facing_deg_), first.x, first.y, block_first,
              nullptr, 0, &face_at_first);
  const bool continue_to_second =
      (n_to_first >= 0) &&
      planStartsWithFwd(first.x, first.y, face_at_first, second, block_second);
  goTo(first, block_first, /*stop_at_end=*/!continue_to_second);

  int face_at_second = facingIndex(facing_deg_);
  const int n_to_second =
      bfsPlan(x_, y_, facingIndex(facing_deg_), second.x, second.y,
              block_second, nullptr, 0, &face_at_second);
  bool continue_after = false;
  if (future_count > 0 && n_to_second >= 0) {
    GateSeg past_next[4];
    int     past_n = 0;
    for (int i = 0; i < past_count && past_n < 4; ++i) {
      past_next[past_n++] = past_gates[i];
    }
    if (past_n < 4) {
      past_next[past_n++] = gate;
    }
    continue_after = passGateStartsWithFwd(
        second.x, second.y, face_at_second, future_gates[0], past_next, past_n,
        (future_count > 1) ? &future_gates[1] : nullptr,
        (future_count > 1) ? future_count - 1 : 0);
  }
  goTo(second, block_second, /*stop_at_end=*/!continue_after);
}

void Nansho3Scene::runRallyLap(GateSeg red, GateSeg blue, GateSeg yellow)
{
  if (!isYAdjacent(red) || !isXAdjacent(blue) || !isYAdjacent(yellow)) {
    beepError();
    return;
  }

  const GateSeg after_red[2] = {blue, yellow};
  passGateShortest(red, nullptr, 0, after_red, 2);
  onGatePassed();

  const GateSeg before_blue[1] = {red};
  const GateSeg after_blue[1]  = {yellow};
  passGateShortest(blue, before_blue, 1, after_blue, 1);
  onGatePassed();

  const GateSeg before_yellow[2] = {red, blue};
  passGateShortest(yellow, before_yellow, 2, nullptr, 0);
  onGatePassed();
}

void Nansho3Scene::prepareNextLap(GateSeg red, GateSeg blue, GateSeg yellow)
{
  gLineTracer.setEnabled(false);

  /* 赤・青・黄マスを避けて開始位置へ（黄の縦抜けで再通過しない） */
  bool blocked[GridSize][GridSize];
  clearBlocked(blocked);
  const QrPoint start = {LapStartX, LapStartY};
  blockGate(blocked, red, start);
  blockGate(blocked, blue, start);
  blockGate(blocked, yellow, start);
  goTo(start, blocked, /*stop_at_end=*/true);

  /* 向き直しはしない。開始位置に着いた向きのまま次の周へ入る。
   * ジャイロも引き直さない（周回中は最初の reset 基準のまま）。 */
}

/* ---------------------------------------------------------------------
 * 指定周回後: 論理 −90°（+y）へ向き直し、完全な黒を検知するまで直進して停止。
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
  dly_tsk(CoreTurnBeforeMs * 1000U);

  facing_deg_       = ExitYLogicalHeadingDeg;
  const float y_abs = absoluteCardinalDeg(ExitYLogicalHeadingDeg);
  turnToAbsolute(y_abs);
  dly_tsk(CoreTurnAfterMs * 1000U);

  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(y_abs);
  gMotors.run(ExitYStraightPower, ExitYStraightPower);

  /* 黒線ではなく完全な黒。ColorLAB::BlackReflectionMax 以下の反射光で判定 */
  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < ExitYStraightMaxMs) {
    if (elapsed_ms >= ExitYBlackIgnoreMs &&
        gColorLAB.is(COLOR_DETECT_BLACK, true)) {
      break;
    }
    gMotors.tickCorrection();
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }
  gMotors.stop();
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

  /* 1) Y軸方向（論理 −90°=+y）へ旋回 → 黒線まで直進 */
  facing_deg_       = ExitYLogicalHeadingDeg;
  const float y_abs = absoluteCardinalDeg(ExitYLogicalHeadingDeg);
  turnToAbsolute(y_abs);
  dly_tsk(CoreTurnAfterMs * 1000U);

  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(y_abs);
  gMotors.run(ExitYStraightPower, ExitYStraightPower);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < ExitYStraightMaxMs) {
    if (elapsed_ms >= ExitYBlackIgnoreMs &&
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
  gMotors.run(ExitAfter17StraightPower, ExitAfter17StraightPower);

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

  /* 低速LT終了時点の向きを 0° にする */
  gGyro.reset();
  dly_tsk(StepMs * 1000U);
}

void Nansho3Scene::initialRun()
{
  gLineTracer.setEnabled(false);

  /* 1) バック（reset 後の絶対 0°。Offset は最初の旋回後の直進だけ） */
  gMotors.setBalancePid(InitialBackBalanceKp, InitialBackBalanceKi,
                        InitialBackBalanceIMax);
  gMotors.setTargetHeading(0.0f);
  {
    const int32_t enc_left_base  = gMotors.leftEncoderCount();
    const int32_t enc_right_base = gMotors.rightEncoderCount();
    int32_t cruise = InitialBackDeg - InitialBackDecelDeg;
    if (cruise < 0) {
      cruise = 0;
    }
    gMotors.run(-InitialBackPower, -InitialBackPower);
    cruiseUntilDeg(enc_left_base, enc_right_base, cruise);
    gMotors.decelToStop(Straight::DecelMs, Straight::DecelStepMs,
                        /*coast_end=*/true);
  }
  dly_tsk(CoreTurnBeforeMs * 1000U);

  /* 2) 絶対 InitialTurnOffsetDeg（R=+90 / L=-90）+ HeadingOffsetDeg */
  const float turn_target =
      Gyro::wrapDeg180(InitialTurnOffsetDeg + HeadingOffsetDeg);
  turnToAbsolute(turn_target);
  dly_tsk(CoreTurnAfterMs * 1000U);

  /* 3) 直進。目標は旋回と同じ */
  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(turn_target);
  {
    const int32_t enc_left_base  = gMotors.leftEncoderCount();
    const int32_t enc_right_base = gMotors.rightEncoderCount();
    int32_t cruise = InitialStraightDeg - InitialStraightDecelDeg;
    if (cruise < 0) {
      cruise = 0;
    }
    gMotors.run(InitialStraightPower, InitialStraightPower);
    cruiseUntilDeg(enc_left_base, enc_right_base, cruise);
    gMotors.decelToStop(CoreStraightDecelMs, CoreStraightDecelStepMs,
                        /*coast_end=*/true);
  }
  dly_tsk(StepMs * 1000U);
}

void Nansho3Scene::run()
{
  gLineTracer.setEnabled(false);
  /* Nansho2 が stp_cyc している。直進中の tickCorrection に必要 */
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);
  gGyro.setKalmanNoise(KalmanProcessNoise, KalmanMeasNoise);

  /* 単体(4) / 3→1(13): 低速LTで白まで＋ジャイロreset
   * 通し(0/6): Nansho2 終了位置から。LT は省略し向きを 0° にしてバックへ */
#if ETROBO_SCENE == 4 || ETROBO_SCENE == 13
  testInitialTrace();
#else
  gGyro.reset();
  dly_tsk(StepMs * 1000U);
#endif
  initialRun();

  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);

  /* 初期 QR・仮ゲート配置（当日公開パターンで差し替え）
   * ETROBO_COURSE_L: 0=L / 1=R。開始は LR とも (2,2)。赤=Y連続, 青=X連続, 黄=Y連続。 */
  x_ = LapStartX;
  y_ = LapStartY;
#if ETROBO_COURSE_L
  /* Rコース */
  const GateSeg red    = {{5, 2}, {5, 3}}; /* Y連続 */
  const GateSeg blue   = {{4, 4}, {5, 4}}; /* X連続 */
  const GateSeg yellow = {{2, 4}, {2, 5}}; /* Y連続 */
#else
  /* Lコース */
  const GateSeg red    = {{4, 2}, {4, 3}}; /* Y連続 */
  const GateSeg blue   = {{4, 4}, {5, 4}}; /* X連続 */
  const GateSeg yellow = {{3, 4}, {3, 5}}; /* Y連続 */
#endif

  facing_deg_            = 0.0f;
  cardinal_offset_       = InitialTurnOffsetDeg;
  straight_running_      = false;
  has_next_target_       = false;
  gates_since_gyro_reset_ = 0;
  memset(grid_, 0, sizeof(grid_));
  grid_[x_][y_] = 1;

  for (int lap = 0; lap < RallyLapCount; ++lap) {
    if (lap > 0) {
      /* 黄終点から続けると赤の逆通過→青への進入が変わり、青ゲートを外す */
      prepareNextLap(red, blue, yellow);
    }
    runRallyLap(red, blue, yellow);
  }

  /* 指定周回終了後: +y 方向へ完全な黒まで直進して停止（Nansho1 移行なし） */
  runExitStraightToBlackLine();
  gLineTracer.setEnabled(false);
}

extern "C" void Nansho3Task(intptr_t exinf)
{
  (void)exinf;
  gNansho3.run();
  ext_tsk();
}
