#include "Nansho3.h"

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
  /* 旋回用: 90° 刻みのみ（HeadingOffset なし） */
  return snapTo90(logical_facing_deg + cardinal_offset_);
}

float Nansho3Scene::straightTargetDeg(float logical_facing_deg) const
{
  const float extra = heading_offset_active_ ? HeadingOffsetDeg : 0.0f;
  return Gyro::wrapDeg180(absoluteCardinalDeg(logical_facing_deg) + extra);
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

void Nansho3Scene::driveOneCell(bool stop_after)
{
  gLineTracer.setEnabled(false);

  uint32_t extra = 0U;
  const int fi_ms = facingIndex(facing_deg_);
  if (fi_ms == 1 || fi_ms == 3) {
    extra = CoreStraightYExtraMs;
  } else {
    extra = CoreStraightXExtraMs;
  }
  const int pwr = CoreRunStraightPower;

  if (stop_after) {
    /* 1マス移動・連続の最後: CoreStraightMs 枠でクルーズしてから減速 */
    uint32_t total = CoreStraightMs + extra;
    uint32_t decel = CoreStraightDecelMs;
    if (decel > total) {
      decel = total;
    }
    const uint32_t cruise = total - decel;
    if (!straight_running_) {
      gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
      gMotors.run(pwr, pwr);
      straight_running_ = true;
    }
    if (cruise > 0U) {
      dly_tsk(cruise * 1000U);
    }
    gMotors.decelToStop(decel, CoreStraightDecelStepMs, /*coast_end=*/true);
    straight_running_ = false;
  } else {
    if (!straight_running_) {
      gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
      gMotors.run(pwr, pwr);
      straight_running_ = true;
    }
    dly_tsk((CoreStraightContinueMs + extra) * 1000U);
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
  /* 周回 reset 後は今の向きが 0°。初期直進の Offset は載せない */
  heading_offset_active_ = false;
  gates_since_gyro_reset_ = 0;
  gMotors.setTargetHeading(straightTargetDeg(facing_deg_));
}

void Nansho3Scene::onGatePassed()
{
  ++gates_since_gyro_reset_;
  if (gates_since_gyro_reset_ >= 2) {
    resetGyroForLap();
  }
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
  /* 最初の旋回後直進はここまで。以降の直進には Offset を足さない */
  heading_offset_active_ = false;
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
   * 連続の最後を CoreStraightMs で止める（2マス→旋回で距離が変わる）。 */
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

void Nansho3Scene::runExitStraightToBlack()
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

  const float target = absoluteCardinalDeg(ExitLogicalHeadingDeg);
  turnToAbsolute(target);
  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(straightTargetDeg(ExitLogicalHeadingDeg));
  dly_tsk(CoreTurnAfterMs * 1000U);

  gMotors.run(ExitStraightPower, ExitStraightPower);
  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < ExitStraightMaxMs) {
    if (elapsed_ms >= ExitBlackIgnoreMs &&
        (gColorLAB.reflection() <= ExitBlackReflectionMax ||
         gColorLAB.is(COLOR_DETECT_BLACKLINE, true) ||
         gColorLAB.is(COLOR_DETECT_BLACK, true))) {
      break;
    }
    dly_tsk(StepMs * 1000U);
    elapsed_ms += StepMs;
  }

  gMotors.stop();
  straight_running_ = false;
  runExitTurnRightAndLineTrace();
}

void Nansho3Scene::runExitTurnRightAndLineTrace()
{
  gLineTracer.setEnabled(false);
  gMotors.coast();
  dly_tsk(CoreTurnBeforeMs * 1000U);

  const float target = Gyro::wrapDeg180(gGyro.heading() + ExitTurnRightDeg);
  turnToAbsolute(target);
  dly_tsk(CoreTurnAfterMs * 1000U);

  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(ExitTracePower, ExitTracePower);
  gLineTracer.setPid(ExitTraceKp, ExitTraceKi, ExitTraceKd, ExitTraceOutMax);
  gLineTracer.setScale(ExitTraceScale);
  gLineTracer.setEdgeFollow(ExitTraceEdgeFollow);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < ExitTraceMaxMs) {
    gLineTracer.step();
    if (elapsed_ms >= ExitTraceWhiteIgnoreMs &&
        (gColorLAB.reflection() >= ExitTraceWhiteReflectionMin ||
         gColorLAB.is(COLOR_DETECT_WHITE, true))) {
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
  gMotors.runForMs(-InitialBackPower, -InitialBackPower, InitialBackMs,
                   Straight::DecelMs, Straight::DecelStepMs, /*coast_end=*/true);
  dly_tsk(CoreTurnBeforeMs * 1000U);

  /* 2) 絶対 −90°（Offset なし） */
  const float turn_target = InitialTurnOffsetDeg;
  turnToAbsolute(turn_target);
  dly_tsk(CoreTurnAfterMs * 1000U);

  /* 3) 直進。ここから HeadingOffsetDeg。次が同じ向きの Fwd ならそれにも乗る */
  heading_offset_active_ = true;
  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);
  gMotors.setTargetHeading(
      Gyro::wrapDeg180(turn_target + HeadingOffsetDeg));
  gMotors.runForMs(InitialStraightPower, InitialStraightPower,
                   InitialStraightMs, CoreStraightDecelMs,
                   CoreStraightDecelStepMs, /*coast_end=*/true);
  dly_tsk(StepMs * 1000U);
}

void Nansho3Scene::run()
{
  gLineTracer.setEnabled(false);
  gGyro.setKalmanNoise(KalmanProcessNoise, KalmanMeasNoise);

  /* 低速LTで白まで（終了時にジャイロreset）→ バック → 旋回 → 直進 */
  testInitialTrace();
  initialRun();

  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax, StraightBalanceDeadbandDeg);

  /* 初期: Lコース例 右上 QR = (2,2)。論理 0°=+x */
  x_                 = 2;
  y_                 = 2;
  facing_deg_            = 0.0f;
  cardinal_offset_       = InitialTurnOffsetDeg;
  straight_running_      = false;
  has_next_target_       = false;
  gates_since_gyro_reset_ = 0;
  /* heading_offset_active_ は initialRun のまま。同じ向きの Fwd が続く間だけ Offset */
  memset(grid_, 0, sizeof(grid_));
  grid_[x_][y_] = 1;

  /* 仮ゲート配置（当日公開パターンで差し替え） */
  const GateSeg red    = {{4, 4}, {4, 5}}; /* Y連続 */
  const GateSeg blue   = {{5, 3}, {6, 3}}; /* X連続 */
  const GateSeg yellow = {{2, 5}, {2, 6}}; /* Y連続 */

  for (int lap = 0; lap < RallyLapCount; ++lap) {
    runRallyLap(red, blue, yellow);
  }
  runExitStraightToBlack();
}

extern "C" void Nansho3Task(intptr_t exinf)
{
  (void)exinf;
  gNansho3.run();
  /*
  gLineTracer.setEnabled(false);
  gMotors.setBalancePid( 0.85f, 0.1f, 50.0f);
  gMotors.run(75, 75);
  dly_tsk(700 * 1000U);
  gMotors.stop();
  */
  ext_tsk();
}
