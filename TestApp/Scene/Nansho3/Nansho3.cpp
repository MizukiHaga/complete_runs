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

float Nansho3Scene::headingWithBias(float logical_facing_deg) const
{
  return Gyro::wrapDeg180(logical_facing_deg + heading_bias_deg_);
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

void Nansho3Scene::driveOneCell(bool stop_after, bool long_run)
{
  gLineTracer.setEnabled(false);
  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax);
  gMotors.setTargetHeading(headingWithBias(facing_deg_));
  gMotors.run(CoreRunStraightPower, CoreRunStraightPower);

  const uint32_t ms =
      long_run ? CoreStraightAfterTurnOnceMs : CoreStraightMs;
  dly_tsk(ms * 1000U);
  if (stop_after) {
    gMotors.stop();
  }

  /* 旋回後 1 回目の Fwd を消費したらクリア（軸によらず共通） */
  just_turned_ = false;

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

void Nansho3Scene::turnByAction(MoveAction action)
{
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  const float offset =
      (action == MoveAction::L90) ? -90.0f : 90.0f;
  const float logical_target =
      Gyro::wrapDeg180(facing_deg_ + offset);
  gMotors.turnToHeading(headingWithBias(logical_target), CoreTurnToleranceDeg,
                        CoreTurnPower, CoreTurnPower, CoreTurnMaxMs);
  facing_deg_ = facingDegFromIndex(facingIndex(logical_target));
  just_turned_ = true;
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

void Nansho3Scene::goTo(QrPoint goal, const bool blocked[GridSize][GridSize],
                        bool stop_at_end)
{
  setNextTarget(goal);

  if (x_ == goal.x && y_ == goal.y) {
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
      /* 連続直進は止めない。次が旋回なら止める。最終マスは stop_at_end に従う */
      const bool stop_after =
          next_is_fwd ? false : (is_last ? stop_at_end : true);
      /* 旋回直後の 1 回目の Fwd だけ長くする（X/Y・±方向共通。goTo またぎも可） */
      const bool long_run =
          just_turned_ ||
          ((i > 0) && (actions[i - 1] != MoveAction::Fwd));
      driveOneCell(stop_after, long_run);
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

  /* 実走: first へ → second へ（通過済・後続は常に禁止）。1マスずつ。 */
  bool block_first[GridSize][GridSize];
  clearBlocked(block_first);
  blockPoint(block_first, second, first);
  blockFutures(block_first, past_gates, past_count, first);
  blockFutures(block_first, future_gates, future_count, first);
  goTo(first, block_first, /*stop_at_end=*/false);

  bool block_second[GridSize][GridSize];
  clearBlocked(block_second);
  blockFutures(block_second, past_gates, past_count, second);
  blockFutures(block_second, future_gates, future_count, second);
  goTo(second, block_second, /*stop_at_end=*/true);
}

void Nansho3Scene::runRallyLap(GateSeg red, GateSeg blue, GateSeg yellow)
{
  if (!isYAdjacent(red) || !isXAdjacent(blue) || !isYAdjacent(yellow)) {
    beepError();
    return;
  }

  const GateSeg after_red[2] = {blue, yellow};
  passGateShortest(red, nullptr, 0, after_red, 2);

  const GateSeg before_blue[1] = {red};
  const GateSeg after_blue[1]  = {yellow};
  passGateShortest(blue, before_blue, 1, after_blue, 1);

  const GateSeg before_yellow[2] = {red, blue};
  passGateShortest(yellow, before_yellow, 2, nullptr, 0);
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

  dly_tsk(StepMs * 1000U);
}

void Nansho3Scene::initialRun()
{
  gLineTracer.setEnabled(false);

  const float base_heading = gGyro.heading();

  /* 1) バック */
  gMotors.setTargetHeading(base_heading);
  gMotors.runFor(-InitialBackPower, -InitialBackPower, InitialBackDeg);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  /* 2) 相対 -90° 旋回 */
  const float turn_target =
      Gyro::wrapDeg180(base_heading + InitialTurnOffsetDeg);
  gMotors.turnToHeading(turn_target, CoreTurnToleranceDeg, CoreTurnPower,
                        CoreTurnPower, CoreTurnMaxMs);
  dly_tsk(StepMs * 1000U);

  /* 3) 直進（旋回後の向き） */
  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax);
  gMotors.setTargetHeading(turn_target);
  gMotors.runFor(InitialStraightPower, InitialStraightPower,
                 InitialStraightDeg);
  gMotors.stop();
  dly_tsk(StepMs * 1000U);

  /* 4) ジャイロreset → 現在向きを 0° に */
  gGyro.reset();
  facing_deg_  = 0.0f;
  just_turned_ = false;
  dly_tsk(StepMs * 1000U);
}

void Nansho3Scene::run()
{
  gLineTracer.setEnabled(false);

  /* 低速LTで白まで → バック → 旋回 → 直進 → ジャイロreset */
  //testInitialTrace();
  initialRun();

  gMotors.setBalancePid(StraightBalanceKp, StraightBalanceKi,
                        StraightBalanceIMax);

  /* 初期: Lコース例 右上 QR (1,1), 向き 0°（reset直後） */
  x_                 = 1;
  y_                 = 1;
  facing_deg_        = 0.0f;
  heading_bias_deg_  = 0.0f;
  just_turned_       = false;
  has_next_target_   = false;
  memset(grid_, 0, sizeof(grid_));
  grid_[x_][y_] = 1;

  /* 仮ゲート配置（当日公開パターンで差し替え） */
  const GateSeg red    = {{5, 4}, {5, 5}}; /* Y連続 */
  const GateSeg blue   = {{3, 3}, {4, 3}}; /* X連続 */
  const GateSeg yellow = {{2, 4}, {2, 5}}; /* Y連続 */

  for (int lap = 0; lap < RallyLapCount; ++lap) {
    runRallyLap(red, blue, yellow);
    /* 一周ごとに基準0°へ定数を加算（2周目以降の直進・旋回目標に反映） */
    heading_bias_deg_ =
        Gyro::wrapDeg180(heading_bias_deg_ + LapHeadingBiasPerLapDeg);
  }
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
