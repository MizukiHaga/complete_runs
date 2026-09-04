#ifndef TESTAPP_NANSHO3_H_
#define TESTAPP_NANSHO3_H_

#include <kernel.h>
#include <stdint.h>

#include "app.h"

/* 仮想 QR 座標点 */
struct QrPoint {
  int x;
  int y;
};

/* ゲート両端（通過の入り順はアルゴリズムが決定）。
 * 端点は軸方向に連続する2点。通過は1マスずつ。 */
struct GateSeg {
  QrPoint a;
  QrPoint b;
};

class Nansho3Scene {
public:
  void run();

  /* 低速LT → 反射光で白検知して停止 → ジャイロreset */
  void testInitialTrace();

  /* バック → 絶対-90°旋回 → 直進（ジャイロresetしない） */
  void initialRun();

  /* 赤→青→黄の1周回（各ゲートは座標2点。通過は1マスずつ） */
  void runRallyLap(GateSeg red, GateSeg blue, GateSeg yellow);

  /* 指定周回後: 論理 −90°（+y）へ旋回 → 直進 → 黒線で停止 */
  void runExitStraightToBlack();

  /* 黒線停止後: 右旋回 → ライントレース（白で停止） */
  void runExitTurnRightAndLineTrace();

  /* ETラリー周回数（課題ポイント最大は3周） */
  static constexpr int RallyLapCount = 3;
  /* 直進中: Q↓R↑。旋回中は KalmanTurn* に切り替えて打ち切り遅れを防ぐ */
  static constexpr float KalmanProcessNoise = 8.0f;
  static constexpr float KalmanMeasNoise    = 100.0f;
  static constexpr float KalmanTurnProcessNoise = 20.0f;
  static constexpr float KalmanTurnMeasNoise    = 32.0f;

private:
  enum class MoveAction : uint8_t {
    Fwd = 0,
    L90 = 1,
    R90 = 2,
  };

  static constexpr int GridSize = 8;
  static constexpr int FacingCount = 4;
  static constexpr int StateCount = GridSize * GridSize * FacingCount;
  static constexpr int MaxPathLen = 64;

  /* 原子動作。stop_after なら CoreStraightMs 枠で減速。連続は ContinueMs */
  void driveOneCell(bool stop_after = true);
  void turnByAction(MoveAction action);
  /* 90° 刻みへその場旋回（追い込みなし） */
  void turnToAbsolute(float target_heading, float extra_early_deg = 0.0f);
  /* 停止 → 現在向きを 0° にし cardinal_offset_ を更新 */
  void resetGyroForLap();
  /* ゲート通過後。2回ごとに resetGyroForLap */
  void onGatePassed();

  /* 次目標へ向き付き最短で移動（BFS）。blocked は通行禁止マス。
   * stop_at_end=false なら最終直進後も停止せず、次の goTo へ連続走行可能。 */
  void goTo(QrPoint goal, const bool blocked[GridSize][GridSize],
            bool stop_at_end = true);
  int  bfsCost(int sx, int sy, int sfacing_i, int gx, int gy,
               const bool blocked[GridSize][GridSize],
               int *out_facing_i) const;
  int  bfsPlan(int sx, int sy, int sfacing_i, int gx, int gy,
               const bool blocked[GridSize][GridSize], MoveAction *out_actions,
               int max_actions, int *out_facing_i) const;

  void passGateShortest(GateSeg gate, const GateSeg *past_gates,
                        int past_count, const GateSeg *future_gates,
                        int future_count);
  /* 指定位置から goal へ行く計画の先頭が Fwd か */
  bool planStartsWithFwd(int sx, int sy, int sfacing_i, QrPoint goal,
                         const bool blocked[GridSize][GridSize]) const;
  /* 指定位置からゲート通過を始めたとき、最初の動作が Fwd か */
  bool passGateStartsWithFwd(int sx, int sy, int sfacing_i, GateSeg gate,
                             const GateSeg *past_gates, int past_count,
                             const GateSeg *future_gates,
                             int future_count) const;
  void setNextTarget(QrPoint p);
  void onArrivedCell();
  void beepError() const;
  /* 論理方位 → 現在のジャイロ枠での絶対角（90° 刻み。旋回は HeadingOffset なし） */
  float absoluteCardinalDeg(float logical_facing_deg) const;
  /* 直進 PI の目標。heading_offset_active_ のときだけ刻み角 + HeadingOffsetDeg */
  float straightTargetDeg(float logical_facing_deg) const;

  static void clearBlocked(bool blocked[GridSize][GridSize]);
  static void blockPoint(bool blocked[GridSize][GridSize], QrPoint p,
                         QrPoint except_goal);
  static void blockGate(bool blocked[GridSize][GridSize], GateSeg g,
                        QrPoint except_goal);
  static void blockFutures(bool blocked[GridSize][GridSize],
                           const GateSeg *future_gates, int future_count,
                           QrPoint except_goal);

  static float snapTo90(float heading_deg);
  static int   facingIndex(float heading_deg);
  static float facingDegFromIndex(int facing_i);
  static bool  inGrid(int x, int y);
  static bool  isYAdjacent(GateSeg g);
  static bool  isXAdjacent(GateSeg g);

  static constexpr uint32_t StepMs = 1U;

  /* === 仮想 QR 格子 ==============================================
   * 範囲: x,y = 0..7（外周に仮想座標を1列追加）
   * 論理: 0°=+x / -90°=+y / ±180°=-x / +90°=-y
   * 絶対角 = snap(論理 + cardinal_offset_)。
   * LT reset 後は -90。ゲート2つ通過ごとの reset で向きに合わせて更新。
   * 初期: Lコース例 右上 QR = (2,2), facing=0° */
  int   grid_[GridSize][GridSize] = {};
  int   x_ = 2;
  int   y_ = 2;
  float facing_deg_ = 0.0f;
  /* 論理 → 絶対。LT 後 -90。ゲート2つごと reset 後は -snap(facing) */
  float cardinal_offset_ = -90.0f;
  /* 連続直進中は目標角・積分を触らない */
  bool straight_running_ = false;
  /* 最初の旋回後の直進（とそのまま続く Fwd）だけ HeadingOffsetDeg を足す */
  bool heading_offset_active_ = false;
  /* 前回ジャイロ reset からのゲート通過数（2で reset） */
  int gates_since_gyro_reset_ = 0;

  /* 次目標座標（一致したときだけ通過進捗） */
  QrPoint next_target_ = {-1, -1};
  bool    has_next_target_ = false;

  /* === 直進 ===
   * 1マス移動と連続の最後は CoreStraightMs（枠内で減速）。
   * 連続 Fwd（減速なし）は CoreStraightContinueMs。 */
  static constexpr int      CoreRunStraightPower = 75;
  static constexpr uint32_t CoreStraightMs = 920U;
  /* 停止する直進の終端でパワーを落とす時間（尾が浮かないように） */
  static constexpr uint32_t CoreStraightDecelMs = 400U;
  /* 台形減速の 1 段 [ms]。大きいほど 1 ステップの減少幅が大きい（急減速） */
  static constexpr uint32_t CoreStraightDecelStepMs = 100U;
  /* 連続 Fwd（減速なし）1マスの満速度時間。短いなら減らす、マスを越えるなら減らす */
  static constexpr uint32_t CoreStraightContinueMs = 670U;
  /* ±90°（論理 +y/-y）の直進に足す時間。Y が短いなら増やす */
  static constexpr uint32_t CoreStraightYExtraMs = 0U;
  /* 0°/±180°（論理 +x/-x）の直進に足す時間。水平が短いなら増やす */
  static constexpr uint32_t CoreStraightXExtraMs = 0U;

  /* === testInitialTrace: 低速LT → 反射光白でジャイロreset === */
  static constexpr float    InitialSlowTracePower  = 30.0f;
  static constexpr float    InitialSlowTraceKp     = 0.30f;
  static constexpr float    InitialSlowTraceKi     = 0.01f;
  static constexpr float    InitialSlowTraceKd     = 0.8f;
  static constexpr float    InitialSlowTraceOutMax = 100.0f;
  static constexpr float    InitialSlowTraceScale  = 1.0f;
  static constexpr bool     InitialSlowTraceEdgeFollow =
      (ETROBO_COURSE_L != 0); /* L=左エッジ, R=右エッジ */
  static constexpr uint32_t InitialSlowTraceWhiteIgnoreMs = 200U;
  static constexpr uint32_t InitialSlowTraceMaxMs         = 15000U;
  /* 反射光がこの値以上なら白（ColorLAB::WhiteReflectionMin と同じ系統） */
  static constexpr int32_t  InitialSlowTraceWhiteReflectionMin = 80;

  /* === initialRun: バック → 絶対-90°旋回 → 直進（ジャイロresetしない） === */
  static constexpr int      InitialBackPower     = 40;
  static constexpr uint32_t InitialBackMs        = 950U;
  static constexpr float    InitialBackBalanceKp   = 3.0f;
  static constexpr float    InitialBackBalanceKi   = 0.1f;
  static constexpr float    InitialBackBalanceIMax = 50.0f;
  static constexpr float    InitialTurnOffsetDeg = -90.0f;
  /* 最初の旋回の直後の直進に足す固定量 [deg]。CW 正。右ずれなら負（左へ）。
   * その直進のあと曲がらず Fwd が続く間も同じ。次の旋回以降は 0。旋回目標には入れない。 */
  static constexpr float    HeadingOffsetDeg     = -3.0f;
  static constexpr int      InitialStraightPower = 50;
  static constexpr uint32_t InitialStraightMs    = 1640U;

  /* 直進ヨー角 PI 補正（Straight::setBalancePid に渡す）
   * I は定常バイアス用。大きく長く残すと水平・垂直とも trim がばらつく。 */
  static constexpr float StraightBalanceKp         = 2.3f;
  static constexpr float StraightBalanceKi         = 0.1f;
  static constexpr float StraightBalanceIMax       = 18.0f;
  /* |error| がこれ以下は補正しない。狭すぎるとノイズで左へ切り続ける */
  static constexpr float StraightBalanceDeadbandDeg = 0.8f;

  /* === 旋回（両輪ピボット） === */
  static constexpr int      CoreTurnPower           = 40;
  /* 開始時に |目標−h| がこれ以下なら旋回しない。
   * 0 以外だと、その角のずれを着いたことにして次の基準になる */
  static constexpr float    CoreTurnToleranceDeg    = 0.0f;
  static constexpr float    CoreTurnSlowApproachDeg = 30.0f;
  static constexpr int      CoreTurnSlowPower       = 33;
  static constexpr uint32_t CoreTurnMaxMs           = 8000U;
  /* 停止してから旋回を始めるまでの待ち [ms] */
  static constexpr uint32_t CoreTurnBeforeMs        = 200U;
  /* 経過角打ち切りをこの角手前で止める（慣性＋h 遅れの行き過ぎ） */
  static constexpr float    CoreTurnEarlyStopDeg    = 3.0f;
  /* 左旋回だけさらに手前で止める（CCW 行き過ぎが積み上がる対策） */
  static constexpr float    CoreTurnLeftEarlyExtraDeg = 6.0f;
  /* 旋回完了後、直進に入る前の待ち [ms] */
  static constexpr uint32_t CoreTurnAfterMs         = 300U;

  /* === 指定周回後: 論理 −90°（+y）へ旋回 → 直進 → 黒線停止
   * LT reset 枠では絶対 −180° */
  static constexpr float    ExitLogicalHeadingDeg     = -90.0f;
  static constexpr int      ExitStraightPower         = 50;
  static constexpr uint32_t ExitStraightMaxMs         = 8000U;
  static constexpr uint32_t ExitBlackIgnoreMs         = 200U;
  /* 反射光がこの値以下なら黒線（Nansho2 のライン停止と同系統） */
  static constexpr int32_t  ExitBlackReflectionMax    = 30;
  /* 黒線停止後の右旋回量 [deg]（現在角 + この値。CW 正） */
  static constexpr float    ExitTurnRightDeg          = 90.0f;
  static constexpr float    ExitTracePower            = 40.0f;
  static constexpr float    ExitTraceKp               = 0.35f;
  static constexpr float    ExitTraceKi               = 0.01f;
  static constexpr float    ExitTraceKd               = 0.8f;
  static constexpr float    ExitTraceOutMax           = 100.0f;
  static constexpr float    ExitTraceScale            = 1.0f;
  static constexpr bool     ExitTraceEdgeFollow =
      (ETROBO_COURSE_L != 0); /* L=左エッジ, R=右エッジ */
  static constexpr uint32_t ExitTraceWhiteIgnoreMs    = 400U;
  static constexpr uint32_t ExitTraceMaxMs            = 15000U;
  static constexpr int32_t  ExitTraceWhiteReflectionMin = 80;
};

#ifdef __cplusplus
extern "C" {
#endif

void Nansho3Task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_NANSHO3_H_
