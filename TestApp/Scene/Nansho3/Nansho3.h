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

  /* 低速LT → 反射光で白検知して停止 */
  void testInitialTrace();

  /* バック → 相対-90°旋回 → 直進 → ジャイロreset（facing=0°） */
  void initialRun();

  /* 赤→青→黄の1周回（各ゲートは座標2点。通過は1マスずつ） */
  void runRallyLap(GateSeg red, GateSeg blue, GateSeg yellow);

  /* ETラリー周回数（課題ポイント最大は3周） */
  static constexpr int RallyLapCount = 3;
  /* 一周完了ごとに基準角度(0°)へ足す量 [deg] */
  static constexpr float LapHeadingBiasPerLapDeg = -3.0f;

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

  /* 原子動作
   * long_run: 旋回直後の 1 回目の Fwd */
  void driveOneCell(bool stop_after = true, bool long_run = false);
  void turnByAction(MoveAction action);

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
  void setNextTarget(QrPoint p);
  void onArrivedCell();
  void beepError() const;
  float headingWithBias(float logical_facing_deg) const;

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
   * 0°=+x / -90°=+y / ±180°=-x / +90°=-y
   * 初期: Lコース例 右上 QR = (1,1), facing=0° */
  int   grid_[GridSize][GridSize] = {};
  int   x_ = 1;
  int   y_ = 1;
  float facing_deg_ = 0.0f;
  /* 基準0°に対するバイアス。一周ごとに LapHeadingBiasPerLapDeg を加算 */
  float heading_bias_deg_ = 0.0f;
  /* 直前の動作が旋回なら true。次の 1 回目 Fwd を長くするため goTo をまたいで保持 */
  bool just_turned_ = false;

  /* 次目標座標（一致したときだけ通過進捗） */
  QrPoint next_target_ = {-1, -1};
  bool    has_next_target_ = false;

  /* === 直進 === */
  static constexpr int      CoreRunStraightPower = 75;
  static constexpr uint32_t CoreStraightMs = 600U;
  /* 旋回直後の 1 回目の Fwd の直進時間 */
  static constexpr uint32_t CoreStraightAfterTurnOnceMs = 600U;

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
  static constexpr int32_t  InitialSlowTraceWhiteReflectionMin = 70;

  /* === initialRun: バック → 相対-90°旋回 → 直進 → ジャイロreset === */
  static constexpr int      InitialBackPower     = 40;
  /* バック回転角 [deg]（左右モータ各） */
  static constexpr int32_t InitialBackDeg        = 930;
  static constexpr float    InitialTurnOffsetDeg = -90.0f;
  static constexpr int      InitialStraightPower = 50;
  /* 直進回転角 [deg]（左右モータ各） */
  static constexpr int32_t InitialStraightDeg    = 1900;

  /* 直進ヨー角 PI 補正（Straight::setBalancePid に渡す） */
  static constexpr float StraightBalanceKp   = 0.85f;
  static constexpr float StraightBalanceKi   = 0.16f;
  static constexpr float StraightBalanceIMax = 50.0f;

  /* === 旋回（turnToHeading） === */
  static constexpr float    CoreTurnToleranceDeg = 6.0f;
  static constexpr uint32_t CoreTurnMaxMs        = 3000U;
  static constexpr int      CoreTurnPower        = 40;
};

#ifdef __cplusplus
extern "C" {
#endif

void Nansho3Task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_NANSHO3_H_
