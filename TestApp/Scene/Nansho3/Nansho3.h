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

  /* 単体起動用: 低速LT → 反射光で白検知して停止 → ジャイロreset
   * 通し(ETROBO_SCENE 0/5/6)では呼ばず、initialRun（バック）から開始する
   * SCENE 4/13 では呼ぶ */
  void testInitialTrace();

  /* バック → 絶対-90°旋回 → 直進（ジャイロresetしない） */
  void initialRun();

  /* 赤→青→黄の1周回（各ゲートは座標2点。通過は1マスずつ） */
  void runRallyLap(GateSeg red, GateSeg blue, GateSeg yellow);

  /* 次周回のため開始マス (LapStart)・論理 0° へ戻す（ゲートマスは避ける） */
  void prepareNextLap(GateSeg red, GateSeg blue, GateSeg yellow);

  /* 指定周回後の退出:
   * Y軸直進→黒線→右旋回(+90相対) → 低速LT → ジャイロreset →
   * LT(±17°) → その角で直進→黒線→絶対+90°旋回 → LT(SKYBLUE) */
  void runExitAfterRally();

  /* 指定周回後: 論理 −90°（+y）へ向き直し、完全な黒を検知するまで直進して停止。 */
  void runExitStraightToBlackLine();

  /* ETラリー周回数（課題ポイント最大は3周） */
  static constexpr int RallyLapCount = 3;
  /* 旋回中: Q↑R↓。ハブの方位角にほぼそのまま追従し、打ち切り遅れを防ぐ */
  static constexpr float KalmanTurnProcessNoise = 800.0f;
  static constexpr float KalmanTurnMeasNoise    = 20.0f;
  /* 直進中も旋回と同じ速い設定にする。
   * 予測に使う角速度と観測のハブ方位角は同じ信号（ハブ側が同じ角速度を
   * 積分したもの）なので、Q を下げても精度は上がらず遅れだけが増える。
   * 遅れると段差・スリップへの直進 PI の反応が鈍るため、速い側に寄せる。
   * ノイズは StraightBalanceDeadbandDeg で切る。 */
  static constexpr float KalmanProcessNoise = KalmanTurnProcessNoise;
  static constexpr float KalmanMeasNoise    = KalmanTurnMeasNoise;

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
  /* 各周回の開始 QR（初期位置と同じ） */
  static constexpr int LapStartX = 2;
  static constexpr int LapStartY = 2;

  /* 原子動作。stop_after なら CoreStraightDeg 枠で減速。連続は ContinueDeg */
  void driveOneCell(bool stop_after = true);
  /* 走り出しの 0 点から target_deg 進むまで待つ（モータは止めない）。 */
  static void cruiseUntilDeg(int32_t left_base, int32_t right_base,
                             int32_t target_deg);
  void turnByAction(MoveAction action);
  /* 90° 刻みへその場旋回（追い込みなし） */
  void turnToAbsolute(float target_heading, float extra_early_deg = 0.0f);
  /* 停止 → 現在向きを 0° にし cardinal_offset_ を更新。
   * 1 周ごとの reset は行わない方針なので、現在は未使用。 */
  void resetGyroForLap();
  /* ゲート通過後（ジャイロresetによる停止はしない） */
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
  /* 論理方位 → 現在のジャイロ枠での絶対角（90° 刻み + HeadingOffsetDeg） */
  float absoluteCardinalDeg(float logical_facing_deg) const;
  /* 直進 PI の目標。旋回目標と同じ値 */
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
   * LT reset 後は InitialTurnOffsetDeg。周回中のジャイロresetは行わない。
   * 初期: 右上 QR = (2,2), facing=0° */
  int   grid_[GridSize][GridSize] = {};
  int   x_ = 2;
  int   y_ = 2;
  float facing_deg_ = 0.0f;
  /* 論理 → 絶対。LT 後 InitialTurnOffsetDeg */
  float cardinal_offset_ = InitialTurnOffsetDeg;
  /* 連続直進中は目標角・積分を触らない */
  bool straight_running_ = false;
  /* 前回ジャイロ reset からのゲート通過数（参考用） */
  int gates_since_gyro_reset_ = 0;

  /* 次目標座標（一致したときだけ通過進捗） */
  QrPoint next_target_ = {-1, -1};
  bool    has_next_target_ = false;

  /* === 直進 ===
   * 1マスの長さはエンコーダ距離で決める。距離は左右エンコーダ [deg] の平均で、
   * 車輪 56mm なら 360deg ≒ 176mm（約 0.49mm/deg）。
   * 1マス移動と連続の最後は CoreStraightDeg（枠内で減速）。
   * 連続 Fwd（減速なし）は CoreStraightContinueDeg。 */
  static constexpr int      CoreRunStraightPower = 75;
  /* 減速して止まる 1マスの距離 [deg]（減速中に進む分を含む） */
  static constexpr int32_t  CoreStraightDeg = 500;
  /* 上のうち減速中に進むと見込む距離 [deg]。ここまで来たら減速に入る。
   * 行き過ぎるなら増やす、手前で止まるなら減らす。 */
  static constexpr int32_t  CoreStraightDecelDeg = 150;
  /* 停止する直進の終端でパワーを落とす時間（尾が浮かないように） */
  static constexpr uint32_t CoreStraightDecelMs = 400U;
  /* 台形減速の 1 段 [ms]。大きいほど 1 ステップの減少幅が大きい（急減速） */
  static constexpr uint32_t CoreStraightDecelStepMs = 100U;
  /* 連続 Fwd（減速なし）1マスの距離 [deg]。マスを越えるなら減らす */
  static constexpr int32_t  CoreStraightContinueDeg = 500;
  /* ±90°（論理 +y/-y）の直進に足す距離 [deg]。Y が短いなら増やす */
  static constexpr int32_t  CoreStraightYExtraDeg = 0;
  /* 0°/±180°（論理 +x/-x）の直進に足す距離 [deg]。水平が短いなら増やす */
  static constexpr int32_t  CoreStraightXExtraDeg = 0;
  /* 距離を見に行く間隔 [ms] */
  static constexpr uint32_t CoreStraightStepMs = 1U;

  /* === testInitialTrace: 低速LT → 反射光白でジャイロreset === */
  static constexpr float    InitialSlowTracePower  = 30.0f;
  static constexpr float    InitialSlowTraceKp     = 0.30f;
  static constexpr float    InitialSlowTraceKi     = 0.01f;
  static constexpr float    InitialSlowTraceKd     = 0.8f;
  static constexpr float    InitialSlowTraceOutMax = 100.0f;
  static constexpr float    InitialSlowTraceScale  = 1.0f;
  static constexpr bool     InitialSlowTraceEdgeFollow =
      (ETROBO_COURSE_L == 0); /* L=左エッジ, R=右エッジ */
  static constexpr uint32_t InitialSlowTraceWhiteIgnoreMs = 200U;
  static constexpr uint32_t InitialSlowTraceMaxMs         = 15000U;
  /* 反射光がこの値以上なら白（ColorLAB::WhiteReflectionMin と同じ系統） */
  static constexpr int32_t  InitialSlowTraceWhiteReflectionMin = 80;

  /* === initialRun: バック → 絶対-90°旋回 → 直進（ジャイロresetしない） === */
  static constexpr int      InitialBackPower     = 40;
  /* 減速中に進む分を含めたバック距離 [deg]。左右エンコーダの絶対値平均 */
  static constexpr int32_t  InitialBackDeg       = 190;
  /* 上のうち減速中に進むと見込む距離 [deg] */
  static constexpr int32_t  InitialBackDecelDeg  = 155;
  static constexpr float    InitialBackBalanceKp   = 5.0f; //テスト3.0
  static constexpr float    InitialBackBalanceKi   = 0.1f;
  static constexpr float    InitialBackBalanceIMax = 50.0f;
  /* バック後に向く絶対角。論理座標系をここで張るので、cardinal_offset_ の
   * 初期値でもある。R と L で向きが逆。 */
  static constexpr float    InitialTurnOffsetDeg =
      ETROBO_COURSE_L ? 90.0f : -90.0f;
  /* 目標角に足す固定量 [deg]。CW 正。右ずれなら負（左へ）。
   * 直進も旋回も、初期直進から周回中のマス移動まですべて同じ量。 */
  static constexpr float    HeadingOffsetDeg     = 0.0f;
  static constexpr int      InitialStraightPower = 50;
  /* 減速中に進む分を含めた直進距離 [deg] */
  static constexpr int32_t  InitialStraightDeg      = 670;
  /* 上のうち減速中に進むと見込む距離 [deg] */
  static constexpr int32_t  InitialStraightDecelDeg = 100;

  /* 直進ヨー角 PI 補正（Straight::setBalancePid に渡す）
   * I は定常バイアス用。大きく長く残すと水平・垂直とも trim がばらつく。 */
  static constexpr float StraightBalanceKp         = 3.0f;
  static constexpr float StraightBalanceKi         = 0.07f;
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

  /* === 指定周回後の退出シーケンス === */
  /* 論理 −90°（+y）へ向いてから黒線まで直進 */
  static constexpr float    ExitYLogicalHeadingDeg   = -90.0f;
  static constexpr int      ExitYStraightPower       = 70;
  static constexpr uint32_t ExitYStraightMaxMs       = 8000U;
  static constexpr uint32_t ExitYBlackIgnoreMs       = 200U;
  static constexpr int32_t  ExitBlackReflectionMax   = 30;
  /* 1回目の黒線つかみ後: 右へ相対 +90° 旋回 → 低速LT → ジャイロreset */
  static constexpr float    ExitFirstRightTurnDeg    = 90.0f;
  static constexpr float    ExitSlowTracePower       = 38.0f;
  static constexpr uint32_t ExitSlowTraceMs          = 600U;
  static constexpr float    ExitTraceKp              = 0.35f;
  static constexpr float    ExitTraceKi              = 0.01f;
  static constexpr float    ExitTraceKd              = 0.8f;
  static constexpr float    ExitTraceOutMax          = 100.0f;
  static constexpr float    ExitTraceScale           = 1.0f;
  static constexpr bool     ExitTraceEdgeFollow =
      (ETROBO_COURSE_L == 0); /* L=左エッジ, R=右エッジ */

  /* reset 後 LT: ±17° 到達で終了し、その角で直進 */
  static constexpr float    ExitLtStopHeadingDeg =
      ETROBO_COURSE_L ? -50.0f : 50.0f;
  static constexpr float    ExitLtStopToleranceDeg   = 2.0f;
  static constexpr float    ExitAfter17TracePower    = 60.0f;
  static constexpr float    ExitAfter17TraceKp       = 0.55f;
  static constexpr float    ExitAfter17TraceKi       = 0.025f;
  static constexpr float    ExitAfter17TraceKd       = 0.8f;
  static constexpr float    ExitAfter17TraceOutMax   = 100.0f;
  static constexpr float    ExitAfter17TraceScale    = 1.0f;
  static constexpr uint32_t ExitAfter17TraceMaxMs    = 15000U;
  static constexpr int      ExitAfter17StraightPower = 50;
  static constexpr uint32_t ExitAfter17StraightMaxMs = 8000U;
  /* 17°直進開始直後は黒判定を無視（直前の線上での即停止防止）[ms] */
  static constexpr uint32_t ExitAfter17BlackIgnoreMs = 800U;
  /* 2回目の黒線つかみ後: 絶対 +90° へ旋回 */
  static constexpr float    ExitSecondTurnAbsDeg     = 90.0f;

  /* 絶対+90°後 → SKYBLUE まで LT */
  static constexpr float    ExitFinalTracePower      = 60.0f;
  static constexpr uint32_t ExitFinalSkyblueIgnoreMs = 400U;
  static constexpr uint32_t ExitFinalTraceMaxMs      = 15000U;
};

#ifdef __cplusplus
extern "C" {
#endif

void Nansho3Task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_NANSHO3_H_
