#ifndef NANSHO3_KYOKU_SETTINGS_H
#define NANSHO3_KYOKU_SETTINGS_H

#include "Path.h"
#include <math.h>

/* Nansho3KYOKU の調整値。
 * 距離:mm / 速度:mm/s / 加速度:mm/s^2 / 角度:rad / 時間:ms。
 * +x は進入直進後の前方、+y は左、角度は左旋回(CCW)が正。
 * 車体寸法・ゲート位置は仮設定のため、実配置に合わせて変更する。 */
namespace kyoku_settings {

/* === 初期動作 =================================================
 * 低速LT → 白検知 → バック → 旋回 → 直進。
 * 動作本体と調整値は Nansho3 を共用する（Nansho3.h の Initial*）。
 * ETROBO_COURSE_L: 1=R / 0=L。 */
constexpr unsigned InitialSettleMs = 200U;  // 進入直進後、姿勢を落ち着かせる待ち

/* === モータ出力・速度 PI ======================================
 * 正のパワーを出す場合は MinRunPower 以上にする。
 * 停止指令、または速度超過で出力不要の場合は 0（35には引き上げない）。
 * 初期LTのパワーは Nansho3 の設定を使うため、この下限の対象外。 */
constexpr int   MinRunPower = 35;
constexpr int   MaxPower    = 70;
constexpr float SpeedFeedForward       = 0.18f;  // 速度→パワーの比例係数
constexpr float StaticFrictionPower    = 18.0f;  // 摩擦分を補うフィードフォワード
constexpr float StaticFrictionBlendSpeed = 30.0f; // この速度まで摩擦補償を増やす
constexpr float SpeedKp                = 0.08f;
constexpr float SpeedKi                = 0.04f;
constexpr float SpeedIntegralMax       = 150.0f;

/* === 車体・エンコーダ =========================================
 * Front/Rear は車軸中央から車体端までの距離。
 * エンコーダ符号は、物理的な前進で距離が正になるように設定。 */
constexpr float WheelDiameterMm = 56.0f;
constexpr float WheelTrackMm    = 120.0f;
constexpr float BodyHalfWidthMm = 80.0f;
constexpr float BodyFrontMm     = 100.0f;
constexpr float BodyRearMm      = 80.0f;
constexpr float BodyHeightMm    = 200.0f;
constexpr float LeftEncoderSign  = 1.0f;
constexpr float RightEncoderSign = 1.0f;

/* === 格子座標・ゲート配置 =====================================
 * 格子は位置指定にだけ使う。実際の走行経路は格子に沿わせない。
 * Nansho3 の GateSeg と同様、a→b は開口を横切る2点（支柱ではない）。
 * 開口中央は2点の中点。方向は両向きを探索するので、a/bの逆順も可。
 * 以下は計算・追従確認用の仮配置。実コースに合わせて必ず変更する。 */
struct GridGate {
  kyoku::Vec a;
  kyoku::Vec b;
};

constexpr float GridStepXMm = 250.0f;  // x方向の1マスの実測距離
constexpr float GridStepYMm = 250.0f;  // y方向の1マスの実測距離
constexpr kyoku::Vec GridStart = {1.0f, 1.0f};
constexpr kyoku::Vec GridFinish = {1.0f, 5.0f};
constexpr float FinishHeadingRad = kyoku::Pi;
constexpr GridGate GridGates[3] = {
  {{2.0f, 1.0f}, {4.0f, 1.0f}},  // 赤: x方向に通過
  {{5.0f, 2.0f}, {5.0f, 4.0f}},  // 青: y方向に通過
  {{4.0f, 5.0f}, {2.0f, 5.0f}},  // 黄: x方向に通過
};

/* 初期直進の完了地点を(0,0)、向きを+xとする。
 * Lは格子+yが左、Rは右。既存 ETROBO_COURSE_L の0=L/1=Rに合わせる。 */
inline kyoku::Vec gridPoint(kyoku::Vec point, bool right_course = false)
{
  return {(point.x - GridStart.x) * GridStepXMm,
          (point.y - GridStart.y) * GridStepYMm * (right_course ? -1.0f : 1.0f)};
}

constexpr kyoku::Pose Start = {{0.0f, 0.0f}, 0.0f};

inline kyoku::Pose finishPose(bool right_course = false)
{
  return {gridPoint(GridFinish, right_course),
          right_course ? -FinishHeadingRad : FinishHeadingRad};
}

inline kyoku::Gate gridGate(GridGate gate, bool right_course = false)
{
  const kyoku::Vec a = gridPoint(gate.a, right_course);
  const kyoku::Vec b = gridPoint(gate.b, right_course);
  const bool valid = (gate.a.x == gate.b.x) != (gate.a.y == gate.b.y);
  return {{(a.x + b.x) / 2, (a.y + b.y) / 2},
          valid ? atan2f(b.y - a.y, b.x - a.x) : NAN};
}

inline void gates(kyoku::Gate result[3], bool right_course = false)
{
  for (int index = 0; index < 3; ++index) {
    result[index] = gridGate(GridGates[index], right_course);
  }
}

/* === ゲート・コース範囲 =======================================
 * 範囲はStart基準のmm、L側で指定。R側ではy範囲を反転する。
 * 車体全体が収まる範囲を指定し、格子の最外点と混同しない。 */
constexpr float GateInnerWidthMm  = 240.0f;
constexpr float GateInnerHeightMm = 245.0f;
constexpr float GateFootRadiusMm  = 25.0f;
constexpr float CollisionMarginMm = 30.0f;
constexpr float CourseMinXMm = -400.0f;
constexpr float CourseMaxXMm = 1600.0f;
constexpr float CourseMinYMm = -500.0f;
constexpr float CourseMaxYMm = 1600.0f;

/* === 経路生成・速度計画 ======================================= */
constexpr int   RallyLapCount          = 6;
constexpr int   CurveStyleCount        = 1;     // Hubは1から。2〜3は計算時間と相談して増やす
constexpr bool  FastPlan               = true;  // Hubはtrue。全候補比較はfalse
constexpr bool  OptimizeTravelTime     = true;  // falseは距離優先（比較・検証用）
constexpr float PathSampleMm           = 8.0f;
constexpr float MinTurnRadiusMm        = 100.0f;
constexpr float CruiseSpeedMmPerSec    = 220.0f;
constexpr float GateSpeedMmPerSec      = 130.0f;
constexpr float MaxWheelSpeedMmPerSec  = 320.0f;
constexpr float AccelerationMmPerSec2  = 180.0f;
constexpr float LateralAccelMmPerSec2  = 180.0f;

/* === Pure Pursuit =============================================
 * 先読み距離 = 現在速度 × LookaheadTimeSec を Min〜Max に制限。
 * MaxTrackingErrorMm は CollisionMarginMm 以下にする。 */
constexpr float LookaheadMinMm     = 45.0f;
constexpr float LookaheadMaxMm     = 90.0f;
constexpr float LookaheadTimeSec   = 0.25f;
constexpr float MaxTrackingErrorMm = 25.0f;

/* === 制御周期・停止判定 ======================================= */
constexpr unsigned PlanTimeoutMs       = 10000U; // 経路計算の上限。超過時はQ表示
constexpr unsigned PlanYieldIntervalMs = 20U;    // 計算中に他タスクへ実行を譲る間隔
constexpr unsigned ControlPeriodMs     = 10U;
constexpr unsigned MaxControlPeriodMs  = 100U;
constexpr unsigned RunTimeoutMs        = 180000U;
constexpr unsigned StallTimeoutMs      = 1000U;
constexpr float    MaxYawStepRad       = 0.3f;   // 1周期の方位変化上限
constexpr float    MaxEncoderStepMm    = 30.0f;  // 片輪の1周期の移動量上限
constexpr float    StallSpeedSumMmPerSec = 50.0f; // 左右目標速度の合計
constexpr float    StallTravelSumMm    = 0.2f;   // 左右移動量の絶対値合計
constexpr unsigned PredictionStepMs    = 20U;
constexpr int      PredictionStepCount = 10;
constexpr float    PredictionExtraMarginMm = 5.0f;
constexpr unsigned PlanErrorToneHz     = 300U;
constexpr unsigned PlanErrorToneMs     = 300U;

/* 経路モジュールへ渡す設定。調整は上の定数だけ変更する。 */
inline kyoku::Config config(bool right_course = false)
{
  kyoku::Config result;
  result.wheelDiameter = WheelDiameterMm;
  result.track         = WheelTrackMm;
  result.halfWidth     = BodyHalfWidthMm;
  result.front         = BodyFrontMm;
  result.rear          = BodyRearMm;
  result.height        = BodyHeightMm;
  result.gateWidth     = GateInnerWidthMm;
  result.gateHeight    = GateInnerHeightMm;
  result.footRadius    = GateFootRadiusMm;
  result.margin        = CollisionMarginMm;
  result.minX          = CourseMinXMm;
  result.maxX          = CourseMaxXMm;
  result.minY          = right_course ? -CourseMaxYMm : CourseMinYMm;
  result.maxY          = right_course ? -CourseMinYMm : CourseMaxYMm;
  result.laps          = RallyLapCount;
  result.curveStyles   = CurveStyleCount;
  result.optimizeTime  = OptimizeTravelTime;
  result.fastPlan      = FastPlan;
  result.sample        = PathSampleMm;
  result.minRadius     = MinTurnRadiusMm;
  result.maxSpeed      = CruiseSpeedMmPerSec;
  result.gateSpeed     = GateSpeedMmPerSec;
  result.wheelSpeed    = MaxWheelSpeedMmPerSec;
  result.acceleration  = AccelerationMmPerSec2;
  result.lateralAcceleration = LateralAccelMmPerSec2;
  result.lookaheadMin   = LookaheadMinMm;
  result.lookaheadMax   = LookaheadMaxMm;
  result.lookaheadTime  = LookaheadTimeSec;
  result.maxTrackingError = MaxTrackingErrorMm;
  return result;
}

static_assert(GridStepXMm > 0 && GridStepYMm > 0,
              "Grid spacing must be positive");
static_assert(CurveStyleCount >= 1 && CurveStyleCount <= 3,
              "CurveStyleCount must be 1..3");
static_assert(MinRunPower > 0 && MinRunPower <= MaxPower && MaxPower <= 100,
              "Motor power must satisfy 0 < MinRunPower <= MaxPower <= 100");
static_assert(StaticFrictionBlendSpeed > 0 && SpeedIntegralMax > 0,
              "Speed control limits must be positive");
static_assert(PlanTimeoutMs > 0 && PlanYieldIntervalMs > 0,
              "Planning time limits must be positive");
static_assert(ControlPeriodMs > 0 && ControlPeriodMs <= MaxControlPeriodMs,
              "Invalid control period");

} // namespace kyoku_settings

#endif // NANSHO3_KYOKU_SETTINGS_H
