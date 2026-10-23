#ifndef NANSHO3_KYOKU_SETTINGS_H
#define NANSHO3_KYOKU_SETTINGS_H

#include "Path.h"

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

/* === ゲート・コース範囲 =======================================
 * ゲート座標は開口の中央。normal は通過方向で、脚の並ぶ方向と直交する。
 * 旧 Nansho3 の格子座標ではなく、進入完了位置を基準にした実寸座標。
 * L/R のゲート配置はここで設定する（自動鏡像変換なし）。 */
constexpr float GateInnerWidthMm  = 240.0f;
constexpr float GateInnerHeightMm = 245.0f;
constexpr float GateFootRadiusMm  = 25.0f;  // 柱・継手・足元を含む円の半径
constexpr float CollisionMarginMm = 30.0f;
constexpr float CourseMinXMm = -400.0f;
constexpr float CourseMaxXMm = 1600.0f;
constexpr float CourseMinYMm = -500.0f;
constexpr float CourseMaxYMm = 1600.0f;

/* 初期動作の直進完了地点を Start とみなす。角度の単位は rad。 */
constexpr kyoku::Pose Start  = {{0.0f,    0.0f}, 0.0f};
constexpr kyoku::Pose Finish = {{0.0f, 1000.0f}, kyoku::Pi};
constexpr kyoku::Gate Gates[3] = {
  {{ 500.0f,    0.0f}, 0.0f},           // 赤: ±x 方向に通過
  {{1000.0f,  500.0f}, kyoku::Pi / 2},  // 青: ±y 方向に通過
  {{ 500.0f, 1000.0f}, kyoku::Pi},      // 黄: ±x 方向に通過
};

/* === 経路生成・速度計画 ======================================= */
constexpr int   RallyLapCount          = 6;
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
inline kyoku::Config config()
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
  result.minY          = CourseMinYMm;
  result.maxY          = CourseMaxYMm;
  result.laps          = RallyLapCount;
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

static_assert(MinRunPower > 0 && MinRunPower <= MaxPower && MaxPower <= 100,
              "Motor power must satisfy 0 < MinRunPower <= MaxPower <= 100");
static_assert(StaticFrictionBlendSpeed > 0 && SpeedIntegralMax > 0,
              "Speed control limits must be positive");
static_assert(ControlPeriodMs > 0 && ControlPeriodMs <= MaxControlPeriodMs,
              "Invalid control period");

} // namespace kyoku_settings

#endif // NANSHO3_KYOKU_SETTINGS_H
