#ifndef TESTAPP_HEADING_FUSION_SETTINGS_H_
#define TESTAPP_HEADING_FUSION_SETTINGS_H_

#include "HeadingFusion.h"

/* 距離:mm、角度:deg、時間:s。校正値は実測して入力する。
 * 56/56/112は従来のEncoderPer90Deg=180と同じ換算。実測済み値ではない。 */
namespace HeadingFusionSettings {
constexpr bool Enabled = true;
constexpr float TurnCorrectionGain = 0.8f; // 累積(車輪角-ジャイロ角)の適用割合
/* 旋回区間外のKF車輪観測の有効/無効。
 * 旋回中はTurnCorrectionGainを使う。静止bias学習は維持する。 */
constexpr bool MovingWheelObservation = true;
constexpr float LeftWheelDiameterMm = 56.0f;
constexpr float RightWheelDiameterMm = 56.0f;
constexpr float EffectiveTrackMm = 112.0f;
constexpr float MaxWheelRateDegPerSec = 3000.0f; // カウントreset等の不連続検出
constexpr unsigned MaxPeriodMs = 200U;
constexpr float GyroNoiseDegPerRootSec = 0.08f;
constexpr float BiasWalkDegPerSecPerRootSec = 0.008f;
constexpr float EncoderDeltaNoiseDeg = 0.15f;
constexpr float WheelRateNoiseDegPerSec = 0.16f;
constexpr float InitialBiasStdDegPerSec = 0.5f;
constexpr float InnovationLimit = 3.0f;
constexpr float MaxWheelNoiseScale = 1000.0f;
constexpr float StationaryEncoderRangeDeg = 2.0f; // 各車輪の最大値-最小値
constexpr float StationaryHoldSec = 0.5f;
constexpr float StationaryRateNoiseDegPerSec = 0.05f;
constexpr float StationaryGyroLimitDegPerSec = 0.5f;

inline HeadingFusionConfig config()
{
  HeadingFusionConfig result;
  result.turnCorrectionGain = TurnCorrectionGain;
  result.movingWheelObservation = MovingWheelObservation;
  result.leftDiameter = LeftWheelDiameterMm;
  result.rightDiameter = RightWheelDiameterMm;
  result.track = EffectiveTrackMm;
  result.gyroNoise = GyroNoiseDegPerRootSec;
  result.biasWalk = BiasWalkDegPerSecPerRootSec;
  result.encoderNoise = EncoderDeltaNoiseDeg;
  result.wheelRateNoise = WheelRateNoiseDegPerSec;
  result.initialBiasStd = InitialBiasStdDegPerSec;
  result.innovationLimit = InnovationLimit;
  result.maxNoiseScale = MaxWheelNoiseScale;
  result.stationaryEncoderRange = StationaryEncoderRangeDeg;
  result.stationaryHold = StationaryHoldSec;
  result.stationaryRateNoise = StationaryRateNoiseDegPerSec;
  result.stationaryGyroLimit = StationaryGyroLimitDegPerSec;
  return result;
}
static_assert(TurnCorrectionGain >= 0 && TurnCorrectionGain <= 1, "Turn gain must be in [0,1]");
static_assert(LeftWheelDiameterMm > 0 && RightWheelDiameterMm > 0 && EffectiveTrackMm > 0,
              "Wheel dimensions must be positive");
static_assert(GyroNoiseDegPerRootSec > 0 && BiasWalkDegPerSecPerRootSec > 0 &&
              EncoderDeltaNoiseDeg > 0 && WheelRateNoiseDegPerSec > 0 &&
              InitialBiasStdDegPerSec > 0 && InnovationLimit > 0 && MaxWheelNoiseScale >= 1,
              "Fusion noise settings must be positive");
static_assert(MaxPeriodMs > 0 && MaxPeriodMs <= 200 && MaxWheelRateDegPerSec > 0 &&
              StationaryEncoderRangeDeg > 0 && StationaryHoldSec > 0 && StationaryRateNoiseDegPerSec > 0 &&
              StationaryGyroLimitDegPerSec > 0,
              "Invalid fusion timing or stationary settings");
}

#endif // TESTAPP_HEADING_FUSION_SETTINGS_H_
