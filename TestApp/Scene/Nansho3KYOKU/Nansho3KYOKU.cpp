#include "Nansho3KYOKU.h"

#include "Nansho3.h"
#include "SpeedControl.h"
#include "app.h"
#include "kernel_cfg.h"
#include "Straight.h"
#include "Gyro.h"
#include "LineTracer.h"

#include <math.h>
#include <spike/hub/display.h>
#include <spike/hub/speaker.h>

extern Straight            gMotors;
extern Gyro                gGyro;
extern LineTracer          gLineTracer;
extern Nansho3KYOKUScene    gNansho3KYOKU;

namespace {

/* 目標速度を1周期あたりの加減速上限以内で更新する。 */
float approachSpeed(float current_speed, float target_speed, float max_change)
{
  const float change = kyoku::clampSpeedValue(
      target_speed - current_speed, -max_change, max_change);
  return current_speed + change;
}

} // namespace

void Nansho3KYOKUScene::runInitialApproach()
{
  using namespace kyoku_settings;

  gMotors.setHeadingCorrection(true);
  gMotors.setEncoderStraightBalance(false);
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);

  /* 新しいインスタンスを使い、既存gNansho3の実行状態を引き継がない。 */
  Nansho3Scene initial_scene;
  initial_scene.testInitialTrace();
  initial_scene.initialRun(/*continue_after=*/false);

  dly_tsk(InitialSettleMs * 1000U);
  stp_cyc(STRAIGHT_BALANCE_TASK_CYC);
}

bool Nansho3KYOKUScene::planRallyPath(const kyoku::Config &config)
{
  using namespace kyoku_settings;

  hub_display_char('P');
  return path_.plan(config, Start, Gates, Finish);
}

bool Nansho3KYOKUScene::isPredictedMotionSafe(
    kyoku::Pose pose, float left_speed, float right_speed,
    float wheel_track) const
{
  using namespace kyoku_settings;

  const float step_sec = PredictionStepMs / 1000.0f;
  const float yaw_rate = (right_speed - left_speed) / wheel_track;
  for (int step = 0; step < PredictionStepCount; ++step) {
    const float next_yaw = pose.yaw + yaw_rate * step_sec;
    pose = kyoku::odometry(pose, left_speed * step_sec,
                            right_speed * step_sec, next_yaw);
    if (!path_.safe(pose, PredictionExtraMarginMm)) {
      return false;
    }
  }
  return true;
}

Nansho3KYOKUScene::StopReason Nansho3KYOKUScene::followRallyPath(
    const kyoku::Config &config)
{
  using namespace kyoku_settings;

  hub_display_char('R');
  gMotors.setHeadingCorrection(false);
  gGyro.setRateHeading(true);

  /* 進入中の走行距離を含めず、ここから自己位置の推定を始める。 */
  const float initial_heading_deg = gGyro.heading();
  const float mm_per_degree = config.wheelDiameter * kyoku::Pi / 360.0f;
  kyoku::Pose pose = Start;
  int32_t previous_left_encoder  = gMotors.leftEncoderCount();
  int32_t previous_right_encoder = gMotors.rightEncoderCount();
  float previous_yaw = Start.yaw;

  float left_target_speed  = 0.0f;
  float right_target_speed = 0.0f;
  float left_integral  = 0.0f;
  float right_integral = 0.0f;
  float elapsed_sec = 0.0f;
  float stalled_sec = 0.0f;
  int path_progress = 0;

  SYSTIM previous_time;
  if (get_tim(&previous_time) != E_OK) {
    return StopReason::PeriodError;
  }

  while (elapsed_sec < RunTimeoutMs / 1000.0f) {
    /* 1) 実際の制御周期を取得する。get_timの単位はus。 */
    dly_tsk(ControlPeriodMs * 1000U);
    SYSTIM current_time;
    if (get_tim(&current_time) != E_OK) {
      return StopReason::PeriodError;
    }
    const float step_sec = static_cast<float>(current_time - previous_time) /
                           1000000.0f;
    previous_time = current_time;
    if (step_sec <= 0.0f || step_sec > MaxControlPeriodMs / 1000.0f) {
      return StopReason::PeriodError;
    }
    elapsed_sec += step_sec;

    /* 2) 左右移動量とジャイロから車軸中央の位置を更新する。
     * 既存Gyroは右旋回が正、経路側は左旋回が正なので符号を反転。 */
    const int32_t left_encoder  = gMotors.leftEncoderCount();
    const int32_t right_encoder = gMotors.rightEncoderCount();
    const float left_distance = (left_encoder - previous_left_encoder) *
                                 LeftEncoderSign * mm_per_degree;
    const float right_distance = (right_encoder - previous_right_encoder) *
                                  RightEncoderSign * mm_per_degree;
    previous_left_encoder  = left_encoder;
    previous_right_encoder = right_encoder;

    const float heading_delta = (gGyro.heading() - initial_heading_deg) *
                                 kyoku::Pi / 180.0f;
    const float yaw = kyoku::wrap(Start.yaw - heading_delta);
    if (!isfinite(yaw) || fabsf(kyoku::wrap(yaw - previous_yaw)) > MaxYawStepRad) {
      return StopReason::GyroJump;
    }
    if (fabsf(left_distance) > MaxEncoderStepMm ||
        fabsf(right_distance) > MaxEncoderStepMm) {
      return StopReason::EncoderJump;
    }
    previous_yaw = yaw;
    pose = kyoku::odometry(pose, left_distance, right_distance, yaw);

    /* 3) Pure Pursuitから左右の目標速度を求め、加減速を制限する。 */
    const float left_measured_speed  = left_distance / step_sec;
    const float right_measured_speed = right_distance / step_sec;
    const float measured_speed = (left_measured_speed + right_measured_speed) / 2;
    const kyoku::Command command = path_.follow(pose, measured_speed, path_progress);
    if (!command.valid) {
      return StopReason::TrackingFailed;
    }
    if (command.done) {
      return StopReason::Complete;
    }

    const float max_speed_change = config.acceleration * step_sec;
    left_target_speed = approachSpeed(left_target_speed, command.left, max_speed_change);
    right_target_speed = approachSpeed(right_target_speed, command.right, max_speed_change);

    /* 4) 接触予測と停滞を確認してからモータへ出力する。 */
    if (!isPredictedMotionSafe(pose, left_target_speed,
                               right_target_speed, config.track)) {
      return StopReason::CollisionRisk;
    }
    const bool commanded_to_move =
        left_target_speed + right_target_speed > StallSpeedSumMmPerSec;
    const bool wheels_not_moving =
        fabsf(left_distance) + fabsf(right_distance) < StallTravelSumMm;
    stalled_sec = commanded_to_move && wheels_not_moving
                    ? stalled_sec + step_sec : 0.0f;
    if (stalled_sec > StallTimeoutMs / 1000.0f) {
      return StopReason::Stalled;
    }

    const int left_power = kyoku::wheelPower(
        left_target_speed, left_measured_speed, left_integral, step_sec);
    const int right_power = kyoku::wheelPower(
        right_target_speed, right_measured_speed, right_integral, step_sec);
    gMotors.run(left_power, right_power);
  }
  return StopReason::Timeout;
}

void Nansho3KYOKUScene::finishRun(StopReason reason)
{
  gMotors.stop();
  hub_display_char(static_cast<char>(reason));
  if (reason == StopReason::PlanFailed) {
    hub_speaker_play_tone(kyoku_settings::PlanErrorToneHz,
                          kyoku_settings::PlanErrorToneMs);
  }
  gMotors.setHeadingCorrection(true);
  sta_cyc(STRAIGHT_BALANCE_TASK_CYC);
}

void Nansho3KYOKUScene::run()
{
  gMotors.stop();
  gLineTracer.setEnabled(false);

  /* 従来のapp.cppの押す→離すを受け、低速LTから開始する。 */
  runInitialApproach();

  const kyoku::Config config = kyoku_settings::config();
  if (!planRallyPath(config)) {
    finishRun(StopReason::PlanFailed);
    return;
  }
  finishRun(followRallyPath(config));
}

extern "C" void Nansho3KYOKUTask(intptr_t exinf)
{
  (void)exinf;
  gNansho3KYOKU.run();
  ext_tsk();
}
