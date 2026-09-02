#include "app.h"
#include "Angle/Angle.h"

#include <math.h>

#include <spike/hub/imu.h>
#include <spike/hub/system.h>

static FILE *output_file = nullptr;

static bool kalman_enabled_ = (ANGLE_KALMAN_ENABLE != 0);
static float kf_q_ = ANGLE_KALMAN_PROCESS_NOISE_DEG2_PER_S;
static float kf_r_ = ANGLE_KALMAN_MEAS_NOISE_DEG2;
static float kf_angle_ = 0.0f;
static float kf_variance_ = ANGLE_KALMAN_MEAS_NOISE_DEG2;
static bool kf_ready_ = false;
static uint64_t kf_last_tim_ = 0U;

static constexpr float kSystimToSec = 0.001f;
static constexpr float kMaxKalmanDtSec = 0.2f;

static float wrap_deg180(float diff)
{
  while (diff >= 180.0f) {
    diff -= 360.0f;
  }
  while (diff < -180.0f) {
    diff += 360.0f;
  }
  return diff;
}

static float read_yaw_rate_deg_per_sec(void)
{
  float angv[3] = {0.0f, 0.0f, 0.0f};
  hub_imu_get_angular_velocity(angv);
  return -angv[2];
}

static void reset_kalman_state(void)
{
  kf_angle_ = hub_imu_get_heading();
  kf_variance_ = kf_r_;
  SYSTIM now = 0U;
  get_tim(&now);
  kf_last_tim_ = (uint64_t)now;
  kf_ready_ = true;
}

static void update_kalman(void)
{
  SYSTIM now = 0U;
  get_tim(&now);

  if (!kf_ready_) {
    reset_kalman_state();
    return;
  }

  float dt_s = (float)((int64_t)now - (int64_t)kf_last_tim_) * kSystimToSec;
  if (dt_s <= 0.0f) {
    return;
  }
  if (dt_s > kMaxKalmanDtSec) {
    dt_s = kMaxKalmanDtSec;
  }

  const float yaw_rate = read_yaw_rate_deg_per_sec();

  kf_angle_ += yaw_rate * dt_s;
  kf_variance_ += kf_q_ * dt_s;

  const float measurement = hub_imu_get_heading();
  const float innovation = wrap_deg180(measurement - kf_angle_);
  const float gain = kf_variance_ / (kf_variance_ + kf_r_);
  kf_angle_ += gain * innovation;
  kf_variance_ *= (1.0f - gain);

  kf_last_tim_ = (uint64_t)now;
}

void Angle_Init(FILE *bt_out)
{
  output_file = bt_out;
  kalman_enabled_ = (ANGLE_KALMAN_ENABLE != 0);
  kf_ready_ = false;

  hub_imu_init();
  hub_imu_set_tilt(ANGLE_IMU_TILT_DEG);
  for (int i = 0; i < 500 && !hub_imu_is_ready(); i++) {
    dly_tsk(20 * 1000);
  }

  if (kalman_enabled_ && hub_imu_is_ready()) {
    reset_kalman_state();
  }
}

void Angle_SetKalmanEnabled(bool enabled)
{
  kalman_enabled_ = enabled;
  if (kalman_enabled_ && hub_imu_is_ready()) {
    reset_kalman_state();
  } else {
    kf_ready_ = false;
  }
}

bool Angle_IsKalmanEnabled(void)
{
  return kalman_enabled_;
}

void Angle_SetKalmanNoise(float process_noise_deg2_per_s,
                          float measurement_noise_deg2)
{
  if (process_noise_deg2_per_s > 0.0f) {
    kf_q_ = process_noise_deg2_per_s;
  }
  if (measurement_noise_deg2 > 0.0f) {
    kf_r_ = measurement_noise_deg2;
  }
}

void Angle_ResetHeading(void)
{
  hub_imu_reset_heading();
  if (kalman_enabled_ && hub_imu_is_ready()) {
    reset_kalman_state();
  } else {
    kf_ready_ = false;
  }
}

void angle_task(intptr_t unused)
{
  (void)unused;
  if (!output_file || !hub_imu_is_ready()) {
    return;
  }

  if (kalman_enabled_) {
    update_kalman();
    const float raw = hub_imu_get_heading();
    const float diff = wrap_deg180(raw - kf_angle_);
    fprintf(output_file, "IMU_h,%.2f\n", (double)kf_angle_);
    fprintf(output_file, "IMU_raw,%.2f\n", (double)raw);
    fprintf(output_file, "IMU_diff,%.2f\n", (double)diff);
  } else {
    const float h = hub_imu_get_heading();
    fprintf(output_file, "IMU_h,%.2f\n", (double)h);
  }
}
