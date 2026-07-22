#include "Arm.h"

#include <kernel.h>

void Arm::configure(pbio_port_id_t motor_port, pup_direction_t positive_direction)
{
  motor_ = pup_motor_get_device(motor_port);
  if (motor_ == nullptr) {
    return;
  }

  for (int retry = 0; retry < 20; ++retry) {
    const pbio_error_t err =
        pup_motor_setup(motor_, positive_direction, true);
    if (err == PBIO_SUCCESS) {
      pup_motor_stop(motor_);
      return;
    }
    if (err != PBIO_ERROR_AGAIN) {
      motor_ = nullptr;
      return;
    }
    dly_tsk(50U * 1000U);
  }

  motor_ = nullptr;
}

int Arm::clampPower(int power)
{
  if (power > 100) {
    return 100;
  }
  if (power < -100) {
    return -100;
  }
  return power;
}

bool Arm::reachedTarget(int32_t count, int power, int32_t target_deg)
{
  const int32_t abs_target =
      target_deg >= 0 ? target_deg : -target_deg;

  if (power >= 0) {
    return count >= abs_target;
  }
  return count <= -abs_target;
}

void Arm::raise(int power, int32_t target_deg, uint32_t max_ms)
{
  if (motor_ == nullptr) {
    return;
  }

  const int signed_power = clampPower(power);

  pup_motor_reset_count(motor_);
  pup_motor_set_power(motor_, signed_power);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < max_ms) {
    if (reachedTarget(pup_motor_get_count(motor_), signed_power, target_deg)) {
      break;
    }
    dly_tsk(RaiseStepMs * 1000U);
    elapsed_ms += RaiseStepMs;
  }

  pup_motor_hold(motor_);
}

void Arm::lower(int power, uint32_t max_ms)
{
  if (motor_ == nullptr) {
    return;
  }

  const int signed_power = clampPower(power);

  pup_motor_set_power(motor_, signed_power);

  uint32_t elapsed_ms = 0U;
  while (elapsed_ms < max_ms) {
    const int32_t count = pup_motor_get_count(motor_);

    if (signed_power > 0 && count >= 0) {
      break;
    }
    if (signed_power < 0 && count <= 0) {
      break;
    }

    dly_tsk(RaiseStepMs * 1000U);
    elapsed_ms += RaiseStepMs;
  }

  pup_motor_hold(motor_);
}

void Arm::stop()
{
  if (motor_ == nullptr) {
    return;
  }
  pup_motor_brake(motor_);
}
