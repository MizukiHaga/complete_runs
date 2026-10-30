#include "Arm.h"

#include <kernel.h>
#include <stdio.h>

void Arm::configure(pbio_port_id_t motor_port, pup_direction_t positive_direction)
{
  motor_ = nullptr;
  motor_ = pup_motor_get_device(motor_port);
  if (motor_ == nullptr) {
    printf("Arm: configure failed (port=%d, device not found)\n",
           (int)motor_port);
    return;
  }

  for (int retry = 0; retry < 20; ++retry) {
    const pbio_error_t err =
        pup_motor_setup(motor_, positive_direction, true);
    if (err == PBIO_SUCCESS) {
      pup_motor_stop(motor_);
      printf("Arm: configure succeeded (port=%d)\n", (int)motor_port);
      return;
    }
    if (err != PBIO_ERROR_AGAIN) {
      motor_ = nullptr;
      printf("Arm: configure failed (port=%d, error=%d)\n",
             (int)motor_port, (int)err);
      return;
    }
    dly_tsk(50U * 1000U);
  }

  motor_ = nullptr;
  printf("Arm: configure failed (port=%d, timeout)\n", (int)motor_port);
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
  printf("Arm: raise started (power=%d, target_deg=%ld, max_ms=%lu)\n",
         signed_power, (long)target_deg, (unsigned long)max_ms);

  uint32_t elapsed_ms = 0U;
  bool target_reached = false;
  while (elapsed_ms < max_ms) {
    const int32_t count = pup_motor_get_count(motor_);
    if (reachedTarget(count, signed_power, target_deg)) {
      target_reached = true;
      printf("Arm: raise target reached (count=%ld, elapsed_ms=%lu)\n",
             (long)count, (unsigned long)elapsed_ms);
      break;
    }
    dly_tsk(RaiseStepMs * 1000U);
    elapsed_ms += RaiseStepMs;
  }

  if (!target_reached) {
    printf("Arm: raise target NOT reached (count=%ld, elapsed_ms=%lu)\n",
           (long)pup_motor_get_count(motor_), (unsigned long)elapsed_ms);
  }
  printf("Arm: entering hold (count=%ld)\n",
         (long)pup_motor_get_count(motor_));
  pup_motor_hold(motor_);
  printf("Arm: raise ended (%s)\n",
         target_reached ? "target reached" : "timeout");
}

void Arm::lower(int power, uint32_t max_ms, int32_t overshoot_deg)
{
  if (motor_ == nullptr) {
    printf("Arm: lower skipped (not configured)\n");
    return;
  }

  const int signed_power = clampPower(power);
  const int32_t overshoot =
      overshoot_deg >= 0 ? overshoot_deg : -overshoot_deg;

  printf("Arm: lower started (power=%d, overshoot_deg=%ld, max_ms=%lu)\n",
         signed_power, (long)overshoot, (unsigned long)max_ms);
  pup_motor_set_power(motor_, signed_power);

  uint32_t elapsed_ms = 0U;
  bool target_reached = false;
  while (elapsed_ms < max_ms) {
    const int32_t count = pup_motor_get_count(motor_);

    /* 上げ時 reset の 0 を過ぎて overshoot 分だけ余分に下げる */
    if (signed_power > 0 && count >= overshoot) {
      target_reached = true;
      break;
    }
    if (signed_power < 0 && count <= -overshoot) {
      target_reached = true;
      break;
    }

    dly_tsk(RaiseStepMs * 1000U);
    elapsed_ms += RaiseStepMs;
  }

  printf("Arm: lower %s (count=%ld, elapsed_ms=%lu)\n",
         target_reached ? "target reached" : "timeout",
         (long)pup_motor_get_count(motor_), (unsigned long)elapsed_ms);
  printf("Arm: entering hold (count=%ld)\n",
         (long)pup_motor_get_count(motor_));
  pup_motor_hold(motor_);
  printf("Arm: lower ended (%s)\n",
         target_reached ? "target reached" : "timeout");
}

void Arm::stop()
{
  if (motor_ == nullptr) {
    return;
  }
  pup_motor_brake(motor_);
}
