/*
 * Straight モジュール
 *  左右モータをまとめて操作する直進・蛇行の共通モジュール。
 */

#include "app.h"
#include "Straight.h"

extern Straight gMotors;

int Straight::clampMotorPower(int power)
{
  if (power > 100) {
    return 100;
  }
  if (power < -100) {
    return -100;
  }
  return power;
}

void Straight::configure(pbio_port_id_t left_motor_port,
                         pbio_port_id_t right_motor_port)
{
  left_ = pup_motor_get_device(left_motor_port);
  right_ = pup_motor_get_device(right_motor_port);

  pup_motor_setup(left_, PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(right_, PUP_DIRECTION_CLOCKWISE, true);
}

void Straight::run(int left_power, int right_power)
{
  pup_motor_set_power(left_, clampMotorPower(left_power));
  pup_motor_set_power(right_, clampMotorPower(right_power));
}

void Straight::stop()
{
  pup_motor_set_power(left_, 0);
  pup_motor_set_power(right_, 0);
}

void Straight::runFor(int left_power, int right_power, uint32_t duration_ms)
{
  if (duration_ms == 0U) {
    stop();
    return;
  }

  if (duration_ms > MaxDurationMs) {
    duration_ms = MaxDurationMs;
  }

  uint32_t decel_ms = DecelMs;
  if (decel_ms > duration_ms) {
    decel_ms = duration_ms;
  }

  const uint32_t cruise_ms = duration_ms - decel_ms;

  if (cruise_ms > 0U) {
    run(left_power, right_power);
    dly_tsk((RELTIM)(cruise_ms * 1000U));
  }

  for (uint32_t t = 0U; t < decel_ms;) {
    uint32_t slice = DecelStepMs;
    if (t + slice > decel_ms) {
      slice = decel_ms - t;
    }
    const uint32_t numer = decel_ms - t;
    const int left =
        (int)((int64_t)left_power * (int64_t)numer / (int64_t)decel_ms);
    const int right =
        (int)((int64_t)right_power * (int64_t)numer / (int64_t)decel_ms);
    run(left, right);
    dly_tsk((RELTIM)(slice * 1000U));
    t += slice;
  }

  stop();
}

void Straight::runSerpentine(int left_base, int right_base, int steer,
                             uint32_t phase_ms, uint32_t total_ms,
                             PollFn poll)
{
  if (phase_ms == 0U) {
    stop();
    return;
  }

  const bool run_forever = (total_ms == SerpentineForever);
  if (!run_forever && total_ms > MaxDurationMs) {
    total_ms = MaxDurationMs;
  }

  uint32_t elapsed_ms = 0U;
  uint32_t phase_elapsed_ms = 0U;
  bool steer_left = true;

  while (1) {
    if (poll != nullptr && poll()) {
      break;
    }
    if (!run_forever && elapsed_ms >= total_ms) {
      break;
    }

    int left_power;
    int right_power;
    if (steer_left) {
      left_power = left_base + steer;
      right_power = right_base - steer;
    } else {
      left_power = left_base - steer;
      right_power = right_base + steer;
    }

    run(left_power, right_power);

    uint32_t step_ms = SerpentineStepMs;
    if (!run_forever && elapsed_ms + step_ms > total_ms) {
      step_ms = total_ms - elapsed_ms;
    }
    dly_tsk((RELTIM)(step_ms * 1000U));
    elapsed_ms += step_ms;
    phase_elapsed_ms += step_ms;

    if (phase_elapsed_ms >= phase_ms) {
      phase_elapsed_ms = 0U;
      steer_left = !steer_left;
    }
  }

  stop();
}

extern "C" void straight_task(intptr_t unused)
{
  (void)unused;

  gMotors.runFor(Straight::DefaultLeftPower, Straight::DefaultRightPower, 1000U);
  gMotors.runFor(50, -50, 500U);
  gMotors.runFor(Straight::DefaultLeftPower, Straight::DefaultRightPower, 1000U);

  ext_tsk();
}
