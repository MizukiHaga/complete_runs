#include "app.h"
#include "Straight.h"

#include <spike/pup/motor.h>

static pup_motor_t *fg_left_motor;
static pup_motor_t *fg_right_motor;

static int clamp_motor_power(int power) {
  if (power > 100) {
    return 100;
  }
  if (power < -100) {
    return -100;
  }
  return power;
}

void Straight_Configure(pbio_port_id_t left_motor_port,
                        pbio_port_id_t right_motor_port) {
  fg_left_motor = pup_motor_get_device(left_motor_port);
  fg_right_motor = pup_motor_get_device(right_motor_port);

  pup_motor_setup(fg_left_motor, PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(fg_right_motor, PUP_DIRECTION_CLOCKWISE, true);
}

void Straight_Run(int left_power, int right_power) {
  pup_motor_set_power(fg_left_motor, clamp_motor_power(left_power));
  pup_motor_set_power(fg_right_motor, clamp_motor_power(right_power));
}

void Straight_Stop(void) {
  pup_motor_set_power(fg_left_motor, 0);
  pup_motor_set_power(fg_right_motor, 0);
}

void Straight_RunFor(int left_power, int right_power, uint32_t duration_ms) {
  if (duration_ms == 0U) {
    Straight_Stop();
    return;
  }
  if (duration_ms > STRAIGHT_MAX_DURATION_MS) {
    duration_ms = STRAIGHT_MAX_DURATION_MS;
  }

  uint32_t decel_ms = STRAIGHT_DECEL_MS;
  if (decel_ms > duration_ms) {
    decel_ms = duration_ms;
  }
  const uint32_t cruise_ms = duration_ms - decel_ms;

  if (cruise_ms > 0U) {
    Straight_Run(left_power, right_power);
    dly_tsk((RELTIM)(cruise_ms * 1000U));
  }

  for (uint32_t t = 0U; t < decel_ms;) {
    uint32_t slice = STRAIGHT_DECEL_STEP_MS;
    if (t + slice > decel_ms) {
      slice = decel_ms - t;
    }
    const uint32_t numer = decel_ms - t;
    const int left =
        (int)((int64_t)left_power * (int64_t)numer / (int64_t)decel_ms);
    const int right =
        (int)((int64_t)right_power * (int64_t)numer / (int64_t)decel_ms);
    Straight_Run(left, right);
    dly_tsk((RELTIM)(slice * 1000U));
    t += slice;
  }

  Straight_Stop();
}

void straight_task(intptr_t unused) {
  Straight_RunFor(STRAIGHT_LEFT_POWER, STRAIGHT_RIGHT_POWER, STRAIGHT_TIME);
  ext_tsk();
}