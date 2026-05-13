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
  for(int i = 0; i < 10; i++) {
    if(pup_motor_get_power(fg_left_motor) <= 0 && pup_motor_get_power(fg_right_motor) <= 0) {
      break;
    }
    pup_motor_set_power(fg_left_motor, pup_motor_get_power(fg_left_motor) - 5);
    pup_motor_set_power(fg_right_motor, pup_motor_get_power(fg_right_motor) - 5);
  }
}

void Straight_RunFor(int left_power, int right_power, uint32_t duration_ms) {
  if (duration_ms == 0U) {
    Straight_Stop();
    return;
  }
  if (duration_ms > STRAIGHT_MAX_DURATION_MS) {
    duration_ms = STRAIGHT_MAX_DURATION_MS;
  }

  Straight_Run(left_power, right_power);
  dly_tsk((RELTIM)duration_ms * 1000U);
  Straight_Stop();
}

void straight_task(intptr_t unused) {
  Straight_RunFor(STRAIGHT_LEFT_POWER, STRAIGHT_RIGHT_POWER, STRAIGHT_TIME);
  ext_tsk();
}
