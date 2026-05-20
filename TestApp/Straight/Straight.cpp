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
  // 時間が長すぎる場合は最大時間にする
  if (duration_ms > STRAIGHT_MAX_DURATION_MS) {
    duration_ms = STRAIGHT_MAX_DURATION_MS;
  }

  // 減速
  uint32_t decel_ms = STRAIGHT_DECEL_MS;
  if (decel_ms > duration_ms) {
    decel_ms = duration_ms;
  }

  //実際に設定した時間で走る時間を計算
  const uint32_t cruise_ms = duration_ms - decel_ms;

  if (cruise_ms > 0U) {
    Straight_Run(left_power, right_power);
    dly_tsk((RELTIM)(cruise_ms * 1000U));
  }

  // 線形減速
  for (uint32_t t = 0U; t < decel_ms;) {
    uint32_t slice = STRAIGHT_DECEL_STEP_MS;
    if (t + slice > decel_ms) {
      slice = decel_ms - t;
    }
    const uint32_t numer = decel_ms - t;
    const int left = (int)((int64_t)left_power * (int64_t)numer / (int64_t)decel_ms);
    const int right = (int)((int64_t)right_power * (int64_t)numer / (int64_t)decel_ms);
    Straight_Run(left, right);
    dly_tsk((RELTIM)(slice * 1000U));
    t += slice;
  }

  Straight_Stop();
}

#define STRAIGHT_SERPENTINE_STEP_MS  (10U)

void Straight_RunSerpentine(int left_base, int right_base, int steer,
                            uint32_t phase_ms, uint32_t total_ms,
                            straight_serpentine_poll_fn poll)
{
  if (phase_ms == 0U) {
    Straight_Stop();
    return;
  }
  const bool run_forever = (total_ms == STRAIGHT_SERPENTINE_FOREVER);
  if (!run_forever && total_ms > STRAIGHT_MAX_DURATION_MS) {
    total_ms = STRAIGHT_MAX_DURATION_MS;
  }

  uint32_t elapsed_ms = 0U;
  uint32_t phase_elapsed_ms = 0U;
  bool steer_left = true;

  for (;;) {
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

    Straight_Run(left_power, right_power);

    uint32_t step_ms = STRAIGHT_SERPENTINE_STEP_MS;
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

  Straight_Stop();
}

void straight_task(intptr_t unused) {
  /* 1 秒前進 */
  Straight_RunFor(STRAIGHT_LEFT_POWER, STRAIGHT_RIGHT_POWER, 1000U);

  /* 右折（ピボットターン）：左モータ前進、右モータ逆回転を 500 ms */
  Straight_RunFor(50, -50, 500U);

  /* さらに 1 秒前進 */
  Straight_RunFor(STRAIGHT_LEFT_POWER, STRAIGHT_RIGHT_POWER, 1000U);

  ext_tsk();
}