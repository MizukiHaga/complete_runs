#include "app.h"
#include "Morter/Morter.h"

#include <spike/pup/motor.h>
#include <spike/hub/system.h>

static pup_motor_t *fg_left_motor;
static pup_motor_t *fg_right_motor;

void Morter_Configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port)
{
  fg_left_motor = pup_motor_get_device(left_motor_port);
  fg_right_motor = pup_motor_get_device(right_motor_port);
  pup_motor_setup(fg_left_motor, PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(fg_right_motor, PUP_DIRECTION_CLOCKWISE, true);
}

void morter_task(intptr_t unused)
{
#if MORTER_ENABLE_DRIVE
  pup_motor_set_power(fg_left_motor, MORTER_LEFT_POWER);
  pup_motor_set_power(fg_right_motor, MORTER_RIGHT_POWER);
#else
  /*
   * Morter.h の MORTER_ENABLE_DRIVE を 1 にする方法のほか、ここを直接
   * 切り替える場合は下の 2 行のコメントを外し、上の #if 〜 #endif 全体を
   * コメントアウトしてください。
   */
  // pup_motor_set_power(fg_left_motor, MORTER_LEFT_POWER);
  // pup_motor_set_power(fg_right_motor, MORTER_RIGHT_POWER);
#endif
}
