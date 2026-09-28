#include "app.h"
#include "Morter/Morter.h"

#include <spike/pup/motor.h>
#include <spike/hub/system.h>

extern "C" {
#include <pbio/tacho.h>
}

static pup_motor_t *fg_left_motor;
static pup_motor_t *fg_right_motor;
static FILE *output_file = nullptr;

/* set_power 中は pup_motor_get_count が止まって 0 のままになるため、
 * 回転センサを直接読む。 */
static int32_t tachoDeg(pup_motor_t *motor)
{
  if (motor == nullptr) {
    return 0;
  }
  pbio_angle_t angle;
  if (pbio_tacho_get_angle(motor->tacho, &angle) != PBIO_SUCCESS) {
    return pup_motor_get_count(motor);
  }
  return (angle.rotations * 360) + (angle.millidegrees / 1000);
}

void Morter_SetOutput(FILE *fp)
{
  output_file = fp;
}

void Morter_Configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port)
{
  fg_left_motor = pup_motor_get_device(left_motor_port);
  fg_right_motor = pup_motor_get_device(right_motor_port);
  pup_motor_setup(fg_left_motor, PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(fg_right_motor, PUP_DIRECTION_CLOCKWISE, true);
}

void morter_task(intptr_t unused)
{
  (void)unused;
  if (fg_left_motor == nullptr || fg_right_motor == nullptr) {
    return;
  }

#if MORTER_ENABLE_DRIVE
  pup_motor_set_power(fg_left_motor, MORTER_LEFT_POWER);
  pup_motor_set_power(fg_right_motor, MORTER_RIGHT_POWER);
#endif

  const int32_t left_deg = tachoDeg(fg_left_motor);
  const int32_t right_deg = tachoDeg(fg_right_motor);
  const int32_t left_pwr = pup_motor_get_power(fg_left_motor);
  const int32_t right_pwr = pup_motor_get_power(fg_right_motor);

  if (output_file) {
    fprintf(output_file, "MOT_L,%d,MOT_R,%d,MOT_Lp,%d,MOT_Rp,%d\n",
            (int)left_deg, (int)right_deg, (int)left_pwr, (int)right_pwr);
  } else {
    printf("MOT_L,%d,MOT_R,%d,MOT_Lp,%d,MOT_Rp,%d\n",
           (int)left_deg, (int)right_deg, (int)left_pwr, (int)right_pwr);
  }
}
