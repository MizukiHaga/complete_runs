#include "app.h"
#include "Angle/Angle.h"

#include <spike/hub/imu.h>
#include <spike/hub/system.h>

static FILE *output_file = nullptr;

void Angle_Init(FILE *bt_out)
{
  output_file = bt_out;
  hub_imu_init();
  hub_imu_set_tilt(ANGLE_IMU_TILT_DEG);
  for (int i = 0; i < 500 && !hub_imu_is_ready(); i++) {
    dly_tsk(20 * 1000);
  }
}

void angle_task(intptr_t unused)
{
  if (!output_file || !hub_imu_is_ready()) {
    return;
  }
  float h = hub_imu_get_heading();
  fprintf(output_file, "IMU_h,%.2f\n", (double)h);
}
