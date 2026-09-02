#include "app.h"
#include "MagneticSensor/MagneticSensor.h"

#include <spike/hub/imu.h>

static FILE *output_file = nullptr;

void MagneticSensor_Init(FILE *bt_out)
{
  output_file = bt_out;
  hub_imu_init();
  hub_imu_set_tilt(MAGNETIC_IMU_TILT_DEG);
  for (int i = 0; i < 500 && !hub_imu_is_ready(); i++) {
    dly_tsk(20 * 1000);
  }
}

void magnetic_sensor_task(intptr_t unused)
{
  (void)unused;
  if (!output_file || !hub_imu_is_ready()) {
    return;
  }

  const float heading = hub_imu_get_heading();

  float angv[3] = {0.0f, 0.0f, 0.0f};
  hub_imu_get_angular_velocity(angv);
  /* pybricks: heading_rate = -angv.z (CW 正) */
  const float yaw_rate = -angv[2];

  fprintf(output_file, "MAG_h,%.2f\n", (double)heading);
  fprintf(output_file, "MAG_wz,%.2f\n", (double)yaw_rate);
}
