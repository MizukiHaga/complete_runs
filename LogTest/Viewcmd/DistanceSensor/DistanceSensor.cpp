#include "app.h"
#include "DistanceSensor/DistanceSensor.h"

#include <spike/pup/ultrasonicsensor.h>

static FILE *output_file = nullptr;
static pup_device_t *fg_ultrasonic = nullptr;

void DistanceSensor_Init(FILE *bt_out, pbio_port_id_t port)
{
  output_file = bt_out;
  fg_ultrasonic = pup_ultrasonic_sensor_get_device(port);
}

void distance_sensor_task(intptr_t unused)
{
  if (!output_file || !fg_ultrasonic) {
    return;
  }
  const int32_t distance_mm = pup_ultrasonic_sensor_distance(fg_ultrasonic);
  fprintf(output_file, "US_dist,%d\n", (int)distance_mm);
}
