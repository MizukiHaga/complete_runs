#include "Sonic.h"

#include <spike/pup/ultrasonicsensor.h>

static pup_device_t *fg_ultrasonic = nullptr;

void Sonic_Configure(pbio_port_id_t ultrasonic_port)
{
  fg_ultrasonic = pup_ultrasonic_sensor_get_device(ultrasonic_port);
}

int32_t Sonic_ReadDistanceMm(void)
{
  if (fg_ultrasonic == nullptr) {
    return SONIC_DISTANCE_INVALID;
  }
  return pup_ultrasonic_sensor_distance(fg_ultrasonic);
}

bool Sonic_IsDetected(void)
{
  return Sonic_ReadDistanceMm() != SONIC_DISTANCE_INVALID;
}

bool Sonic_IsCloserThan(int32_t threshold_mm)
{
  const int32_t distance_mm = Sonic_ReadDistanceMm();
  return distance_mm != SONIC_DISTANCE_INVALID && distance_mm <= threshold_mm;
}
