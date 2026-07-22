#include "app.h"
#include "ReflectionSensor/ReflectionSensor.h"

#include <spike/pup/colorsensor.h>

static pup_device_t *fg_color_sensor = nullptr;
static FILE *output_file = nullptr;

void ReflectionSensor_SetOutput(FILE *fp)
{
  output_file = fp;
}

void ReflectionSensor_Configure(pbio_port_id_t color_sensor_port)
{
  fg_color_sensor = pup_color_sensor_get_device(color_sensor_port);
  if (fg_color_sensor != nullptr) {
    pup_color_sensor_light_on(fg_color_sensor);
  }
}

/* 周期起動。反射光 (0〜100 程度) を "REF,<value>" 形式で出力する。 */
void reflection_sensor_task(intptr_t unused)
{
  (void)unused;
  if (fg_color_sensor == nullptr) {
    return;
  }

  const int32_t ref = pup_color_sensor_reflection(fg_color_sensor);
  if (output_file) {
    fprintf(output_file, "REF,%d\n", (int)ref);
  } else {
    printf("REF,%d\n", (int)ref);
  }
}
