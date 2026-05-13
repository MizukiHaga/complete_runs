#include "app.h"
#include "ColorSensor/ColorSensor.h"

#include <spike/pup/colorsensor.h>
#include <spike/hub/system.h>

static pup_device_t *fg_color_sensor;
static pup_color_hsv_t color_sensor_value;
static FILE *output_file = nullptr;

void ColorSensor_SetOutput(FILE *fp)
{
  output_file = fp;
}

void ColorSensor_Configure(pbio_port_id_t color_sensor_port)
{
  fg_color_sensor = pup_color_sensor_get_device(color_sensor_port);
}

void color_sensor_task(intptr_t unused)
{
  color_sensor_value = pup_color_sensor_hsv(fg_color_sensor, false);
  if (output_file) {
    fprintf(output_file, "H,%d,S,%d,V,%d\n",
            color_sensor_value.h,
            color_sensor_value.s,
            color_sensor_value.v);
  } else {
    printf("H,%d,S,%d,V,%d\n",
            color_sensor_value.h,
            color_sensor_value.s,
            color_sensor_value.v);
  }
}
