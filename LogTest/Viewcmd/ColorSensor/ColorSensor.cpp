#include "app.h"
#include "ColorSensor/ColorSensor.h"
#include "ColorSensor/ColorLabMath.h"

#include <spike/pup/colorsensor.h>
#include <spike/hub/system.h>

static pup_device_t *fg_color_sensor;
static FILE *output_file = nullptr;

void ColorSensor_SetOutput(FILE *fp)
{
  output_file = fp;
}

void ColorSensor_Configure(pbio_port_id_t color_sensor_port)
{
  fg_color_sensor = pup_color_sensor_get_device(color_sensor_port);
  if (fg_color_sensor != nullptr) {
    pup_color_sensor_light_on(fg_color_sensor);
  }
}

void color_sensor_task(intptr_t unused)
{
  (void)unused;
  if (fg_color_sensor == nullptr) {
    return;
  }

  const pup_color_rgb_t rgb = pup_color_sensor_rgb(fg_color_sensor);
  const LabColor lab = ColorLab_rgbToLab(rgb.r, rgb.g, rgb.b);

  if (output_file) {
    fprintf(output_file, "L,%.2f,a,%.2f,b,%.2f\n", lab.L, lab.a, lab.b);
  } else {
    printf("L,%.2f,a,%.2f,b,%.2f\n", lab.L, lab.a, lab.b);
  }
}
