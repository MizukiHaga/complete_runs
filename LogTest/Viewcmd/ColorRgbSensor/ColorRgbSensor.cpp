#include "app.h"
#include "ColorRgbSensor/ColorRgbSensor.h"

#include <spike/pup/colorsensor.h>
#include <spike/hub/system.h>

static pup_device_t *fg_color_sensor;
static pup_color_rgb_t color_sensor_value;
static FILE *output_file = nullptr;

/* 出力先 (BLE の FILE*) を設定。nullptr なら printf に出す。 */
void ColorRgbSensor_SetOutput(FILE *fp)
{
  output_file = fp;
}

/* ポートからカラーセンサのデバイスを取得する。 */
void ColorRgbSensor_Configure(pbio_port_id_t color_sensor_port)
{
  fg_color_sensor = pup_color_sensor_get_device(color_sensor_port);
  if (fg_color_sensor != nullptr) {
    pup_color_sensor_light_on(fg_color_sensor);
  }
}

/* 周期起動。生 RGB (各 0〜1024) を読み、"R,<r>,G,<g>,B,<b>" 形式で出力する。
 * output_file が設定されていれば BLE へ、無ければ printf (USB) へ。 */
void color_rgb_sensor_task(intptr_t unused)
{
  (void)unused;
  if (fg_color_sensor == nullptr) {
    return;
  }
  color_sensor_value = pup_color_sensor_rgb(fg_color_sensor);
  if (output_file) {
    fprintf(output_file, "R,%d,G,%d,B,%d\n",
            color_sensor_value.r,
            color_sensor_value.g,
            color_sensor_value.b);
  } else {
    printf("R,%d,G,%d,B,%d\n",
            color_sensor_value.r,
            color_sensor_value.g,
            color_sensor_value.b);
  }
}
