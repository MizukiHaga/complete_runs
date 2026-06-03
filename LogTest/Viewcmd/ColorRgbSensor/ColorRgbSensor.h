

#ifndef TESTAPP_COLOR_RGB_SENSOR_H_
#define TESTAPP_COLOR_RGB_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>
#include <pbio/port.h>
#include <stdio.h>

/* 既存の ColorSensor (HSV) と同じ作りの RGB 版。
 *   ColorRgbSensor_Configure : ポートからカラーセンサを取得
 *   ColorRgbSensor_SetOutput : 出力先 (BLE の FILE*) を設定。未設定なら printf
 *   color_rgb_sensor_task    : 周期起動され、RGB を読んで出力する */
void ColorRgbSensor_Configure(pbio_port_id_t color_sensor_port);
void ColorRgbSensor_SetOutput(FILE *fp);
void color_rgb_sensor_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_COLOR_RGB_SENSOR_H_
