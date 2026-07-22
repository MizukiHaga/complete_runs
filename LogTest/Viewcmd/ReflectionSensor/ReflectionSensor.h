#ifndef LOGTEST_REFLECTION_SENSOR_H_
#define LOGTEST_REFLECTION_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <pbio/port.h>
#include <stdio.h>

/* 反射光 (pup_color_sensor_reflection) のログ。
 *   ReflectionSensor_Configure : ポートからカラーセンサを取得・ライト点灯
 *   ReflectionSensor_SetOutput : 出力先 (BLE の FILE*)。未設定なら printf
 *   reflection_sensor_task     : 周期起動され、反射値を出力する */
void ReflectionSensor_Configure(pbio_port_id_t color_sensor_port);
void ReflectionSensor_SetOutput(FILE *fp);
void reflection_sensor_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // LOGTEST_REFLECTION_SENSOR_H_
