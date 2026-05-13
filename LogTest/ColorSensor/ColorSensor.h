

#ifndef TESTAPP_COLOR_SENSOR_H_
#define TESTAPP_COLOR_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>
#include <pbio/port.h>
#include <stdio.h>

void ColorSensor_Configure(pbio_port_id_t color_sensor_port);
void ColorSensor_SetOutput(FILE *fp);
void color_sensor_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_COLOR_SENSOR_H_