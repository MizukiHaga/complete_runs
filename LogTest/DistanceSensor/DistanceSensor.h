#ifndef LOGTEST_DISTANCE_SENSOR_H_
#define LOGTEST_DISTANCE_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <pbio/port.h>
#include <stdio.h>

/* 距離センサ（超音波）の PUP ポート。実機配線に合わせて変更する */
#define DISTANCE_SENSOR_PORT  PBIO_PORT_ID_F

void DistanceSensor_Init(FILE *bt_out, pbio_port_id_t port);
void distance_sensor_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // LOGTEST_DISTANCE_SENSOR_H_
