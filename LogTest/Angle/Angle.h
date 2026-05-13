
#ifndef LOGTEST_ANGLE_H_
#define LOGTEST_ANGLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <stdio.h>

/* ハブ IMU の傾き補正 (度)。実機に合わせて調整。 */
#define ANGLE_IMU_TILT_DEG  (51.0f)

void Angle_Init(FILE *bt_out);
void angle_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif
