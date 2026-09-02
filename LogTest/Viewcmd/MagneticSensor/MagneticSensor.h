/*
 * ハブ内蔵 IMU の方位・角速度ログ (Viewcmd)。
 *
 * SPIKE Prime ハブは spike-rt 上で磁気ベクトル (uT) を直接読む API を
 * 公開していない。本モジュールは fusion 方位角 (3D heading) とヨー角速度を
 * BLE へ送る。形式は Angle と同様の CSV 行。
 *
 * 出力例 (100 ms 周期):
 *   MAG_h,12.34
 *   MAG_wz,-1.25
 */

#ifndef LOGTEST_MAGNETIC_SENSOR_H_
#define LOGTEST_MAGNETIC_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <stdio.h>

/* ハブ IMU の傾き補正 [deg]。Angle と同値。 */
#define MAGNETIC_IMU_TILT_DEG  (51.0f)

void MagneticSensor_Init(FILE *bt_out);
void magnetic_sensor_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // LOGTEST_MAGNETIC_SENSOR_H_
