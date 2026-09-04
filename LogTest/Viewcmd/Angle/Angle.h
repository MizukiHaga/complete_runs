
#ifndef LOGTEST_ANGLE_H_
#define LOGTEST_ANGLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <stdbool.h>
#include <stdio.h>

/* ハブ IMU の傾き補正 (度)。実機に合わせて調整。 */
#define ANGLE_IMU_TILT_DEG  (51.0f)

/* 1: Angle_Init 時にカルマン ON（Angle_SetKalmanEnabled で実行時変更可） */
#ifndef ANGLE_KALMAN_ENABLE
#define ANGLE_KALMAN_ENABLE 1
#endif

/* カルマン Q [deg^2/s] / R [deg^2] の既定値 */
#define ANGLE_KALMAN_PROCESS_NOISE_DEG2_PER_S  (4.0f)
#define ANGLE_KALMAN_MEAS_NOISE_DEG2           (100.0f)

void Angle_Init(FILE *bt_out);

/* カルマン推定の ON/OFF。OFF 時は hub_imu_get_heading() をそのまま出力。 */
void Angle_SetKalmanEnabled(bool enabled);
bool Angle_IsKalmanEnabled(void);

/* カルマン Q/R を変更（enabled 時のみ有効）。 */
void Angle_SetKalmanNoise(float process_noise_deg2_per_s,
                          float measurement_noise_deg2);

/* hub 方位リセット + カルマン状態リセット。 */
void Angle_ResetHeading(void);

void angle_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif
