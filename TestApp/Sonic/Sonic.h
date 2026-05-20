#ifndef TESTAPP_SONIC_H_
#define TESTAPP_SONIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

/* 距離が読めなかったとき（未検出/エラー）に返る値 */
#define SONIC_DISTANCE_INVALID  (-1)

void Sonic_Configure(pbio_port_id_t ultrasonic_port);

/* 距離 [mm]。未検出/エラー時は SONIC_DISTANCE_INVALID */
int32_t Sonic_ReadDistanceMm(void);

/* 何かが見えているか（SONIC_DISTANCE_INVALID でないか） */
bool Sonic_IsDetected(void);

/* threshold_mm 以下に何かがあるか（未検出は false） */
bool Sonic_IsCloserThan(int32_t threshold_mm);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_SONIC_H_
