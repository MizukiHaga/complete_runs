#ifndef TESTAPP_NANSHO1_H_
#define TESTAPP_NANSHO1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <pbio/port.h>

#define NANSHO1_ULTRASONIC_PORT  PBIO_PORT_ID_F

#define NANSHO1_LEFT_POWER   (70)
#define NANSHO1_RIGHT_POWER  (65)

/* 超音波待ちの蛇行: 1回の呼び出し時間 [ms]（-1 検出はループ側で確認） */
#define NANSHO1_SERPENTINE_SLICE_MS  (300U)

/* 超音波で「障害物あり」とみなす距離 [mm]（0 と -1 は未検出） */
#define NANSHO1_DETECT_MIN_MM  (1)
#define NANSHO1_DETECT_MAX_MM  (2000)

#define NANSHO1_APPROACH_MM  (300)
#define NANSHO1_POLL_MS      (10U)

void Nansho1_Configure(void);
void Nansho1Task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_NANSHO1_H_
