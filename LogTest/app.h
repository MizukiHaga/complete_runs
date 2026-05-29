#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>

/* タスク優先度 */
#define MAIN_PRIORITY    5 /* メインタスク */
#define ANGLE_PRIORITY     6 /* IMU 角度ログ */
#define MORTER_PRIORITY    7 /* モーター（駆動の有無は Morter.h のマクロ） */
#define DISTANCE_PRIORITY  8 /* 距離センサログ */
#define BATTERY_PRIORITY   9 /* バッテリー残量ログ */
#define SONIC_LIGHT_PRIORITY 10 /* 超音波検知→ハブライト表示 */

/* タスク周期の定義 */
#define ANGLE_PERIOD       (100 * 1000) /* 100 ms */
#define MORTER_PERIOD      (100 * 1000)
#define DISTANCE_PERIOD    (100 * 1000)
#define BATTERY_PERIOD     (1000 * 1000) /* 1 秒 */
#define SONIC_LIGHT_PERIOD (50 * 1000)   /* 50 ms (反応よく) */


#ifndef STACK_SIZE
#define STACK_SIZE      (4096)
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void angle_task(intptr_t exinf);
extern void morter_task(intptr_t exinf);
extern void distance_sensor_task(intptr_t exinf);
extern void battery_task(intptr_t exinf);
extern void sonic_light_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
