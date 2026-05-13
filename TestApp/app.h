#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>

/* タスク優先度 */
#define MAIN_PRIORITY     5 /* メインタスク */
#define TRACER_PRIORITY   6 /* ライントレースタスク */
#define STRAIGHT_PRIORITY 7 /* 直進タスク（数値が大きいほど低優先度） */

/* タスク周期の定義 */
#define LINE_TRACER_PERIOD  (100 * 1000) /* ライントレースタスク:100msec周期 */
#define STRAIGHT_PERIOD     (100 * 1000) /* 直進タスク:100msec周期 */


#ifndef STACK_SIZE
#define STACK_SIZE      (4096)
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void tracer_task(intptr_t exinf);
extern void straight_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
