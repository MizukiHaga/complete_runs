#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>

/* タスク優先度 */
#define MAIN_PRIORITY              5 /* メインタスク */
#define STRAIGHT_BALANCE_PRIORITY  6 /* Straight PID 補正タスク (Nansho1 より高優先度で確実に回す) */
#define STRAIGHT_PRIORITY          STRAIGHT_BALANCE_PRIORITY /* app.cfg の既存参照用 */
#define TRACER_PRIORITY            6 /* ライントレースタスク */
#define NANSHO1_PRIORITY           7 /* Nansho1タスク */
#define HCR_PRIORITY               8 /* HCR タスク */

/* タスク周期の定義 */
#define LINE_TRACER_PERIOD       (5 * 1000) /* ライントレースタスク:100msec周期 */
#define STRAIGHT_BALANCE_PERIOD  ( 10 * 1000) /* Straight PID 補正タスク:10msec周期 */


#ifndef STACK_SIZE
#define STACK_SIZE      (4096)
#endif /* STACK_SIZE */

/* Nansho1Task は PUP デバイス読み出しを行うのでスタック多めに取る。
 * 既存の 4096 ではフォルトする可能性があるため 16384 に拡張。 */
#ifndef NANSHO1_STACK_SIZE
#define NANSHO1_STACK_SIZE (16384)
#endif

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void line_tracer_cyclic_handler(intptr_t exinf);
extern void Nansho1Task(intptr_t exinf);
extern void StraightBalanceTask(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
