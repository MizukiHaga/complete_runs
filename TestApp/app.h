#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>

/* === LRコース切り替え ==========================================
 * ETROBO_COURSE_L を 1 で Lコース、0 で Rコース。
 * 試合前にここだけ変更する。 */
#ifndef ETROBO_COURSE_L
#define ETROBO_COURSE_L 1
#endif

/* === 実行シーン切り替え ==========================================
 * ETROBO_SCENE:
 *   0: HCR → Nansho1 → Nansho2 → Nansho3（本番）
 *   1: Nansho1 のみ
 *   2: Nansho2 のみ
 *   3: HCR のみ
 *   4: Nansho3 のみ
 *   5: Nansho1 → Nansho2（HCRなし）
 *   6: Test → Nansho1 → Nansho2 → Nansho3（LT前段つき）
 *   7: Test のみ（時間制御直進テスト。0°・ヨー補正）
 *   8: Test のみ（直進＋90°旋回の四角走行ループ）
 *   9: Test のみ（前後往復・カルマン確認・台形減速）
 *  10: Test のみ（直進のみ・0°・ヨー補正・台形減速・カルマンログ）
 * 試合前にここだけ変更する。 */
#ifndef ETROBO_SCENE
#define ETROBO_SCENE 2
#endif

/* タスク優先度 */
#define MAIN_PRIORITY              5 /* メインタスク */
#define STRAIGHT_BALANCE_PRIORITY  6 /* Straight PID 補正タスク (Nansho1 より高優先度で確実に回す) */
#define STRAIGHT_PRIORITY          6 /* app.cfg の既存参照用 */
#define TRACER_PRIORITY            7 /* ライントレースタスク */
#define NANSHO1_PRIORITY           8 /* Nansho1タスク */
#define NANSHO2_PRIORITY           9 /* Nansho2タスク */
#define NANSHO3_PRIORITY          10 /* Nansho3タスク */
#define TEST_PRIORITY             11 /* Testタスク */

/* タスク周期の定義 */
#define LINE_TRACER_PERIOD       (5 * 1000) /* ライントレースタスク:5msec周期 */
#define STRAIGHT_BALANCE_PERIOD  ( 10 * 1000) /* Straight PID 補正タスク:10msec周期 */

#ifndef STACK_SIZE
#define STACK_SIZE      (4096)
#endif /* STACK_SIZE */

/* Nansho1Task は PUP デバイス読み出しを行うのでスタック多めに取る。
 * 既存の 4096 ではフォルトする可能性があるため 16384 に拡張。 */
#ifndef NANSHO1_STACK_SIZE
#define NANSHO1_STACK_SIZE (16384)
#endif

#ifndef NANSHO2_STACK_SIZE
#define NANSHO2_STACK_SIZE (16384)
#endif

#ifndef NANSHO3_STACK_SIZE
#define NANSHO3_STACK_SIZE (16384)
#endif

#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE (16384)
#endif

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void tracer_task(intptr_t exinf);
extern void Nansho1Task(intptr_t exinf);
extern void Nansho2Task(intptr_t exinf);
extern void Nansho3Task(intptr_t exinf);
extern void TestTask(intptr_t exinf);
extern void StraightBalanceTask(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
