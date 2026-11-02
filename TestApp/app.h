#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>

/* === LRコース切り替え ==========================================
 * ETROBO_COURSE_L を 1 で Rコース、0 で Lコース。
 * 試合前にここだけ変更する。 */
#ifndef ETROBO_COURSE_L
#define ETROBO_COURSE_L 0
// #define ETROBO_COURSE_L 1
#endif

/* === 実行シーン切り替え ==========================================
 * ETROBO_SCENE:
 *   0: FixedRun → Sumo → Nansho1 → Nansho2Kime → Nansho3（本番）
 *   1: Nansho1 のみ
 *   2: Nansho2 のみ（取り込み前）
 *   3: HCR のみ
 *   4: Nansho3 のみ（単体LT開始つき）
 *   5: Nansho1 → Nansho2Kime → Nansho3（FixedRunなし）
 *   6: Test（ケースは Scene/Test/Test.h の TEST_CASE）
 *   7: Nansho3KYOKU（事前曲線計画＋Pure Pursuit。Settings.h参照）
 *   8: FixedRun 完成（FixedRunComplete。ETROBO_COURSE_L で L/R）
 *   9: FixedRun 短縮（FixedRunShort。ETROBO_COURSE_L で L/R）
 *  10: Sumo
 *  11: Nansho2Kime（決め打ち）
 * 試合前にここだけ変更する。 */
#define ETROBO_SCENE_NANSHO2      2
#define ETROBO_SCENE_NANSHO3      4
#define ETROBO_SCENE_FIXEDRUN     8
#define ETROBO_SCENE_SUMO         10
#define ETROBO_SCENE_NANSHO2KIME  11
/* Nansho3補正切替: 0=固定値（旧Honban）、1=エンコーダ＋ジャイロTurnDrift。
 * 旋回・直進・距離・ゲートの設定は両モード共通。KFではない。 */
#ifndef NANSHO3_USE_TURN_DRIFT
#define NANSHO3_USE_TURN_DRIFT 1
#endif
#if NANSHO3_USE_TURN_DRIFT != 0 && NANSHO3_USE_TURN_DRIFT != 1
#error "NANSHO3_USE_TURN_DRIFT must be 0 or 1"
#endif

#ifndef ETROBO_SCENE
#define ETROBO_SCENE 0
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
#define FIXEDRUN_PRIORITY         12 /* FixedRunタスク */
#define SUMO_PRIORITY             13 /* Sumoタスク */

/* タスク周期の定義 */
#define LINE_TRACER_PERIOD       ( 5 * 1000) /* ライントレースタスク:5msec周期 */
#define STRAIGHT_BALANCE_PERIOD  ( 10 * 1000) /* Straight PID 補正タスク:10msec周期 */
#define GYRO_UPDATE_PERIOD       ( 5 * 1000) /* ジャイロ更新タスク:5msec周期 */

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

#ifndef FIXEDRUN_STACK_SIZE
#define FIXEDRUN_STACK_SIZE (8192)
#endif

#ifndef SUMO_STACK_SIZE
#define SUMO_STACK_SIZE (16384)
#endif

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void tracer_task(intptr_t exinf);
extern void Nansho1Task(intptr_t exinf);
extern void Nansho2Task(intptr_t exinf);
extern void Nansho2KimeTask(intptr_t exinf);
extern void Nansho3Task(intptr_t exinf);
extern void Nansho3KYOKUTask(intptr_t exinf);
extern void TestTask(intptr_t exinf);
extern void StraightBalanceTask(intptr_t exinf);
extern void FixedRunTask(intptr_t exinf);
extern void SumoTask(intptr_t exinf);
extern void GyroUpdateTask(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
