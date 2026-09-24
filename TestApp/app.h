/*
仏説摩訶般若波羅蜜多心経
観自在菩薩

行深般若波羅蜜多時

照見五蘊皆空

度一切苦厄

舎利子

色不異空

空不異色

色即是空

空即是色

受想行識

亦復如是

舎利子

是諸法空相

不生不滅

不垢不浄

不増不減

是故空中

無色無受想行識

無眼耳鼻舌身意

無色声香味触法

無眼界乃至無意識界

無無明

亦無無明尽

乃至無老死

亦無老死尽

無苦集滅道

無智亦無得

以無所得故

菩提薩埵

依般若波羅蜜多故

心無罣礙

無罣礙故

無有恐怖

遠離一切顛倒夢想

究竟涅槃

三世諸仏

依般若波羅蜜多故

得阿耨多羅三藐三菩提

故知般若波羅蜜多

是大神呪

是大明呪

是無上呪

是無等等呪

能除一切苦

真実不虚

故説般若波羅蜜多呪

即説呪曰

羯諦

羯諦

波羅羯諦

波羅僧羯諦

菩提薩婆訶

般若心経
*/
#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>

/* === LRコース切り替え ==========================================
 * ETROBO_COURSE_L を 1 で Rコース、0 で Lコース。
 * 試合前にここだけ変更する。 */
#ifndef ETROBO_COURSE_L
#define ETROBO_COURSE_L 1
// #define ETROBO_COURSE_L 1
#endif

/* === 実行シーン切り替え ==========================================
 * ETROBO_SCENE:
 *   0: HCR → Nansho1 → Nansho2 → Nansho3（本番）
 *   1: Nansho1 のみ
 *   2: Nansho2 のみ22w
 *   3: HCR のみ
 *   4: Nansho3 のみ
 *   5: Nansho1 → Nansho2 → Nansho3（HCRなし）
 *   6: Test → Nansho1 → Nansho2 → Nansho3（LT前段つき）
 *   7: Test のみ（時間制御直進テスト。0°・ヨー補正）
 *   8: Test のみ（直進＋90°旋回の四角走行ループ）
 *   9: Test のみ（前後往復・カルマン確認・台形減速）
 *   10: Test のみ（直進のみ・0°・ヨー補正・台形減速・カルマンログ）
 *   11: Test のみ（Nansho3 直進と同じパワー・PI・0°・減速なし）
 *   12: Test のみ（直進だけ。Nansho3 と同じ 1000 枠・台形減速・coast。旋回なし）
 *   13: Nansho3 のみ（単体LT開始つき。旧 3→1 は廃止）
 * 試合前にここだけ変更する。 */
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
