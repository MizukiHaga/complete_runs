//#ifdef __cplusplus
//extern "C" {
//#endif

/* 直進時の左右モータパワー（-100〜100、機械差は左右で別値にする） */
#define STRAIGHT_LEFT_POWER   (70)
#define STRAIGHT_RIGHT_POWER  (65)

/* straight_task が Straight_RunFor で直進する時間 [ms] */
#define STRAIGHT_TIME  (2000U)

/* 停止直前の減速に使う時間 [ms]（残りは定速）。全体より長い場合は全区間を減速に充てる */
#define STRAIGHT_DECEL_MS      (800U)
/* 減速中のモータ出力更新間隔 [ms]（小さいほどなめらか） */
#define STRAIGHT_DECEL_STEP_MS (40U)

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

/* Straight_RunFor で指定可能な時間の上限 [ms]
 * dly_tsk の上限 TMAX_RELTIM (4,000,000,000 us = 約66分40秒) に合わせる */
#define STRAIGHT_MAX_DURATION_MS  (4000000U)

extern void Straight_Configure(pbio_port_id_t left_motor_port,
                                 pbio_port_id_t right_motor_port);
extern void Straight_Run(int left_power, int right_power);
extern void Straight_Stop(void);

/* 指定パワーで duration_ms ミリ秒だけ直進し、終わりに向けて徐々に減速して停止する (ブロッキング)。
 * duration_ms が 0 の場合は走行せずに停止のみ行う。
 * 上限は STRAIGHT_MAX_DURATION_MS にクランプされる。 */
extern void Straight_RunFor(int left_power, int right_power, uint32_t duration_ms);

/* 蛇行走行: 左右パワー差を phase_ms ごとに切り替える */
#define STRAIGHT_SERPENTINE_STEER      (15)
#define STRAIGHT_SERPENTINE_PHASE_MS   (300U)
#define STRAIGHT_SERPENTINE_TIME       (2000U)
/* total_ms が 0 のとき時間無制限で走り続ける */
#define STRAIGHT_SERPENTINE_FOREVER    (0U)

/* 10ms ごとに呼ばれる。true を返すと蛇行を止める。不要なら nullptr */
typedef bool (*straight_serpentine_poll_fn)(void);

extern void Straight_RunSerpentine(int left_base, int right_base, int steer,
                                   uint32_t phase_ms, uint32_t total_ms,
                                   straight_serpentine_poll_fn poll);

/* total_ms 省略版: 時間無制限で蛇行 */
extern void Straight_RunSerpentineLoop(int left_base, int right_base, int steer,
                                       uint32_t phase_ms,
                                       straight_serpentine_poll_fn poll);

//#ifdef __cplusplus
//}
//#endif
