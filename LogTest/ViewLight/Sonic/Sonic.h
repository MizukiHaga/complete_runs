#ifndef LOGTEST_VIEWLIGHT_SONIC_H_
#define LOGTEST_VIEWLIGHT_SONIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <pbio/port.h>
#include <stdio.h>

/* 超音波センサの PUP ポート (実機配線に合わせて変更可) */
#define SONIC_LIGHT_PORT       PBIO_PORT_ID_F

/* 検知判定の最大距離 [mm]。これ以下なら「検知」とみなしてライトを点灯。 */
#define SONIC_LIGHT_DETECT_MM  2100

/* 初期化。bt_out は Bluetooth ログ出力先 (nullptr ならログを吐かない)。 */
void SonicLight_Init(FILE *bt_out, pbio_port_id_t port);

/* CRE_CYC で周期起動されるタスク。
 * 1 周期ごとに距離を測り、検知の有無でハブ内蔵ライトを切り替える。 */
void sonic_light_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // LOGTEST_VIEWLIGHT_SONIC_H_
