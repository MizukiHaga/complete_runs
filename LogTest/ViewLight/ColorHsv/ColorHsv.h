#ifndef LOGTEST_VIEWLIGHT_COLORHSV_H_
#define LOGTEST_VIEWLIGHT_COLORHSV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <pbio/port.h>

/* カラーセンサ / フォースセンサの PUP ポート (実機配線に合わせて変更可) */
#define COLOR_HSV_COLOR_PORT   PBIO_PORT_ID_E
#define COLOR_HSV_FORCE_PORT   PBIO_PORT_ID_D

/* 「押された」と判定する最低限の力 [N]。 */
#define COLOR_HSV_PRESS_FORCE  3.0f

/* 初期化。センサ取得とディスプレイ消灯だけを行う。 */
void ColorHsvLight_Init(pbio_port_id_t color_port, pbio_port_id_t force_port);

/* CRE_CYC で周期起動される検知＆表示タスク。
 * フォースセンサが押された瞬間にカラーセンサの HSV を読み取り、
 * "H<h> S<s> V<v>" を 5x5 ディスプレイに 1 文字ずつ表示する。
 * ブロックせず毎周期描き直すので、表示中でも押下に毎回反応できる。 */
void color_hsv_light_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // LOGTEST_VIEWLIGHT_COLORHSV_H_
