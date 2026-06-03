#include "app.h"
#include "ColorHsv/ColorHsv.h"

#include <spike/pup/colorsensor.h>
#include <spike/pup/forcesensor.h>

extern "C" {
#include <spike/hub/display.h>
}

#include <cstdio>
#include <cstring>

/* PUP デバイス。ColorHsvLight_Init で取得して以降使い回す。 */
static pup_device_t *fg_color = nullptr;
static pup_device_t *fg_force = nullptr;

/* 直前ループで押されていたか。立ち上がり (押した瞬間) だけ反応させる。 */
static bool was_pressed = false;

/* 表示用の文字列バッファと、1 文字ずつ送る状態。 */
static char disp_buf[32];
static int  disp_len = 0;   /* 表示中の文字列長。0 のとき非表示。 */
static int  disp_idx = 0;   /* 今表示している文字位置。 */
static int  frame_cnt = 0;  /* 同じ文字を表示し続けた周期数。 */

/* 1 文字を何周期表示し続けるか。周期 50ms × 10 = 0.5 秒/文字。 */
#define COLOR_HSV_FRAMES_PER_CHAR 10

void ColorHsvLight_Init(pbio_port_id_t color_port, pbio_port_id_t force_port)
{
  fg_color = pup_color_sensor_get_device(color_port);
  fg_force = pup_force_sensor_get_device(force_port);
  was_pressed = false;
  disp_len = 0;
  disp_idx = 0;
  frame_cnt = 0;
  hub_display_off();
}

/* フォースセンサが押された瞬間にカラーセンサの HSV を読み取り、
 * "H<h> S<s> V<v>" を 5x5 ディスプレイに 1 文字ずつ表示する。
 *
 * 表示はブロックせず、毎周期 1 文字を hub_display_char で描き直す方式。
 * Pybricks の「実行中」アニメ (40ms 周期) が LED 行列を上書きするため、
 * hub_display_text_scroll で一度だけ書いても消えてしまう。hub_display_char
 * は内部でアニメを止めるので、毎周期呼んでアニメを抑え続け表示を維持する。
 *
 *   - 押した瞬間 (立ち上がり) だけ HSV を読み直し、先頭から表示し直す。
 *     → 表示中に押しても毎回すぐ最新値に切り替わる (再起動不要)。 */
void color_hsv_light_task(intptr_t unused)
{
  (void)unused;

  /* どちらかのセンサが取れていない＝ポート設定ミス。
   * 'E' を常時表示して実機画面で気づけるようにする。 */
  if (fg_color == nullptr || fg_force == nullptr) {
    hub_display_char('E');
    return;
  }

  const bool pressed = pup_force_sensor_pressed(fg_force, COLOR_HSV_PRESS_FORCE);

  /* 押した瞬間 (立ち上がり) に HSV を読み取り、表示文字列を作り直す。 */
  if (pressed && !was_pressed) {
    const pup_color_hsv_t hsv = pup_color_sensor_hsv(fg_color, true);

    /* 末尾に空白を足して、最後の文字の後に一瞬の消灯を入れる。 */
    snprintf(disp_buf, sizeof(disp_buf), "H%d S%d V%d ", hsv.h, hsv.s, hsv.v);
    disp_len = (int)strlen(disp_buf);
    disp_idx = 0;
    frame_cnt = 0;
  }
  was_pressed = pressed;

  /* 表示中なら、現在の文字を毎周期描き直す (アニメ対策)。 */
  if (disp_len > 0) {
    hub_display_char(disp_buf[disp_idx]);

    if (++frame_cnt >= COLOR_HSV_FRAMES_PER_CHAR) {
      frame_cnt = 0;
      if (++disp_idx >= disp_len) {
        /* 末尾まで表示し終えたら待機状態へ。 */
        disp_len = 0;
        disp_idx = 0;
      }
    }
  } else {
    /* 待機中は '.' を出しておく。これが見えていれば
     * 「プログラム動作中・ポートOK・押し待ち」が確認できる。
     * 見えるのに押しても変わらない場合は押下判定 (force しきい値) を疑う。 */
    hub_display_char('.');
  }
}
