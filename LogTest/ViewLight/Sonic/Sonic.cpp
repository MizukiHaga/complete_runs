#include "app.h"
#include "Sonic/Sonic.h"

#include <spike/pup/ultrasonicsensor.h>

extern "C" {
#include <spike/hub/light.h>
#include <spike/hub/display.h>
}

/* Bluetooth ログ出力先 (任意)。SonicLight_Init で受け取って保持。 */
static FILE *output_file = nullptr;

/* PUP 超音波センサデバイス。SonicLight_Init で取得して以降使い回す。 */
static pup_device_t *fg_ultrasonic = nullptr;

/* 直前ループでライトを点けていたかの状態。
 * 状態が変わった瞬間だけ hub_light_* を呼ぶことで通信を抑える。 */
static bool light_is_on = false;

void SonicLight_Init(FILE *bt_out, pbio_port_id_t port)
{
  output_file = bt_out;
  fg_ultrasonic = pup_ultrasonic_sensor_get_device(port);
  hub_light_off();
  light_is_on = false;
  /* 初期表示は「未検知」の '0'。
   * hub_display_number(0) は 5x2 フォントを並べると "00" が
   * 塗り潰しのベタ塗りに見えてしまうので、5x5 のキャラクタ
   * フォントで 1 桁だけ表示する hub_display_char を使う。 */
  hub_display_char('0');
}

/* 距離を測り SONIC_LIGHT_DETECT_MM 以下なら検知扱いとして
 *   - ハブ内蔵ライトを赤点灯
 *   - 5x5 ディスプレイに "1"
 * 未検知のときは
 *   - ライト消灯
 *   - ディスプレイに "0"
 * を表示する。
 *   - distance_mm <= 0 はセンサエラー扱いとして「検知なし」。
 *   - 状態が変わったときだけ hub_light_* / hub_display_number を呼ぶ。
 *   - output_file が設定されていれば距離と検知フラグをログ出力する。 */
void sonic_light_task(intptr_t unused)
{
  (void)unused;
  if (fg_ultrasonic == nullptr) {
    return;
  }

  const int32_t distance_mm = pup_ultrasonic_sensor_distance(fg_ultrasonic);
  const bool detected = (distance_mm > 0 && distance_mm <= SONIC_LIGHT_DETECT_MM);

  /* ライトは状態が変わった時だけ切り替える (LED コマンド削減)。 */
  if (detected != light_is_on) {
    if (detected) {
      hub_light_on_color(PBIO_COLOR_RED);
    } else {
      hub_light_off();
    }
    light_is_on = detected;
  }

  /* ディスプレイは毎周期上書きする。
   * Pybricks の「ユーザプログラム実行中」アニメ (40ms 周期でぐるぐる点灯)
   * が hub_display_char 後にも継続して LED 行列を書き換えるため、
   * 1 回だけ表示するとすぐ上書きされてしまう。
   * hub_display_char() は内部で pbio_light_matrix_stop_animation() を
   * 呼ぶので、毎周期呼ぶことでアニメを止め続け、表示を維持する。 */
  hub_display_char(detected ? '1' : '0');

  if (output_file != nullptr) {
    fprintf(output_file, "SONIC_LIGHT,%d,%d\n",
            (int)distance_mm, detected ? 1 : 0);
  }
}
