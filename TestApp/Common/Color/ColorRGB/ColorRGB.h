/*
 * ColorRGB モジュール ヘッダ
 *
 *  使い方:
 *    1. LogTest で対象色の R,G,B を計測
 *    2. rgb_map_edit / rgb_map_detect に書き込む
 *    3. gColorRGB.is(COLOR_DETECT_GRAYLINE) で判定
 *
 *  判定: 読み取り値が基準値 ± 許容誤差 (ToleranceR/G/B) 以内なら true
 */

#ifndef TESTAPP_COLOR_RGB_H_
#define TESTAPP_COLOR_RGB_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

#include "ColorDetect.h"

extern "C" {
#include <spike/pup/colorsensor.h>
}

/* LogTest で測った R,G,B。rgb_map_detect と並び・個数を揃える。 */
static constexpr pup_color_rgb_t rgb_map_edit[] = {
  { 450, 470, 500 },  // Gray
  { 640, 690, 720 },  // GrayLine
};
static constexpr color_detect_t rgb_map_detect[] = {
  COLOR_DETECT_GRAY,
  COLOR_DETECT_GRAYLINE,
};
static constexpr int rgb_map_size =
    sizeof(rgb_map_edit) / sizeof(rgb_map_edit[0]);

class ColorRGB {
public:
  static constexpr int32_t RawMax = 1024;
  static constexpr int32_t BlackBrightnessMax = 10;
  static constexpr int32_t WhiteBrightnessMin = 80;
  /* 基準 RGB からの許容誤差 (実機で調整) */
  static constexpr uint16_t ToleranceR = 80U;
  static constexpr uint16_t ToleranceG = 80U;
  static constexpr uint16_t ToleranceB = 80U;

  void configure(pbio_port_id_t color_sensor_port);
  bool isConfigured() const { return device_ != nullptr; }

  void readRgb(uint16_t *r, uint16_t *g, uint16_t *b);
  int32_t brightness();

  bool is(color_detect_t color);
  bool isGrayLine();

private:
  static int findMapIndex(color_detect_t color);
  static bool withinTolerance(uint16_t r, uint16_t g, uint16_t b,
                              const pup_color_rgb_t &ref);

  pup_device_t *device_ = nullptr;
};

#endif  // TESTAPP_COLOR_RGB_H_
