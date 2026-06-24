/*
 * ColorHSV モジュール ヘッダ
 *
 *  使い方:
 *    1. LogTest で対象色の H,S,V を計測
 *    2. color_map_edit / color_map_detect に書き込む
 *    3. gColorHSV.is(COLOR_DETECT_SKYBLUE, true) で判定
 *
 *  判定: 読み取り値が基準値 ± 許容誤差 (ToleranceH/S/V) 以内なら true
 */

#ifndef TESTAPP_COLOR_HSV_H_
#define TESTAPP_COLOR_HSV_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include <spike/pup/colorsensor.h>
}

typedef enum {
  COLOR_DETECT_NONE = 0,
  COLOR_DETECT_BLACK,
  COLOR_DETECT_WHITE,
  COLOR_DETECT_GRAY,
  COLOR_DETECT_RED,
  COLOR_DETECT_YELLOW,
  COLOR_DETECT_GREEN,
  COLOR_DETECT_BLUE,
  COLOR_DETECT_SKYBLUE,
  COLOR_DETECT_GRAYLINE,
  COLOR_DETECT_OTHER,
} color_detect_t;

/* LogTest で測った H,S,V。color_map_detect と並び・個数を揃える。 */
static constexpr pup_color_hsv_t color_map_edit[] = {
  { 352, 91, 95 },  // Red
  {  56, 53, 100 }, // Yellow
  { 155, 62, 46 },  // Green
  { 212, 93, 72 },  // Blue
  { 215, 20, 90 },  // Skyblue
  {   0, 10, 40 },  // Gray
};
static constexpr color_detect_t color_map_detect[] = {
  COLOR_DETECT_RED,
  COLOR_DETECT_YELLOW,
  COLOR_DETECT_GREEN,
  COLOR_DETECT_BLUE,
  COLOR_DETECT_SKYBLUE,
  COLOR_DETECT_GRAY,
};
static constexpr int color_map_size =
    sizeof(color_map_edit) / sizeof(color_map_edit[0]);

class ColorHSV {
public:
  static constexpr int32_t BlackReflectionMax = 10;
  static constexpr int32_t WhiteReflectionMin = 80;
  /* 基準 HSV からの許容誤差 (実機で調整) */
  static constexpr uint16_t ToleranceH = 15U;
  static constexpr uint8_t  ToleranceS = 5U;
  static constexpr uint8_t  ToleranceV = 20U;

  void configure(pbio_port_id_t color_sensor_port);

  bool is(color_detect_t color, bool surface);
  bool isBlack(bool surface);
  void resetBlackEdge();
  bool blackEdgeHit(bool surface);

private:
  static int findMapIndex(color_detect_t color);
  static bool withinTolerance(uint16_t h, uint8_t s, uint8_t v,
                              const pup_color_hsv_t &ref);

  void readHsv(bool surface, uint16_t *h, uint8_t *s, uint8_t *v);
  int32_t readReflection();

  pup_device_t *device_ = nullptr;
  bool black_edge_armed_ = false;
  bool black_edge_was_black_ = false;
};

#ifndef COLOR_BLACK_REFLECTION_MAX
#define COLOR_BLACK_REFLECTION_MAX ColorHSV::BlackReflectionMax
#endif
#ifndef COLOR_WHITE_REFLECTION_MIN
#define COLOR_WHITE_REFLECTION_MIN ColorHSV::WhiteReflectionMin
#endif

#endif  // TESTAPP_COLOR_HSV_H_
