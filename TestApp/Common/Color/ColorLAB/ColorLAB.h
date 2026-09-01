/*
 * ColorLAB モジュール ヘッダ
 *
 *  使い方:
 *    1. LogTest で対象色の L,a,b を計測し、最小値・最大値を記録
 *    2. lab_map_edit に LAB_RANGE(最小, 最大, ..., margin) を書く
 *    3. gColorLAB.is(COLOR_DETECT_SKYBLUE, true) などで判定
 *
 *  LAB_RANGE は実測の min/max (順不同可) に margin を足して判定範囲を作る。
 *
 *  片側だけ使う軸は LabBoundDisabledLo / LabBoundDisabledHi を指定する。
 */

#ifndef TESTAPP_COLOR_LAB_H_
#define TESTAPP_COLOR_LAB_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

#include "ColorDetect.h"
#include "ColorLabMath.h"

extern "C" {
#include <spike/pup/colorsensor.h>
}

static constexpr float LabBoundDisabledLo = -999.0f;
static constexpr float LabBoundDisabledHi = 999.0f;
/* LogTest の min/max に足すマージン [各軸に同値] */
static constexpr float LabRangeDefaultMargin = 1.0f;

struct LabColorProfile {
  float L_lo;
  float L_hi;
  float a_lo;
  float a_hi;
  float b_lo;
  float b_hi;
};

static constexpr float LabRange_expandLo(float lo, float margin)
{
  return lo <= LabBoundDisabledLo + 1.0f ? lo : lo - margin;
}

static constexpr float LabRange_expandHi(float hi, float margin)
{
  return hi >= LabBoundDisabledHi - 1.0f ? hi : hi + margin;
}

static constexpr float LabRange_min2(float x, float y)
{
  return x < y ? x : y;
}

static constexpr float LabRange_max2(float x, float y)
{
  return x > y ? x : y;
}

static constexpr LabColorProfile LabRange_make(float L_lo, float L_hi,
                                               float a_lo, float a_hi,
                                               float b_lo, float b_hi,
                                               float margin)
{
  const float Llo = LabRange_expandLo(LabRange_min2(L_lo, L_hi), margin);
  const float Lhi = LabRange_expandHi(LabRange_max2(L_lo, L_hi), margin);
  const float alo = LabRange_expandLo(LabRange_min2(a_lo, a_hi), margin);
  const float ahi = LabRange_expandHi(LabRange_max2(a_lo, a_hi), margin);
  const float blo = LabRange_expandLo(LabRange_min2(b_lo, b_hi), margin);
  const float bhi = LabRange_expandHi(LabRange_max2(b_lo, b_hi), margin);

  return LabColorProfile{Llo, Lhi, alo, ahi, blo, bhi};
}

#define LAB_RANGE(L_lo, L_hi, a_lo, a_hi, b_lo, b_hi, margin) \
  LabRange_make((L_lo), (L_hi), (a_lo), (a_hi), (b_lo), (b_hi), (margin))

/* LogTest の L,a,b 最小・最大 (順不同可)。lab_map_detect と並び・個数を揃える。 */
// 學校
static constexpr LabColorProfile lab_map_edit[] = {
  /*  L_lo    L_hi   a_lo   a_hi   b_lo   b_hi  margin */
  LAB_RANGE(-999,   40.0f, -2.0f,  2.0f, -2.0f,  2.0f,  2.0f),  // Black (unused: reflection)
  LAB_RANGE( 30.0f,  75.0f, -2.0f,  -6.0f, -17.0f, -9.0f,  3.0f),  // Skyblue
  LAB_RANGE( 18.0f,  65.0f, -1.0f,  0.0f, -3.0f, -1.0f,  3.0f),  // BlackLine
  LAB_RANGE( 5.0f,  40.0f, 15.5f,  42.0f,  3.0f, 14.0f,  3.0f),  // RED
  LAB_RANGE( 66.0f,  75.0f, 4.0f,  1.0f,  28.0f, 30.0f,  3.0f),  // YELLOW
  LAB_RANGE( 11.0f,  45.0f, -2.0f,  -5.0f, -10.0f, -33.0f,  3.0f),  // BLUE
};
// 本番
// static constexpr LabColorProfile lab_map_edit[] = {
// //   /*  L_lo    L_hi   a_lo   a_hi   b_lo   b_hi  margin */
//   LAB_RANGE(-999,   40.0f, -2.0f,  2.0f, -2.0f,  2.0f,  2.0f),  // Black (unused: reflection)
//   LAB_RANGE( 50.0f,  75.0f, -8.0f,  -6.0f, -17.0f, -12.0f,  3.0f),  // Skyblue テスト30.0f,  75.0f, -2.0f,  -6.0f, -17.0f, -9.0f,  3.0f
//   LAB_RANGE( 40.0f,  65.0f, -6.0f,  0.0f, -4.0f, -1.0f,  3.0f),  // BlackLine　テスト18.0f,  65.0f, -1.0f,  0.0f, -3.0f, -1.0f,  3.0f
//   LAB_RANGE( 40.0f,  50.0f, 57.0f,  38.0f,  24.0f, 9.0f,  3.0f),  // RED　テスト5.0f,  40.0f, 15.5f,  42.0f,  3.0f, 14.0f,  3.0f
//   LAB_RANGE( 83.0f,  90.0f, -7.0f,  -6.0f,  37.0f, 42.0f,  3.0f),  // YELLOW　テスト66.0f,  75.0f, 4.0f,  1.0f,  28.0f, 30.0f,  3.0f
//   LAB_RANGE( 21.0f,  45.0f, -5.0f,  -3.0f, -33.0f, -27.0f,  3.0f),  // BLUE　テスト11.0f,  45.0f, -2.0f,  -5.0f, -10.0f, -33.0f,  3.0f
//   LAB_RANGE( 30.0f, 31.0f, -23.0f ,-21.0f, 8.0f, 9.0f, 3.0f),
// };

static constexpr color_detect_t lab_map_detect[] = {
  COLOR_DETECT_BLACK,
  COLOR_DETECT_SKYBLUE,
  COLOR_DETECT_BLACKLINE,
  COLOR_DETECT_RED,
  COLOR_DETECT_YELLOW,
  COLOR_DETECT_BLUE,
};

static constexpr int lab_map_size =
    sizeof(lab_map_edit) / sizeof(lab_map_edit[0]);

class ColorLAB {
public:
  static constexpr int32_t BlackReflectionMax = 20;
  static constexpr int32_t WhiteReflectionMin = 80;

  void configure(pbio_port_id_t color_sensor_port);

  int32_t reflection();
  bool isBetweenBlackAndWhite();
  bool is(color_detect_t color, bool surface);

  void readLab(bool surface, float *L, float *a, float *b);
  float chroma(bool surface);

private:
  static int findMapIndex(color_detect_t color);
  static const LabColorProfile *findProfile(color_detect_t color);
  static bool inLabRange(float value, float lo, float hi);
  static bool matchesProfile(float L, float a, float b,
                             const LabColorProfile &ref);

  void readRgb(uint16_t *r, uint16_t *g, uint16_t *b);
  int32_t readReflection();

  pup_device_t *device_ = nullptr;
};

#ifndef COLOR_BLACK_REFLECTION_MAX
#define COLOR_BLACK_REFLECTION_MAX ColorLAB::BlackReflectionMax
#endif
#ifndef COLOR_WHITE_REFLECTION_MIN
#define COLOR_WHITE_REFLECTION_MIN ColorLAB::WhiteReflectionMin
#endif

#endif  // TESTAPP_COLOR_LAB_H_
