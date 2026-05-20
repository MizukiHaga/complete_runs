#ifndef TESTAPP_COLOR_H_
#define TESTAPP_COLOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

/* 検出した色 */
typedef enum {
  COLOR_DETECT_NONE = 0,
  COLOR_DETECT_BLACK,
  COLOR_DETECT_WHITE,
  COLOR_DETECT_RED,
  COLOR_DETECT_YELLOW,
  COLOR_DETECT_GREEN,
  COLOR_DETECT_BLUE,
  COLOR_DETECT_OTHER,
} color_detect_t;

/* 黒判定: 反射値がこの値以下（LineTracer と揃える） */
#ifndef COLOR_BLACK_REFLECTION_MAX
#define COLOR_BLACK_REFLECTION_MAX  (10)
#endif

/* 白判定: 反射値がこの値以上 */
#ifndef COLOR_WHITE_REFLECTION_MIN
#define COLOR_WHITE_REFLECTION_MIN  (80)
#endif

/* HSV 判定の最低彩度・明度（これ未満は COLOR_DETECT_NONE） */
#define COLOR_MIN_SATURATION  (30)
#define COLOR_MIN_VALUE       (20)

void Color_Configure(pbio_port_id_t color_sensor_port);

/* 生の HSV（表面=true / 外部光源=false） */
void Color_ReadHsv(bool surface, uint16_t *h, uint8_t *s, uint8_t *v);

/* 反射値 0〜100（エラー時 -1） */
int32_t Color_ReadReflection(void);

/* いまの色を1つに分類 */
color_detect_t Color_Detect(bool surface);

bool Color_IsBlack(bool surface);
bool Color_IsWhite(bool surface);
bool Color_IsRed(bool surface);
bool Color_IsYellow(bool surface);
bool Color_IsGreen(bool surface);
bool Color_IsBlue(bool surface);
bool Color_Is(color_detect_t color, bool surface);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_COLOR_H_
