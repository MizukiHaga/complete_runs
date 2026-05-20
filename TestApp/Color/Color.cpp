#include "Color.h"

#include <spike/pup/colorsensor.h>

static pup_device_t *fg_color_sensor = nullptr;

void Color_Configure(pbio_port_id_t color_sensor_port)
{
  fg_color_sensor = pup_color_sensor_get_device(color_sensor_port);
}

void Color_ReadHsv(bool surface, uint16_t *h, uint8_t *s, uint8_t *v)
{
  const pup_color_hsv_t hsv = pup_color_sensor_hsv(fg_color_sensor, surface);
  if (h != nullptr) {
    *h = hsv.h;
  }
  if (s != nullptr) {
    *s = hsv.s;
  }
  if (v != nullptr) {
    *v = hsv.v;
  }
}

int32_t Color_ReadReflection(void)
{
  return pup_color_sensor_reflection(fg_color_sensor);
}

bool Color_IsBlack(bool surface)
{
  (void)surface;
  const int32_t reflection = Color_ReadReflection();
  return reflection >= 0 && reflection <= COLOR_BLACK_REFLECTION_MAX;
}

bool Color_IsWhite(bool surface)
{
  (void)surface;
  const int32_t reflection = Color_ReadReflection();
  return reflection >= COLOR_WHITE_REFLECTION_MIN;
}

bool Color_IsRed(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  Color_ReadHsv(surface, &h, &s, &v);
  if (s < COLOR_MIN_SATURATION || v < COLOR_MIN_VALUE) {
    return false;
  }
  return h <= 25 || h >= 335;
}

bool Color_IsYellow(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  Color_ReadHsv(surface, &h, &s, &v);
  if (s < COLOR_MIN_SATURATION || v < COLOR_MIN_VALUE) {
    return false;
  }
  return h >= 45 && h <= 75;
}

bool Color_IsGreen(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  Color_ReadHsv(surface, &h, &s, &v);
  if (s < COLOR_MIN_SATURATION || v < COLOR_MIN_VALUE) {
    return false;
  }
  return h >= 95 && h <= 145;
}

bool Color_IsBlue(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  Color_ReadHsv(surface, &h, &s, &v);
  if (s < COLOR_MIN_SATURATION || v < COLOR_MIN_VALUE) {
    return false;
  }
  return h >= 200 && h <= 260;
}

color_detect_t Color_Detect(bool surface)
{
  if (Color_IsBlack(surface)) {
    return COLOR_DETECT_BLACK;
  }
  if (Color_IsWhite(surface)) {
    return COLOR_DETECT_WHITE;
  }

  uint16_t h;
  uint8_t s, v;
  Color_ReadHsv(surface, &h, &s, &v);
  if (s < COLOR_MIN_SATURATION || v < COLOR_MIN_VALUE) {
    return COLOR_DETECT_NONE;
  }
  if (Color_IsRed(surface)) {
    return COLOR_DETECT_RED;
  }
  if (Color_IsYellow(surface)) {
    return COLOR_DETECT_YELLOW;
  }
  if (Color_IsGreen(surface)) {
    return COLOR_DETECT_GREEN;
  }
  if (Color_IsBlue(surface)) {
    return COLOR_DETECT_BLUE;
  }
  return COLOR_DETECT_OTHER;
}

bool Color_Is(color_detect_t color, bool surface)
{
  return Color_Detect(surface) == color;
}
