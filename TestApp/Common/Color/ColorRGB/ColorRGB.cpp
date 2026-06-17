/*
 * ColorRGB モジュール 実装
 *  黒/白は明るさ。それ以外は基準 RGB ± 許容誤差で判定する。
 */

#include "ColorRGB.h"

void ColorRGB::configure(pbio_port_id_t color_sensor_port)
{
  device_ = pup_color_sensor_get_device(color_sensor_port);
}

void ColorRGB::readRgb(uint16_t *r, uint16_t *g, uint16_t *b)
{
  if (device_ == nullptr) {
    if (r != nullptr) *r = 0;
    if (g != nullptr) *g = 0;
    if (b != nullptr) *b = 0;
    return;
  }
  const pup_color_rgb_t rgb = pup_color_sensor_rgb(device_);
  if (r != nullptr) {
    *r = rgb.r;
  }
  if (g != nullptr) {
    *g = rgb.g;
  }
  if (b != nullptr) {
    *b = rgb.b;
  }
}

int32_t ColorRGB::brightness()
{
  if (device_ == nullptr) {
    return -1;
  }
  uint16_t r, g, b;
  readRgb(&r, &g, &b);
  return ((int32_t)r + g + b) * 100 / (RawMax * 3);
}

int ColorRGB::findMapIndex(color_detect_t color)
{
  for (int i = 0; i < rgb_map_size; i++) {
    if (rgb_map_detect[i] == color) {
      return i;
    }
  }
  return -1;
}

bool ColorRGB::withinTolerance(uint16_t r, uint16_t g, uint16_t b,
                               const pup_color_rgb_t &ref)
{
  int32_t dr = (int32_t)r - (int32_t)ref.r;
  if (dr < 0) {
    dr = -dr;
  }
  int32_t dg = (int32_t)g - (int32_t)ref.g;
  if (dg < 0) {
    dg = -dg;
  }
  int32_t db = (int32_t)b - (int32_t)ref.b;
  if (db < 0) {
    db = -db;
  }

  return dr <= (int32_t)ToleranceR &&
         dg <= (int32_t)ToleranceG &&
         db <= (int32_t)ToleranceB;
}

bool ColorRGB::is(color_detect_t color)
{
  if (device_ == nullptr) {
    return false;
  }

  if (color == COLOR_DETECT_BLACK) {
    const int32_t bright = brightness();
    return bright >= 0 && bright <= BlackBrightnessMax;
  }
  if (color == COLOR_DETECT_WHITE) {
    const int32_t bright = brightness();
    return bright >= WhiteBrightnessMin;
  }
  if (color == COLOR_DETECT_NONE || color == COLOR_DETECT_OTHER) {
    return false;
  }

  const int idx = findMapIndex(color);
  if (idx < 0) {
    return false;
  }

  uint16_t r, g, b;
  readRgb(&r, &g, &b);
  return withinTolerance(r, g, b, rgb_map_edit[idx]);
}
