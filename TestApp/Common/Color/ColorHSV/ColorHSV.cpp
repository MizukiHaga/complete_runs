/*
 * ColorHSV モジュール 実装
 *  黒/白は反射値。それ以外は基準 HSV ± 許容誤差で判定する。
 */

#include "ColorHSV.h"

void ColorHSV::configure(pbio_port_id_t color_sensor_port)
{
  device_ = pup_color_sensor_get_device(color_sensor_port);
}

bool ColorHSV::isBlack(bool surface)
{
  return is(COLOR_DETECT_BLACK, surface);
}

void ColorHSV::resetBlackEdge()
{
  black_edge_armed_ = false;
  black_edge_was_black_ = false;
}

bool ColorHSV::blackEdgeHit(bool surface)
{
  const bool is_black = isBlack(surface);

  if (is_black) {
    if (black_edge_armed_) {
      return true;
    }
    black_edge_was_black_ = true;
    return false;
  }

  if (black_edge_was_black_) {
    black_edge_armed_ = true;
  }
  return false;
}

void ColorHSV::readHsv(bool surface, uint16_t *h, uint8_t *s, uint8_t *v)
{
  if (device_ == nullptr) {
    if (h != nullptr) *h = 0;
    if (s != nullptr) *s = 0;
    if (v != nullptr) *v = 0;
    return;
  }
  const pup_color_hsv_t hsv = pup_color_sensor_hsv(device_, surface);
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

int32_t ColorHSV::readReflection()
{
  if (device_ == nullptr) {
    return -1;
  }
  return pup_color_sensor_reflection(device_);
}

int ColorHSV::findMapIndex(color_detect_t color)
{
  for (int i = 0; i < color_map_size; i++) {
    if (color_map_detect[i] == color) {
      return i;
    }
  }
  return -1;
}

bool ColorHSV::withinTolerance(uint16_t h, uint8_t s, uint8_t v,
                               const pup_color_hsv_t &ref)
{
  int32_t dh = (int32_t)h - (int32_t)ref.h;
  if (dh < 0) {
    dh = -dh;
  }
  if (dh > 180) {
    dh = 360 - dh;
  }

  int32_t ds = (int32_t)s - (int32_t)ref.s;
  if (ds < 0) {
    ds = -ds;
  }
  int32_t dv = (int32_t)v - (int32_t)ref.v;
  if (dv < 0) {
    dv = -dv;
  }

  return dh <= (int32_t)ToleranceH &&
         ds <= (int32_t)ToleranceS &&
         dv <= (int32_t)ToleranceV;
}

bool ColorHSV::is(color_detect_t color, bool surface)
{
  if (device_ == nullptr) {
    return false;
  }

  if (color == COLOR_DETECT_BLACK) {
    const int32_t reflection = readReflection();
    return reflection >= 0 && reflection <= BlackReflectionMax;
  }
  if (color == COLOR_DETECT_WHITE) {
    const int32_t reflection = readReflection();
    return reflection >= WhiteReflectionMin;
  }
  if (color == COLOR_DETECT_GRAYLINE || color == COLOR_DETECT_NONE ||
      color == COLOR_DETECT_OTHER) {
    return false;
  }

  const int idx = findMapIndex(color);
  if (idx < 0) {
    return false;
  }

  uint16_t h;
  uint8_t s, v;
  readHsv(surface, &h, &s, &v);
  return withinTolerance(h, s, v, color_map_edit[idx]);
}
