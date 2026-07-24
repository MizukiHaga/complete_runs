/*
 * ColorLAB モジュール 実装
 *  RGB を LAB に変換し、lab_map_edit の範囲で色を判定する。
 */

#include "ColorLAB.h"

#include <math.h>

void ColorLAB::configure(pbio_port_id_t color_sensor_port)
{
  device_ = pup_color_sensor_get_device(color_sensor_port);
  if (device_ != nullptr) {
    pup_color_sensor_light_on(device_);
  }
}

void ColorLAB::readRgb(uint16_t *r, uint16_t *g, uint16_t *b)
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

void ColorLAB::readLab(bool surface, float *L, float *a, float *b)
{
  (void)surface;
  uint16_t r, g, blue;
  readRgb(&r, &g, &blue);
  const LabColor lab = ColorLab_rgbToLab(r, g, blue);
  if (L != nullptr) {
    *L = lab.L;
  }
  if (a != nullptr) {
    *a = lab.a;
  }
  if (b != nullptr) {
    *b = lab.b;
  }
}

float ColorLAB::chroma(bool surface)
{
  float L, a, b;
  readLab(surface, &L, &a, &b);
  (void)L;
  return ColorLab_chroma(a, b);
}

int32_t ColorLAB::readReflection()
{
  if (device_ == nullptr) {
    return -1;
  }
  return pup_color_sensor_reflection(device_);
}

int32_t ColorLAB::reflection()
{
  return readReflection();
}

bool ColorLAB::isBetweenBlackAndWhite()
{
  if (device_ == nullptr) {
    return false;
  }
  const int32_t value = readReflection();
  return value > BlackReflectionMax && value < WhiteReflectionMin;
}

int ColorLAB::findMapIndex(color_detect_t color)
{
  for (int i = 0; i < lab_map_size; i++) {
    if (lab_map_detect[i] == color) {
      return i;
    }
  }
  return -1;
}

const LabColorProfile *ColorLAB::findProfile(color_detect_t color)
{
  const int idx = findMapIndex(color);
  if (idx < 0) {
    return nullptr;
  }
  return &lab_map_edit[idx];
}

bool ColorLAB::inLabRange(float value, float lo, float hi)
{
  if (lo <= LabBoundDisabledLo + 1.0f && hi >= LabBoundDisabledHi - 1.0f) {
    return true;
  }
  if (lo <= LabBoundDisabledLo + 1.0f) {
    return value <= hi;
  }
  if (hi >= LabBoundDisabledHi - 1.0f) {
    return value >= lo;
  }
  return value >= lo && value <= hi;
}

bool ColorLAB::matchesProfile(float L, float a, float b,
                              const LabColorProfile &ref)
{
  if (!inLabRange(L, ref.L_lo, ref.L_hi)) {
    return false;
  }
  if (!inLabRange(a, ref.a_lo, ref.a_hi)) {
    return false;
  }
  if (!inLabRange(b, ref.b_lo, ref.b_hi)) {
    return false;
  }
  return true;
}

bool ColorLAB::is(color_detect_t color, bool surface)
{
  if (device_ == nullptr) {
    return false;
  }
  if (color == COLOR_DETECT_GRAYLINE || color == COLOR_DETECT_NONE ||
      color == COLOR_DETECT_OTHER) {
    return false;
  }
  if (color == COLOR_DETECT_WHITE) {
    (void)surface;
    return reflection() >= WhiteReflectionMin;
  }
  if (color == COLOR_DETECT_BLACK) {
    (void)surface;
    const int32_t value = reflection();
    return value >= 0 && value <= BlackReflectionMax;
  }

  float L, a, b;
  readLab(surface, &L, &a, &b);

  const LabColorProfile *profile = findProfile(color);
  if (profile == nullptr) {
    return false;
  }

  return matchesProfile(L, a, b, *profile);
}
