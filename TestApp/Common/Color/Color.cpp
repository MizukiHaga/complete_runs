#include "Color.h"

void Color::configure(pbio_port_id_t color_sensor_port)
{
  device_ = pup_color_sensor_get_device(color_sensor_port);
}

void Color::readHsv(bool surface, uint16_t *h, uint8_t *s, uint8_t *v)
{
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

int32_t Color::readReflection()
{
  return pup_color_sensor_reflection(device_);
}

bool Color::isBlack(bool surface)
{
  (void)surface;
  const int32_t reflection = readReflection();
  return reflection >= 0 && reflection <= BlackReflectionMax;
}

bool Color::isWhite(bool surface)
{
  (void)surface;
  const int32_t reflection = readReflection();
  return reflection >= WhiteReflectionMin;
}

bool Color::isRed(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  readHsv(surface, &h, &s, &v);
  if (s < MinSaturation || v < MinValue) {
    return false;
  }
  return h <= 25 || h >= 335;
}

bool Color::isYellow(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  readHsv(surface, &h, &s, &v);
  if (s < MinSaturation || v < MinValue) {
    return false;
  }
  return h >= 45 && h <= 75;
}

bool Color::isGreen(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  readHsv(surface, &h, &s, &v);
  if (s < MinSaturation || v < MinValue) {
    return false;
  }
  return h >= 95 && h <= 145;
}

bool Color::isBlue(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  readHsv(surface, &h, &s, &v);
  if (s < MinSaturation || v < MinValue) {
    return false;
  }
  return h >= 200 && h <= 260;
}

color_detect_t Color::detect(bool surface)
{
  if (isBlack(surface)) {
    return COLOR_DETECT_BLACK;
  }
  if (isWhite(surface)) {
    return COLOR_DETECT_WHITE;
  }

  uint16_t h;
  uint8_t s, v;
  readHsv(surface, &h, &s, &v);
  if (s < MinSaturation || v < MinValue) {
    return COLOR_DETECT_NONE;
  }
  if (isRed(surface)) {
    return COLOR_DETECT_RED;
  }
  if (isYellow(surface)) {
    return COLOR_DETECT_YELLOW;
  }
  if (isGreen(surface)) {
    return COLOR_DETECT_GREEN;
  }
  if (isBlue(surface)) {
    return COLOR_DETECT_BLUE;
  }
  return COLOR_DETECT_OTHER;
}

bool Color::is(color_detect_t color, bool surface)
{
  return detect(surface) == color;
}
