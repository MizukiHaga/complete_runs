#ifndef TESTAPP_COLOR_H_
#define TESTAPP_COLOR_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include <spike/pup/colorsensor.h>
}

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

class Color {
public:
  static constexpr int32_t BlackReflectionMax = 10;
  static constexpr int32_t WhiteReflectionMin = 80;
  static constexpr uint8_t MinSaturation = 30;
  static constexpr uint8_t MinValue = 20;

  void configure(pbio_port_id_t color_sensor_port);
  void readHsv(bool surface, uint16_t *h, uint8_t *s, uint8_t *v);
  int32_t readReflection();
  color_detect_t detect(bool surface);
  bool isBlack(bool surface);
  bool isWhite(bool surface);
  bool isRed(bool surface);
  bool isYellow(bool surface);
  bool isGreen(bool surface);
  bool isBlue(bool surface);
  bool is(color_detect_t color, bool surface);

private:
  pup_device_t *device_ = nullptr;
};

#ifndef COLOR_BLACK_REFLECTION_MAX
#define COLOR_BLACK_REFLECTION_MAX Color::BlackReflectionMax
#endif
#ifndef COLOR_WHITE_REFLECTION_MIN
#define COLOR_WHITE_REFLECTION_MIN Color::WhiteReflectionMin
#endif
#define COLOR_MIN_SATURATION Color::MinSaturation
#define COLOR_MIN_VALUE Color::MinValue

#endif  // TESTAPP_COLOR_H_
