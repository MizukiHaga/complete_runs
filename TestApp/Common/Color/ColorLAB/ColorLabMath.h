#ifndef TESTAPP_COLOR_LAB_MATH_H_
#define TESTAPP_COLOR_LAB_MATH_H_

#include <math.h>
#include <stdint.h>

/* SPIKE カラーセンサ RGB (0〜1024) を CIE LAB (D65) に変換する。 */

struct LabColor {
  float L;
  float a;
  float b;
};

inline float ColorLab_srgbToLinear(float channel)
{
  if (channel <= 0.04045f) {
    return channel / 12.92f;
  }
  return powf((channel + 0.055f) / 1.055f, 2.4f);
}

inline float ColorLab_fLab(float t)
{
  const float delta = 6.0f / 29.0f;
  const float delta3 = delta * delta * delta;
  if (t > delta3) {
    return cbrtf(t);
  }
  return t / (3.0f * delta * delta) + (4.0f / 29.0f);
}

inline LabColor ColorLab_rgbToLab(uint16_t r, uint16_t g, uint16_t b)
{
  static constexpr float kRawMax = 1024.0f;
  static constexpr float kXn = 0.95047f;
  static constexpr float kYn = 1.00000f;
  static constexpr float kZn = 1.08883f;

  const float rf = ColorLab_srgbToLinear((float)r / kRawMax);
  const float gf = ColorLab_srgbToLinear((float)g / kRawMax);
  const float bf = ColorLab_srgbToLinear((float)b / kRawMax);

  const float x = 0.4124564f * rf + 0.3575761f * gf + 0.1804375f * bf;
  const float y = 0.2126729f * rf + 0.7151522f * gf + 0.0721750f * bf;
  const float z = 0.0193339f * rf + 0.1191920f * gf + 0.9503041f * bf;

  const float fx = ColorLab_fLab(x / kXn);
  const float fy = ColorLab_fLab(y / kYn);
  const float fz = ColorLab_fLab(z / kZn);

  LabColor lab;
  lab.L = 116.0f * fy - 16.0f;
  lab.a = 500.0f * (fx - fy);
  lab.b = 200.0f * (fy - fz);
  return lab;
}

inline float ColorLab_chroma(float a, float b)
{
  return sqrtf(a * a + b * b);
}

#endif  // TESTAPP_COLOR_LAB_MATH_H_
