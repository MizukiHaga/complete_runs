/*
 * Color モジュール ヘッダ
 *  カラーセンサ (PUP ColorSensor) の判定をまとめたモジュール。
 *  反射値ベースで黒/白を判定し、HSV ベースで色を分類する。
 *
 *  使い方:
 *    Color gColor;
 *    gColor.configure(PBIO_PORT_ID_E);
 *    if (gColor.isBlack(true)) { ... }
 */

#ifndef TESTAPP_COLOR_H_
#define TESTAPP_COLOR_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

/* PUP の C API は C リンクで提供されているので extern "C" で囲む */
extern "C" {
#include <spike/pup/colorsensor.h>
}

/* 検出した色を 1 つに分類するための列挙。
 * detect() / is() が返す値として使う。 */
typedef enum {
  COLOR_DETECT_NONE = 0,  // 彩度や明度が低くて色判定できない
  COLOR_DETECT_BLACK,
  COLOR_DETECT_WHITE,
  COLOR_DETECT_RED,
  COLOR_DETECT_YELLOW,
  COLOR_DETECT_GREEN,
  COLOR_DETECT_BLUE,
  COLOR_DETECT_OTHER,     // どれにも当てはまらない色
} color_detect_t;

class Color {
public:
  /* === 判定しきい値 ====================================== */
  /* 反射値がこの値以下なら黒とみなす (LineTracer の BLACK_BRIGHTNESS と揃える) */
  static constexpr int32_t BlackReflectionMax = 10;
  /* 反射値がこの値以上なら白とみなす */
  static constexpr int32_t WhiteReflectionMin = 80;
  /* HSV 判定の最低彩度。これ未満は無彩色扱い (NONE) */
  static constexpr uint8_t MinSaturation = 30;
  /* HSV 判定の最低明度。これ未満は無彩色扱い (NONE) */
  static constexpr uint8_t MinValue = 20;

  /* === API ============================================== */
  /* 初期化。app.cpp で 1 回だけ呼ぶこと。 */
  void configure(pbio_port_id_t color_sensor_port);

  /* 生の HSV を読み出す。surface=true で表面反射を使う。
   * h / s / v のいずれかが nullptr のときはそれだけ読み飛ばす。 */
  void readHsv(bool surface, uint16_t *h, uint8_t *s, uint8_t *v);

  /* 反射値 (0〜100)。エラー時は負値 (PUP API の仕様) */
  int32_t readReflection();

  /* 黒 / 白 / 各有彩色のいずれか 1 つを返す。
   * 判定不能なら COLOR_DETECT_NONE / 該当なしなら COLOR_DETECT_OTHER。 */
  color_detect_t detect(bool surface);

  /* 各色の判定。surface の意味は readHsv と同じ。
   * （黒/白は反射値ベース、その他は HSV の色相ベース） */
  bool isBlack(bool surface);
  bool isWhite(bool surface);
  bool isRed(bool surface);
  bool isYellow(bool surface);
  bool isGreen(bool surface);
  bool isBlue(bool surface);

  /* 今の検出色が指定した色と一致するか。 */
  bool is(color_detect_t color, bool surface);

private:
  /* 取得した PUP デバイスのハンドル。configure() で初期化。 */
  pup_device_t *device_ = nullptr;
};

/* === 後方互換マクロ ===================================== */
#ifndef COLOR_BLACK_REFLECTION_MAX
#define COLOR_BLACK_REFLECTION_MAX Color::BlackReflectionMax
#endif
#ifndef COLOR_WHITE_REFLECTION_MIN
#define COLOR_WHITE_REFLECTION_MIN Color::WhiteReflectionMin
#endif
#define COLOR_MIN_SATURATION Color::MinSaturation
#define COLOR_MIN_VALUE Color::MinValue

#endif  // TESTAPP_COLOR_H_
