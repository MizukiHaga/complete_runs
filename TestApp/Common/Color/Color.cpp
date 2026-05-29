/*
 * Color モジュール 実装
 *  カラーセンサ (PUP ColorSensor) の各種判定。
 *  PUP の C API はこのファイル内だけで呼び出す。
 */

#include "Color.h"

/* 初期化: ポートからカラーセンサのデバイスを取得する。 */
void Color::configure(pbio_port_id_t color_sensor_port)
{
  device_ = pup_color_sensor_get_device(color_sensor_port);
}

/* 生の HSV を読み、ユーザ指定のポインタにそれぞれ書き込む。
 * 引数が nullptr の要素はスキップする (h だけ欲しい、などに対応)。 */
void Color::readHsv(bool surface, uint16_t *h, uint8_t *s, uint8_t *v)
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

/* 反射値 0〜100 を返す。エラー時は負値。 */
int32_t Color::readReflection()
{
  if (device_ == nullptr) {
    return -1;
  }
  return pup_color_sensor_reflection(device_);
}

/* 黒判定: 反射値がしきい値以下。
 * surface は使わないが、API シグネチャを他の is*() と揃えるため受け取る。 */
bool Color::isBlack(bool surface)
{
  (void)surface;
  const int32_t reflection = readReflection();
  return reflection >= 0 && reflection <= BlackReflectionMax;
}

/* 白判定: 反射値がしきい値以上。 */
bool Color::isWhite(bool surface)
{
  (void)surface;
  const int32_t reflection = readReflection();
  return reflection >= WhiteReflectionMin;
}

/* 赤判定: 色相 h が 335..360, 0..25 の範囲なら赤。
 * 彩度 / 明度が低すぎる場合は色として識別しない (= false)。 */
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

/* 黄判定: 色相 h が 45..75 */
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

/* 緑判定: 色相 h が 95..145 */
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

/* 青判定: 色相 h が 200..260 */
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

/* 今の色を 1 つに分類する。
 *   優先順位: 黒 -> 白 -> (彩度/明度低なら NONE) -> 赤/黄/緑/青 -> OTHER
 * 黒/白を最優先するのは、ライントレース時に有彩色判定より反射値ベースが安定するため。 */
color_detect_t Color::detect(bool surface)
{
  if (isBlack(surface)) {
    return COLOR_DETECT_BLACK;
  }
  if (isWhite(surface)) {
    return COLOR_DETECT_WHITE;
  }

  /* 黒でも白でもないときは HSV で評価。
   * 彩度 / 明度が低い灰色っぽいものは NONE 扱い。 */
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
  /* 範囲外の有彩色 (紫, シアンなど) */
  return COLOR_DETECT_OTHER;
}

/* 今の検出色が指定した color と一致するか。 */
bool Color::is(color_detect_t color, bool surface)
{
  return detect(surface) == color;
}
