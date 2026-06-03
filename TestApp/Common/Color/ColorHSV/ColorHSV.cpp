/*
 * ColorHSV モジュール 実装
 *  カラーセンサ (PUP ColorSensor) の各種判定。
 *  PUP の C API はこのファイル内だけで呼び出す。
 */

#include "ColorHSV.h"

/* 初期化: ポートからカラーセンサのデバイスを取得する。 */
void ColorHSV::configure(pbio_port_id_t color_sensor_port)
{
  device_ = pup_color_sensor_get_device(color_sensor_port);
}

/* 生の HSV を読み、ユーザ指定のポインタにそれぞれ書き込む。
 * 引数が nullptr の要素はスキップする (h だけ欲しい、などに対応)。 */
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

/* 反射値 0〜100 を返す。エラー時は負値。 */
int32_t ColorHSV::readReflection()
{
  if (device_ == nullptr) {
    return -1;
  }
  return pup_color_sensor_reflection(device_);
}

/* 黒判定: 反射値がしきい値以下。
 * surface は使わないが、API シグネチャを他の is*() と揃えるため受け取る。 */
bool ColorHSV::isBlack(bool surface)
{
  (void)surface;
  const int32_t reflection = readReflection();
  return reflection >= 0 && reflection <= BlackReflectionMax;
}

/* 白判定: 反射値がしきい値以上。 */
bool ColorHSV::isWhite(bool surface)
{
  (void)surface;
  const int32_t reflection = readReflection();
  return reflection >= WhiteReflectionMin;
}

/* 灰色判定 (h / s / v で表現):
 *   h (色相) : 無彩色なので不問 (見ない)
 *   s (彩度) : GraySaturationMax 以下 → 色みが無い
 *   v (明度) : GrayValueMin 〜 GrayValueMax → 黒と白の中間の明るさ
 * 「色みが無く (s 低)、明るさが中くらい (v 中)」のとき灰色とみなす。 */
bool ColorHSV::isGray(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  readHsv(surface, &h, &s, &v);
  (void)h;  // 色相は灰色判定に使わない
  return s <= GraySaturationMax &&
         v >= GrayValueMin && v <= GrayValueMax;
}

/* 赤判定: 色相 h が 335..360, 0..25 の範囲なら赤。
 * 彩度 / 明度が低すぎる場合は色として識別しない (= false)。 */
bool ColorHSV::isRed(bool surface)
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
bool ColorHSV::isYellow(bool surface)
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
bool ColorHSV::isGreen(bool surface)
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
bool ColorHSV::isBlue(bool surface)
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
 *   優先順位: 黒 -> 白 -> (無彩色なら 灰/NONE) -> 赤/黄/緑/青 -> OTHER
 * 黒/白を最優先するのは、ライントレース時に有彩色判定より反射値ベースが安定するため。 */
color_detect_t ColorHSV::detect(bool surface)
{
  if (isBlack(surface)) {
    return COLOR_DETECT_BLACK;
  }
  if (isWhite(surface)) {
    return COLOR_DETECT_WHITE;
  }

  /* 黒でも白でもないときは HSV で評価。
   * 彩度 / 明度が低い無彩色は、明度が中間帯なら灰色、それ以外は NONE。 */
  uint16_t h;
  uint8_t s, v;
  readHsv(surface, &h, &s, &v);
  if (s < MinSaturation || v < MinValue) {
    if (isGray(surface)) {
      return COLOR_DETECT_GRAY;
    }
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
bool ColorHSV::is(color_detect_t color, bool surface)
{
  return detect(surface) == color;
}

/* 読み取り HSV と基準色 ref のコスト (小さいほど近い)。
 *   - 色相 h は円環。差の最大は 180。
 *   - どちらかが無彩色 (彩度が低い) なら色相差は無視する
 *     (白/黒/灰は色相が当てにならないため)。
 *   - 色相を重く、彩度・明度を軽めに重み付けして二乗和を返す。 */
int32_t ColorHSV::hsvCost(uint16_t h, uint8_t s, uint8_t v,
                          const pup_color_hsv_t &ref)
{
  int32_t dh = (int32_t)h - (int32_t)ref.h;
  if (dh < 0) dh = -dh;
  if (dh > 180) dh = 360 - dh;
  if (s < MinSaturation || ref.s < MinSaturation) {
    dh = 0;  // 無彩色同士は色相を見ない
  }

  int32_t ds = (int32_t)s - (int32_t)ref.s;
  if (ds < 0) ds = -ds;
  int32_t dv = (int32_t)v - (int32_t)ref.v;
  if (dv < 0) dv = -dv;

  /* 色相 2 倍重み。彩度/明度は等倍。 */
  const int32_t wh = dh * 2;
  return wh * wh + ds * ds + dv * dv;
}

/* 基準色テーブル color_map_edit の中で、読み取り HSV に最も近い色の
 * インデックスを返す。 */
int ColorHSV::matchColor(bool surface)
{
  uint16_t h;
  uint8_t s, v;
  readHsv(surface, &h, &s, &v);

  int best = 0;
  int32_t best_cost = hsvCost(h, s, v, color_map_edit[0]);
  for (int i = 1; i < color_map_size; i++) {
    const int32_t cost = hsvCost(h, s, v, color_map_edit[i]);
    if (cost < best_cost) {
      best_cost = cost;
      best = i;
    }
  }
  return best;
}

/* 最も近い基準色の名前を返す。 */
const char *ColorHSV::matchColorName(bool surface)
{
  return color_map_name[matchColor(surface)];
}

/* 黒線エッジ検出。
 *   - 「黒以外」を 1 度でも読んだら off_black_ を true にする
 *   - その状態で再び黒を読んだ瞬間に true を返す (= 新しい黒線到達)
 * 連続して黒を踏んでいる間 (開始位置の線にいる間など) は false のまま。
 * resetBlackEdge() を呼ぶと off_black_ が false に戻り、最初からやり直せる。 */
bool ColorHSV::blackEdgeHit(bool surface)
{
  const bool now_black = isBlack(surface);
  if (!now_black) {
    off_black_ = true;
    return false;
  }
  return off_black_;
}
