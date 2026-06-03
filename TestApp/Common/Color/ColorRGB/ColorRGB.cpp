/*
 * ColorRGB モジュール 実装
 *  生 RGB 値ベースのカラー判定。PUP の C API はこのファイル内だけで呼ぶ。
 */

#include "ColorRGB.h"

#include <string.h>

/* 初期化: ポートからカラーセンサのデバイスを取得する。 */
void ColorRGB::configure(pbio_port_id_t color_sensor_port)
{
  device_ = pup_color_sensor_get_device(color_sensor_port);
}

/* 生 RGB (各 0〜1024) を読み、指定ポインタに書き込む。
 * nullptr の要素はスキップする。 */
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

/* 明るさ: r+g+b を 0〜100 に正規化。
 * 3 チャンネル各 1024 が最大なので合計の最大は 3072。 */
int32_t ColorRGB::brightness()
{
  if (device_ == nullptr) {
    return -1;
  }
  uint16_t r, g, b;
  readRgb(&r, &g, &b);
  return ((int32_t)r + g + b) * 100 / (RawMax * 3);
}

/* 今の色を 1 つに分類する。
 *   1. 明るさで 黒 / 白 を最優先判定
 *   2. 彩度 (chroma) が低い無彩色は、明るさ中間なら灰、それ以外は NONE
 *   3. 有彩色は R/G/B の大小で 赤/黄/緑/青 を判定 */
color_detect_t ColorRGB::detect()
{
  if (device_ == nullptr) {
    return COLOR_DETECT_NONE;
  }

  uint16_t r, g, b;
  readRgb(&r, &g, &b);

  const int32_t bright = ((int32_t)r + g + b) * 100 / (RawMax * 3);
  if (bright <= BlackBrightnessMax) {
    return COLOR_DETECT_BLACK;
  }
  if (bright >= WhiteBrightnessMin) {
    return COLOR_DETECT_WHITE;
  }

  /* 彩度 chroma = (max-min)/max [0〜100]。色みの強さ。 */
  int32_t maxc = r;
  if (g > maxc) maxc = g;
  if (b > maxc) maxc = b;
  int32_t minc = r;
  if (g < minc) minc = g;
  if (b < minc) minc = b;
  const int32_t chroma = (maxc > 0) ? (maxc - minc) * 100 / maxc : 0;

  /* 無彩色 (色みが弱い) → 明るさ中間なら灰、それ以外は NONE。 */
  if (chroma < MinChroma) {
    if (bright >= GrayBrightnessMin && bright <= GrayBrightnessMax) {
      return COLOR_DETECT_GRAY;
    }
    return COLOR_DETECT_NONE;
  }

  /* 有彩色: 優勢チャンネルで分類。 */
  if (r >= g && r >= b) {
    /* 赤が最大。緑も高く青が低ければ黄色。 */
    if ((int32_t)g * 100 >= (int32_t)r * YellowGreenRatio &&
        (int32_t)b * 100 <= (int32_t)r * YellowBlueRatio) {
      return COLOR_DETECT_YELLOW;
    }
    return COLOR_DETECT_RED;
  }
  if (g >= r && g >= b) {
    return COLOR_DETECT_GREEN;
  }
  return COLOR_DETECT_BLUE;
}

bool ColorRGB::isBlack()  { return detect() == COLOR_DETECT_BLACK; }
bool ColorRGB::isWhite()  { return detect() == COLOR_DETECT_WHITE; }
bool ColorRGB::isGray()   { return detect() == COLOR_DETECT_GRAY; }
bool ColorRGB::isRed()    { return detect() == COLOR_DETECT_RED; }
bool ColorRGB::isYellow() { return detect() == COLOR_DETECT_YELLOW; }
bool ColorRGB::isGreen()  { return detect() == COLOR_DETECT_GREEN; }
bool ColorRGB::isBlue()   { return detect() == COLOR_DETECT_BLUE; }

/* グレーのライン判定。
 * 黒線とは違う「明るめの灰色 (GrayLine)」を区別したいので、
 * 閾値方式ではなく基準色テーブル (rgb_map_edit) の最近傍が
 * "GrayLine" になったときだけ true を返す。
 * しきい値や色味を変えたいときは rgb_map_edit の GrayLine 行を編集する。 */
bool ColorRGB::isGrayLine()
{
  return strcmp(matchColorName(), "GrayLine") == 0;
}

/* 読み取り RGB と基準色 ref のコスト (小さいほど近い)。
 *   - まず r+g+b が一定 (Norm) になるよう比率化し、明るさに依存しない
 *     色味の差 (二乗和) を求める。
 *   - 比率だけだと黒/白/灰が潰れて区別できないので、別途明るさ差も足す。 */
int32_t ColorRGB::rgbCost(uint16_t r, uint16_t g, uint16_t b,
                          const pup_color_rgb_t &ref)
{
  constexpr int32_t Norm = 300;  // 正規化後の r+g+b 合計

  const int32_t sum  = (int32_t)r + g + b;
  const int32_t rsum = (int32_t)ref.r + ref.g + ref.b;

  const int32_t nr = (sum  > 0) ? (int32_t)r     * Norm / sum  : 0;
  const int32_t ng = (sum  > 0) ? (int32_t)g     * Norm / sum  : 0;
  const int32_t nb = (sum  > 0) ? (int32_t)b     * Norm / sum  : 0;
  const int32_t kr = (rsum > 0) ? (int32_t)ref.r * Norm / rsum : 0;
  const int32_t kg = (rsum > 0) ? (int32_t)ref.g * Norm / rsum : 0;
  const int32_t kb = (rsum > 0) ? (int32_t)ref.b * Norm / rsum : 0;

  const int32_t dr = nr - kr;
  const int32_t dg = ng - kg;
  const int32_t db = nb - kb;
  const int32_t chroma_cost = dr * dr + dg * dg + db * db;

  /* 明るさ (0〜100) の差。黒/白/灰の区別用に軽く加える。 */
  const int32_t bright  = sum  * 100 / (RawMax * 3);
  const int32_t kbright = rsum * 100 / (RawMax * 3);
  const int32_t dbr = bright - kbright;

  return chroma_cost + dbr * dbr;
}

/* 基準色テーブル rgb_map_edit の中で、読み取り RGB に最も近い色の
 * インデックスを返す。 */
int ColorRGB::matchColor()
{
  uint16_t r, g, b;
  readRgb(&r, &g, &b);

  int best = 0;
  int32_t best_cost = rgbCost(r, g, b, rgb_map_edit[0]);
  for (int i = 1; i < rgb_map_size; i++) {
    const int32_t cost = rgbCost(r, g, b, rgb_map_edit[i]);
    if (cost < best_cost) {
      best_cost = cost;
      best = i;
    }
  }
  return best;
}

/* 最も近い基準色の名前を返す。 */
const char *ColorRGB::matchColorName()
{
  return rgb_map_name[matchColor()];
}
