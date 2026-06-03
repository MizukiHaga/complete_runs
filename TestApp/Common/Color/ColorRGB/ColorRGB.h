/*
 * ColorRGB モジュール ヘッダ
 *  カラーセンサ (PUP ColorSensor) の生 RGB 値ベースで色を判定するモジュール。
 *  ColorHSV と対になる存在で、色相ではなく R/G/B の大小関係で分類する。
 *
 *  使い方:
 *    ColorRGB gColorRgb;
 *    gColorRgb.configure(PBIO_PORT_ID_E);
 *    uint16_t r, g, b;
 *    gColorRgb.readRgb(&r, &g, &b);
 *    if (gColorRgb.detect() == COLOR_DETECT_RED) { ... }
 */

#ifndef TESTAPP_COLOR_RGB_H_
#define TESTAPP_COLOR_RGB_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

/* color_detect_t は ColorHSV と共有する (列挙の二重定義を避ける)。 */
#include "ColorHSV.h"

/* PUP の C API は C リンクで提供されているので extern "C" で囲む */
extern "C" {
#include <spike/pup/colorsensor.h>
}

/* === ユーザ編集用 基準色テーブル (RGB) =======================
 * ColorHSV の color_map_edit と同じ思想の RGB 版。
 * matchColor() / matchColorName() が「読み取った RGB に最も近い色」を
 * 返すための基準色一覧。色の追加・削除・値の調整はこの 2 つの配列だけ
 * 編集すればよい (要素数と並び順は rgb_map_edit と rgb_map_name で
 * 必ず揃えること)。
 *   r / g / b : 生 RGB 値 0〜1024 (10bit)
 * 末尾の None ({0,0,0}) は「暗くて色が無い」ときの受け皿として使う。
 *
 * 実機で対象に当てて生 RGB を読み (LogTest の ColorRgbSensor など)、
 * その値をここに書き込む。マッチングは明るさを正規化して色味で比較する
 * ので、多少明るさが違っても近い色が選ばれる。 */
static constexpr pup_color_rgb_t rgb_map_edit[] = {
  { 900, 120, 120 },  // Red
  { 850, 750, 120 },  // Yellow
  { 120, 700, 200 },  // Green
  { 120, 250, 800 },  // Blue
  { 200, 600, 850 },  // Skyblue
  { 500, 150, 500 },  // Purple
  { 900, 900, 900 },  // White
  { 120, 120, 120 },  // Gray
  { 600, 630, 650 },  // GrayLine
  {   0,   0,   0 },  // None
};
static constexpr const char *rgb_map_name[] = {
  "Red", "Yellow", "Green", "Blue", "Skyblue", "Purple", "White", "Gray", "GrayLine", "None",
};
/* テーブル要素数 (rgb_map_edit から自動算出)。 */
static constexpr int rgb_map_size =
    sizeof(rgb_map_edit) / sizeof(rgb_map_edit[0]);

class ColorRGB {
public:
  /* === 判定しきい値 ======================================
   * 生 RGB は各チャンネル 0〜1024 (10bit)。
   * 明るさ brightness は (r+g+b) を 0〜100 に正規化した値。 */

  /* 生 RGB チャンネルの最大値 (10bit)。 */
  static constexpr int32_t RawMax = 1024;

  /* 明るさ (0〜100) がこれ以下なら黒。 */
  static constexpr int32_t BlackBrightnessMax = 10;
  /* 明るさ (0〜100) がこれ以上なら白。 */
  static constexpr int32_t WhiteBrightnessMin = 80;

  /* 彩度 chroma = (max-min)/max を 0〜100 にした値。
   * これ未満なら色みが無い (無彩色 = 黒/白/灰) とみなす。 */
  static constexpr int32_t MinChroma = 25;

  /* 灰色とみなす明るさの範囲 [GrayBrightnessMin, GrayBrightnessMax]。
   * 無彩色 (chroma < MinChroma) かつ明るさが黒と白の中間のとき灰色。 */
  static constexpr int32_t GrayBrightnessMin = 15;
  static constexpr int32_t GrayBrightnessMax = 70;

  /* 黄色判定: 赤が最大かつ緑も赤の何 % 以上あるか / 青が何 % 以下か。 */
  static constexpr int32_t YellowGreenRatio = 60;  // g >= r * 60%
  static constexpr int32_t YellowBlueRatio  = 60;  // b <= r * 60%

  /* === API ============================================== */
  /* 初期化。app.cpp で 1 回だけ呼ぶこと。 */
  void configure(pbio_port_id_t color_sensor_port);

  /* configure 済みで PUP デバイスを取得できているか。 */
  bool isConfigured() const { return device_ != nullptr; }

  /* 生 RGB (各 0〜1024) を読み出す。nullptr の要素はスキップ。 */
  void readRgb(uint16_t *r, uint16_t *g, uint16_t *b);

  /* 明るさ (r+g+b を 0〜100 に正規化)。エラー時は負値。 */
  int32_t brightness();

  /* 黒 / 白 / 灰 / 各有彩色のいずれか 1 つを返す。
   * 判定不能なら COLOR_DETECT_NONE。 */
  color_detect_t detect();

  /* 各色の判定 (内部で detect() を 1 回呼ぶ)。 */
  bool isBlack();
  bool isWhite();
  bool isGray();
  bool isRed();
  bool isYellow();
  bool isGreen();
  bool isBlue();
  bool isGrayLine();
  

  /* === 基準色テーブル (rgb_map_edit) によるマッチング ========
   * 読み取った RGB に最も近い基準色を選ぶ。閾値方式 (isRed 等) と違い、
   * 登録した任意の色 (Skyblue, Purple など) も扱える。 */
  /* 最も近い基準色のインデックス (0〜rgb_map_size-1) を返す。 */
  int matchColor();
  /* 最も近い基準色の名前 (rgb_map_name) を返す。 */
  const char *matchColorName();

private:
  /* 読み取り RGB と基準色 ref の「近さ」を表すコスト (小さいほど近い)。
   * 明るさを正規化した色味の差に、明るさ差を加えて求める。 */
  static int32_t rgbCost(uint16_t r, uint16_t g, uint16_t b,
                         const pup_color_rgb_t &ref);

  pup_device_t *device_ = nullptr;
};

#endif  // TESTAPP_COLOR_RGB_H_
