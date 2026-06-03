/*
 * ColorHSV モジュール ヘッダ
 *  カラーセンサ (PUP ColorSensor) の判定をまとめたモジュール。
 *  反射値ベースで黒/白を判定し、HSV ベースで色を分類する。
 *
 *  使い方:
 *    ColorHSV gColor;
 *    gColor.configure(PBIO_PORT_ID_E);
 *    if (gColor.isBlack(true)) { ... }
 */

#ifndef TESTAPP_COLOR_HSV_H_
#define TESTAPP_COLOR_HSV_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

/* PUP の C API は C リンクで提供されているので extern "C" で囲む */
extern "C" {
#include <spike/pup/colorsensor.h>
}

/* 検出した色を 1 つに分類するための列挙。
 * detect() / is() が返す値として使う。
 * ColorHSV / ColorRGB の両方で共有する。 */
typedef enum {
  COLOR_DETECT_NONE = 0,  // 彩度や明度が低くて色判定できない
  COLOR_DETECT_BLACK,
  COLOR_DETECT_WHITE,
  COLOR_DETECT_GRAY,      // 無彩色かつ明るさが黒と白の中間 (灰色)
  COLOR_DETECT_RED,
  COLOR_DETECT_YELLOW,
  COLOR_DETECT_GREEN,
  COLOR_DETECT_BLUE,
  COLOR_DETECT_OTHER,     // どれにも当てはまらない色
} color_detect_t;

/* === ユーザ編集用 基準色テーブル ==============================
 * matchColor() / matchColorName() が「読み取った HSV に最も近い色」を
 * 返すための基準色一覧。色の追加・削除・値の調整はこの 2 つの配列だけ
 * 編集すればよい (要素数と並び順は color_map_edit と color_map_name で
 * 必ず揃えること)。
 *   h : 色相 0〜359   s : 彩度 0〜100   v : 明度 0〜100
 * 末尾の None ({0,0,0}) は「暗くて色が無い」ときの受け皿として使う。
 *
 * 実機で対象に当てて HSV を読み (LogTest など)、その値をここに書き込む。 */
static constexpr pup_color_hsv_t color_map_edit[] = {
  { 352, 91, 95 },  // Red
  {  56, 53, 100 }, // Yellow
  { 155, 62, 46 },  // Green
  { 212, 93, 72 },  // Blue
  { 195, 80, 97 },  // Skyblue
  { 345, 76, 50 },  // Purple
  {   0,  5, 80 },  // White
  {   0,  0,  0 },  // None
};
static constexpr const char *color_map_name[] = {
  "Red", "Yellow", "Green", "Blue", "Skyblue", "Purple", "White", "None",
};
/* テーブル要素数 (color_map_edit から自動算出)。 */
static constexpr int color_map_size =
    sizeof(color_map_edit) / sizeof(color_map_edit[0]);

class ColorHSV {
public:
  /* === 判定しきい値 ====================================== */
  /* 反射値がこの値以下なら黒とみなす (LineTracer の BLACK_BRIGHTNESS と揃える) */
  static constexpr int32_t BlackReflectionMax = 10;
  /* 反射値がこの値以上なら白とみなす */
  static constexpr int32_t WhiteReflectionMin = 80;
  /* 灰色の HSV 判定しきい値。
   *   h (色相) : 無彩色なので不問 (使わない)
   *   s (彩度) : GraySaturationMax 以下 = 色みが無い (無彩色)
   *   v (明度) : GrayValueMin 〜 GrayValueMax = 黒(暗)と白(明)の中間の明るさ
   * 「色みが無く、明るさが中くらい」のものを灰色とみなす。 */
  static constexpr uint8_t GraySaturationMax = 25;
  static constexpr uint8_t GrayValueMin = 28;
  static constexpr uint8_t GrayValueMax = 15;
  /* HSV 判定の最低彩度。これ未満は無彩色扱い (NONE) */
  static constexpr uint8_t MinSaturation = 30;
  /* HSV 判定の最低明度。これ未満は無彩色扱い (NONE) */
  static constexpr uint8_t MinValue = 20;

  /* === API ============================================== */
  /* 初期化。app.cpp で 1 回だけ呼ぶこと。 */
  void configure(pbio_port_id_t color_sensor_port);

  /* configure 済みで PUP デバイスを取得できているか。
   * false の場合はポート違い / 未接続 / 種類違いの可能性が高い。 */
  bool isConfigured() const { return device_ != nullptr; }

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
  /* 灰色判定: 無彩色 (彩度が低い) かつ明度が黒と白の中間帯。 */
  bool isGray(bool surface);
  bool isRed(bool surface);
  bool isYellow(bool surface);
  bool isGreen(bool surface);
  bool isBlue(bool surface);

  /* 今の検出色が指定した色と一致するか。 */
  bool is(color_detect_t color, bool surface);

  /* === 基準色テーブル (color_map_edit) によるマッチング ========
   * 読み取った HSV に最も近い基準色を選ぶ。閾値レンジ方式 (isRed 等) と
   * 違い、登録した任意の色 (Skyblue, Purple など) も扱える。 */
  /* 最も近い基準色のインデックス (0〜color_map_size-1) を返す。 */
  int matchColor(bool surface);
  /* 最も近い基準色の名前 (color_map_name) を返す。 */
  const char *matchColorName(bool surface);

  /* === 黒線エッジ検出 ====================================
   * 「いったん黒以外を踏んでから、再び黒を踏んだ瞬間」を検出するヘルパ。
   * 連続して黒の上にいるだけでは true を返さない (= 開始位置の線で
   * 即座にトリガしない)。
   *
   * 使い方:
   *   gColor.resetBlackEdge();              // シーン開始時にクリア
   *   while (1) {
   *     if (gColor.blackEdgeHit(true)) {    // ループ内で毎周期呼ぶ
   *       // 「新しい黒線を踏んだ」
   *       break;
   *     }
   *   }
   *
   *   surface : isBlack に渡すフラグ (false で内側反射、true で表面)
   *   戻り値  : 一度非黒を見たことがある状態で、いま黒を見ているなら true */
  void resetBlackEdge() { off_black_ = false; }
  bool blackEdgeHit(bool surface);

private:
  /* 読み取り HSV と基準色 ref との「近さ」を表すコスト (小さいほど近い)。
   * 色相は円環 (0/359 が隣接) として扱い、無彩色同士では色相差を無視する。 */
  static int32_t hsvCost(uint16_t h, uint8_t s, uint8_t v,
                         const pup_color_hsv_t &ref);

  /* 取得した PUP デバイスのハンドル。configure() で初期化。 */
  pup_device_t *device_ = nullptr;

  /* 黒線エッジ検出用の内部状態。
   * blackEdgeHit が呼ばれて「黒以外」を読んだ時点で true になり、
   * その後の「黒」読み取りで初めて true が返るトリガとして使う。
   * resetBlackEdge() で false に戻る。 */
  bool off_black_ = false;
};

/* === 後方互換マクロ ===================================== */
#ifndef COLOR_BLACK_REFLECTION_MAX
#define COLOR_BLACK_REFLECTION_MAX ColorHSV::BlackReflectionMax
#endif
#ifndef COLOR_WHITE_REFLECTION_MIN
#define COLOR_WHITE_REFLECTION_MIN ColorHSV::WhiteReflectionMin
#endif
#define COLOR_MIN_SATURATION ColorHSV::MinSaturation
#define COLOR_MIN_VALUE ColorHSV::MinValue

#endif  // TESTAPP_COLOR_HSV_H_
