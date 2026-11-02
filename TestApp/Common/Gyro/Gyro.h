/*
 * Gyro モジュール ヘッダ
 *  Hub 内蔵 IMU のヨー角 (heading) を扱う薄いラッパー。
 *  hub_imu_* の C API はこのモジュールに閉じ込めて、
 *  他のモジュール (Nansho など) からは Gyro クラスのメソッドだけを呼ぶ。
 *
 *  使い方:
 *    Gyro gGyro;
 *    gGyro.init();                       // app.cpp の起動時に 1 回 (IMU 初期化 + 準備完了待ち)
 *    gGyro.reset();                      // シーン開始時など、現在向きを 0 にする
 *    const float h = gGyro.heading();    // 現在ヨー角 [deg]
 *    if (gGyro.reached(target, 5.0f)) {  // |target - heading| <= 5 deg ?
 *      ...
 *    }
 */

#ifndef TESTAPP_GYRO_H_
#define TESTAPP_GYRO_H_

#include <stdbool.h>
#include <stdint.h>

class Gyro {
public:
  /* ハブの後傾 [deg]。未設定だと 3D heading が直進中に流れる。LogTest と同じ。 */
  static constexpr float TiltDeg = 51.0f;

  void init();
  bool isReady() const;
  /* 周期タスク用。観測時に heading() を読むので更新処理は空。 */
  void update();

  /* 現在ヨー角をリセットする。
   * offset_deg を渡すと、リセット直後の heading() が 0° ではなく
   * offset_deg になる（CW 正）。この下駄は次の reset まで載り続ける。
   * 例: offset_deg = -2 なら、目標 0° の直進で車体は 2° CW（右）へ向く。 */
  void reset(float offset_deg = 0.0f);

  /* 直進用: true でヨーレート積分 (1D)、false で 3D 融合方位。
   * 切替時は heading() が連続になるよう offset を合わせる。 */
  void setRateHeading(bool enable);
  bool rateHeading() const { return rate_heading_; }

  /* 現在ヨー角 [deg]。選択中ソース + reset/切替の下駄。 */
  float heading() const;

  /* 実測による角度補正 [deg]。reset/ソース切替の連続性も維持する。 */
  void addHeadingCorrection(float delta_deg) { heading_offset_ += delta_deg; }

  /* 選択中ソースの生値 [deg]（下駄なし）。 */
  float rawHeading() const;

  /* target_heading との差分 [deg]。短い側 ([-180, 180]) に折り畳んで返す。
   *   diff > 0 : 目標は現在より CW 方向 (左前進・右後退で近づく)
   *   diff < 0 : 目標は現在より CCW 方向 */
  float diffTo(float target_heading) const;

  /* target_heading に許容誤差 tol_deg 以内まで到達したか。 */
  bool reached(float target_heading, float tol_deg) const;

  /* 角度差を [-180, 180] に折り畳む。
   * 連続値同士の差や ±180 でラップする値の差、どちらにも使える。 */
  static float wrapDeg180(float diff);

private:
  float heading_offset_ = 0.0f;
  bool  rate_heading_   = false;
};

#endif  // TESTAPP_GYRO_H_
