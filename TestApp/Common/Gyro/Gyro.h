/*
 * Gyro モジュール ヘッダ
 *  Hub内蔵IMUと車輪による連続方位推定。無効時は従来のIMUラッパー。
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
#include "HeadingFusion.h"

class Gyro {
public:
  /* ハブの後傾 [deg]。未設定だと 3D heading が直進中に流れる。LogTest と同じ。 */
  static constexpr float TiltDeg = 51.0f;

  void init();
  bool isReady() const;

  /* 専用10msタスクから更新。制御側はheading()の共通推定角を使う。 */
  void updateFusion();
  void beginTurnFusion();
  void endTurnFusion();
  bool fusionEnabled() const { return fusion_enabled_; }
  HeadingFusion::State fusionState() const;
  void invalidateFusionEncoders();

  /* 現在ヨー角をリセットする。
   * offset_deg を渡すと、リセット直後の heading() が 0° ではなく
   * offset_deg になる（CW 正）。この下駄は次の reset まで載り続ける。
   * 例: offset_deg = -2 なら、目標 0° の直進で車体は 2° CW（右）へ向く。 */
  void reset(float offset_deg = 0.0f);

  /* 連続推定有効時は互換用の指定で、共通の1D観測源は切り替えない。
   * 無効時: true=1D、false=3D。切替前後の方位を連続にする。 */
  void setRateHeading(bool enable);
  bool rateHeading() const { return rate_heading_; }

  /* 現在ヨー角[deg]。有効時は共通KF推定角＋原点offset。 */
  float heading() const;

  /* 実測による角度補正 [deg]。reset/ソース切替の連続性も維持する。 */
  void addHeadingCorrection(float delta_deg) { heading_offset_ += delta_deg; }

  /* 観測源の角度[deg]（ドライバ校正済み、追加offsetなし）。有効時は常に1D。 */
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
  HeadingFusion fusion_;
  bool fusion_enabled_ = false;
  volatile uint32_t fusion_generation_ = 0U;
  bool fusion_encoder_valid_ = false;
  bool fusion_raw_valid_ = false;
  int32_t fusion_left_ = 0;
  int32_t fusion_right_ = 0;
  float fusion_raw_ = 0.0f;
  uint64_t fusion_time_ = 0U;
};

#endif  // TESTAPP_GYRO_H_
