/*
 * Gyro モジュール ヘッダ
 *  Hub 内蔵 IMU のヨー角 (heading) を扱う薄いラッパー。
 *  hub_imu_* の C API はこのモジュールに閉じ込めて、
 *  他のモジュール (Nansho など) からは Gyro クラスのメソッドだけを呼ぶ。
 *
 *  heading() は角速度で予測し、hub の方位角で補正する 1 次元カルマンフィルタ
 *  を通した推定値を返す。
 *
 *  使い方:
 *    Gyro gGyro;
 *    gGyro.init();                       // app.cpp の起動時に 1 回 (IMU 初期化 + 準備完了待ち)
 *    gGyro.reset();                      // シーン開始時など、現在向きを 0 にする
 *    const float h = gGyro.heading();    // 現在ヨー角 [deg]（カルマン推定）
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
  /* プロセスノイズ Q [deg^2/s] と観測ノイズ R [deg^2] の既定値。
   * Q↑ / R↓: hub 方位に追従（旋回は速い、直進で観測ドリフトに引っ張られやすい）
   * Q↓ / R↑: 角速度予測を信頼（直進が安定、旋回で遅れやすい）
   * Nansho3 は Scene 側の KalmanProcessNoise / KalmanMeasNoise で上書きする。 */
  static constexpr float DefaultKalmanProcessNoise = 4.0f;
  static constexpr float DefaultKalmanMeasNoise    = 100.0f;
  /* ハブの後傾 [deg]。未設定だと 3D heading が直進中に流れる。LogTest と同じ。 */
  static constexpr float TiltDeg = 51.0f;

  void init();
  bool isReady() const;

  /* 現在ヨー角を 0° にリセットする。 */
  void reset();

  /* カルマンフィルタのノイズ係数を変更する（実行時チューニング用）。 */
  void setKalmanNoise(float process_noise_deg2_per_s,
                      float measurement_noise_deg2);

  /* 現在ヨー角 [deg]。連続値 (アンラップ済み) のカルマン推定値。 */
  float heading() const;

  /* hub_imu_get_heading() の生値 [deg]（デバッグ・比較用）。 */
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
  /* ASP3 の SYSTIM / get_tim() はマイクロ秒（dly_tsk と同じ）。
   * 0.001 だと dt が 1000 倍になり、旋回中の予測が数十°跳び即終了する。 */
  static constexpr float kSystimToSec = 1.0e-6f;
  static constexpr float kMaxKalmanDtSec = 0.2f;

  float readYawRateDegPerSec() const;
  void updateKalman() const;

  float kf_q_ = DefaultKalmanProcessNoise;
  float kf_r_ = DefaultKalmanMeasNoise;

  mutable float  kf_angle_    = 0.0f;
  mutable float  kf_variance_ = DefaultKalmanMeasNoise;
  mutable bool   kf_ready_    = false;
  mutable uint64_t kf_last_tim_ = 0U;
};

#endif  // TESTAPP_GYRO_H_
