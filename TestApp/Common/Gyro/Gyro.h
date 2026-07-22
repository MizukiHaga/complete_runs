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
  void init();
  bool isReady() const;

  /* 現在ヨー角を 0° にリセットする。 */
  void reset();

  /* 現在ヨー角 [deg]。pybricks IMU の連続値 (アンラップ済み) が返る。 */
  float heading() const;

  /* target_heading との差分 [deg]。短い側 ([-180, 180]) に折り畳んで返す。
   *   diff > 0 : 目標は現在より CW 方向 (左前進・右後退で近づく)
   *   diff < 0 : 目標は現在より CCW 方向 */
  float diffTo(float target_heading) const;

  /* target_heading に許容誤差 tol_deg 以内まで到達したか。 */
  bool reached(float target_heading, float tol_deg) const;

  /* 角度差を [-180, 180] に折り畳む。
   * 連続値同士の差や ±180 でラップする値の差、どちらにも使える。 */
  static float wrapDeg180(float diff);
};

#endif  // TESTAPP_GYRO_H_
