/*
 * Sonic モジュール ヘッダ
 *  PUP の超音波センサ (UltrasonicSensor) を扱う薄いラッパー。
 *  pup_ultrasonic_sensor_* の呼び出しはこのモジュールに閉じ込めて、
 *  他のモジュール（Nansho など）からは Sonic クラスのメソッドだけを呼ぶ。
 *
 *  使い方:
 *    Sonic gSonic;
 *    gSonic.configure(PBIO_PORT_ID_F);     // app.cpp の起動時に 1 度だけ
 *    if (gSonic.isCloserThan(250)) { ... } // 距離で障害物検知
 */

#ifndef TESTAPP_SONIC_H_
#define TESTAPP_SONIC_H_

extern "C" {
#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

/* PUP の C API は C リンクで提供されているので extern "C" で囲む */
#include <spike/pup/ultrasonicsensor.h>
}

class Sonic {
public:
  /* 距離が読めなかったとき (未検出 / 未初期化 / エラー) に返る値。
   * pup_ultrasonic_sensor_distance() の仕様に合わせて -1 を使う。 */
  static constexpr int32_t DistanceInvalid = -1;

  /* 初期化。app.cpp 側で 1 回だけ呼ぶこと。
   *   ultrasonic_port: 超音波センサを接続したポート (例: PBIO_PORT_ID_F) */
  void configure(pbio_port_id_t ultrasonic_port);

  /* configure 済みで PUP デバイスを取得できているか。 */
  bool isConfigured() const { return device_ != nullptr; }

  /* 距離 [mm] を返す。未検出 / 未初期化 / エラー時は DistanceInvalid。
   * ※ DISTL モード。presence と併用するとモード切替が起きる。 */
  int32_t readDistanceMm();

  /* pup_ultrasonic_sensor_presence (LISTN):
   * 他の超音波センサが出す信号を受信したら true。
   * 距離による障害物判定ではない。 */
  bool isPresent();

  /* 走査などの検知判定用。距離が読めたら true（閾値なしの簡易判定）。 */
  bool isDetected();

  /* threshold_mm 以下に何かがあるか (距離 API)。未検出のときは false。 */
  bool isCloserThan(int32_t threshold_mm);

private:
  pup_device_t *device_ = nullptr;
};

/* 後方互換（既存コード用）。旧 C 風マクロ名を使っている箇所のために残す。 */
#define SONIC_DISTANCE_INVALID Sonic::DistanceInvalid

#endif  // TESTAPP_SONIC_H_
