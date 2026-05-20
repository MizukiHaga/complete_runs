/*
 * Sonic モジュール 実装
 *  超音波センサ (PUP UltrasonicSensor) の薄いラッパー。
 *  ハードウェア依存の API はすべてこのファイル内に閉じ込めている。
 */

#include "Sonic.h"

/* 初期化: ポートから PUP デバイスを取得して保持する。
 * 既に他で使われていても pup_ultrasonic_sensor_get_device は
 * 再取得できる (SPIKE-RT の仕様)。 */
void Sonic::configure(pbio_port_id_t ultrasonic_port)
{
  device_ = pup_ultrasonic_sensor_get_device(ultrasonic_port);
}

/* 距離 [mm] を読む。
 * - configure() を呼ぶ前: DistanceInvalid を返す (安全側)。
 * - センサが何も検出していない / エラー: PUP API が -1 (= DistanceInvalid) を返す。
 * - 正常時: 0〜2000 程度の値 [mm] が返る。 */
int32_t Sonic::readDistanceMm()
{
  if (device_ == nullptr) {
    /* 未初期化のまま呼ばれてもクラッシュしないよう、ここで早期 return する。 */
    return DistanceInvalid;
  }
  return pup_ultrasonic_sensor_distance(device_);
}

/* 何か障害物が見えているか。
 * 距離が読めれば true、未検出/エラーなら false。
 * 距離の大小は判定しないので「とにかく見えているか」だけ知りたい時に使う。 */
bool Sonic::isDetected()
{
  return readDistanceMm() != DistanceInvalid;
}

/* threshold_mm 以下に何かあるか。
 * 例: isCloserThan(300)  -> 300mm 以内に何かあれば true
 * 未検出 (DistanceInvalid) は「近くにはない」とみなして false を返す。 */
bool Sonic::isCloserThan(int32_t threshold_mm)
{
  const int32_t distance_mm = readDistanceMm();
  return distance_mm != DistanceInvalid && distance_mm <= threshold_mm;
}
