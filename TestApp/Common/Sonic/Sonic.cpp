/*
 * Sonic モジュール 実装
 *  超音波センサ (PUP UltrasonicSensor) の薄いラッパー。
 *  ハードウェア依存の API はすべてこのファイル内に閉じ込めている。
 */

#include "Sonic.h"

void Sonic::configure(pbio_port_id_t ultrasonic_port)
{
  device_ = pup_ultrasonic_sensor_get_device(ultrasonic_port);
}

int32_t Sonic::readDistanceMm()
{
  if (device_ == nullptr) {
    return DistanceInvalid;
  }
  return pup_ultrasonic_sensor_distance(device_);
}

bool Sonic::isPresent()
{
  if (device_ == nullptr) {
    return false;
  }
  return pup_ultrasonic_sensor_presence(device_);
}

bool Sonic::isDetected()
{
  return readDistanceMm() != DistanceInvalid;
}

bool Sonic::isCloserThan(int32_t threshold_mm)
{
  const int32_t distance_mm = readDistanceMm();
  return distance_mm != DistanceInvalid && distance_mm <= threshold_mm;
}
