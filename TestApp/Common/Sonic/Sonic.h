#ifndef TESTAPP_SONIC_H_
#define TESTAPP_SONIC_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include <spike/pup/ultrasonicsensor.h>
}

class Sonic {
public:
  static constexpr int32_t DistanceInvalid = -1;

  void configure(pbio_port_id_t ultrasonic_port);
  int32_t readDistanceMm();
  bool isDetected();
  bool isCloserThan(int32_t threshold_mm);

private:
  pup_device_t *device_ = nullptr;
};

/* 後方互換（既存コード用） */
#define SONIC_DISTANCE_INVALID Sonic::DistanceInvalid

#endif  // TESTAPP_SONIC_H_
