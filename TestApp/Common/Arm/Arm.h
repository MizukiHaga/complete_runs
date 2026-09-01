/*
 * Arm モジュール ヘッダ
 *  中モータ 1 本でアームを上げ下げする。
 *
 *  使い方:
 *    Arm gArm;
 *    gArm.configure(PBIO_PORT_ID_C);
 *    gArm.raise(-50, 55);   // power の符号で上げ/下げ方向を合わせる
 */

#ifndef TESTAPP_ARM_H_
#define TESTAPP_ARM_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include <spike/pup/motor.h>
}

class Arm {
public:
  static constexpr int      DefaultRaisePower = 70;
  static constexpr int      DefaultLowerPower = -70;
  static constexpr int32_t  DefaultRaiseDeg   = 90;
  static constexpr uint32_t DefaultRaiseMaxMs = 4000U;
  static constexpr uint32_t RaiseStepMs       = 10U;

  void configure(pbio_port_id_t motor_port,
                 pup_direction_t positive_direction =
                     PUP_DIRECTION_COUNTERCLOCKWISE);
  bool isConfigured() const { return motor_ != nullptr; }

  void raise(int power = DefaultRaisePower,
             int32_t target_deg = DefaultRaiseDeg,
             uint32_t max_ms = DefaultRaiseMaxMs);
  /* overshoot_deg: 0 位置を過ぎてさらに下げる量 [deg]（確実に接地させる用） */
  void lower(int power = DefaultLowerPower,
             uint32_t max_ms = DefaultRaiseMaxMs,
             int32_t overshoot_deg = 0);
  void stop();

private:
  static int clampPower(int power);
  static bool reachedTarget(int32_t count, int power, int32_t target_deg);

  pup_motor_t *motor_ = nullptr;
};

#endif  // TESTAPP_ARM_H_
