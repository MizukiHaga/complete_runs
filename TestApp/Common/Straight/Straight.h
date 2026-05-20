#ifndef TESTAPP_STRAIGHT_H_
#define TESTAPP_STRAIGHT_H_

#include <pbio/port.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include <spike/pup/motor.h>
}

class Straight {
public:
  using PollFn = bool (*)(void);

  static constexpr int DefaultLeftPower = 70;
  static constexpr int DefaultRightPower = 65;
  static constexpr uint32_t DefaultTimeMs = 2000U;
  static constexpr uint32_t DecelMs = 800U;
  static constexpr uint32_t DecelStepMs = 40U;
  static constexpr uint32_t MaxDurationMs = 4000000U;
  static constexpr int SerpentineSteer = 15;
  static constexpr uint32_t SerpentinePhaseMs = 300U;
  static constexpr uint32_t SerpentineTimeMs = 2000U;
  static constexpr uint32_t SerpentineForever = 0U;

  void configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port);
  void run(int left_power, int right_power);
  void stop();
  void runFor(int left_power, int right_power, uint32_t duration_ms);
  void runSerpentine(int left_base, int right_base, int steer, uint32_t phase_ms,
                     uint32_t total_ms, PollFn poll);

private:
  static int clampMotorPower(int power);
  static constexpr uint32_t SerpentineStepMs = 10U;

  pup_motor_t *left_ = nullptr;
  pup_motor_t *right_ = nullptr;
};

/* 後方互換マクロ */
#define STRAIGHT_LEFT_POWER        Straight::DefaultLeftPower
#define STRAIGHT_RIGHT_POWER       Straight::DefaultRightPower
#define STRAIGHT_TIME              Straight::DefaultTimeMs
#define STRAIGHT_DECEL_MS          Straight::DecelMs
#define STRAIGHT_DECEL_STEP_MS     Straight::DecelStepMs
#define STRAIGHT_MAX_DURATION_MS   Straight::MaxDurationMs
#define STRAIGHT_SERPENTINE_STEER      Straight::SerpentineSteer
#define STRAIGHT_SERPENTINE_PHASE_MS   Straight::SerpentinePhaseMs
#define STRAIGHT_SERPENTINE_TIME       Straight::SerpentineTimeMs
#define STRAIGHT_SERPENTINE_FOREVER    Straight::SerpentineForever
typedef Straight::PollFn straight_serpentine_poll_fn;

#endif  // TESTAPP_STRAIGHT_H_
