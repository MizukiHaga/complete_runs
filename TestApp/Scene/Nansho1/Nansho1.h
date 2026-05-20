#ifndef TESTAPP_NANSHO1_H_
#define TESTAPP_NANSHO1_H_

#include <kernel.h>
#include <pbio/port.h>

#include "Color.h"
#include "Sonic.h"
#include "Straight.h"

#define NANSHO1_ULTRASONIC_PORT PBIO_PORT_ID_F

class Nansho1Scene {
public:
  static constexpr int LeftPower = 70;
  static constexpr int RightPower = 65;
  static constexpr uint32_t SerpentineSliceMs = 300U;
  static constexpr int32_t DetectMinMm = 1;
  static constexpr int32_t DetectMaxMm = 2000;
  static constexpr int32_t ApproachMm = 300;
  static constexpr uint32_t PollMs = 10U;

  Nansho1Scene(Straight &motors, Color &color, Sonic &sonic)
      : motors_(motors), color_(color), sonic_(sonic) {}

  void run();

private:
  void serpentineUntilUltrasonic();

  Straight &motors_;
  Color &color_;
  Sonic &sonic_;
};

#define NANSHO1_LEFT_POWER Nansho1Scene::LeftPower
#define NANSHO1_RIGHT_POWER Nansho1Scene::RightPower
#define NANSHO1_SERPENTINE_SLICE_MS Nansho1Scene::SerpentineSliceMs
#define NANSHO1_DETECT_MIN_MM Nansho1Scene::DetectMinMm
#define NANSHO1_DETECT_MAX_MM Nansho1Scene::DetectMaxMm
#define NANSHO1_APPROACH_MM Nansho1Scene::ApproachMm
#define NANSHO1_POLL_MS Nansho1Scene::PollMs

#ifdef __cplusplus
extern "C" {
#endif

void Nansho1Task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_NANSHO1_H_
