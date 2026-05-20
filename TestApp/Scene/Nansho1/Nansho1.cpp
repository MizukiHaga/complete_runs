#include "Nansho1.h"

#include <stdio.h>

extern Sonic gSonic;

static bool pollUltrasonicCloser()
{
  return gSonic.isCloserThan(500);
}

void Nansho1Scene::serpentineUntilUltrasonic()
{
  motors_.runSerpentine(20, 20, 20, 1000, Straight::SerpentineForever,
                        pollUltrasonicCloser);
}

void Nansho1Scene::run()
{
  /* 1. 黒まで直進 */
  while (1) {
    if (color_.isBlack(true)) {
      motors_.stop();
      break;
    }
    motors_.run(LeftPower, RightPower);
  }

  /* 2. 超音波で何かが近くにくるまで蛇行 */
  serpentineUntilUltrasonic();

  motors_.runFor(55, 50, 5000);
}

extern Nansho1Scene gNansho1;

extern "C" void Nansho1Task(intptr_t exinf)
{
  (void)exinf;
  gNansho1.run();
  ext_tsk();
}
