#include "Nansho1.h"
#include "Color.h"
#include "Sonic.h"
#include "Straight.h"

#include <spike/hub/speaker.h>
#include <stdio.h>

static bool poll_ultrasonic_while_serpentine(void)
{
  return Sonic_IsCloserThan(800);
}

/* 超音波が 800mm 以内になるまで蛇行（10ms ごとに超音波を読む） */
static void serpentine_until_ultrasonic(void)
{
  printf("Nansho1: serpentine start\n");

  Straight_RunSerpentine(20, 20, 20, 1000, STRAIGHT_SERPENTINE_FOREVER,
                         poll_ultrasonic_while_serpentine);
}

void Nansho1_Configure(void)
{
  /* 各モジュールの初期化は app.cpp 側で行う */
}

void Nansho1Task(intptr_t exinf)
{
  (void)exinf;

  /* 1. 黒まで直進 */
  while (1) {
    if (Color_IsBlack(true)) {
      Straight_Stop();
      printf("Nansho1: black detected\n");
      break;
    }
    Straight_Run(NANSHO1_LEFT_POWER, NANSHO1_RIGHT_POWER);
  }

  /* 2. 超音波で何かが近くにくるまで蛇行 */
  serpentine_until_ultrasonic();

  Straight_RunFor(40, 40, 5000);

  ext_tsk();
}
