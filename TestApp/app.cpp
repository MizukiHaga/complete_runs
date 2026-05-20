#include "kernel_cfg.h"
#include "app.h"
#include <stdio.h>
#include <serial/newlib.h>
#include <serial/serial.h>
#include <cstdio>

#include "LineTracer.h"
#include "Nansho1.h"
#include "Color.h"
#include "Sonic.h"
#include "Straight.h"

extern "C" {
#include <spike/pup/forcesensor.h>
}

/* === Common インスタンス === */
Straight gMotors;
Color gColor;
Sonic gSonic;

/* === Scene: Member A === */
Nansho1Scene gNansho1(gMotors, gColor, gSonic);

/* センサーポートの定義 */
static const pbio_port_id_t color_sensor_port = PBIO_PORT_ID_E,
                            left_motor_port = PBIO_PORT_ID_B,
                            right_motor_port = PBIO_PORT_ID_A,
                            force_sensor_port = PBIO_PORT_ID_D,
                            ultrasonic_port = NANSHO1_ULTRASONIC_PORT;

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused)
{
  (void)unused;

  printf("+---------------------------------+\n");
  printf("|   Press force sensor to start   |\n");
  printf("+---------------------------------+\n");

  pup_device_t *force_sensor = pup_force_sensor_get_device(force_sensor_port);
  while (!pup_force_sensor_touched(force_sensor)) {
    dly_tsk(10 * 1000);
  }
  while (pup_force_sensor_touched(force_sensor)) {
    dly_tsk(10 * 1000);
  }

  gMotors.configure(left_motor_port, right_motor_port);
  gColor.configure(color_sensor_port);
  gSonic.configure(ultrasonic_port);

  printf("Start Nansho1\n");
  act_tsk(NANSHO1_TASK);

  ext_tsk();
}
