#include "kernel_cfg.h"
#include "app.h"
#include <stdio.h>
#include <serial/newlib.h>
#include <serial/serial.h>
#include <cstdio>
#include "LineTracer.h"
#include "Nansho1.h"
#include "Nansho2.h"
#include "Nansho3.h"
#include "Test.h"
#include "Arm.h"
#include "ColorHSV.h"
#include "ColorLAB.h"
#include "ColorRGB.h"
#include "Gyro.h"
#include "Sonic.h"
#include "Straight.h"
#include "HCR.h"

extern "C" {
#include <spike/pup/forcesensor.h>
#include <spike/hub/battery.h>
#include <spike/hub/speaker.h>
}

/* === Common インスタンス === */
Straight   gMotors;
LineTracer gLineTracer;
ColorHSV   gColorHSV;
ColorLAB   gColorLAB;
ColorRGB   gColorRGB;
Sonic      gSonic;
Gyro       gGyro;
Arm        gArm;
HCR        gHCR;

/* === Scene: Member A === */
Nansho1Scene gNansho1;

/* === Scene: Member B === */
Nansho2Scene gNansho2;
Nansho3Scene gNansho3;

/* === Scene: Test === */
TestScene gTest;

/* センサーポートの定義 */
static const pbio_port_id_t color_sensor_port = PBIO_PORT_ID_E,
                            left_motor_port = PBIO_PORT_ID_B,
                            right_motor_port = PBIO_PORT_ID_A,
                            arm_motor_port = NANSHO2_ARM_PORT,
                            force_sensor_port = PBIO_PORT_ID_D,
                            ultrasonic_port = NANSHO1_ULTRASONIC_PORT;

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused)
{
  (void)unused;

  gMotors.configure(left_motor_port, right_motor_port);
  gColorHSV.configure(color_sensor_port);
  gColorLAB.configure(color_sensor_port);
  gColorRGB.configure(color_sensor_port);
  gSonic.configure(ultrasonic_port);
  gArm.configure(arm_motor_port);
  gLineTracer.configure(left_motor_port, right_motor_port, color_sensor_port);
  gGyro.init();

  pup_device_t *force_sensor = pup_force_sensor_get_device(force_sensor_port);
  hub_speaker_set_volume(50);
  hub_speaker_play_tone(1000, 300);
  while (!pup_force_sensor_touched(force_sensor)) {
    dly_tsk(10 * 1000);
  }
  while (pup_force_sensor_touched(force_sensor)) {
    dly_tsk(10 * 1000);
  }

  printf("Battery: %u mV\n", (unsigned)hub_battery_get_voltage());


#if ETROBO_SCENE == 0
  gHCR.configure_hrc(left_motor_port, right_motor_port, color_sensor_port);
  gHCR.run_hrc();
  // gHCR.AllLineTrace();
  act_tsk(NANSHO1_TASK);
#elif ETROBO_SCENE == 1 || ETROBO_SCENE == 5
  act_tsk(NANSHO1_TASK);
#elif ETROBO_SCENE == 2
  act_tsk(NANSHO2_TASK);
#elif ETROBO_SCENE == 3
  gHCR.configure_hrc(left_motor_port, right_motor_port, color_sensor_port);
  gHCR.run_hrc();
#elif ETROBO_SCENE == 4 || ETROBO_SCENE == 13
  act_tsk(NANSHO3_TASK);
#elif ETROBO_SCENE == 6
  act_tsk(TEST_TASK);
#elif ETROBO_SCENE == 7 || ETROBO_SCENE == 8 || ETROBO_SCENE == 9 || ETROBO_SCENE == 10 || ETROBO_SCENE == 11 || ETROBO_SCENE == 12
  act_tsk(TEST_TASK);
#else
#error "ETROBO_SCENE must be 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 or 13"
#endif
  ext_tsk();
}

/* ライントレース周期タスク。
 * app.cfg の LINE_TRACER_TASK_CYC から起動される。 */
extern "C" void tracer_task(intptr_t exinf)
{
  (void)exinf;
  gLineTracer.step();
  ext_tsk();
}

/* Straight PID 補正タスク。
 * CRE_CYC の TA_STA で起動時から STRAIGHT_BALANCE_PERIOD ごとに活性化される。
 * gMotors.tickCorrection() が未 configure / 停止指令中は何もしないので、
 * configure 前から回っていても安全。 */
extern "C" void StraightBalanceTask(intptr_t exinf)
{
  (void)exinf;
  gMotors.tickCorrection();
  ext_tsk();
}
