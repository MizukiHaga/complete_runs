#include "kernel_cfg.h"
#include "app.h"
#include <stdio.h>
#include <serial/newlib.h>
#include <serial/serial.h>
#include <cstdio>
#include "LineTracer.h"
#include "Nansho1.h"
#include "ColorHSV.h"
#include "ColorRGB.h"
  #include "Gyro.h"
#include "Sonic.h"
#include "Straight.h"
#include "Common/HCR/HCR.h"
#include <spike/hub/speaker.h>

extern "C" {
#include <spike/pup/forcesensor.h>
#include <spike/hub/battery.h>
}

/* === Common インスタンス === */
Straight gMotors;
ColorHSV gColorHSV;
ColorRGB gColorRGB;
Sonic gSonic;
Gyro gGyro;
HCR gHCR;
LineTracer gLineTracer;

/* === Scene: Member A ===
 * SPIKE-RT では引数付きグローバル C++ コンストラクタに依存しない。
 * Nansho1Scene は実行時に gMotors/gColorHSV/gColorRGB/gSonic を直接参照する。 */
Nansho1Scene gNansho1;

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
  
  gMotors.configure(left_motor_port, right_motor_port);
  gColorHSV.configure(color_sensor_port);
  gColorRGB.configure(color_sensor_port);
  gSonic.configure(ultrasonic_port);
  gGyro.init();
  gLineTracer.configure(left_motor_port, right_motor_port, color_sensor_port);
  
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
  
  // gMotors.configure(left_motor_port, right_motor_port);
  // gColor.configure(color_sensor_port);
  // gSonic.configure(ultrasonic_port);
  gHCR.configure_hrc(left_motor_port, right_motor_port, color_sensor_port);
  gHCR.run_hrc();


  /* IMU 初期化と準備完了待ち。Nansho1 の旋回がヨー角に依存するため、
   * シーン開始前に IMU の is_ready が true になるのを待つ (内部で polling)。 */

  // printf("Start Nansho1\n");
  // act_tsk(NANSHO1_TASK);

  ext_tsk();
}

/* ライントレース周期ハンドラ。LINE_TRACER_PERIOD ごとに実行される。 */
extern "C" void line_tracer_cyclic_handler(intptr_t exinf)
{
  (void)exinf;
  gLineTracer.step();
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