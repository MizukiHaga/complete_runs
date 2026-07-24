// #include "kernel_cfg.h"
// #include "app.h"
// #include "Angle/Angle.h"
// #include "Morter/Morter.h"
// #include "DistanceSensor/DistanceSensor.h"
// #include "Battery/Battery.h"
// #include "Sonic/Sonic.h"
// #include "ColorHsv/ColorHsv.h"
// #include "ColorSensor/ColorSensor.h"
// #include "ColorRgbSensor/ColorRgbSensor.h"
// #include <pbio/port.h>
// #include <stdio.h>
// #include <serial/newlib.h>
// #include <serial/serial.h>
// #include <syssvc/serial.h>
// #include <cstdio>

// static const pbio_port_id_t
//   morter_left_motor_port  = PBIO_PORT_ID_A,
//   morter_right_motor_port = PBIO_PORT_ID_B,
//   color_sensor_port       = PBIO_PORT_ID_E;

// void main_task(intptr_t unused)
// {
//   serial_opn_por(SIO_BLUETOOTH_PORTID);
//   FILE *bt_out = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);

//   printf("+---------------------------------+\n");
//   printf("|            TestApp start        |\n");
//   printf("+---------------------------------+\n");

//   /* カラーセンサ(E) を最初に初期化・起動する。
//    * 他センサ (モータ A/B, 距離/超音波 F) が未接続だと、その get_device が
//    * デバイス待ちで無限ループして main_task が止まる。先に周期タスクを
//    * 起動しておけば、以降の init がブロックしても color は動き続ける。
//    * Configure を呼ばないとデバイスが NULL のまま task が走り、NULL 参照で
//    * 落ちるので必ず呼ぶこと。 */

//   /* RGB ログ (Viewcmd) → BLE 出力。 */
//   ColorRgbSensor_Configure(color_sensor_port);
//   ColorRgbSensor_SetOutput(bt_out);

//   /* HSV ログ / フォース押下 HSV 表示は、同じセンサ(E)を二重駆動すると
//    * モード切替が競合するため今は無効化。使うときは RGB を止めること。 */
//   //ColorSensor_Configure(color_sensor_port);
//   //ColorSensor_SetOutput(bt_out);
//   //sta_cyc(COLOR_SENSOR_TASK_CYC);
//   //ColorHsvLight_Init(COLOR_HSV_COLOR_PORT, COLOR_HSV_FORCE_PORT);
//   //sta_cyc(COLOR_HSV_LIGHT_TASK_CYC);

//   Angle_Init(bt_out);
//   Morter_Configure(morter_left_motor_port, morter_right_motor_port);
//   DistanceSensor_Init(bt_out, DISTANCE_SENSOR_PORT);
//   Battery_Init(bt_out);
//   SonicLight_Init(nullptr, SONIC_LIGHT_PORT);

//   //sta_cyc(ANGLE_TASK_CYC);
//   //sta_cyc(MORTER_TASK_CYC);
//   // sta_cyc(DISTANCE_TASK_CYC);
//   //sta_cyc(BATTERY_TASK_CYC);
//   //sta_cyc(SONIC_LIGHT_TASK_CYC);
//   sta_cyc(COLOR_SENSOR_TASK_CYC);
//   //sta_cyc(COLOR_RGB_SENSOR_TASK_CYC);
// }
#include "kernel_cfg.h"
#include "app.h"
#include "Angle/Angle.h"
#include "Morter/Morter.h"
#include "DistanceSensor/DistanceSensor.h"
#include "Battery/Battery.h"
#include "Sonic/Sonic.h"
#include "ColorHsv/ColorHsv.h"
#include "ColorSensor/ColorSensor.h"
#include "ColorRgbSensor/ColorRgbSensor.h"
#include "ReflectionSensor/ReflectionSensor.h"
#include <pbio/port.h>
#include <stdio.h>
#include <serial/newlib.h>
#include <serial/serial.h>
#include <syssvc/serial.h>
#include <cstdio>

static const pbio_port_id_t
  morter_left_motor_port  = PBIO_PORT_ID_A,
  morter_right_motor_port = PBIO_PORT_ID_B,
  color_sensor_port       = PBIO_PORT_ID_E;

void main_task(intptr_t unused)
{
  serial_opn_por(SIO_BLUETOOTH_PORTID);
  FILE *bt_out = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);

  printf("+---------------------------------+\n");
  printf("|            TestApp start        |\n");
  printf("+---------------------------------+\n");

  /* カラーセンサ(E) を最初に初期化・起動する。
   * 他センサ (モータ A/B, 距離/超音波 F) が未接続だと、その get_device が
   * デバイス待ちで無限ループして main_task が止まる。先に周期タスクを
   * 起動しておけば、以降の init がブロックしても color は動き続ける。
   * Configure を呼ばないとデバイスが NULL のまま task が走り、NULL 参照で
   * 落ちるので必ず呼ぶこと。 */

  /* LAB ログ (Viewcmd) → BLE 出力。100ms 周期で L,a,b を送信。
   * カラーセンサのライトを点灯し、RGB から LAB を計算する。
   * 同一ポート(E)では HSV/RGB 単体ログと同時起動しない。 */
  ColorSensor_Configure(color_sensor_port);
  ColorSensor_SetOutput(bt_out);
  sta_cyc(COLOR_SENSOR_TASK_CYC);

  ColorRgbSensor_Configure(color_sensor_port);
  ColorRgbSensor_SetOutput(bt_out);
  //sta_cyc(COLOR_RGB_SENSOR_TASK_CYC);
  //ColorHsvLight_Init(COLOR_HSV_COLOR_PORT, COLOR_HSV_FORCE_PORT);
  //sta_cyc(COLOR_HSV_LIGHT_TASK_CYC);

  /* 反射光ログ → BLE。100ms 周期で REF,<0〜100程度> を送信。
   * 同一ポート(E)の LAB/RGB ログと同時起動しないこと。 */
  ReflectionSensor_Configure(color_sensor_port);
  ReflectionSensor_SetOutput(bt_out);
  //sta_cyc(REFLECTION_SENSOR_TASK_CYC);

  Angle_Init(bt_out);
  Morter_Configure(morter_left_motor_port, morter_right_motor_port);
  DistanceSensor_Init(bt_out, DISTANCE_SENSOR_PORT);
  Battery_Init(bt_out);
  SonicLight_Init(nullptr, SONIC_LIGHT_PORT);

  //sta_cyc(ANGLE_TASK_CYC);
  //sta_cyc(MORTER_TASK_CYC);
  //sta_cyc(DISTANCE_TASK_CYC);
  //sta_cyc(BATTERY_TASK_CYC);
  //sta_cyc(SONIC_LIGHT_TASK_CYC);
}
