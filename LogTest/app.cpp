#include "kernel_cfg.h"
#include "app.h"
#include "Angle/Angle.h"
#include "Morter/Morter.h"
#include "DistanceSensor/DistanceSensor.h"
#include "Battery/Battery.h"
#include "Sonic/Sonic.h"
#include <pbio/port.h>
#include <stdio.h>
#include <serial/newlib.h>
#include <serial/serial.h>
#include <syssvc/serial.h>
#include <cstdio>

static const pbio_port_id_t
  morter_left_motor_port  = PBIO_PORT_ID_A,
  morter_right_motor_port = PBIO_PORT_ID_B;

void main_task(intptr_t unused)
{
  serial_opn_por(SIO_BLUETOOTH_PORTID);
  FILE *bt_out = serial_open_newlib_file(SIO_BLUETOOTH_PORTID);

  printf("+---------------------------------+\n");
  printf("|            TestApp start        |\n");
  printf("+---------------------------------+\n");

  Angle_Init(bt_out);
  Morter_Configure(morter_left_motor_port, morter_right_motor_port);
  DistanceSensor_Init(bt_out, DISTANCE_SENSOR_PORT);
  Battery_Init(bt_out);
  SonicLight_Init(nullptr, SONIC_LIGHT_PORT);

  sta_cyc(ANGLE_TASK_CYC);
  //sta_cyc(MORTER_TASK_CYC);
  //sta_cyc(DISTANCE_TASK_CYC);
  //sta_cyc(BATTERY_TASK_CYC);
  //sta_cyc(SONIC_LIGHT_TASK_CYC);

}
