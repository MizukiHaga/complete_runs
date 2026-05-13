#include "kernel_cfg.h"
#include "app.h"
#include <stdio.h>
#include <math.h>
#include <serial/newlib.h>

#include <spike/hub/imu.h>
#include <libcpp/spike/Motor.h> 
#include <libcpp/spike/Clock.h>
#include <libcpp/spike/ForceSensor.h> 

using namespace spikeapi; 

#define LOOP_DELAY_US 10000  // 10 ms loop
#define CLOCKS_PER_SEC 1000000
#define SQUARE_SIDES 8 // 方向転換を行いながら走行する正方形の辺の数
#define KP 1.2
#define KI 0.001
#define KD 0.02
#define MAX_POWER 100
#define BASE_POWER 55
#define MIN_POWER 30

float pid_control(float target, float current, float *integral, float *last_error) {
  float error = target - current;
  *integral += error * (LOOP_DELAY_US / 1000000.0f);
  float derivative = (error - *last_error) / (LOOP_DELAY_US / 1000000.0f);
  *last_error = error;
  return KP * error + KI * (*integral) + KD * derivative;
}

float clamp(float power) {
    if (power >= 0) {
        if (power > MAX_POWER) return MAX_POWER;
        if (power < MIN_POWER) return MIN_POWER;
    } else {
        if (power < -MAX_POWER) return -MAX_POWER;
        if (power > -MIN_POWER) return -MIN_POWER;
    }
    return power;
}

void drive_straight(Motor &right, Motor &left, Clock &clock, float duration_sec, float target_heading) {
  float integral = 0.0;
  float last_error = 0.0;
  float time_start = ((float)clock.now() / CLOCKS_PER_SEC);

  while (((float)clock.now() / CLOCKS_PER_SEC) - time_start < duration_sec) {
    float current_heading = hub_imu_get_heading();
    float correction = pid_control(target_heading, current_heading, &integral, &last_error);

    int left_power = clamp(BASE_POWER + correction);
    int right_power = clamp(BASE_POWER - correction);

    right.setPower(right_power);
    left.setPower(left_power);
    dly_tsk(LOOP_DELAY_US);
  }
  right.stop();
  left.stop();
}

void turn_to_heading(Motor &right, Motor &left, float target_heading) {
  float integral = 0.0;
  float last_error = 0.0;

  while (true) {
    float current_heading = hub_imu_get_heading();
    float error = target_heading - current_heading;
    // normalize error to [-180, 180]
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    if (fabs(error) < 1.0) break;

    float power = clamp(pid_control(target_heading, current_heading, &integral, &last_error));

    right.setPower(-power);
    left.setPower(power);
    dly_tsk(LOOP_DELAY_US);
  }
}

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {

  dly_tsk(5*1000*1000);

  FILE *fp = serial_open_newlib_file(SIO_USB_PORTID);
  fprintf(fp, "Start !!\n");
    
  hub_imu_init();

  // HackSPi's hub is tilted at ~51 degrees.
  hub_imu_set_tilt(51.0f);

  // Wait for IMU to become ready
  while (!hub_imu_is_ready()) {
    fprintf(fp, ".");
    dly_tsk(100*1000);
  }
  fprintf(fp, "\n");

  Motor leftWheel(EPort::PORT_B,Motor::EDirection::COUNTERCLOCKWISE,true);
  Motor rightWheel(EPort::PORT_A,Motor::EDirection::CLOCKWISE,true);
  ForceSensor forceSensor(EPort::PORT_D);
  Clock clock;

  /* フォースセンサーが押下されるまで待機 */
  unsigned int i = 0;
  while (forceSensor.isTouched() == false) {
    if (i++ % 100 == 0) {
      /* IMUの状態を表示 */
      float w[3], a[3];
      bool stationary = hub_imu_is_stationary();
      fprintf(fp, "[%c] ", stationary ? 'T' : 'F');
      hub_imu_get_acceleration(a);
      fprintf(fp, "a = (%.0f %.0f %.0f); ", a[0], a[1], a[2]);
      hub_imu_get_angular_velocity(w);
      fprintf(fp, "ω = (%.0f %.0f %.0f); ", w[0], w[1], w[2]);
      fprintf(fp, "h = %.0f\n", hub_imu_get_heading());
      fprintf(fp, "\n***** Press force sensor to start *****\n");
    }
    dly_tsk(10*1000);
  }

  float heading = hub_imu_get_heading();
  fprintf(fp,"[heading] initial    = %f\n",heading);

  for (int i = 0; i < SQUARE_SIDES; i++) {
    drive_straight(rightWheel, leftWheel, clock,3.0, heading); // 3 seconds straight
    heading = hub_imu_get_heading();
    fprintf(fp, "[heading] before turn= %f\n",heading);
    turn_to_heading(rightWheel, leftWheel, heading+90);
    heading = hub_imu_get_heading();
    fprintf(fp, "[heading] after turn = %f\n",heading);
  }
  rightWheel.stop();
  leftWheel.stop();

  /* タスク終了 */
  ext_tsk();
}
