#include "app.h"
#include "LineTracer.h"
#include <stdio.h>

#include <spike/pup/motor.h>
#include <spike/hub/speaker.h>
#include <spike/pup/colorsensor.h>
#include <spike/hub/system.h>
/* 関数プロトタイプ宣言 */
static int16_t steering_amount_calculation(void);
static void motor_drive_control(int16_t);
static int clamp_motor_power(int power);

static pup_motor_t *fg_left_motor;
static pup_motor_t *fg_right_motor;
static pup_device_t *fg_color_sensor;
static float fg_integral = 0.0F;
static float fg_prev_error = 0.0F;
static float fg_kp = PID_DEFAULT_KP;
static float fg_ki = PID_DEFAULT_KI;
static float fg_kd = PID_DEFAULT_KD;
static float fg_out_max = PID_OUTPUT_MAX;

static float clamp_float(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void LineTracer_Configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port, pbio_port_id_t color_sensor_port)
{
  /* センサー入力ポートの設定 */
  fg_color_sensor = pup_color_sensor_get_device(color_sensor_port);
  fg_left_motor   = pup_motor_get_device(left_motor_port);
  fg_right_motor   = pup_motor_get_device(right_motor_port);  

  pup_motor_setup(fg_left_motor,PUP_DIRECTION_COUNTERCLOCKWISE,true);
  pup_motor_setup(fg_right_motor,PUP_DIRECTION_CLOCKWISE,true);  
}

/* ライントレースタスク(100msec周期で関数コールされる) */
void tracer_task(intptr_t unused) {

    int16_t steering_amount; /* ステアリング操舵量の計算 */

    /* ステアリング操舵量の計算 */
    steering_amount = steering_amount_calculation();
    /* 走行モータ制御 */
    motor_drive_control(steering_amount);

    /* タスク終了 */
    ext_tsk();
}

/* ステアリング操舵量の計算 */
static int16_t steering_amount_calculation(void){

    uint16_t  target_brightness; /* 閾値 */
    float     diff_brightness;   /* 閾値との差分値 */
    float     p_term;
    float     i_term;
    float     d_term;
    float     pid_output;
    int16_t   steering_amount;   /* ステアリング操舵量 */
    int32_t   ref;

    /* 閾値の計算 */
    target_brightness = (WHITE_BRIGHTNESS + BLACK_BRIGHTNESS) / 2;

    /* カラーセンサ値の取得 */
    ref = pup_color_sensor_reflection(fg_color_sensor);

    /* 閾値とカラーセンサ値の差分を計算 */
    diff_brightness = (float)(target_brightness - ref);

    /* PIDの各項を計算 */
    p_term = fg_kp * diff_brightness;

    fg_integral += diff_brightness;
    if (fg_integral > PID_I_MAX) {
        fg_integral = PID_I_MAX;
    } else if (fg_integral < -PID_I_MAX) {
        fg_integral = -PID_I_MAX;
    }
    i_term = fg_ki * fg_integral;

    d_term = fg_kd * (diff_brightness - fg_prev_error);
    fg_prev_error = diff_brightness;

    pid_output = p_term + i_term + d_term;
    pid_output = clamp_float(pid_output, -fg_out_max, fg_out_max);
    steering_amount = (int16_t)pid_output;

    return steering_amount;
}

/* 走行モータ制御 */
static void motor_drive_control(int16_t steering_amount){

    int left_motor_power, right_motor_power; /*左右モータ設定パワー*/

    /* 左右モータ駆動パワーの計算(走行エッジを右にする場合はRIGHT_EDGEに書き換えること) */
    left_motor_power  = (int)(LEFT_BASE_SPEED + (steering_amount * LEFT_EDGE));
    right_motor_power = (int)(RIGHT_BASE_SPEED - (steering_amount * LEFT_EDGE));

    /* 左右モータ駆動パワーの設定 */
    left_motor_power = clamp_motor_power(left_motor_power);
    right_motor_power = clamp_motor_power(right_motor_power);
    pup_motor_set_power(fg_left_motor, left_motor_power);
    pup_motor_set_power(fg_right_motor, right_motor_power);

    return;
}

static int clamp_motor_power(int power){
    if (power > 100) {
        return 100;
    }
    if (power < -100) {
        return -100;
    }
    return power;
}
