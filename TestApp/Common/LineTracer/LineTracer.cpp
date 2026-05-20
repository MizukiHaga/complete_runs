/*
 * LineTracer モジュール 実装
 *  反射値の PID 制御で黒線エッジを追従する。
 *  app.cfg の周期タスク (TRACER_TASK, 100ms) から tracer_task() が呼ばれる。
 */

#include "app.h"
#include "LineTracer.h"
#include <stdio.h>

#include <spike/pup/motor.h>
#include <spike/hub/speaker.h>
#include <spike/pup/colorsensor.h>
#include <spike/hub/system.h>

/* 関数プロトタイプ宣言（ファイル内で先に使う関数のため） */
static int16_t steering_amount_calculation(void);
static void    motor_drive_control(int16_t);
static int     clamp_motor_power(int power);

/* === モジュール内のグローバル状態 ========================= */
static pup_motor_t  *fg_left_motor;
static pup_motor_t  *fg_right_motor;
static pup_device_t *fg_color_sensor;

/* PID 制御の内部状態 */
static float fg_integral   = 0.0F;          // 積分項の累積値
static float fg_prev_error = 0.0F;          // 1 ステップ前の偏差（微分項用）
static float fg_kp         = PID_DEFAULT_KP;
static float fg_ki         = PID_DEFAULT_KI;
static float fg_kd         = PID_DEFAULT_KD;
static float fg_out_max    = PID_OUTPUT_MAX;

/* float 値を [min, max] にクランプ */
static float clamp_float(float value, float min_value, float max_value)
{
  if (value < min_value) { return min_value; }
  if (value > max_value) { return max_value; }
  return value;
}

/* 初期化: 左右モータ＋カラーセンサのデバイスを取得し、回転方向を設定する。 */
void LineTracer_Configure(pbio_port_id_t left_motor_port,
                          pbio_port_id_t right_motor_port,
                          pbio_port_id_t color_sensor_port)
{
  fg_color_sensor = pup_color_sensor_get_device(color_sensor_port);
  fg_left_motor   = pup_motor_get_device(left_motor_port);
  fg_right_motor  = pup_motor_get_device(right_motor_port);

  pup_motor_setup(fg_left_motor,  PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(fg_right_motor, PUP_DIRECTION_CLOCKWISE,        true);
}

/* 周期タスク本体 (100ms 周期で呼ばれる)。
 * 1 回の呼び出しで「センサ読み取り → 操舵量計算 → モータ反映」を一巡実行。 */
void tracer_task(intptr_t unused)
{
  int16_t steering_amount;

  steering_amount = steering_amount_calculation();
  motor_drive_control(steering_amount);

  ext_tsk();
}

/* PID 計算によるステアリング操舵量の算出。
 *
 *   目標輝度 = (WHITE + BLACK) / 2  ←黒と白の中間。これを境界として追従する
 *   diff     = 目標 - 現在
 *
 *   P 項 = Kp * diff
 *   I 項 = Ki * Σdiff   （PID_I_MAX でクランプ）
 *   D 項 = Kd * (diff - prev_diff)
 *
 *   出力 = P + I + D（±PID_OUTPUT_MAX でクランプ）
 */
static int16_t steering_amount_calculation(void)
{
  uint16_t  target_brightness;
  float     diff_brightness;
  float     p_term;
  float     i_term;
  float     d_term;
  float     pid_output;
  int16_t   steering_amount;
  int32_t   ref;

  /* 目標輝度 = 白/黒の中点 */
  target_brightness = (WHITE_BRIGHTNESS + BLACK_BRIGHTNESS) / 2;

  /* 現在の反射値を取得 */
  ref = pup_color_sensor_reflection(fg_color_sensor);

  /* 偏差: + ならセンサが暗い側（黒寄り）、 - なら明るい側（白寄り） */
  diff_brightness = (float)(target_brightness - ref);

  /* --- P 項 --- */
  p_term = fg_kp * diff_brightness;

  /* --- I 項（積分のワインドアップを防ぐためクランプ）--- */
  fg_integral += diff_brightness;
  if (fg_integral >  PID_I_MAX) { fg_integral =  PID_I_MAX; }
  if (fg_integral < -PID_I_MAX) { fg_integral = -PID_I_MAX; }
  i_term = fg_ki * fg_integral;

  /* --- D 項 --- */
  d_term = fg_kd * (diff_brightness - fg_prev_error);
  fg_prev_error = diff_brightness;

  /* PID 合計を出力上限内に丸めて int16_t に変換 */
  pid_output      = p_term + i_term + d_term;
  pid_output      = clamp_float(pid_output, -fg_out_max, fg_out_max);
  steering_amount = (int16_t)pid_output;

  return steering_amount;
}

/* 操舵量を左右モータパワーに反映する。
 * 左エッジ追従なら左モータには + 、右モータには - を加える。
 * 右エッジに切り替える場合は LEFT_EDGE を RIGHT_EDGE に書き換える。 */
static void motor_drive_control(int16_t steering_amount)
{
  int left_motor_power;
  int right_motor_power;

  left_motor_power  = (int)(LEFT_BASE_SPEED  + (steering_amount * LEFT_EDGE));
  right_motor_power = (int)(RIGHT_BASE_SPEED - (steering_amount * LEFT_EDGE));

  /* モータパワーを [-100, 100] にクランプ */
  left_motor_power  = clamp_motor_power(left_motor_power);
  right_motor_power = clamp_motor_power(right_motor_power);

  pup_motor_set_power(fg_left_motor,  left_motor_power);
  pup_motor_set_power(fg_right_motor, right_motor_power);
}

/* モータパワーを [-100, 100] に丸める。 */
static int clamp_motor_power(int power)
{
  if (power >  100) { return  100; }
  if (power < -100) { return -100; }
  return power;
}
