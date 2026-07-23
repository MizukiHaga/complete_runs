/*
 * LineTracer モジュール 実装
 *  反射値の PID 制御で黒線エッジを追従する。
 */

#include "LineTracer.h"
#include <stdio.h>

#include <spike/pup/motor.h>
#include <spike/hub/speaker.h>
#include <spike/pup/colorsensor.h>
#include <spike/hub/system.h>
#include <math.h>

/* float 値を [min, max] にクランプ */

/* float 値を [min, max] にクランプ */
static float clamp_float(float value, float min_value, float max_value)
{
  if (value < min_value) { return min_value; }
  if (value > max_value) { return max_value; }
  return value;
}

/* 調整パラメータ（必要なら LineTracer.h に移動しても良い） */
static const float REF_ALPHA          = 0.30F; // EMA の係数（0..1）
static const float D_FILTER_ALPHA     = 0.50F; // 微分値フィルタ係数（0..1）
static const float STEERING_DEADBAND  = 1.0F;   // 小さな操舵を無視するデッドバンド

int LineTracer::clampMotorPower(int power)
{
  if (power >  100) { return  100; }
  if (power < -100) { return -100; }
  return power;
}

void LineTracer::configure(pbio_port_id_t left_motor_port,
                           pbio_port_id_t right_motor_port,
                           pbio_port_id_t color_sensor_port)
{
  color_sensor_ = pup_color_sensor_get_device(color_sensor_port);
  left_         = pup_motor_get_device(left_motor_port);
  right_        = pup_motor_get_device(right_motor_port);

  pup_motor_setup(left_,  PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(right_, PUP_DIRECTION_CLOCKWISE,        true);
}

void LineTracer::step()
{
  if (!enabled_) {
    return;
  }
  const int16_t steering_amount = steeringAmountCalculation();
  motorDriveControl(steering_amount);
}

void LineTracer::setBaseSpeed(float left_power, float right_power)
{
  left_base_speed_  = (float)clampMotorPower((int)left_power);
  right_base_speed_ = (float)clampMotorPower((int)right_power);
}

void LineTracer::setPid(float kp, float ki, float kd, float out_max)
{
  kp_      = kp;
  ki_      = ki;
  kd_      = kd;
  out_max_ = out_max;

  integral_   = 0.0F;
  prev_error_ = 0.0F;
}

void LineTracer::setEdgeFollow(bool follow_left_edge)
{
  follow_left_edge_ = follow_left_edge;
  integral_         = 0.0F;
  prev_error_       = 0.0F;
}

/* PID 計算によるステアリング操舵量の算出。
*
*   目標輝度 = (WHITE + BLACK) / 2
*   diff     = 目標 - 現在
 *
 *   P 項 = Kp * diff
 *   I 項 = Ki * Σdiff   （IntegralMax でクランプ）
 *   D 項 = Kd * (diff - prev_diff)
 *
 *   出力 = P + I + D（±out_max_ でクランプ）
 */
int16_t LineTracer::steeringAmountCalculation()
{
  uint16_t  target_brightness;
  float     diff_brightness;
  float     d_term;
  float     pid_output;
  int16_t   steering_amount;
  int32_t   ref;
  
  /* 目標輝度 = 白/黒の中点 */
  target_brightness = (WHITE_BRIGHTNESS + BLACK_BRIGHTNESS) / 2;
  
  /* 現在の反射値を取得（生値） */
  ref = pup_color_sensor_reflection(color_sensor_);

  /* EMA による平滑化（初回は生値で初期化） */
  if (!initialized_) {
    smoothed_ref_ = (float)ref;
    prev_error_   = (float)(target_brightness - smoothed_ref_);
    prev_diff_filtered_ = prev_error_;
    initialized_ = true;
  }
  smoothed_ref_ = (REF_ALPHA * (float)ref) + ((1.0F - REF_ALPHA) * smoothed_ref_);

  /* 偏差: EMA 後の反射値を使う */
  diff_brightness = (float)(target_brightness - smoothed_ref_);

  const float p_term = kp_ * diff_brightness;

  integral_ += diff_brightness;
  if (integral_ >  IntegralMax) { integral_ =  IntegralMax; }
  if (integral_ < -IntegralMax) { integral_ = -IntegralMax; }
  const float i_term = ki_ * integral_;

  /* --- D 項 --- */
  {
    float raw_deriv = diff_brightness - prev_error_;
    float filt_deriv = (D_FILTER_ALPHA * raw_deriv) + ((1.0F - D_FILTER_ALPHA) * prev_diff_filtered_);
    d_term = kd_ * filt_deriv;
    prev_diff_filtered_ = filt_deriv;
    prev_error_ = diff_brightness;
  }

  /* PID 合計を出力上限内に丸めて int16_t に変換 */
  pid_output      = p_term + i_term + d_term;
  pid_output      = clamp_float(pid_output, -out_max_, out_max_);
  steering_amount = (int16_t)pid_output;

  /* 小さな操舵はデッドバンドで無視して振動を抑える */
  if (abs(steering_amount) <= (int)STEERING_DEADBAND) {
    steering_amount = 0;
  }

  return steering_amount;
}

/* 操舵量を左右モータパワーに反映する。
 * 左エッジ追従なら左モータには + 、右モータには - を加える。
 *
 * 後退対応:
 *   ベースパワーが負 (後退) のときは、操舵の符号を反転する。 */
void LineTracer::motorDriveControl(int16_t steering_amount)
{
  /* 操舵量にスケールを掛けて曲がりやすくする */
  float scaled_steering = (float)steering_amount * steering_scale_;
  /* エッジの設定に応じて符号を決定（LEFT_EDGE 優先） */
  int edge_sign = (LeftEdge ? 1 : (RightEdge ? -1 : 1));
  int left_motor_power;
  int right_motor_power;
  left_motor_power  = (int)(left_base_speed_  + (scaled_steering * edge_sign));
  right_motor_power = (int)(right_base_speed_ - (scaled_steering * edge_sign));

  left_motor_power  = clampMotorPower(left_motor_power);
  right_motor_power = clampMotorPower(right_motor_power);

  pup_motor_set_power(left_,  left_motor_power);
  pup_motor_set_power(right_, right_motor_power);
}

void LineTracer::setScale(float scale)
{
  steering_scale_ = scale;
}