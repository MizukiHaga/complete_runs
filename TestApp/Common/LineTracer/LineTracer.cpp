/*
 * LineTracer モジュール 実装
 *  反射値の PID 制御で黒線エッジを追従する。
 */

#include "LineTracer.h"

float LineTracer::clampFloat(float value, float min_value, float max_value)
{
  if (value < min_value) { return min_value; }
  if (value > max_value) { return max_value; }
  return value;
}

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
  const uint16_t target_brightness =
      (WhiteBrightness + BlackBrightness) / 2;

  const int32_t ref = pup_color_sensor_reflection(color_sensor_);

  /* 偏差: + ならセンサが暗い側（黒寄り）、 - なら明るい側（白寄り） */
  const float diff_brightness = (float)(target_brightness - ref);

  const float p_term = kp_ * diff_brightness;

  integral_ += diff_brightness;
  if (integral_ >  IntegralMax) { integral_ =  IntegralMax; }
  if (integral_ < -IntegralMax) { integral_ = -IntegralMax; }
  const float i_term = ki_ * integral_;

  const float d_term = kd_ * (diff_brightness - prev_error_);
  prev_error_ = diff_brightness;

  float pid_output = p_term + i_term + d_term;
  pid_output       = clampFloat(pid_output, -out_max_, out_max_);
  return (int16_t)pid_output;
}

/* 操舵量を左右モータパワーに反映する。
 * 左エッジ追従なら左モータには + 、右モータには - を加える。
 *
 * 後退対応:
 *   ベースパワーが負 (後退) のときは、操舵の符号を反転する。 */
void LineTracer::motorDriveControl(int16_t steering_amount)
{
  const float dir_sign =
      ((left_base_speed_ + right_base_speed_) < 0.0F) ? -1.0F : 1.0F;
  const int edge_sign = follow_left_edge_ ? 1 : -1;
  const int steer = (int)(steering_amount * edge_sign * dir_sign);

  int left_motor_power  = (int)left_base_speed_  + steer;
  int right_motor_power = (int)right_base_speed_ - steer;

  left_motor_power  = clampMotorPower(left_motor_power);
  right_motor_power = clampMotorPower(right_motor_power);

  pup_motor_set_power(left_,  left_motor_power);
  pup_motor_set_power(right_, right_motor_power);
}
