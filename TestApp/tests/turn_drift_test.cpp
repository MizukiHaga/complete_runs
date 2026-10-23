#include "../Common/Gyro/TurnDrift.h"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
static float wheel(float angle) {
  return angle * TurnDriftConfig::EncoderPer90Deg / 90.0f;
}
int main() {
  for (float sign : {-1.0f, 1.0f}) {
    const float gyro = sign < 0 ? TurnDriftConfig::LeftGyroStdDevDeg : TurnDriftConfig::RightGyroStdDevDeg;
    const float encoder = sign < 0 ? TurnDriftConfig::LeftEncoderStdDevDeg : TurnDriftConfig::RightEncoderStdDevDeg;
    const float expected_gain = gyro * gyro / (gyro * gyro + encoder * encoder);
    TurnDrift drift;
    float c = drift.update(sign*85, wheel(90), wheel(90), 0.75f, sign);
    assert(fabsf(drift.gain()-expected_gain) < 0.001f);
    assert(fabsf(c-sign*5*expected_gain) < 0.001f);
    assert(fabsf(drift.residual()-sign*5*(1-expected_gain)) < 0.001f);
    // 再適用で部分補正が全量へ近づくことを防ぐ。
    assert(drift.update(sign*85, wheel(90), wheel(90), 1, sign) == 0);
    TurnDrift mismatch;
    float m = mismatch.update(sign*85, wheel(100), wheel(80), 0.75f, sign);
    assert(fabsf(m) < fabsf(c));
    assert(mismatch.gain() < drift.gain());
    TurnDrift large;
    assert(fabsf(large.update(sign*265, wheel(270), wheel(270), 1.5f, sign)-sign*5*expected_gain) < 0.001f);
    TurnDrift exact;
    assert(exact.update(sign*90, wheel(90), wheel(90), 0.75f, sign) == 0);
  }
  TurnDrift invalid;
  assert(invalid.update(NAN, 180, 180, 1, 1) == 0);
  assert(invalid.status() == TurnDrift::Invalid);
  assert(invalid.update(90, 180, 0, 1, 1) == 0);
  assert(invalid.status() == TurnDrift::WheelMismatch);
  assert(invalid.update(90, 180, 180, 0, 1) == 0);
  assert(invalid.status() == TurnDrift::Invalid);
  assert(invalid.update(-90, 180, 180, 1, 1) == 0);
  assert(invalid.status() == TurnDrift::WrongDirection);
  TurnDrift abnormal;
  assert(abnormal.update(98.42f, wheel(233.5f), wheel(233.5f), 1, 1) == 0);
  assert(abnormal.status() == TurnDrift::AngleMismatch);
  TurnDrift rate;
  assert(rate.update(80, wheel(90), wheel(90), 0.2f, 1) == 0);
  assert(rate.status() == TurnDrift::RateMismatch);
  puts("turn_drift_test: PASS");
}
