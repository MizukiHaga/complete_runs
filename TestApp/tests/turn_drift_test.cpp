#include "../Common/Gyro/TurnDrift.h"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
static float wheel(float angle) {
  return angle * TurnDriftConfig::EncoderPer90Deg / 90.0f;
}
static void near(float a, float b) { assert(fabsf(a-b) < 0.0001f); }
int main() {
  const float w = TurnDriftConfig::EncoderWeight;
  for (float sign : {-1.0f, 1.0f}) {
    TurnDrift drift;
    near(drift.estimate(0, 0, 0, sign), 0);
    assert(drift.status() == TurnDrift::WarmingUp);
    // 180°未満でも融合。何度評価しても補正を積算しない。
    for (int k = 0; k < 100; ++k)
      near(drift.estimate(sign*40, wheel(45), wheel(45), sign), sign*5*w);
    near(drift.gain(), w);
    near(drift.update(sign*85, wheel(90), wheel(90), 1, sign), sign*5*w);
    near(drift.residual(), sign*5*(1-w));
    near(drift.update(sign*85, wheel(90), wheel(90), 1, sign), 0);
    TurnDrift long_turn;
    near(long_turn.estimate(sign*265, wheel(270), wheel(270), sign), sign*5*w);
    TurnDrift coast;
    near(coast.estimate(sign*85, wheel(90), wheel(90), sign), sign*5*w);
    // 最終補正は停止直前の値ではなく惰性後の最終値。
    near(coast.update(sign*88, wheel(92), wheel(92), 1, sign), sign*4*w);
  }
  TurnDrift missing;
  near(missing.estimate(10, 20, 20, 1, false), 0);
  assert(missing.status() == TurnDrift::Invalid);
  near(missing.estimate(85, 180, 180, 1), 0); // 異常後は融合へ戻らない
  TurnDrift mismatch;
  near(mismatch.estimate(40, 100, 20, 1), 0);
  assert(mismatch.status() == TurnDrift::WarmingUp);
  mismatch.estimate(40,100,20,1,true,TurnDriftConfig::MotionFaultConfirmMs);
  assert(mismatch.status() == TurnDrift::WheelMismatch);
  near(mismatch.estimate(85,180,180,1,true,100),0);
  TurnDrift wrong;
  near(wrong.estimate(-10, 20, 20, 1), 0);
  assert(wrong.status() == TurnDrift::WarmingUp);
  wrong.estimate(-10,20,20,1,true,TurnDriftConfig::MotionFaultConfirmMs);
  assert(wrong.status() == TurnDrift::WrongDirection);
  for (float sign : {-1.0f, 1.0f}) {
    TurnDrift startup;
    near(startup.estimate(-sign,1,1,sign,true,0),0);
    near(startup.estimate(sign*2,3,5,sign,true,1),0);
    near(startup.estimate(sign*40,90,90,sign,true,30),sign*5*w);
    assert(startup.status() == TurnDrift::Accepted);
    // 一時不一致→正常復帰→再発では確認時間を取り直す。
    startup.estimate(sign*40,100,20,sign,true,40);
    near(startup.estimate(sign*40,90,90,sign,true,50),sign*5*w);
    startup.estimate(sign*40,100,20,sign,true,60);
    startup.estimate(sign*40,100,20,sign,true,79);
    assert(startup.status() == TurnDrift::WarmingUp);
    startup.estimate(sign*40,100,20,sign,true,80);
    assert(startup.status() == TurnDrift::WheelMismatch);
    // エンコーダ逆方向も継続時のみ無効化。
    TurnDrift reverse;
    reverse.estimate(sign*2,4,4,sign,true,0,false);
    reverse.estimate(sign*2,4,4,sign,true,19,false);
    assert(reverse.status() == TurnDrift::WarmingUp);
    reverse.estimate(sign*2,4,4,sign,true,20,false);
    assert(reverse.status() == TurnDrift::WrongDirection);
    TurnDrift recovered;
    recovered.estimate(sign*2,4,4,sign,true,0,false);
    near(recovered.estimate(sign*40,90,90,sign,true,10,true),sign*5*w);
    assert(recovered.status() == TurnDrift::Accepted);
  }
  TurnDrift nan;
  near(nan.estimate(NAN, 180, 180, 1), 0);
  assert(nan.status() == TurnDrift::Invalid);
  TurnDrift invalid_time;
  near(invalid_time.update(90,180,180,0,1),0);
  assert(invalid_time.status() == TurnDrift::Invalid);
  puts("turn_drift_test: PASS");
}
