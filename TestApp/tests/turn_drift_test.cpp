#include "../Common/Gyro/TurnDrift.h"

#include <assert.h>
#include <stdio.h>
#include <initializer_list>

static float wheel(float angle)
{
  return angle * TurnDriftConfig::EncoderPer90Deg / 90.0f;
}

static void near(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.0001f);
}

int main()
{
  const float weight = TurnDriftConfig::EncoderWeight;
  for (float sign : {-1.0f, 1.0f}) {
    TurnDrift drift;
    near(drift.estimate(0, 0, 0, sign), 0);
    assert(drift.status() == TurnDrift::Accepted);
    near(drift.gain(), weight);
    near(drift.estimate(sign, sign * 2, -sign * 4, sign), sign * 0.5f * weight);
    // 左右の符号、繰り返し評価、最終補正の一度だけの適用。
    for (int sample = 0; sample < 100; ++sample) {
      near(drift.estimate(sign * 40, sign * wheel(45), -sign * wheel(45), sign),
           sign * 5 * weight);
    }
    near(drift.update(sign * 85, sign * wheel(90), -sign * wheel(90), 1, sign),
         sign * 5 * weight);
    near(drift.residual(), sign * 5 * (1 - weight));
    near(drift.update(sign * 85, sign * wheel(90), -sign * wheel(90), 1, sign), 0);

    TurnDrift long_turn;
    near(long_turn.estimate(sign * 265, sign * wheel(270), -sign * wheel(270), sign),
         sign * 5 * weight);
    TurnDrift coast;
    near(coast.estimate(sign * 85, sign * wheel(90), -sign * wheel(90), sign),
         sign * 5 * weight);
    near(coast.update(sign * 88, sign * wheel(92), -sign * wheel(92), 1, sign),
         sign * 4 * weight);

    // 起動時の左右差を理由に融合を止めない。
    TurnDrift startup;
    near(startup.estimate(sign * 10, sign * 30, -sign * 10, sign, true, 40), 0);
    near(startup.estimate(sign * 20, sign * 50, -sign * 30, sign, true, 60), 0);
    near(startup.estimate(sign * 85, sign * 180, -sign * 160, sign, true, 200), 0);
    near(startup.estimate(sign * 40, sign * 100, -sign * 20, sign, true, 300),
         -sign * 10 * weight);
    assert(startup.status() == TurnDrift::Accepted);
    near(startup.gain(), weight);
    assert(startup.count(TurnDrift::EncoderUnderRead) == 1);

    // 過大・過小を対称に反映し、ゼロ平均の測定誤差を片側で切り捨てない。
    TurnDrift symmetric;
    const float below = symmetric.estimate(sign * 90, sign * wheel(86),
                                            -sign * wheel(86), sign);
    const float above = symmetric.estimate(sign * 90, sign * wheel(94),
                                            -sign * wheel(94), sign);
    near(below, -sign * 4 * weight);
    near(above, sign * 4 * weight);
    near(below + above, 0);
    assert(symmetric.status() == TurnDrift::Accepted);
    // 停止時の過小側も補正し、経過時間0で拒否しない。
    TurnDrift under;
    near(under.update(sign * 90, sign * wheel(85), -sign * wheel(85), 0, sign),
         -sign * 5 * weight);
    near(under.update(sign * 90, sign * wheel(95), -sign * wheel(95), 1, sign), 0);

    // 両輪に同じ前後移動を加えても旋回角は変わらない。
    TurnDrift translation;
    near(translation.estimate(sign * 85, sign * 180 + 100, -sign * 180 + 100, sign),
         sign * 5 * weight);
    near(translation.encoderAngle(), sign * 90);
    // 指定方向が実際の車輪の向きと違っても、実測の符号を使用する。
    near(translation.estimate(-sign * 85, -sign * 180, sign * 180, sign),
         -sign * 5 * weight);
  }

  for (float movement : {-180.0f, -40.0f, 0.0f, 40.0f, 180.0f}) {
    TurnDrift straight;
    near(straight.estimate(0, movement, movement, 1), 0);
    near(straight.encoderAngle(), 0);
  }
  TurnDrift invalid;
  near(invalid.estimate(10, 20, -20, 1, false), 0);
  assert(invalid.status() == TurnDrift::Invalid);
  near(invalid.estimate(85, 180, -180, 1), 5 * weight);
  near(invalid.estimate(NAN, 180, -180, 1), 0);
  assert(invalid.status() == TurnDrift::Invalid);
  near(invalid.estimate(85, 180, -180, 1), 5 * weight);
  near(invalid.estimate(85, INFINITY, -180, 1), 0);
  near(invalid.estimate(85, 180, -180, 0), 0);
  near(invalid.estimate(85, 180, -180, 1), 5 * weight);
  puts("turn_drift_test: PASS (signed wheels, symmetric fusion, translation cancellation, one-time update)");
}
