#include "../Common/Gyro/TurnDrift.h"
#include <assert.h>
#include <stdio.h>

static void near(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.0001f);
}

int main()
{
  for (int sign = -1; sign <= 1; sign += 2) {
    const float weight = sign < 0 ? TurnDriftConfig::EncoderWeightLeft
                                  : TurnDriftConfig::EncoderWeightRight;
    TurnDrift drift;
    // 開始直後・小回転・左右差・方向判定で補正を除外しない。
    near(drift.estimate(0, 0, 0, sign), 0);
    assert(drift.status() == TurnDrift::Accepted);
    near(drift.estimate(sign * 1, sign * 2, -sign * 4, sign, true, 1, false), sign * 0.5f * weight);
    for (int sample = 0; sample < 100; ++sample)
      near(drift.estimate(sign * 40, sign * 180, -sign * 20, sign, true, sample, false),
           sign * 10 * weight);
    // ジャイロが指定方向と逆でも、値が正常なら同じ式で評価する。
    near(drift.estimate(-sign * 5, sign * 10, -sign * 10, sign), sign * 10 * weight);
    // 欠測・NaN後も正常値で再開し、旋回終了まで無効にしない。
    near(drift.estimate(0, 0, 0, sign, false), 0);
    assert(drift.status() == TurnDrift::Invalid);
    near(drift.estimate(sign * 85, sign * 180, -sign * 180, sign), sign * 5 * weight);
    near(drift.estimate(NAN, sign * 180, -sign * 180, sign), 0);
    near(drift.estimate(sign * 85, sign * 180, -sign * 180, sign), sign * 5 * weight);
    // 過小・過大を対称に反映。
    near(drift.estimate(sign * 95, sign * 180, -sign * 180, sign), -sign * 5 * weight);
    near(drift.estimate(sign * 85, INFINITY, 180, sign), 0);
    near(drift.estimate(sign * 85, sign * 180, -sign * 180, sign), sign * 5 * weight);
    // weight=0設定でも、換算した角度差そのものを検証する。
    near(drift.angleError(), sign * 5);
    // 同じ前後移動が両輪へ加わっても旋回角を変えない。
    near(drift.estimate(sign * 85, sign * 180 + 100,
                        -sign * 180 + 100, sign), sign * 5 * weight);
    near(drift.angleError(), sign * 5);
    // 指定方向ではなく、実際の車輪回転の符号から旋回角を求める。
    near(drift.estimate(-sign * 85, -sign * 180, sign * 180, sign),
         -sign * 5 * weight);
    near(drift.angleError(), -sign * 5);
    // 時間0も許容し、最終補正は一度だけ反映。
    near(drift.update(sign * 85, sign * 180, -sign * 180, 0, sign), sign * 5 * weight);
    near(drift.update(sign * 85, sign * 180, -sign * 180, 1, sign), 0);
    near(drift.gain(), weight);
  }
  for (int movement = -180; movement <= 180; movement += 60) {
    TurnDrift straight;
    near(straight.estimate(0, movement, movement, 1), 0);
    near(straight.angleError(), 0);
  }
  TurnDrift retry;
  near(retry.update(85, 180, -180, 1, 1, false), 0);
  near(retry.update(85, 180, -180, 1, 1), 5 * TurnDriftConfig::EncoderWeightRight);
  puts("turn_drift_test: PASS (signed wheels, translation cancellation, missing-data recovery, one-time update)");
}
