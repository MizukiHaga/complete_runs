#include "../Common/Gyro/HeadingFusion.h"
#include <assert.h>
#include <stdio.h>

static void near(float actual, float expected, float tolerance = 0.001f)
{
  if (fabsf(actual - expected) > tolerance) {
    fprintf(stderr, "actual=%f expected=%f tolerance=%f\n", actual, expected, tolerance);
    assert(false);
  }
}

int main()
{
  HeadingFusion filter;
  HeadingFusionConfig config;
  assert(filter.configure(config));
  near(filter.wheelAngle(180, -180), 90);
  near(filter.wheelAngle(-180, 180), -90);
  near(filter.wheelAngle(180, 180), 0);
  config.leftDiameter = 57;
  assert(filter.configure(config));
  near(filter.wheelAngle(180, 180), 180.0f / 224.0f);
  config = HeadingFusionConfig();
  assert(filter.configure(config));

  // 真の角速度0、残留bias+0.2deg/s。既存補正後の残留だけを学習する。
  for (int sample = 0; sample < 2000; ++sample) {
    assert(filter.step(0.002f, 0, 0, 0.01f, true, true, true));
    const auto &state = filter.state();
    near(state.angle, state.gyroSum + state.biasCorrectionSum +
         state.observationCorrectionSum, 0.001f);
  }
  near(filter.state().bias, 0.2f, 0.015f);
  assert(fabsf(filter.state().angle) < 0.2f);
  assert(filter.state().stationaryUpdates > 0);
  const float bias = filter.state().bias;
  filter.reset(0, true);
  near(filter.state().bias, bias);
  near(filter.state().angle, 0);

  // 実機ログの0/1deg量子化揺れでも静止拘束が成立する。
  HeadingFusion jitter;
  assert(jitter.configure(config));
  for (int sample = 0; sample < 1500; ++sample) {
    const float delta = sample % 2 == 0 ? 1.0f : -1.0f;
    assert(jitter.step(0.000075f, delta, -delta, 0.01f, true, true, true));
  }
  assert(jitter.state().stationaryUpdates > 1400U);
  near(jitter.state().bias, 0.0075f, 0.005f);
  assert(fabsf(jitter.state().angle) < 0.1f);
  assert(jitter.state().stationaryLeftRange <= 2.0f);

  // モータ指令・IMU非静止・欠測のいずれでも即時解除し、確認時間を取り直す。
  uint32_t still_count = jitter.state().stationaryUpdates;
  assert(jitter.step(0, 0, 0, 0.01f, true, true, false));
  assert(jitter.state().stationaryReject == 1U);
  assert(jitter.state().stationaryUpdates == still_count);
  for (int sample = 0; sample < 30; ++sample)
    assert(jitter.step(0, 0, 0, 0.01f, true, true, true));
  assert(jitter.state().stationaryUpdates == still_count);
  assert(jitter.step(0, 0, 0, 0.01f, true, false, true));
  assert(jitter.state().stationaryReject == 2U);
  assert(jitter.step(0, 0, 0, 0.01f, false, true, true));
  assert(jitter.state().stationaryReject == 4U);
  assert(jitter.step(0.01f, 0, 0, 0.01f, true, true, true));
  assert(jitter.state().stationaryReject == 8U);

  // 各周期1deg未満でも、動き続ければ変動幅が上限を超え、静止と認めない。
  assert(jitter.configure(config));
  for (int sample = 0; sample < 200; ++sample) {
    assert(jitter.step(0, 0.1f, 0.1f, 0.01f, true, true, true));
    assert(jitter.state().stationaryUpdates == 0U);
  }
  // 長い更新欠落で確認時間を持ち越さない。
  assert(!jitter.step(0, 0, 0, 1.0f, true, true, true));
  near(jitter.state().stationarySec, 0);

  // 同じ状態を維持して直進相当→右90度→左90度。旋回ごとの再初期化なし。
  for (int sample = 0; sample < 1000; ++sample) {
    assert(filter.step(0.002f, 2, 2, 0.01f));
  }
  const float before = filter.state().angle;
  for (int sample = 0; sample < 200; ++sample) {
    assert(filter.step(0.452f, 0.9f, -0.9f, 0.01f));
  }
  near(filter.state().angle - before, 90, 0.25f);
  for (int sample = 0; sample < 200; ++sample) {
    assert(filter.step(-0.448f, -0.9f, 0.9f, 0.01f));
  }
  near(filter.state().angle, before, 0.35f);

  // 走行中の不一致からbiasを学習しない。静止未校正なら初期値を保持。
  assert(filter.configure(config));
  for (int sample = 0; sample < 12000; ++sample) {
    assert(filter.step(0.102f, 1.2f, 0.8f, 0.01f));
  }
  near(filter.state().bias, 0.0f);

  // 静止で学習したbiasは、左右の旋回倍率不一致・回数の偏りでも変わらない。
  for (int direction = -1; direction <= 1; direction += 2) {
    assert(filter.configure(config));
    for (int sample = 0; sample < 2000; ++sample)
      assert(filter.step(0.002f, 0, 0, 0.01f, true, true, true));
    const float learned = filter.state().bias;
    for (int turn = 0; turn < 30; ++turn) {
      for (int sample = 0; sample < 40; ++sample) {
        assert(filter.step(direction * 83.0f / 40 + 0.002f,
                           direction * 4.5f, -direction * 4.5f, 0.01f));
        near(filter.state().bias, learned);
      }
      for (int sample = 0; sample < 200; ++sample) {
        assert(filter.step(0.002f, 2, 2, 0.01f));
        near(filter.state().bias, learned);
      }
    }
    // 次の静止で残留biasの変化には再追従する。
    for (int sample = 0; sample < 4000; ++sample)
      assert(filter.step(-0.001f, 0, 0, 0.01f, true, true, true));
    near(filter.state().bias, -0.1f, 0.02f);
  }

  // 比較モード: 車輪角度更新のみ止め、静止bias学習とその適用は維持する。
  config.movingWheelObservation = false;
  assert(filter.configure(config));
  for (int sample = 0; sample < 2000; ++sample)
    assert(filter.step(0.002f, 0, 0, 0.01f, true, true, true));
  const float comparison_bias = filter.state().bias;
  filter.reset(0, true);
  for (int sample = 0; sample < 100; ++sample) {
    assert(filter.step(0.902f, 80, -80, 0.01f));
    near(filter.state().observationCorrection, 0);
    near(filter.state().bias, comparison_bias);
  }
  near(filter.state().angle, 90, 0.02f);
  config.movingWheelObservation = true;
  assert(filter.configure(config));

  // 累積角補償: 80deg対90degの差10degへgain0.2なら左右とも2deg。
  for (int direction = -1; direction <= 1; direction += 2) {
    config.turnCorrectionGain = 0.2f;
    assert(filter.configure(config));
    for (int turn = 0; turn < 3; ++turn) {
      const float start_angle = filter.state().angle;
      filter.beginTurn();
      for (int sample = 0; sample < 100; ++sample)
        assert(filter.step(direction * 0.8f, direction * 1.8f,
                           -direction * 1.8f, 0.01f));
      near(filter.state().turnCorrection, direction * 2.0f, 0.002f);
      near(filter.state().angle - start_angle, direction * 82.0f, 0.003f);
      near(filter.state().bias, 0);
      // 同じ角度差を周期ごとに再加算しない。
      for (int sample = 0; sample < 100; ++sample)
        assert(filter.step(0, 0, 0, 0.01f));
      near(filter.state().angle - start_angle, direction * 82.0f, 0.003f);
      filter.endTurn();
      assert(!filter.state().turnActive);
      near(filter.state().turnCorrection, direction * 2.0f, 0.002f);
    }
  }
  // 欠測時は補正を凍結し、ジャイロ予測だけを進める。
  filter.beginTurn();
  assert(filter.step(1, 0, 0, 0.01f, false));
  near(filter.state().turnCorrection, 0);
  near(filter.state().turnGyro, 0);
  assert(filter.step(1, 4, -4, 0.01f));
  near(filter.state().turnCorrection, 0.2f);
  filter.endTurn();
  config.turnCorrectionGain = 0;
  assert(filter.configure(config));
  filter.beginTurn();
  assert(filter.step(1, 4, -4, 0.01f));
  near(filter.state().angle, 1);
  near(filter.state().turnCorrection, 0);
  filter.reset(0, false);
  assert(!filter.state().turnActive);
  config.turnCorrectionGain = 0.2f;
  assert(filter.configure(config));

  // 瞬間的な空転では車輪観測の分散を増やし、その後は自動で戻す。
  filter.reset(0, false);
  assert(filter.step(0, 80, -80, 0.01f));
  assert(filter.state().noiseScale > 1);
  assert(fabsf(filter.state().angle) < 0.2f);
  assert(filter.state().slipUpdates == 1);
  for (int sample = 0; sample < 1000; ++sample) {
    assert(filter.step(0, 0, 0, 0.01f, true, true, true));
  }
  near(filter.state().noiseScale, 1);
  near(filter.state().bias, 0, 0.02f);

  // 静止校正後に、整数カウントと可変周期で直進・左右旋回・静止を繰り返す。
  assert(filter.configure(config));
  for (int sample = 0; sample < 2000; ++sample)
    assert(filter.step(0.002f, 0, 0, 0.01f, true, true, true));
  filter.reset(0, true);
  float left_total = 0, right_total = 0, truth = 0;
  int previous_left = 0, previous_right = 0;
  for (int sample = 0; sample < 12000; ++sample) {
    const float dt = sample % 3 == 0 ? 0.008f : sample % 3 == 1 ? 0.012f : 0.01f;
    const int phase = sample / 1000 % 4;
    const float rate = phase == 0 ? 10.0f : phase == 1 ? -10.0f : 0.0f;
    const float delta = rate * dt;
    const float forward = phase == 3 ? 0 : 200 * dt;
    left_total += forward + 2 * delta;
    right_total += forward - 2 * delta;
    if (sample == 3500) left_total += 80; // 車輪だけ空転した観測
    const int left = static_cast<int>(lroundf(left_total));
    const int right = static_cast<int>(lroundf(right_total));
    const float measured = delta + 0.2f * dt + 0.001f * sinf(static_cast<float>(sample));
    const bool available = sample % 137 != 0;
    assert(filter.step(measured, left - previous_left, right - previous_right,
                         dt, available, phase == 3, phase == 3));
    previous_left = left;
    previous_right = right;
    truth += delta;
    assert(isfinite(filter.state().angle) && isfinite(filter.state().bias));
    assert(filter.state().variance >= 0);
  }
  near(filter.state().bias, 0.2f, 0.04f);
  near(filter.state().angle, truth, 3.0f);
  assert(filter.state().slipUpdates > 0);

  // エンコーダ欠測中もジャイロで進め、正常入力へ復帰できる。
  filter.reset(0, false);
  assert(filter.step(1, NAN, NAN, 0.01f, false));
  near(filter.state().angle, 1);
  assert(filter.step(1, 2, -2, 0.01f));
  near(filter.state().angle, 2);
  const float last = filter.state().angle;
  assert(!filter.step(NAN, 0, 0, 0.01f));
  near(filter.state().angle, last);
  assert(!filter.step(0, 0, 0, 0));
  assert(!filter.step(0, 0, 0, 1));
  config.track = 0;
  assert(!filter.configure(config));
  puts("heading_fusion_test: PASS (calibration, bias, continuous turns, slip recovery, missing data)");
}
