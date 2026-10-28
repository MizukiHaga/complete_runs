#ifndef TESTAPP_HEADING_FUSION_H_
#define TESTAPP_HEADING_FUSION_H_

#include <math.h>
#include <stdint.h>

/* CW正、車輪は前進正。ハードウェアに依存しない角度＋残留biasの2状態KF。 */
struct HeadingFusionConfig {
  float turnCorrectionGain = 0.2f; // 累積旋回角差へ掛ける重み[0,1]
  bool movingWheelObservation = true; // 切り分け用。静止bias学習は維持
  float leftDiameter = 56.0f;
  float rightDiameter = 56.0f;
  float track = 112.0f;
  float gyroNoise = 0.08f;       // deg/sqrt(s)
  float biasWalk = 0.02f;        // (deg/s)/sqrt(s)
  float encoderNoise = 0.15f;    // 車輪から換算した差分角の標準偏差[deg]
  float wheelRateNoise = 2.0f;   // 通常走行時の車輪角速度の標準偏差[deg/s]
  float initialBiasStd = 0.5f;   // deg/s
  float innovationLimit = 3.0f;  // 正規化残差の標準偏差倍率
  float maxNoiseScale = 1000.0f;
  float stationaryEncoderRange = 2.0f; // 停止確認開始からの各車輪の許容変動幅[deg]
  float stationaryHold = 0.5f;   // 静止確認時間[s]
  float stationaryRateNoise = 0.05f; // deg/s
  float stationaryGyroLimit = 0.5f;  // deg/s
};

class HeadingFusion {
public:
  struct State {
    bool turnActive = false;
    float turnGyro = 0.0f;
    float turnWheel = 0.0f;
    float turnCorrection = 0.0f;
    float angle = 0.0f;
    float bias = 0.0f;
    float variance = 0.0f;
    float innovation = 0.0f;
    float noiseScale = 1.0f;
    float encoderDelta = 0.0f;
    float gyroSum = 0.0f;       // reset以降のジャイロ差分積算[deg]
    float biasCorrectionSum = 0.0f; // -bias*dtの積算[deg]
    float observationCorrectionSum = 0.0f; // 車輪/静止観測による角度更新の積算[deg]
    float observationCorrection = 0.0f; // 最新周期の角度更新[deg]
    bool biasLearning = false; // 静止拘束によるbias学習中
    bool imuStationary = false;
    bool motorsStopped = false;
    bool encoderValid = false;
    float gyroRate = 0.0f;
    float stationarySec = 0.0f;
    float stationaryLeftRange = 0.0f;
    float stationaryRightRange = 0.0f;
    uint32_t stationaryReject = 0U; // 1=指令,2=IMU,4=encoder,8=角速度,16=変動幅
    uint32_t updates = 0U;
    uint32_t slipUpdates = 0U;
    uint32_t stationaryUpdates = 0U;
  };

  bool configure(const HeadingFusionConfig &config)
  {
    const float values[] = {config.leftDiameter, config.rightDiameter, config.track,
      config.gyroNoise, config.biasWalk, config.encoderNoise, config.wheelRateNoise,
      config.initialBiasStd, config.innovationLimit, config.maxNoiseScale,
      config.stationaryEncoderRange, config.stationaryHold, config.stationaryRateNoise, config.stationaryGyroLimit};
    for (float value : values) {
      if (!isfinite(value) || value <= 0.0f) return false;
    }
    if (config.maxNoiseScale < 1.0f || !isfinite(config.turnCorrectionGain) ||
        config.turnCorrectionGain < 0.0f || config.turnCorrectionGain > 1.0f) return false;
    config_ = config;
    reset(0.0f, false);
    return true;
  }

  /* 方位の原点変更では、学習済みbiasとその分散を維持できる。 */
  void reset(float angle, bool keep_bias = true)
  {
    const float bias = keep_bias ? state_.bias : 0.0f;
    const float bias_variance = keep_bias ? p11_
                              : config_.initialBiasStd * config_.initialBiasStd;
    state_ = State();
    state_.angle = angle;
    state_.bias = bias;
    p00_ = p01_ = 0.0f;
    p11_ = bias_variance;
    invalidateStationary();
  }

  float wheelAngle(float left_deg, float right_deg) const
  {
    return (config_.leftDiameter * left_deg - config_.rightDiameter * right_deg) /
            (2 * config_.track);
  }

  /* 同じ10ms観測列を基準に旋回角差を積算する。既適用補正は方位に残す。 */
  void beginTurn()
  {
    state_.turnActive = true;
    state_.turnGyro = state_.turnWheel = state_.turnCorrection = 0.0f;
    invalidateStationary();
  }
  void endTurn() { state_.turnActive = false; }

  /* 欠測・時刻飛び・原点変更をまたいで静止確認時間を持ち越さない。 */
  void invalidateStationary()
  {
    stationary_window_ = false;
    stationary_sec_ = 0.0f;
    state_.stationarySec = 0.0f;
    state_.biasLearning = false;
  }

  /* Gは校正済み1Dジャイロの差分角。biasはドライバ補正後に残る角速度偏り。 */
  bool step(float gyro_delta, float left_delta, float right_delta, float dt,
            bool encoder_valid = true, bool stationary = false, bool motors_stopped = false)
  {
    if (!isfinite(gyro_delta) || !isfinite(dt) || dt <= 0.0f || dt > 0.2f) {
      invalidateStationary();
      return false;
    }
    encoder_valid = encoder_valid && isfinite(left_delta) && isfinite(right_delta);
    const float old01 = p01_;
    const float old11 = p11_;
    const float dt2 = dt * dt;
    const float qb = config_.biasWalk * config_.biasWalk;
    const float q00 = config_.gyroNoise * config_.gyroNoise * dt + qb * dt2 * dt / 3;
    const float q01 = -qb * dt2 / 2;
    const float prediction = gyro_delta - state_.bias * dt;
    state_.gyroSum += gyro_delta;
    state_.biasCorrectionSum -= state_.bias * dt;
    state_.observationCorrection = 0.0f;
    state_.angle += prediction;
    p00_ += -2 * dt * old01 + dt2 * old11 + q00;
    p01_ = old01 - dt * old11 + q01;
    p11_ += qb * dt;
    state_.noiseScale = 1.0f;
    state_.innovation = 0.0f;
    state_.encoderDelta = 0.0f;

    /* 静止が継続した区間は、車輪観測の代わりに既知の角度増分0を使う。
     * 同じジャイロ値を独立なbias観測として二重に取り込まない。 */
    state_.imuStationary = stationary;
    state_.motorsStopped = motors_stopped;
    state_.encoderValid = encoder_valid;
    state_.gyroRate = gyro_delta / dt;
    state_.stationaryReject = (!motors_stopped ? 1U : 0U) |
        (!stationary ? 2U : 0U) | (!encoder_valid ? 4U : 0U) |
        (fabsf(state_.gyroRate) >= config_.stationaryGyroLimit ? 8U : 0U);
    if (state_.stationaryReject != 0U) {
      invalidateStationary();
      state_.stationaryLeftRange = state_.stationaryRightRange = 0.0f;
    } else {
      if (!stationary_window_) {
        stationary_window_ = true;
        stationary_left_ = stationary_right_ = 0.0f;
        stationary_left_min_ = stationary_left_max_ = 0.0f;
        stationary_right_min_ = stationary_right_max_ = 0.0f;
      }
      stationary_left_ += left_delta;
      stationary_right_ += right_delta;
      stationary_left_min_ = fminf(stationary_left_min_, stationary_left_);
      stationary_left_max_ = fmaxf(stationary_left_max_, stationary_left_);
      stationary_right_min_ = fminf(stationary_right_min_, stationary_right_);
      stationary_right_max_ = fmaxf(stationary_right_max_, stationary_right_);
      state_.stationaryLeftRange = stationary_left_max_ - stationary_left_min_;
      state_.stationaryRightRange = stationary_right_max_ - stationary_right_min_;
      if (state_.stationaryLeftRange > config_.stationaryEncoderRange ||
          state_.stationaryRightRange > config_.stationaryEncoderRange) {
        state_.stationaryReject = 16U;
        invalidateStationary();
      } else {
        stationary_sec_ += dt;
      }
    }
    state_.stationarySec = stationary_sec_;
    const bool still = !state_.turnActive && stationary_sec_ >= config_.stationaryHold;
    state_.biasLearning = still;

    if (!state_.turnActive && encoder_valid && (still || config_.movingWheelObservation)) {
      const float encoder_delta = still ? 0.0f : wheelAngle(left_delta, right_delta);
      const float innovation = encoder_delta - prediction;
      const float base_r = still
          ? config_.stationaryRateNoise * config_.stationaryRateNoise * dt2
          : config_.encoderNoise * config_.encoderNoise +
            config_.wheelRateNoise * config_.wheelRateNoise * dt2;
      /* 観測はθ今回−θ前回。前回θとの相関を引き、積算した車輪角を
       * 毎周期独立な絶対方位として扱わない（前回θを一時的にcloneした式）。 */
      const float c0 = -dt * old01 + dt2 * old11 + q00;
      const float c1 = -dt * old11 + q01;
      const float increment_variance = dt2 * old11 + q00;
      const float ratio = innovation * innovation /
          ((increment_variance + base_r) * config_.innovationLimit * config_.innovationLimit);
      state_.noiseScale = still ? 1.0f :
          fminf(config_.maxNoiseScale, fmaxf(1.0f, ratio));
      const float s = increment_variance + base_r * state_.noiseScale;
      state_.observationCorrection = c0 / s * innovation;
      state_.observationCorrectionSum += state_.observationCorrection;
      state_.angle += state_.observationCorrection;
      /* 車輪倍率・滑りの誤差を走行中のbiasへ学習しない。
       * 走行時はbiasゲインを0にした部分更新。角度と交差共分散は更新し、
       * bias分散は予測で増えた値を維持する（観測で縮めない）。 */
      if (still) {
        state_.bias += c1 / s * innovation;
        p11_ -= c1 * c1 / s;
      }
      p00_ -= c0 * c0 / s;
      p01_ -= c0 * c1 / s;
      state_.innovation = innovation;
      state_.encoderDelta = encoder_delta;
      if (state_.noiseScale > 1.0f) ++state_.slipUpdates;
      if (still) ++state_.stationaryUpdates;
    }

    if (state_.turnActive && encoder_valid) {
      // 欠測区間は両センサとも累積対象外にして、差の片側だけを増やさない。
      state_.turnGyro += gyro_delta;
      state_.turnWheel += wheelAngle(left_delta, right_delta);
      const float target = config_.turnCorrectionGain *
                           (state_.turnWheel - state_.turnGyro);
      const float change = target - state_.turnCorrection;
      state_.angle += change;
      state_.observationCorrection = change;
      state_.observationCorrectionSum += change;
      state_.encoderDelta = wheelAngle(left_delta, right_delta);
      state_.innovation = state_.turnWheel - state_.turnGyro;
      state_.turnCorrection = target;
      // 固定ゲインの校正補償。独立な絶対方位観測として共分散を縮めない。
    }

    p00_ = fmaxf(p00_, 0.0f);
    p11_ = fmaxf(p11_, 0.0f);
    state_.variance = p00_;
    ++state_.updates;
    return isfinite(state_.angle) && isfinite(state_.bias);
  }

  const State &state() const { return state_; }

private:
  HeadingFusionConfig config_;
  State state_;
  float p00_ = 0.0f;
  float p01_ = 0.0f;
  float p11_ = 0.25f;
  float stationary_sec_ = 0.0f;
  bool stationary_window_ = false;
  float stationary_left_ = 0.0f, stationary_right_ = 0.0f;
  float stationary_left_min_ = 0.0f, stationary_left_max_ = 0.0f;
  float stationary_right_min_ = 0.0f, stationary_right_max_ = 0.0f;
};

#endif // TESTAPP_HEADING_FUSION_H_
