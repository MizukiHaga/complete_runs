#ifndef TESTAPP_HEADING_FUSION_LOG_H_
#define TESTAPP_HEADING_FUSION_LOG_H_

#include "HeadingFusion.h"
#include <stdio.h>

/* 停止後の診断用。周期推定タスクからは呼ばない。 */
inline void writeHeadingFusionLog(FILE *output, const HeadingFusion::State &state,
                                   float heading)
{
  if (output == nullptr) return;
  fprintf(output,
      "KF_h,%.3f,KF_bias,%.5f,KF_var,%.5f,KF_innov,%.4f,KF_noise,%.2f,"
      "KF_enc_delta,%.4f,KF_count,%lu,KF_slip,%lu,KF_still,%lu,"
      "KF_gyro_sum,%.4f,KF_bias_sum,%.4f,KF_obs_sum,%.4f,KF_obs,%.5f,"
      "ST_imu,%d,ST_cmd,%d,ST_enc,%d,ST_rate,%.4f,ST_sec,%.3f,"
      "ST_Lrange,%.1f,ST_Rrange,%.1f,ST_reject,%lu,KF_bias_learn,%d,TURN_active,%d,TURN_gyro,%.4f,TURN_wheel,%.4f,TURN_corr,%.4f\n",
      (double)heading, (double)state.bias, (double)state.variance,
      (double)state.innovation, (double)state.noiseScale, (double)state.encoderDelta,
      (unsigned long)state.updates, (unsigned long)state.slipUpdates,
      (unsigned long)state.stationaryUpdates,
      (double)state.gyroSum, (double)state.biasCorrectionSum,
      (double)state.observationCorrectionSum, (double)state.observationCorrection,
      state.imuStationary ? 1 : 0, state.motorsStopped ? 1 : 0,
      state.encoderValid ? 1 : 0, (double)state.gyroRate,
      (double)state.stationarySec, (double)state.stationaryLeftRange,
      (double)state.stationaryRightRange, (unsigned long)state.stationaryReject, state.biasLearning ? 1 : 0,
      state.turnActive ? 1 : 0, (double)state.turnGyro,
      (double)state.turnWheel, (double)state.turnCorrection);
}

/* 旋回中はメモリへ保存するだけ。通信は旋回終了後にまとめて行う。 */
template <typename Snapshot>
inline void writeTurnEncoderLog(FILE *output, const Snapshot &snapshot)
{
  if (output == nullptr) return;
  if (!snapshot.valid) {
    fprintf(output, "ENC180_NOT_REACHED\n");
    return;
  }
  fprintf(output,
      "ENC180_ms,%lu,L,%.0f,R,%.0f,AVG,%.1f,IMU_delta,%.3f,"
      "KF_h,%.3f,KF_delta,%.3f,WHEEL_delta,%.3f,"
      "BIAS_corr,%.5f,OBS_corr,%.5f,KF_bias,%.5f,KF_noise,%.2f,KF_count,%lu,TURN_gyro,%.4f,TURN_wheel,%.4f,TURN_corr,%.4f\n",
      (unsigned long)snapshot.elapsed_ms,
      (double)snapshot.left, (double)snapshot.right,
      (double)(0.5f * (fabsf(snapshot.left) + fabsf(snapshot.right))),
      (double)snapshot.gyro_delta, (double)snapshot.heading,
      (double)snapshot.heading_delta, (double)snapshot.wheel_delta,
      (double)snapshot.bias_correction, (double)snapshot.observation_correction,
      (double)snapshot.fusion.bias, (double)snapshot.fusion.noiseScale,
      (unsigned long)snapshot.fusion.updates,
      (double)snapshot.fusion.turnGyro, (double)snapshot.fusion.turnWheel,
      (double)snapshot.fusion.turnCorrection);
}

#endif // TESTAPP_HEADING_FUSION_LOG_H_
