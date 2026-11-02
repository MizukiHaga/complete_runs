#ifndef NANSHO3_KYOKU_H
#define NANSHO3_KYOKU_H

#include "Path.h"
#include "Settings.h"  // 走行パワー・速度・寸法・ゲート配置の調整値

class Nansho3KYOKUScene {
public:
  /* 初期進入 → 全経路生成 → Pure Pursuit → 停止。 */
  void run();

private:
  /* Hubに表示する実行結果。P=経路計算中 / R=追従中は別途表示する。 */
  enum class StopReason : char {
    Complete       = 'D',
    PlanFailed     = 'E',
    PlanTimeout    = 'Q',
    TrackingFailed = 'F',
    CollisionRisk  = 'C',
    GyroJump       = 'G',
    EncoderJump    = 'N',
    PeriodError    = 'T',
    Stalled        = 'S',
    Timeout        = 'X',
  };

  /* Nansho3の初期動作を共用し、直進完了地点をStartとして扱う。 */
  void runInitialApproach();
  bool planRallyPath(const kyoku::Config &config);
  StopReason followRallyPath(const kyoku::Config &config);

  /* 現在姿勢と左右速度で短時間先を予測し、接触する指令を止める。 */
  bool isPredictedMotionSafe(kyoku::Pose pose, float left_speed,
                             float right_speed);
  void finishRun(StopReason reason);

  /* 経路は約96KiB。タスクのローカル変数ではなく、静的Sceneのメンバで保持。 */
  kyoku::Path path_;
  kyoku::MotionRisk collision_risk_ = kyoku::MotionRisk::Safe;
};

#endif // NANSHO3_KYOKU_H
