/*
 * LineTracer モジュール ヘッダ
 *  反射値ベースの PID ライントレースを行う共通モジュール。
 *  pup_motor_* / pup_color_sensor_* の C API はこのクラス内だけで使い、
 *  外からは configure / step / setBaseSpeed / setPid で操作する。
 *
 *  使い方:
 *    LineTracer gLineTracer;
 *    gLineTracer.configure(PBIO_PORT_ID_B, PBIO_PORT_ID_A, PBIO_PORT_ID_E);
 *    gLineTracer.setBaseSpeed(50.0f, 50.0f);
 *    gLineTracer.setPid(0.15f, 0.0005f, 0.45f, 100.0f);
 *    gLineTracer.step();   // 1 周期分の制御
 *
 *  周期タスクから使う場合は app.cpp の tracer_task() が gLineTracer.step() を呼ぶ。
 */

#ifndef TESTAPP_LINETRACER_H_
#define TESTAPP_LINETRACER_H_

#include <pbio/port.h>
#include <stdint.h>

extern "C" {
#include <spike/pup/motor.h>
#include <spike/pup/colorsensor.h>
}

class LineTracer {
public:
  /* === 各ロボットに合わせて値を調整するパラメータ ============== */

  /* カラーセンサの輝度設定（白/黒の代表値、目標輝度の計算に使う）。 */
  static constexpr uint16_t WhiteBrightness = 80;
  static constexpr uint16_t BlackBrightness = 10;

  /* PID ゲインの初期値。 */
  static constexpr float DefaultKp     = 0.15f;
  static constexpr float DefaultKi     = 0.0005f;
  static constexpr float DefaultKd     = 0.45f;
  static constexpr float DefaultOutMax = 100.0f;

  /* 積分項の暴れ防止クランプ（ワインドアップ防止）。 */
  static constexpr float IntegralMax = 100.0f;

  /* 左右モータの直進ベースパワー。 */
  static constexpr int LeftBaseSpeed  = 50;
  static constexpr int RightBaseSpeed = 50;

  /* 走行エッジの初期値 (1=左エッジ追従)。実行時は setEdgeFollow() で変更可。 */
  static constexpr int LeftEdge  = 1;
  static constexpr int RightEdge = 0;

  /* === API ================================================ */

  /* 初期化: 左右モータとカラーセンサのデバイスを取得し、回転方向を設定する。 */
  void configure(pbio_port_id_t left_motor_port,
                 pbio_port_id_t right_motor_port,
                 pbio_port_id_t color_sensor_port);

  /* ライントレースを 1 ステップだけ実行する (ext_tsk しない)。
   * 周期タスク以外のタスク内で、時間条件付きループから呼び出す。 */
  void step();

  /* 左右モータのベースパワーを実行時に変更する。値は [-100,100] にクランプ。 */
  void setBaseSpeed(float left_power, float right_power);

  /* PID ゲインと出力上限を実行時に変更する。設定時に積分項・前回偏差をリセット。 */
  void setPid(float kp, float ki, float kd, float out_max);

  /* 追従エッジを切り替える。true=左エッジ、false=右エッジ。 */
  void setEdgeFollow(bool follow_left_edge);

  /* false のとき step() は何もしない（Straight 制御中の周期タスク干渉防止）。 */
  void setEnabled(bool enabled) { enabled_ = enabled; }
  bool isEnabled() const { return enabled_; }

private:
  static int   clampMotorPower(int power);
  static float clampFloat(float value, float min_value, float max_value);

  int16_t steeringAmountCalculation();
  void    motorDriveControl(int16_t steering_amount);

  pup_motor_t  *left_         = nullptr;
  pup_motor_t  *right_        = nullptr;
  pup_device_t *color_sensor_ = nullptr;

  float integral_   = 0.0f;
  float prev_error_ = 0.0f;
  float kp_         = DefaultKp;
  float ki_         = DefaultKi;
  float kd_         = DefaultKd;
  float out_max_    = DefaultOutMax;

  float left_base_speed_  = LeftBaseSpeed;
  float right_base_speed_ = RightBaseSpeed;
  bool  follow_left_edge_ = (LeftEdge != 0);
  bool  enabled_          = true;
};

#endif  // TESTAPP_LINETRACER_H_
