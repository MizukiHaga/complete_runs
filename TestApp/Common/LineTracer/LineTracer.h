/*
 * LineTracer モジュール ヘッダ
 *  反射値ベースの PID ライントレースを行う共通モジュール。
 *  app.cfg の周期タスク TRACER_TASK から tracer_task() が呼ばれる前提。
 */

/* === 各ロボットに合わせて値を調整するパラメータ ============== */

/* カラーセンサの輝度設定（白/黒の代表値、目標輝度の計算に使う）。
 * コース上で実測した値に合わせて変更すること。 */
#define WHITE_BRIGHTNESS  (80)    // 80
#define BLACK_BRIGHTNESS  (10)    // 10

/* PID ゲインの初期値。
 * Kp: 大きいほど反応が鋭くなるが、振動が出やすい。
 * Ki: 累積誤差で定常偏差を消す。大きすぎると暴れる。
 * Kd: 振動を抑える。ノイズに弱いので大きくしすぎない。 */
#define PID_DEFAULT_KP    (0.07F)
#define PID_DEFAULT_KI    (0.0004F)
#define PID_DEFAULT_KD    (0.40F)
#define PID_OUTPUT_MAX    (100.0F)  // PID 合計出力の絶対値上限

/* 積分項の暴れ防止クランプ（風袋り = wind-up 防止）。
 * これを超えて積分が伸びないようにすると、急ハンドル時の戻りが速くなる。 */
#define PID_I_MAX         (100.0F)

/* 左右モータの直進ベースパワー。機械差・曲がり癖をここで吸収。 */
#define LEFT_BASE_SPEED   (80)
#define RIGHT_BASE_SPEED  (80)

/* 走行エッジの切替（LEFT_EDGE / RIGHT_EDGE = 1 のとき有効）。
 *  - LEFT_EDGE  = 1 : 黒線の左側エッジを追従
 *  - RIGHT_EDGE = 1 : 黒線の右側エッジを追従（符号を反転して使う）
 * 両方 1 にすると左エッジが優先される (motor_drive_control の式参照)。 */
#define LEFT_EDGE         (1)
#define RIGHT_EDGE        (1)

/* 操舵量のスケール。振れ幅は小さいが曲がれない場合はここを大きくする（例: 1.0F -> 1.5F） */

#define STEERING_SCALE    (4.5F)
//  *  pup_motor_* / pup_color_sensor_* の C API はこのクラス内だけで使い、
//  *  外からは configure / step / setBaseSpeed / setPid で操作する。
//  *
//  *  使い方:
//  *    LineTracer gLineTracer;
//  *    gLineTracer.configure(PBIO_PORT_ID_B, PBIO_PORT_ID_A, PBIO_PORT_ID_E);
//  *    gLineTracer.setBaseSpeed(50.0f, 50.0f);
//  *    gLineTracer.setPid(0.15f, 0.0005f, 0.45f, 100.0f);
//  *    gLineTracer.step();   // 1 周期分の制御
//  *
//  *  周期タスクから使う場合は app.cpp の tracer_task() が gLineTracer.step() を呼ぶ。
//  */

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

  /* 走行エッジ (1=有効)。LeftEdge=1 で左エッジ追従。 */
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

  void setScale(float scale);

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
  float smoothed_ref_ = 0.0f;
  float prev_diff_filtered_ = 0.0f;
  bool  initialized_ = false;
  float kp_         = DefaultKp;
  float ki_         = DefaultKi;
  float kd_         = DefaultKd;
  float out_max_    = DefaultOutMax;
  float steering_scale_ = STEERING_SCALE;

  float left_base_speed_  = LeftBaseSpeed;
  float right_base_speed_ = RightBaseSpeed;
};

#endif  // TESTAPP_LINETRACER_H_
