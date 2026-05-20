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
#define PID_DEFAULT_KP    (0.1F)
#define PID_DEFAULT_KI    (0.0005F)
#define PID_DEFAULT_KD    (0.45F)
#define PID_OUTPUT_MAX    (100.0F)  // PID 合計出力の絶対値上限

/* 積分項の暴れ防止クランプ（風袋り = wind-up 防止）。
 * これを超えて積分が伸びないようにすると、急ハンドル時の戻りが速くなる。 */
#define PID_I_MAX         (100.0F)

/* 左右モータの直進ベースパワー。機械差・曲がり癖をここで吸収。 */
#define LEFT_BASE_SPEED   (60)
#define RIGHT_BASE_SPEED  (60)

/* 走行エッジの切替（LEFT_EDGE / RIGHT_EDGE = 1 のとき有効）。
 *  - LEFT_EDGE  = 1 : 黒線の左側エッジを追従
 *  - RIGHT_EDGE = 1 : 黒線の右側エッジを追従（符号を反転して使う）
 * 両方 1 にすると左エッジが優先される (motor_drive_control の式参照)。 */
#define LEFT_EDGE         (1)
#define RIGHT_EDGE        (1)

#include <pbio/port.h>

/* 初期化: 左右モータとカラーセンサのデバイスを取得し、回転方向を設定する。 */
extern void LineTracer_Configure(pbio_port_id_t left_motor_port,
                                 pbio_port_id_t right_motor_port,
                                 pbio_port_id_t color_sensor_port);
