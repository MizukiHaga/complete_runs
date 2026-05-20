//#ifdef __cplusplus
//extern "C" {
//#endif

/* 下記の項目は各ロボットに合わせて変えること */

/* カラーセンサの輝度設定 */
#define WHITE_BRIGHTNESS  (80)		// 80  80
#define BLACK_BRIGHTNESS  (10) 		// 10  10

/* PIDゲイン(デフォルト値) */
#define PID_DEFAULT_KP    (0.1F)
#define PID_DEFAULT_KI    (0.0005F)
#define PID_DEFAULT_KD    (0.45F)
#define PID_OUTPUT_MAX    (100.0F)

/* 積分項の上限（風袋り防止） */
#define PID_I_MAX         (100.0F)

/* 左右モータの直進ベースパワー（機械差・曲がり癖の補正で値を変える） */
#define LEFT_BASE_SPEED   (60)
#define RIGHT_BASE_SPEED  (60)

/* ライントレースエッジ切り替え */
#define LEFT_EDGE         (1) 
#define RIGHT_EDGE        (1) 

#include <pbio/port.h> 

  extern void LineTracer_Configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port, pbio_port_id_t color_sensor_port);
  
//#ifdef __cplusplus
//}
//#endif
