#include "Sumo.h"

#include "app.h"
#include "kernel_cfg.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "Common/Straight/Straight.h"
#include "Common/LineTracer/LineTracer.h"
#include "Common/Sonic/Sonic.h"
#include "Common/Gyro/Gyro.h"
#include "Common/Arm/Arm.h"

extern Sonic      gSonic;
extern ColorRGB   gColorRGB;
extern ColorLAB   gColorLAB;
extern Gyro       gGyro;
extern Straight   gMotors;
extern LineTracer gLineTracer;
extern Arm        gArm;
extern SumoScene  gSumo;

void SumoScene::run(){
  // LR切り替え(Rがデフォでfalse、Lにしたい時はtrue)
  set_sumo_reverse(false);
  
  // ジャイロリセット
  gLineTracer.setEnabled(false);
  gMotors.stop();
  gGyro.reset(0.0f);
  dly_tsk(100 * 1000U);

  // 直進・旋回ベース変数
  const int base_chokushin_power = 40;
  const int base_senkai_power = 32;
  const float base_tolerance_deg = 1.0f;

  // ボトル検知しきい値
  const int32_t bottle_detect_dist_wiper = 100;
  const int32_t bottle_detect_dist_spin = 200;

  // ライントレースで土俵横まで直進
  int linetrace_dist_for_dohyo = 660;
  int linetrace_power_for_dohyo = 80;
  float target_angle_for_dohyo = -90.0f;
  drive_linetrace_distance(linetrace_dist_for_dohyo, linetrace_power_for_dohyo,invert_edge(true));
  drive_senkai(invert_angle(target_angle_for_dohyo), base_senkai_power, -base_senkai_power, base_tolerance_deg, invert_ccw(true));
  gGyro.reset(0.0f);
  
  // 土俵内に侵入
  int chokushin_dist_for_wiper = 150;
  int chokushin_dist_for_wiper_next = 100;
  float target_angle_wiper = -30.0f;
  int wiper_power = 32;
  int chokushin_dist_for_dohyo = 350 - (chokushin_dist_for_wiper + chokushin_dist_for_wiper_next);
  drive_chokushin_distance(chokushin_dist_for_wiper, base_chokushin_power, 0.0f);

  // ボトル検知で横に押し出し
  if (gSonic.isCloserThan(bottle_detect_dist_wiper)){
    // 旋回しながら押し出し
    int senkai_power_wiperdetect_outer = 60;
    int senkai_power_wiperdetect_inner = 35;
    float target_angle_wiperdetect = -150.0f; 
    drive_senkai(invert_angle(target_angle_wiperdetect), senkai_power_wiperdetect_outer, senkai_power_wiperdetect_inner, base_tolerance_deg, invert_ccw(true));

    // 斜めに黒ラインへ向かう
    drive_chokushin_until_black(base_chokushin_power, invert_angle(target_angle_wiperdetect));
    drive_chokushin_distance(20, base_chokushin_power, invert_angle(target_angle_wiperdetect));
    gArm.raise(50,180,1000);
    drive_senkai(invert_angle(-90.0f), base_senkai_power, -base_senkai_power, base_tolerance_deg, invert_ccw(false));
    gArm.lower(-50, 1000, 0);

    // ゲートまでライントレース
    int linetrace_dist_wiperdetect = 100;
    int linetrace_power_wiperdetect = 40;
    drive_linetrace_distance(linetrace_dist_wiperdetect, linetrace_power_wiperdetect, invert_edge(true));

    gMotors.stop();
    return;
  }

  // ワイパー
  drive_senkai(invert_angle(target_angle_wiper), wiper_power, -wiper_power, base_tolerance_deg, invert_ccw(true));
  drive_senkai(invert_angle(-target_angle_wiper), wiper_power, -wiper_power, base_tolerance_deg, invert_ccw(false));
  drive_senkai(0.0f, wiper_power, -wiper_power, base_tolerance_deg, invert_ccw(true));
  drive_chokushin_distance(chokushin_dist_for_wiper_next, base_chokushin_power, 0.0f);
  drive_senkai(invert_angle(target_angle_wiper), wiper_power, -wiper_power, base_tolerance_deg, invert_ccw(true));
  drive_senkai(invert_angle(-target_angle_wiper), wiper_power, -wiper_power, base_tolerance_deg, invert_ccw(false));
  drive_senkai(0.0f, wiper_power, -wiper_power, base_tolerance_deg, invert_ccw(true));
  // 土俵中央に移動
  drive_chokushin_distance(chokushin_dist_for_dohyo, base_chokushin_power, 0.0f);
  dly_tsk(250*1000);

  // 目の前にボトルがあったら押し出して終了
  if (gSonic.isCloserThan(bottle_detect_dist_spin)){
    // 土俵外まで押し出し
    int chokushin_dist_dohyodetect = 150;
    drive_chokushin_distance(chokushin_dist_dohyodetect, base_chokushin_power, 0.0f);

    // 斜めに黒ラインまで直進
    float target_angle_dohyodetect = -135.0f;
    gArm.raise(50,180,1000);
    drive_senkai(invert_angle(target_angle_dohyodetect), base_senkai_power, -base_senkai_power, base_tolerance_deg, invert_ccw(true));
    gArm.lower(-50, 1000, 0);
    drive_chokushin_until_black(base_chokushin_power, invert_angle(target_angle_dohyodetect));
    drive_chokushin_distance(20, base_chokushin_power, invert_angle(target_angle_dohyodetect));
    drive_senkai(invert_angle(-90.0f), base_senkai_power, -base_senkai_power, base_tolerance_deg, invert_ccw(false));

    // ゲートまでライントレース
    int linetrace_dist_dohyodetect = 200;
    int linetrace_power_dohyodetect = 40;
    drive_linetrace_distance(linetrace_dist_dohyodetect, linetrace_power_dohyodetect, invert_ccw(true));

    gMotors.stop();
    return;
  }

  // 刈り取り走行
  // 最初に中央で半回転
  int spin_power = 38;
  drive_senkai(180.0f, spin_power, -spin_power, base_tolerance_deg, invert_ccw(true));
  dly_tsk(200*1000);

  // 前進&Uターンで土俵の右側に
  int chokushin_dist_after_spin = 285;
  int senkai_power_after_spin = 42;
  drive_chokushin_distance(chokushin_dist_after_spin, base_chokushin_power, 180.0f);
  drive_senkai(0.0f, senkai_power_after_spin, 0, base_tolerance_deg, invert_ccw(true));
  drive_chokushin_distance(chokushin_dist_after_spin, base_chokushin_power, 0.0f);
  dly_tsk(100*1000);

  // 外周を走行
  int gaishu_power_outer = 71;
  int gaishu_power_inner = 30;
  float target_angle_gaishu = 180.0f;
  drive_senkai(target_angle_gaishu, gaishu_power_outer, gaishu_power_inner, base_tolerance_deg, invert_ccw(true));
  dly_tsk(100*1000);

  // 黒ラインまで直進&アームアップでリリース
  float target_angle_after_gaishu = -90.0f;
  drive_chokushin_until_black(base_chokushin_power, target_angle_gaishu);
  drive_chokushin_distance(30, base_chokushin_power, target_angle_gaishu);
  dly_tsk(200*1000);
  gArm.raise(50,180,1000);
  drive_senkai(invert_angle(target_angle_after_gaishu), base_senkai_power, -base_senkai_power, base_tolerance_deg, invert_ccw(false));
  gArm.lower(-50, 1000, 0);

  // ゲートまでライントレース
  int linetrace_dist_for_gate = 400;
  int linetrace_dist_for_gate_slow = 150;
  int linetrace_power_for_gate = 80;
  int linetrace_power_for_gate_slow = 40;
  drive_linetrace_distance(linetrace_dist_for_gate_slow, linetrace_power_for_gate_slow, invert_edge(true));
  drive_linetrace_distance(linetrace_dist_for_gate, linetrace_power_for_gate, invert_edge(true));
  gMotors.stop();
}

extern "C" void SumoTask(intptr_t exinf){
  (void)exinf;
  gSumo.run();
  /* Nansho2 以降は既定どおりヨー角 PI 補正を使う */
  gMotors.setHeadingCorrection(true);
  ext_tsk();
}

// 直進メソッド
void SumoScene::drive_chokushin_distance(int32_t target_distance_mm, int base_power, float target_heading, float Kp){
  constexpr float WheelDiameterMm = 56.0f;
  constexpr float M_PI_F = 3.14159265f;
  constexpr float MmPerDegree = (WheelDiameterMm * M_PI_F) / 360.0f; // 1度あたりの進む距離(mm)

  // 1. 移動距離の絶対値から目標角度(度)を算出
  const int32_t abs_distance_mm = (target_distance_mm >= 0) ? target_distance_mm : -target_distance_mm;
  const int32_t target_deg = static_cast<int32_t>(abs_distance_mm / MmPerDegree);

  // 前進かバックかの方向判定 (距離またはパワーの符号で判断)
  const int drive_dir = (target_distance_mm < 0 || base_power < 0) ? -1 : 1;
  const int abs_base_power = (base_power >= 0) ? base_power : -base_power;

  /* 2. エンコーダの初期値（開始位置）を取得 */
  const int32_t start_left  = gMotors.leftEncoderCount();
  const int32_t start_right = gMotors.rightEncoderCount();

  const uint32_t step_ms = 10U;

  // パワーから計算される目標スピード (10msあたりの変化量)
  const float target_deg_per_step = static_cast<float>(abs_base_power) * 0.15f; 
  const float K_speed = 0.5f;

  int current_left_power  = abs_base_power;
  int current_right_power = abs_base_power;

  int32_t prev_left  = start_left;
  int32_t prev_right = start_right;

  while (true) {
    /* 3. 走行距離の判定（絶対値で進んだ角度を判定） */
    int32_t current_left  = gMotors.leftEncoderCount();
    int32_t current_right = gMotors.rightEncoderCount();

    int32_t total_left_diff  = current_left - start_left;
    int32_t total_right_diff = current_right - start_right;
    
    // 左右エンコーダの絶対値平均（バック走行でも正の値になるように labs で囲む）
    int32_t avg_deg = (labs(total_left_diff) + labs(total_right_diff)) / 2;

    if (avg_deg >= target_deg) {
      break; // 目標距離に到達したら終了
    }

    /* 4. 速度（進行角/10ms）計算と速度一定補正 */
    float speed_left  = static_cast<float>(labs(current_left - prev_left));
    float speed_right = static_cast<float>(labs(current_right - prev_right));
    prev_left  = current_left;
    prev_right = current_right;

    current_left_power  += static_cast<int>((target_deg_per_step - speed_left) * K_speed);
    current_right_power += static_cast<int>((target_deg_per_step - speed_right) * K_speed);

    /* 5. ジャイロ P 制御による角度補正 */
    float current_h = gGyro.heading();
    float error     = Gyro::wrapDeg180(target_heading - current_h);
    int turn_adjust = static_cast<int>(error * Kp);

    // 前進時とバック時でモータに与えるパワーの符号(drive_dir)を適用
    // バック時(drive_dir = -1)は P 制御の舵角の効き方が逆になるため turn_adjust の符号を調整
    int final_left_power  = (current_left_power  + turn_adjust) * drive_dir;
    int final_right_power = (current_right_power - turn_adjust) * drive_dir;

    // 出力範囲制限 (-100 〜 100)
    if (final_left_power > 100)   final_left_power = 100;
    if (final_left_power < -100)  final_left_power = -100;
    if (final_right_power > 100)  final_right_power = 100;
    if (final_right_power < -100) final_right_power = -100;

    gMotors.run(final_left_power, final_right_power);

    dly_tsk(step_ms * 1000U);
  }

  gMotors.stop();
}

// 旋回メソッド
void SumoScene::drive_senkai(float target_angle, int speed_base, int speed_low, float tolerance_deg, bool is_counterclock){
  const uint32_t step_ms = 10U; // 旋回ループ周期を 10ms に設定

  int cmd_left  = is_counterclock ? speed_low  : speed_base;
  int cmd_right = is_counterclock ? speed_base : speed_low;

  // 目標スピードの推定 (10msあたりの変化量)
  const float target_speed_left  = static_cast<float>(labs(cmd_left)) * 0.15f;
  const float target_speed_right = static_cast<float>(labs(cmd_right)) * 0.15f;
  const float K_speed = 0.5f;

  int current_left_power  = cmd_left;
  int current_right_power = cmd_right;

  int32_t prev_left  = gMotors.leftEncoderCount();
  int32_t prev_right = gMotors.rightEncoderCount();

  while (true) {
    // 1. 目標角度に到達判定
    float current_h = gGyro.heading();
    float delta = Gyro::wrapDeg180(target_angle - current_h);
    if (fabsf(delta) <= tolerance_deg) {
      break; // 目標角度に到達したら終了
    }

    // 2. モーター速度の監視と負荷補正
    int32_t cur_left  = gMotors.leftEncoderCount();
    int32_t cur_right = gMotors.rightEncoderCount();

    float speed_left  = static_cast<float>(labs(cur_left - prev_left));
    float speed_right = static_cast<float>(labs(cur_right - prev_right));
    prev_left  = cur_left;
    prev_right = cur_right;

    // 左ホイールのスピード補正
    if (cmd_left != 0) {
      int sign_l = (cmd_left > 0) ? 1 : -1;
      current_left_power += static_cast<int>((target_speed_left - speed_left) * K_speed) * sign_l;
    }

    // 右ホイールのスピード補正
    if (cmd_right != 0) {
      int sign_r = (cmd_right > 0) ? 1 : -1;
      current_right_power += static_cast<int>((target_speed_right - speed_right) * K_speed) * sign_r;
    }

    // 出力範囲制限 (-100 〜 100)
    if (current_left_power > 100)   current_left_power = 100;
    if (current_left_power < -100)  current_left_power = -100;
    if (current_right_power > 100)  current_right_power = 100;
    if (current_right_power < -100) current_right_power = -100;

    gMotors.run(current_left_power, current_right_power);

    dly_tsk(step_ms * 1000U);
  }

  gMotors.stop();
}

// ライントレースメソッド
void SumoScene::drive_linetrace_distance(int32_t target_distance_mm, float base_power, bool follow_left_edge) {
  constexpr float WheelDiameterMm = 56.0f;
  constexpr float M_PI_F = 3.14159265f;
  constexpr float MmPerDegree = (WheelDiameterMm * M_PI_F) / 360.0f; // 1度あたりの進む距離(mm)

  // 1. 移動距離から目標のエンコーダ回転角度(度)を算出
  const int32_t abs_distance_mm = (target_distance_mm >= 0) ? target_distance_mm : -target_distance_mm;
  const int32_t target_deg = static_cast<int32_t>(abs_distance_mm / MmPerDegree);

  // 2. ライントレーサーの設定
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(base_power, base_power);
  gLineTracer.setEdgeFollow(follow_left_edge);

  /* 3. 開始時のエンコーダ角度を取得 */
  const int32_t start_left  = gMotors.leftEncoderCount();
  const int32_t start_right = gMotors.rightEncoderCount();

  const uint32_t step_ms = 10U;

  while (true) {
    /* 4. 走行距離の判定 */
    int32_t current_left  = gMotors.leftEncoderCount() - start_left;
    int32_t current_right = gMotors.rightEncoderCount() - start_right;

    // 左右エンコーダの絶対値平均で判定
    int32_t avg_deg = (labs(current_left) + labs(current_right)) / 2;

    if (avg_deg >= target_deg) {
      break; // 目標距離に到達したら終了
    }

    /* 5. ライントレース制御を1ステップ実行 */
    gLineTracer.step();

    dly_tsk(step_ms * 1000U);
  }

  // 6. 完了後にライントレースを無効化して停止
  gLineTracer.setEnabled(false);
  gMotors.stop();
  dly_tsk(100 * 1000U);
}

// 黒色検知まで直進メソッド
void SumoScene::drive_chokushin_until_black(int base_power, float target_heading, float Kp) {
  const uint32_t step_ms = 10U;

  // バック走行の判定
  const int drive_dir = (base_power < 0) ? -1 : 1;
  const int abs_base_power = (base_power >= 0) ? base_power : -base_power;

  // 目標スピードの計算 (10msあたりの変化量)
  const float target_deg_per_step = static_cast<float>(abs_base_power) * 0.15f;
  const float K_speed = 0.5f;

  int current_left_power  = abs_base_power;
  int current_right_power = abs_base_power;

  int32_t prev_left  = gMotors.leftEncoderCount();
  int32_t prev_right = gMotors.rightEncoderCount();

  while (true) {
    /* 1. ColorRGB モジュールによる黒色検知 */
    if (gColorRGB.is(COLOR_DETECT_BLACK)  || gColorRGB.is(COLOR_DETECT_BLUE)) {
      break; // 黒色を検知したら抜け出す
    }

    /* 2. エンコーダ現在値取得と速度監視 */
    int32_t current_left  = gMotors.leftEncoderCount();
    int32_t current_right = gMotors.rightEncoderCount();

    float speed_left  = static_cast<float>(labs(current_left - prev_left));
    float speed_right = static_cast<float>(labs(current_right - prev_right));
    prev_left  = current_left;
    prev_right = current_right;

    // 速度低下時の一定出力補正
    current_left_power  += static_cast<int>((target_deg_per_step - speed_left) * K_speed);
    current_right_power += static_cast<int>((target_deg_per_step - speed_right) * K_speed);

    /* 3. ジャイロ P 制御による角度補正 */
    float current_h = gGyro.heading();
    float error     = Gyro::wrapDeg180(target_heading - current_h);
    int turn_adjust = static_cast<int>(error * Kp);

    int final_left_power  = (current_left_power  + turn_adjust) * drive_dir;
    int final_right_power = (current_right_power - turn_adjust) * drive_dir;

    // モーター出力範囲制限 (-100 〜 100)
    if (final_left_power > 100)   final_left_power = 100;
    if (final_left_power < -100)  final_left_power = -100;
    if (final_right_power > 100)  final_right_power = 100;
    if (final_right_power < -100) final_right_power = -100;

    gMotors.run(final_left_power, final_right_power);

    dly_tsk(step_ms * 1000U);
  }

  // 4. 検知後の停止処理
  gMotors.stop();
  dly_tsk(100 * 1000U);
}