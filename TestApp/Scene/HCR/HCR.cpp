#include "app.h"
#include "ColorHSV.h"
#include "ColorRGB.h"
#include "LineTracer.h"
#include "Straight.h"
#include "Gyro.h"
#include "HCR.h"
#include "ColorLAB.h"
#include "kernel_cfg.h"

extern "C" {
#include <spike/pup/motor.h>
#include <spike/hub/imu.h>
#include <spike/hub/display.h>
#include <spike/hub/speaker.h>
}

#include <math.h>
#include <stdio.h>
#include <string.h>

extern Straight gMotors;
extern ColorHSV gColorHSV;
extern ColorRGB gColorRGB;
extern ColorLAB gColorLAB;
extern LineTracer gLineTracer;
extern Gyro gGyro;
extern FILE *gBluetoothOut;

void HCR::configure_hrc(pbio_port_id_t left_motor_port,
                    pbio_port_id_t right_motor_port,
                    pbio_port_id_t color_sensor_port)
{
  left_motor_port_ = left_motor_port;
  right_motor_port_ = right_motor_port;
  color_sensor_port_ = color_sensor_port;
  configured_ = true;

  // gMotors.configure(left_motor_port, right_motor_port);
  // gColorHSV.configure(color_sensor_port);
  // gLineTracer.configure(left_motor_port, right_motor_port, color_sensor_port);
}

void HCR::catch_blue_line()
{
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(90.0f, 90.0f);
  gLineTracer.setPid(0.06f,0.0004f,0.3f, 100.0f);
  gLineTracer.setScale(10.0f);
  sta_cyc(LINE_TRACER_TASK_CYC);
  while (true) {
    if (gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
      gLineTracer.setBaseSpeed(50.0f, 50.0f);
      gLineTracer.setPid(0.06f,0.0004f,0.4f, 100.0f);
      gLineTracer.setScale(5.0f);
      while (true) {
        if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
          gLineTracer.setEnabled(false);
          gMotors.stop();
          dly_tsk(10 * 1000);
          /* 黒到達で HCR 終了。周期 LT を止めて Nansho1 へ渡す */
          stp_cyc(LINE_TRACER_TASK_CYC);
          gLineTracer.setEnabled(false);
          return;
        }
        gLineTracer.step();
        dly_tsk(1 * 1000);
      }
    }
    gLineTracer.step();
    dly_tsk(5 * 1000);
  }
}
void HCR::catch_black_line()
{
  gLineTracer.setEnabled(true);
  while (true) {
    if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
      gLineTracer.setBaseSpeed(80.0f, 80.0f);
      gLineTracer.setPid(0.07f,0.0006f,0.5f, 100.0f);
      gLineTracer.setScale(15.0f);
      sta_cyc(LINE_TRACER_TASK_CYC);
      dly_tsk(800 * 1000);
      stp_cyc(LINE_TRACER_TASK_CYC);
      break;
    }
    gMotors.setTargetHeading(ETROBO_COURSE_L ? 140.0f : -140.0f);
    gMotors.run(80.0f, 80.0f);
    dly_tsk(5 * 1000);
  }
}

void HCR::StraightRotation(int power, float left_rotations, float right_rotations,
                           float correction)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.runForRotationsSameTime((float)power - correction,
                                  (float)power + correction,
                                  left_rotations, right_rotations);
}

void HCR::LineTraceRotation(float left_rotation,float right_rotation, float left_speed, float right_speed, float p, float i, float d, float scale)
{
  /* 前回の走行で停止した周期タスクを、手動 step() と競合させない。 */
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(true);
  /* HCR の左右コースに合わせて追従するエッジを選ぶ。 */
  gLineTracer.setEdgeFollow(ETROBO_COURSE_L != 0);
  gMotors.resetRotationCount();
  const float start_left = gMotors.leftRotationCount();
  const float start_right = gMotors.rightRotationCount();
  gLineTracer.setBaseSpeed(left_speed, right_speed);
  gLineTracer.setPid(p,i,d,100.0f);
  gLineTracer.setScale(scale);
  uint32_t elapsed_ms = 0U;
  while (true) {
    gMotors.setHeadingDebugPhase("HCR::LineTraceRotation");
    gLineTracer.step();
    gMotors.debugHeading();
    const float moved_left = gMotors.leftRotationCount() - start_left;
    const float moved_right = gMotors.rightRotationCount() - start_right;
    if ((elapsed_ms % 100U) == 0U) {
      fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
              "HCR: LineTraceRotation,moved_left=%.3f,"
              "moved_right=%.3f,target_left=%.3f,target_right=%.3f\n",
              moved_left, moved_right, left_rotation, right_rotation);
    }
    if (moved_left >= left_rotation && moved_right >= right_rotation) {
      fprintf(gBluetoothOut != nullptr ? gBluetoothOut : stdout,
              "HCR: LineTraceRotation,complete,moved_left=%.3f,"
              "moved_right=%.3f,target_left=%.3f,target_right=%.3f\n",
              moved_left, moved_right, left_rotation, right_rotation);
      gLineTracer.setEnabled(false);
      gMotors.stop();
      stp_cyc(LINE_TRACER_TASK_CYC);
      dly_tsk(1 * 1000);
      return;
    }
    dly_tsk(1 * 1000);
    ++elapsed_ms;
  }
}
// void HCR::LineTrace()
// {
//   // catch_color_line();
//   gLineTracer.setBaseSpeed(95.0f, 95.0f);
//   gLineTracer.setPid(0.08f,0.00025f,0.72f, 100.0f);
//   gLineTracer.setScale(18.0f);
//   // gLineTracer.setBaseSpeed(85.0f, 85.0f);
//   // gLineTracer.setPid(0.068f,0.0006f,0.6f, 100.0f);
//   // gLineTracer.setScale(15.0f);
//   /* L=右エッジ, R=左エッジ（ShortCutRun と同じ。==0 だと R が右エッジになり曲がり切れない） */
//   gLineTracer.setEdgeFollow(ETROBO_COURSE_L != 0);
//   sta_cyc(LINE_TRACER_TASK_CYC);
//   dly_tsk(3500 * 1000);
//   gGyro.reset();
//   dly_tsk(300 * 1000);
//   gLineTracer.setScale(16.0f);
//   dly_tsk(1000 * 1000);
//   /* 周期 LT は止めない。カーブ終端で制御が途切れると右へ飛ぶ */



//   // gLineTracer.setBaseSpeed(80.0f, 80.0f);
//   // gLineTracer.setPid(0.07f,0.0006f,0.5f, 100.0f);
//   // gLineTracer.setScale(15.0f);
//   // sta_cyc(LINE_TRACER_TASK_CYC);
//   // dly_tsk(3500 * 1000);
//   // stp_cyc(LINE_TRACER_TASK_CYC);
//   // gLineTracer.setEnabled(false);
//   // gGyro.reset();
//   // gLineTracer.setEnabled(true);
//   // sta_cyc(LINE_TRACER_TASK_CYC);
//   // dly_tsk(300 * 1000);
//   // stp_cyc(LINE_TRACER_TASK_CYC);
//   // gLineTracer.setScale(hcr_scale_s);
//   // gLineTracer.setPid(hcr_p_s,hcr_i,hcr_d,100.0f);
//   // sta_cyc(LINE_TRACER_TASK_CYC);
//   // dly_tsk(1000 * 1000);
//   // stp_cyc(LINE_TRACER_TASK_CYC);
  
// }

void HCR::LineTraceRotationCheck(float duration_sec)
{
  if (duration_sec <= 0.0f) {
    hub_display_char('E');
    return;
  }

  stp_cyc(LINE_TRACER_TASK_CYC);
  int32_t previous_left_deg = gMotors.leftEncoderCount();
  int32_t previous_right_deg = gMotors.rightEncoderCount();
  int32_t total_left_deg = 0;
  int32_t total_right_deg = 0;
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(100.0f, 100.0f);
  gLineTracer.setPid(Test_p, Test_i, Test_d, 100.0f);
  gLineTracer.setScale(Test_scale);
  sta_cyc(LINE_TRACER_TASK_CYC);

  const uint32_t total_us = (uint32_t)(duration_sec * 1000000.0f + 0.5f);
  uint32_t elapsed_us = 0U;
  while (elapsed_us < total_us) {
    const uint32_t slice_us = (total_us - elapsed_us < 5000U)
                              ? total_us - elapsed_us : 5000U;
    dly_tsk((RELTIM)slice_us);
    elapsed_us += slice_us;

    const int32_t current_left_deg = gMotors.leftEncoderCount();
    const int32_t current_right_deg = gMotors.rightEncoderCount();
    total_left_deg += abs(current_left_deg - previous_left_deg);
    total_right_deg += abs(current_right_deg - previous_right_deg);
    previous_left_deg = current_left_deg;
    previous_right_deg = current_right_deg;
  }

  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

  const int32_t right_rot_whole = total_right_deg / 360;
  const int32_t right_rot_tenth = (total_right_deg % 360) / 36;
  const int32_t left_rot_whole = total_left_deg / 360;
  const int32_t left_rot_tenth = (total_left_deg % 360) / 36;
  char result[32];
  snprintf(result, sizeof(result), "R%ld.%ldL%ld.%ld ",
           (long)right_rot_whole, (long)right_rot_tenth,
           (long)left_rot_whole, (long)left_rot_tenth);

  while (true) {
    for (size_t i = 0; result[i] != '\0'; ++i) {
      hub_display_char(result[i]);
      dly_tsk(1000 * 1000);
    }
  }
}

void HCR::AllLineTrace()
{
  if (!separate_pattern){
    gLineTracer.setBaseSpeed(motor_speed, motor_speed);
    gLineTracer.setPid(hcr_p,hcr_i,hcr_d, 100.0f);
    gLineTracer.setScale(hcr_scale);
    // gLineTracer.setBaseSpeed(95.0f, 95.0f);
    // gLineTracer.setPid(0.07f,0.00025f,0.72f, 100.0f);
    // gLineTracer.setScale(20.0f);
    /* L=右エッジ, R=左エッジ */
    gLineTracer.setEdgeFollow(ETROBO_COURSE_L == 0);
    sta_cyc(LINE_TRACER_TASK_CYC);
    dly_tsk(3500 * 1000);
    stp_cyc(LINE_TRACER_TASK_CYC);
    gLineTracer.setEnabled(false);
    gGyro.reset();
    gLineTracer.setEnabled(true);
    sta_cyc(LINE_TRACER_TASK_CYC);
    dly_tsk(300 * 1000);
    stp_cyc(LINE_TRACER_TASK_CYC);
    gLineTracer.setScale(hcr_scale_s);
    gLineTracer.setPid(hcr_p_s,hcr_i,hcr_d,100.0f);
    sta_cyc(LINE_TRACER_TASK_CYC);
    dly_tsk(1000 * 1000);
    stp_cyc(LINE_TRACER_TASK_CYC);
    while (true) {
      if (gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
        gLineTracer.setBaseSpeed(50.0f, 50.0f);
        gLineTracer.setPid(0.062f,0.0006f,0.5f, 100.0f);
        gLineTracer.setScale(15.0f);
        while (true) {
          if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
            gLineTracer.setEnabled(false);
            gMotors.stop();
            stp_cyc(LINE_TRACER_TASK_CYC);
            dly_tsk(1000 * 1000);
            return;
          }
          gLineTracer.step();
          dly_tsk(1 * 1000);
        }
      }
      gLineTracer.step();
      dly_tsk(5 * 1000);
    }
  }
  LineTraceRotation(straight_rotation, straight_rotation, motor_speed, motor_speed, hcr_p_s,hcr_i,hcr_d, hcr_scale_s);
  LineTraceRotation(first_curve_rotation_r, first_curve_rotation_l, motor_speed, motor_speed, curve_1and2_p, hcr_i, curve_1and2_d, curve_1and2_scale);
  LineTraceRotation(second_straight_rotation, second_straight_rotation, motor_speed, motor_speed, hcr_p_s,hcr_i,hcr_d, hcr_scale_s);
  LineTraceRotation(second_curve_rotation_r, second_curve_rotation_l, motor_speed, motor_speed, curve_1and2_p, hcr_i, curve_1and2_d, curve_1and2_scale);
  LineTraceRotation(third_straight_rotation, third_straight_rotation, motor_speed, motor_speed, hcr_p_s,hcr_i,hcr_d, hcr_scale_s);
  gMotors.stop();
  dly_tsk(1000 * 1000);
  LineTraceRotation(third_curve_rotation_r, third_curve_rotation_l, motor_speed, motor_speed, third_curve_p, hcr_i, hcr_d, third_curve_scale);
  gMotors.stop();
  dly_tsk(1000 * 1000);
  LineTraceRotation(triple_curve_rotation_r, triple_curve_rotation_l, motor_speed, motor_speed, triple_curve_p, hcr_i, hcr_d, triple_curve_scale);
  gMotors.stop();
  dly_tsk(1000 * 1000);



  // LineTraceRotation(3.0f, 3.0f, 100.0f, 100.0f, 0.07f, 0.0006f, 0.5f, 100.0f);
  gLineTracer.setEnabled(true);
  gLineTracer.setBaseSpeed(motor_speed, motor_speed);
  gLineTracer.setPid(hcr_p_s,hcr_i,hcr_d, 100.0f);
  gLineTracer.setScale(hcr_scale_s);
  while (true) {
    if (gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
      hub_speaker_set_volume(50);
      hub_speaker_play_tone(1000, 300);
      gLineTracer.setBaseSpeed(detect_speed, detect_speed);
      gLineTracer.setPid(hcr_p_s,hcr_i,hcr_d, 100.0f);
      gLineTracer.setScale(hcr_scale_s);
      gLineTracer.setBaseSpeed(50.0f, 50.0f);
      gLineTracer.setPid(0.08f,0.0006f,0.5f, 100.0f); // L = 0.062f,0.0006f,0.5f, 100.0f
      gLineTracer.setScale(15.0f);
      while (true) {
        if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
          gLineTracer.setEnabled(false);
          gMotors.stop();
          stp_cyc(LINE_TRACER_TASK_CYC);
          dly_tsk(1 * 1000);
          return;
        }
        gLineTracer.step();
        dly_tsk(1 * 1000);
      }
    }
    gLineTracer.step();
    dly_tsk(1 * 1000);
  }
}

void HCR::run_hrc()
{
  /* 再走行時に前回の周期制御・ジャイロ基準を持ち越さない。 */
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();

  // LineTraceRotation(hcr_rotation, hcr_rotation, motor_speed, motor_speed, hcr_p_s,hcr_i,hcr_d, hcr_scale_s);
  LineTraceRotation(hcr_rotation, hcr_rotation, motor_speed, motor_speed, hcr_p_s,hcr_i,hcr_d, hcr_scale_s);
  gGyro.reset();
  /* 中間の45度目標は使わず、現在の位置から90度を直接目標にする。 */
  gMotors.setTargetHeading(heading_90);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, heading_90_straight_l,
      heading_90_straight_r, true);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, after_90_straight, after_90_straight, true);

  gMotors.setTargetHeading(heading_165);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, after_90_straight, after_90_straight, true);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, after_165_straight, after_165_straight, true);
  
  gMotors.setTargetHeading(heading_90);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, heading_90_straight_l,
      heading_90_straight_r, true);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, before_triple_curve_straight, before_triple_curve_straight, true);

  gMotors.setTargetHeading(heading_50);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, heading_50_rotation,
      heading_50_rotation, true);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, heading_50_straight,
      heading_50_straight, true);
  
  gMotors.setTargetHeading(triple_curve_heading);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, after_triple_curve_heading_rotation_l,
      after_triple_curve_heading_rotation_r, true);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, triple_curve_rotation, triple_curve_rotation, true);
  
  gMotors.setTargetHeading(after_triple_curve_heading);
  gMotors.runForRotationsSameTimeWithHeading(
      motor_speed, motor_speed, after_triple_curve_heading_rotation_l,
      after_triple_curve_heading_rotation_r, true);

  while (true) {
    if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
      LineTraceRotation(finish_line_trace_rotation, finish_line_trace_rotation,
                        finish_line_trace_speed, finish_line_trace_speed,
                        finish_line_trace_p, hcr_i, finish_line_trace_d,
                        finish_line_trace_scale);
      gLineTracer.setEnabled(true);
      gLineTracer.setBaseSpeed(motor_speed, motor_speed);
      gLineTracer.setPid(hcr_p_s,hcr_i,hcr_d,100.0f);
      gLineTracer.setScale(hcr_scale_s);
      while (true) {
        if (gColorLAB.is(COLOR_DETECT_SKYBLUE, true)) {
          gLineTracer.setBaseSpeed(finish_skyblue_speed, finish_skyblue_speed);
          gLineTracer.setPid(finish_skyblue_p, finish_skyblue_i,
                             finish_skyblue_d, 100.0f);
          gLineTracer.setScale(finish_skyblue_scale);
          while (true) {
            if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
              gMotors.stop();
              gLineTracer.setEnabled(false);
              stp_cyc(LINE_TRACER_TASK_CYC);
              dly_tsk(1 * 1000);
              return;
            }
            gLineTracer.step();
            dly_tsk(1 * 1000);
          }
        }
        gLineTracer.step();
        dly_tsk(1 * 1000);
      }
    }
    gMotors.run(100.0f, 100.0f);
    dly_tsk(1 * 1000);
  }

  /* Nansho1 へ渡す前に周期 LT を確実に停止する */
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(false);
  gMotors.stop();
}

void HCR::SlowDown(float left_speed, float right_speed, float duration_sec, float target_speed)
{
  stp_cyc(LINE_TRACER_TASK_CYC);
  

  if (duration_sec <= 0.0f) {
    sta_cyc(LINE_TRACER_TASK_CYC);
    return;
  }

  const uint32_t total_us = (uint32_t)(duration_sec * 1000000.0f + 0.5f);
  const uint32_t step_us = 10000U;

  uint32_t elapsed_us = 0U;
  while (elapsed_us < total_us) {
    uint32_t slice_us = step_us;
    if (elapsed_us + slice_us > total_us) {
      slice_us = total_us - elapsed_us;
    }

    elapsed_us += slice_us;

    const float ratio = (float)elapsed_us / (float)total_us;
    const float cur_left = left_speed + (target_speed - left_speed) * ratio;
    const float cur_right = right_speed + (target_speed - right_speed) * ratio;

    gLineTracer.setBaseSpeed(cur_left, cur_right);
    sta_cyc(LINE_TRACER_TASK_CYC);
    dly_tsk((RELTIM)slice_us);
  }
  gLineTracer.setBaseSpeed(target_speed, target_speed);
  sta_cyc(LINE_TRACER_TASK_CYC);
  return;
}

void HCR::SpeedUp(float left_speed, float right_speed, float duration_sec, float target_speed, bool is_straight)
{
  stp_cyc(LINE_TRACER_TASK_CYC);

  if (duration_sec <= 0.0f) {
    sta_cyc(LINE_TRACER_TASK_CYC);
    return;
  }

  const uint32_t total_us = (uint32_t)(duration_sec * 1000000.0f + 0.5f);
  const uint32_t step_us = 10000U;

  uint32_t elapsed_us = 0U;
  while (elapsed_us < total_us) {
    uint32_t slice_us = step_us;
    if (elapsed_us + slice_us > total_us) {
      slice_us = total_us - elapsed_us;
    }

    elapsed_us += slice_us;

    const float ratio = (float)elapsed_us / (float)total_us;
    const float cur_left = left_speed + (target_speed - left_speed) * ratio;
    const float cur_right = right_speed + (target_speed - right_speed) * ratio;
    if (is_straight) {
      gMotors.run(cur_left, cur_right);
    }else {
      gLineTracer.setBaseSpeed(cur_left, cur_right);
      sta_cyc(LINE_TRACER_TASK_CYC);
    }
    dly_tsk((RELTIM)slice_us);
  }
  if (is_straight) {
    gMotors.run(target_speed, target_speed);
  } else {
    gLineTracer.setBaseSpeed(target_speed, target_speed);
    sta_cyc(LINE_TRACER_TASK_CYC);
  }
  return;
}
