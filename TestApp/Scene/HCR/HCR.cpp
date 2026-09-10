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
}

#include <math.h>
#include <string.h>

extern Straight gMotors;
extern ColorHSV gColorHSV;
extern ColorRGB gColorRGB;
extern ColorLAB gColorLAB;
extern LineTracer gLineTracer;
extern Gyro gGyro;

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

void HCR::turn_90(const char* direction, int32_t inner_arc_deg)
{
  if (!configured_) {
    return;
  }

  gMotors.setTargetHeading(0.0f);
  gMotors.runFor(80, 80, 360);

  if (strcmp(direction, "left") == 0) {
    gMotors.runFor(-80, -80, inner_arc_deg);
    gMotors.runFor(-80, 80, inner_arc_deg);
  } else if (strcmp(direction, "right") == 0) {
    gMotors.runFor(-80, -80, inner_arc_deg);
    gMotors.runFor(80, -80, inner_arc_deg);
  }

  gMotors.stop();
}

void HCR::catch_blue_line()
{
  gLineTracer.setEnabled(true);
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

void HCR::LineTrace()
{
  // catch_color_line();
  gLineTracer.setBaseSpeed(100.0f, 100.0f);
  gLineTracer.setPid(0.07f,0.0006f,0.52f, 100.0f);
  gLineTracer.setScale(18.0f);
  // gLineTracer.setBaseSpeed(85.0f, 85.0f);
  // gLineTracer.setPid(0.068f,0.0006f,0.6f, 100.0f);
  // gLineTracer.setScale(15.0f);
  /* L=右エッジ, R=左エッジ */
  gLineTracer.setEdgeFollow(ETROBO_COURSE_L == 0);
  sta_cyc(LINE_TRACER_TASK_CYC);
  dly_tsk(3500 * 1000);
  stp_cyc(LINE_TRACER_TASK_CYC);
  gGyro.reset();
  sta_cyc(LINE_TRACER_TASK_CYC);
  dly_tsk(300 * 1000);
  stp_cyc(LINE_TRACER_TASK_CYC);
  gLineTracer.setEnabled(true);
  gLineTracer.setScale(16.0f);
  gLineTracer.setPid(0.067f,0.0006f,0.52f, 100.0f);
  sta_cyc(LINE_TRACER_TASK_CYC);
  dly_tsk(1000 * 1000);
  stp_cyc(LINE_TRACER_TASK_CYC);
  
  

  // gLineTracer.setBaseSpeed(80.0f, 80.0f);
  // gLineTracer.setPid(0.07f,0.0006f,0.5f, 100.0f);
  // gLineTracer.setScale(15.0f);
  // sta_cyc(LINE_TRACER_TASK_CYC);
  // dly_tsk(4100 * 1000);
  // stp_cyc(LINE_TRACER_TASK_CYC);
  // gLineTracer.setEnabled(false);
  // gGyro.reset();
  // gLineTracer.setEnabled(true);


}

void HCR::AllLineTrace()
{
  LineTrace();
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

void HCR::run_hrc()
{
  LineTrace();
  while (gGyro.heading() > -90.0f) {
    gLineTracer.step();
    dly_tsk(5 * 1000);
  }
  gLineTracer.setEnabled(false);
  if (ETROBO_COURSE_L) {
    gMotors.run(90.0f, 100.0f);
  } else {
    gMotors.run(100.0f, 90.0f);
  }
  dly_tsk(320 * 1000);
  if (ETROBO_COURSE_L) {
    while (gGyro.heading() > -175.0f) {
      gMotors.run(77.0f, 100.0f);
      dly_tsk(5 * 1000);
    }
  } else {
    while (gGyro.heading() > 175.0f) {
      gMotors.run(100.0f, 77.0f);
      dly_tsk(5 * 1000);
    }
  }
  gMotors.run(100.0f, 100.0f);
  dly_tsk(400 * 1000);
  gMotors.stop();
  dly_tsk(5000 * 1000);
  if (ETROBO_COURSE_L) {
    gMotors.run(100.0f, 85.0f);
    dly_tsk(830 * 1000);
    gMotors.run(100.0f, 100.0f);
    dly_tsk(310 * 1000);
    gMotors.setTargetHeading(-180.0f);
    gMotors.run(70.0f, 100.0f);
    dly_tsk(500 * 1000);
  } else {
    gMotors.run(85.0f, 100.0f);
    dly_tsk(830 * 1000);
    gMotors.run(100.0f, 100.0f);
    dly_tsk(310 * 1000);
    gMotors.setTargetHeading(180.0f);
    gMotors.run(100.0f, 70.0f);
    dly_tsk(600 * 1000);
  }
  catch_black_line();
  catch_blue_line();

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

void HCR::ShortCutRun()
{
  /* L=右エッジ, R=左エッジ */
  gLineTracer.setEdgeFollow(ETROBO_COURSE_L != 0);
  gLineTracer.setBaseSpeed(80.0f, 80.0f);
  gLineTracer.setPid(0.05f,0.0004f,0.3f, 100.0f);
  gLineTracer.setScale(4.5f);
  sta_cyc(LINE_TRACER_TASK_CYC);
  gGyro.reset();
  stp_cyc(LINE_TRACER_TASK_CYC);
  gMotors.run(-80.0f, -80.0f);
  dly_tsk(500 * 1000);
  if (ETROBO_COURSE_L) {
    gMotors.run(0.0f, -70.0f);
  } else {
    gMotors.run(-70.0f, 0.0f);
  }
  dly_tsk(550 * 1000);
  gMotors.stop();
  dly_tsk(500 * 1000);
  gMotors.setTargetHeading(ETROBO_COURSE_L ? 70.0f : -70.0f);
  gMotors.run(80.0f, 80.0f);
  dly_tsk(1800 * 1000);
  while (true) {
    if (gColorLAB.is(COLOR_DETECT_BLACKLINE, true)) {
      gMotors.stop();
      gLineTracer.setBaseSpeed(80.0f, 80.0f);
      gLineTracer.setPid(0.07f,0.0006f,0.5f, 100.0f);
      gLineTracer.setScale(10.0f);
      sta_cyc(LINE_TRACER_TASK_CYC);
      dly_tsk(1500 * 1000);
      break;
    }
    if (ETROBO_COURSE_L) {
      gMotors.run(80, 77);
    } else {
      gMotors.run(77, 80);
    }
    dly_tsk(5 * 1000);
  }
  stp_cyc(LINE_TRACER_TASK_CYC);
  gMotors.setTargetHeading(ETROBO_COURSE_L ? 162.0f : -162.0f);
  SpeedUp(80.0f, 80.0f, 0.5f, 100.0f, true);
  gMotors.setTargetHeading(ETROBO_COURSE_L ? 153.0f : -153.0f);
  gMotors.run(100.0f, 100.0f);
  dly_tsk(1200 * 1000);
  catch_black_line();
  catch_blue_line();
}