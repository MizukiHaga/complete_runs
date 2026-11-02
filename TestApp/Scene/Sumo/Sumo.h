#ifndef TESTAPP_SUMO_H_
#define TESTAPP_SUMO_H_

#include <kernel.h>

#include "app.h"
#include "ColorLAB.h"
#include "ColorRGB.h"
#include "Gyro.h"
#include "Sonic.h"
#include "Straight.h"

class SumoScene {
public:
  void run();

  void set_sumo_reverse(bool status) { is_reverse_ = status; }

  void drive_chokushin_distance(int32_t target_distance_mm, int base_power, float target_heading, float Kp = 2.0f);
  void drive_senkai(float target_angle, int speed_base, int speed_low, float tolerance_deg = 1.0f, bool is_counterclock = true);
  void drive_linetrace_distance(int32_t target_distance_mm, float base_power, bool follow_left_edge = true);
  void drive_chokushin_until_black(int base_power, float target_heading, float Kp = 2.0f);

private:
  bool is_reverse_ = false;

  inline float invert_angle(float angle) const {return is_reverse_ ? -angle : angle;}
  inline bool invert_ccw(bool is_ccw) const {return is_reverse_ ? !is_ccw : is_ccw;}
  inline bool invert_edge(bool follow_left) const {return is_reverse_ ? !follow_left : follow_left;}
};

#ifdef __cplusplus
extern "C" {
#endif

void SumoTask(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_SUMO_H_
