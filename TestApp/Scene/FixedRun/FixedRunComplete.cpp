/* 決め打ち完成。ETROBO_COURSE_L: 1=R（旧1） / 0=L（旧2） */
#include "kernel.h"

#include "app.h"
#include "FixedRun.h"
#include "Gyro.h"
#include "Straight.h"

extern Straight gMotors;
extern Gyro     gGyro;

void FixedRunComplete::run()
{
#if ETROBO_COURSE_L
  const float turn1_deg    = -90.0f;
  const float turn2_deg    = -165.0f;
  const float turn3_deg    = -90.0f;
  const float turn4_deg    = -29.0f;
  const int   sign_out     = -1;
  const int   sign_in      = 1;
  const int   turn3_inner  = -60;
  const int32_t first_back = 900;
  const int32_t third_back = 700;
  const int32_t fifth_back = 2950;
#else
  const float turn1_deg    = 89.0f;
  const float turn2_deg    = 165.0f;
  const float turn3_deg    = 90.0f;
  const float turn4_deg    = 29.0f;
  const int   sign_out     = 1;
  const int   sign_in      = -1;
  const int   turn3_inner  = -55;
  const int32_t first_back = 905;
  const int32_t third_back = 660;
  const int32_t fifth_back = 2800;
#endif

  gMotors.stop();
  gGyro.reset();
  dly_tsk(200 * 1000U);

  const float fixedHeading = gGyro.heading();
  gMotors.setHeadingCorrection(true);
  gMotors.setTargetHeading(fixedHeading);
  gMotors.setBalancePid(8.0f, 0.01f);

  gMotors.run(-60, -60, 50, false);
  gMotors.run(-80, -80, 50, false);
  gMotors.run(-90, -90, 110, false);
  gMotors.run(-100, -100, first_back, false);

  const float turnHeading = fixedHeading + turn1_deg;
#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(false);
#endif
  gMotors.curveToHeading(turnHeading, -40, -100, 3000, 0.0f, sign_out, false);

#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(true);
#endif
  gMotors.setTargetHeading(turnHeading);
  gMotors.run(-100, -100, 800, false);

  const float secondHeading = fixedHeading + turn2_deg;
#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(false);
#endif
  gMotors.curveToHeading(secondHeading, -60, -100, 280, 0.0f, sign_out, false);

#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(true);
#endif
  gMotors.setTargetHeading(secondHeading);
  gMotors.run(-100, -100, third_back, false);

  const float thirdHeading = fixedHeading + turn3_deg;
#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(false);
#endif
  gMotors.curveToHeading(thirdHeading, turn3_inner, -100, 290, 0.0f, sign_in,
                         false);

#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(true);
#endif
  gMotors.setTargetHeading(thirdHeading);
  gMotors.run(-100, -100, 390, false);

  const float fourHeading = fixedHeading + turn4_deg;
#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(false);
#endif
  gMotors.curveToHeading(fourHeading, -60, -100, 3000, 0.0f, sign_in, false);

#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(true);
#endif
  gMotors.setTargetHeading(fourHeading);
  gMotors.run(-100, -100, fifth_back, false);

  const float fiveHeading = fixedHeading;
#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(false);
#endif
  gMotors.curveToHeading(fiveHeading, -60, -100, 3000, 0.0f, sign_in, false);

#if !ETROBO_COURSE_L
  gMotors.setHeadingCorrection(true);
#endif
  gMotors.setTargetHeading(fiveHeading);
  gMotors.run(-100, -100, 1700, false);

  gMotors.stop();
}
