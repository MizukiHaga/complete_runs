/* 決め打ち短縮。ETROBO_COURSE_L: 1=R（旧3） / 0=L（旧4） */
#include "kernel.h"

#include "app.h"
#include "FixedRun.h"
#include "Gyro.h"
#include "Straight.h"

extern Straight gMotors;
extern Gyro     gGyro;

void FixedRunShort::run()
{
#if ETROBO_COURSE_L
  const float first_deg  = 97.0f;
  const float second_deg = 153.0f;
  const float last_deg   = 178.0f;
  const int   turn_sign  = 1;
  const int32_t last_back = 1650;
#else
  const float first_deg  = -97.0f;
  const float second_deg = -151.0f;
  const float last_deg   = -178.0f;
  const int   turn_sign  = -1;
  const int32_t last_back = 1800;
#endif

  gMotors.stop();
  gGyro.reset();
  dly_tsk(200 * 1000U);

  const float fixedHeading = gGyro.heading();
  gMotors.setHeadingCorrection(true);
  gMotors.setBalancePid(8.0f, 0.01f);
  gMotors.setTargetHeading(fixedHeading);

  gMotors.run(40, 40, 20, true);

  const float rightHeading = fixedHeading + first_deg;
  gMotors.setHeadingCorrection(false);
  gMotors.curveToHeading(rightHeading, -40, -100, 1500, 0.0f, turn_sign, true);

  gMotors.setHeadingCorrection(true);
  gMotors.setTargetHeading(rightHeading);
  gMotors.run(-100, -100, 2450, false);

  const float secondHeading = fixedHeading + second_deg;
  gMotors.setHeadingCorrection(false);
  gMotors.curveToHeading(secondHeading, -40, -100, 1500, 0.0f, turn_sign, true);

  gMotors.setHeadingCorrection(true);
  gMotors.setTargetHeading(secondHeading);
  gMotors.run(-100, -100, 3000, true);

  const float fiveHeading = fixedHeading + last_deg;
  gMotors.setHeadingCorrection(false);
  gMotors.curveToHeading(fiveHeading, -40, -100, 3000, 0.0f, turn_sign, false);

  gMotors.setHeadingCorrection(true);
  gMotors.setTargetHeading(fiveHeading);
  gMotors.run(-100, -100, last_back, false);

  gMotors.stop();
}
